"""ROS callbacks never wait on HTTP; bounded worker jobs are bridged by a timer."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import json
import hashlib
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn as Transition
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from std_srvs.srv import Trigger
from zzx_interfaces.action import MoveArm, ControlHand
from zzx_interfaces.msg import BackendCapabilities, ErrorStatus as E, ExecutionStatus as S
from zzx_http_contracts.protocol import ARM_NAMES

from .worker import HttpWorker, Outcome, validate_arm, validate_hand
from .ownership import CommandLease


@dataclass
class Pending:
    cancel: threading.Event = field(default_factory=threading.Event)
    done: Future = field(default_factory=Future)
    stage: str = 'validating'
    job: object = None


class HttpActionBackend(LifecycleNode):
    def __init__(self):
        super().__init__('http_action_backend')
        path = self.declare_parameter('capabilities_path', str(
            Path(get_package_share_directory('zzx_http_backend')) / 'config/capabilities.json')).value
        with open(path, encoding='utf-8') as stream:
            self.capabilities = json.load(stream)
        for kind in ('arm', 'hand'):
            caps = self.capabilities[kind]
            supported = {'joint_target', 'cancel', 'plan_only', 'teach_mode'} if kind == 'arm' else {'position_command'}
            fields = {'joint_target', 'pose_target', 'trajectory', 'cancel', 'force_control',
                      'plan_only', 'teach_mode', 'position_command'}
            if (set(caps) != fields | {'backend_id'} or
                    not isinstance(caps['backend_id'], str) or not caps['backend_id'] or
                    any(type(caps[key]) is not bool for key in fields)):
                raise ValueError('invalid capabilities schema')
            if any(caps[key] for key in fields - supported):
                raise ValueError('unimplemented capability declared')
        if self.declare_parameter('motion_mode', 'joints').value != 'joints':
            raise ValueError('standard trajectory mode is unsupported by this HTTP backend')
        request_timeout = self.declare_parameter('request_timeout_seconds', 1.).value
        settle = self.declare_parameter('settling_seconds', .1).value
        stop_timeout = self.declare_parameter('stop_timeout_seconds', 1.).value
        self.workers = {
            kind: HttpWorker(self.declare_parameter(kind + '_url', url).value,
                            request_timeout, settle, stop_timeout)
            for kind, url in [('arm', 'http://127.0.0.1:18087'), ('hand', 'http://127.0.0.1:18088')]}
        self.lock = threading.RLock()
        self.lease = CommandLease(self.declare_parameter('ownership_id', 'zzxrobot').value)
        self.endpoint_leases = [CommandLease('http_' + hashlib.sha256(url.encode()).hexdigest()[:32])
                               for url in sorted({worker.url for worker in self.workers.values()})]
        self.active_gate = False
        self.configured = False
        self.ready = False
        self.ready_reason = 'unconfigured'
        self.last_probe = 0.
        self.next_probe = 0.
        self.probe_job = None
        self.probe_moving = False
        self.readiness_epoch = 0
        self.probe_epoch = -1
        self.busy = dict(arm=False, hand=False)
        self.latched = dict(arm=False, hand=False)
        self.pending = {}
        self.pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix='robot-http')
        self.stop_jobs = set()
        self.group = ReentrantCallbackGroup()
        self.publishers_by_kind = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for kind in ('arm', 'hand'):
            publisher = self.create_publisher(BackendCapabilities, '~/' + kind + '_capabilities', qos)
            publisher.publish(BackendCapabilities(**self.capabilities[kind]))
            self.publishers_by_kind[kind] = publisher
        self.servers = [
            ActionServer(self, action, name,
                         execute_callback=self.execute_arm if kind == 'arm' else self.execute_hand,
                         goal_callback=lambda goal, k=kind: self.admit(k, goal),
                         cancel_callback=lambda goal, k=kind: self.cancel(k, goal),
                         handle_accepted_callback=lambda goal, k=kind: self.accept(k, goal),
                         callback_group=self.group)
            for kind, action, name in [
                ('arm', MoveArm, 'zzx/manipulation/move_arm'),
                ('hand', ControlHand, 'zzx/manipulation/control_hand')]]
        self.service = self.create_service(Trigger, '~/get_status', self.status, callback_group=self.group)
        self.timer = self.create_timer(.02, self.tick, callback_group=self.group)

    def on_configure(self, state):
        with self.lock:
            self.configured = True
            self.readiness_epoch += 1
            self.last_probe = 0.
            self.ready = False
            self.ready_reason = 'waiting for read-only device checks'
            self.next_probe = 0.
        return Transition.SUCCESS

    def on_activate(self, state):
        with self.lock:
            if (not self.ready or time.monotonic() - self.last_probe > 1.5 or
                    self.probe_moving or any(self.busy.values()) or any(self.latched.values())):
                return Transition.FAILURE
            self.active_gate = True
        return Transition.SUCCESS

    def on_deactivate(self, state):
        with self.lock:
            self.active_gate = False
            for kind, (handle, pending) in list(self.pending.items()):
                self.cancel(kind, handle)
        return Transition.SUCCESS

    def on_cleanup(self, state):
        with self.lock:
            if any(self.busy.values()):
                return Transition.FAILURE
            self.configured = self.ready = self.active_gate = False
            self.readiness_epoch += 1
            self.ready_reason = 'unconfigured'
        return Transition.SUCCESS

    def on_shutdown(self, state):
        return self.on_deactivate(state)

    def probe(self):
        from zzx_http_contracts.protocol import vector
        try:
            deadline = time.monotonic() + 1.
            arm = self.workers['arm'].request('GET', '/api/status', None, deadline)
            vector([arm.get('left_joints', {}).get(n) for n in ARM_NAMES], 7)
            if arm.get('moveit_available') is not True or type(arm.get('moving')) is not bool:
                raise ValueError('arm status is not ready')
            motors = self.workers['arm'].request('GET', '/api/motors', None, deadline)
            if any(motors.get(n, {}).get('has_feedback') != 1 or
                   motors.get(n, {}).get('fault') != 0 for n in ARM_NAMES):
                raise ValueError('motor feedback unavailable or faulted')
            hand = self.workers['hand'].request('GET', '/api/status', None, deadline)
            vector(hand.get('position'), 10, normalized=True)
            if hand.get('connected') is not True:
                raise ValueError('hand is disconnected')
            return True, 'read-only checks passed', arm['moving']
        except Exception as error:
            return False, str(error), False

    def check_readiness(self):
        now = time.monotonic()
        if self.probe_job is not None and self.probe_job.done():
            valid, reason, moving = self.probe_job.result()
            self.probe_job = None
            if self.configured and self.probe_epoch == self.readiness_epoch:
                self.ready, self.ready_reason, self.probe_moving = valid, reason, moving
                self.last_probe = now
        if self.configured and self.probe_job is None and now >= self.next_probe:
            self.probe_epoch = self.readiness_epoch
            self.probe_job = self.pool.submit(self.probe)
            self.next_probe = now + .25
        if self.configured and now - self.last_probe > 1.5:
            self.ready, self.ready_reason = False, 'device heartbeat expired'
        if any(name == 'arm_controller' for name, namespace in self.get_node_names_and_namespaces()):
            self.ready, self.ready_reason = False, 'legacy arm_controller detected'
        if any(self.latched.values()):
            self.ready, self.ready_reason = False, 'backend outcome needs reconciliation'
        if self.active_gate and not self.ready:
            self.trigger_deactivate()

    def admit(self, kind, request):
        with self.lock:
            required = 'joint_target' if kind == 'arm' else 'position_command'
            if (not self.active_gate or not self.ready or time.monotonic() - self.last_probe > 1.5 or self.busy[kind] or
                    self.latched[kind] or not self.capabilities[kind].get(required)):
                return GoalResponse.REJECT
            self.busy[kind] = True
            return GoalResponse.ACCEPT

    def accept(self, kind, handle):
        with self.lock:
            self.pending[kind] = (handle, Pending())
        handle.execute()

    def cancel(self, kind, handle):
        with self.lock:
            entry = self.pending.get(kind)
            if entry is None or entry[0] is not handle:
                return CancelResponse.REJECT
            pending = entry[1]
            if not pending.cancel.is_set():
                pending.cancel.set()
                # Best effort, not a stop confirmation. The main job must still finish.
                if kind == 'arm':
                    worker = self.workers[kind]
                    job = self.pool.submit(worker.request, 'POST', '/api/cancel', {},
                                           time.monotonic() + worker.stop_timeout, True)
                    self.stop_jobs.add(job)
            return CancelResponse.ACCEPT

    def status(self, request, response):
        with self.lock:
            response.success = True
            response.message = json.dumps(dict(busy=self.busy, latched=self.latched,
                                               active=self.active_gate, ready=self.ready,
                                               reason=self.ready_reason))
        return response

    def tick(self):
        with self.lock:
            self.check_readiness()
            for job in list(self.stop_jobs):
                if job.done():
                    self.stop_jobs.remove(job)
            for kind, (handle, pending) in self.pending.items():
                if pending.job is not None and pending.job.done() and not pending.done.done():
                    try:
                        outcome = pending.job.result()
                    except Exception as error:
                        outcome = Outcome(E.INTERNAL, str(error), locked=True)
                    pending.done.set_result(outcome)
                if handle.is_active:
                    feedback = MoveArm.Feedback() if kind == 'arm' else ControlHand.Feedback()
                    feedback.execution = self.execution(handle, pending.stage)
                    handle.publish_feedback(feedback)

    def execution(self, handle, stage):
        result = S()
        result.header.stamp = self.get_clock().now().to_msg()
        result.operation_id = bytes(handle.goal_id.uuid).hex()
        result.stage = stage
        result.state = {'validating': S.STATE_VALIDATING, 'planning': S.STATE_PLANNING,
                        'settling': S.STATE_SETTLING}.get(stage, S.STATE_EXECUTING)
        result.progress = .95 if stage == 'settling' else 0.
        return result

    async def execute_arm(self, handle):
        return await self.execute('arm', handle)

    async def execute_hand(self, handle):
        return await self.execute('hand', handle)

    async def execute(self, kind, handle):
        with self.lock:
            pending = self.pending[kind][1]
        invalid = (validate_arm if kind == 'arm' else validate_hand)(handle.request)
        if kind == 'arm' and handle.request.plan_only and not self.capabilities[kind]['plan_only']:
            invalid = Outcome(E.UNSUPPORTED, 'plan_only capability disabled')
        if invalid is not None:
            outcome = invalid
        else:
            def progress(stage):
                with self.lock:
                    pending.stage = stage

            with self.lock:
                pending.job = self.pool.submit(getattr(self.workers[kind], kind),
                                               handle.request, pending.cancel, progress)
            outcome = await pending.done
        # Cancellation racing a completed worker must never be reported as success.
        if pending.cancel.is_set() and outcome.code == E.OK:
            outcome = Outcome(E.STOP_UNCONFIRMED, 'cancel raced completion; verify backend', locked=True)
        if outcome.code == E.CANCELED and not handle.is_cancel_requested:
            outcome = Outcome(E.EXECUTION_FAILED, 'lifecycle stop; ' + outcome.message,
                              outcome.positions, outcome.locked)
        result = MoveArm.Result() if kind == 'arm' else ControlHand.Result()
        result.success = outcome.code == E.OK
        result.error.code, result.error.message = outcome.code, outcome.message
        result.execution = self.execution(handle, outcome.message)
        result.execution.error = result.error
        result.execution.state = (S.STATE_SUCCEEDED if result.success else
                                  S.STATE_CANCELED if outcome.code == E.CANCELED else
                                  S.STATE_NEEDS_RECONCILIATION if outcome.code == E.RESULT_UNKNOWN else S.STATE_FAILED)
        result.execution.progress = 1. if result.success else 0.
        if kind == 'arm' and outcome.positions:
            result.final_joint_state.name = ARM_NAMES
            result.final_joint_state.position = outcome.positions
            result.final_joint_state.header.stamp = self.get_clock().now().to_msg()
        elif kind == 'hand':
            result.final_positions = outcome.positions
        with self.lock:
            if result.success:
                handle.succeed()
            elif outcome.code == E.CANCELED and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            del self.pending[kind]
            self.latched[kind] = outcome.locked
            self.busy[kind] = False
        return result

    def close(self):
        with self.lock:
            self.active_gate = self.configured = False
            for handle, pending in self.pending.values():
                pending.cancel.set()
        self.pool.shutdown(wait=True, cancel_futures=True)
        for server in self.servers:
            server.destroy()
        self.destroy_node()
        for lease in self.endpoint_leases:
            lease.close()
        self.lease.close()


def main():
    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = HttpActionBackend()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
        executor.shutdown(timeout_sec=3)
        if rclpy.ok():
            rclpy.shutdown()
