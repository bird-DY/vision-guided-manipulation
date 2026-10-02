"""Blocking HTTP work runs outside the ROS executor. No HTTP mutation is retried."""
from dataclasses import dataclass, field
import math
import time

from zzx_interfaces.msg import ErrorStatus as E
from zzx_http_contracts.client import LoopbackClient, ReportedFailure, UnknownOutcome
from zzx_http_contracts.protocol import ARM_NAMES, HAND_NAMES, joint_payload, vector


@dataclass
class Outcome:
    code: int
    message: str
    positions: list = field(default_factory=list)
    locked: bool = False


class StopRequested(Exception):
    pass


def timeout_seconds(duration):
    value = duration.sec + duration.nanosec * 1e-9
    if duration.sec < 0 or duration.nanosec >= 1000000000 or not 0 < value <= 86400:
        raise ValueError('timeout must be in (0,86400] seconds')
    return value


def validate_arm(goal):
    if goal.target_type != goal.TARGET_JOINTS:
        return Outcome(E.UNSUPPORTED, 'only joint targets are supported')
    from geometry_msgs.msg import PoseStamped
    if goal.pose_target != PoseStamped() or goal.joint_target.velocity or goal.joint_target.effort:
        return Outcome(E.INVALID_ARGUMENT, 'mixed targets or velocity/effort fields are not supported')
    names, positions = goal.joint_target.name, list(goal.joint_target.position)
    if len(names) != 7 or len(set(names)) != 7 or set(names) != set(ARM_NAMES):
        return Outcome(E.INVALID_ARGUMENT, 'all seven named left joints required')
    try:
        timeout_seconds(goal.timeout)
        joint_payload(dict(zip(names, vector(positions, 7))), goal.plan_only,
                      goal.velocity_scaling, goal.acceleration_scaling)
    except ValueError as error:
        return Outcome(E.INVALID_ARGUMENT, str(error))
    return None


def validate_hand(goal):
    if (not math.isfinite(goal.speed_scaling) or not 0 < goal.speed_scaling <= 1 or
            not math.isfinite(goal.max_effort) or goal.max_effort < 0):
        return Outcome(E.INVALID_ARGUMENT, 'invalid hand speed or effort')
    if (goal.position_unit != goal.UNIT_NORMALIZED or goal.stop_on_contact or
            goal.max_effort != 0 or goal.speed_scaling != 1):
        return Outcome(E.UNSUPPORTED, 'only normalized positions with speed=1, effort=0, no contact stop')
    if (not goal.joint_names or len(set(goal.joint_names)) != len(goal.joint_names) or
            not set(goal.joint_names) <= set(HAND_NAMES)):
        return Outcome(E.INVALID_ARGUMENT, 'unique known hand names required')
    try:
        timeout_seconds(goal.timeout)
        vector(list(goal.positions), len(goal.joint_names), normalized=True)
    except ValueError as error:
        return Outcome(E.INVALID_ARGUMENT, str(error))
    return None


class HttpWorker:
    def __init__(self, url, request_timeout=1., settle=.1, stop_timeout=1.):
        LoopbackClient(url, request_timeout)  # Reject non-loopback without connecting.
        if any(not math.isfinite(v) or v <= 0 for v in (settle, stop_timeout)):
            raise ValueError('timing parameters must be finite and positive')
        if stop_timeout <= settle:
            raise ValueError('stop timeout must exceed settling time')
        self.url, self.request_timeout = url, request_timeout
        self.settle, self.stop_timeout = settle, stop_timeout

    def request(self, method, path, payload, deadline, command=False):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('operation deadline exceeded')
        client = LoopbackClient(self.url, min(self.request_timeout, remaining))
        return client.command(path, payload) if command else client.request(method, path, payload)

    def arm_samples(self, deadline, target, cancel=None):
        previous = None
        stable = None
        last_stamp = None
        while time.monotonic() < deadline:
            if cancel is not None and cancel.is_set():
                raise StopRequested()
            requested = time.monotonic()
            data = self.request('GET', '/api/status', None, deadline)
            if time.monotonic() - requested > .2:
                raise UnknownOutcome('feedback request exceeded freshness budget')
            stamp = data.get('timestamp')
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
                    not math.isfinite(stamp) or (last_stamp is not None and stamp <= last_stamp)):
                raise UnknownOutcome('feedback timestamp did not advance')
            positions = vector([data.get('left_joints', {}).get(n) for n in ARM_NAMES], 7)
            if type(data.get('moving')) is not bool:
                raise UnknownOutcome('moving feedback missing')
            now = time.monotonic()
            if previous and now - previous[0] > .2:
                stable = None
                previous = None
            stopped = not data['moving'] and previous is not None
            if previous:
                dt = now - previous[0]
                stopped = stopped and all(abs(a-b) / dt <= .01 for a, b in zip(positions, previous[1]))
            reached = target is None or all(abs(a-b) <= .001 for a, b in zip(positions, target))
            if stopped and reached:
                stable = now if stable is None else stable
                if now - stable >= self.settle:
                    return positions
            else:
                stable = None
            previous, last_stamp = (now, positions), stamp
            time.sleep(.02)
        raise TimeoutError('feedback did not settle before deadline')

    def confirm_arm_stop(self, code, message, progress):
        progress('confirming_stop')
        deadline = time.monotonic() + self.stop_timeout
        try:
            self.request('POST', '/api/cancel', {}, deadline, True)
            positions = self.arm_samples(deadline, None)
            return Outcome(code, message + '; stop confirmed', positions)
        except Exception as error:
            return Outcome(E.STOP_UNCONFIRMED, message + '; ' + str(error), locked=True)

    def arm(self, goal, cancel, progress):
        deadline = time.monotonic() + timeout_seconds(goal.timeout)
        sent = False

        def checkpoint():
            if cancel.is_set():
                raise StopRequested()
            if time.monotonic() >= deadline:
                raise TimeoutError('operation deadline exceeded')

        try:
            checkpoint()
            if not goal.plan_only:
                progress('preparing')
                for path, payload in [('/api/teach_mode', {'enable': False}),
                                      ('/api/control_mode', {'mode': 'pos_vel'}), ('/api/enable', {})]:
                    checkpoint()
                    self.request('POST', path, payload, deadline, True)
                motors = self.request('GET', '/api/motors', None, deadline)
                if any(motors.get(n, {}).get('enabled') != 1 or
                       motors.get(n, {}).get('has_feedback') != 1 or
                       motors.get(n, {}).get('fault') != 0 for n in ARM_NAMES):
                    raise UnknownOutcome('left motor readiness unconfirmed')
            checkpoint()
            progress('planning' if goal.plan_only else 'executing')
            payload = joint_payload(dict(zip(goal.joint_target.name, goal.joint_target.position)),
                                    goal.plan_only, goal.velocity_scaling, goal.acceleration_scaling)
            sent = not goal.plan_only
            self.request('POST', '/api/joints', payload, deadline, True)
            checkpoint()
            if goal.plan_only:
                return Outcome(E.OK, 'planned_only')
            progress('settling')
            positions = self.arm_samples(deadline, payload['left_joints'], cancel)
            checkpoint()
            return Outcome(E.OK, 'target reached and settled', positions)
        except StopRequested:
            return (self.confirm_arm_stop(E.CANCELED, 'canceled', progress) if sent else
                    Outcome(E.CANCELED, 'canceled before joint command'))
        except ReportedFailure as error:
            if sent:
                return self.confirm_arm_stop(E.CANCELED if cancel.is_set() else E.EXECUTION_FAILED,
                                             str(error), progress)
            return Outcome(E.BACKEND_REJECTED, str(error))
        except Exception as error:
            # A timed-out POST may still arrive after a cancel. Never claim confirmed stop.
            if sent:
                try:
                    self.request('POST', '/api/cancel', {}, time.monotonic() + self.stop_timeout, True)
                except Exception:
                    pass
            return Outcome(E.RESULT_UNKNOWN if sent else E.BACKEND_UNAVAILABLE,
                           str(error), locked=True)

    def hand(self, goal, cancel, progress):
        deadline = time.monotonic() + timeout_seconds(goal.timeout)
        sent = False
        try:
            if cancel.is_set():
                return Outcome(E.CANCELED, 'canceled before hand command')
            current = self.request('GET', '/api/pose', None, deadline)
            positions = vector(current.get('position'), 10, normalized=True)
            for name, value in zip(goal.joint_names, goal.positions):
                positions[HAND_NAMES.index(name)] = value
            if cancel.is_set():
                return Outcome(E.CANCELED, 'canceled before hand command')
            progress('executing')
            sent = True
            self.request('POST', '/api/set_pos', {'position': positions}, deadline, True)
            stable = None
            while time.monotonic() < deadline:
                if cancel.is_set():
                    return Outcome(E.STOP_UNCONFIRMED, 'hand has no verified stop API', locked=True)
                actual = vector(self.request('GET', '/api/pose', None, deadline).get('position'),
                                10, normalized=True)
                now = time.monotonic()
                if all(abs(a-b) <= .001 for a, b in zip(actual, positions)):
                    stable = now if stable is None else stable
                    if now - stable >= self.settle:
                        return Outcome(E.OK, 'hand position readback matched',
                                       [actual[HAND_NAMES.index(n)] for n in goal.joint_names])
                else:
                    stable = None
                progress('settling')
                time.sleep(.02)
            raise TimeoutError('hand readback timeout')
        except Exception as error:
            return Outcome(E.RESULT_UNKNOWN if sent else E.BACKEND_UNAVAILABLE, str(error), locked=True)
