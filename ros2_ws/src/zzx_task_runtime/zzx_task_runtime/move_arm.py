"""Adapter for the existing C++ MoveArm action; no task/vision semantics are invented."""
import math
import time


def normalize(request, action_name):
    fields = {'joint_names', 'positions', 'velocity_scaling',
              'acceleration_scaling', 'plan_only', 'timeout_seconds'}
    if not isinstance(request, dict) or set(request) != fields:
        raise ValueError('request must contain exactly: ' + ', '.join(sorted(fields)))
    names, positions = request['joint_names'], request['positions']
    if not isinstance(names, list) or not isinstance(positions, list) or not names:
        raise ValueError('joint_names and positions must be nonempty lists')
    if not all(isinstance(name, str) and name for name in names):
        raise ValueError('joint names must be nonempty strings')
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('joint names must be unique and match positions')

    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('numeric fields must be finite numbers')
        return float(value)

    pairs = sorted(zip(names, [number(value) for value in positions]))
    velocity = number(request['velocity_scaling'])
    acceleration = number(request['acceleration_scaling'])
    timeout = number(request['timeout_seconds'])
    if not 0 < velocity <= 1 or not 0 < acceleration <= 1 or not 0 < timeout <= 86400:
        raise ValueError('scalings must be in (0,1]; timeout must be in (0,86400] seconds')
    if type(request['plan_only']) is not bool:
        raise ValueError('plan_only must be a boolean')
    if not isinstance(action_name, str) or not action_name.startswith('/'):
        raise ValueError('action name must be absolute')
    return dict(schema_version=1, action_name=action_name,
                joint_names=[pair[0] for pair in pairs], positions=[pair[1] for pair in pairs],
                velocity_scaling=velocity, acceleration_scaling=acceleration,
                plan_only=request['plan_only'], timeout_seconds=timeout)


class RosMoveArmBackend:
    """Synchronous gateway adapter; invoke only once at a time through a Ledger."""

    def __init__(self, wait_seconds=10):
        import rclpy
        if not math.isfinite(wait_seconds) or wait_seconds <= 0:
            raise ValueError('wait_seconds must be finite and positive')
        self.node = rclpy.create_node('durable_move_arm_client')
        self.wait_seconds = wait_seconds

    def close(self):
        self.node.destroy_node()

    def __call__(self, payload, operation_id):
        import rclpy
        from rclpy.action import ActionClient
        from rosidl_runtime_py.convert import message_to_ordereddict
        from unique_identifier_msgs.msg import UUID
        from zzx_interfaces.action import MoveArm
        from zzx_interfaces.msg import ErrorStatus, ExecutionStatus

        client = ActionClient(self.node, MoveArm, payload['action_name'])
        deadline = time.monotonic() + self.wait_seconds

        def wait(future):
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=min(.02, max(0, deadline - time.monotonic())))
            if not future.done():
                raise TimeoutError('backend response deadline; physical result is unknown')
            return future.result()

        try:
            if not client.wait_for_server(timeout_sec=min(2, self.wait_seconds)):
                return {'state': 'FAILED', 'reason': 'action server unavailable; nothing sent'}
            goal = MoveArm.Goal()
            goal.target_type = MoveArm.Goal.TARGET_JOINTS
            goal.joint_target.name = payload['joint_names']
            goal.joint_target.position = payload['positions']
            goal.velocity_scaling = payload['velocity_scaling']
            goal.acceleration_scaling = payload['acceleration_scaling']
            goal.plan_only = payload['plan_only']
            sec, nanosec = divmod(round(payload['timeout_seconds'] * 1e9), 1000000000)
            goal.timeout.sec, goal.timeout.nanosec = sec, nanosec
            handle = wait(client.send_goal_async(goal, goal_uuid=UUID(
                uuid=list(bytes.fromhex(operation_id)))))
            if not handle.accepted:
                return {'state': 'FAILED', 'reason': 'goal rejected; no execution accepted'}
            response = wait(handle.get_result_async())
            result = response.result
            state = 'NEEDS_RECONCILIATION'
            if (result.execution.operation_id == operation_id and
                    result.execution.error.code == result.error.code):
                if (response.status == 4 and result.success and
                        result.error.code == ErrorStatus.OK and
                        result.execution.state == ExecutionStatus.STATE_SUCCEEDED):
                    state = 'SUCCEEDED'
                elif (response.status == 5 and not result.success and
                        result.error.code == ErrorStatus.CANCELED and
                        result.execution.state == ExecutionStatus.STATE_CANCELED):
                    state = 'CANCELED'
                elif (response.status == 6 and not result.success and
                        result.execution.state == ExecutionStatus.STATE_FAILED and
                        result.error.code not in {
                        ErrorStatus.OK, ErrorStatus.TIMEOUT, ErrorStatus.STOP_UNCONFIRMED,
                        ErrorStatus.RESULT_UNKNOWN}):
                    state = 'FAILED'
            return {'state': state, 'action_status': response.status,
                    'result': message_to_ordereddict(result)}
        finally:
            client.destroy()
