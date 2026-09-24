"""Semantic validation beyond what ROS 2 IDL can express."""

from __future__ import annotations

import math
from typing import Iterable

from zzx_interfaces.action import ControlHand, MoveArm
from zzx_interfaces.msg import BackendCapabilities
from zzx_interfaces.msg import ErrorStatus, ExecutionStatus, ObjectTarget
from zzx_interfaces.msg import SubsystemStatus, TaskCommand


class ContractError(ValueError):
    """Raised when a ROS message violates the public contract."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def _finite(values: Iterable[float], field: str) -> None:
    _require(all(math.isfinite(value) for value in values), f'{field} must be finite')


def _unit_interval(value: float, field: str) -> None:
    _require(math.isfinite(value) and 0.0 <= value <= 1.0, f'{field} must be in [0, 1]')


def _positive_duration(duration, field: str = 'timeout') -> None:
    _require(duration.sec >= 0, f'{field}.sec must be non-negative')
    _require(duration.nanosec < 1_000_000_000, f'{field}.nanosec is invalid')
    _require(duration.sec > 0 or duration.nanosec > 0, f'{field} must be positive')


def _timestamp(stamp, field: str) -> None:
    _require(stamp.sec >= 0, f'{field}.sec must be non-negative')
    _require(stamp.nanosec < 1_000_000_000, f'{field}.nanosec is invalid')
    _require(stamp.sec > 0 or stamp.nanosec > 0, f'{field} must be non-zero')


def _quaternion(values: Iterable[float], field: str) -> None:
    values = tuple(values)
    _finite(values, field)
    norm = math.sqrt(sum(value * value for value in values))
    _require(abs(norm - 1.0) <= 1e-3, f'{field} must be normalized')


def _joint_state(state, field: str) -> None:
    count = len(state.name)
    _require(count > 0, f'{field}.name must not be empty')
    _require(len(set(state.name)) == count, f'{field}.name contains duplicates')
    _require(len(state.position) == count, f'{field}.position length must match name')
    for optional in ('velocity', 'effort'):
        values = getattr(state, optional)
        _require(
            len(values) in (0, count),
            f'{field}.{optional} must be empty or match name length',
        )
        _finite(values, f'{field}.{optional}')
    _finite(state.position, f'{field}.position')


def validate_error_status(error: ErrorStatus) -> None:
    """Validate shared result error semantics."""
    allowed = {
        ErrorStatus.OK,
        ErrorStatus.INVALID_ARGUMENT,
        ErrorStatus.UNSUPPORTED,
        ErrorStatus.NOT_READY,
        ErrorStatus.BUSY,
        ErrorStatus.RESOURCE_CONFLICT,
        ErrorStatus.PLANNING_FAILED,
        ErrorStatus.EXECUTION_FAILED,
        ErrorStatus.TIMEOUT,
        ErrorStatus.CANCELED,
        ErrorStatus.STOP_UNCONFIRMED,
        ErrorStatus.STALE_DATA,
        ErrorStatus.CALIBRATION_INVALID,
        ErrorStatus.TARGET_NOT_FOUND,
        ErrorStatus.QUALITY_TOO_LOW,
        ErrorStatus.BACKEND_UNAVAILABLE,
        ErrorStatus.BACKEND_REJECTED,
        ErrorStatus.RESULT_UNKNOWN,
        ErrorStatus.INTERNAL,
    }
    _require(error.code in allowed, 'error.code is unknown')
    if error.code != ErrorStatus.OK:
        _require(bool(error.message.strip()), 'non-success error requires a message')


def validate_execution_status(status: ExecutionStatus) -> None:
    """Validate progress and terminal-state/error consistency."""
    valid_states = {
        ExecutionStatus.STATE_UNKNOWN,
        ExecutionStatus.STATE_ACCEPTED,
        ExecutionStatus.STATE_VALIDATING,
        ExecutionStatus.STATE_PLANNING,
        ExecutionStatus.STATE_EXECUTING,
        ExecutionStatus.STATE_SETTLING,
        ExecutionStatus.STATE_SUCCEEDED,
        ExecutionStatus.STATE_FAILED,
        ExecutionStatus.STATE_CANCELED,
        ExecutionStatus.STATE_NEEDS_RECONCILIATION,
    }
    _require(status.state in valid_states, 'execution.state is unknown')
    _unit_interval(status.progress, 'execution.progress')
    validate_error_status(status.error)
    if status.state != ExecutionStatus.STATE_UNKNOWN:
        _require(bool(status.operation_id.strip()), 'operation_id is required')
    if status.state == ExecutionStatus.STATE_SUCCEEDED:
        _require(status.error.code == ErrorStatus.OK, 'succeeded state requires OK')
        _require(status.progress == 1.0, 'succeeded state requires progress=1')
    if status.state == ExecutionStatus.STATE_FAILED:
        _require(status.error.code != ErrorStatus.OK, 'failed state requires an error')
    if status.state == ExecutionStatus.STATE_CANCELED:
        _require(status.error.code == ErrorStatus.CANCELED, 'canceled state requires CANCELED')
    if status.state == ExecutionStatus.STATE_NEEDS_RECONCILIATION:
        _require(
            status.error.code == ErrorStatus.RESULT_UNKNOWN,
            'reconciliation state requires RESULT_UNKNOWN',
        )


def validate_action_result(success: bool, status: ExecutionStatus) -> None:
    """Require the convenience success flag to match the action terminal state."""
    validate_execution_status(status)
    expected = status.state == ExecutionStatus.STATE_SUCCEEDED
    _require(success == expected, 'success flag contradicts execution state')


def validate_backend_capabilities(capabilities: BackendCapabilities) -> None:
    """Validate a backend's explicit, conservative capability declaration."""
    _require(bool(capabilities.backend_id.strip()), 'backend_id is required')
    if capabilities.plan_only:
        _require(
            capabilities.joint_target or capabilities.pose_target,
            'plan_only requires a supported target type',
        )


def validate_subsystem_status(status: SubsystemStatus) -> None:
    """Validate status enums, diagnostics and nested capability declarations."""
    valid_states = {
        SubsystemStatus.STATE_UNKNOWN,
        SubsystemStatus.STATE_READY,
        SubsystemStatus.STATE_BUSY,
        SubsystemStatus.STATE_DEGRADED,
        SubsystemStatus.STATE_ERROR,
        SubsystemStatus.STATE_ESTOPPED,
    }
    _require(bool(status.name.strip()), 'subsystem name is required')
    _require(status.state in valid_states, 'subsystem state is unknown')
    _require(
        len(status.diagnostic_keys) == len(status.diagnostic_values),
        'diagnostic key/value lengths differ',
    )
    _require(
        len(set(status.diagnostic_keys)) == len(status.diagnostic_keys),
        'diagnostic keys contain duplicates',
    )
    validate_backend_capabilities(status.capabilities)
    if status.execution.state != ExecutionStatus.STATE_UNKNOWN:
        validate_execution_status(status.execution)


def validate_object_target(target: ObjectTarget) -> None:
    """Validate an RGB-D target and its coordinate provenance."""
    _require(bool(target.object_id.strip()), 'object_id is required')
    _unit_interval(target.confidence, 'confidence')
    _timestamp(target.image_header.stamp, 'image_header.stamp')
    _require(target.image_width > 0 and target.image_height > 0, 'image size is required')
    _require(bool(target.image_header.frame_id.strip()), 'image_header.frame_id is required')
    roi = target.roi
    _require(roi.width > 0 and roi.height > 0, 'roi must have positive area')
    _require(roi.x_offset + roi.width <= target.image_width, 'roi exceeds image width')
    _require(roi.y_offset + roi.height <= target.image_height, 'roi exceeds image height')

    quality = target.quality
    _unit_interval(quality.score, 'quality.score')
    _unit_interval(quality.valid_depth_ratio, 'quality.valid_depth_ratio')
    _require(quality.sample_count > 0, 'quality.sample_count must be positive')
    _require(bool(quality.method_id.strip()), 'quality.method_id is required')

    if target.depth_valid:
        _require(math.isfinite(target.depth_m) and target.depth_m > 0, 'depth_m is invalid')
    if target.position_valid or target.orientation_valid or target.depth_valid:
        _require(bool(target.calibration_id.strip()), 'calibration_id is required')
    if target.position_valid or target.orientation_valid:
        _require(bool(target.header.frame_id.strip()), 'target frame_id is required')
        position = target.pose.pose.position
        _finite((position.x, position.y, position.z), 'pose.position')
        _finite(target.pose.covariance, 'pose.covariance')
    if target.position_valid:
        _require(
            math.isfinite(quality.position_std_m) and quality.position_std_m >= 0,
            'quality.position_std_m is invalid',
        )
    if target.orientation_valid:
        orientation = target.pose.pose.orientation
        _quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            'pose.orientation',
        )
        _require(
            math.isfinite(quality.orientation_std_rad)
            and quality.orientation_std_rad >= 0,
            'quality.orientation_std_rad is invalid',
        )


def validate_move_arm_goal(goal: MoveArm.Goal) -> None:
    """Validate strict target exclusivity and arm command units."""
    _require(goal.target_type in (goal.TARGET_JOINTS, goal.TARGET_POSE), 'target_type is invalid')
    _require(
        math.isfinite(goal.velocity_scaling) and 0.0 < goal.velocity_scaling <= 1.0,
        'velocity_scaling must be in (0, 1]',
    )
    _require(
        math.isfinite(goal.acceleration_scaling)
        and 0.0 < goal.acceleration_scaling <= 1.0,
        'acceleration_scaling must be in (0, 1]',
    )
    _positive_duration(goal.timeout)

    if goal.target_type == goal.TARGET_JOINTS:
        _joint_state(goal.joint_target, 'joint_target')
        _require(
            not goal.pose_target.header.frame_id,
            'pose_target must be unset for joint target',
        )
    else:
        _require(
            not goal.joint_target.name and not goal.joint_target.position,
            'joint_target must be unset for pose target',
        )
        _require(
            bool(goal.pose_target.header.frame_id.strip()),
            'pose target frame_id is required',
        )
        position = goal.pose_target.pose.position
        orientation = goal.pose_target.pose.orientation
        _finite((position.x, position.y, position.z), 'pose_target.position')
        _quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            'pose_target.orientation',
        )


def validate_control_hand_goal(goal: ControlHand.Goal) -> None:
    """Validate sparse named hand commands without inventing force support."""
    _require(
        goal.position_unit in (goal.UNIT_NORMALIZED, goal.UNIT_RADIANS),
        'position_unit is invalid',
    )
    _require(bool(goal.joint_names), 'joint_names must not be empty')
    _require(
        len(set(goal.joint_names)) == len(goal.joint_names),
        'joint_names contains duplicates',
    )
    _require(
        len(goal.positions) == len(goal.joint_names),
        'positions length must match joint_names',
    )
    _finite(goal.positions, 'positions')
    if goal.position_unit == goal.UNIT_NORMALIZED:
        _require(
            all(0.0 <= value <= 1.0 for value in goal.positions),
            'normalized positions are out of range',
        )
    _require(
        math.isfinite(goal.speed_scaling) and 0.0 < goal.speed_scaling <= 1.0,
        'speed_scaling must be in (0, 1]',
    )
    _require(math.isfinite(goal.max_effort) and goal.max_effort >= 0, 'max_effort is invalid')
    _positive_duration(goal.timeout)


def validate_task_command(command: TaskCommand) -> None:
    """Validate the stable core and bounded extension map of a task."""
    _require(command.schema_version == 2, 'schema_version must be 2')
    _require(bool(command.task_id.strip()), 'task_id is required')
    _require(bool(command.robot_profile_id.strip()), 'robot_profile_id is required')
    valid_types = {
        command.TASK_NAVIGATE,
        command.TASK_INSPECT,
        command.TASK_PICK_PLACE,
        command.TASK_PRESS_BUTTON,
        command.TASK_TOGGLE_SWITCH,
        command.TASK_SORT_OBJECT,
    }
    _require(command.task_type in valid_types, 'task_type is invalid')
    _require(
        len(command.parameter_keys) == len(command.parameter_values),
        'parameter key/value lengths differ',
    )
    _require(
        len(set(command.parameter_keys)) == len(command.parameter_keys),
        'parameter keys contain duplicates',
    )
