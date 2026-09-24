"""Contract validation and ROS serialization tests."""

import math

import pytest
from rclpy.serialization import deserialize_message, serialize_message

from zzx_contracts.validation import ContractError, validate_action_result
from zzx_contracts.validation import validate_backend_capabilities
from zzx_contracts.validation import validate_control_hand_goal
from zzx_contracts.validation import validate_execution_status
from zzx_contracts.validation import validate_move_arm_goal
from zzx_contracts.validation import validate_object_target
from zzx_contracts.validation import validate_subsystem_status
from zzx_contracts.validation import validate_task_command
from zzx_interfaces.action import ControlHand, MoveArm
from zzx_interfaces.msg import BackendCapabilities, ErrorStatus, ExecutionStatus
from zzx_interfaces.msg import ObjectTarget, SubsystemStatus, TaskCommand


def valid_target() -> ObjectTarget:
    target = ObjectTarget()
    target.header.frame_id = 'camera_color_optical_frame'
    target.image_header.frame_id = 'camera_color_optical_frame'
    target.image_header.stamp.sec = 1
    target.image_width = 1280
    target.image_height = 720
    target.object_id = 'block-1'
    target.class_id = 1
    target.class_name = 'block'
    target.confidence = 0.9
    target.roi.x_offset = 100
    target.roi.y_offset = 80
    target.roi.width = 200
    target.roi.height = 160
    target.depth_m = 0.4
    target.depth_valid = True
    target.position_valid = True
    target.orientation_valid = False
    target.pose.pose.position.z = 0.4
    target.calibration_id = 'gemini335-1280x720-v1'
    target.quality.score = 0.8
    target.quality.position_std_m = 0.004
    target.quality.orientation_std_rad = math.nan
    target.quality.valid_depth_ratio = 0.7
    target.quality.sample_count = 120
    target.quality.method_id = 'roi_median_depth_v1'
    return target


def valid_joint_goal() -> MoveArm.Goal:
    goal = MoveArm.Goal()
    goal.target_type = goal.TARGET_JOINTS
    goal.joint_target.name = ['joint2', 'joint1']
    goal.joint_target.position = [0.2, -0.1]
    goal.velocity_scaling = 0.2
    goal.acceleration_scaling = 0.2
    goal.timeout.sec = 5
    return goal


def test_object_target_serializes_and_validates():
    target = valid_target()
    restored = deserialize_message(serialize_message(target), ObjectTarget)
    validate_object_target(restored)
    assert restored.calibration_id == target.calibration_id


@pytest.mark.parametrize('field', ['confidence', 'quality.score', 'quality.valid_depth_ratio'])
def test_object_target_rejects_invalid_probabilities(field):
    target = valid_target()
    owner = target
    parts = field.split('.')
    for part in parts[:-1]:
        owner = getattr(owner, part)
    setattr(owner, parts[-1], 1.1)
    with pytest.raises(ContractError):
        validate_object_target(target)


def test_object_target_rejects_roi_outside_image():
    target = valid_target()
    target.roi.x_offset = 1200
    with pytest.raises(ContractError, match='image width'):
        validate_object_target(target)


def test_object_target_rejects_missing_calibration():
    target = valid_target()
    target.calibration_id = ''
    with pytest.raises(ContractError, match='calibration_id'):
        validate_object_target(target)


def test_object_target_rejects_missing_image_time():
    target = valid_target()
    target.image_header.stamp.sec = 0
    with pytest.raises(ContractError, match='non-zero'):
        validate_object_target(target)


def test_move_arm_accepts_name_mapped_joint_goal():
    validate_move_arm_goal(valid_joint_goal())


@pytest.mark.parametrize('value', [0.0, -0.1, 1.1, math.nan])
def test_move_arm_rejects_invalid_scaling(value):
    goal = valid_joint_goal()
    goal.velocity_scaling = value
    with pytest.raises(ContractError, match='velocity_scaling'):
        validate_move_arm_goal(goal)


def test_move_arm_rejects_duplicate_joint_names():
    goal = valid_joint_goal()
    goal.joint_target.name = ['joint1', 'joint1']
    with pytest.raises(ContractError, match='duplicates'):
        validate_move_arm_goal(goal)


def test_move_arm_rejects_nan_joint():
    goal = valid_joint_goal()
    goal.joint_target.position[0] = math.nan
    with pytest.raises(ContractError, match='finite'):
        validate_move_arm_goal(goal)


def test_move_arm_rejects_mixed_targets():
    goal = valid_joint_goal()
    goal.pose_target.header.frame_id = 'base_link'
    with pytest.raises(ContractError, match='pose_target'):
        validate_move_arm_goal(goal)


def test_control_hand_supports_sparse_named_update():
    goal = ControlHand.Goal()
    goal.position_unit = goal.UNIT_NORMALIZED
    goal.joint_names = ['index_pip']
    goal.positions = [0.5]
    goal.speed_scaling = 0.2
    goal.max_effort = 0.0
    goal.timeout.sec = 2
    validate_control_hand_goal(goal)


def test_control_hand_rejects_vector_mismatch():
    goal = ControlHand.Goal()
    goal.position_unit = goal.UNIT_NORMALIZED
    goal.joint_names = ['index_pip', 'middle_pip']
    goal.positions = [0.5]
    goal.speed_scaling = 0.2
    goal.timeout.sec = 2
    with pytest.raises(ContractError, match='length'):
        validate_control_hand_goal(goal)


def test_execution_terminal_state_is_consistent():
    status = ExecutionStatus()
    status.operation_id = 'op-1'
    status.state = status.STATE_SUCCEEDED
    status.progress = 1.0
    status.error.code = ErrorStatus.OK
    validate_execution_status(status)


def test_execution_failure_requires_error():
    status = ExecutionStatus()
    status.operation_id = 'op-1'
    status.state = status.STATE_FAILED
    status.progress = 0.5
    status.error.code = ErrorStatus.OK
    with pytest.raises(ContractError, match='requires an error'):
        validate_execution_status(status)


def test_action_success_must_match_terminal_state():
    status = ExecutionStatus()
    status.operation_id = 'op-1'
    status.state = status.STATE_FAILED
    status.progress = 0.5
    status.error.code = ErrorStatus.EXECUTION_FAILED
    status.error.message = 'controller rejected trajectory'
    with pytest.raises(ContractError, match='contradicts'):
        validate_action_result(True, status)


def test_backend_capabilities_require_identity():
    capabilities = BackendCapabilities()
    capabilities.joint_target = True
    with pytest.raises(ContractError, match='backend_id'):
        validate_backend_capabilities(capabilities)


def test_subsystem_status_rejects_diagnostic_mismatch():
    status = SubsystemStatus()
    status.name = 'arm'
    status.state = status.STATE_READY
    status.capabilities.backend_id = 'fake_arm'
    status.diagnostic_keys = ['feedback_age_s']
    with pytest.raises(ContractError, match='key/value'):
        validate_subsystem_status(status)


def test_task_command_rejects_duplicate_extension_keys():
    command = TaskCommand()
    command.schema_version = 2
    command.task_id = 'task-1'
    command.robot_profile_id = 'zzx_sim_mobile_6dof'
    command.task_type = command.TASK_INSPECT
    command.parameter_keys = ['threshold', 'threshold']
    command.parameter_values = ['0.7', '0.8']
    with pytest.raises(ContractError, match='duplicates'):
        validate_task_command(command)
