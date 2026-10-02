"""Deterministic worker boundaries, independent of ROS executors and hardware."""
import threading
import time

import pytest
from zzx_interfaces.action import MoveArm, ControlHand
from zzx_interfaces.msg import ErrorStatus as E
from zzx_http_contracts.client import UnknownOutcome
from zzx_http_contracts.protocol import ARM_NAMES
from zzx_http_backend.worker import HttpWorker, validate_arm, validate_hand


def arm_goal():
    goal = MoveArm.Goal()
    goal.target_type = goal.TARGET_JOINTS
    goal.joint_target.name, goal.joint_target.position = ARM_NAMES, [0.] * 7
    goal.velocity_scaling = goal.acceleration_scaling = .1
    goal.timeout.sec = 2
    return goal


def test_cancel_during_prepare_stops_new_mutations():
    worker = HttpWorker('http://127.0.0.1:1')
    cancel = threading.Event()
    calls = []

    def request(method, path, payload, deadline, command=False):
        calls.append(path)
        cancel.set()
        return {'success': True}

    worker.request = request
    outcome = worker.arm(arm_goal(), cancel, lambda stage: None)
    assert outcome.code == E.CANCELED
    assert calls == ['/api/teach_mode']


def test_hand_cancel_after_post_cannot_claim_stopped():
    worker = HttpWorker('http://127.0.0.1:1')
    cancel = threading.Event()
    goal = ControlHand.Goal()
    goal.joint_names, goal.positions, goal.timeout.sec = ['index_pip'], [0.], 2

    def request(method, path, payload, deadline, command=False):
        if path == '/api/set_pos':
            cancel.set()
            return {'success': True}
        return {'position': [.5] * 10}

    worker.request = request
    outcome = worker.hand(goal, cancel, lambda stage: None)
    assert outcome.code == E.STOP_UNCONFIRMED
    assert outcome.locked


def test_replayed_arm_feedback_cannot_confirm_stop():
    worker = HttpWorker('http://127.0.0.1:1')
    worker.request = lambda *args: dict(timestamp=1., moving=False,
                                        left_joints=dict(zip(ARM_NAMES, [0.] * 7)))
    with pytest.raises(UnknownOutcome, match='timestamp'):
        worker.arm_samples(time.monotonic() + 1, None)


def test_invalid_targets_are_rejected_before_http():
    goal = arm_goal()
    goal.joint_target.position[0] = float('nan')
    assert validate_arm(goal).code == E.INVALID_ARGUMENT
    goal = arm_goal()
    goal.target_type = goal.TARGET_POSE
    assert validate_arm(goal).code == E.UNSUPPORTED
    goal = arm_goal()
    goal.velocity_scaling = 0.
    assert validate_arm(goal).code == E.INVALID_ARGUMENT
    hand = ControlHand.Goal()
    hand.speed_scaling = float('nan')
    assert validate_hand(hand).code == E.INVALID_ARGUMENT


def test_real_hardware_endpoint_is_not_enabled():
    with pytest.raises(ValueError):
        HttpWorker('http://192.168.0.22:8087')
