"""Known field protocol, transcribed from final_app/hardware/http_clients.py."""
import math

ARM_NAMES = [f'left_{part}_joint' for part in (
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_roll',
    'elbow_yaw', 'wrist_pitch', 'wrist_yaw')]
HAND_NAMES = ['thumb_roll', 'thumb_abad', 'thumb_mcp', 'index_abad', 'index_pip',
              'middle_pip', 'ring_abad', 'ring_pip', 'pinky_abad', 'pinky_pip']


def vector(values, count, normalized=False):
    if not isinstance(values, list) or len(values) != count:
        raise ValueError(f'expected {count} values')
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or
           not math.isfinite(v) for v in values):
        raise ValueError('values must be finite numbers')
    if normalized and any(not 0 <= v <= 1 for v in values):
        raise ValueError('hand positions must be in [0,1]')
    return [float(v) for v in values]


def joint_payload(named_positions, plan_only=False, velocity=.03, acceleration=.03, label=''):
    if not isinstance(named_positions, dict) or set(named_positions) != set(ARM_NAMES):
        raise ValueError('provide all seven named left joints')
    if type(plan_only) is not bool:
        raise ValueError('plan_only must be boolean')
    for scale in (velocity, acceleration):
        if isinstance(scale, bool) or not isinstance(scale, (int, float)) or not 0 < scale <= 1:
            raise ValueError('scalings must be in (0,1]')
    return dict(mode='left_arm', left_joints=vector([named_positions[n] for n in ARM_NAMES], 7),
                velocity_scaling=velocity, acceleration_scaling=acceleration,
                plan_only=plan_only, label=label)
