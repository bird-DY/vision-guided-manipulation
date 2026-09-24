"""Tests for strict robot profile validation."""

from pathlib import Path
import unittest

import yaml

from zzx_tools.profile import clone_profile
from zzx_tools.profile import load_mapping
from zzx_tools.profile import ProfileValidationError
from zzx_tools.profile import validate_profile


ROOT = Path(__file__).resolve().parents[1]
SIM_PROFILE = ROOT / 'configs/robots/zzx_sim_mobile_6dof.yaml'
COMPETITION_PROFILE = ROOT / 'configs/robots/competition_left_7dof_o10.yaml'


class RobotProfileTest(unittest.TestCase):
    """Exercise schema, semantic and automatic-mode gates."""

    def test_sim_profile_is_automatic_ready(self):
        profile = load_mapping(SIM_PROFILE)
        validate_profile(profile, root=ROOT, automatic=True)

    def test_competition_profile_passes_schema_validation(self):
        profile = load_mapping(COMPETITION_PROFILE)
        validate_profile(profile, root=ROOT)

    def test_competition_profile_rejects_automatic_mode(self):
        profile = load_mapping(COMPETITION_PROFILE)
        with self.assertRaisesRegex(ProfileValidationError, 'placeholder'):
            validate_profile(profile, root=ROOT, automatic=True)

    def test_duplicate_joint_is_rejected(self):
        profile = clone_profile(load_mapping(SIM_PROFILE))
        profile['arm']['joint_names'][1] = profile['arm']['joint_names'][0]
        with self.assertRaisesRegex(ProfileValidationError, 'duplicates'):
            validate_profile(profile, root=ROOT)

    def test_home_vector_length_is_rejected(self):
        profile = clone_profile(load_mapping(SIM_PROFILE))
        profile['arm']['home_positions_rad'].pop()
        with self.assertRaisesRegex(ProfileValidationError, 'home_positions_rad'):
            validate_profile(profile, root=ROOT)

    def test_unknown_tool_is_rejected(self):
        profile = clone_profile(load_mapping(SIM_PROFILE))
        profile['tool']['tool_id'] = 'missing_tool'
        with self.assertRaisesRegex(ProfileValidationError, 'unknown tool_id'):
            validate_profile(profile, root=ROOT)

    def test_invalid_http_url_is_rejected(self):
        profile = clone_profile(load_mapping(COMPETITION_PROFILE))
        profile['arm']['backend']['endpoint'] = 'https://user:secret@example.com/api?q=1'
        with self.assertRaisesRegex(ProfileValidationError, 'HTTP base URL'):
            validate_profile(profile, root=ROOT)

    def test_unknown_field_is_rejected(self):
        profile = clone_profile(load_mapping(SIM_PROFILE))
        profile['arm']['mystery_setting'] = True
        with self.assertRaisesRegex(ProfileValidationError, 'Additional properties'):
            validate_profile(profile, root=ROOT)

    def test_yaml_round_trip_keeps_joint_order(self):
        profile = load_mapping(COMPETITION_PROFILE)
        serialized = yaml.safe_dump(profile, sort_keys=False)
        reloaded = yaml.safe_load(serialized)
        self.assertEqual(
            profile['arm']['joint_names'],
            reloaded['arm']['joint_names'],
        )


if __name__ == '__main__':
    unittest.main()
