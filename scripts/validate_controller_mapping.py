#!/usr/bin/env python3
"""Validate ros2_control and MoveIt controller joint ownership."""

from pathlib import Path
import sys

import yaml


EXPECTED = {
    'my_group_controller': [f'joint{index}' for index in range(1, 7)],
    'gripper_controller': ['joint7', 'joint8'],
}


def load_yaml(path: Path) -> dict:
    """Load a YAML mapping and fail with a useful path."""
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path} does not contain a YAML mapping')
    return data


def read_ros2_control(path: Path) -> dict[str, list[str]]:
    """Extract configured joints from ros2_control YAML."""
    data = load_yaml(path)
    return {
        name: data[name]['ros__parameters']['joints']
        for name in EXPECTED
    }


def read_moveit(path: Path) -> dict[str, list[str]]:
    """Extract configured joints from MoveIt controller YAML."""
    data = load_yaml(path)['moveit_simple_controller_manager']
    configured_names = data['controller_names']
    if set(configured_names) != set(EXPECTED):
        raise ValueError(
            f'MoveIt controller_names={configured_names}, expected={list(EXPECTED)}'
        )
    return {name: data[name]['joints'] for name in configured_names}


def validate(config_directory: Path) -> None:
    """Validate exact mappings and reject joint overlap."""
    control = read_ros2_control(config_directory / 'ros2_controllers.yaml')
    moveit = read_moveit(config_directory / 'moveit_controllers.yaml')

    if control != EXPECTED:
        raise ValueError(f'ros2_control mapping {control} != expected {EXPECTED}')
    if moveit != EXPECTED:
        raise ValueError(f'MoveIt mapping {moveit} != expected {EXPECTED}')

    owners: dict[str, str] = {}
    for controller, joints in control.items():
        for joint in joints:
            if joint in owners:
                raise ValueError(
                    f'{joint} is owned by {owners[joint]} and {controller}'
                )
            owners[joint] = controller


def main() -> int:
    """Run validation against a supplied or repository-relative config path."""
    repository = Path(__file__).resolve().parents[1]
    default_config = repository / 'ros2_ws/src/Zzxrobot/mybot/config'
    config_directory = Path(sys.argv[1]) if len(sys.argv) > 1 else default_config
    validate(config_directory)
    print(f'controller mapping valid: {config_directory}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
