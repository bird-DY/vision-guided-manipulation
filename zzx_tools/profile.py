"""Load and validate Zzxrobot robot profiles."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
from typing import Any, Iterable
from urllib.parse import urlparse

from jsonschema import Draft7Validator
import yaml


class ProfileValidationError(ValueError):
    """Raised when a profile is structurally or semantically invalid."""


def repository_root() -> Path:
    """Return the repository containing this module."""
    return Path(__file__).resolve().parents[1]


def load_mapping(path: Path) -> dict[str, Any]:
    """Load a YAML or JSON mapping from disk."""
    with path.open(encoding='utf-8') as stream:
        if path.suffix == '.json':
            data = json.load(stream)
        else:
            data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ProfileValidationError(f'{path}: expected a mapping')
    return data


def profile_paths(root: Path | None = None) -> list[Path]:
    """List profile files in stable order."""
    base = root or repository_root()
    return sorted((base / 'configs/robots').glob('*.yaml'))


def resolve_profile(reference: str, root: Path | None = None) -> Path:
    """Resolve a profile ID or explicit path."""
    candidate = Path(reference).expanduser()
    if candidate.is_file():
        return candidate.resolve()

    base = root or repository_root()
    by_id = base / 'configs/robots' / f'{reference}.yaml'
    if by_id.is_file():
        return by_id
    raise ProfileValidationError(f'profile not found: {reference}')


def schema_validator(root: Path | None = None) -> Draft7Validator:
    """Build and check the repository JSON Schema validator."""
    base = root or repository_root()
    schema = load_mapping(base / 'configs/schema/robot_profile.schema.json')
    Draft7Validator.check_schema(schema)
    return Draft7Validator(schema)


def registered_tools(root: Path | None = None) -> dict[str, dict[str, Any]]:
    """Load tool definitions keyed by tool_id."""
    base = root or repository_root()
    tools: dict[str, dict[str, Any]] = {}
    for path in sorted((base / 'configs/tools').glob('*.yaml')):
        tool = load_mapping(path)
        tool_id = tool.get('tool_id')
        if not isinstance(tool_id, str) or not tool_id:
            raise ProfileValidationError(f'{path}: missing tool_id')
        if tool_id in tools:
            raise ProfileValidationError(f'duplicate tool_id: {tool_id}')
        tools[tool_id] = tool
    return tools


def _json_path(parts: Iterable[Any]) -> str:
    result = '$'
    for part in parts:
        result += f'[{part}]' if isinstance(part, int) else f'.{part}'
    return result


def _validate_backend(name: str, backend: dict[str, Any], errors: list[str]) -> None:
    backend_type = backend['type']
    endpoint = backend.get('endpoint')
    if backend_type == 'http':
        if not isinstance(endpoint, str):
            errors.append(f'{name}.backend.endpoint is required for HTTP')
            return
        parsed = urlparse(endpoint)
        if (
            parsed.scheme != 'http'
            or not parsed.hostname
            or parsed.username
            or parsed.password
            or parsed.query
            or parsed.fragment
        ):
            errors.append(f'{name}.backend.endpoint is not a permitted HTTP base URL')
    elif endpoint is not None:
        errors.append(f'{name}.backend.endpoint must be null for {backend_type}')


def validate_profile(
    profile: dict[str, Any],
    *,
    root: Path | None = None,
    automatic: bool = False,
) -> None:
    """Validate schema, cross-field constraints and automatic-mode gates."""
    base = root or repository_root()
    errors = [
        f'{_json_path(error.absolute_path)}: {error.message}'
        for error in sorted(
            schema_validator(base).iter_errors(profile),
            key=lambda item: _json_path(item.absolute_path),
        )
    ]
    if errors:
        raise ProfileValidationError('\n'.join(errors))

    for subsystem in ('arm', 'hand'):
        config = profile[subsystem]
        names = config['joint_names']
        positions = config.get('home_positions_rad')
        if len(names) != config['dof']:
            errors.append(
                f'{subsystem}.dof={config["dof"]} but joint_names has {len(names)} entries'
            )
        if len(set(names)) != len(names):
            errors.append(f'{subsystem}.joint_names contains duplicates')
        if positions is not None and len(positions) != config['dof']:
            errors.append(
                f'{subsystem}.home_positions_rad has {len(positions)} entries, '
                f'expected {config["dof"]}'
            )
        _validate_backend(subsystem, config['backend'], errors)

    overlap = set(profile['arm']['joint_names']) & set(profile['hand']['joint_names'])
    if overlap:
        errors.append(f'arm and hand share joints: {sorted(overlap)}')

    tool_id = profile['tool']['tool_id']
    tools = registered_tools(base)
    if tool_id not in tools:
        errors.append(f'unknown tool_id: {tool_id}')
    else:
        tool = tools[tool_id]
        if tool.get('model') != profile['hand']['model']:
            errors.append(f'tool {tool_id} model does not match hand.model')
        if tool.get('joint_names') != profile['hand']['joint_names']:
            errors.append(f'tool {tool_id} joint_names do not match hand.joint_names')
        if tool.get('tcp_frame') != profile['tool']['tcp_frame']:
            errors.append(f'tool {tool_id} tcp_frame does not match profile')

    if automatic:
        calibration = profile['calibration']
        if not profile['safety']['automatic_motion_allowed']:
            errors.append('safety.automatic_motion_allowed is false')
        if calibration['status'] not in ('validated', 'not_required'):
            errors.append(
                f'calibration.status={calibration["status"]} blocks automatic mode'
            )
        if not calibration['transform_available']:
            errors.append('calibration transform is unavailable')

    if errors:
        raise ProfileValidationError('\n'.join(errors))


def validate_file(
    path: Path,
    *,
    root: Path | None = None,
    automatic: bool = False,
) -> dict[str, Any]:
    """Load and validate one profile file."""
    profile = load_mapping(path)
    validate_profile(profile, root=root, automatic=automatic)
    return profile


def _command_list(args: argparse.Namespace) -> int:
    for path in profile_paths(args.root):
        profile = validate_file(path, root=args.root)
        calibration = profile['calibration']['status']
        automatic = profile['safety']['automatic_motion_allowed']
        print(
            f'{profile["profile_id"]}\t{profile["deployment"]}\t'
            f'calibration={calibration}\tautomatic={str(automatic).lower()}'
        )
    return 0


def _command_show(args: argparse.Namespace) -> int:
    path = resolve_profile(args.profile, args.root)
    profile = validate_file(path, root=args.root)
    print(yaml.safe_dump(profile, sort_keys=False, allow_unicode=True), end='')
    return 0


def _command_validate(args: argparse.Namespace) -> int:
    references = args.profiles or [str(path) for path in profile_paths(args.root)]
    for reference in references:
        path = resolve_profile(reference, args.root)
        profile = validate_file(path, root=args.root, automatic=args.automatic)
        mode = 'automatic' if args.automatic else 'schema'
        print(f'valid ({mode}): {profile["profile_id"]}')
    return 0


def build_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    parser = argparse.ArgumentParser(prog='zzx_profile')
    parser.set_defaults(root=repository_root())
    subparsers = parser.add_subparsers(dest='command', required=True)

    list_parser = subparsers.add_parser('list', help='list valid profiles')
    list_parser.set_defaults(handler=_command_list)

    show_parser = subparsers.add_parser('show', help='show one validated profile')
    show_parser.add_argument('profile')
    show_parser.set_defaults(handler=_command_show)

    validate_parser = subparsers.add_parser('validate', help='validate profiles')
    validate_parser.add_argument('profiles', nargs='*')
    validate_parser.add_argument(
        '--automatic',
        action='store_true',
        help='also enforce gates required before automatic motion',
    )
    validate_parser.set_defaults(handler=_command_validate)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the profile command and format validation failures."""
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except (OSError, ProfileValidationError, yaml.YAMLError) as error:
        print(f'profile error: {error}', file=sys.stderr)
        return 2


def clone_profile(profile: dict[str, Any]) -> dict[str, Any]:
    """Return a deep copy for tests and callers that need mutation."""
    return deepcopy(profile)


if __name__ == '__main__':
    raise SystemExit(main())
