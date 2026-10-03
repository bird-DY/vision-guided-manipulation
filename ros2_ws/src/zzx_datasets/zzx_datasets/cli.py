"""Inspect historic captures without modifying images or contacting devices."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from .manifest import DatasetError, digest, finite, load_frame, read_json, require, validate


def inventory(root, batch_id):
    root = Path(root).resolve()
    require(root.is_dir(), 'capture directory missing')
    samples = []
    colors = sorted(set(root.rglob('*_color.jpg')) | set(root.glob('*_panel*.jpg')))
    for color in colors:
        relative = color.relative_to(root).as_posix()
        charuco = color.parent.name == 'charuco_calib'
        paired = color.stem.endswith('_color')
        base = color.stem[:-6] if paired else color.stem
        metadata = color.with_name(base + ('.json' if charuco else '_camera.json'))
        depth = color.with_name(base + '_depth.npy')
        overlay = color.with_name(base + '_overlay.jpg')
        assets = {'color': color}
        for role, path in (('depth', depth), ('metadata', metadata), ('overlay', overlay)):
            if paired and path.is_file():
                assets[role] = path
        meta = read_json(metadata) if 'metadata' in assets else {}
        intrinsic = meta.get('intrinsics_from_sdk', meta)
        keys = ('width', 'height', 'fx', 'fy', 'cx', 'cy')
        intrinsic = {k: intrinsic[k] for k in keys} if all(k in intrinsic for k in keys) else None
        image = cv2.imread(str(color))
        require(image is not None, f'cannot decode: {relative}')
        d = np.load(depth, allow_pickle=False) if 'depth' in assets else None
        require(d is None or (d.ndim == 2 and d.dtype.kind == 'f'),
                'historic depth_m must be a floating-point 2D array; do not guess units')
        timestamp = meta.get('timestamp')
        samples.append({
            'id': relative.replace('/', '__'), 'batch_id': batch_id,
            'source': 'competition_final_samples',
            'scene': 'charuco' if charuco else ('camera_check' if paired else 'panel'),
            'split': 'unassigned', 'label': {'status': 'unknown', 'value': None},
            'color_timestamp_ns': None, 'depth_timestamp_ns': None,
            'host_timestamp_ns': round(timestamp * 1e9) if finite(timestamp) and timestamp > 0 else None,
            'capture_config': None, 'calibration_id': None,
            'alignment': 'unknown', 'alignment_evidence': None,
            'color_size': list(image.shape[1::-1]),
            'depth_size': list(d.shape[::-1]) if d is not None else None,
            'depth_scale_m': 1.0 if d is not None else None,
            'intrinsics': intrinsic,
            'notes': ['Historic final_app saves depth_m; host time is not sensor time.',
                      'SDK intrinsics provenance is unverified; resize does not establish registration.',
                      'Detection stats and filename are not reviewed ground truth.'],
            'assets': {role: {'path': path.relative_to(root).as_posix(),
                              'sha256': digest(path), 'bytes': path.stat().st_size}
                       for role, path in assets.items()},
        })
    included = {a['path'] for s in samples for a in s['assets'].values()}
    manifest = {'schema_version': 1, 'dataset_id': 'competition_final_inventory',
                'importer': 'zzx_datasets.import-final.v1',
                'unmapped_files': sorted(p.relative_to(root).as_posix()
                                         for p in root.rglob('*')
                                         if p.is_file() and p.relative_to(root).as_posix() not in included),
                'samples': samples}
    validate(manifest, root)
    return manifest


def replay(manifest, root, split=None, minimum=0.1, maximum=2.0):
    require(finite(minimum) and finite(maximum) and 0 <= minimum < maximum,
            'invalid depth range')
    # Validate the entire collection before yielding anything, including other splits.
    validate(manifest, root)
    ordered = sorted(manifest['samples'], key=lambda s: s['id'])
    for sample in ordered:
        if split and sample['split'] != split:
            continue
        _, depth = load_frame(root, sample)
        result = {'sample_id': sample['id'], 'batch_id': sample['batch_id'],
                  'split': sample['split'], 'label_status': sample['label']['status'],
                  'host_timestamp_ns': sample['host_timestamp_ns'],
                  'alignment': sample['alignment'], 'motion_eligible': False,
                  'evaluated': False, 'depth_stats': None}
        if depth is not None:
            meters = depth.astype(np.float64) * sample['depth_scale_m']
            valid = np.isfinite(meters) & (meters > minimum) & (meters <= maximum)
            values = meters[valid]
            result['depth_stats'] = {
                'range_m': [minimum, maximum], 'valid_pixels': int(valid.sum()),
                'total_pixels': int(valid.size), 'valid_ratio': float(valid.mean()),
                'median_m': float(np.median(values)) if values.size else None,
                'p05_m': float(np.quantile(values, .05)) if values.size else None,
                'p95_m': float(np.quantile(values, .95)) if values.size else None,
            }
        yield result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    imp = sub.add_parser('import-final', help='Inventory historic final_app samples only')
    imp.add_argument('--root', required=True)
    imp.add_argument('--batch-id', required=True)
    imp.add_argument('--output', required=True)
    for name in ('validate', 'replay'):
        p = sub.add_parser(name)
        p.add_argument('manifest')
        p.add_argument('--root', required=True)
        if name == 'replay':
            p.add_argument('--split', choices=['train', 'tuning', 'holdout', 'unassigned'])
            p.add_argument('--min-depth-m', type=float, default=.1)
            p.add_argument('--max-depth-m', type=float, default=2.)
    args = parser.parse_args(argv)
    try:
        if args.command == 'import-final':
            manifest = inventory(args.root, args.batch_id)
            # Exclusive creation prevents accidental overwrite of reviewed labels/splits.
            with Path(args.output).open('x', encoding='utf-8') as stream:
                json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write('\n')
            print(json.dumps(validate(manifest, args.root), allow_nan=False))
        elif args.command == 'validate':
            print(json.dumps(validate(read_json(args.manifest), args.root), allow_nan=False))
        else:
            for record in replay(read_json(args.manifest), args.root, args.split,
                                 args.min_depth_m, args.max_depth_m):
                print(json.dumps(record, sort_keys=True, allow_nan=False))
        return 0
    except (ValueError, OSError, KeyError, TypeError, cv2.error) as exc:
        print(f'dataset error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
