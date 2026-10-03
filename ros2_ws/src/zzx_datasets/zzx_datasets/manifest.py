"""Version 1 data contract; unknown capture facts remain explicitly unknown."""
import hashlib
import json
import math
from pathlib import Path, PurePosixPath

import cv2
import numpy as np


class DatasetError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise DatasetError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, f'duplicate JSON key: {key}')
            result[key] = value
        return result
    def invalid(value):
        raise DatasetError(f'non-finite JSON constant: {value}')
    return json.loads(Path(path).read_text(encoding='utf-8'),
                      object_pairs_hook=unique, parse_constant=invalid)


def asset_path(root, relative):
    require(isinstance(relative, str) and bool(relative), 'asset path required')
    path = PurePosixPath(relative)
    require(not path.is_absolute() and '..' not in path.parts
            and '\\' not in relative and ':' not in relative,
            'asset path must be relative POSIX without traversal')
    resolved = (Path(root) / relative).resolve()
    require(resolved.is_relative_to(Path(root).resolve()), 'asset escapes data root')
    require(resolved.is_file(), f'missing asset: {relative}')
    return resolved


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def load_frame(root, sample):
    """Load original arrays without resizing, alignment or fabricated timestamps."""
    color = cv2.imread(str(asset_path(root, sample['assets']['color']['path'])),
                       cv2.IMREAD_COLOR)
    require(color is not None, f"undecodable color: {sample['id']}")
    h, w = color.shape[:2]
    require([w, h] == sample['color_size'], f"color size mismatch: {sample['id']}")
    depth = None
    if 'depth' in sample['assets']:
        depth = np.load(asset_path(root, sample['assets']['depth']['path']),
                        allow_pickle=False)
        require(depth.ndim == 2 and depth.dtype.kind in 'uif', 'depth must be numeric 2D')
        require(list(depth.shape[::-1]) == sample['depth_size'], 'depth size mismatch')
        if sample['alignment'] == 'registered_to_color':
            require(depth.shape == color.shape[:2], 'registered depth/color size mismatch')
    return color, depth


def validate(manifest, root):
    require(isinstance(manifest, dict) and type(manifest.get('schema_version')) is int
            and manifest['schema_version'] == 1,
            'unsupported manifest schema')
    require(isinstance(manifest.get('dataset_id'), str) and manifest['dataset_id'],
            'dataset_id required')
    samples = manifest.get('samples')
    require(isinstance(samples, list) and bool(samples), 'nonempty samples required')
    ids, batches, hashes = set(), {}, {}
    for s in samples:
        require(isinstance(s, dict), 'sample must be an object')
        for name in ('id', 'batch_id', 'source', 'scene'):
            require(isinstance(s.get(name), str) and s[name], f'{name} required')
        require(s['id'] not in ids, f"duplicate sample: {s['id']}")
        ids.add(s['id'])
        split = s.get('split')
        require(split in ('train', 'tuning', 'holdout', 'unassigned'), 'invalid split')
        require(batches.setdefault(s['batch_id'], split) == split, 'batch split leakage')
        require(s.get('scene') in ('charuco', 'panel', 'numbered_blocks',
                                  'geometric_objects', 'camera_check', 'unknown'), 'invalid scene')
        require(s.get('alignment') in ('unknown', 'unaligned', 'registered_to_color'),
                'alignment must be explicit')
        require('capture_config' in s and 'calibration_id' in s, 'capture provenance missing')
        require(s['calibration_id'] is None or
                isinstance(s['calibration_id'], str), 'invalid calibration ID')
        require(s['capture_config'] is None or isinstance(s['capture_config'], dict),
                'invalid capture config')
        if s['alignment'] == 'registered_to_color':
            require(isinstance(s.get('alignment_evidence'), str) and
                    bool(s['alignment_evidence']) and bool(s['calibration_id']),
                    'registered depth requires evidence and calibration ID')
        for key in ('color_timestamp_ns', 'depth_timestamp_ns', 'host_timestamp_ns'):
            require(key in s and (s[key] is None or
                    (type(s[key]) is int and s[key] > 0)), f'invalid {key}')
        label = s.get('label')
        require(isinstance(label, dict) and label.get('status') in ('unknown', 'verified'),
                'explicit label status required')
        if label['status'] == 'verified':
            require(isinstance(label.get('value'), str) and bool(label['value']) and
                    isinstance(label.get('reviewer'), str) and bool(label['reviewer']),
                    'verified label needs value and reviewer')
        else:
            require(label.get('value') is None, 'unknown label cannot carry ground truth')
        assets = s.get('assets')
        require(isinstance(assets, dict) and 'color' in assets, 'color asset required')
        require(set(assets) <= {'color', 'depth', 'metadata', 'overlay'}, 'unknown asset role')
        for role, asset in assets.items():
            require(isinstance(asset, dict), 'asset must be an object')
            path = asset_path(root, asset.get('path'))
            sha = digest(path)
            require(asset.get('sha256') == sha, f"hash mismatch: {asset.get('path')}")
            require(type(asset.get('bytes')) is int and asset['bytes'] == path.stat().st_size,
                    'asset size mismatch')
            if role in ('color', 'depth'):
                require(hashes.setdefault(sha, split) == split, 'duplicate content split leakage')
        for key in ('color_size', 'depth_size'):
            size = s.get(key)
            if key == 'depth_size' and 'depth' not in assets:
                require(size is None, 'depth_size without depth')
                continue
            require(isinstance(size, list) and len(size) == 2 and
                    all(type(v) is int and v > 0 for v in size), f'invalid {key}')
        if 'depth' in assets:
            require(finite(s.get('depth_scale_m')) and s['depth_scale_m'] > 0,
                    'depth scale in meters required')
        else:
            require(s.get('depth_scale_m') is None and s['depth_timestamp_ns'] is None,
                    'depth metadata without depth')
            require(s['alignment'] != 'registered_to_color', 'registration without depth')
        intrinsics = s.get('intrinsics')
        if intrinsics is not None:
            require(isinstance(intrinsics, dict), 'intrinsics must be an object')
            require([intrinsics.get('width'), intrinsics.get('height')] == s['color_size'],
                    'intrinsics resolution mismatch')
            require(all(finite(intrinsics.get(k)) for k in ('fx', 'fy', 'cx', 'cy'))
                    and intrinsics['fx'] > 0 and intrinsics['fy'] > 0,
                    'invalid intrinsics')
        load_frame(root, s)
    return {'samples': len(samples), 'batches': len(batches),
            'verified_labels': sum(s['label']['status'] == 'verified' for s in samples),
            'unknown_labels': sum(s['label']['status'] == 'unknown' for s in samples),
            'evaluated_samples': 0, 'success_rate': None}
