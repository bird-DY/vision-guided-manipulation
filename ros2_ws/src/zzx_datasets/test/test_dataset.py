import copy
import json

import cv2
import numpy as np
import pytest

from zzx_datasets.cli import inventory, main, replay
from zzx_datasets.manifest import DatasetError, digest, load_frame, read_json, validate


@pytest.fixture
def data(tmp_path):
    cv2.imwrite(str(tmp_path / 'test_color.jpg'), np.full((3, 4, 3), 100, np.uint8))
    depth = np.array([[0, .3, .4, 65.535], [np.nan, np.inf, -.1, .5],
                      [.4, .4, .4, .4]], dtype=np.float32)
    np.save(tmp_path / 'test_depth.npy', depth)
    (tmp_path / 'test_camera.json').write_text(json.dumps({
        'width': 4, 'height': 3, 'fx': 3., 'fy': 3., 'cx': 2., 'cy': 1.,
        'timestamp': 1787107810.0}))
    return tmp_path, inventory(tmp_path, 'capture-a')


def test_import_preserves_uncertainty(data):
    root, manifest = data
    summary = validate(manifest, root)
    assert summary['unknown_labels'] == 1
    assert summary['success_rate'] is None
    sample = manifest['samples'][0]
    assert sample['host_timestamp_ns'] == 1787107810000000000
    assert sample['color_timestamp_ns'] is None
    assert sample['alignment'] == 'unknown'
    assert sample['capture_config'] is None
    assert load_frame(root, sample)[1].shape == (3, 4)


def test_deterministic_replay_and_range(data):
    root, manifest = data
    first = list(replay(manifest, root))
    assert first == list(replay(manifest, root))
    stats = first[0]['depth_stats']
    assert stats['valid_pixels'] == 7
    assert stats['median_m'] == pytest.approx(.4)
    assert first[0]['motion_eligible'] is False
    assert first[0]['evaluated'] is False
    assert list(replay(manifest, root, 'holdout')) == []


@pytest.mark.parametrize('key,value,match', [
    ('alignment', 'registered_to_color', 'evidence'),
    ('color_size', [8, 6], 'resolution mismatch'),
    ('depth_size', [8, 6], 'depth size mismatch'),
    ('depth_scale_m', 0, 'scale'),
    ('color_timestamp_ns', -1, 'timestamp'),
    ('color_timestamp_ns', True, 'timestamp'),
    ('split', 'test', 'split'),
    ('label', {'status': 'verified', 'value': 'red'}, 'reviewer'),
    ('label', {'status': 'unknown', 'value': 'red'}, 'ground truth'),
])
def test_reject_bad_metadata(data, key, value, match):
    root, manifest = data
    manifest['samples'][0][key] = value
    with pytest.raises(DatasetError, match=match):
        validate(manifest, root)


def test_batch_leakage(data):
    root, manifest = data
    second = copy.deepcopy(manifest['samples'][0])
    second.update(id='second', split='holdout')
    manifest['samples'].append(second)
    with pytest.raises(DatasetError, match='batch split'):
        validate(manifest, root)


def test_content_leakage_across_batches(data):
    root, manifest = data
    second = copy.deepcopy(manifest['samples'][0])
    second.update(id='second', batch_id='capture-b', split='holdout')
    manifest['samples'].append(second)
    with pytest.raises(DatasetError, match='content split'):
        validate(manifest, root)


def test_tamper_and_no_partial_replay(data):
    root, manifest = data
    (root / 'test_camera.json').write_text('{}')
    with pytest.raises(DatasetError, match='hash mismatch'):
        next(replay(manifest, root))


@pytest.mark.parametrize('path', ['../outside.jpg', '/tmp/outside.jpg', 'C:/outside.jpg'])
def test_traversal_rejected(data, path):
    root, manifest = data
    manifest['samples'][0]['assets']['color']['path'] = path
    with pytest.raises(DatasetError, match='relative POSIX'):
        validate(manifest, root)


def test_symlink_escape(data, tmp_path):
    root, manifest = data
    (root / 'escape').symlink_to('/etc/passwd')
    manifest['samples'][0]['assets']['color']['path'] = 'escape'
    with pytest.raises(DatasetError, match='escapes'):
        validate(manifest, root)


def test_missing_file(data):
    root, manifest = data
    (root / 'test_depth.npy').unlink()
    with pytest.raises(DatasetError, match='missing asset'):
        validate(manifest, root)


def test_all_invalid_depth(data):
    root, manifest = data
    path = root / 'test_depth.npy'
    np.save(path, np.zeros((3, 4), np.float32))
    manifest['samples'][0]['assets']['depth'].update(sha256=digest(path), bytes=path.stat().st_size)
    stats = next(replay(manifest, root))['depth_stats']
    assert stats['median_m'] is None
    assert stats['valid_ratio'] == 0


def test_rgb_only_panel_not_ground_truth(tmp_path):
    cv2.imwrite(str(tmp_path / '20260819_panel_failed.jpg'), np.zeros((3, 4, 3), np.uint8))
    manifest = inventory(tmp_path, 'session')
    assert manifest['samples'][0]['label']['status'] == 'unknown'
    assert next(replay(manifest, tmp_path))['depth_stats'] is None


def test_cli_exclusive_output_and_validation(data, capsys):
    root, _ = data
    out = root / 'manifest.json'
    args = ['import-final', '--root', str(root), '--batch-id', 'a', '--output', str(out)]
    assert main(args) == 0
    assert main(args) == 2
    assert main(['validate', str(out), '--root', str(root)]) == 0
    capsys.readouterr()
    assert main(['replay', str(out), '--root', str(root)]) == 0
    assert json.loads(capsys.readouterr().out)['sample_id'] == 'test_color.jpg'


@pytest.mark.parametrize('text', ['{"schema_version":1,"schema_version":2}', '{"x":NaN}'])
def test_ambiguous_json_rejected(tmp_path, text):
    path = tmp_path / 'bad.json'
    path.write_text(text)
    with pytest.raises(DatasetError):
        read_json(path)


def test_invalid_range(data):
    root, manifest = data
    with pytest.raises(DatasetError, match='depth range'):
        next(replay(manifest, root, minimum=2., maximum=.1))


def test_import_refuses_unknown_integer_depth_units(data):
    root, _ = data
    np.save(root / 'test_depth.npy', np.zeros((3, 4), np.uint16))
    with pytest.raises(DatasetError, match='do not guess units'):
        inventory(root, 'capture')


def test_unmapped_files_reported(data):
    root, _ = data
    (root / 'notes.txt').write_text('not a capture')
    assert 'notes.txt' in inventory(root, 'capture')['unmapped_files']


def test_replay_orders_ids_not_manifest_insertion(data):
    root, manifest = data
    second = copy.deepcopy(manifest['samples'][0])
    second['id'] = 'a-first'
    manifest['samples'].append(second)
    assert [r['sample_id'] for r in replay(manifest, root)] == ['a-first', 'test_color.jpg']


def test_no_resize_or_implicit_registration(data):
    root, manifest = data
    path = root / 'test_depth.npy'
    np.save(path, np.ones((2, 2), np.float32))
    sample = manifest['samples'][0]
    sample['assets']['depth'].update(sha256=digest(path), bytes=path.stat().st_size)
    sample['depth_size'] = [2, 2]
    validate(manifest, root)
    assert load_frame(root, sample)[1].shape == (2, 2)
    sample.update(alignment='registered_to_color', alignment_evidence='test fixture',
                  calibration_id='fixture')
    with pytest.raises(DatasetError, match='registered depth/color size'):
        validate(manifest, root)
