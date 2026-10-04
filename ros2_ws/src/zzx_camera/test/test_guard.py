import sys

from builtin_interfaces.msg import Time
import numpy as np
import pytest

from zzx_camera.guard import CameraLease, StreamGuard, official_command
from zzx_camera.sources import packet, SdkWorker


def fixture_packet():
    return packet(np.zeros((16, 24, 3), np.uint8), np.full((16, 24), .5, np.float32),
                  (24., 24., 12., 8.), [0.] * 8, Time(sec=1))


def feed(guard, frames, now):
    return [guard.accept(key, msg, now) for key, msg in frames.items()]


def test_valid_packet_and_units():
    frames = fixture_packet()
    guard = StreamGuard(24, 16, now=0.)
    assert all(feed(guard, frames, .1))
    assert guard.check(.2)
    assert frames['depth'].encoding == '32FC1'
    assert np.frombuffer(frames['depth'].data, dtype=np.float32).mean() == .5


@pytest.mark.parametrize('kind,field,value,reason', [
    ('color', 'width', 25, 'dimensions changed'),
    ('color', 'encoding', 'mono8', 'encoding'),
    ('color', 'step', 1, 'stride'),
    ('color', 'data', b'123', 'truncated'),
    ('depth', 'height', 17, 'dimensions changed'),
    ('depth', 'encoding', '8UC1', 'encoding'),
    ('color_info', 'k', [0.] * 9, 'intrinsics'),
    ('depth_info', 'd', [float('nan')] * 8, 'non-finite'),
    ('color_info', 'r', [float('nan')] * 9, 'non-finite'),
])
def test_invalid_stream_latches(kind, field, value, reason):
    frames = fixture_packet()
    setattr(frames[kind], field, value)
    guard = StreamGuard(24, 16, now=0.)
    assert not guard.accept(kind, frames[kind], .1)
    assert reason in guard.fault
    feed(guard, fixture_packet(), .2)
    assert not guard.check(.2)


def test_resolution_intrinsics_change_invalidate():
    guard = StreamGuard(24, 16, now=0.)
    frames = fixture_packet()
    feed(guard, frames, .1)
    frames['color_info'].k[0] = 25.
    assert not guard.accept('color_info', frames['color_info'], .2)
    assert 'calibration invalid' in guard.fault


def test_registration_frame_mismatch():
    guard = StreamGuard(24, 16, now=0.)
    frames = fixture_packet()
    frames['depth'].header.frame_id = 'other_color_optical_frame'
    feed(guard, frames, .1)
    assert 'frame mismatch' in guard.fault


def test_missing_timestamp():
    guard = StreamGuard(24, 16, now=0.)
    frames = fixture_packet()
    frames['color'].header.stamp = Time()
    assert not guard.accept('color', frames['color'], .1)


def test_registration_intrinsics_mismatch():
    guard = StreamGuard(24, 16, now=0.)
    frames = fixture_packet()
    frames['depth_info'].k[0] = 100.
    feed(guard, frames, .1)
    assert 'intrinsics differ' in guard.fault


def test_partial_stream_loss_cannot_recover():
    guard = StreamGuard(24, 16, timeout=.5, now=0.)
    frames = fixture_packet()
    feed(guard, frames, .1)
    guard.accept('color', frames['color'], .5)
    assert not guard.check(.7)
    assert 'lost' in guard.fault
    assert not any(feed(guard, frames, .8))


def test_startup_timeout():
    guard = StreamGuard(24, 16, startup_timeout=2., now=0.)
    assert not guard.check(1.)
    assert not guard.fault
    assert not guard.check(2.1)
    assert 'startup' in guard.fault


def test_no_resize_in_packet():
    with pytest.raises(ValueError, match='registered'):
        packet(np.zeros((16, 24, 3), np.uint8), np.zeros((8, 12), np.float32),
               (24., 24., 12., 8.), [0.] * 8, Time(sec=1))


def test_lease_mutual_exclusion_and_release(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    first = CameraLease()
    try:
        with pytest.raises(RuntimeError, match='owned'):
            CameraLease()
    finally:
        first.close()
    CameraLease().close()


def test_vendor_command_has_no_viewer_or_network_discovery():
    command = official_command(1280, 720, 30)
    assert 'depth_registration:=true' in command
    assert 'enumerate_net_device:=false' in command
    assert 'enable_d2c_viewer:=false' in command
    assert 'color_width:=1280' in command


def test_missing_sdk_fails_worker_without_fallback(monkeypatch):
    monkeypatch.setitem(sys.modules, 'pyorbbecsdk', None)
    worker = SdkWorker(24, 16, 10)
    try:
        worker.thread.join(timeout=2.)
        assert 'SDK acquisition failed' in worker.error
        assert worker.frames.empty()
    finally:
        worker.close()
