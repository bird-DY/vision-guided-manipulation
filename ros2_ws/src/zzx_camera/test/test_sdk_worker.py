"""Contract mocks only, not a substitute for USB/firmware qualification."""
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from zzx_camera.sources import SdkWorker


def sdk_mock(scale=1., count=1, bad_shape=False, model='BROWN_CONRADY'):
    events = []
    intrinsic = NS(width=24, height=16, fx=24., fy=24., cx=12., cy=8.)
    distortion = NS(model=model, **{k: 0. for k in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')})
    profile = NS(get_intrinsic=lambda: intrinsic, get_distortion=lambda: distortion)
    profile.as_video_stream_profile = lambda: profile
    color = NS(get_stream_profile=lambda: profile, get_width=lambda: 24, get_height=lambda: 16,
               get_data=lambda: np.zeros((16, 24, 3), np.uint8).tobytes())
    dh = 8 if bad_shape else 16
    depth = NS(get_depth_scale=lambda: scale, get_width=lambda: 24, get_height=lambda: dh,
               get_data=lambda: np.full((dh, 24), 500, np.uint16).tobytes())
    frames = NS(get_color_frame=lambda: color, get_depth_frame=lambda: depth)
    frames.as_frame_set = lambda: frames
    def wait(timeout):
        time.sleep(.005)
        return frames
    pipeline = NS(get_stream_profile_list=lambda sensor: NS(get_video_stream_profile=lambda *args: profile),
                  enable_frame_sync=lambda: events.append('sync'), start=lambda c: events.append('start'),
                  stop=lambda: events.append('stop'), wait_for_frames=wait)
    module = NS(
        Context=lambda: NS(enable_net_device_enumeration=lambda v: events.append(('network', v)),
                           query_devices=lambda: NS(get_count=lambda: count, get_device_by_index=lambda i: None)),
        Pipeline=lambda dev: pipeline, Config=lambda: NS(enable_stream=lambda p: None),
        OBSensorType=NS(COLOR_SENSOR=1, DEPTH_SENSOR=2), OBFormat=NS(RGB=1, Y16=2),
        OBStreamType=NS(COLOR_STREAM=1), AlignFilter=lambda **kwargs: NS(process=lambda f: f))
    return module, events


def test_worker_scales_millimeters_bounded_queue_and_stops(monkeypatch):
    sdk, events = sdk_mock()
    monkeypatch.setitem(sys.modules, 'pyorbbecsdk', sdk)
    worker = SdkWorker(24, 16, 20)
    try:
        deadline = time.monotonic() + 2
        while worker.dropped < 2 and time.monotonic() < deadline:
            time.sleep(.01)
        assert not worker.error
        assert worker.frames.qsize() == 1
        assert worker.dropped >= 2
        color, depth, intr, dist = worker.frames.get_nowait()
        assert depth.dtype == np.float32
        assert depth.mean() == pytest.approx(.5)
        assert depth.shape == color.shape[:2]
        assert ('network', False) in events
    finally:
        worker.close()
    assert 'stop' in events


@pytest.mark.parametrize('kwargs,reason', [
    ({'scale': 0.}, 'scale'),
    ({'count': 2}, 'exactly one'),
    ({'bad_shape': True}, 'refusing to resize'),
    ({'model': 'UNKNOWN'}, 'distortion model'),
])
def test_worker_rejects_unsupported_data_and_releases(monkeypatch, kwargs, reason):
    sdk, events = sdk_mock(**kwargs)
    monkeypatch.setitem(sys.modules, 'pyorbbecsdk', sdk)
    worker = SdkWorker(24, 16, 20)
    try:
        worker.thread.join(timeout=2.)
        assert reason in worker.error
        assert worker.frames.empty()
    finally:
        worker.close()
    if 'start' in events:
        assert 'stop' in events
