from copy import deepcopy

from builtin_interfaces.msg import Time
import numpy as np
import pytest

from zzx_camera.rgbd_gate import GateConfig, RgbdGate, depth_ratio
from zzx_camera.sources import packet


def frames(stamp=1.):
    return packet(np.zeros((16, 24, 3), np.uint8),
                  np.full((16, 24), .5, np.float32), (24., 24., 12., 8.),
                  [0.] * 8, Time(sec=int(stamp), nanosec=round(stamp % 1 * 1e9)))


def gate(**kwargs):
    result = RgbdGate(GateConfig(**kwargs))
    result.reset_session('session-one', 'synthetic', 1_000_000_000, 0.)
    return result


def pair(g, messages=None, now=1., mono=0.):
    messages = messages or frames(now)
    g.accept('color', messages['color'], round(now * 1e9), mono)
    return g.accept('depth', messages['depth'], round(now * 1e9), mono)


def test_valid_pair_and_no_motion_authorization():
    g = gate()
    result = pair(g)
    assert result.valid_depth_ratio == 1.
    assert result.sync_delta_s == result.max_age_s == 0.
    assert not result.motion_eligible
    pair(g, now=1.1, mono=.1)
    assert g.snapshot(1_100_000_000, .1)['fps'] == pytest.approx(10.)


@pytest.mark.parametrize('stamp,now', [(0., 1.), (.9, 1.), (1.1, 1.), (1., 1.3)])
def test_invalid_age(stamp, now):
    g = gate()
    assert pair(g, frames(stamp), now) is None
    assert g.drops['invalid frame age'] == 2


def test_unsynchronized_and_bounded_queues():
    g = gate(queue_size=2)
    for value in (1., 1.01, 1.02):
        g.accept('color', frames(value)['color'], round(value * 1e9), value - 1.)
    assert len(g.queues['color']) == 2
    assert g.drops['queue overflow'] == 1
    assert g.accept('depth', frames(1.1)['depth'], 1_100_000_000, .1) is None
    assert g.drops['unsynchronized'] == 2
    assert g.accept('color', frames(1.11)['color'], 1_110_000_000, .11)


def test_repeated_stamp_and_one_use_only():
    g = gate()
    assert pair(g)
    assert pair(g) is None
    assert g.drops['nonmonotonic frame'] == 2


def test_monotonic_watchdog_with_paused_ros_clock():
    g = gate()
    pair(g)
    assert g.snapshot(1_000_000_000, 1.1)['state'] == 'ERROR'
    assert not any(g.queues.values())


def test_expired_pair_is_not_ready_and_old_queue_removed():
    g = gate()
    pair(g)
    assert g.snapshot(1_300_000_000, .3)['state'] == 'DEGRADED'
    g.accept('color', frames(1.3)['color'], 1_300_000_000, .3)
    g.check(1_600_000_000, .6)
    assert not g.queues['color']


def test_clock_reset_latches_until_session_reset():
    g = gate()
    pair(g, now=1.2, mono=.2)
    assert pair(g, now=1.1, mono=.3) is None
    assert pair(g, now=1.3, mono=.4) is None
    assert g.state == 'ERROR'
    g.reset_session('session-two', 'synthetic', 1_300_000_000, .4)
    assert pair(g, now=1.3, mono=.4).session_id == 'session-two'


def test_new_session_flushes_old_frames():
    g = gate()
    g.accept('color', frames()['color'], 1_000_000_000, 0.)
    g.reset_session('next', 'vendor_system', 1_100_000_000, .1)
    assert pair(g, frames(), now=1.1, mono=.1) is None
    assert pair(g, now=1.1, mono=.1)


@pytest.mark.parametrize('encoding,endian', [('16UC1', 0), ('16UC1', 1),
                                             ('32FC1', 0), ('32FC1', 1)])
def test_depth_byte_order_padding_units(encoding, endian):
    d = deepcopy(frames()['depth'])
    d.width, d.height, d.encoding, d.is_bigendian = 2, 2, encoding, endian
    kind = 'u2' if encoding == '16UC1' else 'f4'
    values = [500, 0, 65535, 1000] if kind == 'u2' else [.5, np.nan, np.inf, 1.]
    array = np.array(values, dtype=('>' if endian else '<') + kind).reshape(2, 2)
    d.step = 2 * array.dtype.itemsize + 3
    d.data = b''.join(row.tobytes() + b'xyz' for row in array)
    assert depth_ratio(d, GateConfig()) == .5


@pytest.mark.parametrize('change,reason', [
    ('frame', 'registration mismatch'), ('shape', 'registration mismatch'),
    ('encoding', 'invalid depth layout'), ('stride', 'invalid depth layout'),
    ('data', 'invalid depth layout'), ('color', 'invalid color layout'),
    ('zeros', 'insufficient valid depth')])
def test_quality_rejection(change, reason):
    g = gate()
    data = frames()
    d = data['depth']
    if change == 'frame':
        d.header.frame_id = 'different'
    elif change == 'shape':
        d.width = 1
    elif change == 'encoding':
        d.encoding = 'mono8'
    elif change == 'stride':
        d.step = 1
    elif change == 'data':
        d.data = b''
    elif change == 'color':
        data['color'].encoding = 'mono8'
    elif change == 'zeros':
        d.data = bytes(len(d.data))
    assert pair(g, data) is None
    assert g.reason == reason
    assert pair(g, now=1.1, mono=.1)


@pytest.mark.parametrize('kwargs', [{'max_age_s': 0.}, {'queue_size': True},
                                    {'queue_size': 101}, {'min_valid_ratio': 1.1},
                                    {'max_delta_s': float('nan')}])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        GateConfig(**kwargs)


def test_host_publish_timestamp_not_hardware_acquisition():
    g = RgbdGate()
    with pytest.raises(ValueError):
        g.reset_session('hardware', 'host_publish_time', 1_000_000_000, 0.)
    assert pair(g) is None


def test_one_stream_loss_and_fresh_recovery():
    g = gate()
    assert pair(g)
    g.accept('color', frames(1.8)['color'], 1_800_000_000, .8)
    assert g.snapshot(2_100_000_000, 1.1)['state'] == 'ERROR'
    # The first returning stream cannot reuse a pair buffered before the loss.
    assert g.accept('depth', frames(2.1)['depth'], 2_100_000_000, 1.1) is None
    assert g.accept('color', frames(2.11)['color'], 2_110_000_000, 1.11)
    report = g.snapshot(2_110_000_000, 1.11)
    assert report['last_pair_age_s'] == pytest.approx(.01)
    assert report['last_accepted_quality']['valid_depth_ratio'] == 1.


def test_monotonic_clock_backwards_latches():
    g = gate()
    assert pair(g, now=1.1, mono=.2)
    assert pair(g, now=1.2, mono=.1) is None
    assert g.clock_fault
