"""Bounded, deterministic RGB-D pairing; no ROS executor or motion side effects."""
from collections import Counter, deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GateConfig:
    max_delta_s: float = .03
    max_age_s: float = .25
    input_timeout_s: float = 1.
    min_depth_m: float = .15
    max_depth_m: float = 3.
    min_valid_ratio: float = .6
    queue_size: int = 5

    def __post_init__(self):
        values = (self.max_delta_s, self.max_age_s, self.input_timeout_s,
                  self.min_depth_m, self.max_depth_m, self.min_valid_ratio)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('configuration must be finite')
        if not (0 <= self.max_delta_s <= self.max_age_s
                and self.max_age_s > 0 and self.input_timeout_s > 0
                and 0 < self.min_depth_m < self.max_depth_m
                and 0 <= self.min_valid_ratio <= 1
                and type(self.queue_size) is int and 1 <= self.queue_size <= 100):
            raise ValueError('invalid gate configuration')


@dataclass(frozen=True)
class FramePair:
    color: object
    depth: object
    session_id: str
    timestamp_source: str
    sync_delta_s: float
    max_age_s: float
    valid_depth_ratio: float
    motion_eligible: bool = False


def stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid timestamp')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def depth_ratio(image, config):
    """Honor ROS Image byte order and row padding; never resize registered depth."""
    kinds = {'16UC1': ('u2', .001), '32FC1': ('f4', 1.)}
    if image.encoding not in kinds:
        raise ValueError('unsupported depth encoding')
    kind, scale = kinds[image.encoding]
    dtype = np.dtype(('>' if image.is_bigendian else '<') + kind)
    if (image.width <= 0 or image.height <= 0
            or image.step < image.width * dtype.itemsize
            or len(image.data) != image.step * image.height):
        raise ValueError('invalid depth dimensions or stride')
    raw = np.ndarray((image.height, image.width), dtype=dtype,
                     buffer=bytes(image.data), strides=(image.step, dtype.itemsize))
    meters = raw.astype(np.float64) * scale
    valid = (np.isfinite(meters) & (meters >= config.min_depth_m)
             & (meters <= config.max_depth_m))
    return float(np.count_nonzero(valid) / valid.size)


class RgbdGate:
    """Caller supplies ROS time and monotonic time, and owns session provenance.

    CameraInfo/calibration validation remains an upstream/downstream responsibility.
    A session reset must accompany clock resets, source restarts or configuration changes.
    """

    def __init__(self, config=None):
        self.config = config or GateConfig()
        self.queues = {key: deque() for key in ('color', 'depth')}
        self.drops = Counter()
        self.session_id = ''
        self.timestamp_source = ''
        self.state = 'STARTING'
        self.reason = 'session required'
        self.last_stamp = {}
        self.last_arrival = {}
        self.outputs = deque(maxlen=100)
        self.started = 0.
        self.boundary_ns = 0
        self.last_ros_ns = None
        self.last_mono = None
        self.clock_fault = False
        self.last_pair_stamp = None
        self.last_quality = None

    def reset_session(self, session_id, timestamp_source, ros_ns, mono):
        if not session_id or timestamp_source not in ('synthetic', 'vendor_system'):
            raise ValueError('explicit session and comparable timestamp source required')
        if ros_ns <= 0 or not math.isfinite(mono):
            raise ValueError('invalid clock')
        for queue in self.queues.values():
            queue.clear()
        self.last_stamp.clear()
        self.last_arrival.clear()
        self.outputs.clear()
        self.session_id, self.timestamp_source = session_id, timestamp_source
        self.boundary_ns, self.started = ros_ns, mono
        self.last_ros_ns, self.last_mono = ros_ns, mono
        self.clock_fault = False
        self.last_pair_stamp = None
        self.last_quality = None
        self.state, self.reason = 'STARTING', 'waiting for both streams'

    def _reject(self, reason):
        self.drops[reason] += 1
        self.state, self.reason = 'DEGRADED', reason

    def check(self, ros_ns, mono):
        if not self.session_id:
            return False
        if (not math.isfinite(mono) or ros_ns <= 0
                or ros_ns < self.last_ros_ns or mono < self.last_mono):
            self.clock_fault = True
        if self.clock_fault:
            for queue in self.queues.values():
                queue.clear()
            self.state, self.reason = 'ERROR', 'clock reset; new session required'
            return False
        self.last_ros_ns, self.last_mono = ros_ns, mono
        if (self.state == 'READY' and self.last_pair_stamp is not None
                and (ros_ns - self.last_pair_stamp) / 1e9 > self.config.max_age_s):
            self.state, self.reason = 'DEGRADED', 'last pair expired'
        for queue in self.queues.values():
            while queue and (ros_ns - queue[0][0]) / 1e9 > self.config.max_age_s:
                queue.popleft()
                self._reject('expired queued frame')
        if any(mono - self.last_arrival.get(key, self.started)
               > self.config.input_timeout_s for key in self.queues):
            for queue in self.queues.values():
                queue.clear()
            self.state, self.reason = 'ERROR', 'input missing or stale'
            return False
        return True

    def accept(self, kind, message, ros_ns, mono):
        if kind not in self.queues:
            raise ValueError('unknown stream')
        # Expiry is checked before admission; a fresh pair can recover input loss.
        self.check(ros_ns, mono)
        if not self.session_id or self.clock_fault:
            return None
        try:
            stamp = stamp_ns(message)
        except ValueError:
            self._reject('invalid timestamp')
            return None
        if (stamp <= 0 or stamp < self.boundary_ns or stamp > ros_ns
                or (ros_ns - stamp) / 1e9 > self.config.max_age_s):
            self._reject('invalid frame age')
            return None
        if stamp <= self.last_stamp.get(kind, -1):
            self._reject('nonmonotonic frame')
            return None
        self.last_stamp[kind], self.last_arrival[kind] = stamp, mono
        queue = self.queues[kind]
        if len(queue) == self.config.queue_size:
            queue.popleft()
            self.drops['queue overflow'] += 1
        queue.append((stamp, message))
        color, depth = self.queues['color'], self.queues['depth']
        while color and depth:
            cs, cm = color[0]
            ds, dm = depth[0]
            if abs(cs - ds) / 1e9 > self.config.max_delta_s:
                (color if cs < ds else depth).popleft()
                self._reject('unsynchronized')
                continue
            color.popleft()
            depth.popleft()
            if (not cm.header.frame_id or cm.header.frame_id != dm.header.frame_id
                    or cm.width != dm.width or cm.height != dm.height):
                self._reject('registration mismatch')
                continue
            if (cm.encoding not in ('rgb8', 'bgr8') or cm.step < cm.width * 3
                    or len(cm.data) != cm.step * cm.height):
                self._reject('invalid color layout')
                continue
            try:
                ratio = depth_ratio(dm, self.config)
            except ValueError:
                self._reject('invalid depth layout')
                continue
            if ratio < self.config.min_valid_ratio:
                self._reject('insufficient valid depth')
                continue
            self.outputs.append(mono)
            self.last_pair_stamp = min(cs, ds)
            self.last_quality = {'sync_delta_s': abs(cs - ds) / 1e9,
                                 'valid_depth_ratio': ratio}
            self.state, self.reason = 'READY', 'input quality passed; not motion validated'
            return FramePair(cm, dm, self.session_id, self.timestamp_source,
                             abs(cs - ds) / 1e9, (ros_ns - min(cs, ds)) / 1e9, ratio)
        return None

    def snapshot(self, ros_ns, mono):
        self.check(ros_ns, mono)
        recent = [t for t in self.outputs if mono - t <= 2.]
        fps = ((len(recent) - 1) / (recent[-1] - recent[0])
               if len(recent) > 1 and recent[-1] > recent[0] else 0.)
        return {'state': self.state, 'reason': self.reason, 'fps': fps,
                'session_id': self.session_id, 'timestamp_source': self.timestamp_source,
                'motion_eligible': False, 'drops': dict(self.drops),
                'last_accepted_quality': self.last_quality,
                'last_pair_age_s': ((ros_ns - self.last_pair_stamp) / 1e9
                                    if self.last_pair_stamp is not None else None),
                'queue_lengths': {key: len(q) for key, q in self.queues.items()}}
