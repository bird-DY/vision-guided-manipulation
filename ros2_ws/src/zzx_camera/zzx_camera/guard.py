"""Single-host ownership and immutable stream contract, not RGB-D synchronization."""
import fcntl
import math
from pathlib import Path
import time


class CameraLease:
    def __init__(self):
        root = Path.home() / '.local/state/zzxrobot/owners'
        root.mkdir(parents=True, exist_ok=True)
        # One physical ingress per user, independent of ROS namespace/backend/serial.
        self.file = (root / 'camera_ingress.lock').open('a+b')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.file.close()
            raise RuntimeError('camera ingress already owned')

    def close(self):
        self.file.close()


STREAMS = ('color', 'depth', 'color_info', 'depth_info')


class StreamGuard:
    def __init__(self, width, height, timeout=1., startup_timeout=15., now=None):
        self.width, self.height = width, height
        self.timeout, self.startup_timeout = timeout, startup_timeout
        self.started = time.monotonic() if now is None else now
        self.last = {}
        self.signatures = {}
        self.messages = {}
        self.fault = ''

    def fail(self, reason):
        if not self.fault:
            self.fault = reason

    def accept(self, kind, msg, now):
        if self.fault:
            return False
        try:
            if (msg.width, msg.height) != (self.width, self.height):
                raise ValueError('stream dimensions changed; calibration invalid')
            if not msg.header.frame_id or not msg.header.frame_id.endswith('color_optical_frame'):
                raise ValueError('registered stream must use color optical frame')
            if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1000000000:
                raise ValueError('invalid image timestamp')
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                raise ValueError('missing image timestamp')
            if kind.endswith('_info'):
                if not all(math.isfinite(v) for v in list(msg.k) + list(msg.d) + list(msg.p) + list(msg.r)):
                    raise ValueError('non-finite camera calibration')
                if msg.k[0] <= 0 or msg.k[4] <= 0 or msg.k[8] != 1.:
                    raise ValueError('invalid camera intrinsics')
                signature = (msg.width, msg.height, msg.header.frame_id, msg.distortion_model,
                             tuple(msg.k), tuple(msg.d), tuple(msg.r), tuple(msg.p),
                             msg.binning_x, msg.binning_y,
                             msg.roi.x_offset, msg.roi.y_offset, msg.roi.width, msg.roi.height,
                             msg.roi.do_rectify)
            else:
                formats = {'color': {'rgb8': 3, 'bgr8': 3},
                           'depth': {'16UC1': 2, '32FC1': 4}}
                channels = formats[kind].get(msg.encoding)
                if channels is None or msg.step < msg.width * channels:
                    raise ValueError('unsupported image encoding/stride')
                if len(msg.data) != msg.step * msg.height:
                    raise ValueError('truncated image buffer')
                signature = (msg.width, msg.height, msg.header.frame_id, msg.encoding,
                             msg.step, msg.is_bigendian)
            if kind in self.signatures and self.signatures[kind] != signature:
                raise ValueError('stream mode/intrinsics changed; calibration invalid')
            self.signatures[kind] = signature
            self.messages[kind] = msg
            self.last[kind] = now
            if len(self.messages) == 4:
                if len({m.header.frame_id for m in self.messages.values()}) != 1:
                    raise ValueError('registered stream frame mismatch')
                a, b = self.messages['color_info'], self.messages['depth_info']
                if list(a.k) != list(b.k) or list(a.d) != list(b.d):
                    raise ValueError('registered depth intrinsics differ from color')
            return True
        except (ValueError, KeyError) as exc:
            self.fail(str(exc))
            return False

    def check(self, now):
        if not self.fault:
            if len(self.last) < 4 and now - self.started > self.startup_timeout:
                self.fail('startup stream timeout; restart required')
            elif any(now - stamp > self.timeout for stamp in self.last.values()):
                self.fail('stream lost; restart required')
        return not self.fault and len(self.last) == 4


def official_command(width, height, fps):
    return ['ros2', 'launch', 'orbbec_camera', 'gemini_330_series.launch.py',
            'camera_name:=zzx_camera_raw', 'depth_registration:=true', 'align_mode:=SW',
            'enable_frame_sync:=true', 'enable_depth_scale:=true',
            'color_qos:=default', 'depth_qos:=default',
            'color_camera_info_qos:=default', 'depth_camera_info_qos:=default',
            'enable_color:=true', 'enable_depth:=true', 'enable_point_cloud:=false',
            'enable_colored_point_cloud:=false', 'enable_d2c_viewer:=false',
            'enumerate_net_device:=false', 'time_domain:=system',
            f'color_width:={width}', f'color_height:={height}', f'color_fps:={fps}',
            f'depth_width:={width}', f'depth_height:={height}', f'depth_fps:={fps}']
