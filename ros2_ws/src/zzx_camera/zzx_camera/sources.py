"""Optional SDK acquisition on one worker; no fallback resize or guessed calibration."""
import copy
import queue
import threading

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image


FRAME = 'camera_color_optical_frame'


def packet(color, depth, intrinsic, distortion, stamp):
    height, width = color.shape[:2]
    if color.dtype != np.uint8 or color.shape != (height, width, 3):
        raise ValueError('color must be BGR uint8')
    if depth.shape != (height, width) or depth.dtype != np.float32:
        raise ValueError('SDK must provide registered float32 depth in meters')
    info = CameraInfo()
    info.header.stamp, info.header.frame_id = stamp, FRAME
    info.width, info.height = width, height
    fx, fy, cx, cy = intrinsic
    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    info.distortion_model = 'rational_polynomial'
    info.d = list(distortion)
    result = {'color_info': info, 'depth_info': copy.deepcopy(info)}
    for name, array, encoding, stride in (
            ('color', color, 'bgr8', width * 3), ('depth', depth, '32FC1', width * 4)):
        image = Image()
        image.header = copy.deepcopy(info.header)
        image.width, image.height = width, height
        image.encoding, image.step = encoding, stride
        image.is_bigendian = False
        image.data = np.ascontiguousarray(array).tobytes()
        result[name] = image
    return result


class SdkWorker:
    def __init__(self, width, height, fps):
        self.width, self.height, self.fps = width, height, fps
        self.frames = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.error = ''
        self.dropped = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        pipeline = None
        started = False
        try:
            import pyorbbecsdk as ob
            context = ob.Context()
            context.enable_net_device_enumeration(False)
            devices = context.query_devices()
            if devices.get_count() != 1:
                raise RuntimeError('SDK profile requires exactly one visible USB camera')
            pipeline = ob.Pipeline(devices.get_device_by_index(0))
            config = ob.Config()
            for sensor, fmt in ((ob.OBSensorType.COLOR_SENSOR, ob.OBFormat.RGB),
                                (ob.OBSensorType.DEPTH_SENSOR, ob.OBFormat.Y16)):
                profile = pipeline.get_stream_profile_list(sensor).get_video_stream_profile(
                    self.width, self.height, fmt, self.fps)
                config.enable_stream(profile)
            pipeline.enable_frame_sync()
            pipeline.start(config)
            started = True
            align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
            while not self.stop_event.is_set():
                frames = pipeline.wait_for_frames(100)
                if frames is None:
                    continue
                aligned = align.process(frames)
                if aligned is None:
                    continue
                frames = aligned.as_frame_set()
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if color is None or depth is None:
                    continue
                cp = color.get_stream_profile().as_video_stream_profile()
                intr, dist = cp.get_intrinsic(), cp.get_distortion()
                if 'BROWN' not in str(dist.model).upper():
                    raise RuntimeError('unsupported SDK distortion model; use official ROS driver')
                if (intr.width, intr.height) != (color.get_width(), color.get_height()):
                    raise RuntimeError('SDK intrinsics resolution mismatch')
                rgb = np.frombuffer(color.get_data(), dtype=np.uint8).reshape(
                    color.get_height(), color.get_width(), 3)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                scale = float(depth.get_depth_scale()) * .001
                if not np.isfinite(scale) or scale <= 0:
                    raise RuntimeError('invalid per-frame SDK depth scale')
                meters = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(
                    depth.get_height(), depth.get_width()).astype(np.float32) * scale
                if meters.shape != bgr.shape[:2]:
                    raise RuntimeError('D2C failed; refusing to resize depth')
                value = (bgr, meters.astype(np.float32),
                         tuple(float(v) for v in (intr.fx, intr.fy, intr.cx, intr.cy)),
                         [float(getattr(dist, key)) for key in
                          ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')])
                try:
                    self.frames.put_nowait(value)
                except queue.Full:
                    try:
                        self.frames.get_nowait()
                    except queue.Empty:
                        pass
                    self.dropped += 1
                    self.frames.put_nowait(value)
        except Exception as exc:
            self.error = f'SDK acquisition failed: {type(exc).__name__}: {exc}'
        finally:
            if started:
                try:
                    pipeline.stop()
                except Exception as exc:
                    self.error = f'SDK stop failed: {exc}'

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('SDK worker did not stop; process exit required')
