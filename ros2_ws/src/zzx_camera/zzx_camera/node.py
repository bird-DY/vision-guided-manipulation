"""Single producer with shared ROS topics and latched stream faults."""
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger

from .guard import CameraLease, StreamGuard, official_command
from .sources import SdkWorker, packet


TOPICS = {'color': 'color/image_raw', 'depth': 'depth_registered/image_raw',
          'color_info': 'color/camera_info', 'depth_info': 'depth_registered/camera_info'}
CAMERA_QOS = QoSProfile(depth=5)


class CameraManager(Node):
    def __init__(self, **kwargs):
        super().__init__('camera_manager', **kwargs)
        self.lease = None
        self.child = None
        self.worker = None
        self.stop_sent = None
        self.sequence = 0
        self.session_id = str(uuid.uuid4())
        try:
            defaults = {'backend': 'synthetic', 'allow_device': False, 'width': 1280,
                        'height': 720, 'fps': 30, 'stream_timeout': 1.,
                        'startup_timeout': 15., 'synthetic_drop_after': 0.}
            self.settings = {}
            for key, value in defaults.items():
                self.settings[key] = self.declare_parameter(
                    key, value, ParameterDescriptor(read_only=True)).value
            s = self.settings
            if s['backend'] not in ('synthetic', 'orbbec', 'sdk'):
                raise ValueError('backend must be synthetic, orbbec or sdk')
            if not (16 <= s['width'] <= 4096 and 16 <= s['height'] <= 2160 and 1 <= s['fps'] <= 60):
                raise ValueError('unsupported stream dimensions/fps')
            if not (.1 <= s['stream_timeout'] <= 30 and 1 <= s['startup_timeout'] <= 120
                    and s['synthetic_drop_after'] >= 0):
                raise ValueError('invalid timeout/fault injection')
            if s['backend'] != 'synthetic' and not s['allow_device']:
                raise ValueError('hardware requires explicit allow_device:=true')
            if s['backend'] == 'orbbec':
                path = Path(get_package_share_directory('orbbec_camera')) / 'launch/gemini_330_series.launch.py'
                if not path.is_file():
                    raise RuntimeError('official Gemini 330 launch file is missing')
            if s['backend'] == 'sdk' and importlib.util.find_spec('pyorbbecsdk') is None:
                raise RuntimeError('optional pyorbbecsdk is not installed in this ROS Python environment')
            self.lease = CameraLease()
            self.guard = StreamGuard(s['width'], s['height'], s['stream_timeout'], s['startup_timeout'])
            self.publishers_by_stream = {
                key: self.create_publisher(CameraInfo if key.endswith('_info') else Image,
                                           topic, CAMERA_QOS)
                for key, topic in TOPICS.items()}
            self.diagnostics = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
            self.service = self.create_service(Trigger, '~/get_status', self.status)
            self.add_on_set_parameters_callback(self.reject_changes)
            self.subscriptions_by_stream = []
            if s['backend'] == 'orbbec':
                for key in TOPICS:
                    raw = '/zzx_camera_raw/' + TOPICS[key].replace('depth_registered', 'depth')
                    if self.count_publishers(raw):
                        raise RuntimeError('raw camera topic already has a publisher')
                    self.subscriptions_by_stream.append(self.create_subscription(
                        CameraInfo if key.endswith('_info') else Image, raw,
                        lambda msg, key=key: self.receive(key, msg), CAMERA_QOS))
                self.child = subprocess.Popen(official_command(s['width'], s['height'], s['fps']),
                                              start_new_session=True)
            elif s['backend'] == 'sdk':
                self.worker = SdkWorker(s['width'], s['height'], s['fps'])
            self.create_timer(1. / s['fps'], self.acquire)
            self.create_timer(.1, self.monitor)
        except Exception:
            self.close()
            raise

    def reject_changes(self, params):
        return SetParametersResult(successful=False, reason='stream config immutable; restart and revalidate calibration')

    def receive(self, kind, msg):
        now = time.monotonic()
        # Check the previous stream state before allowing a reconnect to refresh it.
        self.guard.check(now)
        if self.guard.accept(kind, msg, now) and self.guard.check(now):
            self.publishers_by_stream[kind].publish(msg)

    def acquire(self):
        try:
            self.acquire_frame()
        except Exception as exc:
            self.guard.fail(f'frame conversion failed: {type(exc).__name__}: {exc}')

    def acquire_frame(self):
        if self.guard.fault or self.settings['backend'] == 'orbbec':
            return
        s = self.settings
        if s['backend'] == 'synthetic':
            if s['synthetic_drop_after'] and time.monotonic() - self.guard.started > s['synthetic_drop_after']:
                return
            self.sequence += 1
            color = np.zeros((s['height'], s['width'], 3), np.uint8)
            color[:, :, 1] = self.sequence % 255
            color[:, s['width']//3:s['width']//2, 2] = 255
            depth = np.full((s['height'], s['width']), .5, np.float32)
            values = (color, depth, (float(s['width']), float(s['width']),
                                    s['width']/2., s['height']/2.), [0.] * 8)
        else:
            if self.worker.error:
                self.guard.fail(self.worker.error)
                return
            try:
                values = self.worker.frames.get_nowait()
            except queue.Empty:
                return
        frames = packet(*values, self.get_clock().now().to_msg())
        for key in ('color_info', 'depth_info', 'color', 'depth'):
            self.receive(key, frames[key])

    def snapshot(self):
        ready = self.guard.check(time.monotonic())
        return {'backend': self.settings['backend'], 'stream_session_id': self.session_id,
                'configuration_sha256': hashlib.sha256(json.dumps(self.settings, sort_keys=True).encode()).hexdigest(),
                'stream_mode': {k: self.settings[k] for k in ('width', 'height', 'fps')},
                'transport_ready': ready, 'motion_eligible': False,
                'calibration_status': 'invalidated' if self.guard.fault else 'unvalidated',
                'timestamp_source': 'vendor_system' if self.child else 'host_publish_time',
                'depth_units': '16UC1:mm or 32FC1:m',
                'source': 'synthetic' if self.settings['backend'] == 'synthetic' else 'hardware',
                'state': 'ERROR' if self.guard.fault else ('STREAMING' if ready else 'STARTING'),
                'reason': self.guard.fault,
                'dropped_worker_frames': self.worker.dropped if self.worker else 0}

    def status(self, request, response):
        data = self.snapshot()
        response.success, response.message = data['transport_ready'], json.dumps(data)
        return response

    def monitor(self):
        if self.child and self.child.poll() is not None:
            self.guard.fail('official driver exited; restart required')
        for key, pub in self.publishers_by_stream.items():
            if self.count_publishers(pub.topic_name) > 1:
                self.guard.fail('duplicate canonical image producer')
        if self.child:
            for topic in TOPICS.values():
                if self.count_publishers('/zzx_camera_raw/' + topic.replace('depth_registered', 'depth')) > 1:
                    self.guard.fail('duplicate raw image producer')
        data = self.snapshot()
        if self.guard.fault:
            if self.worker:
                self.worker.stop_event.set()
            if self.child and self.stop_sent is None:
                self.signal_child(signal.SIGINT)
                self.stop_sent = time.monotonic()
            elif self.child and time.monotonic() - self.stop_sent > 3:
                self.signal_child(signal.SIGKILL)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        diagnostic = DiagnosticStatus()
        diagnostic.name = self.get_fully_qualified_name()
        diagnostic.hardware_id = 'synthetic' if self.settings['backend'] == 'synthetic' else 'orbbec_usb'
        diagnostic.level = bytes([2 if self.guard.fault else (0 if data['transport_ready'] else 1)])
        diagnostic.message = data['reason'] or data['state']
        diagnostic.values = [KeyValue(key=key, value=str(value)) for key, value in data.items()]
        msg.status = [diagnostic]
        self.diagnostics.publish(msg)

    def signal_child(self, sig):
        if self.child:
            try:
                os.killpg(self.child.pid, sig)
            except ProcessLookupError:
                pass

    def close(self):
        if self.child:
            self.signal_child(signal.SIGINT)
            try:
                self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.signal_child(signal.SIGKILL)
                self.child.wait(timeout=2)
            # Clean up any component child surviving a terminated launch parent.
            self.signal_child(signal.SIGKILL)
            self.child = None
        if self.worker:
            self.worker.close()
            self.worker = None
        self.destroy_node()
        if self.lease:
            self.lease.close()
            self.lease = None


def main():
    rclpy.init()
    node = None
    try:
        node = CameraManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT and launch's forwarded SIGINT can arrive separately.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
