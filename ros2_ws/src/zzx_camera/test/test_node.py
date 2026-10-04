import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import pytest
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger

from zzx_camera.guard import CameraLease
from zzx_camera.node import CameraManager


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '93')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', 'file://' + str(
        Path(get_package_share_directory('zzx_camera')) / 'config/cyclonedds_wsl.xml'))
    rclpy.init()
    yield
    rclpy.shutdown()


def manager(**params):
    values = dict(width=24, height=16, fps=20, stream_timeout=.4, startup_timeout=3.)
    values.update(params)
    return CameraManager(namespace='/camera', parameter_overrides=[
        Parameter(key, value=value) for key, value in values.items()])


def spin_until(executor, predicate, seconds=5.):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        executor.spin_once(timeout_sec=.02)
        if predicate():
            return
    raise AssertionError('ROS condition timeout')


def test_two_consumers_share_one_producer_and_immutable_mode(runtime):
    owner = manager()
    consumer = rclpy.create_node('camera_consumers')
    executor = SingleThreadedExecutor()
    executor.add_node(owner)
    executor.add_node(consumer)
    first, second, infos, diagnostics = [], [], [], []
    consumer.create_subscription(Image, '/camera/color/image_raw', first.append, qos_profile_sensor_data)
    consumer.create_subscription(Image, '/camera/color/image_raw', second.append, qos_profile_sensor_data)
    consumer.create_subscription(CameraInfo, '/camera/depth_registered/camera_info', infos.append, qos_profile_sensor_data)
    consumer.create_subscription(DiagnosticArray, '/camera/diagnostics', diagnostics.append, 10)
    try:
        spin_until(executor, lambda: len(first) >= 3 and len(second) >= 3 and infos and diagnostics)
        assert consumer.count_publishers('/camera/color/image_raw') == 1
        assert first[-1].width == second[-1].width == 24
        assert infos[-1].header.frame_id == first[-1].header.frame_id
        assert owner.snapshot()['transport_ready']
        assert owner.snapshot()['motion_eligible'] is False
        changed = owner.set_parameters([Parameter('width', value=32)])
        assert not changed[0].successful
        assert owner.get_parameter('width').value == 24
        with pytest.raises(RuntimeError, match='already owned'):
            manager()
    finally:
        executor.shutdown()
        consumer.destroy_node()
        owner.close()


def test_dropout_latches_no_resume_and_new_session(runtime):
    owner = manager(synthetic_drop_after=.8)
    executor = SingleThreadedExecutor()
    executor.add_node(owner)
    old_id = owner.session_id
    try:
        spin_until(executor, lambda: owner.snapshot()['transport_ready'])
        spin_until(executor, lambda: owner.snapshot()['state'] == 'ERROR')
        assert owner.snapshot()['calibration_status'] == 'invalidated'
        owner.settings['synthetic_drop_after'] = 0.
        for _ in range(10):
            executor.spin_once(timeout_sec=.02)
        assert not owner.snapshot()['transport_ready']
    finally:
        executor.shutdown()
        owner.close()
    restarted = manager()
    try:
        assert restarted.session_id != old_id
        assert not restarted.snapshot()['transport_ready']
        assert restarted.snapshot()['calibration_status'] == 'unvalidated'
    finally:
        restarted.close()


@pytest.mark.parametrize('backend', ['sdk', 'orbbec'])
def test_hardware_requires_opt_in(runtime, backend):
    with pytest.raises(ValueError, match='allow_device'):
        manager(backend=backend)
    CameraLease().close()


def test_launch_exits_and_releases_camera(runtime):
    process = subprocess.Popen([
        'ros2', 'launch', 'zzx_camera', 'camera.launch.py', 'width:=24', 'height:=16', 'fps:=10'],
        start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    consumer = rclpy.create_node('launch_camera_test')
    service = consumer.create_client(Trigger, '/camera/camera_manager/get_status')
    try:
        assert service.wait_for_service(timeout_sec=8.)
        future = service.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(consumer, future, timeout_sec=3.)
        assert future.done()
        assert json.loads(future.result().message)['backend'] == 'synthetic'
        with pytest.raises(RuntimeError, match='owned'):
            CameraLease()
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=6.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.)
        consumer.destroy_node()
    CameraLease().close()


def test_official_child_startup_failure_is_stopped(runtime, tmp_path, monkeypatch):
    import zzx_camera.node as module
    (tmp_path / 'launch').mkdir()
    (tmp_path / 'launch/gemini_330_series.launch.py').write_text('# test stub only')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(tmp_path))
    monkeypatch.setattr(module, 'official_command', lambda *args: [
        sys.executable, '-c', 'import time; time.sleep(60)'])
    owner = manager(backend='orbbec', allow_device=True, startup_timeout=1.)
    executor = SingleThreadedExecutor()
    executor.add_node(owner)
    child = owner.child
    try:
        spin_until(executor, lambda: bool(owner.guard.fault))
        spin_until(executor, lambda: child.poll() is not None)
        assert 'startup' in owner.guard.fault
    finally:
        executor.shutdown()
        owner.close()
    assert child.poll() is not None
    CameraLease().close()


def test_duplicate_canonical_publisher_latches(runtime):
    owner = manager()
    duplicate = rclpy.create_node('unauthorized_image_producer')
    duplicate.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(owner)
    executor.add_node(duplicate)
    try:
        spin_until(executor, lambda: bool(owner.guard.fault))
        assert 'duplicate canonical' in owner.guard.fault
    finally:
        executor.shutdown()
        duplicate.destroy_node()
        owner.close()


@pytest.mark.parametrize('width,height,fps', [(32, 16, 20), (1280, 720, 5)])
def test_recorder_and_subscriber_share_camera(runtime, tmp_path, width, height, fps):
    owner = manager(width=width, height=height, fps=fps, stream_timeout=2.)
    subscriber = rclpy.create_node('algorithm_image_consumer')
    images = []
    depths = []
    subscriber.create_subscription(Image, '/camera/color/image_raw', images.append, qos_profile_sensor_data)
    subscriber.create_subscription(Image, '/camera/depth_registered/image_raw', depths.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(owner)
    executor.add_node(subscriber)
    topics = ['/camera/color/image_raw', '/camera/depth_registered/image_raw',
              '/camera/color/camera_info', '/camera/depth_registered/camera_info', '/camera/diagnostics']
    log_path = tmp_path / 'recorder.log'
    log = log_path.open('w')
    qos_path = Path(get_package_share_directory('zzx_camera')) / 'config/record_qos.yaml'
    process = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(tmp_path / 'bag'),
                                '--qos-profile-overrides-path', str(qos_path), *topics],
                               start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
    try:
        spin_until(executor, lambda: all(owner.count_subscribers(topic) >= (2 if topic in topics[:2] else 1)
                                        for topic in topics), seconds=8.)
        until = time.monotonic() + 1.
        spin_until(executor, lambda: time.monotonic() > until and len(images) >= 5)
        assert owner.snapshot()['transport_ready']
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=6.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.)
        executor.shutdown()
        subscriber.destroy_node()
        owner.close()
        log.close()
    metadata = yaml.safe_load((tmp_path / 'bag/metadata.yaml').read_text())
    counts = {entry['topic_metadata']['name']: entry['message_count']
              for entry in metadata['rosbag2_bagfile_information']['topics_with_message_count']}
    assert all(counts.get(topic, 0) > 0 for topic in topics), f'{counts}; local_depth={len(depths)}'
