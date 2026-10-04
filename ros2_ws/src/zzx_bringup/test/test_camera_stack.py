"""Combined fake robot and synthetic camera, including terminal-style shutdown."""
import json
import os
import signal
import subprocess

import rclpy
from std_srvs.srv import Trigger
from zzx_camera.guard import CameraLease
from zzx_http_backend.ownership import CommandLease


def test_camera_and_arm_exit_cleanly(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    rclpy.init()
    client_node = rclpy.create_node('combined_camera_test')
    client = client_node.create_client(Trigger, '/camera/camera_manager/get_status')
    log_path = tmp_path / 'launch.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'zzx_bringup', 'stack.launch.py',
            'profile:=fake', 'camera_backend:=synthetic'],
            start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=8.)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(client_node, future, timeout_sec=3.)
            assert future.done()
            assert json.loads(future.result().message)['backend'] == 'synthetic'
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
            client_node.destroy_node()
            rclpy.shutdown()
    content = log_path.read_text()
    assert process.returncode == 0, content
    assert 'Traceback' not in content, content
    assert 'process has died' not in content, content
    assert 'MoveArm fake backend ready' in content, content
    CameraLease().close()
    CommandLease().close()
