"""Ownership is tested across Python, C++ and the disabled legacy entry point."""
import ast
import importlib.util
import inspect
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import textwrap
import time
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext
import pytest
from zzx_http_backend.ownership import CommandLease


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def test_same_owner_cannot_start_cpp_server():
    resource = 'b11_' + uuid.uuid4().hex
    lease = CommandLease(resource)
    try:
        result = subprocess.run([executable('zzx_execution', 'move_arm_server'),
                                 '--ros-args', '-p', 'ownership_id:=' + resource],
                                capture_output=True, text=True, timeout=5)
        assert result.returncode != 0
        assert 'another command producer owns' in result.stderr
    finally:
        lease.close()
    replacement = CommandLease(resource)
    replacement.close()


def test_legacy_default_disabled_and_same_owner_rejected(monkeypatch):
    from mybot_description.move_arm import ArmController
    monkeypatch.delenv('ZZX_ENABLE_LEGACY', raising=False)
    with pytest.raises(RuntimeError, match='disabled'):
        ArmController()
    resource = 'b11_' + uuid.uuid4().hex
    monkeypatch.setenv('ZZX_ENABLE_LEGACY', '1')
    monkeypatch.setenv('ZZX_ROBOT_ID', resource)
    lease = CommandLease(resource)
    try:
        with pytest.raises(RuntimeError, match='command producer'):
            ArmController()
    finally:
        lease.close()
    tree = ast.parse(textwrap.dedent(inspect.getsource(ArmController.__init__)))
    assert not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and
                   n.func.attr in ('send_arm_trajectory', 'send_claw_trajectory') for n in ast.walk(tree))


def test_profiles_exclude_legacy_and_sim_has_no_task_server():
    share = Path(get_package_share_directory('zzx_bringup'))
    profiles = json.loads((share / 'config/profiles.json').read_text())
    assert set(profiles) == {'fake', 'http', 'sim'}
    assert profiles['sim']['task_capable'] is False
    assert 'move_arm.py' not in json.dumps(profiles)
    spec = importlib.util.spec_from_file_location('bringup_launch', share / 'launch/stack.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(profile='invalid', ownership_id='test', namespace='robot')
    with pytest.raises(ValueError, match='profile'):
        module.build(context)


def test_fake_launch_and_second_owner_exit():
    resource = 'b11_' + uuid.uuid4().hex
    command = ['ros2', 'launch', 'zzx_bringup', 'stack.launch.py', 'profile:=fake',
               'ownership_id:=' + resource, 'namespace:=b11_test']
    with tempfile.TemporaryFile(mode='w+') as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 8
            while True:
                output.seek(0)
                if 'MoveArm fake backend ready' in output.read():
                    break
                assert process.poll() is None
                assert time.monotonic() < deadline
                time.sleep(.05)
            second = subprocess.run(command, capture_output=True, text=True, timeout=8)
            assert 'another command producer owns' in second.stdout + second.stderr
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        replacement = CommandLease(resource)
        replacement.close()


def test_http_launch_starts_inactive_and_shuts_down_children(monkeypatch):
    import rclpy
    from std_srvs.srv import Trigger
    from zzx_http_contracts.client import LoopbackClient, UnknownOutcome
    for key, value in dict(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='1',
                           RMW_IMPLEMENTATION='rmw_cyclonedds_cpp').items():
        monkeypatch.setenv(key, value)
    sockets = [socket.socket(), socket.socket()]
    try:
        for sock in sockets:
            sock.bind(('127.0.0.1', 0))
        ports = [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()
    resource = 'b11_' + uuid.uuid4().hex
    command = ['ros2', 'launch', 'zzx_bringup', 'stack.launch.py', 'profile:=http',
               'ownership_id:=' + resource, 'namespace:=b11_http',
               f'arm_port:={ports[0]}', f'hand_port:={ports[1]}']
    rclpy.init()
    node = rclpy.create_node('bringup_probe')
    try:
        with tempfile.TemporaryFile(mode='w+') as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                client = node.create_client(Trigger, '/b11_http/http_action_backend/get_status')
                assert client.wait_for_service(timeout_sec=8)
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                assert future.done()
                state = json.loads(future.result().message)
                assert not state['active']
                motors = LoopbackClient(f'http://127.0.0.1:{ports[0]}').request('GET', '/api/motors')
                assert all(motor['enabled'] == 0 for motor in motors.values())
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
        for port in ports:
            with pytest.raises(UnknownOutcome):
                LoopbackClient(f'http://127.0.0.1:{port}').request('GET', '/api/status')
    finally:
        node.destroy_node()
        rclpy.shutdown()
