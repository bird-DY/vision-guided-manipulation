"""Dispatch twenty same-ID requests through the real C++ fake Action server."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionClient
from zzx_interfaces.action import MoveArm
from zzx_task_runtime.ledger import Ledger, ReconciliationRequired
from zzx_task_runtime.move_arm import RosMoveArmBackend, normalize


class RosDispatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID'] = '88'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.namespace = '/b08_test_' + str(os.getpid())
        executable = Path(get_package_prefix('zzx_execution')) / 'lib/zzx_execution/move_arm_server'
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=' + self.namespace])
        self.directory = tempfile.TemporaryDirectory()
        self.ledger = Ledger(Path(self.directory.name) / 'tasks.sqlite3')
        self.backend = RosMoveArmBackend(wait_seconds=5)
        self.payload = normalize(dict(
            joint_names=['joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1'],
            positions=[.6, .5, .4, .3, .2, .1], velocity_scaling=1.,
            acceleration_scaling=1., plan_only=False, timeout_seconds=3.),
            self.namespace + '/zzx/manipulation/move_arm')
        client = ActionClient(self.backend.node, MoveArm, self.payload['action_name'])
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5))
        finally:
            client.destroy()

    def tearDown(self):
        self.backend.close()
        self.ledger.close()
        self.directory.cleanup()
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)

    def test_concurrent_actual_action_is_sent_once(self):
        barrier = threading.Barrier(20)
        calls = []

        def backend(payload, operation_id):
            calls.append(operation_id)
            return self.backend(payload, operation_id)

        def submit(_):
            barrier.wait(timeout=5)
            return self.ledger.execute('joint-demo', self.payload, backend)

        with ThreadPoolExecutor(max_workers=20) as pool:
            rows = list(pool.map(submit, range(20)))
        self.assertEqual(len(calls), 1)
        self.assertEqual(len({row['operation_id'] for row in rows}), 1)
        row = self.ledger.get('joint-demo')
        self.assertEqual(row['state'], 'SUCCEEDED')
        result = row['backend_result']['result']
        self.assertEqual(result['execution']['operation_id'], row['operation_id'])
        self.assertEqual(result['final_joint_state']['position'], [.1, .2, .3, .4, .5, .6])
        self.assertEqual(self.ledger.execute('joint-demo', self.payload, backend)['state'], 'SUCCEEDED')
        self.assertEqual(len(calls), 1)

    def test_response_timeout_fences_backend(self):
        self.backend.wait_seconds = .1
        row = self.ledger.execute('lost-result', self.payload, self.backend)
        self.assertEqual(row['state'], 'NEEDS_RECONCILIATION')
        with self.assertRaises(ReconciliationRequired):
            self.ledger.reserve('another-operation', self.payload)


if __name__ == '__main__':
    unittest.main()
