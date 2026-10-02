"""Exercise the installed C++ node over real DDS, without robot hardware."""

import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class FakeArmRosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('fake_arm_smoke_client')
        self.process = None
        self.samples = []

    def tearDown(self):
        if self.process is not None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail('Timed out waiting for fake node condition')

    def start(self, fault):
        namespace = '/fake_smoke_' + str(os.getpid()) + '_' + fault
        prefix = namespace + '/fake_arm_backend'
        executable = Path(get_package_prefix('zzx_execution')) / 'lib/zzx_execution/fake_arm_node'
        self.subscription = self.node.create_subscription(
            JointState, prefix + '/joint_states', self.samples.append, 10)
        self.publisher = self.node.create_publisher(JointState, prefix + '/target', 10)
        self.cancel = self.node.create_client(Trigger, prefix + '/cancel')
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'fault:=' + fault, '-p', 'duration_seconds:=1.0',
            '-p', 'fault_after_seconds:=0.25',
        ])
        self.wait_for(lambda: len(self.samples) >= 2 and self.publisher.get_subscription_count() > 0)
        self.assertEqual(list(self.samples[-1].position), [0.0] * 6)

    def send(self):
        target = JointState()
        target.name = ['joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1']
        target.position = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
        self.publisher.publish(target)

    def test_normal(self):
        self.start('normal')
        self.send()
        self.wait_for(lambda: abs(self.samples[-1].position[5] - 0.6) < 1e-8)
        self.assertEqual(self.samples[-1].name[0], 'joint1')
        self.assertAlmostEqual(self.samples[-1].position[0], 0.1)

    def test_cancel_freezes_feedback(self):
        self.start('slow')
        self.send()
        self.wait_for(lambda: self.samples[-1].position[0] > 0.002)
        self.assertTrue(self.cancel.wait_for_service(timeout_sec=3))
        future = self.cancel.call_async(Trigger.Request())
        self.wait_for(future.done)
        self.assertTrue(future.result().success)
        count = len(self.samples)
        self.wait_for(lambda: len(self.samples) >= count + 10)
        tail = self.samples[-5:]
        self.assertTrue(all(list(s.position) == list(tail[-1].position) for s in tail))
        self.assertLess(tail[-1].position[0], 0.1)

    def test_feedback_loss_stops_publishing(self):
        self.start('feedback_loss')
        self.send()
        self.wait_for(lambda: self.samples[-1].position[0] > 0)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        count = len(self.samples)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(self.samples), count)
        self.assertIsNone(self.process.poll())


if __name__ == '__main__':
    unittest.main()
