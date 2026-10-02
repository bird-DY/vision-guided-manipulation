"""Real DDS Action tests. Only the deterministic fake backend is launched."""
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from zzx_interfaces.action import MoveArm
from zzx_interfaces.msg import ErrorStatus, ExecutionStatus


class MoveArmTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('move_arm_test')
        self.client = None
        self.process = None
        self.samples = []
        self.feedback = []

    def tearDown(self):
        if self.process:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if self.client:
            self.client.destroy()
        self.node.destroy_node()

    def wait(self, condition, timeout=6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
        self.fail('action condition timed out')

    def start(self, fault='normal'):
        namespace = '/action_test_' + str(os.getpid()) + '_' + self._testMethodName
        self.subscription = self.node.create_subscription(
            JointState, namespace + '/move_arm_server/joint_states', self.samples.append, 10)
        self.client = ActionClient(self.node, MoveArm, namespace + '/zzx/manipulation/move_arm')
        executable = Path(get_package_prefix('zzx_execution')) / 'lib/zzx_execution/move_arm_server'
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'fault:=' + fault, '-p', 'duration_seconds:=0.4'])
        self.assertTrue(self.client.wait_for_server(timeout_sec=5))
        self.wait(lambda: len(self.samples) >= 3)
        self.assertEqual(list(self.samples[-1].position), [0.] * 6)

    def goal(self):
        goal = MoveArm.Goal()
        goal.target_type = MoveArm.Goal.TARGET_JOINTS
        goal.joint_target.name = ['joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1']
        goal.joint_target.position = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
        goal.velocity_scaling = 1.
        goal.acceleration_scaling = 1.
        goal.timeout.sec = 4
        return goal

    def send(self, goal):
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback.append)
        self.wait(future.done)
        return future.result()

    def result(self, handle):
        self.assertTrue(handle.accepted)
        future = handle.get_result_async()
        self.wait(future.done)
        return future.result()

    def test_success_named_mapping_and_settling(self):
        self.start()
        result = self.result(self.send(self.goal()))
        self.assertEqual(result.status, 4)
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.error.code, ErrorStatus.OK)
        self.assertEqual(list(result.result.final_joint_state.position), [.1, .2, .3, .4, .5, .6])
        states = [f.feedback.execution.state for f in self.feedback]
        self.assertIn(ExecutionStatus.STATE_EXECUTING, states)
        self.assertIn(ExecutionStatus.STATE_SETTLING, states)
        self.assertEqual(len(result.result.execution.operation_id), 32)

    def test_plan_only_does_not_move(self):
        self.start()
        goal = self.goal()
        goal.plan_only = True
        result = self.result(self.send(goal)).result
        self.assertTrue(result.success)
        self.assertEqual(result.execution.stage, 'planned_only')
        self.assertEqual(list(result.final_joint_state.position), [0.] * 6)

    def test_invalid_targets_do_not_move(self):
        self.start()
        for case in ['nan', 'duplicate', 'unknown', 'limit', 'zero', 'negative',
                     'acceleration', 'timeout', 'length', 'effort', 'type', 'mixed']:
            with self.subTest(case=case):
                goal = self.goal()
                if case == 'nan': goal.joint_target.position[0] = float('nan')
                if case == 'duplicate': goal.joint_target.name[0] = 'joint1'
                if case == 'unknown': goal.joint_target.name[0] = 'absent'
                if case == 'limit': goal.joint_target.position[0] = 10.
                if case == 'zero': goal.velocity_scaling = 0.
                if case == 'negative': goal.velocity_scaling = -0.1
                if case == 'acceleration': goal.acceleration_scaling = float('inf')
                if case == 'timeout': goal.timeout.sec = 0
                if case == 'length': goal.joint_target.position = [0.]
                if case == 'effort': goal.joint_target.effort = [1.] * 6
                if case == 'type': goal.target_type = 99
                if case == 'mixed': goal.pose_target.header.frame_id = 'base_link'
                result = self.result(self.send(goal))
                self.assertEqual(result.status, 6)
                self.assertEqual(result.result.error.code, ErrorStatus.INVALID_ARGUMENT)
                self.assertEqual(list(result.result.final_joint_state.position), [0.] * 6)

    def test_pose_is_explicitly_unsupported(self):
        self.start()
        goal = self.goal()
        goal.target_type = MoveArm.Goal.TARGET_POSE
        result = self.result(self.send(goal)).result
        self.assertEqual(result.error.code, ErrorStatus.UNSUPPORTED)
        self.assertEqual(list(result.final_joint_state.position), [0.] * 6)

    def test_busy_cancel_and_reuse(self):
        self.start('slow')
        handle = self.send(self.goal())
        self.assertTrue(handle.accepted)
        self.assertFalse(self.send(self.goal()).accepted)
        self.wait(lambda: self.samples[-1].position[0] > 0)
        cancel = handle.cancel_goal_async()
        self.wait(cancel.done)
        self.assertEqual(len(cancel.result().goals_canceling), 1)
        result = self.result(handle)
        self.assertEqual(result.status, 5)
        self.assertEqual(result.result.error.code, ErrorStatus.CANCELED)
        stopped = list(result.result.final_joint_state.position)
        count = len(self.samples)
        self.wait(lambda: len(self.samples) > count + 5)
        self.assertEqual(list(self.samples[-1].position), stopped)
        goal = self.goal()
        goal.plan_only = True
        self.assertTrue(self.result(self.send(goal)).result.success)

    def test_stuck_times_out(self):
        self.start('stuck')
        goal = self.goal()
        goal.timeout.sec = 1
        result = self.result(self.send(goal)).result
        self.assertEqual(result.error.code, ErrorStatus.TIMEOUT)
        self.assertFalse(result.success)

    def test_feedback_loss_latches_unavailable(self):
        self.start('feedback_loss')
        result = self.result(self.send(self.goal())).result
        self.assertEqual(result.error.code, ErrorStatus.STALE_DATA)
        self.assertEqual(list(result.final_joint_state.position), [])
        self.assertFalse(self.send(self.goal()).accepted)

    def test_backend_reject_is_not_success(self):
        self.start('reject')
        result = self.result(self.send(self.goal())).result
        self.assertEqual(result.error.code, ErrorStatus.BACKEND_REJECTED)
        self.assertFalse(result.success)


if __name__ == '__main__':
    unittest.main()
