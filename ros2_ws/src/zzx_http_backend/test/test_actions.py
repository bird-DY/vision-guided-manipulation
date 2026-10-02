"""ROS-to-HTTP integration using only loopback fake servers."""
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from zzx_interfaces.action import MoveArm, ControlHand
from zzx_interfaces.msg import ErrorStatus as E
from zzx_http_contracts.fake import FakeServer, Scenario
from zzx_http_contracts.protocol import ARM_NAMES


class HttpActionsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.update(ROS_DOMAIN_ID='89', ROS_LOCALHOST_ONLY='1',
                          RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.node = rclpy.create_node('http_adapter_test')
        self.addCleanup(self.node.destroy_node)

    def start(self, scenario=None):
        self.arm_http = self.stack.enter_context(FakeServer(scenario=scenario))
        self.hand_http = self.stack.enter_context(FakeServer(kind='hand'))
        namespace = '/http_test_' + str(os.getpid()) + '_' + self._testMethodName
        executable = Path(get_package_prefix('zzx_http_backend')) / 'lib/zzx_http_backend/http_action_backend'
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=' + namespace,
            '-p', 'arm_url:=' + self.arm_http.url, '-p', 'hand_url:=' + self.hand_http.url])
        self.addCleanup(self.stop_process)
        self.arm = ActionClient(self.node, MoveArm, namespace + '/zzx/manipulation/move_arm')
        self.hand = ActionClient(self.node, ControlHand, namespace + '/zzx/manipulation/control_hand')
        self.addCleanup(self.arm.destroy)
        self.addCleanup(self.hand.destroy)
        self.status = self.node.create_client(Trigger, namespace + '/http_action_backend/get_status')
        self.assertTrue(self.arm.wait_for_server(timeout_sec=6))
        self.assertTrue(self.hand.wait_for_server(timeout_sec=6))
        self.assertTrue(self.status.wait_for_service(timeout_sec=3))
        self.assertEqual(self.arm_http.model.requests, [])
        self.assertEqual(self.hand_http.model.requests, [])

    def stop_process(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)

    def wait(self, future, timeout=6):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.01)
        self.assertTrue(future.done(), 'future did not complete')
        return future.result()

    def goal(self):
        goal = MoveArm.Goal()
        goal.target_type = goal.TARGET_JOINTS
        goal.joint_target.name = list(reversed(ARM_NAMES))
        goal.joint_target.position = [.7, .6, .5, .4, .3, .2, .1]
        goal.velocity_scaling = goal.acceleration_scaling = .1
        goal.timeout.sec = 4
        return goal

    def send(self, client, goal):
        handle = self.wait(client.send_goal_async(goal))
        self.assertTrue(handle.accepted)
        return handle

    def test_arm_prepare_named_order_and_settling(self):
        self.start()
        result = self.wait(self.send(self.arm, self.goal()).get_result_async())
        self.assertEqual(result.status, 4)
        self.assertEqual(result.result.error.code, E.OK)
        self.assertEqual(list(result.result.final_joint_state.position), [.1, .2, .3, .4, .5, .6, .7])
        posts = [path for method, path, body in self.arm_http.model.requests if method == 'POST']
        self.assertEqual(posts, ['/api/teach_mode', '/api/control_mode', '/api/enable', '/api/joints'])

    def test_plan_only_does_not_enable_or_move(self):
        self.start()
        goal = self.goal()
        goal.plan_only = True
        result = self.wait(self.send(self.arm, goal).get_result_async()).result
        self.assertTrue(result.success)
        self.assertEqual(result.execution.stage, 'planned_only')
        self.assertEqual(self.arm_http.model.execution_count, 0)
        self.assertFalse(self.arm_http.model.enabled)

    def test_hand_sparse_and_unsupported_force(self):
        self.start()
        goal = ControlHand.Goal()
        goal.position_unit = goal.UNIT_NORMALIZED
        goal.joint_names, goal.positions = ['index_pip'], [0.]
        goal.speed_scaling, goal.timeout.sec = 1., 3
        result = self.wait(self.send(self.hand, goal).get_result_async()).result
        self.assertTrue(result.success)
        self.assertEqual(list(result.final_positions), [0.])
        self.assertEqual(self.hand_http.model.positions, [.5, .5, .5, .5, 0., .5, .5, .5, .5, .5])
        goal.max_effort = 1.
        result = self.wait(self.send(self.hand, goal).get_result_async()).result
        self.assertEqual(result.error.code, E.UNSUPPORTED)
        self.assertEqual(self.hand_http.model.execution_count, 1)

    def test_backend_failure_stops_and_aborts(self):
        self.start(Scenario(execution_failure=True))
        result = self.wait(self.send(self.arm, self.goal()).get_result_async()).result
        self.assertEqual(result.error.code, E.EXECUTION_FAILED)
        self.assertIn('stop confirmed', result.error.message)
        self.assertFalse(result.success)

    def test_disconnect_latches_and_rejects_next_goal(self):
        self.start(Scenario(disconnect=True))
        result = self.wait(self.send(self.arm, self.goal()).get_result_async()).result
        self.assertEqual(result.error.code, E.RESULT_UNKNOWN)
        self.assertFalse(self.wait(self.arm.send_goal_async(self.goal())).accepted)
        data = json.loads(self.wait(self.status.call_async(Trigger.Request())).message)
        self.assertTrue(data['latched']['arm'])

    def test_blocked_http_does_not_block_status_or_cancel(self):
        self.start(Scenario(motion_seconds=.4, response_delay=.3))
        handle = self.send(self.arm, self.goal())
        deadline = time.monotonic() + 3
        while not self.arm_http.model.moving:
            rclpy.spin_once(self.node, timeout_sec=.01)
            self.assertLess(time.monotonic(), deadline)
        self.assertFalse(self.wait(self.arm.send_goal_async(self.goal())).accepted)
        started = time.monotonic()
        self.assertTrue(self.wait(self.status.call_async(Trigger.Request()), .3).success)
        self.assertLess(time.monotonic() - started, .3)
        canceled = self.wait(handle.cancel_goal_async(), .3)
        self.assertEqual(len(canceled.goals_canceling), 1)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, 5)
        self.assertEqual(result.result.error.code, E.CANCELED)
        self.assertFalse(self.arm_http.model.moving)

    def test_trajectory_configuration_fails_at_startup(self):
        executable = Path(get_package_prefix('zzx_http_backend')) / 'lib/zzx_http_backend/http_action_backend'
        result = subprocess.run([str(executable), '--ros-args', '-p', 'motion_mode:=trajectory'],
                                capture_output=True, text=True, timeout=6)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('trajectory mode is unsupported', result.stderr)


if __name__ == '__main__':
    unittest.main()
