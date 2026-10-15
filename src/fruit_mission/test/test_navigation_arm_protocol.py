"""Isolated DDS action/service checks with simulated peers, never hardware."""

import math
import os
from pathlib import Path
import tempfile
import threading
import unittest

from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
import yaml

from fruit_interfaces.msg import ArmStatus
from fruit_interfaces.srv import ExecuteArmPreset
from fruit_mission.actions import navigate
from fruit_mission.config import load_config
from fruit_mission.mission_context import MissionContext


class NavigationArmProtocolTest(unittest.TestCase):
    def setUp(self):
        self.localhost_before = os.environ.get('ROS_LOCALHOST_ONLY')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(domain_id=202, signal_handler_options=SignalHandlerOptions.NO)
        self.directory = tempfile.TemporaryDirectory()
        config_dir = Path(__file__).resolve().parents[1] / 'config'
        config = load_config(str(config_dir / 'fruit_mission.yaml'))
        config['dataset_output_dir'] = self.directory.name
        config['follow_waypoints_action'] = '/offline/follow_waypoints'
        self.client = rclpy.create_node('offline_mission')
        self.peer = rclpy.create_node('fruit_perception_node')
        self.peer.declare_parameter('zone', 'B')
        self.presets = []
        self.failed_presets = set()
        self.navigation_succeeds = False
        self.goals = 0
        self.stops = 0
        self.status = ArmStatus()
        self.status.task_status = ArmStatus.STATUS_COMPLETED
        with (config_dir / 'camera_extrinsic.yaml').open() as stream:
            arm_config_path = yaml.safe_load(stream)['arm_config_path']
        with Path(arm_config_path).open() as stream:
            arm_config = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
        mapping = arm_config['joint_mapping']
        for index in range(1, 5):
            lower, upper = mapping[f'q{index}']['limits_deg']
            setattr(self.status, f'joint_{index}', math.radians((lower + upper) / 2))
        self.publisher = self.peer.create_publisher(
            ArmStatus, '/fruit_hardware_bridge/arm/status', 10)
        self.peer.create_timer(0.02, lambda: self.publisher.publish(self.status))
        self.peer.create_service(
            ExecuteArmPreset, '/arm_planner_node/execute_preset', self.execute_preset)
        self.peer.create_service(Trigger, '/arm_planner_node/stop', self.stop)
        self.action = ActionServer(
            self.peer, FollowWaypoints, config['follow_waypoints_action'], self.follow,
            callback_group=ReentrantCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.peer)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.ctx = MissionContext(self.client, config, speaker=object())
        self.assertTrue(self.ctx.report_current_area('B'))
        self.goal = self.ctx.route.b_goal_names()[0]

    def tearDown(self):
        self.ctx.close()
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.action.destroy()
        self.peer.destroy_node()
        self.client.destroy_node()
        rclpy.try_shutdown()
        self.directory.cleanup()
        if self.localhost_before is None:
            os.environ.pop('ROS_LOCALHOST_ONLY', None)
        else:
            os.environ['ROS_LOCALHOST_ONLY'] = self.localhost_before

    def follow(self, goal_handle):
        self.goals += 1
        if self.navigation_succeeds:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return FollowWaypoints.Result()

    def execute_preset(self, request, response):
        self.presets.append(request.preset_name)
        response.success = request.preset_name not in self.failed_presets
        response.message = 'offline fixture'
        if response.success:
            self.status.trajectory_id += 1
            response.trajectory_id = self.status.trajectory_id
        # Rejection with ID=0 did not submit motion and preserves stationary feedback.
        self.publisher.publish(self.status)
        return response

    def stop(self, request, response):
        self.stops += 1
        self.status.task_status = ArmStatus.STATUS_IDLE
        self.publisher.publish(self.status)
        response.success = True
        return response

    def test_chassis_failure_does_not_send_arm_requests(self):
        self.assertFalse(navigate(self.ctx, [self.goal]))
        self.navigation_succeeds = True
        self.assertTrue(navigate(self.ctx, [self.goal]))
        self.assertEqual(self.goals, 2)
        self.assertEqual(self.presets, [])
        self.assertEqual(self.stops, 0)

    def test_successful_look_is_consumed_after_chassis_failure(self):
        self.assertFalse(navigate(self.ctx, [self.goal], side='right'))
        self.assertEqual(self.presets, ['right_look_b', 'right_to_home_b'])
        self.assertEqual(self.stops, 0)
        self.navigation_succeeds = True
        self.assertTrue(navigate(self.ctx, [self.goal], side='left'))
        self.assertTrue(self.ctx.finish_observation('left').completed)
        self.assertEqual(self.goals, 2)
        self.assertEqual(self.presets[-1], 'left_look_b')

    def test_arm_failure_recovers_even_when_navigation_succeeded(self):
        self.navigation_succeeds = True
        self.failed_presets.add('shenzhi')
        self.assertFalse(navigate(self.ctx, [self.goal], preset='shenzhi'))
        self.assertEqual(self.presets, ['shenzhi', 'home'])
        self.assertEqual(self.goals, 1)
        self.failed_presets.clear()
        self.assertTrue(navigate(self.ctx, [self.goal]))

    def test_successful_preset_is_consumed_after_chassis_failure(self):
        self.assertFalse(navigate(self.ctx, [self.goal], preset='shenzhi'))
        self.navigation_succeeds = True
        self.assertTrue(navigate(self.ctx, [self.goal]))
        self.assertEqual(self.presets, ['shenzhi'])
        self.assertEqual(self.stops, 0)

    def test_arm_failure_recovers_even_when_navigation_failed(self):
        self.failed_presets.add('shenzhi')
        self.assertFalse(navigate(self.ctx, [self.goal], preset='shenzhi'))
        self.assertEqual(self.presets, ['shenzhi', 'home'])

    def test_look_failure_recovers_its_submitted_side_once(self):
        self.failed_presets.add('right_look_b')
        self.assertFalse(navigate(self.ctx, [self.goal], side='right'))
        self.assertEqual(self.presets, ['right_look_b', 'right_to_home_b'])

    def test_safe_handoff_failure_is_not_retried(self):
        self.failed_presets.add('right_to_home_b')
        self.assertFalse(navigate(self.ctx, [self.goal], side='right'))
        self.assertEqual(self.presets, ['right_look_b', 'right_to_home_b'])
        goals_before = self.goals
        self.assertFalse(navigate(self.ctx, [self.goal], side='left'))
        self.assertEqual(self.goals, goals_before)
        self.assertEqual(self.presets, ['right_look_b', 'right_to_home_b'])


if __name__ == '__main__':
    unittest.main()
