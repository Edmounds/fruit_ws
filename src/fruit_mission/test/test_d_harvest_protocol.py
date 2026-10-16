"""Isolated DDS and target selection checks for D-area harvesting."""

import math
import os
from pathlib import Path
import tempfile
import threading
import unittest

from geometry_msgs.msg import PointStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
import yaml

from fruit_interfaces.msg import ArmStatus, ArmTelemetry
from fruit_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
from fruit_mission.Logic_acbd import _d_transition_straight
from fruit_mission.actions import (
    _select_b_targets,
    _select_d_targets,
    navigate,
    pick_d_tree,
)
from fruit_mission.camera_target_planner_client import store_preset_name
from fruit_mission.config import load_config
from fruit_mission.mission_context import BTreeTarget, MissionContext, _ArmMotionResult


class DHarvestProtocolTest(unittest.TestCase):
    def setUp(self):
        self.localhost_before = os.environ.get('ROS_LOCALHOST_ONLY')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init(domain_id=203, signal_handler_options=SignalHandlerOptions.NO)
        self.directory = tempfile.TemporaryDirectory()
        config_dir = Path(__file__).resolve().parents[1] / 'config'
        config = load_config(str(config_dir / 'fruit_mission.yaml'))
        config['dataset_output_dir'] = self.directory.name
        config['follow_waypoints_action'] = '/offline_d/follow_waypoints'
        config['waypoint_file'] = str(
            Path(__file__).resolve().parents[2]
            / 'navigation/fruit_nav_bringup/waypoints/odom_waypoints_acbd.yaml')
        config['mission_flow'] = 'acbd'
        self.client = rclpy.create_node('offline_d_client')
        self.peer = rclpy.create_node('fruit_perception_node')
        self.peer.declare_parameter('zone', 'D')
        self.presets = []
        self.failed_presets = set()
        self.cartesian_moves = []
        self.navigation_goals = 0
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

        self.status_pub = self.peer.create_publisher(
            ArmStatus, '/fruit_hardware_bridge/arm/status', 10)
        self.peer.create_timer(0.02, lambda: self.status_pub.publish(self.status))

        self.telemetry_pub = self.peer.create_publisher(
            ArmTelemetry, '/arm_planner_node/joint_telemetry', 10)

        def publish_telemetry():
            telem = ArmTelemetry()
            telem.header.frame_id = 'arm_base_link'
            telem.header.stamp = self.peer.get_clock().now().to_msg()
            telem.tcp_position.x = 0.2
            telem.tcp_position.y = 0.0
            telem.tcp_position.z = 0.15
            telem.tcp_pitch_deg = 0.0
            for name in ('hardware_joint_state', 'model_joint_state'):
                state = getattr(telem, name)
                state.name = [f'joint_{i}' for i in range(1, 5)]
                state.position = [0.0, 0.0, 0.0, 0.0]
            self.telemetry_pub.publish(telem)

        self.peer.create_timer(0.02, publish_telemetry)

        self.peer.create_service(
            ExecuteArmPreset, '/arm_planner_node/execute_preset', self.execute_preset)
        self.peer.create_service(
            MoveArmCartesian, '/arm_planner_node/move_cartesian', self.move_cartesian)
        self.peer.create_service(
            MoveArmJoints, '/arm_planner_node/move_joints', self.move_joints)
        self.peer.create_service(Trigger, '/arm_planner_node/stop', self.stop)
        self.action = ActionServer(
            self.peer, FollowWaypoints, config['follow_waypoints_action'], self.follow,
            callback_group=ReentrantCallbackGroup())
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.peer)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.ctx = MissionContext(self.client, config, speaker=object())
        self.ctx.announce = lambda _msg: None
        self.ctx.announce_and_wait = lambda _msg, **_kw: None

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
        self.navigation_goals += 1
        goal_handle.succeed()
        return FollowWaypoints.Result()

    def execute_preset(self, request, response):
        self.presets.append(request.preset_name)
        response.success = request.preset_name not in self.failed_presets
        response.message = 'fixture'
        if response.success:
            self.status.trajectory_id += 1
            response.trajectory_id = self.status.trajectory_id
        self.status_pub.publish(self.status)
        return response

    def move_cartesian(self, request, response):
        self.cartesian_moves.append((request.x, request.y, request.z, request.pitch_deg))
        response.success = True
        self.status.trajectory_id += 1
        response.trajectory_id = self.status.trajectory_id
        self.status_pub.publish(self.status)
        return response

    def move_joints(self, request, response):
        response.success = True
        self.status.trajectory_id += 1
        response.trajectory_id = self.status.trajectory_id
        self.status_pub.publish(self.status)
        return response

    def stop(self, request, response):
        self.status.task_status = ArmStatus.STATUS_IDLE
        self.status_pub.publish(self.status)
        response.success = True
        return response

    def test_store_preset_names_for_d(self):
        self.assertEqual(store_preset_name('right', 'D', 'up'), 'right_store_d_up')
        self.assertEqual(store_preset_name('right', 'D', 'down'), 'right_store_d_down')
        self.assertEqual(store_preset_name('left', 'D', 'up'), 'left_store_d_up')
        self.assertEqual(store_preset_name('left', 'D', 'down'), 'left_store_d_down')

    def test_d_limits_independent_configuration(self):
        planner = self.ctx._camera_planner
        self.assertTrue(hasattr(planner, 'd_height_split_m'))
        self.assertTrue(hasattr(planner, 'b_height_split_m'))
        self.assertEqual(planner.d_height_split_m, 0.32)
        # Modify D split independently and verify it does not mutate B split
        planner.d_height_split_m = 0.18
        self.assertEqual(planner.b_height_split_m, 0.12)
        self.assertEqual(planner.d_height_split_m, 0.18)
        planner.d_height_split_m = 0.32

    def test_select_d_targets_filters_and_caps(self):
        targets = [
            BTreeTarget('apple_ripe', PointStamped(), 0.85),
            BTreeTarget('tomato_ripe', PointStamped(), 0.99),
            BTreeTarget('pear_ripe', PointStamped(), 0.95),
            BTreeTarget('apple_unripe', PointStamped(), 0.90),
            BTreeTarget('bad', PointStamped(), 0.92),
            BTreeTarget('apple_ripe', PointStamped(), 0.70),
        ]
        selected = _select_d_targets(targets)
        self.assertEqual(len(selected), 3)
        self.assertEqual(selected[0].model_type, 'pear_ripe')
        self.assertEqual(selected[1].model_type, 'apple_ripe')
        self.assertEqual(selected[2].model_type, 'apple_ripe')

    def test_pick_d_tree_announcements_and_silence(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        announced = []
        self.ctx.announce = lambda msg: announced.append(msg)

        # 1. No targets -> completely silent, recovers home
        self.ctx.wait_b_targets = lambda _wid: []
        res = pick_d_tree(self.ctx, side='right', waypoint='D1')
        self.assertFalse(res)
        self.assertEqual(announced, [])
        self.assertIn('right_to_home_d', self.presets)

        # 2. Only unripe targets -> completely silent
        pt = PointStamped()
        pt.point.x, pt.point.y, pt.point.z = 0.0, 0.3, 0.35
        self.ctx.wait_b_targets = lambda _wid: [
            BTreeTarget('apple_unripe', pt, 0.95),
        ]
        res = pick_d_tree(self.ctx, side='right', waypoint='D1')
        self.assertFalse(res)
        self.assertEqual(announced, [])

        # 3. Two ripe targets -> announces first grasp without position prefix, then '采摘完成。' on each store
        self.ctx._navigation_stopped = True
        self.ctx.camera_target_request = lambda _pt: (object(), 'up')
        self.ctx.move_to_base_target = lambda _req: _ArmMotionResult(True, 'fixture')
        self.ctx.wait_b_targets = lambda _wid: [
            BTreeTarget('apple_ripe', pt, 0.95),
            BTreeTarget('pear_ripe', pt, 0.85),
        ]
        res = pick_d_tree(self.ctx, side='right', waypoint='D1', max_targets=2)
        self.assertTrue(res)
        self.assertEqual(announced, [
            '识别到成熟苹果，开始抓取。',
            '采摘完成。',
            '采摘完成。',
        ])

    def test_d_target_selection_filters_and_caps(self):
        # In D, target is None. Only ripe apple/pear should be selected, sorted by confidence.
        targets = [
            BTreeTarget('apple_ripe', PointStamped(), 0.85),
            BTreeTarget('tomato_ripe', PointStamped(), 0.99),  # vegetable: reject
            BTreeTarget('pear_ripe', PointStamped(), 0.95),
            BTreeTarget('apple_unripe', PointStamped(), 0.90),  # unripe: reject
            BTreeTarget('bad', PointStamped(), 0.92),           # defect: reject
            BTreeTarget('apple_ripe', PointStamped(), 0.70),
            BTreeTarget('pear_ripe', PointStamped(), 0.60),
        ]
        selected = _select_b_targets(self.ctx, 'D1', None, targets)
        # Should contain pear_ripe (0.95), apple_ripe (0.85), apple_ripe (0.70), pear_ripe (0.60)
        self.assertEqual(len(selected), 4)
        self.assertEqual(selected[0].model_type, 'pear_ripe')
        self.assertEqual(selected[1].model_type, 'apple_ripe')
        self.assertEqual(selected[2].model_type, 'apple_ripe')
        self.assertEqual(selected[3].model_type, 'pear_ripe')

    def test_b_target_selection_filters_by_qr(self):
        targets = [
            BTreeTarget('apple_ripe', PointStamped(), 0.85),
            BTreeTarget('pear_ripe', PointStamped(), 0.95),
        ]
        selected = _select_b_targets(self.ctx, 'B1', '苹果', targets)
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0].model_type, 'apple_ripe')

    def test_d_area_observation_and_recovery_sides(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        self.assertEqual(self.ctx._current_area, 'D')

        # D1 is right side
        self.ctx.start_observation('right')
        res = self.ctx.finish_observation('right')
        self.assertTrue(res.completed)
        self.assertIn('right_look_d', self.presets)
        self.assertEqual(self.ctx._b_observation_side, 'right')

        # Recovery should recover to right_to_home_d
        self.assertTrue(self.ctx.recover_home())
        self.assertIn('right_to_home_d', self.presets)

        # Left side
        self.ctx.start_observation('left')
        res = self.ctx.finish_observation('left')
        self.assertTrue(res.completed)
        self.assertIn('left_look_d', self.presets)
        self.assertEqual(self.ctx._b_observation_side, 'left')

        # Recovery should recover to left_to_home_d
        self.assertTrue(self.ctx.recover_home())
        self.assertIn('left_to_home_d', self.presets)

    def test_d_transition_inhibits_straight_when_right_home_fails(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        self.ctx.start_observation('right')
        look = self.ctx.finish_observation('right')
        self.assertTrue(look.completed)
        self.assertEqual(self.ctx._b_observation_side, 'right')
        self.assertFalse(self.ctx._arm_home_confirmed)

        self.failed_presets.add('right_to_home_d')
        home_res = self.ctx.execute_arm_preset('right_to_home_d')
        self.assertFalse(home_res.completed)
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        self.presets.clear()
        self.navigation_goals = 0
        _d_transition_straight(self.ctx)

        self.assertEqual(self.presets, [])
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        nav_res = navigate(self.ctx, ['D2'])
        self.assertFalse(nav_res)
        self.assertEqual(self.navigation_goals, 0)
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

    def test_d_transition_inhibits_straight_when_left_home_fails(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        self.ctx.start_observation('left')
        look = self.ctx.finish_observation('left')
        self.assertTrue(look.completed)
        self.assertEqual(self.ctx._b_observation_side, 'left')
        self.assertFalse(self.ctx._arm_home_confirmed)

        self.failed_presets.add('left_to_home_d')
        home_res = self.ctx.execute_arm_preset('left_to_home_d')
        self.assertFalse(home_res.completed)
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        self.presets.clear()
        self.navigation_goals = 0
        _d_transition_straight(self.ctx)

        self.assertEqual(self.presets, [])
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        nav_res = navigate(self.ctx, ['D3'])
        self.assertFalse(nav_res)
        self.assertEqual(self.navigation_goals, 0)
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

    def test_d_transition_straight_succeeds_after_safe_home(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        self.ctx.start_observation('right')
        self.ctx.finish_observation('right')

        home_res = self.ctx.execute_arm_preset('right_to_home_d')
        self.assertTrue(home_res.completed)
        self.assertTrue(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        self.presets.clear()
        self.navigation_goals = 0
        _d_transition_straight(self.ctx)

        self.assertEqual(self.presets, ['shenzhi'])
        self.assertTrue(self.ctx._arm_shenzhi_confirmed)
        self.assertFalse(self.ctx._arm_home_confirmed)

        nav_res = navigate(self.ctx, ['D1_to_D2_waypoint_1'])
        self.assertTrue(nav_res)
        self.assertEqual(self.navigation_goals, 1)

    def test_d_transition_recovers_to_side_home_on_straight_failure(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        self.ctx.start_observation('right')
        self.ctx.finish_observation('right')

        home_res = self.ctx.execute_arm_preset('right_to_home_d')
        self.assertTrue(home_res.completed)
        self.assertTrue(self.ctx._arm_home_confirmed)

        self.failed_presets.add('shenzhi')
        self.presets.clear()
        self.navigation_goals = 0

        _d_transition_straight(self.ctx)

        self.assertEqual(self.presets, ['shenzhi', 'right_to_home_d'])
        self.assertTrue(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        nav_res = navigate(self.ctx, ['D1_to_D2_waypoint_1'])
        self.assertTrue(nav_res)
        self.assertEqual(self.navigation_goals, 1)

    def test_d_transition_straight_and_recovery_both_fail(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        self.ctx.start_observation('left')
        self.ctx.finish_observation('left')

        home_res = self.ctx.execute_arm_preset('left_to_home_d')
        self.assertTrue(home_res.completed)
        self.assertTrue(self.ctx._arm_home_confirmed)

        self.failed_presets.update({'shenzhi', 'left_to_home_d'})
        self.presets.clear()
        self.navigation_goals = 0

        _d_transition_straight(self.ctx)

        self.assertEqual(self.presets, ['shenzhi', 'left_to_home_d'])
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        # Subsequent transition should NOT attempt shenzhi or home
        self.presets.clear()
        _d_transition_straight(self.ctx)
        self.assertEqual(self.presets, [])
        self.assertFalse(self.ctx._arm_home_confirmed)
        self.assertFalse(self.ctx._arm_shenzhi_confirmed)

        # Subsequent navigation is blocked
        nav_res = navigate(self.ctx, ['D3'])
        self.assertFalse(nav_res)
        self.assertEqual(self.navigation_goals, 0)

    def test_d_transition_noop_when_already_straight(self):
        self.assertTrue(self.ctx.report_current_area('D'))
        straight_res = self.ctx.execute_arm_preset('shenzhi')
        self.assertTrue(straight_res.completed)
        self.assertTrue(self.ctx._arm_shenzhi_confirmed)

        self.presets.clear()
        _d_transition_straight(self.ctx)
        self.assertEqual(self.presets, [])
        self.assertTrue(self.ctx._arm_shenzhi_confirmed)


if __name__ == '__main__':
    unittest.main()
