"""Manual single-point client for camera recognition and arm target grasping.

Also executes store release and home recovery.
"""

import argparse
import json
import math
import time
import uuid

from geometry_msgs.msg import PointStamped
import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String
from fruit_perception.msg import FruitDecision

from fruit_mission.camera_target_planner_client import (
    CameraTargetPlannerClient,
    DEFAULT_EXTRINSIC_CONFIG,
    store_preset_name,
)
from fruit_mission.config import load_target_offsets
from fruit_mission.script_hardware_client import latched_qos, reliable_qos, set_perception_zone


def _bounded_timeout(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
        raise ValueError('Timeout must be finite and positive')
    return min(10.0, float(value))


class CameraArmTargetClient(Node):
    """Debug client for camera recognition, arm grasping, store release, and home recovery."""

    def __init__(self, debug=False):
        super().__init__('camera_arm_target_client')
        self.debug = debug
        self.declare_parameter(
            'config_path',
            '/home/rc4/fruit_ws/src/fruit_mission/config/fruit_mission.yaml',
        )
        self.declare_parameter('camera_extrinsic_config', DEFAULT_EXTRINSIC_CONFIG)
        self.declare_parameter('observation_window_topic', '/fruit_mission/observation_window')
        self.declare_parameter('decision_topic', '/fruit_perception/decision')
        self.declare_parameter('bridge_ready_topic', '/auto_serial_bridge/ready')
        self.declare_parameter('detection_timeout_sec', 10.0)
        self.declare_parameter('endpoint_timeout_sec', 10.0)
        self.declare_parameter('perception_zone_delivery_timeout_sec', 1.0)
        for name in ('detection_timeout_sec',
                     'endpoint_timeout_sec', 'perception_zone_delivery_timeout_sec'):
            _bounded_timeout(self.get_parameter(name).value)

        self._active_window_id = None
        self._active_window_start_sec = 0
        self._active_window_start_nanosec = 0
        self._active_window_area = 'A'
        self._active_window_purpose = 'harvest'
        self._active_window_expected_type = ''
        self._bridge_ready = False
        self._camera_planner = None
        self._latest_decision = None

        self._window_pub = self.create_publisher(
            String, self.get_parameter('observation_window_topic').value, reliable_qos(10))
        self._perception_zone_client = self.create_client(
            SetParametersAtomically, '/fruit_perception_node/set_parameters_atomically')

        self.create_subscription(
            Bool, self.get_parameter('bridge_ready_topic').value,
            self._on_ready, latched_qos())
        self.create_subscription(
            FruitDecision, self.get_parameter('decision_topic').value,
            self._on_decision, 100)

    def _on_ready(self, message):
        self._bridge_ready = message.data

    def _on_decision(self, message: FruitDecision):
        if (
            self._active_window_id is None
            or message.observation_window_id != self._active_window_id
        ):
            return
        stamp = message.header.stamp
        if (
            stamp.sec < 0
            or not 0 <= stamp.nanosec < 1_000_000_000
            or stamp.sec < self._active_window_start_sec
            or (
                stamp.sec == self._active_window_start_sec
                and stamp.nanosec < self._active_window_start_nanosec
            )
        ):
            return
        self._latest_decision = message
        if self.debug:
            self._debug_decision(message)

    def _debug_decision(self, message: FruitDecision):
        stamp = message.header.stamp
        items = [(item.type, item.confidence,
                  (item.point.x, item.point.y, item.point.z) if item.has_point else None)
                 for item in message.detections]
        self.get_logger().debug(
            f'[CAMERA DEBUG] stamp={stamp.sec}.{stamp.nanosec:09d} '
            f'frame={message.header.frame_id} detections={items}')

    def wait_until(self, predicate, timeout=None):
        deadline = None if timeout is None else time.monotonic() + _bounded_timeout(timeout)
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return predicate()

    def stack_is_absent(self):
        self.wait_until(lambda: False, 1.0)
        conflicts = {'fruit_perception_container',
                     'arm_planner_node', 'fruit_mission'}
        return not conflicts.intersection(
            name for name, _ns in self.get_node_names_and_namespaces())

    def open_observation_window(self, zone: str) -> str:
        """Open an observation window for target acquisition."""
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        window_id = str(uuid.uuid4())
        self._active_window_id = window_id
        self._active_window_start_sec = sec
        self._active_window_start_nanosec = nanosec
        self._active_window_area = zone
        self._active_window_purpose = "harvest"
        self._active_window_expected_type = "ripe" if zone == "A" else ""
        self._latest_decision = None
        cmd = {
            "window_id": window_id,
            "valid": 1,
            "start_sec": sec,
            "start_nanosec": nanosec,
            "area": zone,
            "purpose": "harvest",
            "expected_type": self._active_window_expected_type,
        }
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
        self._window_pub.publish(msg)
        self.get_logger().info(f"Opened observation window for zone {zone}")
        self.get_logger().debug(f"Observation window opened: id='{window_id}' zone={zone}")
        return window_id

    def close_observation_window(self) -> None:
        """Close the active observation window if any."""
        if self._active_window_id is None:
            return
        window_id = self._active_window_id
        self._active_window_id = None
        cmd = {
            "window_id": window_id,
            "valid": 0,
            "start_sec": self._active_window_start_sec,
            "start_nanosec": self._active_window_start_nanosec,
            "area": self._active_window_area,
            "purpose": self._active_window_purpose,
            "expected_type": self._active_window_expected_type,
        }
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
        self._window_pub.publish(msg)
        self.get_logger().info('Closed observation window')
        self.get_logger().debug(f"Observation window closed: id='{window_id}'")

    def report_current_area(self, zone):
        """Require acknowledgment of the perception zone parameter."""
        return set_perception_zone(
            self, self._perception_zone_client, zone, self.wait_until,
            _bounded_timeout(self.get_parameter('endpoint_timeout_sec').value),
            _bounded_timeout(self.get_parameter('perception_zone_delivery_timeout_sec').value))

    def _wait_for_target(self, zone: str, window_id: str, timeout: float = 10.0):
        deadline = time.monotonic() + _bounded_timeout(timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._active_window_id != window_id:
                break
            msg = self._latest_decision
            if msg is None:
                continue

            if zone in ('A', 'C'):
                if msg.has_target:
                    pt = msg.target_point
                    if (math.isfinite(pt.x) and math.isfinite(pt.y)
                            and math.isfinite(pt.z) and pt.z > 0):
                        target = PointStamped()
                        target.header = msg.header
                        target.point = pt
                        self.get_logger().info(
                            f'Zone {zone} (vegetable) target locked: '
                            f'x={pt.x:.3f}, y={pt.y:.3f}, z={pt.z:.3f}')
                        return target
            elif zone == 'B':
                # Model B (fruit)
                ripe_targets = [
                    t for t in msg.targets
                    if t.type.endswith('_ripe') and math.isfinite(t.confidence)
                    and t.has_point and math.isfinite(t.point.x) and math.isfinite(t.point.y)
                    and math.isfinite(t.point.z) and t.point.z > 0
                ]
                if ripe_targets and msg.targets_ready:
                    best = max(ripe_targets, key=lambda t: t.confidence)
                    target = PointStamped()
                    target.header = msg.header
                    target.point = best.point
                    self.get_logger().info(
                        f'Model B (fruit) target selected: type={best.type}, '
                        f'conf={best.confidence:.3f}, '
                        f'point=({best.point.x:.3f}, {best.point.y:.3f}, {best.point.z:.3f})')
                    return target
            elif zone == 'D':
                # Model D (fruit)
                ripe_targets = [
                    t for t in msg.targets
                    if t.type.endswith('_ripe') and math.isfinite(t.confidence)
                    and t.has_point and math.isfinite(t.point.x) and math.isfinite(t.point.y)
                    and math.isfinite(t.point.z) and t.point.z > 0
                ]
                if ripe_targets and msg.targets_ready:
                    best = max(ripe_targets, key=lambda t: t.confidence)
                    target = PointStamped()
                    target.header = msg.header
                    target.point = best.point
                    self.get_logger().info(
                        f'Zone D (fruit) target selected: type={best.type}, '
                        f'conf={best.confidence:.3f}, '
                        f'point=({best.point.x:.3f}, {best.point.y:.3f}, {best.point.z:.3f})')
                    return target
        return None

    def safe_recover_home(self) -> bool:
        """安全恢复：下发 STOP，等待停稳空闲，按 zero -> home 顺序安全归位。"""
        if self._camera_planner is None:
            return False
        self.get_logger().warn('触发安全恢复流程：下发 STOP 指令并等待机械臂停稳...')
        self._camera_planner.request_stop()
        stopped_after = time.monotonic()
        stationary_rps = 0.05

        # 1. 确认停稳并且回到 IDLE
        if not self.wait_until(
            lambda: self._camera_planner.is_stationary(
                stationary_rps, stopped_after, require_idle=True),
            timeout=5.0
        ):
            self.get_logger().error('机械臂未确认停稳或未恢复空闲 (IDLE)，中止自动归位以策安全')
            return False

        # 2. 执行 zero 预设 (大臂朝天垂直抬起)
        self.get_logger().info('安全复位第 1 步：执行 zero 预设 (垂直抬起)...')
        zero_completed, zero_reason = self._camera_planner.execute_preset('zero', self.wait_until)
        if not zero_completed:
            self.get_logger().error(f'安全复位 zero 预设执行失败: {zero_reason}，跳过后续 home 复位')
            return False

        # 3. 执行 home 预设 (收纳归位)
        self.get_logger().info('安全复位第 2 步：执行 home 预设 (收纳复位)...')
        home_completed, home_reason = self._camera_planner.execute_preset('home', self.wait_until)
        if not home_completed:
            self.get_logger().error(f'安全复位 home 预设执行失败: {home_reason}')
            return False

        self.get_logger().info('安全恢复完成：机械臂已按 zero -> home 顺序安全复位')
        return True

    def run_sequence(self, zone: str, direction: str) -> int:
        if zone not in ('A', 'B', 'C', 'D') or direction not in ('left', 'right'):
            raise ValueError('Expected zone A/B/C/D and direction left/right')
        look_preset = f'{direction}_look_{zone.lower()}'
        model_name = 'fruit' if zone in ('B', 'D') else 'vegetable'
        self.get_logger().info(
            f'=== Starting Camera-Arm Target Test ===\n'
            f'  Zone: {zone} ({model_name})\n'
            f'  Preset: {look_preset}')

        # 1. Wait for hardware bridge
        self.get_logger().info('Waiting for auto_serial_bridge ready...')
        if not self.wait_until(lambda: self._bridge_ready, 10.0):
            self.get_logger().error('Serial bridge not ready within 10s')
            return 1
        self.get_logger().info('Serial bridge is ready')

        # 2. Initialize CameraTargetPlannerClient
        extrinsic_cfg = self.get_parameter('camera_extrinsic_config').value
        config_path = self.get_parameter('config_path').value
        target_offset_ac_m, target_offset_b_up_m, target_offset_b_down_m = (
            load_target_offsets(config_path))
        self.get_logger().debug(
            f'Loaded target_offset_ac_m={target_offset_ac_m}, '
            f'target_offset_b_up_m={target_offset_b_up_m}, '
            f'target_offset_b_down_m={target_offset_b_down_m} from {config_path}')
        self._camera_planner = CameraTargetPlannerClient(
            self,
            extrinsic_cfg,
            target_offset_ac_m=target_offset_ac_m,
            target_offset_b_up_m=target_offset_b_up_m,
            target_offset_b_down_m=target_offset_b_down_m,
        )

        # 3. Set the perception zone parameter before observing
        if not self.report_current_area(zone):
            return 1
        self.get_logger().info(f'Confirmed perception zone: {zone}')

        # 4. Move arm to home preset first
        self.get_logger().info('Moving arm to home preset...')
        completed, reason = self._camera_planner.execute_preset('home', self.wait_until)
        if not completed:
            self.get_logger().error(f'执行初始预设 home 失败: {reason}')
            self.safe_recover_home()
            return 1
        self.get_logger().info('Arm reached home position')

        # 5. Move arm to look preset
        self.get_logger().info(f'Moving arm to {look_preset}...')
        completed, reason = self._camera_planner.execute_preset(look_preset, self.wait_until)
        if not completed:
            self.get_logger().error(f'执行观察预设 {look_preset} 失败: {reason}')
            self.safe_recover_home()
            return 1
        self.get_logger().info(f'Arm reached {look_preset}')

        # 6. OPEN/HOLD acquires fresh stationary feedback at each measured pose.
        stationary_rps = 0.05
        self.get_logger().info('Opening gripper at the measured observation pose...')
        completed, reason = self._camera_planner.open_gripper(self.wait_until, stationary_rps)
        if not completed:
            self.get_logger().error(f'张开夹爪动作失败: {reason}')
            self.safe_recover_home()
            return 1

        # 7. Open observation window and collect target
        self.get_logger().info('Opening observation window for target acquisition...')
        window_id = self.open_observation_window(zone)
        timeout_sec = _bounded_timeout(self.get_parameter('detection_timeout_sec').value)
        self.get_logger().info(f'Observing for targets (timeout={timeout_sec}s)...')
        try:
            target_point = self._wait_for_target(zone, window_id, timeout=timeout_sec)
        finally:
            self.close_observation_window()

        if target_point is None:
            self.get_logger().error('未在超时时间内采集到有效目标 (No target acquired within timeout)')
            self.safe_recover_home()
            return 1

        # 8. Transform optical target to arm base coordinates
        self.get_logger().debug('Transforming target to arm base coordinates...')
        try:
            req, layer = self._camera_planner.target_request(target_point, zone)
        except ValueError as err:
            self.get_logger().error(f'目标坐标转换失败 (Target transform failed): {err}')
            self.safe_recover_home()
            return 1

        # 11. Move arm to target point and grasp (CLOSE + dwell HOLD)
        self.get_logger().info(
            f'Moving arm to target: x={req.x:.3f}m, y={req.y:.3f}m, z={req.z:.3f}m, '
            f'pitch={req.pitch_deg:.1f}deg with gripper CLOSE...')
        completed, reason = self._camera_planner.move_to_base_target(
            req, self.wait_until, stationary_rps)
        if not completed:
            self.get_logger().error(f'抓取动作执行失败 (Grasp motion failed): {reason}')
            self.safe_recover_home()
            return 1

        # 12. Move arm to store preset and release fruit
        store_preset = store_preset_name(direction, zone, layer)
        self.get_logger().info(f'Moving arm to {store_preset} and releasing fruit...')
        completed, reason = self._camera_planner.execute_preset(store_preset, self.wait_until)
        if not completed:
            self.get_logger().error(
                f'放果预设执行失败 (Failed to execute preset {store_preset}): {reason}')
            self.safe_recover_home()
            return 1

        if zone == 'D':
            to_home_preset = f'{direction}_to_home_d'
            self.get_logger().info(f'Moving arm via {to_home_preset}...')
            completed, reason = self._camera_planner.execute_preset(
                to_home_preset, self.wait_until)
            if not completed:
                self.get_logger().error(
                    f'过渡预设执行失败 (Failed to execute preset {to_home_preset}): {reason}')
                self.safe_recover_home()
                return 1

        self.get_logger().info(
            f'Grasp and {store_preset} sequence completed successfully!')
        return 0

    def destroy_node(self):
        self.close_observation_window()
        if self._camera_planner is not None:
            self._camera_planner.request_stop()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Camera and arm coordination debug client.')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--require-stack-absent', action='store_true',
                      help='Check if conflicting nodes are absent (serial bridge may exist)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable per-frame detection debug logging')
    parser.add_argument('--zone', '-z', type=str.upper, choices=('A', 'B', 'C', 'D'),
                        help='Competition zone: A/C use vegetable model, B/D use fruit model')
    direction = parser.add_mutually_exclusive_group()
    direction.add_argument('--left', dest='direction', action='store_const', const='left',
                           help='Use the zone-specific left_look and left_store presets')
    direction.add_argument('--right', dest='direction', action='store_const', const='right',
                           help='Use the zone-specific right_look and right_store presets')
    options, ros_args = parser.parse_known_args(args)
    unknown = remove_ros_args(args=ros_args)
    if unknown:
        parser.error(f'unrecognized arguments: {" ".join(unknown)}')
    if not options.require_stack_absent:
        if options.zone is None or options.direction is None:
            parser.error('--zone A|B|C|D and --left/--right are required')

    rclpy.init(args=ros_args)
    node = CameraArmTargetClient(debug=options.debug)
    status = 1
    try:
        if options.require_stack_absent:
            status = 0 if node.stack_is_absent() else 9
        else:
            status = node.run_sequence(options.zone, options.direction)
    except KeyboardInterrupt:
        if node._camera_planner is not None:
            node._camera_planner.request_stop()
            node.wait_until(lambda: False, 0.2)
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)
