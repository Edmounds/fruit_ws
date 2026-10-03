"""Manual single-point client sharing the actual mission grasp implementation."""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from fruit_perception.msg import FruitDecision

from fruit_mission.config import load_config
from fruit_mission.decisions import decide_b_tree_targets, spoken_produce_name
from fruit_mission.mission_context import MissionContext
from fruit_mission.mission_node import DEFAULT_CONFIG_PATH
from fruit_mission.script_hardware_client import latched_qos, reliable_qos


class CameraArmTargetClient(Node):
    """Lifecycle probes do not instantiate a hardware-control context."""

    def __init__(self, debug=False):
        super().__init__('camera_arm_target_client')
        self.debug = debug
        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
        self.declare_parameter('perception_mode_topic', '/fruit_mission/perception_mode')
        self.declare_parameter('active_mode_topic', '/fruit_perception/active_mode')
        self.declare_parameter('bridge_ready_topic', '/auto_serial_bridge/ready')
        self.declare_parameter('cleanup_mode_timeout_sec', 2.0)
        self._mode = ''
        self._generation = 0
        self._bridge_ready = False
        self._ctx = None
        self._mode_pub = self.create_publisher(
            String, self.get_parameter('perception_mode_topic').value, reliable_qos(1))
        self.create_subscription(
            String, self.get_parameter('active_mode_topic').value, self._on_mode, latched_qos())
        self.create_subscription(
            Bool, self.get_parameter('bridge_ready_topic').value,
            self._on_ready, latched_qos())

    def _on_mode(self, message):
        self._mode = message.data
        self._generation += 1

    def _on_ready(self, message):
        self._bridge_ready = message.data

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return predicate()

    def announce(self, text, **_kwargs):
        self.get_logger().info(text)
        return True

    def stack_is_absent(self):
        self.wait_until(lambda: False, 1.0)
        conflicts = {'fruit_perception_container', 'camera_mode_manager',
                     'serial_controller', 'arm_planner_node', 'fruit_mission'}
        return not conflicts.intersection(
            name for name, _ns in self.get_node_names_and_namespaces())

    def set_off_for_cleanup(self):
        previous = self._generation
        self._mode_pub.publish(String(data='OFF'))
        return self.wait_until(
            lambda: self._generation > previous and self._mode == 'OFF',
            min(10.0, self.get_parameter('cleanup_mode_timeout_sec').value))

    def run_sequence(self, area, direction, b_fruit=None):
        if not self.wait_until(lambda: self._bridge_ready, 10.0):
            return 1
        config = load_config(self.get_parameter('config_path').value)
        self._ctx = MissionContext(self, config, speaker=self)
        if self.debug:
            self.create_subscription(
                FruitDecision, config['decision_topic'], self._debug_decision, 100)
        ctx = self._ctx
        ctx.report_current_area(area)
        if not ctx.recover_home():
            return 1
        if area != 'B':
            result = ctx.execute_closed_loop_pick(direction, 'ripe' if area == 'A' else None)
            self.get_logger().info(f'Single-point result: {result.reason_code}')
            return 0 if result.success else 1
        observation = ctx.observe_b_tree(direction)
        if not observation['success']:
            return 1
        targets = observation['targets']
        expected = f'{b_fruit}_ripe'
        decision = decide_b_tree_targets(
            spoken_produce_name(expected), expected,
            ((target.model_type, target.pixel_x, target.pixel_y, target.image_width)
             for target in targets))
        requests = ctx.freeze_b_targets([targets[index] for index in decision.target_indices])
        if not requests:
            ctx.recover_home()
        success = True
        for request in requests:
            result = ctx.execute_b_tree_target(request, direction)
            success = success and result['success']
            if not result['safe_to_continue']:
                break
        return 0 if success else 1

    def _debug_decision(self, message):
        stamp = message.header.stamp
        items = [(item.type, item.confidence,
                  (item.point.x, item.point.y, item.point.z) if item.has_point else None)
                 for item in message.detections]
        self.get_logger().info(
            f'[CAMERA DEBUG] stamp={stamp.sec}.{stamp.nanosec:09d} '
            f'frame={message.header.frame_id} detections={items}')

    def destroy_node(self):
        if self._ctx is not None:
            self._ctx.close()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--require-stack-absent', action='store_true')
    mode.add_argument('--mode-off', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--zone', choices=('A', 'B', 'C'))
    direction = parser.add_mutually_exclusive_group()
    direction.add_argument('--left', dest='direction', action='store_const', const='left')
    direction.add_argument('--right', dest='direction', action='store_const', const='right')
    fruit = parser.add_mutually_exclusive_group()
    fruit.add_argument('--apple', dest='b_fruit', action='store_const', const='apple')
    fruit.add_argument('--pear', dest='b_fruit', action='store_const', const='pear')
    options, ros_args = parser.parse_known_args(args)
    if not (options.require_stack_absent or options.mode_off):
        if options.zone is None or options.direction is None:
            parser.error('--zone and --left/--right are required')
        if (options.zone == 'B') != (options.b_fruit is not None):
            parser.error('--apple/--pear must be specified for B only')
    rclpy.init(args=ros_args)
    node = CameraArmTargetClient(debug=options.debug)
    status = 1
    try:
        if options.require_stack_absent:
            status = 0 if node.stack_is_absent() else 9
        elif options.mode_off:
            status = 0 if node.set_off_for_cleanup() else 7
        else:
            status = node.run_sequence(options.zone, options.direction, options.b_fruit)
    except KeyboardInterrupt:
        if node._ctx is not None:
            node._ctx._camera_planner.request_stop()
            node.wait_until(lambda: False, 0.2)
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)
