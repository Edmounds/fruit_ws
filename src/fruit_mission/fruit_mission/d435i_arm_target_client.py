"""Typed ROS client for the standalone D435i arm-target workflow."""

from __future__ import annotations

import argparse
import math
import time

from geometry_msgs.msg import PointStamped
import rclpy
from std_msgs.msg import String

from fruit_hardware_interfaces.msg import ArmTarget
from fruit_hardware_interfaces.msg import ExecutionCallback
from fruit_hardware_interfaces.msg import StateMachineCommand
from fruit_perception.msg import FruitDecision

from fruit_mission.script_hardware_client import CommandResult
from fruit_mission.script_hardware_client import HardwareCommandClient
from fruit_mission.script_hardware_client import latched_qos
from fruit_mission.script_hardware_client import reliable_qos
EXIT_MODE_TIMEOUT = 7
EXIT_TARGET_TIMEOUT = 8
EXIT_STACK_ALREADY_RUNNING = 9


class D435iArmTargetClient(HardwareCommandClient):
    """Run the current-area, perception, sampling, and arm command sequence."""

    def __init__(self, debug: bool = False) -> None:
        super().__init__("d435i_arm_target_client")
        self.debug = debug
        self.declare_parameter(
            "state_command_topic",
            "/fruit_hardware_bridge/state_machine/command",
        )
        self.declare_parameter(
            "arm_target_topic", "/fruit_hardware_bridge/arm/target"
        )
        self.declare_parameter(
            "perception_mode_topic", "/fruit_mission/perception_mode"
        )
        self.declare_parameter(
            "active_mode_topic", "/fruit_perception/active_mode"
        )
        self.declare_parameter(
            "current_area_topic", "/fruit_mission/current_area"
        )
        self.declare_parameter("decision_topic", "/fruit_perception/decision")
        self.declare_parameter("active_mode_timeout_sec", 30.0)
        self.declare_parameter("detection_timeout_sec", 120.0)
        self.declare_parameter("current_area_delivery_timeout_sec", 1.0)
        self.declare_parameter("perception_mode_min_subscribers", 2)
        self.declare_parameter("cleanup_mode_timeout_sec", 2.0)
        self.declare_parameter("graph_discovery_timeout_sec", 1.0)
        self.declare_parameter(
            "d435i_conflicting_nodes",
            [
                "/fruit_perception_container",
                "/camera_mode_manager",
                "/serial_controller",
            ],
        )

        self.state_command_topic = self._string_parameter(
            "state_command_topic"
        )
        self.arm_target_topic = self._string_parameter("arm_target_topic")
        self.perception_mode_topic = self._string_parameter(
            "perception_mode_topic"
        )
        self.active_mode_topic = self._string_parameter("active_mode_topic")
        self.current_area_topic = self._string_parameter("current_area_topic")
        self.decision_topic = self._string_parameter("decision_topic")
        self.active_mode_timeout_sec = self._float_parameter(
            "active_mode_timeout_sec"
        )
        self.detection_timeout_sec = self._float_parameter(
            "detection_timeout_sec"
        )
        self.area_delivery_timeout_sec = self._float_parameter(
            "current_area_delivery_timeout_sec"
        )
        self.mode_min_subscribers = int(
            self.get_parameter("perception_mode_min_subscribers").value
        )
        self.cleanup_mode_timeout_sec = self._float_parameter(
            "cleanup_mode_timeout_sec"
        )
        self.graph_discovery_timeout_sec = self._float_parameter(
            "graph_discovery_timeout_sec"
        )
        self.conflicting_nodes = set(
            self.get_parameter("d435i_conflicting_nodes").value
        )

        self._active_mode = ""
        self._latest_target_point: PointStamped | None = None
        self._latest_b_targets = None
        self._mode_publisher = self.create_publisher(
            String, self.perception_mode_topic, reliable_qos(depth=1)
        )
        self._current_area_publisher = self.create_publisher(
            String, self.current_area_topic, latched_qos()
        )
        self._state_command_publisher = self.create_publisher(
            StateMachineCommand, self.state_command_topic, reliable_qos()
        )
        self._arm_target_publisher = self.create_publisher(
            ArmTarget, self.arm_target_topic, reliable_qos()
        )
        self._active_mode_subscription = self.create_subscription(
            String,
            self.active_mode_topic,
            self._on_active_mode,
            latched_qos(),
        )
        self._decision_subscription = self.create_subscription(
            FruitDecision,
            self.decision_topic,
            self._on_decision,
            reliable_qos(depth=100),
        )
        if self.debug:
            self.get_logger().info(
                "D435i per-frame debug logging enabled for "
                f"{self.decision_topic}"
            )

    def stack_is_absent(self) -> bool:
        """Return whether none of the script-owned stack nodes exists."""
        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
        names = {
            self._fully_qualified_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }
        existing = sorted(names & self.conflicting_nodes)
        if existing:
            self.get_logger().error(
                "Perception or serial nodes are already running: "
                + ", ".join(existing)
            )
            return False
        return True

    def wait_for_stack(self) -> bool:
        """Wait for OFF mode, bridge ready, and the arm ROS endpoints."""
        if not self.wait_for_active_mode("OFF", self.active_mode_timeout_sec):
            self.get_logger().error(
                "Perception stack did not report OFF on "
                f"{self.active_mode_topic}"
            )
            return False
        if not self.wait_for_bridge_ready():
            self.get_logger().error(
                "Serial bridge did not report ready on "
                f"{self.bridge_ready_topic}"
            )
            return False
        if not self.wait_for_command_endpoints(self.arm_target_topic):
            self.get_logger().error(
                "Serial bridge endpoints are not ready for "
                f"{self.arm_target_topic}"
            )
            return False
        return True

    def run_sequence(self, area: str, direction: str) -> int:
        """Set the area and direction, then run the D435i target sequence."""
        if not self.wait_for_stack():
            return int(CommandResult.BRIDGE_NOT_READY)

        self.get_logger().info(
            f"Reporting current area {area} to the lower controller"
        )
        area_command = StateMachineCommand()
        area_command.task_id = StateMachineCommand.TASK_CURRENT_AREA
        area_command.task_status = {
            "A": StateMachineCommand.AREA_A,
            "B": StateMachineCommand.AREA_B,
            "C": StateMachineCommand.AREA_C,
        }[area]
        area_result = self.publish_and_wait(
            self._state_command_publisher,
            self.state_command_topic,
            area_command,
            ExecutionCallback.CALLBACK_CURRENT_AREA,
            f"current area {area}",
        )
        if area_result != CommandResult.COMPLETED:
            return int(area_result)
        if not self._announce_current_area(area):
            return int(CommandResult.ENDPOINTS_NOT_READY)

        self.get_logger().info(
            f"Moving to the {direction} observation position"
        )
        direction_result = self.send_pick_direction(direction)
        if direction_result != CommandResult.COMPLETED:
            return int(direction_result)

        self.get_logger().info(
            "Observation position reached; switching to TREE mode"
        )
        if not self.set_perception_mode("TREE", self.active_mode_timeout_sec):
            self.get_logger().error("Perception stack did not enter TREE mode")
            return EXIT_MODE_TIMEOUT

        fresh_after_ns = self.get_clock().now().nanoseconds
        if area == "B":
            targets = self.sample_b_targets(fresh_after_ns)
            if not targets:
                self.get_logger().error(
                    "No valid B-zone targets from the completed 50-frame perception window "
                    f"within {self.detection_timeout_sec:.0f}s"
                )
                return EXIT_TARGET_TIMEOUT
            for index, target in enumerate(targets):
                self.get_logger().info(
                    f"B target {index + 1}/{len(targets)} type={target.type!r} "
                    f"confidence={target.confidence:.6f} "
                    f"observations={target.observation_count} "
                    f"inliers={target.inlier_count} "
                    f"point=({target.point.x:.6f},{target.point.y:.6f},"
                    f"{target.point.z:.6f})"
                )
                result = self.send_arm_target(target.point, f"B arm target {index + 1}")
                if result != CommandResult.COMPLETED:
                    return int(result)
                if index + 1 < len(targets):
                    self.get_logger().info(
                        "Re-entering the B observation direction before the next target"
                    )
                    direction_result = self.send_pick_direction(direction)
                    if direction_result != CommandResult.COMPLETED:
                        return int(direction_result)
            return int(CommandResult.COMPLETED)

        target = self.sample_target(fresh_after_ns)
        if target is None:
            self.get_logger().error(
                "No D435i target from a completed perception window "
                f"within {self.detection_timeout_sec:.0f}s"
            )
            return EXIT_TARGET_TIMEOUT
        target_x = float(target.point.x)
        target_y = float(target.point.y)
        target_z = float(target.point.z)
        self.get_logger().info(
            "D435i target selected from the 35-frame perception window: "
            f"x={target_x}, y={target_y}, z={target_z}"
        )

        result = self.send_arm_target(target.point, "arm target")
        return int(result)

    def send_pick_direction(self, direction: str) -> CommandResult:
        """Send a direction pose command and wait for its completion callback."""
        direction_command = StateMachineCommand()
        direction_command.task_id = StateMachineCommand.TASK_PICK_DIRECTION
        direction_command.task_status = (
            StateMachineCommand.PICK_DIRECTION_LEFT
            if direction == "left"
            else StateMachineCommand.PICK_DIRECTION_RIGHT
        )
        return self.publish_and_wait(
            self._state_command_publisher,
            self.state_command_topic,
            direction_command,
            ExecutionCallback.CALLBACK_PICK_DIRECTION,
            f"{direction} observation position",
        )

    def send_arm_target(self, point, label: str) -> CommandResult:
        """Send one point to the arm and wait for the execution callback."""
        self.get_logger().info(f"Sending {label} to the arm")
        message = ArmTarget()
        message.target_x = float(point.x)
        message.target_y = float(point.y)
        message.target_z = float(point.z)
        message.z_type = ArmTarget.Z_GROUND
        return self.publish_and_wait(
            self._arm_target_publisher,
            self.arm_target_topic,
            message,
            ExecutionCallback.CALLBACK_ARM_TARGET,
            label,
        )

    def set_perception_mode(self, mode: str, timeout_sec: float) -> bool:
        """Publish one mode after subscribers exist and await typed state."""
        endpoints_ready = self.wait_until(
            lambda: (
                self.count_subscribers(self.perception_mode_topic)
                >= self.mode_min_subscribers
            ),
            timeout_sec,
        )
        if not endpoints_ready:
            return False
        message = String()
        message.data = mode
        self._mode_publisher.publish(message)
        return self.wait_for_active_mode(mode, timeout_sec)

    def set_off_for_cleanup(self) -> bool:
        """Request OFF during shell cleanup using a bounded typed exchange."""
        return self.set_perception_mode("OFF", self.cleanup_mode_timeout_sec)

    def wait_for_active_mode(self, mode: str, timeout_sec: float) -> bool:
        """Wait for the transient-local active perception mode."""
        return self.wait_until(lambda: self._active_mode == mode, timeout_sec)

    def sample_target(self, fresh_after_ns: int) -> PointStamped | None:
        """Wait for one fresh target already averaged by perception."""
        self._latest_target_point = None
        latest_stamp_ns = fresh_after_ns
        deadline = time.monotonic() + self.detection_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            message = self._latest_target_point
            self._latest_target_point = None
            if message is None:
                continue
            stamp_ns = self._stamp_ns(message)
            if stamp_ns <= latest_stamp_ns:
                continue
            latest_stamp_ns = stamp_ns
            point = (
                float(message.point.x),
                float(message.point.y),
                float(message.point.z),
            )
            if not all(math.isfinite(value) for value in point):
                continue
            return message
        return None

    def sample_b_targets(self, fresh_after_ns: int):
        """Wait for one fresh completed B-zone target window."""
        self._latest_b_targets = None
        deadline = time.monotonic() + self.detection_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            selection = self._latest_b_targets
            self._latest_b_targets = None
            if selection is None:
                continue
            stamp_ns, targets = selection
            if stamp_ns <= fresh_after_ns:
                continue
            valid_targets = []
            for target in targets:
                point = (
                    float(target.point.x),
                    float(target.point.y),
                    float(target.point.z),
                )
                if target.has_point and all(math.isfinite(value) for value in point):
                    valid_targets.append(target)
            return valid_targets
        return None

    def _announce_current_area(self, area: str) -> bool:
        matched = self.wait_until(
            lambda: self.count_subscribers(self.current_area_topic) > 0,
            self.endpoint_timeout_sec,
        )
        if not matched:
            self.get_logger().error(
                f"No subscriber is ready for {self.current_area_topic}"
            )
            return False
        message = String()
        message.data = area
        self._current_area_publisher.publish(message)
        self.wait_until(lambda: False, self.area_delivery_timeout_sec)
        return True

    def _on_active_mode(self, message: String) -> None:
        self._active_mode = message.data

    def _on_decision(self, message: FruitDecision) -> None:
        if message.source != FruitDecision.SOURCE_D435I:
            return
        detections = []
        for index, item in enumerate(message.detections):
            point = (
                f"({item.point.x:.6f},{item.point.y:.6f},"
                f"{item.point.z:.6f})"
                if item.has_point
                else "NONE"
            )
            detections.append(
                f"#{index} type={item.type!r} "
                f"confidence={item.confidence:.6f} "
                f"roi=({item.roi.x_offset},{item.roi.y_offset},"
                f"{item.roi.width},{item.roi.height}) "
                f"has_point={bool(item.has_point)} point={point}"
            )
        if self.debug:
            details = " | ".join(detections) if detections else "<none>"
            self.get_logger().info(
                "[D435I DEBUG] detections "
                f"stamp={self._format_stamp(message)} "
                f"frame={message.header.frame_id!r} "
                f"count={len(message.detections)} {details}"
            )
        if message.targets_ready:
            stamp = message.header.stamp
            self._latest_b_targets = (
                stamp.sec * 1_000_000_000 + stamp.nanosec,
                list(message.targets),
            )
        if not message.has_target:
            return
        target = PointStamped()
        target.header = message.header
        target.point.x = message.target_point.x
        target.point.y = message.target_point.y
        target.point.z = message.target_point.z
        if self.debug:
            self.get_logger().info(
                "[D435I DEBUG] target_point "
                f"stamp={self._format_stamp(message)} "
                f"frame={message.header.frame_id!r} "
                f"point=({target.point.x:.6f},"
                f"{target.point.y:.6f},{target.point.z:.6f})"
            )
        self._latest_target_point = target

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _stamp_ns(message: PointStamped) -> int:
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _format_stamp(message: FruitDecision) -> str:
        stamp = message.header.stamp
        return f"{stamp.sec}.{stamp.nanosec:09d}"

    @staticmethod
    def _fully_qualified_name(name: str, namespace: str) -> str:
        if namespace == "/":
            return f"/{name}"
        return f"{namespace.rstrip('/')}/{name}"


def main(args=None) -> None:
    """Run the typed D435i workflow or one of its lifecycle checks."""
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--require-stack-absent", action="store_true")
    mode.add_argument("--mode-off", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--zone", choices=("A", "B", "C"))
    direction = parser.add_mutually_exclusive_group()
    direction.add_argument(
        "--left", dest="direction", action="store_const", const="left"
    )
    direction.add_argument(
        "--right", dest="direction", action="store_const", const="right"
    )
    options, ros_args = parser.parse_known_args(args)

    if not (options.require_stack_absent or options.mode_off):
        if options.zone is None:
            parser.error("--zone A, --zone B, or --zone C is required")
        if options.direction is None:
            parser.error("--left or --right is required")

    rclpy.init(args=ros_args)
    node = D435iArmTargetClient(debug=options.debug)
    status = 1
    try:
        if options.require_stack_absent:
            status = (
                0
                if node.stack_is_absent()
                else EXIT_STACK_ALREADY_RUNNING
            )
        elif options.mode_off:
            status = 0 if node.set_off_for_cleanup() else EXIT_MODE_TIMEOUT
        else:
            status = node.run_sequence(options.zone, options.direction)
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)


if __name__ == "__main__":
    main()
