"""Typed ROS client for the standalone D435i arm-target workflow."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from std_msgs.msg import String

from fruit_hardware_interfaces.msg import ArmTarget
from fruit_hardware_interfaces.msg import ExecutionCallback
from fruit_hardware_interfaces.msg import StateMachineCommand
from fruit_perception.msg import FruitDetections

from fruit_mission.script_hardware_client import CommandResult
from fruit_mission.script_hardware_client import HardwareCommandClient
from fruit_mission.script_hardware_client import latched_qos
from fruit_mission.script_hardware_client import reliable_qos


EXIT_MODE_TIMEOUT = 7
EXIT_TARGET_TIMEOUT = 8
EXIT_STACK_ALREADY_RUNNING = 9


class D435iArmTargetClient(HardwareCommandClient):
    """Run the current-area, perception, sampling, and arm command sequence."""

    def __init__(self) -> None:
        super().__init__("d435i_arm_target_client")
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
        self.declare_parameter(
            "d435i_detections_topic",
            "/fruit_perception/d435i/detections",
        )
        self.declare_parameter("active_mode_timeout_sec", 30.0)
        self.declare_parameter("detection_timeout_sec", 120.0)
        self.declare_parameter("d435i_target_sample_count", 5)
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
        self.detections_topic = self._string_parameter(
            "d435i_detections_topic"
        )
        self.active_mode_timeout_sec = self._float_parameter(
            "active_mode_timeout_sec"
        )
        self.detection_timeout_sec = self._float_parameter(
            "detection_timeout_sec"
        )
        self.sample_count = int(
            self.get_parameter("d435i_target_sample_count").value
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
        self._latest_detections: FruitDetections | None = None
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
        self._detections_subscription = self.create_subscription(
            FruitDetections,
            self.detections_topic,
            self._on_detections,
            reliable_qos(),
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

    def run_sequence(self) -> int:
        """Run the original D435i target command sequence once."""
        if not self.wait_for_stack():
            return int(CommandResult.BRIDGE_NOT_READY)

        self.get_logger().info(
            "Reporting current area A to the lower controller"
        )
        area_command = StateMachineCommand()
        area_command.task_id = StateMachineCommand.TASK_CURRENT_AREA
        area_command.task_status = StateMachineCommand.AREA_A
        area_result = self.publish_and_wait(
            self._state_command_publisher,
            self.state_command_topic,
            area_command,
            ExecutionCallback.CALLBACK_CURRENT_AREA,
            "current area A",
        )
        if area_result != CommandResult.COMPLETED:
            return int(area_result)
        if not self._announce_current_area("A"):
            return int(CommandResult.ENDPOINTS_NOT_READY)

        self.get_logger().info(
            "Assuming the robot is at the observation position; "
            "switching to TREE mode"
        )
        if not self.set_perception_mode("TREE", self.active_mode_timeout_sec):
            self.get_logger().error("Perception stack did not enter TREE mode")
            return EXIT_MODE_TIMEOUT

        fresh_after_ns = self.get_clock().now().nanoseconds
        target = self.sample_target(fresh_after_ns)
        if target is None:
            self.get_logger().error(
                f"Fewer than {self.sample_count} valid D435i "
                "detection samples "
                f"within {self.detection_timeout_sec:.0f}s"
            )
            return EXIT_TARGET_TIMEOUT
        target_type, target_x, target_y, target_z = target
        self.get_logger().info(
            f"Averaged {self.sample_count} centered D435i samples: "
            f"{target_type} x={target_x}, y={target_y}, z={target_z}"
        )

        self.get_logger().info("Sending the D435i target to the arm")
        message = ArmTarget()
        message.target_x = target_x
        message.target_y = target_y
        message.target_z = target_z
        message.z_type = ArmTarget.Z_GROUND
        result = self.publish_and_wait(
            self._arm_target_publisher,
            self.arm_target_topic,
            message,
            ExecutionCallback.CALLBACK_ARM_TARGET,
            "arm target",
        )
        return int(result)

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

    def sample_target(
        self, fresh_after_ns: int
    ) -> tuple[str, float, float, float] | None:
        """Average five fresh centered typed D435i detections."""
        self._latest_detections = None
        samples: list[tuple[float, float, float]] = []
        latest_type = ""
        deadline = time.monotonic() + self.detection_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            message = self._latest_detections
            self._latest_detections = None
            if message is None or self._stamp_ns(message) <= fresh_after_ns:
                continue
            valid = [item for item in message.detections if self._valid(item)]
            if not valid:
                continue
            selected = min(
                valid,
                key=lambda item: (
                    item.point.x * item.point.x
                    + item.point.y * item.point.y,
                    -float(item.confidence),
                ),
            )
            latest_type = selected.type
            samples.append(
                (
                    float(selected.point.x),
                    float(selected.point.y),
                    float(selected.point.z),
                )
            )
            if len(samples) >= self.sample_count:
                return (
                    latest_type,
                    sum(point[0] for point in samples) / self.sample_count,
                    sum(point[1] for point in samples) / self.sample_count,
                    sum(point[2] for point in samples) / self.sample_count,
                )
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

    def _on_detections(self, message: FruitDetections) -> None:
        self._latest_detections = message

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _stamp_ns(message: FruitDetections) -> int:
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _valid(detection: object) -> bool:
        return bool(
            detection.has_point
            and detection.type
            and math.isfinite(float(detection.point.x))
            and math.isfinite(float(detection.point.y))
            and math.isfinite(float(detection.point.z))
        )

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
    options, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = D435iArmTargetClient()
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
            status = node.run_sequence()
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)


if __name__ == "__main__":
    main()
