"""Typed ROS client for the manual current-area command."""

from __future__ import annotations

import argparse

import rclpy
from std_msgs.msg import String

from fruit_hardware_interfaces.msg import ExecutionCallback
from fruit_hardware_interfaces.msg import StateMachineCommand

from fruit_mission.script_hardware_client import CommandResult
from fruit_mission.script_hardware_client import HardwareCommandClient
from fruit_mission.script_hardware_client import latched_qos
from fruit_mission.script_hardware_client import reliable_qos


EXIT_BRIDGE_ABSENT = 10

AREA_STATUS = {
    "A": StateMachineCommand.AREA_A,
    "B": StateMachineCommand.AREA_B,
    "C": StateMachineCommand.AREA_C,
    "D": StateMachineCommand.AREA_D,
}


class CurrentAreaClient(HardwareCommandClient):
    """Send one area command through the serial bridge and announce it."""

    def __init__(self) -> None:
        super().__init__("current_area_client")
        self.declare_parameter(
            "state_command_topic",
            "/fruit_hardware_bridge/state_machine/command",
        )
        self.declare_parameter(
            "current_area_topic", "/fruit_mission/current_area"
        )
        self.declare_parameter("current_area_delivery_timeout_sec", 1.0)
        self.declare_parameter("graph_discovery_timeout_sec", 1.0)

        self.state_command_topic = str(
            self.get_parameter("state_command_topic").value
        )
        self.current_area_topic = str(
            self.get_parameter("current_area_topic").value
        )
        self.delivery_timeout_sec = float(
            self.get_parameter("current_area_delivery_timeout_sec").value
        )
        self.graph_discovery_timeout_sec = float(
            self.get_parameter("graph_discovery_timeout_sec").value
        )

        self._command_publisher = self.create_publisher(
            StateMachineCommand,
            self.state_command_topic,
            reliable_qos(),
        )
        self._area_publisher = self.create_publisher(
            String,
            self.current_area_topic,
            latched_qos(),
        )

    def bridge_exists(self) -> bool:
        """Wait for graph discovery, then report bridge publisher presence."""
        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
        return self.bridge_publisher_exists()

    def send_area(self, area: str) -> int:
        """Send an area command and publish the matching ROS state."""
        command = StateMachineCommand()
        command.task_id = StateMachineCommand.TASK_CURRENT_AREA
        command.task_status = AREA_STATUS[area]
        self.get_logger().info(
            f"Sending current area {area} through the ROS serial bridge"
        )
        result = self.publish_and_wait(
            self._command_publisher,
            self.state_command_topic,
            command,
            ExecutionCallback.CALLBACK_CURRENT_AREA,
            f"current area {area}",
        )
        if result != CommandResult.COMPLETED:
            return int(result)

        message = String()
        message.data = area
        self._area_publisher.publish(message)
        self.wait_until(lambda: False, self.delivery_timeout_sec)
        self.get_logger().info(
            f"Published {area} to {self.current_area_topic}"
        )
        return 0


def main(args=None) -> None:
    """Probe bridge presence or send one typed current-area command."""
    parser = argparse.ArgumentParser()
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--probe-bridge", action="store_true")
    action.add_argument("--area", choices=tuple(AREA_STATUS))
    options, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = CurrentAreaClient()
    status = 1
    try:
        if options.probe_bridge:
            status = 0 if node.bridge_exists() else EXIT_BRIDGE_ABSENT
        else:
            status = node.send_area(options.area)
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)


if __name__ == "__main__":
    main()
