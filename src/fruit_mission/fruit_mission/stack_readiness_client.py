"""Typed ROS and graph readiness checks for mission startup."""

from __future__ import annotations

import argparse
import time

from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String

from fruit_mission.script_hardware_client import latched_qos


class StackReadinessClient(Node):
    """Evaluate one startup readiness condition without CLI text parsing."""

    def __init__(self) -> None:
        super().__init__("stack_readiness_client")
        self.declare_parameter(
            "bridge_ready_topic", "/auto_serial_bridge/ready"
        )
        self.declare_parameter(
            "active_mode_topic", "/fruit_perception/active_mode"
        )
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("graph_discovery_timeout_sec", 1.0)

        self.bridge_ready_topic = self._string_parameter(
            "bridge_ready_topic"
        )
        self.active_mode_topic = self._string_parameter("active_mode_topic")
        self.follow_waypoints_action = self._string_parameter(
            "follow_waypoints_action"
        )
        self.graph_discovery_timeout_sec = float(
            self.get_parameter("graph_discovery_timeout_sec").value
        )

        self._bridge_ready = False
        self._active_mode = ""
        self._ready_subscription = self.create_subscription(
            Bool,
            self.bridge_ready_topic,
            self._on_bridge_ready,
            latched_qos(),
        )
        self._active_mode_subscription = self.create_subscription(
            String,
            self.active_mode_topic,
            self._on_active_mode,
            latched_qos(),
        )

    def nodes_are_absent(self, expected_absent: list[str]) -> bool:
        """Return false when any named node already exists."""
        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
        existing = sorted(set(expected_absent) & self.node_names())
        if existing:
            self.get_logger().error(
                "Nodes are already running: " + ", ".join(existing)
            )
            return False
        return True

    def wait_for_nodes(self, required: list[str], timeout_sec: float) -> bool:
        """Wait for every required fully qualified node name."""
        ready = self.wait_until(
            lambda: set(required).issubset(self.node_names()),
            timeout_sec,
        )
        if not ready:
            missing = sorted(set(required) - self.node_names())
            self.get_logger().error(
                "Timed out waiting for nodes: " + ", ".join(missing)
            )
        return ready

    def wait_for_action(self, action_name: str, timeout_sec: float) -> bool:
        """Wait for the requested FollowWaypoints action server."""
        client = ActionClient(
            self,
            FollowWaypoints,
            action_name,
        )
        ready = client.wait_for_server(timeout_sec=timeout_sec)
        if not ready:
            self.get_logger().error(
                f"Timed out waiting for action: {action_name}"
            )
        return ready

    def wait_for_bridge(self, timeout_sec: float) -> bool:
        """Wait for the serial bridge's transient-local ready state."""
        ready = self.wait_until(lambda: self._bridge_ready, timeout_sec)
        if not ready:
            self.get_logger().error(
                "Timed out waiting for ready=true on "
                f"{self.bridge_ready_topic}"
            )
        return ready

    def wait_for_endpoint(
        self,
        topic: str,
        endpoint: str,
        minimum: int,
        timeout_sec: float,
    ) -> bool:
        """Wait for a typed topic's graph endpoint count."""
        counter = (
            self.count_publishers
            if endpoint == "publishers"
            else self.count_subscribers
        )
        ready = self.wait_until(
            lambda: counter(topic) >= minimum, timeout_sec
        )
        if not ready:
            self.get_logger().error(
                f"Timed out waiting for {topic} {endpoint}>={minimum}; "
                f"observed={counter(topic)}"
            )
        return ready

    def wait_for_active_mode(self, mode: str, timeout_sec: float) -> bool:
        """Wait for the typed active perception mode."""
        ready = self.wait_until(lambda: self._active_mode == mode, timeout_sec)
        if not ready:
            self.get_logger().error(
                f"Timed out waiting for {self.active_mode_topic}={mode}; "
                f"observed={self._active_mode or '<none>'}"
            )
        return ready

    def node_names(self) -> set[str]:
        """Return current fully qualified ROS graph node names."""
        return {
            self._fully_qualified_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def wait_until(self, predicate, timeout_sec: float) -> bool:
        """Spin until a condition succeeds or a monotonic timeout expires."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok():
            if predicate():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return predicate()

    def _on_bridge_ready(self, message: Bool) -> None:
        self._bridge_ready = bool(message.data)

    def _on_active_mode(self, message: String) -> None:
        self._active_mode = message.data

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    @staticmethod
    def _fully_qualified_name(name: str, namespace: str) -> str:
        if namespace in ("", "/"):
            return f"/{name}"
        return f"{namespace.rstrip('/')}/{name}"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)

    nodes_absent = commands.add_parser("nodes-absent")
    nodes_absent.add_argument("--node", action="append", required=True)

    nodes = commands.add_parser("nodes")
    nodes.add_argument("--node", action="append", required=True)
    nodes.add_argument("--timeout", type=float, required=True)

    action = commands.add_parser("action")
    action.add_argument("--name")
    action.add_argument("--timeout", type=float, required=True)

    bridge = commands.add_parser("bridge")
    bridge.add_argument("--timeout", type=float, required=True)

    endpoint = commands.add_parser("endpoint")
    endpoint.add_argument("--topic", required=True)
    endpoint.add_argument(
        "--kind", choices=("publishers", "subscribers"), required=True
    )
    endpoint.add_argument("--count", type=int, required=True)
    endpoint.add_argument("--timeout", type=float, required=True)

    active = commands.add_parser("active-mode")
    active.add_argument("--mode", required=True)
    active.add_argument("--timeout", type=float, required=True)
    return parser


def main(args=None) -> None:
    """Run one typed startup-readiness condition."""
    options, ros_args = _parser().parse_known_args(args)
    rclpy.init(args=ros_args)
    node = StackReadinessClient()
    ready = False
    try:
        if options.command == "nodes-absent":
            ready = node.nodes_are_absent(options.node)
        elif options.command == "nodes":
            ready = node.wait_for_nodes(options.node, options.timeout)
        elif options.command == "action":
            action_name = options.name or node.follow_waypoints_action
            ready = node.wait_for_action(action_name, options.timeout)
        elif options.command == "bridge":
            ready = node.wait_for_bridge(options.timeout)
        elif options.command == "endpoint":
            ready = node.wait_for_endpoint(
                options.topic,
                options.kind,
                options.count,
                options.timeout,
            )
        elif options.command == "active-mode":
            ready = node.wait_for_active_mode(options.mode, options.timeout)
    except KeyboardInterrupt:
        ready = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ready else 1)


if __name__ == "__main__":
    main()
