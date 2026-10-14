"""Typed ROS and graph readiness checks for mission startup."""

from __future__ import annotations

import argparse
import math
import time

from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
from fruit_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from fruit_mission.script_hardware_client import latched_qos


class StackReadinessClient(Node):
    """Evaluate one startup readiness condition without CLI text parsing."""

    def __init__(self) -> None:
        super().__init__("stack_readiness_client")
        self.declare_parameter(
            "bridge_ready_topic", "/auto_serial_bridge/ready"
        )
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("graph_discovery_timeout_sec", 1.0)
        self.declare_parameter(
            "perception_heartbeat_topic",
            "/fruit_perception/camera/rgbd_heartbeat",
        )
        self.declare_parameter("perception_readiness_timeout_sec", 30.0)
        self.declare_parameter("perception_stable_duration_sec", 3.0)
        self.declare_parameter("perception_stream_max_gap_sec", 1.0)

        self.bridge_ready_topic = self._string_parameter(
            "bridge_ready_topic"
        )
        self.follow_waypoints_action = self._string_parameter(
            "follow_waypoints_action"
        )
        self.graph_discovery_timeout_sec = float(
            self.get_parameter("graph_discovery_timeout_sec").value
        )
        self.perception_heartbeat_topic = self._string_parameter(
            "perception_heartbeat_topic"
        )
        self.perception_readiness_timeout_sec = float(
            self.get_parameter("perception_readiness_timeout_sec").value
        )

        self._bridge_ready = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        self._ready_subscription = self.create_subscription(
            Bool,
            self.bridge_ready_topic,
            self._on_bridge_ready,
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

    def nodes_are_running(self, candidates: list[str], match: str = "all") -> bool:
        """Return True when candidates are running (no error logging)."""
        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
        active = self.node_names()
        if match == "all":
            return set(candidates).issubset(active)
        return bool(set(candidates) & active)

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

    def wait_for_planner(self, timeout_sec: float) -> bool:
        """Check typed motion/preset/STOP services without commanding the arm."""
        deadline = time.monotonic() + timeout_sec
        for service_type, name in (
            (MoveArmCartesian, "/arm_planner_node/move_cartesian"),
            (MoveArmJoints, "/arm_planner_node/move_joints"),
            (ExecuteArmPreset, "/arm_planner_node/execute_preset"),
            (Trigger, "/arm_planner_node/stop"),
        ):
            client = self.create_client(service_type, name)
            try:
                ready = client.wait_for_service(
                    timeout_sec=max(0.0, deadline - time.monotonic())
                )
            finally:
                self.destroy_client(client)
            if not ready:
                self.get_logger().error(f"Timed out waiting for planner service: {name}")
                return False
        return True

    def wait_for_stack(self) -> bool:
        """Run the existing independent, bounded mission startup checks once."""
        checks = (
            ("navigation nodes", lambda: self.wait_for_nodes([
                "/axis_waypoint_action_server", "/laserMapping", "/loam_interface",
            ], 10.0)),
            ("navigation action", lambda: self.wait_for_action(
                self.follow_waypoints_action, 10.0)),
            ("odometry", lambda: self.wait_for_odometry("/odometry", 10.0)),
            ("navigation TF", lambda: self.wait_for_transform(
                "odom", "base_link", 5.0, 1.0)),
            ("perception nodes", lambda: self.wait_for_nodes([
                "/serial_controller", "/fruit_qr_planner",
            ], 10.0)),
            ("perception pipeline", lambda: self.wait_for_perception_heartbeat(
                self.perception_heartbeat_topic, self.perception_readiness_timeout_sec)),
            ("serial bridge", lambda: self.wait_for_bridge(10.0)),
            ("planner node", lambda: self.wait_for_nodes(["/arm_planner_node"], 10.0)),
            ("planner services", lambda: self.wait_for_planner(10.0)),
        )
        for label, check in checks:
            if not rclpy.ok():
                return False
            if check():
                self.get_logger().info(f"Startup ready: {label}")
            else:
                self.get_logger().warning(
                    f"Startup unavailable: {label}; mission will skip unavailable work"
                )
        return rclpy.ok()

    def wait_for_transform(
        self,
        parent_frame: str,
        child_frame: str,
        timeout_sec: float,
        max_age_sec: float,
    ) -> bool:
        """Wait for a fresh dynamic transform used by navigation."""
        latest_detail = "no transform received"

        def transform_is_ready() -> bool:
            nonlocal latest_detail
            try:
                transform = self._tf_buffer.lookup_transform(
                    parent_frame,
                    child_frame,
                    Time(),
                )
            except TransformException as error:
                latest_detail = str(error)
                return False
            stamp = transform.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if stamp_ns <= 0:
                latest_detail = "transform has an invalid zero timestamp"
                return False
            age_sec = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0
            if abs(age_sec) > max_age_sec:
                latest_detail = f"transform age is {age_sec:.3f}s"
                return False
            return True

        ready = self.wait_until(transform_is_ready, timeout_sec)
        if not ready:
            self.get_logger().error(
                f"Timed out waiting for fresh {parent_frame}->{child_frame} TF: "
                f"{latest_detail}"
            )
        return ready

    def wait_for_odometry(self, topic: str, timeout_sec: float) -> bool:
        """Wait for the first finite, timestamped Point-LIO odometry sample."""
        received = False
        latest_detail = "no odometry received"

        def on_odometry(message: Odometry) -> None:
            nonlocal received, latest_detail
            stamp = message.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            pose = message.pose.pose
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            quaternion_norm = sum(value * value for value in values[3:])
            if stamp_ns <= 0:
                latest_detail = "odometry has an invalid zero timestamp"
            elif not all(math.isfinite(value) for value in values):
                latest_detail = "odometry pose contains a non-finite value"
            elif quaternion_norm <= 0.0:
                latest_detail = "odometry orientation is invalid"
            else:
                received = True

        subscription = self.create_subscription(
            Odometry,
            topic,
            on_odometry,
            20,
        )
        ready = self.wait_until(lambda: received, timeout_sec)
        self.destroy_subscription(subscription)
        if not ready:
            self.get_logger().error(
                f"Timed out waiting for valid odometry on {topic}: "
                f"{latest_detail}"
            )
        return ready

    def wait_for_perception_heartbeat(
        self,
        topic: str,
        timeout_sec: float,
    ) -> bool:
        """Wait for the first RGBD heartbeat from the perception pipeline."""
        received = False

        def on_heartbeat(_message: Header) -> None:
            nonlocal received
            received = True

        subscription = self.create_subscription(
            Header,
            topic,
            on_heartbeat,
            qos_profile_sensor_data,
        )
        ready = self.wait_until(lambda: received, timeout_sec)
        self.destroy_subscription(subscription)
        if not ready:
            self.get_logger().error(
                f"Timed out waiting for perception heartbeat on {topic}"
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

    def bridge_start_required(self) -> bool:
        """Select bridge startup without treating hardware readiness as absence."""
        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
        bridge_nodes = [
            name for name, _namespace in self.get_node_names_and_namespaces()
            if name == "serial_controller"
        ]
        status_publishers = self.get_publishers_info_by_topic(
            "/fruit_hardware_bridge/arm/status"
        )
        ready_publishers = self.get_publishers_info_by_topic(self.bridge_ready_topic)
        for publishers, expected_type in (
            (status_publishers, "fruit_interfaces/msg/ArmStatus"),
            (ready_publishers, "std_msgs/msg/Bool"),
        ):
            if len(publishers) > 1 or any(
                publisher.topic_type != expected_type for publisher in publishers
            ):
                raise ValueError("Conflicting serial bridge publishers or message types")
        if len(bridge_nodes) > 1:
            raise ValueError("Multiple serial_controller nodes are already running")
        return not (bridge_nodes or status_publishers or ready_publishers)

    def robviz_bridge_start_required(self) -> bool:
        """Select RobViz bridge startup without duplicate instances."""
        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
        bridge_nodes = [
            name for name, _namespace in self.get_node_names_and_namespaces()
            if name in ("fruit_robviz_image_bridge", "fruit_robviz_config_client")
        ]
        return not bool(bridge_nodes)

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

    node_running = commands.add_parser("node-running")
    node_running.add_argument("--node", action="append", required=True)
    node_running.add_argument(
        "--match", choices=("any", "all"), default="all"
    )

    nodes = commands.add_parser("nodes")
    nodes.add_argument("--node", action="append", required=True)
    nodes.add_argument("--timeout", type=float, required=True)

    action = commands.add_parser("action")
    action.add_argument("--name")
    action.add_argument("--timeout", type=float, required=True)

    bridge = commands.add_parser("bridge")
    bridge.add_argument("--timeout", type=float, required=True)

    commands.add_parser("bridge-start-required")
    commands.add_parser("robviz-bridge-start-required")
    commands.add_parser("stack")

    planner = commands.add_parser("planner")
    planner.add_argument("--timeout", type=float, required=True)

    endpoint = commands.add_parser("endpoint")
    endpoint.add_argument("--topic", required=True)
    endpoint.add_argument(
        "--kind", choices=("publishers", "subscribers"), required=True
    )
    endpoint.add_argument("--count", type=int, required=True)
    endpoint.add_argument("--timeout", type=float, required=True)

    transform = commands.add_parser("tf")
    transform.add_argument("--parent", required=True)
    transform.add_argument("--child", required=True)
    transform.add_argument("--timeout", type=float, required=True)
    transform.add_argument("--max-age", type=float, default=1.0)

    odometry = commands.add_parser("odometry")
    odometry.add_argument("--topic", required=True)
    odometry.add_argument("--timeout", type=float, required=True)

    perception = commands.add_parser("perception")
    perception.add_argument("--topic", default="/fruit_perception/camera/rgbd_heartbeat")
    perception.add_argument("--timeout", type=float, default=35.0)
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
        elif options.command == "node-running":
            ready = node.nodes_are_running(options.node, options.match)
        elif options.command == "nodes":
            ready = node.wait_for_nodes(options.node, options.timeout)
        elif options.command == "action":
            action_name = options.name or node.follow_waypoints_action
            ready = node.wait_for_action(action_name, options.timeout)
        elif options.command == "bridge":
            ready = node.wait_for_bridge(options.timeout)
        elif options.command == "bridge-start-required":
            print("true" if node.bridge_start_required() else "false", flush=True)
            ready = True
        elif options.command == "robviz-bridge-start-required":
            print("true" if node.robviz_bridge_start_required() else "false", flush=True)
            ready = True
        elif options.command == "planner":
            ready = node.wait_for_planner(options.timeout)
        elif options.command == "stack":
            ready = node.wait_for_stack()
        elif options.command == "endpoint":
            ready = node.wait_for_endpoint(
                options.topic,
                options.kind,
                options.count,
                options.timeout,
            )
        elif options.command == "tf":
            ready = node.wait_for_transform(
                options.parent,
                options.child,
                options.timeout,
                options.max_age,
            )
        elif options.command == "odometry":
            ready = node.wait_for_odometry(options.topic, options.timeout)
        elif options.command == "perception":
            ready = node.wait_for_perception_heartbeat(options.topic, options.timeout)
    except KeyboardInterrupt:
        ready = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ready else 1)


if __name__ == "__main__":
    main()
