"""ROS communication helpers used by the mission flow."""

from __future__ import annotations

import json
import math
from pathlib import Path
from dataclasses import dataclass
from threading import Event
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Union

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from fruit_hardware_interfaces.msg import ArmTarget
from fruit_hardware_interfaces.msg import ExecutionCallback
from fruit_hardware_interfaces.msg import StateMachineCommand
from fruit_perception.msg import FruitTypes

from fruit_mission.logic import PickExecutionResult
from fruit_mission.logic import SideObservation
from fruit_mission.logic import merge_seen_types
from fruit_mission.tts import PiperSpeaker
from fruit_mission.waypoint_route import FormalWaypointRoute
from fruit_mission.waypoint_route import WaypointPose


AREA_STATUS = {
    "A": StateMachineCommand.AREA_A,
    "B": StateMachineCommand.AREA_B,
    "C": StateMachineCommand.AREA_C,
    "D": StateMachineCommand.AREA_D,
}
NAV_GOAL_ATTEMPTS = 3


@dataclass(frozen=True)
class _D435iTarget:
    point: PointStamped


@dataclass(frozen=True)
class _D435iSelection:
    reason_code: str
    target: Optional[_D435iTarget] = None


class MissionContext:
    """High-level blocking wrappers around the existing ROS interfaces."""

    def __init__(self, node: Node, config: Mapping[str, Any], speaker=None) -> None:
        self.node = node
        self.config = config
        self.speaker = speaker or PiperSpeaker(config["tts"], node.get_logger())
        self.route = FormalWaypointRoute.from_file(Path(str(config["waypoint_file"])))
        self._route_cursor = 0
        self._latest_active_mode: Optional[str] = None
        self._active_mode_generation = 0
        self._latest_left: Optional[list[str]] = None
        self._latest_right: Optional[list[str]] = None
        self._latest_d435i_types: Optional[list[str]] = None
        self._latest_d435i_target: Optional[PointStamped] = None
        self._latest_b_plan: Optional[Dict[str, Any]] = None
        self._latest_c_plan: Optional[Dict[str, Any]] = None
        self._pending_callback_id: Optional[int] = None
        self._matched_callback_status: Optional[int] = None
        self._callback_event = Event()

        self._mode_publisher = node.create_publisher(
            String,
            config["perception_mode_topic"],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._current_area_publisher = node.create_publisher(
            String,
            config["current_area_topic"],
            _latched_qos(),
        )
        self._command_publisher = node.create_publisher(
            StateMachineCommand,
            config["state_command_topic"],
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._arm_target_publisher = node.create_publisher(
            ArmTarget,
            config["arm_target_topic"],
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._follow_waypoints_client = ActionClient(
            node,
            FollowWaypoints,
            str(config["follow_waypoints_action"]),
        )
        node.create_subscription(
            String,
            config["active_mode_topic"],
            self._on_active_mode,
            _latched_qos(),
        )
        node.create_subscription(
            FruitTypes,
            config["left_types_topic"],
            self._on_left_types,
            QoSProfile(depth=10),
        )
        node.create_subscription(
            FruitTypes,
            config["right_types_topic"],
            self._on_right_types,
            QoSProfile(depth=10),
        )
        node.create_subscription(
            FruitTypes,
            config["d435i_types_topic"],
            self._on_d435i_types,
            QoSProfile(depth=10),
        )
        node.create_subscription(
            PointStamped,
            config["d435i_target_point_topic"],
            self._on_d435i_target,
            QoSProfile(depth=10),
        )
        node.create_subscription(
            String,
            config["b_plan_topic"],
            self._on_b_plan,
            _latched_qos(),
        )
        node.create_subscription(
            String,
            config["c_plan_topic"],
            self._on_c_plan,
            _latched_qos(),
        )
        node.create_subscription(
            ExecutionCallback,
            config["callback_topic"],
            self._on_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    def go_to(self, waypoint_names: Union[str, Sequence[str]]) -> bool:
        """Reach one or more named waypoints through /follow_waypoints."""
        requested_names = (
            [waypoint_names]
            if isinstance(waypoint_names, str)
            else list(waypoint_names)
        )
        if not requested_names:
            return True
        try:
            names, next_cursor = self._expand_waypoint_names(requested_names)
        except KeyError as error:
            self.node.get_logger().error(str(error))
            return False
        if not self._send_waypoint_names(names):
            return False
        self._route_cursor = next_cursor
        return True

    def go_through(self, waypoint_names: Sequence[str]) -> bool:
        """Reach the explicitly listed waypoints without YAML-order expansion."""
        names = (
            [waypoint_names]
            if isinstance(waypoint_names, str)
            else [str(name) for name in waypoint_names]
        )
        if not self._send_waypoint_names(names):
            return False
        if names:
            self._route_cursor = self.route.pose(names[-1]).order + 1
        return True

    def _expand_waypoint_names(
        self,
        requested_names: Sequence[str],
    ) -> tuple[list[str], int]:
        names: list[str] = []
        cursor = self._route_cursor
        for name in requested_names:
            segment, cursor = self.route.ordered_names_to(str(name), cursor)
            names.extend(segment)
        return names, cursor

    def _send_waypoint_names(self, names: Sequence[str]) -> bool:
        if not names:
            return True
        try:
            poses = [self._pose_stamped(self.route.pose(name)) for name in names]
        except KeyError as error:
            self.node.get_logger().error(str(error))
            return False
        if not self._follow_waypoints_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error(
                f"FollowWaypoints server unavailable: {self.config['follow_waypoints_action']}"
            )
            return False

        goal = FollowWaypoints.Goal()
        goal.poses = poses
        route_text = " -> ".join(names)
        goal_handle = None
        for attempt in range(1, NAV_GOAL_ATTEMPTS + 1):
            self.node.get_logger().info(f"Navigating through {route_text}")
            future = self._follow_waypoints_client.send_goal_async(goal)
            if not self._wait_future(future):
                if attempt == NAV_GOAL_ATTEMPTS:
                    return False
                time.sleep(0.3)
                continue
            candidate_handle = future.result()
            if candidate_handle.accepted:
                goal_handle = candidate_handle
                break
            self.node.get_logger().warning(
                f"FollowWaypoints goal rejected "
                f"attempt={attempt}/{NAV_GOAL_ATTEMPTS}: {route_text}"
            )
            time.sleep(0.3)
        if goal_handle is None:
            self.node.get_logger().error(
                "FollowWaypoints goal rejected: " + route_text
            )
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future):
            return False
        result = result_future.result().result
        missed = list(result.missed_waypoints)
        if missed:
            self.node.get_logger().error(
                f"FollowWaypoints missed indexes {missed}: {' -> '.join(names)}"
            )
            return False
        return True

    def stable_stop(self) -> None:
        """Wait after a navigation arrival so cameras see a steady target."""
        time.sleep(float(self.config["stable_stop_sec"]))

    def set_perception_mode(self, mode: str) -> bool:
        """Publish a perception mode and wait for the active-mode echo."""
        message = String()
        message.data = mode
        previous_generation = self._active_mode_generation
        deadline = time.monotonic() + float(self.config["active_mode_timeout_sec"])
        while rclpy.ok() and time.monotonic() < deadline:
            self._mode_publisher.publish(message)
            remaining = deadline - time.monotonic()
            if self._wait_until(
                lambda: self._active_mode_generation > previous_generation
                and self._latest_active_mode == mode,
                min(0.5, max(0.0, remaining)),
            ):
                return True
        return (
            self._active_mode_generation > previous_generation
            and self._latest_active_mode == mode
        )

    def observe_sides(self, timeout_sec: float) -> SideObservation:
        """Collect recent side camera detections for a short window."""
        if not self.set_perception_mode("SIDES"):
            self.node.get_logger().error(
                "Could not reset side target selection before observation"
            )
            return SideObservation(left=[], right=[])
        self._latest_left = None
        self._latest_right = None
        left_seen: list[str] = []
        right_seen: list[str] = []
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            left_seen = merge_seen_types(left_seen, self._latest_left or [])
            right_seen = merge_seen_types(right_seen, self._latest_right or [])
        return SideObservation(left=left_seen, right=right_seen)

    def wait_b_plan(self) -> Optional[Dict[str, Any]]:
        """Wait for the latched B QR plan."""
        self._wait_until(
            lambda: self._latest_b_plan is not None,
            float(self.config["qr_plan_timeout_sec"]),
        )
        return self._latest_b_plan

    def wait_c_plan(self) -> Optional[Dict[str, Any]]:
        """Wait for the latched C QR plan."""
        self._wait_until(
            lambda: self._latest_c_plan is not None,
            float(self.config["qr_plan_timeout_sec"]),
        )
        return self._latest_c_plan

    def send_pick_direction(self, side: str) -> bool:
        """Send one A/B/C pick-direction command and wait for completion."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        status = (
            StateMachineCommand.PICK_DIRECTION_LEFT
            if side == "left"
            else StateMachineCommand.PICK_DIRECTION_RIGHT
        )
        return self._send_command_and_wait(
            StateMachineCommand.TASK_PICK_DIRECTION,
            status,
            ExecutionCallback.CALLBACK_PICK_DIRECTION,
        )

    def execute_closed_loop_pick(
        self,
        side: str,
    ) -> PickExecutionResult:
        """Send pick direction, use D435i depth, and send ArmTarget."""
        if not self.set_perception_mode("SIDES"):
            return PickExecutionResult(False, "sides_mode_timeout")
        if not self.send_pick_direction(side):
            return PickExecutionResult(False, "side_pose_timeout")
        fresh_after_ns = self.node.get_clock().now().nanoseconds
        if not self.set_perception_mode("TREE"):
            return PickExecutionResult(False, "tree_mode_timeout")
        selection = self._wait_d435i_target(fresh_after_ns)
        if selection.target is None:
            return PickExecutionResult(False, selection.reason_code)
        if not self._send_arm_target_and_wait(selection.target.point):
            return PickExecutionResult(
                False,
                "arm_callback_timeout",
            )
        return PickExecutionResult(True, "picked")

    def send_qr_pose(self) -> bool:
        """Ask the lower controller to enter QR recognition pose."""
        return self._send_command_and_wait(
            StateMachineCommand.TASK_QR_RECOGNITION_POSE,
            StateMachineCommand.QR_RECOGNITION_POSE,
            ExecutionCallback.CALLBACK_QR_RECOGNITION_POSE,
        )

    def send_arm_retract_pose(self) -> bool:
        """Ask the lower controller to retract the arm."""
        return self._send_command_and_wait(
            StateMachineCommand.TASK_ARM_RETRACT_POSE,
            StateMachineCommand.ARM_RETRACT_POSE,
            ExecutionCallback.CALLBACK_ARM_RETRACT_POSE,
        )

    def send_return_initial_pose(self) -> bool:
        """Ask the lower controller to return to the initial pose."""
        return self._send_command_and_wait(
            StateMachineCommand.TASK_RETURN_INITIAL_POSE,
            StateMachineCommand.RETURN_INITIAL_POSE,
            ExecutionCallback.CALLBACK_RETURN_INITIAL_POSE,
        )

    def report_current_area(self, area: str) -> bool:
        """Report current competition area to the lower controller."""
        reported = self._send_command_and_wait(
            StateMachineCommand.TASK_CURRENT_AREA,
            AREA_STATUS[area],
            ExecutionCallback.CALLBACK_CURRENT_AREA,
        )
        if reported:
            message = String()
            message.data = area
            self._current_area_publisher.publish(message)
        return reported

    def announce(self, text: str) -> bool:
        """Speak text through the configured TTS backend."""
        self.node.get_logger().info(f"播报：{text}")
        return self.speaker.announce(text)

    def announce_and_wait(self, text: str) -> bool:
        """Speak text and wait until playback completes."""
        self.node.get_logger().info(f"同步播报：{text}")
        announce_and_wait = getattr(self.speaker, "announce_and_wait", None)
        if callable(announce_and_wait):
            return bool(announce_and_wait(text))
        return self.speaker.announce(text)

    def close(self) -> None:
        """Release mission runtime helpers."""
        close = getattr(self.speaker, "close", None)
        if callable(close):
            close(wait=False)

    def save_c_distribution(self, distribution: Mapping[str, Any]) -> None:
        """Write the latest C-zone position table for inspection."""
        output_dir = Path(str(self.config["c_distribution_output_dir"]))
        output_dir.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(distribution, ensure_ascii=False, indent=2)
        latest_path = output_dir / "c_distribution_latest.json"
        latest_path.write_text(encoded + "\n", encoding="utf-8")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        (output_dir / f"c_distribution_{stamp}.json").write_text(
            encoded + "\n",
            encoding="utf-8",
        )

    def _send_command_and_wait(
        self,
        task_id: int,
        task_status: int,
        callback_id: int,
        recover_on_failure: bool = True,
    ) -> bool:
        if not self._wait_for_hardware_bridge_endpoints(
            str(self.config["state_command_topic"])
        ):
            return False
        self._begin_callback_wait(callback_id)
        command = StateMachineCommand()
        command.task_id = task_id
        command.task_status = task_status
        self.node.get_logger().info(
            f"发送下位机状态命令 task_id={task_id}, task_status={task_status}, "
            f"等待 callback_id={callback_id}"
        )
        self._command_publisher.publish(command)
        return self._finish_callback_wait(
            callback_id,
            f"callback_id={callback_id}",
            recover_on_failure,
        )

    def _send_arm_target_and_wait(self, point: PointStamped) -> bool:
        callback_id = ExecutionCallback.CALLBACK_ARM_TARGET
        if not self._wait_for_hardware_bridge_endpoints(
            str(self.config["arm_target_topic"])
        ):
            return False
        self._begin_callback_wait(callback_id)
        target = ArmTarget()
        target.target_x = float(point.point.x)
        target.target_y = float(point.point.y)
        target.target_z = float(point.point.z)
        target.z_type = ArmTarget.Z_GROUND
        self.node.get_logger().info(
            "发送下位机抓取目标 "
            f"x={target.target_x:.3f}, y={target.target_y:.3f}, "
            f"z={target.target_z:.3f}, z_type={target.z_type}, "
            f"等待 callback_id={callback_id}"
        )
        self._arm_target_publisher.publish(target)
        return self._finish_callback_wait(
            callback_id,
            "ArmTarget callback",
            True,
        )

    def _wait_for_hardware_bridge_endpoints(self, outbound_topic: str) -> bool:
        timeout_sec = min(5.0, float(self.config["callback_timeout_sec"]))
        callback_topic = str(self.config["callback_topic"])
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            outbound_subscribers = self.node.count_subscribers(outbound_topic)
            callback_publishers = self.node.count_publishers(callback_topic)
            if outbound_subscribers > 0 and callback_publishers > 0:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().error(
            "下位机串口桥 ROS 端点未就绪："
            f"{outbound_topic} subscribers={self.node.count_subscribers(outbound_topic)}, "
            f"{callback_topic} publishers={self.node.count_publishers(callback_topic)}"
        )
        return False

    def _begin_callback_wait(self, callback_id: int) -> None:
        self._pending_callback_id = callback_id
        self._matched_callback_status = None
        self._callback_event.clear()

    def _finish_callback_wait(
        self,
        callback_id: int,
        timeout_label: str,
        recover_on_failure: bool,
    ) -> bool:
        success = self._wait_until(
            self._callback_event.is_set,
            float(self.config["callback_timeout_sec"]),
        )
        self._pending_callback_id = None
        if not success:
            self.node.get_logger().error(f"Timed out waiting for {timeout_label}")
            return False

        status = self._matched_callback_status
        self._matched_callback_status = None
        if status == ExecutionCallback.STATUS_COMPLETED:
            return True
        if status == ExecutionCallback.STATUS_FAILED:
            self.node.get_logger().error(
                f"Lower controller reported failed {timeout_label}"
            )
            if recover_on_failure:
                self._return_initial_pose_after_failure(callback_id)
            return False

        self.node.get_logger().warning(
            f"Ignoring unexpected callback_status={status} for {timeout_label}"
        )
        return False

    def _return_initial_pose_after_failure(self, failed_callback_id: int) -> None:
        if failed_callback_id == ExecutionCallback.CALLBACK_RETURN_INITIAL_POSE:
            self.node.get_logger().error(
                "Return-initial-pose callback failed; skipping recursive recovery"
            )
            return
        self.node.get_logger().info(
            "Returning to initial pose after failed "
            f"callback_id={failed_callback_id}"
        )
        self._send_command_and_wait(
            StateMachineCommand.TASK_RETURN_INITIAL_POSE,
            StateMachineCommand.RETURN_INITIAL_POSE,
            ExecutionCallback.CALLBACK_RETURN_INITIAL_POSE,
            recover_on_failure=False,
        )

    def _wait_d435i_target(
        self,
        fresh_after_ns: int,
    ) -> _D435iSelection:
        self._latest_d435i_target = None
        latest_stamp_ns = fresh_after_ns
        deadline = time.monotonic() + float(self.config["d435i_detection_timeout_sec"])
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._latest_d435i_target is None:
                continue
            target = self._latest_d435i_target
            self._latest_d435i_target = None
            stamp = target.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns <= latest_stamp_ns:
                continue
            latest_stamp_ns = stamp_ns
            point = (
                float(target.point.x),
                float(target.point.y),
                float(target.point.z),
            )
            if not all(math.isfinite(value) for value in point):
                continue
            self.node.get_logger().info(
                "D435i target received from the 35-frame perception window: "
                f"x={point[0]}, y={point[1]}, z={point[2]}"
            )
            return _D435iSelection("matched", _D435iTarget(target))
        return _D435iSelection("d435i_target_timeout")

    def _wait_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return predicate()

    def _wait_future(self, future) -> bool:
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if future.done():
            return True
        self.node.get_logger().error("Interrupted while waiting for navigation action")
        return False

    def _pose_stamped(self, pose: WaypointPose) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = "map"
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        stamped.pose.position.z = pose.z
        stamped.pose.orientation.x = pose.qx
        stamped.pose.orientation.y = pose.qy
        stamped.pose.orientation.z = pose.qz
        stamped.pose.orientation.w = pose.qw
        return stamped

    def _on_active_mode(self, message: String) -> None:
        self._latest_active_mode = message.data
        self._active_mode_generation += 1

    def _on_left_types(self, message: FruitTypes) -> None:
        self._latest_left = list(message.types)

    def _on_right_types(self, message: FruitTypes) -> None:
        self._latest_right = list(message.types)

    def _on_d435i_types(self, message: FruitTypes) -> None:
        self._latest_d435i_types = list(message.types)

    def _on_d435i_target(self, message: PointStamped) -> None:
        self._latest_d435i_target = message

    def _on_b_plan(self, message: String) -> None:
        self._latest_b_plan = json.loads(message.data)

    def _on_c_plan(self, message: String) -> None:
        self._latest_c_plan = json.loads(message.data)

    def _on_callback(self, message: ExecutionCallback) -> None:
        if message.callback_id != self._pending_callback_id:
            return
        if message.callback_status == ExecutionCallback.STATUS_EXECUTING:
            return
        if message.callback_status not in (
            ExecutionCallback.STATUS_COMPLETED,
            ExecutionCallback.STATUS_FAILED,
        ):
            return
        self._matched_callback_status = message.callback_status
        self._callback_event.set()


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
