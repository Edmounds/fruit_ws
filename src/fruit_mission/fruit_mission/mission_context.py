"""ROS communication helpers used by the mission flow."""

from __future__ import annotations

import json
import math
from pathlib import Path
from dataclasses import dataclass
from threading import Event
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Union

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from fruit_hardware_interfaces.msg import ArmTarget
from fruit_hardware_interfaces.msg import ExecutionCallback
from fruit_hardware_interfaces.msg import StateMachineCommand
from fruit_perception.msg import FruitDecision

from fruit_mission.logic import PickExecutionResult
from fruit_mission.logic import SideObservation
from fruit_mission.logic import merge_seen_types
from fruit_mission.tts import PiperSpeaker
from fruit_mission.waypoint_route import MissionWaypointRoute
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
    matched_type: Optional[str] = None


@dataclass(frozen=True)
class _CallbackWaitResult:
    completed: bool
    reason_code: str
    safe_to_continue: bool


@dataclass(frozen=True)
class BTreeTarget:
    """One stable B-zone perception target retained for mission decisions."""

    model_type: str
    point: PointStamped


class MissionContext:
    """High-level blocking wrappers around the existing ROS interfaces."""

    def __init__(self, node: Node, config: Mapping[str, Any], speaker=None) -> None:
        self.node = node
        self.config = config
        self.speaker = speaker or PiperSpeaker(config["tts"], node.get_logger())
        self.route = MissionWaypointRoute.from_file(
            Path(str(config["waypoint_file"])),
            str(config.get("mission_flow", "full")),
            str(config["map_frame"]),
        )
        self._route_cursor = 0
        self._current_waypoint_name: Optional[str] = None
        self._current_route_yaw: Optional[float] = None
        self._current_area: Optional[str] = None
        self._latest_active_mode: Optional[str] = None
        self._active_mode_generation = 0
        self._latest_left: Optional[list[str]] = None
        self._latest_right: Optional[list[str]] = None
        self._latest_d435i_types: Optional[list[str]] = None
        self._latest_d435i_types_stamp_ns = 0
        self._latest_d435i_target: Optional[PointStamped] = None
        self._latest_b_targets: Optional[list[Any]] = None
        self._latest_b_targets_stamp_ns = 0
        self._latest_b_plan: Optional[Dict[str, Any]] = None
        self._latest_c_plan: Optional[Dict[str, Any]] = None
        self._pending_callback_id: Optional[int] = None
        self._matched_callback_status: Optional[int] = None
        self._callback_event = Event()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
        )

        self._mode_publisher = node.create_publisher(
            String,
            config["perception_mode_topic"],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._expected_fruit_type_publisher = node.create_publisher(
            String,
            config["expected_fruit_type_topic"],
            _latched_qos(),
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
            FruitDecision,
            config["decision_topic"],
            self._on_decision,
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
        self._current_waypoint_name = names[-1]
        self._current_route_yaw = self._waypoint_yaw(names[-1])
        return True

    def navigate_named(self, waypoint_names: Sequence[str]) -> bool:
        """Send exactly the requested names, without YAML-order expansion."""
        names = [str(name) for name in waypoint_names]
        if not names:
            return True
        try:
            for name in names:
                self.route.pose(name)
        except KeyError as error:
            self.node.get_logger().error(str(error))
            return False
        if not self._send_waypoint_names(names):
            return False
        self._current_waypoint_name = names[-1]
        self._current_route_yaw = self._waypoint_yaw(names[-1])
        self._route_cursor = self.route.pose(names[-1]).order + 1
        return True

    def go_through(self, waypoint_names: Sequence[str]) -> bool:
        """Reach explicit C-route points using their saved YAML yaw."""
        names = (
            [waypoint_names]
            if isinstance(waypoint_names, str)
            else [str(name) for name in waypoint_names]
        )
        if not names:
            return True
        if self._current_waypoint_name is None:
            self.node.get_logger().error(
                "Cannot set C-route yaw before reaching a known waypoint"
            )
            return False
        try:
            targets, final_name, final_yaw = self.route.waypoint_yaw_targets(
                self._current_waypoint_name,
                names,
                self._current_route_yaw,
            )
        except KeyError as error:
            self.node.get_logger().error(str(error))
            return False
        target_names = [name for name, _yaw in targets]
        target_yaws = [yaw for _name, yaw in targets]
        if target_names and not self._send_waypoint_names(target_names, target_yaws):
            return False
        self._current_waypoint_name = final_name
        self._current_route_yaw = final_yaw
        self._route_cursor = self.route.pose(final_name).order + 1
        return True

    def navigate_c_position(self, position_id: int) -> Dict[str, Any]:
        """Plan and execute one C task from the live TF map pose."""
        start = self._lookup_map_pose()
        start_source = "tf"
        if start is None:
            start = self._previous_target_pose()
            start_source = "previous_target"
        if start is None:
            return {
                "status": "planning_failed",
                "reason": "tf_unavailable",
                "navigation_start_source": "none",
            }
        start_x, start_y, start_yaw = start
        try:
            plan = self.route.plan_c_navigation(
                start_x,
                start_y,
                start_yaw,
                int(position_id),
                float(self.config["c_graph_max_start_distance"]),
            )
        except ValueError as error:
            self.node.get_logger().error(str(error))
            return {
                "status": "planning_failed",
                "reason": "route_unavailable",
                "navigation_start": {"x": start_x, "y": start_y},
                "navigation_start_source": start_source,
                "detail": str(error),
            }

        result = {
            "status": "navigation_failed",
            "reason": "follow_waypoints_failed",
            "navigation_start": {"x": plan.start_x, "y": plan.start_y},
            "navigation_start_source": start_source,
            "goal_name": plan.goal_name,
            "position_side": plan.position_side,
            "graph_path": list(plan.graph_path),
            "movement_points": list(plan.navigation_names),
            "navigation_yaws": list(plan.navigation_yaws),
            "route_distance": plan.distance,
            "camera_side": plan.camera_side,
            "preferred_side": plan.camera_side,
        }
        if plan.rejoin_point is not None:
            result["route_rejoin"] = {
                "x": plan.rejoin_point[0],
                "y": plan.rejoin_point[1],
            }
        if not self._send_waypoint_names(
            plan.navigation_names,
            plan.navigation_yaws,
        ):
            self._update_c_arrival_side(result, plan.position_side, plan.camera_side)
            return result

        self._update_c_arrival_side(result, plan.position_side, plan.camera_side)
        result["status"] = "succeeded"
        result["reason"] = None
        self._current_waypoint_name = plan.goal_name
        self._current_route_yaw = plan.navigation_yaws[-1]
        self._route_cursor = max(
            self._route_cursor,
            self.route.pose(plan.goal_name).order + 1,
        )
        return result

    def _previous_target_pose(self) -> Optional[tuple[float, float, float]]:
        """Return the last mission target pose for C-route recovery."""
        if self._current_waypoint_name is None:
            return None
        try:
            pose = self.route.pose(self._current_waypoint_name)
        except KeyError:
            return None
        yaw = self._current_route_yaw
        if yaw is None:
            yaw = self._waypoint_yaw(self._current_waypoint_name)
        return float(pose.x), float(pose.y), float(yaw)

    def _update_c_arrival_side(
        self,
        result: Dict[str, Any],
        position_side: str,
        fallback_side: str,
    ) -> None:
        """Use the post-action vehicle yaw when selecting the arm side."""
        arrival = self._lookup_map_pose()
        if arrival is None:
            result["arrival_yaw_source"] = "planned"
            result["arrival_yaw"] = result["navigation_yaws"][-1]
            result["camera_side"] = fallback_side
            result["preferred_side"] = fallback_side
            return
        result["arrival_yaw_source"] = "tf"
        result["arrival_yaw"] = arrival[2]
        side = self.route.c_camera_side(position_side, arrival[2])
        result["camera_side"] = side
        result["preferred_side"] = side

    def _lookup_map_pose(self) -> Optional[tuple[float, float, float]]:
        deadline = time.monotonic() + float(self.config["tf_lookup_timeout_sec"])
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    str(self.config["map_frame"]),
                    str(self.config["base_frame"]),
                    Time(),
                )
            except TransformException as error:
                last_error = error
                rclpy.spin_once(self.node, timeout_sec=0.05)
                continue
            stamp = transform.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            now_ns = self.node.get_clock().now().nanoseconds
            age_sec = (now_ns - stamp_ns) / 1_000_000_000.0
            if stamp_ns <= 0 or age_sec > float(self.config["tf_lookup_timeout_sec"]):
                last_error = RuntimeError(
                    f"map->base_link transform is stale ({age_sec:.3f}s)"
                )
                rclpy.spin_once(self.node, timeout_sec=0.05)
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            return float(translation.x), float(translation.y), yaw
        detail = f": {last_error}" if last_error is not None else ""
        self.node.get_logger().error(
            "Could not resolve live robot pose "
            f"{self.config['map_frame']}->{self.config['base_frame']}{detail}"
        )
        return None

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

    def _send_waypoint_names(
        self,
        names: Sequence[str],
        yaws: Optional[Sequence[float]] = None,
    ) -> bool:
        if not names:
            return True
        try:
            if yaws is not None and len(yaws) != len(names):
                raise ValueError("waypoint names and yaw values must have equal length")
        except ValueError as error:
            self.node.get_logger().error(str(error))
            return False
        if not self._follow_waypoints_client.wait_for_server(
            timeout_sec=float(self.config["navigation_action_server_timeout_sec"])
        ):
            self.node.get_logger().error(
                f"FollowWaypoints server unavailable: {self.config['follow_waypoints_action']}"
            )
            return False

        pending_names = list(names)
        pending_yaws = None if yaws is None else list(yaws)
        for attempt in range(1, NAV_GOAL_ATTEMPTS + 1):
            if not pending_names:
                return True
            try:
                poses = [
                    self._pose_stamped(
                        self.route.pose(name),
                        None if pending_yaws is None else pending_yaws[index],
                    )
                    for index, name in enumerate(pending_names)
                ]
            except (IndexError, KeyError) as error:
                self.node.get_logger().error(str(error))
                return False

            goal = FollowWaypoints.Goal()
            goal.poses = poses
            route_text = " -> ".join(pending_names)
            self.node.get_logger().info(
                f"Navigating through {route_text} "
                f"attempt={attempt}/{NAV_GOAL_ATTEMPTS}"
            )
            future = self._follow_waypoints_client.send_goal_async(goal)
            if not self._wait_future(future):
                if attempt == NAV_GOAL_ATTEMPTS:
                    return False
                time.sleep(0.3)
                continue
            candidate_handle = future.result()
            if not candidate_handle.accepted:
                self.node.get_logger().warning(
                    f"FollowWaypoints goal rejected "
                    f"attempt={attempt}/{NAV_GOAL_ATTEMPTS}: {route_text}"
                )
                if attempt < NAV_GOAL_ATTEMPTS:
                    time.sleep(0.3)
                continue

            result_future = candidate_handle.get_result_async()
            if not self._wait_future(result_future):
                if attempt < NAV_GOAL_ATTEMPTS:
                    time.sleep(0.3)
                continue
            wrapped_result = result_future.result()
            result = wrapped_result.result
            missed = list(result.missed_waypoints)
            if wrapped_result.status == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().error(
                    f"FollowWaypoints canceled: {route_text}"
                )
                return False
            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED and not missed:
                return True

            next_index = missed[0] if missed else 0
            if attempt == NAV_GOAL_ATTEMPTS:
                self.node.get_logger().error(
                    "FollowWaypoints retries exhausted: "
                    f"status={wrapped_result.status} "
                    f"missed={missed} route={route_text}"
                )
                return False
            if 0 <= next_index < len(pending_names):
                pending_names = pending_names[next_index:]
                if pending_yaws is not None:
                    pending_yaws = pending_yaws[next_index:]
                self.node.get_logger().warning(
                    "FollowWaypoints did not finish; retrying unfinished "
                    f"suffix from index={next_index}: {' -> '.join(pending_names)}"
                )
            else:
                self.node.get_logger().error(
                    f"FollowWaypoints returned invalid missed index {missed}: "
                    f"{route_text}"
                )
                return False
            if attempt < NAV_GOAL_ATTEMPTS:
                time.sleep(0.3)
        return False

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
        return self._send_pick_direction_and_wait(side).completed

    def _send_pick_direction_and_wait(self, side: str) -> _CallbackWaitResult:
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        status = (
            StateMachineCommand.PICK_DIRECTION_LEFT
            if side == "left"
            else StateMachineCommand.PICK_DIRECTION_RIGHT
        )
        return self._send_command_and_wait_result(
            StateMachineCommand.TASK_PICK_DIRECTION,
            status,
            ExecutionCallback.CALLBACK_PICK_DIRECTION,
            recover_on_error=self._current_area not in {"B", "C"},
        )

    def execute_closed_loop_pick(
        self,
        side: str,
        expected_type: Optional[str] = None,
    ) -> PickExecutionResult:
        """Use D435i for the expected type, then send direction and ArmTarget."""
        self.set_expected_fruit_type(expected_type)
        direction_result = self._send_pick_direction_and_wait(side)
        if not direction_result.completed:
            return PickExecutionResult(
                False,
                "side_pose_timeout",
                safe_to_continue=direction_result.safe_to_continue,
            )
        fresh_after_ns = self.node.get_clock().now().nanoseconds
        if not self.set_perception_mode("TREE"):
            return PickExecutionResult(False, "tree_mode_timeout")
        selection = self._wait_d435i_target(fresh_after_ns, expected_type)
        if selection.target is None:
            return PickExecutionResult(
                False,
                selection.reason_code,
                selection.matched_type,
            )
        arm_result = self._send_arm_target_and_wait(selection.target.point)
        if not arm_result.completed:
            return PickExecutionResult(
                False,
                arm_result.reason_code,
                selection.matched_type,
                arm_result.safe_to_continue,
            )
        return PickExecutionResult(True, "picked", selection.matched_type)

    def observe_b_tree(
        self,
        side: str,
    ) -> Dict[str, Any]:
        """Turn toward one B tree and return its stable perception targets."""
        direction_result = self._send_pick_direction_and_wait(side)
        if not direction_result.completed:
            return {
                "success": False,
                "reason_code": "side_pose_timeout",
                "safe_to_continue": direction_result.safe_to_continue,
                "targets": [],
            }
        if not self.set_perception_mode("TREE"):
            return {
                "success": False,
                "reason_code": "tree_mode_timeout",
                "targets": [],
            }
        targets = self._wait_b_targets(
            self.node.get_clock().now().nanoseconds,
        )
        if targets is None:
            return {
                "success": False,
                "reason_code": "d435i_target_timeout",
                "targets": [],
            }
        return {"success": True, "reason_code": "observed", "targets": targets}

    def execute_b_tree_targets(
        self,
        side: str,
        targets: Sequence[BTreeTarget],
    ) -> Dict[str, Any]:
        """Send stable B targets, returning to the same side between fruits."""
        picked = 0
        for index, target in enumerate(targets):
            arm_result = self._send_arm_target_and_wait(target.point)
            if not arm_result.completed:
                return {
                    "success": False,
                    "reason_code": arm_result.reason_code,
                    "safe_to_continue": arm_result.safe_to_continue,
                    "picked": picked,
                }
            picked += 1
            if index + 1 < len(targets) and not self.send_pick_direction(side):
                return {
                    "success": False,
                    "reason_code": "side_pose_timeout",
                    "picked": picked,
                }
        return {"success": True, "reason_code": "picked", "picked": picked}

    def set_expected_fruit_type(self, expected_type: Optional[str]) -> None:
        """Set the D435i target class; an empty value disables filtering."""
        message = String()
        message.data = str(expected_type or "")
        self._expected_fruit_type_publisher.publish(message)

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
            recover_on_error=False,
        )

    def send_return_initial_pose(self) -> bool:
        """Ask the lower controller to return to the initial pose."""
        return self._send_command_and_wait(
            StateMachineCommand.TASK_RETURN_INITIAL_POSE,
            StateMachineCommand.RETURN_INITIAL_POSE,
            ExecutionCallback.CALLBACK_RETURN_INITIAL_POSE,
            recover_on_error=False,
        )

    def report_current_area(self, area: str) -> bool:
        """Report current competition area to the lower controller."""
        self._current_area = area
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

    def save_b_plan(self, plan: Mapping[str, Any]) -> None:
        """Write the B QR plan so the later full-flow phase can be inspected."""
        output_dir = Path(str(self.config["c_distribution_output_dir"]))
        output_dir.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(plan, ensure_ascii=False, indent=2)
        (output_dir / "b_plan_latest.json").write_text(
            encoded + "\n", encoding="utf-8"
        )

    def _send_command_and_wait(
        self,
        task_id: int,
        task_status: int,
        callback_id: int,
        recover_on_error: bool = True,
    ) -> bool:
        return self._send_command_and_wait_result(
            task_id,
            task_status,
            callback_id,
            recover_on_error,
        ).completed

    def _send_command_and_wait_result(
        self,
        task_id: int,
        task_status: int,
        callback_id: int,
        recover_on_error: bool = True,
    ) -> _CallbackWaitResult:
        if not self._wait_for_hardware_bridge_endpoints(
            str(self.config["state_command_topic"])
        ):
            return _CallbackWaitResult(False, "endpoints_not_ready", False)
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
            recover_on_error,
        )

    def _send_arm_target_and_wait(self, point: PointStamped) -> _CallbackWaitResult:
        callback_id = ExecutionCallback.CALLBACK_ARM_TARGET
        if not self._wait_for_hardware_bridge_endpoints(
            str(self.config["arm_target_topic"])
        ):
            return _CallbackWaitResult(False, "arm_endpoints_not_ready", False)
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
            self._current_area not in {"B", "C"},
        )

    def _wait_for_hardware_bridge_endpoints(self, outbound_topic: str) -> bool:
        timeout_sec = float(self.config["hardware_endpoint_timeout_sec"])
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
        recover_on_error: bool,
    ) -> _CallbackWaitResult:
        success = self._wait_until(
            self._callback_event.is_set,
            float(self.config["callback_timeout_sec"]),
        )
        self._pending_callback_id = None
        if not success:
            self.node.get_logger().error(
                f"通信超时：{timeout_label} 在 "
                f"{float(self.config['callback_timeout_sec']):.1f}s 内没有终态回调"
            )
            recovered = (
                self._recover_safe_pose_after_error(callback_id)
                if recover_on_error
                else False
            )
            return _CallbackWaitResult(False, "arm_callback_timeout", recovered)

        status = self._matched_callback_status
        self._matched_callback_status = None
        if status == ExecutionCallback.STATUS_COMPLETED:
            return _CallbackWaitResult(True, "completed", True)
        if status == ExecutionCallback.STATUS_FAILED:
            self.node.get_logger().error(
                f"下位机执行失败：{timeout_label} 返回 callback_status=2"
            )
            recovered = (
                self._recover_safe_pose_after_error(callback_id)
                if recover_on_error
                else False
            )
            return _CallbackWaitResult(False, "arm_callback_failed", recovered)

        self.node.get_logger().warning(
            f"Ignoring unexpected callback_status={status} for {timeout_label}"
        )
        return _CallbackWaitResult(False, "unexpected_callback_status", False)

    def _recover_safe_pose_after_error(self, failed_callback_id: int) -> bool:
        use_retract_pose = self._current_area in {"B", "C"}
        task_id = (
            StateMachineCommand.TASK_ARM_RETRACT_POSE
            if use_retract_pose
            else StateMachineCommand.TASK_RETURN_INITIAL_POSE
        )
        task_status = (
            StateMachineCommand.ARM_RETRACT_POSE
            if use_retract_pose
            else StateMachineCommand.RETURN_INITIAL_POSE
        )
        callback_id = (
            ExecutionCallback.CALLBACK_ARM_RETRACT_POSE
            if use_retract_pose
            else ExecutionCallback.CALLBACK_RETURN_INITIAL_POSE
        )
        self.node.get_logger().info(
            f"callback_id={failed_callback_id} 异常，发送安全姿态 task_id={task_id}"
        )
        recovered = self._send_command_and_wait(
            task_id,
            task_status,
            callback_id,
            recover_on_error=False,
        )
        if not recovered:
            self.node.get_logger().error(
                f"安全姿态 task_id={task_id} 未完成，不能继续后续任务"
            )
        return recovered

    def _wait_d435i_target(
        self,
        fresh_after_ns: int,
        expected_type: Optional[str],
    ) -> _D435iSelection:
        self._latest_d435i_target = None
        self._latest_d435i_types = None
        self._latest_d435i_types_stamp_ns = 0
        seen_types: list[str] = []
        latest_stamp_ns = fresh_after_ns
        deadline = time.monotonic() + float(self.config["d435i_detection_timeout_sec"])
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._latest_d435i_types_stamp_ns > fresh_after_ns:
                latest_types = self._latest_d435i_types or []
                seen_types = merge_seen_types(seen_types, latest_types)
                if expected_type and latest_types and expected_type not in latest_types:
                    prefix = expected_type.rsplit("_", 1)[0]
                    unripe_type = f"{prefix}_unripe"
                    if unripe_type in latest_types:
                        return _D435iSelection(
                            "target_unripe",
                            matched_type=unripe_type,
                        )
                    return _D435iSelection(
                        "type_mismatch",
                        matched_type=latest_types[0],
                    )
            if self._latest_d435i_target is None:
                continue
            target = self._latest_d435i_target
            stamp = target.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns <= latest_stamp_ns:
                self._latest_d435i_target = None
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
            if expected_type is None or expected_type in seen_types:
                self._latest_d435i_target = None
                return _D435iSelection(
                    "matched",
                    _D435iTarget(target),
                    expected_type,
                )

        expected_seen = expected_type is not None and expected_type in seen_types
        matched_type = expected_type if expected_seen else (
            seen_types[0] if seen_types else None
        )
        if not seen_types:
            reason_code = "no_detection"
        elif expected_type and not expected_seen:
            prefix = expected_type.rsplit("_", 1)[0]
            if f"{prefix}_unripe" in seen_types:
                reason_code = "target_unripe"
            else:
                reason_code = "type_mismatch"
        else:
            reason_code = "d435i_target_timeout"
        return _D435iSelection(reason_code, matched_type=matched_type)

    def _wait_b_targets(
        self,
        fresh_after_ns: int,
    ) -> Optional[list[BTreeTarget]]:
        self._latest_b_targets = None
        self._latest_b_targets_stamp_ns = 0
        deadline = time.monotonic() + float(self.config["d435i_detection_timeout_sec"])
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._latest_b_targets is None:
                continue
            if self._latest_b_targets_stamp_ns <= fresh_after_ns:
                continue
            targets = []
            for item in self._latest_b_targets:
                if not bool(item.has_point):
                    continue
                point = PointStamped()
                point.header.stamp = self.node.get_clock().now().to_msg()
                point.point.x = float(item.point.x)
                point.point.y = float(item.point.y)
                point.point.z = float(item.point.z)
                coordinates = (point.point.x, point.point.y, point.point.z)
                if all(math.isfinite(value) for value in coordinates):
                    targets.append(BTreeTarget(str(item.type), point))
            return targets
        return None

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

    def _waypoint_yaw(self, name: str) -> float:
        pose = self.route.pose(name)
        return math.atan2(
            2.0 * (pose.qw * pose.qz + pose.qx * pose.qy),
            1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz),
        )

    def _pose_stamped(
        self,
        pose: WaypointPose,
        yaw: Optional[float] = None,
    ) -> PoseStamped:
        stamped = PoseStamped()
        stamped.header.frame_id = str(self.config["map_frame"])
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        stamped.pose.position.x = pose.x
        stamped.pose.position.y = pose.y
        stamped.pose.position.z = pose.z
        if yaw is None:
            stamped.pose.orientation.x = pose.qx
            stamped.pose.orientation.y = pose.qy
            stamped.pose.orientation.z = pose.qz
            stamped.pose.orientation.w = pose.qw
        else:
            stamped.pose.orientation.z = math.sin(yaw * 0.5)
            stamped.pose.orientation.w = math.cos(yaw * 0.5)
        return stamped

    def _on_active_mode(self, message: String) -> None:
        self._latest_active_mode = message.data
        self._active_mode_generation += 1

    def _on_decision(self, message: FruitDecision) -> None:
        if message.source == FruitDecision.SOURCE_LEFT:
            self._latest_left = list(message.types)
            return
        if message.source != FruitDecision.SOURCE_D435I:
            if message.source == FruitDecision.SOURCE_RIGHT:
                self._latest_right = list(message.types)
            return

        self._latest_d435i_types = list(message.types)
        stamp = message.header.stamp
        self._latest_d435i_types_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if message.targets_ready:
            self._latest_b_targets = list(message.targets)
            self._latest_b_targets_stamp_ns = self._latest_d435i_types_stamp_ns
        if message.has_target:
            target = PointStamped()
            target.header = message.header
            target.point.x = message.target_point.x
            target.point.y = message.target_point.y
            target.point.z = message.target_point.z
            self._latest_d435i_target = target

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
