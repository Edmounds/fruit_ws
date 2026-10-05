"""ROS communication helpers used by the mission flow."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Union
import uuid

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

from fruit_perception.msg import FruitDecision

from fruit_mission.camera_target_planner_client import CameraTargetPlannerClient
from fruit_mission.camera_target_planner_client import DEFAULT_EXTRINSIC_CONFIG
from fruit_mission.config import _validate_qr_fallbacks
from fruit_mission.decisions import format_produce_count, merge_seen_types, spoken_produce_name
from fruit_mission.tts import RecordedSpeaker
from fruit_mission.waypoint_route import MissionWaypointRoute
from fruit_mission.waypoint_route import WaypointPose
from fruit_qr_planner.planner import build_b_plan, build_c_plan


NAV_GOAL_ATTEMPTS = 3


@dataclass(frozen=True)
class _CameraTarget:
    point: PointStamped


@dataclass(frozen=True)
class _CameraSelection:
    reason_code: str
    target: Optional[_CameraTarget] = None
    matched_type: Optional[str] = None


@dataclass(frozen=True)
class _ArmMotionResult:
    completed: bool
    reason_code: str


@dataclass(frozen=True)
class BTreeTarget:
    """One stable B-zone perception target retained for mission decisions."""

    model_type: str
    point: PointStamped
    pixel_x: float
    pixel_y: float
    image_width: int


class MissionContext:
    """High-level blocking wrappers around the existing ROS interfaces."""

    def __init__(self, node: Node, config: Mapping[str, Any], speaker=None) -> None:
        self.node = node
        self.config = config
        self.speaker = speaker or RecordedSpeaker(config.get("tts", {}), node.get_logger())
        self._pick_counts: Counter[str] = Counter()
        self.route = MissionWaypointRoute.from_file(
            Path(str(config["waypoint_file"])),
            str(config.get("mission_flow", "full")),
            str(config["map_frame"]),
        )
        self._route_cursor = 0
        self._current_waypoint_name: Optional[str] = None
        self._current_route_yaw: Optional[float] = None
        self._current_area: Optional[str] = None
        self._active_window_id: Optional[str] = None
        self._active_window_start_sec: int = 0
        self._active_window_start_nanosec: int = 0
        self._active_window_area: str = ""
        self._active_window_purpose: str = ""
        self._active_window_expected_type: str = ""
        self._latest_camera_types: Optional[list[str]] = None
        self._latest_camera_types_stamp_ns = 0
        self._latest_camera_target: Optional[PointStamped] = None
        self._latest_b_targets: Optional[list[Any]] = None
        self._latest_b_targets_header = None
        self._latest_b_targets_stamp_ns = 0
        self._latest_b_plan: Optional[Dict[str, Any]] = None
        self._latest_c_plan: Optional[Dict[str, Any]] = None
        self._pending_observation = None
        self._observation_side = None
        self._arm_home_confirmed = False
        self._arm_shenzhi_confirmed = False
        self._shenzhi_required = False
        self._navigation_stopped = True
        self._navigation_handle = None
        self._navigation_result = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
        )

        self._observation_window_publisher = node.create_publisher(
            String,
            config["observation_window_topic"],
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
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
        self._camera_planner = CameraTargetPlannerClient(
            node,
            config.get("camera_extrinsic_config", DEFAULT_EXTRINSIC_CONFIG),
            target_offset_m=config.get("target_offset_m", (0.0, 0.0, 0.0)),
        )
        self._follow_waypoints_client = ActionClient(
            node,
            FollowWaypoints,
            str(config["follow_waypoints_action"]),
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

    def navigate_named(
        self, waypoint_names: Sequence[str], observation_side: Optional[str] = None,
    ) -> bool:
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
        if observation_side is not None:
            self.start_observation(observation_side)
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
        self.start_observation(plan.camera_side)
        if not self._send_waypoint_names(
            plan.navigation_names,
            plan.navigation_yaws,
        ):
            return result

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
        if self._shenzhi_required or not self._navigation_stopped or not (
            ((self._arm_home_confirmed or self._arm_shenzhi_confirmed)
             and self._camera_planner.is_stationary(
                self.config['stationary']['joint_speed_rps'])) or (
                self._pending_observation is not None
                and self._pending_observation[1] is not None
            )
        ):
            self.node.get_logger().warning('Skipping navigation: safe motion handoff unconfirmed')
            return False
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
            try:
                future = self._follow_waypoints_client.send_goal_async(goal)
            except Exception as error:
                self.node.get_logger().warning(f'Navigation submission failed: {error}')
                return False
            self._navigation_stopped = False
            self._navigation_handle = None
            self._navigation_result = None
            if not self._wait_future(future):
                # A late accepted goal must not start after this step has been skipped.
                future.add_done_callback(self._cancel_late_navigation)
                return False
            try:
                candidate_handle = future.result()
            except Exception as error:
                self.node.get_logger().warning(f'Navigation response unknown: {error}')
                return False
            if not candidate_handle.accepted:
                self._navigation_stopped = True
                self.node.get_logger().warning(
                    f"FollowWaypoints goal rejected "
                    f"attempt={attempt}/{NAV_GOAL_ATTEMPTS}: {route_text}"
                )
                if attempt < NAV_GOAL_ATTEMPTS:
                    time.sleep(0.3)
                continue

            self._navigation_handle = candidate_handle
            result_future = candidate_handle.get_result_async()
            self._navigation_result = result_future
            result_future.add_done_callback(self._on_navigation_result)
            if not self._wait_future(result_future, watch_observation=True):
                if not self.cancel_navigation():
                    return False
                if self._observation_failed():
                    return False
                continue
            try:
                wrapped_result = result_future.result()
            except Exception as error:
                self.node.get_logger().warning(f'Navigation result unknown: {error}')
                self.cancel_navigation()
                return False
            result = wrapped_result.result
            missed = list(result.missed_waypoints)
            self._on_navigation_result(result_future)
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

    def _on_navigation_result(self, future) -> None:
        if future is not self._navigation_result:
            return
        try:
            result = future.result()
            self._navigation_stopped = result.status in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            )
        except Exception as error:
            self.node.get_logger().warning(f'Cannot confirm navigation standstill: {error}')

    def _cancel_late_navigation(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self._navigation_stopped = True
                return
            self._navigation_handle = handle
            self._navigation_result = handle.get_result_async()
            self._navigation_result.add_done_callback(self._on_navigation_result)
            handle.cancel_goal_async()
        except Exception as error:
            self.node.get_logger().warning(f'Cannot cancel late navigation: {error}')

    def cancel_navigation(self) -> bool:
        if self._navigation_stopped:
            return True
        if self._navigation_handle is None:
            return False
        try:
            self._navigation_handle.cancel_goal_async()
        except Exception as error:
            self.node.get_logger().warning(f'Navigation cancel failed: {error}')
            return False
        return self._wait_until(lambda: self._navigation_stopped, 10.0)

    def open_observation_window(
        self,
        area: str,
        purpose: str,
        expected_type: Optional[str] = None,
    ) -> str:
        """Open a new observation window and broadcast its parameters."""
        now = self.node.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        window_id = str(uuid.uuid4())
        self._active_window_id = window_id
        self._active_window_start_sec = sec
        self._active_window_start_nanosec = nanosec
        self._active_window_area = area
        self._active_window_purpose = purpose
        self._active_window_expected_type = str(expected_type or "")
        self._latest_camera_target = None
        self._latest_camera_types = None
        self._latest_camera_types_stamp_ns = 0
        self._latest_b_targets = None
        self._latest_b_targets_header = None
        self._latest_b_targets_stamp_ns = 0
        if purpose == "qr":
            if area == "B":
                self._latest_b_plan = None
            elif area == "C":
                self._latest_c_plan = None

        cmd = {
            "window_id": window_id,
            "valid": 1,
            "start_sec": sec,
            "start_nanosec": nanosec,
            "area": area,
            "purpose": purpose,
            "expected_type": str(expected_type or ""),
        }
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
        self._observation_window_publisher.publish(msg)
        self.node.get_logger().info(
            f"Opened observation window '{window_id}' for area={area}, "
            f"purpose={purpose}, expected='{expected_type or ''}'"
        )
        return window_id

    def close_observation_window(self) -> None:
        """Close current observation window if open."""
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
        self._observation_window_publisher.publish(msg)
        self.node.get_logger().info(f"Closed observation window '{window_id}'")

    def wait_b_plan(self, window_id: str) -> Optional[Dict[str, Any]]:
        """Wait for the B QR plan matching current window."""
        self._wait_until(
            lambda: self._latest_b_plan is not None
            and self._latest_b_plan.get("observation_window_id") == window_id,
            float(self.config["qr_plan_timeout_sec"]),
        )
        if (
            self._latest_b_plan is not None
            and self._latest_b_plan.get("observation_window_id") == window_id
        ):
            return self._latest_b_plan
        return None

    def wait_c_plan(self, window_id: str) -> Optional[Dict[str, Any]]:
        """Wait for the C QR plan matching current window."""
        self._wait_until(
            lambda: self._latest_c_plan is not None
            and self._latest_c_plan.get("observation_window_id") == window_id,
            float(self.config["qr_plan_timeout_sec"]),
        )
        if (
            self._latest_c_plan is not None
            and self._latest_c_plan.get("observation_window_id") == window_id
        ):
            return self._latest_c_plan
        return None

    def finish_observation(self, side: str) -> _ArmMotionResult:
        """Consume the matching Look request, open gripper, and wait for hardware completion."""
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if self._pending_observation is None:
            self.start_observation(side)
        pending_side, future = self._pending_observation
        self._pending_observation = None
        if pending_side != side:
            raise ValueError('Observation side must match the navigation plan')
        completed, reason = self._camera_planner.finish_motion(future, self._wait_until)
        if not completed:
            self._observation_side = None
            return _ArmMotionResult(False, reason)
        self._observation_side = side
        gripper_result = self.open_gripper()
        if not gripper_result.completed:
            self._observation_side = None
            return gripper_result
        return _ArmMotionResult(True, 'completed')

    def open_gripper(self) -> _ArmMotionResult:
        """OPEN at measured raw joints, then HOLD for the configured gripper dwell."""
        self._arm_home_confirmed = False
        self._arm_shenzhi_confirmed = False
        completed, reason = self._camera_planner.open_gripper(
            self._wait_until, self.config['stationary']['joint_speed_rps'])
        return _ArmMotionResult(completed, reason)

    def start_observation(self, side: str) -> None:
        """Look and navigation share the executor, not blocking worker threads."""
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if self._current_area not in ('A', 'B', 'C'):
            raise ValueError('Observation requires an A/B/C area')
        if self._pending_observation is not None:
            raise RuntimeError('Previous observation has not been consumed')
        future = None
        if (not self._shenzhi_required
                and (self._arm_home_confirmed or self._arm_shenzhi_confirmed
                     or self.recover_home())
                and self._camera_planner.is_stationary(
                    self.config['stationary']['joint_speed_rps'])):
            name = f'{side}_look_{self._current_area.lower()}'
            future = self._camera_planner.start_preset(name)
            if future is not None:
                self._arm_home_confirmed = False
                self._arm_shenzhi_confirmed = False
        self._pending_observation = (side, future)

    def _observation_failed(self) -> bool:
        if self._pending_observation is None:
            return False
        _side, future = self._pending_observation
        if future is None:
            return False  # No motion submitted; navigation can still reach this point.
        if future.cancelled():
            return True
        if not future.done():
            return False
        try:
            response = future.result()
            return response is None or not response.success
        except Exception:
            return True

    def camera_target_request(self, point: PointStamped):
        """Freeze a camera point using the shared measured-joint transform."""
        return self._camera_planner.target_request(point)

    def move_to_base_target(self, request) -> _ArmMotionResult:
        """One planner CLOSE/dwell request, starting only with stationary feedback."""
        if not self._camera_planner.is_stationary(self.config['stationary']['joint_speed_rps']):
            return _ArmMotionResult(False, 'arm_pose_unavailable')
        self._arm_home_confirmed = False
        self._arm_shenzhi_confirmed = False
        completed, reason = self._camera_planner.move_to_base_target(
            request, self._wait_until, self.config['stationary']['joint_speed_rps'])
        return _ArmMotionResult(completed, reason)

    def set_expected_fruit_type(self, expected_type: Optional[str]) -> None:
        """Set the Camera target class; an empty value disables filtering."""
        message = String()
        message.data = str(expected_type or "")
        self._expected_fruit_type_publisher.publish(message)

    def require_shenzhi(self) -> None:
        """A planned protection boundary cannot be satisfied by recovery Home."""
        self._shenzhi_required = True

    def send_return_initial_pose(self) -> bool:
        """Execute the initial-pose recovery preset."""
        return self.execute_arm_preset("home").completed

    def recover_home(self) -> bool:
        """One recovery, not a retry loop; unsafe physical motion stays inhibited."""
        self._pending_observation = None
        # Do not automatically clear a hardware emergency lock with STOP.
        status = self._camera_planner._status
        if status is not None and status.task_status == status.STATUS_EMERGENCY:
            self.cancel_navigation()
            return False
        stop_future = None
        stationary = self._camera_planner.is_stationary(
            self.config['stationary']['joint_speed_rps'])
        self._arm_home_confirmed = self._arm_home_confirmed and stationary
        motion_future = self._camera_planner._motion_future
        if not self._arm_home_confirmed and (
                (motion_future is not None and not motion_future.done())
                or not stationary):
            stop_future = self._camera_planner.request_stop()
        if not self.cancel_navigation():
            return False
        if self._arm_home_confirmed:
            return True
        if stop_future is not None:
            try:
                if not self._wait_until(stop_future.done, 10.0):
                    return False
                response = stop_future.result()
                if response is None or not response.success:
                    return False
            except Exception as error:
                self.node.get_logger().warning(f'Cannot confirm STOP dispatch: {error}')
                return False
        return self.send_return_initial_pose()

    def report_current_area(self, area: str) -> bool:
        """Publish mission area for perception/QR; no lower-controller policy."""
        if area not in {"A", "B", "C"}:
            raise ValueError("area must be A, B or C")
        self._current_area = area
        message = String()
        message.data = area
        self._current_area_publisher.publish(message)
        return True

    def record_pick(self, matched_type: Optional[str]) -> None:
        """Record one confirmed harvested produce target."""
        name = spoken_produce_name(matched_type)
        if name and name != "未知果实":
            self._pick_counts[name] += 1
            self.node.get_logger().info(f"记录采摘工序完成：{name}，当前累计：{self._pick_counts[name]}")

    def announce_summary(self) -> None:
        """Speak final mission summary by produce category directly."""
        self.announce_and_wait("任务流程结束")
        total = sum(self._pick_counts.values())
        if total == 0:
            self.announce_and_wait("本次没有采摘记录")
            return

        self.announce_and_wait("本次采摘记录")
        ordered_produces = ["苹果", "梨子", "南瓜", "西红柿", "辣椒", "洋葱"]
        for produce in ordered_produces:
            count = self._pick_counts.get(produce, 0)
            if count > 0:
                phrase = format_produce_count(produce, count)
                self.announce_and_wait(phrase)

    def announce(self, text: str, **_kwargs) -> bool:
        """Speak text through the configured TTS backend."""
        self.node.get_logger().info(f"播报：{text}")
        return bool(self.speaker.announce(text))

    def announce_and_wait(
        self,
        text: str,
        lead_time_sec: float = 0.0,
        **_kwargs,
    ) -> bool:
        """Speak text and wait until playback completes or reaches its lead time."""
        self.node.get_logger().info(f"同步播报：{text}")
        announce_and_wait = getattr(self.speaker, "announce_and_wait", None)
        if callable(announce_and_wait):
            return bool(announce_and_wait(text, lead_time_sec=lead_time_sec))
        return bool(self.speaker.announce(text))

    def close(self) -> None:
        """Release mission runtime helpers."""
        self.close_observation_window()
        self._camera_planner.request_stop()
        self.cancel_navigation()
        # Drain the STOP service request before tearing down its ROS client.
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        close = getattr(self.speaker, "close", None)
        if callable(close):
            close(wait=False)

    def save_c_distribution(self, distribution: Mapping[str, Any]) -> None:
        """Write the latest C-zone position table for inspection."""
        output_dir = Path(str(self.config["c_distribution_output_dir"]))
        encoded = json.dumps(distribution, ensure_ascii=False, indent=2)
        latest_path = output_dir / "c_distribution_latest.json"
        stamp = time.strftime("%Y%m%d_%H%M%S")
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            latest_path.write_text(encoded + '\n', encoding='utf-8')
            (output_dir / f'c_distribution_{stamp}.json').write_text(
                encoded + '\n', encoding='utf-8')
        except OSError as error:
            self.node.get_logger().warning(f'Cannot save C task record: {error}')

    def save_b_plan(self, plan: Mapping[str, Any]) -> None:
        """Write the B QR plan so the later full-flow phase can be inspected."""
        output_dir = Path(str(self.config["c_distribution_output_dir"]))
        encoded = json.dumps(plan, ensure_ascii=False, indent=2)
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / 'b_plan_latest.json').write_text(encoded + '\n', encoding='utf-8')
        except OSError as error:
            self.node.get_logger().warning(f'Cannot save B plan record: {error}')

    def execute_arm_preset(self, preset_name: str) -> _ArmMotionResult:
        """Execute a preset directly from arm_presets.yaml and wait for completion."""
        if (not self._navigation_stopped or not self._camera_planner.is_stationary(
                self.config['stationary']['joint_speed_rps'])):
            self._arm_home_confirmed = False
            self._arm_shenzhi_confirmed = False
            return _ArmMotionResult(False, 'arm_pose_unavailable')
        self._arm_home_confirmed = False
        self._arm_shenzhi_confirmed = False
        completed, reason = self._camera_planner.execute_preset(preset_name, self._wait_until)
        self._arm_home_confirmed = preset_name == 'home' and completed
        self._arm_shenzhi_confirmed = preset_name == 'shenzhi' and completed
        if self._arm_shenzhi_confirmed:
            self._shenzhi_required = False
        return _ArmMotionResult(completed, reason)

    def wait_camera_target(
        self,
        window_id: str,
        expected_type: Optional[str],
    ) -> _CameraSelection:
        self._latest_camera_target = None
        self._latest_camera_types = None
        self._latest_camera_types_stamp_ns = 0
        seen_types: list[str] = []
        deadline = time.monotonic() + float(self.config["camera_detection_timeout_sec"])
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._active_window_id != window_id:
                break
            latest_types = self._latest_camera_types or []
            if latest_types:
                seen_types = merge_seen_types(seen_types, latest_types)
            if self._latest_camera_target is None:
                continue
            target = self._latest_camera_target
            point = (
                float(target.point.x),
                float(target.point.y),
                float(target.point.z),
            )
            if not all(math.isfinite(value) for value in point):
                self._latest_camera_target = None
                continue
            self.node.get_logger().info(
                "Camera target received from perception window: "
                f"x={point[0]}, y={point[1]}, z={point[2]}"
            )
            if expected_type is None or expected_type in seen_types:
                self._latest_camera_target = None
                return _CameraSelection(
                    "matched",
                    _CameraTarget(target),
                    expected_type,
                )

        expected_seen = expected_type is not None and expected_type in seen_types
        matched_type = expected_type if expected_seen else (
            seen_types[0] if seen_types else None
        )
        if not seen_types:
            reason_code = "no_detection"
        elif expected_type and not expected_seen:
            if expected_type == "ripe":
                unripe_label = "unripe"
            else:
                unripe_label = f"{expected_type.rsplit('_', 1)[0]}_unripe"
            if unripe_label in seen_types:
                reason_code = "target_unripe"
            else:
                reason_code = "type_mismatch"
        else:
            reason_code = "camera_target_timeout"
        return _CameraSelection(reason_code, matched_type=matched_type)

    def wait_b_targets(
        self,
        window_id: str,
    ) -> Optional[list[BTreeTarget]]:
        self._latest_b_targets = None
        self._latest_b_targets_header = None
        self._latest_b_targets_stamp_ns = 0
        deadline = time.monotonic() + float(self.config["camera_detection_timeout_sec"])
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._active_window_id != window_id:
                break
            if self._latest_b_targets is None:
                continue
            targets = []
            for item in self._latest_b_targets:
                if not bool(item.has_point):
                    continue
                point = PointStamped()
                point.header = self._latest_b_targets_header
                point.point.x = float(item.point.x)
                point.point.y = float(item.point.y)
                point.point.z = float(item.point.z)
                coordinates = (
                    point.point.x,
                    point.point.y,
                    point.point.z,
                    float(item.pixel_x),
                    float(item.pixel_y),
                )
                if (
                    int(item.image_width) > 0
                    and all(math.isfinite(value) for value in coordinates)
                ):
                    targets.append(
                        BTreeTarget(
                            str(item.type),
                            point,
                            float(item.pixel_x),
                            float(item.pixel_y),
                            int(item.image_width),
                        )
                    )
            return targets
        return None

    def _wait_until(self, predicate, timeout_sec: Optional[float] = None) -> bool:
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return predicate()

    def _wait_future(self, future, watch_observation=False) -> bool:
        ready = self._wait_until(
            lambda: future.done() or (watch_observation and self._observation_failed()),
            None if watch_observation else 10.0,
        )
        return ready and future.done()

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

    def _on_decision(self, message: FruitDecision) -> None:
        if (
            self._active_window_id is None
            or message.observation_window_id != self._active_window_id
            or self._active_window_purpose != "harvest"
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
        self._latest_camera_types = list(message.types)
        self._latest_camera_types_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if message.targets_ready:
            self._latest_b_targets = list(message.targets)
            self._latest_b_targets_header = message.header
            self._latest_b_targets_stamp_ns = self._latest_camera_types_stamp_ns
        if message.has_target:
            target = PointStamped()
            target.header = message.header
            target.point.x = message.target_point.x
            target.point.y = message.target_point.y
            target.point.z = message.target_point.z
            self._latest_camera_target = target

    def _on_b_plan(self, message: String) -> None:
        plan = self._receive_plan(message, 'B')
        if plan is not None:
            self._latest_b_plan = plan

    def _on_c_plan(self, message: String) -> None:
        plan = self._receive_plan(message, 'C')
        if plan is not None:
            self._latest_c_plan = plan

    def _receive_plan(self, message: String, zone: str):
        """Reject invalid ROS input without throwing out of the executor callback."""
        try:
            if self._active_window_id is None:
                return None
            if self._active_window_purpose != "qr":
                return None
            if self._active_window_area != zone:
                return None
            plan = json.loads(message.data)
            if not isinstance(plan, dict):
                self.node.get_logger().warning(f"Non-object {zone} QR plan ignored")
                return None
            plan_window_id = plan.get("observation_window_id")
            if plan_window_id != self._active_window_id:
                return None
            img_sec = plan.get("image_stamp_sec")
            img_nanosec = plan.get("image_stamp_nanosec")
            if (
                isinstance(img_sec, bool)
                or not isinstance(img_sec, int)
                or isinstance(img_nanosec, bool)
                or not isinstance(img_nanosec, int)
                or not (0 <= img_nanosec < 1_000_000_000)
                or not 0 <= img_sec <= 0x7FFFFFFF
            ):
                self.node.get_logger().warning(
                    f"Invalid timestamp in {zone} QR plan: sec={img_sec}, nanosec={img_nanosec}"
                )
                return None
            if (
                img_sec < self._active_window_start_sec
                or (
                    img_sec == self._active_window_start_sec
                    and img_nanosec < self._active_window_start_nanosec
                )
            ):
                return None
            if plan.get("zone") != zone:
                raise ValueError("QR plan zone mismatch")
            tasks = sorted(plan["tasks"], key=lambda task: task["sequence"])
            if [task["sequence"] for task in tasks] != list(range(1, 9)):
                raise ValueError("QR plan requires eight ordered tasks")
            fallback = dict(self.config["qr_fallback"])
            names = [task["target"] for task in tasks]
            positions = [task["position_id"] for task in tasks]
            if zone == "B":
                if positions != list(range(1, 9)):
                    raise ValueError("B tree positions must remain ordered")
                fallback["b_targets"] = names
            else:
                fallback["c_targets"], fallback["c_positions"] = names, positions
            _validate_qr_fallbacks(fallback)
            if zone == "B":
                built = build_b_plan("\n".join(names))
            else:
                built = build_c_plan("\n".join(names + [",".join(map(str, positions))]))
            built["observation_window_id"] = plan_window_id
            built["image_stamp_sec"] = img_sec
            built["image_stamp_nanosec"] = img_nanosec
            return built
        except (ValueError, TypeError, KeyError) as error:
            self.node.get_logger().warning(f"Invalid {zone} QR plan ignored: {error}")
            return None


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
