#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
LABEL=${1:-robot}
INTERVAL_SEC=${2:-2.0}

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

exec -a fruit_robot_debug_monitor python3 - "$LABEL" "$INTERVAL_SEC" <<'PY'
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from realsense2_camera_msgs.msg import Metadata
from std_msgs.msg import Header, String

from fruit_perception.msg import FruitDecision


LABEL = sys.argv[1]
INTERVAL_SEC = max(0.5, float(sys.argv[2]))

BOLD = "\033[1m"
GREEN = "\033[32;1m"
YELLOW = "\033[33;1m"
RED = "\033[31;1m"
CYAN = "\033[36;1m"
MAGENTA = "\033[35;1m"
RESET = "\033[0m"

GICP_STATUS_NAME = "pb_gicp_localization/registration"
CAM_TYPES_MIN_INTERVAL_SEC = 2.0
D435I_DETECTIONS_MIN_INTERVAL_SEC = 1.5
D435I_TARGET_MIN_INTERVAL_SEC = 1.5
D435I_TARGET_MIN_DELTA_M = 0.04
D435I_STREAM_STALE_SEC = 1.0


def log(message):
    print(f"[{LABEL}][robot-debug] {message}", flush=True)


def highlight(color, text):
    return f"{color}{text}{RESET}"


def diagnostic_level_value(level):
    if isinstance(level, int):
        return level
    if isinstance(level, bytes):
        return level[0] if level else -1
    if isinstance(level, str):
        return ord(level) if len(level) == 1 else -1
    return -1


def level_name(level):
    value = diagnostic_level_value(level)
    if value == diagnostic_level_value(DiagnosticStatus.OK):
        return "OK"
    if value == diagnostic_level_value(DiagnosticStatus.WARN):
        return "WARN"
    if value == diagnostic_level_value(DiagnosticStatus.ERROR):
        return "ERROR"
    if value == diagnostic_level_value(DiagnosticStatus.STALE):
        return "STALE"
    return str(value)


def ros_log_level_value(level):
    if isinstance(level, int):
        return level
    if isinstance(level, bytes):
        return level[0] if level else -1
    if isinstance(level, str):
        return ord(level) if len(level) == 1 else -1
    return -1


def status_name(value):
    names = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }
    return names.get(int(value), str(value))


def values_dict(status):
    return {item.key: item.value for item in status.values}


def value(values, key):
    return values.get(key, "-")


def format_types(types):
    return ",".join(types) if types else "NONE"


class RobotDebugMonitor:
    def __init__(self, node):
        self.node = node
        self.last_gicp_diagnostic_time = None
        self.last_gicp_signature = None
        self.gicp_accepted_once = False
        self.last_graph_log_time = 0.0
        self.last_health_signature = None
        self.last_nav_signature = None
        self.last_nav_latest_status = None
        self.last_mode = {}
        self.last_perception_signature = {}
        self.last_perception_log_time = {}
        self.last_target_signature = None
        self.last_target_log_time = 0.0
        now = time.monotonic()
        self.d435i_streams = {
            name: {
                "count": 0,
                "window_started": now,
                "last_received": None,
                "last_stamp_ns": None,
            }
            for name in ("color", "depth", "rgbd")
        }

        volatile_qos = QoSProfile(depth=10)
        volatile_qos.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, volatile_qos)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, latched_qos)
        self.node.create_subscription(GoalStatusArray, "/follow_waypoints/_action/status", self.on_follow_status, 10)
        self.node.create_subscription(Log, "/rosout", self.on_rosout, 100)

        self.node.create_subscription(String, "/fruit_mission/perception_mode", lambda msg: self.on_mode("requested", msg), 10)
        self.node.create_subscription(String, "/fruit_perception/active_mode", lambda msg: self.on_mode("active", msg), latched_qos)
        self.node.create_subscription(FruitDecision, "/fruit_perception/decision", self.on_decision, 100)
        self.node.create_subscription(
            Metadata,
            "/camera/camera/color/metadata",
            lambda msg: self.on_d435i_frame("color", msg.header),
            sensor_qos,
        )
        self.node.create_subscription(
            Metadata,
            "/camera/camera/depth/metadata",
            lambda msg: self.on_d435i_frame("depth", msg.header),
            sensor_qos,
        )
        self.node.create_subscription(
            Header,
            "/fruit_perception/d435i/rgbd_heartbeat",
            lambda msg: self.on_d435i_frame("rgbd", msg),
            sensor_qos,
        )

        self.node.create_timer(INTERVAL_SEC, self.on_timer)

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != GICP_STATUS_NAME:
                continue
            self.last_gicp_diagnostic_time = time.monotonic()
            values = values_dict(status)
            level = level_name(status.level)
            accepted_once = (
                level == "OK" and
                value(values, "accepted") == "1" and
                value(values, "has_accepted_once") == "1"
            )
            signature = (
                level,
                status.message,
                value(values, "accepted"),
                value(values, "has_accepted_once"),
                value(values, "consecutive_rejects"),
                value(values, "source_points"),
                value(values, "inliers"),
                value(values, "inlier_ratio"),
                value(values, "mean_error"),
                value(values, "translation_jump"),
                value(values, "rotation_jump"),
            )
            if accepted_once and self.gicp_accepted_once:
                return
            if signature == self.last_gicp_signature:
                return
            self.last_gicp_signature = signature
            if accepted_once:
                self.gicp_accepted_once = True
            color = GREEN if level == "OK" else RED if level == "ERROR" else YELLOW
            log(
                highlight(color, "GICP") + " "
                f"level={level} message={status.message!r} "
                f"accepted={value(values, 'accepted')} "
                f"has_accepted_once={value(values, 'has_accepted_once')} "
                f"rejects={value(values, 'consecutive_rejects')} "
                f"source_points={value(values, 'source_points')} "
                f"inliers={value(values, 'inliers')} "
                f"inlier_ratio={value(values, 'inlier_ratio')} "
                f"mean_error={value(values, 'mean_error')} "
                f"translation_jump={value(values, 'translation_jump')} "
                f"rotation_jump={value(values, 'rotation_jump')}"
            )
            log(
                highlight(CYAN, "GICP thresholds") + " "
                f"min_inliers={value(values, 'min_inliers')} "
                f"min_inlier_ratio={value(values, 'min_inlier_ratio_threshold')} "
                f"max_mean_error={value(values, 'max_mean_error_threshold')} "
                f"max_translation_jump={value(values, 'max_translation_jump_threshold')} "
                f"max_rotation_jump={value(values, 'max_rotation_jump_threshold')} "
                f"initial_max_translation_jump={value(values, 'max_translation_jump_initial_threshold')} "
                f"initial_max_rotation_jump={value(values, 'max_rotation_jump_initial_threshold')}"
            )

    def on_follow_status(self, message):
        statuses = tuple(status_name(item.status) for item in message.status_list)
        latest = statuses[-1] if statuses else "NONE"
        active = [item for item in statuses if item in ("ACCEPTED", "EXECUTING", "CANCELING")]
        signature = (latest, tuple(active), len(statuses))
        if signature == self.last_nav_signature:
            return
        previous = self.last_nav_latest_status
        self.last_nav_signature = signature
        self.last_nav_latest_status = latest
        if latest == "SUCCEEDED" and previous == "SUCCEEDED":
            return
        log(
            highlight(CYAN, "NAV action") +
            f" latest={latest} active={active if active else []} total_goals={len(statuses)}"
        )

    def on_rosout(self, message):
        level = ros_log_level_value(message.level)
        if level < ros_log_level_value(Log.WARN):
            return
        if message.name not in (
            "axis_waypoint_action_server",
            "fruit_mission",
            "pb_gicp_localization",
            "fruit_perception_node",
            "camera_mode_manager",
            "camera.camera",
            "left_camera",
            "right_camera",
        ):
            return
        color = RED if level >= ros_log_level_value(Log.ERROR) else YELLOW
        log(highlight(color, "ROSOUT") + f" {message.name}: {message.msg}")

    def on_mode(self, source, message):
        if self.last_mode.get(source) == message.data:
            return
        self.last_mode[source] = message.data
        log(highlight(MAGENTA, "PERCEPTION mode") + f" {source}={message.data}")

    def on_decision(self, message):
        camera = {
            FruitDecision.SOURCE_LEFT: "left",
            FruitDecision.SOURCE_RIGHT: "right",
            FruitDecision.SOURCE_D435I: "d435i",
        }.get(message.source, "unknown")
        self.on_types(camera, message)
        if message.source != FruitDecision.SOURCE_D435I:
            return
        self.on_d435i_detections(message)
        if message.has_target:
            self.on_target_point(message)

    def on_types(self, camera, message):
        now = time.monotonic()
        types = tuple(message.types)
        if not types:
            self.last_perception_signature[camera] = (camera, types)
            return
        signature = (camera, types)
        if self.last_perception_signature.get(camera) == signature:
            return
        last_time = self.last_perception_log_time.get(camera, 0.0)
        if now - last_time < CAM_TYPES_MIN_INTERVAL_SEC:
            self.last_perception_signature[camera] = signature
            return
        self.last_perception_signature[camera] = signature
        self.last_perception_log_time[camera] = now
        color = GREEN if types else YELLOW
        log(highlight(color, f"CAM {camera}") + f" types={format_types(types)} count={len(types)}")

    def on_d435i_detections(self, message):
        now = time.monotonic()
        detections = []
        for item in message.detections:
            point = (
                f"point=({item.point.x:.3f},{item.point.y:.3f},{item.point.z:.3f})"
                if item.has_point else "point=NONE"
            )
            detections.append(
                f"{item.type}@{item.confidence:.2f} "
                f"roi=({item.roi.x_offset},{item.roi.y_offset},{item.roi.width},{item.roi.height}) "
                f"{point}"
            )
        signature = tuple(detections)
        if self.last_perception_signature.get("d435i_detections") == signature:
            return
        previous = self.last_perception_signature.get("d435i_detections", ())
        previous_types = tuple(item.split("@", 1)[0] for item in previous)
        current_types = tuple(item.split("@", 1)[0] for item in signature)
        types_changed = current_types != previous_types
        if not types_changed and previous:
            self.last_perception_signature["d435i_detections"] = signature
            return
        last_time = self.last_perception_log_time.get("d435i_detections", 0.0)
        if now - last_time < D435I_DETECTIONS_MIN_INTERVAL_SEC:
            self.last_perception_signature["d435i_detections"] = signature
            return
        self.last_perception_signature["d435i_detections"] = signature
        self.last_perception_log_time["d435i_detections"] = now
        color = GREEN if detections else YELLOW
        log(highlight(color, "CAM d435i detections") + f" count={len(detections)} " + " | ".join(detections))

    def on_target_point(self, message):
        now = time.monotonic()
        signature = (
            round(float(message.target_point.x), 3),
            round(float(message.target_point.y), 3),
            round(float(message.target_point.z), 3),
        )
        if self.last_target_signature is not None:
            delta = math.sqrt(
                (signature[0] - self.last_target_signature[0]) ** 2 +
                (signature[1] - self.last_target_signature[1]) ** 2 +
                (signature[2] - self.last_target_signature[2]) ** 2
            )
        else:
            delta = float("inf")
        if (
            signature == self.last_target_signature or
            delta < D435I_TARGET_MIN_DELTA_M or
            now - self.last_target_log_time < D435I_TARGET_MIN_INTERVAL_SEC
        ):
            return
        self.last_target_signature = signature
        self.last_target_log_time = now
        log(
            highlight(GREEN, "CAM d435i target") +
            f" point=({signature[0]:.3f},{signature[1]:.3f},{signature[2]:.3f}) frame={message.header.frame_id}"
        )

    def on_d435i_frame(self, stream_name, header):
        state = self.d435i_streams[stream_name]
        state["count"] += 1
        state["last_received"] = time.monotonic()
        state["last_stamp_ns"] = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    @staticmethod
    def format_d435i_stream(state, now):
        elapsed = max(now - state["window_started"], 1e-6)
        hz = state["count"] / elapsed
        state["count"] = 0
        state["window_started"] = now
        if state["last_received"] is None:
            return "hz=0.0 last=NEVER age=INF", float("inf")
        age = max(0.0, now - state["last_received"])
        stamp_ns = state["last_stamp_ns"]
        if stamp_ns:
            seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
            stamp = time.strftime("%H:%M:%S", time.localtime(seconds))
            stamp = f"{stamp}.{nanoseconds // 1_000_000:03d}"
        else:
            stamp = "ZERO"
        return f"hz={hz:.1f} last={stamp} age={age:.2f}s", age

    def log_d435i_streams(self, now, active_mode):
        values = {}
        ages = {}
        for name, state in self.d435i_streams.items():
            values[name], ages[name] = self.format_d435i_stream(state, now)
        expected = active_mode in ("QR", "TREE")
        stale = expected and any(age > D435I_STREAM_STALE_SEC for age in ages.values())
        status = "STALE" if stale else "OK" if expected else "IDLE"
        color = RED if stale else GREEN if expected else CYAN
        log(
            highlight(color, "D435I streams") + " "
            f"status={status} mode={active_mode} "
            f"color[{values['color']}] depth[{values['depth']}] rgbd[{values['rgbd']}]"
        )

    def on_timer(self):
        now = time.monotonic()
        if now - self.last_graph_log_time < INTERVAL_SEC:
            return
        self.last_graph_log_time = now

        nodes = {name for name, _namespace in self.node.get_node_names_and_namespaces()}
        node_present = "pb_gicp_localization" in nodes
        fruit_perception_present = "fruit_perception_node" in nodes
        active_mode = self.last_mode.get("active", "-")
        requested_mode = self.last_mode.get("requested", "-")
        self.log_d435i_streams(now, active_mode)
        health_signature = (
            int(node_present),
            int(fruit_perception_present),
            self.node.count_publishers('/diagnostics'),
            self.node.count_publishers('/registered_scan'),
            self.node.count_subscribers('/registered_scan'),
            self.node.count_publishers('/follow_waypoints/_action/status'),
            requested_mode,
            active_mode,
            self.node.count_publishers('/fruit_perception/decision'),
        )
        if health_signature == self.last_health_signature:
            return
        self.last_health_signature = health_signature
        log(
            highlight(BOLD, "HEALTH") + " "
            f"gicp_node={health_signature[0]} "
            f"perception_node={health_signature[1]} "
            f"diag_pub={health_signature[2]} "
            f"registered_scan_pub={health_signature[3]} "
            f"registered_scan_sub={health_signature[4]} "
            f"follow_action_status_pub={health_signature[5]} "
            f"perception_requested={requested_mode} perception_active={active_mode} "
            f"decision_pub={health_signature[8]}"
        )


def main():
    rclpy.init(args=["--ros-args", "--log-level", "fatal"])
    safe_label = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in LABEL)
    node = rclpy.create_node(f"{safe_label}_robot_debug_monitor")
    log(
        "started; watching navigation, GICP, rosout warnings/errors, "
        "camera modes, D435i stream rates and unified fruit decisions"
    )
    RobotDebugMonitor(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
