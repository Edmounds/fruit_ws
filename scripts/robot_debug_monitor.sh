#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
LABEL=${1:-robot}
INTERVAL_SEC=${2:-2.0}

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
set -u

exec -a fruit_robot_debug_monitor python3 - "$LABEL" "$INTERVAL_SEC" <<'PY'
import json
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

CAM_TYPES_MIN_INTERVAL_SEC = 2.0

CAMERA_DETECTIONS_MIN_INTERVAL_SEC = 1.5
CAMERA_TARGET_MIN_INTERVAL_SEC = 1.5
CAMERA_TARGET_MIN_DELTA_M = 0.04
CAMERA_STREAM_STALE_SEC = 1.0


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
        self.last_graph_log_time = 0.0
        self.last_health_signature = None
        self.last_nav_signature = None
        self.last_nav_latest_status = None
        self.active_window_info = {
            "window_id": "-",
            "valid": 0,
            "area": "-",
            "purpose": "-",
            "expected": "",
        }
        self.last_perception_signature = {}
        self.last_perception_log_time = {}
        self.last_target_signature = None
        self.last_target_log_time = 0.0
        self.last_camera_stream_status = None
        now = time.monotonic()
        self.camera_streams = {
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

        self.node.create_subscription(GoalStatusArray, "/follow_waypoints/_action/status", self.on_follow_status, 10)
        self.node.create_subscription(Log, "/rosout", self.on_rosout, 100)

        self.node.create_subscription(String, "/fruit_mission/observation_window", self.on_observation_window, 10)
        self.node.create_subscription(FruitDecision, "/fruit_perception/decision", self.on_decision, 100)
        self.node.create_subscription(
            Metadata,
            "/camera/color/metadata",
            lambda msg: self.on_camera_frame("color", msg.header),
            sensor_qos,
        )
        self.node.create_subscription(
            Metadata,
            "/camera/depth/metadata",
            lambda msg: self.on_camera_frame("depth", msg.header),
            sensor_qos,
        )
        self.node.create_subscription(
            Header,
            "/fruit_perception/camera/rgbd_heartbeat",
            lambda msg: self.on_camera_frame("rgbd", msg),
            sensor_qos,
        )

        self.node.create_timer(INTERVAL_SEC, self.on_timer)

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
            "fruit_perception_node",
            "camera.camera",
        ):
            return

        color = RED if level >= ros_log_level_value(Log.ERROR) else YELLOW
        log(highlight(color, "ROSOUT") + f" {message.name}: {message.msg}")

    def on_observation_window(self, message):
        try:
            cmd = json.loads(message.data)
            wid = str(cmd.get("window_id", "-"))
            valid = int(cmd.get("valid", 0))
            area = str(cmd.get("area", "-"))
            purpose = str(cmd.get("purpose", "-"))
            expected = str(cmd.get("expected_type", ""))
            self.active_window_info = {
                "window_id": wid,
                "valid": valid,
                "area": area,
                "purpose": purpose,
                "expected": expected,
            }
            state_str = "OPEN" if valid == 1 else "CLOSED"
            log(
                highlight(MAGENTA, "OBSERVATION window") +
                f" state={state_str} id={wid} area={area} purpose={purpose} expected='{expected}'"
            )
        except Exception:
            pass

    def on_decision(self, message):
        self.on_types("camera", message)
        self.on_camera_detections(message)
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

    def on_camera_detections(self, message):
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
        if self.last_perception_signature.get("camera_detections") == signature:
            return
        previous = self.last_perception_signature.get("camera_detections", ())
        previous_types = tuple(item.split("@", 1)[0] for item in previous)
        current_types = tuple(item.split("@", 1)[0] for item in signature)
        types_changed = current_types != previous_types
        if not types_changed and previous:
            self.last_perception_signature["camera_detections"] = signature
            return
        last_time = self.last_perception_log_time.get("camera_detections", 0.0)
        if now - last_time < CAMERA_DETECTIONS_MIN_INTERVAL_SEC:
            self.last_perception_signature["camera_detections"] = signature
            return
        self.last_perception_signature["camera_detections"] = signature
        self.last_perception_log_time["camera_detections"] = now
        color = GREEN if detections else YELLOW
        log(highlight(color, "CAM camera detections") + f" count={len(detections)} " + " | ".join(detections))

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
            delta < CAMERA_TARGET_MIN_DELTA_M or
            now - self.last_target_log_time < CAMERA_TARGET_MIN_INTERVAL_SEC
        ):
            return
        self.last_target_signature = signature
        self.last_target_log_time = now
        log(
            highlight(GREEN, "CAM camera target") +
            f" point=({signature[0]:.3f},{signature[1]:.3f},{signature[2]:.3f}) frame={message.header.frame_id}"
        )

    def on_camera_frame(self, stream_name, header):
        state = self.camera_streams[stream_name]
        state["count"] += 1
        state["last_received"] = time.monotonic()
        state["last_stamp_ns"] = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    @staticmethod
    def format_camera_stream(state, now):
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

    def log_camera_streams(self, now, perception_present):
        values = {}
        ages = {}
        for name, state in self.camera_streams.items():
            values[name], ages[name] = self.format_camera_stream(state, now)
        expected = bool(perception_present)
        stale = expected and any(age > CAMERA_STREAM_STALE_SEC for age in ages.values())
        status = "STALE" if stale else "OK" if expected else "IDLE"
        color = RED if stale else GREEN if expected else CYAN
        win = self.active_window_info
        win_str = f"{'OPEN' if win['valid'] else 'CLOSED'}:{win['area']}:{win['purpose']}"

        previous_status = self.last_camera_stream_status
        self.last_camera_stream_status = status
        if previous_status == "STALE" and status == "OK":
            log(highlight(GREEN, "CAMERA streams recovered") + f" window={win_str}")

        log(
            highlight(color, "CAMERA streams") + " "
            f"status={status} window={win_str} "
            f"color[{values['color']}] depth[{values['depth']}] rgbd[{values['rgbd']}]"
        )

    def on_timer(self):
        now = time.monotonic()
        if now - self.last_graph_log_time < INTERVAL_SEC:
            return
        self.last_graph_log_time = now

        nodes = {name for name, _namespace in self.node.get_node_names_and_namespaces()}
        fruit_perception_present = "fruit_perception_node" in nodes
        self.log_camera_streams(now, fruit_perception_present)
        win = self.active_window_info
        health_signature = (
            int(fruit_perception_present),
            self.node.count_publishers('/diagnostics'),
            self.node.count_publishers('/follow_waypoints/_action/status'),
            win["valid"],
            win["window_id"],
            self.node.count_publishers('/fruit_perception/decision'),
        )
        if health_signature == self.last_health_signature:
            return
        self.last_health_signature = health_signature
        log(
            highlight(BOLD, "HEALTH") + " "
            f"perception_node={health_signature[0]} "
            f"diag_pub={health_signature[1]} "
            f"follow_action_status_pub={health_signature[2]} "
            f"window_valid={win['valid']} window_id={win['window_id']} "
            f"decision_pub={health_signature[5]}"
        )


def main():
    rclpy.init(args=["--ros-args", "--log-level", "fatal"])
    safe_label = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in LABEL)
    node = rclpy.create_node(f"{safe_label}_robot_debug_monitor")
    log(
        "started; watching navigation, rosout warnings/errors, "
        "observation windows, Camera stream rates and unified fruit decisions"
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

