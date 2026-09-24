#!/usr/bin/env python3
"""ROS 2 interactive console for the fruit robot lower controller via auto_serial_bridge.

This utility communicates with the lower controller over auto_serial_bridge topics:
  - TX /cmd_vel (geometry_msgs/msg/Twist)
  - TX /fruit_hardware_bridge/arm/trajectory_header (fruit_hardware_interfaces/msg/TrajectoryHeader)
  - TX /fruit_hardware_bridge/arm/trajectory_point (fruit_hardware_interfaces/msg/TrajectoryPoint)
  - TX /fruit_hardware_bridge/arm/control_mode (fruit_hardware_interfaces/msg/ArmControlMode)
  - RX /fruit_hardware_bridge/arm/status (fruit_hardware_interfaces/msg/ArmStatus)

If auto_serial_bridge is not already running, it automatically spawns it in the background
and cleanly shuts it down on exit.
"""

from __future__ import annotations

import argparse
import math
import os
import readline  # Enable interactive terminal line editing & history
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from fruit_hardware_interfaces.msg import (
        ArmControlMode,
        ArmStatus,
        TrajectoryHeader,
        TrajectoryPoint,
    )
except ImportError as exc:
    print(
        f"ROS 2 environment error: {exc}\n"
        "Please ensure the workspace runtime environment is sourced:\n"
        "  source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash",
        file=sys.stderr,
    )
    sys.exit(1)

ANSI_CODES = {
    "blue": "\033[34m",
    "cyan": "\033[36m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "red": "\033[31m",
    "dim": "\033[2m",
    "bold": "\033[1m",
}
ANSI_RESET = "\033[0m"

DEFAULT_PROTOCOL_PATH = (
    Path(__file__).resolve().parent
    / "src"
    / "auto_serial_bridge"
    / "config"
    / "protocol.yaml"
)

# 4-DOF joint software safety limits (in degrees) from firmware calibration
Q1_LIMIT_DEG = (-180.0, 180.0)
Q2_LIMIT_DEG = (-180.0, 180.0)
Q3_LIMIT_DEG = (-190.0, 190.0)
Q4_LIMIT_DEG = (-90.0, 92.4)

STATUS_NAMES = {
    0: "IDLE",
    1: "RUNNING",
    2: "COMPLETED",
    3: "FAILED",
    4: "EMERGENCY",
}

STATUS_COLORS = {
    0: "cyan",
    1: "blue",
    2: "green",
    3: "red",
    4: "red",
}

GRIPPER_STATE_NAMES = {
    0: "HOLD/UNKNOWN",
    1: "OPEN",
    2: "CLOSE",
}

ERROR_CODE_NAMES = {
    0: "NONE",
    1001: "TIMEOUT",
    1002: "LIMIT_EXCEEDED",
    1003: "ESTOP_TRIGGERED",
}

CONTROL_MODE_NAMES = {
    0: "TRAJECTORY",
    1: "STOP",
    2: "ESTOP",
}

# Conservative preset arm motions (angles in degrees)
PRESETS = {
    "home": {
        "q_deg": (0.0, 110.0, 50.0, 0.0),
        "gripper": 0,  # HOLD
        "duration_ms": 3000,
        "description": "Safe initial home pose [0°, 110°, 50°, 0°]",
    },
    "retract": {
        "q_deg": (0.0, 110.0, 150.0, 0.0),
        "gripper": 0,  # HOLD
        "duration_ms": 3000,
        "description": "Chassis clearance retract pose [0°, 110°, 150°, 0°]",
    },
}


class CommandError(ValueError):
    """Raised when an interactive command has invalid arguments or violates safety."""


@dataclass(frozen=True)
class ProtocolSettings:
    port: str
    baudrate: int


def load_and_validate_protocol(path: Path) -> ProtocolSettings:
    """Validate protocol.yaml structure and extract serial parameters."""
    if not path.is_file():
        raise CommandError(f"protocol.yaml not found at: {path}")

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as exc:
        raise CommandError(f"Failed to parse protocol.yaml ({path}): {exc}")

    if not isinstance(data, dict):
        raise CommandError("protocol.yaml root must be a mapping")

    # 1. Validate serial parameters
    serial_sec = data.get("serial_controller", {})
    params = serial_sec.get("ros__parameters", {})
    if "port" not in params:
        raise CommandError(
            "protocol.yaml is missing 'serial_controller.ros__parameters.port'"
        )
    if "baudrate" not in params:
        raise CommandError(
            "protocol.yaml is missing 'serial_controller.ros__parameters.baudrate'"
        )

    port = str(params["port"])
    try:
        baudrate = int(params["baudrate"])
    except (ValueError, TypeError):
        raise CommandError(f"Invalid baudrate in protocol.yaml: {params['baudrate']}")

    # 2. Validate required messages
    raw_messages = data.get("messages", [])
    if not isinstance(raw_messages, list):
        raise CommandError("protocol.yaml 'messages' section must be a list")

    found_names = {
        msg.get("name") for msg in raw_messages if isinstance(msg, dict)
    }
    required_messages = {
        "ChassisVelocity",
        "TrajectoryHeader",
        "TrajectoryPoint",
        "ArmControlMode",
        "ArmStatus",
    }
    missing = required_messages - found_names
    if missing:
        raise CommandError(
            f"protocol.yaml is missing required messages: {', '.join(sorted(missing))}"
        )

    return ProtocolSettings(port=port, baudrate=baudrate)


class ConsoleOutput:
    def __init__(self, no_color: bool = False):
        self.no_color = no_color
        self._lock = threading.Lock()

    def raw_line(self, text: str) -> None:
        with self._lock:
            sys.stdout.write(f"\r\033[K{text}\n")
            sys.stdout.flush()
            try:
                readline.redisplay()
            except Exception:
                pass

    def line(self, text: str, color: Optional[str] = None) -> None:
        with self._lock:
            prefix = ANSI_CODES.get(color, "") if not self.no_color and color else ""
            suffix = ANSI_RESET if prefix else ""
            sys.stdout.write(f"\r\033[K{prefix}{text}{suffix}\n")
            sys.stdout.flush()
            try:
                readline.redisplay()
            except Exception:
                pass


class ConsoleNode(Node):
    def __init__(self, output: ConsoleOutput):
        super().__init__("lower_controller_console")
        self.output = output
        self._lock = threading.Lock()

        hardware_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.header_pub = self.create_publisher(
            TrajectoryHeader, "/fruit_hardware_bridge/arm/trajectory_header", hardware_qos
        )
        self.point_pub = self.create_publisher(
            TrajectoryPoint, "/fruit_hardware_bridge/arm/trajectory_point", hardware_qos
        )
        self.mode_pub = self.create_publisher(
            ArmControlMode, "/fruit_hardware_bridge/arm/control_mode", hardware_qos
        )

        self.status_sub = self.create_subscription(
            ArmStatus,
            "/fruit_hardware_bridge/arm/status",
            self._on_arm_status,
            QoSProfile(depth=10),
        )

        self.latest_status: Optional[ArmStatus] = None
        self.latest_status_time: float = 0.0
        self.last_reported_task_status: Optional[int] = None
        self.last_reported_error_code: Optional[int] = None
        self._next_traj_id: int = 1000

    def _on_arm_status(self, msg: ArmStatus) -> None:
        with self._lock:
            status_changed = (
                self.last_reported_task_status is None
                or msg.task_status != self.last_reported_task_status
            )
            error_changed = (
                self.last_reported_error_code is None
                or msg.error_code != self.last_reported_error_code
            )
            self.latest_status = msg
            self.latest_status_time = time.monotonic()

            if status_changed or error_changed:
                self.last_reported_task_status = msg.task_status
                self.last_reported_error_code = msg.error_code
                self._report_status_change(msg)

    def _report_status_change(self, msg: ArmStatus) -> None:
        status_str = STATUS_NAMES.get(msg.task_status, f"UNKNOWN({msg.task_status})")
        color = STATUS_COLORS.get(msg.task_status, "yellow")
        if msg.error_code != 0:
            color = "red"

        deg1 = math.degrees(msg.joint_1)
        deg2 = math.degrees(msg.joint_2)
        deg3 = math.degrees(msg.joint_3)
        deg4 = math.degrees(msg.joint_4)
        err_str = ERROR_CODE_NAMES.get(msg.error_code, f"ERR_{msg.error_code}")
        grip_str = GRIPPER_STATE_NAMES.get(msg.gripper_state, str(msg.gripper_state))

        text = (
            f"RX ArmStatus: status={status_str}, prog={msg.progress:.1f}%, "
            f"Q=[{deg1:+.1f}°, {deg2:+.1f}°, {deg3:+.1f}°, {deg4:+.1f}°], "
            f"grip={grip_str}, err={err_str}"
        )
        self.output.line(text, color)

    def get_latest_status(self) -> Tuple[Optional[ArmStatus], float]:
        with self._lock:
            return self.latest_status, self.latest_status_time

    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        # Note: auto_serial_bridge maps ros: "-angular.z" to lower controller yaw,
        # so we pass standard ROS angular_z directly.
        msg.angular.z = float(angular_z)
        self.vel_pub.publish(msg)

    def send_trajectory(
        self,
        joint_1_rad: float,
        joint_2_rad: float,
        joint_3_rad: float,
        joint_4_rad: float,
        gripper_action: int = 0,
        duration_ms: int = 3000,
    ) -> int:
        with self._lock:
            self._next_traj_id = (self._next_traj_id % 0xFFFFFFFE) + 1
            traj_id = self._next_traj_id

            if self.latest_status is not None:
                j1 = 0.0 if math.isnan(self.latest_status.joint_1) else self.latest_status.joint_1
                j2 = 0.0 if math.isnan(self.latest_status.joint_2) else self.latest_status.joint_2
                j3 = 0.0 if math.isnan(self.latest_status.joint_3) else self.latest_status.joint_3
                j4 = 0.0 if math.isnan(self.latest_status.joint_4) else self.latest_status.joint_4
                start_q = (j1, j2, j3, j4)
            else:
                start_q = (0.0, 0.0, 0.0, 0.0)

        # 1. Header (point_num = 2)
        header = TrajectoryHeader()
        header.trajectory_id = traj_id
        header.point_num = 2
        self.header_pub.publish(header)
        time.sleep(0.01)

        # 2. Point 0 (起点: point_id=0, t=0, HOLD)
        p0 = TrajectoryPoint()
        p0.trajectory_id = traj_id
        p0.point_id = 0
        p0.time_from_start = 0
        p0.joint_1 = float(start_q[0])
        p0.joint_2 = float(start_q[1])
        p0.joint_3 = float(start_q[2])
        p0.joint_4 = float(start_q[3])
        p0.gripper_action = 0  # HOLD
        self.point_pub.publish(p0)
        time.sleep(0.01)

        # 3. Point 1 (终点: point_id=1, t=duration_ms, 目标角度)
        p1 = TrajectoryPoint()
        p1.trajectory_id = traj_id
        p1.point_id = 1
        p1.time_from_start = int(duration_ms)
        p1.joint_1 = float(joint_1_rad)
        p1.joint_2 = float(joint_2_rad)
        p1.joint_3 = float(joint_3_rad)
        p1.joint_4 = float(joint_4_rad)
        p1.gripper_action = int(gripper_action)
        self.point_pub.publish(p1)
        time.sleep(0.01)

        # 4. Mode 0 (TRAJECTORY)
        mode = ArmControlMode()
        mode.control_mode = 0
        self.mode_pub.publish(mode)

        return traj_id

    def send_mode(self, mode_value: int) -> None:
        msg = ArmControlMode()
        msg.control_mode = int(mode_value)
        self.mode_pub.publish(msg)

    def send_stop(self) -> None:
        # Smooth stop: ArmControlMode(1) + Twist(0, 0)
        self.send_mode(1)
        self.send_velocity(0.0, 0.0)

    def send_estop(self) -> None:
        # Emergency stop: ArmControlMode(2) + Twist(0, 0)
        self.send_mode(2)
        self.send_velocity(0.0, 0.0)


def check_joint_limits(q1_deg: float, q2_deg: float, q3_deg: float, q4_deg: float) -> None:
    """Validate 4-DOF joint angles against conservative software safety limits."""
    if not (Q1_LIMIT_DEG[0] <= q1_deg <= Q1_LIMIT_DEG[1]):
        raise CommandError(
            f"Joint 1 (Base) {q1_deg:.2f}° out of safe limits [{Q1_LIMIT_DEG[0]}°, {Q1_LIMIT_DEG[1]}°]"
        )
    if not (Q2_LIMIT_DEG[0] <= q2_deg <= Q2_LIMIT_DEG[1]):
        raise CommandError(
            f"Joint 2 (Shoulder) {q2_deg:.2f}° out of safe limits [{Q2_LIMIT_DEG[0]}°, {Q2_LIMIT_DEG[1]}°]"
        )
    if not (Q3_LIMIT_DEG[0] <= q3_deg <= Q3_LIMIT_DEG[1]):
        raise CommandError(
            f"Joint 3 (Elbow) {q3_deg:.2f}° out of safe limits [{Q3_LIMIT_DEG[0]}°, {Q3_LIMIT_DEG[1]}°]"
        )
    if not (Q4_LIMIT_DEG[0] <= q4_deg <= Q4_LIMIT_DEG[1]):
        raise CommandError(
            f"Joint 4 (Wrist) {q4_deg:.2f}° out of safe limits [{Q4_LIMIT_DEG[0]}°, {Q4_LIMIT_DEG[1]}°]"
        )


def confirm_action(description: str, auto_yes: bool) -> bool:
    """Prompt user confirmation before executing arm motion."""
    if auto_yes:
        return True
    try:
        ans = input(f"Confirm action [{description}]? [y/N]: ").strip().lower()
        if ans in ("y", "yes"):
            return True
        print("Cancelled.")
        return False
    except (KeyboardInterrupt, EOFError):
        print("\nCancelled.")
        return False


def print_status(node: ConsoleNode, output: ConsoleOutput) -> None:
    """Print full detailed snapshot of latest ArmStatus telemetry."""
    telemetry, recv_time = node.get_latest_status()
    if telemetry is None:
        output.line("No ArmStatus telemetry received yet from lower controller.", "yellow")
        return
    age = time.monotonic() - recv_time
    status_str = STATUS_NAMES.get(
        telemetry.task_status, f"UNKNOWN({telemetry.task_status})"
    )
    error_str = ERROR_CODE_NAMES.get(
        telemetry.error_code, f"CODE_{telemetry.error_code}"
    )
    color = STATUS_COLORS.get(telemetry.task_status, "cyan")

    output.line(f"=== Lower Controller ArmStatus (freshness: {age:.2f}s) ===", "blue")
    output.line(f"  Trajectory ID : {telemetry.trajectory_id}")
    output.line(f"  Task Status   : {status_str} ({telemetry.task_status})", color)
    output.line(f"  Progress      : {telemetry.progress:.1f}%")
    output.line(
        f"  Error Code    : {error_str} ({telemetry.error_code})",
        "red" if telemetry.error_code != 0 else "dim",
    )
    output.line(
        f"  Gripper State : {GRIPPER_STATE_NAMES.get(telemetry.gripper_state, 'UNKNOWN')} ({telemetry.gripper_state})"
    )

    j = (telemetry.joint_1, telemetry.joint_2, telemetry.joint_3, telemetry.joint_4)
    v = (
        telemetry.joint_1_velocity,
        telemetry.joint_2_velocity,
        telemetry.joint_3_velocity,
        telemetry.joint_4_velocity,
    )
    output.line("  Joint Positions & Velocities:")
    output.line(
        f"    Q1 (Base)     : {j[0]:+7.3f} rad ({math.degrees(j[0]):+6.1f}°) | vel: {v[0]:+6.2f} rad/s"
    )
    output.line(
        f"    Q2 (Shoulder) : {j[1]:+7.3f} rad ({math.degrees(j[1]):+6.1f}°) | vel: {v[1]:+6.2f} rad/s"
    )
    output.line(
        f"    Q3 (Elbow)    : {j[2]:+7.3f} rad ({math.degrees(j[2]):+6.1f}°) | vel: {v[2]:+6.2f} rad/s"
    )
    output.line(
        f"    Q4 (Wrist)    : {j[3]:+7.3f} rad ({math.degrees(j[3]):+6.1f}°) | vel: {v[3]:+6.2f} rad/s"
    )


def _parse_user_int(value: str, label: str, minimum: int, maximum: int) -> int:
    try:
        try:
            parsed = int(value, 0)
        except ValueError:
            parsed = int(value, 10)
    except ValueError as exc:
        raise CommandError(f"{label} must be an integer: {value}") from exc
    if not minimum <= parsed <= maximum:
        raise CommandError(f"{label} must be in [{minimum}, {maximum}]")
    return parsed


def _parse_user_float(value: str, label: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise CommandError(f"{label} must be a number: {value}") from exc
    if not math.isfinite(parsed):
        raise CommandError(f"{label} must be finite")
    return parsed


def _check_arg_count(tokens: List[str], expected: int, usage: str) -> None:
    if len(tokens) != expected:
        raise CommandError(f"usage: {usage}")


def handle_command(
    tokens: List[str], node: ConsoleNode, output: ConsoleOutput, auto_yes: bool
) -> None:
    command = tokens[0].lower()

    # === Chassis Velocity ===
    if command in {"vel", "velocity"}:
        _check_arg_count(tokens, 3, "vel <linear_x> <angular_z>")
        linear_x = _parse_user_float(tokens[1], "linear_x")
        angular_z = _parse_user_float(tokens[2], "angular_z")
        node.send_velocity(linear_x, angular_z)
        output.line(f"Published cmd_vel: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}", "blue")
        return

    if command == "stop":
        _check_arg_count(tokens, 1, "stop")
        node.send_stop()
        output.line("STOP sent: ArmControlMode(STOP) and cmd_vel(0, 0)", "yellow")
        return

    if command == "estop":
        _check_arg_count(tokens, 1, "estop")
        node.send_estop()
        output.line("ESTOP sent: ArmControlMode(ESTOP) hard lock and cmd_vel(0, 0)", "red")
        return

    # === Arm Mode Control ===
    if command == "mode":
        _check_arg_count(tokens, 2, "mode <0=TRAJECTORY|1=STOP|2=ESTOP>")
        mode = _parse_user_int(tokens[1], "mode", 0, 2)
        node.send_mode(mode)
        mode_name = CONTROL_MODE_NAMES.get(mode, str(mode))
        output.line(f"Sent ArmControlMode({mode_name})", "yellow")
        return

    # === Arm Presets ===
    if command in PRESETS:
        _check_arg_count(tokens, 1, command)
        preset = PRESETS[command]
        q_deg = preset["q_deg"]
        check_joint_limits(*q_deg)
        if not confirm_action(f"{command.upper()} motion: {preset['description']}", auto_yes):
            return
        q_rad = tuple(math.radians(a) for a in q_deg)
        traj_id = node.send_trajectory(
            *q_rad,
            gripper_action=preset["gripper"],
            duration_ms=preset["duration_ms"],
        )
        output.line(
            f"Sent {command.upper()} trajectory (traj_id={traj_id}, duration={preset['duration_ms']}ms)",
            "green",
        )
        return

    if command in {"grip", "open"}:
        _check_arg_count(tokens, 1, command)
        action = 2 if command == "grip" else 1
        action_name = "CLOSE" if action == 2 else "OPEN"

        telemetry, _ = node.get_latest_status()
        if telemetry is not None:
            q_deg = (
                math.degrees(telemetry.joint_1),
                math.degrees(telemetry.joint_2),
                math.degrees(telemetry.joint_3),
                math.degrees(telemetry.joint_4),
            )
        else:
            q_deg = PRESETS["home"]["q_deg"]
        check_joint_limits(*q_deg)
        if not confirm_action(
            f"Gripper {action_name} at joints [{q_deg[0]:.1f}°, {q_deg[1]:.1f}°, {q_deg[2]:.1f}°, {q_deg[3]:.1f}°]",
            auto_yes,
        ):
            return
        q_rad = tuple(math.radians(a) for a in q_deg)
        traj_id = node.send_trajectory(*q_rad, gripper_action=action, duration_ms=1000)
        output.line(f"Triggered gripper {action_name} (traj_id={traj_id})", "green")
        return

    # === Custom Trajectory ===
    if command == "traj":
        if len(tokens) not in (5, 6, 7):
            raise CommandError(
                "usage: traj <q1_deg> <q2_deg> <q3_deg> <q4_deg> [gripper_action: 0=hold,1=open,2=close] [duration_ms]"
            )
        q1 = _parse_user_float(tokens[1], "q1")
        q2 = _parse_user_float(tokens[2], "q2")
        q3 = _parse_user_float(tokens[3], "q3")
        q4 = _parse_user_float(tokens[4], "q4")
        gripper_action = (
            _parse_user_int(tokens[5], "gripper_action", 0, 2)
            if len(tokens) >= 6
            else 0
        )
        duration_ms = (
            _parse_user_int(tokens[6], "duration_ms", 200, 60000)
            if len(tokens) >= 7
            else 3000
        )
        check_joint_limits(q1, q2, q3, q4)
        if not confirm_action(
            f"Trajectory to [{q1:.1f}°, {q2:.1f}°, {q3:.1f}°, {q4:.1f}°] over {duration_ms}ms",
            auto_yes,
        ):
            return
        traj_id = node.send_trajectory(
            math.radians(q1),
            math.radians(q2),
            math.radians(q3),
            math.radians(q4),
            gripper_action=gripper_action,
            duration_ms=duration_ms,
        )
        output.line(
            f"Sent custom trajectory (traj_id={traj_id}, duration={duration_ms}ms)",
            "green",
        )
        return

    # === Status Query ===
    if command == "status":
        _check_arg_count(tokens, 1, "status")
        print_status(node, output)
        return

    raise CommandError(f"unknown command: {tokens[0]}")


def help_text() -> str:
    return """Commands:
  === Chassis Velocity ===
  vel <linear_x> <angular_z>   Send open-loop velocity command (linear_x m/s, angular_z rad/s)
  stop                         Smooth stop: ArmControlMode(STOP) + zero chassis velocity

  === Arm Safe Presets ===
  home                         Return arm to safe initial pose [0°, 110°, 50°, 0°]
  retract                      Fold arm into chassis clearance retract pose [0°, 110°, 150°, 0°]
  grip                         Close gripper (gripper_action=2)
  open                         Open gripper (gripper_action=1)

  === Arm Manual Control ===
  traj <q1> <q2> <q3> <q4> [act] [ms]
                               Send custom 4-joint trajectory (angles in deg; act: 0=hold,1=open,2=close; ms: duration)
  mode <0|1|2>                 Send raw ArmControlMode (0=TRAJECTORY, 1=STOP, 2=ESTOP)
  estop                        Emergency STOP: ArmControlMode(ESTOP) hard emergency lock

  === Diagnostics & System ===
  status                       Print latest 20Hz ArmStatus telemetry snapshot
  help                         Show this help message
  quit / exit                  Exit the console and clean up bridge processes"""


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ROS 2 console for fruit robot lower controller via auto_serial_bridge",
    )
    parser.add_argument(
        "--log-level",
        default="debug",
        choices=["debug", "info", "warn", "error"],
        help="auto_serial_bridge log level (default: debug)",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="disable ANSI color output",
    )
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="skip confirmation prompts before executing arm motions",
    )
    parser.add_argument(
        "--no-spawn",
        action="store_true",
        help="do not automatically spawn auto_serial_bridge if missing",
    )
    return parser


def ensure_bridge_running(
    node: Node,
    port: str,
    baudrate: int,
    output: ConsoleOutput,
    auto_spawn: bool = True,
    log_level: str = "debug",
) -> Tuple[Optional[subprocess.Popen], bool]:
    """Check if /serial_controller node is running, otherwise spawn it via launch in debug mode."""
    node_names = node.get_node_names()
    if any("serial_controller" in n for n in node_names):
        output.line("Detected active /serial_controller. Reusing existing bridge.", "green")
        return None, True

    if not auto_spawn:
        output.line(
            "Warning: /serial_controller is not running and --no-spawn is active.\n"
            "Run in another terminal: ros2 launch auto_serial_bridge serial_bridge_by_node.launch.py",
            "yellow",
        )
        return None, False

    output.line(
        f"Starting auto_serial_bridge in DEBUG mode (port={port}, baudrate={baudrate}, log_level={log_level})...",
        "cyan",
    )

    env = os.environ.copy()
    env["RCUTILS_COLORIZED_OUTPUT"] = "1"
    env["PYTHONUNBUFFERED"] = "1"

    cmd = [
        "ros2",
        "launch",
        "auto_serial_bridge",
        "serial_bridge_by_node.launch.py",
        f"port:={port}",
        f"baudrate:={baudrate}",
        f"log_level:={log_level}",
    ]

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            env=env,
        )
    except Exception as exc:
        output.line(f"Failed to spawn auto_serial_bridge: {exc}", "red")
        return None, False

    def _stream_bridge_output() -> None:
        assert proc.stdout is not None
        for raw_line in iter(proc.stdout.readline, b""):
            text = raw_line.decode("utf-8", errors="replace").rstrip()
            if text:
                output.raw_line(text)

    reader_thread = threading.Thread(
        target=_stream_bridge_output,
        name="bridge-stdout-reader",
        daemon=True,
    )
    reader_thread.start()

    # Wait for node readiness (up to 8s)
    start_time = time.monotonic()
    timeout = 8.0
    ready = False
    while time.monotonic() - start_time < timeout:
        if proc.poll() is not None:
            output.line(f"auto_serial_bridge exited prematurely with code {proc.returncode}.", "red")
            return proc, False

        names = node.get_node_names()
        if any("serial_controller" in n for n in names):
            sub_count = node.count_subscribers("/fruit_hardware_bridge/arm/control_mode")
            if sub_count > 0:
                ready = True
                break
        time.sleep(0.2)

    if ready:
        output.line("auto_serial_bridge is connected and ready in DEBUG mode.", "green")
        return proc, True
    else:
        output.line(
            "Warning: Timeout waiting for bridge readiness.",
            "yellow",
        )
        return proc, False


def cleanup_bridge(spawned_proc: Optional[subprocess.Popen], output: ConsoleOutput) -> None:
    """Clean up bridge processes on exit to release serial device."""
    output.line("Cleaning up bridge processes and releasing serial device...", "dim")

    if spawned_proc is not None and spawned_proc.poll() is None:
        try:
            os.killpg(os.getpgid(spawned_proc.pid), signal.SIGINT)
            spawned_proc.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(spawned_proc.pid), signal.SIGTERM)
                spawned_proc.wait(timeout=1.0)
            except Exception:
                pass

    # Per design consensus: ensure any remaining serial_node is terminated
    try:
        subprocess.run(["pkill", "-SIGINT", "-f", "serial_node"], check=False)
        time.sleep(0.3)
        subprocess.run(["pkill", "-SIGTERM", "-f", "serial_node"], check=False)
    except Exception:
        pass


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    output = ConsoleOutput(no_color=args.no_color)

    try:
        settings = load_and_validate_protocol(DEFAULT_PROTOCOL_PATH)
    except CommandError as exc:
        output.line(f"Protocol configuration error: {exc}", "red")
        return 1

    # Initialize ROS 2
    rclpy.init()
    node = ConsoleNode(output)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    output.line(
        f"Loaded protocol config from {DEFAULT_PROTOCOL_PATH.name} (port={settings.port}, baudrate={settings.baudrate})",
        "dim",
    )

    spawned_proc, _ = ensure_bridge_running(
        node=node,
        port=settings.port,
        baudrate=settings.baudrate,
        output=output,
        auto_spawn=not args.no_spawn,
        log_level=args.log_level,
    )

    output.line("Fruit Lower Controller Console initialized.", "green")
    output.line("20Hz ArmStatus telemetry monitored in background.", "cyan")
    output.line("Type 'help' for command list. Press Ctrl-C during motion to STOP.", "dim")

    last_interrupt_time = 0.0
    auto_yes = args.yes

    try:
        while rclpy.ok():
            try:
                line = input("fruit> ")
                last_interrupt_time = 0.0
            except EOFError:
                break
            except KeyboardInterrupt:
                now = time.monotonic()
                if now - last_interrupt_time < 2.0:
                    output.line("\nExiting...", "yellow")
                    break
                last_interrupt_time = now
                node.send_stop()
                output.line(
                    "\n[STOP SENT] Emergency/Smooth STOP sent (ArmControlMode=STOP, chassis=0). "
                    "Press Ctrl-C again within 2s to exit.",
                    "yellow",
                )
                continue

            line = line.strip()
            if not line:
                continue

            try:
                tokens = shlex.split(line)
            except ValueError as exc:
                output.line(f"Command parse error: {exc}", "red")
                continue

            command = tokens[0].lower()
            if command in {"quit", "exit"}:
                break
            if command in {"help", "?"}:
                output.line(help_text(), "dim")
                continue

            try:
                handle_command(tokens, node, output, auto_yes)
            except CommandError as exc:
                output.line(f"Command error: {exc}", "red")
    finally:
        cleanup_bridge(spawned_proc, output)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        output.line("Console closed.", "dim")

    return 0


if __name__ == "__main__":
    sys.exit(main())
