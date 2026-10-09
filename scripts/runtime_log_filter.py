#!/usr/bin/env python3
"""Runtime log filter and archiver for fruit_ws.

Reads raw stdout/stderr from stdin, writes full unfiltered lines to the target
log file, and filters terminal output so that operators only see concise, high-value
information, warnings, errors, and lifecycle events.
"""

import argparse
import os
import re
import sys

ANSI_ESCAPE_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")

# Patterns that should NEVER be logged to file nor displayed on terminal (pure middleware noise).
DISCARD_PATTERNS = [
    re.compile(r"\[DEBUG\].*?\[(?:rcl|rclpy|rcutils|rmw_cyclonedds_cpp)\]:"),
    re.compile(r"\[rcl\]:\s+(?:Initializing wait set|Subscription taking message|Subscription take succeeded)"),
]

# Patterns that should NEVER be displayed on terminal under any circumstances.
TERMINAL_NEVER_SHOW_PATTERNS = [
    re.compile(r"\[MCU TX RAW\]"),
    re.compile(r"\[MCU RX RAW\]"),
    re.compile(r"MCU TX RAW"),
    re.compile(r"MCU RX RAW"),
    re.compile(r"serial_controller\.detail"),
    re.compile(r"ddsi_udp_conn_write to udp/.*failed with\s*retcode -1"),
]

# High-frequency routine inspection patterns (only written to disk).
ROUTINE_INSPECTION_PATTERNS = [
    re.compile(r"\[robot-debug\]\s+CAMERA streams status=OK"),
    re.compile(r"\[robot-debug\]\s+HEALTH\s+"),
    re.compile(r"\[robot-debug\]\s+CAM camera detections count=0"),
]

# Verbose launch and driver setup patterns (only written to disk).
LAUNCH_NOISE_PATTERNS = [
    re.compile(r"\[INFO\]\s+\[launch\]:\s+All log files can be found below"),
    re.compile(r"\[INFO\]\s+\[launch\]:\s+Default logging verbosity is set to"),
    re.compile(r"\[INFO\]\s+\[launch_ros\.actions\.load_composable_nodes\]:\s+Loaded node"),
    re.compile(r"process started with pid"),
    re.compile(r"\[IoContext::IoContext\]:\s+Thread\(s\) Created"),
    re.compile(r"\[serial_controller\]:\s+Initializing SerialController"),
    re.compile(r"\[serial_controller\]:\s+Port:\s+/dev/"),
    re.compile(r"\[serial_controller\]:\s+Mode selection:"),
    re.compile(r"\[camera_arm_target_client\]:\s+Loaded target_offset_m="),
    re.compile(r"\[camera_target_planner_client\]:\s+Mission camera transform ready"),
    re.compile(r"\[camera_arm_target_client\]:\s+Transforming target to arm base coordinates"),
    re.compile(r"\[camera_arm_target_client\]:\s+Camera\(m\)="),
    re.compile(r"\[camera_arm_target_client\]:\s+Executing arm preset:"),
]

# Explicit warning or error markers that MUST be displayed on terminal.
ALWAYS_SHOW_MARKERS = [
    "[WARN]",
    "[ERROR]",
    "[FATAL]",
    "Traceback (most recent call last):",
    "process has died",
    "exit code",
    "failed to",
    "Failed to",
    "status=STALE",
    "status=ERROR",
]


class LogFilter:
    def __init__(self, output_path: str, component: str, debug_mode: bool = False):
        self.output_path = output_path
        self.component = component
        self.debug_mode = debug_mode
        self.in_traceback = False

        parent_dir = os.path.dirname(output_path)
        if parent_dir and not os.path.exists(parent_dir):
            os.makedirs(parent_dir, exist_ok=True)
        self.log_file = open(output_path, "a", encoding="utf-8", errors="replace")

    def close(self):
        if self.log_file and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()

    def process_line(self, raw_line: str):
        clean_line = ANSI_ESCAPE_RE.sub("", raw_line).strip()

        # 0. Completely discard noisy middleware logs (neither written to file nor displayed).
        for pat in DISCARD_PATTERNS:
            if pat.search(clean_line):
                return

        # 1. Always write raw line to log file.
        try:
            self.log_file.write(raw_line)
            self.log_file.flush()
        except Exception as e:
            sys.stderr.write(f"[log_filter] Write error: {e}\n")

        if not clean_line:
            if self.in_traceback:
                sys.stdout.write(raw_line)
                sys.stdout.flush()
            return

        # 2. Traceback tracking: keep displaying until traceback ends.
        if "Traceback (most recent call last):" in clean_line:
            self.in_traceback = True
            sys.stdout.write(raw_line)
            sys.stdout.flush()
            return

        if self.in_traceback:
            sys.stdout.write(raw_line)
            sys.stdout.flush()
            if raw_line and not raw_line[0].isspace():
                self.in_traceback = False
            return

        # 3. Strictly reject raw frames and detail logger on terminal.
        for pat in TERMINAL_NEVER_SHOW_PATTERNS:
            if pat.search(clean_line):
                return

        # 4. Handle serial decoded packets.
        if "[MCU TX DECODED]" in clean_line or "[MCU RX DECODED]" in clean_line:
            # On-logger packets only show when explicit --debug is enabled.
            if self.debug_mode:
                sys.stdout.write(raw_line)
                sys.stdout.flush()
            return

        # 5. Always display warnings and errors.
        for marker in ALWAYS_SHOW_MARKERS:
            if marker in clean_line:
                sys.stdout.write(raw_line)
                sys.stdout.flush()
                return

        # 6. Filter routine periodic monitor inspections.
        for pat in ROUTINE_INSPECTION_PATTERNS:
            if pat.search(clean_line):
                return

        # 7. In non-debug mode, filter repetitive or per-frame detection logs.
        if not self.debug_mode:
            if "[robot-debug] CAM camera detections" in clean_line:
                return
            if "[CAMERA DEBUG]" in clean_line:
                return

        # 8. Filter verbose launch / configuration echoes.
        for pat in LAUNCH_NOISE_PATTERNS:
            if pat.search(clean_line):
                return

        # 9. Everything else (lifecycle events, targets, coordinates, results, completions) is displayed.
        sys.stdout.write(raw_line)
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="Fruit runtime log filter")
    parser.add_argument("--output-file", "-o", required=True, help="Destination log file path")
    parser.add_argument("--component", "-c", default="app", help="Component name")
    parser.add_argument("--debug", action="store_true", help="Enable debug level terminal output")
    args = parser.parse_args()

    filter_instance = LogFilter(args.output_file, args.component, debug_mode=args.debug)
    try:
        for line in sys.stdin:
            filter_instance.process_line(line)
    except KeyboardInterrupt:
        pass
    finally:
        filter_instance.close()


if __name__ == "__main__":
    main()
