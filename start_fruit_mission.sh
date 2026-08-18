#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws

NAV_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_navigation_launch.py"
PERCEPTION_LAUNCH="$WORKSPACE/src/fruit_perception/launch/fruit_perception.launch.py"
MISSION_LAUNCH="$WORKSPACE/src/fruit_mission/launch/fruit_mission.launch.py"

PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
AXIS_PARAMS="$WORKSPACE/src/fruit_navigation/config/axis_navigation.yaml"
MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.yaml"
LOCALIZATION_MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.pcd"
MISSION_CONFIG="$WORKSPACE/src/fruit_mission/config/fruit_mission.yaml"

RUN_ROOT="$WORKSPACE/tmp/fruit_mission/runs"
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
RUN_DIR="$RUN_ROOT/$RUN_ID"
ORCHESTRATOR_LOG="$RUN_DIR/orchestrator.log"

child_pids=()
child_names=()
debug_pids=()
stopped=0

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

mkdir -p "$RUN_DIR"

log() {
  local line
  line="[$(date '+%F %T')] $*"
  printf '%s\n' "$line" | tee -a "$ORCHESTRATOR_LOG"
}

die() {
  log "ERROR: $*"
  exit 1
}

command_string() {
  local arg
  for arg in "$@"; do
    printf '%q ' "$arg"
  done
}

check_file() {
  local path=$1
  [[ -r "$path" ]] || die "required file is unavailable: $path"
}

check_device() {
  local path=$1
  [[ -e "$path" ]] || die "required device is unavailable: $path"
}

check_no_existing_nodes() {
  local existing_nodes
  local node_name

  existing_nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
  for node_name in "$@"; do
    if grep -Fxq "$node_name" <<< "$existing_nodes"; then
      die "node is already running: $node_name; stop the existing stack first"
    fi
  done
}

process_alive() {
  local state
  state="$(ps -o stat= -p "$1" 2>/dev/null || true)"
  [[ -n "$state" && "$state" != Z* ]]
}

all_children_alive() {
  local pid
  for pid in "${child_pids[@]}"; do
    process_alive "$pid" || return 1
  done
  return 0
}

start_process() {
  local name=$1
  shift
  local log_file="$RUN_DIR/$name.log"
  local pid

  : > "$log_file"
  log "START $name: $(command_string "$@")"
  setsid env "FRUIT_SUBSYSTEM_LOG=$log_file" bash -c '
    set -o pipefail
    stdbuf -oL -eL "$@" 2>&1 | tee -a "$FRUIT_SUBSYSTEM_LOG"
  ' fruit-stack "$@" &
  pid=$!
  child_pids+=("$pid")
  child_names+=("$name")
  printf '%s\t%s\t%s\n' "$name" "$pid" "$log_file" >> "$RUN_DIR/processes.tsv"
  log "STARTED $name pid=$pid log=$log_file"
}

start_debug_process() {
  local name=$1
  shift
  local log_file="$RUN_DIR/$name.log"
  local pid

  : > "$log_file"
  log "START $name: $(command_string "$@")"
  setsid env "FRUIT_SUBSYSTEM_LOG=$log_file" bash -c '
    set -o pipefail
    stdbuf -oL -eL "$@" 2>&1 | tee -a "$FRUIT_SUBSYSTEM_LOG"
  ' fruit-stack "$@" &
  pid=$!
  debug_pids+=("$pid")
  printf '%s\t%s\t%s\n' "$name" "$pid" "$log_file" >> "$RUN_DIR/processes.tsv"
  log "STARTED $name pid=$pid log=$log_file"
}

wait_for_node() {
  local label=$1
  local node_name=$2
  local timeout_sec=$3

  wait_for_nodes "$label" "$timeout_sec" "$node_name"
}

wait_for_nodes() {
  local label=$1
  local timeout_sec=$2
  shift 2
  local -a missing=("$@")
  local -a still_missing=()
  local deadline=$((SECONDS + timeout_sec))
  local node_name
  local nodes

  log "WAIT $label nodes=${missing[*]} timeout=${timeout_sec}s"
  while ((SECONDS < deadline)); do
    all_children_alive || die "$label process exited before readiness"
    nodes="$(ros2 node list --no-daemon 2>/dev/null || true)"
    still_missing=()
    for node_name in "${missing[@]}"; do
      if grep -Fxq "$node_name" <<< "$nodes"; then
        log "READY $label node=$node_name"
      else
        still_missing+=("$node_name")
      fi
    done
    if ((${#still_missing[@]} == 0)); then
      return 0
    fi
    missing=("${still_missing[@]}")
    sleep 1
  done
  die "timeout waiting for $label nodes=${missing[*]}"
}

wait_for_action() {
  local label=$1
  local action_name=$2
  local timeout_sec=$3
  local deadline=$((SECONDS + timeout_sec))

  log "WAIT $label action=$action_name timeout=${timeout_sec}s"
  while ((SECONDS < deadline)); do
    all_children_alive || die "$label process exited before readiness"
    if ros2 action list 2>/dev/null | awk '{print $1}' | grep -Fxq "$action_name"; then
      log "READY $label action=$action_name"
      return 0
    fi
    sleep 1
  done
  die "timeout waiting for $label action=$action_name"
}

wait_for_localization() {
  local timeout_sec=$1
  local deadline=$((SECONDS + timeout_sec))
  local remaining
  local poll_timeout

  log "WAIT GICP localization accepted timeout=${timeout_sec}s"
  while ((SECONDS < deadline)); do
    all_children_alive || die "localization process exited before readiness"
    remaining=$((deadline - SECONDS))
    poll_timeout=$((remaining < 5 ? remaining : 5))
    if python3 - "$poll_timeout" <<'PY'
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def diagnostic_level_value(level):
    if isinstance(level, int):
        return level
    if isinstance(level, bytes):
        return level[0] if level else -1
    if isinstance(level, str):
        return ord(level) if len(level) == 1 else -1
    return -1


def level_is_ok(level):
    return diagnostic_level_value(level) == diagnostic_level_value(DiagnosticStatus.OK)


ready = False
latest_status = None


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


def diagnostic_summary(status):
    values = {item.key: item.value for item in status.values}
    fields = [
        ("accepted", values.get("accepted")),
        ("has_accepted_once", values.get("has_accepted_once")),
        ("rejects", values.get("consecutive_rejects")),
        ("source_points", values.get("source_points")),
        ("inliers", values.get("inliers")),
        ("inlier_ratio", values.get("inlier_ratio")),
        ("mean_error", values.get("mean_error")),
        ("translation_jump", values.get("translation_jump")),
        ("rotation_jump", values.get("rotation_jump")),
    ]
    detail = " ".join(f"{key}={value}" for key, value in fields if value is not None)
    return (
        f"DEBUG GICP wait: level={level_name(status.level)} "
        f"message={status.message!r} {detail}"
    )


def on_diagnostics(message):
    global latest_status, ready
    for status in message.status:
        if status.name != "pb_gicp_localization/registration":
            continue
        latest_status = status
        values = {item.key: item.value for item in status.values}
        if (
            level_is_ok(status.level)
            and values.get("accepted") == "1"
            and values.get("has_accepted_once") == "1"
        ):
            ready = True


timeout_sec = max(0.1, float(sys.argv[1]))
rclpy.init(args=["--ros-args", "--log-level", "fatal"])
node = rclpy.create_node("fruit_mission_wait_gicp_ready")

volatile_qos = QoSProfile(depth=10)
volatile_qos.reliability = ReliabilityPolicy.RELIABLE
node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, volatile_qos)

latched_qos = QoSProfile(depth=1)
latched_qos.reliability = ReliabilityPolicy.RELIABLE
latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, latched_qos)

deadline = time.monotonic() + timeout_sec
try:
    while rclpy.ok() and not ready:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if not ready:
        if latest_status is None:
            print(
                "DEBUG GICP wait: no pb_gicp_localization/registration diagnostics received "
                "during this poll"
            )
        else:
            print(diagnostic_summary(latest_status))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

raise SystemExit(0 if ready else 1)
PY
    then
      log "READY GICP localization accepted"
      return 0
    fi
  done
  die "timeout waiting for GICP localization to accept a registration"
}

wait_for_active_off() {
  local timeout_sec=$1
  local deadline=$((SECONDS + timeout_sec))

  log "WAIT camera mode manager active=OFF timeout=${timeout_sec}s"
  while ((SECONDS < deadline)); do
    all_children_alive || die "perception process exited before camera mode readiness"
    if timeout 5s ros2 topic echo --once \
      --qos-reliability reliable \
      --qos-durability transient_local \
      --qos-depth 1 \
      --field data \
      /fruit_perception/active_mode std_msgs/msg/String 2>/dev/null | grep -Fxq OFF; then
      log "READY camera mode manager active=OFF"
      return 0
    fi
  done
  die "timeout waiting for camera mode manager active=OFF"
}

stop_children() {
  local pid
  local index
  local wait_deadline

  trap - EXIT INT TERM
  if ((stopped)); then
    return 0
  fi
  stopped=1

  if ((${#child_pids[@]} == 0)); then
    return 0
  fi

  log "STOP stopping all child process groups"
  for pid in "${child_pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  for pid in "${debug_pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done

  wait_deadline=$((SECONDS + 15))
  while ((SECONDS < wait_deadline)); do
    local any_alive=0
    for pid in "${child_pids[@]}"; do
      if process_alive "$pid"; then
        any_alive=1
        break
      fi
    done
    for pid in "${debug_pids[@]}"; do
      if process_alive "$pid"; then
        any_alive=1
        break
      fi
    done
    ((any_alive == 0)) && break
    sleep 1
  done

  for index in "${!child_pids[@]}"; do
    pid=${child_pids[index]}
    if process_alive "$pid"; then
      log "STOP forcing ${child_names[index]} pid=$pid"
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  for pid in "${debug_pids[@]}"; do
    if process_alive "$pid"; then
      log "STOP forcing debug pid=$pid"
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done

  for pid in "${child_pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  ((${#debug_pids[@]} == 0)) || wait "${debug_pids[@]}" 2>/dev/null || true
  log "STOP complete; logs kept at $RUN_DIR"
}

on_signal() {
  log "SIGNAL received; shutting down"
  exit 130
}

trap stop_children EXIT
trap on_signal INT TERM

for required_file in \
  "$NAV_LAUNCH" \
  "$PERCEPTION_LAUNCH" \
  "$MISSION_LAUNCH" \
  "$PARAMS" \
  "$AXIS_PARAMS" \
  "$MAP" \
  "$LOCALIZATION_MAP" \
  "$WORKSPACE/robot_debug_monitor.sh" \
  "$MISSION_CONFIG"; do
  check_file "$required_file"
done

mission_runtime_paths="$(
  python3 - "$MISSION_CONFIG" <<'PY'
import sys

import yaml


with open(sys.argv[1], encoding="utf-8") as stream:
    document = yaml.safe_load(stream)
params = document["fruit_mission"]["ros__parameters"]
tts = params["tts"]
for key in (
    "waypoint_file",
    "piper_executable",
    "model_path",
    "config_path",
    "aplay_executable",
):
    value = params.get(key) if key == "waypoint_file" else tts.get(key)
    if not value:
        raise ValueError(f"missing mission config value: {key}")
    print(value)
PY
)" || die "failed to read mission runtime paths from $MISSION_CONFIG"
mapfile -t mission_runtime_path_values <<< "$mission_runtime_paths"
if ((${#mission_runtime_path_values[@]} != 5)); then
  die "mission config returned an unexpected number of runtime paths"
fi

WAYPOINT_FILE=${mission_runtime_path_values[0]}
PIPER_EXECUTABLE=${mission_runtime_path_values[1]}
MODEL_PATH=${mission_runtime_path_values[2]}
MODEL_CONFIG=${mission_runtime_path_values[3]}
APLAY_EXECUTABLE=${mission_runtime_path_values[4]}

for required_file in "$WAYPOINT_FILE" "$MODEL_PATH" "$MODEL_CONFIG"; do
  check_file "$required_file"
done
for required_executable in "$PIPER_EXECUTABLE" "$APLAY_EXECUTABLE"; do
  [[ -x "$required_executable" ]] || die "required executable is unavailable: $required_executable"
done

for required_device in /dev/left_cam /dev/right_cam /dev/stm32; do
  check_device "$required_device"
done

log "RUN directory: $RUN_DIR"
log "RUN branch: $(git -C "$WORKSPACE" branch --show-current)"
log "RUN commit: $(git -C "$WORKSPACE" rev-parse --short HEAD)"
log "RUN policy: perception owns the only serial bridge; navigation does not launch hardware_bridge"

if ! python3 -c \
  'from pathlib import Path; import sys; from fruit_mission.waypoint_route import FormalWaypointRoute; FormalWaypointRoute.from_file(Path(sys.argv[1]))' \
  "$WAYPOINT_FILE" >> "$ORCHESTRATOR_LOG" 2>&1; then
  die "mission waypoint validation failed; see $ORCHESTRATOR_LOG"
fi
log "READY mission waypoint schema and required A/C points"

check_no_existing_nodes \
  /livox_ros_driver2 \
  /laserMapping \
  /loam_interface \
  /sensor_scan_generation \
  /map_server \
  /lifecycle_manager_map \
  /serial_controller \
  /axis_waypoint_action_server \
  /route_marker_node \
  /twist_mux \
  /pb_gicp_localization \
  /fruit_perception_container \
  /left_camera \
  /right_camera \
  /camera/camera \
  /fruit_perception_node \
  /camera_mode_manager \
  /fruit_qr_planner \
  /fruit_mission

start_process "navigation" \
  ros2 launch "$NAV_LAUNCH" \
    "map:=$MAP" \
    "localization_map:=$LOCALIZATION_MAP" \
    "params_file:=$PARAMS" \
    "axis_params_file:=$AXIS_PARAMS" \
    "start_route_client:=false" \
    "use_teleop:=false" \
    "use_rviz:=false"

start_debug_process "robot_debug" \
  "$WORKSPACE/robot_debug_monitor.sh" fruit-mission

start_process "perception" \
  ros2 launch "$PERCEPTION_LAUNCH" \
    "use_sim_time:=false" \
    "serial_debug_raw_frame:=true"

wait_for_nodes "navigation readiness" 60 \
  /axis_waypoint_action_server \
  /pb_gicp_localization
wait_for_action "waypoint navigation" "/follow_waypoints" 60
wait_for_localization 180

wait_for_nodes "perception readiness" 120 \
  /serial_controller \
  /fruit_perception_node \
  /camera_mode_manager \
  /fruit_qr_planner
wait_for_active_off 60

log "READY navigation and perception stack; starting mission"
start_process "mission" \
  ros2 launch "$MISSION_LAUNCH" \
    "use_sim_time:=false"

log "RUN mission started; press Ctrl-C to stop the complete stack"
set +e
wait -n "${child_pids[@]}"
status=$?
set -e
log "EXIT one child process ended with status=$status; stopping remaining processes"
exit "$status"
