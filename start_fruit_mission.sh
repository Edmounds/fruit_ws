#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws

NAV_LAUNCH="$WORKSPACE/src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py"
PERCEPTION_LAUNCH="$WORKSPACE/src/fruit_perception/launch/fruit_perception.launch.py"
MISSION_LAUNCH="$WORKSPACE/src/fruit_mission/launch/fruit_mission.launch.py"
ARM_LAUNCH="$WORKSPACE/src/fruit_arm_planner/launch/fruit_arm_planner.launch.py"
RECORD_SCRIPT="$WORKSPACE/record_rosbag.sh"
MONITOR_SCRIPT="$WORKSPACE/scripts/robot_debug_monitor.sh"

MISSION_CONFIG="$WORKSPACE/src/fruit_mission/config/fruit_mission.yaml"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
WAYPOINT_FILE="$WORKSPACE/src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml"
ODOM_FRAME="odom"

RUN_ROOT="$WORKSPACE/tmp/fruit_mission/runs"
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
RUN_DIR="$RUN_ROOT/$RUN_ID"
ORCHESTRATOR_LOG="$RUN_DIR/orchestrator.log"
MISSION_FLOW="full"
DEBUG_RECORD_ENABLED=0
REUSE_NAVIGATION=false

usage() {
  cat <<'EOF'
Usage: ./start_fruit_mission.sh [--reuse-navigation] [--flow full|ac|a|b|c] [--debug]

Start the complete perception, arm planner, odometry navigation, and mission stack in the
startup-relative odom frame.
Use --reuse-navigation after start_navigation.sh to keep its navigation
and serial bridge running while starting perception, arm planner, and mission.
--flow selects a Logic.py entry function; B/C-only entries also scan their QR.
Use --debug to record ROS diagnostics, LiDAR, point clouds, and the selected
raw/annotated camera topics under rosbags/. Duplicate high-bandwidth image
transports and Camera unaligned raw depth are excluded. Recording starts after
the navigation and perception stack is ready, immediately before the mission.
EOF
}

while (($# > 0)); do
  case "$1" in
    --flow)
      (($# >= 2)) || { usage >&2; exit 2; }
      MISSION_FLOW=$2
      shift 2
      ;;
    --reuse-navigation)
      REUSE_NAVIGATION=true
      shift
      ;;
    --debug)
      DEBUG_RECORD_ENABLED=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

case "$MISSION_FLOW" in
  full|ac|a|b|c) ;;
  *) usage >&2; exit 2 ;;
esac

child_pids=()
child_names=()
child_logs=()
debug_pids=()
readiness_pid=""
stopped=0

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
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
  local -a arguments=(nodes-absent)
  local node_name
  local status

  for node_name in "$@"; do
    arguments+=(--node "$node_name")
  done
  if run_readiness_client "${arguments[@]}"; then
    return 0
  else
    status=$?
  fi
  ((status != 2)) || die "process exited while checking existing ROS nodes"
  die "a required node is already running; stop the existing stack first"
}

process_alive() {
  local state
  state="$(ps -o stat= -p "$1" 2>/dev/null || true)"
  [[ -n "$state" && "$state" != Z* ]]
}

run_readiness_client() {
  local status=0

  setsid timeout --foreground 10s ros2 run fruit_mission stack_readiness_client "$@" \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG" \
    > >(tee -a "$ORCHESTRATOR_LOG") 2>&1 &
  readiness_pid=$!
  while process_alive "$readiness_pid"; do
    sleep 0.2
  done
  wait "$readiness_pid" || status=$?
  readiness_pid=""
  return "$status"
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
  child_logs+=("$log_file")
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
  local -a arguments=(nodes --timeout "$timeout_sec")
  local node_name
  local status

  for node_name in "$@"; do
    arguments+=(--node "$node_name")
  done
  log "WAIT $label nodes=$* timeout=${timeout_sec}s"
  if run_readiness_client "${arguments[@]}"; then
    log "READY $label nodes=$*"
    return 0
  else
    status=$?
  fi
  log "WARNING: $label not ready (status=$status); mission will skip unavailable work"
}

wait_for_action() {
  local label=$1
  local action_name=$2
  local timeout_sec=$3
  local status

  log "WAIT $label action=$action_name timeout=${timeout_sec}s"
  if run_readiness_client action \
    --name "$action_name" \
    --timeout "$timeout_sec"; then
    log "READY $label action=$action_name"
    return 0
  else
    status=$?
  fi
  log "WARNING: $label action unavailable; mission will skip unavailable work"
}

wait_for_arm_planner() {
  local timeout_sec=$1
  local status

  log "WAIT arm planner motion/preset/STOP services timeout=${timeout_sec}s"
  if run_readiness_client planner --timeout "$timeout_sec"; then
    log "READY arm planner motion/preset/STOP services"
    return 0
  else
    status=$?
  fi
  log "WARNING: arm planner unavailable; no unsafe arm motion will be submitted"
}

wait_for_tf() {
  local label=$1
  local parent_frame=$2
  local child_frame=$3
  local timeout_sec=$4
  local status

  log "WAIT $label tf=$parent_frame->$child_frame timeout=${timeout_sec}s"
  if run_readiness_client tf \
    --parent "$parent_frame" \
    --child "$child_frame" \
    --timeout "$timeout_sec" \
    --max-age 1; then
    log "READY $label tf=$parent_frame->$child_frame"
    return 0
  else
    status=$?
  fi
  log "WARNING: $label TF unavailable; mission will skip unavailable work"
}

wait_for_odometry() {
  local label=$1
  local topic=$2
  local timeout_sec=$3
  local status

  log "WAIT $label topic=$topic timeout=${timeout_sec}s"
  if run_readiness_client odometry \
    --topic "$topic" \
    --timeout "$timeout_sec"; then
    log "READY $label topic=$topic"
    return 0
  else
    status=$?
  fi
  log "WARNING: $label odometry unavailable; no recognition without stationary feedback"
}

wait_for_active_off() {
  local timeout_sec=$1
  local status

  log "WAIT camera mode manager active=OFF timeout=${timeout_sec}s"
  if run_readiness_client active-mode --mode OFF --timeout "$timeout_sec"; then
    log "READY camera mode manager active=OFF"
    return 0
  else
    status=$?
  fi
  log "WARNING: camera mode unavailable; mission will skip unavailable work"
}

wait_for_topic_endpoint_count() {
  local label=$1
  local topic=$2
  local endpoint=$3
  local expected_count=$4
  local timeout_sec=$5
  local status

  case "$endpoint" in
    publishers|subscribers)
      ;;
    *)
      die "unsupported topic endpoint kind: $endpoint"
      ;;
  esac

  log "WAIT $label topic=$topic $endpoint>=$expected_count timeout=${timeout_sec}s"
  if run_readiness_client endpoint \
    --topic "$topic" \
    --kind "$endpoint" \
    --count "$expected_count" \
    --timeout "$timeout_sec"; then
    log "READY $label topic=$topic $endpoint>=$expected_count"
    return 0
  else
    status=$?
  fi
  log "WARNING: $label endpoint unavailable; mission will skip unavailable work"
}

wait_for_serial_bridge_running() {
  local timeout_sec=$1
  local log_file="$RUN_DIR/perception.log"
  local status

  log "WAIT serial bridge RUNNING timeout=${timeout_sec}s"
  if run_readiness_client bridge --timeout "$timeout_sec"; then
    log "READY serial bridge RUNNING"
    return 0
  else
    status=$?
  fi
  log "WARNING: serial bridge unavailable; see $log_file; mission remains bounded"
}

log_recent_child_output() {
  local name=$1
  local log_file=$2

  [[ -r "$log_file" ]] || return 0
  log "EXIT recent $name output from $log_file"
  tail -n 80 "$log_file" | sed "s/^/[$name] /" | tee -a "$ORCHESTRATOR_LOG"
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

  if process_alive "$readiness_pid"; then
    kill -INT -- "-$readiness_pid" 2>/dev/null || true
    wait "$readiness_pid" 2>/dev/null || true
    readiness_pid=""
  fi

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

  wait_deadline=$((SECONDS + 10))
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

required_files=(
  "$NAV_LAUNCH" \
  "$PERCEPTION_LAUNCH" \
  "$MISSION_LAUNCH" \
  "$ARM_LAUNCH" \
  "$MONITOR_SCRIPT" \
  "$MISSION_CONFIG" \
  "$SCRIPT_CLIENT_CONFIG"
)
for required_file in "${required_files[@]}"; do
  check_file "$required_file"
done

if ((DEBUG_RECORD_ENABLED)); then
  [[ -x "$RECORD_SCRIPT" ]] || die "required executable is unavailable: $RECORD_SCRIPT"
fi

mission_runtime_paths="$(
  python3 - "$MISSION_CONFIG" <<'PY'
import sys

import yaml


with open(sys.argv[1], encoding="utf-8") as stream:
    document = yaml.safe_load(stream)
params = document["fruit_mission"]["ros__parameters"]
tts = params["tts"]
for key in (
    "piper_executable",
    "model_path",
    "config_path",
    "aplay_executable",
):
    value = tts.get(key)
    if not value:
        raise ValueError(f"missing mission config value: {key}")
    print(value)
PY
)" || die "failed to read mission runtime paths from $MISSION_CONFIG"
mapfile -t mission_runtime_path_values <<< "$mission_runtime_paths"
if ((${#mission_runtime_path_values[@]} != 4)); then
  die "mission config returned an unexpected number of runtime paths"
fi

PIPER_EXECUTABLE=${mission_runtime_path_values[0]}
MODEL_PATH=${mission_runtime_path_values[1]}
MODEL_CONFIG=${mission_runtime_path_values[2]}
APLAY_EXECUTABLE=${mission_runtime_path_values[3]}

check_file "$WAYPOINT_FILE"
for required_file in "$MODEL_PATH" "$MODEL_CONFIG"; do
  [[ -r "$required_file" ]] || log "WARNING: speech file unavailable: $required_file"
done
for required_executable in "$PIPER_EXECUTABLE" "$APLAY_EXECUTABLE"; do
  [[ -x "$required_executable" ]] || log "WARNING: speech executable unavailable: $required_executable"
done

for required_device in /dev/stm32; do
  [[ -e "$required_device" ]] || log "WARNING: device unavailable: $required_device"
done

log "RUN directory: $RUN_DIR"
log "RUN mission flow: $MISSION_FLOW"
log "RUN branch: $(git -C "$WORKSPACE" branch --show-current)"
log "RUN commit: $(git -C "$WORKSPACE" rev-parse --short HEAD)"
if [[ "$REUSE_NAVIGATION" == true ]]; then
  log "RUN mode: reuse existing navigation and serial bridge"
else
  log "RUN navigation: Point-LIO in odom"
fi
if ((DEBUG_RECORD_ENABLED)); then
  log "RUN debug rosbag: enabled with visual-topic exclusions; output is under $WORKSPACE/rosbags"
fi

if ! python3 -c \
  'from pathlib import Path; import sys; from fruit_mission.waypoint_route import MissionWaypointRoute; MissionWaypointRoute.from_file(Path(sys.argv[1]), sys.argv[2], sys.argv[3])' \
  "$WAYPOINT_FILE" "$MISSION_FLOW" "$ODOM_FRAME" >> "$ORCHESTRATOR_LOG" 2>&1; then
  die "mission waypoint validation failed; see $ORCHESTRATOR_LOG"
fi
log "READY mission waypoint schema and required A/C points"

if [[ "$REUSE_NAVIGATION" == true ]]; then
  check_no_existing_nodes \
    /fruit_perception_container \
    /camera \
    /fruit_perception_node \
    /camera_mode_manager \
    /fruit_qr_planner \
    /arm_planner_node \
    /fruit_mission
else
  check_no_existing_nodes \
    /livox_ros_driver2 \
    /laserMapping \
    /loam_interface \
    /serial_controller \
    /axis_waypoint_action_server \
    /twist_mux \
    /fruit_perception_container \
    /camera \
    /fruit_perception_node \
    /camera_mode_manager \
    /fruit_qr_planner \
    /arm_planner_node \
    /fruit_mission
fi

if [[ "$REUSE_NAVIGATION" != true ]]; then
  start_process "navigation" \
    ros2 launch "$NAV_LAUNCH"

  start_debug_process "robot_debug" \
    "$MONITOR_SCRIPT" fruit-mission
fi

START_SERIAL_BRIDGE=true
if [[ "$REUSE_NAVIGATION" == true ]]; then
  START_SERIAL_BRIDGE=false
  log "RUN reuse: existing navigation, serial bridge, and debug monitor remain owned by start_navigation.sh"
fi

wait_for_nodes "odometry navigation readiness" 10 \
  /axis_waypoint_action_server \
  /laserMapping \
  /loam_interface
wait_for_action "waypoint navigation" "/follow_waypoints" 10
wait_for_odometry "base odometry" /odometry 10
wait_for_tf "stable odometry origin" odom base_link 5

start_process "perception" \
  ros2 launch "$PERCEPTION_LAUNCH" \
    "use_sim_time:=false" \
    "serial_debug_raw_frame:=true" \
    "start_serial_bridge:=$START_SERIAL_BRIDGE"


wait_for_nodes "perception readiness" 10 \
  /serial_controller \
  /fruit_perception_node \
  /camera_mode_manager \
  /fruit_qr_planner
wait_for_serial_bridge_running 10
start_process "arm_planner" \
  ros2 launch "$ARM_LAUNCH" "use_sim_time:=false"
wait_for_nodes "arm planner readiness" 10 /arm_planner_node
wait_for_arm_planner 10
wait_for_topic_endpoint_count \
  "trajectory header bridge subscription" \
  "/fruit_hardware_bridge/arm/trajectory_header" \
  subscribers \
  1 \
  10
wait_for_topic_endpoint_count \
  "trajectory point bridge subscription" \
  "/fruit_hardware_bridge/arm/trajectory_point" \
  subscribers \
  1 \
  10
wait_for_topic_endpoint_count \
  "arm mode bridge subscription" \
  "/fruit_hardware_bridge/arm/control_mode" \
  subscribers \
  1 \
  10
wait_for_topic_endpoint_count \
  "arm status bridge publisher" \
  "/fruit_hardware_bridge/arm/status" \
  publishers \
  1 \
  10
wait_for_active_off 10

if ((DEBUG_RECORD_ENABLED)); then
  start_process "rosbag" \
    "$RECORD_SCRIPT" \
    --no-monitor
  wait_for_node "debug rosbag recorder" /rosbag2_recorder 10
fi

log "READY navigation, perception, and arm planner stack; starting mission"
start_process "mission" \
  ros2 launch "$MISSION_LAUNCH" \
    "use_sim_time:=false" \
    "waypoint_file:=$WAYPOINT_FILE" \
    "mission_flow:=$MISSION_FLOW"

if [[ "$REUSE_NAVIGATION" == true ]]; then
  log "RUN mission started; Ctrl-C stops mission, arm planner, and perception but keeps the reused navigation stack running"
else
  log "RUN mission started; press Ctrl-C to stop the complete stack"
fi
set +e
# A dead perception/QR/planner process must not tear down the competition.
mission_pid=${child_pids[${#child_pids[@]}-1]}
wait "$mission_pid"
status=$?
set -e
log "EXIT mission status=$status; local subsystem failures did not terminate the runner"
exit "$status"
