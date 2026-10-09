#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws

NAV_LAUNCH="$WORKSPACE/src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py"
PERCEPTION_LAUNCH="$WORKSPACE/src/fruit_perception/launch/fruit_perception.launch.py"
MISSION_LAUNCH="$WORKSPACE/src/fruit_mission/launch/fruit_mission.launch.py"
ARM_LAUNCH="$WORKSPACE/src/fruit_arm_planner/launch/fruit_arm_planner.launch.py"
RECORD_SCRIPT="$WORKSPACE/record_rosbag.sh"

MISSION_CONFIG="$WORKSPACE/src/fruit_mission/config/fruit_mission.yaml"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
WAYPOINT_FILE="$WORKSPACE/src/navigation/fruit_nav_bringup/waypoints/odom_waypoints.yaml"
ODOM_FRAME="odom"

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
Use --debug to record the mission profile under rosbags/. The profile includes
navigation, perception, arm, QR, and selected raw camera image topics. Camera
recording requires the camera and perception publishers. Recording starts after
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
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
source "$WORKSPACE/scripts/fruit_runtime_recording.bash"
set -u

fruit_logging_init "fruit_mission" "${DEBUG_RECORD_ENABLED}"
if (( DEBUG_RECORD_ENABLED )); then
  fruit_recording_prepare mission
fi
RUN_DIR="${FRUIT_RUNTIME_SESSION_DIR}"
ORCHESTRATOR_LOG="${RUN_DIR}/orchestrator.log"

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
  local filter_script="$WORKSPACE/scripts/runtime_log_filter.py"

  : > "$log_file"
  log "START $name: $(command_string "$@")"
  setsid env "FRUIT_SUBSYSTEM_LOG=$log_file" "DEBUG_OPT=$DEBUG_RECORD_ENABLED" bash -c '
    set -o pipefail
    filter_args=(--output-file "$FRUIT_SUBSYSTEM_LOG" --component "'"$name"'")
    [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
    stdbuf -oL -eL "$@" 2>&1 | /usr/bin/python3 "'"$filter_script"'" "${filter_args[@]}"
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
  local filter_script="$WORKSPACE/scripts/runtime_log_filter.py"

  : > "$log_file"
  log "START $name: $(command_string "$@")"
  setsid env "FRUIT_SUBSYSTEM_LOG=$log_file" "DEBUG_OPT=$DEBUG_RECORD_ENABLED" bash -c '
    set -o pipefail
    filter_args=(--output-file "$FRUIT_SUBSYSTEM_LOG" --component "'"$name"'")
    [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
    stdbuf -oL -eL "$@" 2>&1 | /usr/bin/python3 "'"$filter_script"'" "${filter_args[@]}"
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
  log "WARNING: $label odometry unavailable; navigation requires odometry"
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
  log "EXIT $name output recorded at $log_file"
}

stop_children() {
  local pid
  local index

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

  # Ensure arm motion is stopped if planner is available
  timeout --foreground 2s ros2 service call /arm_planner_node/stop std_srvs/srv/Trigger '{}' \
    >/dev/null 2>&1 || true

  fruit_recording_stop

  if ((${#child_pids[@]} == 0 && ${#debug_pids[@]} == 0)); then
    fruit_logging_close 0
    return 0
  fi

  log "STOP stopping all launched child process groups"
  fruit_stop_all_process_groups "${child_pids[@]}" "${debug_pids[@]}"
  fruit_logging_close 0
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
tts = params.get("tts", {})
for key in (
    "manifest_path",
    "aplay_executable",
):
    value = tts.get(key)
    if not value:
        raise ValueError(f"missing mission config value: {key}")
    print(value)
PY
)" || die "failed to read mission runtime paths from $MISSION_CONFIG"
mapfile -t mission_runtime_path_values <<< "$mission_runtime_paths"
if ((${#mission_runtime_path_values[@]} != 2)); then
  die "mission config returned an unexpected number of runtime paths"
fi

VOICE_MANIFEST=${mission_runtime_path_values[0]}
APLAY_EXECUTABLE=${mission_runtime_path_values[1]}

check_file "$WAYPOINT_FILE"
for required_file in "$VOICE_MANIFEST"; do
  [[ -r "$required_file" ]] || log "WARNING: voice manifest unavailable: $required_file"
done
for required_executable in "$APLAY_EXECUTABLE"; do
  [[ -x "$required_executable" ]] || log "WARNING: speech executable unavailable: $required_executable"
done

for required_device in /dev/stm32; do
  [[ -e "$required_device" ]] || log "WARNING: device unavailable: $required_device"
done

log "RUN directory: $RUN_DIR"
log "RUN mission flow: $MISSION_FLOW"
log "RUN branch: $(git -C "$WORKSPACE" branch --show-current)"
log "RUN commit: $(git -C "$WORKSPACE" rev-parse --short HEAD)"
# Check for existing stack components to reuse
if [[ "$REUSE_NAVIGATION" != true ]]; then
  if run_readiness_client node-running \
    --node /laserMapping --node /loam_interface --node /axis_waypoint_action_server \
    --match all >/dev/null 2>&1; then
    REUSE_NAVIGATION=true
    log "DETECT: navigation nodes are already running; automatically reusing existing navigation stack"
  fi
fi

REUSE_PERCEPTION=false
if run_readiness_client node-running \
  --node /fruit_perception_node --node /camera \
  --match all >/dev/null 2>&1; then
  REUSE_PERCEPTION=true
  log "DETECT: perception nodes are already running; automatically reusing existing perception stack"
fi

REUSE_ARM_PLANNER=false
if run_readiness_client node-running \
  --node /arm_planner_node \
  --match all >/dev/null 2>&1; then
  REUSE_ARM_PLANNER=true
  log "DETECT: arm planner is already running; automatically reusing existing arm planner"
fi

REUSE_ROBVIZ_BRIDGE=false
if run_readiness_client node-running \
  --node /fruit_robviz_image_bridge \
  --match any >/dev/null 2>&1; then
  REUSE_ROBVIZ_BRIDGE=true
  log "DETECT: robviz bridge is already running; automatically reusing existing robviz bridge"
fi

if [[ "$REUSE_NAVIGATION" == true ]]; then
  log "RUN mode: reuse existing navigation"
else
  log "RUN navigation: Point-LIO in odom"
fi
if [[ "$REUSE_PERCEPTION" == true ]]; then
  log "RUN mode: reuse existing perception"
fi
if [[ "$REUSE_ARM_PLANNER" == true ]]; then
  log "RUN mode: reuse existing arm planner"
fi
if [[ "$REUSE_ROBVIZ_BRIDGE" == true ]]; then
  log "RUN mode: reuse existing robviz bridge"
fi
if ((DEBUG_RECORD_ENABLED)); then
  log "RUN debug rosbag: mission profile enabled; output is under $WORKSPACE/rosbags"
fi

if ! python3 -c \
  'from pathlib import Path; import sys; from fruit_mission.waypoint_route import MissionWaypointRoute; MissionWaypointRoute.from_file(Path(sys.argv[1]), sys.argv[2], sys.argv[3])' \
  "$WAYPOINT_FILE" "$MISSION_FLOW" "$ODOM_FRAME" >> "$ORCHESTRATOR_LOG" 2>&1; then
  die "mission waypoint validation failed; see $ORCHESTRATOR_LOG"
fi
log "READY mission waypoint schema and required A/C points"

# Verify no duplicate or conflicting instances for components that need to be launched
nodes_to_check_absent=(/fruit_mission)
if [[ "$REUSE_NAVIGATION" != true ]]; then
  nodes_to_check_absent+=(
    /livox_ros_driver2
    /laserMapping
    /loam_interface
    /axis_waypoint_action_server
    /twist_mux
  )
fi
if [[ "$REUSE_PERCEPTION" != true ]]; then
  nodes_to_check_absent+=(
    /fruit_perception_container
    /camera
    /fruit_perception_node
    /fruit_qr_planner
  )
fi
if [[ "$REUSE_ARM_PLANNER" != true ]]; then
  nodes_to_check_absent+=(/arm_planner_node)
fi

check_no_existing_nodes "${nodes_to_check_absent[@]}"

if [[ "$REUSE_NAVIGATION" != true ]]; then
  start_process "navigation" \
    ros2 launch "$NAV_LAUNCH"
fi

START_SERIAL_BRIDGE=$(run_readiness_client bridge-start-required)
if [[ "$START_SERIAL_BRIDGE" == false ]]; then
  log "RUN reuse: existing serial bridge remains owned by its original launcher"
fi
if [[ "$REUSE_NAVIGATION" == true ]]; then
  START_SERIAL_BRIDGE=false
  log "RUN reuse: existing navigation and serial bridge remain owned by original launcher"
fi

START_ROBVIZ_BRIDGE=true
if [[ "$REUSE_ROBVIZ_BRIDGE" == true ]] || ! run_readiness_client robviz-bridge-start-required >/dev/null 2>&1; then
  START_ROBVIZ_BRIDGE=false
  log "RUN reuse: existing robviz bridge remains owned by original launcher"
fi

wait_for_nodes "odometry navigation readiness" 10 \
  /axis_waypoint_action_server \
  /laserMapping \
  /loam_interface
wait_for_action "waypoint navigation" "/follow_waypoints" 10
wait_for_odometry "base odometry" /odometry 10
wait_for_tf "stable odometry origin" odom base_link 5

if [[ "$REUSE_PERCEPTION" != true ]]; then
  start_process "perception" \
    ros2 launch "$PERCEPTION_LAUNCH" \
      "use_sim_time:=false" \
      "serial_debug_raw_frame:=$([[ $DEBUG_RECORD_ENABLED -eq 1 ]] && echo 'true' || echo 'false')" \
      "start_serial_bridge:=$START_SERIAL_BRIDGE" \
      "start_robviz_bridge:=$START_ROBVIZ_BRIDGE"
else
  log "RUN reuse: existing perception stack remains owned by original launcher"
fi

wait_for_nodes "perception readiness" 10 \
  /serial_controller \
  /fruit_perception_node \
  /fruit_qr_planner
wait_for_serial_bridge_running 10

if [[ "$REUSE_ARM_PLANNER" != true ]]; then
  start_process "arm_planner" \
    ros2 launch "$ARM_LAUNCH" "use_sim_time:=false"
else
  log "RUN reuse: existing arm planner remains owned by original launcher"
fi
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
wait_for_topic_endpoint_count \
  "perception observation window subscription" \
  "/fruit_mission/observation_window" \
  subscribers \
  1 \
  10

if ((DEBUG_RECORD_ENABLED)); then
  fruit_recording_start "$RECORD_SCRIPT" mission
  fruit_recording_wait_ready 10 || true
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
  log "RUN mission started; press Ctrl-C to stop launched processes (reused bridge stays running)"
fi
set +e
# A dead perception/QR/planner process must not tear down the competition.
mission_pid=${child_pids[${#child_pids[@]}-1]}
wait "$mission_pid"
status=$?
set -e
log "EXIT mission status=$status; local subsystem failures did not terminate the runner"
exit "$status"
