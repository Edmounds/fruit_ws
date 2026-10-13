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
OPEN_LOOP=false

usage() {
  cat <<'EOF'
Usage: ./start_fruit_mission.sh [--flow full|ac|a|b|c] [--open-loop] [--debug]

Start the complete perception, arm planner, odometry navigation, and mission stack in the
startup-relative odom frame.
Start only from a safe stationary robot. Existing competition stacks and motion
clients are stopped before this runner starts its own stack and odom origin.
--flow selects a Logic.py entry function; B/C-only entries also scan their QR.
Use --open-loop to perform open-loop picking in Zones A and C using direct presets without 3D depth closure.
Use --debug to record the mission profile under rosbags/. The profile includes
navigation, perception, arm, QR, and selected raw camera image topics. Camera
recording requires the camera and perception publishers. Recording starts in
the background after perception is ready and does not delay the mission. Debug
mode also records kernel USB events in kernel.log and udev events in udev.log.
It may request sudo once before starting these monitors.
The mission launch starts the stack and reuses an independently owned RobViz bridge.
EOF
}

while (($# > 0)); do
  case "$1" in
    --flow)
      (($# >= 2)) || { usage >&2; exit 2; }
      MISSION_FLOW=$2
      shift 2
      ;;
    --open-loop)
      OPEN_LOOP=true
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
  setsid env "FRUIT_SUBSYSTEM_LOG=$log_file" "DEBUG_OPT=$DEBUG_RECORD_ENABLED" "STACK_LOG_DIR=$RUN_DIR" bash -c '
    set -o pipefail
    filter_args=(--output-file "$FRUIT_SUBSYSTEM_LOG" --component "'"$name"'")
    [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
    [[ "'"$name"'" == "mission" ]] && filter_args+=(--stack-log-dir "$STACK_LOG_DIR")
    stdbuf -oL -eL "$@" 2>&1 | /usr/bin/python3 "'"$filter_script"'" "${filter_args[@]}"
  ' fruit-stack "$@" &
  pid=$!
  child_pids+=("$pid")
  child_names+=("$name")
  printf '%s\t%s\t%s\n' "$name" "$pid" "$log_file" >> "$RUN_DIR/processes.tsv"
  log "STARTED $name pid=$pid log=$log_file"
}

start_usb_debug_monitors() {
  (( DEBUG_RECORD_ENABLED )) || return 0

  local -a privilege=()
  if (( EUID != 0 )); then
    if ! command -v sudo >/dev/null 2>&1; then
      log "WARNING: sudo is unavailable; USB debug monitors were not started"
      return 0
    fi
    if ! sudo -v; then
      log "WARNING: sudo authorization failed; USB debug monitors were not started"
      return 0
    fi
    privilege=(sudo -n)
  fi

  start_process "kernel" "${privilege[@]}" \
    journalctl -k -f -o short-iso
  start_process "udev" "${privilege[@]}" \
    udevadm monitor --kernel --udev
  log "RUN USB diagnostics: kernel.log and udev.log"
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

stop_child_by_name() {
  local wanted_name=$1
  local index

  for index in "${!child_names[@]}"; do
    if [[ "${child_names[index]}" == "$wanted_name" ]]; then
      if process_alive "${child_pids[index]}"; then
        log "STOP $wanted_name pid=${child_pids[index]}"
        fruit_stop_process_group "${child_pids[index]}" "$wanted_name" 5 2
      fi
      child_pids[index]=""
      return 0
    fi
  done
  return 0
}

stop_children() {
  local exit_status=$?

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

  # Ensure arm motion is stopped if planner is available.
  if [[ " ${child_names[*]} " == *" mission "* ]]; then
    timeout --foreground 2s ros2 service call /arm_planner_node/stop std_srvs/srv/Trigger '{}' \
      >/dev/null 2>&1 || true
  fi

  # Release recorder subscriptions before closing the composed task stack.
  fruit_recording_stop
  stop_child_by_name mission

  local -a remaining_pids=()
  local pid
  for pid in "${child_pids[@]}"; do
    if process_alive "$pid"; then
      remaining_pids+=("$pid")
    fi
  done

  if ((${#remaining_pids[@]} == 0)); then
    fruit_logging_close "$exit_status"
    return 0
  fi

  log "STOP stopping remaining launched child process groups"
  fruit_stop_all_process_groups "${remaining_pids[@]}"
  fruit_logging_close "$exit_status"
  log "STOP complete; logs kept at $RUN_DIR"
}

on_signal() {
  log "SIGNAL received; shutting down"
  exit 130
}

trap stop_children EXIT
trap on_signal INT TERM

start_usb_debug_monitors

required_files=(
  "$NAV_LAUNCH" \
  "$PERCEPTION_LAUNCH" \
  "$MISSION_LAUNCH" \
  "$ARM_LAUNCH" \
  "$MISSION_CONFIG" \
  "$SCRIPT_CLIENT_CONFIG" \
  "$WORKSPACE/kill_fruit_processes.sh"
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
log "RUN open loop: $OPEN_LOOP"
log "RUN branch: $(git -C "$WORKSPACE" branch --show-current)"
log "RUN commit: $(git -C "$WORKSPACE" rev-parse --short HEAD)"
log "RUN navigation: fresh Point-LIO in odom"
if ((DEBUG_RECORD_ENABLED)); then
  log "RUN debug rosbag: mission profile enabled; output is under $WORKSPACE/rosbags"
fi

if ! python3 -c \
  'from pathlib import Path; import sys; from fruit_mission.waypoint_route import MissionWaypointRoute; MissionWaypointRoute.from_file(Path(sys.argv[1]), sys.argv[2], sys.argv[3])' \
  "$WAYPOINT_FILE" "$MISSION_FLOW" "$ODOM_FRAME" >> "$ORCHESTRATOR_LOG" 2>&1; then
  die "mission waypoint validation failed; see $ORCHESTRATOR_LOG"
fi
log "READY mission waypoint schema and required A/C points"

log "STOP existing competition stacks and motion clients (safe stationary startup required)"
bash "$WORKSPACE/kill_fruit_processes.sh" --competition \
  >> "$ORCHESTRATOR_LOG" 2>&1 || die "could not stop existing competition processes"

# A remote or unidentified node must not cause duplicate hardware ownership.
check_no_existing_nodes \
    /fruit_mission \
    /livox_ros_driver2 \
    /laserMapping \
    /loam_interface \
    /axis_waypoint_action_server \
    /twist_mux \
    /fruit_perception_container \
    /camera \
    /fruit_perception_node \
    /fruit_qr_planner \
    /arm_planner_node \
    /serial_controller \
    /auto_serial_bridge_container \
    /camera_arm_target_client \
    /joy_linux_node \
    /joy_convert_node

# Visualization is not a motion controller and can remain independently owned.
START_ROBVIZ_BRIDGE=$(run_readiness_client robviz-bridge-start-required)

log "RUN starting composed task stack; launch gates mission readiness"
start_process "mission" \
  ros2 launch "$MISSION_LAUNCH" \
    "use_sim_time:=false" \
    "waypoint_file:=$WAYPOINT_FILE" \
    "mission_flow:=$MISSION_FLOW" \
    "open_loop:=$OPEN_LOOP" \
    "start_robviz_bridge:=$START_ROBVIZ_BRIDGE" \
    "serial_log_level:=$([[ $DEBUG_RECORD_ENABLED -eq 1 ]] && echo debug || echo info)"

if ((DEBUG_RECORD_ENABLED)); then
  # Recording readiness is not a second mission startup gate.
  wait_for_nodes "debug recording camera" 10 /camera /fruit_perception_node
  fruit_recording_start "$RECORD_SCRIPT" mission
fi

log "RUN mission started; press Ctrl-C to stop launched processes"
set +e
# A dead perception/QR/planner process must not tear down the competition.
mission_pid=${child_pids[${#child_pids[@]}-1]}
wait "$mission_pid"
status=$?
set -e
log "EXIT mission status=$status; local subsystem failures did not terminate the runner"
exit "$status"
