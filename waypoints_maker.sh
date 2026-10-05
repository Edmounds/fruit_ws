#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
NAV_LAUNCH="$WORKSPACE/src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
RECORDER_LAUNCH="$WORKSPACE/src/fruit_waypoint_recorder/launch/waypoint_recorder.launch.py"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
WAYPOINT_FILE="$WORKSPACE/src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml"
WAYPOINT_TEMPLATE_FILE="$WORKSPACE/src/fruit_waypoint_recorder/waypoints/template.yaml"
WAYPOINT_FRAME="odom"
RVIZ_CONFIG="$WORKSPACE/src/navigation/fruit_nav_bringup/rviz/waypoints_maker.rviz"
MONITOR_SCRIPT="$WORKSPACE/scripts/robot_debug_monitor.sh"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"
USE_TELEOP=false
DEBUG=0

pids=()
debug_pids=()
stopped=0

usage() {
  cat <<'EOF'
Usage: ./waypoints_maker.sh [--tele] [--debug]

Start Point-LIO odometry navigation, RViz, and the waypoint recorder in the
startup-relative odom frame.
Use --tele when joystick teleop is needed.
EOF
}

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if ((${#pids[@]} == 0 && ${#debug_pids[@]} == 0)); then
    return
  fi

  printf '\n[waypoints-maker] stopping launched processes...\n'
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  for pid in "${debug_pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  ((${#pids[@]} == 0)) || wait "${pids[@]}" 2>/dev/null || true
  ((${#debug_pids[@]} == 0)) || wait "${debug_pids[@]}" 2>/dev/null || true
  fruit_logging_close "${status:-0}"
  pids=()
  debug_pids=()
}

start_process() {
  local name=$1
  shift
  local safe_name="${name// /_}"
  local log_file="${FRUIT_RUNTIME_SESSION_DIR}/${safe_name}.log"
  local filter_script="${WORKSPACE}/scripts/runtime_log_filter.py"

  printf '[waypoints-maker] starting %s\n' "$name"
  setsid env "DEBUG_OPT=$DEBUG" bash -c '
    set -o pipefail
    filter_args=(--output-file "'"${log_file}"'" --component "'"${safe_name}"'")
    [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
    exec "$@" 2>&1 | /usr/bin/python3 "'"${filter_script}"'" "${filter_args[@]}"
  ' runner "$@" &
  pids+=("$!")
}

start_debug_monitor() {
  local log_file="${FRUIT_RUNTIME_SESSION_DIR}/monitor.log"
  local filter_script="${WORKSPACE}/scripts/runtime_log_filter.py"

  printf '[waypoints-maker] starting robot debug monitor\n'
  setsid env "DEBUG_OPT=$DEBUG" bash -c '
    set -o pipefail
    filter_args=(--output-file "'"${log_file}"'" --component "monitor")
    [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
    exec "$@" 2>&1 | /usr/bin/python3 "'"${filter_script}"'" "${filter_args[@]}"
  ' runner "$MONITOR_SCRIPT" waypoints-maker &
  debug_pids+=("$!")
}

wait_for_odom_tf() {
  printf '[waypoints-maker] waiting for odometry nodes\n'
  ros2 run fruit_mission stack_readiness_client \
    nodes --node /laserMapping --node /loam_interface \
    --timeout 60 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"
  printf '[waypoints-maker] waiting for the first valid Point-LIO odometry sample\n'
  ros2 run fruit_mission stack_readiness_client \
    odometry --topic /aft_mapped_to_init --timeout 60 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"
  printf '[waypoints-maker] waiting up to 5s for stable odom -> base_link TF\n'
  if ros2 run fruit_mission stack_readiness_client \
    tf --parent odom --child base_link --timeout 5 --max-age 1 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"; then
    printf '[waypoints-maker] stable odom origin is ready\n'
    return 0
  fi
  printf '[waypoints-maker] stable odom origin was not established within 5s\n' >&2
  return 1
}

trap stop_children INT TERM EXIT

while (($# > 0)); do
  case "$1" in
    --tele)
      USE_TELEOP=true
      shift
      ;;
    --debug)
      DEBUG=1
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

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
set -u

fruit_logging_init "waypoints_maker" "${DEBUG}"

required_files=(
  "$NAV_LAUNCH" \
  "$HARDWARE_LAUNCH" \
  "$RECORDER_LAUNCH" \
  "$WAYPOINT_FILE" \
  "$WAYPOINT_TEMPLATE_FILE" \
  "$SCRIPT_CLIENT_CONFIG" \
  "$MONITOR_SCRIPT" \
  "$RVIZ_CONFIG"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -r "$required_file" ]]; then
    printf '[waypoints-maker] required file is unavailable: %s\n' "$required_file" >&2
    exit 1
  fi
done

if ros2 node list 2>/dev/null | grep -Eq \
  '^/(livox_ros_driver2|laserMapping|loam_interface|axis_waypoint_action_server|fruit_waypoint_recorder)$'; then
  printf '[waypoints-maker] an odometry or waypoint node is already running.\n' >&2
  printf '[waypoints-maker] stop the existing stack before starting this script.\n' >&2
  exit 1
fi

if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
  printf '[waypoints-maker] desktop Xauthority is unavailable: %s\n' "$DESKTOP_XAUTHORITY" >&2
  printf '[waypoints-maker] log in to the local desktop before starting waypoint recording.\n' >&2
  exit 1
fi

START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
  --ros-args --params-file "$SCRIPT_CLIENT_CONFIG")
if [[ "$START_SERIAL_BRIDGE" == true ]]; then
  start_process "serial bridge" \
    ros2 launch "$HARDWARE_LAUNCH"
else
  printf '[waypoints-maker] reusing existing serial bridge; it will not be stopped on exit\n'
fi

if [[ "$USE_TELEOP" == true ]]; then
  nav_process_name="axis navigation, odometry, and teleop"
else
  nav_process_name="axis navigation and odometry"
fi

start_process "$nav_process_name" \
  ros2 launch "$NAV_LAUNCH" \
    "use_teleop:=$USE_TELEOP"

start_debug_monitor

start_process "rviz2 waypoint maker" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG" -f "$WAYPOINT_FRAME"

wait_for_odom_tf

start_process "waypoint recorder" \
  ros2 launch "$RECORDER_LAUNCH"

if [[ "$USE_TELEOP" == true ]]; then
  printf '[waypoints-maker] odometry, teleop, RViz, and waypoint recorder started.\n'
else
  printf '[waypoints-maker] odometry, RViz, and waypoint recorder started.\n'
fi
printf '[waypoints-maker] frame=%s; waypoint_file=%s\n' "$WAYPOINT_FRAME" "$WAYPOINT_FILE"
printf '[waypoints-maker] use the RViz Waypoint Maker panel to record, save, undo, and run the next goal. Press Ctrl-C to stop.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"

