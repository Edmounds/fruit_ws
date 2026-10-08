#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
NAV_LAUNCH="$WORKSPACE/src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/auto_serial_bridge/launch/auto_serial_bridge.launch.py"
RECORDER_LAUNCH="$WORKSPACE/src/fruit_robviz_bridge/launch/waypoint_recorder.launch.py"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
WEB_URL="http://127.0.0.1:8000/"
USE_TELEOP=false
DEBUG=0

pids=()
stopped=0

usage() {
  cat <<'EOF'
Usage: ./waypoints_maker.sh [--tele] [--debug]

Start Point-LIO odometry navigation and the waypoint recorder in the
startup-relative odom frame. Use RobViz at http://127.0.0.1:8000/ for editing.
Use --tele when joystick teleop is needed.
EOF
}

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if ((${#pids[@]} == 0)); then
    return
  fi

  printf '\n[waypoints-maker] stopping launched processes...\n'
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  ((${#pids[@]} == 0)) || wait "${pids[@]}" 2>/dev/null || true
  fruit_logging_close "${status:-0}"
  pids=()
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

wait_for_odom_tf() {
  printf '[waypoints-maker] waiting for odometry nodes\n'
  ros2 run fruit_mission stack_readiness_client \
    nodes --node /laserMapping --node /loam_interface \
    --timeout 10 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"
  printf '[waypoints-maker] waiting for the first valid Point-LIO odometry sample\n'
  ros2 run fruit_mission stack_readiness_client \
    odometry --topic /aft_mapped_to_init --timeout 10 \
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
  "$SCRIPT_CLIENT_CONFIG"
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

wait_for_odom_tf

start_process "waypoint recorder" \
  ros2 launch "$RECORDER_LAUNCH"

if [[ "$USE_TELEOP" == true ]]; then
  printf '[waypoints-maker] odometry, teleop, and waypoint recorder started.\n'
else
  printf '[waypoints-maker] odometry and waypoint recorder started.\n'
fi
printf '[waypoints-maker] use RobViz at %s to record, save, undo, and run the next goal.\n' "$WEB_URL"
printf '[waypoints-maker] Press Ctrl-C to stop.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"

