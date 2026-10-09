#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
DEBUG_MODE=0

usage() {
  cat <<'EOF'
Usage: ./start_tele.sh [--debug]

Start joystick teleop and the serial bridge. Reuses existing serial bridge
if already running. Press Ctrl-C to stop.
EOF
}

FILTERED_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--debug" ]]; then
    DEBUG_MODE=1
  elif [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
    usage
    exit 0
  else
    FILTERED_ARGS+=("$arg")
  fi
done
set -- "${FILTERED_ARGS[@]}"

launch_pid=""
stopped=0

cleanup() {
  trap - EXIT INT TERM
  if ((stopped)); then
    return
  fi
  stopped=1
  local status=$?
  if [[ -n "${launch_pid}" ]]; then
    printf '\n[teleop] stopping teleop launch...\n'
    fruit_stop_process_group "${launch_pid}" "teleop" 5 2
  fi
  fruit_logging_close "${status}"
}
trap cleanup EXIT INT TERM

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
set -u

fruit_logging_init "teleop" "${DEBUG_MODE}"

if ros2 run fruit_mission stack_readiness_client node-running \
  --node /joy_node --node /joy_conventor --match any \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml" 2>/dev/null; then
  printf '[teleop] a teleop node is already running.\n' >&2
  exit 1
fi

START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
if [[ "$START_SERIAL_BRIDGE" == false ]]; then
  printf '[teleop] reusing existing serial bridge; it will not be stopped on exit\n'
fi

log_file="${FRUIT_RUNTIME_SESSION_DIR}/teleop.log"
filter_script="${WORKSPACE}/scripts/runtime_log_filter.py"

setsid env "DEBUG_OPT=$DEBUG_MODE" bash -c '
  set -o pipefail
  filter_args=(--output-file "'"${log_file}"'" --component "teleop")
  [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
  exec ros2 launch "'"$WORKSPACE"'/src/joy_linux/launch/joy_cmd_vel_serial.launch.py" \
    "start_serial_bridge:='"$START_SERIAL_BRIDGE"'" 2>&1 | /usr/bin/python3 "'"${filter_script}"'" "${filter_args[@]}"
' runner &
launch_pid=$!

if wait "$launch_pid"; then
  status=0
else
  status=$?
fi

cleanup
exit "$status"