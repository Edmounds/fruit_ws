#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
DEBUG_PID=""
DEBUG_MODE=0

# Parse --debug flag if present
FILTERED_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--debug" ]]; then
    DEBUG_MODE=1
  else
    FILTERED_ARGS+=("$arg")
  fi
done
set -- "${FILTERED_ARGS[@]}"

cleanup()
{
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$DEBUG_PID" ]]; then
    kill -INT "$DEBUG_PID" 2>/dev/null || true
    wait "$DEBUG_PID" 2>/dev/null || true
  fi
  fruit_logging_close "${status}"
}

on_signal()
{
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
set -u

fruit_logging_init "perception" "${DEBUG_MODE}"

START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
if [[ "$START_SERIAL_BRIDGE" == false ]]; then
  printf '[perception] reusing existing serial bridge; it will not be stopped on exit\n'
  set -- "$@" start_serial_bridge:=false
fi

local_monitor_log="${FRUIT_RUNTIME_SESSION_DIR}/monitor.log"
local_filter_script="${WORKSPACE}/scripts/runtime_log_filter.py"

"$WORKSPACE/scripts/robot_debug_monitor.sh" perception 2>&1 | \
  /usr/bin/python3 "${local_filter_script}" --output-file "${local_monitor_log}" --component "monitor" $([[ "$DEBUG_MODE" == "1" ]] && echo "--debug") &
DEBUG_PID=$!

if [[ "${FRUIT_RUNTIME_TOP_ENTRY:-0}" == "1" ]]; then
  fruit_run_logged "perception_launch" ros2 launch fruit_perception fruit_perception.launch.py "$@"
else
  ros2 launch fruit_perception fruit_perception.launch.py "$@"
fi
