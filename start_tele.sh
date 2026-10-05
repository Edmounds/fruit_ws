#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
DEBUG_MODE=0

FILTERED_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--debug" ]]; then
    DEBUG_MODE=1
  else
    FILTERED_ARGS+=("$arg")
  fi
done
set -- "${FILTERED_ARGS[@]}"

cleanup() {
  local status=$?
  fruit_logging_close "${status}"
}
trap cleanup EXIT INT TERM

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
set -u

fruit_logging_init "teleop" "${DEBUG_MODE}"

START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
if [[ "$START_SERIAL_BRIDGE" == false ]]; then
  printf '[teleop] reusing existing serial bridge; it will not be stopped on exit\n'
fi

fruit_run_logged "teleop" ros2 launch "$WORKSPACE/src/joy_linux/launch/joy_cmd_vel_serial.launch.py" \
  "start_serial_bridge:=$START_SERIAL_BRIDGE"