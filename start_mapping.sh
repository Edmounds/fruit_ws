#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
GLIM_DUMP_PATH="$WORKSPACE/tmp/glim_map"
LIVOX_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json"
RVIZ_CONFIG="$WORKSPACE/src/navigation/glim_ros2/rviz/glim_ros.rviz"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"

pids=()
stopped=0

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if ((${#pids[@]} == 0)); then
    return
  fi

  printf '\n[mapping] stopping launched processes...\n'
  kill -INT "${pids[@]}" 2>/dev/null || true
  wait "${pids[@]}" 2>/dev/null || true
  pids=()
}

start_process() {
  local name=$1
  shift

  printf '[mapping] starting %s\n' "$name"
  "$@" &
  pids+=("$!")
}

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
  printf '[mapping] desktop Xauthority is unavailable: %s\n' "$DESKTOP_XAUTHORITY" >&2
  printf '[mapping] log in to the local desktop before starting mapping.\n' >&2
  exit 1
fi

if [[ -d "$GLIM_DUMP_PATH" ]] && [[ -n "$(find "$GLIM_DUMP_PATH" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  previous_dump_path="${GLIM_DUMP_PATH}_$(date +%Y%m%d_%H%M%S)"
  mv "$GLIM_DUMP_PATH" "$previous_dump_path"
  printf '[mapping] archived previous dump to %s\n' "$previous_dump_path"
fi
mkdir -p "$GLIM_DUMP_PATH"

start_process "livox_ros_driver2" \
  ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -r __node:=livox_ros_driver2 \
    -p xfer_format:=0 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:=front_mid360 \
    -p "user_config_path:=$LIVOX_CONFIG"

start_process "glim_ros" \
  ros2 launch "$WORKSPACE/src/navigation/glim_ros2/launch/mapping.launch.py" \
    "dump_path:=$GLIM_DUMP_PATH"

start_process "auto_serial_bridge" \
  ros2 launch "$WORKSPACE/src/auto_serial_bridge/launch/serial_bridge_by_component.launch.py"

start_process "joy_linux" \
  ros2 launch "$WORKSPACE/src/joy_linux/launch/joy_conventor_component.launch.py"

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG"

printf '[mapping] all mapping processes started. Press Ctrl-C to stop them.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
