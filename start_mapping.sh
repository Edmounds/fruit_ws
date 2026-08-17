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
save_map_on_stop=0

save_2d_map() {
  local output_prefix="$GLIM_DUMP_PATH/field"

  printf '\n[mapping] saving current /map before stopping GLIM...\n'
  if ! timeout --signal=TERM 20s \
    ros2 run nav2_map_server map_saver_cli \
      -t /map \
      -f "$output_prefix" \
      --fmt pgm \
      --free 0.25 \
      --occ 0.65; then
    printf '[mapping] ERROR: failed to save /map; 3D shutdown will continue, but this dump has no valid 2D map.\n' >&2
    return 1
  fi

  if [[ ! -s "$output_prefix.pgm" || ! -s "$output_prefix.yaml" ]]; then
    printf '[mapping] ERROR: map_saver_cli returned without valid field.pgm and field.yaml files.\n' >&2
    return 1
  fi

  printf '[mapping] saved 2D map to %s.{pgm,yaml}\n' "$output_prefix"
}

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  local status=0
  if ((save_map_on_stop)); then
    save_2d_map || status=1
  fi

  if ((${#pids[@]} == 0)); then
    return "$status"
  fi

  printf '\n[mapping] stopping launched processes...\n'
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
  pids=()
  return "$status"
}

handle_signal() {
  save_map_on_stop=1
  if stop_children; then
    exit 130
  fi
  exit 1
}

start_process() {
  local name=$1
  shift

  printf '[mapping] starting %s\n' "$name"
  setsid "$@" &
  pids+=("$!")
}

trap handle_signal INT TERM
trap stop_children EXIT

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
  ros2 launch "$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/glim_mapping.launch.py" \
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

printf '[mapping] all mapping processes started. Press Ctrl-C to save 2D/3D maps and stop.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
