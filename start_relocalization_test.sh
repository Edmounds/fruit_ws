#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
LIVOX_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json"
LOCALIZATION_MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.pcd"
SENSOR_FRAMES_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/sensor_frames.launch.py"
RVIZ_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/rviz/relocalization_test.rviz"
PCL_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
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

  printf '\n[relocalization-test] stopping launched processes...\n'
  kill -INT "${pids[@]}" 2>/dev/null || true
  wait "${pids[@]}" 2>/dev/null || true
  pids=()
}

start_process() {
  local name=$1
  shift

  printf '[relocalization-test] starting %s\n' "$name"
  "$@" &
  pids+=("$!")
}

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

for required_file in \
  "$PARAMS" \
  "$LIVOX_CONFIG" \
  "$LOCALIZATION_MAP" \
  "$SENSOR_FRAMES_LAUNCH" \
  "$RVIZ_CONFIG" \
  "$PCL_PRELOAD"; do
  if [[ ! -r "$required_file" ]]; then
    printf '[relocalization-test] required file is unavailable: %s\n' "$required_file" >&2
    exit 1
  fi
done

if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
  printf '[relocalization-test] desktop Xauthority is unavailable: %s\n' "$DESKTOP_XAUTHORITY" >&2
  printf '[relocalization-test] log in to the local desktop before starting the test.\n' >&2
  exit 1
fi

start_process "sensor_frames" \
  ros2 launch "$SENSOR_FRAMES_LAUNCH"

start_process "livox_ros_driver2" \
  ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -r __node:=livox_ros_driver2 \
    --params-file "$PARAMS" \
    -p "user_config_path:=$LIVOX_CONFIG"

start_process "point_lio" \
  env "LD_PRELOAD=$PCL_PRELOAD" \
    ros2 run point_lio pointlio_mapping --ros-args \
    -r __node:=laserMapping \
    --params-file "$PARAMS"

start_process "loam_interface" \
  env "LD_PRELOAD=$PCL_PRELOAD" \
    ros2 run loam_interface loam_interface_node --ros-args \
    -r __node:=loam_interface \
    --params-file "$PARAMS"

start_process "sensor_scan_generation" \
  env "LD_PRELOAD=$PCL_PRELOAD" \
    ros2 run sensor_scan_generation sensor_scan_generation_node --ros-args \
    -r __node:=sensor_scan_generation \
    --params-file "$PARAMS"

start_process "pb_gicp_localization" \
  env "LD_PRELOAD=$PCL_PRELOAD" \
    ros2 run pb_gicp_localization pb_gicp_localization_node --ros-args \
    -r __node:=pb_gicp_localization \
    --params-file "$PARAMS" \
    -p "localization_map:=$LOCALIZATION_MAP"

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG"

printf '[relocalization-test] all processes started. Press Ctrl-C to stop them.\n'
printf '[relocalization-test] initial_map_to_odom uses the identity transform from localization_params.yaml\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
