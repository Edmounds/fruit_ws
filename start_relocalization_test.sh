#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
MAP_PROFILE_HELPER="$WORKSPACE/scripts/map_profile.sh"
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
LIVOX_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json"
SENSOR_FRAMES_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/sensor_frames.launch.py"
RVIZ_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/rviz/relocalization_test.rviz"
PCL_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"
MAP_ID=""

usage() {
  cat <<'EOF'
Usage: ./start_relocalization_test.sh --map A|B

Start the standalone localization test for the selected field profile.
EOF
}

while (($# > 0)); do
  case "$1" in
    --map)
      (($# >= 2)) || { usage >&2; exit 2; }
      MAP_ID=$2
      shift 2
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

[[ -n "$MAP_ID" ]] || { usage >&2; exit 2; }

pids=()
debug_pids=()
stopped=0

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if ((${#pids[@]} == 0 && ${#debug_pids[@]} == 0)); then
    return
  fi

  printf '\n[relocalization-test] stopping launched processes...\n'
  ((${#pids[@]} == 0)) || kill -INT "${pids[@]}" 2>/dev/null || true
  ((${#debug_pids[@]} == 0)) || kill -INT "${debug_pids[@]}" 2>/dev/null || true
  ((${#pids[@]} == 0)) || wait "${pids[@]}" 2>/dev/null || true
  ((${#debug_pids[@]} == 0)) || wait "${debug_pids[@]}" 2>/dev/null || true
  pids=()
  debug_pids=()
}

start_process() {
  local name=$1
  shift

  printf '[relocalization-test] starting %s\n' "$name"
  "$@" &
  pids+=("$!")
}

start_debug_monitor() {
  printf '[relocalization-test] starting robot debug monitor\n'
  "$WORKSPACE/robot_debug_monitor.sh" relocalization-test &
  debug_pids+=("$!")
}

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u
source "$MAP_PROFILE_HELPER"
resolve_map_profile "$WORKSPACE" "$MAP_ID"
require_map_profile_maps

for required_file in \
  "$PARAMS" \
  "$LIVOX_CONFIG" \
  "$LOCALIZATION_MAP" \
  "$INITIAL_POSE_FILE" \
  "$SENSOR_FRAMES_LAUNCH" \
  "$RVIZ_CONFIG" \
  "$WORKSPACE/robot_debug_monitor.sh" \
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
    -p "localization_map:=$LOCALIZATION_MAP" \
    --params-file "$INITIAL_POSE_FILE"

start_debug_monitor

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG"

printf '[relocalization-test] profile=%s; all processes started. Press Ctrl-C to stop them.\n' "$MAP_ID"
printf '[relocalization-test] initial pose: %s\n' "$INITIAL_POSE_FILE"

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
