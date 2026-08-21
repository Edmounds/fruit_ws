#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
MAP_PROFILE_HELPER="$WORKSPACE/scripts/map_profile.sh"
LOCALIZATION_NAV_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_navigation_launch.py"
ODOMETRY_NAV_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/odometry_navigation.launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
AXIS_PARAMS="$WORKSPACE/src/fruit_navigation/config/axis_navigation.yaml"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
RVIZ_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/rviz/navigation.rviz"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"
USE_ODOMETRY=false

pids=()
debug_pids=()
stopped=0
MAP_ID=""

usage() {
  cat <<'EOF'
Usage: ./start_navigation.sh --map A|B [--odom]

Start navigation for the selected field profile.
Use --odom to start Point-LIO navigation in the startup-relative odom frame
without loading a map, localization PCD, initial pose, or GICP.
EOF
}

while (($# > 0)); do
  case "$1" in
    --map)
      (($# >= 2)) || { usage >&2; exit 2; }
      MAP_ID=$2
      shift 2
      ;;
    --odom)
      USE_ODOMETRY=true
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

[[ -n "$MAP_ID" ]] || { usage >&2; exit 2; }

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if ((${#pids[@]} == 0 && ${#debug_pids[@]} == 0)); then
    return
  fi

  printf '\n[navigation] stopping launched processes...\n'
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  for pid in "${debug_pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  ((${#pids[@]} == 0)) || wait "${pids[@]}" 2>/dev/null || true
  ((${#debug_pids[@]} == 0)) || wait "${debug_pids[@]}" 2>/dev/null || true
  pids=()
  debug_pids=()
}

start_process() {
  local name=$1
  shift

  printf '[navigation] starting %s\n' "$name"
  setsid "$@" &
  pids+=("$!")
}

start_debug_monitor() {
  printf '[navigation] starting robot debug monitor\n'
  setsid "$WORKSPACE/robot_debug_monitor.sh" navigation &
  debug_pids+=("$!")
}

wait_for_odom_tf() {
  printf '[navigation] waiting for odometry nodes\n'
  ros2 run fruit_mission stack_readiness_client \
    nodes --node /laserMapping --node /loam_interface \
    --node /sensor_scan_generation --timeout 60 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"
  printf '[navigation] waiting for the first valid Point-LIO odometry sample\n'
  ros2 run fruit_mission stack_readiness_client \
    odometry --topic /aft_mapped_to_init --timeout 60 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"
  printf '[navigation] waiting up to 5s for stable odom -> base_link TF\n'
  if ros2 run fruit_mission stack_readiness_client \
    tf --parent odom --child base_link --timeout 5 --max-age 1 \
    --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"; then
    printf '[navigation] stable odom origin is ready\n'
    return 0
  fi
  printf '[navigation] stable odom origin was not established within 5s\n' >&2
  return 1
}

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u
source "$MAP_PROFILE_HELPER"
resolve_map_profile "$WORKSPACE" "$MAP_ID"

if [[ "$USE_ODOMETRY" == true ]]; then
  NAV_LAUNCH="$ODOMETRY_NAV_LAUNCH"
  NAV_FRAME=odom
  require_map_profile_odometry
else
  NAV_LAUNCH="$LOCALIZATION_NAV_LAUNCH"
  NAV_FRAME=map
  require_map_profile_maps
fi

required_files=(
  "$NAV_LAUNCH" \
  "$HARDWARE_LAUNCH" \
  "$PARAMS" \
  "$AXIS_PARAMS" \
  "$WORKSPACE/robot_debug_monitor.sh" \
  "$RVIZ_CONFIG"
)
if [[ "$USE_ODOMETRY" == true ]]; then
  required_files+=("$SCRIPT_CLIENT_CONFIG")
else
  required_files+=("$MAP_YAML" "$LOCALIZATION_MAP" "$INITIAL_POSE_FILE")
fi
for required_file in "${required_files[@]}"; do
  if [[ ! -r "$required_file" ]]; then
    printf '[navigation] required file is unavailable: %s\n' "$required_file" >&2
    exit 1
  fi
done

if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
  printf '[navigation] desktop Xauthority is unavailable: %s\n' "$DESKTOP_XAUTHORITY" >&2
  printf '[navigation] log in to the local desktop before starting navigation.\n' >&2
  exit 1
fi

start_process "serial bridge" \
  ros2 launch "$HARDWARE_LAUNCH"

if [[ "$USE_ODOMETRY" == true ]]; then
  start_process "axis navigation and odometry" \
    ros2 launch "$NAV_LAUNCH" \
      "params_file:=$PARAMS" \
      "axis_params_file:=$AXIS_PARAMS" \
      "use_teleop:=false" \
      "use_rviz:=false"
else
  start_process "axis navigation and localization" \
    ros2 launch "$NAV_LAUNCH" \
      "map:=$MAP_YAML" \
      "localization_map:=$LOCALIZATION_MAP" \
      "params_file:=$PARAMS" \
      "axis_params_file:=$AXIS_PARAMS" \
      "initial_pose_file:=$INITIAL_POSE_FILE" \
      "use_teleop:=false" \
      "use_rviz:=false"
fi

start_debug_monitor

if [[ "$USE_ODOMETRY" == true ]]; then
  wait_for_odom_tf
fi

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG" -f "$NAV_FRAME"

if [[ "$USE_ODOMETRY" == true ]]; then
  printf '[navigation] profile=%s; axis odometry navigation, serial bridge, and RViz started (frame=%s). Press Ctrl-C to stop them.\n' "$MAP_ID" "$NAV_FRAME"
else
  printf '[navigation] profile=%s; axis navigation, serial bridge, and RViz started (frame=%s). Press Ctrl-C to stop them.\n' "$MAP_ID" "$NAV_FRAME"
fi

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
