#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
MAP_PROFILE_HELPER="$WORKSPACE/scripts/map_profile.sh"
NAV_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_navigation_launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
AXIS_PARAMS="$WORKSPACE/src/fruit_navigation/config/axis_navigation.yaml"
RVIZ_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/rviz/navigation.rviz"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"

pids=()
debug_pids=()
stopped=0
MAP_ID=""

usage() {
  cat <<'EOF'
Usage: ./start_navigation.sh --map A|B

Start navigation and localization for the selected field profile.
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

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u
source "$MAP_PROFILE_HELPER"
resolve_map_profile "$WORKSPACE" "$MAP_ID"
require_map_profile_runtime

for required_file in \
  "$NAV_LAUNCH" \
  "$HARDWARE_LAUNCH" \
  "$PARAMS" \
  "$AXIS_PARAMS" \
  "$MAP_YAML" \
  "$LOCALIZATION_MAP" \
  "$INITIAL_POSE_FILE" \
  "$WORKSPACE/robot_debug_monitor.sh" \
  "$RVIZ_CONFIG"; do
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

start_process "axis navigation and localization" \
  ros2 launch "$NAV_LAUNCH" \
    "map:=$MAP_YAML" \
    "localization_map:=$LOCALIZATION_MAP" \
    "params_file:=$PARAMS" \
    "axis_params_file:=$AXIS_PARAMS" \
    "initial_pose_file:=$INITIAL_POSE_FILE" \
    "use_teleop:=false" \
    "use_rviz:=false"

start_debug_monitor

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG"

printf '[navigation] profile=%s; axis navigation, serial bridge, and RViz started. Press Ctrl-C to stop them.\n' "$MAP_ID"

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
