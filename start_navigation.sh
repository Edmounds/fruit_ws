#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
NAV_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_navigation_launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml"
MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.yaml"
LOCALIZATION_MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.pcd"
RVIZ_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/rviz/navigation.rviz"
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

  printf '\n[navigation] stopping launched processes...\n'
  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
  pids=()
}

start_process() {
  local name=$1
  shift

  printf '[navigation] starting %s\n' "$name"
  setsid "$@" &
  pids+=("$!")
}

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

for required_file in \
  "$NAV_LAUNCH" \
  "$HARDWARE_LAUNCH" \
  "$PARAMS" \
  "$MAP" \
  "$LOCALIZATION_MAP" \
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

start_process "Nav2 and localization" \
  ros2 launch "$NAV_LAUNCH" \
    "map:=$MAP" \
    "localization_map:=$LOCALIZATION_MAP" \
    "params_file:=$PARAMS" \
    "use_teleop:=false" \
    "use_rviz:=false"

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG"

printf '[navigation] navigation, serial bridge, and RViz started. Press Ctrl-C to stop them.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
