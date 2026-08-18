#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
NAV_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_navigation_launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
RECORDER_LAUNCH="$WORKSPACE/src/fruit_waypoint_recorder/launch/waypoint_recorder.launch.py"
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
AXIS_PARAMS="$WORKSPACE/src/fruit_navigation/config/axis_navigation.yaml"
MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.yaml"
LOCALIZATION_MAP="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.pcd"
RVIZ_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/rviz/waypoints_maker.rviz"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"
USE_TELEOP=false

pids=()
debug_pids=()
stopped=0

usage() {
  cat <<'EOF'
Usage: ./waypoints_maker.sh [--tele]

Start localization, axis navigation, RViz, and the waypoint recorder.
Use --tele when joystick teleop is needed.
EOF
}

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if ((${#pids[@]} == 0 && ${#debug_pids[@]} == 0)); then
    return
  fi

  printf '\n[waypoints-maker] stopping launched processes...\n'
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

  printf '[waypoints-maker] starting %s\n' "$name"
  setsid "$@" &
  pids+=("$!")
}

start_debug_monitor() {
  printf '[waypoints-maker] starting robot debug monitor\n'
  setsid "$WORKSPACE/robot_debug_monitor.sh" waypoints-maker &
  debug_pids+=("$!")
}

trap stop_children INT TERM EXIT

while (($# > 0)); do
  case "$1" in
    --tele)
      USE_TELEOP=true
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

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

for required_file in \
  "$NAV_LAUNCH" \
  "$HARDWARE_LAUNCH" \
  "$RECORDER_LAUNCH" \
  "$PARAMS" \
  "$AXIS_PARAMS" \
  "$MAP" \
  "$LOCALIZATION_MAP" \
  "$WORKSPACE/robot_debug_monitor.sh" \
  "$RVIZ_CONFIG"; do
  if [[ ! -r "$required_file" ]]; then
    printf '[waypoints-maker] required file is unavailable: %s\n' "$required_file" >&2
    exit 1
  fi
done

if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
  printf '[waypoints-maker] desktop Xauthority is unavailable: %s\n' "$DESKTOP_XAUTHORITY" >&2
  printf '[waypoints-maker] log in to the local desktop before starting waypoint recording.\n' >&2
  exit 1
fi

start_process "serial bridge" \
  ros2 launch "$HARDWARE_LAUNCH"

if [[ "$USE_TELEOP" == true ]]; then
  nav_process_name="axis navigation, localization, and teleop"
else
  nav_process_name="axis navigation and localization"
fi

start_process "$nav_process_name" \
  ros2 launch "$NAV_LAUNCH" \
    "map:=$MAP" \
    "localization_map:=$LOCALIZATION_MAP" \
    "params_file:=$PARAMS" \
    "axis_params_file:=$AXIS_PARAMS" \
    "start_route_client:=false" \
    "use_teleop:=$USE_TELEOP" \
    "use_rviz:=false"

start_debug_monitor

start_process "rviz2 waypoint maker" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG"

start_process "waypoint recorder" \
  ros2 launch "$RECORDER_LAUNCH"

if [[ "$USE_TELEOP" == true ]]; then
  printf '[waypoints-maker] navigation, teleop, RViz, and waypoint recorder started.\n'
else
  printf '[waypoints-maker] navigation, RViz, and waypoint recorder started.\n'
fi
printf '[waypoints-maker] use the RViz Waypoint Maker panel to record, save, undo, and test points. Press Ctrl-C to stop.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
