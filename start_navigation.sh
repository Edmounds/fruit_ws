#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
NAV_LAUNCH="$WORKSPACE/src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py"
HARDWARE_LAUNCH="$WORKSPACE/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
RVIZ_CONFIG="$WORKSPACE/src/navigation/fruit_nav_bringup/rviz/navigation.rviz"
MONITOR_SCRIPT="$WORKSPACE/scripts/robot_debug_monitor.sh"
DESKTOP_RUNTIME_DIR="/run/user/$(id -u)"
DESKTOP_DISPLAY=:0
DESKTOP_XAUTHORITY="$DESKTOP_RUNTIME_DIR/gdm/Xauthority"

pids=()
debug_pids=()
stopped=0

usage() {
  cat <<'EOF'
Usage: ./start_navigation.sh

Start Point-LIO navigation, serial bridge, and RViz in the startup-relative
odom frame.
EOF
}

while (($# > 0)); do
  case "$1" in
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
  setsid "$MONITOR_SCRIPT" navigation &
  debug_pids+=("$!")
}

wait_for_odom_tf() {
  printf '[navigation] waiting for odometry nodes\n'
  ros2 run fruit_mission stack_readiness_client \
    nodes --node /laserMapping --node /loam_interface \
    --timeout 60 \
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

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
set -u

required_files=(
  "$NAV_LAUNCH" \
  "$HARDWARE_LAUNCH" \
  "$SCRIPT_CLIENT_CONFIG" \
  "$MONITOR_SCRIPT" \
  "$RVIZ_CONFIG"
)
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

start_process "axis navigation and odometry" \
  ros2 launch "$NAV_LAUNCH"

start_debug_monitor

wait_for_odom_tf

start_process "rviz2" \
  env \
    "DISPLAY=$DESKTOP_DISPLAY" \
    "XAUTHORITY=$DESKTOP_XAUTHORITY" \
    "XDG_RUNTIME_DIR=$DESKTOP_RUNTIME_DIR" \
    "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
    rviz2 -d "$RVIZ_CONFIG" -f "odom"

printf '[navigation] axis odometry navigation, serial bridge, and RViz started (frame=odom). Press Ctrl-C to stop them.\n'

if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"

