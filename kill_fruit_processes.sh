#!/usr/bin/env bash

set -u
set -o pipefail

target_pids=()

# These tokens cover the launch files and executable nodes in the current
# perception, mapping, localization, navigation, and teleop design.
target_tokens=(
  "ros2 launch fruit_perception "
  "ros2 launch fruit_hardware_bridge "
  "ros2 launch fruit_qr_planner "
  "ros2 launch auto_serial_bridge "
  "ros2 launch glim_ros "
  "ros2 launch pb2025_nav_bringup "
  "ros2 launch point_lio "
  "ros2 launch loam_interface "
  "ros2 launch sensor_scan_generation "
  "ros2 launch joy_linux "
  "ros2 launch livox_ros_driver2 "
  "__node:=fruit_perception_container"
  "__node:=auto_serial_bridge_container"
  "__node:=joy_container"
  "__node:=base_to_front_mid360"
  "__node:=front_mid360_to_livox_imu"
  "/lib/fruit_perception/"
  "/lib/fruit_qr_planner/"
  "/lib/fruit_hardware_bridge/"
  "/lib/auto_serial_bridge/"
  "/lib/glim_ros/"
  "/lib/livox_ros_driver2/"
  "/lib/point_lio/"
  "/lib/loam_interface/"
  "/lib/sensor_scan_generation/"
  "/lib/terrain_analysis/"
  "/lib/terrain_analysis_ext/"
  "/lib/pb_gicp_localization/"
  "/lib/pb_glim_map_tools/"
  "/lib/nav2_"
  "/lib/twist_mux/"
  "/lib/joy_linux/"
  "/lib/tf2_ros/static_transform_publisher"
  "/lib/tf2_ros/tf2_echo"
  "rviz2"
  "ros2 run fruit_perception "
  "ros2 run fruit_qr_planner "
  "ros2 run fruit_hardware_bridge "
  "ros2 run auto_serial_bridge "
  "ros2 run glim_ros "
  "ros2 run livox_ros_driver2 "
  "ros2 run point_lio "
  "ros2 run loam_interface "
  "ros2 run sensor_scan_generation "
  "ros2 run terrain_analysis "
  "ros2 run terrain_analysis_ext "
  "ros2 run pb_gicp_localization "
  "ros2 run nav2_"
  "ros2 run joy_linux "
  "ros2 run rviz2 "
  "ros2 run pb_glim_map_tools "
  "ros2 run tf2_ros static_transform_publisher "
  "ros2 run tf2_ros tf2_echo "
)

usage() {
  cat <<'EOF'
Usage: ./kill_fruit_processes.sh [--dry-run]

Stop nodes and launch processes in the current fruit robot project design.
This includes perception, mapping, localization, navigation, teleop, and RViz.
foxglove_bridge is intentionally excluded.

--dry-run  Print matching processes without sending signals.
EOF
}

is_target_process() {
  local command_line=$1
  local token

  for token in "${target_tokens[@]}"; do
    if [[ "$command_line" == *"$token"* ]]; then
      return 0
    fi
  done

  return 1
}

collect_target_pids() {
  local pid command_line

  while read -r pid command_line; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue

    if is_target_process "$command_line"; then
      printf '%s\n' "$pid"
    fi
  done < <(ps -eo pid=,args=)
}

refresh_targets() {
  mapfile -t target_pids < <(collect_target_pids | sort -n -u)
}

get_target_command() {
  local pid=$1
  ps -p "$pid" -o args= 2>/dev/null
}

is_live_target() {
  local pid=$1
  local command_line

  command_line=$(get_target_command "$pid") || return 1
  is_target_process "$command_line"
}

print_targets() {
  local pid

  for pid in "${target_pids[@]}"; do
    ps -p "$pid" -o pid=,ppid=,stat=,args= 2>/dev/null || true
  done
}

signal_targets() {
  local signal=$1
  local pid command_line

  for pid in "${target_pids[@]}"; do
    command_line=$(get_target_command "$pid") || continue
    is_target_process "$command_line" || continue

    printf 'Sending SIG%s to PID %s\n' "$signal" "$pid"
    kill "-$signal" "$pid" 2>/dev/null || true
  done
}

wait_for_targets() {
  local attempts=$1
  local pid

  while (( attempts > 0 )); do
    for pid in "${target_pids[@]}"; do
      if is_live_target "$pid"; then
        sleep 0.1
        ((attempts--))
        continue 2
      fi
    done
    return 0
  done

  return 1
}

case "${1:-}" in
  "") ;;
  --dry-run) dry_run=1 ;;
  --help|-h)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

if command -v systemctl >/dev/null 2>&1 && \
  systemctl is-active --quiet fruit-perception.service 2>/dev/null; then
  printf 'Warning: fruit-perception.service is active and may restart the launch.\n' >&2
  printf 'Stop it separately with: sudo systemctl stop fruit-perception.service\n' >&2
fi

refresh_targets

if ((${#target_pids[@]} == 0)); then
  printf 'No project processes found.\n'
  exit 0
fi

printf 'Matching project processes:\n'
print_targets

if ((${dry_run:-0} == 1)); then
  exit 0
fi

signal_targets INT
if wait_for_targets 50; then
  printf 'All matching processes stopped gracefully.\n'
  exit 0
fi

refresh_targets
signal_targets TERM
if wait_for_targets 30; then
  printf 'All matching processes stopped after SIGTERM.\n'
  exit 0
fi

refresh_targets
signal_targets KILL
if wait_for_targets 10; then
  printf 'All matching processes stopped after SIGKILL.\n'
  exit 0
fi

refresh_targets
printf 'Some matching processes are still running:\n' >&2
print_targets >&2
exit 1
