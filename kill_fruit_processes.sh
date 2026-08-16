#!/usr/bin/env bash

set -u
set -o pipefail

target_pids=()

usage() {
  cat <<'EOF'
Usage: ./kill_fruit_processes.sh [--dry-run]

Stop processes started by start_fruit_perception.sh, including duplicate
fruit_perception launch instances and their camera/bridge nodes.

--dry-run  Print matching processes without sending signals.
EOF
}

is_target_process() {
  local command_line=$1

  case "$command_line" in
    *"ros2 launch fruit_perception fruit_perception.launch.py"*) return 0 ;;
    *"component_container"*"__node:=fruit_perception_container"*) return 0 ;;
    *"/fruit_perception/lib/fruit_perception/camera_mode_manager"*) return 0 ;;
    *"/fruit_perception/lib/fruit_perception/fruit_perception_node"*) return 0 ;;
    *"/fruit_qr_planner/lib/fruit_qr_planner/fruit_qr_planner_node"*) return 0 ;;
    *"/fruit_hardware_bridge/lib/fruit_hardware_bridge/fruit_result_bridge"*) return 0 ;;
    *"/auto_serial_bridge/lib/auto_serial_bridge/serial_node"*) return 0 ;;
    *"ros2 run fruit_perception fruit_perception_node"*) return 0 ;;
    *"ros2 run fruit_qr_planner fruit_qr_planner_node"*) return 0 ;;
    *"ros2 run fruit_hardware_bridge fruit_result_bridge"*) return 0 ;;
    *"ros2 run auto_serial_bridge serial_node"*) return 0 ;;
  esac

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
  printf 'No fruit perception processes found.\n'
  exit 0
fi

printf 'Matching fruit perception processes:\n'
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
