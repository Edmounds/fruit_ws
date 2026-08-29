#!/usr/bin/env bash

set -Eeo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_SETUP="${WORKSPACE_DIR}/scripts/setup_fruit_runtime.bash"

CLIENT_CONFIG="${WORKSPACE_DIR}/src/fruit_mission/config/script_ros_clients.yaml"
HARDWARE_LAUNCH="${WORKSPACE_DIR}/src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py"
RUN_DIR="${WORKSPACE_DIR}/tmp/zone-witch"
BRIDGE_LOG="${RUN_DIR}/bridge.log"

BRIDGE_PID=""
STOPPING=0

usage() {
  cat <<'EOF'
Usage: ./zone_witch.sh --A|--B|--C|--D

Send the current-area command through the ROS serial bridge and publish the
same area on /fruit_mission/current_area.
EOF
}

process_is_alive() {
  local pid=$1
  local state=""

  [[ -n "${pid}" ]] || return 1
  state=$(ps -o stat= -p "${pid}" 2>/dev/null) || return 1
  state=${state//[[:space:]]/}
  [[ -n "${state}" && "${state}" != Z* ]]
}

stop_bridge() {
  local deadline=0

  process_is_alive "${BRIDGE_PID}" || return 0
  kill -INT -- "-${BRIDGE_PID}" 2>/dev/null || true
  deadline=$((EPOCHSECONDS + 8))
  while process_is_alive "${BRIDGE_PID}" && (( EPOCHSECONDS < deadline )); do
    sleep 0.2
  done
  if process_is_alive "${BRIDGE_PID}"; then
    kill -TERM -- "-${BRIDGE_PID}" 2>/dev/null || true
  fi
  deadline=$((EPOCHSECONDS + 2))
  while process_is_alive "${BRIDGE_PID}" && (( EPOCHSECONDS < deadline )); do
    sleep 0.2
  done
  if process_is_alive "${BRIDGE_PID}"; then
    kill -KILL -- "-${BRIDGE_PID}" 2>/dev/null || true
  fi
  wait "${BRIDGE_PID}" 2>/dev/null || true
}

cleanup() {
  local status=$?

  (( STOPPING == 0 )) || return
  STOPPING=1
  trap - EXIT INT TERM
  stop_bridge
  exit "${status}"
}

if (($# != 1)); then
  usage >&2
  exit 2
fi

case "$1" in
  --A|--a)
    AREA=A
    ;;
  --B|--b)
    AREA=B
    ;;
  --C|--c)
    AREA=C
    ;;
  --D|--d)
    AREA=D
    ;;
  --help|-h)
    usage
    exit 0
    ;;
  *)
    printf 'Unknown area option: %s\n' "$1" >&2
    usage >&2
    exit 2
    ;;
esac

for required_file in \
  "${RUNTIME_SETUP}" \
  "${CLIENT_CONFIG}" \
  "${HARDWARE_LAUNCH}"; do
  if [[ ! -r "${required_file}" ]]; then
    printf 'Required project file is unavailable: %s\n' "${required_file}" >&2
    exit 1
  fi
done

# The runtime setup must be sourced before nounset is enabled.
source "${RUNTIME_SETUP}"
set -u

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set +e
ros2 run fruit_mission current_area_client \
  --probe-bridge --ros-args --params-file "${CLIENT_CONFIG}" \
  >/dev/null 2>&1
probe_status=$?
set -e

case "${probe_status}" in
  0)
    printf 'Using the existing ROS serial bridge.\n'
    ;;
  10)
    mkdir -p "${RUN_DIR}"
    : >"${BRIDGE_LOG}"
    setsid ros2 launch "${HARDWARE_LAUNCH}" \
      > >(tee -a "${BRIDGE_LOG}") 2>&1 &
    BRIDGE_PID=$!
    printf 'Started a temporary ROS serial bridge (pid=%s).\n' "${BRIDGE_PID}"
    ;;
  *)
    printf 'Failed to inspect the ROS serial bridge (exit=%s).\n' \
      "${probe_status}" >&2
    exit 1
    ;;
esac

ros2 run fruit_mission current_area_client \
  --area "${AREA}" --ros-args --params-file "${CLIENT_CONFIG}" || exit 1
printf 'Published %s to /fruit_mission/current_area\n' "${AREA}"
