#!/usr/bin/env bash

set -Eeo pipefail

readonly WORKSPACE_DIR="/home/rc4/fruit_ws"
readonly RUNTIME_SETUP="${WORKSPACE_DIR}/setup_fruit_runtime.bash"
readonly START_SCRIPT="${WORKSPACE_DIR}/start_fruit_perception.sh"
readonly CLIENT_CONFIG="${WORKSPACE_DIR}/src/fruit_mission/config/script_ros_clients.yaml"
readonly RUN_DIR="${WORKSPACE_DIR}/tmp/d435i-arm-target"
readonly LAUNCH_LOG="${RUN_DIR}/launch.log"
readonly CLIENT_LOG="${RUN_DIR}/client.log"

LAUNCH_PID=""
STOPPING=0
ZONE=""
DIRECTION=""
DEBUG=0

log()
{
  printf '[d435i-arm] %s\n' "$*"
}

usage()
{
  cat <<'EOF'
Usage: ./test_d435i_arm_target.sh --zone A|C --left|--right [--debug]

Select the current area and observation direction before running the existing
D435i recognition and arm-target workflow. --debug prints every D435i detection
message and every selected target point with their ROS timestamps.
EOF
}

parse_arguments()
{
  while (( $# > 0 )); do
    case "$1" in
      --zone)
        (( $# >= 2 )) || { log "ERROR: --zone requires A or C"; return 2; }
        [[ -z "${ZONE}" ]] || { log "ERROR: --zone may only be specified once"; return 2; }
        case "$2" in
          A|a) ZONE=A ;;
          C|c) ZONE=C ;;
          *) log "ERROR: --zone must be A or C"; return 2 ;;
        esac
        shift 2
        ;;
      --left|--right)
        [[ -z "${DIRECTION}" ]] || {
          log "ERROR: choose exactly one of --left or --right"
          return 2
        }
        DIRECTION=$1
        shift
        ;;
      --debug)
        DEBUG=1
        shift
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      *)
        log "ERROR: unknown option: $1"
        return 2
        ;;
    esac
  done

  [[ -n "${ZONE}" ]] || { log "ERROR: --zone A or --zone C is required"; return 2; }
  [[ -n "${DIRECTION}" ]] || {
    log "ERROR: --left or --right is required"
    return 2
  }
}

process_is_alive()
{
  local pid=$1
  local state=""

  [[ -n "${pid}" ]] || return 1
  state=$(ps -o stat= -p "${pid}" 2>/dev/null) || return 1
  state=${state//[[:space:]]/}
  [[ -n "${state}" && "${state}" != Z* ]]
}

stop_process_group()
{
  local pid=$1
  local deadline=0

  process_is_alive "${pid}" || return 0
  kill -INT -- "-${pid}" 2>/dev/null || true
  deadline=$((EPOCHSECONDS + 8))
  while process_is_alive "${pid}" && (( EPOCHSECONDS < deadline )); do
    sleep 0.2
  done
  if process_is_alive "${pid}"; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
  deadline=$((EPOCHSECONDS + 2))
  while process_is_alive "${pid}" && (( EPOCHSECONDS < deadline )); do
    sleep 0.2
  done
  if process_is_alive "${pid}"; then
    kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup()
{
  local status=$?
  local job_pid=""

  (( STOPPING == 0 )) || return
  STOPPING=1
  trap - EXIT INT TERM
  log "Stopping perception stack"

  while read -r job_pid; do
    [[ -n "${job_pid}" && "${job_pid}" != "${LAUNCH_PID}" ]] || continue
    kill -TERM "${job_pid}" 2>/dev/null || true
  done < <(jobs -pr)

  if process_is_alive "${LAUNCH_PID}"; then
    timeout --foreground 4s ros2 run fruit_mission d435i_arm_target_client \
      --mode-off --ros-args --params-file "${CLIENT_CONFIG}" \
      >/dev/null 2>&1 || true
  fi
  stop_process_group "${LAUNCH_PID}"
  exit "${status}"
}

show_failure_logs()
{
  printf '\nLast perception output:\n' >&2
  tail -n 40 "${LAUNCH_LOG}" >&2 || true
}

check_prerequisites()
{
  local device=""
  local realsense_devices=""

  [[ -f "${RUNTIME_SETUP}" ]] || { log "ERROR: missing ${RUNTIME_SETUP}"; return 1; }
  [[ -x "${START_SCRIPT}" ]] || { log "ERROR: ${START_SCRIPT} is not executable"; return 1; }
  [[ -r "${CLIENT_CONFIG}" ]] || { log "ERROR: missing ${CLIENT_CONFIG}"; return 1; }
  for device in /dev/left_cam /dev/right_cam /dev/stm32; do
    [[ -e "${device}" ]] || { log "ERROR: missing device ${device}"; return 1; }
  done
  command -v rs-enumerate-devices >/dev/null || {
    log "ERROR: rs-enumerate-devices is not installed"
    return 1
  }
  realsense_devices=$(rs-enumerate-devices -s 2>/dev/null) || return 1
  grep -Fq 'Intel RealSense D435I' <<<"${realsense_devices}" || {
    log "ERROR: Intel RealSense D435I was not found"
    return 1
  }
}

keep_running()
{
  local stack_failure_reported=0

  log "Perception stack remains resident. Press Ctrl-C to stop."
  while true; do
    if (( stack_failure_reported == 0 )) && ! process_is_alive "${LAUNCH_PID}"; then
      stack_failure_reported=1
      log "ERROR: perception stack exited"
      tail -n 40 "${LAUNCH_LOG}" >&2 || true
    fi
    sleep 1
  done
}

main()
{
  local -a client_debug_args=()

  parse_arguments "$@" || {
    usage >&2
    exit 2
  }

  mkdir -p "${RUN_DIR}"
  : >"${LAUNCH_LOG}"
  : >"${CLIENT_LOG}"

  # shellcheck source=/home/rc4/fruit_ws/setup_fruit_runtime.bash
  source "${RUNTIME_SETUP}"
  set -u
  check_prerequisites || exit 1
  if (( DEBUG != 0 )); then
    client_debug_args+=(--debug)
    log "Per-frame D435i detection and target-point logging is enabled"
  fi
  ros2 run fruit_mission d435i_arm_target_client \
    --require-stack-absent --ros-args --params-file "${CLIENT_CONFIG}" || {
    log "ERROR: perception or serial nodes are already running"
    exit 1
  }

  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  setsid "${START_SCRIPT}" serial_debug_raw_frame:=true \
    > >(tee -a "${LAUNCH_LOG}") 2>&1 &
  LAUNCH_PID=$!

  log "Waiting for the perception stack"
  if ros2 run fruit_mission d435i_arm_target_client \
    --zone "${ZONE}" "${DIRECTION}" "${client_debug_args[@]}" \
    --ros-args --params-file "${CLIENT_CONFIG}" \
    2>&1 | tee -a "${CLIENT_LOG}"; then
    log "Task sequence completed"
  else
    log "Task sequence stopped at the failed step; processes remain available for inspection"
    show_failure_logs
  fi
  keep_running
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
