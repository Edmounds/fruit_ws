#!/usr/bin/env bash

set -Eeo pipefail

readonly WORKSPACE_DIR="/home/rc4/fruit_ws"
readonly RUNTIME_SETUP="${WORKSPACE_DIR}/scripts/setup_fruit_runtime.bash"

readonly START_SCRIPT="${WORKSPACE_DIR}/start_fruit_perception.sh"
readonly CLIENT_CONFIG="${WORKSPACE_DIR}/src/fruit_mission/config/script_ros_clients.yaml"
readonly LOGGING_HELPER="${WORKSPACE_DIR}/scripts/fruit_runtime_logging.bash"

LAUNCH_PID=""
PLANNER_PID=""
STOPPING=0
ZONE=""
DIRECTION=""
DEBUG=0

log()
{
  printf '[camera-arm] %s\n' "$*"
}

usage()
{
  cat <<'EOF'
Usage: ./test_camera_arm_target.sh --zone A|B|C --left|--right [--debug]

Test camera and arm coordination.
Arm returns to the home position first, executes observation and grasp,
then executes the zone-specific store preset (e.g. left_store_a or right_store_c)
to release the fruit.
  --zone A|B|C   Competition zone; A/C use vegetable model, B uses fruit model
  --left|--right  Arm side; presets are <side>_look_<zone> and <side>_store_<zone>
  --debug         Print per-frame camera detections and coordinates
EOF
}

parse_arguments()
{
  while (( $# > 0 )); do
    case "$1" in
      --zone|-z)
        (( $# >= 2 )) || { log "ERROR: $1 requires A, B or C"; return 2; }
        [[ -z "${ZONE}" ]] || { log "ERROR: zone may only be specified once"; return 2; }
        case "$2" in
          A|a) ZONE=A ;;
          B|b) ZONE=B ;;
          C|c) ZONE=C ;;
          *) log "ERROR: zone must be A, B or C"; return 2 ;;
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

  [[ -n "${ZONE}" ]] || { log "ERROR: --zone A|B|C is required"; return 2; }
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

  if process_is_alive "${PLANNER_PID}"; then
    timeout --foreground 2s ros2 service call /arm_planner_node/stop std_srvs/srv/Trigger '{}' \
      >/dev/null 2>&1 || true
  fi

  while read -r job_pid; do
    [[ -n "${job_pid}" && "${job_pid}" != "${LAUNCH_PID}" ]] || continue
    kill -TERM "${job_pid}" 2>/dev/null || true
  done < <(jobs -pr)

  stop_process_group "${LAUNCH_PID}"
  stop_process_group "${PLANNER_PID}"
  fruit_logging_close "${status}"
  exit "${status}"
}

show_failure_logs()
{
  if [[ -n "${FRUIT_RUNTIME_SESSION_DIR:-}" ]]; then
    log "Detailed logs available at: ${FRUIT_RUNTIME_SESSION_DIR}/"
  fi
}

check_prerequisites()
{
  local device=""
  local realsense_devices=""

  [[ -f "${RUNTIME_SETUP}" ]] || { log "ERROR: missing ${RUNTIME_SETUP}"; return 1; }
  [[ -x "${START_SCRIPT}" ]] || { log "ERROR: ${START_SCRIPT} is not executable"; return 1; }
  [[ -r "${CLIENT_CONFIG}" ]] || { log "ERROR: missing ${CLIENT_CONFIG}"; return 1; }
  for device in /dev/stm32; do
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
      log "ERROR: perception stack exited unexpectedly"
      show_failure_logs
    fi
    sleep 1
  done
}

main()
{
  local -a client_args=()

  parse_arguments "$@" || {
    usage >&2
    exit 2
  }

  # shellcheck source=/home/rc4/fruit_ws/setup_fruit_runtime.bash
  source "${RUNTIME_SETUP}"
  source "${LOGGING_HELPER}"
  set -u
  fruit_logging_init "camera_arm_target" "${DEBUG}"
  check_prerequisites || exit 1
  if (( DEBUG != 0 )); then
    client_args+=(--debug)
  fi
  ros2 run fruit_mission camera_arm_target_client \
    --require-stack-absent --ros-args --params-file "${CLIENT_CONFIG}" || {
    log "ERROR: perception, arm planner, or mission nodes are already running"
    exit 1
  }

  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  local perception_log="${FRUIT_RUNTIME_SESSION_DIR}/perception.log"
  local planner_log="${FRUIT_RUNTIME_SESSION_DIR}/arm_planner.log"
  local client_log="${FRUIT_RUNTIME_SESSION_DIR}/client.log"

  local filter_script="${WORKSPACE_DIR}/scripts/runtime_log_filter.py"
  local -a perception_cmd=("${START_SCRIPT}")
  if (( DEBUG != 0 )); then
    perception_cmd+=(--debug)
  fi

  setsid bash -c '
    exec "$@" 2>&1 | /usr/bin/python3 "'"${filter_script}"'" --output-file "'"${perception_log}"'" --component "perception" '"$([[ ${DEBUG} -ne 0 ]] && echo "--debug")"'
  ' runner "${perception_cmd[@]}" &
  LAUNCH_PID=$!

  setsid bash -c '
    exec ros2 launch fruit_arm_planner fruit_arm_planner.launch.py log_level:=debug 2>&1 | /usr/bin/python3 "'"${filter_script}"'" --output-file "'"${planner_log}"'" --component "arm_planner"
  ' runner &
  PLANNER_PID=$!

  log "Waiting for the perception stack"
  if ros2 run fruit_mission camera_arm_target_client \
    --zone "${ZONE}" "${DIRECTION}" "${client_args[@]}" \
    --ros-args --params-file "${CLIENT_CONFIG}" \
    2>&1 | /usr/bin/python3 "${filter_script}" --output-file "${client_log}" --component "client" $( (( DEBUG != 0 )) && echo "--debug" ); then
    log "Task sequence completed"
  else
    log "Task sequence stopped at the failed step (arm safe recovery completed); processes remain available for inspection"
    show_failure_logs
  fi
  keep_running
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
