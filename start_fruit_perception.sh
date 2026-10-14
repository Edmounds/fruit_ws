#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
DEBUG_MODE=0
ZONE=""

usage()
{
  printf 'Usage: ./start_fruit_perception.sh --zone A|B|C [--debug] [launch_argument:=value ...]\n'
}

LAUNCH_ARGS=()
while (($# > 0)); do
  case "$1" in
    --zone)
      if (($# < 2)) || [[ -n "$ZONE" ]] || [[ "$2" != A && "$2" != B && "$2" != C ]]; then
        usage >&2
        exit 2
      fi
      ZONE=$2
      shift 2
      ;;
    --debug)
      DEBUG_MODE=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    zone:=*)
      printf 'Use --zone A|B|C, not zone:=value\n' >&2
      exit 2
      ;;
    *:=*)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done
if [[ -z "$ZONE" ]]; then
  usage >&2
  exit 2
fi
set -- "${LAUNCH_ARGS[@]}" "zone:=$ZONE"

launch_pid=""
robviz_pid=""
stopped=0

cleanup()
{
  trap - EXIT INT TERM
  if ((stopped)); then
    return
  fi
  stopped=1
  local status=$?
  if [[ -n "${launch_pid}" ]]; then
    printf '\n[perception] stopping perception launch...\n'
    fruit_stop_process_group "${launch_pid}" "perception" 5 2
  fi
  if [[ -n "${robviz_pid}" ]]; then
    fruit_stop_process_group "${robviz_pid}" "robviz" 5 2
  fi
  fruit_recording_stop
  fruit_logging_close "${status}"
}

on_signal()
{
  cleanup
  exit 130
}

process_alive()
{
  local state
  state="$(ps -o stat= -p "$1" 2>/dev/null || true)"
  [[ -n "$state" && "$state" != Z* ]]
}

report_robviz_exit()
{
  local status=$1
  local message="[perception] WARNING: RobViz exited while perception continued (status=${status}); see robviz.log"
  printf '%s\n' "$message" | tee -a "${FRUIT_RUNTIME_SESSION_DIR}/orchestrator.log" >&2
}

trap cleanup EXIT
trap on_signal INT TERM

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
source "$WORKSPACE/scripts/fruit_runtime_recording.bash"
set -u

fruit_logging_init "perception" "${DEBUG_MODE}"
if (( DEBUG_MODE )); then
  fruit_recording_prepare perception
fi

if ros2 run fruit_mission stack_readiness_client node-running \
  --node /fruit_perception_node --node /camera --match any \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml" 2>/dev/null; then
  printf '[perception] fruit_perception is already running. Stop existing perception first or reuse it.\n' >&2
  exit 1
fi

START_ROBVIZ_BRIDGE=$(ros2 run fruit_mission stack_readiness_client robviz-bridge-start-required \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
if [[ "$START_ROBVIZ_BRIDGE" == false ]]; then
  printf '[perception] reusing existing robviz bridge; it will not be stopped on exit\n'
fi

log_file="${FRUIT_RUNTIME_SESSION_DIR}/perception_launch.log"
filter_script="${WORKSPACE}/scripts/runtime_log_filter.py"

setsid env "DEBUG_OPT=$DEBUG_MODE" bash -c '
  set -o pipefail
  filter_args=(--output-file "'"${log_file}"'" --component "perception_launch")
  [[ "$DEBUG_OPT" == "1" ]] && filter_args+=(--debug)
  exec ros2 launch fruit_perception fruit_perception.launch.py "$@" 2>&1 | /usr/bin/python3 "'"${filter_script}"'" "${filter_args[@]}"
' runner "$@" &
launch_pid=$!

if [[ "$START_ROBVIZ_BRIDGE" == true ]]; then
  setsid bash -c '
    set -o pipefail
    ros2 launch fruit_robviz_bridge fruit_robviz_bridge.launch.py 2>&1 |
      /usr/bin/python3 "'"${filter_script}"'" --output-file "'"${FRUIT_RUNTIME_SESSION_DIR}"'/robviz.log" --component robviz
  ' &
  robviz_pid=$!
fi

if (( DEBUG_MODE )); then
  ros2 run fruit_mission stack_readiness_client nodes \
    --node /camera --node /fruit_perception_node --match all --timeout 10 \
    --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml" \
    >/dev/null 2>&1 || printf '[perception] perception readiness timed out; recording available topics only\n'
  fruit_recording_start "$WORKSPACE/record_rosbag.sh" perception
  fruit_recording_wait_ready 10 || true
fi

while process_alive "$launch_pid"; do
  if [[ -n "$robviz_pid" ]] && ! process_alive "$robviz_pid"; then
    if wait "$robviz_pid"; then
      robviz_status=0
    else
      robviz_status=$?
    fi
    report_robviz_exit "$robviz_status"
    fruit_stop_process_group "$robviz_pid" "robviz" 5 2
    robviz_pid=""
  fi
  sleep 0.1
done

if wait "$launch_pid"; then
  status=0
else
  status=$?
fi

cleanup
exit "$status"
