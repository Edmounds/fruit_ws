#!/usr/bin/env bash

set -Eeo pipefail

readonly WORKSPACE_DIR="/home/rc4/fruit_ws"
readonly RUNTIME_SETUP="${WORKSPACE_DIR}/setup_fruit_runtime.bash"
readonly START_SCRIPT="${WORKSPACE_DIR}/start_fruit_perception.sh"
readonly RUN_DIR="${WORKSPACE_DIR}/tmp/d435i-arm-target"
readonly LAUNCH_LOG="${RUN_DIR}/launch.log"

readonly STARTUP_TIMEOUT_SECONDS=45
readonly PUBLISH_TIMEOUT_SECONDS=10
readonly MODE_TIMEOUT_SECONDS=30
readonly CALLBACK_TIMEOUT_SECONDS=60
readonly DETECTION_TIMEOUT_SECONDS=120
readonly TARGET_SAMPLE_COUNT=5

readonly MODE_TOPIC="/fruit_mission/perception_mode"
readonly ACTIVE_MODE_TOPIC="/fruit_perception/active_mode"
readonly CURRENT_AREA_TOPIC="/fruit_mission/current_area"
readonly STATE_COMMAND_TOPIC="/fruit_hardware_bridge/state_machine/command"
readonly D435I_TYPES_TOPIC="/fruit_perception/d435i/types"
readonly D435I_DETECTIONS_TOPIC="/fruit_perception/d435i/detections"
readonly ARM_TARGET_TOPIC="/fruit_hardware_bridge/arm/target"
readonly CALLBACK_TOPIC="/fruit_hardware_bridge/execution/callback"

readonly MODE_TYPE="std_msgs/msg/String"
readonly STATE_COMMAND_TYPE="fruit_hardware_interfaces/msg/StateMachineCommand"
readonly TYPES_TYPE="fruit_perception/msg/FruitTypes"
readonly ARM_TARGET_TYPE="fruit_hardware_interfaces/msg/ArmTarget"
readonly CALLBACK_TYPE="fruit_hardware_interfaces/msg/ExecutionCallback"
readonly TASK_CURRENT_AREA=5
readonly AREA_A=0
readonly CURRENT_AREA_CALLBACK_ID=5

LAUNCH_PID=""
CALLBACK_PID=""
STOPPING=0
TARGET_TYPE=""
TARGET_X=""
TARGET_Y=""
TARGET_Z=""

log()
{
  printf '[d435i-arm] %s\n' "$*"
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
    timeout --foreground 3s ros2 topic pub --once --wait-matching-subscriptions 1 \
      "${MODE_TOPIC}" "${MODE_TYPE}" "{data: 'OFF'}" >/dev/null 2>&1 || true
  fi
  stop_process_group "${LAUNCH_PID}"
  exit "${status}"
}

show_failure_logs()
{
  printf '\nLast perception output:\n' >&2
  tail -n 40 "${LAUNCH_LOG}" >&2 || true
}

publish_once()
{
  local topic=$1
  local message_type=$2
  local message=$3

  timeout --foreground "${PUBLISH_TIMEOUT_SECONDS}s" \
    ros2 topic pub --once --wait-matching-subscriptions 1 \
    "${topic}" "${message_type}" "${message}" >/dev/null
}

publish_mode()
{
  timeout --foreground "${PUBLISH_TIMEOUT_SECONDS}s" \
    ros2 topic pub --once --wait-matching-subscriptions 2 \
    "${MODE_TOPIC}" "${MODE_TYPE}" "{data: '$1'}" >/dev/null
}

wait_for_mode()
{
  local mode=$1

  timeout --foreground "${MODE_TIMEOUT_SECONDS}s" \
    ros2 topic echo --once "${ACTIVE_MODE_TOPIC}" "${MODE_TYPE}" \
    --qos-reliability reliable --qos-durability transient_local \
    --filter "m.data == '${mode}'" >/dev/null
}

subscription_count()
{
  ros2 topic info --no-daemon "$1" 2>/dev/null | \
    awk '/Subscription count:/ {print $3; exit}'
}

wait_for_subscription_count()
{
  local topic=$1
  local minimum=$2
  local timeout_seconds=${3:-5}
  local deadline=$((EPOCHSECONDS + timeout_seconds))
  local count=""

  while (( EPOCHSECONDS < deadline )); do
    count=$(subscription_count "${topic}" || true)
    if [[ "${count}" =~ ^[0-9]+$ ]] && (( count >= minimum )); then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

start_callback_listener()
{
  local callback_id=$1
  local output_file="${RUN_DIR}/callback_${callback_id}.yaml"
  local existing=0

  existing=$(subscription_count "${CALLBACK_TOPIC}" || true)
  [[ "${existing}" =~ ^[0-9]+$ ]] || existing=0
  timeout --foreground "${CALLBACK_TIMEOUT_SECONDS}s" \
    ros2 topic echo --once "${CALLBACK_TOPIC}" "${CALLBACK_TYPE}" \
    --qos-reliability reliable --qos-durability volatile \
    --filter "m.callback_id == ${callback_id} and m.callback_status == 0" \
    >"${output_file}" 2>"${output_file}.err" &
  CALLBACK_PID=$!

  if ! wait_for_subscription_count "${CALLBACK_TOPIC}" "$((existing + 1))"; then
    kill -TERM "${CALLBACK_PID}" 2>/dev/null || true
    wait "${CALLBACK_PID}" 2>/dev/null || true
    CALLBACK_PID=""
    return 1
  fi
}

finish_callback()
{
  local description=$1
  local pid=${CALLBACK_PID}

  CALLBACK_PID=""
  if [[ -z "${pid}" ]] || ! wait "${pid}"; then
    log "ERROR: timed out waiting for ${description} callback"
    return 1
  fi
}

cancel_callback()
{
  [[ -n "${CALLBACK_PID}" ]] || return 0
  kill -TERM "${CALLBACK_PID}" 2>/dev/null || true
  wait "${CALLBACK_PID}" 2>/dev/null || true
  CALLBACK_PID=""
}

send_command_and_wait()
{
  local callback_id=$1
  local description=$2
  local topic=$3
  local message_type=$4
  local message=$5

  start_callback_listener "${callback_id}" || {
    log "ERROR: could not subscribe to the ${description} callback"
    return 1
  }
  publish_once "${topic}" "${message_type}" "${message}" || {
    cancel_callback
    log "ERROR: failed to publish ${description}"
    return 1
  }
  finish_callback "${description}"
}

report_current_area()
{
  log "Reporting current area A to the lower controller"
  send_command_and_wait "${CURRENT_AREA_CALLBACK_ID}" "current area A" \
    "${STATE_COMMAND_TOPIC}" "${STATE_COMMAND_TYPE}" \
    "{task_id: ${TASK_CURRENT_AREA}, task_status: ${AREA_A}}" || return 1
  publish_once "${CURRENT_AREA_TOPIC}" "${MODE_TYPE}" "{data: 'A'}"
}

enter_tree_mode()
{
  log "Assuming the robot is at the observation position; switching to TREE mode"
  publish_mode TREE && wait_for_mode TREE || {
    log "ERROR: perception stack did not enter TREE mode"
    return 1
  }
}

parse_target_type()
{
  TARGET_TYPE=$(/usr/bin/python3 - "${RUN_DIR}/target_type.yaml" <<'PY'
import ast
from pathlib import Path
import sys

document = Path(sys.argv[1]).read_text(encoding="utf-8").split("\n---", 1)[0]
for line in document.splitlines():
    try:
        values = ast.literal_eval(line.strip())
    except (SyntaxError, ValueError):
        continue
    if isinstance(values, list) and values and isinstance(values[0], str):
        print(values[0])
        raise SystemExit(0)
raise SystemExit(1)
PY
  )
}

parse_selected_target()
{
  read -r TARGET_TYPE TARGET_X TARGET_Y TARGET_Z < <(
    /usr/bin/python3 - "${RUN_DIR}/selected_target.csv" <<'PY'
import csv
import math
from pathlib import Path
import sys

rows = csv.reader(Path(sys.argv[1]).read_text(encoding="utf-8").splitlines())
for row in rows:
    if len(row) != 4 or not row[0]:
        continue
    try:
        values = [float(value) for value in row[1:]]
    except ValueError:
        continue
    if all(math.isfinite(value) for value in values):
        print(row[0], *values)
        raise SystemExit(0)
raise SystemExit(1)
PY
  )
  [[ -n "${TARGET_TYPE}" && -n "${TARGET_X}" && -n "${TARGET_Y}" && -n "${TARGET_Z}" ]]
}

sample_d435i_target()
{
  timeout --foreground "${DETECTION_TIMEOUT_SECONDS}s" \
    /usr/bin/python3 - "${D435I_DETECTIONS_TOPIC}" "${TARGET_SAMPLE_COUNT}" \
    >"${RUN_DIR}/selected_target.csv" 2>"${RUN_DIR}/selected_target.err" <<'PY'
import csv
import sys

import rclpy
from fruit_perception.msg import FruitDetections

topic = sys.argv[1]
sample_count = int(sys.argv[2])
samples = []
latest_type = ""


def on_detections(message):
    global latest_type
    valid = [detection for detection in message.detections if detection.has_point]
    if not valid:
        return
    selected = min(
        valid,
        key=lambda item: (
            item.point.x * item.point.x + item.point.y * item.point.y,
            -float(item.confidence),
        ),
    )
    latest_type = selected.type
    samples.append(
        (
            float(selected.point.x),
            float(selected.point.y),
            float(selected.point.z),
        )
    )


rclpy.init()
node = rclpy.create_node("d435i_arm_target_sampler")
node.create_subscription(FruitDetections, topic, on_detections, 10)
try:
    while rclpy.ok() and len(samples) < sample_count:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(samples) < sample_count:
        raise SystemExit(1)
    row = [
        latest_type,
        sum(point[0] for point in samples) / sample_count,
        sum(point[1] for point in samples) / sample_count,
        sum(point[2] for point in samples) / sample_count,
    ]
    csv.writer(sys.stdout).writerow(row)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

read_d435i_target()
{
  local type_status=0

  log "Waiting for a D435i target"
  timeout --foreground "${DETECTION_TIMEOUT_SECONDS}s" \
    ros2 topic echo --once "${D435I_TYPES_TOPIC}" "${TYPES_TYPE}" \
    --qos-reliability reliable --qos-durability volatile \
    --filter 'len(m) > 0' --field types \
    >"${RUN_DIR}/target_type.yaml" 2>"${RUN_DIR}/target_type.err" || type_status=$?
  if (( type_status != 0 )); then
    if (( type_status == 124 )); then
      log "ERROR: no D435i target class within ${DETECTION_TIMEOUT_SECONDS}s"
    else
      log "ERROR: D435i target subscription failed with exit ${type_status}"
      tail -n 20 "${RUN_DIR}/target_type.err" >&2 || true
    fi
    return 1
  fi
  parse_target_type || {
    log "ERROR: invalid D435i target class output"
    return 1
  }

  sample_d435i_target || {
    log "ERROR: fewer than ${TARGET_SAMPLE_COUNT} valid D435i detection samples within ${DETECTION_TIMEOUT_SECONDS}s"
    tail -n 20 "${RUN_DIR}/selected_target.err" >&2 || true
    return 1
  }
  parse_selected_target || {
    log "ERROR: invalid selected D435i target output"
    return 1
  }
  log "Averaged ${TARGET_SAMPLE_COUNT} centered D435i samples: ${TARGET_TYPE} x=${TARGET_X}, y=${TARGET_Y}, z=${TARGET_Z}"
}

send_arm_target()
{
  log "Sending the D435i target to the arm"
  send_command_and_wait 3 "arm target" \
    "${ARM_TARGET_TOPIC}" "${ARM_TARGET_TYPE}" \
    "{target_x: ${TARGET_X}, target_y: ${TARGET_Y}, target_z: ${TARGET_Z}, z_type: 0}"
}

run_task_sequence()
{
  report_current_area || return 1
  enter_tree_mode || return 1
  read_d435i_target || return 1
  send_arm_target || return 1
}

project_nodes_are_running()
{
  ros2 node list --no-daemon 2>/dev/null | \
    grep -Eq '^/(fruit_perception_container|camera_mode_manager|serial_controller)$'
}

check_prerequisites()
{
  local device=""
  local realsense_devices=""

  [[ -f "${RUNTIME_SETUP}" ]] || { log "ERROR: missing ${RUNTIME_SETUP}"; return 1; }
  [[ -x "${START_SCRIPT}" ]] || { log "ERROR: ${START_SCRIPT} is not executable"; return 1; }
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
  mkdir -p "${RUN_DIR}"
  : >"${LAUNCH_LOG}"

  # shellcheck source=/home/rc4/fruit_ws/setup_fruit_runtime.bash
  source "${RUNTIME_SETUP}"
  set -u
  check_prerequisites || exit 1
  project_nodes_are_running && {
    log "ERROR: perception or serial nodes are already running"
    exit 1
  }

  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  setsid "${START_SCRIPT}" serial_debug_raw_frame:=true > >(tee -a "${LAUNCH_LOG}") 2>&1 &
  LAUNCH_PID=$!

  log "Waiting for the perception stack"
  if ! wait_for_mode OFF; then
    log "ERROR: perception stack did not reach OFF mode"
    show_failure_logs
    keep_running
  fi
  if ! wait_for_subscription_count \
    "${ARM_TARGET_TOPIC}" 1 "${STARTUP_TIMEOUT_SECONDS}"; then
    log "ERROR: serial bridge did not subscribe to ${ARM_TARGET_TOPIC}"
    show_failure_logs
    keep_running
  fi
  if run_task_sequence; then
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
