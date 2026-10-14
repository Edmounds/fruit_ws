#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
BAG_ROOT="$WORKSPACE/rosbags"
MAX_BAGS=5
BAG_LABEL=""
BAG_PROFILE="${FRUIT_RECORDING_PROFILE:-full}"

BAG_NAME_PATTERN='^([A-Za-z0-9_.-]+_)?([0-9]{8}_[0-9]{6})(_[0-9]+)?$'

usage() {
  cat <<'EOF'
Usage: ./record_rosbag.sh [label] [--no-monitor]

Record the selected fixed topic profile using the MCAP storage format.
The default direct-run profile is full. Debug entrypoints pass their profile
through FRUIT_RECORDING_PROFILE and start this script only once.
Camera topics are added only for profiles that use the camera, when /camera,
/fruit_perception_node, and each selected image publisher are ready.
The selected raw visual topics are /camera/color/image_raw,
/camera/aligned_depth_to_color/image_raw, and
/fruit_perception/camera/annotated_image. Camera info, /camera/rgbd,
unaligned depth, and compressed image transport topics are never recorded.
The bag is written to:
  rosbags/YYYYMMDD_HHMMSS/

The newest five date-named bags are retained. An optional label is kept as a
prefix before the timestamp. --no-monitor is retained for compatibility.

Press Ctrl-C once to stop recording cleanly.
EOF
}

while (($# > 0)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --no-monitor)
      # Retained for compatibility
      shift
      ;;
    --*)
      usage >&2
      exit 2
      ;;
    *)
      [[ -z "$BAG_LABEL" ]] || { usage >&2; exit 2; }
      BAG_LABEL=$1
      shift
      ;;
  esac
done

if [[ -n "$BAG_LABEL" && ! "$BAG_LABEL" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  printf '[rosbag] label must contain only letters, numbers, dot, underscore, or hyphen: %s\n' \
    "$BAG_LABEL" >&2
  exit 2
fi

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
source "$WORKSPACE/scripts/fruit_runtime_recording.bash"
set -u

fruit_logging_init "record_rosbag" "${FRUIT_LOGGING_DEBUG:-0}"

bag_pid=""
stopped=0

cleanup()
{
  trap - EXIT INT TERM
  if ((stopped)); then
    return
  fi
  stopped=1
  local status=$?
  if [[ -n "${bag_pid}" ]]; then
    printf '\n[rosbag] stopping bag recording and writing index...\n'
    fruit_stop_process_group "${bag_pid}" "rosbag" 10 2
  fi
  fruit_logging_close "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

list_recorded_bags()
{
  local name

  while IFS= read -r name; do
    [[ "$name" =~ $BAG_NAME_PATTERN ]] || continue
    printf '%s\t%s\n' "${BASH_REMATCH[2]}" "$name"
  done < <(find "$BAG_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
}

prune_recorded_bags()
{
  local -a bag_records=()
  local oldest_record
  local oldest

  mapfile -t bag_records < <(list_recorded_bags | sort -k1,1 -k2,2)
  while ((${#bag_records[@]} > MAX_BAGS - 1)); do
    oldest_record=${bag_records[0]}
    oldest=${oldest_record#*$'\t'}
    [[ "$oldest" =~ $BAG_NAME_PATTERN ]] || exit 1
    printf '[rosbag] removing oldest bag: %s\n' "$BAG_ROOT/$oldest"
    rm -rf -- "$BAG_ROOT/$oldest"
    bag_records=("${bag_records[@]:1}")
  done
}

topic_has_publishers()
{
  local topic=$1
  local topic_info

  topic_info=$(ros2 topic info "$topic" --verbose 2>/dev/null || true)
  [[ "$topic_info" =~ Publisher[[:space:]]count:[[:space:]]+[1-9][0-9]* ]]
}

node_is_running()
{
  local node=$1
  /usr/bin/grep -Fxq "$node" < <(ros2 node list 2>/dev/null)
}

add_topic()
{
  local topic=$1
  [[ -n "${recorded_topic_set[$topic]+yes}" ]] && return
  recorded_topic_set["$topic"]=1
  recorded_topics+=("$topic")
}

declare -a recorded_topics=()
declare -A recorded_topic_set=()
while IFS= read -r topic; do
  [[ -n "$topic" ]] || continue
  add_topic "$topic"
done < <(fruit_recording_profile_topics "$BAG_PROFILE")

if fruit_recording_profile_uses_camera "$BAG_PROFILE"; then
  if node_is_running /camera && node_is_running /fruit_perception_node; then
    for camera_topic in \
      /camera/color/image_raw \
      /camera/aligned_depth_to_color/image_raw \
      /fruit_perception/camera/annotated_image; do
      if topic_has_publishers "$camera_topic"; then
        add_topic "$camera_topic"
        printf '[rosbag] camera topic included: %s\n' "$camera_topic"
      else
        printf '[rosbag] camera topic skipped: no publisher: %s\n' "$camera_topic"
      fi
    done
  else
    printf '[rosbag] camera topics skipped: /camera or /fruit_perception_node is unavailable\n'
  fi
fi

if [[ "${FRUIT_RECORDING_WORKER:-0}" != 1 ]] && fruit_recording_node_running; then
  printf '[rosbag] recorder already exists; skip duplicate recording\n'
  exit 0
fi

mkdir -p "$BAG_ROOT"
prune_recorded_bags

bag_stamp=$(date +%Y%m%d_%H%M%S)
if [[ -n "$BAG_LABEL" ]]; then
  BAG_NAME="${BAG_LABEL}_${bag_stamp}"
else
  BAG_NAME="$bag_stamp"
fi
BAG_PATH="$BAG_ROOT/$BAG_NAME"
if [[ -e "$BAG_PATH" ]]; then
  BAG_PATH="${BAG_PATH}_$$"
fi

printf '[rosbag] output: %s\n' "$BAG_PATH"
printf '[rosbag] storage: mcap\n'
printf '[rosbag] profile: %s\n' "$BAG_PROFILE"
printf '[rosbag] topics (%s):\n' "${#recorded_topics[@]}"
printf '[rosbag]   %s\n' "${recorded_topics[@]}"
printf '[rosbag] retaining the newest %s date-named bags\n' "$MAX_BAGS"
printf '[rosbag] press Ctrl-C once when the failure has been captured\n'

orchestrator_log="${FRUIT_RUNTIME_SESSION_DIR:-}/orchestrator.log"
if [[ -n "${FRUIT_RUNTIME_SESSION_DIR:-}" && -w "$orchestrator_log" ]]; then
  {
    printf '=== Recording Configuration ===\n'
    printf 'Profile: %s\n' "$BAG_PROFILE"
    printf 'Output: %s\n' "$BAG_PATH"
    printf 'Topics (%s):\n' "${#recorded_topics[@]}"
    printf '  %s\n' "${recorded_topics[@]}"
  } >> "$orchestrator_log"
fi

setsid ros2 bag record \
  --include-hidden-topics \
  --storage mcap \
  "${recorded_topics[@]}" \
  --output "$BAG_PATH" &
bag_pid=$!

if wait "$bag_pid"; then
  status=0
else
  status=$?
fi
cleanup
exit "$status"
