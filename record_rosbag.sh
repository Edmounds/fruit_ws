#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
BAG_ROOT="$WORKSPACE/rosbags"
MAX_BAGS=5
DEBUG_MONITOR_PID=""
START_MONITOR=1
BAG_LABEL=""

RECORD_EXCLUDE_REGEX='(^/camera/camera/depth/image_rect_raw$|^/camera/camera/rgbd$|/(compressed|compressedDepth|theora)$)'

BAG_NAME_PATTERN='^([A-Za-z0-9_.-]+_)?([0-9]{8}_[0-9]{6})(_[0-9]+)?$'

usage() {
  cat <<'EOF'
Usage: ./record_rosbag.sh [label] [--full] [--no-monitor]

Record all discovered ROS topics, including hidden Action topics, LiDAR, and
point-cloud topics. To control camera load and bag size, D435i unaligned raw
depth, the composite RGBD topic, and image transport derivatives are excluded.
The retained image topics are the two USB raw/annotated images and D435i color
raw/aligned-depth/annotated images. --full is accepted as an explicit mode flag.
The bag is written to:
  rosbags/YYYYMMDD_HHMMSS/

The newest five date-named bags are retained. An optional label is kept as a
prefix before the timestamp. --no-monitor is used by start_fruit_mission.sh
to avoid starting a second robot debug monitor.

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
      START_MONITOR=0
      shift
      ;;
    --full)
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

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

cleanup()
{
  if [[ -n "$DEBUG_MONITOR_PID" ]]; then
    kill -INT "$DEBUG_MONITOR_PID" 2>/dev/null || true
    wait "$DEBUG_MONITOR_PID" 2>/dev/null || true
    DEBUG_MONITOR_PID=""
  fi
}

trap cleanup EXIT INT TERM

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
printf '[rosbag] recording all non-excluded topics, including LiDAR and point-cloud topics\n'
printf '[rosbag] visual topics: USB raw/annotated; D435i color raw/aligned depth/annotated\n'
printf '[rosbag] excluded topic regex: %s\n' "$RECORD_EXCLUDE_REGEX"
printf '[rosbag] retaining the newest %s date-named bags\n' "$MAX_BAGS"
printf '[rosbag] press Ctrl-C once when the failure has been captured\n'

if ((START_MONITOR)); then
  "$WORKSPACE/robot_debug_monitor.sh" rosbag &
  DEBUG_MONITOR_PID=$!
fi

ros2 bag record \
  --all \
  --include-hidden-topics \
  --exclude "$RECORD_EXCLUDE_REGEX" \
  --output "$BAG_PATH" \
  --compression-mode file \
  --compression-format zstd
