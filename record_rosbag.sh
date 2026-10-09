#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
BAG_ROOT="$WORKSPACE/rosbags"
MAX_BAGS=5
BAG_LABEL=""
DEBUG_MODE=0

RECORD_EXCLUDE_REGEX='(^/camera/(color|depth|extrinsics|rgbd).*|^/camera/aligned_depth_to_color/camera_info$|/(compressed|compressedDepth|theora)$)'

BAG_NAME_PATTERN='^([A-Za-z0-9_.-]+_)?([0-9]{8}_[0-9]{6})(_[0-9]+)?$'

usage() {
  cat <<'EOF'
Usage: ./record_rosbag.sh [label] [--full] [--no-monitor]

Record all discovered ROS topics using the MCAP storage format.
To control camera load and bag size, Camera raw color, unaligned raw depth,
the composite RGBD topic, camera info/metadata, and image transport derivatives
are excluded.
The retained visual topics are Camera annotated image (/fruit_perception/camera/annotated_image),
aligned depth (/camera/aligned_depth_to_color/image_raw), and fruit decisions (/fruit_perception/decision).
--full is accepted as an explicit mode flag.
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
      # Retained for compatibility
      shift
      ;;
    --debug)
      DEBUG_MODE=1
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

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
set -u

fruit_logging_init "record_rosbag" "${DEBUG_MODE}"

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
printf '[rosbag] storage: mcap\n'
printf '[rosbag] recording all non-excluded topics, including LiDAR and point-cloud topics\n'
printf '[rosbag] visual topics: Camera annotated image, aligned depth, and fruit decisions (raw color excluded)\n'
printf '[rosbag] excluded topic regex: %s\n' "$RECORD_EXCLUDE_REGEX"
printf '[rosbag] retaining the newest %s date-named bags\n' "$MAX_BAGS"
printf '[rosbag] press Ctrl-C once when the failure has been captured\n'

ros2 bag record \
  --all \
  --include-hidden-topics \
  --storage mcap \
  --exclude "$RECORD_EXCLUDE_REGEX" \
  --output "$BAG_PATH" &
bag_pid=$!

if wait "$bag_pid"; then
  status=0
else
  status=$?
fi
cleanup
exit "$status"
