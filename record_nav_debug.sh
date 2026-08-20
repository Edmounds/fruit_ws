#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
BAG_ROOT="$WORKSPACE/rosbags"
MAX_BAGS=5
DEBUG_MONITOR_PID=""
START_MONITOR=1
BAG_LABEL=""

# Keep the high-volume point-cloud streams out while retaining images, scans,
# odometry, diagnostics, mission messages, and navigation debug topics.
POINT_CLOUD_EXCLUDE_REGEX='^/(livox/lidar(/.*)?|cloud_registered(_body)?|cloud_effected|Laser_map|registered_scan|sensor_scan|gicp/(localization_map|source_cloud|aligned_cloud)|glim_ros/aligned_points_odom|([^/]+/)*(pointcloud|point_cloud|points))$'
BAG_NAME_PATTERN='^([A-Za-z0-9_.-]+_)?([0-9]{8}_[0-9]{6})(_[0-9]+)?$'

usage() {
  cat <<'EOF'
Usage: ./record_nav_debug.sh [label] [--no-monitor]

Record all ROS topics, including hidden Action topics, except the project's
LiDAR and point-cloud topics. The bag is written to:
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
  printf '[nav-bag] label must contain only letters, numbers, dot, underscore, or hyphen: %s\n' \
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
    printf '[nav-bag] removing oldest bag: %s\n' "$BAG_ROOT/$oldest"
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

printf '[nav-bag] output: %s\n' "$BAG_PATH"
printf '[nav-bag] recording all topics except LiDAR and point-cloud topics\n'
printf '[nav-bag] retaining the newest %s date-named bags\n' "$MAX_BAGS"
printf '[nav-bag] press Ctrl-C once when the failure has been captured\n'

if ((START_MONITOR)); then
  "$WORKSPACE/robot_debug_monitor.sh" nav-bag &
  DEBUG_MONITOR_PID=$!
fi

ros2 bag record \
  --all \
  --include-hidden-topics \
  --exclude "$POINT_CLOUD_EXCLUDE_REGEX" \
  --output "$BAG_PATH" \
  --compression-mode file \
  --compression-format zstd
