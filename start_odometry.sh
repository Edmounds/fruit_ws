#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
MAP_PROFILE_HELPER="$WORKSPACE/scripts/map_profile.sh"
ODOMETRY_LAUNCH="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/pointlio_odometry.launch.py"
PARAMS="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml"
LIVOX_CONFIG="$WORKSPACE/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json"
SCRIPT_CLIENT_CONFIG="$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml"
PCL_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
MAP_ID=""
pid=""
stopped=0

usage() {
  cat <<'EOF'
Usage: ./start_odometry.sh --map A|B

Start the Livox MID-360 and Point-LIO odometry stack. The selected map profile identifies
the physical starting point; no PCD map or GICP relocalization is loaded.
The odometry origin is published after the initial Point-LIO pose is stable.
EOF
}

while (($# > 0)); do
  case "$1" in
    --map)
      (($# >= 2)) || { usage >&2; exit 2; }
      MAP_ID=$2
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "$MAP_ID" ]] || { usage >&2; exit 2; }

stop_children() {
  trap - INT TERM EXIT

  if ((stopped)); then
    return
  fi
  stopped=1

  if [[ -n "$pid" ]]; then
    printf '\n[odometry] stopping Livox and Point-LIO...\n'
    kill -INT -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

trap stop_children INT TERM EXIT

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u
source "$MAP_PROFILE_HELPER"
resolve_map_profile "$WORKSPACE" "$MAP_ID"

for required_file in \
  "$ODOMETRY_LAUNCH" \
  "$PARAMS" \
  "$LIVOX_CONFIG" \
  "$SCRIPT_CLIENT_CONFIG" \
  "$PCL_PRELOAD"; do
  if [[ ! -r "$required_file" ]]; then
    printf '[odometry] required file is unavailable: %s\n' "$required_file" >&2
    exit 1
  fi
done

printf '[odometry] starting source launch for profile=%s\n' "$MAP_ID"
printf '[odometry] waiting for a stable Point-LIO origin; no map or GICP is loaded.\n'
setsid ros2 launch "$ODOMETRY_LAUNCH" \
  "params_file:=$PARAMS" \
  "mid360_config:=$LIVOX_CONFIG" &
pid=$!

if ! ros2 run fruit_mission stack_readiness_client \
  nodes --node /laserMapping --node /loam_interface \
  --node /sensor_scan_generation --timeout 60 \
  --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"; then
  printf '[odometry] odometry nodes did not become ready\n' >&2
  exit 1
fi
if ! ros2 run fruit_mission stack_readiness_client \
  odometry --topic /aft_mapped_to_init --timeout 60 \
  --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"; then
  printf '[odometry] Point-LIO did not publish valid odometry\n' >&2
  exit 1
fi
if ! ros2 run fruit_mission stack_readiness_client \
  tf --parent odom --child base_link --timeout 5 --max-age 1 \
  --ros-args --params-file "$SCRIPT_CLIENT_CONFIG"; then
  printf '[odometry] stable odom origin was not established within 5s\n' >&2
  exit 1
fi
printf '[odometry] stable odom origin is ready\n'

if wait "$pid"; then
  status=0
else
  status=$?
fi

stop_children
exit "$status"
