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

cleanup()
{
  local status=$?
  trap - EXIT INT TERM
  fruit_logging_close "${status}"
}

on_signal()
{
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
set -u

fruit_logging_init "perception" "${DEBUG_MODE}"

START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
if [[ "$START_SERIAL_BRIDGE" == false ]]; then
  printf '[perception] reusing existing serial bridge; it will not be stopped on exit\n'
  set -- "$@" start_serial_bridge:=false
fi

if [[ "${FRUIT_RUNTIME_TOP_ENTRY:-0}" == "1" ]]; then
  fruit_run_logged "perception_launch" ros2 launch fruit_perception fruit_perception.launch.py "$@"
else
  ros2 launch fruit_perception fruit_perception.launch.py "$@"
fi
