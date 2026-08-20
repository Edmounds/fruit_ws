#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws
DEBUG_PID=""

cleanup()
{
  trap - EXIT INT TERM
  if [[ -n "$DEBUG_PID" ]]; then
    kill -INT "$DEBUG_PID" 2>/dev/null || true
    wait "$DEBUG_PID" 2>/dev/null || true
  fi
}

on_signal()
{
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

source "$WORKSPACE/setup_fruit_runtime.bash"
set -u

"$WORKSPACE/robot_debug_monitor.sh" perception &
DEBUG_PID=$!

ros2 launch fruit_perception fruit_perception.launch.py "$@"
