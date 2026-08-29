#!/usr/bin/env bash

set -eo pipefail

WORKSPACE=/home/rc4/fruit_ws

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
set -u

exec ros2 launch "$WORKSPACE/src/joy_linux/launch/joy_cmd_vel_serial.launch.py"

