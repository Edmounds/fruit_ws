#!/usr/bin/env bash

set -eo pipefail

source /home/rc4/fruit_ws/setup_fruit_runtime.bash

exec ros2 launch fruit_perception fruit_perception.launch.py
