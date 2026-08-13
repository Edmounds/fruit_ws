#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/rc4/fruit_ws/install/setup.bash

exec ros2 launch fruit_perception fruit_perception.launch.py
