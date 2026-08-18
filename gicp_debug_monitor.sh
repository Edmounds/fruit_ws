#!/usr/bin/env bash

set -eo pipefail

exec /home/rc4/fruit_ws/robot_debug_monitor.sh "${1:-gicp}" "${2:-2.0}"
