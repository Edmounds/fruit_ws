#!/usr/bin/env bash

set -eo pipefail

case "${1:-}" in
  --help|-h)
    echo "Usage: ./start_fruit_mission_acb.sh [--debug]"
    echo "A left open-loop -> B QR -> A right closed-loop -> C closed-loop -> summary -> B."
    exit 0
    ;;
  ""|--debug) ;;
  *) echo "Usage: ./start_fruit_mission_acb.sh [--debug]" >&2; exit 2 ;;
esac
(($# <= 1)) || { echo "Only --debug is supported" >&2; exit 2; }

exec bash /home/rc4/fruit_ws/start_fruit_mission.sh --flow acb "$@"
