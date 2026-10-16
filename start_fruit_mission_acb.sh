#!/usr/bin/env bash
set -eo pipefail
args=()

usage() {
  echo "Usage: ./start_fruit_mission_acb.sh [--phase all|prepare|localize] [--debug]"
  echo "A left closed-loop -> B QR -> A right closed-loop -> C closed-loop -> summary -> B."
}

while (($#)); do
  case "$1" in
    --phase)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$2" in all|prepare|localize) ;; *) usage >&2; exit 2 ;; esac
      args+=(--phase "$2"); shift 2 ;;
    --debug) args+=(--debug); shift ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

exec bash /home/rc4/fruit_ws/start_fruit_mission.sh --flow acb --open-loop none "${args[@]}"
