#!/usr/bin/env bash
set -eo pipefail
WORKSPACE=/home/rc4/fruit_ws
PHASE=all
MISSION_FLOW=full
FLOW_EXPLICIT=0
OPEN_LOOP=A
OPEN_LOOP_EXPLICIT=0
debug_args=()

usage() {
  cat <<'EOF'
Usage: ./start_fruit_mission.sh [--phase all|prepare|localize] [--flow full|ac|acb|a|b|c] [--open-loop A|C|AC|none] [--debug]

prepare: prestart camera, perception, QR, arm, radar and navigation, then one Home.
localize: at the stationary start pose, start Point-LIO and automatically run mission.
all: perform both stages (default). Use only at the stationary start pose.
Localization waits up to 10s, then restarts Point-LIO and loam_interface once.
Other module failures are informational. Existing motion safety checks still apply.
Parameters are fixed when preparing. Localize reuses those parameters.
Explicit --flow must match the prepared session. Without --flow, reuse its flow.
Completed missions keep modules alive until Ctrl-C or Kill (not hardware ESTOP).
Use start_fruit_mission_acb.sh for ACB with fixed per-leg picking modes.
ACB fixes A outbound to open-loop, A return and C to closed-loop.
--debug records the mission rosbag profile and USB diagnostics.
EOF
}

while (($#)); do
  case "$1" in
    --phase|--flow|--open-loop)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --phase) PHASE=$2 ;;
        --flow) MISSION_FLOW=$2; FLOW_EXPLICIT=1 ;;
        --open-loop) OPEN_LOOP=$2; OPEN_LOOP_EXPLICIT=1 ;;
      esac
      shift 2 ;;
    --debug) debug_args=(--debug); shift ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
case "$PHASE" in all|prepare|localize) ;; *) usage >&2; exit 2 ;; esac
case "$MISSION_FLOW" in full|ac|acb|a|b|c) ;; *) usage >&2; exit 2 ;; esac
case "$OPEN_LOOP" in A|C|AC|none) ;; *) usage >&2; exit 2 ;; esac
if [[ "$MISSION_FLOW" == acb ]]; then
  if ((OPEN_LOOP_EXPLICIT)) && [[ "$OPEN_LOOP" != none ]]; then
    echo "ACB uses fixed per-leg picking modes; --open-loop must be none" >&2
    exit 2
  fi
  OPEN_LOOP=none
fi
source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
export DISPLAY=:0
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
if [[ "$PHASE" != localize ]]; then
  source "$WORKSPACE/scripts/fruit_runtime_logging.bash"
  fruit_logging_init fruit_mission "${#debug_args[@]}"
  trap 'fruit_logging_close $?' EXIT
fi
runner_args=(--phase "$PHASE")
if [[ "$PHASE" != localize ]] || ((FLOW_EXPLICIT)); then
  runner_args+=(--flow "$MISSION_FLOW")
fi
runner_args+=(--open-loop "$OPEN_LOOP")
/usr/bin/python3 "$WORKSPACE/src/fruit_mission/fruit_mission/competition_runner.py" \
  "${runner_args[@]}" "${debug_args[@]}"
