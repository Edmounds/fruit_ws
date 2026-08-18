#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_SETUP="${WORKSPACE_DIR}/setup_fruit_runtime.bash"
CONSOLE="${WORKSPACE_DIR}/lower_controller_console.py"

usage() {
  cat <<'EOF'
Usage: ./zone_witch.sh --A|--B|--C|--D

Send the current-area command directly to the lower controller. This does not
start auto_serial_bridge.
EOF
}

if (($# != 1)); then
  usage >&2
  exit 2
fi

case "$1" in
  --A|--a)
    AREA=A
    ;;
  --B|--b)
    AREA=B
    ;;
  --C|--c)
    AREA=C
    ;;
  --D|--d)
    AREA=D
    ;;
  --help|-h)
    usage
    exit 0
    ;;
  *)
    printf 'Unknown area option: %s\n' "$1" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ ! -r "$RUNTIME_SETUP" || ! -r "$CONSOLE" ]]; then
  printf 'Required project file is unavailable.\n' >&2
  exit 1
fi

# The runtime setup must be sourced before nounset is enabled.
source "$RUNTIME_SETUP"
set -u

# Keep the console alive briefly after enqueueing the reliable command so its
# reader thread can process the lower controller ACK before closing the port.
{
  printf 'area %s\n' "$AREA"
  sleep 1
  printf 'quit\n'
} | /usr/bin/python3 "$CONSOLE" --no-color
