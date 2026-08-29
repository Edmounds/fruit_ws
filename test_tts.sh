#!/usr/bin/env bash

set -Eeo pipefail

readonly WORKSPACE_DIR="/home/rc4/fruit_ws"
readonly RUNTIME_SETUP="${WORKSPACE_DIR}/scripts/setup_fruit_runtime.bash"

readonly PIPER="${WORKSPACE_DIR}/.venv/piper-tts/bin/piper"
readonly MODEL="${WORKSPACE_DIR}/.cache/piper-tts/zh_CN-huayan-medium/zh_CN-huayan-medium.onnx"
readonly MODEL_CONFIG="${WORKSPACE_DIR}/.cache/piper-tts/zh_CN-huayan-medium/zh_CN-huayan-medium.onnx.json"
readonly WAV_PATH="${WORKSPACE_DIR}/tmp/fruit_mission/tts_test.wav"

usage() {
  cat <<'EOF'
Usage: ./test_tts.sh [--default] [TEXT...]

Play the generated speech through the configured ALSA device.
  --default  Use the ALSA default playback device.
  ALSA_DEVICE=... can select an explicit ALSA device when --default is absent.
EOF
}

ALSA_DEVICE="${ALSA_DEVICE:-plughw:CARD=Device,DEV=0}"
text_args=()
while (($# > 0)); do
  case "$1" in
    --default)
      ALSA_DEVICE=default
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      text_args+=("$@")
      break
      ;;
    *)
      text_args+=("$1")
      shift
      ;;
  esac
done

readonly TEXT="${text_args[*]:-巴巴博弈}"
readonly ALSA_DEVICE

source "${RUNTIME_SETUP}"
set -u

command -v aplay >/dev/null
[[ -x "${PIPER}" ]] || {
  printf 'Piper executable not found: %s\n' "${PIPER}" >&2
  exit 1
}
[[ -f "${MODEL}" ]] || {
  printf 'Piper model not found: %s\n' "${MODEL}" >&2
  exit 1
}
[[ -f "${MODEL_CONFIG}" ]] || {
  printf 'Piper model config not found: %s\n' "${MODEL_CONFIG}" >&2
  exit 1
}
if ! aplay -L | grep -Fqx "${ALSA_DEVICE}"; then
  printf 'ALSA playback device not found: %s\n' "${ALSA_DEVICE}" >&2
  printf 'Available USB ALSA devices:\n' >&2
  aplay -L | grep -E '^(hw|plughw|sysdefault|front):CARD=' >&2 || true
  exit 1
fi

mkdir -p "$(dirname "${WAV_PATH}")"
printf '%s\n' "${TEXT}" | "${PIPER}" \
  --model "${MODEL}" \
  --config "${MODEL_CONFIG}" \
  --output_file "${WAV_PATH}"

/usr/bin/aplay -D "${ALSA_DEVICE}" "${WAV_PATH}"
printf 'TTS playback completed on %s: %s\n' "${ALSA_DEVICE}" "${TEXT}"
