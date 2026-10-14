#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_AUDIO="$SCRIPT_DIR/src/fruit_mission/sounds/wisadel/start.wav"
if [[ ! -f "$DEFAULT_AUDIO" ]]; then
  DEFAULT_AUDIO="$SCRIPT_DIR/sounds/fruit_success.wav"
fi
AUDIO_FILE="${1:-"$DEFAULT_AUDIO"}"

if [[ ! -f "$AUDIO_FILE" ]]; then
  echo "音频文件不存在: $AUDIO_FILE" >&2
  exit 1
fi

echo "正在播放测试音频: $AUDIO_FILE"
if command -v paplay >/dev/null 2>&1; then
  paplay "$AUDIO_FILE"
elif command -v aplay >/dev/null 2>&1; then
  aplay -q "$AUDIO_FILE"
else
  echo "未找到音频播放工具 (paplay / aplay)" >&2
  exit 1
fi
