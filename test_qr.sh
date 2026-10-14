#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="/home/rc4/fruit_ws"
CAMERA_PID=""
ROBVIZ_PID=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$ROBVIZ_PID" ]] && kill -0 "$ROBVIZ_PID" 2>/dev/null; then
    printf '\n[test_qr] 正在停止 RobViz 视频网关进程 (PID: %s)...\n' "$ROBVIZ_PID"
    kill -INT "$ROBVIZ_PID" 2>/dev/null || true
    wait "$ROBVIZ_PID" 2>/dev/null || true
  fi
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" 2>/dev/null; then
    printf '[test_qr] 正在停止 RGB 相机进程 (PID: %s)...\n' "$CAMERA_PID"
    kill -INT "$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
  fi
  printf '[test_qr] 测试已退出。\n'
}

trap cleanup EXIT INT TERM

source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
set -u

# 检查相机节点是否已有外部发布者
CAMERA_RUNNING=0
if ros2 topic info --no-daemon /camera/color/image_raw 2>/dev/null | \
   grep -q '^Publisher count: [1-9]'; then
  CAMERA_RUNNING=1
fi

if (( CAMERA_RUNNING )); then
  printf '[test_qr] 检测到已有相机节点运行，直接复用当前图像流。\n'
else
  printf '[test_qr] 启动 RealSense RGB 相机 (1280x720 @ 30FPS, 仅开彩色)...\n'
  ros2 run realsense2_camera realsense2_camera_node \
    --ros-args -r __ns:=/ -r __node:=camera \
    -p enable_color:=true \
    -p rgb_camera.color_profile:='1280x720x30' \
    -p rgb_camera.color_format:='RGB8' \
    -p rgb_camera.enable_auto_exposure:=false \
    -p rgb_camera.exposure:=706 \
    -p enable_depth:=false \
    -p enable_infra:=false \
    -p enable_infra1:=false \
    -p enable_infra2:=false \
    -p enable_gyro:=false \
    -p enable_accel:=false \
    -p pointcloud.enable:=false \
    -p enable_sync:=false \
    -p publish_tf:=false \
    > /tmp/test_qr_camera.log 2>&1 &
  CAMERA_PID=$!

  # 等待相机进程初始化并确认运行
  sleep 2.5
  if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    printf '\n[test_qr] 错误: 相机进程启动失败，请检查 USB 连接。详细日志:\n' >&2
    tail -n 15 /tmp/test_qr_camera.log >&2 || true
    exit 1
  fi
fi

# 检查并启动 RobViz 视频网关与预览支持
ROBVIZ_RUNNING=0
if ros2 node list 2>/dev/null | grep -q "^/fruit_robviz_image_bridge$"; then
  ROBVIZ_RUNNING=1
fi

if (( ROBVIZ_RUNNING )); then
  printf '[test_qr] 检测到已有 RobViz 视频网关运行，直接复用。\n'
else
  printf '[test_qr] 启动 RobViz 视频网关与融合图像 (端口 8787, WebRTC)...\n'
  ros2 launch fruit_robviz_bridge fruit_robviz_bridge.launch.py > /dev/null 2>&1 &
  ROBVIZ_PID=$!
fi

printf '[test_qr] RobViz 相机原图: /camera/color/image_raw\n'
printf '[test_qr] RobViz 融合图像: /fruit_robviz_bridge/camera/annotated_image\n'
printf '[test_qr] RobViz WebRTC 端口: http://<机器人IP>:8787\n\n'

# 启动比赛同款检验终端
/usr/bin/python3 -u "$WORKSPACE/scripts/check_qr_terminal.py"
