# fruit_robviz_bridge

该功能包为 RobViz 提供图像预览桥接、配置重载接口、Web 航点编辑 HTTP 服务与按需航点录制节点。
它不参与比赛决策，不实现第二套导航控制器或机械臂运动学。

## 模块与职责

1. **原尺寸图像网关与桥接节点 (`fruit_robviz_image_bridge`)**：
   运行在 Python/GStreamer，经批准跨进程订阅本机 raw image，提供标注 RGB、原始 RGB (`/camera/color/image_raw`) 与 Native Depth 三路 WebRTC (H.264) 及可选 JPEG 压缩流。保持输入原尺寸传输，不进行降采样。提供 NUC 本地单张无损 PNG 采集服务端协议。原始 RGB 共用 `rgb.*` 传输设置。
2. **配置重载客户端 (`RobvizConfigClient`)**：
   在启动时及接收到 `/fruit_robviz_bridge/reload_image_settings` Trigger 请求时从 HTTP 服务获取最新图像设置，并通过 ROS 参数原子应用到图像网关节点。
3. **Web 航点编辑器服务 (`waypoint_editor_server`)**：
   常驻 HTTP 服务（默认端口 8000），承载 RobViz 静态页面资源、图像设置、航点和机械臂预设读写。
   `/api/arm/pose` 与 `/api/arm/preview` 复用 planner 纯计算，不执行运动。
4. **航点录制节点 (`waypoint_recorder_node`)**：
   按需由 `./waypoints_maker.sh` 启动，负责 TF 采样、停稳校验、航点 MarkerArray 锁存发布与 `/follow_waypoints` Action 调试客户端。

## 端点与传输

### 视频网关 (端口 8787)
提供 WebSocket 信令及基于 GStreamer x264 的 WebRTC H.264 视频流（Constrained Baseline, packetization-mode=1）。
支持动态网络拥塞控制（根据客户端 RTT 与丢包反馈调节码率）。

### NUC 原始图像采集协议
通过网关 WebSocket (`/ws`) 提供轻量采集命令：
- `getCaptureInfo`: 注册采集客户端并返回原始话题、输出目录、已保存 PNG 数量、最近文件名、最新帧接收延迟与尺寸。
- `captureFrame`: 允许同主机或私有局域网 Origin，校验 Origin 与 1 秒接收新鲜度，将最新原尺寸原始 RGB 帧无损编码为 PNG，安全落盘至 `/home/rc4/fruit_ws/data/yolo_images/camera`。
- 局域网页面通过 `localhost` 转发访问网关时也允许采集。无 Origin 或公网 Origin 仍拒绝写入。
- 网关同一时间仅允许一个保存任务执行（全局互斥），保存取最新未丢弃原始帧，不受视频预览限帧影响。
- 独立单机脚本 `collect_cam_img.py` 仍保留原来的 JPG 行为；RobViz 采集专用于无损 PNG 采集。

### JPEG 压缩话题
当所选传输模式为 JPEG 时，节点按需编码并发布原尺寸压缩图像：

```text
/fruit_perception/camera/annotated_image/compressed        sensor_msgs/msg/CompressedImage
/fruit_robviz_bridge/camera/color/image_raw/compressed    sensor_msgs/msg/CompressedImage
/fruit_perception/camera/native_depth/compressed           sensor_msgs/msg/CompressedImage
```

RGB 预览直接转发感知节点的标注图。
Native Depth 预览读取 `/camera/depth/image_rect_raw` (16UC1 毫米语义)，使用 JET 伪彩色并保留原尺寸。
伪彩色深度图仅供观察，不能替代数值测距。

### 服务
- `/fruit_robviz_bridge/reload_image_settings` (`std_srvs/srv/Trigger`)：从 HTTP 服务拉取设置并通过 `set_parameters_atomically` 应用到网关节点，返回确认版本与会话 ID。
- 参数响应采用异步等待，保留每次请求的超时，启动与并发重载不会阻塞响应处理线程。
- 网关在 HTTP 事件循环中更新订阅和编码任务。退出时关闭浏览器连接、释放管线与监听端口。

## 默认参数与模式

| 默认参数 | RGB | Native Depth |
|---|---:|---:|
| 模式 | WebRTC | WebRTC |
| 编码负载预算 | 25.0 Mbps | 25.0 Mbps |
| 帧率上限 | 10.0 FPS | 10.0 FPS |
| WebRTC目标码率 | 4.0 Mbps | 1.5 Mbps |
| 拥塞调节 | 开启 | 开启 |
| JPEG目标质量 | 90 | 90 |
| JPEG最低质量 | 70 | 70 |
| 分辨率 | 原尺寸 (1280×720) | 原尺寸 (848×480) |

- 当模式为 WebRTC 时，不编码和发布 JPEG。
- 当模式为 JPEG 时，不启动 WebRTC 编码。
- 无客户端观看或订阅时，停止 raw image 订阅与编码管线。
- 网关未启动或信令断线时，活动图像页每秒等待网关恢复，不自动回退至 JPEG 或 Raw。
- WebRTC 协商错误停止，重新协商需用户显式操作。采集连接恢复不会自动重发截图命令。
- WebRTC 在 GStreamer 通知协商就绪后生成 offer，避免初始 RTP caps 尚未就绪时生成空视频协商。

## 运行与验证

构建并 source 当前工作区：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_robviz_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

### 启动桥接与配置服务

```bash
ros2 launch fruit_robviz_bridge fruit_robviz_bridge.launch.py
```

感知启动脚本 (`start_fruit_perception.sh`) 与比赛任务脚本 (`start_fruit_mission.sh`)
会自动检测已运行的 `fruit_robviz_image_bridge` 节点并自动复用，不再重复拉起冲突进程。
航点录制脚本 (`waypoints_maker.sh`) 与导航脚本 (`start_navigation.sh --recorder`)
也会自动检测并复用已运行的 `fruit_waypoint_recorder` 节点。

### 离线自检与实时检查

```bash
# 离线纯算法及编码自检（不依赖 ROS 主节点）：
ros2 run fruit_robviz_bridge check_preview --self-check

# 实时 JPEG 接收与解码检查：
ros2 run fruit_robviz_bridge check_preview --duration-sec 60
```

## 机械臂编辑与预览

机械臂插件、只读运行快照、文件版本和离线检查见
[RobViz 机械臂插件](../../docs/ROBVIZ_ARM_PLUGIN.md)。
`ros2 run fruit_robviz_bridge check_arm_preview` 只使用临时文件，不连接硬件。

## 许可说明

本项目复用的视频网关及流处理代码遵循 MIT 许可证（Copyright (c) 2026 Edmounds）。
功能包其他部分遵循 Apache-2.0 许可证。
