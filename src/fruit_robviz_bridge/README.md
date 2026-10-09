# fruit_robviz_bridge

该功能包为 RobViz 提供图像预览桥接、配置重载接口、Web 航点编辑 HTTP 服务与按需航点录制节点。
它不参与比赛决策，不实现第二套导航控制器或机械臂运动学。

## 模块与职责

1. **同容器图像预览组件 (`ImageBridgeComponent`)**：
   与感知相机处于同一 `ComposableNodeContainer`，提供 RGB 与 Native Depth 两路压缩预览。
2. **配置重载客户端 (`RobvizConfigClient`)**：
   按需从 HTTP 服务获取图像设置并通过 ROS 参数原子应用。
3. **Web 航点编辑器服务 (`waypoint_editor_server`)**：
   常驻 HTTP 服务（默认端口 8000），承载 RobViz 静态页面资源及航点 YAML 读写/版本控制 API。
4. **航点录制节点 (`waypoint_recorder_node`)**：
   按需由 `./waypoints_maker.sh` 启动，负责 TF 采样、停稳校验、航点 MarkerArray 锁存发布与 `/follow_waypoints` Action 调试客户端。

## 端点

组件发布：

```text
/fruit_perception/camera/annotated_image/compressed  sensor_msgs/msg/CompressedImage
/fruit_perception/camera/native_depth/compressed     sensor_msgs/msg/CompressedImage
```

RGB 预览直接转发感知节点的标注图（包含成熟度彩色框、品种类型、置信度、深度与十字准星），不重复叠加额外文字或边框。
Native Depth 预览读取 `/camera/depth/image_rect_raw`，使用最近邻缩放和 JET 伪彩色。
JPEG 深度图只用于观察，不能替代数值深度输入。

Python 配置节点复用 `fruit_mission.camera_target_transform`，生成同一采集时间戳的 Base 元数据。
Base 坐标要求采集时的 ArmTelemetry、ArmStatus 和 `/odometry` 已覆盖连续实测停稳窗口。
图像处理时的新位姿不能替代图像采集时的位姿。
无效或冲突的新反馈立即使 Base 无效。
机械臂停稳速度阈值只作用于原始硬件速度，模型速度另做有限值检查。
运动、过期、缺少反馈、坐标系不匹配或非有限值均显示 `N/A`。

配置客户端在 ROS 图像桥启动时主动向同源 HTTP 服务的 `/api/image-settings` 发起单次请求，并通过原子参数更新应用到图像桥组件中。
运行期间无后台轮询，网页端保存操作仅持久保存为默认配置，将在下次 ROS 图像桥启动时获取生效。
HTTP 服务不可用或返回非法配置时，输出显式告警并自动回退使用内置默认值（两路带宽均为 10.0 Mbps）兜底应用，不阻塞其他节点。
首次参数成功应用前没有 raw-image 订阅，也不推流。
已移除旧的 `/fruit_robviz_bridge/reload_image_settings` Trigger 服务。

## 配置

默认配置由 RobViz 的 `config/fruit-image-settings.json` 管理。
组件 YAML 保存 ROS 接线和 Python 节点的只读配置路径。
Python 节点用标准 YAML 读取外参、planner 限位与现有停稳参数。
C++ 组件不读取标定 YAML，不自行转换坐标，不复制 planner 的标定偏置。
RGB 默认 `640x360`、`10.0 Mbps`、最高 `10 FPS`，Depth 默认 `424x240`、`10.0 Mbps`、最高 `10 FPS`。
带宽预算只计算该路 JPEG `data` 负载，不包含 DDS、WebSocket 或 IP 开销。

自动质量从 70 开始，最低 30。预算不足时先降质量，再丢帧。
固定质量模式不修改质量，只降低发布频率。
两路均使用 latest-wins 和有界令牌桶，预算必须是有限正数，`0` 不表示暂停。
首次参数成功应用前没有 raw-image 订阅，也不推流。
全部压缩订阅者退出后取消对应 raw-image 订阅。
FPS 准入先于复制、缩放、伪彩色和 JPEG 编码。

组件必须和感知相机组件位于同一个 `ComposableNodeContainer`，以保留 raw image 的 intra-process 数据路径。
运行前先构建并 source 当前工作区：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 launch fruit_robviz_bridge fruit_robviz_bridge.launch.py
```

### 航点相关入口

```bash
# Web 航点编辑服务（由 systemd 或独立入口常驻启动）：
ros2 launch fruit_robviz_bridge waypoint_editor.launch.py

# 现场航点录制节点（由 ./waypoints_maker.sh 按需拉起）：
ros2 launch fruit_robviz_bridge waypoint_recorder.launch.py
```

`check_preview.py` 只读统计压缩帧的 JPEG 可解码性、负载和 FPS：
安装后的 ROS 可执行名称为 `check_preview`。

```bash
ros2 run fruit_robviz_bridge check_preview --duration-sec 60
```

该检查不会发送运动命令，也不会写入航点文件。
该检查使用 SensorDataQoS，与 best-effort 压缩输出匹配。
离线合成输入检查不能替代真实相机、浏览器及同容器推理吞吐验收。
