# fruit_robviz_bridge

该功能包为 RobViz 提供两路只读 JPEG 预览和图像配置重载接口。
它不参与比赛决策、机械臂控制或导航控制。

## 端点

组件发布：

```text
/fruit_perception/camera/annotated_image/compressed  sensor_msgs/msg/CompressedImage
/fruit_perception/camera/native_depth/compressed     sensor_msgs/msg/CompressedImage
```

RGB 预览使用感知标注图，叠加同一 RGB 时间戳的检测框、类型、置信度、相机深度、相机坐标和有效的 `arm_base_link` 坐标。
Native Depth 预览读取 `/camera/depth/image_rect_raw`，使用最近邻缩放和 JET 伪彩色。
JPEG 深度图只用于观察，不能替代数值深度输入。

Base 坐标只有在新鲜的 ArmTelemetry、ArmStatus、`/odometry`、有效外参和连续停稳条件同时满足时显示。
运动、过期、缺少反馈、坐标系不匹配或非有限值均显示 `N/A`。

配置客户端提供：

```text
/fruit_robviz_bridge/reload_image_settings  std_srvs/srv/Trigger
```

客户端调用同源 HTTP 服务的 `/api/image-settings`，再通过原子参数更新配置。
没有后台轮询。
HTTP 成功不等于 ROS 应用成功，只有参数更新成功才返回成功。

## 配置

默认配置由 RobViz 的 `config/fruit-image-settings.json` 管理。
组件 YAML 只保存 ROS 话题接线、相机外参、planner 限位文件和停稳阈值。
组件启动时从 `arm_presets.yaml` 读取四个原始硬件角的限位，不复制 planner 的标定偏置。
RGB 默认 `640x360`、`1.5 Mbps`、最高 `10 FPS`，Depth 默认 `424x240`、`0.8 Mbps`、最高 `10 FPS`。
带宽预算只计算该路 JPEG `data` 负载，不包含 DDS、WebSocket 或 IP 开销。

自动质量从 70 开始，最低 30。预算不足时先降质量，再丢帧。
固定质量模式不修改质量，只降低发布频率。
两路均使用 latest-wins 和有界令牌桶，预算必须是有限正数，`0` 不表示暂停。

组件必须和感知相机组件位于同一个 `ComposableNodeContainer`，以保留 raw image 的 intra-process 数据路径。
运行前先构建并 source 当前工作区：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 launch fruit_robviz_bridge fruit_robviz_bridge.launch.py
```

`check_preview.py` 只读统计压缩帧的 JPEG 可解码性、负载和 FPS：

```bash
ros2 run fruit_robviz_bridge check_preview --duration-sec 60
```

该检查不会发送运动命令，也不会写入航点文件。
