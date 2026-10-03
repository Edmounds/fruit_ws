# 感知模式切换命令

项目全流程仅使用单一深度相机 `camera`。感知模式简化为三种：`OFF`、`QR`、`DETECT`。
旧的 `SIDES` 与 `TREE` 模式已完全移除并不再受支持。

每个新终端先加载项目运行环境：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
```

## 果蔬目标识别（正式任务 A / B / C）

在各赛区需要进行果蔬检测与深度测量时切换到 `DETECT` 模式：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'DETECT'}"
```

模型根目录由 `src/fruit_perception/config/fruit_perception.yaml` 的 `model_root`
指定，当前区域 `/fruit_mission/current_area` 决定检测器和候选选择策略：
- Area `A`: `vegetable_type/best.onnx` + 熟度分类，输出 `ripe/unripe`，仅成熟目标可单目标锁定。
- Area `B`: `fruit_type/best.onnx` + 熟度分类，输出 `apple_* / pear_*`，整树窗口最多输出 2 个稳定候选。
- Area `C`: `vegetable_type/best.onnx` + 熟度分类，输出品种与熟度；辣椒使用 `chili_*`，按期望成熟品种锁定。

感知兼容区域 `D` 并复用 B 检测与候选策略，不代表已实现 D 区任务。
默认熟度分类器为 Edge，启动时可选 YOLO；固定 Intel `GPU.0`，详见
[两阶段运行文档](../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md)。

目标确认已使用短滚动窗口，不再固定等待 A/C 35 帧、B 50 帧。
品种/熟度一致的证据足够后提前放行，短暂漏检不会立即清空全部证据；
窗口、最小支持数、支持比例和熟度概率门限在源码 YAML 中配置，参数含义见
[感知说明](../src/fruit_perception/README.md#short-window-target-confirmation)。
修改后需重建并重启感知容器；深度定位和运动安全检查不变。
性能诊断时在以下 launch 命令中添加 `perception_log_level:=info`，查看已有
`camera performance:` 日志。1 秒内确认是优化目标，不是本轮实测结果。

单独检查感知（不启动串口桥或任务）时，可先启动：

```bash
ros2 launch fruit_perception fruit_perception.launch.py start_serial_bridge:=false
# 另一个已加载环境的终端，示例选择 B 区：
ros2 topic pub --once /fruit_mission/current_area std_msgs/msg/String "{data: 'B'}"
```

再发布 `DETECT`。当前区域不是相机模式；正常比赛由 mission 设置，不要在任务运行时手动覆盖。

## 二维码扫描（B / C 区入口）

使用 camera 读取二维码：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'QR'}"
```

## 停止采集与推理

关闭相机流和推理：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'OFF'}"
```

## 查看当前生效模式

模式切换成功后，可查看当前激活模式（带 Transient Local QoS）：

```bash
ros2 topic echo --qos-durability transient_local --once \
  /fruit_perception/active_mode std_msgs/msg/String
```

期望输出为 `OFF`、`QR` 或 `DETECT` 之一。
