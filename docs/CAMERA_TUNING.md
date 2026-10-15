# 摄像头实时调参

`test_camera_detector.py` 支持无头模式（默认）与桌面 GUI 窗口：
- **无头模式（`--display false`，默认）**：不创建 OpenCV 窗口与 Tkinter 调参面板，适合后台运行并通过 RobViz 网页端查看标注流（实时发布至 `/fruit_perception/camera/annotated_image`）。
- **桌面 GUI 模式（`--display true` 或 `rgb`/`depth`/`both`）**：
  1. **YOLO 目标检测与三维测距窗口**（1280x720 RGB，对齐深度三维测距）。
  2. **原生深度伪彩色窗口**（848x480 原生深度，自适应距离伪彩色渲染，显示 Near/Far 图例）。
  3. **英文参数调参窗口**（`RealSense Camera Controls`，分组调节 RGB 和 Depth 核心参数）。

参数修改后立即作用于当前摄像头，可直接观察检测框和深度变化。桌面模式下关闭任一窗口或在 OpenCV 窗口按 `q`/`Esc` 安全退出；无头模式下按 `Ctrl-C` 安全退出。

（注：系统架构中深度相机硬件流保持常开，人工曝光调参针对底层 UVC/RGB 曝光与白平衡，以及深度传感器手动曝光与增益。原生深度伪彩色仅用于可视化观察，底层测距计算继续使用毫米级原始深度数据；正式任务运行时若检测到 `/fruit_mission`，调试工具拒绝启动。）

## RobViz 只读预览

无桌面调试使用 `./start_fruit_perception.sh --zone A|B|C`。
脚本必须显式指定区域。运行中使用 `ros2 param set /fruit_perception_node zone B` 切区。
C++ 标注显示原始品种、熟度百分比、检测置信度、中心深度与准星。
成熟框为绿色，未成熟框为橙色，无深度时显示 `D: NoDepth`。
相机启动配置 `initial_reset: true`，重置设备状态，但不能保证修复硬件 MIPI 故障。

日常预览使用 JPEG `CompressedImage` 话题及 WebRTC 视频通道：

- `/fruit_perception/camera/annotated_image/compressed`：感知标注图 JPEG 预览（包含成熟度颜色框、类别置信度、深度与准星）。
- `/fruit_robviz_bridge/camera/color/image_raw/compressed`：原始相机 RGB 预览（无标注覆盖，共用 rgb 传输参数）。
- `/fruit_perception/camera/native_depth/compressed`：Native Depth JET、距离范围和有效像素比例。

RobViz 顶栏“图像采集”页面提供原始 RGB 实时预览与单张无损 PNG 采集：
- 采集保存源必须为 NUC 本机 `/camera/color/image_raw`，保存为原尺寸无损 PNG。
- 固定保存目录为 `/home/rc4/fruit_ws/data/yolo_images/camera`。
- 超过 1 秒未收到新帧拒绝保存，保证数据新鲜度。
- 必须先通过 `./start_fruit_perception.sh --zone A|B|C` 启动感知与相机；网页端不管理相机生命周期。退出采集页面释放采集与视频订阅，不停止底层相机。

JPEG 深度图不能用于数值测距。
Base 由 Python 节点复用现有变换函数，要求图像采集时已满足连续实测停稳窗口。
无效、过期或冲突反馈显示 N/A，不将最新位姿用于旧图像。
预览设置不改变相机曝光、模型或比赛测距。
首次成功配置重载前不订阅 raw image，关闭全部面板后撤销对应 raw 订阅。
详见 `src/fruit_robviz_bridge/README.md`。

## 雷达接入后的 DDS 传输

`scripts/setup_fruit_runtime.bash` 统一设置 Cyclone DDS 的 `AllowMulticast=spdp`。
节点发现保留组播，图像等用户数据使用单播。
这避免接入雷达网线后，DDS 自动选择百兆有线口并通过数据组播阻塞图像发布。
此设置不改变 Livox 驱动自身的 UDP 通信，也不修改相机曝光或深度处理参数。
故障机制、实机数据和验证边界见 [雷达接入后相机深度停流分析](CAMERA_LIDAR_DDS_ANALYSIS.md)。

已运行的 ROS 进程不会自动应用环境变量变化。
停止任务后重新启动相关 ROS 进程，使其加载统一运行环境。
检查此问题时保持雷达上电、网线连接，仅启动感知，不启动比赛任务或发送运动命令。
确认原生深度、对齐深度和 `/fruit_perception/camera/rgbd_heartbeat` 持续更新。

## 启动与运行

所有命令都在工作空间根目录执行，并先加载 ROS 运行环境：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
```

启动无头检测（默认模式，发布 `/fruit_perception/camera/annotated_image` 供 RobViz 网页端查看）：

```bash
# A 区模型（蔬菜）
python3 test_camera_detector.py --model A

# B 区模型（整树果实）
python3 test_camera_detector.py --model B
```

启动桌面 GUI 窗口与调参面板：

```bash
# 打开桌面 GUI（默认 rgb 窗口与调参面板）
python3 test_camera_detector.py --model B --display true

# 指定显示窗口（rgb / depth / both）
python3 test_camera_detector.py --model B --display both
```

`--model` 必须是 `A`、`B`、`C` 或 `D`。脚本从当前 YAML 的 `model_root`
读取物种检测器：A/C 使用 `vegetable_type/best.onnx`，B/D 使用
`fruit_type/best.onnx`，类别名称读取模型自身元数据，不再依赖旧的
`a_model_path/b_model_path/c_model_path` 或按类别数量猜标签。

此 GUI 用于**物种检测、熟度检查与相机调参**，显示模型内部标签（如 `pepper`）及熟度置信度，
不执行深度锁定或采摘。正式任务的两阶段结果请查看感知节点的
`/fruit_perception/camera/annotated_image` 与 `/fruit_perception/decision`；
正式输出 A 区为 `ripe/unripe`，C 区辣椒为 `chili_ripe/chili_unripe`。

检测阈值默认读取
`/home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml`，也可以只在本次运行中通过命令行覆盖：

```bash
python3 test_camera_detector.py --model B --conf 0.60 --iou 0.45
```

`--conf` 是最低置信度阈值。低于这个值的候选框不会进入 GUI；值越高，通常误检越少，但低置信度的真实目标也更容易漏掉。

`--iou` 是 NMS 使用的交并比阈值。NMS 会在重叠框中保留置信度较高的框；两个框的 IoU 超过这个值时，较低分的框会被抑制。值越低，抑制越严格，显示框通常更少；值越高，越容易保留相互重叠的框，但重复框也可能增加。当前实现没有区分类别，重叠较大的不同类别框也可能互相抑制。

两个命令行参数只影响当前进程，不会修改 YAML。省略参数时继续使用 YAML 中的值，当前默认值为 `conf=0.50`、`iou=0.45`。

脚本默认把窗口显示到机器人连接的桌面屏幕。检测窗口按 `q` 或 `Esc`，或者关闭任一窗口（检测画面、原生深度画面或参数窗口），即可退出。
正常退出会停止脚本自己启动的相机、机械臂调试 launch 及其子进程组，绝不终止复用的外部感知或桥接节点。
关闭终端或收到 `SIGTERM` 时也会执行同样的清理。使用 `SIGKILL`、断电或系统崩溃后，独立的
`test_camera_detector_*_container` 可能继续占用摄像头；返回工作空间根目录执行
`./kill_fruit_processes.sh` 可清理该容器及其 `ros2 component standalone` 包装进程。

## 常用参数

参数窗口使用全英文界面，分为 `Depth Controls` 和 `RGB Controls` 两大区域，仅显示当前摄像头实际支持的项目和范围：

### Depth Controls

| 参数 | 作用 |
| --- | --- |
| Depth Exposure (us) | 深度传感器曝光时间（微秒），默认 10589；调节时自动确保深度自动曝光关闭 |
| Depth Gain | 深度传感器图像增益，默认 16 |

（注：深度预设固定为 `High Accuracy` (3)，深度自动曝光固定为 `false`，发射器固定为 Laser (`emitter_enabled: 1`, `laser_power: 150.0`)，时空滤波保持开启 (`spatial_filter.enable: true`, `temporal_filter.enable: true`)，均从 YAML 加载。）

### RGB Controls

| 参数 | 作用 |
| --- | --- |
| Auto exposure | 自动曝光 |
| Exposure | 曝光时间；过大会造成运动拖影 |
| Gain | 图像增益；过大会增加噪点 |
| Auto white balance | 自动白平衡 |
| White balance | 手动调整色温 |
| Brightness | 亮度 |
| Contrast | 对比度 |
| Saturation | 饱和度 |
| Sharpness | 锐度 |
| Gamma | 中间亮度层次 |
| Backlight compensation | 逆光补偿 |
| Power line frequency | 灯光防闪烁频率 |

拖动滑块、切换勾选框或选择频率后，参数会立即写入当前摄像头。手动修改 `Exposure` 或 `White balance` 时，对应的自动模式会自动关闭。

## 推荐调节顺序

1. 把机器人和目标放到比赛时的实际距离，保持检测画面持续可见。
2. 室内灯光稳定时先关闭自动曝光，调低 `Exposure` 到运动时没有明显拖影，再用 `Gain` 补足亮度。
3. 检查原生深度窗口（`848x480`），根据被测目标表面材质和反光特性，适当微调 `Depth Exposure (us)` 与 `Depth Gain`，确保果蔬目标表面点云稳定连续，黑洞（无效深度）最小化。
4. 颜色偏差明显时关闭自动白平衡，再调 `White balance`，使不同类别的颜色接近训练数据。
5. 少量调整亮度、对比度、饱和度和锐度，同时观察漏检、误检和置信度，不要只追求画面鲜艳。
6. 室内交流灯出现横纹或闪烁时调整 `Power line frequency`；中国大陆通常使用 `50 Hz` 对应的设备选项。
7. 在目标静止、机器人运动和背景干扰三种情况下都确认效果，再保存参数。

一次只调整一个主要参数，便于判断检测变化来自哪里。曝光和增益通常比锐度、Gamma 等后处理参数更优先。

## 保存和恢复

确认效果后点击参数窗口底部的 `Save`。成功保存时，窗口状态栏会显示备份文件名，终端也会输出完整路径。

当前成功应用的值（包括 RGB 参数和 Depth Exposure/Gain）写入源工作空间配置：

```text
/home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml
```

每次保存前，原配置会完整备份到：

```text
/home/rc4/fruit_ws/tmp/fruit_perception_YYYYMMDD_HHMMSS_ffffff.yaml.bak
```

需要恢复时，先停止感知程序，找到所需备份并覆盖配置文件，例如：

```bash
cp /home/rc4/fruit_ws/tmp/fruit_perception_YYYYMMDD_HHMMSS_ffffff.yaml.bak \
  /home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml
```

保存只持久化已经成功应用到摄像头的值。未点击 `Save` 时，本次实时调整只对当前运行有效，重新启动后仍使用配置文件中的值。
