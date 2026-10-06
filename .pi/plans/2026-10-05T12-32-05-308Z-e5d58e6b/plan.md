## 目标

让项目的 RealSense D435i 使用与用户 `realsense-viewer` 截图对应的 High Accuracy 深度预设。

用户已逐轮确认需求，最后选择“确认，写计划”。本阶段仅做只读调查和规划。执行必须等待扩展的人类审批。

目标设置：

| 项目 | 设置 |
| --- | --- |
| 原生深度 | `848x480x30`，`Z16` |
| 深度预设 | `depth_module.visual_preset: 3`，High Accuracy |
| 深度自动曝光 | `depth_module.enable_auto_exposure: false` |
| 深度曝光默认值 | `depth_module.exposure: 10589`，微秒 |
| 深度增益默认值 | `depth_module.gain: 16` |
| 发射器 | `depth_module.emitter_enabled: 1`，Laser |
| 激光功率 | `depth_module.laser_power` 为 150，数值类型以设备描述符为准 |
| RGB | 保持 `1280x720x30`、`RGB8` 和全部现有 RGB 参数 |

配置由 `/home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml` 管理。

`test_camera_detector.py` 默认打开 RGB 检测窗口、原生深度伪彩色窗口、参数 GUI。所有新增 GUI 标题、标签、图例、状态和错误提示使用英文。

## 范围

预计只修改以下五个文件：

1. `src/fruit_perception/config/fruit_perception.yaml`。
2. `test_camera_detector.py`。
3. `docs/CAMERA_TUNING.md`。
4. `src/fruit_perception/README.md`。
5. `docs/PROJECT_ARCHITECTURE.md`，仅更新相机配置相关说明。

已读取的事实依据：

- `image.png` 显示 High Accuracy、848×480、30 FPS、Z16、曝光 10589、Gain 16、Laser 150。
- 当前 YAML 的 RGB 已是 `1280x720x30`。深度目前是 `1280x720x30`，尚未显式配置深度预设、曝光、Gain 和激光值。
- `src/fruit_perception/launch/fruit_perception.launch.py::generate_launch_description()` 直接读取源 YAML，并把完整 `camera.ros__parameters` 传给相机组件。
- 正式相机和感知组件处于同一容器，均启用进程内通信。本任务不改变它们。
- `/usr/include/librealsense2/h/rs_option.h::rs2_rs400_visual_preset` 明确 High Accuracy 的枚举值为 3。
- 同一 SDK 头文件说明 `emitter_enabled=1` 表示 Laser。
- 驱动 `sensor_params.cpp::SensorParams::registerDynamicOptions()` 自动暴露可写传感器选项，使用整数参数暴露枚举。
- `SensorParams::set_parameter()` 将参数值传给 `sensor.set_option()`，并提供设备范围和枚举说明。
- 驱动按 SDK 选项顺序注册参数，不按 YAML 文本顺序执行。本任务不得把 YAML 排序当成预设应用顺序保障。
- 驱动 README 和 `profile_manager.cpp::VideoProfilesManager::registerVideoSensorParams()` 说明 profile/format 更新需要重新启用视频流。
- `test_camera_detector.py` 已完整读取。`select_camera()` 从同一个源 YAML 复制相机配置。
- `FrameSubscriber` 已订阅 RGB 和对齐深度，后者用于现有像素测距和坐标检查。
- `run()` 会复用已有 RGB 发布者，或者通过 `start_camera()` 启动相机组件。
- 当前 GUI 只在 `args.controls` 为真时打开，控件只包含 RGB 参数。
- `CameraControlService` 已提供发现、描述、设置参数的实现，单次调用超时 5 秒。
- `save_camera_controls()` 已先备份原 YAML，再仅更新 `camera.ros__parameters`。
- 当前 GUI 关闭状态尚未用于结束主循环，主循环也尚未检查 OpenCV 窗口关闭。
- `src/fruit_mission/launch/fruit_mission.launch.py` 的正式任务节点名为 `/fruit_mission`。

非目标：

- 不修改检测、熟度分类、目标确认、坐标转换、机械臂控制或任务策略。
- 不增加深度实时预设切换、自动曝光开关或激光调节控件。
- 不复制 Viewer 的颜色算法，不承诺两个 GUI 的颜色完全相同。
- 不启用驱动 `colorizer`，不改变 ROS 测距图像的编码和单位。
- 不添加滤波、补洞、HDR、Advanced Mode JSON 或额外相机。
- 不修改厂商子模块、消息接口、依赖、CMake 或正式 launch。
- 不清理其他任务的代码、文档和格式。
- 不创建新的单元测试文件，不编写源码字符串断言或深层 ROS mock。

## 决策

1. **采用现成预设接口。** 使用 `depth_module.visual_preset: 3`。不实现另一套 High Accuracy 算法。
2. **复现用户选定的截图参数。** 曝光 10589、Gain 16、Laser enabled、Laser power 150。
3. **保留 RGB 参数。** 包括当前手动曝光 639、Gain 0、白平衡等现有设置。用户通过 GUI 主动修改和 Save 不受此初始值边界限制。
4. **保留同步与对齐。** `enable_sync`、`align_depth.enable`、`enable_rgbd` 继续为真。
5. **原生深度单独显示。** 新窗口订阅 `/camera/depth/image_rect_raw`。对齐深度继续用于 RGB 像素测距，不能用原生深度索引 RGB 检测框。
6. **只在 GUI 副本上着色。** 每帧用非零深度范围归一化，显示 Near/Far 图例。零深度黑色。全零帧显示 `No valid depth`。同值帧不得除零。原始毫米数据不变。
7. **默认自动打开参数 GUI。** 保留 `--controls` 兼容性，但不再要求用户传入它。
8. **深度控件仅两个。** `Depth Exposure (us)` 和 `Depth Gain`。默认和手动调节均保持深度自动曝光关闭。
9. **RGB 控件复用现有实现。** 使用英文 `RGB`、`Depth` 分组。参数范围、步长和类型来自设备描述符，不猜测上限。
10. **Save 显式持久化。** 保存两组成功应用的参数到源 YAML。先备份到 `tmp/fruit_perception_<timestamp>.yaml.bak`。未点击 Save 的修改只影响当前运行。
11. **复用时自动应用配置。** 不创建第二个相机。启动工具时将 YAML 中相关视频设置和 RGB/深度控制应用到已有 `/camera`。
12. **自动应用有任务边界。** 检测到 `/fruit_mission` 时，拒绝启动调试工具，并给出英文提示。此检查必须先于相机写参数和新增子进程。
13. **按确定顺序设置深度。** 先应用 High Accuracy，再显式关闭自动曝光，随后设置曝光、Gain、发射器和激光功率。新启动与复用路径共用最小应用逻辑，避免依赖 YAML 顺序。
14. **只重启必要视频流。** profile/format 不同时，通过参数服务停启对应视频流。不得用固定长 sleep 代替帧到达确认。分辨率相同时不无故停启。
15. **不伪装成功。** 逐项检查设置结果和参数回读。确认实际帧尺寸。缺失参数、拒绝、超时、类型错误或尺寸不符时明确报错，不进入正常调参状态。
16. **批准调试跨进程订阅。** 用户批准仅人工 GUI 调试使用 Python ROS 图像订阅。新增深度端点采用 BEST_EFFORT、VOLATILE 和小队列。正式感知进程内通信不变。
17. **关闭任一窗口结束工具。** RGB、Depth、参数 GUI 中任一关闭，以及 q/Esc/Ctrl-C，均进入统一清理。
18. **保留进程所有权。** 只停止工具自己启动的相机、planner 或 bridge。绝不停止复用进程。沿用现有机械臂通信逻辑，不增加运动请求。
19. **调查限制公开。** 当前工具没有联网或命令执行入口。未运行 AnySearch，未完成实时网上查询。预设结论来自本机 SDK 和驱动源码。

## 执行步骤

以下命令只作为执行阶段指引，规划阶段没有运行。

### 1. 核对基线与执行环境

- 文件：本计划目录 `baseline.md`、`progress.md`，以及预计修改的五个文件。
- 改动：此步不改业务文件。记录最新 Git 状态、HEAD、相关文件差异和可用环境。
- 依赖：必须先获得扩展的人类执行审批。
- 核查命令：

```bash
cd /home/rc4/fruit_ws
git status --short
git rev-parse HEAD
git diff -- src/fruit_perception/config/fruit_perception.yaml test_camera_detector.py docs/CAMERA_TUNING.md src/fruit_perception/README.md docs/PROJECT_ARCHITECTURE.md
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 node list
```

- 期望：运行环境校验成功，确认相机/任务运行情况，标明用户既有改动。
- 停止条件：环境校验失败，出现重叠的并发写入，或不能分离本任务改动。
- Git 边界：规划阶段发现仓库大量脏改动，五个目标文件也包含既有改动。不得整文件暂存既有改动。小范围且无并发可留当前工作区，否则使用隔离工作区并保留现状。不得自动 stash、覆盖 main 或提交其他人的改动。

### 2. 配置 High Accuracy 与深度采集

- 文件/符号：`src/fruit_perception/config/fruit_perception.yaml::camera.ros__parameters`。
- 改动：把深度 profile 改为 `848x480x30`，添加上述深度预设、自动曝光、曝光、Gain、发射器和激光值。保留 RGB、同步、对齐、RGBD 和所有感知参数。
- 依赖：步骤 1 完成。
- 先通过实际设备参数描述核对新增选项的类型和范围。若没有运行相机，可在无任务、无 Viewer 占用的人工验收会话使用现有相机启动入口。
- 核查命令，针对已有 `/camera` 执行：

```bash
ros2 param describe /camera depth_module.visual_preset
ros2 param describe /camera depth_module.exposure
ros2 param describe /camera depth_module.gain
ros2 param describe /camera depth_module.emitter_enabled
ros2 param describe /camera depth_module.laser_power
ros2 param describe /camera depth_module.depth_profile
/usr/bin/python3 -c "from pathlib import Path; import yaml; p=Path('src/fruit_perception/config/fruit_perception.yaml'); d=yaml.safe_load(p.read_text()); print(yaml.safe_dump(d['camera']['ros__parameters'], sort_keys=False))"
git diff --check -- src/fruit_perception/config/fruit_perception.yaml
```

- 期望：描述符包含 `High Accuracy : 3` 和请求 profile。曝光 10589、Gain 16、Laser 150 位于支持范围。YAML 解析成功，差异只涉及本任务深度设置。
- 停止条件：设备不支持预设/profile，数值超出范围，或类型不明。先查设备事实，不能自行替换用户数值或回退其他预设。
- 本步是配置修改，预设是否覆盖其他选项需要步骤 3、6 验证，不能在此宣称已实机生效。

### 3. 统一相机配置应用与安全复用

- 文件/已有符号：`test_camera_detector.py::run`、`select_camera`、`start_camera`、`CameraControlService.set`、`CameraControlService._call`。
- 改动：在 `run()` 的相机写入和 `ensure_arm_communication()` 前检查 `/fruit_mission`。保留相机复用与进程所有权。
- 改动：扩展现有参数服务以读写本任务需要的字符串视频参数和数值控制。保留每次请求 5 秒超时与结果检查。
- 改动：新增一个最小配置应用函数，供新启动和复用两条路径调用，不增加第二个服务框架。
- 应用流程：确认唯一 `/camera` 与源配置对应；检查参数类型和范围；比较 profile/format；必要时停对应视频流并更新 profile/format；应用同步/对齐/RGBD 设置；按设备自动/手动模式语义应用现有 RGB 参数；先应用深度预设再写手动深度控制；重新启用所需流；等待新尺寸帧并核对回读。
- 不把 `serial_no` 等启动专用参数写入正在运行的节点。已有相机身份不匹配时退出，不静默更换设备。
- 若设置流过程中失败，尽力恢复该操作暂停的流，报告失败。不得继续标记 Ready，不得保存未成功设置的值。
- 依赖：步骤 2 的设备类型/范围确认。应先于 GUI 参数发现执行，确保控件初值是最终应用值。
- 核查命令：

```bash
PYTHONPYCACHEPREFIX=/home/rc4/fruit_ws/tmp/depth_gui_pycache /usr/bin/python3 -m py_compile test_camera_detector.py
/usr/bin/python3 test_camera_detector.py --help
```

- 期望：语法通过，旧命令行仍兼容。硬件路径通过步骤 6 的新启动/复用验收验证。
- 停止条件：配置应用需要修改厂商子模块，或者无法通过现有 API 满足正式节点的配置要求。记录原因，提交计划变更审批，不悄悄扩大范围。

### 4. 默认打开英文参数 GUI，扩展深度曝光/Gain

- 文件/符号：`test_camera_detector.py::parse_args`、`CAMERA_CONTROL_LABELS`、`AUTO_CONTROL_FOR`、`CameraControlPanel`、`CameraControlService.discover`、`save_camera_controls`、`run`。
- 改动：默认打开参数 GUI，`--controls` 继续接受。
- 改动：保留现有 RGB 控件，添加英文 Depth Exposure/Gain 分组。深度自动曝光不作为控件展示。
- 改动：调深度 Exposure/Gain 前确认并关闭深度自动曝光。前置设置失败则不提交该手动值。
- 改动：复用描述符发现和类型转换。缺少两个必需深度控件时给出英文错误，不静默缩减功能。
- 改动：Save 合并两组成功设置的值，保留其他 YAML 节点、相机设置与感知配置。先备份，保存失败明确提示。必要时用同目录临时文件和原子替换，避免写失败截断原配置。
- 依赖：步骤 3 完成，GUI 发现必须发生在配置应用成功之后。
- 核查命令：

```bash
PYTHONPYCACHEPREFIX=/home/rc4/fruit_ws/tmp/depth_gui_pycache /usr/bin/python3 -m py_compile test_camera_detector.py
```

- 人工验证命令：`/usr/bin/python3 test_camera_detector.py --model A`，须按步骤 6 的安全条件启动。
- 期望：无需 `--controls` 自动出现 GUI。只有 Exposure/Gain 两个深度控件。RGB 控件与原有功能保留。成功应用后才更新可保存值。
- 停止条件：GUI 中文乱码、控件越界、参数失败仍显示成功、Save 丢失其他配置，或原配置在写失败后损坏。

### 5. 原生深度窗口与统一退出

- 文件/已有符号：`test_camera_detector.py::FrameSubscriber.__init__`、`FrameSubscriber.take_latest`、`run`、`CameraControlPanel.is_open`、`stop_process`。
- 新符号：`colorize_depth_image(depth)`，仅负责 uint16 二维毫米深度的显示副本。
- 改动：订阅 `/camera/depth/image_rect_raw`，与现有对齐深度分开保存。输入经 `cv_bridge` 转换，并校验二维 uint16 数据。
- 改动：新增英文原生深度窗口，按实际源帧尺寸显示。窗口标注 Near/Far 距离范围和实际尺寸。
- 改动：非零深度自适应伪彩色，零深度黑色，全零和同值深度无除零错误。不要启用驱动 colorizer。
- 改动：深度显示不依赖检测结果。尚未收到帧时显示英文等待状态，不拿旧配置图冒充新图。
- 改动：保留原有对齐深度与 RGB 尺寸校验及现有三维测距。原生深度不得传入 RGB 坐标计算。
- 改动：主循环检查两个 OpenCV 窗口与参数 GUI。关闭任一窗口或按 q/Esc/Ctrl-C，进入既有 finally 清理路径。
- 依赖：步骤 3 的配置应用完成。与步骤 4 共用主循环和清理。
- 语法核查命令：

```bash
PYTHONPYCACHEPREFIX=/home/rc4/fruit_ws/tmp/depth_gui_pycache /usr/bin/python3 -m py_compile test_camera_detector.py
```

- 最小离线运行检查，不启动 ROS 节点、相机或窗口：

```bash
/usr/bin/python3 - <<'PY'
import runpy
import numpy as np
ns = runpy.run_path('test_camera_detector.py', run_name='depth_preview_check')
colorize = ns['colorize_depth_image']
for depth in (
    np.zeros((3, 4), dtype=np.uint16),
    np.full((3, 4), 1000, dtype=np.uint16),
    np.array([[0, 500, 1000], [2000, 0, 750]], dtype=np.uint16),
):
    original = depth.copy()
    preview = colorize(depth)
    assert preview.shape == depth.shape + (3,)
    assert preview.dtype == np.uint8
    assert np.all(preview[depth == 0] == 0)
    assert np.array_equal(depth, original)
print('Depth preview checks passed')
PY
```

- 期望：全零、同值、混合深度通过，零值黑色，输入没有修改。英文图例在主循环添加，不影响上述纯数学检查。
- 停止条件：除零、输入被改写、原生深度混入 RGB 测距、窗口关闭卡住，或退出停止复用进程。

### 6. 同步文档并完成人工实机验收

- 文件/章节：`docs/CAMERA_TUNING.md` 的启动、参数、保存和恢复；`src/fruit_perception/README.md` 的相机同步说明；`docs/PROJECT_ARCHITECTURE.md` 的单 camera 方案。
- 改动：记录原生深度 848×480、RGB/对齐深度 1280×720、High Accuracy=3、YAML 曝光/Gain，以及 GUI 默认打开、英文界面、Save 和复用边界。
- 改动：说明伪彩色只用于显示，Viewer 颜色不等同于硬件预设。保持其他架构与比赛策略文档原样。
- 依赖：步骤 2 至 5 完成。文档必须与最终实现一致。
- 核查命令：

```bash
git diff --check -- src/fruit_perception/config/fruit_perception.yaml test_camera_detector.py docs/CAMERA_TUNING.md src/fruit_perception/README.md docs/PROJECT_ARCHITECTURE.md
```

- 人工验收前置：机器人保持静止，无 mission，无运动调试客户端。关闭占用同一设备的 `realsense-viewer`。不得自动执行机械臂运动服务。
- 新启动 GUI：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 test_camera_detector.py --model A
```

- 另一个同样加载环境的终端核对：

```bash
ros2 param describe /camera depth_module.visual_preset
ros2 param get /camera depth_module.visual_preset
ros2 param get /camera depth_module.depth_profile
ros2 param get /camera rgb_camera.color_profile
ros2 param get /camera depth_module.enable_auto_exposure
ros2 param get /camera depth_module.exposure
ros2 param get /camera depth_module.gain
ros2 param get /camera depth_module.emitter_enabled
ros2 param get /camera depth_module.laser_power
ros2 topic echo --once /camera/depth/camera_info
ros2 topic echo --once /camera/color/camera_info
ros2 topic echo --once /camera/aligned_depth_to_color/camera_info
ros2 topic echo --once /camera/depth/metadata
ros2 node list
```

- 期望：预设 3，原生深度 848×480，RGB 与对齐深度 1280×720，曝光/Gain 与 YAML 相同。曝光元数据在设备支持时验证实际曝光，不能只拿 ROS 参数缓存当硬件生效证据。
- 调参验收：改变 Depth Exposure/Gain，观察深度效果和参数回读。改变 RGB 常规参数，确认不改变深度控制。点击 Save，检查备份和 YAML。重启后核对保存值。记录实际用于验收的值，并按用户期望保留或恢复，不擅自留下验收临时值。
- 复用验收：先退出 GUI，再单独启动正式感知，不启动串口或 mission：

```bash
ros2 launch fruit_perception fruit_perception.launch.py start_serial_bridge:=false
```

- 在第二终端运行 `/usr/bin/python3 test_camera_detector.py --model A`。验证仅一个 `/camera`，自动应用 YAML，原生深度显示正常，退出后原感知进程继续运行。
- 如已有相机 profile 不同，在上述无任务人工会话验证停启视频流确实生效。不得将未知 profile 回退视为成功。
- 正式感知验收：在未启动 GUI 前就核对上述参数与 metadata，确认正式 launch 自身加载了正确预设和手动曝光。观察标注图和测距仍工作，避免 GUI 的重新应用掩盖正式 launch 初始化问题。
- 若正式 launch 的预设初始化覆盖了手动参数，停止验收并记录证据，提交范围变更审批。不得只修 GUI 而宣称整个项目已满足要求。
- Viewer 对照：在同一静止目标、光照、距离下，分别运行 Viewer 和项目。Viewer 使用同样预设、曝光/Gain/激光值。比较有效深度与目标区域测距，不要求颜色逐像素一致。不同时占用相机。
- 关闭验收：分别测试 RGB 窗口、Depth 窗口、参数 GUI 的关闭，再测 q/Esc/Ctrl-C。检查自有进程已结束，复用进程仍在。
- 任务边界验收：仅在已有安全任务会话中验证拒绝行为，不为此自动启动 mission。没有此条件则记录“未实测”，通过实现审查核对检查发生在相机参数写入之前。
- 日志：GUI 相机日志沿用 `tmp/test_camera_detector_camera.log`。正式入口日志按 `log/runtime/latest` 的会话指引查阅。记录预设/曝光应用错误、帧尺寸和退出结果。
- 停止条件：实际预设/profile/曝光不匹配，GUI 覆盖运行 mission，测距数据变成 RGB 伪彩图，出现重复相机，或者缺少硬件无法证明效果。缺少硬件时标明“待实机验收”，不得声称通过。

### 7. 复核任务差异与交接

- 文件：上述五个目标文件和本任务计划/review 文档。
- 改动：不增加业务功能。审查本任务差异，记录已完成与未完成验收。
- 依赖：步骤 6 的静态和离线检查通过。硬件限制必须明确记录。
- 核查命令：

```bash
git diff --check -- src/fruit_perception/config/fruit_perception.yaml test_camera_detector.py docs/CAMERA_TUNING.md src/fruit_perception/README.md docs/PROJECT_ARCHITECTURE.md
git status --short
```

- 期望：不包含额外文件、旧用户改动的回退、中文 GUI 字符串或未授权依赖。
- 仅暂存明确属于本任务的 hunks，验证通过后按小阶段提交。禁止 `git add .`、`git add -A`、整文件吞并既有改动和绕过 hooks。
- 如果采用隔离分支，main 脏或有并发作者时不集成。保留分支和工作区，交接路径、tip SHA、测试基线 SHA、验证结果及 Pending Integration 原因。
- 本次预期不改 C++ 或已安装脚本，因此不需要全工作区构建。源 YAML 启动时读取，根 Python 脚本直接执行。如确需改已安装组件，先重新审批范围，再定向构建并检查 OpenCV ABI。
- 停止条件：不能分离提交归属、校验失败，或无法安全集成。不得把未合并工作报告为已部署 main。

## 验收

完成条件：

- [ ] YAML 深度 profile 为 `848x480x30`，格式 Z16。
- [ ] SDK/设备描述符确认 High Accuracy=3。
- [ ] 正式 launch 独立启动时，实际深度预设、曝光、Gain 和激光值正确。
- [ ] RGB 1280×720@30 和初始 RGB 参数保持不变。
- [ ] 同步、对齐、RGBD 保留，对齐深度仍匹配 RGB 像素空间。
- [ ] 无需 `--controls` 默认出现三个窗口。
- [ ] GUI 全部新增文字为英文，Depth 只有 Exposure/Gain。
- [ ] 深度窗口显示原生 848×480 伪彩色与英文距离图例。
- [ ] 零深度黑色，全零和同值帧无除零或伪造测距。
- [ ] 离线深度显示检查通过，输入数组不变。
- [ ] 深度和 RGB 调参独立，错误不标记成功。
- [ ] Save 先备份，保存成功值，不破坏其他配置。
- [ ] 保存后重启加载相同值。
- [ ] 复用 `/camera` 自动应用 YAML，不创建第二个相机。
- [ ] 有 `/fruit_mission` 时工具拒绝写相机参数。
- [ ] 关闭任一窗口或 q/Esc/Ctrl-C 均退出，只清理自有进程。
- [ ] 本任务语法检查与定向 diff 检查通过。
- [ ] 记录与 Viewer 的同场景对照结果或明确标为未验收。

规划阶段证据：只读检查已完成。没有业务修改，没有联网查询，没有构建、离线检查、GUI 运行或实机测量。所有执行验收项尚未完成。

## 风险

1. **High Accuracy 可能减少有效深度覆盖。** 不新增补洞或改变无效深度处理，不承诺全部目标可测距。
2. **预设与曝光应用存在顺序风险。** 驱动按 SDK 选项顺序初始化。必须独立验证正式 launch，不能只证明 GUI 应用成功。
3. **ROS 回读不完全等于硬件读回。** SDK 联动选项可能令参数缓存与硬件状态不同。结合 metadata、图像尺寸和实际深度效果验收。
4. **固件/SDK 可能量化曝光。** 若实际值无法符合要求，记录范围、步长和实际值，暂停并询问，不擅自改变默认曝光。
5. **profile 更新会短暂中断出流。** 自动应用仅用于人工调试。正式任务运行时拒绝启动工具。
6. **图像分辨率具有两个空间。** 原生深度为 848×480，对齐深度为 1280×720。混用会产生错误测距。
7. **跨进程图像会增加调试负载。** 用户仅批准 GUI 调试例外。不得扩展到正式感知传输架构。
8. **Tk/OpenCV 双事件循环可能影响响应。** 使用已有主循环更新，不加入线程/额外框架。检查控件服务等待和关闭清理。
9. **自适应颜色跨帧变化。** 图例必须显示当前距离范围。伪彩色不能被解释为绝对精度证明。
10. **复用相机修改会保留到现有会话。** 退出工具不停止复用相机，也不暗中恢复参数。未 Save 的修改不写入源 YAML。
11. **GUI 工具原有流程会启动或复用机械臂通信节点。** 验收属于人工硬件会话，不作为无人值守自动测试。不得调用运动服务。
12. **仓库存在大量既有改动。** 当前配置、GUI 脚本和文档均有用户改动。提交、集成必须保护这些改动。
13. **当前无法联网搜索。** 本计划使用已安装 SDK 和本地厂商驱动作为依据，未声称完成网上检索。
14. **截图未指定所有 Viewer 后处理设置。** 本次仅采用用户确认的传感器设置，不自行开启滤波、JSON 或其他 Viewer 功能。
