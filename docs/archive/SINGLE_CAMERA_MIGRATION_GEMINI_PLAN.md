# 单相机化与左右相机彻底清理：Gemini 执行方案

> 历史归档：单相机清理已实施，保留原方案和当次检查记录；下文执行入口不再适用。当前说明见 [项目架构](../PROJECT_ARCHITECTURE.md) 和 [两阶段运行文档](../../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md)。未进行的硬件采摘验证仍未完成。

## 1. 文档用途与执行入口

本方案由用户经过两轮 `grill-me` 澄清后确认，供 Gemini 后续实施。

**状态更新（2026-10-02）：单相机清理已经实施；本文原始模型边界保留作为历史方案，
不代表当前部署。** 当前工作区另有两阶段模型接入，实际模型与标签契约以
`src/fruit_perception/README.md` 和当前源配置为准。Review 后修复 A 裸熟度标签、
C 辣椒标签、检测调参工具、图片保存/预览及 DETECT 断流监控；未进行硬件采摘验证。

验证记录更正：此前 `colcon test-result` 的“15 tests”来自 `loam_interface`，
不是感知包；当时感知包 `BUILD_TESTING=OFF`。Review 直接运行已有 launch 检查
13 项和模式检查 6 项通过，不能据此宣称完整感知或实机采摘已验证。

修复后验证：`fruit_perception` 构建通过，OpenCV 链接仍为系统 4.5.4；
无窗口检查通过，包括 A/C 标签与任务等待匹配、未成熟候选排除、四区模型路径和
ONNX 元数据、GUI 物种标签解析、写入失败提示、1000 帧预览对象复用与 DETECT 断流状态。
工具 `--help`、Python/Bash 语法和 `git diff --check` 通过。未启动相机采集或硬件运动。

当时提供给 Gemini 的执行入口（仅作历史记录，不要重新执行）：

> 请执行 `/home/rc4/fruit_ws/docs/SINGLE_CAMERA_MIGRATION_GEMINI_PLAN.md`。先读取当前工作区的 AGENTS.md 和实际代码，保留已有及并发修改；仅实施本文定义的单相机化与旧侧相机清理，不启用新两阶段模型，不增加 D 区任务，不写新 tests。按本文完成相关构建、无运动验证和文档更新，最后合入 main。需要启动机械臂、底盘或正式任务时先向我确认。

本文记录的是已确认范围。若实际代码或硬件条件出现会改变这些决策的冲突，应说明冲突并询问用户，不能自行换方案。

## 2. 已确认决策：不得再次降级为“只禁用”

| 项目 | 用户已确认的结果 |
| --- | --- |
| 相机数量 | 项目只使用一个 `camera`，即当前深度相机；左右 USB 相机完全不再使用 |
| 清理方式 | 删除左右相机专用代码、配置、标定、依赖、旧脚本和脚本/GUI 的多余分支，不向下兼容 |
| 识别区域 | 感知支持 A/B/C/D，四区均从同一个 `camera` 获取图像 |
| 正式任务 | 保持现有 A/B/C 导航采摘流程；不新增 D 区导航、二维码计划或机械臂任务 |
| 模型 | 保留当前 `A.onnx`、`B.onnx`、`C.onnx`，不启用归档的新两阶段模型 |
| A 区观察 | 每个停车点先左后右依次观察，可通过配置调整顺序 |
| A 区抓取 | 每侧最多抓一个成熟、深度有效的目标；不是循环抓到空 |
| D 区感知 | 复用 B 模型及 B 的稳定多目标候选策略，最多两个候选；不是一次抓两个 |
| 消息 | 删除 `FruitDecision.source` 和所有 `SOURCE_*` 来源常量 |
| 相机模式 | 只接受 `OFF / QR / DETECT`；删除 `SIDES`，将 `TREE` 改为 `DETECT`，不保留别名 |
| 方向控制 | 保留机械臂向左/向右观察与抓取、导航左右方位和必要方向协议 |
| 数据与系统 | 不删除历史录包、训练图像、模型归档；不默认卸载系统软件或修改系统 udev |

**“左右相机”与“左右观察方向”必须区分。** 不能通过全局删除 `left/right/side` 字样破坏机械臂、航点或 C 区到位方向计算。

## 3. 实施边界与不可破坏项

### 3.1 本轮必须完成

- 从正式启动、感知、任务、调试、采集、参数 GUI、监控、录包说明及依赖声明中清除侧相机链路。
- A 区不再等待侧相机消息，也不再通过侧相机选择抓取方向；直接按配置顺序用 `camera` 观察和闭环抓取。
- D 区真正取得 B 模型推理结果及稳定候选，不能仅在区域字符串校验中接受 `D`。
- 修改消息定义后，同步所有发布者、订阅者、调试工具和相关构建产物。
- 同步维护受影响文档，不把未实现的两阶段模型写成已经启用。

### 3.2 不做的事情

- 不增加摄像头替代实现、兼容模式、旧参数映射、旧命令别名或录包转换器。
- 不引入新 Python ROS 推理节点、新模型框架、新依赖或通用多摄像头抽象。
- 不改变 B/C 区任务计划、导航路线、二维码业务、语音顺序、机械臂协议数值及恢复流程。
- 不调整相机分辨率、曝光、白平衡、同步/深度对齐等现场参数；侧相机参数则直接删除。
- 不改 camera/lidar 的现有命名迁移，不重新引入 D435i/MID-360 项目接口名。
- 不改雷达配置、TF 数值、相机坐标系、深度单位或 ArmTarget 坐标转换。
- 不删除无关旧文件、用户数据、第三方驱动或其他开发 worktree。

### 3.3 工程规则

- 先读当前 `AGENTS.md`；ROS 启动和构建前必须 source `scripts/setup_fruit_runtime.bash`。
- 功能包参数仍放 `config/`，launch 按现有 `template.launch.py` 风格，配置/模型文件直接引用 `src` 路径。
- 相机驱动与 C++ 图像处理组件仍在同一个 `ComposableNodeContainer`，各组件保持 `use_intra_process_comms=True`。
- 保留必要的锁存状态 QoS；Humble 下 `TRANSIENT_LOCAL` 发布器/订阅器若在启用进程内通信的组件内，应仅对该端点禁用进程内通信，不能改成 `VOLATILE` 或关闭整个组件的进程内通信。
- 继续使用系统 OpenCV 4.5.4、系统 NumPy 1.x 与现有 OpenVINO GPU 推理，不混入 `~/.local` 或 `/usr/local` 的视觉库。
- 不写新 tests。可以运行仍有效的现有检查；删除侧相机专用旧测试及混合测试中失效的用例/夹具/导入，现有测试对重命名接口只做必要同步，不借机重写测试体系。
- 非平凡逻辑必须留下最小可复现的检查方法和执行记录；优先使用现有检查、ROS CLI 或临时验证，不新增 tests 文件。
- 配置是现场可调参数；验证类型、合法范围和必要关系，不用测试固定当前曝光、阈值、帧数或顺序数值。

## 4. 已核对的当前代码事实与陷阱

以下事实来自方案编写时 `/home/rc4/fruit_ws` 的 `main` 工作区。执行前重新核实，不能把旧 worktree 的搜索结果当作当前实现。

1. `mission_flow.py::run_a_area()` 仍进入 `SIDES`，调用 `observe_sides()` 和 `_a_pick_decisions()`，再调用 camera 闭环抓取。只删除 USB launch 节点会使 A 区失去任务入口。
2. `mission_context.py` 保存 `_latest_left/_latest_right`，依据 `FruitDecision.source` 分流；`camera_arm_target_client.py` 和监控脚本也检查来源常量。
3. `camera_mode_manager.cpp` 的启动就绪检查同时等待两侧 `/set_capture` 服务。只删相机节点而不删这些等待条件，会让模式管理器一直未初始化。
4. `fruit_perception_node.cpp` 的区域订阅已经接受 `D`，但 `select_camera_request()` 只为 A/B/C 返回请求，多目标候选分支目前仅判断 `area == 'B'`。D 的这两处都必须接通。
5. 当前模型标签不是未来两阶段模型标签：

   | 区域 | 当前权重 | 本轮保留的标签与选择策略 |
   | --- | --- | --- |
   | A | `model/A.onnx` | `ripe / unripe`；单目标锁定；只有 `ripe` 可抓 |
   | B | `model/B.onnx` | `apple_ripe/apple_unripe/pear_ripe/pear_unripe`；现有 B 多候选策略 |
   | C | `model/C.onnx` | `chili/pumpkin/onion/tomato` 的 `_ripe/_unripe`；现有期望品种过滤与单目标锁定 |
   | D | 同一个 `model/B.onnx` | 同 B 标签和候选策略，不需要 `D.onnx` |

6. `DetectionLabelSet::kSideFruit`、`kSideFruitTypes` 和调试脚本的 `SIDE_FRUIT_TYPES` 实际表示 C 模型类别，不是侧相机硬件实现。不得连同 C 标签解析一起删除；若改成清晰的蔬菜类别名称，保持类别顺序和输出字符串完全不变。尤其不能在本轮把 `chili` 改为 `pepper`。
7. `_wait_camera_target()` 按 `<品种>_unripe` 构造未成熟标签，直接传 `expected_type='ripe'` 时会构造错误的 `ripe_unripe`。A 区需要支持裸标签 `unripe`，不能沿用这个错误匹配或影响 B/C 标签匹配。
8. 重复确认 `TREE` 目前会增加目标选择代次，清除旧观察窗口；异步推理结果也检查代次。改成 `DETECT` 后仍须保留这些新观察语义。
9. 当前工作区有大量已修改/删除/未跟踪文件，还存在 `perception-two-stage` 等其他 worktree。不得 reset/clean/stash 用户改动，也不得顺手合并其他任务的整条分支。

## 5. 目标接口与行为

### 5.1 保留的 ROS 接口

以下是当前接口名称，迁移不另改名；现场仍可通过对应 YAML 调整：

```text
/camera/rgbd
/camera/color/image_raw
/camera/set_parameters
/camera/get_parameters
/fruit_perception/camera/annotated_image
/fruit_perception/camera/rgbd_heartbeat
/fruit_perception/decision
/fruit_mission/perception_mode
/fruit_perception/active_mode
/fruit_mission/current_area
/fruit_mission/expected_fruit_type
```

相机组件继续使用节点名 `camera`、根命名空间 `/`。不要恢复 `/camera/camera/*`，不要因为第三方驱动内部名字而改其 API。

删除 `/left_camera/*`、`/right_camera/*`、`/fruit_perception/left/*`、`/fruit_perception/right/*` 相关自有发布、订阅、服务调用及启动检查。

### 5.2 FruitDecision 目标定义

只删除来源字段与常量，其他字段、顺序、类型和语义保留：

```text
std_msgs/Header header
string[] types
FruitDetection[] detections
FruitTarget[] targets
bool targets_ready
bool has_target
geometry_msgs/Point target_point
```

- 一个决策话题天然只有 camera 来源；消费者不再做来源枚举分流。
- 保留 header、frame_id、原始检测、稳定候选、就绪状态及深度目标；不能把“只剩一个相机”当作理由删除深度/时序校验。
- 不改 `FruitDetection`、`FruitTarget` 和其他无关消息定义。
- 不兼容旧的 FruitDecision 消息定义；旧录包即使 remap 话题也不会因此兼容。所有使用此消息的运行进程需停下后统一重构建、重启，不能混用新旧生成代码。

### 5.3 模式状态机

| 模式 | 相机采集 | 果蔬推理 | 用途 |
| --- | --- | --- | --- |
| OFF | 关闭，沿用现有参数服务控制 | 不运行 | 初始化、退出、停止 |
| QR | 开启，保留现有扫码所需流设置 | 不运行果蔬检测 | 原有 B/C 扫码流程 |
| DETECT | 开启 | 根据 A/B/C/D 使用对应旧模型 | 所有区域果蔬识别 |

- `QR ↔ DETECT` 不应仅因名字不同而反复关闭再打开相机流；沿用现有“两个模式均使用 camera”的切换语义。
- 启动复位只需要 camera，不再等任何 USB 服务。
- 非法模式、参数服务不存在、服务返回失败或异常时，不能发布虚假的成功 active_mode，也不能继续发送抓取目标。
- 只解析 `OFF/QR/DETECT`。`TREE/SIDES` 应明确拒绝，不静默映射。
- 同一次模式重复请求 `DETECT`，仍须有成功回显并触发新的观察窗口；区域或期望类别改变后也必须废弃旧代次结果。
- 错误码、日志、帮助文本及 API 函数名中专指旧识别模式的 `tree` 要同步更新；真实水果类别、B 区树位等语义不做机械替换。

### 5.4 A 区单相机闭环

配置新增一个明确的 A 区观察顺序，建议落地为：

```yaml
a_observation_order: ["left", "right"]
```

这是用户确认的默认顺序，现场可改成先右后左。校验应要求它是两个互不重复的合法方向组成的列表，恰好覆盖 `left/right`；不能允许漏观察一侧、重复抓同侧或通过旧 `preferred_side` 隐式回退。

替换现有 A 区流程，保留共用闭环函数与回调等待，参考如下：

```text
上报 A 区
依次到达现有 A 区航点
    沿用导航成功/失败处理和 stable_stop
    按 a_observation_order 处理每个方向
        发布期望类别 ripe
        执行现有 PICK_DIRECTION 方向动作，等待真实完成回调
        记录动作完成后的新鲜度边界，进入/重新确认 DETECT
        用 camera 在既有有界观察窗内形成成熟且深度有效的稳定单目标
        有目标：发送一次 ArmTarget，等待现有抓取完成/故障回调
        无可抓目标：不发送 ArmTarget，按既有安全恢复结果决定能否继续
        如果无法安全继续：停止任务，不观察下一侧、不导航下一点
    完成两侧后继续下一航点
任务退出（包括异常路径）：请求 OFF
```

实施要点：

1. 优先复用 `execute_closed_loop_pick()`，A 区传入 `expected_type='ripe'`；不要另造并行抓取执行器。
2. **成熟筛选必须发生在感知候选进入单目标锁定之前。** 不能先锁定未成熟目标，再因 `types` 曾包含 `ripe` 就把该深度点发送给机械臂。A 区原始识别/标注仍保留 `unripe` 以便调试。
3. 对 A 区支持 `ripe/unripe` 裸标签；在既有 `camera_detection_timeout_sec` 有界窗口内等待有效成熟锁定，不因第一帧只见未成熟目标就误判整个观察结束。超时结果可以区分无检测、仅未成熟、成熟但深度无效；不伪造坐标。
4. 保留目标有限值、有效深度、新时间戳、帧坐标系及异步代次校验。左右观察之间、不同航点之间不能复用旧锁定、旧标签缓存或旧推理输出。
5. 最多每侧一次 ArmTarget；两侧都没成熟目标时正常跳过，不补目标、不抓未成熟目标、不循环重抓。
6. 感知/方向/抓取失败后的安全姿态恢复及 `safe_to_continue` 处理沿用现有实现。无目标也不能在机械臂仍处于不安全姿态时直接导航。
7. 删除 `observe_sides()`、`_a_pick_decisions()`、`choose_maturity_side()` 等旧预筛选路径，不能把它们改造成返回假 SideObservation 的兼容层。
8. 移除 A 专用旧全局 `preferred_side` 和 `side_observation_timeout_sec` 及其声明/校验/调用。C 区导航结果中同名的真实方向信息不是侧相机开关，不能误删。

### 5.5 B/C 保持现有行为，D 复用 B

- B：保留二维码任务、左右树位观察、候选稳定化、期望水果匹配及原有抓取次数/顺序。
- C：保留二维码分配表、到位后方向计算、期望 `<品种>_ripe` 过滤及单目标闭环。
- B/C 的旧 `TREE` 命令统一替换为 `DETECT`；不是重新设计抓取逻辑。
- D：`select_camera_request()` 返回已有 B 推理请求，稳定候选分支使用与 B 相同的选择器、参数、消息输出及就绪语义。
- 保留现有最多两个稳定候选的配置和策略，不截断原始 `types/detections` 来凑数量，不按预期数量补目标。
- D 通过 `current_area` 控制感知即可；不扩展现有任务入口 `mission_flow`，不虚构 D 航点或新增串口命令。
- 模型仍在启动时编译、按区域选请求；不要为 D 复制一份 B 权重或每次切区重新编译。

## 6. 文件级执行清单

下表是已核对的重点位置，不是完整搜索结果。执行者仍需从 main 的真实源码扫描所有调用者，尤其是未跟踪的新名称文件。

| 位置 | 修改/删除要求 |
| --- | --- |
| `src/fruit_perception/msg/FruitDecision.msg` | 删除 source 与全部来源常量 |
| `src/fruit_perception/src/fruit_perception_node.cpp` | 删除 USB 图像流、侧相机请求/锁定/分流/参数；保留 RGBD 深度链路；接通 D；A 成熟过滤；保持代次失效与异常处理 |
| `src/fruit_perception/include/fruit_perception/side_target_selector.hpp`、`src/fruit_perception/src/side_target_selector.cpp` | 删除侧相机专用选择器及所有构建引用 |
| `src/fruit_perception/include/fruit_perception/perception_mode.hpp`、`src/fruit_perception/src/perception_mode.cpp` | 删除 kSides、侧相机动作、USB 返回值兼容判断；kTree 改为 kDetect；只保留 camera 模式切换 |
| `src/fruit_perception/src/camera_mode_manager.cpp` | 删除两侧 SetBool 客户端、就绪条件、并发结果计数、控制动作和参数；保留 camera 参数服务成功检查 |
| `src/fruit_perception/launch/fruit_perception.launch.py` | 删除 usb_cam 组件、侧相机 remap、V4L2 设置函数及相应 OpaqueFunction；保留根命名空间 camera、同容器图像处理、QR 与现有串口启动开关 |
| `src/fruit_perception/config/fruit_perception.yaml` | 删除 left_camera/right_camera/usb_camera_controls、enable_left/right、left/right 输入与标注话题、side_target_*、left/right_capture_service；保留 camera 调参与旧模型配置 |
| `src/fruit_perception/config/left_camera_calibration.yaml`、`left_camera_1280x720_calibration.yaml`、`right_camera_calibration.yaml`、`right_camera_1280x720_uncalibrated.yaml` | 删除四个侧相机专用标定文件，确认没有残余加载引用 |
| `src/fruit_perception/config/fruit_perception_4.23_8_20_backup.yaml` | 核对调用者；若只是不用的旧侧相机备份，删除而不是保留可重新启用的模板；若仍被有效工具加载，先迁移调用再处理 |
| `src/fruit_perception/CMakeLists.txt`、`package.xml` | 删除侧选择器源与 usb_cam 依赖；std_srvs 若只用于侧相机则删依赖；保留其他仍被调用的库与测试注册 |
| `src/fruit_mission/fruit_mission/mission_flow.py` | A 按顺序直接调用 camera 闭环；删除旧选侧函数/导入；B/C 改 DETECT |
| `src/fruit_mission/fruit_mission/mission_context.py` | 删除左右缓存和消息分流、observe_sides、无调用的侧相机开环旧函数；支持 A 裸熟度标签；保持 B/C 恢复和时序语义 |
| `src/fruit_mission/fruit_mission/logic.py` | 核对调用者后删除 SideObservation、旧 PickDecision 与旧选侧/侧相机筛选函数；保留真正用于 B/C、方向、结果和标签映射的帮助函数 |
| `src/fruit_mission/config/fruit_mission.yaml`、`src/fruit_mission/fruit_mission/config.py` | 新 A 顺序参数及合法性校验；删除旧 A 侧相机观察参数；不改其他现场参数 |
| `src/fruit_mission/fruit_mission/camera_arm_target_client.py`、`config/script_ros_clients.yaml`、`test_camera_arm_target.sh` | 去掉消息来源判断，命令/日志改 DETECT；机械臂方向与闭环调试能力保留；实机脚本不能用于无运动验收 |
| `start_fruit_mission.sh` | 删除 `/dev/left_cam`、`/dev/right_cam` 设备要求和两侧节点等待；保留 camera、STM32、其他真实硬件及恢复检查 |
| `collect_cam_img.py` | 只采集 camera；删除 `--cam` 选择参数、USB 启动/标定/控制及多相机选择结构；保留保存、退出清理和 GUI |
| `test_camera_detector.py` | 它是保留的人工检测调试工具，不是应整文件删除的 USB 测试；去掉 `--cam1/2/3` 选择，默认 camera；保留模型/阈值选择、C 标签解析与 GUI |
| `camera_control_gui.py` | 保留 camera 实时参数与保存功能；删除 V4L2/USB 分支及旧 USB 调参辅助导入；保留保存失败处理、备份与防止数据丢失的逻辑 |
| `scripts/robot_debug_monitor.sh` | 删除侧相机节点监测及来源常量映射；统一显示 camera 决策，保留其他监测 |
| `kill_fruit_processes.sh` | 移除已删除侧相机/旧相机选择命令的匹配项；保留唯一 camera 工具、容器及其他实际进程的清理 |
| `record_rosbag.sh`、`fruit-perception.service` | 核对并移除侧相机专用规则/参数/说明；保留 camera 录包策略与正式运行环境加载 |
| `src/fruit_qr_planner` 相关配置、代码和文档 | 核对模式消费与图像话题；仍从 camera 扫码，不扩展 QR 任务语义 |
| 现有 `src/fruit_perception/test`、`src/fruit_mission/test` 及其他消费者检查 | 删除侧相机专用残留，必要同步新模式/消息定义；不新增测试、不删除仍有效的 B/C、深度、协议和安全检查 |

### 6.1 单相机工具的目标命令

camera 是唯一来源，不再要求编号，也不接受旧编号作为别名：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 collect_cam_img.py --help
python3 test_camera_detector.py --model A
python3 test_camera_detector.py --model B
python3 test_camera_detector.py --model C
```

采集工具已有输出目录等非相机选择参数继续保留，实际采集命令按其更新后的 `--help` 写入数据集文档。

若调试工具提供 D 区选项，应明确映射到 B 模型，而非尝试加载不存在的 D.onnx；不要求为这一映射新建通用模型注册层。

GUI 仍默认显示到机器人桌面，创建窗口前保持：

```text
DISPLAY=:0
XDG_RUNTIME_DIR=/run/user/$(id -u)
XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
```

不因为删除 USB 分支而改成远程终端依赖 DISPLAY，也不移除 YAML 保存备份和错误提示。

### 6.2 文档更新清单

实施时至少核对并更新：

- 根 `AGENTS.md`：移除“继续使用左右相机”和侧相机设备命名规范，保留 camera/lidar 及其他约束。
- 根 `README.md`：单相机概述、接口表、设备检查、启动及调试入口。
- `src/fruit_perception/README.md`：模型路由、决策消息、模式、启动与调试。
- `docs/PROJECT_ARCHITECTURE.md`：A 区单相机流程、状态机、消息、同容器链路。
- `docs/PERCEPTION_MODE_COMMANDS.md`：仅 OFF/QR/DETECT，去除侧相机命令及过时“没有任务节点”描述。
- `docs/CAMERA_TUNING.md`、`docs/DATASET_PIPELINE.md`：默认 camera，无 `--cam/--cam1/2/3`；保留当前有效训练/采集步骤。
- 任务、语音、机械臂调试、QR、录包及启动文档中实际受影响的段落；不改无关导航内容。
- `src/fruit_perception/MODEL_UPDATE_RC4.md`：将其“侧相机旧代码不删除”“保留 source 枚举”“OFF/QR/TREE”等未来接入指导对齐本方案；明确两阶段模型仍未启用、未来 A 标签迁移尚未完成。
- 归档模型目录内的原始交付说明视为历史资料，不改权重、校验和或历史交付事实；从当前说明指明它的旧接口描述已过时即可。
- 本文最终补实施状态、实际验证结果和硬件未验证项，不能仅写“已完成”而无证据。

## 7. 建议执行顺序与工作区保护

### 阶段 0：重新建立实际基线

1. 读取 `AGENTS.md`、本文、现有模型说明及受影响代码；涉及规则判断时读取 `docs/RULE.md`，不凭模型说明推断比赛规则。
2. 查看 `git status`、`git diff`、未跟踪文件与 `git worktree list`，记录当前状态。未跟踪文件中已有 camera 新名称实现，不能遗漏。
3. 检查是否有其他 agent 正在改感知/任务。如果存在重叠，应协调，不在同一工作区并行修改。
4. 这是跨功能包、多文件和消息接口变更，按 AGENTS 的大任务要求使用独立 worktree；先确认 `.worktrees/` 被忽略。
5. **新 worktree 从 HEAD 创建不会带入 main 的未提交改动。** 应使用任务相关、可审查的基线补丁/必要未跟踪文件同步建立基线，不得丢掉当前命名迁移、机械臂和导航修改，也不得直接复制全部脏文件后一起合入。
6. 用 `tmp/` 保存任务计划、基线记录、构建与检查日志；保护其他任务的临时资料。

### 阶段 1：接口、模式与感知

完成 FruitDecision 裁剪、模式改名、侧相机节点/配置/锁定清理、D 路由和 A 成熟候选过滤。核对所有调用者，避免先运行混合新旧消息进程。

### 阶段 2：A 区任务与共用消费者

添加/校验 A 观察顺序，改 A 停车点流程，修正 A 裸熟度标签判断与新鲜度边界，移除旧 SideObservation 逻辑。同步 B/C、人工客户端和监控的模式/消息消费，不改变其业务规则。

### 阶段 3：工具、启动、旧文件与文档

清除 GUI/采集/调试中的 USB 分支、设备要求、依赖、旧标定和不可再用的测试；更新当前文档，全面搜索残留。保留非侧相机功能，不添加兼容入口。

### 阶段 4：验证与集成

按下一节执行安全检查，记录实际结果。将任务改动合入用户当前 main 的工作区，重新核对 main 的新增并发改动并复验，不能覆盖用户修改或只交付一个未合回的 worktree。

提交范围仅限本任务；没有用户或仓库流程授权时，不把用户已有脏改动打包提交。若无法隔离，应先询问。

按项目规范，已合回的本地开发分支应清理；不得删除其他实验分支。运行仍只认 main，不切开发分支使用机器人。

## 8. 验证计划：静态与无运动优先

### 8.1 源码与配置审计

- Python/launch 语法、shell 语法、XML、相关 YAML 均可解析；所有现用路径存在。
- A 顺序参数类型正确、覆盖两个方向且不重复；支持反向顺序，不锁死默认值。
- 没有旧 source 常量/字段的发布、读取或样例；没有旧 USB 文件导入、标定路径或依赖。
- 正式程序、脚本和当前用法说明不存在可执行的 SIDES/TREE 入口和旧相机编号参数。
- 保留模型解析对无效类别/输出形状、图像与深度编码/布局、ROI、非有限值和无效深度的必要校验。
- 检查 `git diff --check`，审查删除清单和文件权限，不顺带格式化或修复无关代码。

建议搜索词：

```text
left_camera right_camera /dev/left_cam /dev/right_cam
usb_cam usb_camera_controls left_capture_service right_capture_service
SideObservation choose_maturity_side observe_sides side_target_selector
enable_left_camera enable_right_camera side_observation_timeout_sec
SOURCE_LEFT SOURCE_RIGHT SOURCE_CAMERA message.source result.source
SIDES kSides kTree TREE tree_mode_timeout
--cam --cam1 --cam2 --cam3
```

搜索范围应覆盖主工作区的自有代码、配置、脚本和当前文档，排除 `.git/.worktrees/build/install/log/tmp`、第三方驱动、模型二进制与原始归档资料。

不能要求整个仓库“零字符串命中”：本文说明旧接口、历史归档和真实树位/方向名称会合法命中。逐项判断，清除的是当前可执行旧链路和错误现状说明，不是历史资料或机械臂方向。

### 8.2 构建与生成接口

在未激活训练 venv 的终端先加载完整运行环境，再构建受影响包及实际依赖/消费者。以下为主工作区示例，worktree 的构建路径须按其基线和隔离设置适配：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --packages-up-to fruit_perception fruit_mission fruit_qr_planner
source scripts/setup_fruit_runtime.bash
ros2 interface show fruit_perception/msg/FruitDecision
ros2 launch fruit_perception fruit_perception.launch.py --show-args
git diff --check
```

- 根据真实消费者补充构建范围；不为了图省事重构全部项目。
- worktree 验证前核对实际加载的配置来源：当前 launch 内有主工作区绝对路径，不能用开发分支的新二进制配 main 的旧 YAML 来声称验证成功。验证使用对应源目录配置，正式集成仍使用 main 的 `src` 路径；不提交 `.worktrees` 路径或改为 install 配置。
- 检查 Python 生成消息的字段和 C++ 消费者编译结果；source 字段及常量必须已消失，其余字段保留。
- 消息变更如有旧生成物，定向重建相关包，不能用手工编辑 install 消息或整仓库清理替代。
- 用 `ldd` 检查感知组件的 OpenCV 来源，不得链接 `/usr/local/lib/libopencv*`。
- 运行仍有效的现有相关检查；记录执行命令、通过/失败/未运行原因，不添加 tests，不通过删除无关失败用例伪造通过。

### 8.3 无机械臂、底盘运动的运行验证

先确认没有任务节点、机械臂控制节点或其他进程响应模式/区域消息，选择空闲的隔离 ROS domain；隔离 domain 本身不能替代对实际运行节点的检查。

若 camera 硬件可用，可只启动感知，使用已有关闭串口启动开关：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
# 先确认该 domain 空闲；同一轮验证的所有终端使用一致 domain。
export ROS_DOMAIN_ID=89
ros2 launch fruit_perception fruit_perception.launch.py start_serial_bridge:=false
```

不启动 `start_fruit_mission.sh`、`test_camera_arm_target.sh` 或发送 ArmTarget/方向/串口命令用于本节验收。

验证：

1. 没有两侧设备时感知启动不应再因 `/dev/left_cam/right_cam`、V4L2 命令或 USB set_capture 服务而失败/卡住。
2. `ros2 component list` 显示 camera 与 FruitPerceptionNode 同容器；图中没有左右相机节点/话题。
3. `ros2 topic echo /fruit_perception/active_mode` 观察真实回显；OFF、QR、DETECT 正常切换，TREE/SIDES 拒绝且不误报成功。
4. 切换 `current_area` 为 A/B/C/D，进入 DETECT 后观察决策/标注；D 输出来自 B 模型，并提供与 B 一致的稳定候选。
5. 重复 DETECT 或更换区域/期望类别后旧锁定被清除，旧异步结果不能污染新区域/新方向。
6. A 场景包含未成熟与成熟目标时，显示两者但可抓锁定只能来自成熟目标；无有效深度不发布可抓点。
7. OFF 停止采集及果蔬推理；QR 仍可扫码但不运行果蔬推理。
8. 各次验证结束请求 OFF，停止本轮启动的进程；不要留下相机占用或隔离域节点。

发布模式与区域的示例（仅在上述隔离、无执行机构控制节点的前提下使用）：

```bash
ros2 topic pub --once /fruit_mission/current_area std_msgs/msg/String "{data: 'D'}"
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'DETECT'}"
ros2 topic echo /fruit_perception/decision
# 完成后在同一 domain 请求 OFF，再停止验证进程。
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'OFF'}"
```

**camera 硬件缺失时，这部分必须标记未验证。** 不能以能构建、只有左右 USB 缺失或代码看起来正确代替真实 camera 流验证；也不需要为了消除缺失硬件而新增模拟框架。

### 8.4 任务行为检查与实机授权边界

通过现有 fake context/检查、临时最小验证或代码路径复核，至少覆盖：

- A 默认顺序左→右，反向配置右→左；每侧最多一次抓取。
- 两侧都无目标、只有未成熟、成熟但深度无效：不发 ArmTarget，并遵守安全恢复结果。
- 成熟/未成熟同框时只选成熟；观察窗口内从未成熟帧变成成熟帧时不被首帧提前误拒。
- 方向回调超时、模式失败、抓取失败、安全恢复失败：不越过既有禁止继续边界。
- 左观察缓存、上一航点结果和旧异步输出不能成为下一次抓取目标。
- B/C 的二维码计划、品种过滤、方向和恢复流程没有被来源/模式改动破坏。
- D 区没有被加入任务路由，没有新增真实机械臂/底盘动作。

实际 A 区两侧观察、抓取姿态、B/C 连续任务和真实 QR/ArmTarget 验证，需要用户另行授权和现场安全条件。没有授权不发送运动命令，不把它们写成已经验证。

## 9. 完成标准与交付格式

实施检查确认：

- [x] 自有侧相机链路、专用文件、依赖和工具编号选择已删除，不是仅置 false。
- [x] 唯一 camera 的 A/B/C/D 路由已实现；当前模型已是两阶段链路，不再是原方案旧模型。
- [x] A 按可调顺序观察两侧，每侧最多一个成熟有效目标，保持安全恢复。
- [x] D 确实复用 B 推理与候选；正式任务仍只有 A/B/C。
- [x] FruitDecision 不含 source 或来源常量，消费者和生成接口一致。
- [ ] OFF/QR/DETECT 硬件流切换与采摘完整验证（静态模式检查已通过，实机验证待授权）。
- [x] 启动不依赖侧相机设备/服务，GUI 与采集/调试仍可只用 camera。
- [x] 文档准确反映当前状态，两阶段模型说明已明确与本轮的边界。
- [x] 构建、已有检查及最小验证有记录；之前错误的感知测试数量已在文首更正。
- [x] 已合回 main 且保护用户/并发修改，没有只留开发分支或覆盖其他任务。
- [x] 未做的硬件验证、授权需求、消息/命令不兼容影响已明确列出。

最终报告以结果开头，简述变更文件/删除项、实际验证命令和结果、main 集成状态及剩余硬件验证。不要把计划或静态审查描述成实机验证，也不要在本任务结束时自动启用归档模型。
