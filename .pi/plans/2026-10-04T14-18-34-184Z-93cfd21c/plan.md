## 目标

### 执行中用户确认更新（2026-10-04）

用户明确要求“需要执行放果与 Harvest 动作，然后回 home”。本确认覆盖下文决策9及 review.md 中 R2 的原停止于目标点要求：手动客户端 CLOSE/HOLD 成功后保留对应侧 store 放果工序，再显式执行 home 并确认完成；正式 mission 原配置 Harvest→Home 不删除。store 是手动工具的放果工序，不另叠加一套 Harvest；不改 planner 航点/标定，不运行真实运动。R1/R3 修复、修复步骤2关联的 R4 校验与其余离线范围不变。禁止自动提交/推送/部署和自动 review。

后续配置确认：因 planner 未定义 left_harvest/right_harvest，用户明确选择将 mission 的 harvest_left/harvest_right 映射到已有 left_store/right_store。仅改调用方两个预设字符串，不改 planner 真实航点；既有 CLOSE→Harvest 放果→Home 顺序保留。

将当前单 RealSense D435i 链路改为相机持续出流、模型常驻且持续推理、仅在任务到位后的观察窗口内累积和采纳抓取/二维码证据。完整删除历史 OFF/QR/DETECT 状态机、CameraModeManager、相机硬件流启停请求及上层模式回声等待。

用户已通过四轮决策和最终需求摘要确认理解一致，授权编写本计划；尚未授权实现。本阶段只读取业务代码、提问并保存本文，没有运行代码、构建、测试、ROS 命令或硬件控制。

成功结果是减少硬件启停与任务握手环节，确保本次采纳的数据来自到位后的本次窗口。1～2 秒收益、0.3～0.5 秒锁定、功耗增加百分比均不是已验证指标，不作为交付承诺。

## 范围

- 修改 `src/fruit_perception/` 的节点、消息、构建、launch 和配置；删除模式管理器及其专用状态机和测试。
- 修改 `src/fruit_mission/` 的公共 ROS 编排、A/B/C 动作、生命周期、单点调试、启动就绪客户端及配置。
- 修改 `src/fruit_qr_planner/` 的扫码窗口接入和计划输出元数据，保留解析算法、mission 所有的固定兜底与现有 Python 进程布局。
- 对齐 `start_fruit_mission.sh`、`test_camera_arm_target.sh`、`scripts/robot_debug_monitor.sh`、`kill_fruit_processes.sh` 的在用依赖和展示。已读 `record_rosbag.sh`：它使用 `--all`，现有排除规则不会排除新窗口话题，故无需修改其录制逻辑；仅对齐文档中的记录说明。
- 更新直接受影响的持久参考文档。保留历史归档，不重写过去的验收记录。
- 用户允许在忽略目录 `tmp/` 留一份最小、可重跑、无硬件运动的黑盒运行检查；不新增单测框架、白盒/源码扫描测试或策略测试。
- 当前工作区大量文件已修改，包括 mission 主流程、planner、消息、文档和脚本。执行阶段必须保存这些用户改动，基于当前有效内容在隔离 worktree 开发，验证后将任务差异交回原活动分支。

非目标：不增加 D 区任务、坏果移除、换筐、返航、新的总结流程；不重排当前 A→B QR→C QR→C→B 路线；不新增观察/抓取/扫码重试或 C 到点方向复核；不修改手眼外参、运动学、轨迹算法、模型权重、识别阈值、相机分辨率/曝光；不改 vendor RealSense 源码或串口协议；不迁移二维码为 C++ 组件；不重构独立 GUI `test_camera_detector.py`；不删除原有运动保护。

## 决策

1. **完整清理，无旧模式兼容层。** 用户选择删除 OFF/QR/DETECT 控制与回声、管理节点、相关启动检查和调试入口。原生 RealSense 参数服务继续由驱动提供；业务流程不再通过它启停 `enable_color` / `enable_depth`。曝光等人工调参不属于本次删除范围。
2. **持续推理。** 用户选择相机和模型持续运行，而非可信窗口内才推理。保留当前区域模型选择、单帧在途 `busy` 保护和忙时丢帧，不同时无条件运行两个检测器、不堆积推理队列。窗口外仍发布二维/三维检测及按需标注图，但选择器不累积证据，`has_target=false`、`targets_ready=false`、`targets` 为空。
3. **正式任务到位后开窗。** 开窗前必须导航成功且对应 Look/QR 预设返回执行成功。导航与 Look 仍可并行，只有两者完成后才允许本次证据累积。同点左、右各有独立窗口。用户明确选择删除现有 `stable_stop` 等待和只服务于该等待的配置，不新增窗口内运动/反馈监测。用户保证从开窗、整个多帧观察直到目标坐标冻结期间，底盘及机械臂保持物理静止。该明确指令覆盖 AGENTS 中识别前实测双机构静止的默认要求，限于观察开启，不扩展到运动恢复或 planner 控制。
4. **安全恢复保留。** `CameraTargetPlannerClient.is_stationary`、新鲜有限在限位内的实测反馈、运动结果校验、未知结果 STOP、导航取消、EMERGENCY 锁、恢复前 STOP 后新反馈停稳确认和一次 Home 保留。`stationary.joint_speed_rps` 仍有抓取/恢复调用方，不可随 `stable_stop` 一并删除。软件流程继续不是危险运动许可。
5. **带标识的证据窗口，不等模式回声。** 单一窗口命令携带 ID、开始图像时间、有效标志、区域、用途和期望类型；结果带同一 ID。复用选择器 `reset()` 及已有 generation 机制，拒绝窗前 RGB/Depth 图像、旧在途结果、旧 ID 和区域不匹配的结果。窗口结束在客户端立即生效，并异步告知消费者；后续收到的旧消息不能重新触发采纳。新窗口不复用旧 evidence。
6. **窗口接口复用 String/JSON。** 已核实 `fruit_perception/package.xml` 对 `fruit_qr_planner` 有运行依赖；直接让 QR 依赖 perception 新公共消息会形成环。用户选择复用现有 `std_msgs/String` JSON，并严格校验，不新增公共接口包或 JSON 库。
   - 拟定话题 `/fruit_mission/observation_window`，参数名 `observation_window_topic`。
   - 扁平字段：`window_id`（标准库 uuid 生成的非空字符串）、`valid`（严格整数 0/1，拒绝 bool/浮点/其他值）、`start_sec` / `start_nanosec`（ROS 图像截止时间，合法整数字段，纳秒范围 0～999999999）、`area`（A/B/C）、`purpose`（harvest/qr）、`expected_type`（字符串，B 或 QR 可为空）。关闭命令使用原 ID，`valid=0`，保留同一上下文。
   - 使用整数有效标志是跨 Python JSON 与系统 OpenCV FileStorage 的明确编码约定，不混用 JSON true/false。
   - C++ 使用已经链接的 OpenCV core `cv::FileStorage` 的 MEMORY/FORMAT_JSON 读取；Python 使用标准库 `json`。已读取本机 `/usr/include/opencv4/opencv2/core/persistence.hpp`，确认这些 API 及 `FileNode::isInt/isString` 存在；实际解析行为留待离线运行检查验证。
   - `FruitDecision.msg` 新增 `string observation_window_id`；无有效证据窗口时为空，原 `header` 仍是输入深度图像的光学 frame/时间。QR JSON 追加 `observation_window_id`、`image_stamp_sec`、`image_stamp_nanosec`，不改变原任务字段语义。
   - 命令使用 Reliable、VOLATILE、KEEP_LAST(10)，初始无有效窗口，不锁存旧开窗请求。有效窗口中的区域/期望类型取同一命令快照，避免不同话题到达顺序改变本次证据语义；现有区域状态话题继续用于常态模型选择与状态通知。现有期望类型接口若保留，其更新不得覆盖已冻结的窗口上下文，不引入兼容旧模式的路径。
   - QR 计划保持 Reliable / TRANSIENT_LOCAL / depth 1；mission 仅采纳当前窗口 ID、区域和窗后图像时间全部匹配的计划，拒绝旧锁存计划。新代码进程须一致重建/重启。
7. **二维码保持正式扫码时序。** 用户选择不支持导航途中提前采纳，不使用提前缓存。到站、QR 预设成功后才打开对应 B/C 扫码窗口，节点仅在此窗口解码和生成计划；关闭、无窗口、窗前图像均不生成计划。B 失败仍可直接进入 C 扫码；兜底归 mission，失败推进现有流程。保持 `confirm_frames=1`，不是旧文档中三帧或 20 秒等待。
8. **B 一树一次观察。** 仍使用现有 8 帧窗口和最多两个目标；首次抓取前在同一静止观察姿态下转换并冻结全部基座请求，关闭窗口，再执行抓取。第二抓不重新 Look、识别或转换。
9. **手动调试边界。** 用户选择保留操作者固定底盘的台架前提，不新增 `/odometry` 依赖。保留现有 Home→Look→机械臂静止→OPEN/HOLD→窗口观察→目标 CLOSE/HOLD 的流程，完成后留在目标点，不添加 Harvest。删除模式设置/回声/`--mode-off` 清理。
10. **仅离线验收。** 用户选择构建、已有算法测试及 tmp 黑盒运行检查。本次不运行真实 mission/抓取脚本，不连接真实运动服务，不声称硬件出流稳定性、现场准确率、端到端采摘、整机热负载已验收。

核实依据：

- `camera_mode_manager.cpp::initialize_off/set_camera_capture` 启动主动关流，运行时同时设置 color/depth；状态机实现和专用测试均已读取。
- `FruitPerceptionNode` 构造函数一次初始化两个区域检测器及所选熟度分类器；`submit_rgbd/finish_inference` 由模式门控；`CameraTargetSelector::reset` 与 `BTargetSelector::reset` 可复用。B `ready_` 锁定后持续返回旧目标，因此只过滤消息 header 时间不够。
- `MissionContext::set_perception_mode` 等 generation 回声；当前 mission YAML 10 秒，调试 `_set_perception_mode` 5 秒及清理 2 秒，不能将旧描述的 5 秒套入所有调用方。
- `FruitQrPlannerNode::_on_image` 从启动扫描、`PlanSequence` 一次 B→C，C 完成销毁订阅；计划无图像时间/观察 ID。相机常开时必须明确扫描窗口，否则提前缓存会改变采纳时机。
- `start_fruit_mission.sh::wait_for_active_off` 与节点清单、`StackReadinessClient::wait_for_active_mode`、调试清理及监控均依赖旧模式。
- 已读取受影响 manifests、接口、launch、配置、主流程、动作、共享 planner 客户端和文档；`workflow_git status` 显示大量既有脏改动。所有测试证据尚为空，本阶段仅有只读代码核实。

## 执行步骤

以下步骤只能在扩展的人类执行审批后开始。每步失败先停止该步骤及下游；不能删校验、改依赖来源、增加等待或扩大范围来绕过失败。所有 build、ROS CLI、调试及检查终端先运行：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

隔离 worktree 内执行时，先进入该 worktree，仍加载同一干净 runtime；使用 worktree 的 `--base-paths src` 和独立 build/install/log。源码配置路径在最终交付中仍保持 `/home/rc4/fruit_ws/src/...`；离线检查将 worktree 源配置显式传入，避免测试读取原工作区旧配置。不得扫描 `.worktrees/` 发现包。Humble setup 前不启用 `set -u`；禁止通过 PYTHONPATH 绕过 runtime 校验。

1. **冻结基线并建立隔离交付环境。**
   - 文件/符号：当前 Git 状态与任务相关源文件、`.gitignore` 的 worktree/tmp 规则；不编辑业务代码。
   - 改动：记录当前分支、HEAD、全部脏改动和任务覆盖文件；创建短期分支/worktree，将必要的已修改相关文件按当前内容带入，明确区分原用户差异和本任务差异。未跟踪 `ArmTelemetry.msg` 等现有依赖也必须可用，不能复制凭据或覆盖原文件。
   - 依赖：执行审批先于全部后续步骤。
   - 命令：`git status --short`、`git diff -- <任务相关路径>`、`git check-ignore .worktrees/remove-camera-modes`、`git worktree list`，随后经确认可用的 Git worktree 操作建立隔离环境；`colcon list --base-paths src`。
   - 期望：工作树隔离有效、当前有效用户代码完整、包列表无 worktree 嵌套副本。
   - 停止：基线不能安全带入、目标路径未被忽略、源依赖缺失、冲突不能明确归属或 runtime 校验失败。不得直接重置用户工作区或继续用过时 HEAD 开发。

2. **接入证据窗口并解除推理门控。**
   - 文件/符号：`src/fruit_perception/src/fruit_perception_node.cpp::{FruitPerceptionNode,Stream,submit_rgbd,finish_inference,select_camera_request}`；`src/fruit_perception/msg/FruitDecision.msg`；复用 `camera_target_selector.cpp::reset`、`b_target_selector.cpp::reset`，不改其算法。
   - 改动：增加一个窗口订阅和最小快照状态，严格读取上述 JSON；删除 mode subscription、`perception_mode_` 和 `runs_camera_inference` 门控；持续检测/分类。每次提交冻结区域、期望类型、窗口 ID、起始时间和 generation，异步完成不再按可变 current_area 重新解释旧模型结果。开/关窗与区域变化使旧证据失效，选择器 reset 仍由拥有推理状态的处理路径执行，避免 ROS 回调直接并发 reset。只有有效 harvest 窗口、RGB/Depth 时间均在截止时间之后、generation 和上下文匹配时更新选择器及输出目标；窗口外继续生成检测/标注和性能记录。普通检测结果与抓取证据使用明确的空/有效 window ID；异常输出不得伪造窗口内成功。
   - 依赖：步骤 1；窗口契约为后续所有消费者基础。
   - 命令：`colcon build --base-paths src --packages-up-to fruit_perception --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`；重新 source runtime 后 `ros2 interface show fruit_perception/msg/FruitDecision`；`ament_uncrustify src/fruit_perception/src/fruit_perception_node.cpp`；`ldd install/fruit_perception/lib/libfruit_perception_component.so`。
   - 期望：构建成功，消息包含 window ID，系统 OpenCV 4.5.4 链路，无 `/usr/local/lib/libopencv*`；窗口外推理入口没有证据门控副作用。跨节点行为由步骤 7 黑盒验证，当前不能以构建代替隔离证据。
   - 停止：JSON 解析格式不一致、输入校验遗漏、异步旧结果能被标记为新窗口、选择器出现并发所有权问题、构建或 ABI 校验失败；不新加 JSON 库绕过问题。

3. **删除旧管理器与构建/启动包袱。**
   - 文件/符号：删除 `src/fruit_perception/src/camera_mode_manager.cpp`、`src/perception_mode.cpp`、`include/fruit_perception/perception_mode.hpp`、`test/test_perception_mode.cpp`（后三项均相对 `src/fruit_perception/`）；`CMakeLists.txt` 的 perception_mode library、camera_mode_manager executable、链接/安装与专用 gtest；`package.xml` 的仅管理器使用 `rcl_interfaces` 依赖；`launch/fruit_perception.launch.py::generate_launch_description`；`config/fruit_perception.yaml`。
   - 改动：移除专用文件与全部构建引用，不把已删除策略测试改造成新窗口策略单测；移除 launch 管理节点和 YAML manager/mode 参数。相机启动参数维持 color/depth/align/sync/rgbd 开启；camera/perception 保持同容器、各自 intra-process=true，串口启动所有权与 `use_sim_time` 保持现有规则。配置加入窗口话题。
   - 依赖：步骤 2 不再引用模式头文件后删除。
   - 命令：重复步骤 2 构建命令；`/usr/bin/python3 -m compileall -q src/fruit_perception/launch`；`ros2 launch fruit_perception fruit_perception.launch.py --show-args`；`git diff --check -- src/fruit_perception`。通过只读代码搜索核实在用源/配置引用清理，不创建源码匹配测试。
   - 期望：无残留链接/安装目标；launch 无 manager，仍可配置串口是否启动；启动参数解析成功但不启动节点。
   - 停止：残留引用、相机参数被改成关闭、组件布局/QoS被破坏、发现其他业务仍真实依赖被删符号。

4. **二维码生成严格绑定正式窗口。**
   - 文件/符号：`src/fruit_qr_planner/fruit_qr_planner/qr_node.py::{FruitQrPlannerNode,_on_image,_on_current_area,_stop_scanning,encode_plan_json}`；`config/fruit_qr_planner.yaml`；复用 `planner.py::{PlanSequence,build_b_plan,build_c_plan}` 的既有解析，不改二维码 schema 和任务排序。
   - 改动：增加窗口订阅/校验。无有效 qr 窗口或图像时间早于窗口开始即不解码；同一窗口只生成一次有效计划。开启 B/C 窗口时清空上一候选并选择该区域解析状态，不能在 B 窗口提前解析 C，也不能 B 成功后因连续看到同一二维码完成 C。C 窗口不依赖 B 成功；区域通知本身不启窗。计划添加本窗口 ID 和实际图像时间，关闭后不再生成；保留原图像编码、step、长度、UTF-8 校验和计划发布 latching，不依赖硬件开关来唤醒订阅。
   - 依赖：步骤 2 的 JSON 契约，使用 std_msgs/stdlib 不添加 perception 反向依赖。
   - 命令：`colcon build --base-paths src --packages-select fruit_qr_planner`；`/usr/bin/python3 -m compileall -q src/fruit_qr_planner/fruit_qr_planner src/fruit_qr_planner/launch`；`ament_flake8 src/fruit_qr_planner/fruit_qr_planner/qr_node.py`。
   - 期望：节点和语法检查通过，既有 pyzbar/系统 OpenCV 依赖不变；步骤 7 验证窗外不发布、B 失败后 C 可扫描及输出标识。
   - 停止：B/C 跨窗口误采纳、帧时间未传播、UTF-8或图像边界校验被删除、节点生命周期无法接受 C-only 正式窗口。

5. **mission 以到位结果驱动窗口，删除静止观察等待。**
   - 文件/符号：`src/fruit_mission/fruit_mission/mission_context.py::{__init__,set_perception_mode,start_observation,stable_stop,wait_camera_target,wait_b_targets,wait_b_plan,wait_c_plan,_on_decision,_on_b_plan,_on_c_plan,_receive_plan,close}`；`actions.py::{observe_and_pick,pick_b_tree,_grasp_and_collect,scan_b_qr,scan_c_qr,_b_observation_failure_text,_pick_failure_text}`；`Logic.py::{run_a,run_b,run_c}`；`mission_node.py::run`；`config.py::validate_config`；`config/fruit_mission.yaml`；`camera_target_planner_client.py` 的 DETECT 术语注释。
   - 改动：用最少的开/关窗口公共方法替代模式发布/回声状态；开窗同时冻结区域、期望类型、时间及 UUID，清理本地缓存，不等专门 echo。`start_observation` 只安排 Look，不开放证据；actions 仅在导航已成功且 `finish_observation` 或 QR 预设成功后开窗。移除 `stable_stop` 及只为它存在的 odometry subscription/cache、持续时间/底盘速度/年龄/等待字段和对应配置校验；保留有其他调用方的 `stationary.joint_speed_rps` 及相关验证。删除模式超时与专用失败播报分支。
   - `wait_*` 和 `_receive_plan` 先校验当前 ID、用途、区域及图像时间再纳入缓存；保留原坐标/品种/QR 内容边界验证。注意 `_receive_plan` 当前重建业务计划会丢弃额外字段：必须在重建前检验元数据，不能不经校验先丢掉窗口字段。每次等待的 finally 关闭窗口，超时/异常/恢复/移动前和节点退出同样立即关闭本地采纳。
   - B 在第一抓之前把全部选中目标转换成固定请求，之后不再受新检测结果替换；维持已有第二抓和失败恢复策略。二维码失败继续固定兜底、保存与播报，不改变路线或顺序。区域结束/生命周期清理关闭证据，不停相机/推理。保留所有 planner 保护和已有较短超时。
   - 依赖：步骤 2、4 的生产者元数据和契约。
   - 命令：`colcon build --base-paths src --packages-up-to fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`；`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission`；针对上述实际改变 Python 文件运行 `ament_flake8 <逐个受影响文件>`；人工逐条 review A 左/右、B 先冻结、C 跳过推进、QR 兜底与 close/finally 路径。
   - 期望：无模式阻塞、无开窗前 stable_stop，未到位不采纳；无识别重试；原 STOP/取消/反馈校验/一次 Home 未变。行为交接在步骤 7 验证公开消息，不能声称已运行真实比赛。
   - 停止：删除波及抓取/恢复前停稳保护、导航失败仍开窗、缓存绕过 ID、B 第二抓重新换算、QR 失败不推进、发现业务调用需要扩大修改范围。

6. **更新调试与启动/监控调用方。**
   - 文件/符号：`src/fruit_mission/fruit_mission/camera_arm_target_client.py::{__init__,_on_mode,_set_perception_mode,set_off_for_cleanup,_wait_for_target,run_sequence,destroy_node,main,stack_is_absent}`；`stack_readiness_client.py::{__init__,wait_for_active_mode,_on_active_mode,_parser,main}`；`config/script_ros_clients.yaml`；`start_fruit_mission.sh::{wait_for_active_off,wait_for_odometry,节点清单与启动尾部}`；`test_camera_arm_target.sh::cleanup`；`scripts/robot_debug_monitor.sh::{RobotDebugMonitor,on_mode,log_camera_streams,on_timer,on_decision}`；`kill_fruit_processes.sh::target_tokens`。
   - 改动：调试客户端接入同一窗口契约/ID筛选，在现有 Look→机械臂静止→OPEN/HOLD 完成后开窗；删除 mode 状态、反复发布、cleanup mode 参数与 `--mode-off`。保留固定底盘人工前提及现有真实 motion 验证。
   - 删除 readiness 的 active-mode 子命令、订阅和相关参数；启动脚本删除 manager 必需节点/active OFF 等待，沿用已有节点/端点检查，确认窗口消费者端点而不引入每观察一次的新握手。保留导航所需 odometry/TF 就绪检查，只改错误日志中“无静止反馈不能识别”的过时表述。
   - 监控改为展示窗口 ID/用途/区域/有效状态，并在感知栈运行时始终期望 color/depth/RGBD 连续帧；窗口关闭不能把相机断流标成正常 IDLE。继续 metadata/header 轻量监视，禁止额外订阅原始 RGBD 增加 DDS 负荷。清理脚本删除旧模式 publisher token，必要时覆盖新文档的窗口 publisher；不改进程停止策略。抓取调试 shell 清理删模式请求，仍保留已有 STOP 与进程所有权清理。
   - 依赖：步骤 3、5；消息变更后消费方一起重建。
   - 命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission`；`ament_flake8 src/fruit_mission/fruit_mission/camera_arm_target_client.py src/fruit_mission/fruit_mission/stack_readiness_client.py`；`bash -n start_fruit_mission.sh test_camera_arm_target.sh scripts/robot_debug_monitor.sh kill_fruit_processes.sh`；在步骤 7 隔离域内检查更新的 readiness 命令与监控表现，不执行真实主流程脚本。
   - 期望：旧 mode CLI/必需节点/配置不再阻塞；台架流程不增加导航依赖；监控区分证据关闭与采集丢失。
   - 停止：调试脚本真实运动被自动运行、遗漏 STOP、结束了非自有进程、监控依然允许关窗断流正常化、shell/Python 检查失败。

7. **离线构建与最小黑盒检查。**
   - 文件/符号：已有 `src/fruit_perception/test/{test_detection_parser,test_image_processing,test_target_localization,test_detection_output}.cpp`；新忽略文件 `tmp/check_observation_windows.py` 和结果 `tmp/observation_windows_check.json`。脚本只使用公开 ROS 接口和合成/回放图像，不读源码作断言、不 monkey-patch 私有方法、不固化比赛序列。
   - 改动：脚本在隔离 ROS domain、独立 topic 前缀中启动真实 perception/QR 节点，明确关闭真实相机、串口、mission、planner 的启动；可用现有独立 perception executable 接受 synthetic RGBD，该跨进程传输仅限离线检查，不改变生产组件链路。脚本读取源配置并显式覆盖输入/输出话题，发布合法 RGBD/CameraInfo 和 QR 图像，运行上述端到端消息检查。每项有独立不超过 10 秒等待；记录消息 ID、实际图像时间和结论，清理只停止自有进程。
   - 覆盖：窗外仍推理并发布检测/按需标注、但无可抓取目标；开窗 reset 后只能积累窗后 RGB/Depth，先前在途帧不能填新 ID；关闭和连续新窗不会继承 ready/锁定；A/C 期望类型和 B 一次多目标锁定使用独立窗口；无窗口/旧帧不发布二维码；B 未成功仍能正式扫 C；旧锁存计划/旧 ID 不被当作当前数据； malformed JSON/非法域/时间/用途不打开有效窗口，图像尺寸/encoding/step/长度/UTF-8校验仍有效。
   - 同时加载现有 mission 配置及 fallback，检查原加载校验能通过；mission 到位时序、QR 失败后推进、B 请求冻结与 STOP 恢复以步骤 5 的代码 review 保证，本脚本不冒称完成运动状态机运行验收。
   - 输入：优先使用已有合法回放/目标图片；纯色 synthetic RGBD 可以验证推理持续与空目标窗口，不能替代有目标 evidence reset。QRCode 图像使用本机既有工具/素材生成，无新依赖。执行时若找不到可用于成熟目标与两目标场景的可信输入，标为对应验证阻塞，不能把空帧通过充作完整证据。
   - 依赖：步骤 2～6全部完成。
   - 命令：`colcon build --base-paths src --packages-up-to fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON`；重新 source 后 `colcon test --base-paths src --packages-select fruit_perception --return-code-on-test-failure`、`colcon test-result --all --verbose`；`/usr/bin/python3 tmp/check_observation_windows.py --report tmp/observation_windows_check.json`；`ldd install/fruit_perception/lib/libfruit_perception_component.so`。
   - 期望：已有四组感知算法测试通过，新消息生产/消费构建一致，黑盒覆盖项都有实际可复现消息证据，无运动发布/调用、无真实驱动打开。
   - 停止：干净 runtime/GPU 不可用、合成/回放素材不足、JSON 跨语言格式不一致、任何旧结果能被新窗口采纳、进程清理或隔离失效。准确报告环境/覆盖限制；不得 CPU fallback、延时掩盖、下载新依赖或用私有状态 mock 宣称通过。

8. **更新持久文档与操作示例。**
   - 文件/符号：`src/fruit_perception/README.md`、`docs/TWO_STAGE_RUNTIME.md`（此 docs 相对 perception 包）、`src/fruit_qr_planner/README.md`、`src/fruit_mission/README.md`、`src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`、根 `README.md`、`docs/{PERCEPTION_MODE_COMMANDS,CAMERA_TUNING,PROJECT_ARCHITECTURE,VOICE_FLOW,README}.md`；`src/fruit_perception/MODEL_UPDATE_RC4.md` 顶部“当前模式”提醒。
   - 改动：保留 `PERCEPTION_MODE_COMMANDS.md` 路径以维护链接，但改成相机常开、持续推理及窗口操作参考。给出严格 JSON 开/关窗口、区域选择、窗口 ID结果及纯诊断命令；删除启停模式示例和回声等待。文档写明正式任务导航/预设完成即开窗、用户物理静止保证、无持续监测，以及恢复前实测停稳仍存在。同步 QR 窗口/confirm_frames=1/10 秒现值，模型驻留与检测/证据差别，窗外 raw detections 不授权抓取，消息重建/重启和旧录包兼容限制。
   - `CAMERA_TUNING.md` 仅补充相机生命周期常开与人工曝光调参的边界，不改 GUI 行为。`PROJECT_ARCHITECTURE.md` 受影响模式/短窗口/状态机段以已读实现纠正旧 35/50 帧、三帧扫码、OFF 就绪等描述；不借机重构其他架构章节。`VOICE_FLOW.md` 删除不再可能的模式失败及结束关相机叙述，不生成音频、不改现有音色和汇总逻辑。历史 MODEL_UPDATE 只更新顶部指向当前操作文档的提醒，归档正文和旧测量保留。记录脚本 `--all` 会自动录新窗口话题，视觉排除规则保持现有实现，纠正文档中不一致说明。
   - 依赖：步骤 7提供真实验证结果，文档不能先称测试通过。
   - 命令：`git diff --check -- README.md docs src/fruit_perception/README.md src/fruit_perception/docs src/fruit_perception/MODEL_UPDATE_RC4.md src/fruit_qr_planner/README.md src/fruit_mission/README.md src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`；按文件人工核对链接、参数/消息字段、命令与控制风险，无源码字符串单测。
   - 期望：所有在用指南一致，明确区分本次离线证据与历史实测；原历史记录未被当成当前承诺。
   - 停止：文档与实际接口不一致、示例会意外控制运动、声称未经验证的性能/实机闭环、覆盖用户无关改动。

9. **复核、回交和保存验收证据。**
   - 文件/符号：仅上述任务文件、plan/review 文档及 tmp 检查产物；原活动分支。
   - 改动：对比步骤 1基线，确认只交回本任务增量；将隔离成果合并/应用回原活动分支，保留全部原脏改动。在最终工作区重建接口与直接消费者，确保正式 source 配置路径使用交付内容。删已合并短期 worktree/分支；如阻塞未合并，明确报告，不能留唯一交付在 worktree。
   - 依赖：步骤 7、8成功，review 后按扩展流程处理。
   - 命令：`git diff --check -- <本任务受影响路径>`、`git status --short`、`git diff -- <本任务受影响路径>`；最终根工作区执行步骤 5构建与步骤 7检查命令，重新 source runtime；比对基线确认无 unrelated 变化。
   - 期望：结果在用户活动工作区可检查、可构建，可重跑检查及报告保留于 tmp；清楚列出已验证/未验证事项。
   - 停止：原分支发生交叉修改、无法证明用户改动保留、合并冲突或最终根工作区失败。不得自动丢弃用户差异、提交凭据或修改生成物。

## 验收

- 在用源码、launch、配置、脚本不再依赖 CameraModeManager、perception_mode 状态机、active_mode 回声或业务流启停 color/depth；历史档案中允许保留明确标注的历史记录。四个删除文件及其构建目标均消失。
- 相机源码配置持续启用 color、depth、align_depth、enable_sync、enable_rgbd；camera 与 C++ perception 同容器/intra-process，OpenCV ABI 保持系统 4.5.4。此项离线验收是配置/构建与组件结构核对，不等价于硬件连续出流实测。
- 无证据窗口时，真实节点仍能对离线 RGBD完成检测/分类和诊断输出；没有目标 evidence 及可抓取目标。持续推理不以每帧必处理或 30 FPS 为要求，保留 busy 丢帧。
- 正式任务先等导航和预设执行成功才开窗；源码中没有 `stable_stop` 观察等待及其无用配置。左右窗口独立。手动工具保留固定底盘前提和原机械臂静止/OPEN/HOLD步骤。
- 开/关、新窗口、旧在途推理、窗前 RGB/Depth 帧、旧 ID 和区域/用途不匹配的数据有可重跑的隔离检查证据。窗口时间使用图像 ROS 时间，ID只用于归属，不用收到消息的时间伪造采集时间。
- B 一次观察的现有多目标数学不变，两个基座请求在首次动作前冻结；第二抓不重观察。A/C 期望类型和窗口清理路径通过代码 review，无源扫描/私有 mock 测试。
- QR 仅正式窗口生成/采纳计划，B 缺失仍可扫 C，不提前缓存；真实输出附带窗口 ID与图像时间。mission 保留固定兜底验证、保存、播报与局部失败继续的原顺序。
- STOP、导航取消、未知运动恢复前新鲜停稳反馈、EMERGENCY锁和 planner 输入/执行结果保护通过代码 review，未因删除观察停稳等待被删。
- 构建、Python语法、针对性风格、已有四组感知算法测试、ABI检查及 tmp黑盒检查通过；只对被修改范围做风格检查。
- 当前未提交用户修改完整保留，交付在原活动工作区，报告命令/退出状态/检查覆盖/环境限制。

当前证据：只读核实完成；Q1～Q12 的需求决策与最终理解确认完成。所有实现、构建、测试、黑盒检查、最终回交均未执行，等待扩展的人类审批。实机时延、热负载、识别准确率、驱动稳定性和实际采摘留待现场另行验证，不作为本次通过条件。

## 风险

- 用户明确移除观察前 `stable_stop` 并拒绝新增窗口内运动监测。导航/预设成功不是实测持续静止证明；本方案依赖用户提供的物理保证，尤其多帧观察至坐标冻结必须同姿态。若现场保证不成立，本次离线通过不能证明目标坐标准确；不能在本任务中擅自恢复静止监测或加重试。
- 持续 GPU 推理可能与 Point-LIO、导航、串口共享 CPU/功耗预算；已有性能表为旧测量且 ROI负载不完整。本次不设自动降帧/关推理策略，不承诺发热/功耗变化；实际影响未知。
- JSON 命令是跨进程信任边界，字段/类型/范围必须验证；C++ OpenCV解析与Python编码须黑盒确认。不能通过宽松类型转换接受错时间/错用途。时间跳变或回放不一致时不得采纳无法匹配的帧；检查环境需统一 ROS clock。
- 异步推理可能跨越窗口开启、关闭或区域变化。须冻结提交上下文并结合 generation + ID + 图像时间；仅用新结果 header过滤不能消除 B旧锁定。关闭后已在 DDS队列中的旧消息仍可能到达，客户端 ID采纳条件是最终边界。
- VOLATILE 窗口命令不为晚加入节点自动重播，避免旧开启状态复活。节点不可用或重启时本次观察/扫码可能按现有超时跳过/兜底；不新增命令重试和 mode echo，也不以延长等待掩盖端点发现。
- `FruitDecision` 增字段改变 ROS类型契约，旧进程/旧录包不可假定兼容；所有在用生产/消费方统一重建、重新 source并重启。仅 reload配置不足以更新代码或消息。
- 现有 QRPython进程仍跨进程接收 color；这是保留的既有布局，本次不扩展新的跨进程生产图像路径。持续推理增加的系统负荷与 QR解码并行影响未经现场测量。
- 离线空图只能证明节点运行，不能证明成熟目标证据隔离、两目标冻结或现场准确率；缺可信输入必须明确阻塞相关验收。代码review不是运动状态机实机验证。
- 多包改动与大量既有脏修改重叠，worktree必须建立在当前有效内容上；仅从HEAD开发会丢失当前planner/mission更新。配置绝对源路径需在离线验证时显式指向worktree内容，回交后再检查正式源路径。
- 真实Look/QR/Harvest/Home预设、串口跨消息轨迹装载顺序、STOP优先级、夹爪硬件保护已有未验收项。本次不填假姿态、不靠sleep保证传输、不运行硬件控制、不声称比赛整机可运行。
