## 目标

让人看终端时能快速看到正在做什么、目标在哪里、动作是否完成、哪里发生异常；详细诊断交给文件和 Agent 阅读。

用户已完成逐轮 grill-me 并最终确认：
- 普通终端仅保留关键阶段、目标坐标、动作结果、告警和错误。
- 显式 `--debug` 额外展示当前逐包开关已开启的串口 TX/RX 完整解码；不显示 raw，也不把当前关闭的高频包重新展示到终端。
- 相机逐帧检测、正常相机帧率、常态健康巡检、完整坐标变换链仅落盘。
- 新的文本日志记录当前协议全部可解码的收发包，包括终端关闭的包；不记录 raw 字节转储。
- 日志统一到 `/home/rc4/fruit_ws/log/runtime/<时间_入口_PID>/`，保留最近 20 次已结束运行，保护仍在写入或被复用的目录。
- 在根 `AGENTS.md` 明确告诉 Agent 去哪里、按什么文件和关联 ID 查日志。

当前为 planning；只做了文件读取和需求确认，没有运行程序、修改业务文件或测试。此文档提交完成不代表执行获授权，必须等待扩展的人类审批。

## 范围

### 纳入

统一以下七个入口及它们必需的共享日志路径：
1. `test_camera_arm_target.sh`
2. `start_fruit_perception.sh`
3. `start_navigation.sh`
4. `start_fruit_mission.sh`
5. `start_tele.sh`
6. `waypoints_maker.sh`
7. `record_rosbag.sh`

已确认的关联实现：
- `scripts/robot_debug_monitor.sh`：监視器输出分类、常态数据只落盘、相机流异常/恢复事件。
- `src/fruit_mission/fruit_mission/camera_arm_target_client.py` 与 `camera_target_planner_client.py`：日志级别与摘要；不改运动和筛选逻辑。
- 相关 launch 的日志级别传递：`fruit_perception.launch.py`、`fruit_mission.launch.py`、`fruit_nav.launch.py`、`joy_cmd_vel_serial.launch.py`、`joy_conventor.launch.py`、`waypoint_recorder.launch.py`。桥和 planner launch 已有 `log_level`，优先直接复用。
- 独立仓库 `src/auto_serial_bridge/`：`SerialController` 收发诊断、`scripts/codegen.py` 的描述/接收日志生成、配置注释、相关废弃 raw helper 断言和包 README。
- 根 `AGENTS.md`、`README.md`、mission README，以及现有导航、航点、架构和通信文档中的相关日志说明。
- 至多两个小型共享源文件：拟新增 `scripts/fruit_runtime_logging.bash`、`scripts/runtime_log_filter.py`，避免七个脚本重复实现落盘和筛选。
- 用户同意在忽略的 `tmp/` 中保留最小无运动离线检查，不新增包级测试套件。

### 不纳入

- 不统一所有独立 `ros2 launch`、systemd 服务、交互控制台、相机 GUI 工具的终端/归档行为；桥重建后不输出 raw 的修正自然生效。
- 不改变 A/B/C 编排、模型/观察窗口协议、目标筛选、IK、标定、预设、帧和坐标语义、控制超时、安全门控、STOP/ESTOP、重传或握手策略。
- 不改变串口消息字段、协议哈希输入、MCU 布局、ROS 接口，也不补充当前协议未携带的数据。
- 不改变 rosbag 格式、话题选择及现有五份 bag 保留规则。
- 不修改第三方驱动代码，不把所有第三方驱动强制升为 Debug；不安装依赖。
- 不增设容量监控、配额、压缩、日志服务或网页浏览器；用户明确认为无需容量管理。
- 不删除历史 raw 证据目录，不自动搬迁旧 `tmp/` 或 `~/.ros/log/` 日志。
- 不用本任务清理工作区现有大量未提交改动或既有白盒测试。

## 决策

### 1. 终端和文件分流，不靠降低整体日志级别丢证据

普通运行也采集相关自有节点的 Debug 数据，终端独立筛选。保留 ROS 原生文件日志，并记录各子进程 stdout/stderr，覆盖 launch、非 ROS 输出和异常栈。相关节点通过已有/最小新增的 scoped `log_level` 参数设置，保持第三方与 rcl/rmw 既有级别。

终端规则：
- 保留操作开始/就绪、连接/断连、任务/导航重要阶段、锁定目标和最终基座请求坐标、执行结果、停止、失败原因。
- 所有 WARN/ERROR/FATAL、launch 异常和进程异常退出必须可见，不能以 INFO 白名单吞掉故障或 traceback 的后续行。
- launch 日常 PID、组件加载成功、冗长启动参数、重复配置说明只落盘。
- 正常相机流和健康巡检不周期性输出；相机流首次异常和恢复各显示一次，继续保存既有周期数据。不另加抖动策略、重试或改变检测门限。
- 非 ROS 驱动常态 stdout 留文件；未知输出不能靠盲目关键词删除，错误输出/未识别记录采取保守保留，发现遗漏先校正分类再交付。
- 详细日志保留时间、级别、节点/子进程身份；目标/运动保留 `window_id`、`trajectory_id`，错误不截掉关键上下文。

### 2. 串口：文件完整、终端继续尊重逐包开关

源码事实：
- `test_camera_arm_target.sh:main` 和 `start_fruit_mission.sh` 当前强制传 `serial_debug_raw_frame:=true`，并全量 tee 到终端。
- `SerialController::log_mcu_tx` 生成 TX RAW 和 TX DECODED；`get_parameters` 读取 `debug_raw_frame`。当前读取的桥版本没有独立 RX RAW 输出，但部分握手/心跳告警带 payload 十六进制。
- 接收业务日志由 `codegen.py::_write_guarded_rx_debug_log` / `generate_ros_bindings` 生成；`describe_packet` 目前只覆盖 TX/both 消息。
- 当前 `debug_log_mode=on` 为 TrajectoryHeader、TrajectoryPoint、ArmControlMode；ChassisVelocity、ArmStatus 及内置 Ack/Heartbeat/Handshake 为 off。
- `Packet` 只有 ID 和 payload，不保存收到的 checksum；不可为了日志伪造接收 checksum 或改传输结构。

实施决策：
- 去掉 raw 帧与告警中的 payload 字节串；保留包 ID、长度、期望/实际值、计数、状态、hash 和 checksum 等有意义的解码元数据。
- `debug_raw_frame` 参数保留为废弃兼容参数，无论 true/false 都不能开启 raw；更新其注释与初始化摘要，不新增替代 raw 功能。
- `serial_debug_raw_frame:=true` 保留为旧入口兼容的“解码 Debug 展示”选择，不再具有 raw 含义。新脚本统一 `--debug`，不继续依赖强制 raw 参数采集日志。
- 复用现有解码生成，不手写一份业务协议。将 off 包的诊断写入 `serial_controller.detail` 子 logger，on 包仍用 `serial_controller`；七个入口的输出筛选只在显式 Debug 时展示主 logger 的解码记录，detail logger 始终只落盘。
- 所有 TX、业务 RX、内置系统 RX 的已解析字段均能落盘；保持一次实际收发对应一份解码记录，不重复生成业务 RX。CRC/长度校验被拒绝的字节不是可解码包，不记录它们的 raw。
- 保持现有逐包开关值，不将 20Hz ArmStatus、底盘速度、ACK/心跳搬到 Debug 终端。普通终端同样不输出逐点解码。
- 完整字段指源码 YAML 现有字段；不借日志任务补字段、逆映射或解释未知固件错误码。

### 3. 目录和保留

- 每个最外层入口创建唯一运行目录；子入口通过环境继承并校验目录归属，共用目录，不重复创建运行、不重复捕获同一输出。
- 每个子进程一个有稳定名称的 `.log`，脚本生命周期摘要写 `orchestrator.log`；原生 ROS 日志置于该运行目录的 `ros/` 子树，必要时按子进程分子目录避免多个 launch 的索引相互覆盖。
- 启动只提示一次日志路径；保留可定位的最新运行索引。索引原子更新，不能让它指向已删除目录。
- 自动清理只处理本工具创建并标识的 `log/runtime/` 一级运行目录，保留最近 20 次已结束运行，活动目录不计入这 20 次；启动和结束时执行清理。
- 用操作系统文件锁保护活动写入和清理并发，避免仅凭 PID 判断、PID 复用或崩溃留下标记导致误删。父入口与仍运行的记录进程须保持活动锁；目录名和真实路径需严格校验，不递归跟随 symlink，不触及 colcon 日志、bag 或用户其他目录。
- 不自动删除正在运行的旧桥日志。外部桥未受本工具管理时仅指向其原启动终端或 `~/.ros/log/`。

### 4. 参数、兼容和安全

- 七个入口统一接受 `--debug`；mission 的 `--debug` 仍保留现有启动 rosbag 的含义，同时增加允许的串口解码展示。相机脚本的 `--debug` 不再把逐帧相机详情展示到终端。
- 复用已有桥不改变它的级别、串口、配置、stdout、进程归属或清理责任；提示一次“已有桥的日志在原运行目录，本次 Debug 不接管已有桥”。拿不到其关闭的诊断时明确说明，不承诺补出历史数据。
- `stack_readiness_client bridge-start-required` 的 stdout 是机器消费的 true/false，不能混入摘要、日志路径或过滤器输出；此类检查的 stdout 与 stderr 分开处理，保留退出码。
- 不改变原有进程组和 Ctrl-C 处理责任；日志包装必须让 `setsid` PID、wait、pipefail、信号、STOP 和退出码仍对应原业务生命周期。不能因为输出进程先退出而提前结束 mission，也不能把 tee/filter 成功当成业务成功。
- 初始化目录/文件失败在启动业务前明确报错并停止。运行期间基础写入失败明确提示并继续原有控制/STOP 流程，不静默成功，不新增磁盘监控或容量策略；详细日志不完整必须被说明。

### 5. 已有改动与验证边界

父仓库 baseline/status 显示上述根脚本、监视器、相机客户端、感知 launch、joy launch、AGENTS.md/README 等多数已经存在用户改动。桥目录为父仓库忽略的独立 Git 仓库，父 diff 不覆盖它。实现必须以本次已读取的当前工作区版本为基线，不能从 HEAD 恢复覆盖用户功能。

只自动进行语法、构建、既有协议测试和离线日志行为检查。真实机器人启动、任务、抓取、遥控、运动服务和交互 arm console 均属于人工验收，不由 Agent 运行。新增检查不扫描业务源码、不断言源码字符串，不 deep mock ROS 私有状态。

## 执行步骤

以下步骤均待人类批准。构建与检查终端统一先执行：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

1. **冻结基线并隔离编辑。**
   - 文件/位置：父仓库本次触及文件；独立 `src/auto_serial_bridge/` 仓库；隔离目录 `.worktrees/<本任务>/`。
   - 内容：检查父/桥各自 branch、status、diff；核对运行中的进程但不停止硬件；记录当前脏改动与文件快照。按项目 worktree 要求隔离这次多入口工作，将本任务依赖的当前版本及其必需的既有接口改动作为隔离基线，保留新增 ArmTelemetry 等用户未跟踪文件。不得把隔离 HEAD 的旧版本当成当前行为。
   - 依赖：无，先于任何编辑。桥也独立隔离，不将它变成父仓库子模块。
   - 检查命令：`git status --short`、`git -C src/auto_serial_bridge status --short`、`git check-ignore .worktrees/<本任务>`，并分别记录基线 diff。只登记，不 reset/stash 用户工作。
   - 期望：可明确区分任务增量与已有改动；worktree 与父/桥对应关系清楚。
   - 停止条件：目录不受忽略、基线变化不能隔离、并发编辑冲突、活动分支不符合部署规则。先解决隔离，不能覆写或绕过。

2. **实现最小共享落盘和终端分流。**
   - 拟新增文件/符号：`scripts/fruit_runtime_logging.bash` 的 `fruit_logging_init`、`fruit_run_logged` 与清理函数；`scripts/runtime_log_filter.py` 的流读取/日志写入/终端选择入口。命名在实现时保持这两个小文件，不加包、插件、配置框架或 supervisor 系统。
   - 内容：继承/创建运行目录、设置 ROS_LOG_DIR、活动锁、最近运行索引、20 份已结束运行清理。统一捕获 stdout/stderr 后先写文件再筛选终端，记录完整异常栈，保留原始业务状态码。文件采用正常缓冲/及时刷新，不逐帧 fsync。只为终端移除颜色/噪声，不删文件诊断。
   - 依赖：步骤 1；后续七入口均复用这条路径。优先复用 ROS 原生日志、Bash pipefail/进程组和系统文件锁，不另写日志服务。
   - 检查命令：`bash -n scripts/fruit_runtime_logging.bash`；`/usr/bin/python3 -m py_compile scripts/runtime_log_filter.py`；`/usr/bin/python3 tmp/check_runtime_logging.py`（本步创建并保留的最小离线执行检查）。
   - 离线检查内容：实际喂入正常 INFO、DEBUG、detail logger、WARN/ERROR、ANSI、launch 前缀、多行异常和缺末尾换行记录；检查文件完整、终端选择正确；使用无 ROS 的小子进程验证退出 7 仍返回 7、SIGINT/SIGTERM 到达子进程且无残留。用 `tmp/` 假运行根目录检查 21+ 个目录、活动锁、并发清理、symlink/外部目录保护和索引更新。不是源码扫描测试。
   - 期望：以上行为检查全部通过；清理仅删除受管理的已结束目录。
   - 停止条件：丢异常、吞退出码、信号失效、活动目录误删、目录越界、写入故障静默。此共享路径不合格就不接硬件入口。

3. **修正桥本身，取消 raw 并补足文件解码。**
   - 具体文件/符号：`src/auto_serial_bridge/src/serial_controller.cpp` 的 `get_parameters`、`log_mcu_tx`、`handle_packet`、`process_handshake`、心跳诊断；`include/auto_serial_bridge/serial_controller.hpp` 的 raw 成员/helper/声明；`scripts/codegen.py` 的 `_write_guarded_rx_debug_log`、`generate_ros_bindings` 中 `describe_packet` 生成；`config/protocol.yaml` 中兼容参数的注释。
   - 内容：删除 raw 打印和 payload hex 转储；保留兼容参数声明但不让它开启 raw；给 TX 和业务 RX 增加文件完整性路径，将原 off 包记入 detail 子 logger；内置 RX 复用生成描述。扩展描述生成覆盖 RX，保留 on 包现有明确字段格式。不要改变消息校验、投递、队列、ACK、回包、ready、重连或协议 hash。
   - `test/test_serial_controller_helpers.cpp`：仅移除对应已删 raw/hex helper 的废弃断言，其他握手/接收检查保持；不添加新的私有状态测试。既有 `test_mcu_tx_logging_codegen.py` / `test_codegen_checksum.py` 是文本扫描测试，不扩写它们来冻结新的日志实现。若必须调整其被本任务废弃的日志断言，先报告具体冲突，不顺带重构整个测试文件。
   - 依赖：步骤 1；输出分类遵守步骤 2 的 logger 规则。
   - 检查命令：`/usr/bin/python3 -m py_compile src/auto_serial_bridge/scripts/codegen.py`；集成到当前源后 `colcon build --base-paths src --packages-up-to auto_serial_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON`；重新 source runtime 后 `colcon test --base-paths src --packages-select auto_serial_bridge --return-code-on-test-failure --ctest-args -R '^(test_packet_handler|test_checksum_algorithms|test_reliable_roundtrip|test_reliable_sender|test_serial_controller_helpers)$'`；`colcon test-result --all --verbose`，区分历史结果。
   - 期望：生成绑定和桥编译成功，选定既有协议检查通过；协议哈希、布局和 MCU 产物的线协议部分不因日志变化改变。
   - 停止条件：协议行为/哈希变化、字段解码错误、编译或选定测试失败、完整日志路径绕过校验。不手改 build/install 生成物，不拿延长等待补救。

4. **调整人读摘要的生产端。**
   - 文件/符号：`scripts/robot_debug_monitor.sh` 的 Python `log`、`on_rosout`、`on_decision`、`on_types`、`on_camera_detections`、`on_target_point`、`log_camera_streams`、`on_timer`；`camera_arm_target_client.py` 的 `_debug_decision`、`_on_decision`、窗口日志与 `run_sequence`；`camera_target_planner_client.py` 的初始化说明、`target_request`、`start_preset`、`finish_motion`。
   - 内容：监视器日志明确级别，周期流/巡检/逐帧检测转为详细记录；相机流只将现有状态的异常/恢复事件显示到终端。避免监视器将原进程已显示的 rosout 告警再重复播放，原始告警仍由统一捕获路径保存/显示。相机逐帧详情以 DEBUG 落盘，脚本普通模式也采集；配置路径、完整窗口 UUID、坐标链、共享预设下发说明降到 DEBUG。保留模型/作业侧、观察阶段、选定目标、请求 XYZ/pitch、动作结果摘要；失败的 planner response 必须仍有可见失败原因/trajectory_id，不能因降级结果日志隐藏失败。
   - 依赖：步骤 2、3 的级别规则；只改输出，不改观察、静止、转坐标与动作调用。
   - 检查命令：`bash -n scripts/robot_debug_monitor.sh`；`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/camera_arm_target_client.py src/fruit_mission/fruit_mission/camera_target_planner_client.py`；`ament_flake8` 仅传这两个改动 Python 文件；离线检查对 monitor heredoc 做语法编译而非执行节点。
   - 期望：摘要不重复，详细数据存在；所有安全条件/调用顺序/等待原封不动。
   - 停止条件：改动触及实际筛选或安全逻辑，失败摘要丢失，无法证明监视器仍记录异常。遇到现有非本任务 lint 问题只记录，不全文件重排。

5. **接入相机、感知和 mission 三个入口。**
   - 文件/符号：`test_camera_arm_target.sh` 的 `main`、`show_failure_logs`、`keep_running`、`cleanup`、usage；`start_fruit_perception.sh` 的监视器启动/launch/cleanup；`start_fruit_mission.sh` 的 `log`、`run_readiness_client`、`start_process`、`start_debug_process`、`log_recent_child_output`、运行目录与参数解析；`src/fruit_perception/launch/fruit_perception.launch.py` 的串口/二维码节点参数和日志级别；`src/fruit_mission/launch/fruit_mission.launch.py::_validated_node` 与参数声明。
   - 内容：使用共享 session 和每子进程落盘，替换全量 tee 终端；不再每次覆盖固定 camera-arm 日志。最外层相机入口创建一次 session，感知子入口继承，不二次筛选造成丢失。planner 用已有 `log_level:=debug` 采集；感知自有节点、QR、mission scoped Debug 落盘，第三方保持原级别。加入最小日志参数传递，旧 raw launch 参数兼容解码展示。失败只输出原因和日志路径，不 tail 40/80 行噪声到终端，完整输出依然保留。mission 启动长命令/branch/commit/PID 放文件，核心就绪与任务推进留终端。
   - 特别保护：readiness 布尔 stdout 及状态码；mission 不因非核心进程退出被提前停止；相机任务完成仍保持感知驻留；相机 Ctrl-C 仍请求 planner STOP 后清理本次进程；复用桥不在退出时被杀。
   - 依赖：步骤 2–4。
   - 检查命令：`bash -n test_camera_arm_target.sh start_fruit_perception.sh start_fruit_mission.sh`；`/usr/bin/python3 -m compileall -q src/fruit_perception/launch src/fruit_mission/launch`；`/usr/bin/python3 tmp/check_runtime_logging.py` 中增加实际共享 helper 的父子目录继承、布尔 stdout、普通/Debug 对照检查。
   - 期望：一个顶层运行一份目录，文件完整，普通终端不刷轨迹点/相机数据，Debug 仅允许 on 包解码；原有 mission --debug 录包条件保持。
   - 停止条件：桥重复启动、readiness 值被污染、原 trap/setsid/wait 改变、父子重复日志或日志丢失；禁止用真实相机抓取脚本作自动验证。

6. **接入导航、遥控、录点和录包四个入口。**
   - 文件/符号：`start_navigation.sh` / `waypoints_maker.sh` 的 usage、参数解析、`start_process`、`start_debug_monitor`、`stop_children`；`start_tele.sh` 的 exec/启动流程；`record_rosbag.sh` 的参数、monitor、cleanup 与 recorder 启动。launch：`src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py` 的 axis/joy 日志参数；`src/joy_linux/launch/joy_cmd_vel_serial.launch.py` 的 include 参数、`joy_conventor.launch.py` 的节点级别；`src/fruit_waypoint_recorder/launch/waypoint_recorder.launch.py` 的日志级别。
   - 内容：统一 --debug 和共享目录，桥/自有节点 Debug 文件采集，保留常态终端摘要；录点保存/撤回等人操作结果继续显示。teleop 包装不得改变信号向 joy/bridge 的传递或退出状态；record_rosbag 作为 mission 子入口不新建 session、不重复启动 monitor，不改 MCAP/话题与 bag 清理逻辑。
   - `fruit_hardware_bridge.launch.py` / `fruit_arm_planner.launch.py` 现有 log_level 可直接传，不为此改控制实现。检查 `kill_fruit_processes.sh` 已有命令匹配与新日志辅助进程兼容；新记录进程在 EOF 正常退出，不凭空扩大 kill 匹配范围。
   - 依赖：步骤 2，桥步骤 3，以及步骤 5 的目录继承约定。
   - 检查命令：`bash -n start_navigation.sh start_tele.sh waypoints_maker.sh record_rosbag.sh`；`/usr/bin/python3 -m compileall -q src/navigation/fruit_nav_bringup/launch src/joy_linux/launch src/fruit_waypoint_recorder/launch`；离线检查 `tmp/check_runtime_logging.py` 验证前台/后台包装的正常退出、异常退出和信号。
   - 期望：四入口仅改日志/Debug 参数，GUI 环境、等待值、原进程归属、录包与数据保存行为不变。
   - 停止条件：遥控停止路径或记录收尾被影响、硬件/导航被自动运行、未知 launch 参数、bag 策略变化。不能为了本任务修复既有长等待或其他导航行为。

7. **同步文档和 Agent 查找指引。**
   - 文件：`AGENTS.md`、根 `README.md`、`src/fruit_mission/README.md`、`src/auto_serial_bridge/README.md`、`docs/PROJECT_ARCHITECTURE.md`、`docs/ODOMETRY_NAVIGATION.md`、`docs/WAYPOINT_RECORDING.md`、`docs/LOWER_CONTROLLER_COMMANDS.md`、`src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md`。
   - 内容：以根 README 为日志使用说明入口，其他受影响文档简短链接/说明，避免复制整套规范。AGENTS 指定优先使用终端打印目录，其次最新运行索引，再按入口/时间寻找；先看 orchestrator 和失败子进程，按 window_id/trajectory_id 联查，再读 ros/；说明 20 次保留、活动目录保护、旧日志与复用桥归属、raw 不再记录、当前 YAML 可解码字段边界、服务发布/串口排队日志不等于执行完成。
   - 依赖：步骤 2–6 已确定实际路径、文件名和参数，文档必须与实现一致，不抄旧 --debug 逐帧说明。
   - 检查命令：`git diff --check -- <本任务实际改动文件列表>`；桥仓库 `git -C src/auto_serial_bridge diff --check`。人工核对七入口 help 与示例，不运行硬件命令。
   - 期望：Agent 能按目录找到文件并理解证据限制；用户无需看完整终端刷屏。
   - 停止条件：文档声称未经验证的实机成功、使用 install 配置路径、掩盖字段/复用限制或覆盖用户其他文档改动。

8. **集成任务增量，构建和无运动验收。**
   - 文件：只集成上述任务 diff；父仓库/桥仓库分别记录。隔离编辑先完成语法和离线检查，再把相对于冻结当前基线的任务增量合入用户活动工作区，禁止整文件覆盖；所有 ROS 构建只从当前 `/home/rc4/fruit_ws/src` 发现包，不遍历 `.worktrees/`。
   - 依赖：步骤 1–7；应用前重查当前文件是否发生并发变化，有变化先三方核对。
   - 构建命令：`colcon build --base-paths src --packages-up-to auto_serial_bridge fruit_mission fruit_perception fruit_qr_planner fruit_hardware_bridge fruit_nav_bringup joy_linux fruit_waypoint_recorder --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`；重新 source runtime；随后按步骤 3 单独开启桥测试构建并运行选定现有 C++ 协议检查。
   - ABI 检查（若构建重链感知）：`ldd install/fruit_perception/lib/libfruit_perception_component.so`，不允许 `/usr/local/lib/libopencv*`。
   - 行为检查：`/usr/bin/python3 tmp/check_runtime_logging.py`；`git diff --check -- <任务文件>` 与独立桥 diff 检查；审阅任务差分确保没有运动/时间/筛选/接口改动。禁止启动七个真实业务脚本作自动 smoke test。
   - 期望：所有受影响包编译成功；离线行为检查、选定协议测试、ABI 和任务 diff 检查有实际证据。临时数据/编译样例留忽略的 tmp，不提交运行 log/build/install。
   - 停止条件：编译/协议/信号/删除保护失败、已有改动被覆盖、基线与任务结果难以区分、ABI 混用。既有无关错误明确报告，不擅自修复或称全通过。

9. **形成 review 与人工验收交接。**
   - 文件：本任务目录的 review/执行记录，父仓库与独立桥的变更记录。
   - 内容：记录实际改动清单、完整检查命令/返回码、限制、未完成的人工步骤；明确报告桥变更不会被父仓库 commit 自动捕获。验证并集成后清理短期隔离分支/worktree，不留仅在 worktree 的成果，不提交或删除无关用户改动。
   - 依赖：步骤 8。
   - 人工命令示例（非 Agent 自动执行）：`./test_camera_arm_target.sh --model A --left` 与同命令加 `--debug`；其余入口按原用法比较普通/Debug。需要实机运动的入口必须由操作者确认安全后执行。
   - 期望：普通终端符合下文验收，文件可定位，Debug 解码符合逐包开关，Ctrl-C/复用/驻留保持原状；未执行实机验收须明确标为未完成。
   - 停止条件：实际终端仍高频刷屏、详细文件缺失、错误被隐藏、日志包装影响停止或数据保存。不得以离线检查代替实机安全验收结论。

## 验收

### 普通终端

- 启动时一次提示运行日志路径。
- 相机—机械臂测试可看到模型/侧、Home/Look、观察开始、一次选定目标及基座请求、抓取/放果/归位结果；不重复显示共享层的配置、变换链、每个驻留点和重复完成提示。
- 不出现 `[MCU TX RAW]`、RX raw/payload 字节转储、逐点串口解码、正常相机流 Hz/age 周期行、逐帧相机 detections 或常态 HEALTH。
- 告警、错误、STOP/急停状态、超时、launch 异常和进程失败仍可见；不能只有“failed”而没有原因/定位路径。
- 完成后感知驻留期间，状态正常时不每两秒刷屏；相机流异常与恢复可见。

### 显式 Debug

- 当前 on 的 TrajectoryHeader、TrajectoryPoint、ArmControlMode 逐包解码可显示，不采样删点；字段清楚且保留 ID/时间/关节/夹爪/ack_seq 等现有信息。
- 当前 off 的 ArmStatus、ChassisVelocity、Ack/Heartbeat/Handshake 不因此显示到终端；相机高频详情仍只进文件。
- mission --debug 保持原有录包副作用，旧 raw launch 参数不再开启 raw。

### 文件、保留与查找

- 普通运行也有自有节点详细日志、完整坐标变换链，以及所有当前协议可解码 TX/RX 的文件记录；off 不再意味着文件完全缺证据。
- 子进程 stdout/stderr 和原生 ROS 文件都能在该运行目录找到；没有 raw 转储；有可关联的时间、window_id、trajectory_id。
- 父子入口共用一个目录，互相独立启动仍创建不同目录；现有桥保留原日志归属。
- 自动清理仅删除最旧的受管理已结束目录，保留最新 20 次已结束运行及全部活动运行；colcon、rosbag、外部目录与 symlink 目标不受影响。
- AGENTS 指引能找到发生故障的运行，索引不指向被删除目录；落盘失败有明确提示。

### 不退化

- 无运动离线检查验证数据选择、退出码、信号、目录继承和安全清理；语法、受影响构建、选定既有协议检查有实际结果。
- STOP/ESTOP、实测停稳、新鲜反馈、观察窗口、失败推进、串口可靠传输、桥复用归属、相机驻留、录点保存与 rosbag 收尾不因日志修改改变。
- 本阶段测试证据：**未运行任何检查命令或业务程序**。以上全部为待批准执行/验收事项，不是通过声明。

## 风险

- **管道生命周期**：tee/filter 包装可能吞退出码、先结束、延迟信号或留下进程；必须先离线测实际包装，不能把日志辅助进程升级成新的任务控制器。
- **选择性终端漏错**：ROS 日志、多行 traceback 和厂商非结构化 stderr 格式不同；保守保留未知错误上下文，逐生产端降级常态信息，不靠大规模字符串删除。
- **落盘负担**：文件会增加状态/速度/系统包记录，发生在串口诊断路径；用现有 Debug 和正常缓冲，避免逐包同步刷盘。20 次保留不是容量上限，但用户明确不要求容量策略。
- **raw 取消的证据限制**：CRC 错包和未成帧字节不能事后逐字节还原；这是用户确认的取舍，不通过隐藏文件继续记录 raw。
- **协议字段不一致**：当前 YAML 的 ArmStatus 字段与部分文档提及的角速度并不完全一致；本任务按实际 YAML 解码，不补字段，不修复通信契约。记录该限制，不能承诺文件包含线协议没有的数据。
- **自动删除**：活动锁、继承与并发清理不正确会误删仍在写入的证据；只清理受管理目录，锁/归属验证失败跳过清理并报错，不能扩大删除范围。
- **复用桥**：本入口不能取得外部桥没生成的 Debug 或接管旧 stdout；不重启、不动态改它的配置，明确告诉用户/Agent 查原目录。
- **独立仓库与脏基线**：父 diff 不包含桥，多个将改文件已有用户工作；分别追踪任务增量，先隔离再集成，冲突停止。工作区已有导航/安全/文档差异不能顺带整改。
- **既有测试形态**：桥已有若干源码字符串测试，不用扩写白盒断言验证本次日志，不把它们或未运行的 PTY 测试算作新的验证证据。
- **硬件边界**：真实抓取与遥控脚本会运动，录点 Run Next Goal 也会运动。无运动检查不证明物理 STOP、夹爪、装载顺序或采摘闭环；人工验收未完成必须留在交接记录。

未完成事项：上述执行步骤 1–9 和全部实际检查/人工验收均未开始；下一步仅等待人类审批。