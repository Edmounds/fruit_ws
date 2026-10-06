# TCP 相机目标转换只读审查

## 范围

- 已读取本任务 `plan.md`、`progress.md`，使用 `workflow_git` 查看 baseline、diff、status，并比对计划相关部分。
- 审查范围：`test_camera_detector.py`、`src/fruit_mission/fruit_mission/camera_target_planner_client.py`、`src/fruit_mission/config/camera_extrinsic.yaml`、`src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`、`docs/PERCEPTION_MODE_COMMANDS.md`。必要只读调用链包括 Planner 遥测生产、接口定义/API、三个通信 launch、共享转换纯函数、MissionContext、actions 和单点抓取客户端。
- **基线归因**：ArmTelemetry 接口/Planner 发布、FK/IK 的工具偏移、共享任务客户端迁移，以及检测工具的深度/熟度/模型切换等在 baseline 已存在，不算本次新增成果或新引入缺陷。本次新增主要是查看器 TCP 消费与转换、通信管理、降级、静态姿态选项删除和说明/日志。以下两项均位于本次新增通信管理代码。
- **确认要求保持不变**：TCP 是已含局部 Z `-0.03 m` 的夹爪中心 E；示例不是默认值。同帧 XYZ、实际 pitch、模型 Q1 yaw 只用于静止观测点投影；下发 Base 米坐标、绝对 `pitch=0°`、PTP、1500 ms、末点 CLOSE。查看器仅按需复用/启动通信，不自动 Home/Look/张爪/抓取，不向复用节点发送 STOP 或结束它们。缺失/无效输入不显示伪造 Base。
- **核心坐标链核验**：`CameraTargetPlannerClient.target_request`（141–173 行）和查看器 `camera_to_base_point` 均复用 `P_E=R_E_C·P_C+t_E_C`、`P_B=Rz(yaw)·Ry(-pitch)·P_E+TCP`，米/毫米转换一致，没有再次 FK、逆 mapping 或加减 3 cm。源码 YAML 保持 `pitch_deg=0.0`、`duration_ms=1500`、`linear_path=false`，请求末点 CLOSE；正式 B 仍在首次抓取前冻结目标请求。
- 本轮只读，未执行代码、构建、测试、ROS CLI、节点启动或运动请求，未编辑业务文件。未改写已有 plan/progress。

## P0

无。

## P1

### P1-1：通信启动等待中断时，自启进程尚未移交所有权，退出清理失效

- **优先级**：P1。
- **位置**：`test_camera_detector.py::ensure_arm_communication`，395–462 行（launch 后等待并最终返回句柄）；`run`，1131 行及 1284–1289 行；`launch_background_process`，358 行。
- **触发条件**：工具缺少 bridge 或 Planner，成功创建通信 launch 子进程后，首帧遥测等待期间按 Ctrl-C，或 `rclpy.spin_once` 等操作抛异常。例如串口无反馈时，用户在最长 10 秒等待窗口内退出。
- **影响**：工具退出，但自己启动的 Planner/串口桥继续运行、占用串口并暴露控制入口，违背“退出只清理自有进程且保证启动失败也清理”的确认契约；后续运行可能复用这批未预期存留的节点。
- **证据**：launch 句柄只保存在函数局部 `debug_proc/bridge_proc/planner_proc`；函数无异常清理，直到 462 行才返回。调用者 1131 行的三个 `owned_*` 变量仅在完整函数返回后赋值，此前均为 None。发生上述异常时 `run.finally` 虽然执行，却没有句柄可清理。子进程使用 `start_new_session=True`，不会随父进程收到同一终端 Ctrl-C。本结论由控制流直接证明，本轮没有实机复现。
- **计划依据**：执行步骤 3 要求所有启动进入可靠清理路径，启动失败也清理工具自有进程。

### P1-2：遥测发布端点冲突不被拒绝，Base 可能交替使用不同来源的 TCP

- **优先级**：P1。
- **位置**：`test_camera_detector.py::ensure_arm_communication`，374–392 行；`FrameSubscriber` 139–141 行和 `_on_telemetry` 161–183 行；`run` 1237 行。
- **触发条件**：同一遥测话题同时有两个 ArmTelemetry 发布者，或旧 JointState 与新 ArmTelemetry 发布者并存。这是计划明确要求处理的升级/重复节点冲突场景。
- **影响**：函数仍将栈认作可复用，不告警、不禁止 Base。两个合法 ArmTelemetry 来源会轮流覆盖 `_tcp_pose`；如果来源的映射/几何或对应硬件不同，框和探针显示的 Base 会随来源切换，不能作为当前机械臂的可靠目标点。新旧类型并存也未进入计划要求的冲突降级。
- **证据**：
  1. 发布者数量仅用于 `>0` 判断，`>1` 没有拒绝分支；bridge 发布端也没有类型/重复检查。
  2. 类型判断是 `ArmTelemetry not in planner_types`，因此 `['sensor_msgs/msg/JointState','fruit_arm_interfaces/msg/ArmTelemetry']` 一定通过。
  3. 此处查询的是 topic graph，而不是 publisher-only 信息；查看器已创建自身 ArmTelemetry 订阅，不能据类型列表含有 ArmTelemetry 推断已有正确发布者。Humble 已提供 `get_publishers_info_by_topic`，可读取发布端的类型、节点、GID，无需新依赖。
  4. `_on_telemetry` 只验证消息内容，不限制来源/冲突；符合 frame、有限值和限位的两个来源都会被接受并刷新接收时间。后续 `run` 无冲突门槛，直接把新鲜快照用于 Base 展示。即便在类型告警分支只返回三个 None，也不会主动禁止订阅继续更新位姿。
- **归因**：冲突检测与按需启动是本次新增代码；没有把 baseline 中既有接口迁移或其他控制问题算作本次缺陷。
- **计划依据**：执行步骤 3 明确规定类型化发布端识别，旧类型/重复发布者冲突时告警、Base=N/A，不能因无反馈再开桥。

## 修复步骤

以下仅为待人类审批的修复方案，**本轮未执行**。只修上述两项，不改几何、mapping、外参值、动作配置、任务策略或图像架构，不新增测试套件/依赖。

1. **先修 P1-1，文件 `test_camera_detector.py`。**
   - 在 `ensure_arm_communication` 内为已经启动但尚未返回给调用者的进程添加局部异常清理；需覆盖 `KeyboardInterrupt`（不能仅捕获 Exception）。异常退出先结束自有 Planner，再结束自有 bridge，或结束自有组合 debug launch，然后原样抛出；成功返回后仍由 `run.finally` 清理。
   - 仅清理函数实际创建的句柄，绝不对复用节点或整个 ROS graph 执行 kill/STOP；保留最长 10 秒等待，不引入重试或自动运动。沿用现有 `stop_process`，无需单用途进程管理抽象。
2. **再修 P1-2，同一文件。**
   - 使用现有 `get_publishers_info_by_topic` 分别检查配置中订阅的遥测话题和 ArmStatus 发布端；按实际发布端类型/数量判定，不以自身订阅端宣布兼容，也不把任意类型的发布者当作正确依赖。
   - 正确唯一发布端可复用；已存在旧类型、混合类型或重复发布端时明确告警，不启动第二套通信。
   - 将已识别的冲突状态实际接到 TCP 可用性判断，清除/禁止当前位姿用于 Base，不能只是返回空进程句柄。识别、深度、Cam 显示继续；不在本修复中增加重启/重试策略。
   - 保留四种正常拓扑的原 launch 选择和默认 source-config 路径；避免额外配置键或新模块。
3. **同步受影响说明与证据。**
   - 在 `CAMERA_TARGET_TRANSFORM.md` 和 `PERCEPTION_MODE_COMMANDS.md` 明确启动中断清理及发布端冲突 N/A 规则。
   - 在原 progress.md 追加真实修复/检查结果与仍未验收项目，保留原记录，不将本轮只读结论写成测试通过。
4. **审批后执行无运动检查。所有终端先加载运行环境：**

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 -m compileall -q test_camera_detector.py
ament_flake8 test_camera_detector.py
/usr/bin/python3 test_camera_detector.py --help
git diff --check -- test_camera_detector.py \
  src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md docs/PERCEPTION_MODE_COMMANDS.md
```

   - 期望：语法/lint/差异检查通过；帮助仍无 --preset，不创建节点/GUI/通信。
   - 运行现有 `CAMERA_TARGET_TRANSFORM.md` 的最小纯数学脚本，期望仍输出 `camera/TCP/base geometry checks passed; no motion commanded`。不新写源码扫描或深度 mock 测试。纯数学通过不能作为两个通信问题已修好的证据。

## 验收

### 保留的执行阶段证据（本轮未重新运行）

`progress.md` 已记录：
- 共享客户端和查看器语法/flake8 通过，查看器 --help 无静态预设参数。
- fruit_arm_interfaces、fruit_arm_planner、fruit_mission 三包 Release 构建成功（4.20 秒）。
- 现有纯数学脚本覆盖 yaw 三方向、水平/下俯、零局部点落在 TCP、非有限数拒绝，输出成功。
- 只读遥测话题为 ArmTelemetry，实测 frame=arm_base_link，TCP 约 `[0.00052,0.14831,0.29968]m`、pitch=-57.686°、模型 yaw=1.5673 rad。

这些证据支持接口/坐标链，但不覆盖启动中断或发布端冲突；不能支持“所有运行验收均完成”的结论。

### 修复后必须补齐的验收（待审批，未执行）

- **P1-1**：操作员确认净空、停稳及通信开关安全后，在按需启动通信、尚未得到有效首帧的窗口按 Ctrl-C；确认工具自有 launch/Planner/bridge 不残留。使用仅 Planner 缺少或仅 bridge 缺少场景时，退出前后复用节点的进程/GID保持不变，不发送 STOP，不触发 Home/Look/夹爪或目标运动。正常退出也应保留此行为。
- **P1-2**：在不连接硬件的隔离环境进行公开 ROS 端点级人工验收（禁止把隔离 ROS 域误当串口隔离，不能启动真实硬件 launch）：旧类型、混合类型、两个同类型发布者均告警且 Base=N/A；唯一正确发布者的非冲突场景仍可显示真实 TCP/转换。无需新增私有回调 mock 或源码扫描测试。若没有安全的离线运行条件，只记录未验收，不通过实机自动启动补齐。
- 可用的只读检查命令（在对应验收环境、已存在发布者时）：

```bash
ros2 node list
ros2 topic info -v /arm_planner_node/joint_states
ros2 topic info -v /fruit_hardware_bridge/arm/status
```

  期望：正常场景每种状态源唯一且类型正确；冲突场景端点/GID可辨认并与告警/N/A一致；退出不遗留自有进程，不影响复用端点。必要时由操作员核对系统进程，ROS graph 消失本身不证明串口进程退出或硬件停稳。
- 原计划仍待现场验证：四种正常拓扑、断流 N/A/恢复、缺内外参降级、检测框与鼠标一致、正常/异常退出所有权。实际水平抓取仍需另行运动授权；本审查及数学检查不证明实机精度、标定、串口跨报文排序或 STOP 优先级已可靠。

**结论：P0=0，P1=2。等待扩展的人类审批，未执行修复。**
