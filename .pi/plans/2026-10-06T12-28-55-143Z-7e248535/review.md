## 范围

审查依据：本目录的 `plan.md`、`progress.md`、`baseline.md` 及 workflow_git 的 baseline、diff、status。
范围限定为计划涉及的 B 区 Up/Down 实现、文档、四个 Store 预设及临时验收脚本。

检查了以下调用链：
- `mission_node.load_config` → `MissionContext` → `actions.pick_b_tree` → 共享目标客户端 → planner 服务。
- `CameraArmTargetClient.run_sequence` → `load_target_offsets` → 共享目标客户端 → 分层 Store。
- `Logic.run_b` 的区域设置与抓取调用。
- 相机转换数学函数、既有数学测试及 `tmp/b-up-down/` 两个验收脚本。

启动前四个 B Store 预设和单份 B 偏置改动属于 baseline，不归因于本次执行。
用户已授权纳入预设，但要求保持其现有内容。
当前四个预设与 baseline 一致。
无关技能、图片和外部包不在审查范围。

保留的确认决策：
- 分类使用未加偏置的 `arm_base_link` Z，单位 m。
- Down 为 `[-0.01, 0.12)`，Up 为 `[0.12, 0.30]`。
- Up pitch 为 `0°`，偏置为 `[0.026, 0, -0.01]` m。
- Down pitch 为 `-30°`，偏置为 `[0.022, 0, -0.005]` m。
- 两层偏置旋转参考 pitch 固定为 `0°`。
- 首抓前冻结各目标请求与 Store 名称，不重新分类、不补位、不重试。
- 越界目标不派发抓取或 Store，保留原失败恢复与后续推进。
- A/C 参数与流程不变。
- 调试恢复仅修正 `timeout=5.0`，保留原 `zero → home` 顺序。

只读检查未发现上述业务分层实现的 P0/P1 缺陷。
以下 P1 位于计划要求的无硬件验收工具。
本轮没有运行代码、构建、ROS 服务、修复或提交。

## P0

无。

## P1

### P1-1：无硬件验收脚本未隔离实机控制端点

- **优先级**：P1。
- **文件/符号**：
  - `tmp/b-up-down/check_b_layers.py::run_tests`。
  - `tmp/b-up-down/mock_server.py::MockServer.__init__`、`_on_timer`。
  - 调用依据：`src/fruit_mission/fruit_mission/camera_target_planner_client.py:185-190`、`_gripper_at_measured_pose`。
- **触发条件**：运行验收脚本时，localhost 的 ROS domain `221` 已有真实 planner 或硬件控制节点。
- **影响**：脚本可能向真实服务派发 HOLD、Store、zero 或 home 请求。
  合成零角度、IDLE 反馈也可能被真实 planner 接收，污染实测运动起点及停稳判断。
  不能将当前工具视为具备强制隔离的无硬件验证工具。
- **证据**：
  1. `run_tests` 在创建模拟服务器前没有检查域内外部节点或控制端点。
  2. `mock_server.py` 使用 `/arm_planner_node/execute_preset` 与 `/arm_planner_node/move_joints`。
  3. 它向 `/fruit_hardware_bridge/arm/status` 和 `/arm_planner_node/joint_states` 发布合成反馈。
  4. 客户端没有 remap 这些硬编码绝对名称。
  5. `run_tests` 实际执行 `move_to_base_target`、`execute_preset` 和 `safe_recover_home`。
  6. CLOSE 后驻留调用硬编码 `move_joints`，其关节角来自上述合成状态。
  7. `env['ROS_DOMAIN_ID']='221'` 只写入子进程环境，父进程没有验证相同域及 localhost 设置。
  8. 计划步骤 7 明确要求先检查空闲域，并把硬编码服务和状态话题全部重映射至 `/offline_b_layers/*`。

以上是代码可定位的隔离缺陷，不是已经发生实机运动的证据。
历史成功日志不能证明运行时不存在外部控制节点。

## 修复步骤

仅修复 P1-1，不修改业务分层、机械臂预设或恢复策略。
以下步骤等待人类批准，本轮未执行。

1. 修改 `tmp/b-up-down/check_b_layers.py::run_tests`。
   在启动任何模拟发布者前验证父进程使用 domain `221` 和 localhost。
   环境不符时直接退出，不启动子进程、不调用运动服务。
2. 在启动模拟服务器前执行有限的 ROS 图发现。
   检测到外部节点或控制端点时直接退出。
   不停止、复用或操作外部节点。
3. 为全部测试客户端使用同一组 ROS remap。
   将 `execute_preset`、`move_joints`、`move_cartesian`、`stop`、ArmStatus 和 ArmTelemetry 端点映射至 `/offline_b_layers/*`。
   remap 必须覆盖 `debug_client`、`real_client` 及调试退出时的 STOP。
   保持源码外参文件不变，仅在测试节点上重映射其端点。
4. 修改 `tmp/b-up-down/mock_server.py`。
   只在 `/offline_b_layers/*` 发布反馈、提供服务。
   删除向生产遥测话题额外发布的行为。
5. 在 `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md` 记录安全运行命令及隔离边界。
   在交接记录中保留历史结果，同时明确它们不证明修复后的隔离。
6. 执行下面的验收命令，记录退出码、日志及端点检查结果。
   验收不得启动真实 planner、串口桥或硬件任务。

## 验收

### 现有证据及限制

- `progress.md` 记录 flake8、两个包构建及既有 10 项数学测试通过。
- 历史执行记录显示临时脚本退出码为 0。
- 已读取 `/tmp/pi-unified-exec-33-c3af3a3c.log`，正文包含 `ALL TESTS PASSED SUCCESSFULLY.`。
- 该日志支持边界分类、pitch 解耦、模拟服务成功与恢复参数名检查的历史结果。
- 本轮没有复跑这些检查。
- 既有 10 项测试验证偏置数学，不直接验证新的分层业务工序。
- 当前临时脚本没有调用正式 `actions.pick_b_tree`。
  “越界跳过”检查是脚本自己实现的循环，不能作为正式工序派发证据。
- 当前服务器始终返回运动成功，也未在冻结后改变遥测。
  当前版本不能证明第一抓失败推进、姿态变化后的冻结或反向目标顺序场景。
- 这些覆盖限制不单独列为 P1，也不据此断言业务代码错误。
- 计划步骤 8 尚未完成。当前任务仍未提交、未推送、未部署。
- Store 结构校验和模拟应答不证明 LIN 全路径可达、无碰撞或实机入筐。

### P1-1 修复后的检查命令与期望

全部命令仅供批准后的执行阶段使用。

准备环境：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

检查脚本语法：

```bash
/usr/bin/python3 -m compileall -q tmp/b-up-down/check_b_layers.py tmp/b-up-down/mock_server.py
```

期望：退出码 0。

验证错误环境拒绝：

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tmp/b-up-down/check_b_layers.py
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=0 /usr/bin/python3 tmp/b-up-down/check_b_layers.py
```

期望：非零退出，并在启动模拟服务器或派发请求前报告环境不符。

验证外部节点拒绝：在已确认无硬件节点的 domain `221` 中操作。
先在独立终端运行修复后的模拟服务器：

```bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tmp/b-up-down/mock_server.py
```

再在另一终端执行：

```bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tmp/b-up-down/check_b_layers.py
```

期望：验收脚本非零退出，明确报告外部节点，不启动第二个模拟服务器，不发送运动请求。
停止仅由本次验收启动的模拟服务器。

验证正常隔离运行：

```bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tmp/b-up-down/check_b_layers.py
```

期望：退出码 0，原有边界、pitch、偏置、Store 与恢复参数名检查通过。
测试节点的已解析运动服务和反馈话题全部位于 `/offline_b_layers/*`。
测试不得发布生产 ArmStatus/ArmTelemetry，不得连接生产运动或 STOP 服务。

审查结论：P0 为 0，P1 为 1。等待人类审批，仅修复上述隔离问题。
