## 目标

将视觉目标补偿改为末端相对 XYZ，并拆分 A/C 与 B 两套配置。

用户已通过最终 questionnaire 确认理解。当前阶段只规划，未执行代码、测试、构建或硬件运动。

最终需求以本计划为准，覆盖最初的单配置及观测姿态偏置方案。

## 范围

预计修改以下文件：

- `src/fruit_mission/config/fruit_mission.yaml`
- `src/fruit_mission/fruit_mission/config.py`
- `src/fruit_mission/fruit_mission/camera_target_transform.py`
- `src/fruit_mission/fruit_mission/camera_target_planner_client.py`
- `src/fruit_mission/fruit_mission/mission_context.py`
- `src/fruit_mission/fruit_mission/camera_arm_target_client.py`
- `src/fruit_mission/README.md`
- `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`
- 新增 `src/fruit_mission/test/test_camera_target_transform.py`，只检查纯数学算法。

正式 A/B/C 任务和相机单点调试客户端同时生效。

非目标：

- 不修改 Look/Store/Home、QR、导航、比赛顺序或恢复策略。
- 不补 B 区 Store，不新增“仅抓取”模式。
- 不修改相机外参、planner geometry、关节映射、FK/IK、轨迹或服务定义。
- 不修改 `camera_extrinsic.yaml` 当前抓取 pitch、duration 或 PTP/LIN 配置。
- 不新增 yaw 控制、最终 yaw 耦合求解、重试、依赖或通用配置框架。
- 不修改 `test_camera_detector.py`，它显示原始几何目标，不承担抓取补偿。
- 不自动启动硬件工具或调用运动服务。

## 决策

1. **两套源配置，统一算法。** 删除旧 `target_offset_m`，改为：

   ```yaml
   target_offset_ac_m: [0.0, 0.03, -0.015]
   target_offset_b_m: [0.03, 0.0, -0.015]
   ```

   单位均为米。A/C 共用第一套，B 使用第二套。

2. **A/C 只保留数值，不保留旧算法。** A/C 新含义是局部左移 3 cm、局部下移 1.5 cm。B 新含义是局部深入 3 cm、局部下移 1.5 cm。用户明确接受 A/C 物理含义变化。

3. **偏置坐标轴。** 局部 +X 为夹爪前方，+Y 为夹爪左侧，+Z 为夹爪上方。三个分量保留输入符号。不再使用 `abs(dy)` 或根据 Base Y 自动翻转。

4. **相机转换和偏置采用不同姿态基准。** 相机点继续使用观测时新鲜实测 TCP 的 yaw/pitch 转换。偏置使用补偿前目标方向 yaw 与最终请求 pitch：

   ```text
   P_E_mm = R_E_C × P_C_mm + t_E_C_mm
   P_B_mm = R_observation × P_E_mm + TCP_observation_mm
   yaw_offset = atan2(P_B_mm.y, P_B_mm.x)
   pitch_request = motion.pitch_deg，若为 null 则使用实测观测 pitch
   R_offset = Rz(yaw_offset) × Ry(-pitch_request)
   P_request_mm = P_B_mm + R_offset × (1000 × offset_local_m)
   ```

   当前 `motion.pitch_deg=0.0`，因此局部下移目前与 Base 下移重合。配置 pitch 非零时，X/Z 补偿随抓取 pitch 旋转。

5. **不承诺最终 yaw 完全固定。** 不修改 planner 的 `atan2(final_y, final_x)` 规则。B 当前 Y=0 且补偿不越过基座轴线时，目标方向保持不变。A/C 的 Y 非零，最终 yaw 会变化。偏置仅以补偿前方向为参考，用户接受，不新增耦合求解。

6. **保持有符号 XYZ。** 不强制 Y=0。以后 B 的 Y 非零时，同样按参考姿态旋转，不宣称严格相对补偿后最终姿态。

7. **区域必须明确。** 正式入口使用 `MissionContext._current_area`，调试入口使用已有 `run_sequence(zone, direction)` 的 zone。无效或缺失区域拒绝转换，不猜测区域。

8. **无旧键兼容或跨区回退。** 两套配置均须存在并通过现有三有限数校验。B 缺失配置不能静默使用 A/C，配置错误不能静默归零。

9. **复用现有数学实现。** 保留 `apply_target_offset_mm` 名称，改为接收抓取 pitch，并由原始 Base 目标计算参考 yaw。复用 `transform_end_to_base_mm` 旋转局部偏置，不复制旋转矩阵实现。

10. **测试范围经用户许可。** 新增一个标准库 `unittest` 纯数学检查。不编写源码扫描、ROS 私有状态 mock、比赛策略固定值测试或新增依赖。

已核实的事实：

- `target_request()` 当前顺序是相机→末端→Base→Base 轴补偿。
- `apply_target_offset_mm()` 当前根据原 Base Y 的正负选择 `±abs(dy)`。
- `config.py` 的 `load_config`、`load_target_offset`、`validate_config` 只读取旧键。
- `MissionContext` 与 `CameraArmTargetClient` 共用 `CameraTargetPlannerClient`。
- 正式 B 在首次抓取前冻结最多两个请求，后续不重算。
- `MoveArmCartesian` 只接收 Base XYZ 与 pitch，没有独立 yaw 字段。
- planner IK 根据目标 `atan2(y,x)` 选择 yaw，极小水平半径已有特殊处理。
- `src/fruit_mission/` 当前没有 `test/` 目录。`setup.py` 已排除 test 包，无需增加测试安装规则。
- `script_ros_clients.yaml` 只指定源配置路径，不复制偏置值，无需修改。

## 执行步骤

以下步骤只能在扩展的人类审批后执行。当前全部未完成。

所有验证终端先运行：

```bash
cd /home/rc4/fruit_ws
set -eo pipefail
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
set -u
```

运行环境校验失败立即停止，不使用手工 `PYTHONPATH` 绕过。

1. **建立安全任务基线。**
   - 依赖：人类执行审批。
   - 对象：当前 Git 状态与本计划列出的文件。
   - 当前主工作区已有不属于本任务的改动：`docs/CAMERA_TUNING.md`、`docs/PROJECT_ARCHITECTURE.md`、`src/fruit_arm_planner/config/arm_presets.yaml`、`src/fruit_perception/README.md`、`src/fruit_perception/config/fruit_perception.yaml`、`test_camera_detector.py`，以及未跟踪技能文件和图片。不得暂存、覆盖或提交它们。
   - 执行前重新检查并记录 main HEAD、基线 diff 和活动写入者。
   - 使用隔离分支/worktree，避免触碰并发任务。先确认目标路径被忽略且未被占用：

     ```bash
     git status --short
     git rev-parse main
     git worktree list
     git check-ignore .worktrees/target-offset-ac-b
     git worktree add .worktrees/target-offset-ac-b -b task/target-offset-ac-b main
     export TASK_WORKTREE=/home/rc4/fruit_ws/.worktrees/target-offset-ac-b
     ```

   - 检查期望：隔离 worktree 基于记录的 main，任务文件与已读实现一致。
   - 停止条件：文件被并发修改、路径/分支已存在、忽略检查失败或接口与本计划不一致。不得覆盖已有 worktree。

2. **迁移配置及读取契约。**
   - 依赖：步骤 1。
   - 文件/符号：`config/fruit_mission.yaml`，`config.py::{load_config, validate_config, validate_target_offset, load_target_offset}`。
   - 将 YAML 旧键替换为上述 AC/B 两键，注释写明各区域和局部坐标含义。
   - `validate_target_offset` 保持三有限数校验，默认上下文名改为不指向旧配置键的通用描述。
   - `load_config` 与 `validate_config` 校验并规范化两键，缺键报错。
   - 将单套读取函数改为 `load_target_offsets(config_path)`，返回 AC/B 两个已验证向量。错误消息包含键名与配置路径。
   - 不为旧键提供兼容或回退。
   - 验证命令：

     ```bash
     /usr/bin/python3 -m compileall -q \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/config.py"
     (
       cd "$TASK_WORKTREE/src/fruit_mission"
       /usr/bin/python3 - <<'PY'
     from pathlib import Path
     from fruit_mission.config import load_target_offsets
     path = str(Path('config/fruit_mission.yaml').resolve())
     ac, b = load_target_offsets(path)
     assert ac == (0.0, 0.03, -0.015)
     assert b == (0.03, 0.0, -0.015)
     print('AC/B source configuration checks passed; no motion commanded')
     PY
     )
     ```

   - 期望：语法检查和离线源配置复核通过。该复核是本次迁移验收，不放入固定调参值单元测试。
   - 停止条件：缺键、非有限数、非法维度、读取路径错误或旧配置被静默接受。

3. **修正数学函数并留下最小可运行检查。**
   - 依赖：步骤 2。
   - 文件/符号：`camera_target_transform.py::{apply_target_offset_mm, transform_end_to_base_mm, _vector3}`，新增 `test/test_camera_target_transform.py`。
   - `apply_target_offset_mm(base_xyz_mm, offset_m, pitch_rad)` 校验输入，使用 `atan2(base_y, base_x)` 确定参考 yaw。
   - 将局部偏置转换到 mm，复用 `transform_end_to_base_mm`，以原 Base 目标为平移原点生成补偿后点。
   - 删除旧 Base Y 符号分支，保留非有限数、布尔值、维度和溢出结果的拒绝。
   - 不修改相机外参转换或 FK 姿态约定。
   - 新测试只调用公共纯数学函数。覆盖零偏置、左右目标方向、非正交方向、正负 pitch、负 X/Z、有符号 Y、单位换算、平移不重复、非法维度/布尔/NaN/Inf 和非有限输出。
   - 使用手算轴向算例及独立展开的旋转公式作为期望值，不能只重复调用被测旋转函数作为 oracle。
   - 验证命令：

     ```bash
     (
       cd "$TASK_WORKTREE/src/fruit_mission"
       /usr/bin/python3 -m unittest discover -s test \
         -p test_camera_target_transform.py -v
     )
     ```

   - 期望：全部纯数学检查通过，零 ROS 节点，零硬件命令。
   - 停止条件：毫米/米错误、俯仰符号错误、Y 被自动翻转、重复平移或非法输入未拒绝。

4. **对齐共享客户端及两个区域入口。**
   - 依赖：步骤 2、3。
   - 文件/符号：`camera_target_planner_client.py::{CameraTargetPlannerClient.__init__, target_request}`，`mission_context.py::{MissionContext.__init__, camera_target_request}`，`camera_arm_target_client.py::{imports, run_sequence}`。
   - 共享客户端持有经过验证的 AC/B 两向量。参数名称与新配置键一致。
   - `target_request(target, area)` 要求 A/B/C 区域。按 B 或 A/C 选择向量。
   - 保留原相机→末端→Base 几何链和新鲜实测遥测校验。
   - 在应用偏置前解析最终请求 pitch，偏置使用同一个 pitch。
   - 正式入口显式传入 `_current_area`。调试入口加载两套配置并显式传入 `zone`。
   - 不从 Base Y、左右侧、果实名或缓存区域猜测配置。
   - debug 日志记录区域、配置键、局部偏置、偏置参考 yaw/pitch、原 Base 点与最终请求点。
   - B 两目标仍在第一次运动前冻结，第二抓不重新计算。
   - 验证命令：

     ```bash
     /usr/bin/python3 -m compileall -q \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/config.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/camera_target_transform.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/camera_target_planner_client.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/mission_context.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/camera_arm_target_client.py" \
       "$TASK_WORKTREE/src/fruit_mission/test/test_camera_target_transform.py"
     ament_flake8 \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/config.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/camera_target_transform.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/camera_target_planner_client.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/mission_context.py" \
       "$TASK_WORKTREE/src/fruit_mission/fruit_mission/camera_arm_target_client.py" \
       "$TASK_WORKTREE/src/fruit_mission/test/test_camera_target_transform.py"
     (
       cd "$TASK_WORKTREE/src/fruit_mission"
       /usr/bin/python3 -m unittest discover -s test \
         -p test_camera_target_transform.py -v
     )
     ```

   - 人工审阅所有 `CameraTargetPlannerClient`、`target_request` 和配置读取调用点，核对区域参数。不写源码字符串断言测试，不构造深度 ROS mock。
   - 期望：语法/针对性 lint/数学检查通过，所有调用点使用新契约。
   - 停止条件：遗漏旧调用、区域缺失被默认为 A、pitch 基准不一致，或安全/流程逻辑出现额外 diff。若存在既有 lint 问题，报告基线差异，不做无关格式清理。

5. **同步文档、构建并形成阶段提交。**
   - 依赖：步骤 4。
   - 文件：`README.md` 的偏置段，`docs/CAMERA_TARGET_TRANSFORM.md` 的路径图、第三步公式、配置语义与离线验收命令。
   - 写明新键、两套数值、区域选择、局部符号、原始目标 yaw 与请求 pitch。
   - 明确 A/C 改变物理含义，非零局部 Y 不保证最终 yaw 一致。
   - 删除文档中仍描述为当前行为的 Base Y 自动翻转说明。历史外参验证不冒充本次测试证据。
   - 不改变 planner 接口，仍提交 Base 米制 XYZ。说明源码修改后需重建并重启 mission/调试客户端。
   - 隔离构建，不覆盖正在使用的主 install：

     ```bash
     colcon --log-base /home/rc4/fruit_ws/tmp/target-offset-logs build \
       --base-paths "$TASK_WORKTREE/src/fruit_mission" \
       --packages-select fruit_mission \
       --build-base /home/rc4/fruit_ws/tmp/target-offset-build \
       --install-base /home/rc4/fruit_ws/tmp/target-offset-install
     git -C "$TASK_WORKTREE" diff --check -- \
       src/fruit_mission/config/fruit_mission.yaml \
       src/fruit_mission/fruit_mission/config.py \
       src/fruit_mission/fruit_mission/camera_target_transform.py \
       src/fruit_mission/fruit_mission/camera_target_planner_client.py \
       src/fruit_mission/fruit_mission/mission_context.py \
       src/fruit_mission/fruit_mission/camera_arm_target_client.py \
       src/fruit_mission/test/test_camera_target_transform.py \
       src/fruit_mission/README.md \
       src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
     ```

   - 依赖使用已安装的 workspace/ROS 包。若依赖缺失，停止并报告，不启动无关全工作区构建。
   - 新增测试文件还须单独检查，因为未跟踪文件不在普通 Git diff 中。
   - 将验证命令、退出码和限制写入本任务记录。完整阶段验证通过后，只暂存上述任务文件并提交到任务分支，不绕过 hooks。
   - 期望：构建成功、定向 diff 检查通过、文档与实现一致，提交不包含用户既有改动。
   - 停止条件：构建失败、检查工具缺失、依赖不满足、文档与实际代码不一致或文件所有权不明确。不得把未运行检查记为通过。

6. **安全集成或持久交接。**
   - 依赖：步骤 5 全部验证通过。
   - 再次检查 main 是否干净及是否有活动写入者。当前已知 main 有未提交改动，不能直接集成。
   - 若仍有改动或活动写入者，保留任务分支/worktree，并写持久交接：仓库路径、worktree 路径、分支、tip SHA、验证基线 main SHA、验证结果/限制，以及 `Pending Integration` 原因。
   - 不 stash main，不提交用户文件，不部署未集成版本，不删除未合并任务 worktree。
   - main 干净后串行重核对最新 main，分析任何冲突并重跑步骤 2～5。main 在验证期间移动则重新验证。
   - 只执行 fast-forward 集成，不覆盖 main。集成成功后才清理任务 worktree/分支。
   - 主工作区部署构建命令：

     ```bash
     cd /home/rc4/fruit_ws
     source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
     colcon build --base-paths src --packages-select fruit_mission
     source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
     (
       cd src/fruit_mission
       /usr/bin/python3 -m unittest discover -s test \
         -p test_camera_target_transform.py -v
     )
     ```

   - 期望：部署只来自 main，构建及纯数学检查通过。客户端需人工在安全停稳后重启，本任务不启动真实流程。
   - 停止条件：main 脏、并发写入、意图冲突、非 fast-forward、部署构建失败。安全延期不得报告为已集成。

## 验收

- 配置包含 `target_offset_ac_m` 和 `target_offset_b_m`，不再读取旧键。
- A/C 使用 AC 向量，B 使用 B 向量，正式与调试入口一致。
- 两套偏置均是有符号局部 XYZ，不按 Base Y 自动翻转。
- 相机转换继续使用观测实测姿态，偏置使用原始目标 yaw 与请求 pitch。
- 数学结果满足 `P_request = P_B + R_offset × offset`，不重复加入 TCP 平移。
- 原 Base 目标 `(100, 200, 300)` mm、pitch=0、偏置 `(0.03, 0, -0.015)` m，结果应约为 `(113.416408, 226.832816, 285)` mm。
- 对称右侧目标 `(100, -200, 300)` mm，应约为 `(113.416408, -226.832816, 285)` mm。
- yaw=0、pitch=-45°、同一偏置，Base 增量应约为 `(10.606602, 0, -31.819805)` mm，证明 X/Z 随 pitch 耦合旋转。
- 测试不能以被测函数再次计算期望值。非有限数、非法维度和布尔输入须拒绝。
- 两个 B 请求仍首次运动前冻结，第二抓不重新变换。
- frame/depth/遥测新鲜度、硬件限位、静止检查、STOP、取消、服务结果判断不变。
- 构建、语法、定向 lint、纯数学检查及 diff 检查均提供实际证据。
- 未进行实机验证时，明确报告未实机验收。软件检查不代表抓取成功或轨迹安全。

当前证据状态：只完成文件读取、调用关系检查和用户需求确认。上述测试命令均尚未执行。

## 风险

- A/C 保留数值但改变坐标语义，必须在安全现场复核抓取位置。此变化已获用户确认。
- A/C 的非零局部 Y 会改变最终 yaw。用户选择参考方向补偿，不选择严格最终姿态耦合。
- 偏置过大、水平半径接近零或补偿跨过轴线时，不能保证 B 的 yaw 不变。继续由现有 planner 可达性和限位校验决定运动许可，不新增未请求回退。
- 当前抓取 pitch 为 0°。观察 pitch 即使倾斜，偏置也不跟随观察 pitch，这是确认后的决策。
- 配置键为不兼容迁移。仅有旧键的外部配置须人工迁移，缺少新键应报错。
- 观察期间及请求冻结前必须保持既有静止姿态，不能用本修正宣称运动中定位正确。
- 当前 main 脏，任务很可能需要 Pending Integration 交接。不得覆盖已有调参或并发工作。
- B Store 是否齐全及串口轨迹装载顺序仍是既有集成限制，不属于本任务。
- 偏置几何正确不证明 PTP 轨迹沿夹爪轴线直线推进，也不证明无碰撞或实际采摘成功。
