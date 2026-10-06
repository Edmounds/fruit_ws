## 目标

为 B 区新增 Up/Down 两层抓取配置。
正式 mission 与 `test_camera_arm_target.sh` 共用分层结果。
每个目标独立选择抓取 pitch、局部偏置及对应侧的 Store 预设。

当前阶段仅规划。用户已确认需求理解，但尚未批准执行。
计划保存目录：`/home/rc4/fruit_ws/.pi/plans/2026-10-06T12-28-55-143Z-7e248535`。

## 范围

- 修改 `src/fruit_mission/config/camera_extrinsic.yaml` 的 B 抓取 pitch 与高度配置。
- 修改 `src/fruit_mission/config/fruit_mission.yaml` 的 B 局部偏置配置。
- 修改 mission 配置加载、共享目标转换客户端、正式抓取工序及单点调试客户端。
- 更新 `test_camera_arm_target.sh` 的帮助文字，不改变命令行参数或启动逻辑。
- 同步 mission、planner 与项目架构的相关文档。
- 用户授权将现有四个 B Store 预设改动纳入任务提交。
- 保持 `src/fruit_arm_planner/config/arm_presets.yaml` 的现有内容不变。
- 用户授权只修正调试恢复调用的 `timeout_sec` 参数名，不改变恢复动作或顺序。

非目标：

- 不修改 A/C 抓取参数、偏置或 Store。
- 不修改 B 的 Look、每树观察次数、成熟度/品种判断或像素评分。
- 不调整目标顺序，不强制每层一个目标，不补位，不增加识别或抓取重试。
- 不修改导航、二维码、语音、区域话题或任务路线。
- 不修改 planner FK/IK、关节映射、几何、限位或轨迹算法。
- 不修改 ROS 消息、服务定义、串口协议或下位机行为。
- 不新增依赖、测试框架、配置兼容层或姿态映射字典。
- 不运行硬件抓取脚本、比赛入口、控制台或真实运动服务作为自动验证。
- 不清理其他既有改动，不修改生成文件。

已查证的当前事实：

- 当前分支为 `main`，读取时 HEAD 为 `40c7e3ba1f55ba43d5cb4b4c7946152fbe9d1ae4`。
- `CameraTargetPlannerClient.target_request(target, area)` 当前返回单个 `MoveArmCartesian.Request`。
- 当前 B 统一使用 `motion.pitch_b_deg: -30.0`。
- 当前 B 偏置为 `[0.026, 0.0, -0.01]` m，属于启动前已有配置改动。
- 正式 mission 在 `actions.pick_b_tree` 中先选最多两个目标，再转换并冻结请求。
- 正式 Store 在 `actions._grasp_and_collect` 中仍使用 `<side>_store_b`。
- 调试 Store 在 `CameraArmTargetClient.run_sequence` 中仍使用 `<side>_store_b`。
- 四个 `<side>_store_b_up/down` 已存在于工作区 YAML。
- `load_target_offsets` 当前返回 A/C 与 B 两组偏置。
- `MissionContext` 与调试客户端是共享目标客户端的两个业务调用方。
- `script_ros_clients.yaml` 已指向源码配置，不需要改路径。
- `setup.py` 与 `package.xml` 已包含所需安装规则和依赖，不需要修改。
- `CameraArmTargetClient.wait_until` 参数为 `timeout`，恢复调用却使用 `timeout_sec=5.0`。
- planner 提供公开的 `validate_presets` 函数，可离线校验预设结构，不需要构造控制节点。

## 决策

全部行为决策已通过三轮 questionnaire 确认。
用户最终选择“确认，写计划并等待执行审批”。

1. 高度使用转换到 `arm_base_link` 后、尚未应用抓取偏置的目标 Z。
   它不是相机深度、底盘 `base_link` 高度或当前 TCP 高度。
2. B 高度配置使用米，要求 `min_m < split_m < max_m`。
3. 默认值为 `min_m=-0.01`、`split_m=0.12`、`max_m=0.30`。
4. Down 范围为 `-0.01 ≤ Z < 0.12`。
5. Up 范围为 `0.12 ≤ Z ≤ 0.30`。
6. 上下限均包含端点，恰好 `0.12` m 归 Up。
7. 范围检查和分类使用转换结果，不使用已写入 float32 服务请求的 Z。
8. 不新增阈值容差、滞回或高度夹紧。
9. 范围外目标不发抓取或 Store 请求。
   正式 mission 跳过该目标，继续其他已选目标及后续流程。
   调试入口结束本次单目标工序，沿用现有失败恢复路径。
10. 上下限只约束未补偿的目标 Z。
    补偿后 XYZ 不再次按 B 分层上下限过滤，仍接受 planner 的既有安全校验。
11. 两层初始参数如下：

| 层 | 抓取绝对 pitch | 局部偏置 `[x,y,z]`，m | Store |
| --- | --- | --- | --- |
| Up | `0.0°` | `[0.026, 0.0, -0.01]` | `left_store_b_up` / `right_store_b_up` |
| Down | `-30.0°` | `[0.022, 0.0, -0.005]` | `left_store_b_down` / `right_store_b_down` |

12. 两层偏置参考 pitch 都为 `0°`。
    偏置继续使用未补偿目标方向的 `atan2(base_y, base_x)` 旋转。
    修改层抓取 pitch 不改变该层的补偿 XYZ。
13. 相机到基座换算继续使用新鲜实测观察姿态。
    不用新抓取 pitch 替代观察 pitch。
14. 每个目标在首次抓取前冻结请求、层分类及 Store 名称。
    第二抓不重新转换、不重新分类，不从最后一次转换状态读取层。
15. 保留现有最多两个目标及其顺序。
    两个目标同属一层时都可抓取。
    已选目标越界时不补选其他候选。
16. 不保留旧的 B 单份配置键或 Store 名称回退。
    缺失、非法配置直接报错，不跨层或跨区回退。
17. 配置文件分工保持不变：

```yaml
# src/fruit_mission/config/camera_extrinsic.yaml
motion:
  pitch_deg: 0.0
  pitch_b_up_deg: 0.0
  pitch_b_down_deg: -30.0
  duration_ms: 1500
  linear_path: false
b_height:
  min_m: -0.01
  split_m: 0.12
  max_m: 0.30
```

```yaml
# src/fruit_mission/config/fruit_mission.yaml
# 位于 fruit_mission.ros__parameters 内
 target_offset_ac_m: [0.0, 0.03, -0.015]
 target_offset_b_up_m: [0.026, 0.0, -0.01]
 target_offset_b_down_m: [0.022, 0.0, -0.005]
```

18. 保留原 pitch 的有限数/null 语义。
    null 使用实测观察 pitch，B 偏置仍保持水平参考。
19. 采用简单的内部返回值 `(request, layer)`。
    B 的 layer 为 `up` 或 `down`，A/C 为 `None`。
    ROS 服务请求本身不增加层字段。
20. 在共享客户端模块新增小型公开函数 `store_preset_name(side, area, layer)`。
    两个业务入口共用名称拼接，不增加映射表或第二套分类逻辑。
21. 四个现有 Store 的坐标、时长、夹爪动作、LIN 和 `is_via_point` 全部保持不变。
    用户明确授权将这些现有预设改动纳入本次提交。
22. 调试恢复仅将 `timeout_sec=5.0` 改为 `timeout=5.0`。
    不借此调整现有 STOP、停稳等待或 `zero → home` 策略。
23. 不削弱反馈新鲜度、有限数、frame、时间戳、限位、运动占用、取消和 STOP 校验。
    服务完成与发布成功的现有区别保持不变。

## 执行步骤

所有步骤均待扩展的人类执行审批。
以下命令仅是执行阶段的操作说明，本规划阶段未运行。

### 1. 核对最新基线和任务所有权

- 文件：本计划、`baseline.md`、两份 mission YAML、四个客户端/工序文件、`config.py`、planner preset YAML。
- 先后依赖：执行审批后第一步，先于任何编辑。
- 操作：核对 HEAD、暂存区、工作区及文件当前内容。
- 命令：`git status --short`、`git log -5 --oneline`、`git diff`、`git diff --cached`。
- 期望：识别所有新变化，保留启动前改动及用户授权的四个 Store 预设。
- 停止条件：遇到重叠外部改动或意图不明确时，报告 `External changes detected` 并重新核对。
- 工作区：无并发重叠时使用当前 `main`。确有重叠时按 worktree 流程隔离，不强制切换机器人部署版本。

### 2. 更新源码配置和偏置加载链

- 文件：
  - `src/fruit_mission/config/camera_extrinsic.yaml`
  - `src/fruit_mission/config/fruit_mission.yaml`
  - `src/fruit_mission/fruit_mission/config.py`
- 符号：`load_config`、`validate_config`、`load_target_offsets`、`validate_target_offset`。
- 改动：替换 `motion.pitch_b_deg`，新增两层 pitch 与 `b_height` 三个值。
- 改动：替换 `target_offset_b_m`，保留 A/C 原值，增加两组用户确认的 B 偏置。
- 改动：复用 `validate_target_offset` 验证三组偏置，保持来源路径的错误上下文。
- 改动：`load_target_offsets` 返回 `(ac_offset, b_up_offset, b_down_offset)`。
- 改动：缺失新键直接报错，不读旧 B 键，不引入零偏置或另一层默认回退。
- 先后依赖：依赖步骤 1，与步骤 3、4 作为同一个接口迁移完成，不能发布半迁移版本。
- 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/config.py`。
- 期望：语法通过，最终离线检查确认两个加载入口返回相同三组偏置。
- 停止条件：遗漏加载入口、配置值不一致或非法配置被接受时停止。

### 3. 在共享客户端完成唯一分层入口

- 文件：`src/fruit_mission/fruit_mission/camera_target_planner_client.py`。
- 符号：`CameraTargetPlannerClient.__init__`、`target_request`，新增 `store_preset_name`。
- 改动：接收并验证 Up/Down 两组偏置，更新 pitch 键校验。
- 改动：验证 `b_height` 是映射，三个值为非布尔有限数，且严格递增。
- 改动：保留现有 optical → E → Base 转换。
- 改动：转换后先取 `base_mm[2] / 1000.0`，仅 B 检查范围并确定层。
- 改动：越界抛出带高度及范围信息的 `ValueError`，交业务调用方跳过。
- 改动：按层选择 pitch 与偏置，再复用 `apply_target_offset_mm`。
- 改动：B 偏置旋转 pitch 固定为 0，A/C 保持既有计算。
- 改动：返回 `(request, layer)`，不在客户端保存“最后一次层分类”。
- 改动：共享名称函数验证 side/area/layer，直接生成现有 YAML 名称。
- 改动：B 必须提供合法 layer，A/C 不附加层后缀，不提供旧名称别名。
- 改动：日志明确记录层、未补偿 Base Z、pitch 键、偏置键、参考 pitch 和请求 XYZ。
- 先后依赖：依赖步骤 2 的配置契约，先于步骤 4 的调用方改造。
- 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/camera_target_planner_client.py`。
- 期望：唯一分层入口及返回值契约完整，原安全检查不变。
- 停止条件：使用补偿后 Z、请求 float32 Z 或当前 TCP 高度分类时停止。

### 4. 同步正式 mission 和调试入口

- 文件：
  - `src/fruit_mission/fruit_mission/mission_context.py`
  - `src/fruit_mission/fruit_mission/actions.py`
  - `src/fruit_mission/fruit_mission/camera_arm_target_client.py`
- 符号：
  - `MissionContext.__init__`、`camera_target_request`
  - `observe_and_pick`、`pick_b_tree`、`_grasp_and_collect`
  - `CameraArmTargetClient.run_sequence`、`safe_recover_home`
- 改动：两个构造入口传入三组已验证偏置，不留下旧 B 参数。
- 改动：`camera_target_request` 透传 `(request, layer)`。
- 改动：A/C 单目标工序解包返回值，仍使用原区域 Store。
- 改动：`pick_b_tree` 转换每个已选目标时，同时生成 Store 名称。
- 改动：冻结列表保存请求、Store 名称与现有 model_type。
- 改动：`_grasp_and_collect` 接收冻结的 Store 名称，不再按当前状态重算 B 名称。
- 改动：第一抓失败仍进入下一冻结目标，安全门控仍决定是否允许实际运动。
- 改动：调试入口使用同一层分类及名称函数，越界不重新获取候选。
- 改动：调试恢复只修正 `timeout=5.0` 参数名。
- 保持：正式流程成功 Store 后的 Home 不变，调试成功路径不增加最终 Home。
- 先后依赖：依赖步骤 3，所有返回值调用方必须一起完成。
- 检查命令：

```bash
/usr/bin/python3 -m compileall -q \
  src/fruit_mission/fruit_mission/mission_context.py \
  src/fruit_mission/fruit_mission/actions.py \
  src/fruit_mission/fruit_mission/camera_arm_target_client.py
```

- 期望：所有业务调用方解包一致，第二目标保持自身层和 Store。
- 停止条件：目标之间共享可变层状态、发生补位/重排或变更恢复策略时停止。

### 5. 核对预设并同步说明

- 预设文件：`src/fruit_arm_planner/config/arm_presets.yaml`。
- 操作：保留现有四个 B Store 内容，只核对并按授权纳入提交。
- 不修改 geometry、joint_mapping、planning、Look 或其他预设。
- 文档和帮助文件：
  - `src/fruit_mission/README.md`
  - `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`
  - `docs/PROJECT_ARCHITECTURE.md`
  - `src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md`
  - `src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md`
  - `src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md`
  - `src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md`
  - `test_camera_arm_target.sh`
- 改动：说明 B 的 `<side>_store_b_up/down` 名称、三高度值、两层参数及冻结时机。
- 改动：说明旧 B 配置键必须迁移，缺失不回退。
- 改动：说明预设重载只更新 presets，客户端配置修改需要重启客户端。
- 改动：算法与映射文档仅补充边界说明，B 分层属于客户端策略，不改变 FK/IK 或标定。
- 不修正文档中其他无关历史问题，不重写公式或标定数字。
- 改动：脚本帮助说明自动按目标 Base Z 选择层，不增加 `--up/--down` 参数。
- 先后依赖：依赖步骤 2～4 的实际契约。
- 检查命令：`bash -n test_camera_arm_target.sh`。
- 离线预设检查命令：

```bash
/usr/bin/python3 - <<'PY'
import yaml
from fruit_arm_planner.arm_planner_node import validate_presets
path = '/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml'
with open(path, encoding='utf-8') as stream:
    presets = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']['presets']
validate_presets(presets)
for side in ('left', 'right'):
    for layer in ('up', 'down'):
        name = f'{side}_store_b_{layer}'
        if name not in presets:
            raise ValueError(f'Missing preset: {name}')
print('B layer presets validated; no motion requested')
PY
```

- 期望：四个名称存在且通过现有 planner 结构校验，脚本语法通过。
- 停止条件：预设内容与授权基线不同、缺失或结构校验失败时停止，不擅自补姿态。

### 6. 构建并执行已有数学检查

- 先后依赖：步骤 2～5 完成后执行。
- 所有构建、ROS 和检查终端先加载项目运行环境。
- 不在 `set -u` 已启用时 source Humble。
- 命令：

```bash
cd /home/rc4/fruit_ws
set -eo pipefail
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
set -u

/usr/bin/python3 -m compileall -q \
  src/fruit_mission/fruit_mission/config.py \
  src/fruit_mission/fruit_mission/camera_target_planner_client.py \
  src/fruit_mission/fruit_mission/mission_context.py \
  src/fruit_mission/fruit_mission/actions.py \
  src/fruit_mission/fruit_mission/camera_arm_target_client.py

ament_flake8 \
  src/fruit_mission/fruit_mission/config.py \
  src/fruit_mission/fruit_mission/camera_target_planner_client.py \
  src/fruit_mission/fruit_mission/mission_context.py \
  src/fruit_mission/fruit_mission/actions.py \
  src/fruit_mission/fruit_mission/camera_arm_target_client.py

bash -n test_camera_arm_target.sh
colcon build --base-paths src --packages-select fruit_arm_planner fruit_mission \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

set +u
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
set -u
/usr/bin/python3 -m unittest discover -s src/fruit_mission/test \
  -p test_camera_target_transform.py -v
```

- 期望：运行环境、语法、受影响文件风格、两个包构建及现有 10 项数学检查通过。
- 停止条件：环境验证、依赖解析、构建或任何检查失败时停止。
- 不用 pip、PYTHONPATH 绕过验证，不进行全仓格式化。
- 如存在原有风格问题，记录其来源，不擅自清理无关代码或冒称检查通过。

### 7. 留下最小可运行的无硬件检查

- 新建临时检查文件：`tmp/b-up-down/check_b_layers.py`，不新增永久测试文件。
- 文档：在 `CAMERA_TARGET_TRANSFORM.md` 保留可复现的检查方法及验证边界。
- 先后依赖：步骤 6 构建并重新加载环境后执行。
- 检查方法：使用实际 ROS 类型与公开客户端 API，不构造 fake ROS 类，不改私有状态或方法。
- 仅在独立 ROS domain `221`、localhost 与 `/offline_b_layers/*` 名称下发布合成反馈。
- 先确认该域没有外部节点，否则停止并改用经核实的空闲测试域。
- 请求级检查不调用运动服务，只读取配置、发布合成遥测、构造请求及 Store 名称。
- 对精确高度边界使用临时已知变换配置，避免用任意光学坐标猜测 Base Z。
- 用源码外参另核对转换及偏置算例，不能把临时变换当作实机标定。
- 用 `load_config` 与 `load_target_offsets` 分别读取正式/调试配置，核对结果一致。
- 若检查工序派发，使用隔离域中的公开 ROS 模拟反馈/服务端。
  必须将硬编码服务和状态话题全部重映射到 `/offline_b_layers/*`。
  不启动真实 planner、串口桥或感知 launch。
- 通过 `MissionContext` 已有公开 speaker 注入接口禁用实际语音，不新增业务依赖。
- 脚本使用有限的每场景期限，无后台忙轮询或无限等待。
- 命令：

```bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 tmp/b-up-down/check_b_layers.py
```

- 请求级期望：
  - `Z=-0.010` 归 Down，`Z=0.120` 归 Up，`Z=0.300` 归 Up。
  - `Z=-0.010001` 与 `Z=0.300001` 被拒绝。
  - `Z=0.119999` 归 Down，`Z=0.120001` 归 Up。
  - 左右侧与四个 Store 名称匹配。
  - Up/Down pitch、XYZ 偏置符合确认值。
  - Up 原始 `Z=0.125` 补偿后低于 `0.12`，仍保持 Up。
  - Down 下限目标补偿后低于 `-0.01`，不会被额外按 B 高度范围拒绝。
  - 单独改变某层 pitch，不改变该层 XYZ，也不改变另一层参数。
  - A/C 请求与原参数一致，不接受 B 高度范围限制。
  - 第一目标为 Up、第二目标为 Down，以及反向顺序，都保留各自 Store。
  - 冻结后反馈变化不改变两个已冻结请求和 Store 名称。
  - 缺失键、错序/相等阈值、布尔、非有限数、非法偏置均被拒绝。
  - 无效 optical frame、非正深度及缺失/陈旧遥测仍被拒绝。
- 工序模拟期望：
  - 第一个已选目标越界时仍处理第二个有效目标，不补位或重新观察。
  - 第一抓收到模拟失败结果时仍推进下一冻结目标。
  - 不为越界目标派发抓取或 Store。
  - 调试越界路径返回失败且没有 `TypeError`，恢复沿用原策略。
  - 所有模拟运动请求仅到隔离服务，不产生真实硬件命令。
- 停止条件：域不隔离、出现外部控制端点、任何断言失败或模拟超时时停止。
- 证据：记录命令、退出码、场景结果与日志路径。
  请求级、工序模拟和实机证据分别记录，不互相替代。

### 8. 复核差异并提交验证后的任务范围

- 先后依赖：步骤 1～7 全部完成且验证通过。
- 操作：重新核对最新 HEAD、外部工作区改动及所有任务文件。
- 命令：`git status --short`、`git diff --check`、`git diff --cached --check`、`git diff`。
- 检查：未修改现有四个 B Store 内容，未纳入其他新增外部改动。
- 暂存：仅明确列出的任务文件或任务 hunks，包括用户授权的四个 Store 预设改动。
- 禁止：`git add .`、`git add -A`、自动 stash、丢弃其他改动或绕过提交检查。
- 提交：验证通过后立即提交，不等待第二次提交请求。
- 期望：得到已验证提交，报告 SHA、文件范围及是否已集成/推送。
- 停止条件：HEAD 移动、所有权不清、验证失败或重叠改动时停止提交并重核对。
- 如使用 worktree，按最新 main 重新验证后仅用 `git merge --ff-only` 集成。
- 集成被实际冲突阻塞时保留提交与 worktree，留下 `Pending Integration` 交接。

## 验收

- 正式 mission 与调试入口使用同一 B 高度分类、pitch、偏置和 Store 命名逻辑。
- 配置中的上限、分层值和下限可分别修改，单位明确为米。
- 两层 pitch 与偏置可分别修改，互不覆盖。
- 四个 Store 使用已存在的 YAML 内容，不新增坐标或回退姿态。
- 每个冻结目标携带自己的层与 Store，不依赖最后目标或第二抓时的遥测。
- 越界目标跳过，已有软件流程继续，安全门控不被绕过。
- A/C、Look、识别顺序、导航和二维码逻辑保持原样。
- 调试恢复参数名修正通过隔离失败路径检查。
- 运行环境、构建、现有数学检查及隔离检查有完整证据。
- 留下一项可复现的最小无硬件检查，不新增源码扫描或私有状态测试。
- 验证通过后提交任务文件与用户授权预设，保留其他改动。

当前证据状态：

- 已读取代码、配置、相关文档和启动前 Git diff。
- 已确认全部行为决策及预设提交授权。
- 本阶段未执行代码、未构建、未运行检查、未修改业务文件或提交。
- 上一轮 `4077807` 的构建与无运动验证仅属于历史单份 B pitch 功能。
  它们不是本次 Up/Down 功能的验证证据。
- 当前所有执行步骤、测试结果、提交与部署均未完成。

## 风险

1. 原始目标高度与最终 TCP 请求高度不同。
   Down 下限目标补偿后可到 `-0.015` m，仍须 planner 既有 IK/限位校验。
2. 四个预设有独立名称，但当前 Up/Down 同侧航点相同。
   本任务只负责正确选择，不能宣称已完成两层轨迹标定。
3. Store 使用 LIN 和途经点，结构校验不证明全路径可达、无碰撞或实机容差合适。
4. 层判断依赖正确外参、新鲜 TCP 遥测及静止观察姿态。
   不增加运动中手眼同步或历史姿态补偿。
5. B 配置键和内部返回值发生迁移。
   两个调用方必须一起更新、构建并重启，旧配置不能继续使用。
6. 运行中 planner 可能还保留旧预设名称。
   实机使用前必须确认重启或安全空闲时成功重载新 presets。
7. 当前调试恢复包含 `zero → home`，与正式 mission 的一次 Home 恢复不同。
   用户只批准参数名修正，不把此任务扩大为恢复策略重构。
8. 当前工作区已有未提交改动。
   四个 Store 已获提交授权，其他外部改动仍不得擅自纳入。
9. 浮点分类使用原始转换结果，不用日志四舍五入值或请求 float32 值判断。
   不以新增容差扩大允许高度范围。
10. 隔离 ROS 模拟只证明软件请求、分层和失败推进行为。
    它不证明串口装载顺序、STOP 优先级、硬件停稳或果实实际入筐。
11. 自动验证不驱动实机。
    安全现场的真实抓取、两层 Store 路径与低位空间验证仍需单独人工验收。
