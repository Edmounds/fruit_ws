# 当前 Git diff 审查

## 范围

- 阶段：只读 review。未执行代码、构建、测试、机械臂命令或业务文件修改。
- 文档目录：`/home/rc4/fruit_ws/.pi/plans/2026-10-05T15-20-26-764Z-8ea6ffec`。
- `plan.md` 不存在。`progress.md` 仅记载“尚未执行”。因此按当前 `workflow_git diff` 审查。
- 当前 diff 涉及 `src/fruit_arm_planner/config/arm_presets.yaml` 和 `src/fruit_mission/config/fruit_mission.yaml`。
- 当前 diff 与启动 baseline 中的两个配置 diff 一致。这些是既有脏改动，不是本次执行证据，不推断作者。
- 两次 status/diff 检查一致。未跟踪 skill 文件和图片不属于本次业务范围。
- 已读取 planner 配置加载、预设执行、轨迹拼接、FK/IK、关节映射、相机客户端、mission 观察/采摘调用链，以及现有相机偏置数学测试。
- 中文转译、安全恢复和 shell 提示虽见于当前源码，但不在工具返回的当前 diff 内。本次不将其作为被审查变更。
- `target_offset_b_m` 的三个值满足现有有限数输入契约。没有证据将现场调参判定为 P0/P1。左右 pitch 的物理正确性及轨迹无碰撞性仍需现场验证。

### 保留的用户决策与历史证据

以下来自对话，不扩大本次 diff 范围，也不代替文档或本轮验证：

1. 中文转译仅在 `fruit_mission` 客户端层进行，保留状态/错误代码。
2. 当前不修改轨迹传输、ACK 或启动时序。
3. 调试失败恢复要求为 STOP → 实测停稳且 IDLE → `zero` → `home`。恢复步骤失败时不得继续危险动作。
4. 用户曾确认执行。当前阶段重新限定为只读 review，修复须等待扩展审批。
5. 上轮对话报告提交 `5d33ba8`，涉及两个客户端和 `test_camera_arm_target.sh`。本轮工具未提供该提交详情，不将它当成本轮 diff。
6. 历史工具输出报告：Python compileall 成功，相机偏置数学测试 10 passed，`fruit_mission` 构建成功，两个客户端 targeted flake8 最终通过。
7. 历史测试没有证明真实 STOP、自动恢复、轨迹完整装载或无碰撞。本轮没有重跑测试。
8. 先前关于物理干涉、丢步和装载不全的根因叙述没有在本轮重读日志验证，不提升为已证实结论。

未完成：本轮 P1 尚未修复；真实中间姿态、右侧放果轨迹和现场验收尚需确认。

## P0

无。

## P1

### P1-1：B 区 Look 声明必须从伸直姿态进入，但轨迹没有实现该前置步骤

- **优先级**：P1，关键运动路径违反配置明确声明的前置条件。
- **位置**：`src/fruit_arm_planner/config/arm_presets.yaml:193-205`，`left_look_b` / `right_look_b`。
- **触发条件**：从 Home 等非伸直实测姿态调用 B 区 Look。独立相机客户端每次正常流程均先 Home 后 Look。
- **影响**：planner 直接从当前关节角插值到观察点，不经过配置声明必需的伸直姿态。该保护路径未实现，不能将此配置视为满足安全前置条件。没有证据断言必然发生碰撞，因此不列 P0。
- **证据**：
  - YAML 第 194、201 行明确写“必须从伸直状态切换到这个状态”。
  - 第 197、204 行的伸直中间点被注释，实际每个 Look 仅有一个目标。
  - `CameraArmTargetClient.run_sequence` 第 308-319 行先执行 `home`，随后直接执行 Look，没有正常流程的伸直步骤。
  - `MissionContext.start_observation` 第 676-682 行允许已确认 Home 或已确认 shenzhi 作为起点，不保证每次都是 shenzhi。
  - `TrajectoryPlanner.plan_multi_waypoint_sequence` 第 192、199、255-264 行从实测 `current_q_hw` 开始，默认 PTP，只按实际 waypoints 插值。它不读取 description，也不自动补中间点。
- **归属说明**：该配置改动已在 baseline 中存在。不归因于本次执行。

### P1-2：新增 B 区右侧观察预设，但对应放果预设仍缺失

- **优先级**：P1，右侧采摘的放果关键路径必然失败。
- **位置**：`src/fruit_arm_planner/config/arm_presets.yaml:200-220`，新增 `right_look_b`，仅定义 `left_store_b`，没有 `right_store_b`。
- **触发条件**：使用 B 区右侧流程，观察和目标 CLOSE/HOLD 均成功，随后请求放果。
- **影响**：planner 返回 `Unknown preset 'right_store_b'`，不能执行右侧放果 OPEN/dwell。恢复 Home 不等于果实入筐，右侧采摘工序无法完成。
- **证据**：
  - 已完整读取 YAML。其末尾只有 `right_look_b` 和 `left_store_b`，没有右侧 Store 定义。
  - `CameraArmTargetClient.run_sequence` 第 370-378 行以 direction/zone 拼出并执行 `right_store_b`，失败即进入恢复并退出。
  - `actions._grasp_and_collect` 使用 `f"{side}_store_{area.lower()}"`，正式任务亦依赖同名预设。
  - `ArmPlannerNode._srv_execute_preset_cb` 第 572-574 行对不存在的名称直接返回失败，不提供别名或跨区回退。
  - 接口文档要求补齐实际使用的预设，不允许用假姿态或 A 区姿态替代。
- **归属说明**：缺失右侧 Store 并非本次执行删除。当前新增右侧 Look 的配置仍未组成可完成的右侧工序。

## 修复步骤

以下仅为审批后的修复步骤。当前不执行。

1. 先请操作者确认 B 区伸直过渡的真实 TCP、pitch 和逐段时长。不要直接取消注释并沿用未验证的 200ms。
2. 在 `src/fruit_arm_planner/config/arm_presets.yaml` 的两个 B 区 Look 中加入确认过的伸直中间 waypoint。保留现有观察终点和夹爪 HOLD。优先复用现有 `shenzhi` 的几何语义，由共享 FK/IK 核对，不能另写 IK 或绕过 planner。
3. 请操作者提供真实右侧入筐/张爪/驻留/退出轨迹。在同一 YAML 补齐 `right_store_b`。不要推测镜像坐标、复制 A 区预设或添加空轨迹。
4. 若右侧工序不在用户实际使用范围，先请求明确缩小范围的确认。不得静默删掉右侧支持以规避问题。
5. 同步更新 `src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md`、`04_SETUP_AND_DEBUGGING.md`、`05_PHYSICAL_ARM_MAPPING_REVIEW.md` 中受影响的姿态/验证说明。必要时更新 `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md` 的 B 区使用说明。
6. 用下面的离线数学规划检查验证配置。它只加载 YAML 和纯算法，不创建 ROS 节点，不发布运动。
7. 运行现有数学测试及差异检查。通过后，仅在有人监护、底盘停稳、扫掠空间清空时进行手动硬件验收。
8. 保存验证证据后，只提交已确认归属的修复文件/片段。保留原有未相关配置改动，不提交他人的修改。

### 审批后离线验收命令

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 - <<'PY'
import math
from pathlib import Path
import yaml
from fruit_arm_planner.kinematics import RobotKinematics
from fruit_arm_planner.joint_transformer import JointTransformer
from fruit_arm_planner.trajectory_planner import TrajectoryPlanner

path = Path('src/fruit_arm_planner/config/arm_presets.yaml')
config = yaml.safe_load(path.read_text(encoding='utf-8'))
arm = config['arm_planner_node']['ros__parameters']
kin = RobotKinematics.from_geometry_dict(arm['geometry'])
transformer = JointTransformer.from_config_dict(arm['joint_mapping'])
planner = TrajectoryPlanner(kin)
presets = arm['presets']
planning = arm['planning']

# Synthetic mathematical starts only. Never use these as measured motion feedback.
reference_hw = transformer.link_angles_to_hardware((0.0, 0.0, 0.0, 0.0))

def plan(name, start):
    preset = presets[name]
    ok, points, reason = planner.plan_multi_waypoint_sequence(
        start, preset['waypoints'], transformer, preset['coordinate_frame'],
        planning['interpolation_step_ms'], planning['default_dwell_ms'])
    assert ok, (name, reason)
    assert len(points) >= 2 and points[0].joints == start
    assert points[0].time_from_start == 0 and points[0].gripper_action == 0
    assert all(p.point_id == i for i, p in enumerate(points))
    assert all(a.time_from_start < b.time_from_start
               for a, b in zip(points, points[1:]))
    assert all(transformer.check_hardware_limits(p.joints)[0] for p in points)
    return points

home_hw = plan('home', reference_hw)[-1].joints
extended_hw = plan('shenzhi', home_hw)[-1].joints
extended_tcp = kin.forward_kinematics(
    *transformer.hardware_to_algo(extended_hw))['end_effector']
for side in ('left', 'right'):
    look_name = f'{side}_look_b'
    store_name = f'{side}_store_b'
    look = presets[look_name]
    assert look['coordinate_frame'] == 'cartesian'
    assert len(look['waypoints']) >= 2, look_name
    first = look['waypoints'][0]
    target = first['target']
    assert math.dist(target[:3], extended_tcp[:3]) <= 0.001, look_name
    assert abs(target[3] - math.degrees(extended_tcp[3])) <= 0.5, look_name
    assert first.get('gripper', 0) == 0
    for start in (home_hw, extended_hw):
        plan(look_name, start)
    look_hw = plan(look_name, home_hw)[-1].joints
    plan(store_name, look_hw)
    assert any(wp.get('gripper') == 1 for wp in presets[store_name]['waypoints'])
print('B preset mathematical checks passed; no motion commanded')
PY
/usr/bin/python3 -m pytest -q src/fruit_mission/test/test_camera_target_transform.py
git diff --check -- src/fruit_arm_planner/config/arm_presets.yaml \
  src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md \
  src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md \
  src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md \
  src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
```

上面的 TCP 容差用于复用现有 shenzhi 的方案。若用户确认不同的伸直保护姿态，先记录其依据并调整验收参考。不能以放宽容差掩盖姿态差异。

## 验收

- 本轮结果：P0 0 项，P1 2 项。仅静态阅读证据，没有运行验证。
- 离线检查期望：两个 B 区 Look 均有有效伸直过渡，两侧 Store 均可规划并含 OPEN。所有轨迹满足首点、限位、点号和递增时间契约。
- 现有偏置数学测试期望：10 项通过。它不证明新姿态、右侧入筐或自动恢复正确。
- diff 检查期望：无 whitespace 错误。没有相关行为以外的变更。
- 实机验收必须由操作者手动进行，不作为自动化测试。确认 Home → 伸直过渡 → B 区 Look 的实际路径，再确认右侧 CLOSE → Store OPEN/dwell → Home。
- 对每次运动记录 `trajectory_id`、服务结果和实测反馈。不得把发布成功、规划通过或固定延时当作执行完成。
- 实机验收未通过前，不声称路径无碰撞或右侧采摘工序可运行。
- 不修改串口时序，不新增策略重试，不清除 EMERGENCY，不移除输入校验或 STOP 保护。
- 待扩展询问是否修复。审批前不执行修复、测试或提交。
