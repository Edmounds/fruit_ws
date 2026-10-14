# ACB 路点预设：确认与交接

## 2026-10-14：当前 mission 约定

**本次操作者确认覆盖下方首轮交接中的旧路线。**
当前实现和入口见 [fruit_mission README](../src/fruit_mission/README.md#独立-acb-入口)。

- 当前模板与实际文件均包含 32 个点，不再使用首轮 35 点骨架。
- A 左侧去程开环，右侧回程闭环，C 区闭环。
- 提前扫 B 使用已有 `qr_scan_a_to_b`，从 `shenzhi` 进入，成功后直接接 `right_look_a`。
- C 作业后到 `C_top_waypoint_2`，成功到点才汇总 A+C，不清零、不说“任务流程结束”。
- 然后沿 C 图到 `C_entrance_waypoint_2`，经 `to_B_lower_waypoint_1` 转场。
- B 使用此前保存的计划，按 B1→B8 作业，奇数左、偶数右。
- B8 后直接结束，不再次汇总，不增加返航或卸货。
- 汇总点失败跳过汇总。操作者选择在出口或返 B 通道失败后仍继续既定 B 路线。
- 继续流程不绕过既有安全门控，也不表示失败后的路径已经通过实机安全验收。
- 本次不修改任何操作者标定预设，不启动硬件。

以下正文保留首轮路点交接历史，不再作为当前 mission 的执行依据。

## 1. 状态与范围

**路点预设已并入 `main`，RobViz Maker 和录点器现已支持 ACB 文件切换。**
新 mission 和现场录点尚未实施。未启动真实底盘、机械臂或自动导航。
新模板共 35 点。实际文件包含 13 个 C 点，另 22 点待录制。

目标是在保留原 ABC 预设的前提下，新增 A → C → B 路点预设。
路点与 mission 分开维护。另一位负责人编写新 mission。

| 负责人 | 职责 |
| --- | --- |
| 路点与录点负责人 | 新增路点文件、模板、录点参数和编辑器后端文件登记 |
| 操作者 | 录制 A/B 停车点和通道点，标定右侧 B 扫码姿态 |
| 新 mission 负责人 | 任务顺序、区域切换、扫码计划缓存、观察侧、播报和失败推进 |

路点任务包含：

- 新增独立 ACB 路点文件和模板。
- 通过启动参数选择 ABC 或 ACB 录点预设。
- 让独立航点编辑器能够读写新文件。
- 保留原有 C 区标定及图规划所需结构。
- 更新相关使用说明。

路点任务不包含：

- 修改 `src/fruit_mission/` 的任务逻辑或启动入口。
- 实现 B 区观察侧选择或二维码任务重排。
- 修改 Maker 文件选择之外的 RobViz 前端功能。
- 修改 `src/fruit_arm_planner/config/arm_presets.yaml`。
- 实现物理卸货、人工卸货等待或返回起点。
- 启动真实底盘、机械臂或自动导航。

## 2. 已确认的作业流程

```text
A_left1 → A_left2 → A_left3 → A_left4，每点只抓车体左侧
→ 向前到 B_QR，车体不转，机械臂向右扫码并保存 B 计划
→ 返回右侧回程入口
→ A_right4 → A_right3 → A_right2 → A_right1，每点只抓车体右侧
→ 前往 C_QR，扫码并按原有 C 区图规划作业
→ 沿 C 区图到 C_top_waypoint_1，播报 A+C 累计采摘数量
→ 经下方通道进入 B8
→ B8 → B7 → B6 → B5 → B4 → B3 → B2 → B1
→ 在 B1 结束，播报全程累计采摘数量
```

“四个”表示四个观察位置，不表示必须成功采摘四个目标。
局部观察、扫码或采摘失败不补抓、不反复重试。

**最终结束位置随 B 区新顺序改为 B1。**
此前讨论过的“B8 结束”属于旧方向，不能用于新 mission。

`C_top_waypoint_1` 是操作者确认的中途播报位置。
这里只播报，不执行物理卸货，不清空累计采摘统计。
该位置是否满足赛事收集更换区判定，仍需现场核对。

## 3. 文件与数据约定

已新增文件：

```text
src/navigation/fruit_nav_bringup/waypoints/template_acb.yaml
src/navigation/fruit_nav_bringup/waypoints/odom_waypoints_acb.yaml
```

保留旧文件，不覆盖其内容：

```text
src/navigation/fruit_nav_bringup/waypoints/template.yaml
src/navigation/fruit_nav_bringup/waypoints/odom_waypoints.yaml
```

- 保持 `schema_version: 1`。
- 实际路点文件使用 `frame_id: odom`。
- 模板只规定名称、类型和录点排列，不编码采摘动作。
- 坐标、四元数和 yaw 表示底盘停车位姿，不表示机械臂姿态。
- 模板排列不等于 mission 执行顺序。
- 不增加左右观察侧字段。该信息由新 mission 维护。

新实际路点文件初始只复制原文件中的 C 区点位。
未录制的 A/B、扫码转场和通道点只列入模板，不写入实际坐标文件。
禁止使用零坐标、猜测坐标或未经确认的旧坐标补齐。
不完整文件可以用于继续录点，不能作为完整比赛路线交付。

### 3.1 模板骨架

| 分段 | 点名及排列 | 类型 | 坐标来源 |
| --- | --- | --- | --- |
| A 左侧 | `A_left1`、`A_left2`、`A_left3`、`A_left4` | `goal` | 独立录制 |
| B 扫码转场 | `to_B_QR_waypoint` | `waypoint` | 重新录制 |
| B 扫码位置 | `B_QR` | `goal` | 重新录制 |
| A 右侧回程入口 | 暂定 `to_A_right_waypoint_1` | `waypoint` | 现场确认后录制 |
| A 右侧 | `A_right4`、`A_right3`、`A_right2`、`A_right1` | `goal` | 独立录制 |
| 前往 C | `to_C_waypoint_1` | `waypoint` | 重新录制 |
| C 区 | 见下方完整列表 | 保持原类型 | 复制原 C 标定 |
| 下方返回 B | 暂定 `to_B_lower_waypoint_1`、`to_B_lower_waypoint_2` | `waypoint` | 现场确认后录制 |
| B 作业点 | `B8`、`B7`、`B6`、`B5`、`B4`、`B3`、`B2`、`B1` | `goal` | 全部重新录制 |

通道点名称和数量为暂定骨架，不表示安全通道已经验证。
现场录制前，按实际拐点增删通道点，并同步交接给新 mission 负责人。
同名的非 C 点不表示可复用旧坐标。

### 3.2 A/B 编号与朝向

- `A_leftN` 与 `A_rightN` 对应同一纵向位置编号。
- 左右停车点独立录制，不能通过反向访问左侧点替代右侧点。
- A 去程和右侧回程保持相同车体朝向。
- 右侧回程实际倒车，不把停车 yaw 录成掉头后的朝向。
- 导航通过路径方向和目标 yaw 判断倒车。
- B1～B8 继续对应原规则图中的实际 1～8 号树。
- B8 使用二维码第八行，B1 使用第一行。
- B 区停车坐标、yaw 和逐树观察侧均重新确定。
- 不复制旧 B 标定，不沿用旧 mission 的左右交替规则。

## 4. C 区自动导航兼容条件

保持以下 C 区点的名称、类型和相对排列顺序：

```text
C_QR                    goal
C_entrance_waypoint_1    waypoint
C_8_12                  goal
C_7_11                  goal
C_6_10                  goal
C_5_9                   goal
C_top_waypoint_1        waypoint
C_top_waypoint_2        waypoint
C_1_5                   goal
C_2_6                   goal
C_3_7                   goal
C_4_8                   goal
C_entrance_waypoint_2    waypoint
```

- 从操作者确认的原 C 标定快照复制坐标、四元数和 yaw。
- 新旧流程必须使用相同 Point-LIO 启动位置和朝向。
- 保留两条 C 区通道及两端连接，不改现有固定图拓扑。
- 保持 `C_top_waypoint_1` 为 `waypoint`，不因播报用途改成 `goal`。
- 新 mission 到达 `C_QR` 后继续使用原有扫码和 C 区图规划能力。
- C 区按二维码任务顺序作业，不按模板排列作业。
- 从任意最后采摘位置沿图规划到 `C_top_waypoint_1`。
- 不从随机最后采摘点直接驶向返 B 通道。

当前参考实现位于：

- [`waypoint_route.py`](../src/fruit_mission/fruit_mission/waypoint_route.py)：C 区图规划与观察侧计算。
- [`mission_context.py`](../src/fruit_mission/fruit_mission/mission_context.py)：`navigate_c_position()` 和 `navigate_c_waypoint()`。
- [`actions.py`](../src/fruit_mission/fruit_mission/actions.py)：`scan_c_qr()`。
- [`actions.py`](../src/fruit_mission/fruit_mission/actions.py)：当前共用 `run_c()`。

路点任务只保证数据兼容，不单独承诺新 mission 的 C 自动导航已经通过验收。

## 5. 录点与编辑器的最小改动

录点入口已实现。仅在操作者允许启动硬件后执行：

```text
./waypoints_maker.sh --route abc
./waypoints_maker.sh --route acb
```

默认仍选择 ABC。选择 ACB 时，同时绑定新路点文件、新模板和自动名称顺序。

| 位置 | 已实施改动 |
| --- | --- |
| `waypoints_maker.sh` | 解析预设参数，传递匹配的路点、模板和自动名称文件 |
| `src/fruit_robviz_bridge/launch/waypoint_recorder.launch.py` | 转发 `template_file` 和 `source_root`，保留旧默认值 |
| `src/fruit_robviz_bridge/fruit_robviz_bridge/robviz_web_server.py` | 登记 `odom_acb` 和 `template_acb`，复用现有文件列表 |
| `src/fruit_robviz_bridge/fruit_robviz_bridge/waypoint_files.py` | 让模板几何参考兼容 `A_left1`，保留旧 `A1` 行为 |

HTTP 后端从已有 `waypoint_file`、`template_file` 的目录定位两份新文件。
无需新增配置项，也不接受客户端传入任意文件路径。
录制 `A_left1` 前，ACB 模板的相对几何参考为空，不填入零坐标。

网页“航点 Maker”从录点器状态读取名称列表。
文件下拉框支持 ABC、ACB 和调试文件，模板随正式文件一起切换。
录点和保存通过 ROS 消息执行，保存路径由录点器当前加载文件决定。
独立“航点编辑器”通过 HTTP 后端读写文件，两者不能混为同一配置入口。

切换预设前保存当前录点结果。
启动脚本选择新预设时，停止录点器后用对应参数重新启动。
网页 Maker 可在录点和导航空闲时，通过“文件”下拉框切换。
未保存草稿需要确认丢弃，加载失败保留原草稿和模板。
不必为切换预设重启 Point-LIO。
已有录点器的保存文件、模板或自动名称文件不匹配时，提示退出，不静默复用。
参数读取最多等待 10 秒。读取失败时退出，不启动第二个录点器。
若模板有增删，重新加载对应录点模板后再录点。

保留现有名称和类型校验、未保存提示、版本冲突保护、备份及原子保存。
不新增网页按钮，不重构前端，不引入多步预设切换状态机。
正式文件计数使用匹配模板总数，初始 ACB 显示 `13/35`。

## 6. 新 mission 负责人交接项

以下内容是新 mission 的职责，不在本次路点实施范围内：

1. 显式加载 ACB 文件，并校验所需点已录制完整。
2. 不把新文件直接交给旧 `full` 入口。
3. 分别维护 A 左侧去程和右侧回程。
4. 在 B 扫码位置调用操作者标定的 `qr_scan_b_right`。
5. 保存本轮 B 计划，执行 C 后仍使用同一计划。
6. 提前扫 B 码失败时，沿用配置的固定兜底计划。
7. 返回 B 区时不追加扫码重试。
8. 扫 B 码后，返回 A 右侧采摘前切回 A 感知区域。
9. C/B 作业前分别切换到匹配的感知区域。
10. 根据新停车 yaw 维护 B8→B1 的逐树观察侧。
11. 按实际位置编号匹配二维码品种，不按访问游标错配。
12. 中途播报只汇总 A+C，不播报“任务流程结束”。
13. B1 结束时播报 A+B+C 累计数量。
14. C 出口或中途播报点导航失败时，记录失败并播报累计数量。
15. 不宣称到达失败目标，不重试该目标。
16. 后续 B 导航继续受安全条件限制，不能从未知位置直接穿越场地。
17. 无法安全进入返 B 通道时，跳过相关运动和采摘。

当前扫码调用名称与操作者配置需要核对。
已讨论的姿态名称为 `qr_scan_b`、`qr_scan_c` 和新增 `qr_scan_b_right`。
右侧姿态由操作者标定，不由路点任务猜测或覆盖。
机械臂仍通过 [`fruit_arm_planner` 服务](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md)控制。

当前 `announce_summary()` 包含结束播报，不能直接当作中途汇总调用。
局部失败继续推进，但软件推进不等于运动许可。
保留 STOP、反馈、运动占用及单次安全 Home 恢复边界。

### 比赛规则冲突

操作者确认新流程实际按原编号 B8→B1 采摘。
这与当前 [`RULE.md`](RULE.md) 记录的 B 区 1→8 作业顺序冲突。
本方案不修改规则文档，也不把倒序采摘描述成符合原规则。
比赛使用前，由操作者与 mission 负责人核对裁判解释。
不得通过改名或错配二维码行掩盖顺序冲突。

## 7. 验收与待办

### 路点与工具的离线验收

- [x] 旧 ABC 文件、默认路点文件和默认模板保持不变。
- [x] 新模板与新实际路点文件相互匹配。
- [x] 初始文件只包含已确认 C 标定，不包含假坐标。
- [x] 预设选择同时绑定保存文件、模板和自动名称。
- [ ] 新名称可在原网页 Maker 中选择、录制和保存。
- [x] 独立编辑器 HTTP 接口可读取和保存新文件、新模板。
- [x] 新文件保存复用版本冲突、备份和原子写入保护。
- [x] 复制后的 C 点坐标、四元数、yaw 和类型与确认快照一致。
- [x] 原 C 图规划可访问全部十二个位置。
- [x] 检查 C 位置间转移、观察侧计算和到 `C_top_waypoint_1` 的路径。

已通过 27 项数学与通信检查，以及两项相关包构建。
五份受影响 Python 文件通过 `ament_flake8`，录点脚本通过语法和非法参数检查。
13 个 C 起点到十二个位置共 156 组规划，与原文件逐项比较一致。
这些比较包含路径、距离、到达 yaw、观察侧和采摘后出口规划。
隔离 ROS 域检查使用临时文件，确认录点状态显示 35 个模板名称。
该检查确认下一录点名称为 `A_left1`，保存仅修改临时 ACB 文件并生成备份。
没有发送导航目标，也没有连接硬件。
网页按钮操作、真实 TF 采样和实机录点仍需操作者验证。

数学和通信检查不得启动真实导航或机械臂。
离线检查通过不等于实机安全验收。

### 操作者与新 mission 联调前待办

- [ ] 确认各段实际拐点，增删暂定通道点并同步 mission。
- [ ] 录制 A 左右八个独立停车点。
- [ ] 重新录制 `B_QR`、B1～B8 和各段通道点。
- [ ] 标定 `qr_scan_b_right`。
- [ ] 完成 B 区逐树观察侧维护。
- [ ] 确认 Point-LIO 起始位置和朝向与原 C 标定一致。
- [ ] 新 mission 在启动运动前拒绝不完整路点配置。
- [ ] 核对比赛规则冲突和中途播报位置。
- [ ] 在安全条件下验证倒车、转弯和机械臂扫掠空间。

当前导航没有自动避障或整机碰撞检查。
模板中的通道点不能替代现场安全验证。

## 8. Worktree 交接说明

首轮实现使用独立 worktree `.worktrees/acb-waypoint-handoff/`，现已合并到 `main` 并清理。
日常运行使用主目录 `/home/rc4/fruit_ws`。
新建 worktree 时，它仍不自动包含主目录未提交改动。
本次从主目录当前 C 标定复制数据，并核对 13 点完全一致。
后续联调前仍需核对操作者是否更新了 C 标定。

录点脚本按自身目录选择路点文件，并加载已构建的 worktree overlay。
独立维护的串口桥仍使用主目录中的原 launch 文件。
导航和运行环境仍会复用主目录 underlay、硬件配置与运行日志位置。
联调前核对实际代码、配置、备份和保存路径。
不要通过覆盖主目录文件来绕过隔离。

### 8.1 离线构建与检查

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_nav_bringup fruit_robviz_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
bash -n waypoints_maker.sh
PYTHONPATH="$PWD/src/fruit_robviz_bridge:$PWD/src/fruit_mission:$PWD/src/fruit_qr_planner:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q \
  src/fruit_robviz_bridge/test/test_recorder.py \
  src/fruit_robviz_bridge/test/test_waypoint_protocol.py \
  src/fruit_robviz_bridge/test/test_runtime_protocol.py \
  src/fruit_mission/test/test_c_waypoint_route.py
```

### 8.2 新建 Worktree 时的编辑器配置

原 Web 配置仍指向主目录。只加载 overlay 不会改变数据路径。
主目录现有后端已登记两份 ACB 文件，正常使用不需要以下临时配置。
在 worktree 验证编辑器时，生成临时配置，不修改主目录或前端。
以下命令只生成配置，不启动硬件：

```bash
cd /home/rc4/fruit_ws/.worktrees/acb-waypoint-handoff
source scripts/setup_fruit_runtime.bash
web_config=$(/usr/bin/python3 - <<'PY'
from pathlib import Path
import tempfile
import yaml

root = Path.cwd()
config = yaml.safe_load((root / 'src/fruit_robviz_bridge/config/robviz_web.yaml').read_text())
directory = root / 'src/navigation/fruit_nav_bringup/waypoints'
for key, name in [('waypoint_file', 'odom_waypoints.yaml'),
                  ('debug_waypoint_file', 'debug_waypoints.yaml'),
                  ('template_file', 'template.yaml')]:
    config[key] = str(directory / name)
config['backup_dir'] = str(root / 'tmp/acb_editor_backups')
config['image_settings_file'] = str(root / 'tmp/acb_image_settings.json')
config['arm_config_path'] = None
with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
    yaml.safe_dump(config, stream, allow_unicode=True)
    print(stream.name)
PY
)
```

仅在联调授权后启动该后端。先确认 8000 端口没有旧后端占用。
临时配置禁用机械臂编辑，并把航点备份和图像设置写入 worktree。

```bash
PYTHONPATH="$PWD/src/fruit_robviz_bridge:$PYTHONPATH" \
  /usr/bin/python3 -m fruit_robviz_bridge.robviz_web_server --config "$web_config"
```

在独立航点编辑器选择 `odom_waypoints_acb.yaml` 或 `template_acb.yaml`。
网页 Maker 的保存路径仍由录点器决定，不能通过编辑器文件下拉框切换。
停止临时后端后删除 `$web_config`。

当前交付包含 YAML、工具支持、通信和数学检查。
现场录点、新 mission 和整机联调仍未实施。
