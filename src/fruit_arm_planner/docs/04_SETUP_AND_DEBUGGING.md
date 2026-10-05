# 04. 启动流程、配置与调试

本文档面向现场调试工程师与操作人员，提供 `fruit_arm_planner` 的**环境准备、编译启动、`arm_presets.yaml` 调参校准指南、CLI 快速调试命令集以及常见故障排查手册**。

---

## 目录
- [一、环境准备与依赖构建](#一环境准备与依赖构建)
- [二、Launch 启动文件与参数生命周期](#二launch-启动文件与参数生命周期)
- [三、实车调参指南 (arm_presets.yaml)](#三实车调参指南-arm_presetsyaml)
- [四、命令行快速调试指令大全](#四命令行快速调试指令大全)
- [五、现场常见故障排查手册 (Troubleshooting)](#五现场常见故障排查手册-troubleshooting)

---

## 一、环境准备与依赖构建

根据项目工作区规范，本项目统一使用 CycloneDDS 中间件，并隔离系统环境。每次打开新终端调试前，**必须首先 source 统一环境脚本**：

```bash
# 1. 注入统一运行环境 (设置 CycloneDDS 与系统库路径)
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

# 2. 编译本功能包
cd /home/rc4/fruit_ws
colcon build --base-paths src --packages-up-to fruit_arm_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

---

## 二、Launch 启动文件与参数生命周期

本包启动文件为 `src/fruit_arm_planner/launch/fruit_arm_planner.launch.py`，严格遵循项目根目录的 `template.launch.py` 模版：

### 1. 配置文件加载规则
根据项目规范：**“节点/launch 加载配置文件不要使用 install 位置，直接指定 src 源码目录的位置”**。
启动脚本内部直接锁定：
```python
common_config = '/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml'
```
这意味着：**现场修改 YAML 参数后，无需重新执行 `colcon build`，直接重启 launch 文件即可立即生效！**

### 2. 标准启动命令
```bash
ros2 launch fruit_arm_planner fruit_arm_planner.launch.py
```

**实机联调一键启动桥和规划器**（只在调试时使用；不要与已运行的通信桥重复启动）：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 launch fruit_arm_planner fruit_arm_debug.launch.py
# 串口设备不同可传 port:=/dev/ttyUSB0
```

另开一个终端运行交互式机械臂控制台：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
# 启动时显示全部 20 Hz 状态与 Header/Mode：添加 --log-level debug
# 同时逐点显示轨迹：添加 --log-level trace
```

每次启动自动在 `src/fruit_arm_planner/test/log/` 新建日志，文件名使用机器本地时间，
格式为 `YYYY-MM-DD_HH-MM-SS_微秒.log`；启动时会显示完整路径，不覆盖已有文件。
控制台使用系统自带的 `/usr/bin/script` 记录完整终端会话：标准输出、错误输出、
异常栈、命令回显和 jog 界面均保留，终端显示照常，每次写入后刷新日志。
日志是原始终端记录，含 ANSI 光标/颜色控制序列；日志目录已被 Git 的 `log/` 规则忽略。
日志按当前 `info/debug/trace` 等级记录实际输出，不额外开启被该等级隐藏的遥测。
仅记录控制台进程，不自动收集另一个终端中的 planner/串口桥输出。
如果无法创建或打开日志，则报错退出，不启动机械臂客户端，也不静默降级成无日志运行。

控制台通过规划器服务规划并立即执行以下运动命令：

| 命令 | 含义与示例 |
| --- | --- |
| `move X Y Z PITCH_DEG [DURATION_MS] [ptp|lin] [hold|open|close]` | 笛卡尔目标（m、绝对俯仰角度）；例如 `move 0.20 0 0.25 0 2500 ptp hold` |
| `joints Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]` | 连杆目标角度（度）：Q2 相对竖直（后仰为正），Q3 为小臂相对水平面的绝对角，Q4 为末端相对水平面的绝对俯仰角。控制台按 YAML 偏置换算后下发；例如 `joints 0 20 -40 0 2500 hold` |
| `motors Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]` | 直接输入四个原始电机硬件角（度），不再叠加偏置；沿用旧版 `joints` 的语义 |
| `preset NAME` | 执行源码 YAML 中存在的预设，例如 `preset zero`；当前名称为 `zero`、`left_look`、`right_look`、`pick_sequence`、`home`、`left_store`、`right_store`，现场可调 |
| `status` | 显示同一帧原始电机角、逆映射模型角、连杆角，以及真实 TCP 的 XYZ（m）和绝对 pitch（度） |
| `jog` | 进入终端内键盘微调和示教打点界面，按键直接执行运动 |
| `reload` | 调用 `/arm_planner_node/reload_presets`，重新加载源码 YAML 的全部 presets |

#### 键盘微调和示教打点

先确认周围安全且实测反馈有效，在 `arm>` 输入 `jog`。它使用 PTP 服务运动，不保证末端走直线或路径无碰撞。

| 按键 | 行为 |
| --- | --- |
| ↑ / ↓ | 累计 X 增加 / 减少 1 cm |
| ← / → | 累计 Y 增加 / 减少 1 cm（向左 / 向右） |
| `a` / `z` | 累计 Z 增加 / 减少 1 cm |
| `s` / `x` | 末端绝对俯仰角增加 / 减少 5°，XYZ 目标不变，由 IK 联动关节 |
| `c` / `o` | 原地闭合 / 张开夹爪，当前点记录动作分别为 `gripper: 2` / `gripper: 1` |
| `h` | 当前点记录 HOLD (`gripper: 0`)，取消尚未下发的夹爪动作；不撤销已下发动作，也不张开夹爪 |
| 空格 | 到位后弹出终端输入框，输入 preset 名称；Enter 保存，Esc 取消 |
| `r` | 无运动时热重载全部 presets |
| `t` / `e` | STOP / ESTOP，清除待执行目标；恢复后退出并重新进入 jog |
| Esc | 清除待执行目标并退出；若仍有运动则请求 STOP |
| Ctrl+C | 请求急停并退出控制台 |

进入时使用 planner 最新遥测的真实 TCP 作为初始 XYZ 和俯仰角，不在控制台再次逆 mapping 或 FK。空闲时按键自动提交运动；运动期间继续累加目标，完成后自动执行最新目标，**不逐键排队、不打断当前轨迹**。每次提交使用运行中 Planner 的 `default_duration_ms`，位置/俯仰运动的夹爪动作为保持。界面区分实测位置、累计目标、执行目标、待执行夹爪动作及当前点记录动作；不可达、越限、执行失败或停止后清除所有待执行动作、不自动续发，须检查原因并重新进入模式。

`c/o` 复用 `move_joints` 服务，以最新有效四轴实测硬件角（转为度）作为目标，只改变夹爪动作，不再 IK 或叠加 Cartesian offset。服务起点仍由 Planner 获取实测角，反馈与执行有时间差，不应视为硬件锁轴。夹爪动作在默认时长轨迹的末点触发，并非按键瞬间完成。运动中按 `c/o` 会合并为最新待执行夹爪动作；当前动作结束后先原地执行夹爪动作，再继续累计移动。`h` 取消尚未下发的夹爪动作，已提交的请求不受它撤销。

初始记录标签为 HOLD。`c/o` 成功执行后标签保留，供空格记录；按 `h` 或开始下一次位置/俯仰点动后标签恢复 HOLD，后续移动不会反复发送闭合/张开。不按键不持续发送命令，也不会自动重置刚完成的 `c/o` 标签。该标签是动作意图，不证明夹爪已抓住果实，实测 `gripper_state` 单独显示。

移动键和夹爪键不会写文件。空格只在全部累计运动和夹爪请求完成后可用，冻结当前有效实测位置和姿态，再输入 preset 名称：已有 preset 按其 `cartesian` / `joints` / `link_angles` 类型转换后追加，不存在则新建 Cartesian preset。航点时间读取源码 YAML 的 `planning.default_duration_ms`，`gripper` 保存当前点的动作标签；不额外写入 `dwell_ms`，闭合/张开航点回放沿用 `planning.default_dwell_ms`。原有配置和注释不整体重写，文件写入失败不覆盖原文件。保存后自动请求热重载；若重载失败，文件仍已保存，处理错误后执行 `reload`。

点动显示及记录的是统一 FK 模型的真实夹爪抓取点 TCP 坐标；提交服务和保存 Cartesian preset 使用同一坐标，不再加减输入偏置。终端弹窗期间不执行运动键；Esc 取消和保存均不会执行该 preset。

`reload` 只刷新全部 presets，运动期间或校验失败时保留旧内存内容，不修改几何、关节映射及 planning。调整默认时长、偏置或机械参数后重启 Planner 和控制台，保证双方使用同一份配置；升级本次 Python 代码也需构建并重启 Planner。点动时长是计划运动时间，不含通信及启动等待，不代表固定按键执行频率。

可直接用 ROS CLI 重载预设（不会驱动机械臂）：

```bash
ros2 service call /arm_planner_node/reload_presets std_srvs/srv/Trigger '{}'
```

`DURATION_MS=0` 或省略时使用规划器默认时间；两种关节命令默认夹爪保持。当前标定下，`joints 0 0 0 0` 会换算成约 `[0, 1, -13, -5]°` 的硬件目标；标定修改后以源码 YAML 实际数值为准。`motors 0 0 0 0` 则原样下发全零硬件角。`status` 显示原始电机角、planner 提供的逆映射模型角、由模型角换算的连杆角及真实 TCP。还支持 `stop`、`estop`、`level`、`help`、`quit`。
预设的 `coordinate_frame: link_angles` 使用与 `joints` 相同的四项定义，末端目标为 `0°` 时腕部自动抵消小臂倾角；`coordinate_frame: joints` 保留原始电机角定义。折叠姿态也须校验腕部映射后是否超限，不能复制旧限位数值；`home` 快捷服务仅在同名预设存在时可用，以当前源码 YAML 为准。

控制台现在订阅 `/arm_planner_node/joint_states` 的 `fruit_arm_interfaces/msg/ArmTelemetry`，不再以原始硬件 Status 自行计算 FK。debug 每帧显示 `motors_deg/model_deg/tcp_m/pitch_deg`，jog 和示教使用同一帧校正模型角及 TCP；原地夹爪和 `joints` 硬件角预设保存仍使用该帧原始硬件角，避免二次偏置。消息类型变更必须一起构建接口、planner 和 mission 并重启消费者；旧 standard JointState 订阅者不兼容，话题名称保持不变。

运行中输入 `level info`、`level debug` 或 `level trace` 可立即切换，不需要重启：

| 等级 | 输出 |
| --- | --- |
| `info`（默认） | 请求结果、状态变化、轨迹 ID 和错误码 |
| `debug` | `info` 加每帧 20 Hz 状态、轨迹 Header 和控制 Mode |
| `trace` | `debug` 加每个轨迹路点的 ID、时间、四轴弧度和夹爪动作 |

`ROS TX` 日志来自控制台对规划器输出话题的订阅，表示**观测到 ROS 消息**，
不表示通信桥已获得 ACK 或下位机已装载路点；不同话题的日志先后顺序也不是串口到达顺序。
`move`、`joints`、`motors`、`preset` 都会实际驱动机械臂；`status` 可检查最新四轴反馈和控制器状态。运动服务异步返回，
期间仍可输入 `stop` 或 `estop`。**在控制台按 Ctrl+C 会请求急停，再退出**；
控制台最多等待急停服务响应 2 秒。服务响应仅证明规划器发布了急停 Mode，
是否已到达下位机、是否进入 `EMERGENCY` 仍须观察状态。

规划器只提供运动规划节点。发出运动命令前，必须由现有硬件桥接系统提供
`/fruit_hardware_bridge/arm/status`；节点须收到四轴均有限、处于配置限位内且接收时间不超过
`status_max_age_sec` 的状态。没有状态时服务会拒绝运动，STOP 和急停仍可调用。
修改本包 Python 代码后需要重新构建，并重启已运行的规划节点；它从 `install/fruit_arm_planner` 加载 Python 包，不会因 `src` 文件或安装文件更新而自动替换进程内的代码。控制台在启动时从自身旁边的 `config/arm_presets.yaml` 读取关节映射，因此调整偏置后应重启规划节点和控制台，保证二者使用同一版标定；新终端仍须 source 统一运行环境。

笛卡尔 `pitch_deg=0` 时，不能直接用硬件角 `joint_3 + joint_4` 判断末端是否水平：先按配置将 Q3/Q4 分别换回理论角，俯仰角应为 `(joint_3 - q3.offset_deg) / q3.direction + (joint_4 - q4.offset_deg) / q4.direction`（各项单位一致）。当前 YAML 两轴方向均为 `+1`、偏置为 `-6°` 和 `-5°`，所以 `joint_3 + joint_4` 应为 `-11°`。修改 YAML 后需要重启规划节点；修改 Python 后还需重构建。

### 3. 正常启动控制台输出核对
启动日志应列出 geometry、各轴 offset/direction/limits、已加载 preset 名称和 Service API ready。
逐项与当前源码 YAML 对照，不拿历史日志的几何、限位或 preset 名称作固定验收值。
节点 ready 只证明构造完成；实际运动仍要求新鲜有效的四轴反馈。

---

## 三、实车调参指南 (`arm_presets.yaml`)

现场更换机械件或出现动作偏差时，按以下步骤定位修改 `src/fruit_arm_planner/config/arm_presets.yaml`：

### 1. 机械连杆尺寸微调 (`geometry`)
若机械尺寸发生变动，修改对应字段（单位: 米）。以下为本次核对的配置示例，现场测量与源码 YAML 优先：
```yaml
geometry:
  d1: 0.062      # 底座安装面至肩轴中心距
  a2: 0.26       # 肩轴至肘轴距
  a3: 0.26       # 肘轴至腕轴距
  a4: 0.12       # 腕轴沿末端局部 X 方向的伸出长度，pitch=0 时此段水平
  tool_z_offset_m: -0.03  # 真实抓取点的末端局部 Z 偏移，下方为负，随腕部旋转
```

`tool_z_offset_m` 是预设 YAML 内可调的固定结构尺寸，不是世界坐标平移，也不是腕电机的零位角偏置。`pitch=0°` 表示 12 cm 伸出段水平，XYZ 指定下方 3 cm 的真实抓取点；腕轴到抓取点的斜线不用于定义 pitch。直接输入、PTP/LIN、笛卡尔预设、jog/FK/示教和 mission 共用同一模型。旧 `planning.cartesian_offset_m` 输入补偿已移除，避免重复补偿。

修改后重新启动 planner、控制台及 mission/相机调试客户端；仅 `reload_presets` 不会刷新几何。现有预设航点不自动改写或迁移，旧模型下记录的坐标需由操作者核对。

以下最小检查仅离线执行数学解算，不启动控制节点、不发送运动请求；先完成构建：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 - <<'PY'
import math
import yaml
from fruit_arm_planner.kinematics import RobotKinematics
with open('/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml') as stream:
    geometry = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']['geometry']
kinematics = RobotKinematics.from_geometry_dict(geometry)
target = (0.25, 0.0, 0.25)
ok, joints, message = kinematics.inverse_kinematics(*target, pitch_deg=0.0)
if not ok:
    raise RuntimeError(message)
pose = kinematics.forward_kinematics(*joints)
assert math.dist(pose['end_effector'][:3], target) < 1e-9
assert abs(pose['points'][4][2] - pose['points'][3][2]) < 1e-9
print('TCP (m, rad):', pose['end_effector'], 'wrist (m):', pose['points'][3])
PY
```

当前 12 cm / -3 cm 几何下，此水平目标的 TCP 为 `(0.25,0,0.25)m`，腕轴为 `(0.13,0,0.28)m`。这是数学检查，不验证实机标定或关节安全限位。

### 2. 实机零位偏置与方向校准 (`joint_mapping`)
- **若某关节旋转方向与预期相反（例如给正角反而向下运动）**：
  将该关节的 `direction` 由 `1.0` 修改为 `-1.0` 即可反相，无需拆改电机连线。
- **若机械臂安装初始零位发生变动**：
  修改 `offset_deg`。Q1、Q2、Q4 满足 $q_{\text{hardware}} = \text{direction} \cdot q_{\text{algo}} + \text{offset}$；Q3 电机角与小臂绝对俯仰角对应，满足 $q_{3,\text{hardware}} = \text{direction}_3 \cdot (q_{2,\text{algo}} + q_{3,\text{algo}}) + \text{offset}_3$。反馈计算相对肘角时，先减 Q3 偏置，再减大臂理论角。控制台 `joints` 和 YAML 中 `link_angles` 预设会按偏置换算；ROS `move_joints` 服务、控制台 `motors` 和 YAML 中 `joints` 预设则直接使用硬件角。更多角度关系见[四轴角度与解算链路核查](05_PHYSICAL_ARM_MAPPING_REVIEW.md)。

### 3. 安全限位边界设定 (`limits_deg`)
`limits_deg: [min_deg, max_deg]` 对应物理电机的极限安全读数。
- `q2` 的最小限位 `-87.0°` 设定是为了防止大臂俯得过低磕碰车身顶盖；
- 如果现场发现手臂触碰避障结构，可适度收紧限位范围（例如将 `q2` 下限收紧至 `-75.0°`）。

### 4. 反馈和等待参数 (`planning`)

- `status_max_age_sec`：状态到达规划器后的最大可用时长，当前配置为 0.25 秒。20 Hz 回报正常时约每 50 ms 更新一次。
- 运动执行无限等待匹配轨迹 ID 的下位机完成/报错；主动 STOP/ESTOP 或节点退出可中断。`execution_timeout_buffer_sec` 已从配置与加载校验删除，mission 也不再设置前端运动超时。端点发现和反馈新鲜度仍独立校验。
- `interpolation_step_ms`：路点时间间隔目标值。每条轨迹均从最新实测角的 0 ms 路点开始；预设 `duration_ms` 为逐段时长。
- `default_dwell_ms`：预设夹爪动作路点未指定 `dwell_ms` 时的默认驻留时间。
- `mode_start_delay_ms`：发布全部路点后、发布 `Mode=TRAJECTORY` 前的调试等待时间，默认 100 ms。它只是时间缓冲，不能证明通信桥已经完成逐点发送或下位机已经装载全部路点；正式修复仍应由通信桥提供跨消息发送屏障。

节点启动时从源码目录的 `arm_presets.yaml` 读取几何、四轴偏置/方向/限位、规划时间参数与预设；缺少必需项或基本结构无效时启动失败，不回退到 Python 中的机械参数。修改限位前应核实四轴实测角仍在新范围内，否则规划器会拒绝运动。

---

## 四、命令行快速调试指令大全

在单独调试机械臂节点时，可打开另一终端执行以下 CLI 测试命令：

下列运动命令会使实机动作，现场确认机械臂周围安全、硬件桥接状态正常后再执行。
正式任务使用三个运动 service；`cmd_*` 话题只供调试，没有同步结果。并发运动请求会收到忙错误。

```bash
# 激活环境
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

# ========================================================
# 1. 基础服务调用（预设姿态触发）
# ========================================================
# 执行现有基准预设；home 快捷服务仅在配置了同名预设后可用
ros2 service call /arm_planner_node/execute_preset fruit_arm_interfaces/srv/ExecuteArmPreset \
  "{preset_name: 'zero'}"

# 触发平滑减速停止 (mode=1)
ros2 service call /arm_planner_node/stop std_srvs/srv/Trigger

# 触发紧急急停 (mode=2)
ros2 service call /arm_planner_node/emergency_stop std_srvs/srv/Trigger

# ========================================================
# 2. 预设名称话题触发
# ========================================================
ros2 topic pub --once /arm_planner_node/cmd_preset std_msgs/msg/String "{data: 'zero'}"

# ========================================================
# 3. 笛卡尔空间坐标采摘测试 (高精度直线 vs 快速过渡)
# ========================================================
# 【高精度直线模式】沿空间直线精准刺入果实 (linear_path: true)
ros2 service call /arm_planner_node/move_cartesian fruit_arm_interfaces/srv/MoveArmCartesian \
  "{x: 0.20, y: 0.0, z: 0.25, pitch_deg: 0.0, duration_ms: 2500, gripper_action: 1, linear_path: true}"

# 【快速转移模式】关节平滑过渡到悬停位 (linear_path: false)
ros2 service call /arm_planner_node/move_cartesian fruit_arm_interfaces/srv/MoveArmCartesian \
  "{x: 0.20, y: 0.0, z: 0.25, pitch_deg: 0.0, duration_ms: 2000, gripper_action: 0, linear_path: false}"

# 异步话题触发 (默认使用关节平滑过渡)
ros2 topic pub --once /arm_planner_node/cmd_cartesian geometry_msgs/msg/PointStamped \
  "{point: {x: 0.20, y: 0.0, z: 0.25}}"

# ========================================================
# 4. 多动作序列预设服务调用
# ========================================================
# 触发复合采摘工序 (从当前位置自动过渡 -> 执行序列)
ros2 service call /arm_planner_node/execute_preset fruit_arm_interfaces/srv/ExecuteArmPreset \
  "{preset_name: 'pick_sequence'}"

# ========================================================
# 5. 关节物理角度手动下发
# ========================================================
# 下发 [0°, 0°, -30°, 0°] (弧度: [0, 0, -0.5236, 0])，运动耗时 2500ms
ros2 topic pub --once /arm_planner_node/cmd_joints sensor_msgs/msg/JointState \
  "{position: [0.0, 0.0, -0.5236, 0.0], effort: [2500.0]}"

# ========================================================
# 6. 遥测状态实时打印监视
# ========================================================
# 监听 20Hz 实时物理角度与速度
ros2 topic echo /arm_planner_node/joint_states

# 核对底层 20 Hz 状态和轨迹 ID；仅 joint_states 不能证明动作完成
ros2 topic echo /fruit_hardware_bridge/arm/status
```

---

## 五、现场常见故障排查手册 (Troubleshooting)

### 故障 0：服务返回 `Fresh valid four-joint feedback is required` 或 `Arm is not idle`
- 确认 `/fruit_hardware_bridge/arm/status` 持续更新，四关节角均为有限值且处于 YAML 限位内。
- 检查 `task_status`；运动中或急停锁定时不能发新运动。急停后按固件协议发送 STOP 解锁，并等待新鲜的 IDLE 回报。
- 调用 STOP 的服务成功只表示 Mode 已交给 ROS 发布器；必须观察后续状态确认硬件已经返回空闲。

### 故障 0.1：服务持续等待或已发布轨迹但机械臂没有运行
- 记录服务返回的 `trajectory_id`，对照 `/fruit_hardware_bridge/arm/status` 的 ID、`task_status` 与 `error_code`。
- 规划器按 Header、全部 Point、Mode 顺序发布，但 ROS 话题及桥接层不同报文 ID 之间没有跨队列到达屏障；如果 Mode 在点收齐前到达，固件应报错，具体错误行为仍需现场核实。
- 执行没有自动超时 STOP；需要人工中断时使用 STOP/ESTOP。停止命令可能在桥接层排队，不要把服务返回当作电机已经停稳。

### 故障 1：发布指令后机械臂毫无反应
- **排查步骤 1**：检查底层串口通信桥 `auto_serial_bridge` 是否已启动：
  ```bash
  ros2 topic list | grep fruit_hardware_bridge
  ```
  确认是否存在 `/fruit_hardware_bridge/arm/trajectory_header` 和 `/fruit_hardware_bridge/arm/status`。
- **排查步骤 2**：检查硬件串口设备 `/dev/stm32` 权限：
  ```bash
  ls -l /dev/stm32
  ```
  `/dev/stm32` 是部分项目脚本要求的设备别名，当前 protocol YAML 的 port 为 `/dev/ttyACM0`；
  检查实际 launch 使用的设备，不要只查别名。权限不足时核对 udev 和用户所属组，不默认开放全员写权限。

---

### 故障 2：终端报错 `Target rejected: Joint out of limits`
- **产生原因**：输入的关节角度超出了 `config/arm_presets.yaml` 中配置的 `limits_deg` 软限位。
- **解决方案**：观察终端打印的具体超限关节与超出数值，核实是否由于 `offset_deg` 设置错误，或目标点超出物理机械范围。

---

### 故障 3：发布空间坐标报错 `IK failed: 目标超出工作空间可达范围`
- **产生原因**：先按末端 pitch、`a4` 和 `d1` 换算腕部中心到肩轴的距离；其须在 $|a_2-a_3|$ 到 $a_2+a_3$ 之间，不能把末端到基座的距离直接和旧 `0.32 m` 常量比较。当前 a2/a3 均为 0.26 m，其和为 0.52 m，实际可运动范围还受关节限位影响。
- **解决方案**：检查视觉相机测深坐标系转换是否正确，确认目标点处于机械臂有效工作半球包络面内。

---

### 故障 4：下位机上报 `error_code: 1002 (LIMIT_EXCEEDED)`
- **产生原因**：下位机单片机内部固件所烧录的限位阈值与上位机 YAML 不一致。
- **解决方案**：
  1. 核对实际固件认可的物理限位；[协议参考](../../../docs/LOWER_CONTROLLER_COMMANDS.md) 只定义错误码，不提供已实测固件限位；
  2. 保证 `arm_presets.yaml` 中的 `limits_deg` 严格处于下位机固件范围以内。
