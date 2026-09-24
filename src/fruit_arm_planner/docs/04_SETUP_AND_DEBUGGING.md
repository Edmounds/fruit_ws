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
colcon build --packages-select fruit_arm_planner
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

另开一个终端运行交互式笛卡尔控制台：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
# 启动时显示全部 20 Hz 状态与 Header/Mode：添加 --log-level debug
# 同时逐点显示轨迹：添加 --log-level trace
```

控制台命令为 `move X Y Z PITCH_DEG [DURATION_MS] [ptp|lin] [hold|open|close]`、
`status`、`stop`、`estop`、`level`、`help`、`quit`。例如 `move 0.20 0 0.25 0 2500 ptp hold`。
运行中输入 `level info`、`level debug` 或 `level trace` 可立即切换，不需要重启：

| 等级 | 输出 |
| --- | --- |
| `info`（默认） | 请求结果、状态变化、轨迹 ID 和错误码 |
| `debug` | `info` 加每帧 20 Hz 状态、轨迹 Header 和控制 Mode |
| `trace` | `debug` 加每个轨迹路点的 ID、时间、四轴弧度和夹爪动作 |

`ROS TX` 日志来自控制台对规划器输出话题的订阅，表示**观测到 ROS 消息**，
不表示通信桥已获得 ACK 或下位机已装载路点；不同话题的日志先后顺序也不是串口到达顺序。
`move` 会实际驱动机械臂；`status` 可检查最新四轴反馈和控制器状态。运动服务异步返回，
期间仍可输入 `stop` 或 `estop`。**在控制台按 Ctrl+C 会请求急停，再退出**；
控制台最多等待急停服务响应 2 秒。服务响应仅证明规划器发布了急停 Mode，
是否已到达下位机、是否进入 `EMERGENCY` 仍须观察状态。

规划器只提供运动规划节点。发出运动命令前，必须由现有硬件桥接系统提供
`/fruit_hardware_bridge/arm/status`；节点须收到四轴均有限、处于配置限位内且接收时间不超过
`status_max_age_sec` 的状态。没有状态时服务会拒绝运动，STOP 和急停仍可调用。
修改本包 Python 代码后需要重新构建并重新 source 运行环境；仅修改 YAML 时重启节点即可。

### 3. 正常启动控制台输出核对
当节点启动成功时，终端将打印加载的参数信息：
```text
[arm_planner_node-1] [INFO] [arm_planner_node]: Loaded arm geometry: d1=0.14m, a2=0.26m, a3=0.26m, a4=0.08m
[arm_planner_node-1] [INFO] [arm_planner_node]: Joint [q1] mapping: offset=0.0°, dir=+1.0, limits=[-180.0°, 180.0°]
[arm_planner_node-1] [INFO] [arm_planner_node]: Joint [q2] mapping: offset=-90.0°, dir=+1.0, limits=[-87.0°, 45.0°]
[arm_planner_node-1] [INFO] [arm_planner_node]: Joint [q3] mapping: offset=90.0°, dir=+1.0, limits=[-135.0°, 90.0°]
[arm_planner_node-1] [INFO] [arm_planner_node]: Joint [q4] mapping: offset=0.0°, dir=+1.0, limits=[-90.0°, 92.4°]
[arm_planner_node-1] [INFO] [arm_planner_node]: Loaded 4 arm presets: ['zero', 'home', 'retract', 'pick_sequence']
[arm_planner_node-1] [INFO] [arm_planner_node]: Fruit Arm Planner Node fully ready with Service API.
```

---

## 三、实车调参指南 (`arm_presets.yaml`)

现场更换机械件或出现动作偏差时，按以下步骤定位修改 `src/fruit_arm_planner/config/arm_presets.yaml`：

### 1. 机械连杆尺寸微调 (`geometry`)
若机械加工装配后尺寸发生变动，直接修改对应字段（单位: 米）：
```yaml
geometry:
  d1: 0.14       # 底座安装面至肩轴中心距
  a2: 0.26       # 肩轴至肘轴距
  a3: 0.26       # 肘轴至腕轴距
  a4: 0.08       # 腕轴至夹爪采摘作用点中心距
```

### 2. 实机零位偏置与方向校准 (`joint_mapping`)
- **若某关节旋转方向与预期相反（例如给正角反而向下运动）**：
  将该关节的 `direction` 由 `1.0` 修改为 `-1.0` 即可反相，无需拆改电机连线。
- **若机械臂安装初始零位发生变动**：
  修改 `offset_deg`。换算关系始终满足：
  $$q_{\text{hardware}} = \text{direction} \cdot q_{\text{algo}} + \text{offset\_deg}$$

### 3. 安全限位边界设定 (`limits_deg`)
`limits_deg: [min_deg, max_deg]` 对应物理电机的极限安全读数。
- `q2` 的最小限位 `-87.0°` 设定是为了防止大臂俯得过低磕碰车身顶盖；
- 如果现场发现手臂触碰避障结构，可适度收紧限位范围（例如将 `q2` 下限收紧至 `-75.0°`）。

### 4. 反馈和等待参数 (`planning`)

- `status_max_age_sec`：状态到达规划器后的最大可用时长，默认 0.25 秒。20 Hz 回报正常时约每 50 ms 更新一次。
- `execution_timeout_buffer_sec`：从发布启动 Mode 起，轨迹末点时间之外的等待余量，默认 2 秒。超时会发布 STOP，但该发布不等于固件确认停止。
- `interpolation_step_ms`：路点时间间隔目标值。每条轨迹均从最新实测角的 0 ms 路点开始；预设 `duration_ms` 为逐段时长。
- `mode_start_delay_ms`：发布全部路点后、发布 `Mode=TRAJECTORY` 前的调试等待时间，默认 100 ms。它只是时间缓冲，不能证明通信桥已经完成逐点发送或下位机已经装载全部路点；正式修复仍应由通信桥提供跨消息发送屏障。

配置缺失或几何、关节映射、预设的基本结构无效时，节点启动会报错退出。修改限位前应核实四轴实测角仍在新范围内，否则规划器会拒绝运动。

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
# 触发初始 Home 姿态复位 (大臂微后仰稳固重心)
ros2 service call /arm_planner_node/home std_srvs/srv/Trigger

# 触发底盘导航避障折叠姿态
ros2 service call /arm_planner_node/retract std_srvs/srv/Trigger

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

### 故障 0.1：服务超时或已发布轨迹但机械臂没有运行
- 记录服务返回的 `trajectory_id`，对照 `/fruit_hardware_bridge/arm/status` 的 ID、`task_status` 与 `error_code`。
- 规划器按 Header、全部 Point、Mode 顺序发布，但 ROS 话题及桥接层不同报文 ID 之间没有跨队列到达屏障；如果 Mode 在点收齐前到达，固件应报错，具体错误行为仍需现场核实。
- 超时 STOP 可能在桥接层排队；不要把返回超时当作电机已经停稳。

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
  若权限不足，需确认 udev 规则或临时执行 `sudo chmod 666 /dev/stm32`。

---

### 故障 2：终端报错 `Target rejected: Joint out of limits`
- **产生原因**：输入的关节角度超出了 `config/arm_presets.yaml` 中配置的 `limits_deg` 软限位。
- **解决方案**：观察终端打印的具体超限关节与超出数值，核实是否由于 `offset_deg` 设置错误，或目标点超出物理机械范围。

---

### 故障 3：发布空间坐标报错 `IK failed: 目标超出工作空间可达范围`
- **产生原因**：目标点 $(X, Y, Z)$ 离机械臂过远（大于 $a_2 + a_3 = 0.32\text{m}$）或过近（小于 $|a_2 - a_3| = 0.0\text{m}$），三角形无法闭合。
- **解决方案**：检查视觉相机测深坐标系转换是否正确，确认目标点处于机械臂有效工作半球包络面内。

---

### 故障 4：下位机上报 `error_code: 1002 (LIMIT_EXCEEDED)`
- **产生原因**：下位机单片机内部固件所烧录的限位阈值与上位机 YAML 不一致。
- **解决方案**：
  1. 打开 `docs/LOWER_CONTROLLER_COMMANDS.md` 核对下位机当前固件认可的物理限位；
  2. 保证 `arm_presets.yaml` 中的 `limits_deg` 严格处于下位机固件范围以内。
