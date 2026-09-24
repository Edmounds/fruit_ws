# 下位机通信协议与控制指令

新串口协议将机械臂运动学逆解与轨迹规划交给上位机，不再定义旧的笛卡尔目标点（`ArmTarget`）与离散状态机任务（`StateMachineCommand`）。当前 `fruit_mission` 仍依赖旧消息和回调，尚未迁移到规划器服务。
整机通信采用 `auto_serial_bridge` 框架在单串口（默认 `/dev/stm32`，波特率 115200）上进行二进制通信。

---

## 消息定义概览

| 消息名 | ID | 方向 | Reliable | 对应 ROS 话题 | ROS 消息类型 | 功能说明 |
| :--- | :---: | :---: | :---: | :--- | :--- | :--- |
| **ChassisVelocity** | `0x01` | TX | 否 | `cmd_vel` | `geometry_msgs/msg/Twist` | 底盘开环速度控制（angular_z 沿用取反逻辑） |
| **TrajectoryHeader** | `0x10` | TX | 是 | `fruit_hardware_bridge/arm/trajectory_header` | `fruit_hardware_interfaces/msg/TrajectoryHeader` | 轨迹声明头包（包含任务 ID 与航路点总数） |
| **TrajectoryPoint** | `0x11` | TX | 是 | `fruit_hardware_bridge/arm/trajectory_point` | `fruit_hardware_interfaces/msg/TrajectoryPoint` | 单个航路点包（含时间戳、4关节角度与夹爪动作） |
| **ArmControlMode** | `0x12` | TX | 是 | `fruit_hardware_bridge/arm/control_mode` | `fruit_hardware_interfaces/msg/ArmControlMode` | 控制模式切换（0: 轨迹启动, 1: 平滑停止, 2: 硬件急停） |
| **ArmStatus** | `0x20` | RX | 否 | `fruit_hardware_bridge/arm/status` | `fruit_hardware_interfaces/msg/ArmStatus` | 下位机 20Hz (50ms) 周期上报的综合状态数据包 |

---

## 控制模式与时序规范

### 1. 轨迹下发与启动时序

1. **下发头部声明**：上位机向话题 `fruit_hardware_bridge/arm/trajectory_header` 发送 `TrajectoryHeader`，包含 `trajectory_id`（唯一任务编号，如 1001）与 `point_num`（该轨迹包含的总路点数 N）。
2. **逐点传输**：上位机向话题 `fruit_hardware_bridge/arm/trajectory_point` 依次发送 N 个 `TrajectoryPoint` 包（`point_id` 从 0 到 N-1）。
   - 每个点包含：`time_from_start` (ms), `joint_1` ~ `joint_4` (rad), `gripper_action` (0=保持, 1=张开, 2=闭合)。
3. **启动执行**：下位机接收缓冲完毕后，上位机向话题 `fruit_hardware_bridge/arm/control_mode` 发布 `ArmControlMode(control_mode=0)`（TRAJECTORY）触发电机开始执行。

### 2. 平滑停止与硬件急停

- **正常平滑停止 (STOP, control_mode: 1)**：
  电机按照加减速曲线平滑减速停机，清除运行中状态与未走完的轨迹缓存，复位为 `IDLE`。
- **硬件紧急停止 (ESTOP, control_mode: 2)**：
  最高优先级：立即切断 PWM / 驱动脉冲，电机锁死，进入 `EMERGENCY` 锁定状态。
- **急停解除恢复**：
  必须先向 `control_mode` 发送 `1` (STOP) 指令才能解除 EMERGENCY 状态；解锁后电机保持就地位置闭环，清空残留轨迹，状态复位为 `IDLE`。

### 3. 运行中互锁与断联机制

- **运行中互锁**：严禁在机械臂运动中直接覆盖下发新轨迹。如需变轨或终止，必须先发送 `ArmControlMode(1)` (STOP) 停稳回到 IDLE 后，再下发新轨迹。
- **机械臂与底盘互斥**：采摘机械臂运动时，底盘保持静止；底盘导航移动时，机械臂保持收纳姿态。

### 4. 上位机轨迹规划与解算节点 (`fruit_arm_planner`)

ROS 2 运行环境中，上位机通过 `fruit_arm_planner` 节点自动接管轨迹插值与下发：
- **关节偏置解耦 (`joint_mapping`)**：将算法空间（水平伸直为 0）与下位机硬件空间（电机零位）双向映射：
  $$q_{\text{hardware}} = \text{direction} \cdot q_{\text{algo}} + \text{offset}$$
- **平滑插值**：采用五次多项式（S-Curve）插值，保证首尾速度和加速度均为 0。
- **自动时序执行**：外部只需向 `/arm_planner_node/cmd_joints`、`/arm_planner_node/cmd_cartesian` 发布目标或调用 `/arm_planner_node/home` 等服务，规划器自动完成限位检查、插值并分步下发 Header、Points 与 Mode。

---

## 下位机 20Hz 综合状态包 (`ArmStatus`)

下位机每隔 50ms（20Hz）主动向 ROS 端发送一帧 `ArmStatus`（ID: 0x20）：

- `trajectory_id` (u32): 当前执行或刚完成的轨迹任务 ID
- `task_status` (u8):
  - `0`: 空闲 (IDLE)
  - `1`: 正在执行 (RUNNING)
  - `2`: 执行完成 (COMPLETED)
  - `3`: 失败/超时 (FAILED)
  - `4`: 急停锁定 (EMERGENCY)
- `progress` (f32): 执行进度百分比 (0.0 ~ 100.0 %)
- `joint_1` ~ `joint_4` (f32): 4 个关节当前物理实测角度 (rad)
- `joint_1_velocity` ~ `joint_4_velocity` (f32): 4 个关节当前瞬时角速度 (rad/s)
- `gripper_state` (u8): 0=保持/未知, 1=张开, 2=闭合
- `error_code` (u16):
  - `0`: 正常无错误
  - `1001`: 任务超时报警
  - `1002`: 关节角度超软限位
  - `1003`: 急停保护触发

---

## 下位机交互测试控制台 (`lower_controller_console.py`)

控制台通过 `auto_serial_bridge` 发布的 ROS 2 标准话题与下位机进行通信，具备更高的底层通信健壮性（CRC 校验、可靠重传机制与统一的心跳管理）：
- **自动检测与复用**：启动时自动检测系统中是否已有运行中的 `/serial_controller`。若有则直接复用；若无则自动在后台拉起 `serial_bridge_by_node.launch.py` 桥接进程。
- **默认 DEBUG 彩色日志输出**：控制台默认以 `log_level:=debug` 启动桥接节点，并在终端实时以彩色流式呈现底层收发/心跳/通信调试日志，同时保证 `readline` 输入行刷新不被冲乱。
- **严格读取协议配置**：串口设备路径与波特率直接从 `src/auto_serial_bridge/config/protocol.yaml` 严格读取（启动时同时校验必须包含所需 5 条关键消息定义），控制台不暴露串口命令行覆盖参数。
- **安全与生命周期管理**：退出控制台（或按两次 `Ctrl-C`）时，自动发送平滑停机指令并清理终止 `serial_node` 进程，确保硬件串口立即释放。

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 /home/rc4/fruit_ws/lower_controller_console.py [--log-level debug|info|warn|error] [--yes] [--no-spawn] [--no-color]
```

### 1. 4 自由度机械臂安全范围与硬限位保护

| 关节 | 说明 | 软件安全限位范围 (deg) | 对应弧度范围 (rad) |
| :--- | :--- | :---: | :---: |
| **Q1 (Base)** | 机械臂底座偏航旋转 | `[-180.0°, +180.0°]` | `[-3.1416, +3.1416]` |
| **Q2 (Shoulder)** | 大臂俯仰（避让车身结构） | `[+3.0°, +180.0°]` | `[+0.0524, +3.1416]` |
| **Q3 (Elbow)** | 小臂内夹角（负角定义） | `[-190.0°, -35.0°]` | `[-3.3161, -0.6109]` |
| **Q4 (Wrist)** | 腕部俯仰舵机 | `[-90.0°, +92.4°]` | `[-1.5708, +1.6127]` |

控制台在下发任何轨迹前均强制执行上述限位硬拦截，超限指令直接拒绝发送。

### 2. 控制台指令速查

- **安全预设动作**（默认 3000ms 平滑执行，带二次确认提示）：
  - `home`: 安全初始复位位姿 `[0°, 110°, 50°, 0°]`，夹爪保持
  - `retract`: 底盘避障收纳位姿 `[0°, 110°, 150°, 0°]`，夹爪保持
  - `grip`: 夹爪闭合（`gripper_action=2`）
  - `open`: 夹爪张开（`gripper_action=1`）
- **自定义轨迹与模式**：
  - `traj <q1> <q2> <q3> <q4> [action] [time_ms]`: 自定义 4 关节轨迹点（角度制，带限位校验）
  - `mode <0|1|2>`: 手动切换模式（0: TRAJECTORY, 1: STOP, 2: ESTOP）
  - `estop`: 最高优先级硬件急停（模式 2）
- **底盘速度与停止**：
  - `vel <linear_x> <angular_z>`: 发送底盘开环速度（angular_z 自动取反匹配偏航）
  - `stop`: 平滑减速停机（发送 ArmControlMode(1) 并置零底盘速度）
- **状态监控与保护**：
  - `status`: 打印最新 20Hz `ArmStatus` 的完整物理角度、速度、进度与故障码
  - 遥测 20Hz 高频包在后台静默解析，仅在任务状态变更或报警时回显
  - 运行中按 `Ctrl-C` 立即触发平滑停机，2 秒内再次按 `Ctrl-C` 退出控制台
