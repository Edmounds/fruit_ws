# 机械臂规划器通信架构与数据流

本文档从 mission 发出运动请求，一直追踪到下位机执行并返回结果。它同时说明当前代码实际做了什么、双方已确认的目标行为，以及现有协议无法证明的事情。本文档不修改串口协议或其他功能包。

## 1. 阅读约定与依据

| 标记 | 含义 |
| --- | --- |
| 已确认 | 本项目当前确定的架构与业务约束，规划器后续应满足 |
| 已实现 | 可以从当前仓库代码直接确认的行为 |
| 待核实 | 尚无代码内证据，需现场或跨包确认的行为 |
| 待核实 | 仓库里没有下位机业务实现或实机证据，不能当作已实现能力 |

字段和报文 ID 的权威定义是 [`auto_serial_bridge/config/protocol.yaml`](../../auto_serial_bridge/config/protocol.yaml) 及其[生成的协议说明](../../auto_serial_bridge/mcu_output/PROTOCOL_DOC.md)；ROS 接口定义以 [`fruit_arm_interfaces/srv`](../../fruit_arm_interfaces/srv) 和 [`fruit_hardware_interfaces/msg`](../../fruit_hardware_interfaces/msg) 为准。规划器现状以 [`arm_planner_node.py`](../fruit_arm_planner/arm_planner_node.py) 和 [`trajectory_planner.py`](../fruit_arm_planner/trajectory_planner.py) 为准。生成的 MCU 文件只包含协议层和业务回调桩，不代表真实下位机轨迹执行代码。

## 2. 系统边界

```text
相机/坐标转换模块 -- 已转换的目标 xyz,pitch --> mission
                                            |
                       关节目标 / 笛卡尔目标 / 预设名称
                                            v
                                    fruit_arm_planner
                       状态检查 -> IK/映射 -> 插值 -> 执行监护
                          ^                 |            |
                          |                 | ROS 消息    | ROS 服务结果
                          |                 v            v
                    ArmStatus         auto_serial_bridge  mission
                          ^                 |
                          |              串口帧、CRC、ACK/重传
                          |                 v
                          +------------ 下位机
                               轨迹缓存、关节执行、状态上报
```

- **已确认**：车上只有安装在机械臂末端的一台深度相机。相机标定、拍摄时姿态和相机坐标到机械臂根部坐标的转换由其他模块负责。规划器只接受机械臂根部固定坐标系中的目标；`x/y/z` 为米，`pitch_deg` 为末端连杆相对水平面的俯仰角。底座偏航由目标 `x/y` 决定，不独立输入末端滚转或偏航。逆解只采用肘上构型。
- **已确认**：mission 只发三类运动请求：关节单目标、`x/y/z + pitch` 单目标、预设名称。预设的路点及逐段时间由规划器自己的 YAML 决定；mission 不提交任意路点数组。旧的 `StateMachineCommand`、`ArmTarget` 和左右侧摄像头流程不属于新架构。
- **已实现**：`fruit_hardware_bridge` 当前主要提供串口桥的启动入口；规划器直接在 ROS 话题上对接 `auto_serial_bridge`，并不存在另一个负责轨迹排序的语义中间节点。
- **边界**：下位机接受新轨迹协议并执行关节及夹爪动作。下位机执行逻辑、缓存容量、重复报文处理及硬件保护的具体实现仍需下位机负责人确认。本轮规划器优化不修改通信桥、消息包或下位机协议。

## 3. 上位机内部接口

### 3.1 mission 到规划器

三个正式运动入口都是 ROS 2 service。服务返回 `success=true` 的预期含义是**与本次请求对应的轨迹已经执行完成**，不是“已经受理”或“已经发送”。在等待期间 STOP 和急停应仍然可以触发。

| 服务 | 请求 | 规划器处理 | 响应 |
| --- | --- | --- | --- |
| `/arm_planner_node/move_joints` | `float32[4] joints_deg`，`uint32 duration_ms`，`uint8 gripper_action` | 目标是下位机物理关节角，度转弧度并做关节空间插值 | `bool success`、`string message`、`uint32 trajectory_id` |
| `/arm_planner_node/move_cartesian` | `float32 x/y/z`，`float32 pitch_deg`，`uint32 duration_ms`，`uint8 gripper_action`，`bool linear_path` | 先用肘上 IK 求目标物理角；`false` 为关节空间 PTP，`true` 为逐点 IK 的笛卡尔 LIN | 同上 |
| `/arm_planner_node/execute_preset` | `string preset_name` | 从规划器 YAML 读取路点，按每段 `duration_ms` 顺序规划 | 同上 |

`duration_ms=0` 在两个单目标服务中表示使用包内默认时长；它不是“立即到位”。`gripper_action` 的现有取值是 `0=保持`、`1=张开`、`2=闭合`。运动服务的 `trajectory_id=0` 可用于表达尚未下发就被拒绝的请求；具体错误原因放在 `message` 中。

`/arm_planner_node/home` 和 `/arm_planner_node/retract` 是 `std_srvs/Trigger` 快捷入口，实际调用对应预设。`/arm_planner_node/stop` 和 `/arm_planner_node/emergency_stop` 也是 `Trigger`；当前返回成功只表示规划器发布了命令，**不表示下位机已停稳或已进入急停态**。建议后续把这一差别在服务响应和运行日志中明确写出。

规划器还保留 `/arm_planner_node/cmd_joints`、`cmd_cartesian`、`cmd_preset` 三个话题作为调试入口。它们没有逐请求结果返回；mission 应使用 service，不能把“话题已发布”当成动作完成。`cmd_cartesian` 的 `PointStamped.header.frame_id` 目前没有被代码检查或变换，直接使用 `point` 数值；正式业务不应借这个话题传相机坐标。

串口桥另外发布 `auto_serial_bridge/ready`（`std_msgs/msg/Bool`，Reliable + Transient Local）。它表示桥接节点当前认为串口通信可用，可用于上位机启动检查；它不表示下位机有新鲜关节角、不表示轨迹缓存空闲，也不证明任何路点已装载。规划器仍需独立检查 `ArmStatus`。

### 3.2 下位机状态到规划器、mission

串口桥把下位机 `ArmStatus` 发布到 `/fruit_hardware_bridge/arm/status`。规划器订阅它，缓存四关节物理角，并向 `/arm_planner_node/joint_states` 发布 `sensor_msgs/msg/JointState`。`JointState` 是遥测，mission 的业务完成判定应以本次 service 的响应为准，避免每个上层模块重复实现轨迹 ID 和终态匹配。

**已确认的起点要求**：四个关节都有真实角度回传。三类运动请求必须先取得最近一帧新鲜、有效的四关节实测角；缺失、过期、NaN、Inf 或超出可信范围的关节数据不能静默替换成零或历史值。用于判断“新鲜”的时间应是规划器收到该状态的本机单调时钟时间；阈值应放在包内配置，不能仅以“曾收到过一帧”为充分条件。STOP/急停不依赖这项起点检查。

## 4. 规划器的轨迹计算契约

### 4.1 共同起点与时间轴

不论请求类型，下发给下位机的**第一个 `TrajectoryPoint`**均必须满足：

```text
point_id = 0
time_from_start = 0 ms
joint_1..4 = 本次规划开始时冻结的最新有效 ArmStatus.joint_1..4
gripper_action = HOLD
```

冻结的状态快照必须用于整次规划，不能规划一半又把后续 20 Hz 遥测混入同一轨迹。关节单目标和笛卡尔单目标的 `duration_ms` 是从 `t=0` 到目标的时间。预设 YAML 中每个路点的 `duration_ms` 是**从上一目标路点到当前目标路点的时长**，不是绝对时间；规划器将它们累加为下位机使用的 `time_from_start`。

例如预设第一段 500 ms、第二段 800 ms：

| 关键点 | 下位机时间 | 关节位置 |
| --- | ---: | --- |
| 自动补入的起点 | 0 ms | 请求开始时的实测 `q_current` |
| 第一个 YAML 目标 | 500 ms | 第一个目标 `q_1` |
| 第二个 YAML 目标 | 1300 ms | 第二个目标 `q_2` |

两段之间可以有插值点；跨段拼接时 `point_id` 必须连续，时间必须严格向前。当前多段实现会把上一段终点和下一段起点都放在同一个时间戳，这是包内待修正项。过短的段时长也可能让取整后的中间点拥有相同毫秒时间，需要在生成前验证。

### 4.2 三种计算路径

| 路径 | 计算步骤 | 关键约束 |
| --- | --- | --- |
| 关节单目标 | `joints_deg` 转物理弧度，核验终点限位，从实测 `q_current` 以五次多项式插值到目标 | 第 0 点是原始实测值，末点执行指定夹爪动作 |
| 笛卡尔 PTP | 目标 `x/y/z/pitch` 只做一次肘上 IK，映射到物理关节角，再走关节空间插值 | 终点可达不代表末端路径无碰撞；该模式适合已确认安全的空间转移 |
| 笛卡尔 LIN | 用实测角做 FK 得到当前末端位置与俯仰角；在笛卡尔直线各采样点做肘上 IK、映射和限位核验 | 第 0 个下发点仍必须是原始实测关节角，不能由 FK 后再次 IK 的近似值替换 |
| 预设 | 从 YAML 读取 `coordinate_frame`、各 `target`、逐段 `duration_ms` 和夹爪动作，拼接为一条轨迹；`link_angles` 会按配置映射到硬件角，其中第四项为末端绝对俯仰角 | 每段起点来自上一段终点；`joints` 仍是原始硬件角；`linear` 等配置只有实现了才可宣称有效 |

关节映射和物理角限位均在规划器包内完成：Q1/Q2/Q4 各自使用 `q_hardware = direction * q_algo + offset`，Q3 使用 `q3_hardware = direction * (q2_algo + q3_algo) + offset`，其反馈映射相应减去 `q2_algo`。下位机协议中的四个关节角一律是**物理关节弧度**，其中 Q3 是小臂相对水平面的绝对角；mission 的 `move_joints` 输入则是物理关节**角度**。这是两个不同单位边界。现有 YAML 几何、偏置和限位属于待实机标定参数，不能把当前数值写成永久机械常量。

### 4.3 发送前的一次性检查

规划器在任何报文发布之前检查整条轨迹：关节角有限、起点等于冻结的实测角、物理限位有效、`point_id=0..N-1` 连续、时间从 0 开始并严格递增、`N` 可装入 `uint16 point_num`、`gripper_action` 在 0..2。**下位机实际缓存上限尚未从仓库确认**；当前只检查协议字段上限，不能把 `65535` 当成固件容量，也不能沿用旧文档未经证实的“128 点”断言。

## 5. ROS 话题到串口帧

### 5.1 桥接的输入输出

| 方向 | ROS 话题 | ROS 类型 | 报文 ID | 串口可靠传输 |
| --- | --- | --- | ---: | --- |
| 规划器 -> 桥 | `/fruit_hardware_bridge/arm/trajectory_header` | `TrajectoryHeader` | `0x10` | 是 |
| 规划器 -> 桥 | `/fruit_hardware_bridge/arm/trajectory_point` | `TrajectoryPoint` | `0x11` | 是 |
| 规划器 -> 桥 | `/fruit_hardware_bridge/arm/control_mode` | `ArmControlMode` | `0x12` | 是 |
| 桥 -> 规划器 | `/fruit_hardware_bridge/arm/status` | `ArmStatus` | `0x20` | 否，约 20 Hz 周期上报 |

规划器的三个发送 publisher 当前使用 ROS 2 `RELIABLE`、`VOLATILE`、depth 10。这里的 **ROS 2 RELIABLE** 是规划器与桥之间的 DDS 传输 QoS，和串口帧的 **ACK/重传** 是两层不同机制；任何一层的“可靠”都不表示机械臂已执行完成。

### 5.2 串口封装

```text
字节 0        1        2        3        4 .. (4+Len-1)       4+Len
     0x5A     0xA5     MsgID    Len      Payload (Len 字节)    CRC8
```

整帧长度是 `5 + Len` 字节。CRC8 对 `MsgID + Len + Payload` 计算，多项式 `0x31`，初值 `0`；帧头不参与计算。生成代码使用 packed C 结构体直接拷贝数值字节；当前 NUC 与 STM32 的实际目标均需保持一致的小端 `uint` / IEEE-754 `float32` 布局，跨平台时不能只依赖结构体拷贝。业务报文使用 `0x01`、`0x10`、`0x11`、`0x12`、`0x20`；`0xFD`、`0xFE`、`0xFF` 分别留给框架的 ACK、心跳和握手。规划器只关心四种机械臂业务报文，不生成 CRC 或操作串口。

### 5.3 机械臂报文逐字节结构

表内偏移相对于 **Payload** 起点，不包含 `0x5A 0xA5 MsgID Len`。可靠报文最后的 `seq` 是桥接框架添加的 1 字节，不在 ROS 消息或 MCU `Packet_xxx` 业务结构体中；CRC 又在 Payload 之后。

| ID / 报文 | Len / 整帧 | Payload 偏移与字段 |
| --- | --- | --- |
| `0x10 TrajectoryHeader` | 7 / 12 字节 | `0..3 trajectory_id:u32`；`4..5 point_num:u16`；`6 seq:u8` |
| `0x11 TrajectoryPoint` | 28 / 33 字节 | `0..3 trajectory_id:u32`；`4..5 point_id:u16`；`6..9 time_from_start:u32(ms)`；`10..13 joint_1:f32`；`14..17 joint_2:f32`；`18..21 joint_3:f32`；`22..25 joint_4:f32`；`26 gripper_action:u8`；`27 seq:u8` |
| `0x12 ArmControlMode` | 2 / 7 字节 | `0 control_mode:u8`；`1 seq:u8` |
| `0x20 ArmStatus` | 44 / 49 字节 | `0..3 trajectory_id:u32`；`4 task_status:u8`；`5..8 progress:f32`；`9..24 joint_1..4:f32`；`25..40 joint_1_velocity..4_velocity:f32`；`41 gripper_state:u8`；`42..43 error_code:u16` |
| `0x01 ChassisVelocity` | 8 / 13 字节 | `0..3 linear_x:f32`；`4..7 angular_z:f32`；由底盘速度链路发送，无串口 ACK，桥接配置在发送时对 ROS `angular.z` 取反，规划器不发布此报文 |
| `0xFD Ack` | 2 / 7 字节 | `0 acked_id:u8`；`1 ack_seq:u8` |
| `0xFE Heartbeat` | 4 / 9 字节 | `0..3 count:u32` |
| `0xFF Handshake` | 4 / 9 字节 | `0..3 protocol_hash:u32` |

`trajectory_id` 关联 Header、全部 Point 及后续 ArmStatus；`point_id` 为本轨迹 `0..N-1`；`time_from_start` 是自轨迹开始的毫秒时间。`ArmControlMode` 自身**没有 `trajectory_id` 字段**，因此一次只允许有一个待启动或执行中的轨迹是协议使用前提。

| 字段 | 值 | 语义 |
| --- | --- | --- |
| `control_mode` | `0` / `1` / `2` | 启动当前已缓存轨迹 / 平滑停止并返回空闲（也用于解除急停锁定） / 急停锁定 |
| `task_status` | `0` / `1` / `2` / `3` / `4` | IDLE / RUNNING / COMPLETED / FAILED / EMERGENCY |
| `gripper_action`、`gripper_state` | `0` / `1` / `2` | 保持或未知 / 张开 / 闭合；动作与状态不是同一个证明层级 |
| `error_code` | `0` / `1001` / `1002` / `1003` | 正常 / 超时 / 超限 / 急停触发；下位机是否还有其他错误码待核实 |

### 5.4 ACK、心跳和状态分别证明什么

1. ROS publisher 的 `publish()` 返回：本进程已把消息交给 ROS；不证明桥已经处理。
2. 桥收到串口 `Ack(acked_id, ack_seq)`：下位机协议层已调用对应业务接收回调并返回；不证明路点被业务层接受、缓存完整，更不证明关节已运动。ACK 当前不向规划器暴露。
3. `Heartbeat`：协议配置每 1000 ms 发起一次；当前 `strict_heartbeat=false`，心跳超时不自动阻断业务传输。心跳不是运动完成信号。
4. `ArmStatus(RUNNING)`：下位机声称某轨迹正在运行；`ArmStatus(COMPLETED)` 且 `trajectory_id` 等于本次请求 ID，才是规划器判断完成的业务证据。状态消息未启用串口重传，必须考虑丢帧与终态持续时间。
5. `ArmStatus(IDLE)`：只说明当前空闲，不区分尚未装载、正在装载或已经装载等待启动。现有状态结构没有“已收齐 N 点”字段，20 Hz 上报不能充当装载完成确认。

可靠报文在桥内按**同一消息 ID** FIFO 等待 ACK；不同 ID 使用各自队列。ACK 丢失时桥会重发，语义是进程存活期间的 *at least once*，下位机业务回调需要容忍重复 Header、Point 或 Mode。配置的 1000 ms 是重发间隔，`reliable_max_retries=3` 是告警阈值，不是发送次数上限。CRC 正确和 ACK 正常也不能代替业务状态判断。尤其是 STOP/急停与启动命令同属 `0x12`：如果之前的 Mode 尚未获 ACK，后发的 STOP/急停会在同一队列等待，不能由规划器声称它已经即时送达硬件。

## 6. 一次请求的完整时序

```text
mission            arm_planner_node       ROS 串口桥             下位机
  | 1. 服务请求          |                     |                    |
  |-------------------->|                     |                    |
  |                     | 2. 冻结新鲜 ArmStatus 快照             |
  |                     | 3. IK、限位、插值、检查全轨迹           |
  |                     | 4. Header(id,N)      |                    |
  |                     |-------------------->|-- 0x10 + ACK ----->|
  |                     | 5. Point(id,0..N-1) |                    |
  |                     |-------------------->|-- 0x11 + ACK ----->|
  |                     | 6. Mode(TRAJECTORY) |                    |
  |                     |-------------------->|-- 0x12 + ACK ----->|
  |                     |                     |<-- ArmStatus 20Hz --|
  |                     |<-- status(id,RUNNING/COMPLETED/FAILED) --|
  |<-- 7. 完成或失败 ---|                     |                    |
```

图中步骤 4、5、6 是**规划器发布顺序**，不是当前代码已经证明的串口到达顺序。当前 `arm_planner_node` 在同一函数中依次 `publish` Header、所有 Point、Mode；桥对不同报文 ID 没有跨队列 ACK 屏障，ROS 对不同话题也没有全局顺序保证。因此“Mode 一定在 N 个路点实际到达后才到达”目前是**未经证明的假设**。项目的现有上下位机链路已经调通，本轮不改协议或桥；此风险需在实机传输验证中保留，不应以固定 `sleep` 时间声称解决。若下位机提前收到 Mode，用户期望它拒绝启动并报错；具体 `ArmStatus` 报错内容与 `trajectory_id` 仍待下位机实现确认。

服务等待的当前流程是：收到发布时间之后、匹配本次 `trajectory_id` 的 `COMPLETED` 才成功；匹配的 `FAILED` 或急停应失败；超时发布 STOP 并返回失败；显式 STOP/急停使被打断请求失败。等待期限从启动 Mode 发布后计时，为末点时间加配置余量。`COMPLETED` 上报是否至少保持一个 20 Hz 周期、STOP 后何时回到 IDLE，均需现场确认；串口排队时间仍可能占用这段超时余量。

## 7. 并发、停止与异常

| 场景 | 当前规划器行为 | 仍需核实 |
| --- | --- | --- |
| 第一条普通运动请求 | 状态新鲜、有效且空闲时受理；完整规划后顺序发布 | 固件的轨迹缓存容量 |
| 运动或发送期间又收到普通请求 | 不覆盖现有轨迹，立即返回忙 | 无 |
| STOP / 急停 | 可以打断正在规划、发送或等待的请求；STOP 发 mode 1，急停发 mode 2 | Mode 在桥队列中的实际送达时刻 |
| 无有效关节回传 | 拒绝运动；STOP/急停仍可发布 | 无 |
| 规划失败 / 超限 / 非法输入 | 不发布 Header、Point 或 Mode，服务明确返回失败 | 固件实际缓存上限未确认 |
| 下位机报告 `FAILED` | 以同一 `trajectory_id` 返回失败和错误码 | 固件错误码含义需核实 |
| 串口 ACK 丢失 / 桥断线 | 不把 ROS 发布成功当成动作成功；超时后停止并报失败 | 桥持续重发；规划器只有 ArmStatus 可见，不能知道逐帧 ACK 状态 |
| 终态帧丢失 | 超时失败，不能凭 IDLE 猜测已完成 | 协议为周期上报，但终态保持策略待核实 |

“急停”不是普通动作失败后的自动复位方式；接到硬件急停后是否允许规划器自行发送 STOP 解除锁定，应由任务/操作流程决定，不能由普通新运动请求隐式解锁。下位机的物理保护不能依赖 ROS 服务进程存活。

## 8. 当前已知差距与升级选项

包内请求仲裁、实测首点、轨迹预校验和终态匹配已实现。剩余升级选项如下：

1. 若将来要证明 N 个点都已装载，可让桥暴露逐轨迹投递完成，或让固件报告装载完成。当前协议没有这项证据。
2. 若未来需要远程取消、执行反馈流或多个请求排队，可评估 ROS 2 Action；当前三类调用沿用现有 service。

包内实施情况与待核实事项见[重构记录](REFACTOR_SPEC_AND_CHECKLIST.md)。
