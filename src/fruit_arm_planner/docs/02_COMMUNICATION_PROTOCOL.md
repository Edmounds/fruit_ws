# 上下位机业务通信协议与数据流

本文档只描述上位机需要下发的业务数据、下位机需要上报的状态，以及机械臂任务如何判断完成。串口帧格式、ACK、重传和心跳由 `auto_serial_bridge` 处理，不属于本篇范围。业务字段以 [`protocol.yaml`](../../auto_serial_bridge/config/protocol.yaml) 和 [`fruit_hardware_interfaces/msg`](../../fruit_hardware_interfaces/msg) 为准；下位机具体执行及错误码语义仍需以实机固件确认为准。

## 1. 职责与数据方向

```text
mission / 调试客户端
    | 关节目标、笛卡尔目标或预设名称 (ROS 2 service)
    v
fruit_arm_planner  --状态检查、IK、映射、轨迹规划、等待执行结果--
    | TrajectoryHeader -> TrajectoryPoint[] -> ArmControlMode
    v
auto_serial_bridge  --传输业务数据--  下位机
    ^                                |
    +----------- ArmStatus ----------+
```

- 上位机负责把任务目标转换成机械臂的四轴硬件关节轨迹。视觉坐标应先在上层转换到机械臂根部固定坐标系；下位机不接收笛卡尔目标或预设名称。
- 规划器通过 `/arm_planner_node/move_joints`、`/arm_planner_node/move_cartesian`、`/arm_planner_node/execute_preset` 接收正式运动请求。`home` 是预设快捷服务；`cmd_joints`、`cmd_cartesian`、`cmd_preset` 是无逐请求结果的调试话题。
- `move_joints` 的目标单位是度；`move_cartesian` 的 XYZ 是真实夹爪抓取点 TCP 的基座坐标（米），`pitch_deg` 是末端 a4 伸出段俯仰角（度）。直接输入和笛卡尔预设共用 `geometry.tool_z_offset_m` 固定结构模型，不再叠加旧 `planning.cartesian_offset_m` 输入平移。下发给下位机的四轴角度统一为**硬件关节弧度 rad**，关节角速度回报单位为 rad/s。
- 底盘速度是独立的下发路径，不经过机械臂规划器。

业务遥测仍使用 `/arm_planner_node/joint_states`，类型已改为 `fruit_arm_interfaces/msg/ArmTelemetry`：同一帧原始电机角、逆映射模型角及 FK TCP。硬件 `/fruit_hardware_bridge/arm/status` 的 ArmStatus 与线协议保持不变；模型角不要再次逆 mapping。详见[遥测字段](03_INTERFACES_AND_API.md#六遥测广播接口-telemetry)。

## 2. 上位机下发的数据

| 数据及 ROS 话题 | 字段 | 用途 |
| --- | --- | --- |
| `TrajectoryHeader` `/fruit_hardware_bridge/arm/trajectory_header` | `trajectory_id: uint32`、`point_num: uint16` | 声明新机械臂轨迹的 ID 和路点总数。 |
| `TrajectoryPoint` `/fruit_hardware_bridge/arm/trajectory_point` | `trajectory_id: uint32`、`point_id: uint16`、`time_from_start: uint32`、`joint_1..4: float32`、`gripper_action: uint8`、`is_via_point: uint8` | 按时间排列的四轴目标；时间单位 ms，角度单位 rad。`0` 为精确点，`1` 为途经点。 |
| `ArmControlMode` `/fruit_hardware_bridge/arm/control_mode` | `control_mode: uint8` | 受控停止或急停。下位机收到完整点位后自动启动轨迹。此消息**不包含 `trajectory_id`**。 |
| `ChassisVelocity` `/cmd_vel` | `linear_x: float32`、`angular_z: float32` | 底盘线速度 m/s 和角速度 rad/s；桥在下发时对 ROS `angular.z` 取反以匹配下位机方向。 |

`TrajectoryPoint.gripper_action`：`0` 保持当前开度（HOLD），`1` 张开（OPEN），`2` 闭合（CLOSE）。这是**命令**，不等于已完成的夹爪状态反馈。

`TrajectoryPoint.is_via_point`：`0` 表示精确到达点，`1` 表示途经点。该字段只改变下位机的点到位容差选择，不改变关节限位、急停、STOP、时间轴或通信保护。预设 waypoint 未配置该字段时默认为 `0`；PTP/LIN 自动生成的中间插值点默认为 `1`。service 接口暂不开放该字段，因此 service 的最终目标点默认为 `0`。

`ArmControlMode.control_mode`：`1` 请求受控停止（STOP），`2` 请求急停（ESTOP）。下位机收到完整点位后自动启动轨迹，规划器当前不发送 `control_mode=0`。STOP/急停服务返回成功只表示规划器已发布命令，不能据此确认下位机已停稳。

## 3. 轨迹数据顺序与时间轴

```text
上位机发布                                  下位机业务处理目标
TrajectoryHeader(id, N)                  声明本次轨迹
TrajectoryPoint(id, 0, t=0, 当前实测角)     起点
TrajectoryPoint(id, 1 ... N-1)           后续目标与插值点
下位机收到 N 个点后自动启动轨迹
                     <--- ArmStatus 周期上报执行状态
```

1. `point_num=N` 对应本轨迹实际下发的 N 个路点；规划器要求 `2..64` 点，与当前 FT 下位机缓存容量一致。超出容量时在发布 Header 前返回失败。所有路点使用相同的 `trajectory_id`，`point_id` 按 `0..N-1` 连续编号。
2. 第 0 点必须是规划开始时冻结的、**新鲜且有效的四轴实测角**：`time_from_start=0 ms`、`gripper_action=0`、`is_via_point=0`。缺失、过期、非有限数或超出配置限位的反馈不能用零或历史值代替。不能只发送一个时间为 `15000 ms` 的目标点。
3. 后续路点的 `time_from_start` 是相对于本次轨迹起点的累计时间，必须严格递增；末点时间为整条轨迹的计划时长。预设各段配置的 `duration_ms` 是逐段时长，规划器累加后下发。四轴目标先由上位机完成 IK、关节映射和限位检查。插值点默认是途经点，显式 waypoint 目标点使用其 `is_via_point` 配置。
4. 下位机收到 Header 声明的全部 N 个点后自动启动轨迹。规划器发送 Header 和全部 Point 后等待下位机状态，不发送 `TRAJECTORY` 启动 Mode。

机械臂硬件命令发布队列和串口桥业务订阅队列的 KEEP_LAST 深度均为 `64`，使用 RELIABLE、VOLATILE。一次连续发布的轨迹必须完整驻留在队列中。串口 ACK 仅保护进入桥发送队列的消息，无法恢复 ROS 队列中已被覆盖的点。修改桥的 `config.qos_depth` 后必须重新生成、构建并重启桥。

FT 下位机将点号、时间和执行等多个内部错误映射为 `1001`。即时 `1001` 不一定来自执行期限。队列覆盖的复现与修复验证见 [机械臂 1001 排查记录](../../../docs/ARM_TIMEOUT_1001_DIAGNOSIS.md)。Header 与 Point 的跨消息传输顺序仍需验证，队列容量不能代替加载确认。

例如 2 秒的单目标动作至少有：`Point 0 = (当前实测角, 0 ms, HOLD)`，`Point 1 = (目标硬件角, 2000 ms, 指定夹爪动作)`；规划器也可能生成更多中间插值点，所有插值点和驻留点均计入 64 点上限。笛卡尔 LIN 根据实测角做 FK 得到路径起点，但下发的首点仍是**原始实测关节角**。

## 4. 下位机上传的数据

下位机按当前配置约每 **50 ms（20 Hz）** 上报 `ArmStatus`，通信桥发布到 `/fruit_hardware_bridge/arm/status`：

| 字段 | 类型 | 业务含义 |
| --- | --- | --- |
| `trajectory_id` | `uint32` | 状态对应的轨迹 ID，用于与本次请求匹配。 |
| `task_status` | `uint8` | 执行状态，定义值见下表。 |
| `progress` | `float32` | 执行进度，约定为 `0..100%`。 |
| `joint_1..4` | `float32` | 四轴实测硬件角度，rad；下一条运动用它们作为起点。 |
| `joint_1_velocity..joint_4_velocity` | `float32` | 四轴实测角速度，rad/s。 |
| `gripper_state` | `uint8` | 夹爪状态，定义值为 `0` 保持/未知、`1` 张开、`2` 闭合。 |
| `error_code` | `uint16` | 下位机故障码；`0` 表示无故障。 |

| `task_status` | 接口定义 | 上位机处理 |
| ---: | --- | --- |
| `0` | `IDLE` | 当前空闲；不能仅凭它认定上一条轨迹完成。 |
| `1` | `RUNNING` | 继续等待执行结果。 |
| `2` | `COMPLETED` | 对应轨迹报告完成。 |
| `3` | `FAILED` | 对应轨迹失败，记录故障码。 |
| `4` | `EMERGENCY` | 急停状态，停止正常任务推进并记录故障码。 |

ROS 接口定义的 `error_code` 包括：`0` 无故障、`1001` 超时、`1002` 超限、`1003` 急停触发。实机还曾上报未写入当前接口语义的 `task_status=5`、`error_code=6/9/11`、`gripper_state=3`，这些数值的含义**待下位机负责人确认**，不得直接解释为腕部或夹爪故障。实测关节数据也可能出现 `NaN`；无效角度不能作为下一条运动的起点。

规划器把有效的四轴反馈另行发布为 `/arm_planner_node/joint_states`，供监控使用；它不是独立的运动完成确认。

## 5. 任务结果与异常处理

- 规划器从发送全部 Point 后开始等待新的 `ArmStatus`。只有状态的 `trajectory_id` 等于本次请求 ID 且 `task_status=COMPLETED(2)`，运动 service 才返回 `success=true`；发布轨迹或收到 `RUNNING` 都不表示动作完成。
- 匹配的 `FAILED(3)`、`EMERGENCY(4)` 或显式停止使本次运动返回失败。硬件报错由下位机自然上报；**发布 STOP 不证明实机已经停稳**。
- planner 与 mission 均没有运动执行期限或超时看门狗；等待本次下位机结果，缺少结果时由操作人员主动 STOP/ESTOP 或退出。端点发现和反馈新鲜度仍独立检查，不因无限等待放宽运动起点校验。
- 规划器在一次运动期间拒绝新的普通运动请求；没有新鲜有效的四轴状态、目标不可达、越限或轨迹格式无效时，拒绝下发。STOP/急停不依赖有效的运动起点。
- 当前业务数据中没有“已装载全部 N 个路点”的上报字段，也没有底盘速度执行完成状态。因此不能用 `IDLE`、话题发布成功或夹爪动作命令代替这些确认。下位机的终态保持时长、故障码含义和实际缓存容量仍需实机核对。
