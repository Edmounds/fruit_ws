# 下位机通信协议与控制边界

本页保留线协议与状态核对用途，**不是绕过 planner 的机械臂操作指南**。
上位机控制必须按 [fruit_arm_planner API](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md)
调用服务，由 planner 完成 IK、关节映射、插值和轨迹下发。

## 权威定义与当前集成

- 线协议：`src/auto_serial_bridge/config/protocol.yaml`，修改消息布局后必须重新构建生成协议。
- ROS 接口定义：`src/fruit_interfaces/`（合并原 `fruit_hardware_interfaces` 与 `fruit_arm_interfaces`）。
- 设备与波特率从 source YAML 读取，当前为 `/dev/ttyACM0`、115200，不应假定所有入口都用 `/dev/stm32`。
- 新协议不含旧 `ArmTarget`、`StateMachineCommand`、`ExecutionCallback`。
  mission 视觉目标、观察、二维码、Harvest 和 Home 均通过 `fruit_arm_planner` 服务；区域状态只通过 ROS 话题通知感知/扫码节点。
- `auto_serial_bridge` 负责串口帧、CRC、ACK、重传和心跳，不实现任务或 IK。
- RobViz 使用同一 planner。新增 `get_configuration` 只读服务和 HTTP 预览不改变 MCU 协议。

## 消息概览

下表话题均为全局绝对名称：

| 消息 | ID | 方向 | Reliable | ROS 话题 |
| --- | --- | --- | --- | --- |
| ChassisVelocity | `0x01` | TX | 否 | `/cmd_vel` |
| TrajectoryHeader | `0x10` | TX | 是 | `/fruit_hardware_bridge/arm/trajectory_header` |
| TrajectoryPoint | `0x11` | TX | 是 | `/fruit_hardware_bridge/arm/trajectory_point` |
| ArmControlMode | `0x12` | TX | 是 | `/fruit_hardware_bridge/arm/control_mode` |
| ArmStatus | `0x20` | RX | 否 | `/fruit_hardware_bridge/arm/status` |

`ChassisVelocity` 来自速度仲裁后的 `geometry_msgs/Twist`：`linear.x` 为 m/s，
`angular.z` 为 rad/s；协议映射在串口发送前分别取反 `linear.x` 和 `angular.z`。
手柄与导航客户端继续使用 ROS 方向约定，不要再次取反。
字段映射在编译时生成，修改后必须构建并重启实际持有串口的桥。
录点脚本复用已有桥时，仅重启录点脚本不会更新映射。

## 轨迹、ACK 与执行结果

1. Header：`trajectory_id:uint32`、`point_num:uint16`。
2. Points：同一轨迹 ID，`point_id:uint16` 从 0 连续递增；`time_from_start:uint32` 为毫秒；
   四个原始硬件关节角为 float32 弧度，夹爪动作 0=HOLD、1=OPEN、2=CLOSE；
   `is_via_point:uint8` 为 0=精确到达、1=途经点。
3. 轨迹从 `t=0` 新鲜实测四轴角开始（HOLD），至少两点，时间严格递增。
4. 当前 planner 不发送 Mode=0，假定下位机收到完整点位后自动启动。
   新鲜 `ArmStatus` 的轨迹 ID 匹配且状态为 COMPLETED 才表示执行完成。

**现有限制：**planner 按 Header → Points 发布，串口桥可靠队列按消息 ID 独立处理。
Header/Point 传输顺序仍须验证，尚无 planner 可用的装载确认。
`mode_start_delay_ms` 当前不触发启动 Mode，也不能证明装载完成。
ACK 也不是电机到位确认。详细线布局与时序见[通信文档](../src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md)。

`reliable_retry_interval_ms` 当前为 1000，`reliable_max_retries=3` 表示每累计三次告警，
不是三次后停止重发。当前 `strict_heartbeat=false`，不能据此宣称桥会自动拦截失联后的业务命令；
断联停车和硬件保护必须由固件独立落实并验收。

## 停止与互锁

协议定义 Mode=1 为 STOP（清除剩余轨迹、解除急停锁定并恢复 IDLE），Mode=2 为 ESTOP
（进入 EMERGENCY）。实际减速、驱动禁止和锁定行为须与固件核对。
服务返回成功仅表示 ROS 发布成功，不是收到 ACK 或硬件停稳。
重新发起运动前须观察新鲜有效反馈和可接受的控制状态。导航与 Look 可以并行；进入识别、抓取或二维码判定前，mission 必须确认底盘和机械臂均已静止。

## ArmStatus（协议约定 20 Hz）

| 字段 | 类型 / 含义 |
| --- | --- |
| trajectory_id | uint32，本次或刚完成轨迹编号 |
| task_status | uint8：0=IDLE，1=RUNNING，2=COMPLETED，3=FAILED，4=EMERGENCY |
| progress | float32，0–100% |
| joint_1…4 | float32，原始硬件实测角（rad） |
| joint_1…4_velocity | float32，角速度（rad/s） |
| gripper_state | uint8：0=HOLD/UNKNOWN，1=OPEN，2=CLOSE |
| error_code | uint16：0=NONE，1001=TIMEOUT，1002=LIMIT_EXCEEDED，1003=ESTOP_TRIGGERED |

状态与错误常量以 `ArmStatus.msg` 为准，实际固件行为不能仅凭常量认定已经验证。
Q3 硬件角对应小臂绝对俯仰（含偏置），映射为
`direction_3 * (q2_algo + q3_algo) + offset_3`，不是相对肘角。
限位和几何由 `src/fruit_arm_planner/config/arm_presets.yaml` 维护，不复制旧控制台限位。

## 正式调试入口

确认硬件周围安全；下列控制台能实际驱动机械臂，不作为自动文档检查运行。
已有串口桥时只启动 planner，不重复使用带桥的 debug launch：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
ros2 launch fruit_arm_planner fruit_arm_planner.launch.py
# 另一个准备好环境的终端：
python3 src/fruit_arm_planner/test/cartesian_console.py --log-level info
```

只读状态可使用：

```bash
ros2 topic echo /fruit_hardware_bridge/arm/status
ros2 topic echo /arm_planner_node/joint_states
```

`lower_controller_console.py` 是保留在源码中的旧原始协议工具，其直接发轨迹和固定姿态/限位
不遵循现行 planner 控制边界，本页不再推荐或复制其运动命令。本次仅更新文档，未修改该脚本。
