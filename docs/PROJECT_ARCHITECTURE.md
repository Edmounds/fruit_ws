# 水果采摘机器人当前软件架构

本文是持续维护的模块与集成参考，不是历史实施计划。现场操作与包级接口入口见[文档导航](README.md)。

## 1. 当前目标

省级选拔赛范围为 A、B、C 区，D 区暂不接入。当前固定流程契约为：

- A 区每个停车点按配置顺序先左后右依次转向，单深度相机 `camera` 在到位后的 harvest 观察窗口内识别成熟目标并请求机械臂到目标点。
- B 区入口到站并完成 QR 姿态后开启 QR 观察窗口，确认并保存 B 计划。
- B 区按固定位置 1 到 8 依次导航，每棵树使用二维码对应位置的目标类型完成观察和采摘。
- B 区完成后前往 C 区入口，到站并完成 QR 姿态后开启 QR 观察窗口，确认 C 计划并按二维码行序严格执行 8 项。
- C 区完成后关闭观察窗口并结束当前软件任务流程。回起点航点待现场测出最终位置后加入；当前范围仍不实现收集更换区和返航后的结束判定。

上位机 NUC 负责里程计、导航、识别、比赛编排，以及通过 `fruit_arm_planner` 做机械臂 FK/IK、关节映射和轨迹规划。下位机负责底盘与机械臂轨迹的实时执行、状态回报及硬件保护。

**正式 mission 已接入 planner，整机尚未验收：**mission 和单 camera 调试客户端共用
`CameraTargetPlannerClient` 处理目标。正式 mission 的观察/二维码/收臂/恢复用命名预设服务，
区域通知只走 ROS；独立 camera/区域调试工具的旧姿态接口未迁移。预设轨迹尚待补齐。
当前视觉目标运动使用 CLOSE，随后实测硬件角 HOLD 驻留、对应区域/侧 Store 放果工序和 Home；服务完成仍不能证明果实实际入筐。

## 2. 总体架构

```text
                    +------------------------+
二维码识别包 ------> | fruit_mission          |
                    | A/B/C 比赛编排          |
camera 识别/深度目标 ->| 顺序、播报、日志、决策  |
                    +-----------+------------+
                                |
             Action /follow_waypoints | planner services
                                v
+-------------------+   +------------------------+
| fruit_navigation  |   | fruit_arm_planner      |
| Point-LIO 里程计   |   | 运动学解算/轨迹插值     |
| 轴向定点导航服务端 |   | 关节偏置映射与预设姿态 |
| 偏航角控制         |   +-----------+------------+
+---------+---------+               |
          |                         v
          | cmd_vel_nav             | Header / Points / Mode
          v                         v
+-------------------+       +------------------------+
| twist_mux         |       | auto_serial_bridge     |
| 导航/遥控速度仲裁  |------>| YAML/串口/ACK/心跳/CRC |
+-------------------+       +-----------+------------+
                                        |
                            +-----------+--------+
                            | 下位机              |
                            | 底盘/舵机/4DOF/末端  |
                            +--------------------+
```

必须保持以下边界：

1. 比赛业务节点不直接读写串口字段，只调用有明确语义的 ROS 2 topic、service 或 action。
2. 机械臂客户端按 [planner API](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md) 调用服务，不直接发送轨迹或串口字段；串口通信由 `auto_serial_bridge` 统一承载。
3. `auto_serial_bridge` 只处理串口帧、代码生成、握手、心跳、校验和可靠传输，不承载比赛流程。
4. 下位机不决定 A/B/C 顺序，不承担笛卡尔 IK；planner 下发硬件关节轨迹，并以匹配 `trajectory_id` 的新 `ArmStatus` 终态判断执行结果。正式 mission 姿态调用 `execute_preset`，不再调用旧方向/恢复接口。
5. 只有速度仲裁节点能向串口通信层发布最终底盘速度，禁止导航和任务节点同时直接控制底盘。

### 2.1 ROS 2 中间件

- 比赛运行环境统一使用 `rmw_cyclonedds_cpp`，由 `scripts/setup_fruit_runtime.bash` 固定并校验；所有 ROS 2 节点和 CLI 必须加载同一环境。
- 不再使用 Humble 默认的 `rmw_fastrtps_cpp`。Fast DDS Basic Service Mapping 的 request 和
  reply 端点独立发现，冷启动时可能出现请求已送达、Action 已执行并发布 `SUCCEEDED`，但
  goal response 因 reply reader 尚未匹配而被丢弃，mission 因拿不到 goal handle 无法推进；
  上游跟踪见 [ros2/rmw_fastrtps#842](https://github.com/ros2/rmw_fastrtps/issues/842)。
- 启动脚本中临时 `ActionClient` 的 readiness 结果只属于该客户端，不能证明之后新建的
  mission ActionClient 已完成反向响应端点发现；禁止用延长 future 超时、固定 `sleep` 或
  盲目重发导航 goal 掩盖响应丢失。
- RMW 只能在进程启动时选择。切换时必须停止整套旧进程和旧 ROS 2 daemon，再从
  `scripts/setup_fruit_runtime.bash` 启动完整链路，禁止 Fast DDS 与 Cyclone DDS 混跑。

## 3. ROS 2 工作区划分

```text
fruit_ws/src/
├── driver/
│   ├── livox_ros_driver2/        # MID-360 驱动源码，当前有 COLCON_IGNORE
│   └── realsense-ros/            # camera 相机驱动
├── navigation/
│   ├── point_lio/                # Point-LIO LiDAR-IMU 紧耦合里程计
│   ├── loam_interface/          # TF 转换与 odom->base_link 广播适配
│   ├── fruit_navigation/         # 轴向航点跟踪 action server
│   └── fruit_nav_bringup/        # 纯里程计导航集成 bringup
├── auto_serial_bridge/           # 串口协议生成与传输
├── fruit_interfaces/             # 统一 ROS 2 接口包：机械臂服务与轨迹/状态 msg
├── fruit_arm_planner/            # 4-DOF 机械臂运动学解算、关节偏置映射与轨迹规划
├── fruit_perception/             # 单 camera、观察窗口与两阶段 OpenVINO
├── fruit_qr_planner/             # camera 彩色图扫码并持久发布 B/C 计划
├── fruit_robviz_bridge/         # RobViz 图像预览组件、配置客户端、航点录制节点与 Web HTTP 服务
├── fruit_mission/                # A/B/C 比赛编排
└── joy_linux/                    # 手柄遥控节点
```

`auto_serial_bridge` 不由本仓库作为 Git submodule 管理，而是从 Edmounds fork 的
`dev` 分支独立克隆；父仓库通过 `.gitignore` 忽略该目录。

## 4. 导航结构

### 4.1 纯里程计导航架构

- 导航采用纯 Point-LIO 激光雷达-IMU 里程计方案，彻底去除了离线建图（GLIM）与重定位（GICP）环节。
- 坐标系拓扑简化为 `odom -> base_link -> sensors/arm`，不再使用 `map` 全局坐标系。
- Point-LIO 在开机时以静止状态建立初始参考系，`loam_interface` 在接收到稳定里程计后开始发布 `odom -> base_link` TF。
- 航点文件（`src/navigation/fruit_nav_bringup/waypoints/odom_waypoints.yaml`）中的坐标全部基于 `odom` 坐标系。
 - 导航入口为 `start_navigation.sh`，不提供建图/地图定位模式选择；脚本直接使用 launch 的 source YAML 默认配置，RobViz 提供 Web 可视化。

### 4.2 轴向定点导航控制器 (`fruit_navigation`)

RobViz 的 Maker 沿用 recorder 的 Action 客户端，不添加导航控制器。
未知 Action 结果保持目标所有权，并阻止下一段。
取消终态后的实际里程计样本必须覆盖连续停稳窗口。
MarkerArray 锁存完整航点快照，晚连接无需新的编辑操作。

Web RGB 和 Native Depth 采用 `fruit_robviz_bridge` 的原尺寸 WebRTC (H.264) 与可选 JPEG 图像网关，支持 NUC 本地原始无损 PNG 采集。
Python 配置节点只读检测与遥测，复用 mission 变换生成同戳 Base 元数据。
它不接收 raw image，不执行 IK、机械臂控制或比赛编排。

- `axis_waypoint_action_server` 提供 `/follow_waypoints` Action 接口。
- 支持按轴行走、自动倒车判定，并固定在位置到达后原地调整最终车体朝向；没有逐点 `yaw_mode` 或 `reverse` 模式。
- 速度输出经由 `twist_mux` 输出至串口控制器。


## 5. 相机感知方案

### 5.1 单 camera 与两阶段模型

项目只使用一个深度摄像头 `camera`，左右 USB 相机已彻底移除。
深度相机节点为 `/camera`，原生接口为 `/camera/*`，感知输出为
`/fruit_perception/camera/*`，决策发布于 `/fruit_perception/decision`。

camera 的固定原生流开启同步、深度对齐与 RGBD（Color 为 `1280x720x30` RGB8，原生深度为 `848x480x30` Z16 配合 High Accuracy 预设与手动曝光，对齐深度保持 1280x720 对齐像素空间）。相机硬件流常开、模型持续推理：

1. 到站且 QR 姿态完成后开启 QR 观察窗口，Python/OpenCV 扫码节点对彩色原图解码并确认计划（`confirm_frames=1`）。
2. B/C 计划使用 Reliable、Transient Local、深度 1 发布，计划中附带窗口 ID 及帧时间戳。
   QR 直接按窗口区域选择 B/C 解析，不再维护 B→C→COMPLETED 顺序状态机；新窗口重置候选及连续帧确认，当前区域通知不推进扫码阶段。
3. camera RGBD 持续执行水果/蔬菜推理，整帧 letterbox 到 640x640，不做分块。
   根据动态字符串参数 `zone` 选择两阶段链路：A/C 用 `vegetable_type/best.onnx`，
   B 用 `fruit_type/best.onnx`，再做熟度分类。`model_root` 来自源码 YAML；
   默认 Edge，启动可选 YOLO，固定 Intel `GPU.0`。`zone` 只接受 A/B/C。
   mission 通过参数服务切区并检查应答，区域话题仅保留任务状态。
   观察窗口不切模型，harvest 区域必须匹配 `zone`，切区清除旧窗口和证据。
   C++ 标注保留品种，并显示熟度百分比、检测分数、中心深度和准星。
4. `/fruit_perception/decision` 发布单相机决策消息，携带 `observation_window_id`、一帧内的全部目标及原图 ROI，中心深度无效时保留二维结果。窗口外 `observation_window_id` 为空且不输出抓取目标。
5. A/C 区使用单目标锁定器（8 帧短窗口，A 区只锁定成熟 ripe 目标，C 区按期望品种过滤成熟目标），在有效 harvest 窗口内锁定后发布 `has_target=true` 和 `target_point`。
6. B 区使用多候选选择器（8 帧短窗口，最多输出 2 个稳定候选），在有效 harvest 窗口内发布 `targets_ready=true` 和 `targets` 列表。
7. 任务节点等待携带当前 `observation_window_id` 的有效 `decision`，在 mission 内将 RGB 光学点转换到夹爪系，再用 planner 的新鲜实测 TCP/模型偏航转到机械臂固定基座系，应用 `target_offset_m` 后调用 `/arm_planner_node/move_cartesian` 并检查执行结果。偏置单位为米，当前 `[0.0, 0.03, -0.015]`；X/Z 直接相加，Y 幅值按原 Base Y 正负自动变号，为零则不加。正式任务与单点调试共用校验和调用链，不重复应用工具几何偏移。观测及提交前由现场物理保证机构静止；CLOSE/HOLD、Harvest、Home 均完成才记录工序成功。外参毫米/ROS 米边界及剩余集成限制见 [mission 坐标换算文档](../src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md)。

4DOF 机械臂不按通用 6DOF 位姿设计。现有解析 FK/IK、PTP/LIN 与限位检查在
`fruit_arm_planner` 中实现，几何和映射由 `config/arm_presets.yaml` 配置，不另引入 MoveIt。
相机模型、ROS 标签和性能记录见[两阶段运行文档](../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md)。

### 5.2 NUC 资源优先级

运行优先级从高到低为：

1. 串口心跳、急停和底盘速度输出；
2. Point-LIO、TF 和轴向航点跟踪；
3. camera 扫码与果蔬识别和测深；
4. 可视化、图像落盘和调试日志。

任务入口使用 `--debug` 时，按固定 mission profile 记录导航、Action、雷达、点云、感知、机械臂和控制诊断。
视觉数据限定为 camera 彩色原图、对齐深度图和标注图。未对齐 camera raw depth、
RGBD 复合消息、camera_info 和 `compressed`/`compressedDepth`/`theora` 派生图像不进入录包，避免同一帧
多次 DDS 序列化和落盘。导航、航点和 teleop 入口不因系统中存在 camera 而录制视觉话题。
调试监视器通过 color/depth metadata 与进程内 RGBD 回调产生的轻量 Header heartbeat 计算
相机的实际频率和最后帧时间，不直接订阅高带宽 RGBD 消息。
所有 debug 入口会在相关节点就绪后启动对应 profile 的录包，并输出最终话题清单。
确认 `/rosbag2_recorder` 就绪后，任务入口才启动 mission，避免录包初始化与任务启动争抢资源。
嵌套脚本由共享录包 helper 仲裁，只保留一个 recorder。检测到已有 `/rosbag2_recorder` 时跳过自动录包并继续入口流程。

## 6. 上下位机接口

### 6.1 原始协议

当前业务协议以 `auto_serial_bridge/config/protocol.yaml` 为准：

```text
上位机 -> 下位机:
  ChassisVelocity(0x01): linear_x, angular_z
  TrajectoryHeader(0x10): trajectory_id, point_num
  TrajectoryPoint(0x11): trajectory_id, point_id, time_from_start, joint_1..4, gripper_action
  ArmControlMode(0x12): control_mode (0: TRAJECTORY, 1: STOP, 2: ESTOP)

下位机 -> 上位机:
  ArmStatus(0x20): trajectory_id, task_status, progress, joint_1..4, joint_1..4_velocity, gripper_state, error_code
```

- 新串口协议将机械臂运动学逆解与轨迹生成交给上位机，不再定义旧的 `ArmTarget(0x13)` 与 `StateMachineCommand(0x11)` 报文。正式 mission 与相机调试客户端共用 planner 编排，区域通知只发 ROS String，不让下位机执行比赛策略。
- 轨迹下发时序：上位机先发送 `TrajectoryHeader(0x10)` 声明点数，随后发送 N 个 `TrajectoryPoint(0x11)` 写入下位机缓冲区，最后发送 `ArmControlMode(0x12)` 启动执行。
- 状态与遥测：下位机以 20Hz (50ms) 周期主动上报 `ArmStatus(0x20)`，包含当前任务状态码、进度、4关节角度/角速度、夹爪状态与错误码。
- 停止与急停：平滑停止 `STOP` (mode=1) 清除未走完轨迹并恢复 IDLE；急停 `ESTOP` (mode=2) 切断电机脉冲进入 EMERGENCY 锁定，需先发 STOP 指令解除锁定。
- Header、Point、Mode 使用可靠消息传输 (Reliable ACK)；底盘速度 `ChassisVelocity(0x01)` 高频刷新，不做重传。导航与 Look 并行；开窗要求两者成功完成，观测到目标冻结/提交之间的双机构静止由现场保证，不设置独立静止等待。运动入口仍检查新鲜有效反馈，不能仅凭 planner 的单运动占用权认定整机安全。

当前串口桥按消息 ID 分别排队重传。规划器连续发布 Header、Points、Mode 时，桥尚未提供跨消息类型的 ACK 顺序保证；实机使用前需完成顺序保障与端到端验证。


当前 mission 的通信和识别超时如下：

| 等待项目 | 超时 | 超时后的处理 |
| --- | ---: | --- |
| 启动节点/服务/端点 | 每项不超过 10 秒 | 告警继续启动 mission，外围子进程退出不终止整场；仍禁止重复控制节点 |
| planner 目标、驻留与预设执行 | 无前端执行期限；端点发现最多 10 秒 | 等待下位机结果或主动 STOP/取消；失败局部跳过，未知运动仍取消/STOP |
| camera 目标识别、二维码计划 | 各 10 秒 | 识别失败局部跳过；QR 任一步失败使用独立固定计划，不跳过区域 |
| `FollowWaypoints` action server | 5 秒；goal 应答与取消各最多 10 秒 | 已接受 goal 的执行等待终态，无推进期限；Look 失败可触发取消，仅导航保留既有重试 |
| 串口可靠消息 ACK | 每 1 秒重发 | 收到 ACK 或进程退出前持续重发；`reliable_max_retries=3` 只是每累计三次输出告警，不是停止重发 |

局部失败不提前结束 A/B/C 软件流程，A 左右侧和 B 冻结目标按顺序处理，不跨层传播 `safe_to_continue`。未知运动先取消导航并 STOP，确认派发应答后只尝试一次 Home；恢复不做独立停稳循环或用 sleep 放行。预设入口直接检查导航终态、新鲜无错误、有限且在限位内的低速四轴反馈，不满足即跳过；planner 仍拒绝忙状态及 STOP 后未恢复 IDLE 的请求。不得自动清除 EMERGENCY。没有非导航重试，也没有到点侧向复核。
正式 mission 姿态遵循 `arm_presets.yaml`：`<side>_look_<zone>` / `<side>_store_<zone>`（left/right，a/b/c）、`qr_scan`、`shenzhi`、`home`，无中间映射字典。
真实轨迹由用户配置，已定义名称和坐标以源码 YAML 为准；客户端不覆盖已有坐标，不添加空预设，缺失返回失败。Look 完成后在实测静止角执行 OPEN/HOLD，再开识别窗口；Store 包含 OPEN/驻留，Home 保持，视觉请求末点 CLOSE。Cartesian 服务没有驻留字段，随后复用 `move_joints` 在实测原始四轴角执行 HOLD，耗时读取 planner `default_dwell_ms`；不自行生成轨迹或用 sleep 宣告动作完成。

正常 A 每点、B/C 扫码、B 每树、C 每项结束后进入 `shenzhi`，最后一项与区域出口重合时不重复调用。失败仍一次安全 Home，不追加保护动作；必要保护位未确认时禁止后续导航，软件任务仍推进。导航交接认可已完成且当前实测静止的 `shenzhi`；导航与区域 Look 的既有并行方式保留。

机械臂运动入口的低速阈值取 mission `stationary.joint_speed_rps`，反馈新鲜度取 planner 配置；原始 ArmStatus/遥测的非有限速度不替成零。原开窗前 `stable_stop` 与恢复前独立停稳循环已移除，不宣称仍执行连续 0.5 秒或 `/odometry` 静止检测。识别、扫码、端点和取消各有独立期限，机械臂及已接受的导航 goal 等待执行结果。

现有 `/arm_planner_node/joint_states` 使用 `fruit_interfaces/msg/ArmTelemetry`：同一帧包含原始硬件角、已校正的模型角及真实 TCP XYZ/pitch。控制台和 mission 直接读取模型/TCP，不重复逆映射或 FK；原始硬件角仍用于原始角服务和安全检查。话题名不变但类型与标准 JointState 不兼容，字段单位和迁移要求见 [planner 接口文档](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md)。

字段类型与线协议布局见 [协议参考](LOWER_CONTROLLER_COMMANDS.md) 和
[planner 通信文档](../src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md)。新协议没有旧 `z_type` 高度索引。

### 6.2 上层语义接口

已实现的机械臂服务由 `fruit_arm_planner` 提供：

- `/arm_planner_node/move_cartesian`：固定机械臂根部坐标系 XYZ（m）、绝对末端 pitch（deg），可选 PTP/LIN。
- `/arm_planner_node/move_joints`：四个原始硬件目标角（deg），不是连杆角，不再次叠加偏置。
- `/arm_planner_node/execute_preset`：执行源码 YAML 中存在的预设名称。
- `/arm_planner_node/stop`、`/arm_planner_node/emergency_stop`：发布停止/急停；服务成功不是硬件已停稳确认。

mission 直接使用区域 Look/Store、`qr_scan`、`shenzhi` 和 `home` 预设，无下位机任务状态切换。旧正常 `retract` 调用位置使用 `shenzhi`；服务只执行 YAML 中实际存在的预设，不能假定现场已补齐轨迹。
证据采纳由 `/fruit_mission/observation_window` 观察窗口驱动，与机械臂姿态解耦。
新客户端必须按包级 API 接入；旧方向/收臂接口不再作为新增功能的范例。

轴向导航的差速命令通过 `twist_mux` 发布到 auto_serial_bridge 的速度订阅话题，不经过离散 `task_id` 动作协议。

### 6.3 下位机硬保护

以下保护必须由下位机独立完成，不能依赖 NUC 正常工作：

- 心跳超时立即把底盘速度清零；
- 急停直接切断或禁止所有危险运动；
- 机械臂关节软限位、硬限位、堵转和超时保护；
- 舵机和末端动作超时进入硬件安全状态；故障回传需在通信协议增加明确失败状态后接入；
- 收到非法任务、非法状态或超范围关节角时拒绝执行；
- 通信恢复后不得自动继续失联前未确认完成的危险动作。

## 7. A、B、C 区编排契约

### 7.1 A 区

```text
ENTER_A
  -> FOLLOW_COVERAGE_ROUTE
  -> for waypoint in a_waypoints:
       NAVIGATE_TO_WAYPOINT
       || EXECUTE_LEFT_LOOK
       WAIT_NAV_AND_LOOK_SUCCESS
       for side in ["left", "right"]:
         [right] -> EXECUTE_RIGHT_LOOK
         OPEN_GRIPPER_AND_HOLD -> OPEN_HARVEST_WINDOW
         WAIT_CAMERA_TARGET(ripe)
         [has target] -> TRANSFORM_TO_ARM_BASE -> CLOSE/DWELL -> HARVEST_OPEN/DWELL -> HOME
         [no target] -> SAFE_RECOVER
       [normal work completed] -> SHENZHI
  -> RESUME_COVERAGE_ROUTE
  -> LEAVE_A
```

- A 区作业点按配置顺序先左后右依次转向并开启 camera harvest 观察窗口；左侧失败也推进右侧，实际运动受入口保护。
- A 区候选只允许精确字符串 `ripe` 和 `unripe`；只有 `ripe` 才能进入锁定，`unripe` 不触发抓取。
- 仅当锁定成熟且深度有效的目标时转换坐标并调用 `move_cartesian`。未识别到目标或仅有未成熟目标时正常跳过。
- 导航与 `left_look_a` 并行；到点且 Look 完成后 OPEN/HOLD 再识别。每次抓取完成 `left_store_a` / `right_store_a` 及 Home，再处理另一侧。
- 局部失败只做一次必要 Home 后推进；运动未知不能因 STOP 已发布被视为已停稳或直接放行导航。

### 7.2 C 区

```text
READ_C_QR
  -> ANNOUNCE_ORDER
  -> for task_index in 0..7:
       NAVIGATE_TO_POSITION
       || EXECUTE_PLANNED_SIDE_LOOK_ONCE
       WAIT_NAV_AND_LOOK_SUCCESS
       OPEN_HARVEST_WINDOW
       -> DETECT_EXPECTED_RIPE_TYPE
       [no match/unripe/type mismatch] -> ANNOUNCE_SKIP -> ADVANCE
       [expected ripe candidate] -> TRANSFORM_TO_ARM_BASE -> CLOSE/DWELL -> STORE_OPEN/DWELL -> HOME -> ANNOUNCE_RESULT -> SHENZHI -> ADVANCE
  -> LEAVE_C
```

- C 区导航目标及 Look 侧向由现有路线规划结果提供；不增加到点方向复核。
- 二维码输出只决定严格任务序列，不允许按路线距离重排。
- 到位后即使没有目标，也完成稳定停车、识别、播报和推进。
- 导航时提前调用规划侧的 Look；到点并确认双机构静止后，把 `expected_ripe_type` 交给感知节点过滤坐标，继续使用现有跨帧聚类和评分。
- C 区每个位置只放置一种固定果蔬。方向动作完成后，第一条新鲜的非空 camera 类型结果中没有 `expected_ripe_type` 时立即结束本项：同品种不成熟记为 `target_unripe`，其他品种记为 `type_mismatch`，播报后收臂并进入下一项，不等待期望类型在原位置再次出现。只有类型结果为空，或期望类型已经出现但尚未形成有效抓取坐标时，才继续等待 camera 总超时。
- 不成熟或种类不符时不调用目标运动；左右观察姿态仍通过 planner 预设执行。
- C 区每项开始沿用实时 TF/既有历史位姿规划逻辑；无法规划时记录、播报并跳过当前项，不无限停留。
- FollowWaypoints 失败时记录状态并跳过当前项，不在未到达的位置识别/抓取，继续按二维码顺序推进。
- C 区 `axis_waypoint_action_server` 将中间 waypoint 视为通行参考点：横向误差超限时先以低速继续修正，并允许一次从实时位姿重试；重试后距离 waypoint 不超过 `waypoint_pass_tolerance` 时容错通过。最终 goal 仍使用严格位置和方向收敛。
- FollowWaypoints 已接受但执行为 `ABORTED` 时，Mission 按 `missed_waypoints` 截取未完成路线后缀重试，最多使用既有的三次 action 尝试，不重复经过已完成 waypoint。TF 不可用或取消请求仍保持停车和恢复边界。
- `axis_waypoint_action_server` 读到超过 `tf_timeout_sec` 的瞬时陈旧 TF 时，先发布零速度，再在同一个 `tf_timeout_sec` 窗口内等待新变换；新鲜 TF 到达后才恢复控制，窗口结束后仍没有新鲜 TF 则保持停车并将 action 标记为 `ABORTED`，避免发布周期边界上的单次陈旧采样立即终止导航，也禁止等待期间沿用上一条速度命令。
- 每项任务开始时记录 task index；正式任务使用每次服务结果，planner 校验新收到的匹配轨迹 ID 终态，不依赖旧方向回调。
- C 区局部姿态、识别或目标运动失败后安全恢复/跳过，不提前结束整个比赛流程。

### 7.3 B 区

```text
USE_SAVED_B_PLAN
  -> for position_id in 1..8:
       NAVIGATE_TO_TREE || EXECUTE_LOOK
       WAIT_NAV_AND_LOOK_SUCCESS
       OPEN_HARVEST_WINDOW_ONCE
       DETECT_ALL_FRUITS_IN_FRAME
       ANNOUNCE_RECOGNITION_AND_QR_MATCH
       FREEZE_UP_TO_TWO_MATCHED_RIPE_BASE_TARGETS
       for frozen_target:
         CLOSE/DWELL -> HARVEST_OPEN/DWELL -> HOME
       [normal work completed] -> SHENZHI
       ADVANCE_ONCE
  -> CLOSE_OBSERVATION_WINDOW
  -> END_MISSION_FLOW
```

- B 计划在首次经过 B 入口时已经扫码保存，完成 C 后由晚启动订阅者读取。
- 每棵树完成 50 帧统计后，以置信度最高的稳定物理目标确定整树为苹果或梨；不同品种的稳定目标在感知侧丢弃，同品种最多保留两个目标，成熟度仍分别保留。
- 无效中心深度的检测保留二维证据，但不得生成三维抓取点。
- B 区机械臂目标由 mission 转为固定基座坐标，调用 `move_cartesian` 并等待服务执行结果，不再发送 `ArmTarget(0x13)` 或等待目标 `callback_id=3`。
- B 区 camera 固定采集 50 帧，按 3D 轨迹输出最多两个稳定目标；原始类型、检测框和标注图保留误识别证据。二维码期望类型不传入感知节点，mission 做品种匹配、成熟筛选。
- 成熟且匹配目标按既有原像素 `score = abs(pixel_x - image_width / 2) - pixel_y` 排序，同分保留原顺序，最多取两个。首次观察姿态下将两个目标一次性转换为基座坐标/绝对 pitch，并根据未补偿 Base Z 判定 Up/Down 层级与对应的 `left_store_b_up/down` 或 `right_store_b_up/down` 预设，后续直接用冻结请求与 Store 名称，不重新观察/换算。越界目标跳过。每抓一次都完成对应 Store 和 Home。
- 第一抓失败也推进第二个冻结目标，未获有效反馈时该次物理动作被拒绝，不重试抓取或重新观察。
- `test_camera_arm_target.sh --zone A|B|C --left|--right [--debug]` 专用于单点联调，旧 `--model` 删除：先可靠交付区域，Home→区域 Look→新鲜静止角 OPEN/HOLD→识别→CLOSE/HOLD→区域 Store。A/C 用蔬菜模型，B 用水果模型且只选成熟目标；不调用跨区 `shenzhi`。端点与识别独立限时，运动等待下位机结果。脚本会驱动硬件，不是自动验证工具。

### 7.4 语音时序

- B 区二维码或固定计划异步播报，C 区按配置提前量等待；同步语音等待不超过 10 秒。语音设备或素材缺失只告警，继续比赛。
- C 区二维码计划按 8 项独立短任务顺序播报，等待时间缩短为预计总音频结束前 `c_qr_announce_lead_sec` 秒；当前配置文件为 `10.0` 秒。达到提前点后进入机械臂收缩和下一项任务，音频在后台 FIFO 队列继续播放至结束。
- 采用维什戴尔（Wis'adel）gremlin 音色预生成短语素材与本地 WAV 拼接播报；移除 Piper 实时模型及 `qr_length_scale`。
- B 区二维码摘要使用“`第一棵树苹果；第二棵树梨子`”格式；C 区二维码摘要使用“`第一项，四号位置，南瓜；第二项，三号位置，西红柿`”格式。
- 任务正常遍历结束时直接播报各品种采摘工序完成总数汇总；异常或 Ctrl-C 退出立即停播、不汇总。计数依据软件确认的 CLOSE、Store、Home 完成记录，不作为物理入筐的硬件证明。

## 8. 维护与实机验收边界

- 当前导航、感知、扫码、航点和 planner 均已有实现，不再按“建包骨架、等待机械尺寸、双侧相机”的旧阶段计划开发。
- 所有运行终端加载 `scripts/setup_fruit_runtime.bash`；构建与 CLI 方法见根 `AGENTS.md`。
- `start_fruit_mission.sh` 自动启动唯一串口桥与 planner-only launch；检查当前服务/端点，超时告警，外围进程退出不结束 mission。真实预设仍由用户补齐，不能视为已验收整机。
- mission 直接启动 `fruit_perception.launch.py`，不调用调试脚本。
  调试入口 `start_fruit_perception.sh` 必须提供 `--zone A|B|C`，无需桌面即可从 RobViz 查看。
  相机启动启用 `initial_reset`，这不是硬件 MIPI 故障修复保证。
- 验证轨迹跨消息类型的装载顺序、ACK 和匹配终态；ROS 发布先后或固定延时不等于下位机完整装载。
- camera 坐标换算要求观测姿态静止、新鲜实测四轴角及正确外参；服务确认工序完成不是视觉证明果实入筐。
- 断联、STOP/ESTOP、底盘与机械臂互锁必须以实际硬件反馈验收，不能把协议意图或静态检查写成实测结果。
- 保留已记录的历史测量与验收边界；本次文档核对未构建、未启动节点、未驱动机械臂或底盘。
