# 水果采摘机器人软件架构与实施路线

## 1. 当前目标

省级选拔赛范围为 A、B、C 区，D 区暂不接入。当前固定流程契约为：

- A 区发布 `SIDES`，左右 USB 相机使用 `A.onnx` 只判断 `ripe/unripe`，不判断水果品种。
- B 区入口发布 `QR`，连续三帧确认并保存 B 计划，但暂不抓取。
- 前往 C 区入口时保持 `QR`，确认 C 计划后使用 `TREE`，按 C 计划严格执行 8 项。
- 完成 C 后返回 B，发布 `TREE`，D435i 对整树的全部果实进行检测和测深，并按 B 计划位置 1 到 8 作业。
- B 区完成后发布 `OFF` 并结束当前软件任务流程。当前范围明确不实现收集更换区、收获汇总播报、返航起始区和返航后的结束判定。

上位机运行在 Intel Core i5/i7 NUC，负责定位、导航、识别、任务决策，以及 B/D 区机械臂目标生成；下位机负责底盘、舵机、关节和末端执行器的实时闭环执行及硬件保护，并在收到当前区域和左右抓取方向命令后执行对应位置动作。

## 2. 总体架构

```text
                    +------------------------+
二维码识别包 ------> | fruit_mission          |
                    | A/B/C 比赛状态机        |
侧相机候选 --------> | 顺序、播报、日志、决策  |
D435i 整树目标 -----> |                        |
                    +-----------+------------+
                                |
             Action /follow_waypoints | typed hardware actions
                                v
+-------------------+   +------------------------+
| fruit_navigation  |   | fruit_hardware_bridge  |
| Point-LIO 里程计   |   | ROS 语义接口           |
| 轴向定点导航服务端 |   | task/callback 映射     |
| 偏航角平滑控制     |   +-----------+------------+
+---------+---------+               |
          |                         v
          | cmd_vel         +--------------------+
          v                 | auto_serial_bridge |
+-------------------+       | YAML/串口/心跳/CRC  |
| twist_mux         |       +----------+---------+
| 导航/遥控速度仲裁  |                  |
+---------+---------+                  v
          +---------------->+--------------------+
                            | 下位机              |
                            | 底盘/舵机/4DOF/末端  |
                            +--------------------+
```

必须保持以下边界：

1. 比赛业务节点不直接读写串口字段，只调用有明确语义的 ROS 2 topic、service 或 action。
2. `fruit_hardware_bridge` 是业务语义与 `task_id/task_status` 协议之间唯一的映射层。
3. `auto_serial_bridge` 只处理串口帧、代码生成、握手、心跳、校验和可靠传输，不承载比赛流程。
4. 下位机不决定 A/B/C 任务顺序和是否采收；上位机只上报所在区域并指定左/右抓取方向，下位机按当前区域解释 A/B/C 不同的识别和动作方向；B/D 执行上位机给出的机械臂目标，并通过 callback 返回执行阶段。
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
│   ├── livox_ros_driver2/        # MID-360 驱动
│   └── realsense-ros/            # D435i 相机驱动
├── navigation/
│   ├── point_lio/                # Point-LIO LiDAR-IMU 紧耦合里程计
│   ├── loam_interface/          # TF 转换与里程计适配
│   ├── sensor_scan_generation/   # 点云生成
│   ├── fruit_navigation/         # 轴向航点跟踪 action server
│   └── fruit_nav_bringup/        # 纯里程计导航集成 bringup
├── auto_serial_bridge/           # 串口协议生成与传输
├── fruit_hardware_interfaces/    # 上下位机控制与回调 msg
├── fruit_hardware_bridge/        # 语义动作到 task/callback 协议的适配
├── fruit_perception/             # 双侧 UVC、D435i、模式管理与 OpenVINO
├── fruit_qr_planner/             # D435i 彩色图扫码并持久发布 B/C 计划
├── fruit_waypoint_recorder/      # 航点录制与 Web 航点编辑器
├── fruit_mission/                # A/B/C 比赛状态机与编排
└── joy_linux/                    # 手柄遥控节点
```

`auto_serial_bridge` 不由本仓库作为 Git submodule 管理，而是从 Edmounds fork 的
`dev` 分支独立克隆；父仓库通过 `.gitignore` 忽略该目录。

## 4. 导航结构

### 4.1 纯里程计导航架构

- 导航采用纯 Point-LIO 激光雷达-IMU 里程计方案，彻底去除了离线建图（GLIM）与重定位（GICP）环节。
- 坐标系拓扑简化为 `odom -> base_link -> sensors/arm`，不再使用 `map` 全局坐标系。
- Point-LIO 在开机时以静止状态建立初始参考系，`loam_interface` 在接收到稳定里程计后开始发布 `odom -> base_link` TF。
- 航点文件（`src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml`）中的坐标全部基于 `odom` 坐标系。

### 4.2 轴向定点导航控制器 (`fruit_navigation`)

- `axis_waypoint_action_server` 提供 `/follow_waypoints` Action 接口。
- 支持按轴直线行走、原地旋转调整航向（`rotate_in_place`）与沿路径平滑插值偏航角（`blend_on_path`）。
- 速度输出经由 `twist_mux` 输出至串口控制器。


## 5. 感知结构与资源调度

### 5.1 双侧相机

左右 USB UVC 相机只在 A 区的 `SIDES` 模式采集。OpenVINO 模型在感知节点启动时一次性编译，不在区域切换时重新加载：

- A 区使用 `fruit_perception/model/A.onnx`，该模型只有 `ripe` 和 `unripe` 两个类别。
- C 区不使用左右 USB 相机；C 模型只由 D435i RGBD 流使用。
- 左、右相机只持有 A 区异步推理请求，当前区域不是 A 时不输出侧相机推理结果。
- 不保留“所有非 B 区都使用 C 模型”的兜底逻辑；区域不匹配时不输出侧相机推理结果。
- 左右相机分别完整统计 3 秒内实际完成的推理结果，用同类别 ROI 的 IoU 做跨帧关联，
  不设置最少出现次数。候选按置信度中位数和 ROI 水平居中度中位数以 70%/30% 评分；
  与最高分相差不超过 0.05 时优先置信度中位数更高者。
- 每个侧相机只在 `/fruit_perception/{left,right}/types` 发布其锁定目标的一个类别；
  获取期间或锁定目标漏检时发布空列表，不用同帧其他目标顶替。标注图仍显示全部原始检测框。
  A 区每次正式观察会重新请求 `SIDES`，让感知节点清除沿途旧锁并从停车后的帧重新统计。
- YOLO11n 模型先导出 OpenVINO FP16；只有在真实 NUC 上出现持续算力压力后，再做有代表性标定集的 INT8 量化。
- 首版目标为每侧约 `5-10 FPS`，输入尺寸先使用 `640`；资源不足时先降低相机采集帧率或推理帧率，不先破坏导航频率。
- 行驶阶段侧相机只输出粗候选：类别、成熟度、置信度、图像位置、左右侧、相机模式和时间戳。
- 单帧检测不能直接触发抓取。候选必须经过短时多帧确认，避免晃动、遮挡或曝光变化导致频繁停车。
- `QR`、`TREE` 模式关闭左右相机，避免三路相机同时采集和推理。
- A 区最后一个点完成后任务节点发布 `OFF`，在进入 B 区二维码前释放两路 USB 采集和侧相机推理资源。

### 5.2 D435i 扫码与整树识别

D435i 的固定原生流为 `1920x1080@30 RGB8` 和 `1280x720@30 Z16`，开启同步、
深度对齐与 RGBD。对齐后的两幅消息均为 1920x1080，但深度信息源仍是
1280x720。`QR` 与 `TREE` 之间不重启相机：

1. `QR` 直接把 D435i 彩色原图送给 Python/OpenCV 扫码节点，按 B 后 C 的顺序各连续确认三帧。
2. B/C 计划使用 Reliable、Transient Local、深度 1 发布，C 确认后销毁图像订阅。
3. `TREE` 只执行 D435i RGBD 水果推理，整帧 letterbox 到 640x640，不做分块。
   D435i 在启动时持有 A/B/C 三套推理请求，并严格按 `/fruit_mission/current_area`
   选择：A 区用 `A.onnx`，B 区用 `B.onnx`，C 区用 `C.onnx`。
4. `/fruit_perception/decision` 按 `source` 发布左右相机和 D435i 的统一决策消息；D435i 消息携带一帧内的全部目标及原图 ROI，中心深度无效时保留二维结果。
5. 感知节点完整连续观察 35 个推理帧，按 3 cm 三维距离把各帧有效检测关联为物理目标；
   不设置最少出现次数。每个目标使用置信度
   中位数和 ROI 水平居中度中位数按 70%/30% 评分；与最高分相差不超过 0.05 时，
   置信度中位数最高者胜出。选中轨迹按逐轴 2σ 剔除坐标离群点，以剩余坐标的均值
   作为目标点。标准差只写入诊断日志，不阻止最高分目标发布。
6. 选中目标后感知节点锁定其三维位置，并只在锁定当帧发布一次上述 35 帧统计均值到
   同一消息的 `has_target=true` 和 `target_point` 字段；锁定期间的单帧原始点不再发布。锁定目标单帧漏检时不使用其他目标顶替；
   连续漏检 8 帧后才清除锁并重新执行多帧获取。TREE 模式或区域改变也会清除旧锁。
7. 测试客户端和 mission 都只等待一个时间戳新鲜、`has_target=true` 且坐标有限的 `decision`，直接生成
   `/fruit_hardware_bridge/arm/target`，不再收集第二个 8 点窗口或重复平均。

4DOF 机械臂不能按通用 6DOF 末端位姿设计。应固定末端接近方向或腕部姿态，只求解比赛需要的受限位置和姿态。机械尺寸确定后，再选择解析 IK 或轻量数值 IK；首版不必为了发送四个目标角而引入完整 MoveIt 运行栈。

### 5.3 NUC 资源优先级

运行优先级从高到低为：

1. 串口心跳、急停和底盘速度输出；
2. Point-LIO、small_gicp、TF 和 Nav2 局部避障；
3. D435i 停车扫码或整树识别和测深；
4. 左右侧相机 YOLO；
5. 可视化、图像落盘和调试日志。

`QR/TREE` 模式会停止左右侧采集和 YOLO，把带宽与算力让给 D435i。正式比赛不启动 RViz，不持续保存原始图像和完整点云，只记录状态事件、检测摘要和故障证据。

任务使用 `--debug` 时，rosbag 仍记录导航、Action、雷达、点云和控制诊断，但视觉数据限定为
左右 USB 原始图/标注图及 D435i 彩色原图、对齐深度图、标注图。未对齐 D435i raw depth、
RGBD 复合消息和 `compressed`/`compressedDepth`/`theora` 派生图像进入黑名单，避免同一帧
多次 DDS 序列化和落盘。左右 `usb_cam` 的各 transport 话题分别位于自己的相机命名空间。
调试监视器通过 color/depth metadata 与进程内 RGBD 回调产生的轻量 Header heartbeat 计算
三条流的实际频率和最后帧时间，不直接订阅高带宽 RGBD 消息。
完整任务脚本会在导航、感知、串口桥和相机模式全部初始化完成且相机处于 `OFF` 后启动录包，
确认 `/rosbag2_recorder` 就绪后才启动 mission，避免录包初始化与 D435i 启动争抢资源。

USB 接线时，D435i 与两台 UVC 相机尽量分配到不同 USB 控制器；两组相机通过模式互斥采集，避免三路视频同时占满一个 USB 总线。

## 6. 上下位机接口

### 6.1 原始协议

当前业务协议以 `auto_serial_bridge/config/protocol.yaml` 为准：

```text
上位机 -> 下位机:
  StateMachineCommand(0x11): task_id, task_status
  ArmTarget(0x13): target_x, target_y, target_z, z_type
  VelocityCommand(0x14): linear_x, angular_z

下位机 -> 上位机:
  ExecutionCallback(0x12): callback_id, callback_status
```

- 原 `FruitDetection(0x10)` 已删除且 ID 暂不复用；感知结果只留在 ROS 话题中供上位机决策。
- `task_id=0/1/2` 分别表示夹爪、摄像头云台和 ABC 通用抓取方向；当 `task_id=2` 时，`task_status=0/1` 分别表示左侧和右侧，具体动作由下位机结合最近一次 `task_id=5` 所在区域判断。A 区将该命令解释为对应方向的完整开环抓取任务，B/C 区仍只执行各自既有的方向动作。
- `task_id=4` 表示进入二维码识别姿态，当前仅定义 `task_status=0`。
- `task_id=5` 表示机器人所在区域，`task_status=0/1/2/3` 分别表示 A/B/C/D 区。
- `task_id=6` 表示返回初始姿态，当前仅定义 `task_status=0`。
- `task_id=7` 表示机械臂收缩姿态，当前仅定义 `task_status=0`，用于转弯通过狭窄通道前收回机械臂。
- `callback_id=0/1/2/3/4/5/6/7` 分别表示夹爪、摄像头云台、ABC 通用抓取方向任务、B/D 机械臂目标、二维码识别姿态、所在区域、返回初始姿态和机械臂收缩姿态；A 区的 `callback_id=2, callback_status=0` 表示对应方向的开环抓取已经完成。
- `callback_status=1` 表示正在执行，`callback_status=0` 表示执行完成，`callback_status=2` 表示下位机因自身动作故障执行失败。
- 左右抓取方向命令和 B/D 机械臂目标各发布一次，并使用可靠消息保证送达；链路 ACK 只表示收到数据，只有对应的 `ExecutionCallback` 完成状态才能推进任务状态机。
- 舵机、末端定位等离散动作使用可靠消息；底盘速度属于连续刷新数据，不做历史重传。
- 上位机重复发送同一离散状态时，下位机不得重复执行已经完成的危险动作。
- 上位机收到 `callback_status=1` 后继续等待，但不会重新计算超时起点；从命令发布开始 30 秒内必须收到 `callback_status=0/2` 终态。
- `callback_status=2` 和 30 秒内没有终态回调是两类不同故障，分别记录“下位机执行失败”和“通信超时”；通信超时不得解释为动作完成。

当前 mission 的通信和识别超时如下：

| 等待项目 | 超时 | 超时后的处理 |
| --- | ---: | --- |
| 串口桥 ROS 命令/回调端点 | 10 秒 | 记录端点数量并返回失败；端点不可用时无法下发安全姿态命令 |
| 方向、二维码姿态、区域上报、`ArmTarget`、返回初始姿态、收臂等执行终态 | 30 秒 | 记录通信超时；按下方区域恢复规则进入安全姿态 |
| D435i 目标识别 | 20 秒 | 当前目标记为识别超时且不发送 `ArmTarget`；B/C 区按逐项流程发送 `task_id=7`，A 区不使用 D435i 目标识别 |
| Nav2 `FollowWaypoints` action server | 5 秒 | 本次导航返回失败，由现有区域流程决定跳过、重试或停止；已发送的导航 goal 仍没有执行总时限 |
| 串口可靠消息 ACK | 每 1 秒重发 | 收到 ACK 或进程退出前持续重发；`reliable_max_retries=3` 只是每累计三次输出告警，不是停止重发 |

机械臂动作失败、感知失败和通信超时的安全恢复规则：

| 当前区域 | 安全姿态 | 后续推进条件 |
| --- | --- | --- |
| A 区 | 发送 `task_id=6, task_status=0` 返回 home | 收到 `callback_id=6, callback_status=0` 后继续下一个任务；恢复未完成则停止 A 区流程 |
| B/C 区 | 当前树位或当前项发送 `task_id=7, task_status=0` 收臂 | 收到 `callback_id=7, callback_status=0` 后继续下一棵或下一项；收臂未完成则停止区域流程 |

`task_id=6/7` 本身就是安全恢复命令。它们失败或超时时只记录错误并停止，不再递归发送另一条恢复命令。二维码姿态、区域上报等后续流程必需命令失败时也会先尝试上述安全恢复，但即使恢复成功，当前阶段仍按原有规则返回失败，不会把恢复成功当成原命令成功。

字段类型、字节偏移和 `z_type=0..15` 的高度索引见构建生成的 `PROTOCOL_DOC.md`。

### 6.2 上层语义接口

业务层不直接构造 `task_id`。建议由 `fruit_hardware_bridge` 暴露以下强类型接口：

- `SetPickDirection`: 左侧或右侧；侧相机确认目标后只调用一次并等待回调。下位机用最近一次所在区域区分 A/B/C 的动作；A 区回调完成即代表开环抓取完成。
- `SetCameraMode`: 左/右/双侧，`GROUND` 或 `TREE`，等待舵机到位反馈。
- `MoveArmJoints`: 四个目标角、速度档或速度比例、超时，持续反馈执行阶段和当前角度。
- `RunEndEffector`: 夹取、释放、复位等离散动作。
- `SetStorageState`: 需要下位机控制入仓机构时使用。
- `LowerControllerState`: 心跳、急停、故障码、当前 task/callback 和关节状态。

Nav2 的最终差速命令由 `cmd_vel_safety_mux` 发布到 auto_serial_bridge 的速度订阅话题，不经过离散 `task_id` 动作协议。

### 6.3 下位机硬保护

以下保护必须由下位机独立完成，不能依赖 NUC 正常工作：

- 心跳超时立即把底盘速度清零；
- 急停直接切断或禁止所有危险运动；
- 机械臂关节软限位、硬限位、堵转和超时保护；
- 舵机和末端动作超时进入硬件安全状态；故障回传需在通信协议增加明确失败状态后接入；
- 收到非法任务、非法状态或超范围关节角时拒绝执行；
- 通信恢复后不得自动继续失联前未确认完成的危险动作。

## 7. A、B、C 区状态机契约

### 7.1 A 区

```text
ENTER_A
  -> SET_CAMERAS_GROUND
  -> FOLLOW_COVERAGE_ROUTE
  -> CONFIRM_SIDE_CANDIDATE
  -> CANCEL_NAV_AND_LOCK_CHASSIS
  -> STABLE_STOP
  -> PUBLISH_PICK_DIRECTION_ONCE
  -> WAIT_PICK_DIRECTION_CALLBACK_COMPLETED
  -> RESUME_COVERAGE_ROUTE
  -> LEAVE_A
```

- 没有候选时沿固定覆盖路线继续行驶。
- A 区候选只允许精确字符串 `ripe` 和 `unripe`；`ripe` 才允许抓取，`unripe` 不触发抓取。
- A 区不接受 `apple_ripe`、`pumpkin_ripe` 等带品种的旧类别，也不做苹果、梨或其他品种过滤。
- 未检测到成熟目标、目标不成熟或坏水果时，不向下位机发布左右抓取方向命令。
- 仅左侧或右侧存在成熟目标时分别发送 `task_status=0/1`，让下位机按当前区域执行对应方向动作。
- 侧面 USB 相机确认 `ripe` 后，mission 直接发送一次对应方向的 `task_id=2`。下位机在 A 区把方向观察和开环抓取作为同一个任务执行；mission 不切换 `TREE` 模式、不等待 D435i 目标，也不发送 `ArmTarget`。
- 业务层不重复发送该命令；`auto_serial_bridge` 负责可靠重传，只有收到 `callback_id=2, callback_status=0` 才把本次抓取记为完成并恢复路线。
- A 区方向开环抓取返回 `callback_status=2` 或 30 秒内没有终态回调时，先发送 `task_id=6` 并确认 home 完成；只有恢复成功才处理下一个候选或作业点。
- 路线必须保存当前段索引，抓取后从安全恢复点继续，而不是从 A 区入口重新开始。

### 7.2 C 区

```text
READ_C_QR
  -> ANNOUNCE_ORDER
  -> for task_index in 0..7:
       NAVIGATE_TO_POSITION
       STABLE_STOP
       VERIFY_POSITION_TASK
       PUBLISH_PICK_DIRECTION_ONCE
       -> WAIT_PICK_DIRECTION_CALLBACK_COMPLETED
       -> TREE_CHECK_EXPECTED_RIPE_TYPE
       [no match/unripe/type mismatch] -> ANNOUNCE_SKIP -> ADVANCE
       [expected ripe candidate] -> SEND_ARM_TARGET -> ANNOUNCE_RESULT -> ADVANCE
  -> LEAVE_C
```

- C 区导航目标由 `(position_id, approach_pose, expected_side)` 配置表提供；`expected_side` 会根据到位后的实际停车姿态转换成相对车辆的 `left/right`。
- 二维码输出只决定严格任务序列，不允许按路线距离重排。
- 到位后即使没有目标，也完成稳定停车、识别、播报和推进。
- 到位后先按实际停车姿态计算出的车辆相对侧发送一次方向命令，让机械臂从常态前视转到目标侧；随后 D435i 把当前二维码任务的 `expected_ripe_type` 交给感知节点过滤抓取坐标，并继续使用现有跨帧聚类和综合评分选择目标点。
- C 区每个位置只放置一种固定果蔬。方向动作完成后，第一条新鲜的非空 D435i 类型结果中没有 `expected_ripe_type` 时立即结束本项：同品种不成熟记为 `target_unripe`，其他品种记为 `type_mismatch`，播报后收臂并进入下一项，不等待期望类型在原位置再次出现。只有类型结果为空，或期望类型已经出现但尚未形成有效抓取坐标时，才继续等待 D435i 总超时。
- 不成熟、坏水果或种类不符时不发送 ArmTarget；车辆方向命令仍保留，因为它是进入左右观察姿态的必要动作。
- C 区每项任务开始优先使用实时 `map -> base_link` TF；TF 暂不可用时使用上一项目标点位姿作为规划起点，首项没有历史位姿才停止。
- FollowWaypoints 导航失败时记录失败状态，但仍继续当前目标的稳定停车、方向调整、识别和抓取流程，再按二维码顺序推进下一项，避免因导航 action 状态丢失而放弃现场目标。
- C 区 `axis_waypoint_action_server` 将中间 waypoint 视为通行参考点：横向误差超限时先以低速继续修正，并允许一次从实时位姿重试；重试后距离 waypoint 不超过 `waypoint_pass_tolerance` 时容错通过。最终 goal 仍使用严格位置和方向收敛。
- FollowWaypoints 已接受但执行为 `ABORTED` 时，Mission 按 `missed_waypoints` 截取未完成路线后缀重试，最多使用既有的三次 action 尝试，不重复经过已完成 waypoint。TF 不可用、GICP ERROR 或取消请求仍保持停车和恢复边界。
- 每项任务开始时记录 task index；当前方向回调协议只有 `callback_id`，无法区分同一 ID 的迟到回调，
  因此下位机协议增加命令序号前不得把跨任务回调校验写成已完成能力。
- C 区方向或 `ArmTarget` 执行失败、通信超时后，不直接停止整区任务；当前项继续发送 `task_id=7`，确认收臂完成后再按二维码顺序推进下一项。收臂失败或超时才停止 C 区流程。

### 7.3 B 区

```text
USE_SAVED_B_PLAN
  -> SET_MODE_TREE
  -> for position_id in 1..8:
       NAVIGATE_TO_TREE
       STABLE_STOP
       DETECT_ALL_FRUITS_IN_FRAME
       ANNOUNCE_RECOGNITION_AND_QR_MATCH
       PICK_BEST_MATCHED_RIPE_FRUIT
       ADVANCE_ONCE
  -> SET_MODE_OFF
  -> END_MISSION_FLOW
```

- B 计划在首次经过 B 入口时已经扫码保存，完成 C 后由晚启动订阅者读取。
- 每棵树完成 50 帧统计后，以置信度最高的稳定物理目标确定整树为苹果或梨；不同品种的稳定目标在感知侧丢弃，同品种最多保留两个目标，成熟度仍分别保留。
- 无效中心深度的检测保留二维证据，但不得生成三维抓取点。
- B 区机械臂目标继续使用 `ArmTarget(0x13)`，并等待 `callback_id=3` 的执行完成状态。
- B 区 D435i 固定采集 50 帧，按 3D 轨迹输出最多两个稳定目标；逐帧原始类型、检测框和标注图不做品种过滤，继续保留模型误识别证据。二维码期望类型不传入感知节点，mission 后续负责二维码品种匹配、成熟度筛选和单目标择优。
- mission 保留最终 B 目标的类型和三维坐标，先异步播报识别品种及其与当前树位二维码是否一致。种类一致时只在成熟目标中选择一个：D435i 光学坐标系使用 `score = abs(x) - y`，横向越偏、位置越高的目标得分越高；同分时保留感知侧已有的置信度和观测次数排序。选中后只下发一次 `ArmTarget`，不再返回同一方向抓第二个果实。B 区二维码计划播报同样异步入队，允许机械臂收缩和后续导航与播报并行。
- B 区方向或 `ArmTarget` 执行失败、通信超时后，当前树位继续发送 `task_id=7`；确认收臂完成后处理下一棵树，收臂失败或超时才停止 B 区流程。
- B 区单树测试使用 `test_d435i_arm_target.sh --zone B --left|--right --apple|--pear` 提供等价于二维码的期望品种，并复用 mission 的整树品种匹配、成熟度筛选和 `abs(x) - y` 单目标择优逻辑。完成、正常跳过、识别失败或机械臂目标失败后均发送 `task_id=7` 并等待收臂回调；完成窗口为空属于正常跳过，未等到完成窗口才属于识别超时。

### 7.4 语音时序

- B 区二维码计划解析成功后立即调用异步播报，任务流程不等待音频结束；后台语音队列保证音频按入队顺序完整播放。
- C 区二维码计划仍按整段串词播报，但等待时间缩短为预计音频结束前 `c_qr_announce_lead_sec` 秒；当前配置文件为 `10.0` 秒。达到提前点后进入机械臂收缩和下一项任务，音频继续播放至结束。
- B 区二维码摘要使用“`1苹果，2梨子`”格式；C 区二维码摘要使用“`1，12南瓜，2，11西红柿`”格式。Piper 的 `qr_length_scale` 默认配置为 `0.8`，只用于加快二维码摘要播报。

## 8. 现在就能完成的工作

在底盘和机械臂尚未完成时，优先做不会因机械尺寸变化而返工的部分：

1. 建立上述 ROS 2 包骨架及 `fruit_interfaces`，冻结模块边界和 topic/action 名称。
2. 新建本车专用 Nav2 配置和 launch，移除全向控制器、云台假坐标系及所有手柄遥控启动链路。
3. 建立 `map -> odom -> base_footprint` 的统一 TF 约定，整理 GLIM 地图到 PCD/栅格地图的离线流程。
4. 继续用现有 OpenVINO 模型，实机验证 D435i 整树 640x640 letterbox 推理能稳定看见两个果实。
5. 定义侧相机粗候选与 D435i 全目标输出之间的数据契约，不提前实现未知机械臂尺寸的正式 IK。
6. 实现 A/B/C 纯任务状态机、事件日志、播报触发和严格游标推进逻辑。
7. 按已冻结的 ABC 通用抓取方向命令和 task/callback 映射，实现 `fruit_hardware_bridge` 的上层语义接口。
8. 等机械臂尺寸、关节方向和限位确定后，补 URDF、相机外参、受限 IK 和安全抓取姿态。

用户当前选择暂不做 ROS bag、Gazebo 或假硬件验证，因此这些不作为第一阶段交付物。接口设计仍应允许后续添加回放或模拟下位机，而不修改比赛业务状态机。

## 9. 第一阶段完成标准

- `fruit_bringup` 能明确区分 `mapping`、`navigation`、`perception_only` 和 `competition_abc` 模式。
- 比赛模式不会启动 GLIM、RViz、手柄遥控或任何全向控制器。
- 所有底盘命令经过安全仲裁，只输出 `linear.x/angular.z`。
- 双侧相机共享一份 OpenVINO 模型，能独立报告左右候选和当前视角模式。
- C 区任务结构固定为 8 项有序队列，失败和跳过都只推进一次。
- B 区使用先前持久发布的计划，并按位置 1 到 8 处理每棵树；感知最多保留两个候选，mission 每棵树最多抓取一个成熟目标。
- 上层只使用语义化硬件接口，不出现散落的 `task_id` 数字常量。
- 下位机通信中断时，即使 NUC 任务节点卡死，底盘和机械臂仍由硬件侧进入安全状态。
