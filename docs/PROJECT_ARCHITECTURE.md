# 水果采摘机器人软件架构与实施路线

## 1. 当前目标

省级选拔赛范围为 A、B、C 区，D 区暂不接入。当前固定流程契约为：

- A 区发布 `SIDES`，使用左右 USB 相机完成识别和作业。
- B 区入口发布 `QR`，连续三帧确认并保存 B 计划，但暂不抓取。
- 前往 C 区入口时保持 `QR`，确认 C 计划后发布 `SIDES`，按 C 计划严格执行 8 项。
- 完成 C 后返回 B，发布 `TREE`，D435i 对整树的全部果实进行检测和测深，并按 B 计划位置 1 到 8 作业。
- B 区完成后发布 `OFF` 并返航。当前仓库只实现话题契约和感知侧，不实现导航与完整任务状态机。

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
             Nav2 action        | typed hardware actions
                                v
+-------------------+   +------------------------+
| fruit_navigation  |   | fruit_hardware_bridge  |
| GLIM 离线地图工具  |   | ROS 语义接口           |
| Point-LIO 里程计   |   | task/callback 映射     |
| small_gicp 重定位  |   +-----------+------------+
| Nav2 + 动态障碍物  |               |
+---------+---------+               v
          |                 +--------------------+
          | cmd_vel         | auto_serial_bridge |
          v                 | YAML/串口/心跳/CRC  |
+-------------------+       +----------+---------+
| cmd_vel_safety_mux|                  |
| 导航/抓取/急停仲裁 |                  v
+---------+---------+       +--------------------+
          +---------------->| 下位机              |
                            | 底盘/舵机/4DOF/末端  |
                            +--------------------+
```

必须保持以下边界：

1. 比赛业务节点不直接读写串口字段，只调用有明确语义的 ROS 2 topic、service 或 action。
2. `fruit_hardware_bridge` 是业务语义与 `task_id/task_status` 协议之间唯一的映射层。
3. `auto_serial_bridge` 只处理串口帧、代码生成、握手、心跳、校验和可靠传输，不承载比赛流程。
4. 下位机不决定 A/B/C 任务顺序和是否采收；上位机只上报所在区域并指定左/右抓取方向，下位机按当前区域解释 A/B/C 不同的识别和动作方向；B/D 执行上位机给出的机械臂目标，并通过 callback 返回执行阶段。
5. 只有速度仲裁节点能向串口通信层发布最终底盘速度，禁止 Nav2 和任务节点同时直接控制底盘。

## 3. ROS 2 工作区划分

保留现有第三方仓库，不直接把比赛业务写进上游代码：

```text
ros2_ws/src/
├── auto_serial_bridge/       # 已拉取，串口协议生成与传输
├── glim/                     # 仅用于赛前离线建图
├── pb2025_sentry_nav/        # 复用 Livox、Point-LIO、small_gicp 等模块
├── fruit_hardware_interfaces/# 上下位机控制与回调 msg
├── fruit_bringup/            # 全车 launch、参数、模式切换
├── fruit_navigation/         # 本车 Nav2 配置、地图、路线和速度仲裁
├── fruit_perception/         # 双侧 UVC、D435i、模式管理与 OpenVINO
├── fruit_qr_planner/         # D435i 彩色图扫码并持久发布 B/C 计划
├── fruit_hardware_bridge/    # 语义动作到 task/callback 协议的适配
└── fruit_mission/            # A/B/C 状态机（尚未创建）
```

`auto_serial_bridge` 不由本仓库作为 Git submodule 管理，而是从 Edmounds fork 的
`dev` 分支独立克隆；父仓库通过 `.gitignore` 忽略该目录：

```bash
git clone --branch dev git@github.com:Edmounds/auto_serial_bridge.git src/auto_serial_bridge
git -C src/auto_serial_bridge pull --ff-only origin dev
```

机械臂尺寸确定后再增加 `fruit_description`，保存 URDF、关节限制、相机和雷达外参。现在不要根据未知尺寸编写正式 IK 参数或运动轨迹。

`model_train_ws` 继续作为独立训练工作区。ROS 运行包只使用导出的模型，不依赖 Ultralytics 训练环境。

## 4. 导航结构

### 4.1 离线建图与比赛运行分离

- GLIM 仅在赛前建图阶段运行，输出全局点云先验图。
- 从同一建图结果生成 small_gicp 使用的先验点云，以及 Nav2 使用的二维占据栅格；两份地图必须使用同一原点和方向。
- 比赛运行时不启动 GLIM，避免与 Point-LIO、small_gicp、双路 YOLO 争抢 NUC 资源。
- 比赛时运行链为 `MID360 -> Point-LIO -> small_gicp -> map->odom`，Nav2 使用该 TF 完成全局导航。
- 随机障碍物不得固化到先验地图中，必须由 MID360 在线加入 local/global costmap。

### 4.2 不直接使用现有实车启动文件

当前 `pb2025_sentry_nav` 面向 RoboMaster 全向哨兵，其配置包含：

- 输出 `linear.y` 的 `pb_omni_pid_pursuit_controller`；
- `gimbal_yaw` 和 `gimbal_yaw_fake` 坐标系；
- 默认启动手柄遥控节点；
- 最高约 `2.5 m/s` 的比赛外场速度参数。

这些行为不适合差速/滑移转向采摘车，也不满足本比赛禁止遥控和全向轮的约束。项目应新建自己的 bringup 和 Nav2 参数，只复用底层定位、重定位和点云处理包。

### 4.3 本车导航约束

- TF 主链固定为 `map -> odom -> base_footprint -> base_link -> sensors/arm`，不得继续使用云台坐标系充当底盘基准。
- Nav2 控制器使用差速兼容控制器，首选 `nav2_regulated_pure_pursuit_controller`；速度命令只允许 `linear.x` 和 `angular.z`。
- 全局规划首选 `SmacPlanner2D`，因为差速底盘可原地旋转，不需要为首版引入阿克曼模型。
- footprint 必须在整车和机械臂实际尺寸确定后填写，不能沿用当前 `robot_radius: 0.2`。
- 抓取阶段由速度仲裁器强制输出零速度；机械臂未收回安全姿态时禁止 Nav2 恢复移动。
- 障碍物碰撞扣分高，速度、膨胀半径和最小障碍距离应以保守通过为目标，而不是沿用竞速导航参数。

## 5. 感知结构与资源调度

### 5.1 双侧相机

左右 USB UVC 相机只在 `SIDES` 模式采集，且只启动一个 OpenVINO 推理组件：

- 模型只加载一份，使用两个异步推理请求分别处理左、右相机，逻辑上保持双路同时识别。
- YOLO11n 模型先导出 OpenVINO FP16；只有在真实 NUC 上出现持续算力压力后，再做有代表性标定集的 INT8 量化。
- 首版目标为每侧约 `5-10 FPS`，输入尺寸先使用 `640`；资源不足时先降低相机采集帧率或推理帧率，不先破坏导航频率。
- 行驶阶段侧相机只输出粗候选：类别、成熟度、置信度、图像位置、左右侧、相机模式和时间戳。
- 单帧检测不能直接触发抓取。候选必须经过短时多帧确认，避免晃动、遮挡或曝光变化导致频繁停车。
- `QR`、`TREE` 模式关闭左右相机，避免三路相机同时采集和推理。

### 5.2 D435i 扫码与整树识别

D435i 的固定原生流为 `1920x1080@30 RGB8` 和 `1280x720@30 Z16`，开启同步、
深度对齐与 RGBD。对齐后的两幅消息均为 1920x1080，但深度信息源仍是
1280x720。`QR` 与 `TREE` 之间不重启相机：

1. `QR` 直接把 D435i 彩色原图送给 Python/OpenCV 扫码节点，按 B 后 C 的顺序各连续确认三帧。
2. B/C 计划使用 Reliable、Transient Local、深度 1 发布，C 确认后销毁图像订阅。
3. `TREE` 只执行 D435i RGBD 水果推理，整帧 letterbox 到 640x640，不做分块。
4. `/fruit_perception/d435i/detections` 发布一帧内的全部目标及 1920x1080 ROI；中心深度无效时保留二维结果。
5. 原有类型、最高置信度三维点和下位机接口继续保留。

4DOF 机械臂不能按通用 6DOF 末端位姿设计。应固定末端接近方向或腕部姿态，只求解比赛需要的受限位置和姿态。机械尺寸确定后，再选择解析 IK 或轻量数值 IK；首版不必为了发送四个目标角而引入完整 MoveIt 运行栈。

### 5.3 NUC 资源优先级

运行优先级从高到低为：

1. 串口心跳、急停和底盘速度输出；
2. Point-LIO、small_gicp、TF 和 Nav2 局部避障；
3. D435i 停车扫码或整树识别和测深；
4. 左右侧相机 YOLO；
5. 可视化、图像落盘和调试日志。

`QR/TREE` 模式会停止左右侧采集和 YOLO，把带宽与算力让给 D435i。正式比赛不启动 RViz，不持续保存原始图像和完整点云，只记录状态事件、检测摘要和故障证据。

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
- `task_id=0/1/2` 分别表示夹爪、摄像头云台和 ABC 通用抓取方向；当 `task_id=2` 时，`task_status=0/1` 分别表示左侧和右侧，具体识别/动作方向由下位机结合最近一次 `task_id=5` 所在区域判断。
- `task_id=4` 表示进入二维码识别姿态，当前仅定义 `task_status=0`。
- `task_id=5` 表示机器人所在区域，`task_status=0/1/2/3` 分别表示 A/B/C/D 区。
- `task_id=6` 表示返回初始姿态，当前仅定义 `task_status=0`。
- `task_id=7` 表示机械臂收缩姿态，当前仅定义 `task_status=0`，用于转弯通过狭窄通道前收回机械臂。
- `callback_id=0/1/2/3/4/5/6/7` 分别表示夹爪、摄像头云台、ABC 通用抓取方向、B/D 机械臂目标、二维码识别姿态、所在区域、返回初始姿态和机械臂收缩姿态。
- `callback_status=1` 表示开始执行，`callback_status=0` 表示执行完成。
- 左右抓取方向命令和 B/D 机械臂目标各发布一次，并使用可靠消息保证送达；链路 ACK 只表示收到数据，只有对应的 `ExecutionCallback` 完成状态才能推进任务状态机。
- 舵机、末端定位等离散动作使用可靠消息；底盘速度属于连续刷新数据，不做历史重传。
- 上位机重复发送同一离散状态时，下位机不得重复执行已经完成的危险动作。
- 当前回调未定义失败状态或错误码；上位机不得把通信超时解释为执行完成，故障反馈需要在后续协议中明确扩展。

字段类型、字节偏移和 `z_type=0..15` 的高度索引见构建生成的 `PROTOCOL_DOC.md`。

### 6.2 上层语义接口

业务层不直接构造 `task_id`。建议由 `fruit_hardware_bridge` 暴露以下强类型接口：

- `SetPickDirection`: 左侧或右侧；侧相机确认目标后只调用一次，等待抓取方向回调。下位机用最近一次所在区域区分 A/B/C 的识别和动作方向。
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
- 未检测到成熟目标、目标不成熟或坏水果时，不向下位机发布左右抓取方向命令。
- 仅左侧或右侧存在成熟目标时分别发送 `task_status=0/1`，让下位机按当前区域执行对应方向动作。
- 业务层不重复发送抓取方向命令；`auto_serial_bridge` 负责可靠重传，只有收到 `callback_id=2, callback_status=0` 才恢复路线。
- 路线必须保存当前段索引，抓取后从安全恢复点继续，而不是从 A 区入口重新开始。

### 7.2 C 区

```text
READ_C_QR
  -> ANNOUNCE_ORDER
  -> for task_index in 0..7:
       NAVIGATE_TO_POSITION
       STABLE_STOP
       VERIFY_POSITION_TASK
       SIDE_CAMERA_CHECK
       [no match/unripe/low confidence] -> ANNOUNCE_SKIP -> ADVANCE
       [ripe candidate] -> LOCK_CHASSIS -> PUBLISH_PICK_DIRECTION_ONCE
                        -> WAIT_PICK_DIRECTION_CALLBACK_COMPLETED
                        -> ANNOUNCE_RESULT -> ADVANCE
  -> LEAVE_C
```

- C 区导航目标由 `(position_id, approach_pose, expected_side)` 配置表提供。
- 二维码输出只决定严格任务序列，不允许按路线距离重排。
- 到位后即使没有目标，也完成稳定停车、识别、播报和推进。
- 不成熟、坏水果或种类不符时不发布左右抓取方向命令；成熟且种类匹配时按目标所在侧发送一次方向命令。
- 导航失败只允许进行有限的局部恢复；仍失败则记录本项失败并推进，避免后续全部错序。
- 每项任务开始时记录 task index，所有异步 callback 必须验证仍属于当前任务，防止旧反馈污染下一项。

### 7.3 B 区

```text
USE_SAVED_B_PLAN
  -> SET_MODE_TREE
  -> for position_id in 1..8:
       NAVIGATE_TO_TREE
       STABLE_STOP
       DETECT_ALL_FRUITS_IN_FRAME
       PROCESS_BOTH_FRUITS
       ADVANCE_ONCE
  -> SET_MODE_OFF
  -> RETURN_HOME
```

- B 计划在首次经过 B 入口时已经扫码保存，完成 C 后由晚启动订阅者读取。
- 每棵树处理整帧中按置信度排序的全部两个果实，不能只消费旧接口中的最高置信度目标。
- 无效中心深度的检测保留二维证据，但不得生成三维抓取点。
- B 区机械臂目标继续使用 `ArmTarget(0x13)`，并等待 `callback_id=3` 的执行完成状态。

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
- B 区使用先前持久发布的计划，并按位置 1 到 8 处理每棵树的两个果实。
- 上层只使用语义化硬件接口，不出现散落的 `task_id` 数字常量。
- 下位机通信中断时，即使 NUC 任务节点卡死，底盘和机械臂仍由硬件侧进入安全状态。
