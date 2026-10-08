# 03. 对外暴露接口与 API 说明文档 (INTERFACES_AND_API)

本文档面向任务状态机（`fruit_mission`）、视觉决策层以及上层二次开发人员，详细列举 `arm_planner_node` 对外提供的所有 **ROS 2 话题接口、服务接口、消息字段规范及代码调用示例**。

本篇描述当前接口。三类正式调用共用运动占用权；必须有新鲜、有效的四轴回传，轨迹首点为 `t=0` 实测角。跨层通信时序及其限制见[通信架构文档](02_COMMUNICATION_PROTOCOL.md)。

三类正式调用共用 `2..64` 点的单条轨迹限制，与当前 FT 下位机缓存容量一致。插值点和驻留点均计入总数。超出容量时，在发布 Header 前返回 `success=false`、`trajectory_id=0` 和包含实际点数的错误信息。机械臂命令发布队列及桥订阅队列深度均为 64。

---

## 目录
- [一、接口架构概览](#一接口架构概览)
- [二、话题控制接口 (Topics)](#二话题控制接口-topics)
- [三、解算方式动态选择机制 (PTP vs LIN 切换规范)](#三解算方式动态选择机制-ptp-vs-lin-切换规范)
- [四、预设动作工序数组规范 (Multi-Waypoint Sequence API)](#四预设动作工序数组规范-multi-waypoint-sequence-api)
- [五、服务控制接口 (Services)](#五服务控制接口-services)
- [六、遥测广播接口 (Telemetry)](#六遥测广播接口-telemetry)
- [七、mission 客户端调用约定](#七mission-客户端调用约定)

---

## 一、接口架构概览

`fruit_arm_planner` 作为高内聚、低耦合的运动解算包，向上屏蔽了所有底层的串口分包、重传和 `trajectory_id` 管理，仅暴露标准语义接口：

```text
                               【外部调用方 (fruit_mission / CLI)】
                                                │
         ┌───────────────────┬──────────────────┴──────────────────┬──────────────────┐
         ▼                   ▼                                     ▼                  ▼
话题: ~/cmd_joints   话题: ~/cmd_cartesian                 话题: ~/cmd_preset   服务: ~/home
(输入 4 个目标关节角) (输入末端空间坐标 X,Y,Z)              (输入姿态名称 "home") (同步触发 Home 复位)
         │                   │                                     │                  │
         └───────────────────┴──────────────────┬──────────────────┴──────────────────┘
                                                ▼
                                    ┌───────────────────────┐
                                    │    arm_planner_node   │
                                    └───────────────────────┘
                                                │
                                                ▼ 广播
                                     话题: ~/joint_states (20Hz 原始角、模型角、TCP)
```

> **节点命名空间约定**：
> 下列所有以 `~/` 开头的话题和服务，在实际运行时展开为前缀 `/arm_planner_node/`。例如 `~/cmd_joints` 即 `/arm_planner_node/cmd_joints`。

---

## 二、话题控制接口 (Topics)

### 1. 目标关节角控制 (`~/cmd_joints`)
- **完整话题名**：`/arm_planner_node/cmd_joints`
- **消息类型**：`sensor_msgs/msg/JointState`
- **通信语义**：异步非阻塞下发。机械臂自动以当前实时物理角度为起点，五次多项式平滑插补至目标角度。
- **字段规范**：

| 字段名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `header.stamp` | `builtin_interfaces/Time` | 时间戳（可选） |
| `name` | `string[]` | 关节名称列表（可选，如 `['q1', 'q2', 'q3', 'q4']`） |
| `position` | `float64[]` | **核心必填项**：客户端提供 4 个原始硬件目标弧度（rad）；Q3 映射小臂绝对俯仰，仍含硬件方向/偏置，不是未经映射的连杆角 |
| `velocity` | `float64[]` | 保留未用 |
| `effort` | `float64[]` | **运动耗时指定**：首项为有限正数时作为运动时间（毫秒 ms）；否则使用源码 YAML 的 `planning.default_duration_ms` |

- **命令行测试示例**：
```bash
# 让机械臂在 2000ms 内平滑运动至 [0°, 0°, -30°, 0°] (弧度: [0, 0, -0.5236, 0])
ros2 topic pub --once /arm_planner_node/cmd_joints sensor_msgs/msg/JointState \
  "{position: [0.0, 0.0, -0.5236, 0.0], effort: [2000.0]}"
```

---

### 2. 末端空间笛卡尔坐标控制 (`~/cmd_cartesian`)
- **完整话题名**：`/arm_planner_node/cmd_cartesian`
- **消息类型**：`geometry_msgs/msg/PointStamped`
- **通信语义**：异步非阻塞下发。节点自动调用高位构型逆运动学 (IK) 求解理论关节角，通过 `JointTransformer` 换算为物理硬件角度并完成限位校验，随后平滑执行。
- **字段规范**：

| 字段名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `header.frame_id` | `string` | 当前回调不读取、不做 TF 转换；XYZ 必须已在固定机械臂根部坐标系中，填写光学 frame 不会转换坐标 |
| `point.x` | `float64` | 目标点在前进方向的前向距离（单位: 米 m） |
| `point.y` | `float64` | 目标点在车身左右方向的横向距离（单位: 米 m，左正右负） |
| `point.z` | `float64` | 目标点在垂直地面的高度（单位: 米 m，以机械臂底座安装面为0） |

- **命令行测试示例**：
```bash
# 让夹爪末端移动到机械臂前方 20cm，高度 25cm 位置
ros2 topic pub --once /arm_planner_node/cmd_cartesian geometry_msgs/msg/PointStamped \
  "{point: {x: 0.20, y: 0.0, z: 0.25}}"
```

---

### 3. 预设固定姿态调用 (`~/cmd_preset`)
- **完整话题名**：`/arm_planner_node/cmd_preset`
- **消息类型**：`std_msgs/msg/String`
- **通信语义**：异步非阻塞下发。通过名称索引触发 `arm_presets.yaml` 中配置的固定姿态；当前关节预设使用 `link_angles`（大臂相对竖直、小臂相对水平、末端绝对俯仰），其中末端设为 `0°` 时会按小臂角度和偏置换算腕电机目标。若预设改为 `coordinate_frame: joints`，四项则是原始硬件角。
- **支持名称**：以源码 `config/arm_presets.yaml` 的 `presets` 为准。比赛客户端引用 `<side>_look_<zone>`、`<side>_store_<zone>`（left/right，a/c；B 区细分为 `<side>_store_b_up` 与 `<side>_store_b_down`）、`qr_scan`、`shenzhi`、`home`。操作者须检查并补齐所用预设；已有坐标保持不变，不存在的名称返回失败，不提供旧名称别名或假姿态。

- **命令行测试示例**：
```bash
ros2 topic pub --once /arm_planner_node/cmd_preset std_msgs/msg/String "{data: 'zero'}"
```

---

## 三、解算方式动态选择机制 (PTP vs LIN 切换规范)

为了满足调用方（`fruit_mission`）在不同任务阶段对“运动速度”与“几何精度”的不同诉求，系统设计了**解算模式动态切换机制**：

### 1. 两种解算模式的适用规则
- **快速关节平滑过渡模式 (`linear_path: false`, PTP)**：
  - **解算逻辑**：仅对目标点 $(X, Y, Z, \text{pitch})$ 求解 1 次逆运动学，4 关节平滑过渡，末端在空中走平滑自然弧线；
  - **建议使用场景**：果树之间的快速转移、空旷空间的复位与收纳。
- **高精度笛卡尔空间严格直线模式 (`linear_path: true`, LIN)**：
  - **解算逻辑**：按 YAML 的 `planning.interpolation_step_ms` 离散直线路径，逐点逆解并校验限位；规划点在直线上，不是硬件连续路径误差或无碰撞保证；
  - **建议使用场景**：夹爪精准水平刺入果实簇、夹紧果实后垂直向上拔起 6cm 脱柄、将果实直线放进果篮口。

### 2. 动态切换在接口中的承载方案
1. **服务接口调用时**：在 `MoveArmCartesian` 服务请求中，显式传入布尔字段 `linear_path = true/false`；
2. **现有调试话题**：`~/cmd_cartesian` 固定使用 PTP，不提供 LIN 选择。正式业务要选择 LIN，应调用 `MoveArmCartesian` 服务并设置 `linear_path=true`。

---

## 四、预设动作工序数组规范 (Multi-Waypoint Sequence API)

针对“预设不仅是一个单点姿态，而是包含时间排序的动作数组，从当前位置平滑插值过去”的需求，系统在配置层与解析器中定义了**多航路点时间切片工序数组**规范：

### 1. YAML 格式规范
在 `config/arm_presets.yaml` 中，每个预设支持定义为一组关键航路点列表（`waypoints`）：

```yaml
presets:
  # 与当前解析器一致的关节预设示例；时间均为逐段时长
  pick_sequence:
    coordinate_frame: "joints"
    waypoints:
      - { target: [0.0, 20.0, -40.0, 0.0], duration_ms: 1500, gripper: 1, dwell_ms: 400 }
      - { target: [0.0, 10.0, -60.0, 0.0], duration_ms: 1000, gripper: 2, dwell_ms: 600 }
      - { target: [0.0, 20.0, -100.0, 0.0], duration_ms: 2000, gripper: 0, is_via_point: true }
```

### 2. 底层插值与时间片无缝拼接机制
航点内的 `linear: true` 选择 LIN，`false` 或省略选择 PTP。
该字段控制上一位置到该航点的运动段；第一个航点从当前实测姿态开始。
LIN 只支持 `coordinate_frame: cartesian`；不要把 `linear` 放在预设层与 `coordinate_frame` 并列。
waypoint 可选 `is_via_point: true`，表示该 waypoint 目标是途经点；省略或写 `false` 表示精确到达点。
PTP/LIN 自动生成的中间插值点默认标记为途经点，轨迹第 0 个实测起点始终标记为精确点。
service 请求不开放该字段，`move_cartesian` 和 `move_joints` 的最终目标默认是精确点。
带 OPEN/CLOSE 或 `dwell_ms` 的 waypoint 也可以配置为途经点；驻留点继承该 waypoint 的标记。
源码 YAML 的 `pick_sequence` 四个航点均已设置 `linear: true`，坐标、时长与夹爪动作保持不变。
修改后，在机械臂不运动时调用 `reload_presets` 或使用控制台 `reload`，并检查重载结果。

当调用方触发 `ExecuteArmPreset("pick_sequence")` 时：
1. **自动提取起点**：解算节点实时提取下位机当前物理角度 $Q_{\text{current}}$；
2. **引导段插值（当前位置 $\to$ 阶段 0）**：根据阶段 0 的 `duration_ms` 和配置的 `interpolation_step_ms` 生成五次多项式过渡点；
3. **工序段连续缝合（阶段 0 $\to$ 阶段 1 $\to$ ...）**：各阶段依次以五次多项式在各自指定的 `duration_ms` 内离散化，时间戳累加单调递增；
4. **生成单一连续点流**：将整套工序合成一条 `TrajectoryPoint` 序列下发；配置了 `dwell_ms` 的段会在目标位置驻留。

---

## 五、服务控制接口 (Services)

服务接口适用于任务状态机（`fruit_mission`）需要**同步确认机械臂执行到位**的场景：

### 1. 笛卡尔末端带模式选择服务 (`~/move_cartesian`)
- **服务类型**：`fruit_interfaces/srv/MoveArmCartesian`
- **请求字段**：
  - `x, y, z` (`float32`): 真实夹爪抓取点 TCP 的基座空间坐标 (m)
  - `pitch_deg` (`float32`): 末端 a4 伸出段俯仰角（度，0度表示该段水平，不是腕轴到 TCP 的斜线水平）
  - `duration_ms` (`uint32`): 期望耗时 (ms)
  - `gripper_action` (`uint8`): 夹爪动作 (0=保持, 1=张开, 2=闭合)
  - **`linear_path` (`bool`)**: **解算模式选择！** `true`=高精度严格空间直线（逐点逆解），`false`=快速平滑弧线（终点单次逆解）
- **应答字段**：
  - `success` (`bool`): 是否成功执行完毕
  - `message` (`string`): 执行状态描述或报错原因
  - `trajectory_id` (`uint32`): 任务流水号
- **执行结果**：节点从发布全部轨迹点后，等待下位机上报本次轨迹 ID 的新状态。下位机收到完整点位后自动启动。收到 `COMPLETED` 返回成功；下位机报错（`FAILED`、`EMERGENCY`）或外部主动调用 STOP 返回失败。发布 STOP 成功不代表硬件已停稳。
- **统一抓取点模型**：`arm_presets.yaml` 的 `geometry.a4` 定义腕轴沿末端局部 X 的伸出长度，`geometry.tool_z_offset_m` 定义真实抓取点相对伸出段末端的局部 Z 偏移（米，下方为负，当前 `-0.03`）。偏移随腕部旋转，FK/IK、PTP/LIN、`cmd_cartesian` 和 Cartesian preset 共用，不再额外加减输入坐标。`pitch_deg=0°` 时 a4 段水平，真实抓取点比腕轴低 3 cm，并到达请求 XYZ。关节角命令/预设保持原角度语义，不额外转 IK；其 FK 输出同样为 TCP。旧 `planning.cartesian_offset_m` 已移除。现有预设航点数值不自动迁移，由操作者按新 TCP 定义核对；几何修改后须重启 planner、控制台和 mission，不能仅 reload presets。

---

### 2. 关节角度直发服务 (`~/move_joints`)
- **服务类型**：`fruit_interfaces/srv/MoveArmJoints`
- **请求字段**：`joints_deg[4]` 为四个原始硬件角（度），不应用连杆零位偏置；另有 `duration_ms`、`gripper_action`。控制台 `joints` 是连杆角，`motors` 才与本服务语义一致。
- **应答字段**：`success`, `message`, `trajectory_id`

正式 mission 和相机单点调试客户端在区域 Look 后、开启识别前，以新鲜且静止的实测四轴硬件角调用本服务，
先 `gripper_action=1`（OPEN），再以更新的实测角调用 `gripper_action=0`（HOLD）驻留；
两次请求均无限等待下位机执行结果，失败不继续目标运动；端点与实测反馈取得仍分别
限幅到 10 秒，较短配置不变。此流程不修改 Look 预设，不引入直接串口或原始轨迹控制。

---

### 3. 预设工序调用服务 (`~/execute_preset`)
- **服务类型**：`fruit_interfaces/srv/ExecuteArmPreset`
- **请求字段**：`preset_name` (`string`，如 `"home"`, `"retract"`, `"pick_sequence"`)
- **应答字段**：`success`, `message`, `trajectory_id`

---

### 4. 基础停机与急停服务
- `~/home` (`std_srvs/srv/Trigger`)：执行同名 `home` 预设快捷服务
- `~/stop` (`std_srvs/srv/Trigger`)：平滑受控停止 (mode=1)
- `~/emergency_stop` (`std_srvs/srv/Trigger`)：硬件最高优先级急停 (mode=2)

---

### 5. 预设热重载 (`~/reload_presets`)
- **完整服务名 / 类型**：`/arm_planner_node/reload_presets`，`std_srvs/srv/Trigger`，请求为空。
- 从节点的 `config_path` 重新读取全部 `presets`，校验成功后一次性替换内存中的预设；新增、修改、删除及各航点属性均生效。
- 运动期间或配置无效时返回 `success=false` 和原因，保留原有内存预设。空的预设映射可用于清空预设。
- **只重载 presets**，不更新 geometry、joint_mapping 或 planning；修改这些参数仍须重启节点。
- 控制台 `reload` 命令及打点保存后调用此服务；保存到文件成功不等于热重载成功，需查看服务结果。

---

## 六、遥测广播接口 (Telemetry)

### 机械臂实时状态广播 (`~/joint_states`)
- **完整话题名**：`/arm_planner_node/joint_states`
- **消息类型**：`fruit_interfaces/msg/ArmTelemetry`
- **发布频率**：仅在收到有效四轴反馈时发布，正常情况下跟随下位机约 20 Hz 状态回传。
- **功能描述**：同一帧有效电机反馈同时提供原始硬件角、逆 Joint Mapping 后的模型角，以及共享 FK 算出的真实夹爪抓取点 TCP。不新增话题、不修改标准 ROS 消息定义。

| 字段 | 含义 / 单位 |
| --- | --- |
| `header` | planner 收到反馈后的 ROS 时间；`frame_id=arm_base_link` 是 geometry 定义的固定机械臂根部坐标系，不是相机或底盘坐标系；此消息不发布 TF。 |
| `hardware_joint_state` | `sensor_msgs/JointState`：原始电机角 `h1..h4`，`position` 为 rad，`velocity` 为 rad/s，不消除标定偏置。 |
| `model_joint_state` | `sensor_msgs/JointState`：逆 mapping 后的 `q1..q4`，`position` 为 rad，`velocity` 为 rad/s；Q1=偏航，Q2=大臂相对水平，Q3=小臂相对大臂，Q4=腕部相对小臂。**不再对这组数据执行逆 mapping。** |
| `tcp_position` | `geometry_msgs/Point`：真实夹爪抓取点 XYZ，单位 m，已经包含 `geometry.tool_z_offset_m`。 |
| `tcp_pitch_deg` | a4 伸出段的绝对俯仰角，单位度；0° 表示 12 cm 段水平，TCP 位于其下方 3 cm，不是腕轴到 TCP 的斜线水平。 |
| `trajectory_id/task_status/progress/gripper_state/error_code` | 原帧硬件状态元数据，原样保留；数值语义以 `fruit_interfaces/msg/ArmStatus` 为准。 |

两个 JointState 的 `name` 均为 `['joint_1','joint_2','joint_3','joint_4']`，四轴 position/velocity 各四项，effort 为空；header 与外层一致。角速度按方向及 Q3 耦合求导，不减常量 offset；非有限速度保留，不伪造为零。无效/越限位置不发布遥测，消费者必须检查反馈新鲜度，不能把最后一帧当成当前状态。

**兼容性变更**：
1. **遥测消息格式**：该话题不再是标准 JointState，旧订阅者必须改为 ArmTelemetry；RViz/robot_state_publisher 不能直接把此话题当成标准 JointState 输入。本次不新增适配话题。`~/cmd_joints` 仍使用标准 JointState，运动服务及硬件状态协议不变。
2. **包名合并迁移**：原 `fruit_arm_interfaces` 与 `fruit_hardware_interfaces` 的全部 8 个消息和服务已统一合并至 `fruit_interfaces`。服务路径（`/arm_planner_node/*`）、话题路径及结果字段语义严格保持不变，线协议与下位机固件完全兼容；但旧包客户端代码与旧类型 rosbag 不会自动适配，必须与消费者一同重构建并重启。

确认硬件安全停稳，停止旧 planner、控制台及 mission；构建接口和消费者，然后重新加载运行环境并重启这些进程。不能让同名话题的新旧消息类型同时运行，`reload_presets` 也不能代替重启。可只读核对：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_interfaces fruit_arm_planner fruit_mission fruit_robviz_bridge auto_serial_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
# 重启 planner 和客户端后执行以下只读命令，不会命令机械臂运动：
ros2 interface show fruit_interfaces/msg/ArmTelemetry
ros2 topic echo --once /arm_planner_node/joint_states fruit_interfaces/msg/ArmTelemetry
```

---

## 七、mission 客户端调用约定

1. 分别创建 `MoveArmJoints`、`MoveArmCartesian` 和 `ExecuteArmPreset` service 客户端。mission 只提交这三类请求，不直接发布 `TrajectoryHeader`、`TrajectoryPoint` 或 `ArmControlMode`。
2. 调用笛卡尔服务前，由负责视觉和坐标转换的模块把目标转换到机械臂根部固定坐标系。`pitch_deg` 是末端俯仰角；`linear_path` 只选择 PTP 或 LIN，不改变坐标系。
3. 客户端异步发出 service 请求，由 mission 现有 executor 等待其 future；机械臂执行没有前端期限或自动超时 STOP。下位机完成/报错、主动 STOP/取消或节点退出决定结果。端点发现仍独立限时。不要在正在由同一 executor 调用的回调中再次 `spin_until_future_complete`，避免回调互相等待。
4. future 完成后先处理 service 异常，再判断 `response.success`。仅 `success=true` 表示对应轨迹已由下位机报告完成；`trajectory_id` 用于诊断关联，`message` 记录失败原因。请求异常时结果未知，应取消并请求 STOP，不能推断机械臂已停止。
5. 若任务中途需中断，调用独立的 STOP 或急停入口。它们当前的 `Trigger.success=true` 只表示规划器已发布控制 Mode；下位机是否停稳应观察后续状态。详细时序和残余风险见[通信架构文档](02_COMMUNICATION_PROTOCOL.md)。
6. 正式 mission 的观察、二维码、放果与恢复均调用 `execute_preset`，名称直接遵循
   `arm_presets.yaml` 姿态定义：`<side>_look_<zone>`、`<side>_store_<zone>`（side 为 left/right，A/C 为小写 a/c，B 区细分为 `<side>_store_b_up` 与 `<side>_store_b_down`）、`qr_scan`、`shenzhi`、`home`，不使用中间映射字典。
   预设缺失/失败不能视为动作完成。导航可与 Look 并行，其余运动仍受有效反馈、
   导航终态和 planner 占用保护。恢复只尝试一次 Home，不设独立停稳循环，
   未取得有效低速反馈则跳过；不因 STOP 已发布而放行。旧正常 retract 边界改为 shenzhi，重合出口不重复执行；失败仍只恢复 Home，不能将 Home 冒充所需保护姿态后放行转场。区域通知只供感知模型选择，不走下位机旧 task/callback 协议；扫码区域由 QR 窗口指定，无 B→C 状态推进。
