# 机械臂解算包重构技术规格书与逐项改动实施计划 (REFACTOR_SPEC_AND_CHECKLIST)

> ⚠️ **执行原则**：本文档为技术方案规划与改动实施指导书。**代码尚未改动**，后续代码开发将严格依照本文档中的检查清单（Checklist）逐项完成。

---

## 目录
- [一、重构背景与设计目标](#一重构背景与设计目标)
- [二、五大核心技术规格规范 (Architecture Specs)](#二五大核心技术规格规范-architecture-specs)
  - [规格 1：Service 为唯一官方业务控制接口（同步阻塞与超时监护）](#规格-1service-为唯一官方业务控制接口同步阻塞与超时监护)
  - [规格 2：动态解算模式选择参数 (PTP 快速弧线 vs LIN 严格直线)](#规格-2动态解算模式选择参数-ptp-快速弧线-vs-lin-严格直线)
  - [规格 3：以笛卡尔空间为主的多航路点预设工序规范](#规格-3以笛卡尔空间为主的多航路点预设工序规范)
  - [规格 4：工业级双点驻留法（Dwell Point）到位后夹取时序机制](#规格-4工业级双点驻留法dwell-point到位后夹取时序机制)
  - [规格 5：首航路点 duration_ms 驱动的当前位置平滑引导过渡](#规格-5首航路点-duration_ms-驱动的当前位置平滑引导过渡)
- [三、逐项改动实施计划表 (Step-by-Step Checklist)](#三逐项改动实施计划表-step-by-step-checklist)

---

## 一、重构背景与设计目标

在前期初步架构设计中，控制接口优先采用了 Topic 异步流式发布，预设动作采用了单一目标姿态。经过深入推敲与实车业务契约对齐，暴露出以下必须重构的关键问题：

1. **业务闭环跟踪缺失**：机械臂运动耗时通常为 1.5 ~ 3.5 秒。如果 `fruit_mission` 仅发 Topic，无法原生获知“机械臂何时停稳到位、是否夹紧成功、能否继续导航”，迫使任务层自行轮询监听，极易造成代码臃肿与死锁。**必须将 Service 提升为唯一官方业务控制接口**。
2. **缺乏动态精度切换机制**：大范围移臂需要快速平滑关节弧线（PTP），而靠近果树刺入采摘需要严格沿 3D 空间直线前进（LIN）。**必须支持在调用时由上层动态传参选择解算模式**。
3. **关节角配置不直观**：在 YAML 中手动测量和定义各电机的弯曲角度极为抽象；而末端笛卡尔空间坐标（前方 $X\text{m}$，高度 $Z\text{m}$，俯仰角 $\text{pitch}^\circ$）与视觉测量值直接对应，极其直观。**预设工序应优先使用笛卡尔空间坐标定义**。
4. **到点夹取时序模糊**：下位机协议是时间连续点流，若在移动过程中同时触发夹爪动作，极易在飞驰中磕碰撞飞果实。**必须在算法层严格实现“停稳到位后再夹紧”的时序硬约束**。

---

## 二、五大核心技术规格规范 (Architecture Specs)

### 规格 1：Service 为唯一官方业务控制接口（同步阻塞与超时监护）
- **核心定位**：`fruit_mission` 任务状态机**只与 Service 交互**；Topic 降级为只读遥测（`~/joint_states`）与命令行轻量辅助调试；
- **同步返回语义**：
  - 任务层发起 `MoveArmCartesian` 或 `ExecuteArmPreset` 请求后，客户端挂起等待；
  - 解算节点将轨迹整体打包下发并自动打出 `ArmControlMode(0)` 点火；
  - 节点在后台启动超时监护器（超时时限 = `轨迹总规划耗时 + 2.0s 缓冲`）；
  - 监听到下位机 20Hz 状态上报 `task_status == 2 (COMPLETED)` 时，服务立即返回 `success=True`，任务层无缝推进下一步！
  - 若超时未完成，解算节点主动打出 `ArmControlMode(1)` (STOP) 停机保护，服务返回 `success=False, message="Timeout waiting for execution"`。

---

### 规格 2：动态解算模式选择参数 (PTP 快速弧线 vs LIN 严格直线)
在笛卡尔服务中通过布尔参数 `linear_path` 实现解算模式切换：

| 模式 | 参数取值 | 规划所在空间 | 逆解调用逻辑 | 末端空间几何轨迹 | 适用场景 |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **快速平滑过渡 (PTP)** | `linear_path: false` | 关节空间 | 仅终点求解 1 次 IK | 平滑三维空间弧线 | 大范围移臂、收纳折叠、快速转移 |
| **严格笛卡尔直线 (LIN)** | `linear_path: true` | 笛卡尔操作空间 | 沿空间直线按 50ms 逐点求解 $N$ 次 IK | **严格三维几何直线** | 水平笔直刺入果实分支、垂直提拉脱柄 |

---

### 规格 3：以笛卡尔空间为主的多航路点预设工序规范
在 `config/arm_presets.yaml` 中，预设工序格式升级为**以笛卡尔空间坐标为主、兼顾关键关节角**的双格式体系：

```yaml
presets:
  # 格式 A：以笛卡尔空间为主的复合采摘工序 (推荐，直观易调)
  pick_apple_sequence:
    description: "苹果采摘多动作时间切片序列"
    coordinate_frame: "cartesian"  # 声明采用末端笛卡尔空间坐标
    waypoints:
      # 段 0: 移动到果实前方 5cm 处准备位 (耗时 1500ms, 夹爪张开)
      - { target: [0.20, 0.0, 0.25, 0.0], duration_ms: 1500, gripper: 1 } # [x, y, z, pitch_deg]
      # 段 1: 严格沿直线向前探入 8cm 包裹果实 (耗时 800ms, 夹爪保持张开)
      - { target: [0.28, 0.0, 0.25, 0.0], duration_ms: 800,  gripper: 1 }
      # 段 2: 闭合夹爪夹紧水果 (原地停顿 600ms, 夹爪闭合)
      - { target: [0.28, 0.0, 0.25, 0.0], duration_ms: 600,  gripper: 2 }
      # 段 3: 向上垂直直线拔起 6cm 脱柄 (耗时 1000ms, 夹爪保持闭合)
      - { target: [0.28, 0.0, 0.31, 0.0], duration_ms: 1000, gripper: 2 }

  # 格式 B：纯关节角度定义的特殊避障姿态 (用于紧贴车身的折叠收纳)
  retract:
    description: "底盘避障折叠姿态"
    coordinate_frame: "joints"     # 声明采用 4 电机物理角度
    waypoints:
      - { target: [0.0, 20.0, -100.0, 0.0], duration_ms: 2500, gripper: 0 }
```

---

### 规格 4：工业级双点驻留法（Dwell Point）到位后夹取时序机制
为了在时间连续路点流中 100% 确保机械臂**“停稳到位后再夹取，绝不在飞行过程中夹取”**，算法采用工业机器人经典的**双点驻留法 (Dwell Point)**：

```text
       [移动阶段: 耗时 T_move]               [夹取驻留阶段: 耗时 T_dwell]
起点 P0 ─────────────────────────────► 目标点 P1 ─────────────────────────────► 目标点 P1 (保持坐标不变)
夹爪动作: 保持 (HOLD)                    到达目标点    夹爪动作: 闭合 (CLOSE) / 张开 (OPEN)
                                                     机械臂原地静止，电机抱死，等待夹爪到位
```

1. **移动阶段**：从当前位置插值运动到目标点 $P_1$，耗时为 `duration_ms`。在移动过程中，夹爪动作强制锁定为 `HOLD (0)`，保持原有开合状态，严禁变动；
2. **夹取驻留阶段**：在目标点 $P_1$ 原地停留指定的驻留时间（例如 500ms），坐标不发生位移，此时下发目标夹爪状态（`OPEN` 或 `CLOSE`）；
3. 这样保证机械臂在空间中速度已完全降为 0、机械完全静止后，夹爪才通电开合，杜绝碰落果实。

---

### 规格 5：首航路点 duration_ms 驱动的当前位置平滑引导过渡
对于工序预设：
1. 算法启动时，从下位机 20Hz 实时状态提取当前物理角度 $\mathbf{Q}_{\text{current}}$；
2. 取预设中第 0 个航路点，读取其配置的 `duration_ms`（例如 1500ms）：
   - 如果目标是笛卡尔坐标，先通过高位逆解转为硬件角度 $\mathbf{Q}_{\text{target\_0}}$；
   - 算法自动在 $0 \sim \text{duration\_ms}$ 时间窗口内，使用五次多项式在 $\mathbf{Q}_{\text{current}}$ 与 $\mathbf{Q}_{\text{target\_0}}$ 之间插补生成 50ms 连续平滑过渡点集；
3. 后续各航路点紧随其后时间单调累加，形成完整的无缝整体轨迹。

---

## 三、逐项改动实施计划表 (Step-by-Step Checklist)

后续改动实施时，必须严格按照以下 5 个阶段逐项勾选推进：

### 阶段一：接口包定义升级 (`fruit_arm_interfaces`)
- [x] **1.1** 在 `src/fruit_arm_interfaces/srv/` 目录下新增 `MoveArmCartesian.srv`：
  - 输入：`x, y, z, pitch_deg, duration_ms, gripper_action, bool linear_path`
  - 输出：`bool success, string message, uint32 trajectory_id`
- [x] **1.2** 在 `src/fruit_arm_interfaces/srv/` 目录下新增 `MoveArmJoints.srv`：
  - 输入：`float32[4] joints_deg, uint32 duration_ms, uint8 gripper_action`
  - 输出：`bool success, string message, uint32 trajectory_id`
- [x] **1.3** 在 `src/fruit_arm_interfaces/srv/` 目录下新增 `ExecuteArmPreset.srv`：
  - 输入：`string preset_name`
  - 输出：`bool success, string message, uint32 trajectory_id`
- [x] **1.4** 编写 `src/fruit_arm_interfaces/CMakeLists.txt`，将上述 3 个 `.srv` 加入 `rosidl_generate_interfaces`；
- [x] **1.5** 执行 `colcon build --packages-select fruit_arm_interfaces` 编译生成 C++ / Python 服务头文件与类型绑定。

---

### 阶段二：算法内核升级 (`fruit_arm_planner`)
- [x] **2.1** 在 `trajectory_planner.py` 中升级 `plan_cartesian_linear_path`：
  - 输入起点与终点 $(X, Y, Z, \text{pitch})$ 与时间，沿空间直线按 50ms 离散化；
  - 对每个时间片点调用 `kinematics.inverse_kinematics()`，由 transformer 校验限位，输出保证空间绝对直线的点流。
- [x] **2.2** 在 `trajectory_planner.py` 中新增 `plan_multi_waypoint_sequence`：
  - 支持多航路点输入，支持 `coordinate_frame` 为 `cartesian` 与 `joints`；
  - 实现双点驻留法（Dwell Point）：到位点与夹取动作用时间切片分离；
  - 实现首航路点平滑引导段生成。

---

### 阶段三：配置文件规范升级 (`config/arm_presets.yaml`)
- [x] **3.1** 重构 `config/arm_presets.yaml` 中的 `presets`：
  - 将 `zero`、`home`、`retract` 改写为符合新标准的格式；
  - 增加标准的笛卡尔采摘工序预设 `pick_sequence` 示例，包含完整的悬停、探入、夹紧、提拉与收回阶段。

---

### 阶段四：ROS 2 服务端点与安全看门狗重构 (`arm_planner_node.py`)
- [x] **4.1** 在 `arm_planner_node.py` 中注册 3 个新服务服务端：
  - `/arm_planner_node/move_cartesian`
  - `/arm_planner_node/move_joints`
  - `/arm_planner_node/execute_preset`
- [x] **4.2** 在 `_srv_move_cartesian_cb` 中根据请求中的 `linear_path` 动态分支：
  - 若 `linear_path == True`：调用笛卡尔逐点逆解直线规划；
  - 若 `linear_path == False`：调用终点单次逆解关节平滑插补；
- [x] **4.3** 实现执行闭环监视看门狗函数 `execute_trajectory_sync(points, duration_ms)`：
  - 后台自动下发 `Header -> Points -> Mode(0)`；
  - 使用 `MultiThreadedExecutor` 与 `ReentrantCallbackGroup` 周期监听 `ArmStatus` 直至收到 `COMPLETED`；
  - 若超时未收到，主动下发 `ArmControlMode(1)` (STOP) 停机，保障硬件安全。

---

### 阶段五：文档与工程最终归档 (`docs/`)
- [x] **5.1** 更新 `03_INTERFACES_AND_API.md`，将 Service 置为一等公民，采用 `fruit_arm_interfaces`；
- [x] **5.2** 更新 `04_SETUP_AND_DEBUGGING.md`，提供针对新服务的完整 CLI 调试指令；
- [x] **5.3** 更新 `README.md`，更新接口调用总览；
- [x] **5.4** 执行全包编译与端到端运行测试验证。
