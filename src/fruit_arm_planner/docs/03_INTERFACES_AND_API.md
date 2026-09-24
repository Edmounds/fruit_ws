# 03. 对外暴露接口与 API 说明文档 (INTERFACES_AND_API)

本文档面向任务状态机（`fruit_mission`）、视觉决策层以及上层二次开发人员，详细列举 `arm_planner_node` 对外提供的所有 **ROS 2 话题接口、服务接口、消息字段规范及代码调用示例**。

---

## 目录
- [一、接口架构概览](#一接口架构概览)
- [二、话题控制接口 (Topics)](#二话题控制接口-topics)
- [三、解算方式动态选择机制 (PTP vs LIN 切换规范)](#三解算方式动态选择机制-ptp-vs-lin-切换规范)
- [四、预设动作工序数组规范 (Multi-Waypoint Sequence API)](#四预设动作工序数组规范-multi-waypoint-sequence-api)
- [五、服务控制接口 (Services)](#五服务控制接口-services)
- [六、遥测广播接口 (Telemetry)](#六遥测广播接口-telemetry)
- [七、Python 代码端调用示例 (Client SDK)](#七python-代码端调用示例-client-sdk)

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
                                     话题: ~/joint_states (20Hz 实时实测物理角)
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
| `position` | `float64[]` | **核心必填项**：必须包含 4 个元素，依次为 $[q_1, q_2, q_3, q_4]$ 的物理目标弧度（rad） |
| `velocity` | `float64[]` | 保留未用 |
| `effort` | `float64[]` | **运动耗时指定**：若包含至少 1 个正数，`effort[0]` 即为期望运行时间（毫秒 ms）；若为空则使用默认的 `3000ms` |

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
| `header.frame_id` | `string` | 目标参考坐标系（默认为基座坐标系 `arm_base_link`） |
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
- **通信语义**：异步非阻塞下发。通过名称索引触发 `arm_presets.yaml` 中配置的固定姿态。
- **支持名称**：
  - `"zero"`：物理基准零位（大臂竖直朝天，小臂水平朝前）；
  - `"home"`：安全初始复位位姿（大臂微后仰稳固重心，小臂水平微收）；
  - `"retract"`：底盘导航避障折叠收纳位姿。

- **命令行测试示例**：
```bash
ros2 topic pub --once /arm_planner_node/cmd_preset std_msgs/msg/String "{data: 'home'}"
```

---

## 三、解算方式动态选择机制 (PTP vs LIN 切换规范)

为了满足调用方（`fruit_mission`）在不同任务阶段对“运动速度”与“几何精度”的不同诉求，系统设计了**解算模式动态切换机制**：

### 1. 两种解算模式的适用规则
- **快速关节平滑过渡模式 (`linear_path: false`, PTP)**：
  - **解算逻辑**：仅对目标点 $(X, Y, Z, \text{pitch})$ 求解 1 次逆运动学，4 关节平滑过渡，末端在空中走平滑自然弧线；
  - **建议使用场景**：果树之间的快速转移、空旷空间的复位与收纳。
- **高精度笛卡尔空间严格直线模式 (`linear_path: true`, LIN)**：
  - **解算逻辑**：在 3D 空间以 50ms（20Hz）切分直线路径，沿线逐点逆解并连续校验限位，末端在空中走严格几何直线；
  - **建议使用场景**：夹爪精准水平刺入果实簇、夹紧果实后垂直向上拔起 6cm 脱柄、将果实直线放进果篮口。

### 2. 动态切换在接口中的承载方案
1. **服务接口调用时**：在 `MoveArmCartesian` 服务请求中，显式传入布尔字段 `linear_path = true/false`；
2. **话题接口发布时**：可在 `config/arm_presets.yaml` 中配置 `default_cartesian_mode: "ptp"` 作为默认值；若需临时以直线移动，可通过发布专门的直线话题 `~/cmd_cartesian_linear` 或在未来扩展字段中指定。

---

## 四、预设动作工序数组规范 (Multi-Waypoint Sequence API)

针对“预设不仅是一个单点姿态，而是包含时间排序的动作数组，从当前位置平滑插值过去”的需求，系统在配置层与解析器中定义了**多航路点时间切片工序数组**规范：

### 1. YAML 格式规范
在 `config/arm_presets.yaml` 中，每个预设支持定义为一组关键航路点列表（`waypoints`）：

```yaml
presets:
  # 复合采摘工序预设示例
  pick_sequence:
    description: "苹果采摘多段复合工序"
    waypoints:
      # 阶段 0: 移动到果树前方准备位，耗时 1500ms，夹爪张开 (gripper=1)
      - { joints_deg: [0.0, 10.0, -20.0, 0.0], duration_ms: 1500, gripper: 1 }
      # 阶段 1: 伸向果实并包裹，耗时 1000ms，保持张开 (gripper=1)
      - { joints_deg: [0.0, -10.0, -40.0, 0.0], duration_ms: 1000, gripper: 1 }
      # 阶段 2: 闭合夹爪夹紧水果，原地停留 600ms，夹爪闭合 (gripper=2)
      - { joints_deg: [0.0, -10.0, -40.0, 0.0], duration_ms: 600,  gripper: 2 }
      # 阶段 3: 向上垂直提拉脱柄，耗时 1200ms，保持闭合 (gripper=2)
      - { joints_deg: [0.0, 10.0, -20.0, 0.0], duration_ms: 1200, gripper: 2 }
      # 阶段 4: 折叠收回底盘上方，耗时 2000ms，保持闭合 (gripper=2)
      - { joints_deg: [0.0, 20.0, -100.0, 0.0], duration_ms: 2000, gripper: 2 }
```

### 2. 底层插值与时间片无缝拼接机制
当调用方触发 `ExecuteArmPreset("pick_sequence")` 时：
1. **自动提取起点**：解算节点实时提取下位机当前物理角度 $Q_{\text{current}}$；
2. **引导段插值（当前位置 $\to$ 阶段 0）**：根据阶段 0 的 `duration_ms`，以 50ms 时间步长生成连续五次多项式过渡点；
3. **工序段连续缝合（阶段 0 $\to$ 阶段 1 $\to$ ...）**：各阶段依次以五次多项式在各自指定的 `duration_ms` 内离散化，时间戳累加单调递增；
4. **生成单一连续点流**：将整套工序合成一条完整的 `TrajectoryPoint` 序列一次性下发至下位机，电机连续丝滑运转，无任何段间停顿。

---

## 五、服务控制接口 (Services)

服务接口适用于任务状态机（`fruit_mission`）需要**同步确认机械臂执行到位**的场景：

### 1. 笛卡尔末端带模式选择服务 (`~/move_cartesian`)
- **服务类型**：`fruit_arm_interfaces/srv/MoveArmCartesian`
- **请求字段**：
  - `x, y, z` (`float32`): 空间坐标 (m)
  - `pitch_deg` (`float32`): 俯仰角 (度，0度严格水平)
  - `duration_ms` (`uint32`): 期望耗时 (ms)
  - `gripper_action` (`uint8`): 夹爪动作 (0=保持, 1=张开, 2=闭合)
  - **`linear_path` (`bool`)**: **解算模式选择！** `true`=高精度严格空间直线（逐点逆解），`false`=快速平滑弧线（终点单次逆解）
- **应答字段**：
  - `success` (`bool`): 是否成功执行完毕
  - `message` (`string`): 执行状态描述或报错原因
  - `trajectory_id` (`uint32`): 任务流水号
- **安全看门狗**：节点在后台监控下位机 20Hz 状态，若在 `duration_ms + 2.0s` 缓冲期内收到 `COMPLETED` 则返回成功；若超时未报完成，节点自动发送 `ArmControlMode(1)` (STOP) 停机保护并返回超时失败。

---

### 2. 关节角度直发服务 (`~/move_joints`)
- **服务类型**：`fruit_arm_interfaces/srv/MoveArmJoints`
- **请求字段**：`joints_deg[4]` (度), `duration_ms`, `gripper_action`
- **应答字段**：`success`, `message`, `trajectory_id`

---

### 3. 预设工序调用服务 (`~/execute_preset`)
- **服务类型**：`fruit_arm_interfaces/srv/ExecuteArmPreset`
- **请求字段**：`preset_name` (`string`，如 `"home"`, `"retract"`, `"pick_sequence"`)
- **应答字段**：`success`, `message`, `trajectory_id`

---

### 4. 基础停机与急停服务
- `~/home` (`std_srvs/srv/Trigger`)：复位回安全 Home 姿态
- `~/retract` (`std_srvs/srv/Trigger`)：折叠收纳
- `~/stop` (`std_srvs/srv/Trigger`)：平滑受控停止 (mode=1)
- `~/emergency_stop` (`std_srvs/srv/Trigger`)：硬件最高优先级急停 (mode=2)

---

## 六、遥测广播接口 (Telemetry)

### 机械臂实时状态广播 (`~/joint_states`)
- **完整话题名**：`/arm_planner_node/joint_states`
- **消息类型**：`sensor_msgs/msg/JointState`
- **发布频率**：与下位机状态回传频率同步（**20Hz / 每 50ms 一帧**）
- **功能描述**：实时将下位机回传的物理编码器数值转为标准 ROS 2 `JointState`，供 RViz 机械臂 3D 模型联动或上层监控节点监视。
- **字段规范**：
  - `name`: `['joint_1', 'joint_2', 'joint_3', 'joint_4']`
  - `position`: 当前 4 个关节的实测物理弧度值 $[q_1, q_2, q_3, q_4]$
  - `velocity`: 当前 4 个关节的瞬时角速度 (rad/s)

---

## 七、Python 代码端调用示例 (Client SDK)

在 `fruit_mission` 业务状态机节点中，推荐使用以下范式与机械臂进行高精度或快速交互：

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from fruit_arm_interfaces.srv import MoveArmCartesian, ExecuteArmPreset

class MissionArmCommander(Node):
    def __init__(self):
        super().__init__('mission_arm_commander')

        # 1. 创建笛卡尔末端控制服务客户端
        self._cartesian_client = self.create_client(
            MoveArmCartesian,
            '/arm_planner_node/move_cartesian'
        )

        # 2. 创建预设动作服务客户端
        self._preset_client = self.create_client(
            ExecuteArmPreset,
            '/arm_planner_node/execute_preset'
        )

    def pick_fruit_straight_line(self, x: float, y: float, z: float, pitch_deg: float = 0.0) -> bool:
        """【高精度采摘模式】：末端严格走 3D 直线平直探入果实 (linear_path=True)."""
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arm cartesian service not ready!")
            return False

        req = MoveArmCartesian.Request()
        req.x = x
        req.y = y
        req.z = z
        req.pitch_deg = pitch_deg
        req.duration_ms = 2500
        req.gripper_action = 1          # 探入时张开夹爪
        req.linear_path = True          # 启用严格空间几何直线插补！

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success

    def move_to_hover_fast(self, x: float, y: float, z: float, pitch_deg: float = 0.0) -> bool:
        """【快速转移模式】：关节平滑过渡，空中走平滑弧线 (linear_path=False)."""
        req = MoveArmCartesian.Request()
        req.x = x
        req.y = y
        req.z = z
        req.pitch_deg = pitch_deg
        req.duration_ms = 2000
        req.gripper_action = 0          # 保持夹爪
        req.linear_path = False         # 仅终点逆解，关节平滑过渡，速度更快

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success

    def run_pick_sequence(self, sequence_name: str = "pick_sequence") -> bool:
        """执行完整的多动作数组预设工序."""
        req = ExecuteArmPreset.Request()
        req.preset_name = sequence_name
        future = self._preset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result().success
```
