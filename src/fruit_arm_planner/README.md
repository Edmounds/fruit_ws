# fruit_arm_planner

水果采摘机器人 4 自由度采摘机械臂运动学解算、关节坐标映射与轨迹规划功能包。

mission 可调用关节单目标、机械臂根部坐标系下的笛卡尔单目标或预设名称；本包从新鲜四关节实测状态生成以 `t=0` 为起点的轨迹，并等待下位机报告匹配的执行终态。相机坐标转换和比赛任务流程由其他模块负责。详细边界与现状见[通信架构文档](docs/02_COMMUNICATION_PROTOCOL.md)，实施与待核实事项见[重构清单](docs/REFACTOR_SPEC_AND_CHECKLIST.md)。

## 当前集成状态

规划器和服务接口可编译并可启动，但尚未接入 `fruit_mission` 的采摘流程。任务层目前仍发布旧的 `ArmTarget` / `StateMachineCommand` 并等待 `ExecutionCallback`；这些消息已从 `fruit_hardware_interfaces` 的生成列表移除。串口桥的可靠发送按消息 ID 分别排队，现有 Header、Point、Mode 连续发布尚不能保证下位机按完整轨迹顺序接收。因此，在完成任务层迁移和轨迹传输时序验证前，本包不能用于实机采摘闭环。

---

## 专题文档导航 (Documentation Index)

本包文档存放于 [`docs/`](docs/) 目录下。通信文档区分已实现行为与跨包待核实事实。

| 分册编号 | 专题文档名称 | 核心内容概述 |
| :---: | :--- | :--- |
| **01** | [**底层算法与运动学推导**](docs/01_ALGORITHM_AND_KINEMATICS.md) | 机械臂连杆几何模型、解析几何法正逆解 (FK/IK) 公式推导、高位构型 (Elbow-Up) 约束、双向坐标系偏置映射 (JointTransformer) 与五次多项式平滑插值数学证明 |
| **02** | [**通信架构与数据流**](docs/02_COMMUNICATION_PROTOCOL.md) | mission 到规划器、规划器到桥、桥到下位机的接口、二进制结构、ACK/状态语义、完整时序与已知限制 |
| **03** | [**对外暴露接口与 API 说明**](docs/03_INTERFACES_AND_API.md) | 节点对外暴露的所有 ROS 2 服务 (`move_cartesian`, `move_joints`, `execute_preset`, `home`, `retract`, `stop`)、双模式解算与 Python 客户端代码调用范式 |
| **04** | [**启动、调参配置与实机调试**](docs/04_SETUP_AND_DEBUGGING.md) | Launch 启动与参数注入、`arm_presets.yaml` 零位偏置与限位调参手册、CLI 命令行快速调试指令集与现场常见故障排查手册 (Troubleshooting) |
| **附录** | [**重构清单**](docs/REFACTOR_SPEC_AND_CHECKLIST.md) | 已实施的包内改造、验收边界及待核实的固件事实 |

---

## 快速上手与运行 (Quick Start)

### 1. 编译功能包
```bash
# 注入统一运行环境 (CycloneDDS 与系统 OpenCV 4.5.4)
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

# 编译
colcon build --packages-select fruit_arm_planner
```

### 2. 启动规划节点
```bash
ros2 launch fruit_arm_planner fruit_arm_planner.launch.py
```

### 3. 常用调试指令
```bash
# 触发安全 Home 姿态复位
ros2 service call /arm_planner_node/home std_srvs/srv/Trigger

# 触发底盘避障折叠姿态
ros2 service call /arm_planner_node/retract std_srvs/srv/Trigger

# 发布笛卡尔采摘坐标 (X=0.20m, Y=0.0m, Z=0.25m)
ros2 topic pub --once /arm_planner_node/cmd_cartesian geometry_msgs/msg/PointStamped \
  "{point: {x: 0.20, y: 0.0, z: 0.25}}"

# 监听当前实测物理角度 (20Hz)
ros2 topic echo /arm_planner_node/joint_states
```

---

## 目录结构一览

```text
src/fruit_arm_planner/
├── config/
│   └── arm_presets.yaml             # 几何尺寸、关节偏置与预设姿态配置文件
├── docs/
│   ├── 01_ALGORITHM_AND_KINEMATICS.md   # 底层算法与运动学推导
│   ├── 02_COMMUNICATION_PROTOCOL.md    # 上位机与下位机完整通信架构
│   ├── 03_INTERFACES_AND_API.md        # 对外暴露接口与 API 说明
│   ├── 04_SETUP_AND_DEBUGGING.md       # 启动、调参配置与实机调试手册
│   └── REFACTOR_SPEC_AND_CHECKLIST.md  # 重构记录与待核实事项
├── fruit_arm_planner/
│   ├── __init__.py                  # 模块导出定义
│   ├── arm_planner_node.py          # ROS 2 核心节点
│   ├── joint_transformer.py         # 关节坐标系映射与零位偏置解耦模块
│   ├── kinematics.py                # 4-DOF 纯几何运动学求解器 (FK/IK)
│   └── trajectory_planner.py        # 五次多项式平滑轨迹插值器
├── launch/
│   └── fruit_arm_planner.launch.py  # 启动文件 (指向 src 下的 config)
├── package.xml                      # ROS 2 包定义
├── README.md                        # 总导航入口与快速上手手册
├── resource/
│   └── fruit_arm_planner
├── setup.cfg
└── setup.py
```
