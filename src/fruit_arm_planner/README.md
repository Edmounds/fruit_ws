# fruit_arm_planner

水果采摘机器人 4 自由度采摘机械臂运动学解算、关节坐标映射与轨迹规划功能包。

mission 可调用关节单目标、机械臂根部坐标系下的笛卡尔单目标或预设名称；本包从新鲜四关节实测状态生成以 `t=0` 为起点的轨迹，并等待下位机报告匹配的执行终态。相机坐标转换和比赛任务流程由其他模块负责。详细边界与现状见[通信架构文档](docs/02_COMMUNICATION_PROTOCOL.md)，实施与待核实事项见[重构清单](docs/REFACTOR_SPEC_AND_CHECKLIST.md)。

## 当前集成状态

`fruit_mission` 和单 camera 调试客户端已通过本包服务编排 Look、二维码观察、目标 CLOSE、Harvest 和 Home。
mission 负责相机坐标转换、目标筛选和 A/B/C 顺序，本包负责 FK/IK、关节映射、轨迹规划和执行结果；
任务层不再发送旧 `ArmTarget` / `StateMachineCommand` 或等待 `ExecutionCallback`。
目标 CLOSE 完成后，mission 复用 `move_joints` 在新鲜实测四轴角执行 planner 的 HOLD 驻留，再调用 Harvest/Home。
启动脚本已启动 planner-only launch。当前仅发布 Header/Point，完整点位由下位机自动启动。
桥可靠发送仍按消息 ID 排队，Header/Point 尚无跨类型装载确认；
真实预设轨迹、标定、STOP 优先级和下位机保护仍需现场验收，服务成功不能单独证明完整实机闭环。
坐标边界见[mission 换算说明](../fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md)，控制必须按[接口 API](docs/03_INTERFACES_AND_API.md)。

笛卡尔 XYZ 统一指真实夹爪抓取点 TCP。`config/arm_presets.yaml` 的 `geometry.tool_z_offset_m` 配置随腕部旋转的固定下偏（当前 `-0.03 m`）；`pitch=0°` 表示 `a4` 伸出段水平，不是腕轴到 TCP 的斜线水平。直接输入和笛卡尔预设共用 FK/IK，不再额外平移输入。现有预设数值不自动迁移；几何调参后重启 planner、控制台和 mission。

`/arm_planner_node/joint_states` 现为 `fruit_interfaces/msg/ArmTelemetry`，原话题名不变：保留原始硬件角/速度，新增逆映射模型角/速度及真实 TCP XYZ（m）/pitch（度）。控制台 `status`、jog/示教与 mission 直接使用 planner 遥测，不重复减偏置或 FK。标准 JointState 订阅者需适配；接口、planner、mission 必须一起构建并重启。原始硬件 Status 和运动服务输入语义不变。

---

## RobViz 手动调试

`fruit-arm` 插件通过正式服务执行单姿态、预设、Home、夹爪和累计 jog。
二维拖动、三维轨迹、文件保存均不执行运动。
`get_configuration` 提供只读内存快照，预览复用 `planning_api.py` 的纯规划函数。
文件保存与 `reload_presets` 分开。几何和 mapping 仍只读。
详见 [RobViz 机械臂插件](../../docs/ROBVIZ_ARM_PLUGIN.md)。

## 专题文档导航 (Documentation Index)

本包文档存放于 [`docs/`](docs/) 目录下。通信文档区分已实现行为与跨包待核实事实。

| 分册编号 | 专题文档名称 | 核心内容概述 |
| :---: | :--- | :--- |
| **01** | [**底层算法与运动学推导**](docs/01_ALGORITHM_AND_KINEMATICS.md) | 机械臂连杆几何模型、解析几何法正逆解 (FK/IK) 公式推导、高位构型 (Elbow-Up) 约束、双向坐标系偏置映射 (JointTransformer) 与五次多项式平滑插值数学证明 |
| **02** | [**通信架构与数据流**](docs/02_COMMUNICATION_PROTOCOL.md) | mission 到规划器、规划器到桥、桥到下位机的接口、二进制结构、ACK/状态语义、完整时序与已知限制 |
| **03** | [**对外暴露接口与 API 说明**](docs/03_INTERFACES_AND_API.md) | 节点对外暴露的所有 ROS 2 服务 (`move_cartesian`, `move_joints`, `execute_preset`, `home`, `stop`)、双模式解算与 Python 客户端代码调用范式 |
| **04** | [**启动、调参配置与实机调试**](docs/04_SETUP_AND_DEBUGGING.md) | Launch 启动与参数注入、`arm_presets.yaml` 零位偏置与限位调参手册、CLI 命令行快速调试指令集与现场常见故障排查手册 (Troubleshooting) |
| **附录** | [**重构清单**](docs/REFACTOR_SPEC_AND_CHECKLIST.md) | 已实施的包内改造、验收边界及待核实的固件事实 |
| **实机核查** | [**四轴角度与解算链路核查**](docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md) | 同步带小臂与相对腕部的角度关系、Q3 零位修正和剩余校准项 |

---

## 快速上手与运行 (Quick Start)

### 1. 编译功能包
```bash
# 注入统一运行环境 (CycloneDDS 与系统 OpenCV 4.5.4)
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

# 编译
cd /home/rc4/fruit_ws
colcon build --base-paths src --packages-up-to fruit_arm_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

### 2. 启动规划节点
```bash
ros2 launch fruit_arm_planner fruit_arm_planner.launch.py
```

实机调试时可用 `ros2 launch fruit_arm_planner fruit_arm_debug.launch.py` 同时启动
通信桥与规划器，再于另一终端运行
`python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py`。
控制台命令及 Ctrl+C 急停语义见[启动与调试手册](docs/04_SETUP_AND_DEBUGGING.md)。
控制台支持 `move` 笛卡尔目标、`joints` 连杆角目标（第四项为末端绝对俯仰角，按 YAML 标定映射到硬件角）、`motors` 原始硬件角目标和 `preset` 预设动作，命令提交后由规划器直接执行；
还支持 `--log-level info|debug|trace`，运行中可用 `level` 命令切换；
`debug` 显示每帧状态，`trace` 还显示每个 ROS 轨迹路点。

### 3. 常用调试指令

下列运动调用会实际驱动硬件，先确认周围安全且反馈有效。
预设名以源码 YAML 为准；当前有 `zero/left_look/right_look/pick_sequence/home/left_store/right_store`，
`home` 快捷服务依赖同名预设存在，否则会返回失败。
```bash
# 通过正式服务执行配置中存在的预设
ros2 service call /arm_planner_node/execute_preset fruit_interfaces/srv/ExecuteArmPreset \
  "{preset_name: 'zero'}"

# 固定机械臂基座坐标，夹爪保持；success 表示匹配轨迹已报告完成
ros2 service call /arm_planner_node/move_cartesian fruit_interfaces/srv/MoveArmCartesian \
  "{x: 0.20, y: 0.0, z: 0.25, pitch_deg: 0.0, duration_ms: 2500, gripper_action: 0, linear_path: false}"

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
│   ├── planning_api.py              # ROS 与 HTTP 共用的纯规划入口
│   └── trajectory_planner.py        # 五次多项式平滑轨迹插值器
├── launch/
│   ├── fruit_arm_planner.launch.py  # 单独启动规划器
│   └── fruit_arm_debug.launch.py    # 同时启动通信桥与规划器
├── test/
│   └── cartesian_console.py         # 手动运行的实机笛卡尔调试控制台
├── package.xml                      # ROS 2 包定义
├── README.md                        # 总导航入口与快速上手手册
├── resource/
│   └── fruit_arm_planner
├── setup.cfg
└── setup.py
```
