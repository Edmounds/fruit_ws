# 纯里程计导航架构与使用指南 (Odometry Navigation)

本项目采用 **Point-LIO + fruit_navigation** 的纯里程计导航方案，去除了传统建图（GLIM）、地图保存与 GICP 全局重定位链路，直接在开机原点建立的 `odom` 坐标系下实现高精度轴向跟踪与全自动水果采摘作业。

---

## 1. 架构设计与坐标系约定

### 1.1 坐标系拓扑
- **`odom` -> `base_link`**: 由 Point-LIO 前端结合 `loam_interface` 实时解算并发布 TF，描述机器人相对于开机上电瞬间的位姿。
- **无 `map` 坐标系**: 全系统统一在 `odom` 坐标系下运行，无地图加载、无重定位跳变、无初始位姿对齐需求。
- **航向约定**: 机器人开机初始朝向即为 `odom` 坐标系的 $+X$ 轴正方向，左侧为 $+Y$，垂直向上为 $+Z$。

### 1.2 核心导航组件
1. **`livox_ros_driver2`**: MID-360 激光雷达驱动（位于 `src/driver/livox_ros_driver2`）。
2. **`point_lio`**: 高频 LiDAR-IMU 紧耦合里程计（位于 `src/navigation/point_lio`）。
3. **`loam_interface`**: 转换 Point-LIO 的 `/aft_mapped_to_init` 并在 `odom` 坐标系下发布 `base_link` TF。
4. **`fruit_navigation`**: 轴向定点导航 Action 服务端（`axis_waypoint_action_server`），执行逐轴对齐、直线行走与偏航角控制。
5. **`fruit_nav_bringup`**: 纯里程计导航集成启动包（位于 `src/navigation/fruit_nav_bringup`）。

---

## 2. 航向角控制模式

`axis_waypoint_action_server` 支持三种偏航角执行模式（通过 waypoint YAML 中每个点的 `yaw_mode` 配置或全局默认 `final_yaw_mode` 设置）：
1. **`blend_on_path`（默认）**: 移动过程中平滑插值偏航角，在接近目标点时融合到位，效率最高。
2. **`rotate_in_place`**: 到达目标 $(X, Y)$ 位置后原地旋转调整到目标偏航角。
3. **`reverse`**: 允许倒退对齐，用于特定狭小空间的快速回退。

---

## 3. 核心脚本与使用方法

### 3.1 启动纯里程计 (`start_odometry.sh`)
仅启动雷达与 Point-LIO 里程计，用于测试雷达通信及里程计稳定性：
```bash
./start_odometry.sh
```
- 启动 `pointlio_odometry.launch.py`。
- 自动检测 `/aft_mapped_to_init` 话题及 `odom -> base_link` 5 秒内的稳定 TF 输出。

### 3.2 启动里程计导航与可视化 (`start_navigation.sh`)
启动底层串口通讯、Point-LIO 里程计、轴向导航服务以及 RViz 界面：
```bash
./start_navigation.sh
```
- 无需指定 `--map` 或 `--odom` 参数。
- RViz 默认 Fixed Frame 为 `odom`。
- 可直接通过 RViz 交互或发送 Action 目标控制底盘。

### 3.3 录制与编辑航点 (`waypoints_maker.sh`)
用于现场采集或示教目标航点：
```bash
./waypoints_maker.sh
# 若需要配合手柄遥控底盘移动：
./waypoints_maker.sh --tele
```
- 录制的航点保存至 `src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml`（`frame_id: odom`）。

### 3.4 启动全流程采摘比赛 (`start_fruit_mission.sh`)
一键拉起里程计导航、全相机视觉感知、串口协议桥接与任务状态机：
```bash
# 全流程启动（包含 A 区、B 区、C 区）
./start_fruit_mission.sh

# 指定局部流程（例如仅执行 AC 区）
./start_fruit_mission.sh --flow ac

# 复用已有 start_navigation.sh 运行实例
./start_fruit_mission.sh --reuse-navigation

# 开启调试录包
./start_fruit_mission.sh --debug
```

### 3.5 进程管理与停止 (`kill_fruit_processes.sh`)
安全终止所有任务进程、感知节点、导航启动栈并清理 ROS 2 守护进程：
```bash
./kill_fruit_processes.sh
```

---

## 4. 故障排查与运行监视

- **运行监视工具**: `scripts/robot_debug_monitor.sh` 实时监视导航状态、D435i 双目流帧率、相机模式切换及异常告警。
- **环境检查**: 启动前确保已 source `scripts/setup_fruit_runtime.bash`。
