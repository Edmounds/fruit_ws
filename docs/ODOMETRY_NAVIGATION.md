# 纯里程计导航架构与使用指南 (Odometry Navigation)

本项目采用 **Point-LIO + fruit_navigation** 的纯里程计导航方案，去除了建图（GLIM）、地图保存与 GICP 全局重定位链路，在开机原点建立的 `odom` 坐标系下进行轴向航点跟踪。机械臂整机集成限制见[项目架构](PROJECT_ARCHITECTURE.md)，不据此宣称全自动采摘已验收。

---

## 1. 架构设计与坐标系约定

### 1.1 坐标系拓扑
项目中 `lidar` 指雷达（当前 MID-360），`camera` 指深度摄像头（当前 D435i）。
雷达仍发布 `/livox/lidar` 与 `/livox/imu`；`lidar_link` 和 `lidar_imu_link` 分别描述
测距中心与内置 IMU，不可合并。驱动配置为 `config/reality/lidar_user_config.json`，
启动参数为 `lidar_config`，厂商 JSON 的 `MID360` 字段保持原样。
Point-LIO 原生的 `camera_init` / `body` 坐标系也保留；它们属于雷达里程计内部约定，
不是深度摄像头的 `camera_*` 坐标系。

- **`odom` -> `base_link`**: 由 Point-LIO 前端结合 `loam_interface` 实时解算并发布 TF，描述机器人相对于开机上电瞬间的位姿。
- **无 `map` 坐标系**: 全系统统一在 `odom` 坐标系下运行，无地图加载、无重定位跳变、无初始位姿对齐需求。
- **航向约定**: 机器人开机初始朝向即为 `odom` 坐标系的 $+X$ 轴正方向，左侧为 $+Y$，垂直向上为 $+Z$。
- **`base_link` -> `lidar_link` (传感器安装外参)**:
  - 权威统一定义于 `src/navigation/fruit_nav_bringup/config/reality/sensor_frames.yaml`（本项目遵循 KISS 原则，不引入整机 URDF，外参数值仅在此单一文件中维护，下游节点一律通过 TF 动态获取）。
  - 由 `sensor_frames.launch.py` 启动 `tf2_ros static_transform_publisher` 广播。
  - 当前参数：`translation: [0.13085, 0.0, 0.0]`，`rotation: [0.0, 0.0, 0.0, 1.0]`（雷达 X 轴与 `base_link` 平行正向，Y/Z/Yaw/Pitch/Roll 均为 0）。
- **`lidar_link` -> `lidar_imu_link` (MID-360 机内 IMU 外参)**:
  - 描述 MID-360 光学测距中心到机内内置 IMU 芯片的物理偏移（`[0.011, 0.02329, -0.04412]`），独立于整机安装外参。

### 1.2 核心导航组件
1. **`livox_ros_driver2`**: MID-360 激光雷达驱动（位于 `src/driver/livox_ros_driver2`；该源码目录当前有 `COLCON_IGNORE`，普通 workspace 构建不会编译它，启动前须确认已安装驱动可发现）。
2. **`point_lio`**: 高频 LiDAR-IMU 紧耦合里程计（位于 `src/navigation/point_lio`）。
3. **`loam_interface`**: 转换 Point-LIO 的 `/aft_mapped_to_init` 并在 `odom` 坐标系下发布 `base_link` TF。
4. **`fruit_navigation`**: 定点导航 Action 服务端（`axis_waypoint_action_server`），执行入段对齐、点间直线跟踪与航点偏航角控制，不在服务端重新拆分 X/Y 路段。
5. **`fruit_nav_bringup`**: 纯里程计导航集成启动包（位于 `src/navigation/fruit_nav_bringup`）。

---

## 2. 航向角控制模式

`axis_waypoint_action_server` 的 `final_yaw_mode` 参数只支持两种值，配置在
`src/navigation/fruit_navigation/config/axis_navigation.yaml`：
1. **`rotate_at_goal`（默认）**: 沿点间直线到达目标 $(X, Y)$ 并减速停车后，原地调整到航点保存的目标偏航角，不提前切弯。
2. **`blend_on_path`**: 最终航点的末段融合目标偏航角，位置到位并减速停车后，剩余航向误差交给独立转向阶段完成。目标航向不沿入射直线时仍可能产生弧线甚至无法到点，窄通道不推荐此模式。

航点 YAML 保存目标 orientation/yaw，不提供逐点 `yaw_mode`。倒车由目标 yaw 与
反向路径航向的夹角及 `reverse_yaw_tolerance` 自动判定，不是第三种模式。
这里的 orientation/yaw 控制**车体朝向**，不是机械臂朝向；机械臂控制不由导航节点执行。

### 2.1 直线跟踪与到点判据

直线控制借鉴 [Nav2 Regulated Pure Pursuit（RPP）](https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller)
的前视点和曲率调速思路，不引入完整 Nav2 栈或新依赖：

- 用 `lookahead_distance`（米）选取直线前方的转向参考点，按 `angular.z = linear.x * curvature` 耦合转向与有符号线速度。倒车以及纵向越过终点后的速度反向使用同一公式，不再使用固定符号的横向角速度反馈。
- 转向参考点在末端仍保留前视距离，避免它缩到零时放大定位噪声；它只是转向参考，不是允许驶过目标的导航目标。
- 线速度同时受现有速度范围、接近目标的比例减速、制动距离与角速度上限约束；保留底盘最低线速度和现有加速度限制。
- `lookahead_distance` 替代旧 `lateral_gain`。较大值通常转向更柔和、纠偏更慢；较小值纠偏更快，也更敏感。调整前视距离后应重启导航节点。
- 位置阶段只检查既有 `forward_tolerance` / `lateral_tolerance`；位置到位后先将速度指令减到零，再由既有 `_rotate_to` 检查目标 yaw。位置进度使用平面距离，转向进度使用角度误差，避免原地调朝向时被位置 watchdog 误报失败。
- `waypoint_retry_attempts` 和 `waypoint_pass_tolerance` 的既有恢复语义不变，失败兜底仍仅用于非最终航点；不新增 mission 特判或扩大容差。航点位置、最终车体朝向以及机械臂行为不变；默认 `final_yaw_mode` 改为 `rotate_at_goal`，只改变最终航点的转向时机。

此控制器没有 costmap、避障或整机轮廓碰撞检查，前视点跟踪不构成安全走廊保证；必须验证转弯和纠偏时的整机扫掠空间。速度指令归零也不等于实测底盘已经静止。
源码变更后需构建 `fruit_navigation` 并重启导航；纯运动学检查不代表实机比赛验收。

离线检查与单包构建（不启动节点、不命令底盘）：
```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
/usr/bin/python3 -m compileall -q src/navigation/fruit_navigation/fruit_navigation
/usr/bin/python3 -m pytest -q src/navigation/fruit_navigation/test/test_axis_planner.py
colcon build --base-paths src --packages-select fruit_navigation
```

---

## 3. 核心脚本与使用方法

### 3.1 导航入口 (`start_navigation.sh`)
启动底层串口通讯、Point-LIO 里程计和轴向导航服务。可通过 RobViz 查看状态和发送导航目标：
```bash
./start_navigation.sh
```
- 无需选择建图或里程计模式，也无需传配置路径；参数由 source 目录中的 launch 和 YAML 加载。
- 入口统一为 `start_navigation.sh`，不再保留独立的里程计 shell 入口。
- 启动前检测 `livox_ros_driver2`、`laserMapping`、`loam_interface`、`axis_waypoint_action_server` 等残留节点，避免重复启动造成 UDP 端口绑定冲突与 TF 冲突。
- 自动检测 `/aft_mapped_to_init` 话题及 `odom -> base_link` 5 秒内的稳定 TF 输出。
- RobViz 的 Fixed Frame 使用 `odom`。
- 可通过 RobViz 交互或发送 Action 目标控制底盘。

### 3.2 录制与编辑航点 (`waypoints_maker.sh`)
用于现场采集或示教目标航点：
```bash
./waypoints_maker.sh
# 若需要配合手柄遥控底盘移动：
./waypoints_maker.sh --tele
```
- 录制的航点保存至 `src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml`（`frame_id: odom`）。

### 3.3 启动全流程采摘比赛 (`start_fruit_mission.sh`)
`--flow` 现在选择 `Logic.py` 中的入口函数，完整比赛函数不再混入模式分支。
固定 A 导航点和区间通道点在调用处可见；完整流程按 A → B → C 执行，
B 区固定访问 1～8 号树，C 区按二维码顺序访问。B/C 单区调试仍先扫描对应二维码，
失败则用 mission 自有兜底计划。路线坐标、C 几何规划与停稳/取消门控不变。
编排和重复动作的文件边界见 [fruit_mission README](../src/fruit_mission/README.md)。

任务启动入口负责里程计导航、单 camera 感知、串口桥与任务状态机。以下是脚本使用方法，
不是实机采摘验收结论；观察/收臂旧接口及 planner 启动需按架构文档核对：
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

### 3.4 进程管理与停止 (`kill_fruit_processes.sh`)
安全终止所有任务进程、感知节点、导航启动栈并清理 ROS 2 守护进程：
```bash
./kill_fruit_processes.sh
```

---

## 4. 故障排查与运行监视

- **运行监视工具**: `scripts/robot_debug_monitor.sh` 监视导航状态、camera 彩色/深度流与 RGBD heartbeat、模式切换及异常告警。
- **环境检查**: 启动前确保已 source `scripts/setup_fruit_runtime.bash`。
- **底层里程计调试**: 仅需雷达和里程计时，可在加载运行环境后直接执行
  `ros2 launch /home/rc4/fruit_ws/src/navigation/fruit_nav_bringup/launch/pointlio_odometry.launch.py`。
- **接口约定**: 控制器和任务配置仍沿用 `map_frame` 参数名，其值为 `odom`；这不是地图模式开关。
  `/follow_waypoints` 沿用 `nav2_msgs/action/FollowWaypoints` 消息接口，不启动 Nav2 地图导航栈。
  控制器不再包含 GICP 开关、诊断订阅或定位状态门控；新鲜 TF、取消和停车保护继续保留。

### 4.1 静止实测参考（2026-10-02）

在用户确认底盘静止、仅启动雷达与 Point-LIO 里程计栈的条件下，从 `/odometry` 采集约 30 秒、600 帧数据（约 20 Hz）：

| 分量 | 标准差（相对窗口均值） | 峰峰值（最大值减最小值） |
| --- | --- | --- |
| X | 5.60 mm | 31.72 mm |
| Y | 3.07 mm | 16.29 mm |
| Z | 4.25 mm | 37.04 mm |
| 航向角 yaw | 0.027° | 0.242° |

- 平面位置相对窗口均值的偏离：95% 不超过 12.88 mm，最大 23.43 mm。首尾各 1 秒均值的平面位移约 1.11 mm；本次短窗口以抖动为主，不代表长期无漂移。
- `/odometry` 的速度由相邻位姿差分得到，静止时三维线速度模长的中位数仍约 0.22 m/s、95 分位约 0.43 m/s。这是位姿噪声经差分放大的输出，不能据此认定底盘实际移动。
- 同时间戳对照验证：底盘位姿与 TF 一致，且满足 `T_odom_base = T_odom_lidar * inverse(T_base_lidar)`。`loam_interface` 负责坐标转换与初始原点对齐，不消除 Point-LIO 自身的噪声或长期漂移。
- 上述结果是当次环境的重复性统计，没有外部真值，不能当作绝对定位精度或永久验收阈值。
- 本次调试发现有线口原 DHCP 配置无法保留雷达所需主机地址。使用了不落盘的临时 NetworkManager 连接 `fruit-odom-review`（`192.168.1.5/24`，无默认路由）；它不替代后续正式网络配置。
