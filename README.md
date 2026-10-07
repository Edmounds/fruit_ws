# 水果采摘机器人 ROS 2 工作区

[项目文档导航](docs/README.md)汇总架构、导航、感知、录点、语音和历史计划。
开发约束见 [AGENTS.md](AGENTS.md)。当前比赛范围为 A/B/C。

驱动源码位于 `src/driver/`：RealSense 使用仓库已配置的 Git submodule；
Livox 源码当前有 `COLCON_IGNORE`，普通 workspace 构建不会编译它。
不要在工作区根目录重复 clone 驱动来替代已配置的路径。

## ROS 运行环境

### 传感器命名

项目默认称深度摄像头为 `camera`（当前 Intel RealSense D435i），雷达为 `lidar`
（当前 Livox MID-360）。左右 USB 相机已彻底删除，全赛区仅使用单 `camera`。

| 用途 | 当前名称 |
| --- | --- |
| 深度相机节点 / 原生接口 | `/camera` / `/camera/*`，例如 `/camera/rgbd`、`/camera/set_parameters` |
| 深度相机感知接口 | `/fruit_perception/camera/*`；决策统一发布至 `/fruit_perception/decision`（不含 source 字段） |
| 雷达测距 / 内置 IMU 坐标系 | `lidar_link` / `lidar_imu_link`（外参不变） |
| 雷达驱动话题 | 保留 `/livox/lidar`、`/livox/imu`，消息类型不变 |
| 雷达启动参数 / 配置文件 | `lidar_config` / `src/navigation/fruit_nav_bringup/config/reality/lidar_user_config.json` |
| 深度相机单树调试入口 | `./test_camera_arm_target.sh` / `ros2 run fruit_mission camera_arm_target_client` |

第三方 `realsense2_camera`、`livox_ros_driver2` 和 SDK/API 保留原名；雷达 JSON 中
厂商规定的 `MID360` 字段也保留。相机光学坐标系仍使用驱动的 `camera_*_optical_frame`。
旧配置键、脚本入口、话题和 TF 名称不提供兼容别名；旧录包不迁移，回放时需按需映射
话题并处理消息内旧 `frame_id`，只 remap 话题不会修改坐标系名。

更新后停止旧节点，在未激活训练 venv 的终端中重建并重新加载环境，再启动项目：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

项目统一使用 Cyclone DDS，避免 Fast DDS 的 service request/reply 独立发现竞态导致Action 已执行但 goal response 丢失。首次部署需要安装：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

所有启动、调试和 ROS 2 CLI 终端都必须先加载项目环境；该脚本会固定
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，缺少运行库时直接停止：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```


切换前已经启动的 ROS 进程仍使用旧 RMW，不能与新进程混跑。停止整套旧进程后重新加载
环境并启动；若 ROS 2 daemon 曾由旧环境启动，切换时执行一次 `ros2 daemon stop`。

### 启动脚本复用串口桥

`start_fruit_perception.sh`、`start_navigation.sh`、`waypoints_maker.sh`、
`start_tele.sh`、`start_fruit_mission.sh` 和 `test_camera_arm_target.sh`
会检查当前 ROS 图中的 `serial_controller`、机械臂状态和桥 ready 发布者。
检测到已有桥就复用，不再重复启动，也不在退出时停止该桥；没有才启动。
已有桥即使 `ready=false` 也不会被当成缺失，硬件就绪等待仍由原有流程负责。
复用不会修改已有桥的串口、日志或 raw-frame 配置；重复桥或消息类型冲突仍报错。
相机测试仍拒绝已有感知容器、arm planner 或 mission，避免重复控制。
不使用串口桥的脚本无需改动；`test_camera_detector.py` 已有复用逻辑。

更新后重新构建 `fruit_mission` 和 `joy_linux`，再启动脚本：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_mission joy_linux \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
# 只读检查：true 表示需要启动桥，false 表示复用已有桥
ros2 run fruit_mission stack_readiness_client bridge-start-required
```

### 清理残留进程和设备占用

启动整套任务、摄像头调参或串口直连工具前，可先检查再清理项目进程：

```bash
cd /home/rc4/fruit_ws
./kill_fruit_processes.sh --dry-run
./kill_fruit_processes.sh
fuser -v /dev/stm32
```

清理脚本覆盖任务启动器、ROS launch/run、`test_camera_detector.py` 创建的
`test_camera_detector_*_container`、`lower_controller_console.py`、录包进程和调试监视器；
它会先恢复被 `Ctrl-Z` 挂起的目标，再依次发送 `SIGINT`、`SIGTERM` 和 `SIGKILL`，成功后
停止 ROS 2 daemon 以清除过期图缓存。RobViz、`foxglove_bridge.service` 和
`waypoint-editor.service` 是独立工具，不由该脚本停止，也不占用摄像头或下位机串口。

`fruit-perception.service` 会占用摄像头和 `/dev/stm32`。如果清理脚本提示该服务处于
active，必须先执行：

```bash
sudo systemctl stop fruit-perception.service
```

### 输出水果检测结果
```bash
ros2 topic echo /fruit_perception/decision
```

### 相机与机械臂配合调试

测试机械臂观察与单点识别抓取闭环：

```bash
cd /home/rc4/fruit_ws
# 蔬菜模型 (A区): 左侧观察并抓取
./test_camera_arm_target.sh --zone A --left

# 水果模型 (B区): 不区分苹果或梨，抓取成熟水果
./test_camera_arm_target.sh --zone B --right
./test_camera_arm_target.sh --zone B --left --debug

# C 区蔬菜模型及 C 区姿态（补齐 YAML 姿态后才运行）
./test_camera_arm_target.sh --zone C --left
```

参数说明：
- `--zone A|B|C`：同时选择区域模型和姿态；A/C 用蔬菜模型，B 用水果模型。旧 `--model` 已删除。
- `--left|--right`：指定作业侧，执行 `<side>_look_<zone>` / `<side>_store_<zone>`，区域后缀小写，例如 `left_look_a`、`right_store_c`。
- `--debug`：显式开启串口关键数据帧（轨迹头、轨迹点、控制模式）的终端解码打印；原始 raw 十六进制数据已彻底移除，逐帧相机详情与高频状态包统一落盘至会话日志文件。

流程说明：
脚本等待实际感知节点的区域订阅端就绪后发布区域，并等待可靠 DDS 确认。工序为 `home` → 区域 Look → 实测关节角 OPEN/HOLD → harvest 窗口 → 目标 CLOSE/HOLD → 区域 Store → `home`。B 只选统计完成后的有效成熟水果；A/C 使用蔬菜目标。坐标换算和 `target_offset_m` 仍共用 mission 实现。运动等待下位机结果，端点、识别与区域交付各自限时最多 10 秒；共享 OPEN/HOLD 逐次等待新鲜有效的静止反馈，不做外层即时拒绝。

正式 mission 在旧正常收臂边界调用 `shenzhi`，扫码直接调用 `qr_scan`，不使用下位机任务或摄像头切换状态机。QR 按当前窗口 B/C 区域解析，不依赖先 B 后 C。真实姿态及已定义名称以源码 YAML 为准，现场运行前须补齐所用预设；客户端不填空轨迹或覆盖已有坐标。缺失按失败处理，必要保护姿态未确认不放行转场；单点测试不调用 `shenzhi`。

## 运行日志与终端调试说明

工作区对 7 个常用启动入口（`test_camera_arm_target.sh`、`start_fruit_perception.sh`、`start_fruit_mission.sh`、`start_navigation.sh`、`start_tele.sh`、`waypoints_maker.sh`、`record_rosbag.sh`）实施了统一的日志落盘与终端分流机制：

- **终端精简输出**：
  - 默认仅输出关键生命周期推进（就绪、连接、观察窗口、目标锁定坐标、轨迹执行完成结果、告警与错误），不刷高频周期巡检与冗长启动参数。
  - **彻底移除 raw 原始十六进制字节**（如 `[MCU TX RAW]`），无论是否开启 debug，终端均不再输出 raw 数据。
  - 显式传入 `--debug` 选项时，仅额外在终端打印当前允许的串口关键数据帧（轨迹头、轨迹点、控制模式）的结构化解码文本。高频底盘速度、机械臂 20Hz 状态、系统 ACK/心跳等即便在 `--debug` 下也仅保存于磁盘日志，避免刷屏。
- **详细诊断落盘与保留**：
  - 每次运行的完整日志存放于 `/home/rc4/fruit_ws/log/runtime/<时间戳>_<入口>_<PID>/`。
  - 最新的运行目录可通过软链接 `/home/rc4/fruit_ws/log/runtime/latest` 快速访问。
  - 详细内容包括：`orchestrator.log`（全局生命周期与进程列表）、各子组件独立日志（`perception.log`、`arm_planner.log`、`client.log`、`navigation.log`、`monitor.log` 等）、所有收发包完整解码字段以及 `ros/` 原生日志。
  - 系统启动与退出时自动清理已结束的运行目录，严格保留**最近 20 次**记录；活动运行通过 `.active.lock` 文件锁保护，禁止误删。
- **AI Agent 排查指引**：详见 [AGENTS.md](AGENTS.md)。

## 上位机机械臂调试

机械臂控制统一通过 `fruit_arm_planner`，先阅读
[接口 API](src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md) 和
[调试手册](src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md)。以下为实机操作入口，
会驱动机械臂；已有串口桥时只启动 planner，不重复启动 debug launch 中的桥。

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 launch fruit_arm_planner fruit_arm_planner.launch.py
# 或桥 + planner（与上一条二选一，且当前没有其他串口桥）：
# ros2 launch fruit_arm_planner fruit_arm_debug.launch.py
# 另一个已加载同一运行环境的终端：
/usr/bin/python3 src/fruit_arm_planner/test/cartesian_console.py --log-level info
```

控制台 `move` 使用基座 XYZ（m）；`joints` 使用连杆角（deg），`motors` 使用原始硬件角（deg）。
预设名称以 `src/fruit_arm_planner/config/arm_presets.yaml` 为准，不能假定 home/retract 一定存在。
`status` 查看反馈，Ctrl+C 请求急停；STOP/ESTOP 成功响应只代表发布，不确认硬件停稳。
旧 `lower_controller_console.py` 直接发轨迹，不再作为正常机械臂控制入口。

**集成状态：**mission 和单树客户端通过 planner 服务编排 Look、二维码观察、目标 CLOSE、Harvest 和 Home；
mission 负责相机坐标转换、目标筛选与 A/B/C 顺序，planner 负责运动学、映射和轨迹执行。
启动脚本已启动 planner-only launch。串口桥跨类型轨迹装载顺序、真实预设轨迹、标定、STOP 优先级和下位机保护仍需现场验收，不能把服务成功当作完整实机闭环已验收。

## 录制调试 ROS 话题
```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
./record_rosbag.sh --full
```


录包保留所有非视觉话题。视觉数据保留单 camera 的彩色原图、对齐深度图和标注图。camera 未对齐的
`/camera/depth/image_rect_raw`、重复的 `/camera/rgbd`，以及所有
`compressed`、`compressedDepth`、`theora` 派生话题均通过 rosbag 黑名单排除。

## 语音播报排查

原 `test_tts.sh` 已不存在。播报文本与顺序见 [VOICE_FLOW.md](docs/VOICE_FLOW.md)，
本机蓝牙音量与服务排查见 [AUDIO_VOLUME.md](docs/AUDIO_VOLUME.md)。
语音采用维什戴尔（Wis'adel）gremlin 预录素材离线拼接播放，素材清单以 `src/fruit_mission/config/wisadel_voice.yaml` 为准。
可使用离线检查与试听工具排查（只产生声音，不启动任务或机械臂）：
```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
# 检查文案与本地音频文件完整性
python3 scripts/prepare_wisadel_voice.py check
# 生成并试听代表场景音频
python3 scripts/prepare_wisadel_voice.py render --scenario intro --play
python3 scripts/prepare_wisadel_voice.py render --scenario summary_multi --play
```
当前播报实现调用 aplay 播放本地拼接 WAV，比赛运行时完全离线，不依赖 GPU 服务器。
