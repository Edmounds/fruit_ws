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
停止 ROS 2 daemon 以清除过期图缓存。RViz、`foxglove_bridge.service` 和
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

### B 区单树感知与机械臂测试

测试 B 区单棵树时必须指定观察方向和二维码期望品种：

```bash
cd /home/rc4/fruit_ws
./test_camera_arm_target.sh --zone B --right --apple
./test_camera_arm_target.sh --zone B --left --pear --debug
```

`--apple` 和 `--pear` 只能用于 B 区且必须二选一。脚本等待感知节点完成 50 帧
整树统计，再按正式 mission 的规则比较整树品种和期望品种，只向下位机发送品种一致的
成熟目标对应的 planner 请求。没有目标、品种不一致或只有未成熟目标时正常跳过，不发送 `ArmTarget`。
单树抓取、跳过或执行失败后都会发送 `task_id=7` 并等待机械臂收缩姿态回调；感知进程
继续保留，便于现场检查，按 `Ctrl-C` 后退出。

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

**集成限制：**mission 视觉目标和单树客户端已通过共享坐标换算客户端调用 planner，
当前夹爪 HOLD，仅到目标点；方向/区域/扫码姿态/收臂仍用旧接口，正式启动脚本也尚未启动 planner。
串口桥跨类型轨迹装载顺序未验证，不能把上述命令或任务脚本视为完整采摘闭环已验收。

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
Piper 可执行文件、模型和 aplay 路径以 `src/fruit_mission/config/fruit_mission.yaml` 的 `tts` 为准。
已生成音频时可单独播放（只产生声音，不启动任务或机械臂）：
```bash
cd /home/rc4/fruit_ws
aplay tmp/fruit_mission/tts_latest.wav
```
当前播报实现调用 aplay 播放 WAV，不提供旧脚本的 `--default` 或 `ALSA_DEVICE` 包装接口。
