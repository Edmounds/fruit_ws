## drvier 
```bash
git clone git@github.com:realsenseai/realsense-ros.git
git clone git@github.com:ConQU2026/livox_ros_driver2.git
```

## ROS 运行环境

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
fuser -v /dev/left_cam /dev/right_cam /dev/stm32
```

清理脚本覆盖任务启动器、ROS launch/run、`test_camera_detector.py` 创建的
`test_camera_detector_*_container`、`lower_controller_console.py`、录包进程和调试监视器；
它会先恢复被 `Ctrl-Z` 挂起的目标，再依次发送 `SIGINT`、`SIGTERM` 和 `SIGKILL`，成功后
停止 ROS 2 daemon 以清除过期图缓存。RViz、`foxglove_bridge.service` 和
`waypoint-editor.service` 是独立工具，不由该脚本停止，也不占用三台摄像头或下位机串口。

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
./test_d435i_arm_target.sh --zone B --right --apple
./test_d435i_arm_target.sh --zone B --left --pear --debug
```

`--apple` 和 `--pear` 只能用于 B 区且必须二选一。脚本等待感知节点完成 50 帧
整树统计，再按正式 mission 的规则比较整树品种和期望品种，只向下位机发送品种一致的
成熟目标。没有目标、品种不一致或只有未成熟目标时正常跳过，不发送 `ArmTarget`。
单树抓取、跳过或执行失败后都会发送 `task_id=7` 并等待机械臂收缩姿态回调；感知进程
继续保留，便于现场检查，按 `Ctrl-C` 后退出。

## 下位机交互调试控制台

控制台通过 `auto_serial_bridge` 发布的 ROS 2 标准话题（`/fruit_hardware_bridge/arm/*` 与 `/cmd_vel`）与下位机进行通信，依托桥接底层的 CRC 校验、自动心跳与可靠重传保障通信健壮性。

控制台会自动检测系统中是否已运行 `auto_serial_bridge`：
- 若已运行则直接复用；
- 若未运行则自动在后台拉起 `serial_node`，默认以 DEBUG 模式流式在终端彩色输出底层通信与心跳调试信息；
- 串口设备与波特率直接从 `src/auto_serial_bridge/config/protocol.yaml` 严格读取，并在启动时对所需 5 条关键消息进行结构校验；
- 退出控制台时自动平滑停机并清理释放串口。

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 /home/rc4/fruit_ws/lower_controller_console.py
```

支持参数：
- `--log-level`：设置 bridge 日志级别，支持 `debug`（默认）、`info`、`warn`、`error`。
- `--yes` / `-y`：跳过机械臂动作前的 `[y/N]` 二次确认提示。
- `--no-spawn`：禁用后台自动拉起 bridge（仅在外部已启动 bridge 时连接）。
- `--no-color`：禁用终端 ANSI 彩色输出。

进入 `fruit> ` 提示符后使用以下命令：

| 命令 | 功能说明 |
| --- | --- |
| `home` | 机械臂回到初始安全复位姿态：`[0°, 110°, 50°, 0°]` |
| `retract` | 机械臂收拢到避障收纳姿态：`[0°, 110°, 150°, 0°]` |
| `grip` | 夹爪闭合（`gripper_action=2`） |
| `open` | 夹爪张开（`gripper_action=1`） |
| `traj <q1> <q2> <q3> <q4> [act] [ms]` | 自定义发送 4 关节轨迹（角度 deg，act: 0=保持/1=开/2=闭，默认 3000ms） |
| `mode <0\|1\|2>` | 发送控制模式（0: 轨迹执行, 1: 平滑停止, 2: 硬件急停） |
| `vel <linear_x> <angular_z>` | 发送一次底盘速度命令（线速度 m/s，角速度 rad/s） |
| `stop` | 平滑停机：下发 `ArmControlMode(STOP)` 并将底盘速度置零 |
| `estop` | 紧急硬件停机：下发 `ArmControlMode(ESTOP)` 硬件锁死脉冲 |
| `status` | 打印当前下位机 20Hz `ArmStatus`（关节角度、速度、进度、状态与故障码） |
| `help` | 显示命令帮助 |
| `quit` | 退出脚本 |

安全与保护说明：
- **软限位拦截**：脚本端严格校验 4 关节软件限位（$Q_1 \in [-180^\circ, 180^\circ]$、$Q_2 \in [3^\circ, 180^\circ]$、$Q_3 \in [-190^\circ, -35^\circ]$、$Q_4 \in [-90^\circ, 92.4^\circ]$），超限指令直接拒绝下发。
- **20Hz 遥测过滤**：静默接收 `ArmStatus`，仅在任务状态改变（如 IDLE $\to$ RUNNING $\to$ COMPLETED）或故障码非零时回显提示，彻底避免终端刷屏；完整状态可输入 `status` 随时查看。
- **Ctrl-C 安全停止**：运行中首次按 `Ctrl-C` 立即下发平滑停机与底盘置零；2 秒内再次按 `Ctrl-C` 退出控制台。

## 录制调试 ROS 话题
```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
./record_rosbag.sh --full
```


录包保留所有非视觉话题。视觉数据只保留左右 USB 相机的原始图和标注图，以及
D435i 的彩色原图、对齐深度图和标注图。D435i 未对齐的
`/camera/camera/depth/image_rect_raw`、重复的 `/camera/camera/rgbd`，以及所有
`compressed`、`compressedDepth`、`theora` 派生话题均通过 rosbag 黑名单排除。

## 语音播报检查
```bash
cd /home/rc4/fruit_ws
./test_tts.sh --default
```
不带 `--default` 时，脚本默认使用配置的显式 USB ALSA 设备；也可以通过
`ALSA_DEVICE=... ./test_tts.sh` 临时指定播放设备。
