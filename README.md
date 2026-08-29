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

## 下位机直连交互脚本

这个脚本直接打开串口与下位机通信，不启动 ROS，也不启动
`auto_serial_bridge`。启动前先确认没有其他进程占用 `/dev/stm32`。

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 /home/rc4/fruit_ws/lower_controller_console.py
```


默认串口和波特率从
`src/auto_serial_bridge/config/protocol.yaml` 读取，也可以覆盖：

```bash
/usr/bin/python3 /home/rc4/fruit_ws/lower_controller_console.py \
  --port /dev/stm32 --baudrate 115200
```

进入 `fruit> ` 提示符后使用以下命令：

| 命令 | 发送状态 |
| --- | --- |
| `grip` | 夹爪抓取：`task_id=0, task_status=0` |
| `open` | 夹爪打开：`task_id=0, task_status=1` |
| `gimbal-down` | 摄像头云台向下：`task_id=1, task_status=0` |
| `gimbal-up` | 摄像头云台向上：`task_id=1, task_status=1` |
| `left` | 左侧观察位置：`task_id=2, task_status=0` |
| `right` | 右侧观察位置：`task_id=2, task_status=1` |
| `qr-pose` | 进入二维码识别姿态：`task_id=4, task_status=0` |
| `area A` | 设置当前区域为 A：`task_id=5, task_status=0` |
| `area B` | 设置当前区域为 B：`task_id=5, task_status=1` |
| `area C` | 设置当前区域为 C：`task_id=5, task_status=2` |
| `area D` | 设置当前区域为 D：`task_id=5, task_status=3` |
| `task <id> <status>` | 直接发送通用状态命令 |
| `arm <x> <y> <z> <z_type>` | 发送机械臂目标，坐标单位为米 |
| `vel <linear_x> <angular_z>` | 发送一次速度命令 |
| `stop` | 发送零速度命令 |
| `help` | 显示命令帮助 |
| `quit` | 退出脚本 |

脚本会自动发送心跳，并对状态命令和机械臂目标进行可靠重试。可靠帧收到的
`Ack` 只表示下位机已收到，动作是否完成要看回传的
`ExecutionCallback`：`executing` 表示执行中，`completed` 表示完成。
默认不显示心跳收发和心跳超时日志；需要调试心跳时加 `--show-heartbeat`。
`vel` 只发送一帧，不会持续刷新速度。

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
