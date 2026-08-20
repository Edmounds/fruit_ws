## drvier 
```bash
git clone git@github.com:realsenseai/realsense-ros.git
git clone git@github.com:ConQU2026/livox_ros_driver2.git
```


### 输出水果检测结果
```bash
ros2 topic echo /fruit_perception/decision
```

## 下位机直连交互脚本

这个脚本直接打开串口与下位机通信，不启动 ROS，也不启动
`auto_serial_bridge`。启动前先确认没有其他进程占用 `/dev/stm32`。

```bash
source /home/rc4/fruit_ws/setup_fruit_runtime.bash
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
