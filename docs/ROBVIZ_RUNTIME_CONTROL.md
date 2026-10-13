# RobViz 运行任务控制

RobViz 使用两个常驻 HTTP 服务：

| 服务 | 端口 | 职责 |
| --- | ---: | --- |
| `robviz.service` | 8000 | 静态页面、航点文件和配置文件读写 |
| `fruit-runtime.service` | 8001 | 按需任务的状态、启动和停止 |

`foxglove_bridge.service` 继续独立监听 8765，负责浏览器与 ROS 2 的运行时数据交互。
运行管理服务不启动 Foxglove，也不执行任意浏览器提交的 shell 命令。

## 任务配置

任务登记在
`src/fruit_robviz_bridge/config/runtime_tasks.yaml`。
任务可以是一个启动脚本，也可以是依赖其他任务的任务组。

当前配置包含：

- `arm`：复用常驻 Foxglove，并按需启动串口桥和 arm planner。
- `perception`：按需启动 `start_fruit_perception.sh`。
- `navigation`：按需启动 `start_navigation.sh`。
- `mission`：按需启动 `start_fruit_mission.sh`。
- `waypoint_recording`：按需启动 `waypoints_maker.sh`。

任务命令和就绪检查属于机器人本机配置。浏览器只能选择任务 ID，不能传入命令。

## HTTP API

```text
GET  /health
GET  /api/runtime/tasks
GET  /api/runtime/tasks/{task_id}
POST /api/runtime/tasks/{task_id}/start
POST /api/runtime/tasks/{task_id}/stop
```

启动会先检查任务是否已经就绪，再启动缺失任务。
任务启动在独立进程组中运行，日志写入 `log/runtime-control/`。

停止 `arm` 时，管理器先调用 planner 的 STOP 服务，再停止 planner 和串口桥进程。
停止响应表示停止请求已经发布或进程已经退出，不代表硬件已经确认停稳。

## 命令行调用

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 run fruit_robviz_bridge fruit-runtime status
ros2 run fruit_robviz_bridge fruit-runtime start arm
ros2 run fruit_robviz_bridge fruit-runtime stop arm
```

命令行客户端和网页使用同一个 HTTP API。
运行管理服务本身由 `fruit-runtime.service` 开机启动。

## 脚本控制插件

RobViz 的「脚本控制」页复用端口 8001 的运行服务。
左侧显示七个脚本卡片，右侧只显示当前启动的脚本日志。
页面使用主页的间隙、圆角和主题变量，分隔条可拖动缩放。
参数选择和布局保存在当前机器的插件状态中。

支持 `start_fruit_mission.sh`、`start_tele.sh`、`waypoints_maker.sh`、
`test_camera_arm_target.sh`、`start_fruit_perception.sh`、`test_voice.sh` 和 `test_qr.sh`。
卡片从服务配置读取参数及可选值，不允许提交 shell 命令。
Mission 支持 flow、open-loop 和 debug。
open-loop 提供 `A`、`C`、`AC`、`none` 四个选项，默认 `A`。
`A` 或 `C` 只让对应区域开环；`AC` 让两区开环；`none` 让两区闭环。B 区保持原流程。
感知支持 zone、debug、成熟度模型、性能模式、日志等级和仿真时间。
相机机械臂测试必须选择 zone 和 left/right，支持 debug。
遥控和打点支持 debug。打点的 teleop 已默认开启。
语音和 QR 测试不提供参数。

顶部只有一个 Kill 控件，点击后立即执行 `kill_fruit_processes.sh`。
每次启动同样先执行 Kill，清理失败或超过 10 秒时禁止启动新脚本。
清理范围包括机器人任务、图像桥，以及测试脚本及其子进程。
独立运行的 RobViz 页面服务、runtime 服务和 Foxglove 服务不属于清理目标。
**Kill 是进程清理，不是硬件急停，也不证明硬件已经停稳。**

```text
GET  /api/runtime/scripts
GET  /api/runtime/scripts/log?run_id={run_id}&cursor={byte_offset}
POST /api/runtime/scripts/{task_id}/start  {"parameters": {...}}
POST /api/runtime/scripts/kill             {}
```

写请求只接受 `application/json`，最大 16 KiB。
服务端检查参数白名单、必填值及 Origin。
无 Origin 的本机命令行客户端仍可使用 API。
浏览器 Origin 必须与运行服务同主机，页面端口必须为 8000。

启动返回进程状态，不代表相机、ROS 节点或硬件已经就绪。
脚本使用 `DISPLAY=:0` 和当前用户的 `/run/user/{uid}` 会话环境，支持本机窗口和音频。
日志保留 ANSI 颜色和回车更新，使用只读 xterm 显示。
每次日志响应最多 64 KiB，返回页面时最多恢复最近 256 KiB。
cursor 使用字节偏移，UTF-8 字符不会在读取边界拆坏。
脚本退出后保留日志和退出码，新一次启动只显示新一次日志。
切换插件或关闭网页不会停止脚本。
运行服务重启会丢失内存中的当前任务引用，页面不会自动重启任务。

### 离线通信检查

以下检查只使用临时脚本，不连接 ROS 或硬件：

```bash
PYTHONPATH=src/fruit_robviz_bridge /usr/bin/python3 -m unittest discover \
  -s src/fruit_robviz_bridge/test -p test_runtime_protocol.py -v
bash -n kill_fruit_processes.sh
```

更新后构建 `fruit_robviz_bridge` 和 RobViz 前端。
在机器人任务已经停止时，执行 `sudo systemctl restart fruit-runtime.service` 加载新 API。
