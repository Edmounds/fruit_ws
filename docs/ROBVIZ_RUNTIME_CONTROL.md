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
