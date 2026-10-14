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
- `mission_acb`：按需启动 `start_fruit_mission_acb.sh`，使用固定 ACB 模式。
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
左侧显示八个脚本卡片，支持多个脚本并发启动与单独停止。右侧日志面板通过顶部 Tab 标签栏切换各脚本日志。
页面使用主页的间隙、圆角和主题变量，分隔条可拖动缩放。
参数选择和布局保存在当前机器的插件状态中。

支持 `start_fruit_mission.sh`、`start_fruit_mission_acb.sh`、`start_tele.sh`、`waypoints_maker.sh`、
`test_camera_arm_target.sh`、`start_fruit_perception.sh`、`test_voice.sh` 和 `test_qr.sh`。
卡片从服务配置读取参数及可选值，不允许提交 shell 命令。
每张卡片提供独立的「启动」与「停止」按钮：
- 未运行时「启动」可用、「停止」禁用。
- 运行中「启动」禁用、「停止」可用。
- 点击「停止」后进入过渡态，发送 SIGINT 信号；超时 5 秒未退出升级为 SIGTERM。
- SIGTERM 后最多再等待 5 秒。仍未退出时返回 `409`，保留进程跟踪以便再次停止。
- 停止检查整个所属进程组，不以组长退出作为清理完成。
- 停止前记录嵌套子进程组，覆盖相机、录包和单点调试启动的独立会话。
- 仍有所属进程或工作线程存活时保持 `RUNNING`，拒绝重新启动。纯僵尸进程不视为运行。
- 单独停止不调用全局 Kill，不清理其他卡片或复用的外部节点。
- 同一脚本运行中再次启动返回 `409`，不会停止或替换原进程。
- 已退出的脚本保留退出码显示。
- 点击左侧脚本卡片可直接在右侧激活或打开对应的日志 Tab。

顶部保留全局 Kill 控件，点击后立即执行 `kill_fruit_processes.sh`。
清理范围包括机器人任务、图像桥，以及测试脚本及其子进程。
独立运行的 RobViz 页面服务、runtime 服务和 Foxglove 服务不属于清理目标。
**Kill 是进程清理，不是硬件急停，也不证明硬件已经停稳。**
Kill 后只移除已确认退出的进程跟踪。仍存活的脚本继续支持单独停止。

```text
GET  /api/runtime/scripts
GET  /api/runtime/scripts/log?script_id={script_id}&run_id={run_id}&cursor={byte_offset}
POST /api/runtime/scripts/{task_id}/start  {"parameters": {...}}
POST /api/runtime/scripts/{task_id}/stop   {}
POST /api/runtime/scripts/kill             {}
```

写请求只接受 `application/json`，最大 16 KiB。
服务端检查参数白名单、必填值及 Origin。
无 Origin 的本机命令行客户端仍可使用 API。
浏览器 Origin 必须与运行服务同主机，页面端口必须为 8000。

启动返回进程状态，不代表相机、ROS 节点或硬件已经就绪。
正式比赛的 `mission` 和 `mission_acb` 脚本支持 `phase=prepare|localize|all`。
`prepare` 在场外启动相机、感知、二维码、机械臂、雷达、loam 和导航，并执行一次 Home。
`localize` 补启动 Point-LIO。每次初始化最多等待 10 秒，失败后重启 Point-LIO 与 `loam_interface` 一次。
第二次失败会报错，不启动 mission。定位成功后自动进入所选路线。
参数和航点文件在准备阶段确定，定位阶段不接受改路线或改参数。
CLI 显式指定 `--flow` 时，只能定位同流程的准备会话。
`start_fruit_mission_acb.sh --phase localize` 拒绝已准备的 ABC 会话。
ABC 脚本不传 `--flow` 的 `--phase localize` 沿用当前准备参数。
实现复用 Trigger 服务：通用入口为 `/fruit_competition/start_localization`，
显式流程入口为 `/fruit_competition/start_localization/{flow}`。正式比赛页仍使用通用入口。
脚本卡片使用 HTTP `/api/runtime/competition/localize/{flow}` 校验流程，拒绝不匹配的准备会话。
该接口仍只接受空 JSON，流程标识不修改任何准备参数，并通过 CLI 显式流程入口再次校验。
准备阶段 Home 服务结果等待最多 10 秒，超时保留客户端原有 STOP 和失败处理。
runner 在 ROS context 关闭后仍清理子进程，并写入停止状态。
清理未确认时写入 `ERROR`，不会伪报 `STOPPED`。STOP 未发送也会明确告警。
准备栈、Point-LIO 和 loam 的输出继续分类到 `arm_planner.log`、`perception.log` 和 `navigation.log`。
刷新或切换页面不会停止本轮。比赛会话锁阻止 ABC 和 ACB 同时拥有硬件。

「正式比赛」的流程下拉框显示 `ABC` 和 `ACB`，并保留单区调试选择。
ABC 使用 `mission` 任务，`flow=full`。ACB 使用 `mission_acb` 任务和独立 ACB 启动脚本。
ACB 固定 A 去程开环、A 回程及 C 闭环，因此页面禁用开环选择。
ACB 卡片只提供 `phase` 和 `debug`，不接受 `flow` 或 `open_loop`。
两条路线分别使用 `odom_waypoints.yaml` 和 `odom_waypoints_acb.yaml`。
脚本使用 `DISPLAY=:0` 和当前用户的 `/run/user/{uid}` 会话环境，支持本机窗口和音频。
日志保留 ANSI 颜色和回车更新，使用只读 xterm 显示。
右侧日志面板为单终端实例，切换 Tab 时重置并按需拉取目标脚本最近尾部数据（最多 256 KiB）。
切换 Tab 后丢弃旧日志请求的响应，避免写入错误日志或游标。
页面持续刷新全部脚本状态，不限于当前日志 Tab。
停止脚本不会改变当前日志选择，也不会重新打开已关闭的退出标签。
每次日志响应最多 64 KiB。
cursor 使用字节偏移，UTF-8 字符不会在读取边界拆坏。
脚本退出后保留日志和退出码，已退出的 Tab 支持点击关闭。
切换插件或关闭网页不会停止脚本。
运行服务重启会丢失内存中的当前任务引用，页面不会自动重启任务。

正式比赛页面使用以下状态接口：

```text
GET  /api/runtime/competition/status
POST /api/runtime/competition/localize {}
POST /api/runtime/competition/localize/{flow} {}
```

状态接口只读。它显示感知、机械臂、定位导航和 mission 四组健康状态，并读取 `/fruit_mission/status` 的当前动作与按果蔬 type 分类的完成、失败、跳过统计。连续数据超过 3 秒未更新会显示无数据。健康状态不阻塞比赛，也不自动重启模块。相机页面记忆标注 RGB、原始 RGB 或深度选择。

### 离线通信检查

以下检查只使用临时脚本，不连接 ROS 或硬件：

```bash
PYTHONPATH=src/fruit_robviz_bridge /usr/bin/python3 -m unittest discover \
  -s src/fruit_robviz_bridge/test -p test_runtime_protocol.py -v
PYTHONPATH=src/fruit_robviz_bridge /usr/bin/python3 -m unittest discover \
  -s src/fruit_robviz_bridge/test -p test_process_groups.py -v
bash -n start_fruit_mission.sh start_fruit_mission_acb.sh kill_fruit_processes.sh
```

八张卡片的独立停止检查使用同名临时程序，不运行真实机器人脚本。
假音频播放器检查不播放音频。runner 协议检查使用 localhost-only DDS domain 201，
仅创建空节点和定位 Trigger 端点，不调用机械臂服务、不发布导航目标。

```bash
source scripts/setup_fruit_runtime.bash
PYTHONPATH=src/fruit_mission:src/fruit_robviz_bridge:$PYTHONPATH \
  /usr/bin/python3 -m unittest discover -s src/fruit_mission/test \
  -p test_competition_runner_protocol.py -v
```

更新后构建 `fruit_robviz_bridge` 和 RobViz 前端。
在机器人任务已经停止时，执行 `sudo systemctl restart fruit-runtime.service` 加载新 API。
