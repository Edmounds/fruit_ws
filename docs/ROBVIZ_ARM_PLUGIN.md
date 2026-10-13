# RobViz 机械臂插件

前端位于 `/home/rc4/robviz/src/plugins/fruit-arm/`。
完整设计见 [ROBVIZ_ARM_MODULE_PLAN.md](ROBVIZ_ARM_MODULE_PLAN.md)。

插件提供 TCP/原始电机角直接控制、预设编辑、示教追加和累计 Jog。
前端通过原有 planner 服务控制机械臂。
HTTP 服务只提供文件编辑和纯数学计算，不执行运动。
不修改下位机协议，不创建第二套机械臂控制器。

机械臂节点的按需启动和停止由常驻 `fruit-runtime.service` 管理。
网页通过端口 8001 的通用运行任务 API 控制 `arm` 任务。
详细接口见 [RobViz 运行任务控制](ROBVIZ_RUNTIME_CONTROL.md)。

页面不提供 2D/3D 机械臂示意、路径预览、时间轴或模拟播放。TCP 目标直接调用
`/arm_planner_node/move_cartesian`，原始硬件角直接调用
`/arm_planner_node/move_joints`。TCP 可选择 `linear_path`，关节角固定使用 PTP。

## 配置与接口

`src/fruit_robviz_bridge/config/robviz_web.yaml` 的 `arm_config_path` 指定唯一可编辑文件。
客户端不能提交任意文件路径。
默认文件为源码 `src/fruit_arm_planner/config/arm_presets.yaml`。

| HTTP 端点 | 输入 / 结果 |
| --- | --- |
| `GET /api/arm/document` | 文件路径、原始字节版本、语义版本、配置、预设版本、`preset_order` |
| `PUT /api/arm/presets` | `version`、完整 `presets`、可选 `preset_order`，返回新文档、备份路径 |
| `POST /api/arm/pose` | `configuration`，可选 `frame`、四项 `target`，返回配置版本、角度、六个 FK 节点和 TCP |

`configuration` 是四个配置段的对象，或 planner 快照中的 YAML 字符串。
单目标包含 `frame`、`target`、`duration_ms`、`gripper` 和 `linear`。
编辑坐标类型为 `cartesian`、`joints`、`link_angles`。
姿态计算另外支持 `hardware_rad` 和 `model_joints`。
模型角直接进入 FK，不再次逆映射。

`POST /api/arm/pose` 用于姿态合法性检查和坐标转换。它不执行运动，也不替代 planner 的正式规划和执行结果。

planner 的 `/arm_planner_node/get_configuration` 返回实际内存配置。
前端只读该服务，再把快照交给 HTTP 姿态接口或 planner 正式服务。
文件字节哈希用于保存冲突，语义哈希用于配置比较。
注释和 `1`/`1.0` 不改变语义版本。
重载只更新预设与快照版本，其他配置仍须安全停稳后重启。

## 文件保存

保存只修改 `presets`。
保持其他配置段、未知字段和注释。
携带 `preset_order` 时，移动原有路点集文本块以调整 YAML 内部排列；集合内部注释随集合移动，`presets` 区域开头的说明保留在原位。
遇到别名/锚点、重复键或非标准块映射等不安全结构时拒绝保存并保持原文件。
旧客户端不携带 `preset_order` 时保持原有编辑兼容路径。
目标数列统一输出浮点数，满足 ROS 参数数组同类型要求。
保存先验证基线和 YAML 语义，再创建备份并原子替换。
纯排序仅改变字节版本，不改变语义版本。
保存期间检测到外部修改时拒绝覆盖。
旧基线返回 HTTP `409`。没有自动重试、重载或执行。
其他编辑工具仍须避免同时写入同一文件。
网络失败后先核对文件，不能假定保存未发生。

## 构建与离线检查

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_interfaces fruit_arm_planner fruit_robviz_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source scripts/setup_fruit_runtime.bash
ros2 run fruit_robviz_bridge check_arm_preview
```

离线检查使用临时 YAML，不连接 ROS 或硬件。
它检查 FK、输入、注释、备份和版本冲突。

```bash
cd /home/rc4/robviz
npm run build
```

构建不会更新已运行进程。
确认现场安全后，由操作者重启已有 planner 和 HTTP 服务。
不要重复启动串口桥或自动重载真实预设。
默认 HTTP 服务发布 `/home/rc4/robviz/dist`。
从 `http://<robot>:8000` 打开页面，并选择同一机器人。

## 验收边界

启用控制前，先停止 mission 和其他运动客户端。
页面默认不启用实机控制，不保存启用状态或待执行命令。
点击“启用实机控制”直接开启调试操作，无确认弹窗，不自动运动。
网页不检查反馈时效（年龄）和 IDLE 状态，时效与硬件状态由 planner 检查。
页面显示为最后接收到的有效反馈，不代表当前物理姿态。
闲置失焦或页面隐藏保持启用状态，但退出 Jog 并清理待发送输入；在途请求取消，已发送 Jog 请求 STOP。
离页、断连或切换机器仍撤销启用状态。
普通运动等待执行结果，不受网页原有 10 秒等待限制。
断连后的执行结果未知，不自动重发。
STOP/ESTOP 的返回仅表示请求已发布，硬件停稳未由网页确认。
planner 负责正式轨迹检查、硬件加载和执行结果确认。
Header/Point 传输顺序和固件保护仍须实机验收。

页面桌面端使用 `react-resizable-panels` 调整预设、控制、遥测和诊断区域。
布局比例保存于 RobViz 当前会话的 `fruit-arm` 插件状态，不写入 `arm_presets.yaml`。
