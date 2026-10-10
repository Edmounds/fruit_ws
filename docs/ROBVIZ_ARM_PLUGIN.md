# RobViz 机械臂插件

前端位于 `/home/rc4/robviz/src/plugins/fruit-arm/`。
完整设计见 [ROBVIZ_ARM_MODULE_PLAN.md](ROBVIZ_ARM_MODULE_PLAN.md)。

插件提供二维 TCP 编辑、三维离散轨迹播放、预设编辑、示教、单姿态执行和累计 jog。
前端通过原有 planner 服务控制机械臂。
HTTP 服务只提供文件编辑和纯数学计算，不执行运动。
不修改下位机协议，不创建第二套机械臂控制器。

## 配置与接口

`src/fruit_robviz_bridge/config/waypoint_editor.yaml` 的 `arm_config_path` 指定唯一可编辑文件。
客户端不能提交任意文件路径。
默认文件为源码 `src/fruit_arm_planner/config/arm_presets.yaml`。

| HTTP 端点 | 输入 / 结果 |
| --- | --- |
| `GET /api/arm/document` | 文件路径、原始字节版本、语义版本、配置、预设版本 |
| `PUT /api/arm/presets` | `version`、完整 `presets`，返回新文档、备份路径 |
| `POST /api/arm/pose` | `configuration`，可选 `frame`、四项 `target`，返回配置版本、角度、六个 FK 节点和 TCP |
| `POST /api/arm/preview` | `configuration`、四项 `start_hw_rad`，以及 `preset` 或单目标，返回实际点流 |

`configuration` 是四个配置段的对象，或 planner 快照中的 YAML 字符串。
单目标包含 `frame`、`target`、`duration_ms`、`gripper` 和 `linear`。
编辑坐标类型为 `cartesian`、`joints`、`link_angles`。
姿态计算另外支持 `hardware_rad` 和 `model_joints`。
模型角直接进入 FK，不再次逆映射。

预览点包含硬件弧度/角度、模型弧度、连杆角度、TCP、节点、时间、夹爪动作和途经点标记。
所有插值点和驻留点计入 `2..64` 点限制。
大时长或工序在生成点流前检查容量。
不可达、越限、非法字段和容量超限返回失败。
预览起点不改变实机起点。正式执行仍冻结当时的新鲜硬件反馈。

planner 的 `/arm_planner_node/get_configuration` 返回实际内存配置。
前端只读该服务，再把快照交给 HTTP 数学接口。
文件字节哈希用于保存冲突，语义哈希用于配置比较。
注释和 `1`/`1.0` 不改变语义版本。
重载只更新预设与快照版本，其他配置仍须安全停稳后重启。

## 文件保存

保存只修改 `presets`。
保持其他配置段、未知字段和注释。
目标数列统一输出浮点数，满足 ROS 参数数组同类型要求。
保存先验证基线和 YAML 语义，再创建备份并原子替换。
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
它检查 FK、共享轨迹、容量、输入、注释、备份和版本冲突。

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
页面默认仅预览，不保存启用状态或待执行命令。
普通运动等待执行结果，不受网页原有 10 秒等待限制。
断连后的执行结果未知，不自动重发。
STOP/ESTOP 的返回不表示硬件已停稳。
预览不提供碰撞检测或硬件加载确认。
Header/Point 传输顺序和固件保护仍须实机验收。
