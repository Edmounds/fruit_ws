# 项目文档导航

`AGENTS.md` 是开发约束，不能替代架构、现场操作和接口参考。以下文档继续保留；涉及数值以源码配置为准，比赛规则以原文件和裁判解释为准。

## 当前参考

| 文档 | 用途 |
| --- | --- |
| [RULE.md](RULE.md)、[原规则 PDF](2026水果采摘.pdf) | 比赛约束与 A/B/C 当前范围 |
| [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) | 模块边界、数据流、任务顺序及未完成集成 |
| [ODOMETRY_NAVIGATION.md](ODOMETRY_NAVIGATION.md) | Point-LIO、TF、轴向导航和启动入口 |
| [WAYPOINT_RECORDING.md](WAYPOINT_RECORDING.md) | Web 录点、航点编辑、C 区拓扑与 yaw |
| [PERCEPTION_MODE_COMMANDS.md](PERCEPTION_MODE_COMMANDS.md) | 单相机常开与观察窗口的 ROS CLI 操作 |
| [CAMERA_TUNING.md](CAMERA_TUNING.md) | 单 camera 的物种检测 GUI 与参数持久化 |
| [DATASET_PIPELINE.md](DATASET_PIPELINE.md) | 当前图像采集工具与训练环境边界 |
| [LOWER_CONTROLLER_COMMANDS.md](LOWER_CONTROLLER_COMMANDS.md) | 串口消息契约、状态与通信限制；不是绕过 planner 的操作指南 |
| [VOICE_FLOW.md](VOICE_FLOW.md) | 当前代码的播报文本与时序 |
| [AUDIO_VOLUME.md](AUDIO_VOLUME.md) | 本机蓝牙音响与用户级服务排查 |

## 包级权威参考

- [机械臂 API](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md)：上位机机械臂控制必须使用此接口。
- [机械臂启动与标定](../src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md)。
- [camera 到机械臂的坐标换算](../src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md)。
- [两阶段模型运行与性能记录](../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md)。

## 历史与删除项

- [archive/PERCEPTION_MODEL_UPDATE_PLAN.md](archive/PERCEPTION_MODEL_UPDATE_PLAN.md)：已实施的模型接入计划，保留决策和性能记录。
- [archive/SINGLE_CAMERA_MIGRATION_GEMINI_PLAN.md](archive/SINGLE_CAMERA_MIGRATION_GEMINI_PLAN.md)：已实施的单相机迁移计划，保留验收边界。
- 两份计划均为历史记录，不应再次作为执行指令；历史硬件未验收项不因归档而变成已验证。
- 原 `CALIBRATION.md` 仅包含已删除的左右 USB 相机标定命令，已删除。当前 camera 到夹爪外参见包级坐标换算文档。
- 原数据集清洗模块、配置及 requirements 已不在工作区，已移除不可执行的流水线命令；不把旧 12 类编号当作当前两阶段模型标签契约。

**现状提醒：**视觉目标已接入 planner，但观察/收臂等旧接口尚未迁移，轨迹跨消息 ACK 顺序也未验证。服务目标到位且夹爪 HOLD 不等于夹取、脱柄或入仓完成。
