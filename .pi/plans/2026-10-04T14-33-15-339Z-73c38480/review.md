# Review

## 范围

审查范围限定为本计划新增的 `target_offset_m` 配置、校验、Base 坐标偏置数学函数、正式 Mission 与独立 `camera_arm_target_client` 的接入，以及对应文档和验证证据。启动前 baseline 已包含大量未提交改动；这些 baseline 内容未归因于本任务，也未作为本次问题报告依据。

已核对的关键路径：

- `src/fruit_mission/config/fruit_mission.yaml` 的 `target_offset_m` 配置。
- `src/fruit_mission/fruit_mission/config.py` 的 `validate_target_offset()`, `load_target_offset()`, `load_config()` 和 `validate_config()`。
- `src/fruit_mission/fruit_mission/camera_target_transform.py::apply_target_offset_mm()`。
- `src/fruit_mission/fruit_mission/camera_target_planner_client.py::CameraTargetPlannerClient.target_request()`。
- `MissionContext.__init__()` 与 `camera_target_request()`，以及 `actions.py` 中 A/B/C 的目标调用。
- `camera_arm_target_client.py`、`script_ros_clients.yaml` 和 `test_camera_arm_target.sh` 的共享配置入口。

## P0

无。

## P1

无。

## 修复步骤

无。当前没有需要修复的 P0/P1 问题。

## 验收

- 配置值为 `[0.0, 0.03, -0.03]` m；正式 Mission 通过 `load_config()` 校验，独立调试客户端通过同一 `fruit_mission.yaml` 路径加载并校验。
- `apply_target_offset_mm()` 在 Base 坐标中执行：X/Z 直接按米转毫米叠加，Y 依据转换后 Base Y 的正负自动取方向，不读取 `left/right` 参数。
- A、B、C 当前目标请求和 B 区一次观测冻结的多个目标都经共享入口；调试客户端也调用同一转换客户端，未发现第二套会提交机械臂请求的 D435i 目标转换路径。
- 配置校验拒绝缺失、错误长度、bool、NaN、Inf 和非数值输入；未增加人为绝对值上限。
- 已有验证证据：
  - 受影响 Python 文件 `compileall` 通过。
  - 受影响 Python 文件 `ament_flake8` 通过。
  - `bash -n test_camera_arm_target.sh` 通过。
  - `git diff --check` 通过。
  - `colcon build --base-paths src --packages-up-to fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF` 成功，9 个包完成。
  - 离线请求验证中，Base Y 为正时偏置差为 `(0.0000, +0.0300, -0.0300)` m，Base Y 为负时为 `(0.0000, -0.0300, -0.0300)` m。
  - `camera_arm_target_client --help` 与 `test_camera_arm_target.sh --help` 正常退出。

本 review 未执行硬件运动、ROS 运动服务或现场抓取验收。