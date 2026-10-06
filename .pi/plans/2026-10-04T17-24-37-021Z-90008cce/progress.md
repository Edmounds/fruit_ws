# 执行进度与验证证据

## 任务目标
解决运行 `./test_camera_arm_target.sh` 等 7 个入口脚本时终端被大量低价值日志刷屏的问题：
- 终端：默认仅保留关键生命周期事件、识别锁定目标基座坐标、动作结果、告警与错误；`--debug` 模式下额外展示原已开启包的解码文本，彻底移除 raw 十六进制数据；
- 文件：所有 TX/RX 解码、节点详细 Debug 日志、完整坐标变换链统一存放到 `/home/rc4/fruit_ws/log/runtime/<时间_入口_PID>/`，保留最近 20 次已结束运行目录并具备文件锁保护；
- Agent 指引：在 `AGENTS.md` 明确指引 AI 前往统一落盘目录查阅排查日志。

## 完成步骤

- [x] **步骤 1：基线冻结与隔离**
  - 检查父仓库及 `src/auto_serial_bridge` 独立 Git 仓库的基线状态，确认无损增量实施边界，保护已有未提交改动。
- [x] **步骤 2：实现共享落盘与终端分流辅助工具**
  - 新增 `scripts/runtime_log_filter.py`：实现 stdin 全量原样落盘至指定文件，终端严格过滤 raw 十六进制数据、高频巡检（CAMERA streams status=OK、HEALTH、count=0）、详细 launch 噪声；普通模式下滤除逐点解码与逐帧检测，`--debug` 模式下仅放行主 logger 解码；Traceback、异常及各类告警错误 100% 完整放行。
  - 新增 `scripts/fruit_runtime_logging.bash`：实现会话目录初始化、环境继承复用、基于 `.active.lock` 的活动锁保护、`latest` 软链接原子维护、最近 20 次已结束运行自动清理、`fruit_run_logged` 管道退出码无损透传（通过局部 errexit 状态保持）。
- [x] **步骤 3：修正 `auto_serial_bridge` 串口桥**
  - 修改 `src/auto_serial_bridge/src/serial_controller.cpp`：彻底删除 `[MCU TX RAW]` 与原始 payload 十六进制打印；`log_mcu_tx` 中将已开启包送主 logger，未开启包（ChassisVelocity 等）送 `serial_controller.detail` 子 logger；系统包（Heartbeat、Ack、Handshake）全部复用结构化解码。
  - 修改 `src/auto_serial_bridge/include/auto_serial_bridge/serial_controller.hpp`：移除 raw 打印 helper，保留兼容参数。
  - 修改 `src/auto_serial_bridge/scripts/codegen.py`：扩展 `describe_packet` 覆盖全部 TX、RX 与系统包；`_write_guarded_rx_debug_log` 针对 off 包（如 ArmStatus）生成至 `serial_controller.detail` 子 logger。
  - 修改 `src/auto_serial_bridge/config/protocol.yaml` 与 `README.md`：标注 `debug_raw_frame` 废弃兼容说明。
  - 调整 `src/auto_serial_bridge/test/test_serial_controller_helpers.cpp`：移除对已删 raw helper 的废弃断言。
- [x] **步骤 4：调整监视器与客户端输出级别**
  - 修改 `scripts/robot_debug_monitor.sh`：常态相机巡检数据只落盘；增加相机流由 STALE 恢复至 OK 时的单次显著通知 `CAMERA streams recovered`。
  - 修改 `src/fruit_mission/fruit_mission/camera_target_planner_client.py`：将冗长坐标变换链、初始化信息降为 DEBUG，`finish_motion` 中保留明确的完成/失败警告日志。
  - 修改 `src/fruit_mission/fruit_mission/camera_arm_target_client.py`：将窗口 UUID、配置路径、逐帧 detection 降为 DEBUG，保留高价值阶段、锁定坐标、请求目标与执行结果。
- [x] **步骤 5：接入相机、感知与任务入口**
  - 修改 `test_camera_arm_target.sh`：接入统一日志会话，子进程与客户端全量落盘至 `perception.log`、`arm_planner.log`、`client.log`；终端应用精简过滤；失败提示日志文件具体路径，不刷屏 40 行历史日志。
  - 修改 `start_fruit_perception.sh`：解析 `--debug`，接入统一日志会话，monitor 输出流式落盘与过滤。
  - 修改 `start_fruit_mission.sh`：使用统一会话目录，子进程全量写日志并过滤终端，失败仅打印退出码及日志定位路径。
  - 修改 `src/fruit_perception/launch/fruit_perception.launch.py` 与 `src/fruit_mission/launch/fruit_mission.launch.py`：更新参数描述与 `log_level` 透传。
- [x] **步骤 6：接入导航、遥控、录点与录包入口**
  - 修改 `start_navigation.sh`、`start_tele.sh`、`waypoints_maker.sh`、`record_rosbag.sh`：统一支持 `--debug`，接入统一会话日志目录管理。
  - 修改 `src/navigation/fruit_nav_bringup/launch/fruit_nav.launch.py` 与 `src/fruit_waypoint_recorder/launch/waypoint_recorder.launch.py`：增加 `log_level` 传递支持。
- [x] **步骤 7：同步文档与 AI Agent 指引**
  - 更新 `AGENTS.md`：新增《Runtime Logging and Diagnostics for AI Agents》小节，规定排查顺序（终端提示路径 -> latest 软链接 -> orchestrator.log -> 专用子进程日志 -> 依 window_id/trajectory_id 联查 -> 串口结构化解码；明确 raw 已移除）；
  - 更新根 `README.md` 与 `src/auto_serial_bridge/README.md`。
- [x] **步骤 8：构建与无运动离线验收**
  - 执行 `colcon build` 重新编译受影响的 16 个包，全量编译成功；
  - 检查感知库 ABI 依赖，确认无 `/usr/local/lib/libopencv*` 混用；
  - 运行 `tmp/check_runtime_logging.py` 离线测试（验证 raw 过滤、解码分流、错误与 traceback 放行、常态巡检过滤、退出码 7 透传、子会话继承、并发锁与 20 份保留清理），全部通过；
  - 运行 `auto_serial_bridge` 5 项选定协议单元测试，全量通过；
  - 运行 `git diff --check`，格式完全干净。

## 实际测试命令与结果证据

1. **离线过滤与日志会话管理器测试**
   - 命令：`/usr/bin/python3 tmp/check_runtime_logging.py`
   - 结果：`Ran 5 tests in 0.324s, OK`。
2. **`auto_serial_bridge` 选定 C++ 协议测试**
   - 命令：`colcon test --base-paths src --packages-select auto_serial_bridge --return-code-on-test-failure --ctest-args -R '^(test_packet_handler|test_checksum_algorithms|test_reliable_roundtrip|test_reliable_sender|test_serial_controller_helpers)$'`
   - 结果：`114 tests, 0 errors, 0 failures, 1 skipped`。
3. **工作区编译**
   - 命令：`colcon build --base-paths src --packages-up-to auto_serial_bridge fruit_mission fruit_perception fruit_qr_planner fruit_hardware_bridge fruit_nav_bringup joy_linux fruit_waypoint_recorder --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`
   - 结果：`Summary: 16 packages finished`，全部成功。
4. **感知库 ABI 依赖检查**
   - 命令：`ldd install/fruit_perception/lib/libfruit_perception_component.so | grep -i opencv`
   - 结果：仅链接 `/lib/x86_64-linux-gnu/libopencv_*.so.4.5d`，无外部冲突。
5. **Git Diff 检查**
   - 命令：`git diff --check -- AGENTS.md ...` 及 `git -C src/auto_serial_bridge diff --check`
   - 结果：输出为空，无任何空白或语法问题。

## 阻塞与人工交接事项
- **当前无阻塞**。
- **人工实机验收提醒**：实机运行命令（如 `./test_camera_arm_target.sh --model A --left`）会触发机械臂真实动作，需由操作者在现场确认机械臂与机器人周边物理安全后手动执行验收。可分别以普通模式和 `--debug` 模式执行，对比终端精炼效果并检查 `/home/rc4/fruit_ws/log/runtime/latest/` 下的完整日志。
