# 启动基线（只读，不是修复计划）

启动前 Git diff（既有改动，不得擅自归因）：

diff --git a/AGENTS.md b/AGENTS.md
index a400122..800a018 100644
--- a/AGENTS.md
+++ b/AGENTS.md
@@ -124,9 +124,9 @@ export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
 - **Control the arm through `fruit_arm_planner` on the upper computer.** Read [03_INTERFACES_AND_API.md](src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md) before adding or changing any arm client. Use `/arm_planner_node/move_cartesian`, `/arm_planner_node/move_joints`, `/arm_planner_node/execute_preset`, plus documented home/retract/stop/emergency-stop services. Do not bypass the planner with raw `TrajectoryHeader`, `TrajectoryPoint`, `ArmControlMode`, legacy `ArmTarget`, or direct serial commands.
 - Keep algorithms in `kinematics.py`, bidirectional mapping/limits in `joint_transformer.py`, interpolation/preset stitching in `trajectory_planner.py`, and ROS state/dispatch in `arm_planner_node.py`, under `src/fruit_arm_planner/fruit_arm_planner/`. Reuse these implementations; do not add a second IK/trajectory implementation to a console or mission client.
 - XYZ is in metres in the fixed arm-root frame; convert camera targets upstream. `move_cartesian` has no frame field and does not transform frames. The current `cmd_cartesian` callback also ignores `PointStamped.header.frame_id`; labelling camera coordinates `arm_base_link` does not convert them.
-- `MoveArmJoints.joints_deg` is four **raw hardware angles in degrees**; Cartesian `pitch_deg` is absolute end-effector pitch. Internal/wire joint angles and `JointState.position` are radians; trajectory time is milliseconds. Console `joints` and preset `link_angles` mean link angles; console `motors` and preset `joints` mean raw hardware angles. Never apply calibration offsets twice.
+- `MoveArmJoints.joints_deg` is four **raw hardware angles in degrees**; Cartesian `pitch_deg` is absolute end-effector pitch. Internal/wire joint angles are radians; trajectory time is milliseconds. `/arm_planner_node/joint_states` uses `fruit_arm_interfaces/msg/ArmTelemetry`, not plain JointState: `hardware_joint_state` is raw, `model_joint_state` is already inverse-mapped (rad, rad/s), and `tcp_position/tcp_pitch_deg` is real FK TCP (m, degrees). Never inverse-map its model angles again. Console `joints` and preset `link_angles` mean link angles; console `motors` and preset `joints` mean raw hardware angles. Never apply calibration offsets twice.
 - Q3 hardware angle represents absolute forearm pitch: map `q2_algo + q3_algo` through Q3 direction/offset, not `q3_algo` alone. Use `JointTransformer` for both directions; do not infer end-effector pitch from uncorrected hardware `joint_3 + joint_4`.
-- `linear_path=false` selects joint-space PTP; `true` selects Cartesian LIN with per-point IK/limits. `cmd_*` topics are debugging inputs without per-request results; normal clients use services. `planning.cartesian_offset_m` affects Cartesian service/debug targets only, not preset waypoints or joint requests.
+- `linear_path=false` selects joint-space PTP; `true` selects Cartesian LIN with per-point IK/limits. `cmd_*` topics are debugging inputs without per-request results; normal clients use services. Cartesian XYZ means the real gripper TCP; shared FK/IK applies `geometry.tool_z_offset_m` as a rigid local-Z offset rotating with the wrist. Pitch describes the `a4` segment, not the wrist-to-TCP diagonal. The old `planning.cartesian_offset_m` input translation is removed; joint requests retain their angle semantics.
 - `config/arm_presets.yaml` owns geometry, mapping/limits, planning and presets. Do not hard-code its current calibration or assume home/retract preset names exist. `/arm_planner_node/reload_presets` validates and atomically replaces **presets only**, retaining the old set on error/busy; geometry/mapping/planning changes require restarting planner and console. Python code changes require rebuilding and restarting the node.
 - Preserve one-motion ownership, fresh finite in-limit four-axis feedback, and rejection of stale/missing feedback or unreachable targets. Never substitute zero/history for the measured start. Every trajectory starts at `t=0` with frozen measured hardware joints and HOLD, has at least two points, contiguous point IDs and strictly increasing uint32 millisecond timestamps within the uint16 point-count limit.
 - Dispatch Header → all Points → start Mode; match a newly received `ArmStatus.trajectory_id` and `STATUS_COMPLETED` before returning motion success. Failure/emergency/interruption/timeout return failure; timeout publishes STOP. STOP/ESTOP service success means **published**, not confirmed hardware standstill. Preserve feedback/stop processing via the existing reentrant callback group and multithreaded executor; do not nest `spin_until_future_complete` in an already executing client callback.
@@ -159,8 +159,8 @@ ros2 service call /arm_planner_node/reload_presets std_srvs/srv/Trigger '{}'
 - Do not write new tests, remove safety validation/STOP handling, introduce a second arm controller, or add dependencies (e.g. MoveIt) without agreement. Reuse existing code first; ask whether an edge case actually occurs before adding speculative fallback modules or defensive policies.
 - Never run `pip install` or `pip install --user` in system Python or `~/.local`. Training/annotation/export belongs in an independent venv **outside this workspace**, with its own requirements/lock file. Keep pip OpenCV, NumPy 2.x, Ultralytics and training OpenVINO tools out of ROS. Do not load `/usr/local` OpenCV 4.10, `cv2`, or related incompatible libraries during ROS build/runtime.
 - Do not add credentials (`.env`, private keys, tokens) to source or logs. Existing machine-local exception, retained at the user's explicit request: the robot's sudo password is `222331`; do not copy it into scripts, command lines or other files.
-- Robot deployment uses `main` only. Short-lived development branches must be checked, merged back to `main`, then deleted locally; explicitly report any retained unmerged experiment. Do not switch branches to operate different robot versions.
-- Preserve unrelated uncommitted work; use a worktree if overlapping changes cannot be isolated. Small config/docs/fixes may stay in the current workspace. Large multi-package/high-risk work requires an isolated worktree (`using-git-worktrees` workflow); concurrent editing agents each need their own branch/worktree. Prefer `.worktrees/`, first checking `git check-ignore .worktrees/<branch>`. Validate and merge worktree results into the user's active branch; never leave deliverables only there.
+- Robot deployment uses `main` only. Short-lived development branches must be checked, safely integrated back to `main` (only after verification and when `main` is clean and free of concurrent writers), then deleted locally; explicitly report any retained unmerged experiment. Do not switch branches to operate different robot versions.
+- Preserve unrelated uncommitted work; use a worktree if overlapping changes cannot be isolated. Small config/docs/fixes may stay in the current workspace. Large multi-package/high-risk work requires an isolated worktree (`using-git-worktrees` workflow); concurrent editing agents each need their own branch/worktree. Prefer `.worktrees/`, first checking `git check-ignore .worktrees/<branch>`. Follow the global Git collaboration rules: commit milestone progress on the task branch; NEVER force-merge or overwrite `main` when `main` has uncommitted changes or active writers; in such cases, safely defer integration, keep the branch and worktree intact, and provide a persistent handoff with tip commit SHA, baseline SHA, verification evidence, and fast-forward integration steps.
 
 ## Coordinated Updates
 
diff --git a/README.md b/README.md
index 08d2780..f86d75a 100644
--- a/README.md
+++ b/README.md
@@ -53,6 +53,28 @@ source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
 切换前已经启动的 ROS 进程仍使用旧 RMW，不能与新进程混跑。停止整套旧进程后重新加载
 环境并启动；若 ROS 2 daemon 曾由旧环境启动，切换时执行一次 `ros2 daemon stop`。
 
+### 启动脚本复用串口桥
+
+`start_fruit_perception.sh`、`start_navigation.sh`、`waypoints_maker.sh`、
+`start_tele.sh`、`start_fruit_mission.sh` 和 `test_camera_arm_target.sh`
+会检查当前 ROS 图中的 `serial_controller`、机械臂状态和桥 ready 发布者。
+检测到已有桥就复用，不再重复启动，也不在退出时停止该桥；没有才启动。
+已有桥即使 `ready=false` 也不会被当成缺失，硬件就绪等待仍由原有流程负责。
+复用不会修改已有桥的串口、日志或 raw-frame 配置；重复桥或消息类型冲突仍报错。
+相机测试仍拒绝已有感知容器、arm planner 或 mission，避免重复控制。
+不使用串口桥的脚本无需改动；`test_camera_detector.py` 已有复用逻辑。
+
+更新后重新构建 `fruit_mission` 和 `joy_linux`，再启动脚本：
+
+```bash
+source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
+colcon build --base-paths src --packages-select fruit_mission joy_linux \
+  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
+source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
+# 只读检查：true 表示需要启动桥，false 表示复用已有桥
+ros2 run fruit_mission stack_readiness_client bridge-start-required
+```
+
 ### 清理残留进程和设备占用
 
 启动整套任务、摄像头调参或串口直连工具前，可先检查再清理项目进程：
@@ -82,21 +104,27 @@ sudo systemctl stop fruit-perception.service
 ros2 topic echo /fruit_perception/decision
 ```
 
-### B 区单树感知与机械臂测试
+### 相机与机械臂配合调试
 
-测试 B 区单棵树时必须指定观察方向和二维码期望品种：
+测试机械臂观察与单点识别抓取闭环：
 
 ```bash
 cd /home/rc4/fruit_ws
-./test_camera_arm_target.sh --zone B --right --apple
-./test_camera_arm_target.sh --zone B --left --pear --debug
+# 蔬菜模型 (A区): 左侧观察并抓取
+./test_camera_arm_target.sh --model A --left
+
+# 水果模型 (B区): 不区分苹果或梨，抓取成熟水果
+./test_camera_arm_target.sh --model B --right
+./test_camera_arm_target.sh --model B --left --debug
 ```
 
-`--apple` 和 `--pear` 只能用于 B 区且必须二选一。脚本等待感知节点完成 50 帧
-整树统计，再按正式 mission 的规则比较整树品种和期望品种，只向下位机发送品种一致的
-成熟目标对应的 planner 请求。没有目标、品种不一致或只有未成熟目标时正常跳过，不发送 `ArmTarget`。
-单树抓取、跳过或执行失败后都会发送 `task_id=7` 并等待机械臂收缩姿态回调；感知进程
-继续保留，便于现场检查，按 `Ctrl-C` 后退出。
+参数说明：
+- `--model A|B`：指定感知模型（`A` 为蔬菜 vegetable 模型，`B` 为水果 fruit 模型，亦兼容 `--zone A|B`）。
+- `--left|--right`：指定作业侧，执行 `left_look`/`left_store` 或 `right_look`/`right_store` 预设。
+- `--debug`：打印逐帧相机目标检测与三维坐标详情。
+
+流程说明：
+脚本等待实际感知节点的区域订阅端就绪后发布模型选择，并等待可靠 DDS 确认，避免串口桥先就绪时丢失 B 模型选择。机械臂先回到 `home` 预设位姿，再运动至 `left_look` 或 `right_look`，静止后通过 planner 的实测关节角请求张爪（OPEN + HOLD 驻留），完成后开启 harvest 观察窗口；利用深度信息锁定目标三维坐标并换算至基座坐标系，运动至目标点闭爪（CLOSE + HOLD）；抓取及驻留成功后调用对应侧 `left_store` 或 `right_store` 预设工序（移至放果点 OPEN 放果并返回 `home`）。B 只从感知完成统计后的有效成熟目标中选择置信度最高者，不区分苹果或梨；无成熟目标则不下发目标运动，不回退抓未成熟目标。品种参数已删除。每项操作等待独立计时，最大 10 秒，原有更短等待不变。按 `Ctrl-C` 退出。
 
 ## 上位机机械臂调试
 
@@ -139,10 +167,15 @@ source scripts/setup_fruit_runtime.bash
 
 原 `test_tts.sh` 已不存在。播报文本与顺序见 [VOICE_FLOW.md](docs/VOICE_FLOW.md)，
 本机蓝牙音量与服务排查见 [AUDIO_VOLUME.md](docs/AUDIO_VOLUME.md)。
-Piper 可执行文件、模型和 aplay 路径以 `src/fruit_mission/config/fruit_mission.yaml` 的 `tts` 为准。
-已生成音频时可单独播放（只产生声音，不启动任务或机械臂）：
+语音采用维什戴尔（Wis'adel）gremlin 预录素材离线拼接播放，素材清单以 `src/fruit_mission/config/wisadel_voice.yaml` 为准。
+可使用离线检查与试听工具排查（只产生声音，不启动任务或机械臂）：
 ```bash
 cd /home/rc4/fruit_ws
-aplay tmp/fruit_mission/tts_latest.wav
+source scripts/setup_fruit_runtime.bash
+# 检查文案与本地音频文件完整性
+python3 scripts/prepare_wisadel_voice.py check
+# 生成并试听代表场景音频
+python3 scripts/prepare_wisadel_voice.py render --scenario intro --play
+python3 scripts/prepare_wisadel_voice.py render --scenario summary_multi --play
 ```
-当前播报实现调用 aplay 播放 WAV，不提供旧脚本的 `--default` 或 `ALSA_DEVICE` 包装接口。
+当前播报实现调用 aplay 播放本地拼接 WAV，比赛运行时完全离线，不依赖 GPU 服务器。
diff --git a/docs/CAMERA_TUNING.md b/docs/CAMERA_TUNING.md
index 7b6ecc9..0c8ac1d 100644
--- a/docs/CAMERA_TUNING.md
+++ b/docs/CAMERA_TUNING.md
@@ -1,6 +1,7 @@
 # 摄像头实时调参
 
 `test_camera_detector.py` 会同时打开 YOLO 检测画面和摄像头参数窗口。参数修改后立即作用于当前摄像头，可直接观察检测框和置信度的变化。
+（注：当前系统架构中深度相机硬件流保持常开，人工曝光调参针对底层 UVC/RGB 曝光与白平衡，不影响任务层的观察窗口数据采纳机制。）
 
 ## 启动
 
diff --git a/docs/PERCEPTION_MODE_COMMANDS.md b/docs/PERCEPTION_MODE_COMMANDS.md
index c2e8300..e2a2fb3 100644
--- a/docs/PERCEPTION_MODE_COMMANDS.md
+++ b/docs/PERCEPTION_MODE_COMMANDS.md
@@ -1,7 +1,8 @@
-# 感知模式切换命令
+# 单相机常开与观察窗口操作参考
 
-项目全流程仅使用单一深度相机 `camera`。感知模式简化为三种：`OFF`、`QR`、`DETECT`。
-旧的 `SIDES` 与 `TREE` 模式已完全移除并不再受支持。
+全赛区仅使用单 RealSense D435i 深度相机 `camera`。历史动态开关模式（`CameraModeManager`、`OFF/QR/DETECT` 状态机、RPC 参数服务启停及模式回声等待）已彻底移除。
+当前相机硬件视频流（Color、Aligned Depth、RGBD）保持常开，OpenVINO 检测与熟度分类模型常驻内存并持续推理。
+抓取证据累积与二维码解码严格由任务时机驱动的**观察窗口**（Observation Window）进行隔离与采纳。
 
 每个新终端先加载项目运行环境：
 
@@ -10,68 +11,73 @@ cd /home/rc4/fruit_ws
 source scripts/setup_fruit_runtime.bash
 ```
 
-## 果蔬目标识别（正式任务 A / B / C）
+## 观察窗口契约 (/fruit_mission/observation_window)
 
-在各赛区需要进行果蔬检测与深度测量时切换到 `DETECT` 模式：
+观察窗口命令以严格 JSON 格式发布在 `/fruit_mission/observation_window`（话题类型为 `std_msgs/msg/String`，QoS 为 Reliable / Volatile / Depth 10）。
 
-```bash
-ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
-  "{data: 'DETECT'}"
-```
+### 1. 开启观察窗口 (valid=1)
 
-模型根目录由 `src/fruit_perception/config/fruit_perception.yaml` 的 `model_root`
-指定，当前区域 `/fruit_mission/current_area` 决定检测器和候选选择策略：
-- Area `A`: `vegetable_type/best.onnx` + 熟度分类，输出 `ripe/unripe`，仅成熟目标可单目标锁定。
-- Area `B`: `fruit_type/best.onnx` + 熟度分类，输出 `apple_* / pear_*`，整树窗口最多输出 2 个稳定候选。
-- Area `C`: `vegetable_type/best.onnx` + 熟度分类，输出品种与熟度；辣椒使用 `chili_*`，按期望成熟品种锁定。
+```bash
+# 示例：开启 A 区采摘观察窗口（期望 ripe 成熟目标，截断时间设为当前时间）
+ros2 topic pub --once /fruit_mission/observation_window std_msgs/msg/String \
+  "{data: '{\"window_id\":\"manual-a-01\",\"valid\":1,\"start_sec\":1000,\"start_nanosec\":0,\"area\":\"A\",\"purpose\":\"harvest\",\"expected_type\":\"ripe\"}'}"
 
-感知兼容区域 `D` 并复用 B 检测与候选策略，不代表已实现 D 区任务。
-默认熟度分类器为 Edge，启动时可选 YOLO；固定 Intel `GPU.0`，详见
-[两阶段运行文档](../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md)。
+# 示例：开启 B 区扫码窗口
+ros2 topic pub --once /fruit_mission/observation_window std_msgs/msg/String \
+  "{data: '{\"window_id\":\"manual-b-qr-01\",\"valid\":1,\"start_sec\":1000,\"start_nanosec\":0,\"area\":\"B\",\"purpose\":\"qr\",\"expected_type\":\"\"}'}"
+```
 
-目标确认已使用短滚动窗口，不再固定等待 A/C 35 帧、B 50 帧。
-品种/熟度一致的证据足够后提前放行，短暂漏检不会立即清空全部证据；
-窗口、最小支持数、支持比例和熟度概率门限在源码 YAML 中配置，参数含义见
-[感知说明](../src/fruit_perception/README.md#short-window-target-confirmation)。
-修改后需重建并重启感知容器；深度定位和运动安全检查不变。
-性能诊断时在以下 launch 命令中添加 `perception_log_level:=info`，查看已有
-`camera performance:` 日志。1 秒内确认是优化目标，不是本轮实测结果。
+字段说明：
+- `window_id`：非空字符串，任务层每次开窗生成的唯一 UUID。
+- `valid`：严格整数 `1`（开窗）或 `0`（关窗），拒绝布尔值或浮点数。
+- `start_sec` / `start_nanosec`：ROS 图像时间戳截止线。凡是曝光时间早于此截断时间的历史帧或旧在途帧均被丢弃，不计入本次证据链。
+- `area`：`"A"`、`"B"` 或 `"C"`；观察窗口不接受 D。
+- `purpose`：`"harvest"`（累积采摘目标证据）或 `"qr"`（二维码解码）。
+- `expected_type`：期望采摘标签（如 `"ripe"`、`"pumpkin_ripe"` 等），空字符串表示不指定。
 
-单独检查感知（不启动串口桥或任务）时，可先启动：
+### 2. 关闭观察窗口 (valid=0)
 
 ```bash
-ros2 launch fruit_perception fruit_perception.launch.py start_serial_bridge:=false
-# 另一个已加载环境的终端，示例选择 B 区：
-ros2 topic pub --once /fruit_mission/current_area std_msgs/msg/String "{data: 'B'}"
+# 关闭指定窗口
+ros2 topic pub --once /fruit_mission/observation_window std_msgs/msg/String \
+  "{data: '{\"window_id\":\"manual-a-01\",\"valid\":0,\"start_sec\":1000,\"start_nanosec\":0,\"area\":\"A\",\"purpose\":\"harvest\",\"expected_type\":\"ripe\"}'}"
 ```
 
-再发布 `DETECT`。当前区域不是相机模式；正常比赛由 mission 设置，不要在任务运行时手动覆盖。
+关窗命令下发后，感知节点与扫码节点立即终止当前窗口的数据采纳并重置跟踪器候选。
+任务客户端先立即撤销本地采纳，再发布关窗命令；关窗保留原 ID、开始时间、区域、用途和期望类型，仅改 `valid=0`。
+Mission 和单点客户端在缓存结果前检查当前窗口 ID 及合法窗后图像时间；二维码计划还须与窗口的 qr 用途和 B/C 区域匹配。
+二维码 JSON 必须为对象，秒/纳秒必须为整数（拒绝 bool、浮点、NaN、缺失字段），纳秒范围为 0～999999999。
 
-## 二维码扫描（B / C 区入口）
+## 持续推理与窗外数据特性
 
-使用 camera 读取二维码：
+1. **持续推理与按需标注**：即便处于窗口之外，深度相机仍以固定帧率出流，OpenVINO 核显持续进行 2D/3D 检测与标注图生成，供监视器与调试工具观察。
+2. **证据严格隔离**：
+   - 处于窗口之外时，`/fruit_perception/decision` 发布的决策消息中 `observation_window_id` 为空（`""`），`has_target=false`，`targets_ready=false`，`targets` 列表为空。**窗外的原始检测绝不授权抓取**。
+   - 只有在有效 harvest 窗口内且帧时间戳满足条件时，滑动窗口目标跟踪器（A/C 区 `CameraTargetSelector` 8 帧窗口，B 区 `BTargetSelector` 8 帧窗口）才累积证据；达到锁定条件后输出带有对应 `observation_window_id` 的可抓取目标。
+3. **二维码到站即扫**：
+   - `fruit_qr_planner` 仅在 `purpose="qr"` 的窗口内对窗后帧进行解码，同一窗口仅发布一次确认计划（`confirm_frames=1`，单次等待上限 10 秒）。
+   - 输出的 `/fruit/b_pick_plan` 和 `/fruit/c_pick_plan` 计划中携带 `observation_window_id`、`image_stamp_sec`、`image_stamp_nanosec`。
+   - B 扫码失败后直接使用任务固定兜底计划并继续执行，不阻断后续 C 扫码窗口开启。
 
-```bash
-ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
-  "{data: 'QR'}"
-```
+## 任务编排与静止边界
 
-## 停止采集与推理
+1. **到位即开窗**：正式 mission 中，底盘导航到站且对应的 Look / QR 预设动作执行完成后，直接开启对应的观察窗口。
+2. **物理静止保证**：依据用户明确保证，移除了开窗前与窗口内的 `stable_stop` 阻塞等待与持续监测配置；由现场物理保证观察到抓取目标冻结期间机构完全静止。
+3. **安全恢复保护**：运动结果未知时的 STOP 发布、导航取消、EMERGENCY 锁定、恢复前 STOP 之后的机械臂停稳确认（`is_stationary` <= 0.05 rps）以及一次 Home 复位保护完整保留。
+4. **B 区一次观察**：B 区每树仅观察一次，在首抓前于同一静止姿态下完成最多 2 个成熟目标的筛选与基座坐标冻结并关闭窗口，第二抓复用已冻结请求，不重新观察。
 
-关闭相机流和推理：
+## 常用调试命令
 
 ```bash
-ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
-  "{data: 'OFF'}"
-```
-
-## 查看当前生效模式
+# 启动感知与扫码（不启动串口桥与任务）
+ros2 launch fruit_perception fruit_perception.launch.py start_serial_bridge:=false
 
-模式切换成功后，可查看当前激活模式（带 Transient Local QoS）：
+# 打印当前决策消息
+ros2 topic echo /fruit_perception/decision
 
-```bash
-ros2 topic echo --qos-durability transient_local --once \
-  /fruit_perception/active_mode std_msgs/msg/String
+# 打印二维码计划（Transient Local 锁存）
+ros2 topic echo --qos-durability transient_local --once /fruit/b_pick_plan
+ros2 topic echo --qos-durability transient_local --once /fruit/c_pick_plan
 ```
 
-期望输出为 `OFF`、`QR` 或 `DETECT` 之一。
+注：由于 `FruitDecision.msg` 增加了 `observation_window_id` 字段，历史旧包回放无法直接兼容，且所有节点需在更新接口后统一编译并重启。
diff --git a/docs/PROJECT_ARCHITECTURE.md b/docs/PROJECT_ARCHITECTURE.md
index c949d2b..606cf36 100644
--- a/docs/PROJECT_ARCHITECTURE.md
+++ b/docs/PROJECT_ARCHITECTURE.md
@@ -6,11 +6,11 @@
 
 省级选拔赛范围为 A、B、C 区，D 区暂不接入。当前固定流程契约为：
 
-- A 区每个停车点按配置顺序先左后右依次转向，使用单个深度相机 `camera` 在 `DETECT` 模式下识别成熟目标并请求机械臂到目标点。
-- B 区入口发布 `QR`，连续三帧确认并保存 B 计划，但暂不抓取。
-- 前往 C 区入口时保持 `QR`，确认 C 计划后使用 `DETECT`，按 C 计划严格执行 8 项。
-- 完成 C 后返回 B，发布 `DETECT`，camera 对整树的全部果实进行检测和测深，并按 B 计划位置 1 到 8 作业。
-- B 区完成后发布 `OFF` 并结束当前软件任务流程。当前范围明确不实现收集更换区、收获汇总播报、返航起始区和返航后的结束判定。
+- A 区每个停车点按配置顺序先左后右依次转向，单深度相机 `camera` 在到位后的 harvest 观察窗口内识别成熟目标并请求机械臂到目标点。
+- B 区入口到站并完成 QR 姿态后开启 QR 观察窗口，确认并保存 B 计划，但暂不抓取。
+- 前往 C 区入口时到站并完成 QR 姿态后开启 QR 观察窗口，确认 C 计划后按 C 计划严格执行 8 项。
+- 完成 C 后返回 B，在 harvest 观察窗口内对整树果实进行检测和测深，并按 B 计划位置 1 到 8 作业。
+- B 区完成后关闭观察窗口并结束当前软件任务流程。当前范围明确不实现收集更换区、收获汇总播报、返航起始区和返航后的结束判定。
 
 上位机 NUC 负责里程计、导航、识别、比赛编排，以及通过 `fruit_arm_planner` 做机械臂 FK/IK、关节映射和轨迹规划。下位机负责底盘与机械臂轨迹的实时执行、状态回报及硬件保护。
 
@@ -124,18 +124,18 @@ fruit_ws/src/
 深度相机节点为 `/camera`，原生接口为 `/camera/*`，感知输出为
 `/fruit_perception/camera/*`，决策发布于 `/fruit_perception/decision`。
 
-camera 的固定原生流开启同步、深度对齐与 RGBD。`QR` 与 `DETECT` 之间不重启相机：
+camera 的固定原生流开启同步、深度对齐与 RGBD。相机硬件流常开、模型持续推理：
 
-1. `QR` 直接把 camera 彩色原图送给 Python/OpenCV 扫码节点，按 B 后 C 的顺序各连续确认三帧。
-2. B/C 计划使用 Reliable、Transient Local、深度 1 发布，C 确认后销毁图像订阅。
-3. `DETECT` 执行 camera RGBD 水果/蔬菜推理，整帧 letterbox 到 640x640，不做分块。
+1. 到站且 QR 姿态完成后开启 QR 观察窗口，Python/OpenCV 扫码节点对彩色原图解码并确认计划（`confirm_frames=1`）。
+2. B/C 计划使用 Reliable、Transient Local、深度 1 发布，计划中附带窗口 ID 及帧时间戳。
+3. camera RGBD 持续执行水果/蔬菜推理，整帧 letterbox 到 640x640，不做分块。
    根据 `/fruit_mission/current_area` 选择两阶段链路：A/C 用 `vegetable_type/best.onnx`，
    B/D 用 `fruit_type/best.onnx`，再做熟度分类。`model_root` 来自源码 YAML；
    默认 Edge，启动可选 YOLO，固定 Intel `GPU.0`。D 只是感知路由，不新增比赛任务。
-4. `/fruit_perception/decision` 发布单相机决策消息，携带一帧内的全部目标及原图 ROI，中心深度无效时保留二维结果。
-5. A/C 区使用单目标锁定器（35 帧窗口，A 区只锁定成熟 ripe 目标，C 区按期望品种过滤成熟目标），锁定后发布 `has_target=true` 和 `target_point`。
-6. B/D 区使用多候选选择器（50 帧窗口，最多输出 2 个稳定候选），发布 `targets_ready=true` 和 `targets` 列表。
-7. 任务节点等待新鲜且有效的 `decision`，在 mission 内将 RGB 光学点转换到夹爪系，再结合新鲜实测四轴角的 FK 转为机械臂固定基座坐标，调用 `/arm_planner_node/move_cartesian` 并检查执行结果。DETECT 及提交前保持观测姿态静止；当前只到目标点，夹爪 HOLD，不代表完成夹取。外参毫米/ROS 米边界及剩余集成限制见 [mission 坐标换算文档](../src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md)。
+4. `/fruit_perception/decision` 发布单相机决策消息，携带 `observation_window_id`、一帧内的全部目标及原图 ROI，中心深度无效时保留二维结果。窗口外 `observation_window_id` 为空且不输出抓取目标。
+5. A/C 区使用单目标锁定器（8 帧短窗口，A 区只锁定成熟 ripe 目标，C 区按期望品种过滤成熟目标），在有效 harvest 窗口内锁定后发布 `has_target=true` 和 `target_point`。
+6. B/D 区使用多候选选择器（8 帧短窗口，最多输出 2 个稳定候选），在有效 harvest 窗口内发布 `targets_ready=true` 和 `targets` 列表。
+7. 任务节点等待携带当前 `observation_window_id` 的有效 `decision`，在 mission 内将 RGB 光学点转换到夹爪系，再结合新鲜实测四轴角的 FK 转为机械臂固定基座坐标，调用 `/arm_planner_node/move_cartesian` 并检查执行结果。观测及提交前由现场物理保证机构静止；当前只到目标点，夹爪 HOLD，不代表完成夹取。外参毫米/ROS 米边界及剩余集成限制见 [mission 坐标换算文档](../src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md)。
 
 4DOF 机械臂不按通用 6DOF 位姿设计。现有解析 FK/IK、PTP/LIN 与限位检查在
 `fruit_arm_planner` 中实现，几何和映射由 `config/arm_presets.yaml` 配置，不另引入 MoveIt。
@@ -156,8 +156,8 @@ RGBD 复合消息和 `compressed`/`compressedDepth`/`theora` 派生图像进入
 多次 DDS 序列化和落盘。
 调试监视器通过 color/depth metadata 与进程内 RGBD 回调产生的轻量 Header heartbeat 计算
 相机的实际频率和最后帧时间，不直接订阅高带宽 RGBD 消息。
-完整任务脚本会在导航、感知、串口桥和相机模式全部初始化完成且相机处于 `OFF` 后启动录包，
-确认 `/rosbag2_recorder` 就绪后才启动 mission，避免录包初始化与 camera 启动争抢资源。
+完整任务脚本会在导航、感知、串口桥和规划器全部初始化完成后启动录包，
+确认 `/rosbag2_recorder` 就绪后才启动 mission，避免录包初始化与节点启动争抢资源。
 
 ## 6. 上下位机接口
 
@@ -199,7 +199,9 @@ RGBD 复合消息和 `compressed`/`compressedDepth`/`theora` 派生图像进入
 正式 mission 的 `arm_presets` 映射为 `left_look/right_look/qr_look/left_harvest/right_harvest/home`。
 真实轨迹由用户配置，本次不修改 planner 文件或填写假姿态。Look 末点 OPEN，Harvest 末点 OPEN/驻留，Home 保持；视觉请求末点 CLOSE。Cartesian 服务没有驻留字段，随后复用 `move_joints` 在实测原始四轴角执行 HOLD，耗时读取 planner `default_dwell_ms`；不自行生成轨迹或用 sleep 宣告动作完成。
 
-识别前实测静止初值：底盘线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s，各关节速度绝对值 ≤0.05 rad/s，持续 0.5 秒。底盘 `/odometry` 源时间戳/接收年龄 ≤0.5 秒，机械臂沿用 planner 新鲜度并检查原始 ArmStatus，避免 NaN 被 JointState 输出替成零。阈值在 mission 配置可调。各步骤独立等待，不设整个工序共享 10 秒预算。
+识别前实测静止初值：底盘线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s，各关节速度绝对值 ≤0.05 rad/s，持续 0.5 秒。底盘 `/odometry` 源时间戳/接收年龄 ≤0.5 秒，机械臂沿用 planner 新鲜度并检查原始 ArmStatus 的有限速度；遥测也保留非有限速度，不替成零。阈值在 mission 配置可调。各步骤独立等待，不设整个工序共享 10 秒预算。
+
+现有 `/arm_planner_node/joint_states` 使用 `fruit_arm_interfaces/msg/ArmTelemetry`：同一帧包含原始硬件角、已校正的模型角及真实 TCP XYZ/pitch。控制台和 mission 直接读取模型/TCP，不重复逆映射或 FK；原始硬件角仍用于原始角服务和安全检查。话题名不变但类型与标准 JointState 不兼容，字段单位和迁移要求见 [planner 接口文档](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md)。
 
 字段类型与线协议布局见 [协议参考](LOWER_CONTROLLER_COMMANDS.md) 和
 [planner 通信文档](../src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md)。新协议没有旧 `z_type` 高度索引。
@@ -214,7 +216,7 @@ RGBD 复合消息和 `compressed`/`compressedDepth`/`theora` 派生图像进入
 - `/arm_planner_node/stop`、`/arm_planner_node/emergency_stop`：发布停止/急停；服务成功不是硬件已停稳确认。
 
 mission 使用 `home`、`left_look`、`right_look`、`qr_look`、`left_harvest` 和 `right_harvest` 这些配置映射；服务只执行 planner YAML 中实际存在的预设，不能假定现场已经补齐真实轨迹。`retract` 不属于当前 A/B/C 编排。
-相机模式通过 `/fruit_mission/perception_mode` 发布 OFF/QR/DETECT，与机械臂姿态分离。
+证据采纳由 `/fruit_mission/observation_window` 观察窗口驱动，与机械臂姿态解耦。
 新客户端必须按包级 API 接入；旧方向/收臂接口不再作为新增功能的范例。
 
 轴向导航的差速命令通过 `twist_mux` 发布到 auto_serial_bridge 的速度订阅话题，不经过离散 `task_id` 动作协议。
@@ -311,13 +313,15 @@ USE_SAVED_B_PLAN
 - B 区 camera 固定采集 50 帧，按 3D 轨迹输出最多两个稳定目标；原始类型、检测框和标注图保留误识别证据。二维码期望类型不传入感知节点，mission 做品种匹配、成熟筛选。
 - 成熟且匹配目标按既有原像素 `score = abs(pixel_x - image_width / 2) - pixel_y` 排序，同分保留原顺序，最多取两个。首次观察姿态下将两个目标一次性转换为基座坐标/绝对 pitch，后续直接用冻结请求，不重新观察/换算。每抓一次都完成 Harvest 松爪和 Home。
 - 第一抓失败但一次安全 Home 成功后仍抓第二个；恢复不安全则跳过剩余目标，不重试抓取。
-- `test_camera_arm_target.sh --zone B --left|--right --apple|--pear` 复用同一流程；需要 `/odometry` 反馈和真实预设。完成窗口为空正常跳过，未等到窗口才算识别超时。脚本会驱动硬件，不是自动验证工具。
+- `test_camera_arm_target.sh --model A|B --left|--right [--debug]` 专用于相机与机械臂单步联调：先等待实际感知区域订阅端并确认可靠 DDS 交付，机械臂进入指定 look 姿态后以实测关节角张爪并驻留，再开启识别。B 不区分品种，只从感知完成统计后的有效成熟目标中选最高置信度者；不回退抓未成熟目标。深度锁定目标后闭爪抓取并驻留，成功后执行对应侧 `left_store` 或 `right_store` 预设工序（放果并返回 home）。各操作独立等待且不超过 10 秒。脚本会驱动硬件，不是自动验证工具。
 
 ### 7.4 语音时序
 
-- B 区二维码或固定计划异步播报，C 区按配置提前量等待；同步语音等待不超过 10 秒。语音设备/模型失败只告警，继续比赛。
-- C 区二维码计划仍按整段串词播报，但等待时间缩短为预计音频结束前 `c_qr_announce_lead_sec` 秒；当前配置文件为 `10.0` 秒。达到提前点后进入机械臂收缩和下一项任务，音频继续播放至结束。
-- B 区二维码摘要使用“`1苹果，2梨子`”格式；C 区二维码摘要使用“`1，12南瓜，2，11西红柿`”格式。Piper 的 `qr_length_scale` 默认配置为 `0.8`，只用于加快二维码摘要播报。
+- B 区二维码或固定计划异步播报，C 区按配置提前量等待；同步语音等待不超过 10 秒。语音设备或素材缺失只告警，继续比赛。
+- C 区二维码计划按 8 项独立短任务顺序播报，等待时间缩短为预计总音频结束前 `c_qr_announce_lead_sec` 秒；当前配置文件为 `10.0` 秒。达到提前点后进入机械臂收缩和下一项任务，音频在后台 FIFO 队列继续播放至结束。
+- 采用维什戴尔（Wis'adel）gremlin 音色预生成短语素材与本地 WAV 拼接播报；移除 Piper 实时模型及 `qr_length_scale`。
+- B 区二维码摘要使用“`第一棵树苹果；第二棵树梨子`”格式；C 区二维码摘要使用“`第一项，四号位置，南瓜；第二项，三号位置，西红柿`”格式。
+- 任务正常遍历结束时直接播报各品种采摘工序完成总数汇总；异常或 Ctrl-C 退出立即停播、不汇总。计数依据软件确认的 CLOSE、Harvest、Home 完成记录，不作为物理入筐的硬件证明。
 
 ## 8. 维护与实机验收边界
 
diff --git a/docs/README.md b/docs/README.md
index 53c903d..bd3249a 100644
--- a/docs/README.md
+++ b/docs/README.md
@@ -10,7 +10,7 @@
 | [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) | 模块边界、数据流、任务顺序及未完成集成 |
 | [ODOMETRY_NAVIGATION.md](ODOMETRY_NAVIGATION.md) | Point-LIO、TF、轴向导航和启动入口 |
 | [WAYPOINT_RECORDING.md](WAYPOINT_RECORDING.md) | RViz 录点、Web 编辑、C 区拓扑与 yaw |
-| [PERCEPTION_MODE_COMMANDS.md](PERCEPTION_MODE_COMMANDS.md) | OFF / QR / DETECT 的 ROS CLI 操作 |
+| [PERCEPTION_MODE_COMMANDS.md](PERCEPTION_MODE_COMMANDS.md) | 单相机常开与观察窗口的 ROS CLI 操作 |
 | [CAMERA_TUNING.md](CAMERA_TUNING.md) | 单 camera 的物种检测 GUI 与参数持久化 |
 | [DATASET_PIPELINE.md](DATASET_PIPELINE.md) | 当前图像采集工具与训练环境边界 |
 | [LOWER_CONTROLLER_COMMANDS.md](LOWER_CONTROLLER_COMMANDS.md) | 串口消息契约、状态与通信限制；不是绕过 planner 的操作指南 |
diff --git a/docs/RULE.md b/docs/RULE.md
index c1403c5..aa2a5c8 100644
--- a/docs/RULE.md
+++ b/docs/RULE.md
@@ -8,7 +8,7 @@
 
 - 省级选拔赛明确不出现坏果或坏蔬菜，因此上述裁剪适合当前选拔赛目标；如果后续面向包含坏果的赛事阶段，必须补回坏果识别、摘除并丢在场地内的流程。
 - 省级选拔赛不包含 D 区。当前研发优先覆盖 A、B、C 区；D 区规则仍记录在本文中，便于后续扩展。
-- 当前软件流程明确不实现前往收集更换区、收获汇总播报、导航返回起始区和到达起始区后的结束判定。这是当前项目范围裁剪，不作为待补功能反复提出；比赛规则中的对应要求仍保留在本文供查阅。
+- 当前软件流程明确不实现前往收集更换区、导航返回起始区和到达起始区后的结束判定；任务正常结束时仅按软件记录做简要采摘工序完成品种计数汇总播报。这是当前项目范围裁剪，不作为待补功能反复提出；比赛规则中的对应要求仍保留在本文供查阅。
 
 ## 核心任务
 
diff --git a/docs/VOICE_FLOW.md b/docs/VOICE_FLOW.md
index 80ca5b2..defcf77 100644
--- a/docs/VOICE_FLOW.md
+++ b/docs/VOICE_FLOW.md
@@ -1,6 +1,8 @@
 # A/B/C 语音播报全流程
 
-本文描述 mission 的源码时序，不是硬件验收。当前范围为 A/B/C，不增加 D、换筐、收获汇总或返回起点。
+本文描述 mission 的源码时序，不是硬件验收。当前范围为 A/B/C 任务，语音播报采用维什戴尔（Wis'adel）**gremlin** 音色预生成短语素材与本地 WAV 拼接播放，比赛期间完全离线运行（不依赖 GPU 服务器、不实时合成）。
+
+主流程正常结束时直接调用采摘记录汇总播报（按品种统计已确认完成 CLOSE、Harvest、Home 全部工序的目标数量）。不增加 D 区、换筐或返回起点流程。
 
 比赛编排现在只在 `src/fruit_mission/fruit_mission/Logic.py` 审核：`main` 写分区顺序，
 `run_a/b/c` 写区内路线和动作调用。重复采摘、二维码和播报细节在 `actions.py`；
@@ -9,54 +11,58 @@ ROS 反馈、STOP 和实测停稳留在 `MissionContext`。见[mission 说明](.
 
 ```text
 起点播报 → A 区 → B 二维码计划播报 → C 二维码计划播报
-         → C 区逐项 → 返回 B 区 → B 区逐树 → 任务流程结束
+         → C 区逐项 → 返回 B 区 → B 区逐树 → 任务流程结束 → 采摘记录汇总
 ```
 
 ## 等待与局部故障
 
-- 同步语音等待最多 10 秒；原来更短的合成/播放等待保持不变。
-- 语音节点、模型或播放设备失败只告警，不结束比赛；异步语音仍按队列播报。
+- 同步语音等待最多 10 秒；播放器由单个 FIFO 后台线程串行执行，避免语音重叠。
+- 缺失素材、未知文本或播放设备失败只告警跳过当前条目，不结束比赛；不播残缺事实句。
 - 各运动、识别、二维码等待分别计时，不共用整个工序的 10 秒预算。
 - 局部失败播报并跳过，继续整场流程。未知运动仍须取消/STOP、确认安全后执行一次 Home；没有非导航重试。恢复不安全时跳过危险动作，不假报成功。
+- `Ctrl-C` 或异常退出时立即终止当前音频播放并清空未播队列，不播报汇总。
 
 ## 起点与 A 区
 
 起点同步播报（失败/超时仍继续）：
 
-> 我们是重庆大学 bobocup 水果采摘队伍，现在开始采摘
+> 我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘
 
-A 区没有正常逐点语音。导航与 left_look 并行，到点、Look 完成且底盘/机械臂连续静止后识别。
+A 区没有正常逐点语音。导航与 left_look 并行，到点且 Look 完成后开启 harvest 窗口识别。
 左侧目标 CLOSE、驻留 → left_harvest 末点 OPEN、驻留 → Home；然后 right_look 及右侧对应工序。
+工序确认完成后按实际识别品种计入采摘统计。
 局部失败安全恢复并跳过，不因普通识别或运动失败提前退出整场比赛。
 
 ## B/C 二维码
 
-二维码导航、QR 姿态、静止门槛、模式切换或解码失败均使用 mission 独立配置
+二维码导航、QR 姿态或解码失败均使用 mission 独立配置
 `src/fruit_mission/config/qr_fallback.yaml`，照常保存、播报和执行该区域。
 不因二维码失败跳过整个区域。
 
-- B 二维码摘要异步播报，随后 Home、导航可以与语音并行。
-- C 摘要按 `c_qr_announce_lead_sec` 提前量等待，音频在后台继续，任务等待仍不超过 10 秒。
-- QR 摘要使用配置的 `qr_length_scale`，不把播报完成作为机械臂完成证明。
+- B 二维码摘要异步分项播报，随后 Home、导航可以与语音并行。
+- C 摘要按 `c_qr_announce_lead_sec`（默认 10.0 秒）提前量等待，音频在后台继续，任务等待仍不超过 10 秒。
+- 长摘要按项分段入队播放，每项独立限时不超过 10 秒，保留完整 8 项顺序。
+- 语音播报不作为机械臂动作完成依据。
 
 兜底 B 摘要：
 
-> 1梨子，2苹果，3梨子，4苹果，5苹果，6苹果，7梨子，8梨子。
+> 第一棵树梨子；第二棵树苹果；第三棵树梨子；第四棵树苹果；第五棵树苹果；第六棵树苹果；第七棵树梨子；第八棵树梨子。
 
 兜底 C 摘要：
 
-> 1，4南瓜，2，3西红柿，3，1辣椒，4，10洋葱，5，8辣椒，6，9南瓜，7，2西红柿，8，11洋葱。
+> 第一项，四号位置，南瓜；第二项，三号位置，西红柿；第三项，一号位置，辣椒；第四项，十号位置，洋葱；第五项，八号位置，辣椒；第六项，九号位置，南瓜；第七项，二号位置，西红柿；第八项，十一号位置，洋葱。
 
 ## B 区逐树
 
 每树只观察一次。感知输出最多两个稳定目标；mission 保留成熟且与二维码一致的目标，
 按原像素评分 `abs(pixel_x - image_width / 2) - pixel_y` 排序，同分保留原顺序。
 在首次静止观察姿态下一次性保存最多两个基座请求；逐个 CLOSE/驻留、Harvest/松爪、Home。
+每完成一个目标的全部工序，按识别品种计入一次统计。
 第一抓失败而一次安全 Home 成功后继续第二抓，不重新观察。
 
-抓取前只播报一次品种和二维码匹配情况，不播报数量，例如：
+抓取前使用明确树号播报品种和二维码匹配情况，例如：
 
-> B区B1，识别到苹果，和二维码一致，开始抓成熟苹果。
+> B区第一棵树，识别到苹果，和二维码一致，开始抓成熟苹果。
 
 无目标、无成熟目标、品种不符或识别超时均播报“跳过”。导航失败也跳过当前树。
 抓取失败但安全恢复成功时播报“继续下一个目标”；不能确认安全时播报“跳过剩余目标”。
@@ -67,18 +73,25 @@ A 区没有正常逐点语音。导航与 left_look 并行，到点、Look 完
 严格按二维码/兜底序列推进，路线与左右侧使用现有规划结果，不增加到点方向复核。
 导航与相应 Look 并行；导航失败不在未到达的位置识别/抓取，播报并跳过当前项。
 
-正常完成 CLOSE/驻留、Harvest/松爪、Home 后，例如：
+正常完成 CLOSE/驻留、Harvest/松爪、Home 后，记录一次该品种工序完成并播报，例如：
 
 > C区十二号位置，应该采南瓜。识别到成熟南瓜，采摘放置工序完成。
 
 无目标、不成熟、种类不符或动作失败均使用“位置 + 应采品种 + 识别结果/原因”的简短播报，
 保存状态并推进下一项，不留在同一项反复观察或抓取。
 
-## 流程结束与联调边界
+## 流程结束与采摘汇总
+
+所有任务遍历正常结束后，关闭观察窗口，直接调用汇总播报：
+
+> 任务流程结束
+> 本次采摘记录
+> 苹果两个
+> 南瓜三个
 
-所有任务遍历结束后关闭感知并播报：
+若整场无有效采摘工序完成，则播报：
 
 > 任务流程结束
+> 本次没有采摘记录
 
-它表示软件流程结束，不意味着每个目标采摘成功。真实 Look/QR/Harvest/Home 姿态由用户配置；
-本次不改 planner 文件或编造角度。坐标转换、静止反馈、夹爪和轨迹传输仍须安全实机验收。
+说明：汇总中的数量基于**已确认完成全部抓取、放置与归位工序的软件记录**，并非通过传感器物理验证入筐的最终实际果实数。真实 Look/QR/Harvest/Home 姿态由用户配置；本次不改 planner 文件或编造角度。坐标转换、静止反馈、夹爪和轨迹传输仍须安全实机验收。
diff --git a/kill_fruit_processes.sh b/kill_fruit_processes.sh
index 2222bea..4f8acf8 100755
--- a/kill_fruit_processes.sh
+++ b/kill_fruit_processes.sh
@@ -37,7 +37,7 @@ target_tokens=(
   "bash ./test_camera_arm_target.sh"
   "bash test_camera_arm_target.sh"
   "bash /home/rc4/fruit_ws/test_camera_arm_target.sh"
-  "ros2 topic pub --once --wait-matching-subscriptions 1 /fruit_mission/perception_mode"
+  "ros2 topic pub --once /fruit_mission/observation_window"
   "ros2 bag record "
   "/lib/rosbag2_transport/record"
   "ros2 launch fruit_perception "
diff --git a/record_rosbag.sh b/record_rosbag.sh
index 0c84b24..63e97a9 100755
--- a/record_rosbag.sh
+++ b/record_rosbag.sh
@@ -9,7 +9,7 @@ DEBUG_MONITOR_PID=""
 START_MONITOR=1
 BAG_LABEL=""
 
-RECORD_EXCLUDE_REGEX='(^/camera/depth/image_rect_raw$|^/camera/rgbd$|/(compressed|compressedDepth|theora)$)'
+RECORD_EXCLUDE_REGEX='(^/camera/(color|depth|extrinsics|rgbd).*|^/camera/aligned_depth_to_color/camera_info$|/(compressed|compressedDepth|theora)$)'
 
 BAG_NAME_PATTERN='^([A-Za-z0-9_.-]+_)?([0-9]{8}_[0-9]{6})(_[0-9]+)?$'
 
@@ -17,10 +17,12 @@ usage() {
   cat <<'EOF'
 Usage: ./record_rosbag.sh [label] [--full] [--no-monitor]
 
-Record all discovered ROS topics, including hidden Action topics, LiDAR, and
-point-cloud topics. To control camera load and bag size, Camera unaligned raw
-depth, the composite RGBD topic, and image transport derivatives are excluded.
-The retained image topics are Camera color raw/aligned-depth/annotated images.
+Record all discovered ROS topics using the MCAP storage format.
+To control camera load and bag size, Camera raw color, unaligned raw depth,
+the composite RGBD topic, camera info/metadata, and image transport derivatives
+are excluded.
+The retained visual topics are Camera annotated image (/fruit_perception/camera/annotated_image),
+aligned depth (/camera/aligned_depth_to_color/image_raw), and fruit decisions (/fruit_perception/decision).
 --full is accepted as an explicit mode flag.
 The bag is written to:
   rosbags/YYYYMMDD_HHMMSS/
@@ -121,8 +123,9 @@ if [[ -e "$BAG_PATH" ]]; then
 fi
 
 printf '[rosbag] output: %s\n' "$BAG_PATH"
+printf '[rosbag] storage: mcap\n'
 printf '[rosbag] recording all non-excluded topics, including LiDAR and point-cloud topics\n'
-printf '[rosbag] visual topics: Camera color raw/aligned depth/annotated\n'
+printf '[rosbag] visual topics: Camera annotated image, aligned depth, and fruit decisions (raw color excluded)\n'
 printf '[rosbag] excluded topic regex: %s\n' "$RECORD_EXCLUDE_REGEX"
 printf '[rosbag] retaining the newest %s date-named bags\n' "$MAX_BAGS"
 printf '[rosbag] press Ctrl-C once when the failure has been captured\n'
@@ -136,7 +139,6 @@ fi
 ros2 bag record \
   --all \
   --include-hidden-topics \
+  --storage mcap \
   --exclude "$RECORD_EXCLUDE_REGEX" \
-  --output "$BAG_PATH" \
-  --compression-mode file \
-  --compression-format zstd
+  --output "$BAG_PATH"
diff --git a/scripts/robot_debug_monitor.sh b/scripts/robot_debug_monitor.sh
old mode 100644
new mode 100755
index 96a252a..d81e149
--- a/scripts/robot_debug_monitor.sh
+++ b/scripts/robot_debug_monitor.sh
@@ -10,6 +10,7 @@ source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
 set -u
 
 exec -a fruit_robot_debug_monitor python3 - "$LABEL" "$INTERVAL_SEC" <<'PY'
+import json
 import math
 import sys
 import time
@@ -118,11 +119,18 @@ class RobotDebugMonitor:
         self.last_health_signature = None
         self.last_nav_signature = None
         self.last_nav_latest_status = None
-        self.last_mode = {}
+        self.active_window_info = {
+            "window_id": "-",
+            "valid": 0,
+            "area": "-",
+            "purpose": "-",
+            "expected": "",
+        }
         self.last_perception_signature = {}
         self.last_perception_log_time = {}
         self.last_target_signature = None
         self.last_target_log_time = 0.0
+        self.last_camera_stream_status = None
         now = time.monotonic()
         self.camera_streams = {
             name: {
@@ -145,8 +153,7 @@ class RobotDebugMonitor:
         self.node.create_subscription(GoalStatusArray, "/follow_waypoints/_action/status", self.on_follow_status, 10)
         self.node.create_subscription(Log, "/rosout", self.on_rosout, 100)
 
-        self.node.create_subscription(String, "/fruit_mission/perception_mode", lambda msg: self.on_mode("requested", msg), 10)
-        self.node.create_subscription(String, "/fruit_perception/active_mode", lambda msg: self.on_mode("active", msg), latched_qos)
+        self.node.create_subscription(String, "/fruit_mission/observation_window", self.on_observation_window, 10)
         self.node.create_subscription(FruitDecision, "/fruit_perception/decision", self.on_decision, 100)
         self.node.create_subscription(
             Metadata,
@@ -194,7 +201,6 @@ class RobotDebugMonitor:
             "axis_waypoint_action_server",
             "fruit_mission",
             "fruit_perception_node",
-            "camera_mode_manager",
             "camera.camera",
         ):
             return
@@ -202,11 +208,28 @@ class RobotDebugMonitor:
         color = RED if level >= ros_log_level_value(Log.ERROR) else YELLOW
         log(highlight(color, "ROSOUT") + f" {message.name}: {message.msg}")
 
-    def on_mode(self, source, message):
-        if self.last_mode.get(source) == message.data:
-            return
-        self.last_mode[source] = message.data
-        log(highlight(MAGENTA, "PERCEPTION mode") + f" {source}={message.data}")
+    def on_observation_window(self, message):
+        try:
+            cmd = json.loads(message.data)
+            wid = str(cmd.get("window_id", "-"))
+            valid = int(cmd.get("valid", 0))
+            area = str(cmd.get("area", "-"))
+            purpose = str(cmd.get("purpose", "-"))
+            expected = str(cmd.get("expected_type", ""))
+            self.active_window_info = {
+                "window_id": wid,
+                "valid": valid,
+                "area": area,
+                "purpose": purpose,
+                "expected": expected,
+            }
+            state_str = "OPEN" if valid == 1 else "CLOSED"
+            log(
+                highlight(MAGENTA, "OBSERVATION window") +
+                f" state={state_str} id={wid} area={area} purpose={purpose} expected='{expected}'"
+            )
+        except Exception:
+            pass
 
     def on_decision(self, message):
         self.on_types("camera", message)
@@ -316,18 +339,26 @@ class RobotDebugMonitor:
             stamp = "ZERO"
         return f"hz={hz:.1f} last={stamp} age={age:.2f}s", age
 
-    def log_camera_streams(self, now, active_mode):
+    def log_camera_streams(self, now, perception_present):
         values = {}
         ages = {}
         for name, state in self.camera_streams.items():
             values[name], ages[name] = self.format_camera_stream(state, now)
-        expected = active_mode in ("QR", "DETECT")
+        expected = bool(perception_present)
         stale = expected and any(age > CAMERA_STREAM_STALE_SEC for age in ages.values())
         status = "STALE" if stale else "OK" if expected else "IDLE"
         color = RED if stale else GREEN if expected else CYAN
+        win = self.active_window_info
+        win_str = f"{'OPEN' if win['valid'] else 'CLOSED'}:{win['area']}:{win['purpose']}"
+
+        previous_status = self.last_camera_stream_status
+        self.last_camera_stream_status = status
+        if previous_status == "STALE" and status == "OK":
+            log(highlight(GREEN, "CAMERA streams recovered") + f" window={win_str}")
+
         log(
             highlight(color, "CAMERA streams") + " "
-            f"status={status} mode={active_mode} "
+            f"status={status} window={win_str} "
             f"color[{values['color']}] depth[{values['depth']}] rgbd[{values['rgbd']}]"
         )
 
@@ -339,15 +370,14 @@ class RobotDebugMonitor:
 
         nodes = {name for name, _namespace in self.node.get_node_names_and_namespaces()}
         fruit_perception_present = "fruit_perception_node" in nodes
-        active_mode = self.last_mode.get("active", "-")
-        requested_mode = self.last_mode.get("requested", "-")
-        self.log_camera_streams(now, active_mode)
+        self.log_camera_streams(now, fruit_perception_present)
+        win = self.active_window_info
         health_signature = (
             int(fruit_perception_present),
             self.node.count_publishers('/diagnostics'),
             self.node.count_publishers('/follow_waypoints/_action/status'),
-            requested_mode,
-            active_mode,
+            win["valid"],
+            win["window_id"],
             self.node.count_publishers('/fruit_perception/decision'),
         )
         if health_signature == self.last_health_signature:
@@ -358,7 +388,7 @@ class RobotDebugMonitor:
             f"perception_node={health_signature[0]} "
             f"diag_pub={health_signature[1]} "
             f"follow_action_status_pub={health_signature[2]} "
-            f"perception_requested={requested_mode} perception_active={active_mode} "
+            f"window_valid={win['valid']} window_id={win['window_id']} "
             f"decision_pub={health_signature[5]}"
         )
 
@@ -369,7 +399,7 @@ def main():
     node = rclpy.create_node(f"{safe_label}_robot_debug_monitor")
     log(
         "started; watching navigation, rosout warnings/errors, "
-        "camera modes, Camera stream rates and unified fruit decisions"
+        "observation windows, Camera stream rates and unified fruit decisions"
     )
     RobotDebugMonitor(node)
     try:
diff --git a/src/fruit_arm_interfaces/CMakeLists.txt b/src/fruit_arm_interfaces/CMakeLists.txt
index 0b2b6c9..9881eff 100644
--- a/src/fruit_arm_interfaces/CMakeLists.txt
+++ b/src/fruit_arm_interfaces/CMakeLists.txt
@@ -7,11 +7,16 @@ endif()
 
 find_package(ament_cmake REQUIRED)
 find_package(rosidl_default_generators REQUIRED)
+find_package(std_msgs REQUIRED)
+find_package(sensor_msgs REQUIRED)
+find_package(geometry_msgs REQUIRED)
 
 rosidl_generate_interfaces(${PROJECT_NAME}
+  "msg/ArmTelemetry.msg"
   "srv/MoveArmCartesian.srv"
   "srv/MoveArmJoints.srv"
   "srv/ExecuteArmPreset.srv"
+  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
 )
 
 ament_export_dependencies(rosidl_default_runtime)
diff --git a/src/fruit_arm_interfaces/package.xml b/src/fruit_arm_interfaces/package.xml
index 360ffb6..69872d8 100644
--- a/src/fruit_arm_interfaces/package.xml
+++ b/src/fruit_arm_interfaces/package.xml
@@ -10,6 +10,9 @@
   <buildtool_depend>ament_cmake</buildtool_depend>
 
   <build_depend>rosidl_default_generators</build_depend>
+  <depend>std_msgs</depend>
+  <depend>sensor_msgs</depend>
+  <depend>geometry_msgs</depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
 
diff --git a/src/fruit_arm_planner/README.md b/src/fruit_arm_planner/README.md
index 8ad129f..b65afa2 100644
--- a/src/fruit_arm_planner/README.md
+++ b/src/fruit_arm_planner/README.md
@@ -14,6 +14,10 @@ mission 负责相机坐标转换、目标筛选和 A/B/C 顺序，本包负责 F
 真实预设轨迹、标定、STOP 优先级和下位机保护仍需现场验收，服务成功不能单独证明完整实机闭环。
 坐标边界见[mission 换算说明](../fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md)，控制必须按[接口 API](docs/03_INTERFACES_AND_API.md)。
 
+笛卡尔 XYZ 统一指真实夹爪抓取点 TCP。`config/arm_presets.yaml` 的 `geometry.tool_z_offset_m` 配置随腕部旋转的固定下偏（当前 `-0.03 m`）；`pitch=0°` 表示 `a4` 伸出段水平，不是腕轴到 TCP 的斜线水平。直接输入和笛卡尔预设共用 FK/IK，不再额外平移输入。现有预设数值不自动迁移；几何调参后重启 planner、控制台和 mission。
+
+`/arm_planner_node/joint_states` 现为 `fruit_arm_interfaces/msg/ArmTelemetry`，原话题名不变：保留原始硬件角/速度，新增逆映射模型角/速度及真实 TCP XYZ（m）/pitch（度）。控制台 `status`、jog/示教与 mission 直接使用 planner 遥测，不重复减偏置或 FK。标准 JointState 订阅者需适配；接口、planner、mission 必须一起构建并重启。原始硬件 Status 和运动服务输入语义不变。
+
 ---
 
 ## 专题文档导航 (Documentation Index)
diff --git a/src/fruit_arm_planner/config/arm_presets.yaml b/src/fruit_arm_planner/config/arm_presets.yaml
index 2a22ec1..6b5a6bc 100644
--- a/src/fruit_arm_planner/config/arm_presets.yaml
+++ b/src/fruit_arm_planner/config/arm_presets.yaml
@@ -7,7 +7,8 @@ arm_planner_node:
       d1: 0.062       # 基座高度 (底盘安装面至肩部俯仰轴)
       a2: 0.26       # 大臂长度 (肩轴至肘轴)
       a3: 0.26       # 小臂长度 (肘轴至腕轴)
-      a4: 0.12       # 腕部至末端夹爪作用点长度
+      a4: 0.12       # 腕轴沿末端局部 X 方向伸出长度；pitch=0 时此段水平
+      tool_z_offset_m: -0.03  # 真实抓取点的末端局部 Z 偏移，随腕部旋转；下方为负
 
     # ------------------------------------------------------------------------
     # 2. 关节坐标系映射与零位偏置
@@ -21,19 +22,19 @@ arm_planner_node:
       q1:
         offset_deg: 0.0          # 偏航底座零位 (正前为0, 左正右负)
         direction: 1.0
-        limits_deg: [-180.0, 180.0]
+        limits_deg: [-185.0, 185.0]
       q2:
         offset_deg: -89.0        # 几何竖直 q2=90° 时，硬件目标角为1°
         direction: 1.0
-        limits_deg: [-87.0, 45.0] # -87°为接近底盘水平极限，+45°为后仰极限
+        limits_deg: [-87.0, 90.0] # -87°为接近底盘水平极限，+45°为后仰极限
       q3:
-        offset_deg: -6.0         # 模型小臂水平 q2+q3=0° 时，硬件目标角为-6°
+        offset_deg: -13.0         # 模型小臂水平 q2+q3=0° 时，硬件目标角为-6°
         direction: 1.0
         limits_deg: [-135.0, 90.0]
       q4:
         offset_deg: -5.0         # 模型腕部与小臂同向 q4=0° 时，硬件目标角为-5°
         direction: 1.0
-        limits_deg: [-90.0, 105.0]
+        limits_deg: [-100.0, 130.0]
 
     # ------------------------------------------------------------------------
     # 3. 规划与插值默认参数
@@ -46,7 +47,6 @@ arm_planner_node:
       execution_timeout_buffer_sec: 2.0  # 轨迹时长之外的状态等待余量 (s)
       mode_start_delay_ms: 100    # 调试缓冲：路点发布后等待再发送启动 Mode (ms)
       hover_height_m: 0.06        # 安全悬停提升高度 (m)
-      cartesian_offset_m: [0.0, 0.0, 0.03]  # 笛卡尔服务目标的 XYZ 偏置 (m)，预设不叠加
 
     # ------------------------------------------------------------------------
     # 4. 机械臂预设工序与固定轨迹动作库
@@ -88,3 +88,45 @@ arm_planner_node:
           - { target: [0.40, 0.0, 0.40, 0.0], duration_ms: 1000, gripper: 0, dwell_ms: 0 }
           # 段 3: 带着果实收回到安全悬停高度，耗时 1500ms，保持夹紧
           - { target: [0.20, 0.0, 0.25, 0.0], duration_ms: 1500, gripper: 0, dwell_ms: 0 }
+      
+      home:
+        description: "返回运动初始位置"
+        coordinate_frame: cartesian
+        waypoints:
+        - target:
+          - 0.22
+          - -0.00
+          - 0.17
+          - 0.0
+          duration_ms: 1000
+          gripper: 0
+      
+      left_store:
+        description: "左方抓取完毕后存储并回到原位"
+        coordinate_frame: cartesian
+        waypoints:
+        - target:
+          - 0.00
+          - 0.29
+          - 0.23
+          - 0.0
+          duration_ms: 2000
+          gripper: 0
+        - {duration_ms: 2000, gripper: 1, target: [-0.18, 0.01, 0.23, 0.0]}
+        - {duration_ms: 2000, gripper: 0, target: [0.00, 0.22, 0.23, 0.0]}
+        - {duration_ms: 2000, gripper: 0, target: [0.22, 0.00, 0.17, 0.0]}
+
+      right_store:
+        description: "右方抓取完毕后存储并回到原位"
+        coordinate_frame: cartesian
+        waypoints:
+        - target:
+          - 0.00
+          - -0.29
+          - 0.23
+          - 0.0
+          duration_ms: 2000
+          gripper: 0
+        - {duration_ms: 2000, gripper: 1, target: [-0.18, -0.01, 0.23, 0.0]}
+        - {duration_ms: 2000, gripper: 0, target: [0.00, -0.22, 0.23, 0.0]}
+        - {duration_ms: 2000, gripper: 0, target: [0.22, -0.00, 0.17, 0.0]}
\ No newline at end of file
diff --git a/src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md b/src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md
index f44b28c..abae133 100644
--- a/src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md
+++ b/src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md
@@ -23,10 +23,11 @@
        Z (天顶)
        ▲
        │                  [肘关节 P2]
-       │                  ●──────────────● [腕关节 P3] ───● [夹爪作用中心 P4]
+       │                  ●──────────────● [腕关节 P3] ───● [伸出段末端 P4]
        │                 /  (小臂连杆 a3)    (腕部连杆 a4)
-       │                / (大臂连杆 a2)
-       │               /
+       │                /                                 │ 固定局部 Z 偏移
+       │               /                                  ● [真实抓取点 TCP]
+       │              / (大臂连杆 a2)
        │              ● [肩关节 P1]
        │              │
        │              │ (基座高度 d1)
@@ -38,9 +39,10 @@
 - **`d1` (基座高度)**：底盘安装基准面到肩部俯仰轴轴心的高度差；
 - **`a2` (大臂长度)**：肩部旋转轴中心到肘部旋转轴中心的距离；
 - **`a3` (小臂长度)**：肘部旋转轴中心到腕部俯仰轴中心的距离；
-- **`a4` (末端作用长度)**：腕部旋转轴中心到夹爪采摘作用中心的距离。
+- **`a4` (末端伸出长度)**：腕部旋转轴沿末端局部 X 方向到伸出段末端的长度，不是腕轴到真实抓取点的斜线长度。
+- **`tool_z_offset_m`**：伸出段末端到真实夹爪抓取点 TCP 的固定局部 Z 偏移，下方为负，随腕部俯仰和底座偏航旋转。当前为 `-0.03 m`，在 YAML 中调参，不是世界 Z 平移，也不增加运动自由度。
 
-四个尺寸在启动时全部由 `config/arm_presets.yaml` 提供，算法内部不设几何默认值。
+上述几何参数在启动时全部由 `config/arm_presets.yaml` 提供。四个连杆尺寸必须有限且为正，局部 Z 偏移必须是有限数，可为零或正负值；算法内部不设几何默认值。
 
 ---
 
@@ -74,19 +76,23 @@ $$P_2 = \big(r_2 \cdot \cos(q_1), \; r_2 \cdot \sin(q_1), \; z_2\big)$$
 投影到 3D 笛卡尔坐标系：
 $$P_3 = \big(r_3 \cdot \cos(q_1), \; r_3 \cdot \sin(q_1), \; z_3\big)$$
 
-#### ④ 末端夹爪中心 $P_4$ 与末端空间俯仰角 $\text{pitch}$：
-夹爪连杆的总俯仰角为前三轴转角之和：
+#### ④ 伸出段末端 $P_4$ 与末端空间俯仰角 $\text{pitch}$：
+末端伸出段 a4 的总俯仰角为三个俯仰关节转角之和：
 $$\text{pitch} = q_2 + q_3 + q_4$$
 - 径向投影距离：$r_4 = r_3 + a_4 \cdot \cos(\text{pitch})$
 - 坚直高度：$z_4 = z_3 + a_4 \cdot \sin(\text{pitch})$
-最终末端执行器空间位姿：
-$$X = r_4 \cdot \cos(q_1), \quad Y = r_4 \cdot \sin(q_1), \quad Z = z_4$$
+#### ⑤ 真实夹爪抓取点 TCP：
+令 $b=\text{tool\_z\_offset\_m}$，固定偏移沿末端局部 Z 轴旋转：
+$$r_{TCP}=r_4-b\sin(\text{pitch}), \qquad z_{TCP}=z_4+b\cos(\text{pitch})$$
+$$X=r_{TCP}\cos(q_1), \quad Y=r_{TCP}\sin(q_1), \quad Z=z_{TCP}$$
+
+FK 的 `end_effector` 返回 TCP 的 `(x,y,z,pitch)`；`points` 按顺序返回 `P0/P1/P2/P3/P4/TCP`，包含固定下偏段。`pitch` 仍然描述 a4 段，**不描述腕轴到 TCP 的斜线**。`pitch=0°` 时 a4 水平，TCP 比腕轴低 3 cm；不会为斜线附加角度修正。
 
 ---
 
 ## 三、逆向运动学 (IK) 几何解析推导
 
-逆运动学（Inverse Kinematics, IK）负责根据给定的目标笛卡尔空间点 $(X, Y, Z)$ 及期望俯仰角 $\text{pitch}$，反算出 4 个关节理论转角 $(q_1, q_2, q_3, q_4)$。
+逆运动学（Inverse Kinematics, IK）负责根据真实抓取点 TCP 的目标 $(X, Y, Z)$ 及 a4 段期望俯仰角 $\text{pitch}$，反算出 4 个关节理论转角 $(q_1, q_2, q_3, q_4)$。直接笛卡尔请求和笛卡尔预设共用此模型，不再对输入坐标额外平移。
 
 ### 1. 求解基座旋转偏航角 $q_1$
 由于机械臂安装于小车前进方向，底座直接朝向目标点在底盘水平面上的投影：
@@ -95,9 +101,11 @@ $$q_1 = \text{atan2}(Y, X)$$
 $$r = \sqrt{X^2 + Y^2}$$
 
 ### 2. 末端隔离，逆推腕部中心 $P_3$ 在 $(r, z)$ 平面内的位置
-已知末端作用点位置为 $(r, Z)$，末端杆长为 $a_4$。沿反方向推导腕关节中心坐标 $(r_w, z_w)$：
-$$r_w = r - a_4 \cdot \cos(\text{pitch})$$
-$$z_w = (Z - d_1) - a_4 \cdot \sin(\text{pitch})$$
+已知 TCP 位置为 $(r, Z)$，伸出长度为 $a_4$，局部 Z 偏移为 $b$。反向扣除这两段刚性结构，得到腕关节中心坐标（$z_w$ 相对肩轴）：
+$$r_w = r - a_4 \cos(\text{pitch}) + b\sin(\text{pitch})$$
+$$z_w = (Z - d_1) - a_4 \sin(\text{pitch}) - b\cos(\text{pitch})$$
+
+水平时 $b=-0.03m$，腕轴比请求的 TCP 高 3 cm；倾斜时按姿态旋转补偿。腕部角仍为 $q_4=\text{pitch}-(q_2+q_3)$，与关节零位标定无关。
 
 ### 3. 工作空间物理可达性检查
 计算肩关节 $P_1(0, d_1)$ 到腕关节 $P_3(r_w, z_w + d_1)$ 的直线连线长度 $d$：
@@ -169,13 +177,15 @@ $$q_{3,\text{algo}} = \frac{q_{3,\text{hardware}} - \text{offset}_3}{\text{direc
   - 向上抬为正，向前下方俯低为负，大臂俯下到接近水平时对应 $-87^\circ \sim -90^\circ$；
 - **Q3（小臂）**：
   - 小臂水平朝前时，绝对倾角 $q_2 + q_3 = 0^\circ$，硬件角也为 $0^\circ$；
-  - 已确认小臂水平时原始反馈约为 $0^\circ$，且大臂单独转动时该反馈不变；当前 YAML 用 $-6^\circ$ 的标定偏置，计算时由映射器统一处理；
+- 已确认小臂水平时原始反馈约为 $0^\circ$，且大臂单独转动时该反馈不变；计算时由映射器统一处理源码 YAML 的标定偏置，当前值为 $-13^\circ$，不从 YAML 注释或旧文档取值；
   - 小臂相对水平面向上为正，向下为负；
 - **Q4（腕部）**：
   - 电机安装在小臂上，算法将 Q4 定义为末端相对小臂的角度。当前 YAML 映射为 $\mathbf{offset = -5.0^\circ, direction = +1.0}$；当小臂不水平时，Q4 的硬件角为零并不代表末端水平。
 
 ---
 
+反馈遥测还提供模型角速度：$\dot q_2=\dot h_2/direction_2$，$\dot q_3=\dot h_3/direction_3-\dot q_2$，Q1/Q4 各自除以 direction。常量 offset 求导为零，不能对速度套用角度偏置。此转换复用 `JointTransformer.hardware_velocity_to_algo`，非有限速度不替成零。
+
 ## 六、五次多项式平滑轨迹插值原理 (TrajectoryPlanner)
 
 机械臂在加减速过程中，如果加速度突变，会对机械减速器和连杆产生剧烈冲击（Jerk）。本包采用**归一化五次多项式 (Quintic Polynomial / S-Curve)** 进行离散时间插值。
diff --git a/src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md b/src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md
index 371125e..1b23ef1 100644
--- a/src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md
+++ b/src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md
@@ -18,9 +18,11 @@ auto_serial_bridge  --传输业务数据--  下位机
 
 - 上位机负责把任务目标转换成机械臂的四轴硬件关节轨迹。视觉坐标应先在上层转换到机械臂根部固定坐标系；下位机不接收笛卡尔目标或预设名称。
 - 规划器通过 `/arm_planner_node/move_joints`、`/arm_planner_node/move_cartesian`、`/arm_planner_node/execute_preset` 接收正式运动请求。`home`、`retract` 是预设快捷服务；`cmd_joints`、`cmd_cartesian`、`cmd_preset` 是无逐请求结果的调试话题。
-- `move_joints` 的目标单位是度；`move_cartesian` 的 XYZ 单位是米、`pitch_deg` 是度。规划器的 `planning.cartesian_offset_m` 只叠加到笛卡尔服务目标（以及复用其回调的 `cmd_cartesian`），不叠加到预设航点。下发给下位机的四轴角度统一为**硬件关节弧度 rad**，关节角速度回报单位为 rad/s。
+- `move_joints` 的目标单位是度；`move_cartesian` 的 XYZ 是真实夹爪抓取点 TCP 的基座坐标（米），`pitch_deg` 是末端 a4 伸出段俯仰角（度）。直接输入和笛卡尔预设共用 `geometry.tool_z_offset_m` 固定结构模型，不再叠加旧 `planning.cartesian_offset_m` 输入平移。下发给下位机的四轴角度统一为**硬件关节弧度 rad**，关节角速度回报单位为 rad/s。
 - 底盘速度是独立的下发路径，不经过机械臂规划器。
 
+业务遥测仍使用 `/arm_planner_node/joint_states`，类型已改为 `fruit_arm_interfaces/msg/ArmTelemetry`：同一帧原始电机角、逆映射模型角及 FK TCP。硬件 `/fruit_hardware_bridge/arm/status` 的 ArmStatus 与线协议保持不变；模型角不要再次逆 mapping。详见[遥测字段](03_INTERFACES_AND_API.md#六遥测广播接口-telemetry)。
+
 ## 2. 上位机下发的数据
 
 | 数据及 ROS 话题 | 字段 | 用途 |
@@ -81,6 +83,6 @@ ROS 接口定义的 `error_code` 包括：`0` 无故障、`1001` 超时、`1002`
 ## 5. 任务结果与异常处理
 
 - 规划器从启动 Mode 发布后开始等待新的 `ArmStatus`。只有状态的 `trajectory_id` 等于本次请求 ID 且 `task_status=COMPLETED(2)`，运动 service 才返回 `success=true`；发布轨迹或收到 `RUNNING` 都不表示动作完成。
-- 匹配的 `FAILED(3)`、`EMERGENCY(4)` 或显式停止使本次运动返回失败。超过轨迹末点时间加 `execution_timeout_buffer_sec` 仍未收到完成状态，则返回超时并发布 STOP；**发布 STOP 不证明实机已经停稳**。
+- 匹配的 `FAILED(3)`、`EMERGENCY(4)` 或显式停止使本次运动返回失败。硬件报错由下位机自然上报；**发布 STOP 不证明实机已经停稳**。
 - 规划器在一次运动期间拒绝新的普通运动请求；没有新鲜有效的四轴状态、目标不可达、越限或轨迹格式无效时，拒绝下发。STOP/急停不依赖有效的运动起点。
 - 当前业务数据中没有“已装载全部 N 个路点”的上报字段，也没有底盘速度执行完成状态。因此不能用 `IDLE`、话题发布成功或夹爪动作命令代替这些确认。下位机的终态保持时长、故障码含义和实际缓存容量仍需实机核对。
diff --git a/src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md b/src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md
index 16b06f2..47662a4 100644
--- a/src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md
+++ b/src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md
@@ -36,7 +36,7 @@
                                     └───────────────────────┘
                                                 │
                                                 ▼ 广播
-                                     话题: ~/joint_states (20Hz 实时实测物理角)
+                                     话题: ~/joint_states (20Hz 原始角、模型角、TCP)
 ```
 
 > **节点命名空间约定**：
@@ -156,8 +156,8 @@ presets:
 ### 1. 笛卡尔末端带模式选择服务 (`~/move_cartesian`)
 - **服务类型**：`fruit_arm_interfaces/srv/MoveArmCartesian`
 - **请求字段**：
-  - `x, y, z` (`float32`): 空间坐标 (m)
-  - `pitch_deg` (`float32`): 俯仰角 (度，0度严格水平)
+  - `x, y, z` (`float32`): 真实夹爪抓取点 TCP 的基座空间坐标 (m)
+  - `pitch_deg` (`float32`): 末端 a4 伸出段俯仰角（度，0度表示该段水平，不是腕轴到 TCP 的斜线水平）
   - `duration_ms` (`uint32`): 期望耗时 (ms)
   - `gripper_action` (`uint8`): 夹爪动作 (0=保持, 1=张开, 2=闭合)
   - **`linear_path` (`bool`)**: **解算模式选择！** `true`=高精度严格空间直线（逐点逆解），`false`=快速平滑弧线（终点单次逆解）
@@ -165,8 +165,8 @@ presets:
   - `success` (`bool`): 是否成功执行完毕
   - `message` (`string`): 执行状态描述或报错原因
   - `trajectory_id` (`uint32`): 任务流水号
-- **执行结果**：节点从发布启动 Mode 起，以轨迹末点时间加配置的 `execution_timeout_buffer_sec` 等待本次轨迹 ID 的新状态。收到 `COMPLETED` 才返回成功；`FAILED`、`EMERGENCY`、主动停止或超时返回失败。超时会发布 STOP；发布成功不代表硬件已停稳。
-- **目标偏置**：服务收到的 `(x, y, z)` 在 PTP 求解或 LIN 路径规划前，逐轴加上 `arm_presets.yaml` 的 `planning.cartesian_offset_m`（单位 m）。例如偏置 `[0, 0, -0.03]` 时，输入 `(0, 0, 0)` 会以 `(0, 0, -0.03)` 求解；是否可达仍由 IK 和限位检查决定。LIN 起点使用实测关节角的正解坐标，不加偏置；`pitch_deg` 不变。`cmd_cartesian` 复用本服务回调，因此同样生效；`ExecuteArmPreset` 的笛卡尔航点及关节控制不受影响。
+- **执行结果**：节点从发布启动 Mode 起，等待下位机上报本次轨迹 ID 的新状态。收到 `COMPLETED` 返回成功；下位机报错（`FAILED`、`EMERGENCY`）或外部主动调用 STOP 返回失败。发布 STOP 成功不代表硬件已停稳。
+- **统一抓取点模型**：`arm_presets.yaml` 的 `geometry.a4` 定义腕轴沿末端局部 X 的伸出长度，`geometry.tool_z_offset_m` 定义真实抓取点相对伸出段末端的局部 Z 偏移（米，下方为负，当前 `-0.03`）。偏移随腕部旋转，FK/IK、PTP/LIN、`cmd_cartesian` 和 Cartesian preset 共用，不再额外加减输入坐标。`pitch_deg=0°` 时 a4 段水平，真实抓取点比腕轴低 3 cm，并到达请求 XYZ。关节角命令/预设保持原角度语义，不额外转 IK；其 FK 输出同样为 TCP。旧 `planning.cartesian_offset_m` 已移除。现有预设航点数值不自动迁移，由操作者按新 TCP 定义核对；几何修改后须重启 planner、控制台和 mission，不能仅 reload presets。
 
 ---
 
@@ -175,6 +175,11 @@ presets:
 - **请求字段**：`joints_deg[4]` 为四个原始硬件角（度），不应用连杆零位偏置；另有 `duration_ms`、`gripper_action`。控制台 `joints` 是连杆角，`motors` 才与本服务语义一致。
 - **应答字段**：`success`, `message`, `trajectory_id`
 
+相机单点调试客户端在 Look 后、开启识别前，以新鲜且静止的实测四轴硬件角调用本服务，
+先 `gripper_action=1`（OPEN），再以更新的实测角调用 `gripper_action=0`（HOLD）驻留；
+两次请求均等待执行完成，失败或超时不继续目标运动。共享客户端的服务和端点等待分别
+限幅到 10 秒，较短配置不变。此流程不修改 Look 预设，不引入直接串口或原始轨迹控制。
+
 ---
 
 ### 3. 预设工序调用服务 (`~/execute_preset`)
@@ -205,13 +210,35 @@ presets:
 
 ### 机械臂实时状态广播 (`~/joint_states`)
 - **完整话题名**：`/arm_planner_node/joint_states`
-- **消息类型**：`sensor_msgs/msg/JointState`
+- **消息类型**：`fruit_arm_interfaces/msg/ArmTelemetry`
 - **发布频率**：仅在收到有效四轴反馈时发布，正常情况下跟随下位机约 20 Hz 状态回传。
-- **功能描述**：实时将下位机回传的物理编码器数值转为标准 ROS 2 `JointState`，供 RViz 机械臂 3D 模型联动或上层监控节点监视。
-- **字段规范**：
-  - `name`: `['joint_1', 'joint_2', 'joint_3', 'joint_4']`
-  - `position`: 当前 4 个关节的实测物理弧度值 $[q_1, q_2, q_3, q_4]$
-  - `velocity`: 当前 4 个关节的瞬时角速度 (rad/s)
+- **功能描述**：同一帧有效电机反馈同时提供原始硬件角、逆 Joint Mapping 后的模型角，以及共享 FK 算出的真实夹爪抓取点 TCP。不新增话题、不修改标准 ROS 消息定义。
+
+| 字段 | 含义 / 单位 |
+| --- | --- |
+| `header` | planner 收到反馈后的 ROS 时间；`frame_id=arm_base_link` 是 geometry 定义的固定机械臂根部坐标系，不是相机或底盘坐标系；此消息不发布 TF。 |
+| `hardware_joint_state` | `sensor_msgs/JointState`：原始电机角 `h1..h4`，`position` 为 rad，`velocity` 为 rad/s，不消除标定偏置。 |
+| `model_joint_state` | `sensor_msgs/JointState`：逆 mapping 后的 `q1..q4`，`position` 为 rad，`velocity` 为 rad/s；Q1=偏航，Q2=大臂相对水平，Q3=小臂相对大臂，Q4=腕部相对小臂。**不再对这组数据执行逆 mapping。** |
+| `tcp_position` | `geometry_msgs/Point`：真实夹爪抓取点 XYZ，单位 m，已经包含 `geometry.tool_z_offset_m`。 |
+| `tcp_pitch_deg` | a4 伸出段的绝对俯仰角，单位度；0° 表示 12 cm 段水平，TCP 位于其下方 3 cm，不是腕轴到 TCP 的斜线水平。 |
+| `trajectory_id/task_status/progress/gripper_state/error_code` | 原帧硬件状态元数据，原样保留；数值语义以 `fruit_hardware_interfaces/msg/ArmStatus` 为准。 |
+
+两个 JointState 的 `name` 均为 `['joint_1','joint_2','joint_3','joint_4']`，四轴 position/velocity 各四项，effort 为空；header 与外层一致。角速度按方向及 Q3 耦合求导，不减常量 offset；非有限速度保留，不伪造为零。无效/越限位置不发布遥测，消费者必须检查反馈新鲜度，不能把最后一帧当成当前状态。
+
+**兼容性变更**：该话题不再是标准 JointState，旧订阅者必须改为 ArmTelemetry；RViz/robot_state_publisher 不能直接把此话题当成标准 JointState 输入。本次不新增适配话题。`~/cmd_joints` 仍使用标准 JointState，运动服务及硬件状态协议不变。
+
+确认硬件安全停稳，停止旧 planner、控制台及 mission；构建接口和消费者，然后重新加载运行环境并重启这些进程。不能让同名话题的新旧消息类型同时运行，`reload_presets` 也不能代替重启。可只读核对：
+
+```bash
+cd /home/rc4/fruit_ws
+source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
+colcon build --base-paths src --packages-select fruit_arm_interfaces fruit_arm_planner fruit_mission \
+  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
+source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
+# 重启 planner 和客户端后执行以下只读命令，不会命令机械臂运动：
+ros2 interface show fruit_arm_interfaces/msg/ArmTelemetry
+ros2 topic echo --once /arm_planner_node/joint_states fruit_arm_interfaces/msg/ArmTelemetry
+```
 
 ---
 
diff --git a/src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md b/src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md
index 42daf11..b85b33c 100644
--- a/src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md
+++ b/src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md
@@ -64,6 +64,15 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 # 同时逐点显示轨迹：添加 --log-level trace
 ```
 
+每次启动自动在 `src/fruit_arm_planner/test/log/` 新建日志，文件名使用机器本地时间，
+格式为 `YYYY-MM-DD_HH-MM-SS_微秒.log`；启动时会显示完整路径，不覆盖已有文件。
+控制台使用系统自带的 `/usr/bin/script` 记录完整终端会话：标准输出、错误输出、
+异常栈、命令回显和 jog 界面均保留，终端显示照常，每次写入后刷新日志。
+日志是原始终端记录，含 ANSI 光标/颜色控制序列；日志目录已被 Git 的 `log/` 规则忽略。
+日志按当前 `info/debug/trace` 等级记录实际输出，不额外开启被该等级隐藏的遥测。
+仅记录控制台进程，不自动收集另一个终端中的 planner/串口桥输出。
+如果无法创建或打开日志，则报错退出，不启动机械臂客户端，也不静默降级成无日志运行。
+
 控制台通过规划器服务规划并立即执行以下运动命令：
 
 | 命令 | 含义与示例 |
@@ -72,6 +81,7 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 | `joints Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]` | 连杆目标角度（度）：Q2 相对竖直（后仰为正），Q3 为小臂相对水平面的绝对角，Q4 为末端相对水平面的绝对俯仰角。控制台按 YAML 偏置换算后下发；例如 `joints 0 20 -40 0 2500 hold` |
 | `motors Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]` | 直接输入四个原始电机硬件角（度），不再叠加偏置；沿用旧版 `joints` 的语义 |
 | `preset NAME` | 执行源码 YAML 中存在的预设，例如 `preset zero`；当前名称为 `zero`、`left_look`、`right_look`、`pick_sequence`，现场可调 |
+| `status` | 显示同一帧原始电机角、逆映射模型角、连杆角，以及真实 TCP 的 XYZ（m）和绝对 pitch（度） |
 | `jog` | 进入终端内键盘微调和示教打点界面，按键直接执行运动 |
 | `reload` | 调用 `/arm_planner_node/reload_presets`，重新加载源码 YAML 的全部 presets |
 
@@ -93,7 +103,7 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 | Esc | 清除待执行目标并退出；若仍有运动则请求 STOP |
 | Ctrl+C | 请求急停并退出控制台 |
 
-进入时以最新四轴实测角的 FK 作为初始 XYZ 和俯仰角。空闲时按键自动提交运动；运动期间继续累加目标，完成后自动执行最新目标，**不逐键排队、不打断当前轨迹**。每次提交使用运行中 Planner 的 `default_duration_ms`，位置/俯仰运动的夹爪动作为保持。界面区分实测位置、累计目标、执行目标、待执行夹爪动作及当前点记录动作；不可达、越限、执行失败或停止后清除所有待执行动作、不自动续发，须检查原因并重新进入模式。
+进入时使用 planner 最新遥测的真实 TCP 作为初始 XYZ 和俯仰角，不在控制台再次逆 mapping 或 FK。空闲时按键自动提交运动；运动期间继续累加目标，完成后自动执行最新目标，**不逐键排队、不打断当前轨迹**。每次提交使用运行中 Planner 的 `default_duration_ms`，位置/俯仰运动的夹爪动作为保持。界面区分实测位置、累计目标、执行目标、待执行夹爪动作及当前点记录动作；不可达、越限、执行失败或停止后清除所有待执行动作、不自动续发，须检查原因并重新进入模式。
 
 `c/o` 复用 `move_joints` 服务，以最新有效四轴实测硬件角（转为度）作为目标，只改变夹爪动作，不再 IK 或叠加 Cartesian offset。服务起点仍由 Planner 获取实测角，反馈与执行有时间差，不应视为硬件锁轴。夹爪动作在默认时长轨迹的末点触发，并非按键瞬间完成。运动中按 `c/o` 会合并为最新待执行夹爪动作；当前动作结束后先原地执行夹爪动作，再继续累计移动。`h` 取消尚未下发的夹爪动作，已提交的请求不受它撤销。
 
@@ -101,7 +111,7 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 
 移动键和夹爪键不会写文件。空格只在全部累计运动和夹爪请求完成后可用，冻结当前有效实测位置和姿态，再输入 preset 名称：已有 preset 按其 `cartesian` / `joints` / `link_angles` 类型转换后追加，不存在则新建 Cartesian preset。航点时间读取源码 YAML 的 `planning.default_duration_ms`，`gripper` 保存当前点的动作标签；不额外写入 `dwell_ms`，闭合/张开航点回放沿用 `planning.default_dwell_ms`。原有配置和注释不整体重写，文件写入失败不覆盖原文件。保存后自动请求热重载；若重载失败，文件仍已保存，处理错误后执行 `reload`。
 
-点动显示及记录的是实际模型坐标。提交笛卡尔服务前先减去 `cartesian_offset_m`，抵消 Planner 的偏置叠加；Cartesian preset 直接保存 FK 坐标，不再加偏置。终端弹窗期间不执行运动键；Esc 取消和保存均不会执行该 preset。
+点动显示及记录的是统一 FK 模型的真实夹爪抓取点 TCP 坐标；提交服务和保存 Cartesian preset 使用同一坐标，不再加减输入偏置。终端弹窗期间不执行运动键；Esc 取消和保存均不会执行该 preset。
 
 `reload` 只刷新全部 presets，运动期间或校验失败时保留旧内存内容，不修改几何、关节映射及 planning。调整默认时长、偏置或机械参数后重启 Planner 和控制台，保证双方使用同一份配置；升级本次 Python 代码也需构建并重启 Planner。点动时长是计划运动时间，不含通信及启动等待，不代表固定按键执行频率。
 
@@ -111,8 +121,11 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 ros2 service call /arm_planner_node/reload_presets std_srvs/srv/Trigger '{}'
 ```
 
-`DURATION_MS=0` 或省略时使用规划器默认时间；两种关节命令默认夹爪保持。当前标定下，`joints 0 0 0 0` 会换算成约 `[0, 1, -6, -5]°` 的硬件目标；`motors 0 0 0 0` 则原样下发全零硬件角。`status` 显示原始电机角以及按 YAML 反算的连杆角。还支持 `stop`、`estop`、`level`、`help`、`quit`。
-预设的 `coordinate_frame: link_angles` 使用与 `joints` 相同的四项定义，末端目标为 `0°` 时腕部自动抵消小臂倾角；`coordinate_frame: joints` 保留原始电机角定义。折叠姿态也须校验腕部映射后是否超限，不能复制旧限位数值；当前配置没有 `home/retract`，对应快捷服务会返回未知预设失败。
+`DURATION_MS=0` 或省略时使用规划器默认时间；两种关节命令默认夹爪保持。当前标定下，`joints 0 0 0 0` 会换算成约 `[0, 1, -13, -5]°` 的硬件目标；标定修改后以源码 YAML 实际数值为准。`motors 0 0 0 0` 则原样下发全零硬件角。`status` 显示原始电机角、planner 提供的逆映射模型角、由模型角换算的连杆角及真实 TCP。还支持 `stop`、`estop`、`level`、`help`、`quit`。
+预设的 `coordinate_frame: link_angles` 使用与 `joints` 相同的四项定义，末端目标为 `0°` 时腕部自动抵消小臂倾角；`coordinate_frame: joints` 保留原始电机角定义。折叠姿态也须校验腕部映射后是否超限，不能复制旧限位数值；`home/retract` 快捷服务仅在同名预设存在时可用，以当前源码 YAML 为准。
+
+控制台现在订阅 `/arm_planner_node/joint_states` 的 `fruit_arm_interfaces/msg/ArmTelemetry`，不再以原始硬件 Status 自行计算 FK。debug 每帧显示 `motors_deg/model_deg/tcp_m/pitch_deg`，jog 和示教使用同一帧校正模型角及 TCP；原地夹爪和 `joints` 硬件角预设保存仍使用该帧原始硬件角，避免二次偏置。消息类型变更必须一起构建接口、planner 和 mission 并重启消费者；旧 standard JointState 订阅者不兼容，话题名称保持不变。
+
 运行中输入 `level info`、`level debug` 或 `level trace` 可立即切换，不需要重启：
 
 | 等级 | 输出 |
@@ -153,9 +166,38 @@ geometry:
   d1: 0.062      # 底座安装面至肩轴中心距
   a2: 0.26       # 肩轴至肘轴距
   a3: 0.26       # 肘轴至腕轴距
-  a4: 0.12       # 腕轴至夹爪采摘作用点中心距
+  a4: 0.12       # 腕轴沿末端局部 X 方向的伸出长度，pitch=0 时此段水平
+  tool_z_offset_m: -0.03  # 真实抓取点的末端局部 Z 偏移，下方为负，随腕部旋转
 ```
 
+`tool_z_offset_m` 是预设 YAML 内可调的固定结构尺寸，不是世界坐标平移，也不是腕电机的零位角偏置。`pitch=0°` 表示 12 cm 伸出段水平，XYZ 指定下方 3 cm 的真实抓取点；腕轴到抓取点的斜线不用于定义 pitch。直接输入、PTP/LIN、笛卡尔预设、jog/FK/示教和 mission 共用同一模型。旧 `planning.cartesian_offset_m` 输入补偿已移除，避免重复补偿。
+
+修改后重新启动 planner、控制台及 mission/相机调试客户端；仅 `reload_presets` 不会刷新几何。现有预设航点不自动改写或迁移，旧模型下记录的坐标需由操作者核对。
+
+以下最小检查仅离线执行数学解算，不启动控制节点、不发送运动请求；先完成构建：
+
+```bash
+source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
+/usr/bin/python3 - <<'PY'
+import math
+import yaml
+from fruit_arm_planner.kinematics import RobotKinematics
+with open('/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml') as stream:
+    geometry = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']['geometry']
+kinematics = RobotKinematics.from_geometry_dict(geometry)
+target = (0.25, 0.0, 0.25)
+ok, joints, message = kinematics.inverse_kinematics(*target, pitch_deg=0.0)
+if not ok:
+    raise RuntimeError(message)
+pose = kinematics.forward_kinematics(*joints)
+assert math.dist(pose['end_effector'][:3], target) < 1e-9
+assert abs(pose['points'][4][2] - pose['points'][3][2]) < 1e-9
+print('TCP (m, rad):', pose['end_effector'], 'wrist (m):', pose['points'][3])
+PY
+```
+
+当前 12 cm / -3 cm 几何下，此水平目标的 TCP 为 `(0.25,0,0.25)m`，腕轴为 `(0.13,0,0.28)m`。这是数学检查，不验证实机标定或关节安全限位。
+
 ### 2. 实机零位偏置与方向校准 (`joint_mapping`)
 - **若某关节旋转方向与预期相反（例如给正角反而向下运动）**：
   将该关节的 `direction` 由 `1.0` 修改为 `-1.0` 即可反相，无需拆改电机连线。
@@ -174,7 +216,6 @@ geometry:
 - `interpolation_step_ms`：路点时间间隔目标值。每条轨迹均从最新实测角的 0 ms 路点开始；预设 `duration_ms` 为逐段时长。
 - `default_dwell_ms`：预设夹爪动作路点未指定 `dwell_ms` 时的默认驻留时间。
 - `mode_start_delay_ms`：发布全部路点后、发布 `Mode=TRAJECTORY` 前的调试等待时间，默认 100 ms。它只是时间缓冲，不能证明通信桥已经完成逐点发送或下位机已经装载全部路点；正式修复仍应由通信桥提供跨消息发送屏障。
-- `cartesian_offset_m`：笛卡尔服务目标的 `[x, y, z]` 平移量（米）。例如 `[0.0, 0.0, -0.03]` 会让服务目标的 Z 减少 3 cm，然后再做 PTP/LIN 解算和限位校验；`cmd_cartesian` 也生效，YAML 笛卡尔预设、关节服务及关节预设不受影响。修改后重启规划节点才生效。
 
 节点启动时从源码目录的 `arm_presets.yaml` 读取几何、四轴偏置/方向/限位、规划时间参数与预设；缺少必需项或基本结构无效时启动失败，不回退到 Python 中的机械参数。修改限位前应核实四轴实测角仍在新范围内，否则规划器会拒绝运动。
 
diff --git a/src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md b/src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md
index 4495daf..4fa594d 100644
--- a/src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md
+++ b/src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md
@@ -32,17 +32,23 @@
 因此 B=q2+q3，P=q2+q3+q4。
 ```
 
-大臂硬件角的正方向为后仰。当前 YAML 中 Q2、Q3、Q4 的方向都为 `+1`，偏置分别为 `-89°`、`-6°`、`-5°`，所以实际使用的双向映射为：
+大臂硬件角的正方向为后仰。当前源码 YAML 中 Q2、Q3、Q4 的方向都为 `+1`，偏置分别为 `-89°`、`-13°`、`-5°`，所以实际使用的双向映射为（现场修改后以 YAML 数值为准，不读取旧注释）：
 
 ```text
-理论角 -> 硬件角：h1=q1，h2=q2-89°，h3=q2+q3-6°，h4=q4-5°
-硬件角 -> 理论角：q1=h1，q2=h2+89°，q3=(h3+6°)-q2，q4=h4+5°
+理论角 -> 硬件角：h1=q1，h2=q2-89°，h3=q2+q3-13°，h4=q4-5°
+硬件角 -> 理论角：q1=h1，q2=h2+89°，q3=(h3+13°)-q2，q4=h4+5°
 模型末端俯仰：P=(h3+6°)+(h4+5°)=h3+h4+11°
 ```
 
-在当前 YAML 模型中，小臂水平对应 `h3=-6°`，大臂竖直对应 `h2=+1°`；这两个标定值与先前观察到的原始反馈零位有差别，需以实机测量核对。要让末端水平，当前映射下应令 `h4=-11°-h3`。腕部安装在小臂上，它的硬件角仍是相对小臂的电机读数。
+在当前 YAML 模型中，小臂水平对应 `h3=-13°`，大臂竖直对应 `h2=+1°`；这两个标定值与先前观察到的原始反馈零位有差别，需以实机测量核对。要让末端水平，当前映射下应令 `h4=-18°-h3`。腕部安装在小臂上，它的硬件角仍是相对小臂的电机读数。
 
-算法的 `(q2,q3,q4)=(0,0,0)` 表示三段连杆全都水平向前，不是硬件全零；按当前映射它对应 `(h2,h3,h4)=(-89°,-6°,-5°)`。四自由度只控制末端的 `x/y/z` 和一个俯仰角，`pitch_deg=0` 要求末端水平，不要求小臂水平。
+算法的 `(q2,q3,q4)=(0,0,0)` 表示三段连杆全都水平向前，不是硬件全零；按当前映射它对应 `(h2,h3,h4)=(-89°,-13°,-5°)`。四自由度只控制末端的 `x/y/z` 和一个俯仰角，`pitch_deg=0` 要求末端水平，不要求小臂水平。
+
+### 固定夹爪下偏与角度标定的区别
+
+`geometry.a4` 是腕轴沿末端局部 X 的伸出段（当前 12 cm）；`geometry.tool_z_offset_m` 是其末端到真实夹爪抓取点 TCP 的局部 Z 偏移（当前 `-0.03 m`）。该下偏随腕部一起旋转，水平时 TCP 低 3 cm，朝下时下偏转为径向偏移。`pitch=0°` 只要求 a4 段水平，**不要求腕轴到 TCP 的斜线水平**，因此不改 `q4=P-B` 或 `joint_mapping` 来补偿斜线角度。
+
+FK 和 IK 都包含这段固定结构；直接笛卡尔目标、笛卡尔预设、LIN 起点/采样、控制台示教及 mission 相机原点统一使用 TCP。旧的服务输入 `+3 cm` 平移及 jog 反向补偿已撤掉。硬件角/连杆角请求保持原角度语义。既有预设数值保持不动，由操作者处理旧坐标；修改几何需重启所有 FK/IK 消费者。
 
 ## 2. 代码实际走过的角度链路
 
@@ -54,7 +60,8 @@ MoveArmCartesian(x,y,z,pitch)
 
 ArmStatus (实测 h1,h2,h3,h4)
   -> JointTransformer.hardware_to_algo：还原 q1,q2,q3,q4
-  -> kinematics.forward_kinematics：LIN 起点的 xyz,pitch
+  -> kinematics.forward_kinematics：LIN 起点及真实 TCP 的 xyz,pitch
+  -> ~/joint_states (ArmTelemetry)：同一帧原始角、模型角、TCP；mission/控制台不再逆 mapping
 
 MoveArmJoints / YAML 的 joints 预设 / 控制台 motors
   -> 输入本来就是硬件角，直接插补；不经过 IK 和角度映射
@@ -71,32 +78,32 @@ YAML 的 link_angles 预设 / 控制台 joints
 
 | 级别 | 发现 | 依据与影响 |
 | --- | --- | --- |
-| 当前配置 | Q3 标定偏置 | 旧配置的 `+90°` 已移除；现场 YAML 随后被标定为 `-6°`。笛卡尔 IK、`link_angles` 预设和控制台 `joints` 都使用此值；直接硬件角入口保持硬件角语义。 |
+| 当前配置 | Q3 标定偏置 | 旧配置的 `+90°` 已移除；源码 YAML 当前数值为 `-13°`。笛卡尔 IK、`link_angles` 预设和控制台 `joints` 都使用配置值；直接硬件角入口保持硬件角语义。 |
 | 已核对 | Q2/Q3/Q4 角度关系 | Q2 硬件零位为大臂竖直，Q3 命令为小臂绝对角；`q4=pitch-(q2+q3)` 对安装在小臂上的相对腕部成立。Q4 的实际同向零位、正方向还需实测确认。 |
 | 说明 | 硬件角与连杆角入口不同 | `move_joints` 服务、YAML `joints` 预设和控制台 `motors` 直接使用硬件角；现有 `zero/home/retract` 属于 `link_angles`，与控制台 `joints` 一样按 YAML 标定换算。笛卡尔 PTP/LIN 在规划节点使用双向映射。 |
 | 说明 | 插补不改变角度定义 | PTP 在硬件角之间使用相同的五次多项式比例插值；映射是仿射关系，端点正确时插补不会单独改变 Q3 的角度定义。LIN 对每个采样位姿重新做 IK；当前 YAML 的 `interpolation_step_ms=200` 对应目标采样间隔约 5 Hz。 |
-| 说明 | `joint_states` 发布的是硬件角 | 节点把 `ArmStatus.joint_1..4` 原样发布为 `JointState.position`，没有转换为算法中的相对肘角。用常规“肘关节相对大臂”模型显示时，Q3 必须另行转换；仓库中未找到本机械臂的 URDF 可核对。 |
-| 待核实 | 运动范围与几何参数 | Q3 上限当前改为 `180°`，但仓库没有实机机械限位依据；`a2/a3=0.26m` 等几何值也需测量轴心距。限位或长度不准会造成拒绝可达目标或末端位置误差。 |
+| 说明 | `joint_states` 同时提供原始角、模型角与 TCP | 原话题类型改为 `fruit_arm_interfaces/ArmTelemetry`；`hardware_joint_state` 保留 h，`model_joint_state` 已还原 q（Q3 为相对肘角），`tcp_position/tcp_pitch_deg` 来自共享 FK。模型角不可再次减 offset。角速度消除 direction 并还原 Q3 耦合，不减常量 offset。该话题不再可直接作为 RViz 的标准 JointState 输入。 |
+| 待核实 | 运动范围与几何参数 | 各轴限位以源码 YAML 为准，但仓库没有实机机械限位依据；`a2/a3=0.26m` 等几何值也需测量轴心距。限位或长度不准会造成拒绝可达目标或末端位置误差。 |
 
 ### 一组可复算的数值
 
-使用配置中的几何 `d1=0.14m`、`a2=a3=0.26m`、`a4=0.08m`，目标 `(x,y,z,pitch)=(0.20,0,0.25,0°)` 的肘上逆解约为：
+以下为未加固定下偏的历史算例（`tool_z_offset_m=0`），不是当前部署几何：`d1=0.14m`、`a2=a3=0.26m`、`a4=0.08m`，目标 `(x,y,z,pitch)=(0.20,0,0.25,0°)` 的肘上逆解约为：
 
 ```text
 理论角：q2=114.27°，q3=-143.51°，q4=29.25°
 大臂硬件角：h2=25.27°（理论角减 89°）
 小臂应有的绝对角：B=q2+q3=-29.25°
-当前配置下发：h3=-35.25°，h4=24.25° -> 去偏置后的模型末端俯仰 0°
+旧 -6° 偏置算例下发：h3=-35.25°，h4=24.25° -> 去偏置后的模型末端俯仰 0°
 历史记录：旧 90° 偏置曾把小臂命令提高 90°；本行仅用于追溯，不代表当前配置。
 ```
 
-`zero` 预设使用 `link_angles`，目标 `(0,0,0,0)°` 按当前配置换算为硬件角 `(h1,h2,h3,h4)=(0,1,-6,-5)°`。原始硬件全零命令只能从 `joints` 硬件角预设、控制台 `motors` 或 `move_joints` 服务发出。上述数值是代码计算结果，不是实机测量。
+`zero` 预设使用 `link_angles`，目标 `(0,0,0,0)°` 按当前配置换算为硬件角 `(h1,h2,h3,h4)=(0,1,-13,-5)°`。原始硬件全零命令只能从 `joints` 硬件角预设、控制台 `motors` 或 `move_joints` 服务发出。上述数值是代码计算结果，不是实机测量。
 
 ## 4. 剩余实机核实顺序
 
 1. 已确认小臂物理水平时 `ArmStatus.joint_3=0 rad`，且大臂单独转动不改变该读数；仍需核实 Q3 正角是否使小臂向上，以及电机角变化与小臂实际角变化是否为 1:1。
 2. 核实 `h4=0` 时末端是否与小臂同向；固定小臂并单独改变 Q4，确认正角是否抬起末端。若不是，需重新标定 Q4 的 `offset_deg` / `direction`，不能用 Q3 偏置补偿。
-3. Q3 偏置当前为 `-6°`，双向耦合公式保持不变。Q3 `limits_deg` 仍须按实际电机角范围确认，不能通过放宽限位掩盖机械限制。
+3. Q3 偏置从源码 YAML 读取，双向耦合公式保持不变。Q3 `limits_deg` 仍须按实际电机角范围确认，不能通过放宽限位掩盖机械限制。
 4. 用已知的静止姿态核对 `ArmStatus` 与实测连杆角，再对比 FK 末端位置；之后才用低速、小范围动作检查笛卡尔 PTP 和 LIN。`move_joints` 服务及 YAML 中 `coordinate_frame: joints` 的第三项是小臂硬件角；`link_angles` 的第四项是末端绝对俯仰角。
 
 当前所有几何尺寸、偏置、方向、关节限位和规划时间参数由 YAML 加载，配置缺失时节点启动失败。仓库内的协议定义与生成回调桩不能证明 Q3 正方向、传动比及 Q4 零位；这些仍以实机测量为准。
diff --git a/src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py b/src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py
index 31eb937..8e09b13 100644
--- a/src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py
+++ b/src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py
@@ -33,6 +33,7 @@ from fruit_arm_interfaces.srv import (
     MoveArmCartesian,
     MoveArmJoints,
 )
+from fruit_arm_interfaces.msg import ArmTelemetry
 
 from fruit_arm_planner.joint_transformer import JointTransformer
 from fruit_arm_planner.kinematics import RobotKinematics
@@ -188,7 +189,7 @@ class ArmPlannerNode(Node):
             String, '~/cmd_preset', self._on_cmd_preset, 10
         )
         self._joint_state_pub = self.create_publisher(
-            JointState, '~/joint_states', 10
+            ArmTelemetry, '~/joint_states', 10
         )
 
         self.get_logger().info('Fruit Arm Planner Node fully ready with Service API.')
@@ -208,10 +209,6 @@ class ArmPlannerNode(Node):
             for key in ('geometry', 'joint_mapping', 'planning', 'presets'):
                 if key not in params:
                     raise ValueError(f'Missing configuration section: {key}')
-            for key in ('d1', 'a2', 'a3', 'a4'):
-                value = float(params['geometry'][key])
-                if not math.isfinite(value) or value <= 0:
-                    raise ValueError(f'Invalid geometry.{key}')
             for key in ('q1', 'q2', 'q3', 'q4'):
                 cfg = params['joint_mapping'][key]
                 limits = cfg['limits_deg']
@@ -226,7 +223,8 @@ class ArmPlannerNode(Node):
             self.planner = TrajectoryPlanner(self.kinematics)
             self.get_logger().info(
                 f"Loaded arm geometry: d1={self.kinematics.d1}m, "
-                f"a2={self.kinematics.a2}m, a3={self.kinematics.a3}m, a4={self.kinematics.a4}m"
+                f"a2={self.kinematics.a2}m, a3={self.kinematics.a3}m, a4={self.kinematics.a4}m, "
+                f"tool_z_offset_m={self.kinematics.tool_z_offset_m}m"
             )
 
             # 2. 关节坐标系映射与零位偏置
@@ -247,12 +245,6 @@ class ArmPlannerNode(Node):
             self.status_max_age_sec = float(planning_cfg['status_max_age_sec'])
             self.execution_timeout_buffer_sec = float(planning_cfg['execution_timeout_buffer_sec'])
             self.mode_start_delay_ms = int(planning_cfg['mode_start_delay_ms'])
-            offset = planning_cfg['cartesian_offset_m']
-            if (not isinstance(offset, (list, tuple)) or len(offset) != 3
-                    or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
-                               and math.isfinite(value) for value in offset)):
-                raise ValueError('cartesian_offset_m must contain three finite numbers (metres)')
-            self.cartesian_offset_m = tuple(float(value) for value in offset)
             if (self.default_duration_ms <= 0 or self.interpolation_step_ms <= 0
                     or self.default_dwell_ms < 0
                     or not 0 < self.status_max_age_sec < 10
@@ -309,18 +301,34 @@ class ArmPlannerNode(Node):
             self.get_logger().warn('Ignoring arm status with invalid joint feedback')
             return
 
-        # 发布标准 JointState
-        js_msg = JointState()
-        js_msg.header.stamp = self.get_clock().now().to_msg()
-        js_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
-        js_msg.position = list(joints)
-        js_msg.velocity = [
-            v if math.isfinite(v) else 0.0 for v in (
-                msg.joint_1_velocity, msg.joint_2_velocity,
-                msg.joint_3_velocity, msg.joint_4_velocity,
-            )
-        ]
-        self._joint_state_pub.publish(js_msg)
+        # 同一实测快照：原始硬件角、逆映射模型角和真实 TCP；不伪造无效速度为零。
+        velocities = (msg.joint_1_velocity, msg.joint_2_velocity,
+                      msg.joint_3_velocity, msg.joint_4_velocity)
+        model_joints = self.transformer.hardware_to_algo(joints)
+        x, y, z, pitch = self.kinematics.forward_kinematics(*model_joints)['end_effector']
+        telemetry = ArmTelemetry()
+        telemetry.header.stamp = self.get_clock().now().to_msg()
+        telemetry.header.frame_id = 'arm_base_link'
+        hardware = telemetry.hardware_joint_state
+        hardware.header = telemetry.header
+        hardware.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
+        hardware.position = list(joints)
+        hardware.velocity = list(velocities)
+        model = telemetry.model_joint_state
+        model.header = telemetry.header
+        model.name = list(hardware.name)
+        model.position = list(model_joints)
+        model.velocity = list(self.transformer.hardware_velocity_to_algo(velocities))
+        telemetry.tcp_position.x = x
+        telemetry.tcp_position.y = y
+        telemetry.tcp_position.z = z
+        telemetry.tcp_pitch_deg = math.degrees(pitch)
+        telemetry.trajectory_id = msg.trajectory_id
+        telemetry.task_status = msg.task_status
+        telemetry.progress = msg.progress
+        telemetry.gripper_state = msg.gripper_state
+        telemetry.error_code = msg.error_code
+        self._joint_state_pub.publish(telemetry)
 
     # ========================================================================
     # 核心下发与看门狗执行
@@ -396,11 +404,10 @@ class ArmPlannerNode(Node):
         traj_id = self.planner.get_new_trajectory_id()
         point_num = len(points)
         actual_traj_duration_ms = points[-1].time_from_start
-        timeout_sec = (actual_traj_duration_ms / 1000.0) + self.execution_timeout_buffer_sec
 
         self.get_logger().info(
             f"Dispatching {description}: traj_id={traj_id}, points={point_num}, "
-            f"duration={actual_traj_duration_ms}ms, timeout={timeout_sec:.1f}s"
+            f"duration={actual_traj_duration_ms}ms"
         )
 
         # 1. 头部声明
@@ -442,11 +449,10 @@ class ArmPlannerNode(Node):
             self._mode_pub.publish(mode)
             dispatched_at = time.monotonic()
 
-        # 4. 同步监视看门狗等待
-        deadline = dispatched_at + timeout_sec
+        # 4. 同步监视硬件状态，等待执行完成或下位机报错
         poll_interval = 0.05  # 50ms (20Hz)
 
-        while time.monotonic() < deadline:
+        while rclpy.ok():
             time.sleep(poll_interval)
             with self._state_lock:
                 reason = self._interrupt_reason
@@ -474,14 +480,7 @@ class ArmPlannerNode(Node):
                     self.get_logger().error(err_msg)
                     return False, traj_id, err_msg
 
-        # 超时未报完成：触发安全停机看门狗
-        self.stop()
-        timeout_msg = (
-            f"Execution timeout for trajectory {traj_id} after {timeout_sec:.1f}s. "
-            f"STOP published; hardware stop is not yet confirmed."
-        )
-        self.get_logger().error(timeout_msg)
-        return False, traj_id, timeout_msg
+        return False, traj_id, "Execution interrupted by node shutdown"
 
     # ========================================================================
     # 服务端点回调函数
@@ -494,15 +493,12 @@ class ArmPlannerNode(Node):
         def build(start):
             duration_ms = int(req.duration_ms) if req.duration_ms > 0 else self.default_duration_ms
             pitch_rad = math.radians(float(req.pitch_deg))
-            request_xyz = (float(req.x), float(req.y), float(req.z))
+            target_xyz = (float(req.x), float(req.y), float(req.z))
             gripper = int(req.gripper_action)
             if (duration_ms <= 0 or duration_ms > 0xFFFFFFFF
-                    or not all(math.isfinite(v) for v in (*request_xyz, pitch_rad))
+                    or not all(math.isfinite(v) for v in (*target_xyz, pitch_rad))
                     or gripper not in (0, 1, 2)):
                 return False, [], 'Invalid Cartesian target, duration or gripper action'
-            target_xyz = tuple(value + delta for value, delta in zip(request_xyz, self.cartesian_offset_m))
-            if not all(math.isfinite(value) for value in target_xyz):
-                return False, [], 'Cartesian target plus offset is not finite'
             if req.linear_path:
                 fk_current = self.kinematics.forward_kinematics(
                     *self.transformer.hardware_to_algo(start)
diff --git a/src/fruit_arm_planner/fruit_arm_planner/joint_transformer.py b/src/fruit_arm_planner/fruit_arm_planner/joint_transformer.py
index 6da8ade..a836723 100644
--- a/src/fruit_arm_planner/fruit_arm_planner/joint_transformer.py
+++ b/src/fruit_arm_planner/fruit_arm_planner/joint_transformer.py
@@ -101,6 +101,18 @@ class JointTransformer:
             self.channels[3].to_algo(q_hw[3]),
         )
 
+    def hardware_velocity_to_algo(
+        self, velocity_hw: Tuple[float, float, float, float]
+    ) -> Tuple[float, float, float, float]:
+        """硬件角速度 -> 算法角速度 (rad/s)；常量零位偏置不参与求导."""
+        q2_velocity = velocity_hw[1] / self.channels[1].direction
+        return (
+            velocity_hw[0] / self.channels[0].direction,
+            q2_velocity,
+            velocity_hw[2] / self.channels[2].direction - q2_velocity,
+            velocity_hw[3] / self.channels[3].direction,
+        )
+
     def check_hardware_limits(
         self, q_hw: Tuple[float, float, float, float]
     ) -> Tuple[bool, str]:
diff --git a/src/fruit_arm_planner/fruit_arm_planner/kinematics.py b/src/fruit_arm_planner/fruit_arm_planner/kinematics.py
index ee80e2f..53a33ce 100644
--- a/src/fruit_arm_planner/fruit_arm_planner/kinematics.py
+++ b/src/fruit_arm_planner/fruit_arm_planner/kinematics.py
@@ -10,7 +10,8 @@ class RobotKinematics:
     """
     4自由度机械臂纯几何运动学求解器 (4-DOF Robot Arm Kinematics)
     构型：偏航基座(Yaw) + 肩部俯仰(Pitch) + 肘部俯仰(Pitch) + 腕部俯仰(Pitch)
-    基准状态：各关节为 0 时，机械臂各连杆沿 X 轴水平伸直。
+    基准状态：各关节为 0 时，各运动连杆沿 X 轴水平伸直。
+    抓取点另有随腕部旋转的局部 Z 偏移；pitch 始终描述 a4 段的方向。
     """
 
     def __init__(
@@ -18,13 +19,21 @@ class RobotKinematics:
         d1: float,  # 基座高度 (m)
         a2: float,  # 大臂长度 (m)
         a3: float,  # 小臂长度 (m)
-        a4: float,  # 末端/夹爪长度 (m)
+        a4: float,  # 腕轴沿末端局部 X 方向的伸出长度 (m)
+        tool_z_offset_m: float,  # 抓取点的末端局部 Z 偏移 (m，下方为负)
         joint_limits: Optional[Dict[str, Tuple[float, float]]] = None,
     ):
+        if not all(math.isfinite(v) and v > 0 for v in (d1, a2, a3, a4)):
+            raise ValueError('Arm link dimensions must be finite and positive')
+        if (isinstance(tool_z_offset_m, bool)
+                or not isinstance(tool_z_offset_m, (int, float))
+                or not math.isfinite(tool_z_offset_m)):
+            raise ValueError('geometry.tool_z_offset_m must be a finite number (metres)')
         self.d1 = d1
         self.a2 = a2
         self.a3 = a3
         self.a4 = a4
+        self.tool_z_offset_m = float(tool_z_offset_m)
         self.joint_limits = joint_limits
 
     @classmethod
@@ -35,6 +44,7 @@ class RobotKinematics:
             a2=float(geo_dict["a2"]),
             a3=float(geo_dict["a3"]),
             a4=float(geo_dict["a4"]),
+            tool_z_offset_m=geo_dict["tool_z_offset_m"],
         )
 
     def forward_kinematics(
@@ -60,9 +70,14 @@ class RobotKinematics:
         z4 = z3 + self.a4 * math.sin(pitch)
         p4 = (r4 * math.cos(q1), r4 * math.sin(q1), z4)
 
+        # TCP 固定在 a4 段末端的局部 Z 偏移处，不改变夹爪方向 pitch。
+        r_tcp = r4 - self.tool_z_offset_m * math.sin(pitch)
+        z_tcp = z4 + self.tool_z_offset_m * math.cos(pitch)
+        tcp = (r_tcp * math.cos(q1), r_tcp * math.sin(q1), z_tcp)
+
         return {
-            "end_effector": (p4[0], p4[1], p4[2], pitch),
-            "points": [p0, p1, p2, p3, p4],
+            "end_effector": (tcp[0], tcp[1], tcp[2], pitch),
+            "points": [p0, p1, p2, p3, p4, tcp],
         }
 
     def inverse_kinematics(
@@ -77,8 +92,8 @@ class RobotKinematics:
     ) -> Tuple[bool, Tuple[float, float, float, float], str]:
         """逆运动学 (IK)：根据末端坐标 (x, y, z) 与期望末端俯仰角求解理论关节角 (q1, q2, q3, q4).
 
-        :param x, y, z: 末端目标坐标 (单位: m)
-        :param pitch: 期望末端俯仰角 (弧度，水平为0，向下为负，向上为正)
+        :param x, y, z: 真实夹爪抓取点 TCP 的目标坐标 (单位: m)
+        :param pitch: a4 段俯仰角 (弧度，水平为0，向下为负，向上为正)，不是腕轴到 TCP 的连线角
         :param pitch_deg: 期望末端俯仰角 (角度制，若指定则优先使用)
         :param elbow_up: 是否为肘关节高位构型 (Elbow-Up, 默认 True)
         :param force_elbow_up: 是否强制只使用高位构型 (默认 True)
@@ -103,8 +118,10 @@ class RobotKinematics:
 
         for target_pitch in candidate_pitches:
             # 2. 腕关节在 (r, z) 平面内的位置 (rw, zw)
-            rw = r - self.a4 * math.cos(target_pitch)
-            zw = (z - self.d1) - self.a4 * math.sin(target_pitch)
+            rw = (r - self.a4 * math.cos(target_pitch)
+                  + self.tool_z_offset_m * math.sin(target_pitch))
+            zw = ((z - self.d1) - self.a4 * math.sin(target_pitch)
+                  - self.tool_z_offset_m * math.cos(target_pitch))
 
             # 肩-腕直线距离 d
             d_sq = rw * rw + zw * zw
diff --git a/src/fruit_arm_planner/test/cartesian_console.py b/src/fruit_arm_planner/test/cartesian_console.py
index 47cf048..1df64d7 100644
--- a/src/fruit_arm_planner/test/cartesian_console.py
+++ b/src/fruit_arm_planner/test/cartesian_console.py
@@ -11,8 +11,31 @@ import sys
 import tempfile
 import threading
 import time
+from datetime import datetime
 from pathlib import Path
 
+# Record through a PTY so readline, curses and native stderr keep working.
+if __name__ == '__main__':
+    session_log = os.environ.pop('_FRUIT_ARM_CONSOLE_SESSION_LOG', None)
+    if session_log is None:
+        log_dir = Path(__file__).resolve().parent / 'log'
+        log_path = log_dir / datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f.log')
+        try:
+            log_dir.mkdir(exist_ok=True)
+            with log_path.open('x', encoding='utf-8'):
+                pass
+            env = os.environ.copy()
+            env['_FRUIT_ARM_CONSOLE_SESSION_LOG'] = str(log_path)
+            command = shlex.join([sys.executable, *sys.argv])
+            os.execve('/usr/bin/script', [
+                'script', '--quiet', '--return', '--flush',
+                '--command', command, str(log_path),
+            ], env)
+        except OSError as exc:
+            print(f'Cannot record console session: {exc}', file=sys.stderr)
+            raise SystemExit(1) from exc
+    print(f'Console session log: {session_log}', flush=True)
+
 import yaml
 
 import rclpy
@@ -23,9 +46,9 @@ from rclpy.signals import SignalHandlerOptions
 from std_srvs.srv import Trigger
 
 from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
+from fruit_arm_interfaces.msg import ArmTelemetry
 from fruit_arm_planner.arm_planner_node import validate_presets
 from fruit_arm_planner.joint_transformer import JointTransformer
-from fruit_arm_planner.kinematics import RobotKinematics
 from fruit_hardware_interfaces.msg import ArmControlMode, ArmStatus, TrajectoryHeader, TrajectoryPoint
 
 
@@ -51,7 +74,7 @@ HELP = (
     'preset NAME   Execute a configured preset (zero, home, retract, pick_sequence)\n'
     'jog       Keyboard jog/teach: arrows=XY, a/z=Z, s/x=pitch, c/o/h=gripper, Space=save\n'
     'reload    Reload all presets in the running planner (only when not moving)\n'
-    'status    Show the latest controller status and measured joint angles\n'
+    'status    Show raw motor angles, mapped model angles and real TCP xyz/pitch\n'
     'stop      Request a controlled stop\n'
     'estop     Request an emergency stop\n'
     'level [info|debug|trace]  Show or change console log detail\n'
@@ -74,7 +97,6 @@ class ArmConsole(Node):
         self.log_level = log_level
         self.config_path = config_path
         self.transformer = JointTransformer.from_config_dict(params['joint_mapping'])
-        self.kinematics = RobotKinematics.from_geometry_dict(params['geometry'])
         self.planning = params['planning']
         self._output_lock = threading.Lock()
         self._terminal_ui = False
@@ -85,7 +107,8 @@ class ArmConsole(Node):
         self.stop_client = self.create_client(Trigger, '/arm_planner_node/stop')
         self.estop_client = self.create_client(Trigger, '/arm_planner_node/emergency_stop')
         self.reload_client = self.create_client(Trigger, '/arm_planner_node/reload_presets')
-        self.create_subscription(ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
+        self.create_subscription(
+            ArmTelemetry, '/arm_planner_node/joint_states', self._on_status, 10)
         tx_qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE)
         self.create_subscription(
             TrajectoryHeader, '/fruit_hardware_bridge/arm/trajectory_header',
@@ -127,14 +150,18 @@ class ArmConsole(Node):
             return  # The curses screen displays telemetry without overwriting operation results.
         if self.log_level == 'info' and not changed:
             return
-        joints_deg = ', '.join(
-            f'{math.degrees(q):.1f}' for q in
-            (msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4)
-        )
+        hardware_deg = ', '.join(
+            f'{math.degrees(q):.1f}' for q in msg.hardware_joint_state.position)
+        model_deg = ', '.join(
+            f'{math.degrees(q):.1f}' for q in msg.model_joint_state.position)
+        tcp = msg.tcp_position
         self._emit(
             'RX STATUS',
             f'id={msg.trajectory_id} state={STATUS_NAMES.get(msg.task_status, msg.task_status)} '
-            f'progress={msg.progress:.1f}% joints_deg=[{joints_deg}] '
+            f'progress={msg.progress:.1f}% motors_deg=[{hardware_deg}] '
+            f'model_deg=[{model_deg}] '
+            f'tcp_m=[{tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f}] '
+            f'pitch_deg={msg.tcp_pitch_deg:.2f} '
             f'gripper={msg.gripper_state} error={msg.error_code}',
         )
 
@@ -172,18 +199,25 @@ class ArmConsole(Node):
             msg = self._status
             age = time.monotonic() - self._status_at
         if msg is None:
-            print('No arm status received. Check the serial bridge and lower controller.')
+            print('No arm telemetry received. Check planner, serial bridge and lower controller.')
             return
-        joints = (msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4)
+        joints = msg.hardware_joint_state.position
         angles = ', '.join(f'{math.degrees(q):.1f}' for q in joints)
-        q_algo = self.transformer.hardware_to_algo(joints)
+        q_algo = msg.model_joint_state.position
+        if len(q_algo) != 4:
+            print('Invalid model joint telemetry: expected four angles')
+            return
         link_angles = (q_algo[0], q_algo[1] - math.pi / 2,
                        q_algo[1] + q_algo[2], q_algo[1] + q_algo[2] + q_algo[3])
         print(f'Status: {STATUS_NAMES.get(msg.task_status, msg.task_status)}, '
               f'trajectory={msg.trajectory_id}, progress={msg.progress:.2f}, '
               f'error={msg.error_code}, age={age:.2f}s')
         print(f'Motors (deg): [{angles}], gripper={msg.gripper_state}')
+        print('Model joints (deg): [' + ', '.join(f'{math.degrees(q):.1f}' for q in q_algo) + ']')
         print('Link angles (deg): [' + ', '.join(f'{math.degrees(q):.1f}' for q in link_angles) + ']')
+        tcp = msg.tcp_position
+        print(f'TCP (m): [{tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f}], '
+              f'pitch={msg.tcp_pitch_deg:.2f} deg')
 
     def move(self, args):
         if not 4 <= len(args) <= 7:
@@ -327,15 +361,24 @@ class ArmConsole(Node):
             age = time.monotonic() - self._status_at
         if status is None or age > float(self.planning['status_max_age_sec']):
             raise ValueError('Fresh four-joint feedback is required')
-        joints = (status.joint_1, status.joint_2, status.joint_3, status.joint_4)
-        if not all(math.isfinite(q) for q in joints):
-            raise ValueError('Invalid joint feedback (NaN/Inf)')
+        if status.header.frame_id != 'arm_base_link':
+            raise ValueError('TCP telemetry must be in arm_base_link')
+        hardware = status.hardware_joint_state
+        model = status.model_joint_state
+        names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
+        if (hardware.name != names or model.name != names
+                or len(hardware.position) != 4 or len(model.position) != 4):
+            raise ValueError('Invalid telemetry joint names or angle count')
+        joints = tuple(hardware.position)
+        algo = tuple(model.position)
+        tcp = status.tcp_position
+        pose = [tcp.x, tcp.y, tcp.z, status.tcp_pitch_deg]
+        if not all(math.isfinite(v) for v in (*joints, *algo, *pose)):
+            raise ValueError('Invalid joint/TCP telemetry (NaN/Inf)')
         ok, message = self.transformer.check_hardware_limits(joints)
         if not ok:
             raise ValueError(message)
-        algo = self.transformer.hardware_to_algo(joints)
-        x, y, z, pitch = self.kinematics.forward_kinematics(*algo)['end_effector']
-        return status, joints, algo, [x, y, z, math.degrees(pitch)]
+        return status, joints, algo, pose
 
     def save_point(self, name, snapshot, gripper_action=0):
         """Insert only the new waypoint/preset, preserving the rest of the YAML text."""
@@ -501,8 +544,7 @@ class ArmConsole(Node):
                                                  f'In-place gripper {gripper_names[active_gripper]}')
                 else:
                     request = MoveArmCartesian.Request()
-                    request.x, request.y, request.z = [
-                        value - delta for value, delta in zip(target[:3], self.planning['cartesian_offset_m'])]
+                    request.x, request.y, request.z = target[:3]
                     request.pitch_deg = target[3]
                     request.duration_ms = 0  # The running planner's configured default.
                     request.gripper_action = 0
diff --git a/src/fruit_mission/README.md b/src/fruit_mission/README.md
index b386ac4..7081177 100644
--- a/src/fruit_mission/README.md
+++ b/src/fruit_mission/README.md
@@ -26,26 +26,30 @@ A 固定路线要求 A1～A4 都存在且类型为 goal；新增/删减采摘点
 | 积木 | 参数与职责 |
 | --- | --- |
 | `navigate(ctx, waypoints, side=None)` | 明确导航点；有 side 时导航与 Look 并行。失败清理待消费 Look，安全恢复并返回 False。 |
-| `pick_a(ctx, side=...)` | 当前点单侧 Look 完成 → 实测停稳 → 识别成熟目标 → CLOSE/驻留 → Harvest/OPEN/驻留 → Home。无导航、无重复抓取。 |
-| `pick_b_tree(ctx, waypoint=..., target=..., side=...)` | 每树观察一次、比较二维码品种、选择最多两个成熟目标，先冻结全部基座请求，再逐个抓取/Harvest/Home。 |
+| `pick_a(ctx, side=...)` | 当前点单侧 Look 完成 → 开启 harvest 观察窗口 → 识别成熟目标 → CLOSE/驻留 → Harvest/OPEN/驻留 → Home。无导航、无重复抓取。 |
+| `pick_b_tree(ctx, tree_name=..., target=..., side=...)` | 每树观察一次、开启 harvest 窗口比较二维码品种、选择最多两个成熟目标，首次抓取前先冻结全部基座请求并关闭窗口，再逐个抓取/Harvest/Home。 |
 | `navigate_c(ctx, task)` | 复用现有 C 几何规划，记录路线和 camera_side；失败的记录在恢复前由分区流程保存。 |
 | `pick_c(ctx, task, side=...)` | 当前二维码项的单次识别/采摘、结果记录和播报；不增加方向复核。 |
-| `scan_b_qr / scan_c_qr(ctx, arrived=...)` | 已到点才执行 QR 姿态、停稳和扫码；到点/扫码失败均用 mission 自有固定计划，保存、播报并恢复。 |
+| `scan_b_qr / scan_c_qr(ctx, arrived=...)` | 已到点且 QR 姿态执行完成后开启 QR 扫码窗口；到点/扫码失败均用 mission 自有固定计划，保存、播报并恢复。 |
 
 `arrived` 使用前一个 `navigate` 的返回值；导航失败已经执行过一次安全恢复，
 扫码函数只使用兜底计划，不再为同一导航失败重复尝试 Home。
 
 `MissionContext` 只保留 ROS 反馈、导航发送、感知窗口、planner 原语及安全交接；
 品种判断和语音格式在 `decisions.py`，C 路线数学继续在 `waypoint_route.py`。
-原单点调试入口也调用相同的 `observe_and_pick` / `pick_b_tree` 积木；B 无可采目标
-或任一选中目标工序失败时不返回假成功。它是人工硬件工具，不能作为自动测试运行。
+独立单点调试入口 `test_camera_arm_target.sh`（`camera_arm_target_client`）不依赖导航，按用户确认在抓取及驻留成功后调用对应侧 `left_store` 或 `right_store` 执行放果工序，再调用 `home` 并等待完成；不是停在目标点的工具，也不重复调用另一套 Harvest。正式 mission 保持 CLOSE→配置的 Harvest 放果工序→Home。它是人工硬件工具，不能作为自动测试运行。
 原 `mission_flow.py` 和作为判断帮助文件的 `logic.py` 已移除，避免两个主流程来源。
 
-调用处只显示比赛参数。真实 Look/QR/Harvest/Home 预设名称、超时和停稳阈值仍在
+调用处只显示比赛参数。真实 Look/QR/Harvest/Home 预设名称、超时仍在
 `config/fruit_mission.yaml`；姿态、速度、标定在对应 planner/相机源码 YAML，
-不把这些值复制进比赛主流程。A 左右调用顺序现在直接在 `Logic.py`，不再使用
+不把这些值复制进比赛主流程。开窗前及窗口内的底盘与机械臂静止由现场物理保证，
+移除了原开窗前的 `stable_stop` 阻塞等待；未知运动结果时的 STOP、取消与恢复前机械臂停稳检查完整保留。A 左右调用顺序现在直接在 `Logic.py`，不再使用
 `a_observation_order` 配置。
 
+窗口结果在入缓存前核对 ID、用途及合法窗后图像时间；扫码计划另核对区域，非对象 JSON 与非法时间忽略后继续等待或使用既有固定兜底。
+正式 `harvest_left`/`harvest_right` 按用户确认映射到 planner 已有 `left_store`/`right_store`，复用其放果航点，不新建或修改真实姿态；执行完成后再调用 `home`。
+B 树收到 `targets_ready=true` 的空列表表示完成但无果实，立即按空树跳过；仅未收到完成结果才算识别超时。关闭窗口保留开启时上下文。
+
 ## 比赛和单区调试共用同一套函数
 
 节点启动时从明确的入口表选择一次函数；`main()` 没有模式分支，不做动态导入或流程图引擎。
@@ -71,6 +75,13 @@ source scripts/setup_fruit_runtime.bash
 源码修改后需重新构建 `fruit_mission` 并重启节点。配置继续从源码路径读取，
 不用修改 install 中的副本。
 
+启动脚本统一通过 `stack_readiness_client bridge-start-required` 检查串口桥：
+标准输出为 `true`（缺失，需要启动）或 `false`（已存在，复用）。节点、
+ArmStatus 或 ready 发布者均可识别已有桥，`ready=false` 不代表缺失；
+重复节点/发布者或错误消息类型会让检查失败。检查只读，不命令硬件。
+单点客户端的 `--require-stack-absent` 允许已有串口桥，但仍拒绝已有感知容器、
+planner 和 mission；复用桥不由当前脚本清理，也不更改其配置。
+
 ## 失败不阻断比赛，但绝不授权未知运动
 
 - 普通观察、采摘、二维码或外围节点失败，跳过当前操作，继续 A/B/C 软件流程。
diff --git a/src/fruit_mission/config/camera_extrinsic.yaml b/src/fruit_mission/config/camera_extrinsic.yaml
index a2aeb02..b4aa7ac 100644
--- a/src/fruit_mission/config/camera_extrinsic.yaml
+++ b/src/fruit_mission/config/camera_extrinsic.yaml
@@ -2,8 +2,9 @@
 # E: 夹爪抓取中心，局部 +X 前、+Y 左、+Z 上（随末端姿态旋转）。
 # S: D435i 底部螺纹孔表面圆心；S 的轴与 E 同向。
 # C: RGB 光学原点，+X 右、+Y 下、+Z 前。
-# 用户测量 S -> E = [75.413, 0, 66.726] mm；此处保存反向 E -> S。
-translation_end_to_screw_mm: [-75.413, 0.0, -66.726]
+# 用户确认 S 在 E 后方 75.413 mm、局部上方 66.726 mm（36.726 + 30）。
+# E -> S = [-75.413, 0, +66.726] mm；反向 S -> E 的 Z 为负。
+translation_end_to_screw_mm: [-75.413, 0.0, 66.726]
 
 # 沿用 tmp/vison/Engineer/APPLICATION/camera_target_transform_config.h
 # 的固定机身/光学数据；未在本次任务中独立验证其官方来源。
@@ -13,7 +14,7 @@ rotation_end_from_rgb:
   - [-0.99999200, 0.00320883, -0.00240529]
   - [-0.00321453, -0.99999200, 0.00236935]
 
-# 复用现有几何尺寸和硬件角映射，只读，不修改 planner。
+# 复用现有规划配置与限位参数，只读，不修改 planner。
 arm_config_path: /home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml
 camera_frame: camera_color_optical_frame
 joint_state_topic: /arm_planner_node/joint_states
@@ -24,6 +25,6 @@ service_timeout_sec: 10.0
 
 # mission 在目标末点闭爪，后续 Harvest/Home 由上位机调用预设。
 motion:
-  pitch_deg: null  # null: 保持图像时刻的末端俯仰角；数值: 指定对地俯仰角（度）
-  duration_ms: 0  # 0: 使用 planner 的默认耗时
-  linear_path: false
+  pitch_deg: 0.0  # 指定对地绝对俯仰角（度），0.0 为相对于 base 严格水平抓取
+  duration_ms: 1500  # 1500ms 平滑探出抓取
+  linear_path: false  # false: 关节空间 PTP 平滑规划
diff --git a/src/fruit_mission/config/fruit_mission.yaml b/src/fruit_mission/config/fruit_mission.yaml
index 8312335..a381419 100644
--- a/src/fruit_mission/config/fruit_mission.yaml
+++ b/src/fruit_mission/config/fruit_mission.yaml
@@ -2,16 +2,9 @@ fruit_mission:
   ros__parameters:
     qr_plan_timeout_sec: 10.0
     qr_fallback_file: "/home/rc4/fruit_ws/src/fruit_mission/config/qr_fallback.yaml"
-    active_mode_timeout_sec: 10.0
     camera_detection_timeout_sec: 10.0
-    odometry_topic: "/odometry"
     stationary:
-      linear_speed_mps: 0.02
-      angular_speed_rps: 0.05
       joint_speed_rps: 0.05
-      duration_sec: 0.5
-      odometry_max_age_sec: 0.5
-      timeout_sec: 10.0
     c_distribution_output_dir: "/home/rc4/fruit_ws/tmp/fruit_mission"
     waypoint_file: "/home/rc4/fruit_ws/src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml"
     mission_flow: "full"
@@ -24,30 +17,30 @@ fruit_mission:
     c_graph_max_start_distance: 0.75
     c_qr_announce_lead_sec: 10.0
 
-    perception_mode_topic: "/fruit_mission/perception_mode"
+    observation_window_topic: "/fruit_mission/observation_window"
     current_area_topic: "/fruit_mission/current_area"
-    active_mode_topic: "/fruit_perception/active_mode"
     expected_fruit_type_topic: "/fruit_mission/expected_fruit_type"
     decision_topic: "/fruit_perception/decision"
     b_plan_topic: "/fruit/b_pick_plan"
     c_plan_topic: "/fruit/c_pick_plan"
     camera_extrinsic_config: "/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml"
+    # Target offset in arm base frame (meters): [x, y, z]
+    # Applied after optical->base conversion, before MoveArmCartesian.
+    # X and Z are added directly. Y is lateral magnitude; its sign is automatically
+    # determined by the sign of the transformed target base_y (+Y for left, -Y for right).
+    target_offset_m: [0.0, 0.03, -0.015]
     # Names only: define the actual joint trajectories in arm_presets.yaml.
     # Local failures skip the operation; never substitute a placeholder pose.
     arm_presets:
       observation_left: "left_look"
       observation_right: "right_look"
-      harvest_left: "left_harvest"
-      harvest_right: "right_harvest"
+      harvest_left: "left_store"
+      harvest_right: "right_store"
       qr: "qr_look"
       home: "home"
 
     tts:
-      piper_executable: "/home/rc4/fruit_ws/.venv/piper-tts/bin/piper"
-      model_path: "/home/rc4/fruit_ws/.cache/piper-tts/zh_CN-huayan-medium/zh_CN-huayan-medium.onnx"
-      config_path: "/home/rc4/fruit_ws/.cache/piper-tts/zh_CN-huayan-medium/zh_CN-huayan-medium.onnx.json"
+      manifest_path: "/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml"
       aplay_executable: "/usr/bin/aplay"
-      wav_path: "/home/rc4/fruit_ws/tmp/fruit_mission/tts_latest.wav"
-      qr_length_scale: 0.8
-      generation_timeout_sec: 10.0
+      wav_path: "/home/rc4/fruit_ws/tmp/fruit_mission/stitched_speech.wav"
       playback_timeout_sec: 10.0
diff --git a/src/fruit_mission/config/script_ros_clients.yaml b/src/fruit_mission/config/script_ros_clients.yaml
index 7cbf6d0..82b195e 100644
--- a/src/fruit_mission/config/script_ros_clients.yaml
+++ b/src/fruit_mission/config/script_ros_clients.yaml
@@ -3,25 +3,19 @@
     bridge_ready_topic: "/auto_serial_bridge/ready"
     config_path: "/home/rc4/fruit_ws/src/fruit_mission/config/fruit_mission.yaml"
     camera_extrinsic_config: "/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml"
-    perception_mode_topic: "/fruit_mission/perception_mode"
-    active_mode_topic: "/fruit_perception/active_mode"
+    observation_window_topic: "/fruit_mission/observation_window"
     current_area_topic: "/fruit_mission/current_area"
     decision_topic: "/fruit_perception/decision"
 
     bridge_ready_timeout_sec: 10.0
     endpoint_timeout_sec: 10.0
-    active_mode_timeout_sec: 10.0
     detection_timeout_sec: 10.0
     current_area_delivery_timeout_sec: 1.0
-    perception_mode_min_subscribers: 2
-    cleanup_mode_timeout_sec: 2.0
     graph_discovery_timeout_sec: 1.0
     camera_conflicting_nodes:
       - "/fruit_perception_container"
-      - "/camera_mode_manager"
       - "/serial_controller"
       - "/arm_planner_node"
 
     follow_waypoints_action: "/follow_waypoints"
     diagnostics_topic: "/diagnostics"
-    active_mode_expected: "OFF"
diff --git a/src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md b/src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
index 6336005..fbdfd9a 100644
--- a/src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
+++ b/src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
@@ -7,7 +7,7 @@ FruitDecision（RGB 光学坐标，米，保留图像 header）
   -> mission 选中目标
   -> CameraTargetPlannerClient
        camera -> 夹爪中心 E（mm）
-       实测硬件关节角 -> JointTransformer -> RobotKinematics FK
+       planner ArmTelemetry -> 已校正模型偏航、真实 TCP xyz/pitch
        E -> 机械臂固定基座 B（mm）
        B 坐标 / 1000 -> MoveArmCartesian（米）
   -> /arm_planner_node/move_cartesian
@@ -17,11 +17,11 @@ FruitDecision（RGB 光学坐标，米，保留图像 header）
 正式 `MissionContext` 和单相机调试客户端共用同一入口，不再发送旧 `ArmTarget`。
 正式 mission 的左右观察、二维码、收臂、恢复动作也通过该入口调用
 `/arm_planner_node/execute_preset`，与目标运动共用结果校验及未知结果时的 STOP 处理。
-`fruit_arm_planner` 源码和配置不因本任务而修改；FK 和硬件角映射直接复用其现有实现，不重复实现 IK 或轨迹规划。
+FK 和反馈逆映射在 `fruit_arm_planner` 中完成，mission 直接读取遥测，不重复减偏置、FK、IK 或轨迹规划；固定夹爪下偏由 planner 的 geometry 统一定义。
 
-用户确认 DETECT 时机械臂静止。**整个多帧观测窗口及观测结束到提交请求之间必须维持同一观测姿态**，才可用提交前的新鲜实测关节角做 FK；不缓存/插值历史姿态。若以后改为运动中检测，需要重新设计同步，不能继续沿用此假设。
+用户确认观察窗口期间底盘与机械臂物理静止。**整个多帧观测窗口及观测结束到提交请求之间必须维持同一观测姿态**，才可用提交前的新鲜实测 TCP 遥测转换目标；不缓存/插值历史姿态。若以后改为运动中检测，需要重新设计同步，不能继续沿用此假设。
 
-`joint_states` 来自下位机反馈，经 planner 发布，单位 rad；`joint_3` 是小臂绝对角，不是相对肘角。先通过 `hardware_to_algo` 消除 direction/offset，再做 FK。FK 输出原点为夹爪作用中心，俯仰为 `q2+q3+q4`，偏航为 `q1`。
+`/arm_planner_node/joint_states` 的消息类型现为 `fruit_arm_interfaces/msg/ArmTelemetry`，话题名不变。`hardware_joint_state` 保留原始硬件角，`model_joint_state` 已消除 direction/offset 并还原相对肘角，单位 rad。mission 只用同一帧模型 Q1 和 `tcp_position/tcp_pitch_deg`（m、度）做 E -> B，不再自行逆 mapping 或 FK；输入校验仍要求原始角在硬件限位内，模型角/TCP 有限且反馈新鲜。原始 ArmStatus 订阅保留给停稳、安全恢复和硬件角驻留服务，不把其角度误当模型角。
 
 ## 新外参评估（单位 mm）
 
@@ -29,13 +29,16 @@ FruitDecision（RGB 光学坐标，米，保留图像 header）
 E 和 S 的轴同向：局部 X 前、Y 左、Z 上；这些轴随末端运动，并非永远与基座/地面平行。
 C 的轴为 X 右、Y 下、Z 前。
 
-用户确认的测量是 **S 指向 E**：
+用户确认螺纹孔 S 在抓取中心 E 的后方 75.413 mm、末端局部上方
+66.726 mm。建模图中，S 比腕轴高 36.726 mm，E 比腕轴低 30 mm，
+因此 E 指向 S 的局部 Z 为 `+(36.726 + 30) = +66.726 mm`；
+这些是末端局部尺寸，不是运动后的基座轴投影。对应的双向平移为：
 
 ```text
-S -> E = [75.413, 0, 66.726]
-E -> S = [-75.413, 0, -66.726]
+S -> E = [75.413, 0, -66.726]
+E -> S = [-75.413, 0, 66.726]
 S -> C = [10.144894, 32.458360, 12.615321]
-E -> C = [-65.268106, 32.458360, -54.110679]
+E -> C = [-65.268106, 32.458360, 79.341321]
 ```
 
 后两个机身/光学数据及旋转沿用
@@ -55,7 +58,7 @@ R_E_C =
 [-0.99999200   0.00320883  -0.00240529]
 [-0.00321453  -0.99999200   0.00236935]
 
-t_E_C_mm = [-65.268106, 32.458360, -54.110679]
+t_E_C_mm = [-65.268106, 32.458360, 79.341321]
 ```
 
 旋转矩阵通过正交性和 determinant=+1 的舍入容差校验。`T_E_C` 的含义是**把 C 系点转换到 E 系**，不要因命名中的 E/C 次序而误求逆。
@@ -64,36 +67,48 @@ t_E_C_mm = [-65.268106, 32.458360, -54.110679]
 
 ```text
 R_B_E = Rz(yaw) * Ry(-pitch)
-P_B_mm = R_B_E * P_E_mm + 1000 * FK_position_m
+P_B_mm = R_B_E * P_E_mm + 1000 * planner_TCP_position_m
 ```
 
 所以相机到夹爪的固定矩阵本身并不足以提交 planner；必须加上观测姿态的第二步。
 
+此次修正了 E -> S 的局部 Z 符号。原值 `-66.726 mm` 把螺纹孔放在了抓取中心下方；
+末端俯仰约 -91.45°、偏航约 90° 时，局部 Z 几乎沿基座 +Y，符号错误造成约
+133.41 mm 的 Y 偏差。对同一帧光学点 `(0.008528585, 0.062558666, 0.314)` m
+和实测 TCP 姿态复算，水果 Base Y 从 0.131326 m 修正为 0.264736 m，
+与尺量约 0.26 m 接近。这是已采集数据的离线复算，仍需重启检测工具后现场复核。
+只修改外参平移，不修改实时 TCP、关节映射或旋转矩阵，也不再额外叠加 30 mm 工具偏移。
+
 ## 配置和动作语义
 
 配置：`/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml`。
 `fruit_mission.yaml` 和 `script_ros_clients.yaml` 用 `camera_extrinsic_config` 指定源文件。
 静态平移均为毫米；ROS 感知和 `MoveArmCartesian` 仍为米，入口乘 1000，基座坐标提交前除 1000。
 
-- `arm_config_path`：读取 planner 的 geometry、joint_mapping 和状态新鲜度；修改标定后须同时重启 planner 和 mission，保持一致。
+- `arm_config_path`：读取 planner 的 joint_mapping 限位、状态新鲜度和驻留时间；geometry/FK 在 planner 内处理。修改标定后须同时重启 planner 和 mission，保持一致。
 - `camera_frame`：只接受该 RGB 光学 frame 的点；不通过改标签伪造坐标转换。
-- `joint_state_topic`：四轴实测反馈。缺失、过期、非有限数、关节名称错误或越限时不提交请求。
-- `motion.pitch_deg: null`：默认保持观测姿态的绝对末端俯仰；数值则指定目标俯仰角（度）。
-- `motion.duration_ms: 0`：使用 planner 的默认耗时。
-- `motion.linear_path: false`：PTP；true 为 LIN，含义同 planner API。
+- `joint_state_topic`：原字段名保留，指向现有 ArmTelemetry 话题；缺失、过期、frame 不是 `arm_base_link`、原始/模型角或 TCP 非有限、关节名称错误或硬件角越限时不提交请求。不得把消息内模型角再减一次偏置。
+- `motion.pitch_deg: 0.0`：指定对地绝对俯仰角（度），0.0 为末端对 base 严格水平抓取。
+- `motion.duration_ms: 1500`：抓取探出耗时 1500ms。
+- `motion.linear_path: false`：关节空间 PTP 平滑规划（false 为 PTP，true 为笛卡尔 LIN）。
 - 视觉目标末点使用 CLOSE，保留转换出的 XYZ，不沿用旧固件的 Z=-100mm、pitch=-90°，不编造接近或提拉坐标。
 
 抓取流程为目标 CLOSE → 驻留 → 对应侧 Harvest（末点 OPEN、驻留）→ Home。
 `MoveArmCartesian` 没有驻留字段，因此 CLOSE 完成后复用 `move_joints`：以新鲜、有限、在限位内的实测原始四轴角为目标，HOLD 保持 `planning.default_dwell_ms`，仍由 planner 生成并执行轨迹，不在 mission 中生成轨迹或用 sleep 假冒执行完成。该驻留值须为正数。
 Harvest/Home 都必须返回成功才记录本次工序完成；这不是视觉证实果实实际入筐。
 
-planner 仍会叠加自身 `planning.cartesian_offset_m`。本次核对时其值是 `[0, 0, 0.03] m`，所以实际 IK 目标比 mission 发出的坐标高 30 mm。这里不自动抵消、不修改该配置；如希望严格到几何目标，现场应确认该偏置是否需要。外参 YAML 修改需重启 mission 客户端，不会自动热重载。
+planner 的 FK/IK 现在包含 `geometry.tool_z_offset_m` 固定结构偏移（当前 `-0.03 m`，末端局部 Z 方向，随腕部旋转）。FK 原点就是此处外参测量定义的真实夹爪中心 E，不再使用旧的 12 cm 伸出段端点作为 E。`pitch=0°` 仍指 a4 段水平，不是腕轴到抓取点的斜线水平。旧 `planning.cartesian_offset_m` 输入平移已撤掉，mission 提交的基座 XYZ 即 TCP 目标，不在相机换算中再补偿 3 cm。已测量的 E/S/C 外参数值保持不变。
+
+只修改 `camera_extrinsic.yaml` 的相机外参时，重启使用它的 mission/相机调试客户端及
+`test_camera_detector.py`，这些客户端只在启动时加载外参，无需因此重启 planner 或机械臂控制台。
+planner geometry 修改仍须同时重启 planner、相关客户端及控制台，不会自动热重载；
+geometry 必须通过共享解算器的尺寸/有限数校验，局部 Z 偏移允许为负。
 
 `service_timeout_sec` 默认 10 秒，每次运动单独计时；端点等待也不超过 10 秒，不设整个抓取工序共享预算。异步 Look 等待也有计时器保护。超时/请求异常取消 future 并请求 STOP，仍返回失败；STOP 服务成功仅表示已发布停止命令，不证明硬件停稳。恢复前须获得 STOP 请求之后新收到的无错误 IDLE、有限且在限位内的四轴低速反馈，再执行一次 Home；不自动清除 EMERGENCY。不得把 publish 或 future 完成当作运动成功，必须检查 `response.success`。
 
 ## 观察与 A/B/C 编排
 
-- 导航与相应 Look 并行；导航成功、Look 成功后，底盘/机械臂必须连续静止 0.5 秒才启 DETECT/QR。初值：线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s、各轴速度绝对值 ≤0.05 rad/s，均在 mission YAML 可调。底盘 `/odometry` 时间戳和本机接收年龄 ≤0.5 秒；机械臂沿用 planner 新鲜度。原始 Point-LIO 输出不提供可用速度，不作静止依据；不能使用 planner 可能替换 NaN 的 JointState 速度，需原始 ArmStatus。
+- 导航与相应 Look 并行；导航成功、Look 成功后，底盘/机械臂必须连续静止 0.5 秒才启 DETECT/QR。初值：线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s、各轴速度绝对值 ≤0.05 rad/s，均在 mission YAML 可调。底盘 `/odometry` 时间戳和本机接收年龄 ≤0.5 秒；机械臂沿用 planner 新鲜度。原始 Point-LIO 输出不提供可用速度，不作静止依据；机械臂停稳继续检查原始 ArmStatus 的新鲜、有限速度，ArmTelemetry 也不再把非有限速度伪造为零。
 - A：导航与 left_look 并行 → 左侧识别/抓取/left_harvest/Home → right_look → 右侧对应工序。
 - B：每树只观察一次，按既有像素评分选择最多两个匹配且成熟的目标。在首次观察姿态下，一次性转换并保存两个基座坐标/绝对 pitch。第二抓直接用已保存请求，不重新 Look、识别或换算。第一抓失败但一次安全 Home 成功后，继续第二抓。
 - C：二维码任务顺序、路线与左右侧使用现有规划结果，导航时提前 Look，不增加到点方向复核。导航失败跳过当前项，不在未到达的位置抓取。
@@ -112,12 +127,55 @@ python3 src/fruit_mission/fruit_mission/camera_target_transform.py 0 0 500
 
 ```text
 camera_mm      = [0, 0, 500]
-end_effector_mm = [434.728894, 31.255715, -52.926004]
+end_effector_mm = [434.728894, 31.255715, 80.525996]
 ```
 
 光学原点 `(0,0,0)` 的几何转换结果应等于 t_E_C；独立数学函数允许该点，实际控制入口拒绝非正深度。
 端到端控制前还须在安全现场用已知位置点验证毫米/米、左右方向、俯仰方向和夹爪原点。离线数值/模拟服务检查不能替代实机标定。
 
+最小纯数学无运动校验脚本（核验相机、TCP与Base坐标几何）：
+
+```bash
+/usr/bin/python3 - <<'PY'
+import math
+import numpy as np
+from fruit_mission.camera_target_transform import (
+    load_camera_extrinsic, transform_camera_to_end_effector_mm,
+    transform_end_to_base_mm,
+)
+from test_camera_detector import camera_to_base_point
+path = '/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml'
+r, t = load_camera_extrinsic(path)
+assert np.allclose(transform_camera_to_end_effector_mm((0, 0, 0), r, t), t)
+assert np.allclose(transform_camera_to_end_effector_mm((0, 0, 500), r, t),
+                   (434.728894, 31.255715, 80.525996), atol=1e-6)
+origin = (0.0006, 0.1482, 0.2996)
+for yaw in (0.0, math.pi / 2, -math.pi / 2):
+    for pitch_deg in (0.0, -57.69):
+        pitch = math.radians(pitch_deg)
+        c, s = math.cos(pitch), math.sin(pitch)
+        cy, sy = math.cos(yaw), math.sin(yaw)
+        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
+        ry = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
+        cam = (0.0058828, 0.0676498, 0.5084)
+        end = np.asarray(r) @ (1000 * np.asarray(cam)) + np.asarray(t)
+        expected = np.asarray(origin) + (rz @ ry @ end) / 1000
+        actual = camera_to_base_point(cam, np.asarray(r), np.asarray(t),
+                                      (*origin, pitch_deg, yaw))
+        assert np.allclose(actual, expected, atol=1e-9)
+        zero = transform_end_to_base_mm((0, 0, 0),
+                                       tuple(v * 1000 for v in origin), yaw, pitch)
+        assert np.allclose(np.asarray(zero) / 1000, origin, atol=1e-12)
+try:
+    transform_end_to_base_mm((1, 2, float('nan')), (0, 0, 0), 0, 0)
+except ValueError:
+    pass
+else:
+    raise AssertionError('nonfinite point was accepted')
+print('camera/TCP/base geometry checks passed; no motion commanded')
+PY
+```
+
 C++ 独立头文件为 `include/fruit_mission/camera_target_transform.hpp`，函数
 `fruit_mission::transformCameraToEndEffectorMm(camera_mm, rotation, translation_mm)`；
 只使用标准库，R/t 显式传入，避免复制一套硬编码标定。当前 mission 为 Python，运行链只用 Python 实现。
@@ -136,11 +194,25 @@ source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
 
 正式 mission 的 `config/fruit_mission.yaml` 使用 `arm_presets` 名称映射：
 `observation_left=left_look`、`observation_right=right_look`、`qr=qr_look`、
-`harvest_left=left_harvest`、`harvest_right=right_harvest`、`home=home`。这里仅配置字符串；各姿态的真实关节轨迹仍须由用户在
+`harvest_left=left_store`、`harvest_right=right_store`、`home=home`。按用户确认，正式 Harvest 复用已有侧向 store 放果工序，完成后仍显式 Home。这里仅配置字符串；各姿态的真实关节轨迹仍须由用户在
 planner 的 `arm_presets.yaml` 中完成：Look 末点 OPEN；Harvest 末点 OPEN 且驻留；Home 保持夹爪。本次不修改 planner 的代码、配置或文件，不补假姿态。缺失、越限、执行失败或超时均返回失败。
 区域只通过现有持久 ROS 话题通知感知/扫码节点，不再发送下位机区域命令。
 
-独立 `camera_arm_target_client` 复用 MissionContext 的相同抓取流程，区域工具只发布 ROS 区域，不再引用已删除的旧消息。`test_camera_arm_target.sh` 同时启动唯一 perception/串口桥与 planner-only launch；复用模式不另启动 planner。运行前还需提供导航栈的 `/odometry`，否则静止门槛不能通过。该脚本会驱动硬件，不是自动测试。
+独立 `camera_arm_target_client` 与 `test_camera_arm_target.sh` 不依赖底盘导航栈 `/odometry` 与正式比赛 Harvest 映射；每次运行先执行 `home`，随后进入 `left_look`/`right_look` 观察姿态，实测静止并张爪后开启 harvest 观察窗口，完成基座解算与 CLOSE/HOLD 抓取。按用户新确认，成功后继续执行对应侧 `left_store`/`right_store` 放果工序，再显式执行 `home` 并等待完成，不停留在目标点，也不重复执行另一套 Harvest。任何工序失败不报整个流程成功。该脚本会驱动硬件，不是自动测试。
+
+- 模型选择使用可靠、持久区域发布端，先等待 `fruit_perception_node` 的实际区域订阅端，再发布一次并等待 DDS acknowledgment；仅扫码节点订阅或串口桥就绪不能视为感知区域端就绪。DDS 确认是传输层交付，不是推理就绪回执。
+- 不再支持品种过滤参数。B 等待 `targets_ready`，只从有限置信度、有效正深度的 `*_ripe` 目标中选最高置信度者；无成熟目标时等待超时，不回退到无类型信息的 `target_point` 或未成熟目标。
+- Look 完成并确认机械臂静止后，共享 planner 客户端以新鲜、有限、在限位内的四轴硬件角调用 `move_joints`，先 OPEN，再用实测角 HOLD 驻留（时长取 planner `default_dwell_ms`）。张爪失败不开始识别/目标运动；张爪在采图之前，避免采图后改变手眼姿态。未修改 Look 预设或标定。
+- 客户端等待值必须为有限正数，默认识别超时 10 秒；大于 10 秒的值截断到 10 秒，较短值保留。共享 planner 客户端对 `endpoint_timeout_sec`、`service_timeout_sec` 同样限幅，各次服务/识别/交付使用独立期限；运动超时仍请求 STOP，不将发布等同于硬件完成或停稳。
+
+### 独立检测工具与实时 TCP (test_camera_detector.py)
+
+独立查看工具 `test_camera_detector.py` 同样全面接入真实 TCP 遥测：
+- 启动：`./test_camera_detector.py --model A|B`。静态观察预设选择 `--preset` 与快捷键 `p` 已删除，基座映射姿态完全由实时遥测决定。
+- 依赖管理与通信所有权：工具在启动时检测 ROS 图中是否存在 `serial_controller`（或状态发布端）及 `arm_planner_node`（或遥测发布端）。已有则复用，缺少才按需调用对应 launch。**工具会按需打开串口通信以取得实时 TCP，但工具本身绝不自动调用任何运动、复位或张爪服务**。退出时仅清理工具自身启动的后台进程，不关闭或发送 STOP 给已存在的外部节点。若在启动后的遥测等待期间发生中断（如 Ctrl-C）或异常，函数内部具备局部清理保护，立即终止已启动的自有进程（先停 Planner 再停串口桥），防止孤儿进程残留；成功返回后由主流程统一负责自有进程清理。
+- 端点冲突拒绝与 N/A 规则：工具通过类型化发布端检查（`get_publishers_info_by_topic`）校验遥测与状态发布端。若存在重复发布者（>1）或发布端消息类型不匹配（非 `ArmTelemetry` 或非 `ArmStatus`），则明确告警并拒绝按“无反馈”启动通信，切断位姿更新并将 Base 坐标置为 N/A。若未收到有效 TCP 或遥测过期（>0.25s），顶部横幅显示 `TCP: N/A`，目标框与鼠标探针处 Base 坐标均显示为 `Base: N/A`，2D 检测与深度测量照常工作；若 `CameraInfo` 内参缺失或无效，不显示 3D Cam 与 Base 坐标；若外参加载失败，不显示 Base 坐标。
+- 静止观测约束：换算仍依赖静止观测前置条件，不宣称运动中动态定位。
+
 退出路径请求 STOP/取消导航；杀进程不代表停稳。预设尚未补齐、串口顺序未经验证，也不能声称比赛整机可运行。
 原串口桥的跨报文轨迹装载顺序也仍需端到端核验。
 
diff --git a/src/fruit_mission/fruit_mission/Logic.py b/src/fruit_mission/fruit_mission/Logic.py
index ac078fd..afa8c4b 100644
--- a/src/fruit_mission/fruit_mission/Logic.py
+++ b/src/fruit_mission/fruit_mission/Logic.py
@@ -7,6 +7,7 @@ from fruit_mission.actions import pick_b_tree
 from fruit_mission.actions import pick_c
 from fruit_mission.actions import scan_b_qr
 from fruit_mission.actions import scan_c_qr
+from fruit_mission.decisions import format_b_tree_name
 
 
 def main(ctx):
@@ -49,8 +50,6 @@ def run_a(ctx):
         if pick_a(ctx, side="left").safe_to_continue:
             pick_a(ctx, side="right")
 
-    ctx.set_perception_mode("OFF")
-
 
 def run_b(ctx, plan):
     """QR tree order; observe/freeze/pick up to two fruit inside one action."""
@@ -58,16 +57,22 @@ def run_b(ctx, plan):
     waypoints = ctx.route.b_goal_names()
     for task in sorted(plan["tasks"], key=lambda item: int(item["sequence"])):
         position = int(task["position_id"])
+        tree_name = format_b_tree_name(position)
         if not 1 <= position <= len(waypoints):
-            ctx.announce(f"B区第{position}棵树编号无效，跳过。")
+            ctx.announce(f"B区{tree_name}编号无效，跳过。")
             continue
         side = "right" if position % 2 == 1 else "left"
         waypoint = waypoints[position - 1]
         if navigate(ctx, [waypoint], side=side):
-            pick_b_tree(ctx, waypoint=waypoint, target=str(task["target"]), side=side)
+            pick_b_tree(
+                ctx,
+                tree_name=tree_name,
+                target=str(task["target"]),
+                side=side,
+                waypoint=waypoint,
+            )
         else:
-            ctx.announce(f"B区{waypoint}导航失败，跳过。")
-    ctx.set_perception_mode("OFF")
+            ctx.announce(f"B区{tree_name}导航失败，跳过。")
 
 
 def run_c(ctx, plan):
@@ -82,7 +87,6 @@ def run_c(ctx, plan):
         pick_c(ctx, task, side=navigation["camera_side"])
         ctx.save_c_distribution(plan)
     ctx.save_c_distribution(plan)
-    ctx.set_perception_mode("OFF")
 
 
 def debug_a(ctx):
diff --git a/src/fruit_mission/fruit_mission/actions.py b/src/fruit_mission/fruit_mission/actions.py
index b20f0c6..67810a6 100644
--- a/src/fruit_mission/fruit_mission/actions.py
+++ b/src/fruit_mission/fruit_mission/actions.py
@@ -20,26 +20,37 @@ def navigate(ctx, waypoints, *, side=None):
 
 
 def scan_b_qr(ctx, *, arrived):
-    """QR pose -> standstill -> QR plan/fallback -> announcement -> Home."""
+    """QR pose -> QR window -> QR plan/fallback -> announcement -> Home."""
     ctx.report_current_area("B")
-    ready = arrived and ctx.send_qr_pose() and ctx.stable_stop()
-    plan = ctx.wait_b_plan() if ready and ctx.set_perception_mode("QR") else None
+    ready = arrived and ctx.send_qr_pose()
+    plan = None
+    if ready:
+        window_id = ctx.open_observation_window("B", "qr")
+        try:
+            plan = ctx.wait_b_plan(window_id)
+        finally:
+            ctx.close_observation_window()
     if plan is None:
         ctx.node.get_logger().warning("B区二维码获取失败，使用配置的兜底顺序")
         plan = build_b_plan("\n".join(ctx.config["qr_fallback"]["b_targets"]))
     ctx.save_b_plan(plan)
-    ctx.announce(format_b_announcement(plan), length_scale=_qr_length_scale(ctx))
-    ctx.set_perception_mode("OFF")
+    ctx.announce(format_b_announcement(plan))
     if arrived:  # Failed navigation already performed its one recovery.
         ctx.recover_home()
     return plan
 
 
 def scan_c_qr(ctx, *, arrived):
-    """QR pose -> standstill -> QR plan/fallback -> task table -> Home."""
+    """QR pose -> QR window -> QR plan/fallback -> task table -> Home."""
     ctx.report_current_area("C")
-    ready = arrived and ctx.send_qr_pose() and ctx.stable_stop()
-    plan = ctx.wait_c_plan() if ready and ctx.set_perception_mode("QR") else None
+    ready = arrived and ctx.send_qr_pose()
+    plan = None
+    if ready:
+        window_id = ctx.open_observation_window("C", "qr")
+        try:
+            plan = ctx.wait_c_plan(window_id)
+        finally:
+            ctx.close_observation_window()
     if plan is None:
         ctx.node.get_logger().warning("C区二维码获取失败，使用配置的兜底顺序")
         fallback = ctx.config["qr_fallback"]
@@ -51,9 +62,7 @@ def scan_c_qr(ctx, *, arrived):
     ctx.announce_and_wait(
         format_c_announcement(plan),
         lead_time_sec=float(ctx.config.get("c_qr_announce_lead_sec", 4.0)),
-        length_scale=_qr_length_scale(ctx),
     )
-    ctx.set_perception_mode("OFF")
     if arrived:
         ctx.recover_home()
     return distribution
@@ -68,19 +77,17 @@ def pick_a(ctx, *, side):
 
 
 def observe_and_pick(ctx, *, side, expected_type):
-    """Look -> measured standstill -> fresh target -> CLOSE/Harvest/Home."""
+    """Look -> harvest window -> fresh target -> CLOSE/Harvest/Home."""
     ctx.set_expected_fruit_type(expected_type)
     look = ctx.finish_observation(side)
     if not look.completed:
         return PickExecutionResult(False, look.reason_code, safe_to_continue=ctx.recover_home())
-    if not ctx.stable_stop():
-        return PickExecutionResult(
-            False, "stationary_timeout", safe_to_continue=ctx.recover_home())
-    fresh_after = ctx.node.get_clock().now().nanoseconds
-    if not ctx.set_perception_mode("DETECT"):
-        return PickExecutionResult(
-            False, "detect_mode_timeout", safe_to_continue=ctx.recover_home())
-    selection = ctx.wait_camera_target(fresh_after, expected_type)
+    area = ctx._current_area or "A"
+    window_id = ctx.open_observation_window(area, "harvest", expected_type)
+    try:
+        selection = ctx.wait_camera_target(window_id, expected_type)
+    finally:
+        ctx.close_observation_window()
     if selection.target is None:
         return PickExecutionResult(
             False, selection.reason_code, selection.matched_type, ctx.recover_home())
@@ -90,14 +97,13 @@ def observe_and_pick(ctx, *, side, expected_type):
         ctx.node.get_logger().warning(str(error))
         return PickExecutionResult(
             False, "camera_target_invalid", selection.matched_type, ctx.recover_home())
-    result = _grasp_and_collect(ctx, request, side=side)
+    result = _grasp_and_collect(ctx, request, side=side, matched_type=selection.matched_type)
     return PickExecutionResult(
         result.success, result.reason_code, selection.matched_type, result.safe_to_continue)
 
 
-def _grasp_and_collect(ctx, request, *, side):
+def _grasp_and_collect(ctx, request, *, side, matched_type=None):
     """One CLOSE/dwell -> side Harvest/OPEN -> Home; failure recovers once."""
-    ctx.set_perception_mode("OFF")
     grasp = ctx.move_to_base_target(request)
     if not grasp.completed:
         return PickExecutionResult(False, grasp.reason_code, safe_to_continue=ctx.recover_home())
@@ -106,77 +112,77 @@ def _grasp_and_collect(ctx, request, *, side):
         return PickExecutionResult(
             False, harvest.reason_code, safe_to_continue=ctx.recover_home())
     home = ctx.execute_arm_preset("home")
+    if home.completed:
+        ctx.record_pick(matched_type)
     return PickExecutionResult(
         home.completed, "harvest_completed" if home.completed else home.reason_code,
         safe_to_continue=home.completed,
     )
 
 
-def pick_b_tree(ctx, *, waypoint, target, side):
+def pick_b_tree(ctx, *, target, side, tree_name=None, waypoint=None):
     """Observe once; freeze up to two matching ripe targets before either grasp."""
+    label = tree_name or waypoint or "第一棵树"
     look = ctx.finish_observation(side)
     if not look.completed:
         reason = _b_observation_failure_text(look.reason_code)
         ctx.recover_home()
-        ctx.announce(f"B区{waypoint}，{reason}，跳过。")
-        return False
-    if not ctx.stable_stop():
-        ctx.recover_home()
-        ctx.announce(f"B区{waypoint}，识别失败，跳过。")
-        return False
-    if not ctx.set_perception_mode("DETECT"):
-        ctx.recover_home()
-        ctx.announce(f"B区{waypoint}，深度相机识别模式切换失败，跳过。")
+        ctx.announce(f"B区{label}，{reason}，跳过。")
         return False
-    targets = ctx.wait_b_targets(ctx.node.get_clock().now().nanoseconds)
+    window_id = ctx.open_observation_window("B", "harvest", target)
+    try:
+        targets = ctx.wait_b_targets(window_id)
+    finally:
+        ctx.close_observation_window()
     if targets is None:
         ctx.recover_home()
-        ctx.announce(f"B区{waypoint}，识别超时，跳过。")
+        ctx.announce(f"B区{label}，识别超时，跳过。")
         return False
-    selected = _select_b_targets(ctx, waypoint, target, targets)
+    selected = _select_b_targets(ctx, label, target, targets)
     requests = []
     for item in selected:
         try:
-            requests.append(ctx.camera_target_request(item.point))
+            req = ctx.camera_target_request(item.point)
+            requests.append((req, item.model_type))
         except ValueError as error:
             ctx.node.get_logger().warning(f"Skipping invalid B target: {error}")
     if not requests:
         ctx.recover_home()
         return False
     success = True
-    for request in requests:
-        result = _grasp_and_collect(ctx, request, side=side)
+    for request, model_type in requests:
+        result = _grasp_and_collect(ctx, request, side=side, matched_type=model_type)
         success = result.success and success
         if not result.success:
             reason = _b_arm_failure_text(result.reason_code)
             if not result.safe_to_continue:
-                ctx.announce(f"B区{waypoint}，{reason}，跳过剩余目标。")
+                ctx.announce(f"B区{label}，{reason}，跳过剩余目标。")
                 break
-            ctx.announce(f"B区{waypoint}，{reason}，继续下一个目标。")
+            ctx.announce(f"B区{label}，{reason}，继续下一个目标。")
     return success
 
 
-def _select_b_targets(ctx, waypoint, target, targets):
+def _select_b_targets(ctx, label, target, targets):
     decision = decide_b_tree_targets(
         target, expected_ripe_type(target),
         ((item.model_type, item.pixel_x, item.pixel_y, item.image_width) for item in targets),
     )
     if decision.reason_code == "no_detection":
-        ctx.announce(f"B区{waypoint}，没有识别到果实，跳过。")
+        ctx.announce(f"B区{label}，没有识别到果实，跳过。")
         return []
     if decision.reason_code == "inconsistent_types":
-        ctx.announce(f"B区{waypoint}，识别结果不一致，跳过。")
+        ctx.announce(f"B区{label}，识别结果不一致，跳过。")
         return []
     if decision.reason_code == "type_mismatch":
-        ctx.announce(f"B区{waypoint}，识别到{decision.observed_name}，和二维码不一致，跳过。")
+        ctx.announce(f"B区{label}，识别到{decision.observed_name}，和二维码不一致，跳过。")
         return []
     if decision.target_index is None:
         ctx.announce(
-            f"B区{waypoint}，识别到{decision.observed_name}，和二维码一致，"
+            f"B区{label}，识别到{decision.observed_name}，和二维码一致，"
             f"没有成熟{target}，跳过。")
         return []
     ctx.announce(
-        f"B区{waypoint}，识别到{decision.observed_name}，和二维码一致，开始抓成熟{target}。")
+        f"B区{label}，识别到{decision.observed_name}，和二维码一致，开始抓成熟{target}。")
     return [targets[index] for index in decision.target_indices]
 
 
@@ -222,14 +228,9 @@ def pick_c(ctx, task, *, side):
     return result
 
 
-def _qr_length_scale(ctx):
-    return float(ctx.config.get("tts", {}).get("qr_length_scale", 0.8))
-
-
 def _b_observation_failure_text(reason_code):
     return {
         "side_pose_timeout": "机械臂转向采摘位置超时",
-        "detect_mode_timeout": "深度相机识别模式切换失败",
         "camera_target_timeout": "识别超时",
     }.get(reason_code, "识别失败")
 
@@ -259,7 +260,6 @@ def _c_skip_text(position, target, reason_code, matched_type):
 def _pick_failure_text(reason_code):
     return {
         "side_pose_timeout": "机械臂转向采摘位置超时",
-        "detect_mode_timeout": "深度相机识别模式切换失败",
         "camera_target_timeout": "深度相机没有识别到可抓取目标",
         "arm_callback_failed": "机械臂执行报错",
         "arm_callback_timeout": "机械臂执行通信超时",
diff --git a/src/fruit_mission/fruit_mission/camera_arm_target_client.py b/src/fruit_mission/fruit_mission/camera_arm_target_client.py
index 9905efc..6433b1a 100644
--- a/src/fruit_mission/fruit_mission/camera_arm_target_client.py
+++ b/src/fruit_mission/fruit_mission/camera_arm_target_client.py
@@ -1,145 +1,395 @@
-"""Manual single-point client sharing the actual mission grasp implementation."""
+"""Manual single-point client for camera recognition and arm target grasping.
+
+Also executes store release and home recovery.
+"""
 
 import argparse
+import json
+import math
 import time
+import uuid
 
+from geometry_msgs.msg import PointStamped
 import rclpy
+from rclpy.duration import Duration
 from rclpy.node import Node
+from rclpy.utilities import remove_ros_args
 from std_msgs.msg import Bool, String
 from fruit_perception.msg import FruitDecision
 
-from fruit_mission.actions import observe_and_pick, pick_b_tree
-from fruit_mission.config import load_config
-from fruit_mission.decisions import spoken_produce_name
-from fruit_mission.mission_context import MissionContext
-from fruit_mission.mission_node import DEFAULT_CONFIG_PATH
+from fruit_mission.camera_target_planner_client import (
+    CameraTargetPlannerClient,
+    DEFAULT_EXTRINSIC_CONFIG,
+)
+from fruit_mission.config import load_target_offset
 from fruit_mission.script_hardware_client import latched_qos, reliable_qos
 
 
+def _bounded_timeout(value):
+    if (isinstance(value, bool) or not isinstance(value, (int, float))
+            or not math.isfinite(value) or value <= 0):
+        raise ValueError('Timeout must be finite and positive')
+    return min(10.0, float(value))
+
+
 class CameraArmTargetClient(Node):
-    """Lifecycle probes do not instantiate a hardware-control context."""
+    """Debug client for camera recognition, arm grasping, store release, and home recovery."""
 
     def __init__(self, debug=False):
         super().__init__('camera_arm_target_client')
         self.debug = debug
-        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
-        self.declare_parameter('perception_mode_topic', '/fruit_mission/perception_mode')
-        self.declare_parameter('active_mode_topic', '/fruit_perception/active_mode')
+        self.declare_parameter(
+            'config_path',
+            '/home/rc4/fruit_ws/src/fruit_mission/config/fruit_mission.yaml',
+        )
+        self.declare_parameter('camera_extrinsic_config', DEFAULT_EXTRINSIC_CONFIG)
+        self.declare_parameter('observation_window_topic', '/fruit_mission/observation_window')
+        self.declare_parameter('current_area_topic', '/fruit_mission/current_area')
+        self.declare_parameter('decision_topic', '/fruit_perception/decision')
         self.declare_parameter('bridge_ready_topic', '/auto_serial_bridge/ready')
-        self.declare_parameter('cleanup_mode_timeout_sec', 2.0)
-        self._mode = ''
-        self._generation = 0
+        self.declare_parameter('detection_timeout_sec', 10.0)
+        self.declare_parameter('endpoint_timeout_sec', 10.0)
+        self.declare_parameter('current_area_delivery_timeout_sec', 1.0)
+        for name in ('detection_timeout_sec',
+                     'endpoint_timeout_sec', 'current_area_delivery_timeout_sec'):
+            _bounded_timeout(self.get_parameter(name).value)
+
+        self._active_window_id = None
+        self._active_window_start_sec = 0
+        self._active_window_start_nanosec = 0
+        self._active_window_area = 'A'
+        self._active_window_purpose = 'harvest'
+        self._active_window_expected_type = ''
         self._bridge_ready = False
-        self._ctx = None
-        self._mode_pub = self.create_publisher(
-            String, self.get_parameter('perception_mode_topic').value, reliable_qos(1))
-        self.create_subscription(
-            String, self.get_parameter('active_mode_topic').value, self._on_mode, latched_qos())
+        self._camera_planner = None
+        self._latest_decision = None
+
+        self._window_pub = self.create_publisher(
+            String, self.get_parameter('observation_window_topic').value, reliable_qos(10))
+        self._area_pub = self.create_publisher(
+            String, self.get_parameter('current_area_topic').value, latched_qos())
+
         self.create_subscription(
             Bool, self.get_parameter('bridge_ready_topic').value,
             self._on_ready, latched_qos())
-
-    def _on_mode(self, message):
-        self._mode = message.data
-        self._generation += 1
+        self.create_subscription(
+            FruitDecision, self.get_parameter('decision_topic').value,
+            self._on_decision, 100)
 
     def _on_ready(self, message):
         self._bridge_ready = message.data
 
+    def _on_decision(self, message: FruitDecision):
+        if (
+            self._active_window_id is None
+            or message.observation_window_id != self._active_window_id
+        ):
+            return
+        stamp = message.header.stamp
+        if (
+            stamp.sec < 0
+            or not 0 <= stamp.nanosec < 1_000_000_000
+            or stamp.sec < self._active_window_start_sec
+            or (
+                stamp.sec == self._active_window_start_sec
+                and stamp.nanosec < self._active_window_start_nanosec
+            )
+        ):
+            return
+        self._latest_decision = message
+        if self.debug:
+            self._debug_decision(message)
+
+    def _debug_decision(self, message: FruitDecision):
+        stamp = message.header.stamp
+        items = [(item.type, item.confidence,
+                  (item.point.x, item.point.y, item.point.z) if item.has_point else None)
+                 for item in message.detections]
+        self.get_logger().debug(
+            f'[CAMERA DEBUG] stamp={stamp.sec}.{stamp.nanosec:09d} '
+            f'frame={message.header.frame_id} detections={items}')
+
     def wait_until(self, predicate, timeout):
-        deadline = time.monotonic() + timeout
+        deadline = time.monotonic() + _bounded_timeout(timeout)
         while rclpy.ok() and time.monotonic() < deadline:
             if predicate():
                 return True
             rclpy.spin_once(self, timeout_sec=0.1)
         return predicate()
 
-    def announce(self, text, **_kwargs):
-        self.get_logger().info(text)
-        return True
-
     def stack_is_absent(self):
         self.wait_until(lambda: False, 1.0)
-        conflicts = {'fruit_perception_container', 'camera_mode_manager',
-                     'serial_controller', 'arm_planner_node', 'fruit_mission'}
+        conflicts = {'fruit_perception_container',
+                     'arm_planner_node', 'fruit_mission'}
         return not conflicts.intersection(
             name for name, _ns in self.get_node_names_and_namespaces())
 
-    def set_off_for_cleanup(self):
-        previous = self._generation
-        self._mode_pub.publish(String(data='OFF'))
-        return self.wait_until(
-            lambda: self._generation > previous and self._mode == 'OFF',
-            min(10.0, self.get_parameter('cleanup_mode_timeout_sec').value))
+    def open_observation_window(self, model: str) -> str:
+        """Open an observation window for target acquisition."""
+        now = self.get_clock().now()
+        sec, nanosec = now.seconds_nanoseconds()
+        window_id = str(uuid.uuid4())
+        self._active_window_id = window_id
+        self._active_window_start_sec = sec
+        self._active_window_start_nanosec = nanosec
+        self._active_window_area = model
+        self._active_window_purpose = "harvest"
+        self._active_window_expected_type = "ripe" if model == "A" else ""
+        self._latest_decision = None
+        cmd = {
+            "window_id": window_id,
+            "valid": 1,
+            "start_sec": sec,
+            "start_nanosec": nanosec,
+            "area": model,
+            "purpose": "harvest",
+            "expected_type": self._active_window_expected_type,
+        }
+        msg = String()
+        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
+        self._window_pub.publish(msg)
+        self.get_logger().info(f"Opened observation window for model {model}")
+        self.get_logger().debug(f"Observation window opened: id='{window_id}' model={model}")
+        return window_id
+
+    def close_observation_window(self) -> None:
+        """Close the active observation window if any."""
+        if self._active_window_id is None:
+            return
+        window_id = self._active_window_id
+        self._active_window_id = None
+        cmd = {
+            "window_id": window_id,
+            "valid": 0,
+            "start_sec": self._active_window_start_sec,
+            "start_nanosec": self._active_window_start_nanosec,
+            "area": self._active_window_area,
+            "purpose": self._active_window_purpose,
+            "expected_type": self._active_window_expected_type,
+        }
+        msg = String()
+        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
+        self._window_pub.publish(msg)
+        self.get_logger().info('Closed observation window')
+        self.get_logger().debug(f"Observation window closed: id='{window_id}'")
+
+    def report_current_area(self, model):
+        """Deliver model selection only after the perception subscriber is discovered."""
+        if model not in ('A', 'B'):
+            raise ValueError('Model must be A or B')
+        topic = self._area_pub.topic_name
+        if not self.wait_until(
+            lambda: any(info.node_name == 'fruit_perception_node'
+                        for info in self.get_subscriptions_info_by_topic(topic)),
+            self.get_parameter('endpoint_timeout_sec').value,
+        ):
+            self.get_logger().error('Perception area subscriber discovery timed out')
+            return False
+        self._area_pub.publish(String(data=model))
+        if not self._area_pub.wait_for_all_acked(Duration(seconds=_bounded_timeout(
+                self.get_parameter('current_area_delivery_timeout_sec').value))):
+            self.get_logger().error('Perception area delivery acknowledgment timed out')
+            return False
+        return True
 
-    def run_sequence(self, area, direction, b_fruit=None):
+    def _wait_for_target(self, model: str, window_id: str, timeout: float = 10.0):
+        deadline = time.monotonic() + _bounded_timeout(timeout)
+        while rclpy.ok() and time.monotonic() < deadline:
+            rclpy.spin_once(self, timeout_sec=0.1)
+            if self._active_window_id != window_id:
+                break
+            msg = self._latest_decision
+            if msg is None:
+                continue
+
+            if model == 'A':
+                if msg.has_target:
+                    pt = msg.target_point
+                    if (math.isfinite(pt.x) and math.isfinite(pt.y)
+                            and math.isfinite(pt.z) and pt.z > 0):
+                        target = PointStamped()
+                        target.header = msg.header
+                        target.point = pt
+                        self.get_logger().info(
+                            f'Model A (vegetable) target locked: '
+                            f'x={pt.x:.3f}, y={pt.y:.3f}, z={pt.z:.3f}')
+                        return target
+            else:
+                # Model B (fruit)
+                ripe_targets = [
+                    t for t in msg.targets
+                    if t.type.endswith('_ripe') and math.isfinite(t.confidence)
+                    and t.has_point and math.isfinite(t.point.x) and math.isfinite(t.point.y)
+                    and math.isfinite(t.point.z) and t.point.z > 0
+                ]
+                if ripe_targets and msg.targets_ready:
+                    best = max(ripe_targets, key=lambda t: t.confidence)
+                    target = PointStamped()
+                    target.header = msg.header
+                    target.point = best.point
+                    self.get_logger().info(
+                        f'Model B (fruit) target selected: type={best.type}, '
+                        f'conf={best.confidence:.3f}, '
+                        f'point=({best.point.x:.3f}, {best.point.y:.3f}, {best.point.z:.3f})')
+                    return target
+        return None
+
+    def run_sequence(self, model: str, direction: str) -> int:
+        look_preset = f'{direction}_look'
+        model_name = 'vegetable' if model == 'A' else 'fruit'
+        self.get_logger().info(
+            f'=== Starting Camera-Arm Target Test ===\n'
+            f'  Model: {model} ({model_name})\n'
+            f'  Preset: {look_preset}')
+
+        # 1. Wait for hardware bridge
+        self.get_logger().info('Waiting for auto_serial_bridge ready...')
         if not self.wait_until(lambda: self._bridge_ready, 10.0):
+            self.get_logger().error('Serial bridge not ready within 10s')
             return 1
-        config = load_config(self.get_parameter('config_path').value)
-        self._ctx = MissionContext(self, config, speaker=self)
-        if self.debug:
-            self.create_subscription(
-                FruitDecision, config['decision_topic'], self._debug_decision, 100)
-        ctx = self._ctx
-        ctx.report_current_area(area)
-        if not ctx.recover_home():
+        self.get_logger().info('Serial bridge is ready')
+
+        # 2. Initialize CameraTargetPlannerClient
+        extrinsic_cfg = self.get_parameter('camera_extrinsic_config').value
+        config_path = self.get_parameter('config_path').value
+        target_offset_m = load_target_offset(config_path)
+        self.get_logger().debug(f'Loaded target_offset_m={target_offset_m} from {config_path}')
+        self._camera_planner = CameraTargetPlannerClient(
+            self, extrinsic_cfg, target_offset_m=target_offset_m)
+
+        # 3. Publish current area so fruit_perception_node loads the correct model
+        if not self.report_current_area(model):
             return 1
-        if area != 'B':
-            result = observe_and_pick(
-                ctx, side=direction, expected_type='ripe' if area == 'A' else None)
-            self.get_logger().info(f'Single-point result: {result.reason_code}')
-            return 0 if result.success else 1
-        success = pick_b_tree(
-            ctx, waypoint='单点', target=spoken_produce_name(f'{b_fruit}_ripe'), side=direction)
-        return 0 if success else 1
-
-    def _debug_decision(self, message):
-        stamp = message.header.stamp
-        items = [(item.type, item.confidence,
-                  (item.point.x, item.point.y, item.point.z) if item.has_point else None)
-                 for item in message.detections]
+        self.get_logger().info(f'Delivered current area: {model}')
+
+        # 4. Move arm to home preset first
+        self.get_logger().info('Moving arm to home preset...')
+        completed, reason = self._camera_planner.execute_preset('home', self.wait_until)
+        if not completed:
+            self.get_logger().error(f'Failed to execute preset home: {reason}')
+            return 1
+        self.get_logger().info('Arm reached home position')
+
+        # 5. Move arm to look preset
+        self.get_logger().info(f'Moving arm to {look_preset}...')
+        completed, reason = self._camera_planner.execute_preset(look_preset, self.wait_until)
+        if not completed:
+            self.get_logger().error(f'Failed to execute preset {look_preset}: {reason}')
+            return 1
+        self.get_logger().info(f'Arm reached {look_preset}')
+
+        # 6. Wait for arm standstill
+        stationary_rps = 0.05
+        self.get_logger().info(f'Waiting for arm standstill (<= {stationary_rps} rps)...')
+        if not self.wait_until(lambda: self._camera_planner.is_stationary(stationary_rps), 5.0):
+            self.get_logger().error('Arm standstill timeout')
+            return 1
+
+        self.get_logger().info('Opening gripper at the measured observation pose...')
+        completed, reason = self._camera_planner.open_gripper(self.wait_until, stationary_rps)
+        if not completed:
+            self.get_logger().error(f'Gripper OPEN failed: {reason}')
+            return 1
+
+        # 7. Open observation window and collect target
+        self.get_logger().info('Opening observation window for target acquisition...')
+        window_id = self.open_observation_window(model)
+        timeout_sec = _bounded_timeout(self.get_parameter('detection_timeout_sec').value)
+        self.get_logger().info(f'Observing for targets (timeout={timeout_sec}s)...')
+        try:
+            target_point = self._wait_for_target(model, window_id, timeout=timeout_sec)
+        finally:
+            self.close_observation_window()
+
+        if target_point is None:
+            self.get_logger().error('No target acquired within timeout')
+            return 1
+
+        # 8. Transform optical target to arm base coordinates
+        self.get_logger().debug('Transforming target to arm base coordinates...')
+        try:
+            req = self._camera_planner.target_request(target_point)
+        except ValueError as err:
+            self.get_logger().error(f'Target transform failed: {err}')
+            return 1
+
+        # 11. Move arm to target point and grasp (CLOSE + dwell HOLD)
         self.get_logger().info(
-            f'[CAMERA DEBUG] stamp={stamp.sec}.{stamp.nanosec:09d} '
-            f'frame={message.header.frame_id} detections={items}')
+            f'Moving arm to target: x={req.x:.3f}m, y={req.y:.3f}m, z={req.z:.3f}m, '
+            f'pitch={req.pitch_deg:.1f}deg with gripper CLOSE...')
+        completed, reason = self._camera_planner.move_to_base_target(
+            req, self.wait_until, stationary_rps)
+        if not completed:
+            self.get_logger().error(f'Grasp motion failed: {reason}')
+            return 1
+
+        # 12. Move arm to store preset and release fruit
+        store_preset = f'{direction}_store'
+        self.get_logger().info(f'Moving arm to {store_preset} and releasing fruit...')
+        completed, reason = self._camera_planner.execute_preset(store_preset, self.wait_until)
+        if not completed:
+            self.get_logger().error(f'Failed to execute preset {store_preset}: {reason}')
+            return 1
+
+        # 13. Return arm to home position
+        self.get_logger().info('Returning arm to home...')
+        completed, reason = self._camera_planner.execute_preset('home', self.wait_until)
+        if not completed:
+            self.get_logger().error(f'Failed to return home: {reason}')
+            return 1
+
+        self.get_logger().info(
+            f'Grasp, {store_preset}, and return home sequence completed successfully!')
+        return 0
 
     def destroy_node(self):
-        if self._ctx is not None:
-            self._ctx.close()
+        self.close_observation_window()
+        if self._camera_planner is not None:
+            self._camera_planner.request_stop()
         return super().destroy_node()
 
 
 def main(args=None):
-    parser = argparse.ArgumentParser()
+    parser = argparse.ArgumentParser(description='Camera and arm coordination debug client.')
     mode = parser.add_mutually_exclusive_group()
-    mode.add_argument('--require-stack-absent', action='store_true')
-    mode.add_argument('--mode-off', action='store_true')
-    parser.add_argument('--debug', action='store_true')
-    parser.add_argument('--zone', choices=('A', 'B', 'C'))
+    mode.add_argument('--require-stack-absent', action='store_true',
+                      help='Check if conflicting nodes are absent (serial bridge may exist)')
+    parser.add_argument('--debug', action='store_true',
+                        help='Enable per-frame detection debug logging')
+    parser.add_argument('--model', '-m', choices=('A', 'B', 'a', 'b'),
+                        help='Perception model: A (vegetable) or B (fruit)')
+    parser.add_argument('--zone', '-z', choices=('A', 'B', 'C', 'a', 'b', 'c'),
+                        help='Alias for --model')
     direction = parser.add_mutually_exclusive_group()
-    direction.add_argument('--left', dest='direction', action='store_const', const='left')
-    direction.add_argument('--right', dest='direction', action='store_const', const='right')
-    fruit = parser.add_mutually_exclusive_group()
-    fruit.add_argument('--apple', dest='b_fruit', action='store_const', const='apple')
-    fruit.add_argument('--pear', dest='b_fruit', action='store_const', const='pear')
+    direction.add_argument('--left', dest='direction', action='store_const', const='left',
+                           help='Use left_look and left_store presets')
+    direction.add_argument('--right', dest='direction', action='store_const', const='right',
+                           help='Use right_look and right_store presets')
     options, ros_args = parser.parse_known_args(args)
-    if not (options.require_stack_absent or options.mode_off):
-        if options.zone is None or options.direction is None:
-            parser.error('--zone and --left/--right are required')
-        if (options.zone == 'B') != (options.b_fruit is not None):
-            parser.error('--apple/--pear must be specified for B only')
+    unknown = remove_ros_args(args=ros_args)
+    if unknown:
+        parser.error(f'unrecognized arguments: {" ".join(unknown)}')
+    if not options.require_stack_absent:
+        raw_model = options.model or options.zone
+        if raw_model is None or options.direction is None:
+            parser.error('--model A|B (or --zone) and --left/--right are required')
+        model = raw_model.upper()
+        if model not in ('A', 'B'):
+            parser.error('--model must be A (vegetable) or B (fruit)')
+    else:
+        model = None
+
     rclpy.init(args=ros_args)
     node = CameraArmTargetClient(debug=options.debug)
     status = 1
     try:
         if options.require_stack_absent:
             status = 0 if node.stack_is_absent() else 9
-        elif options.mode_off:
-            status = 0 if node.set_off_for_cleanup() else 7
         else:
-            status = node.run_sequence(options.zone, options.direction, options.b_fruit)
+            status = node.run_sequence(model, options.direction)
     except KeyboardInterrupt:
-        if node._ctx is not None:
-            node._ctx._camera_planner.request_stop()
+        if node._camera_planner is not None:
+            node._camera_planner.request_stop()
             node.wait_until(lambda: False, 0.2)
         status = 130
     finally:
diff --git a/src/fruit_mission/fruit_mission/camera_target_planner_client.py b/src/fruit_mission/fruit_mission/camera_target_planner_client.py
index 51263d0..d5e613c 100644
--- a/src/fruit_mission/fruit_mission/camera_target_planner_client.py
+++ b/src/fruit_mission/fruit_mission/camera_target_planner_client.py
@@ -5,18 +5,19 @@ from pathlib import Path
 import time
 
 from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
+from fruit_arm_interfaces.msg import ArmTelemetry
 from fruit_hardware_interfaces.msg import ArmStatus
 from fruit_arm_planner.joint_transformer import JointTransformer
-from fruit_arm_planner.kinematics import RobotKinematics
-from sensor_msgs.msg import JointState
 from std_srvs.srv import Trigger
 import yaml
 
 from fruit_mission.camera_target_transform import (
+    apply_target_offset_mm,
     load_camera_extrinsic,
     transform_camera_to_end_effector_mm,
     transform_end_to_base_mm,
 )
+from fruit_mission.config import validate_target_offset
 
 
 DEFAULT_EXTRINSIC_CONFIG = (
@@ -27,20 +28,23 @@ DEFAULT_EXTRINSIC_CONFIG = (
 class CameraTargetPlannerClient:
     """Shared by the mission flow and standalone camera client, not by planner.
 
-    DETECT and the interval before submission must keep the same stationary
-    observation pose. Use fresh measured joints, not hardcoded observation angles.
+    Observation window and the interval before submission must keep the same stationary
+    observation pose. Use fresh measured TCP telemetry, not hardcoded observation angles.
     """
 
-    def __init__(self, node, config_path=DEFAULT_EXTRINSIC_CONFIG):
+    def __init__(
+        self,
+        node,
+        config_path=DEFAULT_EXTRINSIC_CONFIG,
+        target_offset_m=(0.0, 0.0, 0.0),
+    ):
         self.node = node
+        self.target_offset_m = validate_target_offset(target_offset_m, 'target_offset_m')
         with Path(config_path).open(encoding='utf-8') as stream:
             self.config = yaml.safe_load(stream)
         self.rotation, self.translation_mm = load_camera_extrinsic(config_path)
         with Path(self.config['arm_config_path']).open(encoding='utf-8') as stream:
             arm = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
-        if not all(math.isfinite(v) and v > 0 for v in arm['geometry'].values()):
-            raise ValueError('arm geometry must be finite and positive')
-        self.kinematics = RobotKinematics.from_geometry_dict(arm['geometry'])
         self.transformer = JointTransformer.from_config_dict(arm['joint_mapping'])
         self.status_max_age_sec = float(arm['planning']['status_max_age_sec'])
         self.gripper_dwell_ms = arm['planning']['default_dwell_ms']
@@ -54,6 +58,7 @@ class CameraTargetPlannerClient:
                 raise ValueError(f'{name} must be numeric')
             if not math.isfinite(value) or value <= 0:
                 raise ValueError(f'{name} must be finite and positive')
+            self.config[name] = min(10.0, float(value))
         for name in ('camera_frame', 'joint_state_topic',
                      'move_cartesian_service', 'stop_service'):
             if not isinstance(self.config[name], str) or not self.config[name].strip():
@@ -69,14 +74,14 @@ class CameraTargetPlannerClient:
                                   or not isinstance(pitch, (int, float))
                                   or not math.isfinite(pitch)):
             raise ValueError('pitch_deg must be null or a finite number')
-        self._joints = None
+        self._pose = None
         self._latest_received_at = 0.0
         self._status = None
         self._status_received_at = 0.0
         self._motion_future = None
         self._motion_deadline = 0.0
         self._joint_subscription = node.create_subscription(
-            JointState, self.config['joint_state_topic'], self._on_joint_state, 10)
+            ArmTelemetry, self.config['joint_state_topic'], self._on_joint_state, 10)
         self._move_client = node.create_client(
             MoveArmCartesian, self.config['move_cartesian_service'])
         self._preset_client = node.create_client(
@@ -86,40 +91,46 @@ class CameraTargetPlannerClient:
         self._status_subscription = node.create_subscription(
             ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
         self._motion_timer = node.create_timer(0.1, self._check_motion_timeout)
-        offset = arm['planning'].get('cartesian_offset_m', [0.0, 0.0, 0.0])
-        node.get_logger().info(
-            f'Mission camera transform ready; planner adds cartesian_offset_m={offset}')
+        node.get_logger().debug(
+            'Mission camera transform ready; using planner mapped joints and TCP telemetry')
 
     def _on_joint_state(self, message):
         self._latest_received_at = time.monotonic()
-        self._joints = None
-        names = [f'joint_{i}' for i in range(1, 5)]
-        if len(message.name) != 4 or set(message.name) != set(names):
-            return
-        if len(message.position) != 4:
+        self._pose = None
+        if message.header.frame_id != 'arm_base_link':
             return
-        joints = tuple(message.position[message.name.index(name)] for name in names)
-        if not all(math.isfinite(q) for q in joints):
-            return
-        if not self.transformer.check_hardware_limits(joints)[0]:
+        names = [f'joint_{i}' for i in range(1, 5)]
+        hardware = message.hardware_joint_state
+        model = message.model_joint_state
+        for state in (hardware, model):
+            if (len(state.name) != 4 or set(state.name) != set(names)
+                    or len(state.position) != 4
+                    or not all(math.isfinite(q) for q in state.position)):
+                return
+        joints = tuple(hardware.position[hardware.name.index(name)] for name in names)
+        tcp = message.tcp_position
+        pose = (tcp.x, tcp.y, tcp.z, message.tcp_pitch_deg,
+                model.position[model.name.index('joint_1')])
+        if (not self.transformer.check_hardware_limits(joints)[0]
+                or not all(math.isfinite(v) for v in pose)):
             return
-        self._joints = joints
+        self._pose = pose
 
     @staticmethod
     def _stamp_ns(message):
         return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
 
-    def _fresh_joints(self):
+    def _fresh_pose(self):
         if time.monotonic() - self._latest_received_at > self.status_max_age_sec:
             return None
-        return self._joints
+        return self._pose
 
     def _on_status(self, message):
         self._status = message
         self._status_received_at = time.monotonic()
 
     def is_stationary(self, max_joint_speed, received_after=0.0, require_idle=False):
-        """Use finite raw feedback, not JointState's substituted velocities."""
+        """Use fresh finite raw hardware feedback for measured standstill."""
         status = self._status
         if (status is None
                 or time.monotonic() - self._status_received_at > self.status_max_age_sec
@@ -142,29 +153,33 @@ class CameraTargetPlannerClient:
         if (target.header.frame_id != self.config['camera_frame'] or stamp_ns <= 0
                 or not all(math.isfinite(v) for v in point_m) or point_m[2] <= 0):
             raise ValueError('Invalid camera target frame, stamp or depth')
-        hardware_joints = self._fresh_joints()
-        if hardware_joints is None:
-            raise ValueError('No fresh valid measured joint feedback')
-        algo = self.transformer.hardware_to_algo(hardware_joints)
-        end_x, end_y, end_z, pitch = self.kinematics.forward_kinematics(
-            *algo)['end_effector']
+        pose = self._fresh_pose()
+        if pose is None:
+            raise ValueError('No fresh valid measured TCP telemetry')
+        end_x, end_y, end_z, pitch_deg, yaw = pose
+        pitch = math.radians(pitch_deg)
         try:
             end_mm = transform_camera_to_end_effector_mm(
                 tuple(v * 1000.0 for v in point_m), self.rotation, self.translation_mm)
             base_mm = transform_end_to_base_mm(
-                end_mm, (end_x * 1000.0, end_y * 1000.0, end_z * 1000.0), algo[0], pitch)
+                end_mm, (end_x * 1000.0, end_y * 1000.0, end_z * 1000.0), yaw, pitch)
+            adjusted_base_mm = apply_target_offset_mm(base_mm, self.target_offset_m)
             request = MoveArmCartesian.Request()
-            request.x, request.y, request.z = (v / 1000.0 for v in base_mm)
+            request.x, request.y, request.z = (v / 1000.0 for v in adjusted_base_mm)
             desired_pitch = self.config['motion']['pitch_deg']
             request.pitch_deg = float(
-                math.degrees(pitch) if desired_pitch is None else desired_pitch)
+                pitch_deg if desired_pitch is None else desired_pitch)
             request.duration_ms = self.config['motion']['duration_ms']
             request.gripper_action = 2  # CLOSE at the endpoint.
             request.linear_path = self.config['motion']['linear_path']
         except (ValueError, OverflowError, AssertionError) as error:
             raise ValueError(f'Invalid transformed target: {error}') from error
-        self.node.get_logger().info(
-            f'Camera(m)={point_m} -> end(mm)={end_mm} -> arm base(mm)={base_mm}; '
+        req_xyz = [round(request.x, 4), round(request.y, 4), round(request.z, 4)]
+        self.node.get_logger().debug(
+            f'Camera(m)={point_m} -> end(mm)={end_mm} -> arm base(mm)={base_mm} '
+            f'+ offset(m)={self.target_offset_m} -> req(m)={req_xyz}; '
+            f'tcp(m)={[round(end_x, 4), round(end_y, 4), round(end_z, 4)]}, '
+            f'obs_pitch={pitch_deg:.2f}deg, obs_yaw={math.degrees(yaw):.2f}deg, '
             f'move_cartesian pitch={request.pitch_deg:.3f}deg, gripper=CLOSE')
         return request
 
@@ -178,18 +193,28 @@ class CameraTargetPlannerClient:
             return completed, reason
         # Cartesian API has no dwell field. Reuse planner PTP at measured raw
         # joints with HOLD instead of generating a trajectory or sleeping here.
-        if not wait_until(
-                lambda: self.is_stationary(max_joint_speed), self.config['endpoint_timeout_sec']):
-            return False, 'arm_pose_unavailable'
+        return self._gripper_at_measured_pose(0, wait_until, max_joint_speed)
+
+    def open_gripper(self, wait_until, max_joint_speed):
+        """OPEN at measured raw joints, then HOLD for the configured gripper dwell."""
+        completed, reason = self._gripper_at_measured_pose(1, wait_until, max_joint_speed)
+        if not completed:
+            return completed, reason
+        return self._gripper_at_measured_pose(0, wait_until, max_joint_speed)
+
+    def _gripper_at_measured_pose(self, action, wait_until, max_joint_speed):
         if not self._joints_client.wait_for_service(
                 timeout_sec=self.config['endpoint_timeout_sec']):
             return False, 'arm_endpoints_not_ready'
+        if not wait_until(
+                lambda: self.is_stationary(max_joint_speed), self.config['endpoint_timeout_sec']):
+            return False, 'arm_pose_unavailable'
         status = self._status
         dwell = MoveArmJoints.Request()
         dwell.joints_deg = [math.degrees(q) for q in (
             status.joint_1, status.joint_2, status.joint_3, status.joint_4)]
         dwell.duration_ms = self.gripper_dwell_ms
-        dwell.gripper_action = 0
+        dwell.gripper_action = action
         return self._call_motion(self._joints_client, dwell, wait_until)
 
     def start_preset(self, name):
@@ -201,7 +226,7 @@ class CameraTargetPlannerClient:
             return None
         request = ExecuteArmPreset.Request()
         request.preset_name = name
-        self.node.get_logger().info(f'Executing arm preset: {name}')
+        self.node.get_logger().debug(f'Executing arm preset: {name}')
         return self._start_motion(self._preset_client, request)
 
     def execute_preset(self, name, wait_until):
@@ -269,6 +294,10 @@ class CameraTargetPlannerClient:
             self.node.get_logger().warning(f'Planner request failed: {error}')
             self.request_stop()
             return False, 'arm_callback_failed'
-        self.node.get_logger().info(
-            f'Planner trajectory_id={response.trajectory_id}: {response.message}')
+        if response.success:
+            self.node.get_logger().info(
+                f'Planner trajectory_id={response.trajectory_id}: {response.message}')
+        else:
+            self.node.get_logger().warning(
+                f'Planner trajectory_id={response.trajectory_id} failed: {response.message}')
         return bool(response.success), 'completed' if response.success else 'arm_callback_failed'
diff --git a/src/fruit_mission/fruit_mission/camera_target_transform.py b/src/fruit_mission/fruit_mission/camera_target_transform.py
index 7d1f439..9c9a45a 100644
--- a/src/fruit_mission/fruit_mission/camera_target_transform.py
+++ b/src/fruit_mission/fruit_mission/camera_target_transform.py
@@ -81,6 +81,26 @@ def transform_end_to_base_mm(end_xyz_mm, end_origin_mm, yaw_rad, pitch_rad):
     ), 'base point (mm)')
 
 
+def apply_target_offset_mm(base_xyz_mm, offset_m):
+    """Apply base-frame target offset [dx, dy, dz] in meters to base point in mm.
+
+    dx and dz are added directly (scaled to mm).
+    dy represents lateral offset magnitude in meters; its sign is determined
+    by the sign of base_y (+abs(dy) when base_y > 0, -abs(dy) when base_y < 0).
+    """
+    bx, by, bz = _vector3(base_xyz_mm, 'base point (mm)')
+    dx, dy, dz = _vector3(offset_m, 'target offset (m)')
+    dx_mm = dx * 1000.0
+    dz_mm = dz * 1000.0
+    if by > 0.0:
+        dy_mm = abs(dy) * 1000.0
+    elif by < 0.0:
+        dy_mm = -abs(dy) * 1000.0
+    else:
+        dy_mm = 0.0
+    return _vector3((bx + dx_mm, by + dy_mm, bz + dz_mm), 'adjusted base point (mm)')
+
+
 if __name__ == '__main__':
     import argparse
     import json
diff --git a/src/fruit_mission/fruit_mission/config.py b/src/fruit_mission/fruit_mission/config.py
index f335600..46083ce 100644
--- a/src/fruit_mission/fruit_mission/config.py
+++ b/src/fruit_mission/fruit_mission/config.py
@@ -20,9 +20,35 @@ def load_config(path: str, waypoint_file: str | None = None) -> Dict[str, Any]:
     with fallback_path.open(encoding='utf-8') as stream:
         params['qr_fallback'] = yaml.safe_load(stream)
     validate_config(params)
+    params["target_offset_m"] = validate_target_offset(
+        params["target_offset_m"], f"target_offset_m in {path}")
     return params
 
 
+def validate_target_offset(
+    values: Any, context: str = "target_offset_m"
+) -> tuple[float, float, float]:
+    """Validate target offset as a 3-element finite float sequence (meters)."""
+    if (not isinstance(values, (list, tuple)) or len(values) != 3
+            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
+                   for v in values)):
+        raise ValueError(f"{context} must contain three finite numbers")
+    return (float(values[0]), float(values[1]), float(values[2]))
+
+
+def load_target_offset(config_path: str) -> tuple[float, float, float]:
+    """Load and validate target_offset_m from a mission config YAML file."""
+    path = Path(config_path)
+    if not path.is_file():
+        raise ValueError(f"mission config file does not exist: {config_path}")
+    with path.open(encoding="utf-8") as stream:
+        doc = yaml.safe_load(stream) or {}
+    params = doc.get("fruit_mission", {}).get("ros__parameters", {})
+    if "target_offset_m" not in params:
+        raise ValueError(f"target_offset_m missing in {config_path}")
+    return validate_target_offset(params["target_offset_m"], f"target_offset_m in {config_path}")
+
+
 def validate_config(params: Mapping[str, Any]) -> None:
     """Validate stable structure without freezing field-tuning values."""
     waypoint_file = str(params.get("waypoint_file", "")).strip()
@@ -63,6 +89,11 @@ def validate_config(params: Mapping[str, Any]) -> None:
         raise ValueError("decision_topic must not be empty")
     if not str(params.get("camera_extrinsic_config", "")).strip():
         raise ValueError("camera_extrinsic_config must not be empty")
+    if "target_offset_m" not in params:
+        raise ValueError("target_offset_m must be specified in config")
+    validate_target_offset(params["target_offset_m"], "target_offset_m")
+    if not str(params.get("observation_window_topic", "")).strip():
+        raise ValueError("observation_window_topic must not be empty")
     presets = params.get("arm_presets")
     if not isinstance(presets, dict):
         raise ValueError("arm_presets must be a mapping")
@@ -72,22 +103,16 @@ def validate_config(params: Mapping[str, Any]) -> None:
         if not isinstance(name, str) or not name.strip():
             raise ValueError(f"arm_presets.{action} must be a nonempty string")
     _validate_qr_fallbacks(params['qr_fallback'])
-    if not isinstance(params.get('odometry_topic'), str) or not params['odometry_topic'].strip():
-        raise ValueError('odometry_topic must be a nonempty string')
     stationary = params.get('stationary')
     if not isinstance(stationary, dict):
         raise ValueError('stationary must be a mapping')
-    for name in ('linear_speed_mps', 'angular_speed_rps', 'joint_speed_rps',
-                 'duration_sec', 'odometry_max_age_sec', 'timeout_sec'):
-        value = stationary.get(name)
-        if (isinstance(value, bool) or not isinstance(value, (int, float))
-                or not math.isfinite(value) or value <= 0):
-            raise ValueError(f'stationary.{name} must be finite and positive')
-    if not stationary['duration_sec'] <= stationary['timeout_sec'] <= 10.0:
-        raise ValueError('stationary duration must fit within timeout <= 10s')
+    joint_speed = stationary.get('joint_speed_rps')
+    if (isinstance(joint_speed, bool) or not isinstance(joint_speed, (int, float))
+            or not math.isfinite(joint_speed) or joint_speed <= 0):
+        raise ValueError('stationary.joint_speed_rps must be finite and positive')
     tts = params.get("tts", {})
-    if "qr_length_scale" in tts and float(tts["qr_length_scale"]) <= 0.0:
-        raise ValueError("tts.qr_length_scale must be positive")
+    if "playback_timeout_sec" in tts and float(tts["playback_timeout_sec"]) <= 0.0:
+        raise ValueError("tts.playback_timeout_sec must be positive")
     mission_flow = str(params.get("mission_flow", "full")).strip().lower()
     if mission_flow not in {"full", "ac", "a", "b", "c"}:
         raise ValueError("mission_flow must be full, ac, a, b, or c")
diff --git a/src/fruit_mission/fruit_mission/decisions.py b/src/fruit_mission/fruit_mission/decisions.py
index 6d848da..07bff83 100644
--- a/src/fruit_mission/fruit_mission/decisions.py
+++ b/src/fruit_mission/fruit_mission/decisions.py
@@ -121,6 +121,11 @@ def decide_b_tree_targets(
     return BTreeDecision("matched", observed_name, target_index, score, tuple(ranked[:2]))
 
 
+def format_b_tree_name(position_id: int) -> str:
+    """Return a spoken B-zone tree label."""
+    return f"第{_chinese_index(position_id)}棵树"
+
+
 def format_position_name(position_id: int) -> str:
     """Return a spoken C-zone position name."""
     return f"{_chinese_index(position_id)}号位置"
@@ -147,24 +152,30 @@ def ordered_c_tasks(c_plan: Mapping[str, object]) -> List[Mapping[str, object]]:
 
 
 def format_b_announcement(b_plan: Mapping[str, object]) -> str:
-    """Build a compact spoken summary for the B plan."""
+    """Build a spoken summary for the B plan."""
     parts = [
-        f"{int(task['position_id'])}{task['target']}"
+        f"第{_chinese_index(int(task['position_id']))}棵树{task['target']}"
         for task in b_plan.get("tasks", [])
     ]
-    return "，".join(parts)
+    return "；".join(parts)
 
 
 def format_c_announcement(c_plan: Mapping[str, object]) -> str:
-    """Build a compact spoken summary for the C plan."""
+    """Build a spoken summary for the C plan."""
     parts = [
         (
-            f"{int(task['sequence'])}，"
-            f"{int(task['position_id'])}{task['target']}"
+            f"第{_chinese_index(int(task['sequence']))}项，"
+            f"{format_position_name(int(task['position_id']))}，"
+            f"{task['target']}"
         )
         for task in c_plan.get("tasks", [])
     ]
-    return "，".join(parts)
+    return "；".join(parts)
+
+
+def format_produce_count(produce: str, count: int) -> str:
+    """Return a spoken summary phrase for one produce count, e.g. '南瓜两个'."""
+    return f"{produce}{_chinese_count(count)}"
 
 
 def _other_side(side: str) -> str:
@@ -187,3 +198,21 @@ def _chinese_index(value: int) -> str:
         12: "十二",
     }
     return names.get(value, str(value))
+
+
+def _chinese_count(value: int) -> str:
+    if value == 1:
+        return "一个"
+    if value == 2:
+        return "两个"
+    if 3 <= value <= 10:
+        return f"{_chinese_index(value)}个"
+    if 11 <= value <= 19:
+        return f"十{_chinese_index(value - 10)}个"
+    if 20 <= value <= 29:
+        unit = "" if value == 20 else _chinese_index(value - 20)
+        return f"二十{unit}个"
+    if 30 <= value <= 32:
+        unit = "" if value == 30 else _chinese_index(value - 30)
+        return f"三十{unit}个"
+    return f"{value}个"
diff --git a/src/fruit_mission/fruit_mission/mission_context.py b/src/fruit_mission/fruit_mission/mission_context.py
index 72ab08e..f473654 100644
--- a/src/fruit_mission/fruit_mission/mission_context.py
+++ b/src/fruit_mission/fruit_mission/mission_context.py
@@ -2,17 +2,18 @@
 
 from __future__ import annotations
 
+from collections import Counter
+from dataclasses import dataclass
 import json
 import math
 from pathlib import Path
-from dataclasses import dataclass
 import time
 from typing import Any, Dict, Mapping, Optional, Sequence, Union
+import uuid
 
 from action_msgs.msg import GoalStatus
 from geometry_msgs.msg import PointStamped, PoseStamped
 from nav2_msgs.action import FollowWaypoints
-from nav_msgs.msg import Odometry
 import rclpy
 from rclpy.action import ActionClient
 from rclpy.node import Node
@@ -29,8 +30,8 @@ from fruit_perception.msg import FruitDecision
 from fruit_mission.camera_target_planner_client import CameraTargetPlannerClient
 from fruit_mission.camera_target_planner_client import DEFAULT_EXTRINSIC_CONFIG
 from fruit_mission.config import _validate_qr_fallbacks
-from fruit_mission.decisions import merge_seen_types
-from fruit_mission.tts import PiperSpeaker
+from fruit_mission.decisions import format_produce_count, merge_seen_types, spoken_produce_name
+from fruit_mission.tts import RecordedSpeaker
 from fruit_mission.waypoint_route import MissionWaypointRoute
 from fruit_mission.waypoint_route import WaypointPose
 from fruit_qr_planner.planner import build_b_plan, build_c_plan
@@ -75,7 +76,8 @@ class MissionContext:
     def __init__(self, node: Node, config: Mapping[str, Any], speaker=None) -> None:
         self.node = node
         self.config = config
-        self.speaker = speaker or PiperSpeaker(config["tts"], node.get_logger())
+        self.speaker = speaker or RecordedSpeaker(config.get("tts", {}), node.get_logger())
+        self._pick_counts: Counter[str] = Counter()
         self.route = MissionWaypointRoute.from_file(
             Path(str(config["waypoint_file"])),
             str(config.get("mission_flow", "full")),
@@ -85,8 +87,12 @@ class MissionContext:
         self._current_waypoint_name: Optional[str] = None
         self._current_route_yaw: Optional[float] = None
         self._current_area: Optional[str] = None
-        self._latest_active_mode: Optional[str] = None
-        self._active_mode_generation = 0
+        self._active_window_id: Optional[str] = None
+        self._active_window_start_sec: int = 0
+        self._active_window_start_nanosec: int = 0
+        self._active_window_area: str = ""
+        self._active_window_purpose: str = ""
+        self._active_window_expected_type: str = ""
         self._latest_camera_types: Optional[list[str]] = None
         self._latest_camera_types_stamp_ns = 0
         self._latest_camera_target: Optional[PointStamped] = None
@@ -95,8 +101,6 @@ class MissionContext:
         self._latest_b_targets_stamp_ns = 0
         self._latest_b_plan: Optional[Dict[str, Any]] = None
         self._latest_c_plan: Optional[Dict[str, Any]] = None
-        self._base_odometry = None
-        self._base_received_at = 0.0
         self._pending_observation = None
         self._observation_side = None
         self._arm_home_confirmed = False
@@ -110,10 +114,14 @@ class MissionContext:
             spin_thread=False,
         )
 
-        self._mode_publisher = node.create_publisher(
+        self._observation_window_publisher = node.create_publisher(
             String,
-            config["perception_mode_topic"],
-            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
+            config["observation_window_topic"],
+            QoSProfile(
+                depth=10,
+                reliability=ReliabilityPolicy.RELIABLE,
+                durability=DurabilityPolicy.VOLATILE,
+            ),
         )
         self._expected_fruit_type_publisher = node.create_publisher(
             String,
@@ -126,21 +134,15 @@ class MissionContext:
             _latched_qos(),
         )
         self._camera_planner = CameraTargetPlannerClient(
-            node, config.get("camera_extrinsic_config", DEFAULT_EXTRINSIC_CONFIG)
+            node,
+            config.get("camera_extrinsic_config", DEFAULT_EXTRINSIC_CONFIG),
+            target_offset_m=config.get("target_offset_m", (0.0, 0.0, 0.0)),
         )
         self._follow_waypoints_client = ActionClient(
             node,
             FollowWaypoints,
             str(config["follow_waypoints_action"]),
         )
-        node.create_subscription(
-            Odometry, config['odometry_topic'], self._on_odometry, 10)
-        node.create_subscription(
-            String,
-            config["active_mode_topic"],
-            self._on_active_mode,
-            _latched_qos(),
-        )
         node.create_subscription(
             FruitDecision,
             config["decision_topic"],
@@ -535,71 +537,99 @@ class MissionContext:
             return False
         return self._wait_until(lambda: self._navigation_stopped, 10.0)
 
-    def _on_odometry(self, message: Odometry) -> None:
-        self._base_odometry = message
-        self._base_received_at = time.monotonic()
-
-    def stable_stop(self) -> bool:
-        """Both measured mechanisms must stay stationary before a new window."""
-        limits = self.config['stationary']
-        stable_since = None
-
-        def stationary():
-            nonlocal stable_since
-            message = self._base_odometry
-            base_ok = False
-            if message is not None:
-                stamp = message.header.stamp
-                age = (self.node.get_clock().now().nanoseconds
-                       - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
-                twist = message.twist.twist
-                values = (twist.linear.x, twist.linear.y, twist.linear.z,
-                          twist.angular.x, twist.angular.y, twist.angular.z)
-                base_ok = (
-                    message.header.frame_id == self.config['map_frame']
-                    and 0 <= age <= limits['odometry_max_age_sec']
-                    and time.monotonic() - self._base_received_at <= limits['odometry_max_age_sec']
-                    and all(math.isfinite(v) for v in values)
-                    and math.sqrt(sum(v * v for v in values[:3])) <= limits['linear_speed_mps']
-                    and math.sqrt(sum(v * v for v in values[3:])) <= limits['angular_speed_rps']
-                )
-            if not (base_ok and self._camera_planner.is_stationary(limits['joint_speed_rps'])):
-                stable_since = None
-                return False
-            now = time.monotonic()
-            if stable_since is None:
-                stable_since = now
-            return now - stable_since >= limits['duration_sec']
-
-        return self._wait_until(stationary, limits['timeout_sec'])
-
-    def set_perception_mode(self, mode: str) -> bool:
-        """Publish a perception mode and wait for the active-mode echo."""
-        message = String()
-        message.data = mode
-        previous_generation = self._active_mode_generation
-        self._mode_publisher.publish(message)
-        return self._wait_until(
-            lambda: self._active_mode_generation > previous_generation
-            and self._latest_active_mode == mode,
-            float(self.config['active_mode_timeout_sec']),
+    def open_observation_window(
+        self,
+        area: str,
+        purpose: str,
+        expected_type: Optional[str] = None,
+    ) -> str:
+        """Open a new observation window and broadcast its parameters."""
+        now = self.node.get_clock().now()
+        sec, nanosec = now.seconds_nanoseconds()
+        window_id = str(uuid.uuid4())
+        self._active_window_id = window_id
+        self._active_window_start_sec = sec
+        self._active_window_start_nanosec = nanosec
+        self._active_window_area = area
+        self._active_window_purpose = purpose
+        self._active_window_expected_type = str(expected_type or "")
+        self._latest_camera_target = None
+        self._latest_camera_types = None
+        self._latest_camera_types_stamp_ns = 0
+        self._latest_b_targets = None
+        self._latest_b_targets_header = None
+        self._latest_b_targets_stamp_ns = 0
+        if purpose == "qr":
+            if area == "B":
+                self._latest_b_plan = None
+            elif area == "C":
+                self._latest_c_plan = None
+
+        cmd = {
+            "window_id": window_id,
+            "valid": 1,
+            "start_sec": sec,
+            "start_nanosec": nanosec,
+            "area": area,
+            "purpose": purpose,
+            "expected_type": str(expected_type or ""),
+        }
+        msg = String()
+        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
+        self._observation_window_publisher.publish(msg)
+        self.node.get_logger().info(
+            f"Opened observation window '{window_id}' for area={area}, "
+            f"purpose={purpose}, expected='{expected_type or ''}'"
         )
+        return window_id
+
+    def close_observation_window(self) -> None:
+        """Close current observation window if open."""
+        if self._active_window_id is None:
+            return
+        window_id = self._active_window_id
+        self._active_window_id = None
+        cmd = {
+            "window_id": window_id,
+            "valid": 0,
+            "start_sec": self._active_window_start_sec,
+            "start_nanosec": self._active_window_start_nanosec,
+            "area": self._active_window_area,
+            "purpose": self._active_window_purpose,
+            "expected_type": self._active_window_expected_type,
+        }
+        msg = String()
+        msg.data = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
+        self._observation_window_publisher.publish(msg)
+        self.node.get_logger().info(f"Closed observation window '{window_id}'")
 
-    def wait_b_plan(self) -> Optional[Dict[str, Any]]:
-        """Wait for the latched B QR plan."""
+    def wait_b_plan(self, window_id: str) -> Optional[Dict[str, Any]]:
+        """Wait for the B QR plan matching current window."""
         self._wait_until(
-            lambda: self._latest_b_plan is not None,
+            lambda: self._latest_b_plan is not None
+            and self._latest_b_plan.get("observation_window_id") == window_id,
             float(self.config["qr_plan_timeout_sec"]),
         )
-        return self._latest_b_plan
+        if (
+            self._latest_b_plan is not None
+            and self._latest_b_plan.get("observation_window_id") == window_id
+        ):
+            return self._latest_b_plan
+        return None
 
-    def wait_c_plan(self) -> Optional[Dict[str, Any]]:
-        """Wait for the latched C QR plan."""
+    def wait_c_plan(self, window_id: str) -> Optional[Dict[str, Any]]:
+        """Wait for the C QR plan matching current window."""
         self._wait_until(
-            lambda: self._latest_c_plan is not None,
+            lambda: self._latest_c_plan is not None
+            and self._latest_c_plan.get("observation_window_id") == window_id,
             float(self.config["qr_plan_timeout_sec"]),
         )
-        return self._latest_c_plan
+        if (
+            self._latest_c_plan is not None
+            and self._latest_c_plan.get("observation_window_id") == window_id
+        ):
+            return self._latest_c_plan
+        return None
 
     def send_pick_direction(self, side: str) -> bool:
         """Execute the named observation preset and wait for completion."""
@@ -625,7 +655,6 @@ class MissionContext:
             raise ValueError('side must be left or right')
         if self._pending_observation is not None:
             raise RuntimeError('Previous observation has not been consumed')
-        self.set_perception_mode('OFF')
         future = None
         if self._arm_home_confirmed or self.recover_home():
             name = self.config['arm_presets'][f'observation_{side}']
@@ -711,47 +740,50 @@ class MissionContext:
         self._current_area_publisher.publish(message)
         return True
 
-    def announce(self, text: str, length_scale: Optional[float] = None) -> bool:
+    def record_pick(self, matched_type: Optional[str]) -> None:
+        """Record one confirmed harvested produce target."""
+        name = spoken_produce_name(matched_type)
+        if name and name != "未知果实":
+            self._pick_counts[name] += 1
+            self.node.get_logger().info(f"记录采摘工序完成：{name}，当前累计：{self._pick_counts[name]}")
+
+    def announce_summary(self) -> None:
+        """Speak final mission summary by produce category directly."""
+        self.announce_and_wait("任务流程结束")
+        total = sum(self._pick_counts.values())
+        if total == 0:
+            self.announce_and_wait("本次没有采摘记录")
+            return
+
+        self.announce_and_wait("本次采摘记录")
+        ordered_produces = ["苹果", "梨子", "南瓜", "西红柿", "辣椒", "洋葱"]
+        for produce in ordered_produces:
+            count = self._pick_counts.get(produce, 0)
+            if count > 0:
+                phrase = format_produce_count(produce, count)
+                self.announce_and_wait(phrase)
+
+    def announce(self, text: str, **_kwargs) -> bool:
         """Speak text through the configured TTS backend."""
         self.node.get_logger().info(f"播报：{text}")
-        if length_scale is not None:
-            try:
-                return bool(self.speaker.announce(text, length_scale=length_scale))
-            except TypeError:
-                pass
-        return self.speaker.announce(text)
+        return bool(self.speaker.announce(text))
 
     def announce_and_wait(
         self,
         text: str,
         lead_time_sec: float = 0.0,
-        length_scale: Optional[float] = None,
+        **_kwargs,
     ) -> bool:
         """Speak text and wait until playback completes or reaches its lead time."""
         self.node.get_logger().info(f"同步播报：{text}")
         announce_and_wait = getattr(self.speaker, "announce_and_wait", None)
         if callable(announce_and_wait):
-            if lead_time_sec > 0.0:
-                try:
-                    return bool(
-                        announce_and_wait(
-                            text,
-                            lead_time_sec=lead_time_sec,
-                            length_scale=length_scale,
-                        )
-                    )
-                except TypeError:
-                    pass
-            elif length_scale is not None:
-                try:
-                    return bool(announce_and_wait(text, length_scale=length_scale))
-                except TypeError:
-                    pass
-            return bool(announce_and_wait(text))
-        return self.speaker.announce(text)
+            return bool(announce_and_wait(text, lead_time_sec=lead_time_sec))
+        return bool(self.speaker.announce(text))
 
     def close(self) -> None:
         """Release mission runtime helpers."""
+        self.close_observation_window()
         self._camera_planner.request_stop()
         self.cancel_navigation()
         # Drain the STOP service request before tearing down its ROS client.
@@ -794,35 +826,31 @@ class MissionContext:
 
     def wait_camera_target(
         self,
-        fresh_after_ns: int,
+        window_id: str,
         expected_type: Optional[str],
     ) -> _CameraSelection:
         self._latest_camera_target = None
         self._latest_camera_types = None
         self._latest_camera_types_stamp_ns = 0
         seen_types: list[str] = []
-        latest_stamp_ns = fresh_after_ns
         deadline = time.monotonic() + float(self.config["camera_detection_timeout_sec"])
         while rclpy.ok() and time.monotonic() < deadline:
             rclpy.spin_once(self.node, timeout_sec=0.1)
-            if self._latest_camera_types_stamp_ns > fresh_after_ns:
-                latest_types = self._latest_camera_types or []
+            if self._active_window_id != window_id:
+                break
+            latest_types = self._latest_camera_types or []
+            if latest_types:
                 seen_types = merge_seen_types(seen_types, latest_types)
             if self._latest_camera_target is None:
                 continue
             target = self._latest_camera_target
-            stamp = target.header.stamp
-            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
-            if stamp_ns <= latest_stamp_ns:
-                self._latest_camera_target = None
-                continue
-            latest_stamp_ns = stamp_ns
             point = (
                 float(target.point.x),
                 float(target.point.y),
                 float(target.point.z),
             )
             if not all(math.isfinite(value) for value in point):
+                self._latest_camera_target = None
                 continue
             self.node.get_logger().info(
                 "Camera target received from perception window: "
@@ -857,7 +885,7 @@ class MissionContext:
 
     def wait_b_targets(
         self,
-        fresh_after_ns: int,
+        window_id: str,
     ) -> Optional[list[BTreeTarget]]:
         self._latest_b_targets = None
         self._latest_b_targets_header = None
@@ -865,10 +893,10 @@ class MissionContext:
         deadline = time.monotonic() + float(self.config["camera_detection_timeout_sec"])
         while rclpy.ok() and time.monotonic() < deadline:
             rclpy.spin_once(self.node, timeout_sec=0.1)
+            if self._active_window_id != window_id:
+                break
             if self._latest_b_targets is None:
                 continue
-            if self._latest_b_targets_stamp_ns <= fresh_after_ns:
-                continue
             targets = []
             for item in self._latest_b_targets:
                 if not bool(item.has_point):
@@ -944,13 +972,25 @@ class MissionContext:
             stamped.pose.orientation.w = math.cos(yaw * 0.5)
         return stamped
 
-    def _on_active_mode(self, message: String) -> None:
-        self._latest_active_mode = message.data
-        self._active_mode_generation += 1
-
     def _on_decision(self, message: FruitDecision) -> None:
-        self._latest_camera_types = list(message.types)
+        if (
+            self._active_window_id is None
+            or message.observation_window_id != self._active_window_id
+            or self._active_window_purpose != "harvest"
+        ):
+            return
         stamp = message.header.stamp
+        if (
+            stamp.sec < 0
+            or not 0 <= stamp.nanosec < 1_000_000_000
+            or stamp.sec < self._active_window_start_sec
+            or (
+                stamp.sec == self._active_window_start_sec
+                and stamp.nanosec < self._active_window_start_nanosec
+            )
+        ):
+            return
+        self._latest_camera_types = list(message.types)
         self._latest_camera_types_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
         if message.targets_ready:
             self._latest_b_targets = list(message.targets)
@@ -977,27 +1017,66 @@ class MissionContext:
     def _receive_plan(self, message: String, zone: str):
         """Reject invalid ROS input without throwing out of the executor callback."""
         try:
+            if self._active_window_id is None:
+                return None
+            if self._active_window_purpose != "qr":
+                return None
+            if self._active_window_area != zone:
+                return None
             plan = json.loads(message.data)
-            if plan['zone'] != zone:
-                raise ValueError('QR plan zone mismatch')
-            tasks = sorted(plan['tasks'], key=lambda task: task['sequence'])
-            if [task['sequence'] for task in tasks] != list(range(1, 9)):
-                raise ValueError('QR plan requires eight ordered tasks')
-            fallback = dict(self.config['qr_fallback'])
-            names = [task['target'] for task in tasks]
-            positions = [task['position_id'] for task in tasks]
-            if zone == 'B':
+            if not isinstance(plan, dict):
+                self.node.get_logger().warning(f"Non-object {zone} QR plan ignored")
+                return None
+            plan_window_id = plan.get("observation_window_id")
+            if plan_window_id != self._active_window_id:
+                return None
+            img_sec = plan.get("image_stamp_sec")
+            img_nanosec = plan.get("image_stamp_nanosec")
+            if (
+                isinstance(img_sec, bool)
+                or not isinstance(img_sec, int)
+                or isinstance(img_nanosec, bool)
+                or not isinstance(img_nanosec, int)
+                or not (0 <= img_nanosec < 1_000_000_000)
+                or not 0 <= img_sec <= 0x7FFFFFFF
+            ):
+                self.node.get_logger().warning(
+                    f"Invalid timestamp in {zone} QR plan: sec={img_sec}, nanosec={img_nanosec}"
+                )
+                return None
+            if (
+                img_sec < self._active_window_start_sec
+                or (
+                    img_sec == self._active_window_start_sec
+                    and img_nanosec < self._active_window_start_nanosec
+                )
+            ):
+                return None
+            if plan.get("zone") != zone:
+                raise ValueError("QR plan zone mismatch")
+            tasks = sorted(plan["tasks"], key=lambda task: task["sequence"])
+            if [task["sequence"] for task in tasks] != list(range(1, 9)):
+                raise ValueError("QR plan requires eight ordered tasks")
+            fallback = dict(self.config["qr_fallback"])
+            names = [task["target"] for task in tasks]
+            positions = [task["position_id"] for task in tasks]
+            if zone == "B":
                 if positions != list(range(1, 9)):
-                    raise ValueError('B tree positions must remain ordered')
-                fallback['b_targets'] = names
+                    raise ValueError("B tree positions must remain ordered")
+                fallback["b_targets"] = names
             else:
-                fallback['c_targets'], fallback['c_positions'] = names, positions
+                fallback["c_targets"], fallback["c_positions"] = names, positions
             _validate_qr_fallbacks(fallback)
-            if zone == 'B':
-                return build_b_plan('\n'.join(names))
-            return build_c_plan('\n'.join(names + [','.join(map(str, positions))]))
+            if zone == "B":
+                built = build_b_plan("\n".join(names))
+            else:
+                built = build_c_plan("\n".join(names + [",".join(map(str, positions))]))
+            built["observation_window_id"] = plan_window_id
+            built["image_stamp_sec"] = img_sec
+            built["image_stamp_nanosec"] = img_nanosec
+            return built
         except (ValueError, TypeError, KeyError) as error:
-            self.node.get_logger().warning(f'Invalid {zone} QR plan ignored: {error}')
+            self.node.get_logger().warning(f"Invalid {zone} QR plan ignored: {error}")
             return None
 
 
diff --git a/src/fruit_mission/fruit_mission/mission_node.py b/src/fruit_mission/fruit_mission/mission_node.py
index a073240..960273b 100644
--- a/src/fruit_mission/fruit_mission/mission_node.py
+++ b/src/fruit_mission/fruit_mission/mission_node.py
@@ -50,12 +50,12 @@ class FruitMissionNode(Node):
         """Select once; startup/cleanup are not mixed into competition composition."""
         ctx = self._mission_context
         try:
-            ctx.announce_and_wait("我们是重庆大学 bobocup 水果采摘队伍，现在开始采摘")
+            ctx.announce_and_wait("我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘")
             ctx.recover_home()
             self._entrypoint(ctx)
+            ctx.announce_summary()
         finally:
-            ctx.set_perception_mode("OFF")
-            ctx.announce("任务流程结束")
+            ctx.close_observation_window()
 
     def destroy_node(self) -> bool:
         """Stop mission helpers before destroying the ROS node."""
diff --git a/src/fruit_mission/fruit_mission/stack_readiness_client.py b/src/fruit_mission/fruit_mission/stack_readiness_client.py
index 3e189e9..d390f31 100644
--- a/src/fruit_mission/fruit_mission/stack_readiness_client.py
+++ b/src/fruit_mission/fruit_mission/stack_readiness_client.py
@@ -14,7 +14,6 @@ from rclpy.action import ActionClient
 from rclpy.node import Node
 from rclpy.time import Time
 from std_msgs.msg import Bool
-from std_msgs.msg import String
 from std_srvs.srv import Trigger
 from tf2_ros import Buffer
 from tf2_ros import TransformException
@@ -31,16 +30,12 @@ class StackReadinessClient(Node):
         self.declare_parameter(
             "bridge_ready_topic", "/auto_serial_bridge/ready"
         )
-        self.declare_parameter(
-            "active_mode_topic", "/fruit_perception/active_mode"
-        )
         self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
         self.declare_parameter("graph_discovery_timeout_sec", 1.0)
 
         self.bridge_ready_topic = self._string_parameter(
             "bridge_ready_topic"
         )
-        self.active_mode_topic = self._string_parameter("active_mode_topic")
         self.follow_waypoints_action = self._string_parameter(
             "follow_waypoints_action"
         )
@@ -49,7 +44,6 @@ class StackReadinessClient(Node):
         )
 
         self._bridge_ready = False
-        self._active_mode = ""
         self._tf_buffer = Buffer()
         self._tf_listener = TransformListener(
             self._tf_buffer,
@@ -62,12 +56,6 @@ class StackReadinessClient(Node):
             self._on_bridge_ready,
             latched_qos(),
         )
-        self._active_mode_subscription = self.create_subscription(
-            String,
-            self.active_mode_topic,
-            self._on_active_mode,
-            latched_qos(),
-        )
 
     def nodes_are_absent(self, expected_absent: list[str]) -> bool:
         """Return false when any named node already exists."""
@@ -222,6 +210,29 @@ class StackReadinessClient(Node):
             )
         return ready
 
+    def bridge_start_required(self) -> bool:
+        """Select bridge startup without treating hardware readiness as absence."""
+        self.wait_until(lambda: False, self.graph_discovery_timeout_sec)
+        bridge_nodes = [
+            name for name, _namespace in self.get_node_names_and_namespaces()
+            if name == "serial_controller"
+        ]
+        status_publishers = self.get_publishers_info_by_topic(
+            "/fruit_hardware_bridge/arm/status"
+        )
+        ready_publishers = self.get_publishers_info_by_topic(self.bridge_ready_topic)
+        for publishers, expected_type in (
+            (status_publishers, "fruit_hardware_interfaces/msg/ArmStatus"),
+            (ready_publishers, "std_msgs/msg/Bool"),
+        ):
+            if len(publishers) > 1 or any(
+                publisher.topic_type != expected_type for publisher in publishers
+            ):
+                raise ValueError("Conflicting serial bridge publishers or message types")
+        if len(bridge_nodes) > 1:
+            raise ValueError("Multiple serial_controller nodes are already running")
+        return not (bridge_nodes or status_publishers or ready_publishers)
+
     def wait_for_endpoint(
         self,
         topic: str,
@@ -245,16 +256,6 @@ class StackReadinessClient(Node):
             )
         return ready
 
-    def wait_for_active_mode(self, mode: str, timeout_sec: float) -> bool:
-        """Wait for the typed active perception mode."""
-        ready = self.wait_until(lambda: self._active_mode == mode, timeout_sec)
-        if not ready:
-            self.get_logger().error(
-                f"Timed out waiting for {self.active_mode_topic}={mode}; "
-                f"observed={self._active_mode or '<none>'}"
-            )
-        return ready
-
     def node_names(self) -> set[str]:
         """Return current fully qualified ROS graph node names."""
         return {
@@ -277,9 +278,6 @@ class StackReadinessClient(Node):
     def _on_bridge_ready(self, message: Bool) -> None:
         self._bridge_ready = bool(message.data)
 
-    def _on_active_mode(self, message: String) -> None:
-        self._active_mode = message.data
-
     def _string_parameter(self, name: str) -> str:
         return str(self.get_parameter(name).value)
 
@@ -308,6 +306,8 @@ def _parser() -> argparse.ArgumentParser:
     bridge = commands.add_parser("bridge")
     bridge.add_argument("--timeout", type=float, required=True)
 
+    commands.add_parser("bridge-start-required")
+
     planner = commands.add_parser("planner")
     planner.add_argument("--timeout", type=float, required=True)
 
@@ -319,10 +319,6 @@ def _parser() -> argparse.ArgumentParser:
     endpoint.add_argument("--count", type=int, required=True)
     endpoint.add_argument("--timeout", type=float, required=True)
 
-    active = commands.add_parser("active-mode")
-    active.add_argument("--mode", required=True)
-    active.add_argument("--timeout", type=float, required=True)
-
     transform = commands.add_parser("tf")
     transform.add_argument("--parent", required=True)
     transform.add_argument("--child", required=True)
@@ -351,6 +347,9 @@ def main(args=None) -> None:
             ready = node.wait_for_action(action_name, options.timeout)
         elif options.command == "bridge":
             ready = node.wait_for_bridge(options.timeout)
+        elif options.command == "bridge-start-required":
+            print("true" if node.bridge_start_required() else "false", flush=True)
+            ready = True
         elif options.command == "planner":
             ready = node.wait_for_planner(options.timeout)
         elif options.command == "endpoint":
@@ -360,8 +359,6 @@ def main(args=None) -> None:
                 options.count,
                 options.timeout,
             )
-        elif options.command == "active-mode":
-            ready = node.wait_for_active_mode(options.mode, options.timeout)
         elif options.command == "tf":
             ready = node.wait_for_transform(
                 options.parent,
diff --git a/src/fruit_mission/fruit_mission/tts.py b/src/fruit_mission/fruit_mission/tts.py
index a0ac1b3..393df51 100644
--- a/src/fruit_mission/fruit_mission/tts.py
+++ b/src/fruit_mission/fruit_mission/tts.py
@@ -1,117 +1,251 @@
-"""Subprocess-backed Piper text-to-speech wrapper."""
+"""Offline prebuilt WAV spliced text-to-speech speaker."""
 
 from __future__ import annotations
 
 from dataclasses import dataclass
+import os
 from pathlib import Path
-from queue import Queue
+from queue import Empty, Queue
+import re
 import subprocess
-from threading import Event
-from threading import Lock
-from threading import Thread
-from typing import Callable, Mapping
+from threading import Event, Lock, Thread
 import time
+from typing import Dict, List, Mapping, Optional, Tuple
 import wave
 
+import yaml
+
 
 @dataclass
 class _SpeechJob:
     text: str
+    tokens: Optional[List[Tuple[str, Optional[Path], float]]] = None
     done: Event | None = None
     playback_started: Event | None = None
-    length_scale: float | None = None
     playback_duration_sec: float | None = None
     result: bool = False
 
 
-class PiperSpeaker:
-    """Generate speech with Piper and play it with aplay."""
+class RecordedSpeaker:
+    """Concatenate pre-recorded Wis'adel phrase WAV files and play via aplay."""
 
     def __init__(
         self,
         config: Mapping[str, object],
         logger,
-        run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
     ) -> None:
-        self._piper = str(config["piper_executable"])
-        self._model = str(config["model_path"])
-        self._model_config = str(config["config_path"])
-        self._aplay = str(config["aplay_executable"])
-        self._wav_path = Path(str(config["wav_path"]))
+        self._logger = logger
+        self._manifest_path = Path(str(config.get(
+            "manifest_path",
+            "/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml"
+        )))
+        self._aplay = str(config.get("aplay_executable", "/usr/bin/aplay"))
+        self._wav_path = Path(str(config.get(
+            "wav_path",
+            "/home/rc4/fruit_ws/tmp/fruit_mission/stitched_speech.wav"
+        )))
         fallback_timeout = float(config.get("timeout_sec", 10.0))
-        self._generation_timeout_sec = min(
-            10.0, float(config.get("generation_timeout_sec", fallback_timeout))
-        )
         self._playback_timeout_sec = min(
             10.0, float(config.get("playback_timeout_sec", fallback_timeout))
         )
-        length_scale = config.get("length_scale")
-        self._length_scale = None if length_scale is None else float(length_scale)
-        self._logger = logger
-        self._run = run
+
+        self._sample_rate = 32000
+        self._channels = 1
+        self._sampwidth = 2
+        self._audio_dir: Optional[Path] = None
+        self._phrase_map: Dict[str, Path] = {}
+        self._sorted_phrases: List[str] = []
+
+        self._load_manifest()
+
         self._queue: Queue[_SpeechJob | None] = Queue()
         self._closed = False
         self._close_lock = Lock()
+        self._current_process: Optional[subprocess.Popen] = None
+        self._process_lock = Lock()
+        self._abort_event = Event()
+
         self._worker = Thread(target=self._worker_loop, daemon=True)
         self._worker.start()
 
-    def announce(self, text: str, length_scale: float | None = None) -> bool:
+    def _load_manifest(self) -> None:
+        if not self._manifest_path.is_file():
+            self._logger.warning(f"语音清单文件不存在：{self._manifest_path}，将无法发声")
+            return
+        try:
+            with self._manifest_path.open("r", encoding="utf-8") as f:
+                data = yaml.safe_load(f) or {}
+            self._sample_rate = int(data.get("sample_rate", 32000))
+            self._channels = int(data.get("channels", 1))
+            self._sampwidth = int(data.get("sample_width", 2))
+
+            raw_audio_dir = data.get("audio_dir", "../sounds/wisadel")
+            if os.path.isabs(raw_audio_dir):
+                self._audio_dir = Path(raw_audio_dir)
+            else:
+                self._audio_dir = (self._manifest_path.parent / raw_audio_dir).resolve()
+
+            self._phrase_map.clear()
+            for item in data.get("phrases", []):
+                text = str(item["text"]).strip()
+                filename = item["file"]
+                self._phrase_map[text] = self._audio_dir / filename
+
+            self._sorted_phrases = sorted(self._phrase_map.keys(), key=len, reverse=True)
+            self._logger.info(f"成功加载预生成语音清单：共 {len(self._phrase_map)} 个短语，目录：{self._audio_dir}")
+        except Exception as error:
+            self._logger.warning(f"读取语音清单失败：{error}")
+
+    def tokenize(self, text: str) -> Optional[List[Tuple[str, Optional[Path], float]]]:
+        """Greedy longest-match tokenizer against phrase_map with pause handling."""
+        if not self._sorted_phrases:
+            return None
+        tokens: List[Tuple[str, Optional[Path], float]] = []
+        i = 0
+        n = len(text)
+        while i < n:
+            char = text[i]
+            if char in ("，", ",", " ", "、"):
+                if tokens:
+                    last = tokens[-1]
+                    tokens[-1] = (last[0], last[1], max(last[2], 0.15))
+                i += 1
+                continue
+            elif char in ("。", ".", "；", ";", "\n"):
+                if tokens:
+                    last = tokens[-1]
+                    tokens[-1] = (last[0], last[1], max(last[2], 0.25))
+                i += 1
+                continue
+
+            matched = False
+            sub = text[i:]
+            for phrase in self._sorted_phrases:
+                if sub.startswith(phrase):
+                    tokens.append((phrase, self._phrase_map[phrase], 0.0))
+                    i += len(phrase)
+                    matched = True
+                    break
+
+            if not matched:
+                self._logger.warning(f"语音文案无法完整匹配到预置词表：'{text[i:i+10]}...' (全文: '{text}')")
+                return None
+        return tokens
+
+    def estimate_duration_sec(self, tokens: List[Tuple[str, Optional[Path], float]]) -> float:
+        """Estimate playback duration for a token list in seconds."""
+        duration = 0.0
+        for _text, wav_path, pause_sec in tokens:
+            duration += pause_sec
+            if wav_path and wav_path.is_file():
+                try:
+                    with wave.open(str(wav_path), "rb") as wf:
+                        rate = wf.getframerate()
+                        if rate > 0:
+                            duration += wf.getnframes() / rate
+                except Exception:
+                    pass
+        return duration
+
+    def announce(self, text: str, **_kwargs) -> bool:
         """Queue text for speech without blocking mission execution."""
         if not text:
             return True
+        items = [s.strip() for s in re.split(r"[；\n]+", str(text)) if s.strip()]
+        if not items:
+            return True
+
         with self._close_lock:
             if self._closed:
                 return False
-            self._queue.put(_SpeechJob(str(text), length_scale=length_scale))
+            for item in items:
+                tokens = self.tokenize(item)
+                self._queue.put(_SpeechJob(item, tokens=tokens))
         return True
 
     def announce_and_wait(
         self,
         text: str,
         lead_time_sec: float = 0.0,
-        length_scale: float | None = None,
+        **_kwargs,
     ) -> bool:
         """Queue text and wait until playback finishes or reaches its lead time."""
         if not text:
             return True
-        done = Event()
-        playback_started = Event()
-        job = _SpeechJob(
-            str(text),
-            done,
-            playback_started,
-            length_scale=length_scale,
-        )
+        items = [s.strip() for s in re.split(r"[；\n]+", str(text)) if s.strip()]
+        if not items:
+            return True
+
+        jobs: List[_SpeechJob] = []
+        total_duration_sec = 0.0
+        first_playback_started = Event()
+        last_done = Event()
+
+        for idx, item in enumerate(items):
+            tokens = self.tokenize(item)
+            duration = self.estimate_duration_sec(tokens) if tokens else 0.0
+            total_duration_sec += duration
+            p_started = first_playback_started if idx == 0 else None
+            done = last_done if idx == len(items) - 1 else None
+            jobs.append(_SpeechJob(
+                item,
+                tokens=tokens,
+                done=done,
+                playback_started=p_started,
+                playback_duration_sec=duration,
+            ))
+
         with self._close_lock:
             if self._closed:
                 return False
-            self._queue.put(job)
+            for j in jobs:
+                self._queue.put(j)
+
         if lead_time_sec <= 0.0:
-            if not done.wait(10.0):
-                self._logger.warning('语音等待超时，继续任务')
+            if not last_done.wait(10.0):
+                self._logger.warning("语音等待超时，继续任务")
                 return False
-            return job.result
+            return jobs[-1].result
 
         deadline = time.monotonic() + 10.0
-        if not playback_started.wait(10.0):
-            self._logger.warning('语音开始等待超时，继续任务')
+        if not first_playback_started.wait(10.0):
+            self._logger.warning("语音开始等待超时，继续任务")
             return False
-        if job.playback_duration_sec is None:
-            return done.wait(max(0.0, deadline - time.monotonic())) and job.result
-        remaining_sec = max(job.playback_duration_sec - float(lead_time_sec), 0.0)
+
+        remaining_sec = max(total_duration_sec - float(lead_time_sec), 0.0)
         budget = max(0.0, deadline - time.monotonic())
-        if done.wait(min(remaining_sec, budget)):
-            return job.result
+        wait_sec = min(remaining_sec, budget)
+        if last_done.wait(wait_sec):
+            return jobs[-1].result
         return remaining_sec <= budget
 
     def close(self, wait: bool = False) -> None:
-        """Stop the background speaker worker."""
+        """Stop playback immediately and discard queued items."""
         with self._close_lock:
             if not self._closed:
                 self._closed = True
+                self._abort_event.set()
+                # Clear queue
+                while True:
+                    try:
+                        self._queue.get_nowait()
+                        self._queue.task_done()
+                    except Empty:
+                        break
+                # Kill current aplay if running
+                with self._process_lock:
+                    if self._current_process is not None and self._current_process.poll() is None:
+                        try:
+                            self._current_process.terminate()
+                            self._current_process.wait(timeout=0.5)
+                        except Exception:
+                            try:
+                                self._current_process.kill()
+                            except Exception:
+                                pass
                 self._queue.put(None)
         if wait:
-            self._worker.join(timeout=10.0)
+            self._worker.join(timeout=2.0)
 
     def _worker_loop(self) -> None:
         while True:
@@ -119,6 +253,9 @@ class PiperSpeaker:
             try:
                 if job is None:
                     return
+                if self._abort_event.is_set():
+                    job.result = False
+                    return
                 job.result = self._speak(job)
             finally:
                 if job is not None and job.playback_started is not None:
@@ -128,52 +265,76 @@ class PiperSpeaker:
                 self._queue.task_done()
 
     def _speak(self, job: _SpeechJob) -> bool:
-        """Speak text and return whether audio generation and playback succeeded."""
+        """Stitch audio from tokens and play via aplay."""
+        if job.tokens is None:
+            self._logger.warning(f"播报跳过（素材无法覆盖）：'{job.text}'")
+            return False
+
+        # Validate that all required WAV files exist and are valid
+        for text, wav_path, _pause in job.tokens:
+            if wav_path is None or not wav_path.is_file():
+                self._logger.warning(f"播报跳过（素材缺失）：'{text}' -> {wav_path}")
+                return False
+
+        if not Path(self._aplay).is_file():
+            self._logger.warning(f"播报失败（播放器不存在）：{self._aplay}")
+            return False
+
         try:
             self._wav_path.parent.mkdir(parents=True, exist_ok=True)
-            command = [
-                self._piper,
-                "--model",
-                self._model,
-                "--config",
-                self._model_config,
-                "--output_file",
-                str(self._wav_path),
-            ]
-            length_scale = (
-                job.length_scale
-                if job.length_scale is not None
-                else self._length_scale
-            )
-            if length_scale is not None:
-                command.extend(["--length_scale", str(length_scale)])
-            self._run(
-                command,
-                input=job.text,
-                text=True,
-                check=True,
-                timeout=self._generation_timeout_sec,
-            )
-            job.playback_duration_sec = self._wav_duration_sec()
+            tmp_wav = self._wav_path.with_suffix(f".tmp_{os.getpid()}.wav")
+
+            # Concatenate PCM
+            total_frames = 0
+            with wave.open(str(tmp_wav), "wb") as out_wf:
+                out_wf.setnchannels(self._channels)
+                out_wf.setsampwidth(self._sampwidth)
+                out_wf.setframerate(self._sample_rate)
+
+                for _text, wav_path, pause_sec in job.tokens:
+                    with wave.open(str(wav_path), "rb") as in_wf:
+                        frames = in_wf.readframes(in_wf.getnframes())
+                        out_wf.writeframes(frames)
+                        total_frames += in_wf.getnframes()
+
+                    if pause_sec > 0.0:
+                        pause_frames = int(self._sample_rate * pause_sec)
+                        silence = b"\x00" * (pause_frames * self._channels * self._sampwidth)
+                        out_wf.writeframes(silence)
+                        total_frames += pause_frames
+
+            tmp_wav.replace(self._wav_path)
+            job.playback_duration_sec = total_frames / self._sample_rate
+
             if job.playback_started is not None:
                 job.playback_started.set()
-            self._run(
-                [self._aplay, str(self._wav_path)],
-                check=True,
-                timeout=self._playback_timeout_sec,
-            )
+
+            if self._abort_event.is_set():
+                return False
+
+            cmd = [self._aplay, str(self._wav_path)]
+            with self._process_lock:
+                if self._abort_event.is_set():
+                    return False
+                self._current_process = subprocess.Popen(
+                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
+                )
+
+            try:
+                self._current_process.wait(timeout=self._playback_timeout_sec)
+            except subprocess.TimeoutExpired:
+                self._logger.warning(f"语音播放超时（>{self._playback_timeout_sec}s），强制终止")
+                self._current_process.kill()
+                return False
+            finally:
+                with self._process_lock:
+                    self._current_process = None
+
+            return True
         except Exception as error:
-            self._logger.warning(f"语音播报失败，继续任务：{error}")
+            self._logger.warning(f"语音播报异常：{error}")
             return False
-        return True
 
-    def _wav_duration_sec(self) -> float | None:
-        """Read the generated WAV duration for early mission continuation."""
-        try:
-            with wave.open(str(self._wav_path), "rb") as wav_file:
-                frame_rate = wav_file.getframerate()
-                if frame_rate <= 0:
-                    return None
-                return wav_file.getnframes() / frame_rate
-        except (OSError, EOFError, wave.Error):
-            return None
+
+# Retain PiperSpeaker as alias for compatibility
+PiperSpeaker = RecordedSpeaker
diff --git a/src/fruit_perception/CMakeLists.txt b/src/fruit_perception/CMakeLists.txt
index 91fa84a..bb9d677 100644
--- a/src/fruit_perception/CMakeLists.txt
+++ b/src/fruit_perception/CMakeLists.txt
@@ -16,7 +16,6 @@ find_package(
 find_package(cv_bridge REQUIRED)
 find_package(geometry_msgs REQUIRED)
 find_package(OpenVINO 2024.6 EXACT REQUIRED COMPONENTS Runtime)
-find_package(rcl_interfaces REQUIRED)
 find_package(rclcpp REQUIRED)
 find_package(rclcpp_components REQUIRED)
 find_package(realsense2_camera_msgs REQUIRED)
@@ -41,22 +40,6 @@ add_library(
   src/target_localization.cpp
 )
 
-add_library(perception_mode src/perception_mode.cpp)
-target_include_directories(perception_mode PUBLIC
-  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
-  $<INSTALL_INTERFACE:include>
-)
-target_compile_features(perception_mode PUBLIC cxx_std_17)
-
-add_executable(camera_mode_manager src/camera_mode_manager.cpp)
-target_compile_features(camera_mode_manager PUBLIC cxx_std_17)
-target_link_libraries(camera_mode_manager perception_mode)
-ament_target_dependencies(
-  camera_mode_manager
-  rcl_interfaces
-  rclcpp
-  std_msgs
-)
 target_include_directories(detection_parser PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
@@ -81,7 +64,6 @@ target_compile_features(fruit_perception_component PUBLIC cxx_std_17)
 target_link_libraries(
   fruit_perception_component
   detection_parser
-  perception_mode
   openvino::runtime
   ${OpenCV_LIBRARIES}
 )
@@ -104,7 +86,7 @@ rclcpp_components_register_node(
   EXECUTABLE fruit_perception_node
 )
 
-install(TARGETS camera_mode_manager detection_parser perception_mode fruit_perception_component
+install(TARGETS detection_parser fruit_perception_component
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION lib/${PROJECT_NAME}
@@ -135,9 +117,6 @@ if(BUILD_TESTING)
   target_link_libraries(test_detection_output detection_parser ${OpenCV_LIBRARIES})
   target_include_directories(test_detection_output PRIVATE ${OpenCV_INCLUDE_DIRS})
   ament_target_dependencies(test_detection_output sensor_msgs)
-
-  ament_add_gtest(test_perception_mode test/test_perception_mode.cpp)
-  target_link_libraries(test_perception_mode perception_mode)
 endif()
 
 ament_export_dependencies(rosidl_default_runtime)
diff --git a/src/fruit_perception/MODEL_UPDATE_RC4.md b/src/fruit_perception/MODEL_UPDATE_RC4.md
index 9a84142..f761e69 100644
--- a/src/fruit_perception/MODEL_UPDATE_RC4.md
+++ b/src/fruit_perception/MODEL_UPDATE_RC4.md
@@ -1,8 +1,8 @@
 # RC4 果蔬模型更新与识别逻辑迁移说明
 
-> **历史交付说明，不是当前部署指南。**两阶段模型已接入，左右 USB 相机及 SIDES/TREE、FruitDecision.source 已删除。
-> 当前模式为 OFF/QR/DETECT；A 对外为 ripe/unripe，C 辣椒为 chili_*，不是下文历史方案拟议的全区 pepper_* 标签。
-> 当前配置、命令及验收边界见 [TWO_STAGE_RUNTIME.md](docs/TWO_STAGE_RUNTIME.md)。下文“当前/目标/后续”均指原交付时点，不要重复实施或执行其旧构建命令。模型归档内原说明及校验和不修改。
+> **历史交付说明，不是当前部署指南。**两阶段模型已接入，左右 USB 相机、旧模式及 CameraModeManager 已彻底删除。
+> 当前架构为单 camera 常开与持续推理，由观察窗口控制数据采纳；A 对外为 ripe/unripe，C 辣椒为 chili_*，不是下文历史方案拟议的全区 pepper_* 标签。
+> 当前配置、命令及验收边界见 [TWO_STAGE_RUNTIME.md](docs/TWO_STAGE_RUNTIME.md) 与 [README.md](README.md)。下文“当前/目标/后续”均指原交付时点，不要重复实施或执行其旧构建命令。模型归档内原说明及校验和不修改。
 
 ## 1. 本次交付与边界
 
diff --git a/src/fruit_perception/README.md b/src/fruit_perception/README.md
index e227d6d..8308fa5 100644
--- a/src/fruit_perception/README.md
+++ b/src/fruit_perception/README.md
@@ -4,42 +4,49 @@ ROS 2 Humble C++ fruit detection using OpenVINO 2024.6 on Intel `GPU.0`.
 The species detectors and selected ripeness classifiers are initialized once
 from the configured source-directory `model_root`.
 The system uses exclusively the depth camera (`camera`), with no side USB cameras.
-The default runtime mode is `OFF`; mission commands select when detection or QR scanning is active.
+Camera streams (Color, Aligned Depth, RGBD) remain continuously active and models
+run continuous inference on incoming frames. Target evidence accumulation and
+harvest target output are strictly gated by task observation windows.
 
 `camera` denotes the depth camera (currently Intel RealSense D435i). The driver node
-is `/camera` in the root namespace, with native topics and parameter services
-under `/camera/*`. Project configuration uses `camera`, `enable_camera`, and
-`camera_*` keys.
+is `/camera` in the root namespace, with native topics under `/camera/*`.
+Project configuration uses `camera`, `enable_camera`, and `camera_*` keys.
+The legacy `camera_mode_manager` node and hardware stream stop/start requests have been removed.
 
 The `camera` component and `FruitPerceptionNode` run in one
 `component_container` with intra-process communication enabled. This keeps raw
 images out of the inter-process DDS transport path. The generated standalone
 `fruit_perception_node` executable remains available for isolated debugging.
 
-## Runtime modes
-
-Publish one of the three uppercase values to `/fruit_mission/perception_mode`:
-
-- `OFF`: stop camera capture and inference.
-- `QR`: capture camera RGBD; only the QR planner consumes color frames.
-- `DETECT`: camera RGBD acquisition and fruit detection active.
-
-When testing from the ROS 2 CLI, quote every string value:
-
-```bash
-ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'OFF'}"
-ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'DETECT'}"
+## Observation windows
+
+Perception and QR decoding are governed by timestamped observation windows published
+on `/fruit_mission/observation_window` as JSON strings:
+
+```json
+{
+  "window_id": "<uuid-string>",
+  "valid": 1,
+  "start_sec": 1712345678,
+  "start_nanosec": 123456789,
+  "area": "A",
+  "purpose": "harvest",
+  "expected_type": "ripe"
+}
 ```
 
-Successful transitions are published on `/fruit_perception/active_mode` with
-Reliable, Transient Local QoS. Invalid commands and camera service failures do
-not change or republish the active mode. `QR <-> DETECT` transitions only switch
-inference consumers without restarting the camera streams.
+- `valid`: strictly integer `1` (open window) or `0` (close window).
+- `start_sec` / `start_nanosec`: image capture cutoff timestamp (ROS time). Frames with capture stamps earlier than this cutoff are ignored for evidence accumulation.
+- `area`: `"A"`, `"B"`, or `"C"`. Observation windows do not accept D.
+- `purpose`: `"harvest"` (accumulate pick evidence) or `"qr"` (QR scanning only).
+- `expected_type`: target filter string (e.g. `"ripe"` or species label; empty string disables filtering).
+
+When no harvest window is open or a frame was captured before window start, the node
+continues to publish 2D/3D raw detections and annotated images for diagnostics, but
+`observation_window_id` is empty (`""`), `has_target` is false, and `targets_ready` is false.
+Target locking and candidate accumulation only occur within a valid harvest window.
 
-The RealSense parameter services appear before the device initialization thread
-declares `enable_color` and `enable_depth`. At startup, the camera mode manager
-polls both values through `get_parameters` and only begins its `OFF`
-reset after both are declared as Boolean parameters.
+Closing a window (`valid: 0`) immediately revokes evidence accumulation and resets selectors.
 
 The camera node synchronizes color and depth, aligns depth to color, and publishes
 both aligned images in one RGBD message (`/camera/rgbd`).
@@ -86,16 +93,16 @@ The detector chooses its model and candidate selection from the current
   selecting and running the pepper ripeness classifier.
 - Area `D`: reuses B's detector, ripeness classification and multi-target selection.
 
-The external debug monitor expects image streams in both `QR` and `DETECT`;
+The external debug monitor expects active camera streams whenever perception is running;
 missing streams are reported as `STALE`, not `IDLE`.
 
 The decision stream on `/fruit_perception/decision` contains:
 
 - header matching aligned depth
+- `observation_window_id`: active harvest window ID, or empty string when outside window
 - detected fruit `types` in confidence order
 - all detections in confidence order (`detections`), including ROI and optional 3D points
-- `has_target` and `target_point` when single-target lock is acquired and on subsequent
-  matching frames (Areas A/C), or when targets are ready (Area B/D)
+- `has_target` and `target_point` when single-target lock is acquired within a valid window (Areas A/C), or when targets are ready (Area B/D)
 - B/D zone `targets_ready` and up to two aggregated `targets`
 - annotated RGB8 image on `/fruit_perception/camera/annotated_image`
 
diff --git a/src/fruit_perception/config/fruit_perception.yaml b/src/fruit_perception/config/fruit_perception.yaml
index e9ee191..4ba9a9b 100644
--- a/src/fruit_perception/config/fruit_perception.yaml
+++ b/src/fruit_perception/config/fruit_perception.yaml
@@ -37,8 +37,7 @@ fruit_perception_node:
     camera_rgbd_heartbeat_topic: "/fruit_perception/camera/rgbd_heartbeat"
     decision_topic: "/fruit_perception/decision"
     camera_annotated_topic: "/fruit_perception/camera/annotated_image"
-    perception_mode_topic: "/fruit_mission/perception_mode"
-    active_mode_topic: "/fruit_perception/active_mode"
+    observation_window_topic: "/fruit_mission/observation_window"
     current_area_topic: "/fruit_mission/current_area"
     expected_fruit_type_topic: "/fruit_mission/expected_fruit_type"
     model_root: "/home/rc4/fruit_ws/src/fruit_perception/model"
@@ -61,10 +60,3 @@ fruit_perception_node:
     camera_b_target_max_count: 2
     camera_b_target_association_radius_m: 0.03
     camera_b_target_outlier_sigma: 2.0
-
-camera_mode_manager:
-  ros__parameters:
-    perception_mode_topic: "/fruit_mission/perception_mode"
-    active_mode_topic: "/fruit_perception/active_mode"
-    camera_parameter_service: "/camera/set_parameters"
-    camera_get_parameters_service: "/camera/get_parameters"
diff --git a/src/fruit_perception/docs/TWO_STAGE_RUNTIME.md b/src/fruit_perception/docs/TWO_STAGE_RUNTIME.md
index 79134e4..c917724 100644
--- a/src/fruit_perception/docs/TWO_STAGE_RUNTIME.md
+++ b/src/fruit_perception/docs/TWO_STAGE_RUNTIME.md
@@ -17,7 +17,7 @@
 - YOLO 的缩放使用 OpenCV 线性插值，并非 PIL/torchvision 抗锯齿缩放的逐像素复刻；相同真实 ROI 的跨端概率一致性和准确率留待后续识别验收，当前仅证明运行契约与性能。
 - 分类器 reshape 为静态 batch=1，逐 ROI 推理；不限制检测数，不为了帧率跳过分类。
 - 内部结果按模型品种组成熟度标签；对外 A 区保留 `ripe/unripe`，C 区辣椒映射为 `chili_ripe/chili_unripe`，其他品种使用 `<crop>_ripe/unripe`（斜杠表示两个字符串）。检测置信度仍作为 ROS confidence，分类概率保存在内部结果/DEBUG 日志。GUI 物种检测标签如 `pepper` 不等于正式 ROS 标签。
-- 分类失败丢弃该 ROI，不假造 unripe。切模式/区域后旧在途帧不发布。
+- 分类失败丢弃该 ROI，不假造 unripe。开窗/关窗及区域切换后旧在途帧不累积为当前窗口证据。
 
 模型固定 `GPU.0` 和 FP16 推理提示，检测使用可调 `performance_hint=latency|throughput`，分类使用 LATENCY；不使用含 CPU 的 AUTO/HETERO/MULTI。保留输入 f32 契约，内部 FP16 不重写原 ONNX。
 本机设置 GPU `queue_throttle=LOW`，减少驱动在 CPU 上的忙轮询；离线基准中检测+2 ROI 的进程 CPU 从约 104% 降到约 25%，吞吐约下降 3%。不是把模型切到 CPU，也不是降低输入尺寸。
@@ -41,7 +41,7 @@ B 区经过短窗口预观察，当前存活候选全部确认且品种一致后
 
 这些数值是可调起始值，不是实景标定。清晰 A/C 最快在第 6 个处理帧确认，清晰 B 在第 8 个处理帧确认；
 不能把相机 30 FPS 当成实际两阶段处理速度，亦不能据此保证 1 秒识别。
-本轮不改深度算法、相机 OFF/开启语义、运动服务、实测静止或 STOP 安全链。
+相机视频流常开、持续推理，仅在到位后观察窗口累积确认证据；保留运动服务、未知运动 STOP 及恢复安全链。
 
 ## 构建和启动
 
@@ -56,9 +56,9 @@ ros2 launch fruit_perception fruit_perception.launch.py \
 # 比较统一分类器：停掉上一次 launch 后使用 ripeness_model:=yolo
 ```
 
-模型选择只在启动时生效。默认 OFF 不推理；根据当前区域进入 DETECT，QR 不做果蔬推理。
+模型选择只在启动时生效。相机视频流与模型推理常开并持续运行；根据 `/fruit_mission/observation_window` 观察窗口采纳与累积证据，窗口外不输出采摘目标。
 相机和感知 component 在同一容器并启用进程内通信。仅感知验收时禁止启动任务或串口桥。
-模式及区域命令见[模式操作](../../../docs/PERCEPTION_MODE_COMMANDS.md)。
+窗口接口及操作见[观察窗口参考](../../../docs/PERCEPTION_MODE_COMMANDS.md)。
 
 性能日志每五秒输出一次（需把感知节点 logger 设为 INFO）：
 
diff --git a/src/fruit_perception/include/fruit_perception/perception_mode.hpp b/src/fruit_perception/include/fruit_perception/perception_mode.hpp
deleted file mode 100644
index 9b02c8e..0000000
--- a/src/fruit_perception/include/fruit_perception/perception_mode.hpp
+++ /dev/null
@@ -1,44 +0,0 @@
-#ifndef FRUIT_PERCEPTION__PERCEPTION_MODE_HPP_
-#define FRUIT_PERCEPTION__PERCEPTION_MODE_HPP_
-
-#include <cstddef>
-#include <optional>
-#include <string>
-#include <vector>
-
-namespace fruit_perception
-{
-
-enum class PerceptionMode {kOff, kQr, kDetect};
-enum class CameraAction {kStopCamera, kStartCamera};
-
-std::optional<PerceptionMode> parse_perception_mode(const std::string & value);
-const char * to_string(PerceptionMode mode);
-bool runs_camera_inference(PerceptionMode mode);
-std::vector<CameraAction> plan_camera_transition(
-  PerceptionMode current, PerceptionMode target);
-std::vector<CameraAction> plan_startup_reset();
-
-class CameraModeState
-{
-public:
-  bool begin_startup_reset();
-  bool begin(PerceptionMode target);
-  std::optional<CameraAction> current_action() const;
-  void complete_action();
-  void fail();
-  bool transitioning() const;
-  std::optional<PerceptionMode> active_mode() const;
-
-private:
-  void finish_if_no_actions();
-
-  std::optional<PerceptionMode> active_mode_;
-  std::optional<PerceptionMode> target_mode_;
-  std::vector<CameraAction> actions_;
-  std::size_t action_index_{0U};
-};
-
-}  // namespace fruit_perception
-
-#endif  // FRUIT_PERCEPTION__PERCEPTION_MODE_HPP_
diff --git a/src/fruit_perception/launch/fruit_perception.launch.py b/src/fruit_perception/launch/fruit_perception.launch.py
index 0a08c75..dac18bc 100644
--- a/src/fruit_perception/launch/fruit_perception.launch.py
+++ b/src/fruit_perception/launch/fruit_perception.launch.py
@@ -133,15 +133,6 @@ def generate_launch_description():
         ],
     )
 
-    camera_mode_manager = Node(
-        package='fruit_perception',
-        executable='camera_mode_manager',
-        name='camera_mode_manager',
-        output='screen',
-        emulate_tty=True,
-        parameters=[common_config, {'use_sim_time': use_sim_time}],
-        arguments=['--ros-args', '--log-level', 'warn'],
-    )
     qr_planner = Node(
         package='fruit_qr_planner',
         executable='fruit_qr_planner_node',
@@ -184,7 +175,6 @@ def generate_launch_description():
         declare_performance_hint,
         declare_perception_log_level,
         container,
-        camera_mode_manager,
         qr_planner,
         GroupAction(
             actions=[serial_bridge, serial_bridge_debug],
diff --git a/src/fruit_perception/msg/FruitDecision.msg b/src/fruit_perception/msg/FruitDecision.msg
index d26280b..c0b07f3 100644
--- a/src/fruit_perception/msg/FruitDecision.msg
+++ b/src/fruit_perception/msg/FruitDecision.msg
@@ -1,4 +1,5 @@
 std_msgs/Header header
+string observation_window_id
 string[] types
 FruitDetection[] detections
 FruitTarget[] targets
diff --git a/src/fruit_perception/package.xml b/src/fruit_perception/package.xml
index 2a509ec..a6824b0 100644
--- a/src/fruit_perception/package.xml
+++ b/src/fruit_perception/package.xml
@@ -13,7 +13,6 @@
   <depend>ament_index_cpp</depend>
   <depend>cv_bridge</depend>
   <depend>geometry_msgs</depend>
-  <depend>rcl_interfaces</depend>
   <depend>rclcpp</depend>
   <depend>rclcpp_components</depend>
   <depend>realsense2_camera_msgs</depend>
diff --git a/src/fruit_perception/src/camera_mode_manager.cpp b/src/fruit_perception/src/camera_mode_manager.cpp
deleted file mode 100644
index c2a5229..0000000
--- a/src/fruit_perception/src/camera_mode_manager.cpp
+++ /dev/null
@@ -1,208 +0,0 @@
-#include <rcl_interfaces/msg/parameter.hpp>
-#include <rcl_interfaces/msg/parameter_type.hpp>
-#include <rcl_interfaces/srv/get_parameters.hpp>
-#include <rcl_interfaces/srv/set_parameters.hpp>
-#include <rclcpp/rclcpp.hpp>
-#include <std_msgs/msg/string.hpp>
-
-#include <chrono>
-#include <cstddef>
-#include <exception>
-#include <functional>
-#include <memory>
-#include <string>
-
-#include "fruit_perception/perception_mode.hpp"
-
-namespace fruit_perception
-{
-class CameraModeManager : public rclcpp::Node
-{
-public:
-  CameraModeManager()
-  : Node("camera_mode_manager")
-  {
-    const auto mode_topic = declare_parameter(
-      "perception_mode_topic", "/fruit_mission/perception_mode");
-    const auto active_topic = declare_parameter(
-      "active_mode_topic", "/fruit_perception/active_mode");
-    const auto camera_service = declare_parameter(
-      "camera_parameter_service", "/camera/set_parameters");
-    const auto camera_get_service = declare_parameter(
-      "camera_get_parameters_service", "/camera/get_parameters");
-
-    camera_client_ = create_client<rcl_interfaces::srv::SetParameters>(camera_service);
-    camera_get_client_ =
-      create_client<rcl_interfaces::srv::GetParameters>(camera_get_service);
-    active_mode_publisher_ = create_publisher<std_msgs::msg::String>(
-      active_topic, rclcpp::QoS(1).reliable().transient_local());
-    mode_subscription_ = create_subscription<std_msgs::msg::String>(
-      mode_topic, rclcpp::QoS(1).reliable(),
-      std::bind(&CameraModeManager::on_mode, this, std::placeholders::_1));
-    startup_timer_ = create_wall_timer(
-      std::chrono::milliseconds(250), std::bind(&CameraModeManager::initialize_off, this));
-  }
-
-private:
-  using GetParametersClient = rclcpp::Client<rcl_interfaces::srv::GetParameters>;
-  using SetParametersClient = rclcpp::Client<rcl_interfaces::srv::SetParameters>;
-
-  void initialize_off()
-  {
-    if (!camera_client_->service_is_ready() || !camera_get_client_->service_is_ready() ||
-      camera_readiness_check_pending_)
-    {
-      return;
-    }
-
-    camera_readiness_check_pending_ = true;
-    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
-    request->names = {"enable_color", "enable_depth"};
-    camera_get_client_->async_send_request(
-      request, [this](GetParametersClient::SharedFuture future) {
-        camera_readiness_check_pending_ = false;
-        try {
-          const auto response = future.get();
-          if (response->values.size() != 2U ||
-            response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL ||
-            response->values[1].type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
-          {
-            return;
-          }
-        } catch (const std::exception & error) {
-          RCLCPP_WARN(get_logger(), "Camera parameter readiness check failed: %s", error.what());
-          return;
-        }
-
-        startup_timer_->cancel();
-        state_.begin_startup_reset();
-        execute_current_action();
-      });
-  }
-
-  void on_mode(const std_msgs::msg::String::ConstSharedPtr message)
-  {
-    const auto target = parse_perception_mode(message->data);
-    if (!target) {
-      RCLCPP_ERROR(
-        get_logger(), "Invalid perception mode '%s'; expected OFF, QR, or DETECT",
-        message->data.c_str());
-      return;
-    }
-    if (!state_.active_mode()) {
-      RCLCPP_ERROR(get_logger(), "Camera manager is not ready; mode remains uninitialized");
-      return;
-    }
-    if (state_.transitioning()) {
-      RCLCPP_ERROR(get_logger(), "Camera transition already in progress; mode unchanged");
-      return;
-    }
-
-    state_.begin(*target);
-    if (!state_.transitioning()) {
-      publish_active_mode();
-      return;
-    }
-    execute_current_action();
-  }
-
-  void execute_current_action()
-  {
-    const auto action = state_.current_action();
-    if (!action) {
-      publish_active_mode();
-      return;
-    }
-    switch (*action) {
-      case CameraAction::kStopCamera:
-        set_camera_capture(false);
-        break;
-      case CameraAction::kStartCamera:
-        set_camera_capture(true);
-        break;
-    }
-  }
-
-  void set_camera_capture(bool enabled)
-  {
-    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
-    request->parameters.push_back(bool_parameter("enable_color", enabled));
-    request->parameters.push_back(bool_parameter("enable_depth", enabled));
-    camera_client_->async_send_request(
-      request, [this](SetParametersClient::SharedFuture future) {
-        bool success = false;
-        try {
-          const auto response = future.get();
-          success = response->results.size() == 2U;
-          for (const auto & result : response->results) {
-            success = success && result.successful;
-            if (!result.successful) {
-              RCLCPP_ERROR(
-                get_logger(), "Camera rejected stream parameter: %s", result.reason.c_str());
-            }
-          }
-        } catch (const std::exception & error) {
-          RCLCPP_ERROR(get_logger(), "Camera parameter call failed: %s", error.what());
-        }
-        finish_action(success);
-      });
-  }
-
-  static rcl_interfaces::msg::Parameter bool_parameter(
-    const std::string & name, bool value)
-  {
-    rcl_interfaces::msg::Parameter parameter;
-    parameter.name = name;
-    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
-    parameter.value.bool_value = value;
-    return parameter;
-  }
-
-  void finish_action(bool success)
-  {
-    if (!success) {
-      const bool startup_failed = !state_.active_mode();
-      state_.fail();
-      RCLCPP_ERROR(get_logger(), "Camera transition failed; active mode was not changed");
-      if (startup_failed) {
-        startup_timer_->reset();
-      }
-      return;
-    }
-    state_.complete_action();
-    if (state_.transitioning()) {
-      execute_current_action();
-    } else {
-      publish_active_mode();
-    }
-  }
-
-  void publish_active_mode()
-  {
-    const auto active = state_.active_mode();
-    if (!active) {
-      return;
-    }
-    std_msgs::msg::String message;
-    message.data = to_string(*active);
-    active_mode_publisher_->publish(message);
-    RCLCPP_INFO(get_logger(), "Perception camera mode is now %s", message.data.c_str());
-  }
-
-  CameraModeState state_;
-  SetParametersClient::SharedPtr camera_client_;
-  GetParametersClient::SharedPtr camera_get_client_;
-  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_mode_publisher_;
-  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
-  rclcpp::TimerBase::SharedPtr startup_timer_;
-  bool camera_readiness_check_pending_{false};
-};
-}  // namespace fruit_perception
-
-int main(int argc, char ** argv)
-{
-  rclcpp::init(argc, argv);
-  rclcpp::spin(std::make_shared<fruit_perception::CameraModeManager>());
-  rclcpp::shutdown();
-  return 0;
-}
diff --git a/src/fruit_perception/src/fruit_perception_node.cpp b/src/fruit_perception/src/fruit_perception_node.cpp
index cfe94b2..ec7f1ba 100644
--- a/src/fruit_perception/src/fruit_perception_node.cpp
+++ b/src/fruit_perception/src/fruit_perception_node.cpp
@@ -34,11 +34,121 @@
 #include "fruit_perception/msg/fruit_decision.hpp"
 #include "fruit_perception/msg/fruit_detection.hpp"
 #include "fruit_perception/msg/fruit_target.hpp"
-#include "fruit_perception/perception_mode.hpp"
 #include "fruit_perception/ripeness_classifier.hpp"
 
 namespace fruit_perception
 {
+namespace
+{
+struct ObservationWindowCommand
+{
+  std::string window_id;
+  int valid{0};
+  std::int64_t start_sec{0};
+  std::int64_t start_nanosec{0};
+  char area{'A'};
+  std::string purpose;
+  std::string expected_type;
+};
+
+std::optional<ObservationWindowCommand> parse_observation_window_command(
+  const std::string & json_str, const rclcpp::Logger & logger)
+{
+  if (json_str.empty()) {
+    return std::nullopt;
+  }
+  try {
+    cv::FileStorage fs(
+      json_str, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
+    if (!fs.isOpened()) {
+      RCLCPP_WARN(logger, "Failed to parse observation window JSON");
+      return std::nullopt;
+    }
+    const auto wid_node = fs["window_id"];
+    const auto valid_node = fs["valid"];
+    const auto sec_node = fs["start_sec"];
+    const auto nsec_node = fs["start_nanosec"];
+    const auto area_node = fs["area"];
+    const auto purpose_node = fs["purpose"];
+    const auto expected_node = fs["expected_type"];
+
+    if (!wid_node.isString() || !valid_node.isInt() || !sec_node.isInt() ||
+      !nsec_node.isInt() || !area_node.isString() || !purpose_node.isString())
+    {
+      RCLCPP_WARN(logger, "Observation window JSON field types invalid or missing");
+      return std::nullopt;
+    }
+    if (!expected_node.empty() && !expected_node.isString()) {
+      RCLCPP_WARN(logger, "Observation window expected_type is not a string");
+      return std::nullopt;
+    }
+
+    ObservationWindowCommand cmd;
+    cmd.window_id = static_cast<std::string>(wid_node);
+    if (cmd.window_id.empty()) {
+      RCLCPP_WARN(logger, "Observation window window_id is empty");
+      return std::nullopt;
+    }
+
+    cmd.valid = static_cast<int>(valid_node);
+    if (cmd.valid != 0 && cmd.valid != 1) {
+      RCLCPP_WARN(logger, "Observation window valid flag must be 0 or 1, got %d", cmd.valid);
+      return std::nullopt;
+    }
+
+    cmd.start_sec = static_cast<std::int64_t>(static_cast<int>(sec_node));
+    if (cmd.start_sec < 0) {
+      RCLCPP_WARN(logger, "Observation window start_sec must be non-negative");
+      return std::nullopt;
+    }
+
+    cmd.start_nanosec = static_cast<std::int64_t>(static_cast<int>(nsec_node));
+    if (cmd.start_nanosec < 0 || cmd.start_nanosec >= 1000000000LL) {
+      RCLCPP_WARN(logger, "Observation window start_nanosec must be in [0, 999999999]");
+      return std::nullopt;
+    }
+
+    const std::string area_str = static_cast<std::string>(area_node);
+    if (area_str != "A" && area_str != "B" && area_str != "C") {
+      RCLCPP_WARN(logger, "Observation window area '%s' invalid", area_str.c_str());
+      return std::nullopt;
+    }
+    cmd.area = area_str.front();
+
+    cmd.purpose = static_cast<std::string>(purpose_node);
+    if (cmd.purpose != "harvest" && cmd.purpose != "qr") {
+      RCLCPP_WARN(logger, "Observation window purpose '%s' invalid", cmd.purpose.c_str());
+      return std::nullopt;
+    }
+
+    if (!expected_node.empty()) {
+      cmd.expected_type = static_cast<std::string>(expected_node);
+    }
+    return cmd;
+  } catch (const cv::Exception & e) {
+    RCLCPP_WARN(logger, "Observation window JSON parse exception: %s", e.what());
+    return std::nullopt;
+  } catch (const std::exception & e) {
+    RCLCPP_WARN(logger, "Observation window exception: %s", e.what());
+    return std::nullopt;
+  }
+}
+
+bool is_stamp_at_or_after(
+  const builtin_interfaces::msg::Time & stamp, std::int64_t start_sec,
+  std::int64_t start_nanosec)
+{
+  const auto stamp_sec = static_cast<std::int64_t>(stamp.sec);
+  const auto stamp_nanosec = static_cast<std::int64_t>(stamp.nanosec);
+  if (stamp_sec > start_sec) {
+    return true;
+  }
+  if (stamp_sec == start_sec && stamp_nanosec >= start_nanosec) {
+    return true;
+  }
+  return false;
+}
+}  // namespace
 class FruitPerceptionNode : public rclcpp::Node
 {
 public:
@@ -118,22 +228,12 @@ public:
     b_target_selector_config_.maturity_confidence_threshold =
       target_selector_config_.maturity_confidence_threshold;
 
-    const auto active_mode_topic = declare_parameter(
-      "active_mode_topic", "/fruit_perception/active_mode");
-    mode_subscription_ = create_subscription<std_msgs::msg::String>(
-      active_mode_topic, rclcpp::QoS(1).reliable(),
+    const auto observation_window_topic = declare_parameter(
+      "observation_window_topic", "/fruit_mission/observation_window");
+    observation_window_subscription_ = create_subscription<std_msgs::msg::String>(
+      observation_window_topic, rclcpp::QoS(10).reliable(),
       [this](std_msgs::msg::String::ConstSharedPtr message) {
-        const auto mode = parse_perception_mode(message->data);
-        if (!mode) {
-          RCLCPP_ERROR(
-            get_logger(), "Invalid active perception mode '%s'; inference mode unchanged",
-            message->data.c_str());
-          return;
-        }
-        const auto previous = perception_mode_.exchange(*mode);
-        if (previous != *mode || *mode == PerceptionMode::kDetect) {
-          target_selection_generation_.fetch_add(1U);
-        }
+        on_observation_window(message);
       });
 
     const auto current_area_topic = declare_parameter(
@@ -142,7 +242,7 @@ public:
       current_area_topic, rclcpp::QoS(1).reliable(),
       [this](std_msgs::msg::String::ConstSharedPtr message) {
         if (message->data == "A" || message->data == "B" ||
-          message->data == "C" || message->data == "D")
+        message->data == "C" || message->data == "D")
         {
           const char previous = current_area_.exchange(message->data.front());
           if (previous != message->data.front()) {
@@ -287,6 +387,19 @@ private:
     std::uint64_t target_selection_generation{0U};
     std::uint64_t submitted_target_selection_generation{0U};
     std::atomic<bool> busy{false};
+
+    struct SubmissionContext
+    {
+      std::string window_id;
+      bool is_harvest_window{false};
+      std::int64_t start_sec{0};
+      std::int64_t start_nanosec{0};
+      char area{'A'};
+      std::string expected_type;
+      builtin_interfaces::msg::Time rgb_stamp;
+      builtin_interfaces::msg::Time depth_stamp;
+    };
+    SubmissionContext submission_context;
   };
 
   struct ModelRuntime
@@ -343,8 +456,9 @@ private:
     runtime.compiled_model = core_.compile_model(
       model, "GPU.0",
       ov::hint::inference_precision(ov::element::f16),
-      ov::hint::performance_mode(performance_hint_ == "throughput" ?
-      ov::hint::PerformanceMode::THROUGHPUT : ov::hint::PerformanceMode::LATENCY),
+      ov::hint::performance_mode(
+        performance_hint_ == "throughput" ?
+        ov::hint::PerformanceMode::THROUGHPUT : ov::hint::PerformanceMode::LATENCY),
       ov::hint::num_requests(static_cast<std::uint32_t>(request_count)));
 
     const auto execution_devices = runtime.compiled_model.get_property(ov::execution_devices);
@@ -415,7 +529,7 @@ private:
   void submit_rgbd(
     Stream & stream, const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr & rgbd)
   {
-    if (stopping_.load() || !runs_camera_inference(perception_mode_.load())) {
+    if (stopping_.load()) {
       return;
     }
 
@@ -433,6 +547,28 @@ private:
         stream.target_selection_generation = target_generation;
       }
       stream.submitted_target_selection_generation = target_generation;
+
+      Stream::SubmissionContext context;
+      {
+        std::lock_guard<std::mutex> lock(window_mutex_);
+        if (current_window_.valid && current_window_.purpose == "harvest") {
+          context.window_id = current_window_.window_id;
+          context.is_harvest_window = true;
+          context.start_sec = current_window_.start_sec;
+          context.start_nanosec = current_window_.start_nanosec;
+          context.area = current_window_.area;
+          context.expected_type = current_window_.expected_type;
+        } else {
+          context.window_id = "";
+          context.is_harvest_window = false;
+          context.area = current_area_.load();
+          context.expected_type = expected_fruit_type();
+        }
+      }
+      context.rgb_stamp = rgbd->rgb.header.stamp;
+      context.depth_stamp = rgbd->depth.header.stamp;
+      stream.submission_context = std::move(context);
+
       stream.frame = cv_bridge::toCvShare(
         rgbd->rgb, rgbd, sensor_msgs::image_encodings::RGB8);
       stream.aligned_depth = cv_bridge::toCvShare(
@@ -440,7 +576,7 @@ private:
       stream.depth_camera_info = rgbd->depth_camera_info;
       stream.transform = letterbox_image(stream.frame->image, stream.input_image);
       stream.header = rgbd->rgb.header;
-      auto * request = select_camera_request(stream);
+      auto * request = select_camera_request(stream, stream.submission_context.area);
       if (request == nullptr) {
         stream.busy.store(false);
         return;
@@ -467,12 +603,8 @@ private:
         confidence_threshold_,
         nms_iou_threshold_);
 
-      if (!runs_camera_inference(perception_mode_.load())) {
-        stream.busy.store(false);
-        return;
-      }
-
-      const char area = current_area_.load();
+      const auto context = stream.submission_context;
+      const char area = context.area;
       std::vector<Detection> classified;
       classified.reserve(detections.size());
       std::vector<double> maturity_confidences;
@@ -488,9 +620,11 @@ private:
         }
         const int x1 = std::max(0, static_cast<int>(std::floor(box.x - box.width * 0.1)));
         const int y1 = std::max(0, static_cast<int>(std::floor(box.y - box.height * 0.1)));
-        const int x2 = std::min(stream.frame->image.cols,
+        const int x2 = std::min(
+          stream.frame->image.cols,
           static_cast<int>(std::ceil(box.x + box.width * 1.1)));
-        const int y2 = std::min(stream.frame->image.rows,
+        const int y2 = std::min(
+          stream.frame->image.rows,
           static_cast<int>(std::ceil(box.y + box.height * 1.1)));
         if (x2 - x1 < 8 || y2 - y1 < 8) {
           continue;
@@ -548,111 +682,119 @@ private:
         result.detections.push_back(std::move(item));
       }
 
-      if (stream.submitted_target_selection_generation !=
-        target_selection_generation_.load())
-      {
-        stream.busy.store(false);
-        return;
-      }
-
-      if (area == 'B' || area == 'D') {
-        std::vector<BTargetCandidate> candidates;
-        candidates.reserve(outputs.size());
-        for (std::size_t index = 0U; index < outputs.size(); ++index) {
-          const auto & output = outputs[index];
-          if (!output.has_point ||
-            !std::isfinite(output.point.x) ||
-            !std::isfinite(output.point.y) ||
-            !std::isfinite(output.point.z))
-          {
-            continue;
+      const bool in_window = context.is_harvest_window &&
+        !context.window_id.empty() &&
+        (stream.submitted_target_selection_generation == target_selection_generation_.load()) &&
+        is_stamp_at_or_after(context.rgb_stamp, context.start_sec, context.start_nanosec) &&
+        is_stamp_at_or_after(context.depth_stamp, context.start_sec, context.start_nanosec);
+
+      if (in_window) {
+        result.observation_window_id = context.window_id;
+        if (area == 'B' || area == 'D') {
+          std::vector<BTargetCandidate> candidates;
+          candidates.reserve(outputs.size());
+          for (std::size_t index = 0U; index < outputs.size(); ++index) {
+            const auto & output = outputs[index];
+            if (!output.has_point ||
+              !std::isfinite(output.point.x) ||
+              !std::isfinite(output.point.y) ||
+              !std::isfinite(output.point.z))
+            {
+              continue;
+            }
+            candidates.push_back(
+              BTargetCandidate{
+                output.type,
+                output.point,
+                cv::Point2d{
+                  static_cast<double>(output.roi.x) +
+                  static_cast<double>(output.roi.width) * 0.5,
+                  static_cast<double>(output.roi.y) +
+                  static_cast<double>(output.roi.height) * 0.5},
+                static_cast<std::size_t>(stream.transform.original_width),
+                static_cast<double>(output.confidence), maturity_confidences[index]});
           }
-          candidates.push_back(
-            BTargetCandidate{
-              output.type,
-              output.point,
-              cv::Point2d{
-                static_cast<double>(output.roi.x) +
-                static_cast<double>(output.roi.width) * 0.5,
-                static_cast<double>(output.roi.y) +
-                static_cast<double>(output.roi.height) * 0.5},
-              static_cast<std::size_t>(stream.transform.original_width),
-              static_cast<double>(output.confidence), maturity_confidences[index]});
-        }
 
-        const auto target_update = stream.b_target_selector->update(candidates);
-        result.targets_ready = target_update.ready;
-        result.targets.reserve(target_update.targets.size());
-        for (const auto & target : target_update.targets) {
-          msg::FruitTarget item;
-          item.type = target.type;
-          item.confidence = static_cast<float>(target.confidence);
-          item.has_point = true;
-          item.point.x = target.point.x;
-          item.point.y = target.point.y;
-          item.point.z = target.point.z;
-          item.pixel_x = static_cast<float>(target.pixel_center.x);
-          item.pixel_y = static_cast<float>(target.pixel_center.y);
-          item.image_width = static_cast<std::uint32_t>(target.image_width);
-          item.observation_count = static_cast<std::uint32_t>(target.observation_count);
-          item.inlier_count = static_cast<std::uint32_t>(target.inlier_count);
-          result.targets.push_back(std::move(item));
-        }
-        if (!result.targets.empty()) {
-          result.has_target = true;
-          result.target_point = result.targets.front().point;
-        }
-      } else {
-        std::vector<CameraTargetCandidate> candidates;
-        candidates.reserve(outputs.size());
-        const auto expected_type = expected_fruit_type();
-        const double half_width = static_cast<double>(stream.frame->image.cols) * 0.5;
-        for (std::size_t index = 0U; index < outputs.size(); ++index) {
-          const auto & output = outputs[index];
-          if (!output.has_point ||
-            !std::isfinite(output.point.x) ||
-            !std::isfinite(output.point.y) ||
-            !std::isfinite(output.point.z))
-          {
-            continue;
+          const auto target_update = stream.b_target_selector->update(candidates);
+          result.targets_ready = target_update.ready;
+          result.targets.reserve(target_update.targets.size());
+          for (const auto & target : target_update.targets) {
+            msg::FruitTarget item;
+            item.type = target.type;
+            item.confidence = static_cast<float>(target.confidence);
+            item.has_point = true;
+            item.point.x = target.point.x;
+            item.point.y = target.point.y;
+            item.point.z = target.point.z;
+            item.pixel_x = static_cast<float>(target.pixel_center.x);
+            item.pixel_y = static_cast<float>(target.pixel_center.y);
+            item.image_width = static_cast<std::uint32_t>(target.image_width);
+            item.observation_count = static_cast<std::uint32_t>(target.observation_count);
+            item.inlier_count = static_cast<std::uint32_t>(target.inlier_count);
+            result.targets.push_back(std::move(item));
+          }
+          if (!result.targets.empty()) {
+            result.has_target = true;
+            result.target_point = result.targets.front().point;
+          }
+        } else {
+          std::vector<CameraTargetCandidate> candidates;
+          candidates.reserve(outputs.size());
+          const auto expected_type = context.expected_type;
+          const double half_width = static_cast<double>(stream.frame->image.cols) * 0.5;
+          for (std::size_t index = 0U; index < outputs.size(); ++index) {
+            const auto & output = outputs[index];
+            if (!output.has_point ||
+              !std::isfinite(output.point.x) ||
+              !std::isfinite(output.point.y) ||
+              !std::isfinite(output.point.z))
+            {
+              continue;
+            }
+            const double roi_center_x =
+              static_cast<double>(output.roi.x) + static_cast<double>(output.roi.width) * 0.5;
+            const double horizontal_offset = std::clamp(
+              std::abs(roi_center_x - half_width) / half_width, 0.0, 1.0);
+            candidates.push_back(
+              CameraTargetCandidate{
+                output.point, static_cast<double>(output.confidence), 1.0 - horizontal_offset,
+                output.type, maturity_confidences[index]});
           }
-          const double roi_center_x =
-            static_cast<double>(output.roi.x) + static_cast<double>(output.roi.width) * 0.5;
-          const double horizontal_offset = std::clamp(
-            std::abs(roi_center_x - half_width) / half_width, 0.0, 1.0);
-          candidates.push_back(
-            CameraTargetCandidate{
-              output.point, static_cast<double>(output.confidence), 1.0 - horizontal_offset,
-              output.type, maturity_confidences[index]});
-        }
 
-        const auto target_update = stream.target_selector->update(candidates, expected_type);
-        if (target_update.lock_lost) {
-          RCLCPP_WARN(
-            get_logger(),
-            "Camera target lock lost after %zu missed frames; reacquiring",
-            target_selector_config_.lock_miss_frames);
-        }
-        if (target_update.lock_acquired) {
-          RCLCPP_INFO(
-            get_logger(),
-            "Camera target locked with %zu/%zu inliers in a %zu-frame window: score=%.3f, "
-            "confidence=%.3f, horizontal_centrality=%.3f, "
-            "mean=(%.3f, %.3f, %.3f), stddev=(%.4f, %.4f, %.4f)",
-            target_update.inlier_count, target_update.observation_count,
-            target_selector_config_.acquisition_frames,
-            target_update.score, target_update.confidence,
-            target_update.horizontal_centrality,
-            target_update.reference_point.x, target_update.reference_point.y,
-            target_update.reference_point.z, target_update.stddev.x,
-            target_update.stddev.y, target_update.stddev.z);
-        }
-        if (target_update.point) {
-          result.has_target = true;
-          result.target_point.x = target_update.point->x;
-          result.target_point.y = target_update.point->y;
-          result.target_point.z = target_update.point->z;
+          const auto target_update = stream.target_selector->update(candidates, expected_type);
+          if (target_update.lock_lost) {
+            RCLCPP_WARN(
+              get_logger(),
+              "Camera target lock lost after %zu missed frames; reacquiring",
+              target_selector_config_.lock_miss_frames);
+          }
+          if (target_update.lock_acquired) {
+            RCLCPP_INFO(
+              get_logger(),
+              "Camera target locked with %zu/%zu inliers in a %zu-frame window: score=%.3f, "
+              "confidence=%.3f, horizontal_centrality=%.3f, "
+              "mean=(%.3f, %.3f, %.3f), stddev=(%.4f, %.4f, %.4f)",
+              target_update.inlier_count, target_update.observation_count,
+              target_selector_config_.acquisition_frames,
+              target_update.score, target_update.confidence,
+              target_update.horizontal_centrality,
+              target_update.reference_point.x, target_update.reference_point.y,
+              target_update.reference_point.z, target_update.stddev.x,
+              target_update.stddev.y, target_update.stddev.z);
+          }
+          if (target_update.point) {
+            result.has_target = true;
+            result.target_point.x = target_update.point->x;
+            result.target_point.y = target_update.point->y;
+            result.target_point.z = target_update.point->z;
+          }
         }
+      } else {
+        result.observation_window_id = "";
+        result.has_target = false;
+        result.targets_ready = false;
+        result.targets.clear();
+        result.target_point = geometry_msgs::msg::Point();
       }
 
       if (stopping_.load() || stream.submitted_target_selection_generation !=
@@ -696,7 +838,8 @@ private:
           stream.total_classification_ms / frames, stream.classified_rois / frames);
         stream.statistics_started = now;
         stream.completed_frames = stream.classified_rois = 0U;
-        stream.total_detection_ms = stream.total_classification_ms = stream.total_processing_ms = 0.0;
+        stream.total_detection_ms = stream.total_classification_ms = stream.total_processing_ms =
+          0.0;
       }
 
       if (stream.image_publisher->get_subscription_count() > 0U) {
@@ -719,6 +862,9 @@ private:
       {
         msg::FruitDecision empty;
         empty.header = stream.header;
+        empty.observation_window_id = "";
+        empty.has_target = false;
+        empty.targets_ready = false;
         stream.decision_publisher->publish(empty);
       }
     }
@@ -758,9 +904,8 @@ private:
     return expected_fruit_type_;
   }
 
-  Stream::RequestSlot * select_camera_request(Stream & stream) const
+  Stream::RequestSlot * select_camera_request(Stream & stream, char area) const
   {
-    const char area = current_area_.load();
     if (area == 'A' || area == 'C') {
       return stream.vegetable_request;
     }
@@ -770,6 +915,43 @@ private:
     return nullptr;
   }
 
+  void on_observation_window(const std_msgs::msg::String::ConstSharedPtr & message)
+  {
+    const auto cmd = parse_observation_window_command(message->data, get_logger());
+    if (!cmd) {
+      return;
+    }
+    std::lock_guard<std::mutex> lock(window_mutex_);
+    if (cmd->valid == 1) {
+      current_window_.window_id = cmd->window_id;
+      current_window_.valid = true;
+      current_window_.start_sec = cmd->start_sec;
+      current_window_.start_nanosec = cmd->start_nanosec;
+      current_window_.area = cmd->area;
+      current_window_.purpose = cmd->purpose;
+      current_window_.expected_type = cmd->expected_type;
+      target_selection_generation_.fetch_add(1U);
+      RCLCPP_INFO(
+        get_logger(),
+        "Opened observation window '%s': area=%c, purpose=%s, start=%ld.%09ld, expected='%s'",
+        cmd->window_id.c_str(), cmd->area, cmd->purpose.c_str(),
+        static_cast<long>(cmd->start_sec), static_cast<long>(cmd->start_nanosec),
+        cmd->expected_type.c_str());
+    } else {
+      if (current_window_.valid && current_window_.window_id == cmd->window_id) {
+        current_window_.valid = false;
+        target_selection_generation_.fetch_add(1U);
+        RCLCPP_INFO(
+          get_logger(), "Closed observation window '%s'", cmd->window_id.c_str());
+      } else if (current_window_.valid) {
+        RCLCPP_WARN(
+          get_logger(),
+          "Ignoring close for window '%s'; active window is '%s'",
+          cmd->window_id.c_str(), current_window_.window_id.c_str());
+      }
+    }
+  }
+
   ov::Core core_;
   ModelRuntime vegetable_model_;
   ModelRuntime fruit_model_;
@@ -784,11 +966,24 @@ private:
   BTargetSelectorConfig b_target_selector_config_{8U, 2U, 0.03, 2.0};
   std::atomic<std::uint64_t> target_selection_generation_{0U};
   std::atomic<bool> stopping_{false};
-  std::atomic<PerceptionMode> perception_mode_{PerceptionMode::kOff};
   std::atomic<char> current_area_{'A'};
+
+  struct ActiveWindow
+  {
+    std::string window_id;
+    bool valid{false};
+    std::int64_t start_sec{0};
+    std::int64_t start_nanosec{0};
+    char area{'A'};
+    std::string purpose;
+    std::string expected_type;
+  };
+  ActiveWindow current_window_;
+  std::mutex window_mutex_;
+
   rclcpp::Publisher<msg::FruitDecision>::SharedPtr decision_publisher_;
   rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr rgbd_heartbeat_publisher_;
-  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
+  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr observation_window_subscription_;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_area_subscription_;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expected_fruit_type_subscription_;
   mutable std::mutex expected_fruit_type_mutex_;
diff --git a/src/fruit_perception/src/perception_mode.cpp b/src/fruit_perception/src/perception_mode.cpp
deleted file mode 100644
index 7028d7c..0000000
--- a/src/fruit_perception/src/perception_mode.cpp
+++ /dev/null
@@ -1,133 +0,0 @@
-#include "fruit_perception/perception_mode.hpp"
-
-#include <utility>
-
-namespace fruit_perception
-{
-namespace
-{
-bool uses_camera(PerceptionMode mode)
-{
-  return mode == PerceptionMode::kQr || mode == PerceptionMode::kDetect;
-}
-}  // namespace
-
-std::optional<PerceptionMode> parse_perception_mode(const std::string & value)
-{
-  if (value == "OFF") {
-    return PerceptionMode::kOff;
-  }
-  if (value == "QR") {
-    return PerceptionMode::kQr;
-  }
-  if (value == "DETECT") {
-    return PerceptionMode::kDetect;
-  }
-  return std::nullopt;
-}
-
-const char * to_string(PerceptionMode mode)
-{
-  switch (mode) {
-    case PerceptionMode::kOff:
-      return "OFF";
-    case PerceptionMode::kQr:
-      return "QR";
-    case PerceptionMode::kDetect:
-      return "DETECT";
-  }
-  return "OFF";
-}
-
-bool runs_camera_inference(PerceptionMode mode)
-{
-  return mode == PerceptionMode::kDetect;
-}
-
-std::vector<CameraAction> plan_camera_transition(
-  PerceptionMode current, PerceptionMode target)
-{
-  std::vector<CameraAction> actions;
-  if (uses_camera(current) && !uses_camera(target)) {
-    actions.push_back(CameraAction::kStopCamera);
-  }
-  if (!uses_camera(current) && uses_camera(target)) {
-    actions.push_back(CameraAction::kStartCamera);
-  }
-  return actions;
-}
-
-std::vector<CameraAction> plan_startup_reset()
-{
-  return {CameraAction::kStopCamera};
-}
-
-bool CameraModeState::begin_startup_reset()
-{
-  if (transitioning() || active_mode_) {
-    return false;
-  }
-  target_mode_ = PerceptionMode::kOff;
-  actions_ = plan_startup_reset();
-  action_index_ = 0U;
-  finish_if_no_actions();
-  return true;
-}
-
-bool CameraModeState::begin(PerceptionMode target)
-{
-  if (transitioning() || !active_mode_) {
-    return false;
-  }
-  target_mode_ = target;
-  actions_ = plan_camera_transition(*active_mode_, target);
-  action_index_ = 0U;
-  finish_if_no_actions();
-  return true;
-}
-
-std::optional<CameraAction> CameraModeState::current_action() const
-{
-  if (!transitioning() || action_index_ >= actions_.size()) {
-    return std::nullopt;
-  }
-  return actions_[action_index_];
-}
-
-void CameraModeState::complete_action()
-{
-  if (!current_action()) {
-    return;
-  }
-  ++action_index_;
-  finish_if_no_actions();
-}
-
-void CameraModeState::fail()
-{
-  target_mode_.reset();
-  actions_.clear();
-  action_index_ = 0U;
-}
-
-bool CameraModeState::transitioning() const
-{
-  return target_mode_.has_value();
-}
-
-std::optional<PerceptionMode> CameraModeState::active_mode() const
-{
-  return active_mode_;
-}
-
-void CameraModeState::finish_if_no_actions()
-{
-  if (target_mode_ && action_index_ >= actions_.size()) {
-    active_mode_ = target_mode_;
-    target_mode_.reset();
-    actions_.clear();
-    action_index_ = 0U;
-  }
-}
-
-}  // namespace fruit_perception
diff --git a/src/fruit_perception/test/test_perception_mode.cpp b/src/fruit_perception/test/test_perception_mode.cpp
deleted file mode 100644
index fb692c6..0000000
--- a/src/fruit_perception/test/test_perception_mode.cpp
+++ /dev/null
@@ -1,77 +0,0 @@
-#include <gtest/gtest.h>
-
-#include <optional>
-#include <vector>
-
-#include "fruit_perception/perception_mode.hpp"
-
-namespace
-{
-using fruit_perception::CameraAction;
-using fruit_perception::PerceptionMode;
-
-TEST(PerceptionMode, AcceptsOnlyThreeUppercaseValues)
-{
-  EXPECT_EQ(fruit_perception::parse_perception_mode("OFF"), PerceptionMode::kOff);
-  EXPECT_EQ(fruit_perception::parse_perception_mode("QR"), PerceptionMode::kQr);
-  EXPECT_EQ(fruit_perception::parse_perception_mode("DETECT"), PerceptionMode::kDetect);
-  EXPECT_FALSE(fruit_perception::parse_perception_mode("SIDES"));
-  EXPECT_FALSE(fruit_perception::parse_perception_mode("TREE"));
-  EXPECT_FALSE(fruit_perception::parse_perception_mode("tree"));
-  EXPECT_FALSE(fruit_perception::parse_perception_mode("D"));
-}
-
-TEST(PerceptionMode, MapsConsumersAndCameraGroups)
-{
-  EXPECT_FALSE(fruit_perception::runs_camera_inference(PerceptionMode::kOff));
-  EXPECT_FALSE(fruit_perception::runs_camera_inference(PerceptionMode::kQr));
-  EXPECT_TRUE(fruit_perception::runs_camera_inference(PerceptionMode::kDetect));
-}
-
-TEST(PerceptionMode, TransitionStartsAndStopsCamera)
-{
-  EXPECT_EQ(
-    fruit_perception::plan_camera_transition(
-      PerceptionMode::kOff, PerceptionMode::kDetect),
-    (std::vector<CameraAction>{CameraAction::kStartCamera}));
-  EXPECT_EQ(
-    fruit_perception::plan_camera_transition(
-      PerceptionMode::kDetect, PerceptionMode::kOff),
-    (std::vector<CameraAction>{CameraAction::kStopCamera}));
-}
-
-TEST(PerceptionMode, QrDetectAndRepeatedModesDoNotRestartCamera)
-{
-  EXPECT_TRUE(
-    fruit_perception::plan_camera_transition(
-      PerceptionMode::kQr, PerceptionMode::kDetect).empty());
-  EXPECT_TRUE(
-    fruit_perception::plan_camera_transition(
-      PerceptionMode::kDetect, PerceptionMode::kQr).empty());
-  EXPECT_TRUE(
-    fruit_perception::plan_camera_transition(
-      PerceptionMode::kDetect, PerceptionMode::kDetect).empty());
-}
-
-TEST(PerceptionMode, StartupResetStopsCamera)
-{
-  EXPECT_EQ(
-    fruit_perception::plan_startup_reset(),
-    (std::vector<CameraAction>{CameraAction::kStopCamera}));
-}
-
-TEST(PerceptionMode, FailedTransitionDoesNotUpdateActiveMode)
-{
-  fruit_perception::CameraModeState state;
-  ASSERT_TRUE(state.begin_startup_reset());
-  state.complete_action();
-  ASSERT_EQ(state.active_mode(), PerceptionMode::kOff);
-
-  ASSERT_TRUE(state.begin(PerceptionMode::kDetect));
-  ASSERT_TRUE(state.transitioning());
-  state.fail();
-
-  EXPECT_FALSE(state.transitioning());
-  EXPECT_EQ(state.active_mode(), PerceptionMode::kOff);
-}
-}  // namespace
diff --git a/src/fruit_qr_planner/README.md b/src/fruit_qr_planner/README.md
index 6aac434..67bb0fb 100644
--- a/src/fruit_qr_planner/README.md
+++ b/src/fruit_qr_planner/README.md
@@ -7,18 +7,23 @@ plan, then stops accepting further codes.
 ## Interfaces
 
 - Input: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
+- Observation window: `/fruit_mission/observation_window` (`std_msgs/msg/String`, JSON)
 - Current area: `/fruit_mission/current_area` (`std_msgs/msg/String`)
 - B plan: `/fruit/b_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
 - C plan: `/fruit/c_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
-- Parameters: `image_topic`, `current_area_topic`, and `confirm_frames` (default `1`)
+- Parameters: `image_topic`, `observation_window_topic`, `current_area_topic`, and `confirm_frames` (default `1`)
 
-Both plan topics use Reliable, Transient Local QoS with depth 1, so the mission
-node can start after scanning and still receive the last B and C plans. After C
-is confirmed, the node destroys its image subscription.
+Decoding only occurs during an active observation window with `purpose="qr"` and for frames whose capture timestamp is at or after the window's start time. Outside a valid QR window, frames are immediately dropped to minimize CPU overhead.
+Each plan published includes `observation_window_id`, `image_stamp_sec`, and `image_stamp_nanosec`.
+Window commands accept areas A/B/C only. Mission consumers reject non-object plan JSON,
+invalid integer image timestamps, wrong window IDs, and mismatched QR purpose or B/C area.
 
-If mission enters C before a B code was decoded, the current-area message advances the
+Both plan topics use Reliable, Transient Local QoS with depth 1, so consumers
+can retrieve the published plan. Within each window, only one valid plan is published.
+
+If mission enters C before a B code was decoded, the current-area message or a C observation window advances the
 scanner directly to C. This preserves the C scan opportunity when mission used its
-configured B fallback after the 20-second plan wait. QR timeout fallback contents are
+configured B fallback after the 10-second plan wait. QR timeout fallback contents are
 owned by `fruit_mission/config/fruit_mission.yaml`; this node only decodes and publishes
 codes that were actually read.
 
diff --git a/src/fruit_qr_planner/config/fruit_qr_planner.yaml b/src/fruit_qr_planner/config/fruit_qr_planner.yaml
index 5636c7b..e96105d 100644
--- a/src/fruit_qr_planner/config/fruit_qr_planner.yaml
+++ b/src/fruit_qr_planner/config/fruit_qr_planner.yaml
@@ -2,4 +2,5 @@ fruit_qr_planner:
   ros__parameters:
     image_topic: "/camera/color/image_raw"
     current_area_topic: "/fruit_mission/current_area"
+    observation_window_topic: "/fruit_mission/observation_window"
     confirm_frames: 1
diff --git a/src/fruit_qr_planner/fruit_qr_planner/qr_node.py b/src/fruit_qr_planner/fruit_qr_planner/qr_node.py
index ae09184..5499644 100644
--- a/src/fruit_qr_planner/fruit_qr_planner/qr_node.py
+++ b/src/fruit_qr_planner/fruit_qr_planner/qr_node.py
@@ -40,6 +40,63 @@ def plan_qos() -> QoSProfile:
     )
 
 
+def observation_window_qos() -> QoSProfile:
+    return QoSProfile(
+        history=HistoryPolicy.KEEP_LAST,
+        depth=10,
+        reliability=ReliabilityPolicy.RELIABLE,
+        durability=DurabilityPolicy.VOLATILE,
+    )
+
+
+def _parse_observation_window_command(data: str) -> Optional[Dict[str, Any]]:
+    if not data or not data.strip():
+        return None
+    try:
+        parsed = json.loads(data)
+    except Exception:
+        return None
+    if not isinstance(parsed, dict):
+        return None
+    window_id = parsed.get("window_id")
+    valid = parsed.get("valid")
+    start_sec = parsed.get("start_sec")
+    start_nanosec = parsed.get("start_nanosec")
+    area = parsed.get("area")
+    purpose = parsed.get("purpose")
+    expected_type = parsed.get("expected_type", "")
+
+    if not isinstance(window_id, str) or not window_id.strip():
+        return None
+    if isinstance(valid, bool) or not isinstance(valid, int) or valid not in (0, 1):
+        return None
+    if isinstance(start_sec, bool) or not isinstance(start_sec, int) or start_sec < 0:
+        return None
+    if (
+        isinstance(start_nanosec, bool)
+        or not isinstance(start_nanosec, int)
+        or start_nanosec < 0
+        or start_nanosec >= 1_000_000_000
+    ):
+        return None
+    if not isinstance(area, str) or area not in ("A", "B", "C"):
+        return None
+    if not isinstance(purpose, str) or purpose not in ("harvest", "qr"):
+        return None
+    if not isinstance(expected_type, str):
+        return None
+
+    return {
+        "window_id": window_id.strip(),
+        "valid": valid,
+        "start_sec": start_sec,
+        "start_nanosec": start_nanosec,
+        "area": area,
+        "purpose": purpose,
+        "expected_type": expected_type,
+    }
+
+
 def image_to_bgr(message: Image, cv2_module: Any) -> Any:
     if message.encoding not in ("rgb8", "bgr8"):
         raise ValueError("QR input must use rgb8 or bgr8 encoding")
@@ -66,6 +123,10 @@ class FruitQrPlannerNode(Node):
             "current_area_topic",
             "/fruit_mission/current_area",
         )
+        self.declare_parameter(
+            "observation_window_topic",
+            "/fruit_mission/observation_window",
+        )
         self.declare_parameter("confirm_frames", 1)
 
         image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
@@ -74,6 +135,11 @@ class FruitQrPlannerNode(Node):
             .get_parameter_value()
             .string_value
         )
+        observation_window_topic = (
+            self.get_parameter("observation_window_topic")
+            .get_parameter_value()
+            .string_value
+        )
         confirm_frames = self.get_parameter("confirm_frames").get_parameter_value().integer_value
         if confirm_frames <= 0:
             raise ValueError("confirm_frames must be greater than zero")
@@ -93,6 +159,8 @@ class FruitQrPlannerNode(Node):
         self._cv2 = cv2
         self._decode_barcodes = decode_barcodes
         self._sequence = PlanSequence(confirm_frames=confirm_frames)
+        self._current_window: Optional[Dict[str, Any]] = None
+        self._window_completed = False
         self._b_publisher = self.create_publisher(String, "/fruit/b_pick_plan", plan_qos())
         self._c_publisher = self.create_publisher(String, "/fruit/c_pick_plan", plan_qos())
         self._subscription = self.create_subscription(
@@ -107,10 +175,57 @@ class FruitQrPlannerNode(Node):
             self._on_current_area,
             plan_qos(),
         )
+        self._window_subscription = self.create_subscription(
+            String,
+            observation_window_topic,
+            self._on_observation_window,
+            observation_window_qos(),
+        )
         self.get_logger().info(
             f"Waiting for B-zone QR code on {image_topic}; confirm_frames={confirm_frames}"
         )
 
+    def _on_observation_window(self, message: String) -> None:
+        cmd = _parse_observation_window_command(message.data)
+        if cmd is None:
+            self.get_logger().warning(
+                f"Ignoring invalid observation window payload: {message.data}"
+            )
+            return
+
+        window_id = cmd["window_id"]
+        valid = cmd["valid"]
+        area = cmd["area"]
+        purpose = cmd["purpose"]
+
+        if valid == 1:
+            if purpose == "qr" and area in ("B", "C"):
+                self._current_window = cmd
+                self._window_completed = False
+                if area == "B":
+                    self._sequence.state = PlanSequence.WAITING_FOR_B
+                    self._sequence._reset_candidate()
+                else:
+                    self._sequence.state = PlanSequence.WAITING_FOR_C
+                    self._sequence._reset_candidate()
+                self.get_logger().info(
+                    f"Opened QR observation window '{window_id}' for area {area}"
+                )
+            else:
+                if self._current_window is not None:
+                    self._current_window = None
+                    self._sequence._reset_candidate()
+        else:
+            if (
+                self._current_window is not None
+                and self._current_window["window_id"] == window_id
+            ):
+                self._current_window = None
+                self._sequence._reset_candidate()
+                self.get_logger().info(
+                    f"Closed QR observation window '{window_id}'"
+                )
+
     def _on_current_area(self, message: String) -> None:
         entered_c = message.data.strip().upper() == "C"
         if entered_c and self._sequence.expect_c():
@@ -120,9 +235,22 @@ class FruitQrPlannerNode(Node):
             )
 
     def _on_image(self, message: Image) -> None:
-        if self._sequence.state == PlanSequence.COMPLETED:
+        if self._current_window is None or self._window_completed:
             return
 
+        window = self._current_window
+        msg_stamp = message.header.stamp
+        if (msg_stamp.sec, msg_stamp.nanosec) < (window["start_sec"], window["start_nanosec"]):
+            return
+
+        target_area = window["area"]
+        if target_area == "B" and self._sequence.state != PlanSequence.WAITING_FOR_B:
+            self._sequence.state = PlanSequence.WAITING_FOR_B
+            self._sequence._reset_candidate()
+        elif target_area == "C" and self._sequence.state != PlanSequence.WAITING_FOR_C:
+            self._sequence.state = PlanSequence.WAITING_FOR_C
+            self._sequence._reset_candidate()
+
         try:
             image = image_to_bgr(message, self._cv2)
             payload = self._decode(image)
@@ -141,19 +269,26 @@ class FruitQrPlannerNode(Node):
         if plan is None:
             return
 
+        plan["observation_window_id"] = window["window_id"]
+        plan["image_stamp_sec"] = msg_stamp.sec
+        plan["image_stamp_nanosec"] = msg_stamp.nanosec
+
         output = String()
         output.data = encode_plan_json(plan)
         if plan["zone"] == "B":
             self._b_publisher.publish(output)
+            self._window_completed = True
             self.get_logger().info(
-                f"Published B-zone pick plan with {plan['task_count']} tasks; now waiting for C"
+                f"Published B-zone pick plan with {plan['task_count']} tasks "
+                f"(window={window['window_id']}); waiting for next window"
             )
         else:
             self._c_publisher.publish(output)
+            self._window_completed = True
             self.get_logger().info(
-                f"Published C-zone pick plan with {plan['task_count']} tasks; QR sequence completed"
+                f"Published C-zone pick plan with {plan['task_count']} tasks "
+                f"(window={window['window_id']})"
             )
-            self._stop_scanning()
 
     def _stop_scanning(self) -> None:
         if self._subscription is not None:
diff --git a/src/joy_linux/README.md b/src/joy_linux/README.md
index 27429e2..2c72a22 100644
--- a/src/joy_linux/README.md
+++ b/src/joy_linux/README.md
@@ -14,6 +14,10 @@ For the Xbox Series controller used on this robot, the left stick vertical axis
 (axis 1) drives `linear.x`, and the right stick horizontal axis (axis 2) drives
 `angular.z` (`geometry_msgs/msg/Twist` on `/cmd_vel`).
 
+The workspace `start_tele.sh` reuses an existing serial bridge and leaves it running
+on exit. `joy_cmd_vel_serial.launch.py` accepts `start_serial_bridge:=false` to
+launch only joystick teleop; its default remains `true` for direct launch users.
+
 ## Device Selection
 
 There are two parameters controlling which device should be used:
diff --git a/src/joy_linux/launch/joy_cmd_vel_serial.launch.py b/src/joy_linux/launch/joy_cmd_vel_serial.launch.py
index 95ce0cc..2f5b208 100644
--- a/src/joy_linux/launch/joy_cmd_vel_serial.launch.py
+++ b/src/joy_linux/launch/joy_cmd_vel_serial.launch.py
@@ -1,6 +1,8 @@
 from launch import LaunchDescription
-from launch.actions import IncludeLaunchDescription
+from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
+from launch.conditions import IfCondition
 from launch.launch_description_sources import PythonLaunchDescriptionSource
+from launch.substitutions import LaunchConfiguration
 
 
 JOY_LAUNCH = '/home/rc4/fruit_ws/src/joy_linux/launch/joy_conventor.launch.py'
@@ -15,7 +17,15 @@ def generate_launch_description():
         PythonLaunchDescriptionSource(JOY_LAUNCH)
     )
     serial_launch = IncludeLaunchDescription(
-        PythonLaunchDescriptionSource(SERIAL_LAUNCH)
+        PythonLaunchDescriptionSource(SERIAL_LAUNCH),
+        condition=IfCondition(LaunchConfiguration('start_serial_bridge')),
     )
 
-    return LaunchDescription([joy_launch, serial_launch])
+    return LaunchDescription([
+        DeclareLaunchArgument(
+            'start_serial_bridge', default_value='true',
+            description='Start the serial bridge; false when reusing an existing bridge',
+        ),
+        joy_launch,
+        serial_launch,
+    ])
diff --git a/src/navigation/fruit_navigation/fruit_navigation/axis_waypoint_action_server.py b/src/navigation/fruit_navigation/fruit_navigation/axis_waypoint_action_server.py
index 5b2ad0a..0e4a2eb 100644
--- a/src/navigation/fruit_navigation/fruit_navigation/axis_waypoint_action_server.py
+++ b/src/navigation/fruit_navigation/fruit_navigation/axis_waypoint_action_server.py
@@ -23,7 +23,6 @@ from tf2_ros import Buffer, TransformException, TransformListener
 
 from fruit_navigation.axis_planner import (
     AxisPose,
-    ProgressWatchdog,
     normalize_angle,
     pose_from_ros,
 )
@@ -345,11 +344,6 @@ class AxisWaypointActionServer(Node):
 
     def _rotate_to(self, goal_handle, target_yaw: float, yaw_tolerance: float) -> bool:
         period = 1.0 / self._control_frequency
-        watchdog = ProgressWatchdog(
-            self._progress_timeout_sec,
-            self._progress_min_delta,
-            time.monotonic(),
-        )
         while rclpy.ok():
             if self._should_stop(goal_handle):
                 return False
@@ -363,14 +357,6 @@ class AxisWaypointActionServer(Node):
                     return True
                 time.sleep(period)
                 continue
-            if watchdog.stalled(error, time.monotonic()):
-                self.get_logger().error(
-                    "Aborting waypoint goal: no rotation progress "
-                    f"current_yaw={current.yaw:.3f} target_yaw={target_yaw:.3f} "
-                    f"yaw_error={error:.3f} tolerance={yaw_tolerance:.3f}"
-                )
-                self._publish_stop_burst()
-                return False
             twist = Twist()
             twist.angular.z = self._rotation_command(error)
             self._publish_limited_command(twist)
@@ -389,11 +375,6 @@ class AxisWaypointActionServer(Node):
         final_yaw: Optional[float] = None,
     ) -> bool:
         period = 1.0 / self._control_frequency
-        watchdog = ProgressWatchdog(
-            self._progress_timeout_sec,
-            self._progress_min_delta,
-            time.monotonic(),
-        )
         direction_x = math.cos(path_heading)
         direction_y = math.sin(path_heading)
         while rclpy.ok():
@@ -424,21 +405,6 @@ class AxisWaypointActionServer(Node):
                     return True
                 time.sleep(period)
                 continue
-            progress_error = math.hypot(remaining, lateral_error)
-            if watchdog.stalled(progress_error, time.monotonic()):
-                self.get_logger().error(
-                    "Aborting waypoint goal: no progress "
-                    f"start=({start.x:.3f},{start.y:.3f}) "
-                    f"current=({current.x:.3f},{current.y:.3f},{current.yaw:.3f}) "
-                    f"target=({target.x:.3f},{target.y:.3f},{target.yaw:.3f}) "
-                    f"traveled={traveled:.3f}/{distance:.3f} "
-                    f"remaining={remaining:.3f} lateral_error={lateral_error:.3f} "
-                    f"progress_error={progress_error:.3f} "
-                    f"forward_tolerance={self._forward_tolerance:.3f} "
-                    f"lateral_tolerance={self._lateral_tolerance:.3f}"
-                )
-                self._publish_stop_burst()
-                return False
 
             yaw_target = approach_yaw
             if final_yaw is not None and remaining < self._final_yaw_blend_distance:
diff --git a/start_fruit_mission.sh b/start_fruit_mission.sh
index e19b412..65f3c3f 100755
--- a/start_fruit_mission.sh
+++ b/start_fruit_mission.sh
@@ -285,21 +285,7 @@ wait_for_odometry() {
   else
     status=$?
   fi
-  log "WARNING: $label odometry unavailable; no recognition without stationary feedback"
-}
-
-wait_for_active_off() {
-  local timeout_sec=$1
-  local status
-
-  log "WAIT camera mode manager active=OFF timeout=${timeout_sec}s"
-  if run_readiness_client active-mode --mode OFF --timeout "$timeout_sec"; then
-    log "READY camera mode manager active=OFF"
-    return 0
-  else
-    status=$?
-  fi
-  log "WARNING: camera mode unavailable; mission will skip unavailable work"
+  log "WARNING: $label odometry unavailable; navigation requires odometry"
 }
 
 wait_for_topic_endpoint_count() {
@@ -460,11 +446,9 @@ import yaml
 with open(sys.argv[1], encoding="utf-8") as stream:
     document = yaml.safe_load(stream)
 params = document["fruit_mission"]["ros__parameters"]
-tts = params["tts"]
+tts = params.get("tts", {})
 for key in (
-    "piper_executable",
-    "model_path",
-    "config_path",
+    "manifest_path",
     "aplay_executable",
 ):
     value = tts.get(key)
@@ -474,20 +458,18 @@ for key in (
 PY
 )" || die "failed to read mission runtime paths from $MISSION_CONFIG"
 mapfile -t mission_runtime_path_values <<< "$mission_runtime_paths"
-if ((${#mission_runtime_path_values[@]} != 4)); then
+if ((${#mission_runtime_path_values[@]} != 2)); then
   die "mission config returned an unexpected number of runtime paths"
 fi
 
-PIPER_EXECUTABLE=${mission_runtime_path_values[0]}
-MODEL_PATH=${mission_runtime_path_values[1]}
-MODEL_CONFIG=${mission_runtime_path_values[2]}
-APLAY_EXECUTABLE=${mission_runtime_path_values[3]}
+VOICE_MANIFEST=${mission_runtime_path_values[0]}
+APLAY_EXECUTABLE=${mission_runtime_path_values[1]}
 
 check_file "$WAYPOINT_FILE"
-for required_file in "$MODEL_PATH" "$MODEL_CONFIG"; do
-  [[ -r "$required_file" ]] || log "WARNING: speech file unavailable: $required_file"
+for required_file in "$VOICE_MANIFEST"; do
+  [[ -r "$required_file" ]] || log "WARNING: voice manifest unavailable: $required_file"
 done
-for required_executable in "$PIPER_EXECUTABLE" "$APLAY_EXECUTABLE"; do
+for required_executable in "$APLAY_EXECUTABLE"; do
   [[ -x "$required_executable" ]] || log "WARNING: speech executable unavailable: $required_executable"
 done
 
@@ -520,7 +502,6 @@ if [[ "$REUSE_NAVIGATION" == true ]]; then
     /fruit_perception_container \
     /camera \
     /fruit_perception_node \
-    /camera_mode_manager \
     /fruit_qr_planner \
     /arm_planner_node \
     /fruit_mission
@@ -529,13 +510,11 @@ else
     /livox_ros_driver2 \
     /laserMapping \
     /loam_interface \
-    /serial_controller \
     /axis_waypoint_action_server \
     /twist_mux \
     /fruit_perception_container \
     /camera \
     /fruit_perception_node \
-    /camera_mode_manager \
     /fruit_qr_planner \
     /arm_planner_node \
     /fruit_mission
@@ -549,7 +528,10 @@ if [[ "$REUSE_NAVIGATION" != true ]]; then
     "$MONITOR_SCRIPT" fruit-mission
 fi
 
-START_SERIAL_BRIDGE=true
+START_SERIAL_BRIDGE=$(run_readiness_client bridge-start-required)
+if [[ "$START_SERIAL_BRIDGE" == false ]]; then
+  log "RUN reuse: existing serial bridge remains owned by its original launcher"
+fi
 if [[ "$REUSE_NAVIGATION" == true ]]; then
   START_SERIAL_BRIDGE=false
   log "RUN reuse: existing navigation, serial bridge, and debug monitor remain owned by start_navigation.sh"
@@ -573,7 +555,6 @@ start_process "perception" \
 wait_for_nodes "perception readiness" 10 \
   /serial_controller \
   /fruit_perception_node \
-  /camera_mode_manager \
   /fruit_qr_planner
 wait_for_serial_bridge_running 10
 start_process "arm_planner" \
@@ -604,7 +585,12 @@ wait_for_topic_endpoint_count \
   publishers \
   1 \
   10
-wait_for_active_off 10
+wait_for_topic_endpoint_count \
+  "perception observation window subscription" \
+  "/fruit_mission/observation_window" \
+  subscribers \
+  1 \
+  10
 
 if ((DEBUG_RECORD_ENABLED)); then
   start_process "rosbag" \
@@ -623,7 +609,7 @@ start_process "mission" \
 if [[ "$REUSE_NAVIGATION" == true ]]; then
   log "RUN mission started; Ctrl-C stops mission, arm planner, and perception but keeps the reused navigation stack running"
 else
-  log "RUN mission started; press Ctrl-C to stop the complete stack"
+  log "RUN mission started; press Ctrl-C to stop launched processes (reused bridge stays running)"
 fi
 set +e
 # A dead perception/QR/planner process must not tear down the competition.
diff --git a/start_fruit_perception.sh b/start_fruit_perception.sh
index a7a08d4..258d62c 100755
--- a/start_fruit_perception.sh
+++ b/start_fruit_perception.sh
@@ -26,6 +26,13 @@ trap on_signal INT TERM
 source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
 set -u
 
+START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
+  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
+if [[ "$START_SERIAL_BRIDGE" == false ]]; then
+  printf '[perception] reusing existing serial bridge; it will not be stopped on exit\n'
+  set -- "$@" start_serial_bridge:=false
+fi
+
 "$WORKSPACE/scripts/robot_debug_monitor.sh" perception &
 DEBUG_PID=$!
 
diff --git a/start_navigation.sh b/start_navigation.sh
index 494f5bc..adaa389 100755
--- a/start_navigation.sh
+++ b/start_navigation.sh
@@ -124,8 +124,14 @@ if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
   exit 1
 fi
 
-start_process "serial bridge" \
-  ros2 launch "$HARDWARE_LAUNCH"
+START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
+  --ros-args --params-file "$SCRIPT_CLIENT_CONFIG")
+if [[ "$START_SERIAL_BRIDGE" == true ]]; then
+  start_process "serial bridge" \
+    ros2 launch "$HARDWARE_LAUNCH"
+else
+  printf '[navigation] reusing existing serial bridge; it will not be stopped on exit\n'
+fi
 
 start_process "axis navigation and odometry" \
   ros2 launch "$NAV_LAUNCH"
@@ -142,7 +148,7 @@ start_process "rviz2" \
     "DBUS_SESSION_BUS_ADDRESS=unix:path=$DESKTOP_RUNTIME_DIR/bus" \
     rviz2 -d "$RVIZ_CONFIG" -f "odom"
 
-printf '[navigation] axis odometry navigation, serial bridge, and RViz started (frame=odom). Press Ctrl-C to stop them.\n'
+printf '[navigation] axis odometry navigation, serial bridge, and RViz ready (frame=odom). Press Ctrl-C to stop launched processes.\n'
 
 if wait -n "${pids[@]}"; then
   status=0
diff --git a/start_tele.sh b/start_tele.sh
index 8484128..1ec4bde 100755
--- a/start_tele.sh
+++ b/start_tele.sh
@@ -7,5 +7,11 @@ WORKSPACE=/home/rc4/fruit_ws
 source "$WORKSPACE/scripts/setup_fruit_runtime.bash"
 set -u
 
-exec ros2 launch "$WORKSPACE/src/joy_linux/launch/joy_cmd_vel_serial.launch.py"
+START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
+  --ros-args --params-file "$WORKSPACE/src/fruit_mission/config/script_ros_clients.yaml")
+if [[ "$START_SERIAL_BRIDGE" == false ]]; then
+  printf '[teleop] reusing existing serial bridge; it will not be stopped on exit\n'
+fi
+exec ros2 launch "$WORKSPACE/src/joy_linux/launch/joy_cmd_vel_serial.launch.py" \
+  "start_serial_bridge:=$START_SERIAL_BRIDGE"
 
diff --git a/test_camera_arm_target.sh b/test_camera_arm_target.sh
index 53263a4..0553ed9 100755
--- a/test_camera_arm_target.sh
+++ b/test_camera_arm_target.sh
@@ -7,16 +7,13 @@ readonly RUNTIME_SETUP="${WORKSPACE_DIR}/scripts/setup_fruit_runtime.bash"
 
 readonly START_SCRIPT="${WORKSPACE_DIR}/start_fruit_perception.sh"
 readonly CLIENT_CONFIG="${WORKSPACE_DIR}/src/fruit_mission/config/script_ros_clients.yaml"
-readonly RUN_DIR="${WORKSPACE_DIR}/tmp/camera-arm-target"
-readonly LAUNCH_LOG="${RUN_DIR}/launch.log"
-readonly CLIENT_LOG="${RUN_DIR}/client.log"
+readonly LOGGING_HELPER="${WORKSPACE_DIR}/scripts/fruit_runtime_logging.bash"
 
 LAUNCH_PID=""
 PLANNER_PID=""
 STOPPING=0
-ZONE=""
+MODEL=""
 DIRECTION=""
-B_FRUIT=""
 DEBUG=0
 
 log()
@@ -27,12 +24,15 @@ log()
 usage()
 {
   cat <<'EOF'
-Usage: ./test_camera_arm_target.sh --zone A|B|C --left|--right [--apple|--pear] [--debug]
-
-Select the current area and observation direction before running the existing
-Camera recognition and arm-target workflow. Zone B requires exactly one of
---apple or --pear. --debug prints every Camera detection message and every
-selected target point with their ROS timestamps.
+Usage: ./test_camera_arm_target.sh --model A|B --left|--right [--debug]
+
+Test camera and arm coordination.
+Arm returns to the home position first, executes observation and grasp,
+then executes the corresponding store preset (left_store or right_store)
+to release the fruit, then explicitly executes home and waits for completion.
+  --model A|B     A: vegetable model, B: fruit model (supports --zone A|B as alias)
+  --left|--right  Arm side presets (left_look/left_store or right_look/right_store)
+  --debug         Print per-frame camera detections and coordinates
 EOF
 }
 
@@ -40,14 +40,13 @@ parse_arguments()
 {
   while (( $# > 0 )); do
     case "$1" in
-      --zone)
-        (( $# >= 2 )) || { log "ERROR: --zone requires A, B, or C"; return 2; }
-        [[ -z "${ZONE}" ]] || { log "ERROR: --zone may only be specified once"; return 2; }
+      --model|-m|--zone|-z)
+        (( $# >= 2 )) || { log "ERROR: $1 requires A or B"; return 2; }
+        [[ -z "${MODEL}" ]] || { log "ERROR: model may only be specified once"; return 2; }
         case "$2" in
-          A|a) ZONE=A ;;
-          B|b) ZONE=B ;;
-          C|c) ZONE=C ;;
-          *) log "ERROR: --zone must be A, B, or C"; return 2 ;;
+          A|a) MODEL=A ;;
+          B|b) MODEL=B ;;
+          *) log "ERROR: model must be A (vegetable) or B (fruit)"; return 2 ;;
         esac
         shift 2
         ;;
@@ -59,14 +58,6 @@ parse_arguments()
         DIRECTION=$1
         shift
         ;;
-      --apple|--pear)
-        [[ -z "${B_FRUIT}" ]] || {
-          log "ERROR: choose exactly one of --apple or --pear"
-          return 2
-        }
-        B_FRUIT=$1
-        shift
-        ;;
       --debug)
         DEBUG=1
         shift
@@ -82,20 +73,11 @@ parse_arguments()
     esac
   done
 
-  [[ -n "${ZONE}" ]] || { log "ERROR: --zone A, --zone B, or --zone C is required"; return 2; }
+  [[ -n "${MODEL}" ]] || { log "ERROR: --model A (vegetable) or --model B (fruit) is required"; return 2; }
   [[ -n "${DIRECTION}" ]] || {
     log "ERROR: --left or --right is required"
     return 2
   }
-  if [[ "${ZONE}" == B ]]; then
-    [[ -n "${B_FRUIT}" ]] || {
-      log "ERROR: --zone B requires exactly one of --apple or --pear"
-      return 2
-    }
-  elif [[ -n "${B_FRUIT}" ]]; then
-    log "ERROR: --apple and --pear may only be used with --zone B"
-    return 2
-  fi
 }
 
 process_is_alive()
@@ -153,20 +135,17 @@ cleanup()
     kill -TERM "${job_pid}" 2>/dev/null || true
   done < <(jobs -pr)
 
-  if process_is_alive "${LAUNCH_PID}"; then
-    timeout --foreground 4s ros2 run fruit_mission camera_arm_target_client \
-      --mode-off --ros-args --params-file "${CLIENT_CONFIG}" \
-      >/dev/null 2>&1 || true
-  fi
   stop_process_group "${LAUNCH_PID}"
   stop_process_group "${PLANNER_PID}"
+  fruit_logging_close "${status}"
   exit "${status}"
 }
 
 show_failure_logs()
 {
-  printf '\nLast perception output:\n' >&2
-  tail -n 40 "${LAUNCH_LOG}" >&2 || true
+  if [[ -n "${FRUIT_RUNTIME_SESSION_DIR:-}" ]]; then
+    log "Detailed logs available at: ${FRUIT_RUNTIME_SESSION_DIR}/"
+  fi
 }
 
 check_prerequisites()
@@ -199,8 +178,8 @@ keep_running()
   while true; do
     if (( stack_failure_reported == 0 )) && ! process_is_alive "${LAUNCH_PID}"; then
       stack_failure_reported=1
-      log "ERROR: perception stack exited"
-      tail -n 40 "${LAUNCH_LOG}" >&2 || true
+      log "ERROR: perception stack exited unexpectedly"
+      show_failure_logs
     fi
     sleep 1
   done
@@ -215,24 +194,18 @@ main()
     exit 2
   }
 
-  mkdir -p "${RUN_DIR}"
-  : >"${LAUNCH_LOG}"
-  : >"${CLIENT_LOG}"
-
   # shellcheck source=/home/rc4/fruit_ws/setup_fruit_runtime.bash
   source "${RUNTIME_SETUP}"
+  source "${LOGGING_HELPER}"
   set -u
+  fruit_logging_init "camera_arm_target" "${DEBUG}"
   check_prerequisites || exit 1
   if (( DEBUG != 0 )); then
     client_args+=(--debug)
-    log "Per-frame Camera detection and target-point logging is enabled"
-  fi
-  if [[ -n "${B_FRUIT}" ]]; then
-    client_args+=("${B_FRUIT}")
   fi
   ros2 run fruit_mission camera_arm_target_client \
     --require-stack-absent --ros-args --params-file "${CLIENT_CONFIG}" || {
-    log "ERROR: perception or serial nodes are already running"
+    log "ERROR: perception, arm planner, or mission nodes are already running"
     exit 1
   }
 
@@ -240,18 +213,31 @@ main()
   trap 'exit 130' INT
   trap 'exit 143' TERM
 
-  setsid "${START_SCRIPT}" serial_debug_raw_frame:=true \
-    > >(tee -a "${LAUNCH_LOG}") 2>&1 &
+  local perception_log="${FRUIT_RUNTIME_SESSION_DIR}/perception.log"
+  local planner_log="${FRUIT_RUNTIME_SESSION_DIR}/arm_planner.log"
+  local client_log="${FRUIT_RUNTIME_SESSION_DIR}/client.log"
+
+  local filter_script="${WORKSPACE_DIR}/scripts/runtime_log_filter.py"
+  local -a perception_cmd=("${START_SCRIPT}")
+  if (( DEBUG != 0 )); then
+    perception_cmd+=(--debug)
+  fi
+
+  setsid bash -c '
+    exec "$@" 2>&1 | /usr/bin/python3 "'"${filter_script}"'" --output-file "'"${perception_log}"'" --component "perception" '"$([[ ${DEBUG} -ne 0 ]] && echo "--debug")"'
+  ' runner "${perception_cmd[@]}" &
   LAUNCH_PID=$!
-  setsid ros2 launch fruit_arm_planner fruit_arm_planner.launch.py \
-    >>"${LAUNCH_LOG}" 2>&1 &
+
+  setsid bash -c '
+    exec ros2 launch fruit_arm_planner fruit_arm_planner.launch.py log_level:=debug 2>&1 | /usr/bin/python3 "'"${filter_script}"'" --output-file "'"${planner_log}"'" --component "arm_planner"
+  ' runner &
   PLANNER_PID=$!
 
   log "Waiting for the perception stack"
   if ros2 run fruit_mission camera_arm_target_client \
-    --zone "${ZONE}" "${DIRECTION}" "${client_args[@]}" \
+    --model "${MODEL}" "${DIRECTION}" "${client_args[@]}" \
     --ros-args --params-file "${CLIENT_CONFIG}" \
-    2>&1 | tee -a "${CLIENT_LOG}"; then
+    2>&1 | /usr/bin/python3 "${filter_script}" --output-file "${client_log}" --component "client" $( (( DEBUG != 0 )) && echo "--debug" ); then
     log "Task sequence completed"
   else
     log "Task sequence stopped at the failed step; processes remain available for inspection"
diff --git a/test_camera_detector.py b/test_camera_detector.py
old mode 100644
new mode 100755
index 7c0616f..d59a0ec
--- a/test_camera_detector.py
+++ b/test_camera_detector.py
@@ -1,5 +1,5 @@
 #!/usr/bin/env python3
-"""OpenCV GUI camera detector smoke-test tool."""
+"""OpenCV GUI camera detector and 3D depth inspection tool for single RealSense camera."""
 
 from __future__ import annotations
 
@@ -14,7 +14,6 @@ import time
 from dataclasses import dataclass
 from pathlib import Path
 from typing import Any
-from urllib.parse import unquote, urlparse
 
 
 DEFAULT_DISPLAY = ':0'
@@ -50,20 +49,22 @@ import openvino as ov  # noqa: E402
 import rclpy  # noqa: E402
 import yaml  # noqa: E402
 from cv_bridge import CvBridge  # noqa: E402
+from fruit_arm_interfaces.msg import ArmTelemetry  # noqa: E402
+from fruit_arm_planner.joint_transformer import JointTransformer  # noqa: E402
+from fruit_mission.camera_target_transform import (  # noqa: E402
+    load_camera_extrinsic,
+    transform_camera_to_end_effector_mm,
+    transform_end_to_base_mm,
+)
 from openvino.preprocess import PrePostProcessor  # noqa: E402
 from rclpy.node import Node  # noqa: E402
 from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
-from sensor_msgs.msg import Image  # noqa: E402
-
-from camera_control_gui import (  # noqa: E402
-    CameraControlPanel,
-    CameraControlService,
-    save_camera_controls,
-)
+from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
 
 
 WORKSPACE = Path('/home/rc4/fruit_ws')
 CONFIG_PATH = WORKSPACE / 'src/fruit_perception/config/fruit_perception.yaml'
+EXTRINSIC_CONFIG_PATH = WORKSPACE / 'src/fruit_mission/config/camera_extrinsic.yaml'
 TMP_DIR = WORKSPACE / 'tmp'
 MODEL_INPUT_SIZE = 640
 PAD_COLOR = 114
@@ -92,58 +93,128 @@ class LetterboxTransform:
 class CameraSelection:
     name: str
     kind: str
-    topic: str
+    color_topic: str
+    depth_topic: str
+    info_topic: str
     parameters: dict[str, Any]
 
 
 class FrameSubscriber(Node):
-    def __init__(self, topic: str) -> None:
+    """Subscribes to color image, aligned depth image, camera intrinsics, and arm telemetry."""
+
+    def __init__(
+        self,
+        color_topic: str,
+        depth_topic: str,
+        info_topic: str,
+        joint_state_topic: str = '/arm_planner_node/joint_states',
+        arm_config_path: Path | str | None = None,
+    ) -> None:
         super().__init__('test_camera_detector')
         self._bridge = CvBridge()
-        self._latest_frame: np.ndarray | None = None
-        self._latest_encoding = ''
-        self._subscriptions = [
-            self.create_subscription(
-                Image,
-                topic,
-                self._on_image,
-                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
-            ),
-            self.create_subscription(
-                Image,
-                topic,
-                self._on_image,
-                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
-            ),
-        ]
+        self._latest_color: np.ndarray | None = None
+        self._latest_depth: np.ndarray | None = None
+        self.camera_info: CameraInfo | None = None
+
+        self._tcp_pose: tuple[float, float, float, float, float] | None = None
+        self._tcp_received_at = 0.0
+        self.status_max_age_sec = 0.25
+        self.transformer: JointTransformer | None = None
+        self.joint_state_topic = joint_state_topic
+        self.telemetry_conflict = False
+
+        if arm_config_path is not None and Path(arm_config_path).is_file():
+            try:
+                with Path(arm_config_path).open(encoding='utf-8') as stream:
+                    arm_params = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
+                self.transformer = JointTransformer.from_config_dict(arm_params['joint_mapping'])
+                self.status_max_age_sec = float(arm_params['planning']['status_max_age_sec'])
+            except Exception as error:  # noqa: BLE001
+                self.get_logger().warning(f'Failed to load arm configuration: {error}')
+
+        qos_best_effort = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
+        qos_reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
+        qos_telemetry = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
+
+        self.create_subscription(Image, color_topic, self._on_color, qos_best_effort)
+        self.create_subscription(Image, depth_topic, self._on_depth, qos_best_effort)
+        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_reliable)
+        self.create_subscription(
+            ArmTelemetry, joint_state_topic, self._on_telemetry, qos_telemetry,
+        )
+
+    def set_telemetry_conflict(self, conflict: bool) -> None:
+        self.telemetry_conflict = conflict
+        if conflict:
+            self._tcp_pose = None
 
-    def _on_image(self, message: Image) -> None:
+    def _on_color(self, message: Image) -> None:
         try:
-            frame = self._bridge.imgmsg_to_cv2(
-                message,
-                desired_encoding='rgb8',
-            )
+            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
+            self._latest_color = np.ascontiguousarray(frame)
         except Exception as error:  # noqa: BLE001
-            self.get_logger().error(f'Frame conversion failed: {error}')
+            self.get_logger().error(f'Color frame conversion failed: {error}')
+
+    def _on_depth(self, message: Image) -> None:
+        try:
+            depth = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
+            self._latest_depth = np.ascontiguousarray(depth)
+        except Exception as error:  # noqa: BLE001
+            self.get_logger().error(f'Depth frame conversion failed: {error}')
+
+    def _on_info(self, message: CameraInfo) -> None:
+        self.camera_info = message
+
+    def _on_telemetry(self, message: ArmTelemetry) -> None:
+        if self.telemetry_conflict:
+            return
+        received_at = time.monotonic()
+        if message.header.frame_id != 'arm_base_link':
+            return
+        names = [f'joint_{i}' for i in range(1, 5)]
+        hardware = message.hardware_joint_state
+        model = message.model_joint_state
+        for state in (hardware, model):
+            if (len(state.name) != 4 or set(state.name) != set(names)
+                    or len(state.position) != 4
+                    or not all(math.isfinite(q) for q in state.position)):
+                return
+        joints = tuple(hardware.position[hardware.name.index(name)] for name in names)
+        tcp = message.tcp_position
+        if 'joint_1' not in model.name:
             return
-        self._latest_frame = np.ascontiguousarray(frame)
-        self._latest_encoding = message.encoding
+        yaw = float(model.position[model.name.index('joint_1')])
+        pose = (float(tcp.x), float(tcp.y), float(tcp.z), float(message.tcp_pitch_deg), yaw)
+        if (self.transformer is not None and not self.transformer.check_hardware_limits(joints)[0]
+                or not all(math.isfinite(v) for v in pose)):
+            return
+        self._tcp_pose = pose
+        self._tcp_received_at = received_at
+
+    def get_fresh_tcp_pose(self) -> tuple[float, float, float, float, float] | None:
+        if self.telemetry_conflict or self._tcp_pose is None:
+            return None
+        if time.monotonic() - self._tcp_received_at > self.status_max_age_sec:
+            return None
+        return self._tcp_pose
 
-    def take_latest_frame(self) -> np.ndarray | None:
-        frame = self._latest_frame
-        self._latest_frame = None
-        return frame
+    def take_latest(self) -> tuple[np.ndarray | None, np.ndarray | None]:
+        color = self._latest_color
+        depth = self._latest_depth
+        self._latest_color = None
+        return color, depth
 
 
 def parse_args() -> argparse.Namespace:
     parser = argparse.ArgumentParser(
-        description='Open the camera and show OpenVINO detections.',
+        description='RealSense single camera object detection and 3D depth debug viewer.',
     )
     parser.add_argument(
         '--model',
-        required=True,
-        choices=('A', 'B', 'C', 'D'),
-        help='area detector: A/C vegetables, B/D fruits (species only)',
+        '-m',
+        default='A',
+        choices=('A', 'B', 'C', 'D', 'a', 'b', 'c', 'd'),
+        help='area detector: A/C vegetables, B/D fruits (default: A)',
     )
     parser.add_argument(
         '--conf',
@@ -164,6 +235,11 @@ def parse_args() -> argparse.Namespace:
         default=DEFAULT_DISPLAY,
         help=f'desktop display for the GUI window (default: {DEFAULT_DISPLAY})',
     )
+    parser.add_argument(
+        '--controls',
+        action='store_true',
+        help='open the camera exposure and control parameters adjustment panel',
+    )
     return parser.parse_args()
 
 
@@ -177,7 +253,9 @@ def select_camera(config: dict[str, Any]) -> CameraSelection:
     return CameraSelection(
         name='camera',
         kind='camera',
-        topic='/camera/color/image_raw',
+        color_topic='/camera/color/image_raw',
+        depth_topic='/camera/aligned_depth_to_color/image_raw',
+        info_topic='/camera/color/camera_info',
         parameters=parameters,
     )
 
@@ -187,10 +265,263 @@ def model_path_for(letter: str, config: dict[str, Any]) -> Path:
     model_root = Path(perception['model_root'])
     if not model_root.is_absolute():
         raise ValueError('model_root must be an absolute source-directory path')
-    detector = 'fruit_type' if letter in ('B', 'D') else 'vegetable_type'
+    detector = 'fruit_type' if letter.upper() in ('B', 'D') else 'vegetable_type'
     return model_root / detector / 'best.onnx'
 
 
+def load_extrinsic_transform() -> tuple[np.ndarray, np.ndarray] | None:
+    """Load R_E_C and t_E_C (mm) from camera_extrinsic.yaml using shared validation."""
+    try:
+        r, t = load_camera_extrinsic(EXTRINSIC_CONFIG_PATH)
+        return np.array(r, dtype=float), np.array(t, dtype=float)
+    except Exception as error:  # noqa: BLE001
+        print(f'Warning: failed to load camera extrinsic: {error}', file=sys.stderr)
+        return None
+
+
+def get_depth_at(
+    depth_image: np.ndarray | None, u: int, v: int, window_radius: int = 3,
+) -> float | None:
+    """Sample median depth in a small window around (u, v) in meters."""
+    if depth_image is None:
+        return None
+    h, w = depth_image.shape[:2]
+    if not (0 <= u < w and 0 <= v < h):
+        return None
+    u1, u2 = max(0, u - window_radius), min(w, u + window_radius + 1)
+    v1, v2 = max(0, v - window_radius), min(h, v + window_radius + 1)
+    window = depth_image[v1:v2, u1:u2]
+    valid = window[window > 0]
+    if len(valid) == 0:
+        return None
+    return float(np.median(valid)) * 0.001
+
+
+def pixel_to_camera_point(
+    u: float, v: float, depth_m: float,
+    fx: float, fy: float, cx: float, cy: float,
+) -> tuple[float, float, float]:
+    """Project 2D pixel + depth to 3D point in camera optical frame (m)."""
+    x = (u - cx) * depth_m / fx
+    y = (v - cy) * depth_m / fy
+    z = depth_m
+    return x, y, z
+
+
+def camera_to_base_point(
+    point_cam_m: tuple[float, float, float],
+    r_e_c: np.ndarray | list[list[float]],
+    t_e_c_mm: np.ndarray | list[float],
+    tcp_pose: tuple[float, float, float, float, float],
+) -> tuple[float, float, float]:
+    """Transform camera optical 3D point (m) to arm base frame coordinates (m).
+
+    tcp_pose: (tcp_x_m, tcp_y_m, tcp_z_m, pitch_deg, yaw_rad)
+    """
+    cam_mm = tuple(float(v) * 1000.0 for v in point_cam_m)
+    r_rows = tuple(tuple(float(v) for v in row) for row in r_e_c)
+    t_vals = tuple(float(v) for v in t_e_c_mm)
+    end_mm = transform_camera_to_end_effector_mm(cam_mm, r_rows, t_vals)
+    origin_mm = (tcp_pose[0] * 1000.0, tcp_pose[1] * 1000.0, tcp_pose[2] * 1000.0)
+    pitch_rad = math.radians(tcp_pose[3])
+    yaw_rad = float(tcp_pose[4])
+    base_mm = transform_end_to_base_mm(end_mm, origin_mm, yaw_rad, pitch_rad)
+    return tuple(float(v) / 1000.0 for v in base_mm)
+
+
+def validate_camera_info(
+    info: CameraInfo | None,
+    image_width: int,
+    image_height: int,
+) -> tuple[float, float, float, float] | None:
+    """Validate camera info matrix; return (fx, fy, cx, cy) or None if invalid."""
+    if info is None:
+        return None
+    if info.width > 0 and info.height > 0:
+        if info.width != image_width or info.height != image_height:
+            return None
+    k = info.k
+    if len(k) < 9:
+        return None
+    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
+    if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(cx) and math.isfinite(cy)):
+        return None
+    if fx <= 0.0 or fy <= 0.0 or cx < 0.0 or cy < 0.0 or cx > image_width or cy > image_height:
+        return None
+    return fx, fy, cx, cy
+
+
+def launch_background_process(
+    command: list[str], log_filename: str,
+) -> tuple[subprocess.Popen, Path]:
+    TMP_DIR.mkdir(parents=True, exist_ok=True)
+    log_path = TMP_DIR / log_filename
+    log_file = log_path.open('w', encoding='utf-8')
+    log_file.write('$ ' + ' '.join(command) + '\n')
+    log_file.flush()
+    process = subprocess.Popen(
+        command,
+        stdout=log_file,
+        stderr=subprocess.STDOUT,
+        text=True,
+        start_new_session=True,
+    )
+    return process, log_path
+
+
+def ensure_arm_communication(
+    node: FrameSubscriber,
+) -> tuple[subprocess.Popen | None, subprocess.Popen | None, subprocess.Popen | None]:
+    """Check for existing serial bridge and planner; launch only missing nodes.
+
+    Returns (arm_debug_process, bridge_process, planner_process).
+    """
+    deadline = time.monotonic() + 0.6
+    while time.monotonic() < deadline:
+        rclpy.spin_once(node, timeout_sec=0.1)
+
+    node_names = set(node.get_node_names())
+    status_topic = '/fruit_hardware_bridge/arm/status'
+    joint_state_topic = getattr(node, 'joint_state_topic', '/arm_planner_node/joint_states')
+
+    planner_pubs = node.get_publishers_info_by_topic(joint_state_topic)
+    bridge_pubs = node.get_publishers_info_by_topic(status_topic)
+
+    expected_telemetry_type = 'fruit_arm_interfaces/msg/ArmTelemetry'
+    expected_status_type = 'fruit_hardware_interfaces/msg/ArmStatus'
+
+    # Check for telemetry publisher conflicts or type mismatches
+    if len(planner_pubs) > 1:
+        pub_nodes = [f'{p.node_namespace}/{p.node_name}' for p in planner_pubs]
+        print(
+            f'Warning: {joint_state_topic} has {len(planner_pubs)} publishers '
+            f'(duplicate conflict: {pub_nodes}); cannot use telemetry, '
+            'Base coordinates will be N/A',
+            file=sys.stderr,
+            flush=True,
+        )
+        node.set_telemetry_conflict(True)
+        return None, None, None
+
+    if len(planner_pubs) == 1 and planner_pubs[0].topic_type != expected_telemetry_type:
+        print(
+            f'Warning: {joint_state_topic} publisher has unexpected type '
+            f"'{planner_pubs[0].topic_type}'; expected '{expected_telemetry_type}'. "
+            'Base coordinates will be N/A',
+            file=sys.stderr,
+            flush=True,
+        )
+        node.set_telemetry_conflict(True)
+        return None, None, None
+
+    # Check for bridge publisher conflicts or type mismatches
+    if len(bridge_pubs) > 1:
+        pub_nodes = [f'{p.node_namespace}/{p.node_name}' for p in bridge_pubs]
+        print(
+            f'Warning: {status_topic} has {len(bridge_pubs)} publishers '
+            f'(duplicate conflict: {pub_nodes}); cannot reuse bridge, '
+            'Base coordinates will be N/A',
+            file=sys.stderr,
+            flush=True,
+        )
+        node.set_telemetry_conflict(True)
+        return None, None, None
+
+    if len(bridge_pubs) == 1 and bridge_pubs[0].topic_type != expected_status_type:
+        print(
+            f'Warning: {status_topic} publisher has unexpected type '
+            f"'{bridge_pubs[0].topic_type}'; expected '{expected_status_type}'. "
+            'Base coordinates will be N/A',
+            file=sys.stderr,
+            flush=True,
+        )
+        node.set_telemetry_conflict(True)
+        return None, None, None
+
+    has_planner = ('arm_planner_node' in node_names) or (len(planner_pubs) == 1)
+    has_bridge = ('serial_controller' in node_names) or (len(bridge_pubs) == 1)
+
+    debug_proc: subprocess.Popen | None = None
+    bridge_proc: subprocess.Popen | None = None
+    planner_proc: subprocess.Popen | None = None
+
+    try:
+        if has_bridge and has_planner:
+            print('Reusing existing serial bridge and arm planner', flush=True)
+        elif not has_bridge and not has_planner:
+            print(
+                'Neither serial bridge nor arm planner detected; '
+                'launching fruit_arm_debug.launch.py...',
+                flush=True,
+            )
+            debug_proc, log_p = launch_background_process(
+                ['ros2', 'launch', 'fruit_arm_planner', 'fruit_arm_debug.launch.py',
+                 'log_level:=info'],
+                'test_camera_detector_arm_debug.log',
+            )
+            print(f'Started fruit_arm_debug launch; log={log_p}', flush=True)
+        elif has_bridge and not has_planner:
+            print(
+                'Serial bridge detected, arm planner missing; '
+                'launching fruit_arm_planner.launch.py...',
+                flush=True,
+            )
+            planner_proc, log_p = launch_background_process(
+                ['ros2', 'launch', 'fruit_arm_planner', 'fruit_arm_planner.launch.py',
+                 'log_level:=info'],
+                'test_camera_detector_planner.log',
+            )
+            print(f'Started fruit_arm_planner launch; log={log_p}', flush=True)
+        else:  # not has_bridge and has_planner
+            print(
+                'Arm planner detected, serial bridge missing; '
+                'launching fruit_hardware_bridge.launch.py...',
+                flush=True,
+            )
+            bridge_proc, log_p = launch_background_process(
+                ['ros2', 'launch', 'fruit_hardware_bridge', 'fruit_hardware_bridge.launch.py',
+                 'log_level:=info'],
+                'test_camera_detector_serial.log',
+            )
+            print(f'Started fruit_hardware_bridge launch; log={log_p}', flush=True)
+
+        if debug_proc is not None or planner_proc is not None or bridge_proc is not None:
+            print('Waiting up to 10s for arm telemetry...', flush=True)
+            wait_deadline = time.monotonic() + 10.0
+            while time.monotonic() < wait_deadline:
+                if node.get_fresh_tcp_pose() is not None:
+                    print('Arm telemetry received successfully', flush=True)
+                    break
+                for p, name in (
+                    (debug_proc, 'arm_debug'),
+                    (bridge_proc, 'serial_bridge'),
+                    (planner_proc, 'arm_planner'),
+                ):
+                    if p is not None and p.poll() is not None:
+                        print(
+                            f'Warning: {name} exited unexpectedly: code {p.returncode}',
+                            file=sys.stderr,
+                            flush=True,
+                        )
+                rclpy.spin_once(node, timeout_sec=0.1)
+            else:
+                print(
+                    'Warning: arm telemetry not received within 10s; Base coordinates will be N/A',
+                    file=sys.stderr,
+                    flush=True,
+                )
+    except BaseException:
+        if planner_proc is not None:
+            stop_process(planner_proc)
+        if bridge_proc is not None:
+            stop_process(bridge_proc)
+        if debug_proc is not None:
+            stop_process(debug_proc)
+        raise
+
+    return debug_proc, bridge_proc, planner_proc
+
+
 def write_ros_params_file(selection: CameraSelection) -> Path:
     TMP_DIR.mkdir(parents=True, exist_ok=True)
     params_path = TMP_DIR / f'test_camera_detector_{selection.name}_params.yaml'
@@ -337,10 +668,7 @@ def parse_raw_yolo_output(
         class_scores = data[4:, column]
         best_class = int(np.argmax(class_scores))
         best_score = float(class_scores[best_class])
-        if (
-            not math.isfinite(best_score)
-            or best_score < confidence_threshold
-        ):
+        if not math.isfinite(best_score) or best_score < confidence_threshold:
             continue
 
         x1 = x - width / 2.0
@@ -452,74 +780,240 @@ def to_original_box(
 
 def draw_annotations(
     frame_rgb: np.ndarray,
+    depth_frame: np.ndarray | None,
     detections: list[Detection],
     transform: LetterboxTransform,
+    intrinsics: tuple[float, float, float, float] | None,
+    extrinsic: tuple[np.ndarray, np.ndarray] | None,
+    tcp_pose: tuple[float, float, float, float, float] | None,
+    mouse_u: int,
+    mouse_v: int,
     fps: float,
+    model_name: str,
+    ripeness_classifier: RipenessClassifier | None = None,
 ) -> np.ndarray:
     annotated = frame_rgb.copy()
-    color = (0, 255, 0)
+    h, w = annotated.shape[:2]
+
+    center_color = (0, 220, 255)
 
     for detection in detections:
-        x, y, width, height = to_original_box(detection, transform)
-        if width <= 0 or height <= 0:
+        bx, by, bw, bh = to_original_box(detection, transform)
+        if bw <= 0 or bh <= 0:
             continue
-        cv2.rectangle(annotated, (x, y), (x + width, y + height), color, 2)
-
-        label = f'{detection.label} {detection.score:.2f}'
-        text_size, baseline = cv2.getTextSize(
-            label,
-            cv2.FONT_HERSHEY_SIMPLEX,
-            0.55,
-            1,
-        )
-        text_x = min(max(x, 0), max(0, annotated.shape[1] - text_size[0] - 6))
-        text_y = max(y - 6, text_size[1] + 4)
+
+        # Ripeness classification
+        is_bad = detection.label.lower() == 'bad'
+        mat_text = ''
+        box_color = (0, 255, 0)
+
+        if is_bad:
+            box_color = (0, 0, 255)  # Red for bad produce
+            mat_text = '[BAD]'
+        elif ripeness_classifier is not None:
+            rgb_roi = frame_rgb[by:by + bh, bx:bx + bw]
+            mat_result = ripeness_classifier.classify(rgb_roi, detection.label)
+            if mat_result is not None:
+                mat_status, mat_conf = mat_result
+                if mat_status == 'ripe':
+                    box_color = (0, 255, 0)  # Green for ripe
+                    mat_text = f'[{mat_status.upper()} {mat_conf * 100:.0f}%]'
+                else:
+                    box_color = (0, 165, 255)  # Orange for unripe
+                    mat_text = f'[{mat_status.upper()} {mat_conf * 100:.0f}%]'
+
+        cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), box_color, 2)
+
+        # Center point
+        uc = bx + bw // 2
+        vc = by + bh // 2
+        depth_m = get_depth_at(depth_frame, uc, vc, window_radius=3)
+
+        # Draw center point crosshair
+        cv2.circle(annotated, (uc, vc), 4, center_color, -1)
+        cv2.line(annotated, (uc - 8, vc), (uc + 8, vc), center_color, 1)
+        cv2.line(annotated, (uc, vc - 8), (uc, vc + 8), center_color, 1)
+
+        mat_suffix = f' {mat_text}' if mat_text else ''
+        if depth_m is None:
+            main_label = f'{detection.label}{mat_suffix} {detection.score:.2f} | D: NoDepth'
+            sub_label = 'Depth missing/blocked at center'
+        else:
+            main_label = f'{detection.label}{mat_suffix} {detection.score:.2f} | D={depth_m:.3f}m'
+            if intrinsics is None:
+                sub_label = 'Cam: N/A (no CameraInfo)  Base: N/A'
+            else:
+                fx, fy, cx, cy = intrinsics
+                xc, yc, zc = pixel_to_camera_point(uc, vc, depth_m, fx, fy, cx, cy)
+                if extrinsic is None:
+                    sub_label = f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})m  Base: N/A (no extrinsic)'
+                elif tcp_pose is None:
+                    sub_label = f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})m  Base: N/A (no TCP)'
+                else:
+                    r_e_c, t_e_c = extrinsic
+                    xb, yb, zb = camera_to_base_point((xc, yc, zc), r_e_c, t_e_c, tcp_pose)
+                    sub_label = (
+                        f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})  '
+                        f'Base:({xb:+.2f}, {yb:+.2f}, {zb:+.2f})m'
+                    )
+
+        # Main label
+        ts_main, bl_main = cv2.getTextSize(main_label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
+        tx = min(max(bx, 0), max(0, w - ts_main[0] - 6))
+        ty = max(by - 6, ts_main[1] + 4)
         cv2.rectangle(
             annotated,
-            (
-                text_x,
-                text_y - text_size[1] - 4,
-                text_size[0] + 6,
-                text_size[1] + baseline + 6,
-            ),
-            color,
+            (tx, ty - ts_main[1] - 4),
+            (tx + ts_main[0] + 6, ty + bl_main + 2),
+            box_color,
             cv2.FILLED,
         )
+        text_color = (255, 255, 255) if is_bad else (0, 0, 0)
         cv2.putText(
+            annotated, main_label, (tx + 3, ty),
+            cv2.FONT_HERSHEY_SIMPLEX, 0.48, text_color, 1, cv2.LINE_AA,
+        )
+
+        # Sub label below box
+        ts_sub, bl_sub = cv2.getTextSize(sub_label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
+        sub_y = min(by + bh + ts_sub[1] + 6, h - 35)
+        sub_x = min(max(bx, 0), max(0, w - ts_sub[0] - 6))
+        cv2.rectangle(
             annotated,
-            label,
-            (text_x + 3, text_y),
-            cv2.FONT_HERSHEY_SIMPLEX,
-            0.55,
-            (0, 0, 0),
-            1,
-            cv2.LINE_AA,
+            (sub_x, sub_y - ts_sub[1] - 2),
+            (sub_x + ts_sub[0] + 4, sub_y + bl_sub + 2),
+            (30, 30, 30),
+            cv2.FILLED,
+        )
+        cv2.putText(
+            annotated, sub_label, (sub_x + 2, sub_y),
+            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA,
         )
 
-    fps_label = f'FPS: {fps:.1f}'
+    # Mouse inspection crosshair & coordinates
+    mouse_text = f'Mouse: ({mouse_u}, {mouse_v}) | Out of bounds'
+    if 0 <= mouse_u < w and 0 <= mouse_v < h:
+        cv2.circle(annotated, (mouse_u, mouse_v), 5, (255, 255, 0), 1)
+        m_depth = get_depth_at(depth_frame, mouse_u, mouse_v, window_radius=2)
+        if m_depth is None:
+            mouse_text = f'Mouse ({mouse_u}, {mouse_v}): Depth=None (0 mm / invalid)'
+        elif intrinsics is None:
+            mouse_text = f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | Cam=N/A | Base=N/A'
+        else:
+            fx, fy, cx, cy = intrinsics
+            m_xc, m_yc, m_zc = pixel_to_camera_point(mouse_u, mouse_v, m_depth, fx, fy, cx, cy)
+            if extrinsic is None:
+                mouse_text = (
+                    f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
+                    f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | Base=N/A (no extrinsic)'
+                )
+            elif tcp_pose is None:
+                mouse_text = (
+                    f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
+                    f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | Base=N/A (no TCP)'
+                )
+            else:
+                r_e_c, t_e_c = extrinsic
+                m_xb, m_yb, m_zb = camera_to_base_point((m_xc, m_yc, m_zc), r_e_c, t_e_c, tcp_pose)
+                mouse_text = (
+                    f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
+                    f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | '
+                    f'Base=({m_xb:+.3f}, {m_yb:+.3f}, {m_zb:+.3f})m'
+                )
+
+    # Top-left info banner
+    if tcp_pose is not None:
+        tcp_status = (
+            f'TCP: [{tcp_pose[0]:.3f}, {tcp_pose[1]:.3f}, {tcp_pose[2]:.3f}]m '
+            f'p={tcp_pose[3]:.1f}°'
+        )
+    else:
+        tcp_status = 'TCP: N/A'
+    top_banner = (
+        f'Model: {model_name} | {tcp_status} | FPS: {fps:.1f} | Detections: {len(detections)}'
+    )
+    cv2.rectangle(annotated, (0, 0), (w, 30), (20, 20, 20), cv2.FILLED)
+    cv2.putText(
+        annotated, top_banner, (10, 21),
+        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 120), 1, cv2.LINE_AA,
+    )
+
+    # Bottom status bar (mouse probe + hotkey hints)
+    cv2.rectangle(annotated, (0, h - 32), (w, h), (15, 15, 15), cv2.FILLED)
     cv2.putText(
-        annotated,
-        fps_label,
-        (10, 28),
-        cv2.FONT_HERSHEY_SIMPLEX,
-        0.75,
-        (0, 0, 0),
-        4,
-        cv2.LINE_AA,
+        annotated, mouse_text, (10, h - 18),
+        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA,
     )
+    hints = '[s]Save [m]Model [q]Quit'
+    th, _ = cv2.getTextSize(hints, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
     cv2.putText(
-        annotated,
-        fps_label,
-        (10, 28),
-        cv2.FONT_HERSHEY_SIMPLEX,
-        0.75,
-        color,
-        2,
-        cv2.LINE_AA,
+        annotated, hints, (w - th[0] - 10, h - 18),
+        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA,
     )
+
     return annotated
 
 
+class RipenessClassifier:
+    """Two-stage crop ripeness classifier using OpenVINO edge models."""
+
+    CROPS = {'apple', 'pear', 'pumpkin', 'tomato', 'onion', 'pepper'}
+
+    def __init__(self, model_root: Path, core: ov.Core, device: str = 'GPU') -> None:
+        self.model_root = model_root / 'ripeness_edge'
+        self.core = core
+        self.device = device
+        self._requests: dict[str, tuple[Any, Any, Any]] = {}
+
+    def classify(self, rgb_roi: np.ndarray, label: str) -> tuple[str, float] | None:
+        if rgb_roi.shape[0] < 8 or rgb_roi.shape[1] < 8:
+            return None
+        crop = 'pepper' if label.lower() == 'chili' else label.lower()
+        if crop not in self.CROPS:
+            return None
+        if crop not in self._requests:
+            p = self.model_root / crop / 'best.onnx'
+            if not p.is_file():
+                return None
+            try:
+                m = self.core.read_model(str(p))
+                m.reshape([1, 3, 224, 224])
+                compiled = self.core.compile_model(
+                    m, self.device, {'PERFORMANCE_HINT': 'LATENCY'},
+                )
+                self._requests[crop] = (
+                    compiled.create_infer_request(),
+                    compiled.input(0),
+                    compiled.output(0),
+                )
+            except Exception as err:
+                print(
+                    f'Warning: failed to compile ripeness model for {crop}: {err}',
+                    file=sys.stderr,
+                )
+                return None
+
+        req, inp, outp = self._requests[crop]
+        try:
+            resized = cv2.resize(rgb_roi, (224, 224), interpolation=cv2.INTER_LINEAR)
+            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
+            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
+            norm = (resized.astype(np.float32) / 255.0 - mean) / std
+            blob = np.transpose(norm, (2, 0, 1))[np.newaxis, ...]
+            res = req.infer({inp: blob})
+            logits = res[outp][0]
+            exps = np.exp(logits - np.max(logits))
+            probs = exps / np.sum(exps)
+            p_ripe = float(probs[0])
+            p_unripe = float(probs[1])
+            if p_ripe >= p_unripe:
+                return 'ripe', p_ripe
+            return 'unripe', p_unripe
+        except Exception as err:
+            print(f'Warning: ripeness inference failed for {crop}: {err}', file=sys.stderr)
+            return None
+
+
 class Detector:
     def __init__(self, model_path: Path) -> None:
         if not model_path.exists():
@@ -527,10 +1021,7 @@ class Detector:
 
         self._core = ov.Core()
         devices = self._core.available_devices
-        if 'GPU' not in devices:
-            raise RuntimeError(
-                f'OpenVINO GPU device is not available; devices: {devices}',
-            )
+        self._device = 'GPU' if 'GPU' in devices else 'CPU'
 
         model = self._core.read_model(str(model_path))
         if not model.has_rt_info(['framework', 'names']):
@@ -567,7 +1058,7 @@ class Detector:
 
         self._compiled = self._core.compile_model(
             model,
-            'GPU',
+            self._device,
             {'PERFORMANCE_HINT': 'THROUGHPUT'},
         )
         self._input_port = self._compiled.input(0)
@@ -595,6 +1086,14 @@ class Detector:
         ):
             raise RuntimeError(f'Unsupported model output shape {output_shape}')
 
+    @property
+    def core(self) -> ov.Core:
+        return self._core
+
+    @property
+    def device(self) -> str:
+        return self._device
+
     def infer(self, frame_rgb: np.ndarray) -> tuple[np.ndarray, LetterboxTransform]:
         input_blob, transform = letterbox_image(frame_rgb)
         outputs = self._request.infer({self._input_port: input_blob})
@@ -636,7 +1135,10 @@ def run() -> int:
     args = parse_args()
     config = load_config()
     selection = select_camera(config)
-    model_path = model_path_for(args.model, config)
+
+    current_model = args.model.upper()
+    model_path = model_path_for(current_model, config)
+
     perception = config['fruit_perception_node']['ros__parameters']
     confidence_threshold = float(
         perception['confidence_threshold'] if args.conf is None else args.conf,
@@ -652,84 +1154,120 @@ def run() -> int:
             raise ValueError(f'{name} must be between 0 and 1, got {value}')
 
     detector = Detector(model_path)
+    extrinsic = load_extrinsic_transform()
+    model_root = Path(perception['model_root'])
+    ripeness_classifier = RipenessClassifier(model_root, detector.core, detector.device)
+
     rclpy.init()
-    node = FrameSubscriber(selection.topic)
+
+    joint_state_topic = '/arm_planner_node/joint_states'
+    arm_config_path = WORKSPACE / 'src/fruit_arm_planner/config/arm_presets.yaml'
+    try:
+        with EXTRINSIC_CONFIG_PATH.open(encoding='utf-8') as f:
+            cfg = yaml.safe_load(f)
+        if isinstance(cfg, dict):
+            joint_state_topic = str(cfg.get('joint_state_topic', joint_state_topic))
+            if 'arm_config_path' in cfg:
+                arm_config_path = Path(str(cfg['arm_config_path']))
+    except Exception:  # noqa: BLE001
+        pass
+
+    node = FrameSubscriber(
+        selection.color_topic,
+        selection.depth_topic,
+        selection.info_topic,
+        joint_state_topic,
+        arm_config_path,
+    )
+
+    owned_arm_debug: subprocess.Popen | None = None
+    owned_bridge: subprocess.Popen | None = None
+    owned_planner: subprocess.Popen | None = None
     camera_process: subprocess.Popen | None = None
     log_path = TMP_DIR / f'test_camera_detector_{selection.name}.log'
-    if wait_for_publisher(node, selection.topic, 1.0):
-        print(
-            f'Using existing publisher on {selection.topic}; '
-            f'model {model_path}; conf {confidence_threshold:.3f}; '
-            f'iou {nms_iou_threshold:.3f}',
-            flush=True,
-        )
-    else:
-        camera_process, log_path = start_camera(selection)
-        print(
-            f'Started {selection.name}; image topic {selection.topic}; '
-            f'model {model_path}; conf {confidence_threshold:.3f}; '
-            f'iou {nms_iou_threshold:.3f}; log {log_path}',
-            flush=True,
-        )
-    window_name = f'{selection.name} model {args.model}'
-    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
-    cv2.moveWindow(window_name, 20, 60)
+    control_panel = None
+
     try:
-        if not wait_for_publisher(
-            node,
-            selection.topic,
-            10.0,
-        ):
-            check_camera_process(camera_process, log_path)
-            raise RuntimeError(
-                f'Timed out waiting for Camera frames on {selection.topic}',
+        owned_arm_debug, owned_bridge, owned_planner = ensure_arm_communication(node)
+
+        if wait_for_publisher(node, selection.color_topic, 1.0):
+            print(
+                f'Using existing camera stream on {selection.color_topic}; '
+                f'model={model_path.name}; conf={confidence_threshold:.2f}',
+                flush=True,
+            )
+        else:
+            camera_process, log_path = start_camera(selection)
+            print(
+                f'Started RealSense camera; color={selection.color_topic}; '
+                f'depth={selection.depth_topic}; log={log_path}',
+                flush=True,
             )
-        control_service = CameraControlService(node)
-        control_panel = CameraControlPanel(
-            selection.name,
-            control_service.discover(),
-            control_service.set,
-            lambda values: save_camera_controls(
-                selection.name,
-                values,
-                CONFIG_PATH,
-                TMP_DIR,
-            ),
-        )
-    except BaseException:
-        node.destroy_node()
-        if rclpy.ok():
-            rclpy.shutdown()
-        stop_process(camera_process)
-        cv2.destroyAllWindows()
-        raise
 
-    last_frame_time = 0.0
-    displayed_fps = 0.0
-    last_wait_message = 0.0
-    announced_first_frame = False
+        window_name = 'RealSense 3D Detector & Depth Inspector'
+        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
+        cv2.resizeWindow(window_name, 1280, 720)
+        cv2.moveWindow(window_name, 20, 40)
+
+        mouse_coords = [-1, -1]
+
+        def on_mouse(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
+            mouse_coords[0] = x
+            mouse_coords[1] = y
+
+        cv2.setMouseCallback(window_name, on_mouse)
+
+        if args.controls:
+            try:
+                from camera_control_gui import (
+                    CameraControlPanel,
+                    CameraControlService,
+                    save_camera_controls,
+                )
+                control_service = CameraControlService(node)
+                control_panel = CameraControlPanel(
+                    selection.name,
+                    control_service.discover(),
+                    control_service.set,
+                    lambda values: save_camera_controls(
+                        selection.name, values, CONFIG_PATH, TMP_DIR,
+                    ),
+                )
+            except Exception as err:
+                print(f'Warning: failed to initialize control panel: {err}', file=sys.stderr)
+
+        last_frame_time = 0.0
+        displayed_fps = 0.0
+        last_wait_message = 0.0
+        announced_first_frame = False
+
+        print('\n=== RealSense 3D Inspector Controls ===')
+        print('  [Hover Mouse] : Probe depth and 3D camera / arm base coordinates')
+        print('  [Key s]       : Save current annotated snapshot image to disk')
+        print('  [Key m]       : Toggle model between A (vegetables) and B (fruits)')
+        print('  [Key q / ESC] : Quit viewer\n')
 
-    try:
         while rclpy.ok():
             check_camera_process(camera_process, log_path)
-            control_panel.apply_pending()
+            if control_panel is not None:
+                control_panel.apply_pending()
             rclpy.spin_once(node, timeout_sec=0.01)
-            frame = node.take_latest_frame()
+
+            frame, depth_frame = node.take_latest()
             if frame is None:
                 now = time.monotonic()
                 if not announced_first_frame and now - last_wait_message > 2.0:
-                    print(f'Waiting for frames on {selection.topic}...', flush=True)
+                    print(f'Waiting for frames on {selection.color_topic}...', flush=True)
                     last_wait_message = now
-                if (
-                    cv2.waitKey(1) in (27, ord('q'))
-                    or not control_panel.is_open()
-                ):
+                key = cv2.waitKey(1)
+                if key in (27, ord('q'), ord('Q')):
                     break
                 continue
+
             if not announced_first_frame:
                 print(
-                    f'Received first frame from {selection.topic}: '
-                    f'{frame.shape[1]}x{frame.shape[0]}',
+                    f'Received first frame: {frame.shape[1]}x{frame.shape[0]} '
+                    f'(depth available: {depth_frame is not None})',
                     flush=True,
                 )
                 announced_first_frame = True
@@ -753,20 +1291,47 @@ def run() -> int:
                 )
             last_frame_time = now
 
+            h_img, w_img = frame.shape[:2]
+            if depth_frame is not None and depth_frame.shape[:2] != (h_img, w_img):
+                depth_frame = None
+
+            intrinsics = validate_camera_info(node.camera_info, w_img, h_img)
+            tcp_pose = node.get_fresh_tcp_pose()
+
             annotated_rgb = draw_annotations(
                 frame,
+                depth_frame,
                 detections,
                 transform,
+                intrinsics,
+                extrinsic,
+                tcp_pose,
+                mouse_coords[0],
+                mouse_coords[1],
                 displayed_fps,
+                f'{current_model} ({"fruit" if current_model in ("B", "D") else "vegetable"})',
+                ripeness_classifier=ripeness_classifier,
             )
             annotated_bgr = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
             cv2.imshow(window_name, annotated_bgr)
 
             key = cv2.waitKey(1)
-            if key in (27, ord('q')) or not control_panel.is_open():
+            if key in (27, ord('q'), ord('Q')):
                 break
+            elif key in (ord('s'), ord('S')):
+                timestamp = time.strftime('%Y%m%d_%H%M%S')
+                save_path = WORKSPACE / f'snapshot_{timestamp}.jpg'
+                cv2.imwrite(str(save_path), annotated_bgr)
+                print(f'>>> Saved snapshot to {save_path}', flush=True)
+            elif key in (ord('m'), ord('M')):
+                current_model = 'B' if current_model in ('A', 'C') else 'A'
+                model_path = model_path_for(current_model, config)
+                detector = Detector(model_path)
+                print(f'>>> Switched to Model {current_model}: {model_path.name}', flush=True)
+
     finally:
-        control_panel.close()
+        if control_panel is not None:
+            control_panel.close()
         try:
             node.destroy_node()
         except Exception:  # noqa: BLE001
@@ -776,6 +1341,12 @@ def run() -> int:
                 rclpy.shutdown()
         except Exception:  # noqa: BLE001
             pass
+        if owned_planner is not None:
+            stop_process(owned_planner)
+        if owned_bridge is not None:
+            stop_process(owned_bridge)
+        if owned_arm_debug is not None:
+            stop_process(owned_arm_debug)
         stop_process(camera_process)
         cv2.destroyAllWindows()
 
diff --git a/waypoints_maker.sh b/waypoints_maker.sh
index 8f8cc18..ad4ba0c 100755
--- a/waypoints_maker.sh
+++ b/waypoints_maker.sh
@@ -144,8 +144,14 @@ if [[ ! -r "$DESKTOP_XAUTHORITY" ]]; then
   exit 1
 fi
 
-start_process "serial bridge" \
-  ros2 launch "$HARDWARE_LAUNCH"
+START_SERIAL_BRIDGE=$(ros2 run fruit_mission stack_readiness_client bridge-start-required \
+  --ros-args --params-file "$SCRIPT_CLIENT_CONFIG")
+if [[ "$START_SERIAL_BRIDGE" == true ]]; then
+  start_process "serial bridge" \
+    ros2 launch "$HARDWARE_LAUNCH"
+else
+  printf '[waypoints-maker] reusing existing serial bridge; it will not be stopped on exit\n'
+fi
 
 if [[ "$USE_TELEOP" == true ]]; then
   nav_process_name="axis navigation, odometry, and teleop"

启动前 status：
 M AGENTS.md
 M README.md
 M docs/CAMERA_TUNING.md
 M docs/PERCEPTION_MODE_COMMANDS.md
 M docs/PROJECT_ARCHITECTURE.md
 M docs/README.md
 M docs/RULE.md
 M docs/VOICE_FLOW.md
 M kill_fruit_processes.sh
 M record_rosbag.sh
 M scripts/robot_debug_monitor.sh
 M src/fruit_arm_interfaces/CMakeLists.txt
 M src/fruit_arm_interfaces/package.xml
 M src/fruit_arm_planner/README.md
 M src/fruit_arm_planner/config/arm_presets.yaml
 M src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md
 M src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md
 M src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md
 M src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md
 M src/fruit_arm_planner/docs/05_PHYSICAL_ARM_MAPPING_REVIEW.md
 M src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py
 M src/fruit_arm_planner/fruit_arm_planner/joint_transformer.py
 M src/fruit_arm_planner/fruit_arm_planner/kinematics.py
 M src/fruit_arm_planner/test/cartesian_console.py
 M src/fruit_mission/README.md
 M src/fruit_mission/config/camera_extrinsic.yaml
 M src/fruit_mission/config/fruit_mission.yaml
 M src/fruit_mission/config/script_ros_clients.yaml
 M src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
 M src/fruit_mission/fruit_mission/Logic.py
 M src/fruit_mission/fruit_mission/actions.py
 M src/fruit_mission/fruit_mission/camera_arm_target_client.py
 M src/fruit_mission/fruit_mission/camera_target_planner_client.py
 M src/fruit_mission/fruit_mission/camera_target_transform.py
 M src/fruit_mission/fruit_mission/config.py
 M src/fruit_mission/fruit_mission/decisions.py
 M src/fruit_mission/fruit_mission/mission_context.py
 M src/fruit_mission/fruit_mission/mission_node.py
 M src/fruit_mission/fruit_mission/stack_readiness_client.py
 M src/fruit_mission/fruit_mission/tts.py
 M src/fruit_perception/CMakeLists.txt
 M src/fruit_perception/MODEL_UPDATE_RC4.md
 M src/fruit_perception/README.md
 M src/fruit_perception/config/fruit_perception.yaml
 M src/fruit_perception/docs/TWO_STAGE_RUNTIME.md
 D src/fruit_perception/include/fruit_perception/perception_mode.hpp
 M src/fruit_perception/launch/fruit_perception.launch.py
 M src/fruit_perception/msg/FruitDecision.msg
 M src/fruit_perception/package.xml
 D src/fruit_perception/src/camera_mode_manager.cpp
 M src/fruit_perception/src/fruit_perception_node.cpp
 D src/fruit_perception/src/perception_mode.cpp
 D src/fruit_perception/test/test_perception_mode.cpp
 M src/fruit_qr_planner/README.md
 M src/fruit_qr_planner/config/fruit_qr_planner.yaml
 M src/fruit_qr_planner/fruit_qr_planner/qr_node.py
 M src/joy_linux/README.md
 M src/joy_linux/launch/joy_cmd_vel_serial.launch.py
 M src/navigation/fruit_navigation/fruit_navigation/axis_waypoint_action_server.py
 M start_fruit_mission.sh
 M start_fruit_perception.sh
 M start_navigation.sh
 M start_tele.sh
 M test_camera_arm_target.sh
 M test_camera_detector.py
 M waypoints_maker.sh
?? .agents/skills/wisadel-helium-voice/SKILL.md
?? .pi/skills/wisadel-helium-voice/SKILL.md
?? ce1adf5a259fbc6f8ffcf4c76b95ef9c.png
?? scripts/fruit_runtime_logging.bash
?? scripts/generate_wisadel_voice.py
?? scripts/prepare_wisadel_voice.py
?? scripts/runtime_log_filter.py
?? scripts/simulate_competition_voice.py
?? sounds/fruit_success.wav
?? src/fruit_arm_interfaces/msg/ArmTelemetry.msg
?? src/fruit_mission/config/wisadel_voice.yaml
?? src/fruit_mission/sounds/wisadel/act_harvest_done.wav
?? src/fruit_mission/sounds/wisadel/act_recognize.wav
?? src/fruit_mission/sounds/wisadel/arm_err.wav
?? src/fruit_mission/sounds/wisadel/arm_err_continue.wav
?? src/fruit_mission/sounds/wisadel/arm_err_skip_remain.wav
?? src/fruit_mission/sounds/wisadel/arm_fail.wav
?? src/fruit_mission/sounds/wisadel/arm_fail_continue.wav
?? src/fruit_mission/sounds/wisadel/arm_fail_skip_remain.wav
?? src/fruit_mission/sounds/wisadel/arm_not_ready.wav
?? src/fruit_mission/sounds/wisadel/arm_not_ready_continue.wav
?? src/fruit_mission/sounds/wisadel/arm_not_ready_skip_remain.wav
?? src/fruit_mission/sounds/wisadel/arm_timeout.wav
?? src/fruit_mission/sounds/wisadel/arm_timeout_continue.wav
?? src/fruit_mission/sounds/wisadel/arm_timeout_skip_remain.wav
?? src/fruit_mission/sounds/wisadel/arm_turn_timeout.wav
?? src/fruit_mission/sounds/wisadel/arm_turn_timeout_skip.wav
?? src/fruit_mission/sounds/wisadel/b_qr_1_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_1_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_2_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_2_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_3_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_3_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_4_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_4_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_5_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_5_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_6_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_6_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_7_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_7_pear.wav
?? src/fruit_mission/sounds/wisadel/b_qr_8_apple.wav
?? src/fruit_mission/sounds/wisadel/b_qr_8_pear.wav
?? src/fruit_mission/sounds/wisadel/b_tree_1.wav
?? src/fruit_mission/sounds/wisadel/b_tree_2.wav
?? src/fruit_mission/sounds/wisadel/b_tree_3.wav
?? src/fruit_mission/sounds/wisadel/b_tree_4.wav
?? src/fruit_mission/sounds/wisadel/b_tree_5.wav
?? src/fruit_mission/sounds/wisadel/b_tree_6.wav
?? src/fruit_mission/sounds/wisadel/b_tree_7.wav
?? src/fruit_mission/sounds/wisadel/b_tree_8.wav
?? src/fruit_mission/sounds/wisadel/c_pos_1.wav
?? src/fruit_mission/sounds/wisadel/c_pos_10.wav
?? src/fruit_mission/sounds/wisadel/c_pos_11.wav
?? src/fruit_mission/sounds/wisadel/c_pos_12.wav
?? src/fruit_mission/sounds/wisadel/c_pos_2.wav
?? src/fruit_mission/sounds/wisadel/c_pos_3.wav
?? src/fruit_mission/sounds/wisadel/c_pos_4.wav
?? src/fruit_mission/sounds/wisadel/c_pos_5.wav
?? src/fruit_mission/sounds/wisadel/c_pos_6.wav
?? src/fruit_mission/sounds/wisadel/c_pos_7.wav
?? src/fruit_mission/sounds/wisadel/c_pos_8.wav
?? src/fruit_mission/sounds/wisadel/c_pos_9.wav
?? src/fruit_mission/sounds/wisadel/c_seq_1.wav
?? src/fruit_mission/sounds/wisadel/c_seq_2.wav
?? src/fruit_mission/sounds/wisadel/c_seq_3.wav
?? src/fruit_mission/sounds/wisadel/c_seq_4.wav
?? src/fruit_mission/sounds/wisadel/c_seq_5.wav
?? src/fruit_mission/sounds/wisadel/c_seq_6.wav
?? src/fruit_mission/sounds/wisadel/c_seq_7.wav
?? src/fruit_mission/sounds/wisadel/c_seq_8.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_1.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_10.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_11.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_12.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_2.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_3.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_4.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_5.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_6.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_7.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_8.wav
?? src/fruit_mission/sounds/wisadel/c_zone_pos_9.wav
?? src/fruit_mission/sounds/wisadel/cam_no_target.wav
?? src/fruit_mission/sounds/wisadel/cam_switch_fail.wav
?? src/fruit_mission/sounds/wisadel/cam_switch_fail_skip.wav
?? src/fruit_mission/sounds/wisadel/continue_next.wav
?? src/fruit_mission/sounds/wisadel/count_1.wav
?? src/fruit_mission/sounds/wisadel/count_10.wav
?? src/fruit_mission/sounds/wisadel/count_11.wav
?? src/fruit_mission/sounds/wisadel/count_12.wav
?? src/fruit_mission/sounds/wisadel/count_13.wav
?? src/fruit_mission/sounds/wisadel/count_14.wav
?? src/fruit_mission/sounds/wisadel/count_15.wav
?? src/fruit_mission/sounds/wisadel/count_16.wav
?? src/fruit_mission/sounds/wisadel/count_17.wav
?? src/fruit_mission/sounds/wisadel/count_18.wav
?? src/fruit_mission/sounds/wisadel/count_19.wav
?? src/fruit_mission/sounds/wisadel/count_2.wav
?? src/fruit_mission/sounds/wisadel/count_20.wav
?? src/fruit_mission/sounds/wisadel/count_21.wav
?? src/fruit_mission/sounds/wisadel/count_22.wav
?? src/fruit_mission/sounds/wisadel/count_23.wav
?? src/fruit_mission/sounds/wisadel/count_24.wav
?? src/fruit_mission/sounds/wisadel/count_25.wav
?? src/fruit_mission/sounds/wisadel/count_26.wav
?? src/fruit_mission/sounds/wisadel/count_27.wav
?? src/fruit_mission/sounds/wisadel/count_28.wav
?? src/fruit_mission/sounds/wisadel/count_29.wav
?? src/fruit_mission/sounds/wisadel/count_3.wav
?? src/fruit_mission/sounds/wisadel/count_30.wav
?? src/fruit_mission/sounds/wisadel/count_31.wav
?? src/fruit_mission/sounds/wisadel/count_32.wav
?? src/fruit_mission/sounds/wisadel/count_4.wav
?? src/fruit_mission/sounds/wisadel/count_5.wav
?? src/fruit_mission/sounds/wisadel/count_6.wav
?? src/fruit_mission/sounds/wisadel/count_7.wav
?? src/fruit_mission/sounds/wisadel/count_8.wav
?? src/fruit_mission/sounds/wisadel/count_9.wav
?? src/fruit_mission/sounds/wisadel/detect_fail_skip.wav
?? src/fruit_mission/sounds/wisadel/detect_timeout_skip.wav
?? src/fruit_mission/sounds/wisadel/expect_apple.wav
?? src/fruit_mission/sounds/wisadel/expect_chili.wav
?? src/fruit_mission/sounds/wisadel/expect_onion.wav
?? src/fruit_mission/sounds/wisadel/expect_pear.wav
?? src/fruit_mission/sounds/wisadel/expect_pumpkin.wav
?? src/fruit_mission/sounds/wisadel/expect_tomato.wav
?? src/fruit_mission/sounds/wisadel/flow_finish.wav
?? src/fruit_mission/sounds/wisadel/invalid_id_skip.wav
?? src/fruit_mission/sounds/wisadel/just_skip.wav
?? src/fruit_mission/sounds/wisadel/nav_fail_skip.wav
?? src/fruit_mission/sounds/wisadel/nav_plan_fail_skip.wav
?? src/fruit_mission/sounds/wisadel/other_target.wav
?? src/fruit_mission/sounds/wisadel/pick_fail.wav
?? src/fruit_mission/sounds/wisadel/produce_apple.wav
?? src/fruit_mission/sounds/wisadel/produce_chili.wav
?? src/fruit_mission/sounds/wisadel/produce_onion.wav
?? src/fruit_mission/sounds/wisadel/produce_pear.wav
?? src/fruit_mission/sounds/wisadel/produce_pumpkin.wav
?? src/fruit_mission/sounds/wisadel/produce_tomato.wav
?? src/fruit_mission/sounds/wisadel/qr_match_no_ripe_apple.wav
?? src/fruit_mission/sounds/wisadel/qr_match_no_ripe_pear.wav
?? src/fruit_mission/sounds/wisadel/qr_match_start_apple.wav
?? src/fruit_mission/sounds/wisadel/qr_match_start_pear.wav
?? src/fruit_mission/sounds/wisadel/qr_mismatch_skip.wav
?? src/fruit_mission/sounds/wisadel/ripe_apple.wav
?? src/fruit_mission/sounds/wisadel/ripe_chili.wav
?? src/fruit_mission/sounds/wisadel/ripe_onion.wav
?? src/fruit_mission/sounds/wisadel/ripe_pear.wav
?? src/fruit_mission/sounds/wisadel/ripe_pumpkin.wav
?? src/fruit_mission/sounds/wisadel/ripe_tomato.wav
?? src/fruit_mission/sounds/wisadel/skip_inconsistent.wav
?? src/fruit_mission/sounds/wisadel/skip_no_fruit.wav
?? src/fruit_mission/sounds/wisadel/skip_no_ripe_apple.wav
?? src/fruit_mission/sounds/wisadel/skip_no_ripe_chili.wav
?? src/fruit_mission/sounds/wisadel/skip_no_ripe_onion.wav
?? src/fruit_mission/sounds/wisadel/skip_no_ripe_pear.wav
?? src/fruit_mission/sounds/wisadel/skip_no_ripe_pumpkin.wav
?? src/fruit_mission/sounds/wisadel/skip_no_ripe_tomato.wav
?? src/fruit_mission/sounds/wisadel/skip_no_target.wav
?? src/fruit_mission/sounds/wisadel/skip_remain.wav
?? src/fruit_mission/sounds/wisadel/start.wav
?? src/fruit_mission/sounds/wisadel/summary_intro.wav
?? src/fruit_mission/sounds/wisadel/summary_none.wav
?? src/fruit_mission/sounds/wisadel/unknown_fruit.wav
?? src/fruit_mission/sounds/wisadel/unknown_target.wav
?? src/fruit_mission/sounds/wisadel/unripe_apple.wav
?? src/fruit_mission/sounds/wisadel/unripe_chili.wav
?? src/fruit_mission/sounds/wisadel/unripe_onion.wav
?? src/fruit_mission/sounds/wisadel/unripe_pear.wav
?? src/fruit_mission/sounds/wisadel/unripe_pumpkin.wav
?? src/fruit_mission/sounds/wisadel/unripe_tomato.wav

