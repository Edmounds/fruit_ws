# 启动基线（只读，不是修复计划）

启动前 Git diff（既有改动，不得擅自归因）：

diff --git a/AGENTS.md b/AGENTS.md
index a400122..58f5c0f 100644
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
diff --git a/README.md b/README.md
index 08d2780..caba272 100644
--- a/README.md
+++ b/README.md
@@ -82,21 +82,27 @@ sudo systemctl stop fruit-perception.service
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
+- `--left|--right`：机械臂进入 `left_look` 或 `right_look` 观察预设位姿。
+- `--debug`：打印逐帧相机目标检测与三维坐标详情。
+
+流程说明：
+脚本等待实际感知节点的区域订阅端就绪后发布模型选择，并等待可靠 DDS 确认，避免串口桥先就绪时丢失 B 模型选择。机械臂从当前实测姿态运动至 `left_look` 或 `right_look`，静止后通过 planner 的实测关节角请求张爪（OPEN + HOLD 驻留），完成后开启 `DETECT`；利用深度信息锁定目标三维坐标并换算至基座坐标系，运动至目标点闭爪（CLOSE + HOLD）后原地停止，不执行 Harvest。B 只从感知完成统计后的有效成熟目标中选择置信度最高者，不区分苹果或梨；无成熟目标则不下发目标运动，不回退抓未成熟目标。品种参数已删除。每项操作等待独立计时，最大 10 秒，原有更短等待不变。按 `Ctrl-C` 退出。
 
 ## 上位机机械臂调试
 
diff --git a/docs/PROJECT_ARCHITECTURE.md b/docs/PROJECT_ARCHITECTURE.md
index c949d2b..76ff665 100644
--- a/docs/PROJECT_ARCHITECTURE.md
+++ b/docs/PROJECT_ARCHITECTURE.md
@@ -199,7 +199,9 @@ RGBD 复合消息和 `compressed`/`compressedDepth`/`theora` 派生图像进入
 正式 mission 的 `arm_presets` 映射为 `left_look/right_look/qr_look/left_harvest/right_harvest/home`。
 真实轨迹由用户配置，本次不修改 planner 文件或填写假姿态。Look 末点 OPEN，Harvest 末点 OPEN/驻留，Home 保持；视觉请求末点 CLOSE。Cartesian 服务没有驻留字段，随后复用 `move_joints` 在实测原始四轴角执行 HOLD，耗时读取 planner `default_dwell_ms`；不自行生成轨迹或用 sleep 宣告动作完成。
 
-识别前实测静止初值：底盘线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s，各关节速度绝对值 ≤0.05 rad/s，持续 0.5 秒。底盘 `/odometry` 源时间戳/接收年龄 ≤0.5 秒，机械臂沿用 planner 新鲜度并检查原始 ArmStatus，避免 NaN 被 JointState 输出替成零。阈值在 mission 配置可调。各步骤独立等待，不设整个工序共享 10 秒预算。
+识别前实测静止初值：底盘线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s，各关节速度绝对值 ≤0.05 rad/s，持续 0.5 秒。底盘 `/odometry` 源时间戳/接收年龄 ≤0.5 秒，机械臂沿用 planner 新鲜度并检查原始 ArmStatus 的有限速度；遥测也保留非有限速度，不替成零。阈值在 mission 配置可调。各步骤独立等待，不设整个工序共享 10 秒预算。
+
+现有 `/arm_planner_node/joint_states` 使用 `fruit_arm_interfaces/msg/ArmTelemetry`：同一帧包含原始硬件角、已校正的模型角及真实 TCP XYZ/pitch。控制台和 mission 直接读取模型/TCP，不重复逆映射或 FK；原始硬件角仍用于原始角服务和安全检查。话题名不变但类型与标准 JointState 不兼容，字段单位和迁移要求见 [planner 接口文档](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md)。
 
 字段类型与线协议布局见 [协议参考](LOWER_CONTROLLER_COMMANDS.md) 和
 [planner 通信文档](../src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md)。新协议没有旧 `z_type` 高度索引。
@@ -311,7 +313,7 @@ USE_SAVED_B_PLAN
 - B 区 camera 固定采集 50 帧，按 3D 轨迹输出最多两个稳定目标；原始类型、检测框和标注图保留误识别证据。二维码期望类型不传入感知节点，mission 做品种匹配、成熟筛选。
 - 成熟且匹配目标按既有原像素 `score = abs(pixel_x - image_width / 2) - pixel_y` 排序，同分保留原顺序，最多取两个。首次观察姿态下将两个目标一次性转换为基座坐标/绝对 pitch，后续直接用冻结请求，不重新观察/换算。每抓一次都完成 Harvest 松爪和 Home。
 - 第一抓失败但一次安全 Home 成功后仍抓第二个；恢复不安全则跳过剩余目标，不重试抓取。
-- `test_camera_arm_target.sh --zone B --left|--right --apple|--pear` 复用同一流程；需要 `/odometry` 反馈和真实预设。完成窗口为空正常跳过，未等到窗口才算识别超时。脚本会驱动硬件，不是自动验证工具。
+- `test_camera_arm_target.sh --model A|B --left|--right [--debug]` 专用于相机与机械臂单步联调：先等待实际感知区域订阅端并确认可靠 DDS 交付，机械臂进入指定 look 姿态后以实测关节角张爪并驻留，再开启识别。B 不区分品种，只从感知完成统计后的有效成熟目标中选最高置信度者；不回退抓未成熟目标。深度锁定目标后闭爪抓取并停留在目标点（不执行 Harvest）。各操作独立等待且不超过 10 秒。脚本会驱动硬件，不是自动验证工具。
 
 ### 7.4 语音时序
 
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
index 2a22ec1..7afceb0 100644
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
@@ -88,3 +88,14 @@ arm_planner_node:
           - { target: [0.40, 0.0, 0.40, 0.0], duration_ms: 1000, gripper: 0, dwell_ms: 0 }
           # 段 3: 带着果实收回到安全悬停高度，耗时 1500ms，保持夹紧
           - { target: [0.20, 0.0, 0.25, 0.0], duration_ms: 1500, gripper: 0, dwell_ms: 0 }
+      home:
+        description: "返回运动初始位置"
+        coordinate_frame: cartesian
+        waypoints:
+        - target:
+          - 0.22002881667529685
+          - -0.00012017945379207707
+          - 0.16796761109539254
+          - -1.9758004323461622
+          duration_ms: 1000
+          gripper: 0
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
index 371125e..2200e10 100644
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
diff --git a/src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md b/src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md
index 16b06f2..0f77d73 100644
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
@@ -166,7 +166,7 @@ presets:
   - `message` (`string`): 执行状态描述或报错原因
   - `trajectory_id` (`uint32`): 任务流水号
 - **执行结果**：节点从发布启动 Mode 起，以轨迹末点时间加配置的 `execution_timeout_buffer_sec` 等待本次轨迹 ID 的新状态。收到 `COMPLETED` 才返回成功；`FAILED`、`EMERGENCY`、主动停止或超时返回失败。超时会发布 STOP；发布成功不代表硬件已停稳。
-- **目标偏置**：服务收到的 `(x, y, z)` 在 PTP 求解或 LIN 路径规划前，逐轴加上 `arm_presets.yaml` 的 `planning.cartesian_offset_m`（单位 m）。例如偏置 `[0, 0, -0.03]` 时，输入 `(0, 0, 0)` 会以 `(0, 0, -0.03)` 求解；是否可达仍由 IK 和限位检查决定。LIN 起点使用实测关节角的正解坐标，不加偏置；`pitch_deg` 不变。`cmd_cartesian` 复用本服务回调，因此同样生效；`ExecuteArmPreset` 的笛卡尔航点及关节控制不受影响。
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
index 42daf11..4b6fa52 100644
--- a/src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md
+++ b/src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md
@@ -72,6 +72,7 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 | `joints Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]` | 连杆目标角度（度）：Q2 相对竖直（后仰为正），Q3 为小臂相对水平面的绝对角，Q4 为末端相对水平面的绝对俯仰角。控制台按 YAML 偏置换算后下发；例如 `joints 0 20 -40 0 2500 hold` |
 | `motors Q1 Q2 Q3 Q4 [DURATION_MS] [hold|open|close]` | 直接输入四个原始电机硬件角（度），不再叠加偏置；沿用旧版 `joints` 的语义 |
 | `preset NAME` | 执行源码 YAML 中存在的预设，例如 `preset zero`；当前名称为 `zero`、`left_look`、`right_look`、`pick_sequence`，现场可调 |
+| `status` | 显示同一帧原始电机角、逆映射模型角、连杆角，以及真实 TCP 的 XYZ（m）和绝对 pitch（度） |
 | `jog` | 进入终端内键盘微调和示教打点界面，按键直接执行运动 |
 | `reload` | 调用 `/arm_planner_node/reload_presets`，重新加载源码 YAML 的全部 presets |
 
@@ -93,7 +94,7 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 | Esc | 清除待执行目标并退出；若仍有运动则请求 STOP |
 | Ctrl+C | 请求急停并退出控制台 |
 
-进入时以最新四轴实测角的 FK 作为初始 XYZ 和俯仰角。空闲时按键自动提交运动；运动期间继续累加目标，完成后自动执行最新目标，**不逐键排队、不打断当前轨迹**。每次提交使用运行中 Planner 的 `default_duration_ms`，位置/俯仰运动的夹爪动作为保持。界面区分实测位置、累计目标、执行目标、待执行夹爪动作及当前点记录动作；不可达、越限、执行失败或停止后清除所有待执行动作、不自动续发，须检查原因并重新进入模式。
+进入时使用 planner 最新遥测的真实 TCP 作为初始 XYZ 和俯仰角，不在控制台再次逆 mapping 或 FK。空闲时按键自动提交运动；运动期间继续累加目标，完成后自动执行最新目标，**不逐键排队、不打断当前轨迹**。每次提交使用运行中 Planner 的 `default_duration_ms`，位置/俯仰运动的夹爪动作为保持。界面区分实测位置、累计目标、执行目标、待执行夹爪动作及当前点记录动作；不可达、越限、执行失败或停止后清除所有待执行动作、不自动续发，须检查原因并重新进入模式。
 
 `c/o` 复用 `move_joints` 服务，以最新有效四轴实测硬件角（转为度）作为目标，只改变夹爪动作，不再 IK 或叠加 Cartesian offset。服务起点仍由 Planner 获取实测角，反馈与执行有时间差，不应视为硬件锁轴。夹爪动作在默认时长轨迹的末点触发，并非按键瞬间完成。运动中按 `c/o` 会合并为最新待执行夹爪动作；当前动作结束后先原地执行夹爪动作，再继续累计移动。`h` 取消尚未下发的夹爪动作，已提交的请求不受它撤销。
 
@@ -101,7 +102,7 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
 
 移动键和夹爪键不会写文件。空格只在全部累计运动和夹爪请求完成后可用，冻结当前有效实测位置和姿态，再输入 preset 名称：已有 preset 按其 `cartesian` / `joints` / `link_angles` 类型转换后追加，不存在则新建 Cartesian preset。航点时间读取源码 YAML 的 `planning.default_duration_ms`，`gripper` 保存当前点的动作标签；不额外写入 `dwell_ms`，闭合/张开航点回放沿用 `planning.default_dwell_ms`。原有配置和注释不整体重写，文件写入失败不覆盖原文件。保存后自动请求热重载；若重载失败，文件仍已保存，处理错误后执行 `reload`。
 
-点动显示及记录的是实际模型坐标。提交笛卡尔服务前先减去 `cartesian_offset_m`，抵消 Planner 的偏置叠加；Cartesian preset 直接保存 FK 坐标，不再加偏置。终端弹窗期间不执行运动键；Esc 取消和保存均不会执行该 preset。
+点动显示及记录的是统一 FK 模型的真实夹爪抓取点 TCP 坐标；提交服务和保存 Cartesian preset 使用同一坐标，不再加减输入偏置。终端弹窗期间不执行运动键；Esc 取消和保存均不会执行该 preset。
 
 `reload` 只刷新全部 presets，运动期间或校验失败时保留旧内存内容，不修改几何、关节映射及 planning。调整默认时长、偏置或机械参数后重启 Planner 和控制台，保证双方使用同一份配置；升级本次 Python 代码也需构建并重启 Planner。点动时长是计划运动时间，不含通信及启动等待，不代表固定按键执行频率。
 
@@ -111,8 +112,11 @@ python3 /home/rc4/fruit_ws/src/fruit_arm_planner/test/cartesian_console.py
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
@@ -153,9 +157,38 @@ geometry:
   d1: 0.062      # 底座安装面至肩轴中心距
   a2: 0.26       # 肩轴至肘轴距
   a3: 0.26       # 肘轴至腕轴距
-  a4: 0.12       # 腕轴至夹爪采摘作用点中心距
+  a4: 0.12       # 腕轴沿末端局部 X 方向的伸出长度，pitch=0 时此段水平
+  tool_z_offset_m: -0.03  # 真实抓取点的末端局部 Z 偏移，下方为负，随腕部旋转
+```
+
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
 ```
 
+当前 12 cm / -3 cm 几何下，此水平目标的 TCP 为 `(0.25,0,0.25)m`，腕轴为 `(0.13,0,0.28)m`。这是数学检查，不验证实机标定或关节安全限位。
+
 ### 2. 实机零位偏置与方向校准 (`joint_mapping`)
 - **若某关节旋转方向与预期相反（例如给正角反而向下运动）**：
   将该关节的 `direction` 由 `1.0` 修改为 `-1.0` 即可反相，无需拆改电机连线。
@@ -174,7 +207,6 @@ geometry:
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
index 31eb937..ddb22fd 100644
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
@@ -494,15 +502,12 @@ class ArmPlannerNode(Node):
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
index 47cf048..5384bcf 100644
--- a/src/fruit_arm_planner/test/cartesian_console.py
+++ b/src/fruit_arm_planner/test/cartesian_console.py
@@ -23,9 +23,9 @@ from rclpy.signals import SignalHandlerOptions
 from std_srvs.srv import Trigger
 
 from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
+from fruit_arm_interfaces.msg import ArmTelemetry
 from fruit_arm_planner.arm_planner_node import validate_presets
 from fruit_arm_planner.joint_transformer import JointTransformer
-from fruit_arm_planner.kinematics import RobotKinematics
 from fruit_hardware_interfaces.msg import ArmControlMode, ArmStatus, TrajectoryHeader, TrajectoryPoint
 
 
@@ -51,7 +51,7 @@ HELP = (
     'preset NAME   Execute a configured preset (zero, home, retract, pick_sequence)\n'
     'jog       Keyboard jog/teach: arrows=XY, a/z=Z, s/x=pitch, c/o/h=gripper, Space=save\n'
     'reload    Reload all presets in the running planner (only when not moving)\n'
-    'status    Show the latest controller status and measured joint angles\n'
+    'status    Show raw motor angles, mapped model angles and real TCP xyz/pitch\n'
     'stop      Request a controlled stop\n'
     'estop     Request an emergency stop\n'
     'level [info|debug|trace]  Show or change console log detail\n'
@@ -74,7 +74,6 @@ class ArmConsole(Node):
         self.log_level = log_level
         self.config_path = config_path
         self.transformer = JointTransformer.from_config_dict(params['joint_mapping'])
-        self.kinematics = RobotKinematics.from_geometry_dict(params['geometry'])
         self.planning = params['planning']
         self._output_lock = threading.Lock()
         self._terminal_ui = False
@@ -85,7 +84,8 @@ class ArmConsole(Node):
         self.stop_client = self.create_client(Trigger, '/arm_planner_node/stop')
         self.estop_client = self.create_client(Trigger, '/arm_planner_node/emergency_stop')
         self.reload_client = self.create_client(Trigger, '/arm_planner_node/reload_presets')
-        self.create_subscription(ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
+        self.create_subscription(
+            ArmTelemetry, '/arm_planner_node/joint_states', self._on_status, 10)
         tx_qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE)
         self.create_subscription(
             TrajectoryHeader, '/fruit_hardware_bridge/arm/trajectory_header',
@@ -127,14 +127,18 @@ class ArmConsole(Node):
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
 
@@ -172,18 +176,25 @@ class ArmConsole(Node):
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
@@ -327,15 +338,24 @@ class ArmConsole(Node):
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
@@ -501,8 +521,7 @@ class ArmConsole(Node):
                                                  f'In-place gripper {gripper_names[active_gripper]}')
                 else:
                     request = MoveArmCartesian.Request()
-                    request.x, request.y, request.z = [
-                        value - delta for value, delta in zip(target[:3], self.planning['cartesian_offset_m'])]
+                    request.x, request.y, request.z = target[:3]
                     request.pitch_deg = target[3]
                     request.duration_ms = 0  # The running planner's configured default.
                     request.gripper_action = 0
diff --git a/src/fruit_mission/config/camera_extrinsic.yaml b/src/fruit_mission/config/camera_extrinsic.yaml
index a2aeb02..ad9e6e6 100644
--- a/src/fruit_mission/config/camera_extrinsic.yaml
+++ b/src/fruit_mission/config/camera_extrinsic.yaml
@@ -24,6 +24,6 @@ service_timeout_sec: 10.0
 
 # mission 在目标末点闭爪，后续 Harvest/Home 由上位机调用预设。
 motion:
-  pitch_deg: null  # null: 保持图像时刻的末端俯仰角；数值: 指定对地俯仰角（度）
-  duration_ms: 0  # 0: 使用 planner 的默认耗时
-  linear_path: false
+  pitch_deg: 0.0  # 指定对地绝对俯仰角（度），0.0 为相对于 base 严格水平抓取
+  duration_ms: 1500  # 1500ms 平滑探出抓取
+  linear_path: false  # false: 关节空间 PTP 平滑规划
diff --git a/src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md b/src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
index 6336005..4ea019e 100644
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
+用户确认 DETECT 时机械臂静止。**整个多帧观测窗口及观测结束到提交请求之间必须维持同一观测姿态**，才可用提交前的新鲜实测 TCP 遥测转换目标；不缓存/插值历史姿态。若以后改为运动中检测，需要重新设计同步，不能继续沿用此假设。
 
-`joint_states` 来自下位机反馈，经 planner 发布，单位 rad；`joint_3` 是小臂绝对角，不是相对肘角。先通过 `hardware_to_algo` 消除 direction/offset，再做 FK。FK 输出原点为夹爪作用中心，俯仰为 `q2+q3+q4`，偏航为 `q1`。
+`/arm_planner_node/joint_states` 的消息类型现为 `fruit_arm_interfaces/msg/ArmTelemetry`，话题名不变。`hardware_joint_state` 保留原始硬件角，`model_joint_state` 已消除 direction/offset 并还原相对肘角，单位 rad。mission 只用同一帧模型 Q1 和 `tcp_position/tcp_pitch_deg`（m、度）做 E -> B，不再自行逆 mapping 或 FK；输入校验仍要求原始角在硬件限位内，模型角/TCP 有限且反馈新鲜。原始 ArmStatus 订阅保留给停稳、安全恢复和硬件角驻留服务，不把其角度误当模型角。
 
 ## 新外参评估（单位 mm）
 
@@ -64,7 +64,7 @@ t_E_C_mm = [-65.268106, 32.458360, -54.110679]
 
 ```text
 R_B_E = Rz(yaw) * Ry(-pitch)
-P_B_mm = R_B_E * P_E_mm + 1000 * FK_position_m
+P_B_mm = R_B_E * P_E_mm + 1000 * planner_TCP_position_m
 ```
 
 所以相机到夹爪的固定矩阵本身并不足以提交 planner；必须加上观测姿态的第二步。
@@ -75,25 +75,27 @@ P_B_mm = R_B_E * P_E_mm + 1000 * FK_position_m
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
+外参或 planner geometry 修改需同时重启 planner、mission/相机调试客户端及控制台，不会自动热重载；geometry 必须通过共享解算器的尺寸/有限数校验，局部 Z 偏移允许为负。
 
 `service_timeout_sec` 默认 10 秒，每次运动单独计时；端点等待也不超过 10 秒，不设整个抓取工序共享预算。异步 Look 等待也有计时器保护。超时/请求异常取消 future 并请求 STOP，仍返回失败；STOP 服务成功仅表示已发布停止命令，不证明硬件停稳。恢复前须获得 STOP 请求之后新收到的无错误 IDLE、有限且在限位内的四轴低速反馈，再执行一次 Home；不自动清除 EMERGENCY。不得把 publish 或 future 完成当作运动成功，必须检查 `response.success`。
 
 ## 观察与 A/B/C 编排
 
-- 导航与相应 Look 并行；导航成功、Look 成功后，底盘/机械臂必须连续静止 0.5 秒才启 DETECT/QR。初值：线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s、各轴速度绝对值 ≤0.05 rad/s，均在 mission YAML 可调。底盘 `/odometry` 时间戳和本机接收年龄 ≤0.5 秒；机械臂沿用 planner 新鲜度。原始 Point-LIO 输出不提供可用速度，不作静止依据；不能使用 planner 可能替换 NaN 的 JointState 速度，需原始 ArmStatus。
+- 导航与相应 Look 并行；导航成功、Look 成功后，底盘/机械臂必须连续静止 0.5 秒才启 DETECT/QR。初值：线速度 ≤0.02 m/s、角速度 ≤0.05 rad/s、各轴速度绝对值 ≤0.05 rad/s，均在 mission YAML 可调。底盘 `/odometry` 时间戳和本机接收年龄 ≤0.5 秒；机械臂沿用 planner 新鲜度。原始 Point-LIO 输出不提供可用速度，不作静止依据；机械臂停稳继续检查原始 ArmStatus 的新鲜、有限速度，ArmTelemetry 也不再把非有限速度伪造为零。
 - A：导航与 left_look 并行 → 左侧识别/抓取/left_harvest/Home → right_look → 右侧对应工序。
 - B：每树只观察一次，按既有像素评分选择最多两个匹配且成熟的目标。在首次观察姿态下，一次性转换并保存两个基座坐标/绝对 pitch。第二抓直接用已保存请求，不重新 Look、识别或换算。第一抓失败但一次安全 Home 成功后，继续第二抓。
 - C：二维码任务顺序、路线与左右侧使用现有规划结果，导航时提前 Look，不增加到点方向复核。导航失败跳过当前项，不在未到达的位置抓取。
@@ -140,7 +142,13 @@ source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
 planner 的 `arm_presets.yaml` 中完成：Look 末点 OPEN；Harvest 末点 OPEN 且驻留；Home 保持夹爪。本次不修改 planner 的代码、配置或文件，不补假姿态。缺失、越限、执行失败或超时均返回失败。
 区域只通过现有持久 ROS 话题通知感知/扫码节点，不再发送下位机区域命令。
 
-独立 `camera_arm_target_client` 复用 MissionContext 的相同抓取流程，区域工具只发布 ROS 区域，不再引用已删除的旧消息。`test_camera_arm_target.sh` 同时启动唯一 perception/串口桥与 planner-only launch；复用模式不另启动 planner。运行前还需提供导航栈的 `/odometry`，否则静止门槛不能通过。该脚本会驱动硬件，不是自动测试。
+独立 `camera_arm_target_client` 与 `test_camera_arm_target.sh` 现已收敛为专用的“相机-机械臂单步抓取配合调试工具”：不再强依赖底盘导航栈 `/odometry` 与 Harvest 动作；每次运行先复位回 `home` 预设位置，随后进入 `left_look`/`right_look` 观察姿态，稳定后开启识别并完成基座解算与闭爪抓取，抓取完成后停留在目标点。该脚本会驱动硬件，不是自动测试。
+
+- 模型选择使用可靠、持久区域发布端，先等待 `fruit_perception_node` 的实际区域订阅端，再发布一次并等待 DDS acknowledgment；仅扫码节点订阅或串口桥就绪不能视为感知区域端就绪。DDS 确认是传输层交付，不是推理就绪回执。
+- 不再支持品种过滤参数。B 等待 `targets_ready`，只从有限置信度、有效正深度的 `*_ripe` 目标中选最高置信度者；无成熟目标时等待超时，不回退到无类型信息的 `target_point` 或未成熟目标。
+- Look 完成并确认机械臂静止后，共享 planner 客户端以新鲜、有限、在限位内的四轴硬件角调用 `move_joints`，先 OPEN，再用实测角 HOLD 驻留（时长取 planner `default_dwell_ms`）。张爪失败不开始识别/目标运动；张爪在采图之前，避免采图后改变手眼姿态。未修改 Look 预设或标定。
+- 客户端等待值必须为有限正数，默认识别超时 10 秒；大于 10 秒的值截断到 10 秒，较短值保留。共享 planner 客户端对 `endpoint_timeout_sec`、`service_timeout_sec` 同样限幅，各次服务/识别/交付使用独立期限；运动超时仍请求 STOP，不将发布等同于硬件完成或停稳。
+
 退出路径请求 STOP/取消导航；杀进程不代表停稳。预设尚未补齐、串口顺序未经验证，也不能声称比赛整机可运行。
 原串口桥的跨报文轨迹装载顺序也仍需端到端核验。
 
diff --git a/src/fruit_mission/fruit_mission/camera_arm_target_client.py b/src/fruit_mission/fruit_mission/camera_arm_target_client.py
index 9905efc..31ba635 100644
--- a/src/fruit_mission/fruit_mission/camera_arm_target_client.py
+++ b/src/fruit_mission/fruit_mission/camera_arm_target_client.py
@@ -1,43 +1,70 @@
-"""Manual single-point client sharing the actual mission grasp implementation."""
+"""Manual single-point client for camera and arm target grasping debug."""
 
 import argparse
+import math
 import time
 
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
+    """Debug client for camera recognition and robotic arm targeting."""
 
     def __init__(self, debug=False):
         super().__init__('camera_arm_target_client')
         self.debug = debug
-        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
+        self.declare_parameter('camera_extrinsic_config', DEFAULT_EXTRINSIC_CONFIG)
         self.declare_parameter('perception_mode_topic', '/fruit_mission/perception_mode')
         self.declare_parameter('active_mode_topic', '/fruit_perception/active_mode')
+        self.declare_parameter('current_area_topic', '/fruit_mission/current_area')
+        self.declare_parameter('decision_topic', '/fruit_perception/decision')
         self.declare_parameter('bridge_ready_topic', '/auto_serial_bridge/ready')
         self.declare_parameter('cleanup_mode_timeout_sec', 2.0)
+        self.declare_parameter('detection_timeout_sec', 10.0)
+        self.declare_parameter('endpoint_timeout_sec', 10.0)
+        self.declare_parameter('current_area_delivery_timeout_sec', 1.0)
+        for name in ('cleanup_mode_timeout_sec', 'detection_timeout_sec',
+                     'endpoint_timeout_sec', 'current_area_delivery_timeout_sec'):
+            _bounded_timeout(self.get_parameter(name).value)
+
         self._mode = ''
         self._generation = 0
         self._bridge_ready = False
-        self._ctx = None
+        self._camera_planner = None
+        self._latest_decision = None
+
         self._mode_pub = self.create_publisher(
             String, self.get_parameter('perception_mode_topic').value, reliable_qos(1))
+        self._area_pub = self.create_publisher(
+            String, self.get_parameter('current_area_topic').value, latched_qos())
+
         self.create_subscription(
             String, self.get_parameter('active_mode_topic').value, self._on_mode, latched_qos())
         self.create_subscription(
             Bool, self.get_parameter('bridge_ready_topic').value,
             self._on_ready, latched_qos())
+        self.create_subscription(
+            FruitDecision, self.get_parameter('decision_topic').value,
+            self._on_decision, 100)
 
     def _on_mode(self, message):
         self._mode = message.data
@@ -46,18 +73,28 @@ class CameraArmTargetClient(Node):
     def _on_ready(self, message):
         self._bridge_ready = message.data
 
+    def _on_decision(self, message: FruitDecision):
+        self._latest_decision = message
+        if self.debug:
+            self._debug_decision(message)
+
+    def _debug_decision(self, message: FruitDecision):
+        stamp = message.header.stamp
+        items = [(item.type, item.confidence,
+                  (item.point.x, item.point.y, item.point.z) if item.has_point else None)
+                 for item in message.detections]
+        self.get_logger().info(
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
         conflicts = {'fruit_perception_container', 'camera_mode_manager',
@@ -70,63 +107,216 @@ class CameraArmTargetClient(Node):
         self._mode_pub.publish(String(data='OFF'))
         return self.wait_until(
             lambda: self._generation > previous and self._mode == 'OFF',
-            min(10.0, self.get_parameter('cleanup_mode_timeout_sec').value))
+            self.get_parameter('cleanup_mode_timeout_sec').value)
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
+
+    def _set_perception_mode(self, mode: str, timeout: float = 5.0) -> bool:
+        if self._mode == mode:
+            return True
+        previous = self._generation
+        deadline = time.monotonic() + _bounded_timeout(timeout)
+        while rclpy.ok() and time.monotonic() < deadline:
+            self._mode_pub.publish(String(data=mode))
+            if self._generation > previous and self._mode == mode:
+                return True
+            rclpy.spin_once(self, timeout_sec=0.1)
+        return self._mode == mode
+
+    def _wait_for_target(self, model: str, fresh_after_ns: int, timeout: float = 10.0):
+        deadline = time.monotonic() + _bounded_timeout(timeout)
+        while rclpy.ok() and time.monotonic() < deadline:
+            rclpy.spin_once(self, timeout_sec=0.1)
+            msg = self._latest_decision
+            if msg is None:
+                continue
+            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
+            if stamp_ns <= fresh_after_ns:
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
 
-    def run_sequence(self, area, direction, b_fruit=None):
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
+        self._camera_planner = CameraTargetPlannerClient(self, extrinsic_cfg)
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
+        # 7. Switch perception mode to DETECT
+        self.get_logger().info('Setting perception mode to DETECT...')
+        if not self._set_perception_mode('DETECT', 5.0):
+            self.get_logger().error('Failed to set perception mode to DETECT')
+            return 1
+        self.get_logger().info('Perception mode is DETECT')
+
+        # 8. Collect target from perception
+        fresh_after_ns = self.get_clock().now().nanoseconds
+        timeout_sec = _bounded_timeout(self.get_parameter('detection_timeout_sec').value)
+        self.get_logger().info(f'Observing for targets (timeout={timeout_sec}s)...')
+        target_point = self._wait_for_target(model, fresh_after_ns, timeout=timeout_sec)
+        if target_point is None:
+            self.get_logger().error('No target acquired within timeout')
+            self._set_perception_mode('OFF', 2.0)
+            return 1
+
+        # 9. Switch perception mode to OFF
+        self.get_logger().info('Target acquired. Switching perception mode to OFF...')
+        self._set_perception_mode('OFF', 2.0)
+
+        # 10. Transform optical target to arm base coordinates
+        self.get_logger().info('Transforming target to arm base coordinates...')
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
+        self.get_logger().info(
+            'Target grasped successfully! Stopping at target position as requested (no harvest).')
+        return 0
 
     def destroy_node(self):
-        if self._ctx is not None:
-            self._ctx.close()
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
+                      help='Check if conflicting nodes are absent')
+    mode.add_argument('--mode-off', action='store_true',
+                      help='Set perception mode to OFF for cleanup')
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
+                           help='Use left_look preset')
+    direction.add_argument('--right', dest='direction', action='store_const', const='right',
+                           help='Use right_look preset')
     options, ros_args = parser.parse_known_args(args)
+    unknown = remove_ros_args(args=ros_args)
+    if unknown:
+        parser.error(f'unrecognized arguments: {" ".join(unknown)}')
     if not (options.require_stack_absent or options.mode_off):
-        if options.zone is None or options.direction is None:
-            parser.error('--zone and --left/--right are required')
-        if (options.zone == 'B') != (options.b_fruit is not None):
-            parser.error('--apple/--pear must be specified for B only')
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
@@ -136,10 +326,10 @@ def main(args=None):
         elif options.mode_off:
             status = 0 if node.set_off_for_cleanup() else 7
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
index 51263d0..22e38f3 100644
--- a/src/fruit_mission/fruit_mission/camera_target_planner_client.py
+++ b/src/fruit_mission/fruit_mission/camera_target_planner_client.py
@@ -5,10 +5,9 @@ from pathlib import Path
 import time
 
 from fruit_arm_interfaces.srv import ExecuteArmPreset, MoveArmCartesian, MoveArmJoints
+from fruit_arm_interfaces.msg import ArmTelemetry
 from fruit_hardware_interfaces.msg import ArmStatus
 from fruit_arm_planner.joint_transformer import JointTransformer
-from fruit_arm_planner.kinematics import RobotKinematics
-from sensor_msgs.msg import JointState
 from std_srvs.srv import Trigger
 import yaml
 
@@ -38,9 +37,6 @@ class CameraTargetPlannerClient:
         self.rotation, self.translation_mm = load_camera_extrinsic(config_path)
         with Path(self.config['arm_config_path']).open(encoding='utf-8') as stream:
             arm = yaml.safe_load(stream)['arm_planner_node']['ros__parameters']
-        if not all(math.isfinite(v) and v > 0 for v in arm['geometry'].values()):
-            raise ValueError('arm geometry must be finite and positive')
-        self.kinematics = RobotKinematics.from_geometry_dict(arm['geometry'])
         self.transformer = JointTransformer.from_config_dict(arm['joint_mapping'])
         self.status_max_age_sec = float(arm['planning']['status_max_age_sec'])
         self.gripper_dwell_ms = arm['planning']['default_dwell_ms']
@@ -54,6 +50,7 @@ class CameraTargetPlannerClient:
                 raise ValueError(f'{name} must be numeric')
             if not math.isfinite(value) or value <= 0:
                 raise ValueError(f'{name} must be finite and positive')
+            self.config[name] = min(10.0, float(value))
         for name in ('camera_frame', 'joint_state_topic',
                      'move_cartesian_service', 'stop_service'):
             if not isinstance(self.config[name], str) or not self.config[name].strip():
@@ -69,14 +66,14 @@ class CameraTargetPlannerClient:
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
@@ -86,40 +83,46 @@ class CameraTargetPlannerClient:
         self._status_subscription = node.create_subscription(
             ArmStatus, '/fruit_hardware_bridge/arm/status', self._on_status, 10)
         self._motion_timer = node.create_timer(0.1, self._check_motion_timeout)
-        offset = arm['planning'].get('cartesian_offset_m', [0.0, 0.0, 0.0])
         node.get_logger().info(
-            f'Mission camera transform ready; planner adds cartesian_offset_m={offset}')
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
@@ -142,22 +145,21 @@ class CameraTargetPlannerClient:
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
             request = MoveArmCartesian.Request()
             request.x, request.y, request.z = (v / 1000.0 for v in base_mm)
             desired_pitch = self.config['motion']['pitch_deg']
             request.pitch_deg = float(
-                math.degrees(pitch) if desired_pitch is None else desired_pitch)
+                pitch_deg if desired_pitch is None else desired_pitch)
             request.duration_ms = self.config['motion']['duration_ms']
             request.gripper_action = 2  # CLOSE at the endpoint.
             request.linear_path = self.config['motion']['linear_path']
@@ -178,18 +180,28 @@ class CameraTargetPlannerClient:
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
diff --git a/test_camera_arm_target.sh b/test_camera_arm_target.sh
index 53263a4..a367bf1 100755
--- a/test_camera_arm_target.sh
+++ b/test_camera_arm_target.sh
@@ -14,9 +14,8 @@ readonly CLIENT_LOG="${RUN_DIR}/client.log"
 LAUNCH_PID=""
 PLANNER_PID=""
 STOPPING=0
-ZONE=""
+MODEL=""
 DIRECTION=""
-B_FRUIT=""
 DEBUG=0
 
 log()
@@ -27,12 +26,13 @@ log()
 usage()
 {
   cat <<'EOF'
-Usage: ./test_camera_arm_target.sh --zone A|B|C --left|--right [--apple|--pear] [--debug]
+Usage: ./test_camera_arm_target.sh --model A|B --left|--right [--debug]
 
-Select the current area and observation direction before running the existing
-Camera recognition and arm-target workflow. Zone B requires exactly one of
---apple or --pear. --debug prints every Camera detection message and every
-selected target point with their ROS timestamps.
+Test camera and arm coordination.
+Arm returns to the home position first, then executes observation and grasp.
+  --model A|B     A: vegetable model, B: fruit model (supports --zone A|B as alias)
+  --left|--right  Arm observation preset (left_look or right_look)
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
@@ -227,9 +209,6 @@ main()
     client_args+=(--debug)
     log "Per-frame Camera detection and target-point logging is enabled"
   fi
-  if [[ -n "${B_FRUIT}" ]]; then
-    client_args+=("${B_FRUIT}")
-  fi
   ros2 run fruit_mission camera_arm_target_client \
     --require-stack-absent --ros-args --params-file "${CLIENT_CONFIG}" || {
     log "ERROR: perception or serial nodes are already running"
@@ -249,7 +228,7 @@ main()
 
   log "Waiting for the perception stack"
   if ros2 run fruit_mission camera_arm_target_client \
-    --zone "${ZONE}" "${DIRECTION}" "${client_args[@]}" \
+    --model "${MODEL}" "${DIRECTION}" "${client_args[@]}" \
     --ros-args --params-file "${CLIENT_CONFIG}" \
     2>&1 | tee -a "${CLIENT_LOG}"; then
     log "Task sequence completed"
diff --git a/test_camera_detector.py b/test_camera_detector.py
old mode 100644
new mode 100755
index 7c0616f..9e561e7
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
@@ -53,17 +52,12 @@ from cv_bridge import CvBridge  # noqa: E402
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
@@ -92,58 +86,70 @@ class LetterboxTransform:
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
+    """Subscribes to color image, aligned depth image, and camera intrinsics."""
+
+    def __init__(self, color_topic: str, depth_topic: str, info_topic: str) -> None:
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
+        qos_best_effort = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
+        qos_reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
 
-    def _on_image(self, message: Image) -> None:
+        self.create_subscription(Image, color_topic, self._on_color, qos_best_effort)
+        self.create_subscription(Image, depth_topic, self._on_depth, qos_best_effort)
+        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_reliable)
+
+    def _on_color(self, message: Image) -> None:
         try:
-            frame = self._bridge.imgmsg_to_cv2(
-                message,
-                desired_encoding='rgb8',
-            )
+            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
+            self._latest_color = np.ascontiguousarray(frame)
+        except Exception as error:  # noqa: BLE001
+            self.get_logger().error(f'Color frame conversion failed: {error}')
+
+    def _on_depth(self, message: Image) -> None:
+        try:
+            depth = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
+            self._latest_depth = np.ascontiguousarray(depth)
         except Exception as error:  # noqa: BLE001
-            self.get_logger().error(f'Frame conversion failed: {error}')
-            return
-        self._latest_frame = np.ascontiguousarray(frame)
-        self._latest_encoding = message.encoding
+            self.get_logger().error(f'Depth frame conversion failed: {error}')
 
-    def take_latest_frame(self) -> np.ndarray | None:
-        frame = self._latest_frame
-        self._latest_frame = None
-        return frame
+    def _on_info(self, message: CameraInfo) -> None:
+        self.camera_info = message
+
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
+    )
+    parser.add_argument(
+        '--preset',
+        '-p',
+        default='left_look',
+        choices=('left_look', 'right_look'),
+        help='arm observation preset reference for base coordinate projection (default: left_look)',
     )
     parser.add_argument(
         '--conf',
@@ -164,6 +170,11 @@ def parse_args() -> argparse.Namespace:
         default=DEFAULT_DISPLAY,
         help=f'desktop display for the GUI window (default: {DEFAULT_DISPLAY})',
     )
+    parser.add_argument(
+        '--controls',
+        action='store_true',
+        help='open the camera exposure and control parameters adjustment panel',
+    )
     return parser.parse_args()
 
 
@@ -177,7 +188,9 @@ def select_camera(config: dict[str, Any]) -> CameraSelection:
     return CameraSelection(
         name='camera',
         kind='camera',
-        topic='/camera/color/image_raw',
+        color_topic='/camera/color/image_raw',
+        depth_topic='/camera/aligned_depth_to_color/image_raw',
+        info_topic='/camera/color/camera_info',
         parameters=parameters,
     )
 
@@ -187,10 +200,82 @@ def model_path_for(letter: str, config: dict[str, Any]) -> Path:
     model_root = Path(perception['model_root'])
     if not model_root.is_absolute():
         raise ValueError('model_root must be an absolute source-directory path')
-    detector = 'fruit_type' if letter in ('B', 'D') else 'vegetable_type'
+    detector = 'fruit_type' if letter.upper() in ('B', 'D') else 'vegetable_type'
     return model_root / detector / 'best.onnx'
 
 
+def load_extrinsic_transform() -> tuple[np.ndarray, np.ndarray]:
+    """Load R_E_C and t_E_C (mm) from camera_extrinsic.yaml."""
+    try:
+        with EXTRINSIC_CONFIG_PATH.open(encoding='utf-8') as f:
+            cfg = yaml.safe_load(f)
+        r_rows = cfg['rotation_end_from_rgb']
+        e_s = cfg['translation_end_to_screw_mm']
+        s_c = cfg['translation_screw_to_rgb_mm']
+        r_mat = np.array(r_rows, dtype=float)
+        t_vec = np.array(e_s, dtype=float) + np.array(s_c, dtype=float)
+        return r_mat, t_vec
+    except Exception as error:  # noqa: BLE001
+        print(f'Warning: failed to load camera extrinsic: {error}', file=sys.stderr)
+        return np.eye(3), np.zeros(3)
+
+
+def get_depth_at(depth_image: np.ndarray | None, u: int, v: int, window_radius: int = 3) -> float | None:
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
+    r_e_c: np.ndarray,
+    t_e_c_mm: np.ndarray,
+    preset: str = 'left_look',
+) -> tuple[float, float, float]:
+    """Transform camera optical 3D point to arm base frame coordinates (m)."""
+    cam_mm = np.array(point_cam_m, dtype=float) * 1000.0
+    end_mm = r_e_c @ cam_mm + t_e_c_mm
+    if preset == 'right_look':
+        yaw = math.radians(-90.0)
+        pitch = math.radians(-58.0)
+        origin_e_in_b = np.array([0.0, -150.0, 300.0])
+    else:  # left_look
+        yaw = math.radians(90.0)
+        pitch = math.radians(-58.0)
+        origin_e_in_b = np.array([0.0, 150.0, 300.0])
+
+    cy, sy = math.cos(yaw), math.sin(yaw)
+    cp, sp = math.cos(pitch), math.sin(pitch)
+    r_b_e = np.array([
+        [cp * cy, -sy, -sp * cy],
+        [cp * sy,  cy, -sp * sy],
+        [sp,        0,   cp   ],
+    ])
+    base_mm = origin_e_in_b + r_b_e @ end_mm
+    return tuple(float(v) for v in base_mm / 1000.0)
+
+
 def write_ros_params_file(selection: CameraSelection) -> Path:
     TMP_DIR.mkdir(parents=True, exist_ok=True)
     params_path = TMP_DIR / f'test_camera_detector_{selection.name}_params.yaml'
@@ -337,10 +422,7 @@ def parse_raw_yolo_output(
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
@@ -452,74 +534,202 @@ def to_original_box(
 
 def draw_annotations(
     frame_rgb: np.ndarray,
+    depth_frame: np.ndarray | None,
     detections: list[Detection],
     transform: LetterboxTransform,
+    intrinsics: tuple[float, float, float, float],
+    extrinsic: tuple[np.ndarray, np.ndarray],
+    preset: str,
+    mouse_u: int,
+    mouse_v: int,
     fps: float,
+    model_name: str,
+    ripeness_classifier: RipenessClassifier | None = None,
 ) -> np.ndarray:
     annotated = frame_rgb.copy()
-    color = (0, 255, 0)
+    h, w = annotated.shape[:2]
+    fx, fy, cx, cy = intrinsics
+    r_e_c, t_e_c = extrinsic
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
+        if depth_m is not None:
+            xc, yc, zc = pixel_to_camera_point(uc, vc, depth_m, fx, fy, cx, cy)
+            xb, yb, zb = camera_to_base_point((xc, yc, zc), r_e_c, t_e_c, preset)
+            main_label = f'{detection.label}{mat_suffix} {detection.score:.2f} | D={depth_m:.3f}m'
+            sub_label = f'Cam:({xc:+.2f}, {yc:+.2f}, {zc:.2f})  Base:({xb:+.2f}, {yb:+.2f}, {zb:+.2f})m'
+        else:
+            main_label = f'{detection.label}{mat_suffix} {detection.score:.2f} | D: NoDepth'
+            sub_label = 'Depth missing/blocked at center'
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
         )
+        cv2.putText(
+            annotated, sub_label, (sub_x + 2, sub_y),
+            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA,
+        )
+
+    # Mouse inspection crosshair & coordinates
+    mouse_text = f'Mouse: ({mouse_u}, {mouse_v}) | Out of bounds'
+    if 0 <= mouse_u < w and 0 <= mouse_v < h:
+        cv2.circle(annotated, (mouse_u, mouse_v), 5, (255, 255, 0), 1)
+        m_depth = get_depth_at(depth_frame, mouse_u, mouse_v, window_radius=2)
+        if m_depth is not None:
+            m_xc, m_yc, m_zc = pixel_to_camera_point(mouse_u, mouse_v, m_depth, fx, fy, cx, cy)
+            m_xb, m_yb, m_zb = camera_to_base_point((m_xc, m_yc, m_zc), r_e_c, t_e_c, preset)
+            mouse_text = (
+                f'Mouse ({mouse_u}, {mouse_v}): Depth={m_depth:.3f}m | '
+                f'Cam=({m_xc:+.3f}, {m_yc:+.3f}, {m_zc:.3f})m | '
+                f'Base=({m_xb:+.3f}, {m_yb:+.3f}, {m_zb:+.3f})m'
+            )
+        else:
+            mouse_text = f'Mouse ({mouse_u}, {mouse_v}): Depth=None (0 mm / invalid)'
 
-    fps_label = f'FPS: {fps:.1f}'
+    # Top-left info banner
+    top_banner = f'Model: {model_name} | Preset: {preset} | FPS: {fps:.1f} | Detections: {len(detections)}'
+    cv2.rectangle(annotated, (0, 0), (w, 30), (20, 20, 20), cv2.FILLED)
     cv2.putText(
-        annotated,
-        fps_label,
-        (10, 28),
-        cv2.FONT_HERSHEY_SIMPLEX,
-        0.75,
-        (0, 0, 0),
-        4,
-        cv2.LINE_AA,
+        annotated, top_banner, (10, 21),
+        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 120), 1, cv2.LINE_AA,
     )
+
+    # Bottom status bar (mouse probe + hotkey hints)
+    cv2.rectangle(annotated, (0, h - 32), (w, h), (15, 15, 15), cv2.FILLED)
     cv2.putText(
-        annotated,
-        fps_label,
-        (10, 28),
-        cv2.FONT_HERSHEY_SIMPLEX,
-        0.75,
-        color,
-        2,
-        cv2.LINE_AA,
+        annotated, mouse_text, (10, h - 18),
+        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA,
     )
+    hints = "[s]Save [m]Model [p]Preset [q]Quit"
+    th, _ = cv2.getTextSize(hints, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
+    cv2.putText(
+        annotated, hints, (w - th[0] - 10, h - 18),
+        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA,
+    )
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
+                print(f'Warning: failed to compile ripeness model for {crop}: {err}', file=sys.stderr)
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
@@ -527,10 +737,7 @@ class Detector:
 
         self._core = ov.Core()
         devices = self._core.available_devices
-        if 'GPU' not in devices:
-            raise RuntimeError(
-                f'OpenVINO GPU device is not available; devices: {devices}',
-            )
+        self._device = 'GPU' if 'GPU' in devices else 'CPU'
 
         model = self._core.read_model(str(model_path))
         if not model.has_rt_info(['framework', 'names']):
@@ -567,7 +774,7 @@ class Detector:
 
         self._compiled = self._core.compile_model(
             model,
-            'GPU',
+            self._device,
             {'PERFORMANCE_HINT': 'THROUGHPUT'},
         )
         self._input_port = self._compiled.input(0)
@@ -595,6 +802,14 @@ class Detector:
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
@@ -636,7 +851,11 @@ def run() -> int:
     args = parse_args()
     config = load_config()
     selection = select_camera(config)
-    model_path = model_path_for(args.model, config)
+
+    current_model = args.model.upper()
+    current_preset = args.preset
+    model_path = model_path_for(current_model, config)
+
     perception = config['fruit_perception_node']['ros__parameters']
     confidence_threshold = float(
         perception['confidence_threshold'] if args.conf is None else args.conf,
@@ -652,84 +871,96 @@ def run() -> int:
             raise ValueError(f'{name} must be between 0 and 1, got {value}')
 
     detector = Detector(model_path)
+    extrinsic = load_extrinsic_transform()
+    model_root = Path(perception['model_root'])
+    ripeness_classifier = RipenessClassifier(model_root, detector.core, detector.device)
+
     rclpy.init()
-    node = FrameSubscriber(selection.topic)
+    node = FrameSubscriber(selection.color_topic, selection.depth_topic, selection.info_topic)
     camera_process: subprocess.Popen | None = None
     log_path = TMP_DIR / f'test_camera_detector_{selection.name}.log'
-    if wait_for_publisher(node, selection.topic, 1.0):
+
+    if wait_for_publisher(node, selection.color_topic, 1.0):
         print(
-            f'Using existing publisher on {selection.topic}; '
-            f'model {model_path}; conf {confidence_threshold:.3f}; '
-            f'iou {nms_iou_threshold:.3f}',
+            f'Using existing camera stream on {selection.color_topic}; '
+            f'model={model_path.name}; preset={current_preset}; conf={confidence_threshold:.2f}',
             flush=True,
         )
     else:
         camera_process, log_path = start_camera(selection)
         print(
-            f'Started {selection.name}; image topic {selection.topic}; '
-            f'model {model_path}; conf {confidence_threshold:.3f}; '
-            f'iou {nms_iou_threshold:.3f}; log {log_path}',
+            f'Started RealSense camera; color={selection.color_topic}; '
+            f'depth={selection.depth_topic}; log={log_path}',
             flush=True,
         )
-    window_name = f'{selection.name} model {args.model}'
+
+    window_name = 'RealSense 3D Detector & Depth Inspector'
     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
-    cv2.moveWindow(window_name, 20, 60)
-    try:
-        if not wait_for_publisher(
-            node,
-            selection.topic,
-            10.0,
-        ):
-            check_camera_process(camera_process, log_path)
-            raise RuntimeError(
-                f'Timed out waiting for Camera frames on {selection.topic}',
+    cv2.resizeWindow(window_name, 1280, 720)
+    cv2.moveWindow(window_name, 20, 40)
+
+    mouse_coords = [-1, -1]
+
+    def on_mouse(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
+        mouse_coords[0] = x
+        mouse_coords[1] = y
+
+    cv2.setMouseCallback(window_name, on_mouse)
+
+    control_panel = None
+    if args.controls:
+        try:
+            from camera_control_gui import (
+                CameraControlPanel,
+                CameraControlService,
+                save_camera_controls,
             )
-        control_service = CameraControlService(node)
-        control_panel = CameraControlPanel(
-            selection.name,
-            control_service.discover(),
-            control_service.set,
-            lambda values: save_camera_controls(
+            control_service = CameraControlService(node)
+            control_panel = CameraControlPanel(
                 selection.name,
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
+                control_service.discover(),
+                control_service.set,
+                lambda values: save_camera_controls(
+                    selection.name, values, CONFIG_PATH, TMP_DIR,
+                ),
+            )
+        except Exception as err:
+            print(f'Warning: failed to initialize control panel: {err}', file=sys.stderr)
 
     last_frame_time = 0.0
     displayed_fps = 0.0
     last_wait_message = 0.0
     announced_first_frame = False
 
+    print('\n=== RealSense 3D Inspector Controls ===')
+    print('  [Hover Mouse] : Probe depth and 3D camera / arm base coordinates')
+    print('  [Key s]       : Save current annotated snapshot image to disk')
+    print('  [Key m]       : Toggle model between A (vegetables) and B (fruits)')
+    print('  [Key p]       : Toggle arm preset between left_look and right_look')
+    print('  [Key q / ESC] : Quit viewer\n')
+
     try:
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
@@ -753,20 +984,55 @@ def run() -> int:
                 )
             last_frame_time = now
 
+            # Camera intrinsics
+            h_img, w_img = frame.shape[:2]
+            if node.camera_info is not None and node.camera_info.k[0] > 0.0:
+                intrinsics = (
+                    float(node.camera_info.k[0]),
+                    float(node.camera_info.k[4]),
+                    float(node.camera_info.k[2]),
+                    float(node.camera_info.k[5]),
+                )
+            else:
+                intrinsics = (900.0, 900.0, w_img / 2.0, h_img / 2.0)
+
             annotated_rgb = draw_annotations(
                 frame,
+                depth_frame,
                 detections,
                 transform,
+                intrinsics,
+                extrinsic,
+                current_preset,
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
+            elif key in (ord('p'), ord('P')):
+                current_preset = 'right_look' if current_preset == 'left_look' else 'left_look'
+                print(f'>>> Switched reference preset to {current_preset}', flush=True)
+
     finally:
-        control_panel.close()
+        if control_panel is not None:
+            control_panel.close()
         try:
             node.destroy_node()
         except Exception:  # noqa: BLE001

启动前 status：
 M AGENTS.md
 M README.md
 M docs/PROJECT_ARCHITECTURE.md
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
 M src/fruit_mission/config/camera_extrinsic.yaml
 M src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md
 M src/fruit_mission/fruit_mission/camera_arm_target_client.py
 M src/fruit_mission/fruit_mission/camera_target_planner_client.py
 M test_camera_arm_target.sh
 M test_camera_detector.py
?? .agents/skills/wisadel-helium-voice/SKILL.md
?? .pi/skills/wisadel-helium-voice/SKILL.md
?? scripts/generate_wisadel_voice.py
?? sounds/fruit_success.wav
?? src/fruit_arm_interfaces/msg/ArmTelemetry.msg

