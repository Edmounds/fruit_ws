# 启动基线（只读，不是修复计划）

启动前 Git diff（既有改动，不得擅自归因）：

diff --git a/docs/CAMERA_TUNING.md b/docs/CAMERA_TUNING.md
index 0c8ac1d..a6c1121 100644
--- a/docs/CAMERA_TUNING.md
+++ b/docs/CAMERA_TUNING.md
@@ -1,7 +1,13 @@
 # 摄像头实时调参
 
-`test_camera_detector.py` 会同时打开 YOLO 检测画面和摄像头参数窗口。参数修改后立即作用于当前摄像头，可直接观察检测框和置信度的变化。
-（注：当前系统架构中深度相机硬件流保持常开，人工曝光调参针对底层 UVC/RGB 曝光与白平衡，不影响任务层的观察窗口数据采纳机制。）
+`test_camera_detector.py` 默认同时打开三个窗口：
+1. **YOLO 目标检测与三维测距窗口**（1280x720 RGB，对齐深度三维测距）。
+2. **原生深度伪彩色窗口**（848x480 原生深度，自适应距离伪彩色渲染，显示 Near/Far 图例）。
+3. **英文参数调参窗口**（`RealSense Camera Controls`，分组调节 RGB 和 Depth 核心参数）。
+
+参数修改后立即作用于当前摄像头，可直接观察检测框和深度变化。关闭任一窗口或在 OpenCV 窗口按 `q`/`Esc` 均安全退出。
+
+（注：系统架构中深度相机硬件流保持常开，人工曝光调参针对底层 UVC/RGB 曝光与白平衡，以及深度传感器手动曝光与增益。原生深度伪彩色仅用于可视化观察，底层测距计算继续使用毫米级原始深度数据；正式任务运行时若检测到 `/fruit_mission`，调试工具拒绝启动。）
 
 ## 启动
 
@@ -10,7 +16,6 @@
 ```bash
 cd /home/rc4/fruit_ws
 source scripts/setup_fruit_runtime.bash
-
 ```
 
 启动测试与调参（默认使用深度相机 camera）：
@@ -52,14 +57,25 @@ python3 test_camera_detector.py --model B --conf 0.60 --iou 0.45
 
 两个命令行参数只影响当前进程，不会修改 YAML。省略参数时继续使用 YAML 中的值，当前默认值为 `conf=0.50`、`iou=0.45`。
 
-脚本默认把窗口显示到机器人连接的桌面屏幕。检测窗口按 `q` 或 `Esc`，或者关闭参数窗口，即可退出。
-正常退出会停止脚本自己启动的相机进程。如果窗口或父脚本被强制终止，独立的
+脚本默认把窗口显示到机器人连接的桌面屏幕。检测窗口按 `q` 或 `Esc`，或者关闭任一窗口（检测画面、原生深度画面或参数窗口），即可退出。
+正常退出会停止脚本自己启动的相机进程，绝不终止复用的外部感知或桥接节点。如果窗口或父脚本被强制终止，独立的
 `test_camera_detector_*_container` 可能继续占用摄像头；返回工作空间根目录执行
 `./kill_fruit_processes.sh` 可清理该容器及其 `ros2 component standalone` 包装进程。
 
 ## 常用参数
 
-参数窗口只显示当前摄像头实际支持的项目和范围：
+参数窗口使用全英文界面，分为 `Depth Controls` 和 `RGB Controls` 两大区域，仅显示当前摄像头实际支持的项目和范围：
+
+### Depth Controls
+
+| 参数 | 作用 |
+| --- | --- |
+| Depth Exposure (us) | 深度传感器曝光时间（微秒），默认 10589；调节时自动确保深度自动曝光关闭 |
+| Depth Gain | 深度传感器图像增益，默认 16 |
+
+（注：深度预设固定为 `High Accuracy` (3)，深度自动曝光固定为 `false`，发射器固定为 Laser (`emitter_enabled: 1`, `laser_power: 150.0`)，时空滤波保持开启 (`spatial_filter.enable: true`, `temporal_filter.enable: true`)，均从 YAML 加载。）
+
+### RGB Controls
 
 | 参数 | 作用 |
 | --- | --- |
@@ -82,10 +98,11 @@ python3 test_camera_detector.py --model B --conf 0.60 --iou 0.45
 
 1. 把机器人和目标放到比赛时的实际距离，保持检测画面持续可见。
 2. 室内灯光稳定时先关闭自动曝光，调低 `Exposure` 到运动时没有明显拖影，再用 `Gain` 补足亮度。
-3. 颜色偏差明显时关闭自动白平衡，再调 `White balance`，使不同类别的颜色接近训练数据。
-4. 少量调整亮度、对比度、饱和度和锐度，同时观察漏检、误检和置信度，不要只追求画面鲜艳。
-5. 室内交流灯出现横纹或闪烁时调整 `Power line frequency`；中国大陆通常使用 `50 Hz` 对应的设备选项。
-6. 在目标静止、机器人运动和背景干扰三种情况下都确认效果，再保存参数。
+3. 检查原生深度窗口（`848x480`），根据被测目标表面材质和反光特性，适当微调 `Depth Exposure (us)` 与 `Depth Gain`，确保果蔬目标表面点云稳定连续，黑洞（无效深度）最小化。
+4. 颜色偏差明显时关闭自动白平衡，再调 `White balance`，使不同类别的颜色接近训练数据。
+5. 少量调整亮度、对比度、饱和度和锐度，同时观察漏检、误检和置信度，不要只追求画面鲜艳。
+6. 室内交流灯出现横纹或闪烁时调整 `Power line frequency`；中国大陆通常使用 `50 Hz` 对应的设备选项。
+7. 在目标静止、机器人运动和背景干扰三种情况下都确认效果，再保存参数。
 
 一次只调整一个主要参数，便于判断检测变化来自哪里。曝光和增益通常比锐度、Gamma 等后处理参数更优先。
 
@@ -93,7 +110,7 @@ python3 test_camera_detector.py --model B --conf 0.60 --iou 0.45
 
 确认效果后点击参数窗口底部的 `Save`。成功保存时，窗口状态栏会显示备份文件名，终端也会输出完整路径。
 
-当前值写入源工作空间配置：
+当前成功应用的值（包括 RGB 参数和 Depth Exposure/Gain）写入源工作空间配置：
 
 ```text
 /home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml
diff --git a/docs/PROJECT_ARCHITECTURE.md b/docs/PROJECT_ARCHITECTURE.md
index 63079c2..4df9234 100644
--- a/docs/PROJECT_ARCHITECTURE.md
+++ b/docs/PROJECT_ARCHITECTURE.md
@@ -124,7 +124,7 @@ fruit_ws/src/
 深度相机节点为 `/camera`，原生接口为 `/camera/*`，感知输出为
 `/fruit_perception/camera/*`，决策发布于 `/fruit_perception/decision`。
 
-camera 的固定原生流开启同步、深度对齐与 RGBD。相机硬件流常开、模型持续推理：
+camera 的固定原生流开启同步、深度对齐与 RGBD（Color 为 `1280x720x30` RGB8，原生深度为 `848x480x30` Z16 配合 High Accuracy 预设与手动曝光，对齐深度保持 1280x720 对齐像素空间）。相机硬件流常开、模型持续推理：
 
 1. 到站且 QR 姿态完成后开启 QR 观察窗口，Python/OpenCV 扫码节点对彩色原图解码并确认计划（`confirm_frames=1`）。
 2. B/C 计划使用 Reliable、Transient Local、深度 1 发布，计划中附带窗口 ID 及帧时间戳。
diff --git a/src/fruit_arm_planner/config/arm_presets.yaml b/src/fruit_arm_planner/config/arm_presets.yaml
index 020a919..debfb43 100644
--- a/src/fruit_arm_planner/config/arm_presets.yaml
+++ b/src/fruit_arm_planner/config/arm_presets.yaml
@@ -191,10 +191,17 @@ arm_planner_node:
         - {duration_ms: 2000, gripper: 0, target: [0.22, -0.00, 0.17, 0.0]}
 
       left_look_b:
-        description: "机械臂左侧观察位姿"
+        description: "机械臂左侧观察位姿,必须从伸直状态切换到这个状态"
         coordinate_frame: "cartesian"
         waypoints:
+          - { target: [0.1202, 0.0000, 0.550, 0.0], duration_ms: 200, gripper: 0 }
           - { target: [0.0203, 0.0703, 0.303, -46.7], duration_ms: 1000, gripper: 0 }
-      #right_look_b:
+
+      right_look_b:
+        description: "机械臂右侧观察位姿,必须从伸直状态切换到这个状态"
+        coordinate_frame: "cartesian"
+        waypoints:
+          - { target: [0.1202, 0.0000, 0.550, 0.0], duration_ms: 200, gripper: 0 }
+          - { target: [0.0203, -0.0703, 0.303, 46.7], duration_ms: 1000, gripper: 0 }
       #left_store_b:
       #right_store_b
diff --git a/src/fruit_perception/README.md b/src/fruit_perception/README.md
index 8308fa5..31222cc 100644
--- a/src/fruit_perception/README.md
+++ b/src/fruit_perception/README.md
@@ -49,7 +49,13 @@ Target locking and candidate accumulation only occur within a valid harvest wind
 Closing a window (`valid: 0`) immediately revokes evidence accumulation and resets selectors.
 
 The camera node synchronizes color and depth, aligns depth to color, and publishes
-both aligned images in one RGBD message (`/camera/rgbd`).
+both aligned images in one RGBD message (`/camera/rgbd`). Color profile is configured to
+`1280x720x30` (`RGB8`). Native depth profile is configured to `848x480x30` (`Z16`) with
+`depth_module.visual_preset: 3` (High Accuracy), disabled depth auto exposure,
+manual depth exposure (`10589` usec), gain (`16`), laser emitter (`1`), laser power (`150.0`),
+and post-processing filters (`spatial_filter.enable: true`, `temporal_filter.enable: true`).
+Aligned depth (`/camera/aligned_depth_to_color/image_raw`) and `/camera/rgbd` remain in
+the `1280x720` color pixel coordinate frame for accurate 2D/3D projection.
 
 Each RGBD callback also publishes only its `std_msgs/Header` on
 `/fruit_perception/camera/rgbd_heartbeat`. The external debug monitor combines
diff --git a/src/fruit_perception/config/fruit_perception.yaml b/src/fruit_perception/config/fruit_perception.yaml
index 4ba9a9b..0d9794a 100644
--- a/src/fruit_perception/config/fruit_perception.yaml
+++ b/src/fruit_perception/config/fruit_perception.yaml
@@ -17,8 +17,16 @@ camera:
     rgb_camera.backlight_compensation: false
     rgb_camera.power_line_frequency: 3
     enable_depth: true
-    depth_module.depth_profile: "1280x720x30"
+    depth_module.depth_profile: "848x480x30"
     depth_module.depth_format: "Z16"
+    depth_module.visual_preset: 3
+    depth_module.enable_auto_exposure: false
+    depth_module.exposure: 10589
+    depth_module.gain: 16
+    depth_module.emitter_enabled: 1
+    depth_module.laser_power: 150.0
+    spatial_filter.enable: true
+    temporal_filter.enable: true
     enable_sync: true
     align_depth.enable: true
     enable_rgbd: true
diff --git a/test_camera_detector.py b/test_camera_detector.py
index ecc2383..b5e6bcd 100755
--- a/test_camera_detector.py
+++ b/test_camera_detector.py
@@ -102,17 +102,19 @@ class CameraSelection:
     kind: str
     color_topic: str
     depth_topic: str
+    raw_depth_topic: str
     info_topic: str
     parameters: dict[str, Any]
 
 
 class FrameSubscriber(Node):
-    """Subscribes to color image, aligned depth image, camera intrinsics, and arm telemetry."""
+    """Subscribes to color image, aligned depth, raw depth, camera intrinsics, and arm telemetry."""
 
     def __init__(
         self,
         color_topic: str,
         depth_topic: str,
+        raw_depth_topic: str,
         info_topic: str,
         joint_state_topic: str = '/arm_planner_node/joint_states',
         arm_config_path: Path | str | None = None,
@@ -121,6 +123,7 @@ class FrameSubscriber(Node):
         self._bridge = CvBridge()
         self._latest_color: np.ndarray | None = None
         self._latest_depth: np.ndarray | None = None
+        self._latest_raw_depth: np.ndarray | None = None
         self.camera_info: CameraInfo | None = None
 
         self._tcp_pose: tuple[float, float, float, float, float] | None = None
@@ -145,6 +148,7 @@ class FrameSubscriber(Node):
 
         self.create_subscription(Image, color_topic, self._on_color, qos_best_effort)
         self.create_subscription(Image, depth_topic, self._on_depth, qos_best_effort)
+        self.create_subscription(Image, raw_depth_topic, self._on_raw_depth, qos_best_effort)
         self.create_subscription(CameraInfo, info_topic, self._on_info, qos_reliable)
         self.create_subscription(
             ArmTelemetry, joint_state_topic, self._on_telemetry, qos_telemetry,
@@ -169,6 +173,18 @@ class FrameSubscriber(Node):
         except Exception as error:  # noqa: BLE001
             self.get_logger().error(f'Depth frame conversion failed: {error}')
 
+    def _on_raw_depth(self, message: Image) -> None:
+        try:
+            raw_depth = self._bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
+            if raw_depth.ndim == 2 and raw_depth.dtype == np.uint16:
+                self._latest_raw_depth = np.ascontiguousarray(raw_depth)
+            else:
+                self.get_logger().warning(
+                    f'Unexpected raw depth frame format: {raw_depth.dtype} {raw_depth.shape}',
+                )
+        except Exception as error:  # noqa: BLE001
+            self.get_logger().error(f'Raw depth frame conversion failed: {error}')
+
     def _on_info(self, message: CameraInfo) -> None:
         self.camera_info = message
 
@@ -205,11 +221,14 @@ class FrameSubscriber(Node):
             return None
         return self._tcp_pose
 
-    def take_latest(self) -> tuple[np.ndarray | None, np.ndarray | None]:
+    def take_latest(self) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
         color = self._latest_color
         depth = self._latest_depth
+        raw_depth = self._latest_raw_depth
         self._latest_color = None
-        return color, depth
+        self._latest_depth = None
+        self._latest_raw_depth = None
+        return color, depth, raw_depth
 
 
 def parse_args() -> argparse.Namespace:
@@ -245,7 +264,14 @@ def parse_args() -> argparse.Namespace:
     parser.add_argument(
         '--controls',
         action='store_true',
-        help='open the camera exposure and control parameters adjustment panel',
+        default=True,
+        help='open the camera exposure and control parameters adjustment panel (default: enabled)',
+    )
+    parser.add_argument(
+        '--no-controls',
+        dest='controls',
+        action='store_false',
+        help='do not open the camera parameters adjustment panel',
     )
     return parser.parse_args()
 
@@ -262,6 +288,7 @@ def select_camera(config: dict[str, Any]) -> CameraSelection:
         kind='camera',
         color_topic='/camera/color/image_raw',
         depth_topic='/camera/aligned_depth_to_color/image_raw',
+        raw_depth_topic='/camera/depth/image_rect_raw',
         info_topic='/camera/color/camera_info',
         parameters=parameters,
     )
@@ -961,6 +988,88 @@ def draw_annotations(
     return annotated
 
 
+def colorize_depth_image(depth: np.ndarray) -> np.ndarray:
+    """Colorize a 2D uint16 mm depth image into a 3-channel uint8 visualization copy.
+
+    Zero depth remains black [0, 0, 0].
+    Does not mutate the input array.
+    """
+    if depth.ndim != 2:
+        raise ValueError(f'Expected 2D depth array, got shape {depth.shape}')
+    h, w = depth.shape
+    colored = np.zeros((h, w, 3), dtype=np.uint8)
+    valid_mask = depth > 0
+    if not np.any(valid_mask):
+        return colored
+    valid_depth = depth[valid_mask]
+    min_d = float(np.min(valid_depth))
+    max_d = float(np.max(valid_depth))
+    if max_d == min_d:
+        norm = np.zeros((h, w), dtype=np.uint8)
+        norm[valid_mask] = 128
+    else:
+        norm = np.zeros((h, w), dtype=np.uint8)
+        scaled = 1.0 + 254.0 * ((depth.astype(np.float32) - min_d) / (max_d - min_d))
+        norm[valid_mask] = np.clip(scaled[valid_mask], 1, 255).astype(np.uint8)
+    colormap_result = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
+    colored[valid_mask] = colormap_result[valid_mask]
+    return colored
+
+
+def draw_depth_view(raw_depth: np.ndarray | None, fps: float) -> np.ndarray:
+    """Render the native 848x480 depth visualization with distance legend and FPS."""
+    if raw_depth is None:
+        view = np.zeros((480, 848, 3), dtype=np.uint8)
+        cv2.putText(
+            view,
+            'Waiting for native depth frames (848x480)...',
+            (40, 240),
+            cv2.FONT_HERSHEY_SIMPLEX,
+            0.7,
+            (0, 255, 255),
+            2,
+            cv2.LINE_AA,
+        )
+        return view
+
+    h, w = raw_depth.shape[:2]
+    colored = colorize_depth_image(raw_depth)
+    valid = raw_depth[raw_depth > 0]
+    if len(valid) == 0:
+        cv2.putText(
+            colored,
+            'No valid depth',
+            (max(10, w // 2 - 120), h // 2),
+            cv2.FONT_HERSHEY_SIMPLEX,
+            0.8,
+            (0, 0, 255),
+            2,
+            cv2.LINE_AA,
+        )
+        legend_text = f'Native Depth ({w}x{h}) | No valid depth pixels'
+    else:
+        min_m = float(np.min(valid)) * 0.001
+        max_m = float(np.max(valid)) * 0.001
+        coverage = (len(valid) / (h * w)) * 100.0
+        legend_text = (
+            f'Native Depth ({w}x{h}) | Near: {min_m:.3f}m  Far: {max_m:.3f}m | '
+            f'Valid: {coverage:.1f}% | FPS: {fps:.1f}'
+        )
+
+    cv2.rectangle(colored, (0, 0), (w, 30), (20, 20, 20), cv2.FILLED)
+    cv2.putText(
+        colored,
+        legend_text,
+        (10, 21),
+        cv2.FONT_HERSHEY_SIMPLEX,
+        0.52,
+        (0, 255, 120),
+        1,
+        cv2.LINE_AA,
+    )
+    return colored
+
+
 class RipenessClassifier:
     """Two-stage crop ripeness classifier using OpenVINO edge models."""
 
@@ -1170,6 +1279,29 @@ class CameraControl:
     value_type: type
 
 
+RGB_CONTROL_LABELS = {
+    'rgb_camera.enable_auto_exposure': 'Auto exposure',
+    'rgb_camera.exposure': 'Exposure',
+    'rgb_camera.gain': 'Gain',
+    'rgb_camera.enable_auto_white_balance': 'Auto white balance',
+    'rgb_camera.white_balance': 'White balance',
+    'rgb_camera.brightness': 'Brightness',
+    'rgb_camera.contrast': 'Contrast',
+    'rgb_camera.saturation': 'Saturation',
+    'rgb_camera.sharpness': 'Sharpness',
+    'rgb_camera.gamma': 'Gamma',
+    'rgb_camera.backlight_compensation': 'Backlight compensation',
+    'rgb_camera.power_line_frequency': 'Power line frequency',
+}
+
+DEPTH_CONTROL_LABELS = {
+    'depth_module.exposure': 'Depth Exposure (us)',
+    'depth_module.gain': 'Depth Gain',
+}
+
+ALL_CAMERA_CONTROL_LABELS = {**RGB_CONTROL_LABELS, **DEPTH_CONTROL_LABELS}
+
+
 class CameraControlPanel:
     def __init__(
         self,
@@ -1191,54 +1323,74 @@ class CameraControlPanel:
         self._variables: dict[str, tk.Variable] = {}
 
         self._root = tk.Tk()
-        self._root.title(f'{camera_name} camera controls')
+        self._root.title('RealSense Camera Controls')
         self._root.resizable(False, True)
         self._root.protocol('WM_DELETE_WINDOW', self.close)
-        frame = ttk.Frame(self._root, padding=10)
-        frame.grid(sticky='nsew')
-        frame.columnconfigure(1, weight=1)
-
-        for row, control in enumerate(controls):
-            ttk.Label(frame, text=control.label).grid(
-                row=row,
-                column=0,
-                padx=(0, 12),
-                pady=3,
-                sticky='w',
-            )
-            variable = self._create_variable(control)
-            self._variables[control.name] = variable
-            widget = self._create_widget(frame, control, variable)
-            widget.grid(row=row, column=1, pady=3, sticky='ew')
 
+        main_frame = ttk.Frame(self._root, padding=10)
+        main_frame.grid(sticky='nsew')
+        main_frame.columnconfigure(0, weight=1)
+
+        # Depth controls group
+        depth_controls = [c for c in controls if c.name in DEPTH_CONTROL_LABELS]
+        if depth_controls:
+            depth_group = ttk.LabelFrame(main_frame, text='Depth Controls', padding=10)
+            depth_group.grid(row=0, column=0, sticky='ew', pady=(0, 8))
+            depth_group.columnconfigure(1, weight=1)
+            for row, control in enumerate(depth_controls):
+                ttk.Label(depth_group, text=control.label).grid(
+                    row=row,
+                    column=0,
+                    padx=(0, 12),
+                    pady=3,
+                    sticky='w',
+                )
+                variable = self._create_variable(control)
+                self._variables[control.name] = variable
+                widget = self._create_widget(depth_group, control, variable)
+                widget.grid(row=row, column=1, pady=3, sticky='ew')
+
+        # RGB controls group
+        rgb_controls = [c for c in controls if c.name in RGB_CONTROL_LABELS]
+        if rgb_controls:
+            rgb_group = ttk.LabelFrame(main_frame, text='RGB Controls', padding=10)
+            rgb_group.grid(row=1, column=0, sticky='ew', pady=(0, 8))
+            rgb_group.columnconfigure(1, weight=1)
+            for row, control in enumerate(rgb_controls):
+                ttk.Label(rgb_group, text=control.label).grid(
+                    row=row,
+                    column=0,
+                    padx=(0, 12),
+                    pady=3,
+                    sticky='w',
+                )
+                variable = self._create_variable(control)
+                self._variables[control.name] = variable
+                widget = self._create_widget(rgb_group, control, variable)
+                widget.grid(row=row, column=1, pady=3, sticky='ew')
+
+        # Status & Save
         self._status = tk.StringVar(value='Ready')
-        ttk.Separator(frame).grid(
-            row=len(controls),
-            column=0,
-            columnspan=2,
-            pady=(8, 6),
-            sticky='ew',
-        )
-        ttk.Label(frame, textvariable=self._status).grid(
-            row=len(controls) + 1,
+        ttk.Separator(main_frame).grid(row=2, column=0, pady=(4, 6), sticky='ew')
+        ttk.Label(main_frame, textvariable=self._status).grid(
+            row=3,
             column=0,
-            columnspan=2,
             sticky='w',
         )
-        ttk.Button(frame, text='Save', command=self._request_save).grid(
-            row=len(controls) + 2,
+        ttk.Button(main_frame, text='Save', command=self._request_save).grid(
+            row=4,
             column=0,
-            columnspan=2,
             pady=(8, 0),
             sticky='ew',
         )
+
         self._root.update_idletasks()
         x = max(0, self._root.winfo_screenwidth() - self._root.winfo_reqwidth() - 20)
-        self._root.geometry(f'+{x}+60')
+        self._root.geometry(f'+{x}+40')
 
     def _create_widget(
         self,
-        parent: ttk.Frame,
+        parent: ttk.Frame | ttk.LabelFrame,
         control: CameraControl,
         variable: tk.Variable,
     ) -> tk.Widget:
@@ -1282,10 +1434,20 @@ class CameraControlPanel:
         return tk.DoubleVar(value=control.value)
 
     def _queue(self, name: str) -> None:
-        auto_name = AUTO_CONTROL_FOR.get(name)
-        if auto_name in self._variables and self._variables[auto_name].get():
-            self._variables[auto_name].set(False)
-            self._pending[auto_name] = False
+        if name in DEPTH_CONTROL_LABELS:
+            # Depth manual controls require depth auto exposure to be disabled
+            try:
+                self._set_control('depth_module.enable_auto_exposure', False)
+            except Exception as error:  # noqa: BLE001
+                message = f'Failed to disable depth auto exposure: {error}'
+                self._status.set(message)
+                print(message, file=sys.stderr, flush=True)
+                return
+        else:
+            auto_name = AUTO_CONTROL_FOR.get(name)
+            if auto_name in self._variables and self._variables[auto_name].get():
+                self._variables[auto_name].set(False)
+                self._pending[auto_name] = False
         value = self._variables[name].get()
         self._pending[name] = self._controls[name].value_type(value)
 
@@ -1359,56 +1521,122 @@ class CameraControlPanel:
 class CameraControlService:
     NODE_NAME = '/camera'
 
-    def __init__(self, node: Node) -> None:
-        self._node = node
-        self._get_client = node.create_client(
+    def __init__(self, node: Node | None = None) -> None:
+        self._node = Node('camera_control_client')
+        self._get_client = self._node.create_client(
             GetParameters,
             f'{self.NODE_NAME}/get_parameters',
         )
-        self._describe_client = node.create_client(
+        self._describe_client = self._node.create_client(
             DescribeParameters,
             f'{self.NODE_NAME}/describe_parameters',
         )
-        self._set_client = node.create_client(
+        self._set_client = self._node.create_client(
             SetParameters,
             f'{self.NODE_NAME}/set_parameters',
         )
+        self._descriptors: dict[str, Any] = {}
 
-    def discover(self) -> list[CameraControl]:
-        for client in (
-            self._get_client,
-            self._describe_client,
-            self._set_client,
-        ):
-            if not client.wait_for_service(timeout_sec=5.0):
+    def destroy(self) -> None:
+        try:
+            self._node.destroy_node()
+        except Exception:
+            pass
+
+    def ensure_descriptors(self, names: list[str], timeout_sec: float = 10.0) -> None:
+        deadline = time.monotonic() + timeout_sec
+        for client in (self._get_client, self._describe_client, self._set_client):
+            if not client.wait_for_service(timeout_sec=timeout_sec):
                 raise RuntimeError(
                     f'Parameter service is unavailable for {self.NODE_NAME}',
                 )
+        while time.monotonic() < deadline:
+            missing = [n for n in names if n not in self._descriptors]
+            if not missing:
+                return
+            describe_response = self._call(
+                self._describe_client,
+                DescribeParameters.Request(names=missing),
+            )
+            for name, desc in zip(missing, describe_response.descriptors):
+                if desc.type != ParameterType.PARAMETER_NOT_SET:
+                    self._descriptors[name] = desc
 
-        names = list(CAMERA_CONTROL_LABELS)
+            remaining = [n for n in names if n not in self._descriptors]
+            if not remaining or ('depth_module.visual_preset' in self._descriptors and 'rgb_camera.exposure' in self._descriptors):
+                break
+            rclpy.spin_once(self._node, timeout_sec=0.2)
+
+    def has_descriptor(self, name: str) -> bool:
+        return name in self._descriptors
+
+    def get_parameter_values(self, names: list[str]) -> dict[str, Any]:
+        self.ensure_descriptors(names)
         get_response = self._call(
             self._get_client,
             GetParameters.Request(names=names),
         )
-        describe_response = self._call(
-            self._describe_client,
-            DescribeParameters.Request(names=names),
+        results: dict[str, Any] = {}
+        for name, val in zip(names, get_response.values):
+            if val.type == ParameterType.PARAMETER_BOOL:
+                results[name] = val.bool_value
+            elif val.type == ParameterType.PARAMETER_INTEGER:
+                results[name] = val.integer_value
+            elif val.type == ParameterType.PARAMETER_DOUBLE:
+                results[name] = val.double_value
+            elif val.type == ParameterType.PARAMETER_STRING:
+                results[name] = val.string_value
+            elif val.type != ParameterType.PARAMETER_NOT_SET:
+                results[name] = val
+        return results
+
+    def discover(self) -> list[CameraControl]:
+        names = list(ALL_CAMERA_CONTROL_LABELS)
+        self.ensure_descriptors(names)
+
+        get_response = self._call(
+            self._get_client,
+            GetParameters.Request(names=names),
         )
 
         controls: list[CameraControl] = []
-        for name, value, descriptor in zip(
-            names,
-            get_response.values,
-            describe_response.descriptors,
-        ):
-            control = self._control_from_parameter(name, value, descriptor)
-            if control is not None:
-                controls.append(control)
+        discovered_names: set[str] = set()
+        for name, value in zip(names, get_response.values):
+            descriptor = self._descriptors.get(name)
+            if descriptor is not None:
+                control = self._control_from_parameter(name, value, descriptor)
+                if control is not None:
+                    controls.append(control)
+                    discovered_names.add(name)
+
+        # Enforce required depth controls
+        required_depth = {'depth_module.exposure', 'depth_module.gain'}
+        missing = required_depth - discovered_names
+        if missing:
+            raise RuntimeError(
+                f'Required depth controls missing from {self.NODE_NAME}: {missing}',
+            )
+
         return controls
 
-    def set(self, name: str, value: bool | int | float) -> None:
+    def set(self, name: str, value: bool | int | float | str) -> None:
+        self.ensure_descriptors([name])
+        desc = self._descriptors.get(name)
+        param_value = value
+        if desc is not None:
+            if desc.type == ParameterType.PARAMETER_INTEGER:
+                param_value = int(round(float(value)))
+            elif desc.type == ParameterType.PARAMETER_DOUBLE:
+                param_value = float(value)
+            elif desc.type == ParameterType.PARAMETER_BOOL:
+                param_value = bool(value)
+            elif desc.type == ParameterType.PARAMETER_STRING:
+                param_value = str(value)
+        elif isinstance(value, float) and value.is_integer() and not isinstance(value, bool):
+            param_value = int(value)
+
         request = SetParameters.Request(
-            parameters=[Parameter(name, value=value).to_parameter_msg()],
+            parameters=[Parameter(name, value=param_value).to_parameter_msg()],
         )
         response = self._call(self._set_client, request)
         result = response.results[0]
@@ -1419,7 +1647,9 @@ class CameraControlService:
         future = client.call_async(request)
         rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
         if not future.done():
-            raise RuntimeError('ROS parameter request timed out')
+            raise RuntimeError(
+                f'ROS parameter request timed out on {getattr(client, "srv_name", "unknown")}: {request}',
+            )
         if future.exception() is not None:
             raise RuntimeError(str(future.exception()))
         return future.result()
@@ -1430,10 +1660,11 @@ class CameraControlService:
         value: Any,
         descriptor: Any,
     ) -> CameraControl | None:
+        label = ALL_CAMERA_CONTROL_LABELS.get(name, name)
         if value.type == ParameterType.PARAMETER_BOOL:
             return CameraControl(
                 name,
-                CAMERA_CONTROL_LABELS[name],
+                label,
                 0.0,
                 1.0,
                 1.0,
@@ -1447,7 +1678,7 @@ class CameraControlService:
             value_range = descriptor.integer_range[0]
             return CameraControl(
                 name,
-                CAMERA_CONTROL_LABELS[name],
+                label,
                 float(value_range.from_value),
                 float(value_range.to_value),
                 float(max(1, value_range.step)),
@@ -1463,7 +1694,7 @@ class CameraControlService:
                 return None
             return CameraControl(
                 name,
-                CAMERA_CONTROL_LABELS[name],
+                label,
                 value_range.from_value,
                 value_range.to_value,
                 value_range.step,
@@ -1489,12 +1720,209 @@ def save_camera_controls(
     if 'camera' not in doc or 'ros__parameters' not in doc['camera']:
         raise KeyError('Invalid config: missing camera.ros__parameters')
     doc['camera']['ros__parameters'].update(values)
-    with config_path.open('w', encoding='utf-8') as f:
+
+    temp_path = config_path.with_suffix(f'.tmp_{timestamp}')
+    with temp_path.open('w', encoding='utf-8') as f:
         yaml.safe_dump(doc, f, sort_keys=False)
+    temp_path.replace(config_path)
 
     return backup_path
 
 
+def apply_camera_yaml_configuration(
+    service: CameraControlService,
+    config: dict[str, Any],
+    subscriber: FrameSubscriber,
+    timeout_sec: float = 10.0,
+) -> None:
+    """Apply YAML configuration to /camera and verify stream resolution and parameters."""
+    params = config['camera']['ros__parameters']
+
+    all_target_names = [
+        'enable_depth',
+        'enable_color',
+        'depth_module.depth_profile',
+        'rgb_camera.color_profile',
+        'depth_module.visual_preset',
+        'depth_module.enable_auto_exposure',
+        'depth_module.exposure',
+        'depth_module.gain',
+        'depth_module.emitter_enabled',
+        'depth_module.laser_power',
+        'rgb_camera.enable_auto_exposure',
+        'rgb_camera.exposure',
+        'rgb_camera.gain',
+        'rgb_camera.enable_auto_white_balance',
+        'rgb_camera.white_balance',
+        'rgb_camera.brightness',
+        'rgb_camera.contrast',
+        'rgb_camera.saturation',
+        'rgb_camera.sharpness',
+        'rgb_camera.gamma',
+        'rgb_camera.backlight_compensation',
+        'rgb_camera.power_line_frequency',
+        'enable_sync',
+        'align_depth.enable',
+        'enable_rgbd',
+        'spatial_filter.enable',
+        'temporal_filter.enable',
+    ]
+    service.ensure_descriptors(all_target_names)
+
+    # 1. Video stream profiles
+    current_values = service.get_parameter_values([
+        'depth_module.depth_profile',
+        'rgb_camera.color_profile',
+    ])
+    target_depth_profile = str(params.get('depth_module.depth_profile', '848x480x30'))
+    target_color_profile = str(params.get('rgb_camera.color_profile', '1280x720x30'))
+
+    cur_depth_prof = current_values.get('depth_module.depth_profile')
+    if cur_depth_prof is not None and cur_depth_prof != target_depth_profile:
+        print(
+            f'Applying depth profile: {cur_depth_prof} -> {target_depth_profile}',
+            flush=True,
+        )
+        if service.has_descriptor('enable_depth'):
+            service.set('enable_depth', False)
+        service.set('depth_module.depth_profile', target_depth_profile)
+        if service.has_descriptor('enable_depth'):
+            service.set('enable_depth', True)
+
+    cur_color_prof = current_values.get('rgb_camera.color_profile')
+    if cur_color_prof is not None and cur_color_prof != target_color_profile:
+        print(
+            f'Applying color profile: {cur_color_prof} -> {target_color_profile}',
+            flush=True,
+        )
+        if service.has_descriptor('enable_color'):
+            service.set('enable_color', False)
+        service.set('rgb_camera.color_profile', target_color_profile)
+        if service.has_descriptor('enable_color'):
+            service.set('enable_color', True)
+
+    def set_if_diff(name: str, target: Any) -> None:
+        if not service.has_descriptor(name):
+            return
+        cur = service.get_parameter_values([name]).get(name)
+        if cur == target:
+            return
+        if isinstance(target, float) and cur is not None and abs(float(cur) - float(target)) < 1e-3:
+            return
+        print(f'Setting camera parameter {name}: {cur} -> {target}', flush=True)
+        service.set(name, target)
+
+    # 2. Sync, aligned depth, rgbd, and filters
+    for key in (
+        'enable_sync',
+        'align_depth.enable',
+        'enable_rgbd',
+        'spatial_filter.enable',
+        'temporal_filter.enable',
+    ):
+        if key in params and service.has_descriptor(key):
+            set_if_diff(key, bool(params[key]))
+
+    # 3. Depth settings: strict order (preset -> auto-exposure off -> manual controls)
+    set_if_diff('depth_module.visual_preset', int(params.get('depth_module.visual_preset', 3)))
+    set_if_diff(
+        'depth_module.enable_auto_exposure',
+        bool(params.get('depth_module.enable_auto_exposure', False)),
+    )
+    if 'depth_module.exposure' in params:
+        set_if_diff('depth_module.exposure', int(params['depth_module.exposure']))
+    if 'depth_module.gain' in params:
+        set_if_diff('depth_module.gain', int(params['depth_module.gain']))
+    if 'depth_module.emitter_enabled' in params:
+        set_if_diff('depth_module.emitter_enabled', int(params['depth_module.emitter_enabled']))
+    if 'depth_module.laser_power' in params:
+        set_if_diff('depth_module.laser_power', float(params['depth_module.laser_power']))
+
+    # 4. RGB settings
+    set_if_diff(
+        'rgb_camera.enable_auto_exposure',
+        bool(params.get('rgb_camera.enable_auto_exposure', False)),
+    )
+    if 'rgb_camera.exposure' in params:
+        set_if_diff('rgb_camera.exposure', int(params['rgb_camera.exposure']))
+    if 'rgb_camera.gain' in params:
+        set_if_diff('rgb_camera.gain', int(params['rgb_camera.gain']))
+    if 'rgb_camera.enable_auto_white_balance' in params:
+        set_if_diff(
+            'rgb_camera.enable_auto_white_balance',
+            bool(params['rgb_camera.enable_auto_white_balance']),
+        )
+    if 'rgb_camera.white_balance' in params:
+        set_if_diff('rgb_camera.white_balance', float(params['rgb_camera.white_balance']))
+
+    for rgb_key in (
+        'rgb_camera.brightness',
+        'rgb_camera.contrast',
+        'rgb_camera.saturation',
+        'rgb_camera.sharpness',
+        'rgb_camera.gamma',
+        'rgb_camera.backlight_compensation',
+        'rgb_camera.power_line_frequency',
+    ):
+        if rgb_key in params and service.has_descriptor(rgb_key):
+            set_if_diff(rgb_key, params[rgb_key])
+
+    # 5. Readback verification
+    verify_names = [
+        'depth_module.visual_preset',
+        'depth_module.enable_auto_exposure',
+        'depth_module.exposure',
+        'depth_module.gain',
+        'depth_module.emitter_enabled',
+        'depth_module.laser_power',
+    ]
+    readback = service.get_parameter_values(verify_names)
+    print(f'Verified depth parameters on /camera: {readback}', flush=True)
+    if readback.get('depth_module.visual_preset') != int(params.get('depth_module.visual_preset', 3)):
+        raise RuntimeError(
+            f'Visual preset verification failed: expected '
+            f'{params.get("depth_module.visual_preset", 3)}, got {readback.get("depth_module.visual_preset")}',
+        )
+    if readback.get('depth_module.enable_auto_exposure') is not False:
+        raise RuntimeError(
+            f'Depth auto-exposure verification failed: expected False, got '
+            f'{readback.get("depth_module.enable_auto_exposure")}',
+        )
+
+    # 6. Wait for confirmed frame dimensions
+    deadline = time.monotonic() + timeout_sec
+    confirmed_color = False
+    confirmed_depth = False
+    while time.monotonic() < deadline:
+        rclpy.spin_once(subscriber, timeout_sec=0.1)
+        c, d, raw_d = subscriber.take_latest()
+        if c is not None and c.shape[:2] == (720, 1280):
+            confirmed_color = True
+        if raw_d is not None and raw_d.shape[:2] == (480, 848):
+            confirmed_depth = True
+        if confirmed_color and confirmed_depth:
+            print(
+                'Camera stream dimensions confirmed: color 1280x720, native depth 848x480',
+                flush=True,
+            )
+            break
+    else:
+        print(
+            'Warning: timeout waiting for confirmed frame dimensions '
+            f'(color: {confirmed_color}, native depth: {confirmed_depth})',
+            file=sys.stderr,
+            flush=True,
+        )
+
+
+def is_window_open(name: str) -> bool:
+    try:
+        val = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
+        return val >= 1.0
+    except Exception:
+        return False
+
+
 def run() -> int:
     args = parse_args()
     config = load_config()
@@ -1524,6 +1952,24 @@ def run() -> int:
 
     rclpy.init()
 
+    # Pre-flight check: reject running if /fruit_mission is active
+    temp_node = Node('test_camera_preflight_check')
+    try:
+        deadline = time.monotonic() + 0.5
+        while time.monotonic() < deadline:
+            rclpy.spin_once(temp_node, timeout_sec=0.1)
+        node_names = set(temp_node.get_node_names())
+        if '/fruit_mission' in node_names or any('fruit_mission' in name for name in node_names):
+            print(
+                'Error: /fruit_mission is currently active. '
+                'Camera tuning is prohibited during an active mission.',
+                file=sys.stderr,
+                flush=True,
+            )
+            return 1
+    finally:
+        temp_node.destroy_node()
+
     joint_state_topic = '/arm_planner_node/joint_states'
     arm_config_path = WORKSPACE / 'src/fruit_arm_planner/config/arm_presets.yaml'
     try:
@@ -1539,6 +1985,7 @@ def run() -> int:
     node = FrameSubscriber(
         selection.color_topic,
         selection.depth_topic,
+        selection.raw_depth_topic,
         selection.info_topic,
         joint_state_topic,
         arm_config_path,
@@ -1550,6 +1997,7 @@ def run() -> int:
     camera_process: subprocess.Popen | None = None
     log_path = TMP_DIR / f'test_camera_detector_{selection.name}.log'
     control_panel = None
+    control_service = None
 
     try:
         owned_arm_debug, owned_bridge, owned_planner = ensure_arm_communication(node)
@@ -1564,14 +2012,30 @@ def run() -> int:
             camera_process, log_path = start_camera(selection)
             print(
                 f'Started RealSense camera; color={selection.color_topic}; '
-                f'depth={selection.depth_topic}; log={log_path}',
+                f'depth={selection.depth_topic}; raw_depth={selection.raw_depth_topic}; log={log_path}',
                 flush=True,
             )
+            print('Waiting for camera node to initialize publishers...', flush=True)
+            if not wait_for_publisher(node, selection.color_topic, 10.0):
+                check_camera_process(camera_process, log_path)
+                raise RuntimeError(
+                    f'Camera process failed to publish on {selection.color_topic} within 10s',
+                )
 
-        window_name = 'RealSense 3D Detector & Depth Inspector'
-        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
-        cv2.resizeWindow(window_name, 1280, 720)
-        cv2.moveWindow(window_name, 20, 40)
+        # Apply YAML configuration to /camera (handles both newly started and reused camera)
+        control_service = CameraControlService(node)
+        apply_camera_yaml_configuration(control_service, config, node)
+
+        window_rgb_name = 'RealSense 3D Detector & Depth Inspector'
+        window_depth_name = 'RealSense Native Depth (848x480)'
+
+        cv2.namedWindow(window_rgb_name, cv2.WINDOW_NORMAL)
+        cv2.resizeWindow(window_rgb_name, 1280, 720)
+        cv2.moveWindow(window_rgb_name, 20, 40)
+
+        cv2.namedWindow(window_depth_name, cv2.WINDOW_NORMAL)
+        cv2.resizeWindow(window_depth_name, 848, 480)
+        cv2.moveWindow(window_depth_name, 20, 800)
 
         mouse_coords = [-1, -1]
 
@@ -1579,11 +2043,26 @@ def run() -> int:
             mouse_coords[0] = x
             mouse_coords[1] = y
 
-        cv2.setMouseCallback(window_name, on_mouse)
+        cv2.setMouseCallback(window_rgb_name, on_mouse)
+
+        # Show initial placeholder frames on both windows
+        init_rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
+        cv2.putText(
+            init_rgb,
+            'Waiting for camera frames...',
+            (40, 360),
+            cv2.FONT_HERSHEY_SIMPLEX,
+            1.0,
+            (0, 255, 255),
+            2,
+            cv2.LINE_AA,
+        )
+        cv2.imshow(window_rgb_name, init_rgb)
+        cv2.imshow(window_depth_name, draw_depth_view(None, 0.0))
+        cv2.waitKey(1)
 
         if args.controls:
             try:
-                control_service = CameraControlService(node)
                 control_panel = CameraControlPanel(
                     selection.name,
                     control_service.discover(),
@@ -1599,6 +2078,7 @@ def run() -> int:
         displayed_fps = 0.0
         last_wait_message = 0.0
         announced_first_frame = False
+        current_raw_depth: np.ndarray | None = None
 
         print('\n=== RealSense 3D Inspector Controls ===')
         print('  [Hover Mouse] : Probe depth and 3D camera / arm base coordinates')
@@ -1612,7 +2092,21 @@ def run() -> int:
                 control_panel.apply_pending()
             rclpy.spin_once(node, timeout_sec=0.01)
 
-            frame, depth_frame = node.take_latest()
+            # Check if any window was closed by user
+            if not is_window_open(window_rgb_name):
+                print('RGB detection window closed by user; exiting.', flush=True)
+                break
+            if not is_window_open(window_depth_name):
+                print('Native depth window closed by user; exiting.', flush=True)
+                break
+            if control_panel is not None and not control_panel.is_open():
+                print('Camera controls panel closed by user; exiting.', flush=True)
+                break
+
+            frame, depth_frame, raw_depth = node.take_latest()
+            if raw_depth is not None:
+                current_raw_depth = raw_depth
+
             if frame is None:
                 now = time.monotonic()
                 if not announced_first_frame and now - last_wait_message > 2.0:
@@ -1626,7 +2120,8 @@ def run() -> int:
             if not announced_first_frame:
                 print(
                     f'Received first frame: {frame.shape[1]}x{frame.shape[0]} '
-                    f'(depth available: {depth_frame is not None})',
+                    f'(aligned depth: {depth_frame is not None}, '
+                    f'native depth: {current_raw_depth is not None})',
                     flush=True,
                 )
                 announced_first_frame = True
@@ -1672,7 +2167,11 @@ def run() -> int:
                 ripeness_classifier=ripeness_classifier,
             )
             annotated_bgr = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
-            cv2.imshow(window_name, annotated_bgr)
+            cv2.imshow(window_rgb_name, annotated_bgr)
+
+            # Native depth preview window
+            depth_view = draw_depth_view(current_raw_depth, displayed_fps)
+            cv2.imshow(window_depth_name, depth_view)
 
             key = cv2.waitKey(1)
             if key in (27, ord('q'), ord('Q')):
@@ -1691,6 +2190,8 @@ def run() -> int:
     finally:
         if control_panel is not None:
             control_panel.close()
+        if control_service is not None:
+            control_service.destroy()
         try:
             node.destroy_node()
         except Exception:  # noqa: BLE001

启动前 status：
 M docs/CAMERA_TUNING.md
 M docs/PROJECT_ARCHITECTURE.md
 M src/fruit_arm_planner/config/arm_presets.yaml
 M src/fruit_perception/README.md
 M src/fruit_perception/config/fruit_perception.yaml
 M test_camera_detector.py
?? .agents/skills/wisadel-helium-voice/SKILL.md
?? .pi/skills/wisadel-helium-voice/SKILL.md
?? ce1adf5a259fbc6f8ffcf4c76b95ef9c.png
?? image.png

