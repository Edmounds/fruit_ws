# fruit_perception

ROS 2 Humble C++ fruit detection using OpenVINO 2024.6 on Intel `GPU.0`.
All enabled camera components and A/B/C model infer requests are created once.
The default runtime mode is `OFF`; mission commands select which camera group
and consumer are active without reloading any model.

The enabled camera components and `FruitPerceptionNode` run in one
`component_container` with intra-process communication enabled. This keeps raw
images out of the inter-process DDS transport path. The generated standalone
`fruit_perception_node` executable remains available for isolated debugging.
The left and right `usb_cam` components independently remap their `compressed`,
`compressedDepth`, and `theora` transport topics below their own camera
namespaces; they never share a global `/image_raw/*` transport topic.

## Runtime modes

Publish one of the four uppercase values to `/fruit_mission/perception_mode`:

- `OFF`: stop all camera capture and inference.
- `SIDES`: capture and infer only `/dev/left_cam` and `/dev/right_cam`.
- `QR`: capture D435i RGBD; only the QR planner consumes color frames.
- `TREE`: keep the same D435i streams running; only D435i fruit RGBD inference runs.

When testing from the ROS 2 CLI, quote every string value. In particular, YAML
interprets an unquoted `OFF` as a boolean instead of `std_msgs/msg/String`:

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'OFF'}"
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'SIDES'}"
```

Successful transitions are published on `/fruit_perception/active_mode` with
Reliable, Transient Local QoS. Invalid commands and camera service failures do
not change or republish the active mode. Hardware transitions stop the outgoing
camera group before starting the incoming group. `QR <-> TREE` only changes the
consumer and does not restart the D435i.

The RealSense parameter services appear before the device initialization thread
declares `enable_color` and `enable_depth`. At startup, the camera mode manager
therefore polls both values through `get_parameters` and only begins its `OFF`
reset after both are declared as Boolean parameters. A transient failure during
that initial reset returns to the readiness check; failures after an active mode
has been established keep the current mode and are not retried automatically.

The installed `usb_cam 0.8.1` performs `set_capture` correctly but returns
`success=false` with the messages `Start Capturing` or `Stop Capturing`. The
mode manager accepts those two observed responses as successful; any other
false response still aborts the transition without updating `active_mode`.

The installed D435i serial is configured under `d435i_camera`. The RealSense
node configures native `1920x1080@30 RGB8` color and native
`1280x720@30 Z16` depth, synchronizes them, aligns depth to color, and publishes
both aligned images at 1920x1080 in one RGBD message. The aligned depth image has
1920x1080 pixels, but its measurements still originate from the native 1280x720
depth stream. Infrared, IMU,
point cloud, and TF output remain disabled.

Each RGBD callback also publishes only its `std_msgs/Header` on
`/fruit_perception/d435i/rgbd_heartbeat`. The external debug monitor combines
that lightweight heartbeat with the D435i color/depth metadata topics to report
actual rates and last-frame times without subscribing to and serializing the
multi-megabyte RGBD message outside the component container.

These settings follow the RealSense ROS RGBD topic requirements:
https://github.com/realsenseai/realsense-ros#rgbd-topic

## Runtime

The project launch starts the D435i component, OpenVINO detector, and serial
bridge together. Perception results remain on ROS topics for the future mission
node and are not forwarded directly to the lower controller. The serial device
is fixed to `/dev/ttyACM0`.

```bash
source /home/rc4/fruit_ws/setup_fruit_runtime.bash
colcon build --packages-select fruit_perception \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch fruit_perception fruit_perception.launch.py
```

For normal operation, use the workspace entry point:

```bash
/home/rc4/fruit_ws/start_fruit_perception.sh
```

The installed `fruit-perception.service` invokes this script automatically at
boot.

The detector chooses its model strictly from the current
`/fruit_mission/current_area` string:

- Area `A`: `model/A.onnx`, two maturity classes only: `ripe` and `unripe`.
- Area `B`: `model/B.onnx`, apple/pear tree-fruit classes.
- Area `C`: `model/C.onnx`, side-fruit classes.

The left and right USB cameras create A and C infer requests. `SIDES` mode uses
the A request in Area A and the C request in Area C. The D435i creates A, B, and
C infer requests; `TREE` mode uses the request matching the current area. There
is no fallback where every non-B area silently uses the C model.

Each USB camera publishes its source-tagged result on
`/fruit_perception/decision`. The node groups same-type
detections across a complete 3-second window when their source-image ROIs
overlap by at least the configured IoU threshold. There is no minimum
observation-count gate. Tracks use the same median-confidence and
horizontal-centrality 70%/30% score as the D435i selector; near-tied scores are
decided by median confidence. Once a track is selected, the topic contains only
that locked type while the matching ROI is present. During acquisition or a
lock miss it publishes an empty type list, not another visible target's type.
Annotated images continue to show all per-frame detections for diagnosis. A
repeated `SIDES` request and a current-area change reset both side selectors, so
each formal A/C observation starts a new window. The mission-side wait includes
one extra second so the inference that closes the three-second selection window
can publish its result.

The D435i decision stream on `/fruit_perception/decision` contains:

- synchronized aligned input: `/camera/camera/rgbd`
- `source=SOURCE_D435I` and fruit types in confidence order
- all detections in confidence order, including ROI and optional 3D points
- `has_target` and `target_point` only when the selected target lock is acquired
- B-zone `targets_ready` and up to two aggregated `targets` after the configured
  50-frame window; B targets are independent physical tracks from one inferred
  tree-fruit species and may differ in maturity, while mission selects at most
  one target for picking
- annotated RGB8 image: `/fruit_perception/d435i/annotated_image`

The detection message contains every model result above the configured
threshold after NMS, in confidence order. Each item includes the model type,
confidence, ROI in source-image coordinates, `has_point`, and a
camera optical-frame point. Invalid center depth keeps the 2D detection with
`has_point=false`.

The target point is the `target_point` field in the decision message and uses
the D435i color optical frame from the message header. Coordinates are meters: `x` points right in the image, `y`
points down, and `z` points forward. Candidates must have a valid center-depth
point. Before publishing, the node observes the complete configured acquisition
window and associates valid detections into physical-target tracks by their 3D
distance. There is no minimum observation-count gate for a track. Each track is
scored as
`confidence_weight * median_confidence + (1 - confidence_weight) *
median_horizontal_centrality`. Horizontal centrality ignores ROI y and is one
minus the normalized distance of the ROI center from the image center along the
image x axis. Tracks within `d435i_target_score_close_threshold` of the highest
score are treated as a near tie, and median confidence decides first. After the
winner is chosen, coordinate observations farther than the configured sigma
limit on any axis are removed. The mean and per-axis standard deviation are
recomputed from the remaining observations. Standard deviation is diagnostic
only and never prevents the highest-scoring target from being published.

The A/C winning physical target is locked, and its filtered 35-frame mean is
published exactly once with `has_target=true`. B uses a separate selector that
collects exactly 50 frames and removes 2-sigma coordinate outliers independently
for each physical track. The highest-median-confidence stable track determines
whether the tree is apple or pear; stable tracks from the other species are
excluded before at most two `targets` are published repeatedly after
`targets_ready=true`. Per-frame `types`, `detections`, and annotated images keep
all raw model results for diagnosis. B association and species selection do not
use the mission's expected fruit type. TREE-mode and current-area changes reset
both selectors.

The target point published when the lock is acquired is the filtered mean from
that winning track. Arm-target consumers wait for one fresh finite
decision with `has_target=true` and forward it as `ArmTarget`; they do not
collect or average a second point window. In B, QR expectations are deliberately
not applied in the perception node. Mission compares the inferred species with
the QR plan, filters ripe targets, then selects at most one using `abs(x) - y`
in the optical frame.

The annotated image keeps the camera's original aspect ratio. Inference uses a
centered 640x640 letterbox with value 114, so no source pixels are cropped. Each
detection is mapped back to the source image and drawn with its model class and
confidence. The top-left FPS is the smoothed annotated inference output rate.
The annotated image is published as `rgb8` and only rendered when the topic has
at least one subscriber, so it costs nothing during normal competition runs.

Host-side preprocessing is limited to letterboxing the `rgb8` frame into a
reusable u8 buffer. The u8 -> f32 conversion, /255 scaling, and NHWC -> NCHW
transpose are folded into the compiled model via
`ov::preprocess::PrePostProcessor` and run on the GPU.

The node intentionally compiles only for `GPU`; startup fails if the actual
execution device is not exactly `GPU.0`.

The implemented competition contract is `A/SIDES -> B/QR -> C/QR -> C/SIDES ->
B/TREE -> OFF`. This package provides the mode, QR plan, and perception topics;
the repository still has no `fruit_mission` package, so navigation and the full
mission state machine are outside this implementation.

The previous rollback model was generated by Ultralytics 8.3.239 with:

```bash
yolo export model=best.pt format=openvino imgsz=640 half=True nms=True \
  conf=0.25 iou=0.7 device=cpu
```

Its generated metadata and AGPL-3.0 license declaration are retained in
`model/metadata.yaml`. The runtime models are `model/A.onnx` for Area A
maturity-only detection, `model/B.onnx` for B-zone D435i tree-fruit detection,
and `model/C.onnx` for C-zone side-fruit detection.

## B-zone model export

The active `model/B.onnx` was exported from the workspace-root
`fruit_apple_pear.pt` checkpoint with Ultralytics 8.4.126 in the isolated
non-ROS environment `/home/rc4/fruit_model_export_env/.venv`:

```bash
mkdir -p /home/rc4/fruit_ws/tmp/b_model_export
cp /home/rc4/fruit_ws/fruit_apple_pear.pt \
  /home/rc4/fruit_ws/tmp/b_model_export/fruit_apple_pear.pt
env -u PYTHONPATH /home/rc4/fruit_model_export_env/.venv/bin/yolo export \
  model=/home/rc4/fruit_ws/tmp/b_model_export/fruit_apple_pear.pt \
  format=onnx imgsz=640 \
  batch=1 half=False dynamic=False simplify=True opset=12 nms=False device=cpu
```

The ONNX interface is `float32 [1,3,640,640]` input and raw
`float32 [1,8,8400]` output. OpenVINO 2024.6 reads this ONNX model directly,
folds the u8 NHWC preprocessing into the compiled graph, and runs it on
`GPU.0`. NMS remains in the C++ detection parser. The model class order is:

| Class ID | Name |
| ---: | --- |
| 0 | `apple_ripe` |
| 1 | `apple_unripe` |
| 2 | `pear_ripe` |
| 3 | `pear_unripe` |

`model/B_old.onnx` is the byte-for-byte backup of the previous B-zone model
(SHA-256 `8a60fc5737592d506f892949a889b3bca876d459fd3cc71634c870fab8cf19ea`).
It is retained only for manual rollback and is never loaded automatically.
