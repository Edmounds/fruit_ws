# fruit_perception

ROS 2 Humble C++ fruit detection using OpenVINO 2024.6 on Intel `GPU.0`.
All enabled camera components and A/B/C model infer requests are created once.
The default runtime mode is `OFF`; mission commands select which camera group
and consumer are active without reloading any model.

The enabled camera components and `FruitPerceptionNode` run in one
`component_container` with intra-process communication enabled. This keeps raw
images out of the inter-process DDS transport path. The generated standalone
`fruit_perception_node` executable remains available for isolated debugging.

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

Each USB camera filters its `/fruit_perception/left/types` or
`/fruit_perception/right/types` output independently. The node groups same-type
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

The D435i topics are:

- synchronized aligned input: `/camera/camera/rgbd`
- fruit types in confidence order: `/fruit_perception/d435i/types`
- annotated RGB8 image: `/fruit_perception/d435i/annotated_image`
- selected arm target center: `/fruit_perception/d435i/target_point`
- all detections in confidence order: `/fruit_perception/d435i/detections`

The detection message contains every model result above the configured
threshold after NMS, in confidence order. Each item includes the model type,
confidence, ROI in source-image coordinates, `has_point`, and a
camera optical-frame point. Invalid center depth keeps the 2D detection with
`has_point=false`.

The target point is a `geometry_msgs/msg/PointStamped` in the D435i color
optical frame. Coordinates are meters: `x` points right in the image, `y`
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

The winning physical target is locked, and its filtered 35-frame mean is
published exactly once on `target_point`. While locked, matching detections only
maintain the lock; their raw per-frame points are not published. A frame that
misses the locked target does not substitute another target. The node clears
the lock and starts a new acquisition only after the configured number of
consecutive misses. TREE-mode and current-area changes also reset the selection
state. With the default configuration, acquisition uses exactly 35 frames,
associates observations within a 3 cm radius, applies a 2-sigma outlier filter,
and releases a lock after 8 missed frames.

The target point published when the lock is acquired is the filtered mean from
that winning 35-frame track. Arm-target consumers wait for one fresh finite
`target_point` and forward it as `ArmTarget`; they do not collect or average a
second point window.

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
