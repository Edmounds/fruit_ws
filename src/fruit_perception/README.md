# fruit_perception

ROS 2 Humble C++ fruit detection using OpenVINO 2024.6 on Intel `GPU.0`.
All three camera components and all three infer requests are created once. The
default runtime mode is `OFF`; mission commands select which camera group and
consumer are active without reloading the model.

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
source /opt/ros/humble/setup.bash
colcon build --packages-select fruit_perception \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch fruit_perception fruit_perception.launch.py
```

For normal operation, use the workspace entry point:

```bash
/home/rc4/fruit_ws/start_fruit_perception.sh
```

The installed `fruit-perception.service` invokes this script automatically at
boot.

The D435i topics are:

- synchronized aligned input: `/camera/camera/rgbd`
- highest-confidence fruit type: `/fruit_perception/d435i/types`
- annotated RGB8 image: `/fruit_perception/d435i/annotated_image`
- highest-confidence detection center: `/fruit_perception/d435i/target_point`
- highest-confidence detection in the frame: `/fruit_perception/d435i/detections`

The detection message contains at most one item: the highest-confidence model
result above the configured threshold. It includes the model type, confidence,
ROI in original 1920x1080 color coordinates, `has_point`, and a camera
optical-frame point. Invalid center depth keeps the 2D detection with
`has_point=false`.

The legacy target point is a `geometry_msgs/msg/PointStamped` in the D435i color
optical frame. Coordinates are meters: `x` points right in the image, `y`
points down, and `z` points forward. The point uses the depth value at the
center pixel of the highest-confidence detection box. No point is published
when that center pixel has invalid zero depth.

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

The installed model was generated by Ultralytics 8.3.239 with:

```bash
yolo export model=best.pt format=openvino imgsz=640 half=True nms=True \
  conf=0.25 iou=0.7 device=cpu
```

Its generated metadata and AGPL-3.0 license declaration are retained in
`model/metadata.yaml`.
