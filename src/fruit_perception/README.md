# fruit_perception

ROS 2 Humble C++ fruit detection using OpenVINO 2024.6 on Intel `GPU.0`.
The species detectors and selected ripeness classifiers are initialized once
from the configured source-directory `model_root`.
The system uses exclusively the depth camera (`camera`), with no side USB cameras.
The default runtime mode is `OFF`; mission commands select when detection or QR scanning is active.

`camera` denotes the depth camera (currently Intel RealSense D435i). The driver node
is `/camera` in the root namespace, with native topics and parameter services
under `/camera/*`. Project configuration uses `camera`, `enable_camera`, and
`camera_*` keys.

The `camera` component and `FruitPerceptionNode` run in one
`component_container` with intra-process communication enabled. This keeps raw
images out of the inter-process DDS transport path. The generated standalone
`fruit_perception_node` executable remains available for isolated debugging.

## Runtime modes

Publish one of the three uppercase values to `/fruit_mission/perception_mode`:

- `OFF`: stop camera capture and inference.
- `QR`: capture camera RGBD; only the QR planner consumes color frames.
- `DETECT`: camera RGBD acquisition and fruit detection active.

When testing from the ROS 2 CLI, quote every string value:

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'OFF'}"
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'DETECT'}"
```

Successful transitions are published on `/fruit_perception/active_mode` with
Reliable, Transient Local QoS. Invalid commands and camera service failures do
not change or republish the active mode. `QR <-> DETECT` transitions only switch
inference consumers without restarting the camera streams.

The RealSense parameter services appear before the device initialization thread
declares `enable_color` and `enable_depth`. At startup, the camera mode manager
polls both values through `get_parameters` and only begins its `OFF`
reset after both are declared as Boolean parameters.

The camera node synchronizes color and depth, aligns depth to color, and publishes
both aligned images in one RGBD message (`/camera/rgbd`).

Each RGBD callback also publishes only its `std_msgs/Header` on
`/fruit_perception/camera/rgbd_heartbeat`. The external debug monitor combines
that lightweight heartbeat with the camera metadata topics to report
actual rates without subscribing to the RGBD message outside the container.

## Runtime

The project launch starts the camera component, OpenVINO detector, and serial
bridge together. Perception results remain on ROS topics for the mission
node and are not forwarded directly to the lower controller. The serial device
is fixed to `/dev/ttyACM0`.

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

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

The detector chooses its model and candidate selection from the current
`/fruit_mission/current_area` string:

- Area `A`: `vegetable_type/best.onnx` plus ripeness classification; publishes
  bare `ripe/unripe` labels. Only `ripe` enters single-target locking.
- Area `B`: `fruit_type/best.onnx` plus ripeness classification; publishes
  `apple_ripe/apple_unripe/pear_ripe/pear_unripe`, with up to 2 stable candidates.
- Area `C`: the same vegetable detector as A, plus ripeness classification;
  publishes species/maturity labels and locks mature targets matching the expected
  type. Model-internal `pepper` is normalized to business label `chili` **after**
  selecting and running the pepper ripeness classifier.
- Area `D`: reuses B's detector, ripeness classification and multi-target selection.

The external debug monitor expects image streams in both `QR` and `DETECT`;
missing streams are reported as `STALE`, not `IDLE`.

The decision stream on `/fruit_perception/decision` contains:

- header matching aligned depth
- detected fruit `types` in confidence order
- all detections in confidence order (`detections`), including ROI and optional 3D points
- `has_target` and `target_point` when single-target lock is acquired (Areas A/C) or when targets are ready (Area B/D)
- B/D zone `targets_ready` and up to two aggregated `targets`
- annotated RGB8 image on `/fruit_perception/camera/annotated_image`

The target point is the `target_point` field in the decision message and uses
the camera color optical frame from the message header. Coordinates are in meters: `x` points right in the image, `y`
points down, and `z` points forward.

The annotated image keeps the camera's original aspect ratio. Inference uses a
centered 640x640 letterbox with value 114, so no source pixels are cropped. Each
detection is mapped back to the source image and drawn with its model class and
confidence.
