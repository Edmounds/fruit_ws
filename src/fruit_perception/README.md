# fruit_perception

ROS 2 Humble C++ fruit detection using OpenVINO 2024.6 on Intel `GPU.0`.
The species detectors and selected ripeness classifiers are initialized once
from the configured source-directory `model_root`.
The system uses exclusively the depth camera (`camera`), with no side USB cameras.
Camera streams (Color, Aligned Depth, RGBD) remain continuously active and models
run continuous inference on incoming frames. Target evidence accumulation and
harvest target output are strictly gated by task observation windows.

`camera` denotes the depth camera (currently Intel RealSense D435i). The driver node
is `/camera` in the root namespace, with native topics under `/camera/*`.
Project configuration uses `camera`, `enable_camera`, and `camera_*` keys.
The legacy `camera_mode_manager` node and hardware stream stop/start requests have been removed.

The `camera` component and `FruitPerceptionNode` run in one
`component_container` with intra-process communication enabled. This keeps raw
images out of the inter-process DDS transport path. The generated standalone
`fruit_perception_node` executable remains available for isolated debugging.

## Observation windows

Perception and QR decoding are governed by timestamped observation windows published
on `/fruit_mission/observation_window` as JSON strings:

```json
{
  "window_id": "<uuid-string>",
  "valid": 1,
  "start_sec": 1712345678,
  "start_nanosec": 123456789,
  "area": "A",
  "purpose": "harvest",
  "expected_type": "ripe"
}
```

- `valid`: strictly integer `1` (open window) or `0` (close window).
- `start_sec` / `start_nanosec`: image capture cutoff timestamp (ROS time). Frames with capture stamps earlier than this cutoff are ignored for evidence accumulation.
- `area`: `"A"`, `"B"`, or `"C"`. Observation windows do not accept D.
- `purpose`: `"harvest"` (accumulate pick evidence) or `"qr"` (QR scanning only).
- `expected_type`: target filter string (e.g. `"ripe"` or species label; empty string disables filtering).

When no harvest window is open or a frame was captured before window start, the node
continues to publish 2D/3D raw detections and annotated images for diagnostics, but
`observation_window_id` is empty (`""`), `has_target` is false, and `targets_ready` is false.
Target locking and candidate accumulation only occur within a valid harvest window.

Closing a window (`valid: 0`) immediately revokes evidence accumulation and resets selectors.

The camera node synchronizes color and depth, aligns depth to color, and publishes
both aligned images in one RGBD message (`/camera/rgbd`). Color profile is configured to
`1280x720x30` (`RGB8`). Native depth profile is configured to `848x480x30` (`Z16`) with
`depth_module.visual_preset: 3` (High Accuracy), disabled depth auto exposure,
manual depth exposure (`10589` usec), gain (`16`), laser emitter (`1`), laser power (`150.0`),
and post-processing filters (`spatial_filter.enable: true`, `temporal_filter.enable: true`).
Aligned depth (`/camera/aligned_depth_to_color/image_raw`) and `/camera/rgbd` remain in
the `1280x720` color pixel coordinate frame for accurate 2D/3D projection.

Each RGBD callback also publishes only its `std_msgs/Header` on
`/fruit_perception/camera/rgbd_heartbeat`. The external debug monitor combines
that lightweight heartbeat with the camera metadata topics to report
actual rates without subscribing to the RGBD message outside the container.

## Runtime

The project launch starts the camera component, OpenVINO detector, RobViz image
bridge, and serial bridge together. Perception results remain on ROS topics for
the mission node and are not forwarded directly to the lower controller. The
serial device is fixed to `/dev/ttyACM0`.

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

colcon build --packages-up-to fruit_robviz_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch fruit_perception fruit_perception.launch.py
```

The RobViz bridge component runs in the same composable container as the camera
and detector. Do not launch `fruit_robviz_bridge.launch.py` separately when this
perception launch is active.

It publishes JPEG previews on `/fruit_perception/camera/annotated_image/compressed`
and `/fruit_perception/camera/native_depth/compressed`. RGB annotations include
valid measured coordinates. Invalid, stale, moving, or unavailable base
coordinates are rendered as `N/A`. Image settings come from RobViz over the
explicit `/fruit_robviz_bridge/reload_image_settings` service.

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

The external debug monitor expects active camera streams whenever perception is running;
missing streams are reported as `STALE`, not `IDLE`.

The decision stream on `/fruit_perception/decision` contains:

- header matching aligned depth
- `observation_window_id`: active harvest window ID, or empty string when outside window
- detected fruit `types` in confidence order
- all detections in confidence order (`detections`), including ROI and optional 3D points
- `has_target` and `target_point` when single-target lock is acquired within a valid window (Areas A/C), or when targets are ready (Area B/D)
- B/D zone `targets_ready` and up to two aggregated `targets`
- annotated RGB8 image on `/fruit_perception/camera/annotated_image`

The target point is the `target_point` field in the decision message and uses
the camera color optical frame from the message header. Coordinates are in meters: `x` points right in the image, `y`
points down, and `z` points forward.

The annotated image keeps the camera's original aspect ratio. Inference uses a
centered 640x640 letterbox with value 114, so no source pixels are cropped. Each
detection is mapped back to the source image and drawn with its model class and
confidence.

## Short-window target confirmation

Target selection no longer waits for fixed 35/50-frame batches. Source YAML provides
starting values, **not measured calibration or a one-second latency guarantee**:

| Parameter | Starting value | Meaning |
| --- | --- | --- |
| `camera_target_acquisition_frames` | 8 | A/C rolling evidence window in processed frames |
| `camera_b_target_acquisition_frames` | 8 | B rolling window and minimum tree-observation warmup |
| `camera_target_min_observations` | 6 | Minimum supporting observations of one species/maturity label |
| `camera_target_type_support_ratio` | 0.75 | Minimum supporting fraction of that track's observations |
| `camera_target_max_missed_frames` | 2 | Tolerated consecutive missing frames during confirmation |
| `camera_target_maturity_confidence_threshold` | 0.70 | Minimum classifier probability for a supporting observation |

The last four parameters apply to both selectors. `min_observations` must fit both
windows; support ratio must be greater than 0.5 and at most 1, missing-frame count
must be smaller than both windows, and maturity threshold must be in [0.5, 1].

Association remains spatial using the existing radius. All localized classified
candidates enter the evidence window, including wrong species, unripe and uncertain
maturity observations. Species is retained internally even in A, while public A
labels remain `ripe/unripe`. Low-confidence maturity observations count against
consistency but do not support any label. Detector confidence stays separate from
classifier probability; the existing detector threshold remains 0.50.

A/C can confirm as soon as enough evidence exists: the dominant label must be ripe
and match the expected type when supplied. The current observation must support
that label, so a stale, absent or currently conflicting candidate cannot trigger
confirmation. Coordinate aggregation uses only supporting observations, retaining
the existing outlier math and single-pixel depth algorithm. After locking, matching
frames repeat the frozen aggregate point; missed/conflicting frames publish no
single target, and the existing eight-frame lock-loss threshold is unchanged.
Matching uses the frozen aggregate as its spatial anchor, so successive small
displacements cannot keep republishing an arbitrarily old location.

B observes once per tree. After the short warmup, all live candidates must be
confirmed and currently visible; conflicting confirmed species keep observation
pending rather than selecting a species from a single high score. Both ripe and
unripe labels need evidence. Up to two confirmed targets are frozen together and
repeated until the next observation reset. A fully empty window can return ready
with no targets; uncertain tracks remain pending until confirmation, expiry or
the client's existing per-operation timeout. No additional observation retries
are introduced.

At an actual processing rate F, clean A/C needs roughly 6/F seconds and clean B
roughly 8/F seconds, plus capture/startup/transport costs. Occasional misses or
label conflicts can extend this. Changes require rebuilding perception and
restarting its container; source YAML is read at startup. Measure with
`perception_log_level:=info` before claiming field latency/accuracy. Motion safety,
standstill confirmation and client timeouts are unchanged.
