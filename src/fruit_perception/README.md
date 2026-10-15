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
Candidate voting only occurs within a valid harvest window. Duplicate RGB capture stamps do not count as new frames.

Closing a window (`valid: 0`) immediately revokes evidence accumulation and resets selectors.
The window's `expected_type` is the only target-filter input.
The duplicate `/fruit_mission/expected_fruit_type` topic and unused `FruitTypes.msg` are removed.

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

`fruit_perception_geometry` installs shared pure camera/target geometry for mission and RobViz.
It does not initialize ROS nodes or control the arm.

The project launch starts the camera component, OpenVINO detector, and QR decoding only.
It does not start the serial bridge or RobViz.
Perception results remain on ROS topics for the mission node.

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

colcon build --packages-up-to fruit_robviz_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch fruit_perception fruit_perception.launch.py
```

`start_fruit_perception.sh` starts RobViz through its own launch, or reuses an existing instance.
If its owned RobViz process exits while perception continues, the script reports the exit status
and keeps the perception process running.
`fruit_mission.launch.py` composes the complete competition stack.
Neither the perception launch nor its script starts a serial bridge.
The camera-arm debug script separately starts or reuses the bridge.

It publishes JPEG previews on `/fruit_perception/camera/annotated_image/compressed`
and `/fruit_perception/camera/native_depth/compressed`. The RGB stream forwards
the perception node's annotated image without redundant duplicate overlays.
Image settings are fetched from RobViz by the configuration client at
startup and applied via atomic parameter updates.

For headless debugging, specify the initial zone explicitly:

```bash
/home/rc4/fruit_ws/start_fruit_perception.sh --zone A
# Use --zone B for fruit, --zone C for vegetables.
# Runtime switch, without restarting the camera:
ros2 param set /fruit_perception_node zone B
```

Use `start_fruit_perception.sh --zone A --debug` to capture depth failures.
The perception recording profile always requests native depth
(`/camera/depth/image_rect_raw`), aligned depth
(`/camera/aligned_depth_to_color/image_raw`), RGBD heartbeat
(`/fruit_perception/camera/rgbd_heartbeat`), and `/parameter_events`.
Recording does not require these publishers to exist at startup.
The script saves actual `/camera` ROS parameters after recorder startup to
`log/runtime/latest/camera_params.yaml`, with a maximum 10-second snapshot attempt.
Snapshot failures produce a warning and `camera_params.log`; perception continues.
The snapshot and recorded parameter events capture startup values and subsequent changes.
The mission debug profile records the same depth diagnostics.
Non-debug perception startup is unchanged.

The service template supplies `--zone A`. Update an existing installed unit before
using the new script. Do not restart camera services during an active mission.
`start_fruit_mission.sh` uses the complete mission launch, not this debug script.
Mission clients set `zone` through `SetParametersAtomically` and
check the response before observation. The mission area topic remains state only.

The validated string parameter `zone` selects the model and candidate selection.
Launch accepts `zone:=A|B|C`, defaulting to the source YAML value (`A`).
Perception no longer subscribes to `/fruit_mission/current_area`.
Observation windows do not select models. A harvest window must match `zone`.
Changing `zone` closes the active window and discards old in-flight evidence.
Open a new matching window after a successful parameter response.

- Area `A`: `vegetable_type/best.onnx` plus ripeness classification; retains species/maturity labels for voting and diagnostics.
- Area `B`: `fruit_type/best.onnx` plus ripeness classification; publishes
  `apple_ripe/apple_unripe/pear_ripe/pear_unripe` and diagnostic `bad`,
  with independently confirmed candidates.
- Area `C`: the same vegetable detector as A, plus ripeness classification;
  publishes species/maturity labels. Model-internal `pepper` is normalized to business label `chili` **after**
  selecting and running the pepper ripeness classifier.

Camera startup uses `initial_reset: true`. This resets device state, but does not
guarantee recovery from a physical MIPI, cable, or USB fault.

The species detectors were updated from workspace `models/` exports dated 2026-10-10.
Fruit species indices are `0=apple`, `1=pear`, `2=bad`.
Vegetable species indices remain `0=bad`, `1=onion`, `2=pepper`, `3=pumpkin`, `4=tomato`.
Ripeness models remain unchanged.
GPU kernel compilation uses one thread to avoid the reproduced Intel OpenCL
LLVM/SPIR-V startup crash with the updated detectors and Edge classifiers.
This setting does not switch inference to CPU or limit inference threads.
Both detectors publish `bad` in `types`, `detections`, preview detections and annotations,
including Area A. `bad` has no ripeness suffix and bypasses ripeness classification.
Its ROI supplies per-fruit defect evidence. Its optional depth point remains diagnostic.
Raw `types` and `detections` remain unchanged, even when ripe and bad overlap.
Only the completed vote authorizes harvesting. Bad-produce removal is not implemented.

The external debug monitor expects active camera streams whenever perception is running;
missing streams are reported as `STALE`, not `IDLE`.

The decision stream on `/fruit_perception/decision` contains:

- header matching aligned depth
- `observation_window_id`: active harvest window ID, or empty string when outside window
- detected fruit `types` in confidence order
- all detections in confidence order (`detections`), including ROI and optional 3D points
- `has_target` and `target_point` only for a completed ripe target with valid depth; A/C also checks the expected label
- A/B/C `targets_ready`: the complete frame batch has finished, including empty observations
- `targets`: the A/C primary result or all B per-fruit results, including diagnostic `bad` and `unknown`
- `FruitTarget.has_point`: valid aggregated depth exists. A/C open-loop does not require it
- annotated RGB8 image on `/fruit_perception/camera/annotated_image`

The target point is the `target_point` field in the decision message and uses
the camera color optical frame from the message header. Coordinates are in meters: `x` points right in the image, `y`
points down, and `z` points forward.

The annotated image keeps the camera's original aspect ratio. Inference uses a
centered 640x640 letterbox with value 114, so no source pixels are cropped. Each
detection retains its model species even in A. Labels show separate detector and
ripeness confidence, for example `apple [RIPE 92%] 0.85 | D=0.450m`.
Ripe boxes are green, unripe boxes are orange, and centers have a crosshair.
The image encoding is RGB8, so orange uses `(255, 165, 0)` in the drawing buffer.
Depth reuses the existing aligned center pixel. Missing depth displays `D: NoDepth`
and never supplies a grasp point. ROS business labels and confidence semantics stay unchanged.

## Short-window target confirmation

A/B/C observe a complete batch of ten new processed frames. Empty detections count as frames.
Source YAML sets `camera_target_acquisition_frames: 10` and `camera_b_target_acquisition_frames: 10`.
There is no minimum observation duration, vote count, support ratio, or final-frame support requirement.
Legacy minimum-vote, support-ratio, missed-frame, and lock-loss parameters are removed.

Each spatial fruit track votes for its species/maturity label.
Only classifier probabilities at or above `camera_target_maturity_confidence_threshold: 0.70` vote.
The highest vote count wins. Equal counts use mean detector confidence of supporting observations.
Equal counts and equal confidence produce `unknown`. No valid votes also produce `unknown`.
Published confidence is mean detector confidence across that fruit's observations, not classifier probability.
The detector threshold remains `0.50`.

Association uses original-image boxes with IoU greater than `0.30`.
The existing 3D association radius remains available for candidates without boxes.
Tracks survive misses throughout this one batch. Different fruits do not share votes.
Depth aggregation uses only valid winning-label observations and retains statistical outlier rejection.
A/C open-loop can use a winning label without depth. Visual grasping still requires valid depth and mission geometry checks.

A/C selects one primary fruit using `0.7 * confidence + 0.3 * horizontal_centrality`.
Within `camera_target_score_close_threshold: 0.05` of the highest score, higher detector confidence wins.
Selection occurs **before** checking bad, maturity, or expected species.
A bad primary is skipped, not replaced by a neighboring healthy fruit for a fixed preset.
Multiple detections alone do not force a skip.
B publishes independent per-fruit results. Mission selects at most two matching ripe targets after geometry checks.
As before, B's expected species is filtered by mission, not by `has_target`.

A defect belongs to a fruit when its center is inside the fruit box and intersection covers at least half the defect area.
Small defect boxes therefore do not need high whole-fruit IoU.
Every associated fruit is vetoed if attribution is ambiguous. Unrelated fruits remain eligible.
One valid defect observation latches `bad` until this window resets, even when subsequent frames miss the defect.
Defect boxes are retained through the batch, including when the fruit ROI is temporarily unavailable.
An internal candidate's `bad` flag provides the same veto for a future ROI defect detector.
No third-stage model, loader, or missing-model fallback is implemented now.

Completed results freeze and repeat until reset. Mission consumes only `targets_ready/targets`, never raw-label history.
If ten frames do not finish within the existing timeout, capped at ten seconds, mission skips without retries.
**One valid mature vote among nine empty frames may win. A missing final-frame target does not revoke the result.**
These operator-selected rules favor fewer missed harvests. They do not guarantee stable detection or absence of defects.
At processing rate F, the batch needs roughly `10/F` seconds plus transport costs, not a fixed wall-clock delay.
Rebuild perception and mission, then restart their nodes. No preset, route, or motion-service contract changes.
