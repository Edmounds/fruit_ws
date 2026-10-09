# fruit_perception

Perception container, detection, ripeness classification, and preview bridge.

## Interfaces and Boundaries

The package publishes:

```text
/fruit_perception/camera/annotated_image/compressed  sensor_msgs/msg/CompressedImage
/fruit_perception/camera/native_depth/compressed     sensor_msgs/msg/CompressedImage
```

## Launch and Operation

The production launch is `fruit_perception.launch.py`. It owns the camera driver,
neural-network detection component, and the RobViz bridge image component.

Do not launch `fruit_robviz_bridge.launch.py` separately when this perception
launch is active.

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

The service template supplies `--zone A`. Update an existing installed unit before
using the new script. Do not restart camera services during an active mission.
`start_fruit_mission.sh` launches `fruit_perception.launch.py` directly, not this
debug script. Mission clients set `zone` through `SetParametersAtomically` and
check the response before observation. The mission area topic remains state only.

The validated string parameter `zone` selects the model and candidate selection.
Launch accepts `zone:=A|B|C`, defaulting to the source YAML value (`A`).
Perception no longer subscribes to `/fruit_mission/current_area`.
Observation windows do not select models. A harvest window must match `zone`.
Changing `zone` closes the active window and discards old in-flight evidence.
Open a new matching window after a successful parameter response.

- Area `A`: `vegetable_type/best.onnx` plus ripeness classification; publishes
  bare `ripe/unripe` labels. Only `ripe` enters single-target locking.
- Area `B`: `fruit_type/best.onnx` with multi-target output and target locking.
- Area `C`: single-class fruit detection and ripeness estimation.
