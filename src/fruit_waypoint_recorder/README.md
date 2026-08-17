# fruit_waypoint_recorder

Records the latest `map -> base_link` transform to a timestamped YAML file while
the robot is positioned with the Xbox controller.

## Controls and interface

- Xbox A button: append the current pose once per button press.
- D-pad left: remove the last saved pose once per press.
- `/waypoint_recorder/save` (`std_srvs/srv/Trigger`): append the current pose.

The default Xbox Linux mapping uses button index `0` for A and axis index `6`
with value `+1` for D-pad left. All mappings, TF frames, and thresholds are
parameters in `config/waypoint_recorder.yaml`.

Each launch creates a new file under the source package:

```text
/home/rc4/fruit_ws/src/fruit_waypoint_recorder/waypoints/
  waypoints_YYYYMMDD_HHMMSS.yaml
```

Each entry contains an automatic name, recording time, XYZ position, quaternion,
and yaw. A failed TF lookup does not append a waypoint. Undo rewrites the current
session file after removing its last entry.

## Build and run

Build the package outside any training virtual environment:

```bash
cd /home/rc4/fruit_ws
source setup_fruit_runtime.bash
colcon build --symlink-install --packages-select fruit_waypoint_recorder
source setup_fruit_runtime.bash
```

Start navigation with `use_teleop:=true` so the existing `joy_linux` node
publishes `/joy`, then start the recorder:

```bash
ros2 launch fruit_waypoint_recorder waypoint_recorder.launch.py
```

The same save operation can be triggered without the controller:

```bash
ros2 service call /waypoint_recorder/save std_srvs/srv/Trigger '{}'
```

The node prints the exact session path at startup and logs every save and undo.
