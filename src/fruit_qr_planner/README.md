# fruit_qr_planner

ROS 2 Humble package for the competition's one-shot QR workflow. It reads camera images,
confirms the same valid QR payload across three consecutive frames, publishes the B-zone
plan, then publishes the C-zone plan and stops accepting further codes.

## Interfaces

- Input: `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- B plan: `/fruit/b_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- C plan: `/fruit/c_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- Parameters: `image_topic` (default `/camera/camera/color/image_raw`) and `confirm_frames` (default `3`)

Both plan topics use Reliable, Transient Local QoS with depth 1, so the mission
node can start after scanning and still receive the last B and C plans. After C
is confirmed, the node destroys its image subscription.

The B code contains exactly eight produce names. The C code contains eight produce-name
lines followed by one comma-separated line of eight unique positions in the range 1-12.
Allowed names are 苹果、梨子、南瓜、西红柿、辣椒、洋葱.

The fixed D435i `rgb8` image is validated and converted with the system NumPy
and OpenCV directly. The ROS runtime must be started through
`/home/rc4/fruit_ws/setup_fruit_runtime.bash`, which excludes user-site NumPy 2
and pip OpenCV packages from this process.

## Build and run

```bash
cd /path/to/fruit_ws/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select fruit_qr_planner
source install/setup.bash
ros2 launch fruit_qr_planner fruit_qr_planner.launch.py
```

Inspect results with:

```bash
ros2 topic echo /fruit/b_pick_plan
ros2 topic echo /fruit/c_pick_plan
```

Restart the node before the next competition round to reset the B-then-C sequence.
