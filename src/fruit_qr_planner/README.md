# fruit_qr_planner

ROS 2 Humble package for the competition's one-shot QR workflow. It reads camera images,
uses the first decoded official QR payload, publishes the B-zone plan, then publishes the
C-zone plan and stops accepting further codes.

## Interfaces

- Input: `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- B plan: `/fruit/b_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- C plan: `/fruit/c_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- Parameters: `image_topic` (default `/camera/camera/color/image_raw`) and `confirm_frames` (default `1`)

Both plan topics use Reliable, Transient Local QoS with depth 1, so the mission
node can start after scanning and still receive the last B and C plans. After C
is confirmed, the node destroys its image subscription.

The official B/C QR payloads are trusted. The planner only normalizes formatting before
building JSON: BOM, CRLF line endings, surrounding whitespace, ASCII commas, Chinese
commas, and semicolon-separated position lists are accepted. The B code is interpreted as
one target per non-empty line. The C code accepts either target lines followed by one
position list, or one `target + position` task per line.

The fixed D435i `rgb8` image is validated and converted with the system NumPy
and OpenCV directly. QR payloads are decoded by the system `python3-pyzbar`
and `libzbar0` packages, which support UTF-8 Chinese payloads without relying
on OpenCV's optional QUIRC decoder. The ROS runtime must be started through
`/home/rc4/fruit_ws/setup_fruit_runtime.bash`, which excludes user-site NumPy 2
and pip OpenCV packages from this process.

## Build and run

```bash
cd /path/to/fruit_ws/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt-get install -y python3-pyzbar
rosdep install --from-paths src --ignore-src -r -y --skip-keys python3-pyzbar
colcon build --symlink-install --packages-select fruit_qr_planner
source /home/rc4/fruit_ws/setup_fruit_runtime.bash
ros2 launch fruit_qr_planner fruit_qr_planner.launch.py
```

Inspect results with:

```bash
ros2 topic echo /fruit/b_pick_plan
ros2 topic echo /fruit/c_pick_plan
```

Restart the node before the next competition round to reset the B-then-C sequence.
