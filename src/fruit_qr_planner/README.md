# fruit_qr_planner

ROS 2 Humble package for mission-owned B/C QR observation windows. It reads camera
images and parses each payload according to the active window's area. There is no
B-then-C sequence state machine: C can be scanned first, and later B/C windows
can be processed without restarting the node.

## Interfaces

- Input: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- Observation window: `/fruit_mission/observation_window` (`std_msgs/msg/String`, JSON)
- B plan: `/fruit/b_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- C plan: `/fruit/c_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- Preview: `/fruit_qr_planner/camera/preview_detections` (`std_msgs/msg/String`, UTF-8 JSON)
- Parameters: `image_topic`, `observation_window_topic`, and `confirm_frames` (default `1`)

Decoding only occurs during an active observation window with `purpose="qr"` and for frames whose capture timestamp is at or after the window's start time. Outside a valid QR window, frames are immediately dropped to minimize CPU overhead.
Each plan published includes `observation_window_id`, `image_stamp_sec`, and `image_stamp_nanosec`.
Preview messages contain `stamp_sec`, `stamp_nanosec`, `frame_id`, and `detections`.
Each detection contains `text` and pixel `roi: [x, y, width, height]`.
Preview results use the source image header, including empty detection lists.
They publish before task-plan validation, so invalid competition contents do not hide a detected QR rectangle.
Preview limits are 64 rectangles and 4096 characters per rectangle. Task-plan parsing is unchanged.
Mission preview decoding stops after success or window closure. The fused camera video does not stop.
`test_qr.sh` continuously decodes and publishes the same preview protocol without publishing JPEG or fruit annotation images.
RobViz renders QR and fruit results independently into `/fruit_robviz_bridge/camera/annotated_image` for WebRTC.
Plans retain tasks and area metadata, without unused `stages` or `scan_index` fields.
Window commands accept areas A/B/C only. Mission consumers reject non-object plan JSON,
invalid integer image timestamps, wrong window IDs, and mismatched QR purpose or B/C area.

Both plan topics use Reliable, Transient Local QoS with depth 1, so consumers
can retrieve the published plan. Within each window, only one valid plan is published.

Window area B directly selects the B parser; C directly selects the C parser.
Consecutive-frame confirmation resets for each window and when no code is read.
Current-area messages do not advance a QR state or start scanning. QR timeout
fallback contents are owned by `fruit_mission/config/qr_fallback.yaml`; this node
only decodes and publishes codes that were actually read. Mission directly executes
the `qr_scan` arm preset before opening a window; no MCU task/callback or camera
switch is involved.

The official B/C QR payloads are trusted. The planner only normalizes formatting before
building JSON: BOM, CRLF line endings, surrounding whitespace, ASCII commas, Chinese
commas, and semicolon-separated position lists are accepted. The B code is interpreted as
one target per non-empty line. The C code accepts either target lines followed by one
position list, or one `target + position` task per line.

The fixed camera `rgb8` image is validated and converted with the system NumPy
and OpenCV directly. QR payloads are decoded by the system `python3-pyzbar`
and `libzbar0` packages, which support UTF-8 Chinese payloads without relying
on OpenCV's optional QUIRC decoder. Always run nodes and launch commands in an environment initialized by
`/home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash`, which excludes user-site NumPy 2
and verifies that system NumPy 1.x and OpenCV 4.5.4 are loaded from
`/usr/lib/python3/dist-packages`.

## Build and run

```bash
cd /path/to/fruit_ws/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt-get install -y python3-pyzbar
rosdep install --from-paths src --ignore-src -r -y --skip-keys python3-pyzbar
colcon build --symlink-install --packages-select fruit_qr_planner
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
ros2 launch fruit_qr_planner fruit_qr_planner.launch.py
```

Inspect results with:

```bash
ros2 topic echo /fruit/b_pick_plan
ros2 topic echo /fruit/c_pick_plan
```

Close the old window and open a new one for another scan; no sequence reset is needed.
