# fruit_qr_planner

ROS 2 Humble package for the competition's one-shot QR workflow. It reads camera images,
uses the decoded official QR payloads to publish the B-zone plan followed by the C-zone
plan, then stops accepting further codes.

## Interfaces

- Input: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- Observation window: `/fruit_mission/observation_window` (`std_msgs/msg/String`, JSON)
- Current area: `/fruit_mission/current_area` (`std_msgs/msg/String`)
- B plan: `/fruit/b_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- C plan: `/fruit/c_pick_plan` (`std_msgs/msg/String`, UTF-8 JSON)
- Parameters: `image_topic`, `observation_window_topic`, `current_area_topic`, and `confirm_frames` (default `1`)

Decoding only occurs during an active observation window with `purpose="qr"` and for frames whose capture timestamp is at or after the window's start time. Outside a valid QR window, frames are immediately dropped to minimize CPU overhead.
Each plan published includes `observation_window_id`, `image_stamp_sec`, and `image_stamp_nanosec`.
Window commands accept areas A/B/C only. Mission consumers reject non-object plan JSON,
invalid integer image timestamps, wrong window IDs, and mismatched QR purpose or B/C area.

Both plan topics use Reliable, Transient Local QoS with depth 1, so consumers
can retrieve the published plan. Within each window, only one valid plan is published.

If mission enters C before a B code was decoded, the current-area message or a C observation window advances the
scanner directly to C. This preserves the C scan opportunity when mission used its
configured B fallback after the 10-second plan wait. QR timeout fallback contents are
owned by `fruit_mission/config/fruit_mission.yaml`; this node only decodes and publishes
codes that were actually read.

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

Restart the node before the next competition round to reset the B-then-C sequence.
