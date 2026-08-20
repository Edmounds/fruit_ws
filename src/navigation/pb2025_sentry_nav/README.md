# MID360 GLIM + Point-LIO/GICP Differential Navigation

This repository is a ROS 2 Humble real-robot navigation stack for one differential-drive robot
with one Livox MID360. It intentionally does not contain the previous RoboMaster sentry,
multi-robot, simulation, gimbal, shooting, or omnidirectional-control paths.

The runtime TF contract is:

```text
map -> odom -> base_link -> front_mid360
```

- `pb_gicp_localization` publishes the only `map -> odom` transform.
- `sensor_scan_generation` publishes the only `odom -> base_link` transform.
- `localization_navigation_launch.py` publishes the calibrated static
  `base_link -> front_mid360` transform for the installed MID360.
- Wheel odometry may be published as `wheel/odom`, but its TF publication must be disabled.

## Mapping And Map Handoff

GLIM and `glim_ros2` are installed in a separate workspace and are not dependencies of this
repository. Configure the MID360 driver to publish all three streams needed by the two pipelines:

```text
/livox/lidar             livox_ros_driver2/msg/CustomMsg
/livox/lidar/pointcloud  sensor_msgs/msg/PointCloud2
/livox/imu               sensor_msgs/msg/Imu
```

Use the CustomMsg and IMU streams for Point-LIO. Use PointCloud2 and IMU for GLIM. Build the map
with GLIM, inspect/repair it with `offline_viewer`, and export the globally optimized PLY. Convert
the export without applying any LiDAR, base, or odom extrinsic transform:

```bash
ros2 run pb_glim_map_tools glim_ply_to_pcd \
  --input /abs/path/glim_map.ply \
  --output /abs/path/localization.pcd \
  --leaf-size 0.10
```

The resulting PCD is defined directly in `map`. The tool removes NaNs, voxel-downsamples the map,
writes binary `PointXYZ` PCD, and prints point counts and the map-frame bounding box.

Prepare a Nav2 `map.yaml` plus PGM separately. The 2D map and GLIM PCD must have exactly the same
`map` origin, axis directions, scale, and orientation. This project does not generate a 2D
occupancy grid from the 3D map.

## Localization

Point-LIO is odometry only. Keep both settings disabled:

```yaml
prior_pcd:
  enable: false
pcd_save:
  pcd_save_en: false
```

`loam_interface` converts Point-LIO's native `camera_init/body` products to odom-frame
`registered_scan` and `lidar_odometry`. Point-LIO `body` is the Mid360 internal IMU frame
(`livox_frame`), not `front_mid360`; `loam_interface` applies the same LiDAR-in-IMU extrinsic
as Point-LIO before publishing `odom -> front_mid360`. `sensor_scan_generation` uses the
calibrated static TF to publish Nav2 `odometry`, `sensor_scan`, and `odom -> base_link`. If the
static TF is unavailable, the synchronized sample is dropped instead of silently using identity.

`pb_gicp_localization` treats the GLIM PCD as its map-frame target and `registered_scan` as its
odom-frame source. Set the known startup transform in the selected parameter file:

```yaml
pb_gicp_localization:
  ros__parameters:
    initial_map_to_odom: [x, y, z, roll, pitch, yaw]
```

Start within roughly 1 m and 20 degrees of that pose (`max_translation_jump_initial` /
`max_rotation_jump_initial`). After the first accepted lock, tighter jump gates apply. GICP runs
at 2 Hz on a worker thread so `map -> odom` TF keeps publishing at 20 Hz; it rejects
non-convergence, too few inliers, low overlap, excessive mean error, and implausible pose jumps.
The last accepted transform remains active after a rejected result; consecutive rejects raise an
ERROR diagnostic. Debug clouds are published on `gicp/localization_map`, `gicp/source_cloud`, and
`gicp/aligned_cloud`.

## Build And Run

Install the normal ROS dependencies, including `twist_mux`, then build:

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Launch the one supported real-robot entrypoint:

```bash
ros2 launch pb2025_nav_bringup localization_navigation_launch.py \
  map:=/abs/path/map.yaml \
  localization_map:=/abs/path/localization.pcd \
  params_file:=/abs/path/localization_params.yaml \
  use_teleop:=false
```

The launch file automatically loads `/abs/path/localization.initial_pose.yaml` as
the final `pb_gicp_localization` parameter override. Generate the PCD and matching
initial-pose sidecar together with `/home/rc4/fruit_ws/save_glim_map.sh`.

The launch fails immediately when any required map or parameter file is missing. Set
`use_teleop:=true` to start the Bluetooth Xbox Wireless Controller path. The launch selects the
controller by its Linux input name and publishes through the existing `twist_mux` safety timeout.

## Differential Drive Commands

The Nav2 controller and joystick only command `linear.x` and `angular.z`; all other Twist fields
remain zero. Nav2 is limited to 0.6 m/s forward, 0.2 m/s reverse, and 1.0 rad/s. The controller
rotates in place above 0.7 rad heading error and performs final-yaw alignment inside 0.20 m.

Command arbitration is:

```text
Nav2 velocity smoother -> cmd_vel_nav ----+
                                         +-> twist_mux -> cmd_vel -> base
Xbox controller -> joy_linux -> JoyConvert -> cmd_vel_teleop -+
```

Teleop has the higher priority and a 0.5 s timeout. The Xbox configuration limits commands to
0.4 m/s and 0.8 rad/s, and only uses `linear.x` and `angular.z` for the differential-drive base.

## Relocalization Visualization

Start the launch with `use_rviz:=true`. The default RViz configuration uses `map` as Fixed Frame.
In the `GICP Localization` group, compare the gray `GICP Map` with the green `GICP Aligned`
cloud; fixed structures should overlap and `GICP Path` should be continuous without jumps.
Enable the red `GICP Source Odom` display when the uncorrected odom cloud is needed for comparison.
The `Global Costmap`, `Local Costmap`, `Path`, and `Local Plan` displays show whether navigation is
using the corrected robot pose. The full command sequence and diagnostic interpretation are in
[`MAPPING_NAVIGATION.md`](../../../docs/MAPPING_NAVIGATION.md#5-查看重定位效果).

## Verification

```bash
colcon test
colcon test-result --verbose
ros2 run tf2_tools view_frames
ros2 topic echo /cmd_vel
```

On the robot, confirm the three MID360 topics, one unbroken TF chain, continuous Point-LIO odometry,
accepted GICP diagnostics near the configured initial pose, and zero lateral velocity on every
base command.
