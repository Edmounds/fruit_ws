# 建图到定位导航

以下命令均在 `/home/rc4/fruit_ws` 执行。每个新终端都必须先加载运行环境：

```bash
cd /home/rc4/fruit_ws
source setup_fruit_runtime.bash
```

## 1. 建图

终端 1：启动 MID-360，给 GLIM 发布 `PointCloud2`。

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=front_mid360 \
  -p user_config_path:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json
```

终端 2：启动 GLIM 建图。

```bash
ros2 launch glim_ros mapping.launch.py \
  dump_path:=/home/rc4/fruit_ws/tmp/glim_map
```

缓慢移动车辆完成场地扫描。结束时先在终端 2 按 `Ctrl+C`，等待 GLIM 保存完成，再停止终端 1。

## 2. 导出定位点云

```bash
mkdir -p /home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps

ros2 run glim_ros offline_viewer \
  /home/rc4/fruit_ws/tmp/glim_map \
  --export_path /home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.ply \
  --config_path /home/rc4/fruit_ws/src/navigation/glim/config

ros2 run pb_glim_map_tools glim_ply_to_pcd \
  --input /home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.ply \
  --output /home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.pcd \
  --leaf-size 0.10
```

另外准备 Nav2 使用的 `field.yaml` 和 `field.pgm`，放在同一 `maps` 目录。二维地图必须与 `field.pcd` 使用相同的 `map` 原点、方向和比例；本工程目前不自动从 GLIM 点云生成二维占据栅格。

## 3. 设置定位初值

编辑：

```bash
nano /home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml
```

设置车辆启动时在地图中的位姿：

```yaml
pb_gicp_localization:
  ros__parameters:
    initial_map_to_odom: [x, y, z, roll, pitch, yaw]
```

车辆从建图起点、相同朝向启动时可填写 `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`。

## 4. 启动定位导航

```bash
ros2 launch pb2025_nav_bringup localization_navigation_launch.py \
  map:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.yaml \
  localization_map:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/field.pcd \
  params_file:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml \
  use_teleop:=false \
  use_rviz:=true
```

确认定位正常：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /diagnostics --once
```

在 RViz 中使用 `2D Goal Pose` 下发导航目标。结束运行时在 launch 终端按 `Ctrl+C`。
