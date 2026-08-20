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

  `mapping.launch.py` 会同时启动
  `/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/sensor_frames.launch.py`
  发布 MID-360 安装外参。Livox driver 只负责在消息头里写 frame：
  `/livox/lidar` 是 `front_mid360`，`/livox/imu` 是 `livox_frame`，它本身不发布安装 TF。

  建图前用 `tf2` 验证完整链路，不要只看网页面板里的 TF 树；有些面板只显示动态
  `/tf`，不会把 `/tf_static` 里的静态外参画出来。

  ```bash
  ros2 run tf2_ros tf2_echo base_link front_mid360
  ros2 run tf2_ros tf2_echo front_mid360 livox_frame
  ros2 run tf2_ros tf2_echo map front_mid360
  ros2 run tf2_tools view_frames
  ```

  正常链路应为：

  ```text
  map -> odom -> base_link -> front_mid360 -> livox_frame
  ```

  缓慢移动车辆完成场地扫描。结束时先在终端 2 按 `Ctrl+C`，等待 GLIM 保存完成，再停止终端 1。

  ## 2. 导出定位点云

  ```bash
  ./save_glim_map.sh
  ```

  脚本从 `/home/rc4/fruit_ws/tmp/glim_map` 导出同一批次的三个文件：

  ```text
  field.ply
  field.pcd
  field.initial_pose.yaml
  ```

  `field.initial_pose.yaml` 根据 `traj_lidar.txt` 第一帧和
  `config/reality/sensor_frames.yaml` 中的安装外参自动计算。三个文件先在 `tmp`
  中生成，全部成功后才一起替换 `maps` 目录中的旧文件，避免地图与初值不匹配。

  另外准备 Nav2 使用的 `field.yaml` 和 `field.pgm`，放在同一 `maps` 目录。二维地图必须与 `field.pcd` 使用相同的 `map` 原点、方向和比例；本工程目前不自动从 GLIM 点云生成二维占据栅格。

  ## 3. 定位初值

  `field.pcd` 保留 GLIM 的 `map` 坐标系。`000000` 表示第一个子地图，不表示
  建图起点的 `base_link` 位姿一定为全零。建图起点车体位姿应按下式从 GLIM
  第一帧 LiDAR 位姿和安装外参计算：

  ```text
  T_map_base_start = T_map_lidar_start * inverse(T_base_lidar)
  ```

  `localization_navigation_launch.py` 会根据 `localization_map` 自动寻找同名侧车
  文件。例如定位地图为 `field.pcd` 时，自动加载 `field.initial_pose.yaml`，并覆盖
  `nav2_params.yaml` 中的兜底值。测试脚本 `start_relocalization_test.sh` 使用相同规则。

  当前 `field.pcd` 自动生成的建图起点初值为：

  ```yaml
  initial_map_to_odom: [-0.009653, -0.141873, 0.051420, -0.004372, 0.014471, 1.569590]
  ```

  其中 yaw 约为 `89.93` 度，主要来自 MID-360 相对 `base_link` 的 `-90` 度
  安装方向。以后重新建图后只需运行 `./save_glim_map.sh`，不要再手工计算或修改
  `nav2_params.yaml`。

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

  ## 5. 查看重定位效果

  启动命令中的 `use_rviz:=true` 会加载默认 RViz 配置，Fixed Frame 必须是 `map`。在左侧 Displays 中重点查看：

  - `GICP Map`（灰色）：定位用的 `field.pcd`。
  - `GICP Aligned`（绿色）：当前扫描经过 GICP 变换后得到的点云。
  - `GICP Source Odom`（红色）：尚未经过 GICP 变换的 odom 点云，默认关闭，需要观察原始偏差时手动打开。
  - `GICP Pose` 和 `GICP Path`：当前定位姿态和已接受的 `map -> odom` 轨迹。
  - `Global Costmap`、`Local Costmap`、`Path`、`Local Plan`：导航规划是否建立在正确的机器人位置上。

  判断重定位成功的标准是：绿色 `GICP Aligned` 点云与灰色 `GICP Map` 的墙面、树干等固定结构基本重合，`GICP Path` 连续且没有突然跳跃，机器人模型位于扫描点云正确位置。

  不要用 `map -> odom` 是否接近零作为判断标准；它就是用来修正 odom 漂移的，车辆运动时缓慢变化是正常的。应以点云重合、`registration accepted` 和轨迹连续为准。

  终端中可以分别观察 TF 和 GICP 诊断：

  ```bash
  ros2 run tf2_ros tf2_echo map odom
  ros2 run tf2_ros tf2_echo map base_link
  ros2 topic echo /diagnostics
  ```

  在 `/diagnostics` 中找到 `pb_gicp_localization/registration`，正常时应看到：

  ```text
  level: 0
  message: registration accepted
  accepted: "1"
  consecutive_rejects: "0"
  ```

  `inliers` 和 `inlier_ratio` 应保持稳定，`mean_error` 不应持续升高。出现 `level: 1` 表示本次匹配被拒绝但尚未丢失定位；连续拒绝达到阈值后会变成 `level: 2`，并显示 `localization lost`。

  在 RViz 中使用 `2D Goal Pose` 下发导航目标。结束运行时在 launch 终端按 `Ctrl+C`。
