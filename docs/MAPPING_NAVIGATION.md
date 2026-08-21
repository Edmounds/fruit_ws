  # 建图到定位导航

  以下命令均在 `/home/rc4/fruit_ws` 执行。每个新终端都必须先加载运行环境：

  ```bash
  cd /home/rc4/fruit_ws
  source setup_fruit_runtime.bash
  ```

  运行环境固定使用 `rmw_cyclonedds_cpp`。导航、mission、调试监视器和 ROS 2 CLI 必须从
  同一次环境加载后启动，禁止把仍在运行的 Fast DDS 节点与 Cyclone DDS 节点混在一套任务中。
  首次从 Fast DDS 切换时先停止全部旧进程，并执行一次 `ros2 daemon stop` 后再重新启动。

  `map A`、`map B` 是两个独立的物理场地 profile，不是任务流程中的 A 区、B 区。场馆在建筑物中的绝对位置不会进入 ROS `map` 坐标系，
  但只要地图原点、朝向或现场路线需要独立调节，就必须使用对应 profile 的地图、初始位姿
  和 waypoint YAML。所有现场脚本都要求显式指定 `--map A` 或 `--map B`。

  ```bash
  ./start_mapping.sh --map A
  ./save_glim_map.sh --map A
  ./start_relocalization_test.sh --map A
  ./start_odometry.sh --map A
  ./waypoints_maker.sh --map A --odom
  ./start_navigation.sh --map A
  ./start_navigation.sh --map A --odom
  ./start_fruit_mission.sh --map A
  ./start_fruit_mission.sh --map A --odom --flow a
  ./start_fruit_mission.sh --map A --debug
  ./record_rosbag.sh --full
  ```

  将命令中的 `--map A` 替换为 `--map B` 即可使用另一套物理场地地图、定位数据和 waypoint
  文件。地图文件位于 `src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/{A,B}/`，
  路点文件位于 `src/fruit_waypoint_recorder/waypoints/{A,B}/competition_waypoints.yaml`。
  地图选择会同时切换该场地的地图、定位初始位姿和 waypoint 文件。

  纯里程计调试不读取地图、`field.initial_pose.yaml` 或 GICP，只复用同一个 `--map A|B`
  profile 标识现场起点：

  ```bash
  ./start_odometry.sh --map A
  ```

  该命令启动 Livox MID-360、Point-LIO 及其坐标适配节点。机器人必须在起点保持静止；
  启动脚本先等待雷达连接、IMU 初始化和第一条有效 Point-LIO 里程计，再开始稳定倒计时。
  适配节点会等待连续约 1.5 秒的低速、低位置/yaw 方差数据，并用稳定窗口的平均位姿建立
  本次运行的 `odom` 原点。速度按完整窗口的首尾位姿计算，避免把 20 Hz 单帧位置噪声
  放大成错误的运动速度。稳定完成前不会发布可供控制使用的 `odom -> base_link`。
  收到第一条有效里程计后，稳定检测最多等待 5 秒；超时后脚本退出且不会用漂移中的位姿
  降级建立原点。
  `--map A`/`--map B` 不会加载对应 PCD，也不会发布 `map -> odom`；它只保证现场脚本使用
  一致的 profile 标识。需要全局坐标时仍使用下面的建图、导出和重定位流程。

  `start_navigation.sh --odom`、`waypoints_maker.sh --odom` 和
  `start_fruit_mission.sh --odom` 复用同一条纯里程计导航链，并等待上述稳定 TF。
  它们不读取地图、初始位姿或 GICP；mission 同时切换到该 profile 的
  `odometry_waypoints.yaml`，并把 waypoint 的坐标系设为 `odom`。里程计 waypoint 必须在
  同一个物理起点和朝向启动后复用。`fruit_mission_ui.py` 中打开“纯里程计模式”推拉开关后，
  两个启动按钮会自动追加 `--odom`；状态栏会提示正在等待
  稳定起点。

  控制面板允许先启动定位/导航，确认定位正常后再启动比赛流程。第二步会自动给
  `start_fruit_mission.sh` 追加 `--reuse-navigation`，复用已经运行的导航、定位、串口桥和
  调试监视器，只补充启动感知与任务节点，不会重启雷达或定位。此模式下比赛流程退出只停止
  它启动的感知与任务进程，导航仍保持运行；“停止全部进程”仍会统一停止所有项目进程。

  Mission 流程使用独立的 `--flow full|ac|a|b|c` 参数选择，默认是 `full`。`--flow b` 表示
  B 区流程，不代表切换到 `map B`，例如：

  ```bash
  ./start_fruit_mission.sh --map A --flow b
  ```

  完整任务启动命令增加 `--debug` 时，会在导航、摄像头和任务节点启动前自动录制 ROS
  调试数据，包括隐藏的 Action 话题、LiDAR、点云和选定的视觉图像。视觉数据只保留
  `/left_camera/image_raw`、`/right_camera/image_raw`、两路对应的
  `/fruit_perception/{left,right}/annotated_image`，以及 D435i 的
  `/camera/camera/color/image_raw`、`/camera/camera/aligned_depth_to_color/image_raw` 和
  `/fruit_perception/d435i/annotated_image`。黑名单排除未对齐的 D435i raw depth、重复的
  RGBD 复合消息和所有 `compressed`、`compressedDepth`、`theora` 派生图像，避免调试录包
  把进程内图像链全部拉到 DDS 并重复落盘。录包保存在项目根目录的
  `rosbags/YYYYMMDD_HHMMSS/`，最多保留最新的 5 个录包。单独执行 `record_rosbag.sh` 时
  使用同一黑名单；`--full` 仅作为兼容的显式模式标识。

  `robot_debug_monitor.sh` 每个统计周期输出一行 `D435I streams`。`color` 和 `depth` 的频率
  来自 RealSense metadata，`rgbd` 来自感知 component 每收到一帧进程内 RGBD 时发布的轻量
  `/fruit_perception/d435i/rgbd_heartbeat`。每项同时显示窗口实际频率、最后一帧 ROS 时间和
  距当前的接收时长；`QR/TREE` 模式下任一流超过 1 秒没有新帧时状态变为 `STALE`。

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
  ./start_mapping.sh --map A
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
  ./save_glim_map.sh --map A
  ```

  脚本从当前 profile 的 `/home/rc4/fruit_ws/tmp/glim_map/{A,B}` 导出同一批次的文件：

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

  启动脚本会显式传入当前 profile 的 `field.initial_pose.yaml`，覆盖公共定位参数中的
  初始值。缺失侧车文件时启动会失败，不会静默使用全零初值。

  当前 `field.pcd` 自动生成的建图起点初值为：

  ```yaml
  initial_map_to_odom: [-0.009653, -0.141873, 0.051420, -0.004372, 0.014471, 1.569590]
  ```

  其中 yaw 约为 `89.93` 度，主要来自 MID-360 相对 `base_link` 的 `-90` 度
  安装方向。以后重新建图后只需运行 `./save_glim_map.sh --map A`（或 `B`），不要再手工计算或修改
  `nav2_params.yaml`。

  ## 4. 启动定位导航

  ```bash
  ./start_navigation.sh --map A
  ```

  直接使用 ROS 2 launch 时仍然传入显式路径，并同时提供初始位姿侧车：

  ```bash
  ros2 launch pb2025_nav_bringup localization_navigation_launch.py \
    map:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/A/field.yaml \
    localization_map:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/A/field.pcd \
    initial_pose_file:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/A/field.initial_pose.yaml \
    params_file:=/home/rc4/fruit_ws/src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/localization_params.yaml \
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
