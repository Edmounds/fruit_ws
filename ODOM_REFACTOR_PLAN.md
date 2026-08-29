# 纯 Point-LIO/odom 改造执行计划

本文档是给执行模型使用的逐步实施方案。目标是：在保留 `mapping` 分支原始建图/重定位版本的前提下，把 `main` 改造成只运行 Point-LIO 纯里程计、`fruit_navigation` 轴向路径跟随、感知和任务流程的版本。

## 1. 已确认的最终边界

- 先保留当前提交创建 `mapping` 分支，再在 `main` 实施改造；不要在 `mapping` 分支做删除。
- 保留顶层 `src/navigation/` 作为功能包分级目录，但其下面直接放 ROS 功能包，不再保留 `pb2025_sentry_nav/` 二级工程目录。
- 保留 `src/driver/realsense-ros` submodule。
- 删除 `src/driver/livox_ros_driver2` 原有空 submodule 引用；把当前实际使用的 `src/navigation/pb2025_sentry_nav/livox_ros_driver2` 迁移到 `src/driver/livox_ros_driver2`。
- 保留并迁移四个纯 odom 运行包：`livox_ros_driver2`、`point_lio`、`loam_interface`、`sensor_scan_generation`。
- `pb2025_nav_bringup` 改名为 `fruit_nav_bringup`，仅保留纯 odom bringup、传感器 TF、Point-LIO 参数和 MID-360 配置。
- 删除元包 `pb2025_sentry_nav`、GICP 包 `pb_gicp_localization`、GLIM 工具包 `pb_glim_map_tools`、GLIM 源码和所有地图/重定位 launch 与配置。
- `fruit_navigation` 移到 `src/navigation/fruit_navigation`，包名仍为 `fruit_navigation`。
- 只保留 A 的 odom waypoint，重命名为 `src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml`；删除 A/B 的 competition waypoint、B 的 odom waypoint 和 profile 配置。
- 所有运行默认使用 `odom`，不再提供 `--map` 或 `--odom` 选择分支。
- 根目录只放人工直接执行的入口脚本；runtime 和仅被其他脚本调用的脚本放入 `scripts/`。
- 不新增测试。可以删除因功能包/功能删除而失效的旧测试，并用构建、静态引用审计和 YAML/launch 解析做验证。

## 2. 分支保护与开始条件

在任何删除或移动前执行：

```bash
cd /home/rc4/fruit_ws
git status --short --branch
git branch --show-current
git rev-parse HEAD
```

如果 `mapping` 不存在，用当前 HEAD 创建：

```bash
git branch mapping HEAD
```

确认两个分支指向同一提交，然后只切回 `main`：

```bash
git rev-parse main mapping
git switch main
```

若开始时工作区有未提交修改，必须先记录并保留，不能 reset、checkout 或覆盖无关修改。

## 3. 包目录迁移

### 3.1 先处理 Livox submodule

保留 `src/driver/realsense-ros`，只移除 Livox 空 submodule：

```bash
git rm --cached src/driver/livox_ros_driver2
rmdir src/driver/livox_ros_driver2 2>/dev/null || true
```

不要删除 `src/driver/realsense-ros`。`.gitmodules` 最终只保留 RealSense 条目。

### 3.2 迁移实际使用的 ROS 包

将当前嵌套包直接提升一级：

```bash
git mv src/navigation/pb2025_sentry_nav/point_lio src/navigation/point_lio
git mv src/navigation/pb2025_sentry_nav/loam_interface src/navigation/loam_interface
git mv src/navigation/pb2025_sentry_nav/sensor_scan_generation src/navigation/sensor_scan_generation
git mv src/navigation/pb2025_sentry_nav/pb2025_nav_bringup src/navigation/fruit_nav_bringup
git mv src/navigation/pb2025_sentry_nav/livox_ros_driver2 src/driver/livox_ros_driver2
git mv src/fruit_navigation src/navigation/fruit_navigation
```

然后删除空的 `src/navigation/pb2025_sentry_nav/` 目录。若目录中还存在未列出的文件，先逐个检查是否为运行时必需项；地图、GICP、GLIM 相关内容不得迁移到新版本。

### 3.3 包名与构建元数据

- 编辑 `src/navigation/fruit_nav_bringup/package.xml` 和 `CMakeLists.txt`：包名改为 `fruit_nav_bringup`。
- 编辑四个迁移包的 README、CMake、launch、测试路径和注释，清除 `pb2025_sentry_nav` 路径。
- `fruit_navigation/package.xml` 内包名保持 `fruit_navigation`，只修复物理路径引用。
- `point_lio`、`loam_interface`、`sensor_scan_generation` 的 ROS package name 保持不变，避免下游 executable 名称变化。
- `livox_ros_driver2` package name 保持不变；它从 source package 位置改变，不改变 ROS 接口。
- 检查所有 CMake 的 `CMAKE_SOURCE_DIR`、安装目录和 `INSTALL_TO_SHARE` 路径，确保迁移后仍能安装 launch/config/rviz 资源。

## 4. `fruit_nav_bringup` 内容裁剪

最终建议目录：

```text
src/navigation/fruit_nav_bringup/
  package.xml
  CMakeLists.txt
  launch/
    pointlio_odometry.launch.py
    sensor_frames.launch.py
    fruit_nav.launch.py
  config/
    reality/
      pointlio_params.yaml
      mid360_user_config.json
      sensor_frames.yaml
  rviz/
    navigation.rviz
```

执行步骤：

1. 将 `config/reality/localization_params.yaml` 改名为 `pointlio_params.yaml`。
2. 从该 YAML 删除 `slam_toolbox`、`pb_gicp_localization`、`map_server` 等整个区块；保留 `livox_ros_driver2`、`laserMapping`、`loam_interface`、`sensor_scan_generation`、`pointcloud_to_laserscan`（若仍由现有 odom 链使用）和 `twist_mux`。
3. 检查 Point-LIO 的 `prior_pcd.enable`、`prior_pcd_map_path`、`pcd_save`、`publish.tf_send_en` 等字段，保持纯 odom 禁用先验地图、PCD 保存和重复 TF 发布。
4. `loam_interface` 默认必须是 `odom -> base_link`，并保留初始 odom 稳定化参数。
5. 删除 `glim_mapping.launch.py`、`localization_navigation_launch.py` 及其专用 RViz/map 资源。
6. 将 `odometry_navigation.launch.py` 改名为 `fruit_nav.launch.py`，更新包内路径和描述。该 launch 只启动：
   - `pointlio_odometry.launch.py`；
   - `fruit_navigation/axis_waypoint_action_server`；
   - `twist_mux`；
   - 可选 joystick 节点；
   - 可选 RViz，固定 frame 为 `odom`。
7. `fruit_nav.launch.py` 中 axis server 参数固定覆盖为 `map_frame: odom`、`require_gicp_localization: false`，不再接收 map、localization_map、initial_pose_file 参数。
8. `pointlio_odometry.launch.py` 和 `sensor_frames.launch.py` 的所有 source path 改为 `src/navigation/fruit_nav_bringup/...`；Livox 配置路径指向同一 bringup 包的 `config/reality/mid360_user_config.json`。
9. 按 AGENTS.md 要求，启动文件继续直接引用 `src` 路径，不引用 `install` 路径。

## 5. 删除不需要的包和资源

在确认 `mapping` 分支已保存后，从 `main` 删除：

```text
src/navigation/glim/
src/navigation/glim_ros2/
src/navigation/pb2025_sentry_nav/pb2025_sentry_nav/
src/navigation/pb2025_sentry_nav/pb_gicp_localization/
src/navigation/pb2025_sentry_nav/pb_glim_map_tools/
src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/glim_mapping.launch.py
src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/launch/localization_navigation_launch.py
src/navigation/pb2025_sentry_nav/pb2025_nav_bringup/maps/
```

使用 `git rm -r` 删除已纳入 Git 的目录。不要删除 `src/driver/realsense-ros`。

同时删除只服务于建图/重定位的根目录脚本：

```text
start_mapping.sh
save_glim_map.sh
start_relocalization_test.sh
gicp_debug_monitor.sh
```

删除这些脚本后，清理 `kill_fruit_processes.sh` 中对应的进程匹配、命令行匹配和提示文本。

## 6. Waypoint 单文件化

执行：

```bash
git mv src/fruit_waypoint_recorder/waypoints/A/odometry_waypoints.yaml \
  src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml
git rm -r src/fruit_waypoint_recorder/waypoints/A \
  src/fruit_waypoint_recorder/waypoints/B
git rm src/fruit_waypoint_recorder/config/map_profiles.yaml
```

保留 `waypoints/template.yaml`。检查新文件：

- `schema_version` 保持现有值；
- `frame_id` 必须为 `odom`；
- 点位顺序和名称保持 A odom 文件原样；
- 不要把 map 坐标或 B 区空文件合并进来。

更新以下配置为新路径和 `odom`：

- `src/fruit_mission/config/fruit_mission.yaml`：`waypoint_file`、`map_frame`；
- `src/fruit_waypoint_recorder/config/waypoint_recorder.yaml`：`waypoint_file`、`frame_id`；
- `src/fruit_waypoint_recorder/config/waypoint_editor.yaml`：移除 `profile_config` 依赖，改为单一 waypoint 文件配置，或按服务端最小改动改成直接读取 `odom_waypoints.yaml`；
- `fruit_mission.launch.py`、`waypoint_recorder.launch.py`、`waypoint_editor_server.py`：移除 profile/mode/map 分支，统一加载单一 odom 文件；
- `fruit_navigation/config/axis_navigation.yaml`（迁移后路径为 `src/navigation/fruit_navigation/config/axis_navigation.yaml`）：`map_frame: odom`、`require_gicp_localization: false`。

`waypoint_editor_server.py` 是单 profile 后最容易残留旧逻辑的文件。必须删除或改写 `maps.A/maps.B`、`competition` 模式和 `odometry` 模式的分支，确保页面只显示并编辑 `odom_waypoints.yaml`，备份仍写入既有 `tmp/waypoint_editor_backups`。

## 7. 脚本目录和入口改造

### 7.1 根目录最终保留

根目录保留人工直接执行的脚本：

```text
start_fruit_mission.sh
start_fruit_perception.sh
start_odometry.sh
start_navigation.sh
start_tele.sh
waypoints_maker.sh
record_rosbag.sh
zone_witch.sh
test_d435i_arm_target.sh
test_tts.sh
kill_fruit_processes.sh
```

### 7.2 移入 `scripts/`

```bash
git mv setup_fruit_runtime.bash scripts/setup_fruit_runtime.bash
git mv robot_debug_monitor.sh scripts/robot_debug_monitor.sh
```

然后把所有脚本、systemd service、README 和 docs 中的 runtime/monitor 路径改成 `scripts/...`。runtime 仍必须在 `set -u` 前 source；不要把它改成可执行启动器。

### 7.3 统一脚本参数和路径

所有启动脚本使用固定工作区路径 `/home/rc4/fruit_ws`，但路径变量必须指向新目录：

- bringup：`src/navigation/fruit_nav_bringup`；
- navigation：`src/navigation/fruit_navigation`；
- Livox driver：`src/driver/livox_ros_driver2` 仅作为源码包位置，ROS package 名仍为 `livox_ros_driver2`；
- 参数：`fruit_nav_bringup/config/reality/pointlio_params.yaml`；
- waypoint：`src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml`；
- runtime：`scripts/setup_fruit_runtime.bash`；
- monitor：`scripts/robot_debug_monitor.sh`。

所有脚本删除 `--map` 和 `--odom` 的解析、帮助文本和条件分支。

### 7.4 各入口的目标行为

`start_odometry.sh`：无必需参数，启动 `fruit_nav_bringup/launch/pointlio_odometry.launch.py`；不读取 map profile，不等待或验证 map/GICP。

`start_navigation.sh`：无必需参数，启动 `fruit_nav_bringup/launch/fruit_nav.launch.py`，默认 odom，包含 Point-LIO、axis waypoint action server、twist mux、硬件桥和 RViz；不启动 map server、GICP、重定位或 localization launch。

`start_fruit_mission.sh`：删除 localization 分支和 map profile；固定 `MAP_FRAME=odom`、`WAYPOINT_FILE` 为单一 odom 文件；保留 `--flow`、`--debug`、`--reuse-navigation`。`--reuse-navigation` 只表示复用纯 odom 的 `start_navigation.sh`，日志中不得再出现 localization 语义。

`waypoints_maker.sh`：只保留可选 `--tele`；固定启动纯 odom `fruit_nav.launch.py`、waypoint recorder 和 RViz，录点 frame 为 `odom`。

`start_fruit_perception.sh`、`start_tele.sh`、`record_rosbag.sh`、`zone_witch.sh`、两个 test 脚本：只更新 runtime/monitor 路径，不引入 map 参数。

`fruit_mission_ui.py`：删除 A/B 地图选择控件和 odom/map 切换状态；启动按钮直接调用无参数 `start_navigation.sh` 或带 `--flow` 的 `start_fruit_mission.sh`。状态文本统一为 odom，不再显示 map/GICP/localization。

`kill_fruit_processes.sh`：保留对现役入口和 ROS 节点的清理；删除 mapping、relocalization、GICP、GLIM 的模式匹配；monitor 路径改为 `scripts/robot_debug_monitor.sh`。

## 8. 依赖和旧引用清理

用 `rg` 反复搜索并清零非历史运行引用：

```bash
rg -n -i "glim|pb_gicp|gicp|relocal|localization_navigation|slam_toolbox|map_server|map_profiles|competition_waypoints|/map|--map|--odom|pb2025_sentry_nav|src/navigation/pb2025" \
  --glob '!build/**' --glob '!install/**' --glob '!log/**' .
```

允许保留的内容只有：

- `odom` 作为当前 frame；
- `mapping` 分支或历史说明中的明确历史文字（main 的现役文档不要继续给出已删除命令）；
- ROS `nav2_msgs` 依赖，因为 `FollowWaypoints` action 仍被 mission/recorder 使用；
- `fruit_navigation` 名称，因为它仍是路径跟随 action server，不等于地图导航。

从 `package.xml` 删除只属于已删除功能的依赖：`glim_ros`、`pb_gicp_localization`、`pb_glim_map_tools`、`slam_toolbox`、`nav2_map_server`、`nav2_lifecycle_manager` 等。不要误删 `nav2_msgs`、`tf2_ros`、`twist_mux` 或 `pointcloud_to_laserscan`，先用 launch 和代码引用确认。

删除或更新引用已删除脚本/包的旧测试文件；不新增测试文件，不修改普通 YAML 参数为测试固定断言。

## 9. 文档同步

在 `main`：

1. 删除过时的 `docs/MAPPING_NAVIGATION.md`，新建 `docs/ODOMETRY_NAVIGATION.md`。
2. 新文档只说明：runtime source、Point-LIO 启动、纯 odom 导航、单一 waypoint 文件、waypoint 录制、mission 启动、故障排查和 `/dev/left_cam`、`/dev/right_cam` 设备约束。
3. 更新 `README.md` 的 driver、启动、清理和 waypoint 命令；删除 GLIM/GICP/map 命令。
4. 更新 `docs/PROJECT_ARCHITECTURE.md` 的目录树、导航结构、启动链和完成标准；明确 `fruit_navigation` 是 odom frame 下的路径跟随包。
5. 更新 `docs/WAYPOINT_RECORDING.md` 为单文件 odom 路线；删除 A/B profile、competition/map 模式和 `--map` 示例。
6. 更新 `src/fruit_waypoint_recorder/README.md` 的 frame、文件路径和启动命令。
7. 更新 service 文件：`fruit-perception.service`、`waypoint-editor.service` 中 runtime/launch 路径必须指向现役 source 文件。若 service 是独立工具，说明其不由 kill 脚本停止。

`mapping` 分支保留旧文档和旧启动命令，不要为了同步 main 而改写该分支历史。

## 10. 构建与静态验证

按项目规则先加载运行环境；构建时不要在训练 venv 中执行：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon list --names-only
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

构建前若 runtime 仍在根目录，先完成路径迁移并重新 source。验证：

```bash
colcon list --names-only | sort
ros2 pkg prefix fruit_nav_bringup
ros2 pkg prefix fruit_navigation
ros2 pkg prefix point_lio
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix pb_gicp_localization   # 必须失败
ros2 pkg prefix glim_ros               # 必须失败
```

解析关键文件而不启动硬件：

```bash
python3 -m py_compile src/navigation/fruit_nav_bringup/launch/*.py
python3 -m py_compile src/fruit_mission/launch/*.py src/fruit_waypoint_recorder/launch/*.py
python3 - <<'PY'
from pathlib import Path
import yaml

paths = [
    Path('src/navigation/fruit_nav_bringup/config/reality/pointlio_params.yaml'),
    Path('src/navigation/fruit_nav_bringup/config/reality/sensor_frames.yaml'),
    Path('src/navigation/fruit_navigation/config/axis_navigation.yaml'),
    Path('src/fruit_mission/config/fruit_mission.yaml'),
    Path('src/fruit_waypoint_recorder/config/waypoint_recorder.yaml'),
    Path('src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml'),
]
for path in paths:
    with path.open(encoding='utf-8') as stream:
        yaml.safe_load(stream)
print('YAML parse OK')
PY
```

检查单一 waypoint 合同：文件存在、`frame_id == odom`、所有点含 `name`/`position`/`orientation`/`type`，且 mission/recorder 配置指向同一绝对路径。

检查 ROS 运行图的静态契约：

```bash
ros2 launch fruit_nav_bringup fruit_nav.launch.py --show-args
# 需要短暂加载 launch 时使用超时，不能让验证命令永久占用终端：
timeout --foreground 10s ros2 launch fruit_nav_bringup fruit_nav.launch.py \
  use_rviz:=false use_teleop:=false || test $? -eq 124
```

在真实硬件启动前只做 launch 文件加载检查；实机启动前必须 source 新 runtime，并确认 `/dev/left_cam`、`/dev/right_cam`、`/dev/stm32`。

源码和动态库边界检查：

```bash
ldd install/point_lio/lib/point_lio/pointlio_mapping | rg '/usr/local/lib/libopencv|opencv' || true
```

不得出现 `/usr/local` OpenCV；同一 composable 图像容器继续使用系统 OpenCV 4.5.4。

## 11. Git 提交顺序与回滚

建议拆成可审查的提交：

1. `preserve mapping branch and move packages`：包目录迁移、Livox submodule 调整、bringup 重命名。
2. `remove mapping and localization stack`：删除 GLIM/GICP/map/relocalization 包、资源和脚本。
3. `make odom runtime default`：参数、launch、mission、UI、waypoint 单文件化和脚本参数调整。
4. `reorganize scripts and sync docs`：runtime/monitor 移入 `scripts/`、service/README/docs 更新。

每一步提交前运行 `git diff --check`、`rg` 引用审计和必要的 `colcon build`。任何一步失败时回退到该提交前的工作树，不要动 `mapping` 分支。

完成后确认：

```bash
git status --short --branch
git log --oneline --decorate -5
git branch -vv
```

最终应停留在 `main`；`mapping` 分支必须仍指向改造前的提交，除非用户另行要求合并或删除。

## 12. 交付前清单

- [ ] `mapping` 分支保留且未被改写。
- [ ] `main` 的 `src/navigation/` 下没有 `pb2025_sentry_nav/` 二级目录。
- [ ] Livox 驱动位于 `src/driver/livox_ros_driver2`；RealSense submodule 保留。
- [ ] `fruit_nav_bringup`、`fruit_navigation`、Point-LIO 四个运行包可被 `colcon list` 发现。
- [ ] GLIM、GICP、SLAM、map server、relocalization 运行代码和启动脚本已删除。
- [ ] 只有 `odom_waypoints.yaml` 被 mission、recorder、editor 使用。
- [ ] 所有默认 frame 为 `odom`，没有 `--map`/`--odom` 参数分支。
- [ ] 根目录没有 runtime/monitor 子脚本；它们都在 `scripts/`。
- [ ] service、README、docs 和脚本路径一致。
- [ ] 构建、Python launch 编译、YAML 解析和静态引用审计通过。
- [ ] 使用 `neat-freak` 做最终知识收尾，明确代码、文档、规则和工作区状态；不要在未获用户确认前清理分支或临时文件。

## 13. 当前工作区交接注意事项

本计划生成时 `git status` 显示 `tasks/plan.md` 和 `tasks/todo.md` 已删除。这两个文件不属于本次 odom 改造的目标，执行模型必须先确认它们的归属，不得为了让工作区变干净而自动恢复、提交或再次删除。计划文件 `ODOM_REFACTOR_PLAN.md` 是本次明确新增的唯一根目录文档。
