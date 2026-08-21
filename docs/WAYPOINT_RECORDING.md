# 比赛路线打点说明

## 0. 两套场地地图与任务区域

本项目有两个不同的物理比赛场地：`map A` 和 `map B`。这里的 A/B 是地图 profile
标识，不是任务流程中的 A 区、B 区。`--map A|B` 只选择对应场地的地图、定位数据和
waypoint 文件，不会选择或跳过任何采摘区域。

采摘区域由独立的 `--flow full|ac|a|b|c` 选择。例如，`--map A --flow b` 表示在 A
场地地图上调试 B 区流程；`--map B --flow full` 表示在 B 场地地图上执行全流程。

## 0.1 场地 profile

默认模式的路径点使用 `map` 坐标系，`--odom` 模式使用 `odom` 坐标系；两者禁止混用。
A、B 在 `map_profiles.yaml` 中分别配置地图 waypoint 文件和里程计 waypoint 文件。

启动打点流程时必须显式选择场地：

```bash
./waypoints_maker.sh --map A
./waypoints_maker.sh --map B --tele
```

脚本的 ROS 节点重复启动检查使用系统自带的 `grep`，不要求安装 ripgrep（`rg`）。启动前仍
须先执行项目运行环境脚本：

```bash
source /home/rc4/fruit_ws/setup_fruit_runtime.bash
```

不使用地图和 GICP、直接按 Point-LIO 里程计打点时增加 `--odom`：

```bash
./waypoints_maker.sh --map A --odom
./waypoints_maker.sh --map B --odom --tele
```

该模式启动 MID-360、Point-LIO、`odom -> base_link`、轴向导航、RViz 和 waypoint
recorder，RViz Fixed Frame 自动设为 `odom`。启动后机器人必须保持静止；坐标适配器会
先等待第一条有效 Point-LIO 里程计，再从连续约 1.5 秒的低速、低位置/yaw 方差数据计算
平均起始位姿。稳定完成后才发布 `odom -> base_link` 并启动 recorder；收到有效里程计后
5 秒内未稳定则脚本退出，不会在漂移中的坐标系录点。
点位分别保存到：

```text
A -> src/fruit_waypoint_recorder/waypoints/A/odometry_waypoints.yaml
B -> src/fruit_waypoint_recorder/waypoints/B/odometry_waypoints.yaml
```

里程计模式每次启动都会把稳定窗口内的平均车体位置和朝向作为 `odom` 原点。打点和后续
回放必须把机器人放在同一个物理起点并保持相同朝向后再启动；起点不同会让全部点整体平移或旋转，
纯里程计没有全局地图可以自动修正。`--odom` 不读取 `field.yaml`、`field.pcd`、
`field.initial_pose.yaml`，也不启动 GICP。

地图模式脚本会同时选择对应的地图、定位 PCD、GICP 初始位姿和 waypoint 文件。起点只有小范围
xy/yaw 偏差时，优先检查 GICP registration 和初始位姿，不要为了启动位置偏差修改所有
路径点；只有场地相对地图发生平移、旋转或布置变化时才重新录制对应 profile。

## 1. 路点文件由配置决定

正式任务按地图 profile 选择 waypoint 文件。当前实际使用的文件固定为：

```text
A -> src/fruit_waypoint_recorder/waypoints/A/competition_waypoints.yaml
B -> src/fruit_waypoint_recorder/waypoints/B/competition_waypoints.yaml
```

Mission、waypoint maker、网页编辑器和 `start_fruit_mission.sh` 都跟随当前 `--map` 选择加载
对应文件。不要把地图 profile 和任务流程名称混用。

路点文件使用以下结构：

```yaml
schema_version: 1
frame_id: map
points:
- name: example
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  yaw: 0.0
  type: waypoint
```

`waypoint` 是必须经过的通行点，`goal` 是需要停车处理的目标点。名称必须唯一，坐标必须
是机器人 `base_link` 在 `map` 坐标系中的实际停车位置。

## 2. C 区路点拓扑

C 区不是任意两点直连。二维码只确定 8 个采摘任务的顺序，Mission 使用固定名称表达
可通行拓扑，点的实际位置全部取自当前 waypoint YAML 的 XY。

```text
C_entrance_passway_1 -- C_8_12 -- C_7_11 -- C_6_10 -- C_5_9 -- C_top_waypoint_1
          |                                                               |
          |                                                               |
C_entrance_waypoint_2 -- C_4_8 -- C_3_7 -- C_2_6 -- C_1_5 -- C_top_waypoint_2
```

- `C_QR` 只用于到达二维码扫描位置；扫描完成后不属于 C 命名图，也不作为路线锚点。
- 当前 YAML 中 `C_entrance_passway_1` 和 `C_entrance_waypoint_2` 是二维码附近的两个入口 waypoint。
- 每个二维码任务优先从扫描完成后的当前 `map -> base_link` TF 位置重新规划；TF 暂不可用时回退到上一项目标位姿，首项没有历史目标才停止。
- 两侧链和 entrance/top 两条横向连接共同组成双向命名图，禁止根据坐标最近邻自动加边。
- 图搜索使用 YAML XY 计算边的欧氏距离，选择当前点到候选 goal 的最短路径。
- 如果实时位置偏离 C 图但仍在 `c_graph_max_start_distance` 内，导航先到最近图边的投影点，再沿命名图继续行驶；不会从偏离位置直接斜穿到图节点。
- 中间 C goal 参与图搜索，但不会作为 `FollowWaypoints` 停靠点发送。
- 实际导航发送最近图边回图点（需要时）、最短路径经过的 waypoint 和当前任务最终 goal。
- 同通道通常直接发送最终 goal；跨通道根据实际距离选择 entrance 或 top 横向连接。
- 5 至 8 号位置可以从两侧合法停车点处理，但不能跨越障碍物直连。
- 二维码任务顺序不可按距离重排。
- FollowWaypoints 导航 action 返回失败时，记录导航失败原因，但仍完成当前目标的稳定停车、方向调整、识别和抓取；下一任务开始时再使用实时 TF 重新规划。
- 中间 waypoint 是路线通行参考点，不要求像最终 goal 一样严格停靠；当前控制器会在横向误差仍超限时保持最低线速度修正，失败后从实时位姿重试一次，距离不超过 `0.08 m` 时允许容错通过。最终 goal 仍执行严格位置和 yaw 收敛。
- action 已 accepted 但返回 `ABORTED` 时，Mission 读取 `missed_waypoints`，只重试未完成的 waypoint 后缀，最多三次；不重复已经完成的路线段。TF/GICP 等硬故障仍停车，不执行 waypoint 容错放行。
- 每项任务保存 `navigation_status`、`navigation_start_source`、`navigation_failure_reason`、
  `arrival_yaw`、`arrival_yaw_source`、`camera_side` 和目标路线，便于现场区分实时 TF、导航
  action 失败，以及机械臂左右侧判定依据。

重新打点时必须保持上述 C 区语义名称不变。只修改 XY 会自动改变边长和最短路线；若现场
可通行关系发生变化，则必须同步修改 Mission 的命名边，程序不会根据坐标猜测新拓扑。

## 3. C 区 yaw 约定

- `C_QR` 使用 YAML 中完整姿态，yaw 必须保证 D435i 正对二维码。
- `C_QR` 之后的 C 区点在正式 Mission 中使用 YAML 的位置和姿态；`C_QR` 自身只保留扫描所需姿态，
  但不加入 C 动态图。
- Mission 的命名图仍只使用 YAML 点的 XY 计算边长和最短路径；图搜索不会根据 yaw 猜测
  通道，也不会因为 yaw 改变合法拓扑。
- 图搜索结果发送给 Nav2 时，所有正式 C waypoint 和当前目标 goal 都使用正式 YAML 中
  保存的 orientation/yaw。车头不要求始终沿着当前 XY 路段正向行驶，Nav2 会在到达各点时
  收敛到该点要求的 yaw；这对 `C_top_waypoint_1`、`C_top_waypoint_2` 和 C 目标的观察方向
  尤其重要。
- 当 TF 起点落在命名图的一条边中间时，Mission 会插入一个只存在于本次请求的临时 rejoin
  点。该点没有 YAML 姿态，使用机器人重新接入所选图边的局部行进方向；接入后的正式点仍
  立即恢复使用各自 YAML yaw。
- 机械臂 `left/right` 由到位后的 `map -> base_link` yaw 决定；若到位 TF 暂不可用，回退到本次
  规划的最终 YAML yaw。正方向由 `C_entrance_passway_1 -> C_top_waypoint_1` 的 YAML 几何
  推导，不假设 C 区永远与 map X 轴平行。
- C 点的 YAML yaw 不会改变图搜索选中的 waypoint、通道或横向连接；中间 goal 在发送前按设计
  过滤。临时 rejoin 点的 yaw 只存在于本次导航请求，不回写 YAML。
- 录点器的单点/逐段调试仍使用 YAML yaw，因此 orientation 和 yaw 字段继续保留。

## 4. 正式任务分区流程

当前正式 YAML 的流程名称和顺序如下，坐标始终以 YAML 为准：

```text
A: A1 -> A2 -> A3 -> A4 -> to_B_QR_waypoint
B QR: B_QR
B 到 C: to_C_waypoint_1 -> to_C_waypoint_2 -> to_C_waypoint_3 -> C_QR
C: C_entrance_passway_1 / C_top_waypoint_1 / C_top_waypoint_2 /
   C_entrance_waypoint_2，以及 C_8_12 ... C_4_8
C 回 B: C_entrance_waypoint_2 -> to_B_waypoint_1 -> to_B_waypoint_2
       -> toB_waypoint_3 -> B1
B 树位: B1 -> B2 -> B3 -> B4 -> B5 -> B6 -> B7 -> B8
```

`C_QR` 只负责停车扫码。C 任务完成后，`full` 才沿 C 到 B 过渡返回 B；`ac` 在 C
完成后结束。`a` 在 `to_B_QR_waypoint` 结束，`b` 从 `B_QR` 扫码后执行 B1-B8，
再到 `to_C_waypoint_2 -> to_C_waypoint_3` 结束，`c` 在 `C_QR` 扫码并完成 C 后结束。
B 区观察方向按车辆视角交替：B1、B3、B5、B7 使用 `right`，B2、B4、B6、B8 使用 `left`。

## 5. RViz 录点流程

1. 地图模式启动导航、GICP 重定位、RViz 和 `waypoint_recorder.launch.py`；里程计模式使用 `./waypoints_maker.sh --map A --odom`（或 `--map B --odom`），启动后保持机器人静止，脚本确认稳定 `odom -> base_link` 后才启动录点器。
2. 面板启动后自动显示录点器配置的 `waypoint_file`；自动命名读取统一的 `waypoints/template.yaml`，不读取会随比赛坐标变化的 `competition_waypoints.yaml`。
3. 将机器人移动到实际停车位置，面板会按当前 profile 和录点顺序显示下一个名称（A profile 从 `A1` 开始，B profile 从 `B1` 开始），选择 `waypoint` 或 `goal` 后记录。无需手动填写名称。
4. 点击保存后，点位按统一 `waypoints/template.yaml` 的正式名称顺序写入当前 profile 的 odom YAML；如果文件里已经有半套点位，面板会从模板中找到下一个尚未记录的名称，不会自行猜测编号。
5. 使用 Undo 撤销内存中的最后一点，使用 Save 写回当前加载的 YAML。
6. 使用 Run Next Goal 按 YAML 顺序执行到下一个 `goal`；该调试功能会使用 YAML yaw。
7. 需要编辑另一份路线时使用 Load Waypoint YAML，保存目标随加载文件一起切换。

## 6. 网页编辑器

Waypoint 网页编辑器由机器人开机服务自动启动，监听所有局域网网卡的 TCP `8000` 端口。
同一局域网中的电脑、手机或平板访问：

```text
http://<机器人局域网 IP>:8000/
```

网页不读取 PGM 地图。地图 waypoint 和 odom waypoint 都显示在各自坐标系的 XY 网格上，
并标出原点、顺序连线和每个点的朝向。文件下拉框固定提供以下四项，网页和服务端均不接受
任意文件路径：

```text
A / competition_waypoints.yaml  (frame_id: map)
A / odometry_waypoints.yaml     (frame_id: odom)
B / competition_waypoints.yaml  (frame_id: map)
B / odometry_waypoints.yaml     (frame_id: odom)
```

点表和选中点表单都显示 `x`、`y`、`yaw`。网页中的 yaw 单位固定为度；保存时服务端自动
规范到 `[-180, 180)`，转换成 YAML 使用的弧度，并同步生成平面四元数。已有点的 `z` 和
额外 YAML 字段保持不变。

点位名称和列表顺序以 `waypoints/template.yaml` 为准，页面始终显示模板中的全部 33 个点；
模板中的类型作为缺失点补充时的默认类型，补充后仍可编辑。当前 YAML 已有的点正常显示；
缺少的点在列表中显示为灰色，在 XY 网格中显示为浅色
占位点。占位坐标来自 A 的正式 `A/competition_waypoints.yaml`：服务端按同名点计算其相对
`A1` 的 XY/Z 偏移。当前文件已有 `A1` 时，以该点平移整套缺失点预览；当前文件也缺少 `A1`
时，临时以 `(0, 0, 0)` 作为 A1 预览位置。占位点只用于编辑提示，不会在保存时写进 YAML。

使用流程：

1. 从下拉框选择文件，在表格或网格中按模板名称选择点；名称固定为只读，不能创建模板外名称。
2. 已有点可直接修改类型、X、Y 或 Yaw；也可在网格中拖动点，拖动选中点的朝向手柄修改 yaw。
3. 选中灰色缺失点后点击 `+`，网页按浅色预览的坐标和模板名称补充该点，并立即用实际点替换占位点；若当前选中点已存在，`+` 会选择并补充其后的第一个缺失点。
4. 使用 `×` 确认后删除已有点；该模板名称会立即恢复为灰色占位点。
5. 保存只写入实际点，并强制按模板顺序排列；灰色占位点不会写入 YAML。
6. 页面显示“未保存”时，点击“保存”才会写回 YAML；重新加载、切换文件或关闭页面前会提示放弃未保存修改。
7. 每次保存前，旧文件备份到 `tmp/waypoint_editor_backups/<A|B>/`，然后原子替换正式 YAML。

该页面按可信局域网工具设计，不提供登录；任何能够访问机器人 `8000` 端口的设备都可以修改
上述四个文件，不要把该端口暴露到公网。默认按单客户端使用，多个页面同时编辑时后保存者会覆盖
先保存者。

服务管理命令：

```bash
sudo systemctl status waypoint-editor.service
sudo systemctl restart waypoint-editor.service
journalctl -u waypoint-editor.service -n 100 --no-pager
curl http://127.0.0.1:8000/health
```

服务启动会复用 `/home/rc4/fruit_ws/setup_fruit_runtime.bash`。修改网页服务端、launch 或
package 入口后先重新构建 `fruit_waypoint_recorder`，再重启服务；只修改根目录 HTML 时直接
刷新浏览器即可。

## 7. 现场检查

- [ ] `map_profiles.yaml` 中所选 profile 的 `mission_waypoint_file` 和 `recorder_waypoint_file`
      都指向预期的 `competition_waypoints.yaml`。
- [ ] 所有名称唯一，`schema_version` 为 1；地图模式 `frame_id` 为 `map`，里程计模式为 `odom`。
- [ ] 里程计打点使用对应 profile 的 `odometry_waypoint_file`，`frame_id` 为 `odom`；后续复用时必须从同一个物理起点、同一个朝向启动。
- [ ] `waypoint-editor.service` 为 enabled/active，局域网设备可以访问机器人 TCP `8000`。
- [ ] 网页编辑 yaw 时使用角度，保存后的 YAML `yaw` 和 `orientation` 使用一致的弧度姿态。
- [ ] 网页始终按 `template.yaml` 显示 33 个名称；缺失点为灰色，补点后占位显示被实际点替换。
- [ ] 启动 odom 打点后保持机器人静止，终端出现 `stable odom origin is ready` 后再移动或录点。
- [ ] `C_QR` 的 yaw 能稳定识别二维码。
- [ ] `C_entrance_passway_1` 和 `C_entrance_waypoint_2` 均位于二维码附近的可通行入口连接处。
- [ ] 两条 C 通道中的复合采摘点编号与现场位置一致。
- [ ] `C_top_waypoint_1`、`C_top_waypoint_2` 能安全完成 top 侧换道。
- [ ] 模拟二维码顺序后，同通道只发送最终 goal，跨通道包含正确的入口或 top waypoint。
- [ ] 模拟 TF 暂不可用时，首个 C 目标因没有历史位姿而停止；已有 C 目标时使用上一项目标位姿作为规划起点。
- [ ] 模拟导航 action 失败后，当前目标仍执行稳定停车、方向调整、识别和抓取；下一项按二维码顺序重新读取当前 TF。
- [ ] C 任务记录包含 `arrival_yaw`、`arrival_yaw_source` 和 `camera_side`，能复盘机械臂左右侧判定依据。
