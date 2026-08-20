# 比赛路线打点说明

## 0. 两套场地地图与任务区域

本项目有两个不同的物理比赛场地：`map A` 和 `map B`。这里的 A/B 是地图 profile
标识，不是任务流程中的 A 区、B 区。`--map A|B` 只选择对应场地的地图、定位数据和
waypoint 文件，不会选择或跳过任何采摘区域。

采摘区域由独立的 `--flow full|ac|a|b|c` 选择。例如，`--map A --flow b` 表示在 A
场地地图上调试 B 区流程；`--map B --flow full` 表示在 B 场地地图上执行全流程。

## 0.1 场地 profile

路径点使用 `map` 坐标系，不能跨地图直接复用。A、B 在 `map_profiles.yaml` 中分别配置
自己的 `mission_waypoint_file` 和 `recorder_waypoint_file`；文件名和目录可以是任意
`.yaml`/`.yml` 路径，不由代码推断用途。

启动打点流程时必须显式选择场地：

```bash
./waypoints_maker.sh --map A
./waypoints_maker.sh --map B --tele
```

脚本会同时选择对应的地图、定位 PCD、GICP 初始位姿和 waypoint 文件。起点只有小范围
xy/yaw 偏差时，优先检查 GICP registration 和初始位姿，不要为了启动位置偏差修改所有
路径点；只有场地相对地图发生平移、旋转或布置变化时才重新录制对应 profile。

## 1. 路点文件由配置决定

正式任务按地图 profile 选择 waypoint 文件。`map_profiles.yaml` 中 `maps.A`、`maps.B`
分别保存两个物理场地的 `mission_waypoint_file` 和 `recorder_waypoint_file`。在某个场地
尚未完成独立录点时，可以临时让两个 profile 都指向
`src/fruit_waypoint_recorder/waypoints/test_waypoints.yaml`；完成分场地录点后，再分别指向
`waypoints/A/` 和 `waypoints/B/` 下的正式文件。Mission、waypoint maker、独立 GUI 和
`start_fruit_mission.sh` 都跟随当前 `--map` 选择加载文件。不要把地图 profile 和任务流程名称混用。

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
- 每个二维码任务都从扫描完成后的当前 `map -> base_link` TF 位置重新规划，不沿用 `C_QR` 或旧路线。
- 每项开始规划都必须取得当前 `map -> base_link` TF；TF 不可用时不能沿用旧路线或上一项目标位姿。
- 两侧链和 entrance/top 两条横向连接共同组成双向命名图，禁止根据坐标最近邻自动加边。
- 图搜索使用 YAML XY 计算边的欧氏距离，选择当前点到候选 goal 的最短路径。
- 中间 C goal 参与图搜索，但不会作为 `FollowWaypoints` 停靠点发送。
- 实际导航只发送最短路径经过的 waypoint 和当前任务最终 goal。
- 同通道通常直接发送最终 goal；跨通道根据实际距离选择 entrance 或 top 横向连接。
- 5 至 8 号位置可以从两侧合法停车点处理，但不能跨越障碍物直连。
- 二维码任务顺序不可按距离重排。
- FollowWaypoints 导航 action 返回失败时，记录导航失败原因，跳过当前目标并使用下一任务开始时的实时 TF 重新规划。
- 每项任务保存 `navigation_status`、`navigation_start_source`、`navigation_failure_reason` 和目标路线，便于现场区分实时 TF 和导航 action 失败。

重新打点时必须保持上述 C 区语义名称不变。只修改 XY 会自动改变边长和最短路线；若现场
可通行关系发生变化，则必须同步修改 Mission 的命名边，程序不会根据坐标猜测新拓扑。

## 3. C 区 yaw 约定

- `C_QR` 使用 YAML 中完整姿态，yaw 必须保证 D435i 正对二维码。
- `C_QR` 之后的 C 区点在正式 Mission 中只使用 YAML 的位置；`C_QR` 自身只保留扫描所需姿态。
- Mission 生成 waypoint 与最终 goal 列表后，按实际起点和相邻导航点的 XY 方向生成
  本次导航请求的临时 yaw。
- 动态 yaw 不会改变图搜索选中的 waypoint、通道或横向连接；中间 goal 在发送前按设计过滤。
- 临时 yaw 不回写 YAML。
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

1. 启动导航、GICP 重定位、RViz 和 `waypoint_recorder.launch.py`。
2. 面板启动后自动显示录点器配置的 `waypoint_file` 及其中全部点。
3. 将机器人移动到实际停车位置，填写固定且唯一的名称，选择 `waypoint` 或 `goal` 后记录。
4. 使用 Undo 撤销内存中的最后一点，使用 Save 写回当前加载的 YAML。
5. 使用 Run Next Goal 按 YAML 顺序执行到下一个 `goal`；该调试功能会使用 YAML yaw。
6. 需要编辑另一份路线时使用 Load Waypoint YAML，保存目标随加载文件一起切换。

## 6. 现场检查

- [ ] 两个配置中的 `waypoint_file` 指向预期文件。
- [ ] 所有名称唯一，`schema_version` 为 1，`frame_id` 为 `map`。
- [ ] `C_QR` 的 yaw 能稳定识别二维码。
- [ ] `C_entrance_passway_1` 和 `C_entrance_waypoint_2` 均位于二维码附近的可通行入口连接处。
- [ ] 两条 C 通道中的复合采摘点编号与现场位置一致。
- [ ] `C_top_waypoint_1`、`C_top_waypoint_2` 能安全完成 top 侧换道。
- [ ] 模拟二维码顺序后，同通道只发送最终 goal，跨通道包含正确的入口或 top waypoint。
- [ ] 模拟 TF 暂不可用时，不沿用旧路线继续发送。
- [ ] 模拟导航 action 失败后，当前目标被记录并跳过，下一项重新读取当前 TF。
