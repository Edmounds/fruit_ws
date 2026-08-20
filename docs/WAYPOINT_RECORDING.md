# 比赛路线打点说明

## 0. A/B 场地 profile

路径点使用 `map` 坐标系，不能跨地图直接复用。A、B 在 `map_profiles.yaml` 中分别配置
自己的 waypoint file；文件名和目录可以是任意 `.yaml`/`.yml` 路径，不由代码推断用途。

启动打点流程时必须显式选择场地：

```bash
./waypoints_maker.sh --map A
./waypoints_maker.sh --map B --tele
```

脚本会同时选择对应的地图、定位 PCD、GICP 初始位姿和 waypoint 文件。起点只有小范围
xy/yaw 偏差时，优先检查 GICP registration 和初始位姿，不要为了启动位置偏差修改所有
路径点；只有场地相对地图发生平移、旋转或布置变化时才重新录制对应 profile。

## 1. 路点文件由配置决定

正式任务不依赖固定文件名。Mission 从
`src/fruit_mission/config/fruit_mission.yaml` 的 `waypoint_file` 加载路线，录点器从
`src/fruit_waypoint_recorder/config/waypoint_recorder.yaml` 的 `waypoint_file` 加载和保存路线。

两个参数当前可以指向同一份现场文件，也可以分别指向不同文件。更换路线文件时只修改
配置，不修改 Python、C++、launch 或启动脚本。

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

C 区不是任意两点直连。二维码只确定 8 个采摘任务的顺序，Mission 必须使用路点文件中
已有的入口、通道和换道点生成每一段 `movement_points`。

```text
C_QR
  |-- to_C_entrance ------ C_8_12 -- C_7_11 -- C_6_10 -- C_5_9
  |                                                        |
  |                                              C_passway_1
  |                                                        |
  |                                              C_passway_2
  |                                                        |
  `-- to_C_other_entrance - C_4_8 -- C_3_7 -- C_2_6 -- C_1_5
```

- 首个任务先经过对应入口，再沿所在通道逐点到达目标。
- 同一通道移动必须包含当前位置与目标之间的所有中间点。
- 跨通道必须走到通道端部并依次经过 `C_passway_1`、`C_passway_2`。
- 5 至 8 号位置可以从两条通道中的合法停车点处理，但不能跨越障碍物直连。
- 二维码任务顺序不可按距离重排。

新增或修改现场点位时，应在配置指向的 YAML 中显式记录其 XY。若新点改变了 C 区通道
拓扑，还必须同步修改 Mission 中的通道名称列表；程序不会根据坐标猜测可通行关系。

## 3. C 区 yaw 约定

- `C_QR` 使用 YAML 中完整姿态，yaw 必须保证 D435i 正对二维码。
- `C_QR` 之后的 C 区点在正式 Mission 中只使用 YAML 的位置。
- Mission 展开完整 `movement_points` 后，按每一对相邻点的 XY 方向生成本次导航请求的
  临时 yaw。
- 动态 yaw 不会删除中间点、改变通道或把两个采摘点直接相连。
- 临时 yaw 不回写 YAML。
- 录点器的单点/逐段调试仍使用 YAML yaw，因此 orientation 和 yaw 字段继续保留。

## 4. RViz 录点流程

1. 启动导航、GICP 重定位、RViz 和 `waypoint_recorder.launch.py`。
2. 面板启动后自动显示录点器配置的 `waypoint_file` 及其中全部点。
3. 将机器人移动到实际停车位置，填写唯一名称，选择 `waypoint` 或 `goal` 后记录。
4. 使用 Undo 撤销内存中的最后一点，使用 Save 写回当前加载的 YAML。
5. 使用 Test Next 按 YAML 顺序执行到下一个 `goal`；该调试功能会使用 YAML yaw。
6. 需要编辑另一份路线时使用 Load Waypoint YAML，保存目标随加载文件一起切换。

## 5. 现场检查

- [ ] 两个配置中的 `waypoint_file` 指向预期文件。
- [ ] 所有名称唯一，`schema_version` 为 1，`frame_id` 为 `map`。
- [ ] `C_QR` 的 yaw 能稳定识别二维码。
- [ ] `to_C_entrance` 和 `to_C_other_entrance` 均位于可通行入口。
- [ ] 两条 C 通道中的复合采摘点编号与现场位置一致。
- [ ] `C_passway_1`、`C_passway_2` 能安全完成两条通道之间的换道。
- [ ] 模拟二维码顺序后，每段路线均沿已有点行驶且跨通道必经两个 passway。
