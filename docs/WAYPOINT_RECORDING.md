# 比赛路线航点打点与编辑说明

## 1. 航点坐标系与文件规范

本项目采用纯 Point-LIO 里程计导航架构，航点全部基于开机 `odom` 坐标系。

- **航点文件**: `src/navigation/fruit_nav_bringup/waypoints/odom_waypoints.yaml`
- **航点模板**: `src/navigation/fruit_nav_bringup/waypoints/template.yaml`
- **坐标系**: `frame_id: odom`

航点文件规范：

```yaml
schema_version: 1
frame_id: odom
points:
- name: A1
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  yaw: 0.0
  type: waypoint
```

`waypoint` 为通行途径点，`goal` 为停车作业目标点。名称必须唯一且符合 template 定义，坐标为机器人 `base_link` 在 `odom` 坐标系下的位置。

---

## 2. 启动航点录制工具

启动打点工具（若导航已启动，脚本自动复用导航；若未启动，则完整拉起导航）：

```bash
# 启动基础录制（包含雷达、Point-LIO、recorder 与 RobViz Web 面板）
./waypoints_maker.sh

# 若需配合手柄遥控移动车体打点：
./waypoints_maker.sh --tele

# 也可以直接在启动导航时附带录点器：
./start_navigation.sh --recorder
```

启动前请确保加载了运行环境：

```bash
source scripts/setup_fruit_runtime.bash
```

启动后：
1. 保持机器人静止，等待 Point-LIO 稳定并发布 `odom -> base_link` TF（约 2-3 秒）。
2. 在 RobViz 的 **Waypoints Maker** 面板中进行航点添加、保存、撤回或前往下一个航点。
3. 录制完成点击保存，数据将写入当前选择的正式或调试文件。

---

## 3. Web 航点可视化编辑器

系统提供 Web 航点编辑服务，支持通过浏览器拖拽调整与微调航点：
- 默认监听端口：`http://<机器人IP>:8000`
- 配置文件：`src/fruit_robviz_bridge/config/waypoint_editor.yaml`
- 服务源文件：`services/robviz.service`
- 静态产物：`/home/rc4/robviz/dist/index.html`

---

## 4. C 区路点拓扑


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
- 每个二维码任务优先从扫描完成后的当前 `odom -> base_link` TF 位置重新规划；TF 暂不可用时回退到上一项目标位姿，首项没有历史目标才停止。
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
- action 已 accepted 但返回 `ABORTED` 时，Mission 读取 `missed_waypoints`，只重试未完成的 waypoint 后缀，最多三次；不重复已经完成的路线段。TF 不可用或取消请求仍停车，不执行 waypoint 容错放行。
- 每项任务保存 `navigation_status`、`navigation_start_source`、`navigation_failure_reason`、
  `arrival_yaw`、`arrival_yaw_source`、`camera_side` 和目标路线，便于现场区分实时 TF、导航
  action 失败，以及机械臂左右侧判定依据。

重新打点时必须保持上述 C 区语义名称不变。只修改 XY 会自动改变边长和最短路线；若现场
可通行关系发生变化，则必须同步修改 Mission 的命名边，程序不会根据坐标猜测新拓扑。

## 5. C 区 yaw 约定

- `C_QR` 使用 YAML 中完整姿态，yaw 必须保证 camera 正对二维码。
- `C_QR` 之后的 C 区点在正式 Mission 中使用 YAML 的位置和姿态；`C_QR` 自身只保留扫描所需姿态，
  但不加入 C 动态图。
- Mission 的命名图仍只使用 YAML 点的 XY 计算边长和最短路径；图搜索不会根据 yaw 猜测
  通道，也不会因为 yaw 改变合法拓扑。
- 图搜索结果发送给 `fruit_navigation` 时，所有正式 C waypoint 和当前目标 goal 都使用正式 YAML 中
  保存的 orientation/yaw。车头不要求始终沿着当前 XY 路段正向行驶，轴向导航控制器会在到达各点时
  收敛到该点要求的 yaw；这对 `C_top_waypoint_1`、`C_top_waypoint_2` 和 C 目标的观察方向
  尤其重要。
- 当 TF 起点落在命名图的一条边中间时，Mission 会插入一个只存在于本次请求的临时 rejoin
  点。该点没有 YAML 姿态，使用机器人重新接入所选图边的局部行进方向；接入后的正式点仍
  立即恢复使用各自 YAML yaw。
- 机械臂 `left/right` 由到位后的 `odom -> base_link` yaw 决定；若到位 TF 暂不可用，回退到本次
  规划的最终 YAML yaw。正方向由 `C_entrance_passway_1 -> C_top_waypoint_1` 的 YAML 几何
  推导，不假设 C 区永远与 odom X 轴平行。
- C 点的 YAML yaw 不会改变图搜索选中的 waypoint、通道或横向连接；中间 goal 在发送前按设计
  过滤。临时 rejoin 点的 yaw 只存在于本次导航请求，不回写 YAML。
- 录点器的单点/逐段调试仍使用 YAML yaw，因此 orientation 和 yaw 字段继续保留。

## 6. 正式任务分区流程

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

## 7. Waypoint Maker 录点流程

1. 使用 `./waypoints_maker.sh`（需要手柄时加 `--tele`），启动后保持机器人静止，脚本确认稳定 `odom -> base_link` 后才启动录点器。
2. 在 RobViz 的 **航点 Maker** 区块中选择正式或调试文件。状态从 recorder 的本地话题恢复，不从浏览器缓存重放命令。
3. 录点前节点检查 TF 新鲜度、非零四元数、连续采样和采样窗口内的 TF 位姿包络。窗口内相对首个样本的平面位移超过 0.05 m 或 yaw 超过 0.10 rad 时拒绝录点。
4. 正式文件的名称和类型只能来自 `waypoints/template.yaml`。调试文件允许自由名称，但不能通过命令切换到任意路径。
5. 覆盖已有点需要界面确认和 `confirm_overwrite: true`。Undo 会恢复被覆盖的旧点，而不是删除列表末尾的其他点。
6. Save 使用当前文件版本、文件锁、备份和原子替换。磁盘被其他进程修改时保存失败，必须重新加载。
7. Run Next Goal 保留 `next_test_segment()` 的分段逻辑。只有 action 成功才推进游标。
8. Cancel 只取消当前 action goal。取消期间到达的旧 future 不会启动新目标，节点在实测停稳前保持忙状态。
9. 浏览器关闭不会自动取消已经接受的导航。现场必须保留显式取消和硬件急停。

采样期间释放长时间状态锁，里程计和取消回调继续处理。
停稳证据只使用时间戳严格递增的实际样本，不接受重复旧消息补足窗口。
取消终态后重新开始窗口，未知 Action 结果仍保持导航所有权。
切换或重载必须确认丢弃未保存草稿，确认字段只接受布尔值。
候选文件校验失败保留草稿与 Undo，成功才原子替换内存。
内容和 SHA-256 版本使用同一字节快照，陈旧保存返回冲突。
MarkerArray 锁存完整航点快照，浏览器晚连接仍能收到名称和点位。

Point-LIO 的 `/odometry.twist` 由相邻雷达位姿差分得到，静止时会放大雷达抖动，不能直接作为录点的速度阈值。录点器使用连续 `odom -> base_link` TF 的窗口位姿包络判定运动，并继续检查 TF 时间戳、四元数和采样连续性。导航取消后的安全停稳确认仍使用 `/odometry` 的独立停稳逻辑。

recorder 话题：

```text
/fruit/waypoint_maker/command   std_msgs/msg/String
/fruit/waypoint_maker/state     std_msgs/msg/String
/fruit_waypoint_recorder/prompt std_msgs/msg/String
/fruit/waypoint_maker/markers   visualization_msgs/msg/MarkerArray
```

## 8. 网页编辑器

Waypoint 网页编辑器由机器人开机服务自动启动，监听所有局域网网卡的 TCP `8000` 端口。
同一局域网中的电脑、手机或平板访问：

```text
http://<机器人局域网 IP>:8000/
```

网页不读取 PGM 地图。航点显示在 odom 坐标系的 XY 网格上。航点编辑采用**纯离线编辑模式**：仅通过 HTTP 接口读写 YAML 文件，不联动机器人实时活动（不订阅 TF/里程计，不显示实时机器人箭头），即使机器人未连接 ROS 亦可正常离线查看与编辑。文件下拉框只提供正式和调试两项，网页和服务端均不接受任意文件路径：

```text
里程计 / odom_waypoints.yaml  (frame_id: odom)
调试 / debug_waypoints.yaml   (frame_id: odom)
```

点表和选中点表单都显示 `x`、`y`、`z`、`yaw`。网页中的 yaw 单位固定为度；保存时服务端自动
规范到 `[-180, 180)`，转换成 YAML 使用的弧度，并同步生成平面四元数。已有点的额外 YAML
字段保持不变。

航点分类与交互特性：
1. **类型颜色与名称布局**：列表中 goal 采用绿色标识与 `[G]` 徽章，waypoint 采用蓝色标识与 `[W]` 徽章，缺失点采用黄色提示与 `[!]` 徽章。长航点名称在左侧列表自动换行完整展示，坐标独立分行，在不同全局字号下均不发生文字挤占与重叠。
2. **网格局部缩放与触控板手势**：网格视图支持类似 3D 视图的局部缩放与平移。支持 Mac 触控板双指捏合（Pinch-to-zoom）及普通滚轮以光标为中心缩放，支持触控板双指滑动平移与空白处拖拽平移，工具栏提供放大、缩小与“自适应居中”按钮。局部缩放和平移不改动航点坐标，不污染撤销栈。
3. **标签避让与引线机制**：网格标签采用确定性空间碰撞检测与多方向避让，密集时自动绘制引线连接点位；极密集区域提供“标签清单”侧栏查看完整列表并联动高亮聚焦。
4. **窄屏自适应**：在小屏或高倍缩放下提供“列表 / 网格 / 属性”视图切换，确保所有属性输入与保存操作均可正常访问。

正式文件的点位名称和列表顺序以 `waypoints/template.yaml` 为准，页面始终显示模板中的全部 33 个点；
当前 YAML 已有的点正常显示；缺少的点只在列表中显示为缺失状态，不进入 XY 网格，也不会获得
虚假的坐标。正式点的名称和类型不能在网页中改动。调试文件保留 YAML 中已有点的自由名称和顺序。

使用流程：

1. 从下拉框选择正式或调试文件。调试文件必须先由 recorder 创建并写入。
2. 选择已有点，直接修改 X、Y、Z 或 Yaw；调试文件还可以修改点类型。
3. 在 XY 网格中拖动已有点，只修改当前草稿的 X/Y。
4. 使用 Waypoints Maker 录制正式缺失点或新的调试点。网页编辑器不为缺失点生成坐标。
5. 页面显示“草稿已修改”时，点击“保存”并确认覆盖，才会写回 YAML。
6. 保存只写入实际点。正式文件强制按模板顺序排列，调试文件保留用户顺序。
7. 每次保存前，旧文件备份到 `tmp/waypoint_editor_backups/<profile>/`，然后原子替换目标 YAML。
8. 页面加载的文件版本与磁盘版本不一致时，保存会被拒绝，必须重新加载后再编辑。

该页面按可信局域网工具设计，不提供登录；任何能够访问机器人 `8000` 端口的设备都可以修改
上述航点文件，不要把该端口暴露到公网。多个页面同时编辑时，陈旧版本保存会被拒绝。

服务管理命令：

```bash
sudo systemctl status robviz.service
sudo systemctl restart robviz.service
journalctl -u robviz.service -n 100 --no-pager
curl http://127.0.0.1:8000/health
```

服务启动会复用 `/home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash`。修改网页服务端、launch 或
package 入口后先重新构建 `fruit_robviz_bridge`，再重启服务。

`start_navigation.sh` 和 `waypoints_maker.sh` 不启动 RViz、HTTP 服务或 Foxglove bridge。
它们只启动自身需要的导航、里程计、recorder 和调试进程，并输出 RobViz URL。

## 9. 现场检查

- [ ] 航点文件 `src/navigation/fruit_nav_bringup/waypoints/odom_waypoints.yaml` 存在且 `frame_id: odom`。
- [ ] 所有点名称唯一，`schema_version` 为 1。
- [ ] 里程计打点使用 `odom_waypoints.yaml`；后续复用时必须从同一个物理起点、同一个朝向启动。
- [ ] `robviz.service` 为 enabled/active，局域网设备可以访问机器人 TCP `8000`。

- [ ] 网页编辑 yaw 时使用角度，保存后的 YAML `yaw` 和 `orientation` 使用一致的弧度姿态。

导航使用航点 `orientation/yaw` 作为**车体**最终朝向，不是机械臂朝向。当前默认先沿点间直线到点并减速停车，再原地转到该朝向（`final_yaw_mode: rotate_at_goal`）；航点坐标和保存的 yaw 不需要为此修改。转弯时仍须为整机轮廓预留空间，详见 [直线跟踪与到点判据](ODOMETRY_NAVIGATION.md#21-直线跟踪与到点判据)。
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
