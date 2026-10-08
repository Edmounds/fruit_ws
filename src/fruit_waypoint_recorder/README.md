# fruit_waypoint_recorder

该功能包通过 recorder 节点编辑、显示、保存并逐段试跑显式 waypoint YAML。

节点从 `config/waypoint_recorder.yaml` 加载正式 `odom` 文件、调试文件和正式模板。
运行时只允许在 `odom` 与 `debug` 两个配置文件之间切换。
配置路径必须位于项目 `src` 下且文件已经存在。

面板支持：

- 在正式文件和调试文件之间切换；
- 从当前 `odom -> base_link` TF 采样并记录 `waypoint` 或 `goal`；
- 连续采样期间检查 TF 新鲜度、四元数、里程计速度和实测停稳；
- 撤销追加或覆盖，并恢复被覆盖的旧点；
- 使用版本检查、备份和原子替换保存到当前文件；
- 按 YAML 顺序执行到下一个 `goal`；
- 在 RobViz 中显示点名、类型和 YAML yaw。旧 RViz 插件仍保留用于兼容调试。

正式文件的名称和类型来自 `waypoints/template.yaml`，保存时按模板顺序排列。
调试文件允许自由名称和顺序，不能覆盖正式文件。
当前加载的文件就是保存目标，不维护独立的“正式点”和“测试点”两套内存数据。
逐段试跑游标只记录本次调试进度，不写入 YAML。

状态和控制话题：

```text
/fruit/waypoint_maker/command   std_msgs/msg/String
/fruit/waypoint_maker/state     std_msgs/msg/String
/fruit_waypoint_recorder/prompt std_msgs/msg/String
/fruit/waypoint_maker/markers   visualization_msgs/msg/MarkerArray
```

控制消息使用 JSON 对象。录点覆盖必须发送布尔值
`confirm_overwrite: true`，字符串值不会被接受。
导航取消只作用于当前已接受的 action goal。
取消后节点等待新鲜里程计持续满足停稳窗口，未确认时不允许下一段运行。
窗口终点使用最后实际收到的里程计样本，不让时间流逝补足停稳证据。
窗口必须从 Action 取消终态之后开始。
Action 接受或结果未知时保留导航所有权，`navigation.unknown` 为 true。
下一段保持 blocked，不能将通信失败视为运动完成。

录点采样不持有长时间状态锁，里程计和取消回调可以继续处理。
切换文件和重载前必须确认未保存草稿的丢弃。
`confirm_discard` 和兼容字段 `force` 只接受布尔值。
候选文件全部校验成功后才替换内存，加载失败保留草稿和 Undo。
文件内容与 SHA-256 版本来自同一字节快照。
MarkerArray 使用 TRANSIENT_LOCAL，新订阅者直接获得完整点位和名称。

构建和启动：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --symlink-install --packages-select \
  fruit_waypoint_recorder fruit_waypoint_rviz_plugin
source scripts/setup_fruit_runtime.bash
ros2 launch fruit_waypoint_recorder waypoint_recorder.launch.py
```

录点器逐段试跑会使用 YAML 中的 yaw。正式 Mission 对 `C_QR` 保留 YAML yaw，对其后的 C 区
路点和当前 C goal 使用各自 YAML 中保存的 yaw；只有 TF 位于命名图边中间时临时插入的
rejoin 点使用局部边方向；详见 `docs/WAYPOINT_RECORDING.md`。
