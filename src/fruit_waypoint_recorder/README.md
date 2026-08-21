# fruit_waypoint_recorder

该功能包通过 RViz 面板编辑、显示、保存并逐段试跑一份显式 waypoint YAML。

节点从 `config/waypoint_recorder.yaml` 的 `waypoint_file` 参数加载默认文件。文件名不是代码
契约；修改配置即可切换到其他 `.yaml` 或 `.yml` 文件。配置路径必须位于项目 `src` 下且
文件已经存在。

面板支持：

- 加载另一份 waypoint YAML；
- 从当前 `map -> base_link` TF 采样并记录 `waypoint` 或 `goal`；
- 撤销内存中的最后一点；
- 保存到当前加载的文件；
- 按 YAML 顺序执行到下一个 `goal`；
- 在 RViz 中显示点名、类型和 YAML yaw。

当前加载的文件就是保存目标，不再维护独立的“正式点”和“测试点”两套内存数据。
逐段试跑游标只记录本次调试进度，不改变文件用途，也不会写入 YAML。

构建和启动：

```bash
cd /home/rc4/fruit_ws
source setup_fruit_runtime.bash
colcon build --symlink-install --packages-select \
  fruit_waypoint_recorder fruit_waypoint_rviz_plugin
source setup_fruit_runtime.bash
ros2 launch fruit_waypoint_recorder waypoint_recorder.launch.py
```
录点器逐段试跑会使用 YAML 中的 yaw。正式 Mission 对 `C_QR` 保留 YAML yaw，对其后的 C 区
路点和当前 C goal 使用各自 YAML 中保存的 yaw；只有 TF 位于命名图边中间时临时插入的
rejoin 点使用局部边方向；详见 `docs/WAYPOINT_RECORDING.md`。
