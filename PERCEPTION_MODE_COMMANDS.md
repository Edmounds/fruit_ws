# 感知区域切换命令

以下命令用于手动切换比赛不同区域的感知模式。每个新终端先加载项目运行环境：

```bash
cd /home/rc4/fruit_ws
source setup_fruit_runtime.bash
```

确认 `fruit_perception` 只启动了一套后，再执行对应阶段的一条切换命令。

## A 区作业

使用左右 USB 摄像头：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'SIDES'}"
```

对应设备：`/dev/left_cam`、`/dev/right_cam`。

## B 区

### B 区入口扫码

使用 D435i 读取二维码：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'QR'}"
```

### 返回 B 区进行果树作业

使用 D435i 检测整树果实：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'TREE'}"
```

## C 区

### C 区入口扫码

使用 D435i 读取二维码：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'QR'}"
```

### C 区作业

使用左右 USB 摄像头：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'SIDES'}"
```

## 任务结束

关闭全部感知采集和推理：

```bash
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String \
  "{data: 'OFF'}"
```

## 查看当前模式

模式切换成功后，可以查看当前生效模式：

```bash
ros2 topic echo --qos-durability transient_local --once \
  /fruit_perception/active_mode std_msgs/msg/String
```

期望输出为 `OFF`、`SIDES`、`QR` 或 `TREE` 之一。当前仓库还没有自动根据定位区域发布这些命令的 `fruit_mission` 节点，因此比赛流程暂时需要由任务节点或人工发布模式。
