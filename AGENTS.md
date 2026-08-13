# 本机sudo密码是222331
# 比赛相关
## 比赛项目说明

本项目用于参加中国机器人大赛暨 RoboCup 机器人世界杯中国赛 2026 年度选拔赛的“农业机器人 - 采摘机器人”项目。

本文件中的比赛理解来自 [`docs/2026水果采摘.pdf`](docs/2026水果采摘.pdf)。开发、调试和方案讨论应以原规则及现场裁判的最终解释为准。本文件的作用是把规则转换为工程约束和可验证的任务定义，不替代原规则。
## 涉及到比赛规则请看`RULE.md`

# 代码相关

## 功能包对应的配置文件放到config中
节点/launch，加载配置文件/map之类的文件，不要使用install的位置，直接指定src的位置

## USB摄像头设备名
- 左右USB2.0摄像头分别使用`/dev/left_cam`和`/dev/right_cam`，相关配置不要使用动态的`/dev/videoN`或机器相关的`/dev/v4l/by-path`路径。
- 两台摄像头没有硬件序列号，udev名称绑定物理USB端口，不要交换左右摄像头的插口。

## 本项目是ROS2项目 需要遵守ROS2的相关规范 
常使用ros2 cli 能用这个解决的问题就不要自己写
## 创建新功能包
使用ros2 cli创建新功能包

## 代码规范
### launch都按照`template.launch.py`这个模版来写
### 遵循KISS原则 非必要不加入过多的模块 非必要不加入过多的保守策略 如果要加你一定要问我这个情况是否会出现

### 每个功能包都需要有launch去启动

### 图像处理节点使用进程内通信
- 相机驱动和订阅其原始图像的 C++ 图像处理节点必须实现为 ROS 2 component，并加载到同一个 `ComposableNodeContainer` 中。
- 每个相关组件必须设置 `extra_arguments=[{'use_intra_process_comms': True}]`，避免大尺寸 `sensor_msgs/Image` 经过 DDS 跨进程序列化和分片。
- 同一容器中的组件必须链接同一套 OpenCV ABI；ROS 2 Humble 节点统一使用系统 OpenCV 4.5.4，禁止与 `/usr/local` 下的其他 OpenCV 版本混用。
- 第三方节点没有 component 接口时，先说明限制并确认方案，再使用跨进程图像传输。

# Worktree Isolation

- Before editing the project, use the `using-git-worktrees` skill to create an isolated worktree.
- Each agent must use its own worktree and branch. Never allow multiple agents to edit files in the same worktree.
- Prefer the project-local `.worktrees/` directory and verify that Git ignores it before creating a worktree.
- Run the project checks in the worktree before integrating its changes.

## 雷达补充
Point-LIO 的 extrinsic_T 和 extrinsic_R。那组参数描述的是 MID-360 内部“雷达测距坐标系到内置 IMU”的外参，不是 base_link -> front_mid360 的整机安装外参。


## 临时文件放到tmp中
