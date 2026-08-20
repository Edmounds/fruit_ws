# 本机sudo密码是222331
# 不要写任何tests
# 比赛相关
## 比赛项目说明

本项目用于参加中国机器人大赛暨 RoboCup 机器人世界杯中国赛 2026 年度选拔赛的“农业机器人 - 采摘机器人”项目。

本文件中的比赛理解来自 [`docs/2026水果采摘.pdf`](docs/2026水果采摘.pdf)。开发、调试和方案讨论应以原规则及现场裁判的最终解释为准。本文件的作用是把规则转换为工程约束和可验证的任务定义，不替代原规则。
## 涉及到比赛规则请看`docs/RULE.md`

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

## GUI 调试显示
- 本机经常通过远程终端调试；凡是需要打开 GUI 窗口的脚本或命令，默认显示到机器人实际连接的桌面屏幕。
- GUI 启动环境按现有启动脚本约定设置：`DISPLAY=:0`、`XDG_RUNTIME_DIR=/run/user/$(id -u)`、`XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority`、`DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus`。
- Python/Tkinter 等单文件 GUI 工具应在创建窗口前设置上述环境变量，避免远程终端没有 `DISPLAY` 时窗口无法打开。

## 代码规范
### launch都按照`template.launch.py`这个模版来写
### 遵循KISS原则 非必要不加入过多的模块 非必要不加入过多的保守策略 如果要加你一定要问我这个情况是否会出现

### 每个功能包都需要有launch去启动

### 测试与配置
- YAML 配置用于现场调参，普通参数没有永久不变的绝对值。禁止通过测试断言某个普通 YAML 参数必须等于当前数值、开关、分辨率、阈值或话题名，以免正常调参导致测试失败。
- 配置测试应验证 YAML 可解析、必需字段存在、类型或取值范围有效，以及确有必要的跨字段关系；不要复制一份当前配置值到测试中。
- 通信协议 ID 等稳定契约应测试其权威接口定义，不要通过读取部署或调参 YAML 后复制当前值来间接测试。

### 图像处理节点使用进程内通信
- 相机驱动和订阅其原始图像的 C++ 图像处理节点必须实现为 ROS 2 component，并加载到同一个 `ComposableNodeContainer` 中。
- 每个相关组件必须设置 `extra_arguments=[{'use_intra_process_comms': True}]`，避免大尺寸 `sensor_msgs/Image` 经过 DDS 跨进程序列化和分片。
- ROS 2 Humble 的进程内通信只支持 `VOLATILE` durability。启用进程内通信的 component 一旦创建 `TRANSIENT_LOCAL` 发布器或订阅器，构造函数会抛出 `intraprocess communication allowed only with volatile durability`，导致组件加载失败。
- 对确实需要 `TRANSIENT_LOCAL` 的状态/控制话题，必须保留其 QoS 语义，并只对该发布器或订阅器关闭进程内通信：使用 `rclcpp::PublisherOptions` 或 `rclcpp::SubscriptionOptions`，将 `use_intra_process_comm` 设为 `rclcpp::IntraProcessSetting::Disable` 后传给 `create_publisher`/`create_subscription`。这样低频状态话题走 DDS，而图像话题继续走进程内通信。
- 禁止为了消除上述异常把需要锁存的状态话题改成 `VOLATILE`，也禁止关闭整个相机/图像处理 component 的进程内通信。给已启用进程内通信的 component 新增发布器或订阅器时，必须检查其 history、depth 和 durability 是否满足 ROS 2 Humble 的进程内通信限制。
- 同一容器中的组件必须链接同一套 OpenCV ABI；ROS 2 Humble 节点统一使用系统 OpenCV 4.5.4，禁止与 `/usr/local` 下的其他 OpenCV 版本混用。
- 第三方节点没有 component 接口时，先说明限制并确认方案，再使用跨进程图像传输。

## Python 与 OpenCV 环境边界

本机同时存在多套计算机视觉环境，不能在同一个 ROS 2 进程中混用：

- ROS Humble 的 `cv_bridge`、apt `python3-opencv` 和本项目 C++ 节点统一使用系统 OpenCV 4.5.4；apt NumPy 为 1.x。
- `~/.local` 中的 NumPy 2.x、pip OpenCV、Ultralytics 和 OpenVINO Python 工具只用于训练/导出，不得进入 ROS 运行进程。
- `/usr/local` 中另有 OpenCV 4.10，且其 Python `cv2` 依赖不同的 OpenVINO ABI；ROS 构建和运行均不得从 `/usr/local` 加载 OpenCV、`cv2` 或相关动态库。

执行规则：

- 启动或实机调试本项目之前必须 `source /home/rc4/fruit_ws/setup_fruit_runtime.bash`，不要只 source `install/setup.bash`。正式启动脚本也必须复用该环境脚本。
- ROS 环境脚本必须在启用 Bash `nounset`（`set -u`）之前 source；`/opt/ros/humble/setup.bash` 可能读取尚未定义的 `AMENT_TRACE_SETUP_FILES`。启动脚本应先使用 `set -eo pipefail`，source 完整运行环境后再执行 `set -u`。
- 环境脚本会清除继承的其他工作区、Python 和动态库路径，禁用 Python user site，并检查 Python OpenCV 4.5.4 与 NumPy 1.x 都来自 `/usr/lib/python3/dist-packages`。检查失败时应停止启动并修复环境，禁止临时改 `PYTHONPATH` 绕过。
- 禁止在系统 Python 或 `~/.local` 执行 `pip install`、`pip install --user`。训练、标注、模型导出等非 ROS Python 工具必须使用项目外独立 venv，并在该 venv 内维护 requirements/lock 文件。
- 不要在已激活训练 venv 的终端中构建或启动 ROS；退出 venv 后重新 source `setup_fruit_runtime.bash`。
- ROS Python 包依赖通过 `package.xml` 声明并使用 apt/rosdep 安装。需要与 `sensor_msgs/Image` 交互时优先使用与系统 ABI 一致的 `cv_bridge`；若固定编码可安全直接解析，则必须验证 encoding、step 和数据长度。
- C++ 包继续通过 CMake 精确限定系统 OpenCV 4.5.4，并在合入前用 `ldd` 确认没有链接 `/usr/local/lib/libopencv*`。

# Worktree 使用

- 机器人实机运行只认 `main` 分支；开发分支只用于短期开发，完成并验证后必须合回 `main`，不要通过切换分支来使用不同功能版本。
- 功能分支合回 `main` 后应删除本地开发分支，避免后续误用旧代码；未合回的实验分支必须明确说明是否保留。
- 只有大工程才要求使用 `using-git-worktrees` skill 建立隔离 worktree，例如跨多个功能包、改动文件较多、风险较高或需要并行开发的任务。
- 修改配置、文档、单个测试或简单修复等小范围工作，由执行者根据影响范围判断，可以直接在当前工作区和当前分支完成，不需要为了形式额外创建 worktree。
- 当前工作区存在未提交修改时必须保留无关改动；若任务与这些修改重叠且无法可靠隔离，再创建 worktree。
- 多个 agent 并行修改代码时，每个 agent 必须使用独立 worktree 和分支，禁止同时编辑同一个工作区。
- 使用 worktree 完成的工作必须先通过相关检查，再合并进用户当前正在使用的分支，不能只留在功能 worktree 中。
- 创建 worktree 时优先使用项目内 `.worktrees/` 目录，并先确认 Git 已忽略该目录。

# 仓库结构

- `src/joy_linux` 是本项目内的普通 ROS2 功能包，禁止作为 git submodule 或独立嵌套 git 仓库维护。

## 雷达补充
Point-LIO 的 extrinsic_T 和 extrinsic_R。那组参数描述的是 MID-360 内部“雷达测距坐标系到内置 IMU”的外参，不是 base_link -> front_mid360 的整机安装外参。


## 临时文件放到tmp中
