## 目标

直接使用 `fruit_arm_planner` 发布的实时真实夹爪中心 TCP 位姿，将相机 RGB 光学目标转换为机械臂固定 Base 目标，再通过 Planner 执行水平抓取。相机检测工具与任务链使用相同坐标语义，不再以写死的观察姿态显示 Base 坐标。

用户确认：
- `TCP (m): [0.0006, 0.1482, 0.2996], pitch=-57.69 deg` 只是实时输出的例子，不能成为运行默认值。
- TCP 已经是应用局部 Z `-0.03 m` 后的夹爪中心 E；相机底部螺丝的测量也相对于这个 E。客户端不得再次加减 3 cm。
- 实际 TCP pitch 只用于观测坐标换算；最终抓取请求仍要求绝对 `pitch_deg=0°`，末端水平。
- 本阶段只读调查与规划，未修改业务文件、未执行程序、未发运动请求。最终确认已通过 questionnaire；仍须扩展的人类执行审批。

## 范围

计划改动：
- `test_camera_detector.py`：真实 TCP 订阅、现有转换函数复用、按需复用/启动桥和 Planner、输入不可用时的坐标显示、删除静态姿态选择。
- `src/fruit_mission/fruit_mission/camera_target_planner_client.py`：核验现有 TCP 消费链；仅修正与本需求直接相关的说明及补充转换日志，不重写已满足要求的行为。
- `src/fruit_mission/config/camera_extrinsic.yaml`：仅修正“复用几何尺寸”等已过时注释，实际外参、路径、动作配置保持不变。
- `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`、`docs/PERCEPTION_MODE_COMMANDS.md`：同步坐标语义、检测工具启动及降级边界，并保留最小离线数学检查。

只读核验、不计划修改：
- `src/fruit_arm_interfaces/msg/ArmTelemetry.msg`、Planner 节点及接口文档、现有 bridge/planner launch。
- `MissionContext.camera_target_request`、`actions.observe_and_pick/pick_b_tree`、`camera_arm_target_client` 的调用路径。

非目标：
- 不修改 FK/IK、joint mapping、连杆长度、工具偏移、关节限位、预设轨迹、接口定义或线协议。
- 不重新标定相机外参，不承诺解决之前 Base-Y 约 10 cm 的实测偏差或 USB/串口掉线。
- 不改变检测模型、熟度策略、深度采样方式、单相机图像传输结构、导航/QR/A/B/C 编排、恢复次数。
- 不加入运动中定位、历史姿态插值、TF 查询、四元数接口或固定姿态/FK 回退。
- 不新增依赖、测试套件、源码扫描测试、深度 ROS mock 或第二套控制器。
- 检测工具虽然获准按需启动通信依赖，但仍不自动 Home、Look、张爪、抓取，不调用运动服务；实机抓取留给操作员另行安全确认。

## 决策

### 已核实的代码事实

1. `/arm_planner_node/joint_states` 当前源码类型是 `fruit_arm_interfaces/msg/ArmTelemetry`，不是标准 JointState。消息包含：
   - `header.frame_id=arm_base_link`；时间戳是 Planner 收到反馈后的 ROS 时间，不是硬件采样时间。
   - `tcp_position`：真实 E 的 Base XYZ，米，已含工具偏移。
   - `tcp_pitch_deg`：实际末端绝对俯仰角，度。
   - `model_joint_state` 中按名称获取 `joint_1`：已映射 yaw，弧度；不能再次逆映射。
   - 原始硬件角/速度及控制器状态元数据。
2. `ArmPlannerNode._on_arm_status` 已负责映射和 FK，只有有限且在限位内的位置反馈才发布遥测。
3. 工作区既有改动中的 `CameraTargetPlannerClient._on_joint_state/_fresh_pose/target_request` 已直接使用遥测 TCP/pitch/yaw，不再做 FK 或 `hardware_to_algo`；`JointTransformer` 仍用于硬件限位安全检查，应保留。
4. 当前共享客户端校验 Base frame、四轴名称/长度、有限位置/TCP、硬件限位及本机接收年龄。`status_max_age_sec` 来自源码 planner YAML，当前 `0.25 s`；不在新代码中写死该数值。原始 `ArmStatus` 用于停稳和夹爪驻留安全检查，应保留。
5. `test_camera_detector.camera_to_base_point` 仍写死左右 yaw、pitch=-58° 和原点；`parse_args/run/draw_annotations` 仍支持 `--preset`/p 键。外参加载失败回退单位矩阵，缺少内参回退焦距 900，这些不能作为真实坐标的依据。
6. 可以复用以下现有入口，不需要新 launch：
   - 两者均缺少：`ros2 launch fruit_arm_planner fruit_arm_debug.launch.py`，包含 bridge + planner。
   - 仅 Planner 缺少：`ros2 launch fruit_arm_planner fruit_arm_planner.launch.py`。
   - 仅桥缺少：`ros2 launch fruit_hardware_bridge fruit_hardware_bridge.launch.py`。
   - bridge launch 节点名为 `serial_controller`，Planner 为 `arm_planner_node`；串口路径来自源码 `src/auto_serial_bridge/config/protocol.yaml`，当前 `/dev/ttyACM0`。不复制其他脚本的 `/dev/stm32` 假设，也不修改当前串口配置。
7. 任务与单点抓取调试调用同一个共享转换客户端。正式 B 在第一次抓取前冻结最多两个 Base 请求；本次不改变。
8. 当前工作区大量文件已有改动，`ArmTelemetry.msg` 也是已有未跟踪文件；这些不是本计划新成果，必须保留。已用只读 Git baseline/status 确认。

### 用户逐轮确认的选择

- 范围包含共享任务链核验和独立检测工具同步。
- 使用同一帧遥测的 XYZ/pitch/yaw，继续采用静止观测：整个观测窗口和冻结前不改变姿态，不做历史插值。
- 保持 `pitch_deg=0.0`、`linear_path=false`、`duration_ms=1500`、末点 CLOSE；仍通过上位机 Planner 算 IK/轨迹，再由下位机执行。
- 检测工具复用已有桥和 Planner，仅缺少时启动；退出只结束自己启动的进程，不停止复用的外部节点。
- 最多 10 秒未获得有效 TCP，或运行中断流：告警，Base=N/A，识别/深度继续；不另起第二个桥、不替代真实位姿。有效遥测恢复后恢复显示。
- 删除 `--preset` 和 p 键，不保留假姿态兼容模式。
- 缺少有效 CameraInfo 时不显示相机 3D/Base；外参无效时不显示 Base；2D 检测、熟度、有效深度继续。不伪造内参/外参。
- 验收先构建、纯数学检查和可用时的只读遥测检查，不自动驱动硬件。

### 固定转换契约

保留现有外参及米/毫米路径：

```text
P_E_mm = R_E_C × (1000 × P_C_m) + t_E_C_mm
P_B_mm = Rz(model_Q1_rad) × Ry(-radians(tcp_pitch_deg)) × P_E_mm
         + 1000 × tcp_position_m
MoveArmCartesian.xyz = P_B_mm / 1000
MoveArmCartesian.pitch_deg = 0.0
```

直接复用 `load_camera_extrinsic`、`transform_camera_to_end_effector_mm`、`transform_end_to_base_mm`。E 就是实际夹爪中心，客户端无任何额外工具补偿。yaw 必须来自同帧模型 Q1，不能从示例 XYZ 猜测，也不能由用户选择左右姿态替代。

## 执行步骤

以下均为审批后的待执行步骤；本阶段未运行任何命令。每个构建、检查或 ROS 终端先执行：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

运行环境校验失败立即停止，不通过额外 PYTHONPATH、pip 或训练环境绕过。

1. **保护基线并核验已有任务链（无前置依赖）。**
   - 文件/符号：`camera_target_planner_client.py` 的 `__init__/_on_joint_state/_fresh_pose/target_request/is_stationary`；`mission_context.py::camera_target_request`；`actions.py::observe_and_pick/pick_b_tree`；`camera_arm_target_client.py` 的转换/提交段。
   - 内容：重新比对审批时工作区与记录，保留既有改动。确认任务链无需恢复旧 JointState/FK。仅将共享客户端关于“实测关节”的旧说明改成同帧 TCP；在 `target_request` 的现有日志中补充实际 TCP XYZ、观测 pitch/yaw 与最终请求 pitch，便于复算，不改服务/恢复逻辑。`camera_extrinsic.yaml` 只更新旧注释，保留外参和所有动作值。
   - 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/camera_target_planner_client.py`；`ament_flake8 src/fruit_mission/fruit_mission/camera_target_planner_client.py`。
   - 期望：语法通过，日志明确观测 pitch 与目标 0° 的区别，行为仍直接消费遥测。
   - 停止条件：发现重叠用户编辑、实际接口不同或必须修改控制/安全逻辑才能满足需求时，暂停并重新确认；不借机修复无关已有 lint。

2. **检测工具订阅真实遥测并替换静态转换（依赖步骤 1 的接口核验）。**
   - 文件/符号：`test_camera_detector.py::FrameSubscriber/load_extrinsic_transform/camera_to_base_point`，及导入区。
   - 内容：在现有 `FrameSubscriber` 中订阅外参配置的 `joint_state_topic`，使用 ArmTelemetry、匹配现有可靠 VOLATILE QoS（depth=10）。维护一份同帧 `(x_m,y_m,z_m,pitch_deg,yaw_rad)` 和接收时间；校验 frame、四轴名字/长度、有限值及原始硬件限位，新鲜度读取现有 planner 源码 YAML。仅复用 `JointTransformer.check_hardware_limits` 做安全校验，不计算映射或 FK；不实例化带运动客户端的 `CameraTargetPlannerClient` 来给只读查看器取得位姿。
   - 复用现有三项纯转换函数；删除手写左右位姿和重复矩阵计算。外参加载使用共享验证逻辑；失败报告明确原因并标记外参不可用，不回退单位矩阵。
   - 不新增模块/抽象层或新配置键；优先直接在现有类和函数内完成。
   - 检查命令：`/usr/bin/python3 -m compileall -q test_camera_detector.py`；步骤 5 的纯数学检查。
   - 期望：输入来自同帧 Planner TCP/pitch/yaw；输出保持米，缺失/过期/无效遥测不能产出真实 Base 显示。
   - 停止条件：缺少已构建 ArmTelemetry 或需要复制 FK/增加依赖时停止该检查，先完成步骤 5 构建；不能改回标准 JointState 或写死示例。

3. **接入现有依赖启动和进程所有权（依赖步骤 2）。**
   - 文件/符号：`test_camera_detector.py::run/wait_for_publisher/check_camera_process/stop_process` 及现有子进程日志管理；只读参考三个现有 launch。
   - 内容：ROS graph 有限发现后按节点及类型化遥测/ArmStatus 发布端点辨认已有 bridge/Planner；已有就复用，只有确认缺少才调用上面对应 launch。两者都缺少时可用现有 debug launch；只缺一项时不可用 debug launch 重复启动另一项。存在同名旧类型/重复发布者冲突时明确告警、禁止按“无反馈”再次开桥，Base=N/A。
   - 子进程沿用现有独立进程组与 `stop_process`，记录工具所有权；串口/Planner 日志放忽略的 `tmp/`。启动等待每项最多 10 秒，较短已有等待不延长。断流仅更新不可用状态，不新增重启/重试策略。
   - 所有依赖启动和 GUI 创建放在可靠清理路径内，保证启动失败也清理自己启动的进程；先关闭自有 Planner 再关闭自有桥。复用的节点绝不杀死或发送 STOP。查看器自身从不调用 Home/Look/运动/张爪服务。
   - 检查命令：`/usr/bin/python3 -m compileall -q test_camera_detector.py`；人工只读核对用 `ros2 node list`、`ros2 topic info -v /arm_planner_node/joint_states`、`ros2 topic info -v /fruit_hardware_bridge/arm/status`。
   - 期望：已有/仅桥/仅Planner/均无四种拓扑的启动选择明确，无重复串口或 Planner。未来操作员运行查看器时验证退出只清理自有进程。
   - 停止条件：现有外部进程正在运动、端点冲突、通信启动可能影响未知硬件状态或清理所有权不清楚时，不进行启动实测；报告限制。本任务不以启动节点来做自动硬件验收。

4. **更新检测框/鼠标坐标与界面降级（依赖步骤 2、3）。**
   - 文件/符号：`test_camera_detector.py::parse_args/draw_annotations/run`。
   - 内容：删除 `--preset`、`current_preset`、p 键及提示。每幅展示图只获取一份新鲜有效 TCP 快照，检测框与鼠标探针都使用同一快照。横幅标明 TCP 有效/不可用，界面及必要日志区分缺失、过期、非法遥测/内参/外参。
   - CameraInfo 校验有限正 fx/fy、有限 cx/cy 和对应图像尺寸；移除假定焦距用于坐标显示的回退。深度格式/尺寸/像素有效性不满足现有投影前提时不显示相机 3D/Base，保留能可靠获取的 2D/深度信息。不改当前邻域深度统计算法。
   - 外参无效但内参/深度有效时仍可显示 Cam；没有 TCP 时 Base=N/A。正常状态下显示真实 TCP 与相机→Base 结果。所有结果仍受静止观测约束，不宣称运动中精确同步。
   - 检查命令：`/usr/bin/python3 test_camera_detector.py --help`（应仅输出参数帮助，不创建 GUI、不启动 ROS 节点/通信）；`ament_flake8 test_camera_detector.py`（仅报告本次相关问题，不无关重排整个旧脚本）。
   - 期望：帮助无静态姿态选项；两处坐标显示不再使用左右假位姿；识别/熟度/深度在坐标不可用时继续。
   - 停止条件：为了显示 Base 必须伪造任何标定、使用过期/历史值或改变图像架构时，停止并保留 N/A；若帮助路径出现启动硬件副作用立即停止核查。

5. **构建与最小离线数学验收（依赖步骤 1—4；不启动通信或运动）。**
   - 文件/符号：已有 ArmTelemetry 源定义及消费包、上述变更函数；不编辑 rosidl 输出，不新建测试套件。
   - 命令：

```bash
colcon build --base-paths src --packages-select fruit_arm_interfaces fruit_arm_planner fruit_mission \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 -m compileall -q test_camera_detector.py \
  src/fruit_mission/fruit_mission/camera_target_planner_client.py
ros2 interface show fruit_arm_interfaces/msg/ArmTelemetry
```

   - 留在步骤 6 文档中的最小可运行检查，执行共享纯转换和查看器纯函数，不构造 ROS 节点、不扫描源码、不测试私有回调：

```bash
/usr/bin/python3 - <<'PY'
import math
import numpy as np
from fruit_mission.camera_target_transform import (
    load_camera_extrinsic, transform_camera_to_end_effector_mm,
    transform_end_to_base_mm,
)
from test_camera_detector import camera_to_base_point
path = '/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml'
r, t = load_camera_extrinsic(path)
assert np.allclose(transform_camera_to_end_effector_mm((0, 0, 0), r, t), t)
assert np.allclose(transform_camera_to_end_effector_mm((0, 0, 500), r, t),
                   (434.728894, 31.255715, -52.926004), atol=1e-6)
origin = (0.0006, 0.1482, 0.2996)
for yaw in (0.0, math.pi / 2, -math.pi / 2):
    for pitch_deg in (0.0, -57.69):
        pitch = math.radians(pitch_deg)
        c, s = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        ry = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        cam = (0.0058828, 0.0676498, 0.5084)
        end = np.asarray(r) @ (1000 * np.asarray(cam)) + np.asarray(t)
        expected = np.asarray(origin) + (rz @ ry @ end) / 1000
        actual = camera_to_base_point(cam, np.asarray(r), np.asarray(t),
                                      (*origin, pitch_deg, yaw))
        assert np.allclose(actual, expected, atol=1e-9)
        zero = transform_end_to_base_mm((0, 0, 0),
                                       tuple(v * 1000 for v in origin), yaw, pitch)
        assert np.allclose(np.asarray(zero) / 1000, origin, atol=1e-12)
try:
    transform_end_to_base_mm((1, 2, float('nan')), (0, 0, 0), 0, 0)
except ValueError:
    pass
else:
    raise AssertionError('nonfinite point was accepted')
print('camera/TCP/base geometry checks passed; no motion commanded')
PY
```

   - `camera_to_base_point` 第四参数按步骤 2 改为同帧 TCP 五元组，测试值仅供数学检查，yaw 是人为覆盖三个数学方向，不声称是用户示例的实测 yaw。
   - 期望：接口存在、三包构建成功、语法通过、共享函数与独立标准矩阵运算在双侧/水平/下俯情况下吻合，零局部点严格落在已偏移 TCP，不再产生额外 3 cm 补偿，非有限输入被拒绝。
   - 停止条件：构建/数学断言失败或系统 OpenCV/NumPy 校验失败时停止进入运行验收。若引入新包依赖才能通过，重新确认而不安装依赖。

6. **同步持久文档并定向检查差异（依赖步骤 1—5）。**
   - 文件/章节：`CAMERA_TARGET_TRANSFORM.md` 的当前路径、外参、配置、离线算例、集成限制；`docs/PERCEPTION_MODE_COMMANDS.md` 新增独立单相机检查说明。
   - 内容：写明 E=已含-3cm的中心、同帧模型 Q1/pitch/TCP、最终 pitch=0 与观测 pitch 的分工；将步骤 5 的最小纯数学检查存入文档。描述 `./test_camera_detector.py --model A|B`、按需启动/复用、所有权清理、删除 --preset/p、N/A 规则及静止假设。明示“会打开串口通信，但不会自动运动”，替代此前“纯相机绝不启动串口桥”的要求；保留手动验证边界，不复制文档中无关过期策略。
   - 检查命令：`git diff --check -- test_camera_detector.py src/fruit_mission/fruit_mission/camera_target_planner_client.py src/fruit_mission/config/camera_extrinsic.yaml src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md docs/PERCEPTION_MODE_COMMANDS.md`；定向查看这些文件与启动基线的差异。
   - 期望：文档与实现一致；没有改变标定、限位或策略；新增差异仅落在约定范围。
   - 停止条件：出现其他文件改动或重写用户基线内容时先隔离/报告，不提交未经授权的改动。

7. **可用时只读核对与后续操作员验收（依赖步骤 5、6）。**
   - 前置：已有节点正确构建并经操作员确认安全；不为本步自动启动 bridge/Planner，也不运行 `test_camera_arm_target.sh` 或交互 arm console。
   - 只读命令（每个终端先加载运行环境）：

```bash
ros2 node list
ros2 topic info -v /arm_planner_node/joint_states
# 只在有发布者时读取单帧，外部 timeout 不控制机械臂：
timeout 10s ros2 topic echo --once /arm_planner_node/joint_states \
  fruit_arm_interfaces/msg/ArmTelemetry
```

   - 期望：只有正确类型的遥测，frame=arm_base_link，XYZ/pitch/模型 Q1 同帧且有限。没有节点/消息则记为未验证，不以启动硬件来补齐证据。
   - 后续由操作员在静止安全环境手动验证查看器：已有全栈/仅桥/仅 Planner/无栈四种启动选择，断流 N/A/恢复，内外参缺失降级，框与鼠标一致，以及退出不影响复用节点。串口打开/关闭或 MCU 复位的硬件影响需现场核对。
   - 实际水平抓取另行授权后使用现有抓取入口，核对日志中请求 pitch=0、PTP/1500ms/CLOSE，并确认服务真实完成；不得把软件数学通过当成实机水平、精度或串口可靠性的证明。
   - 停止条件：旧/新同名话题并存、反馈陈旧、串口断开、错误/急停/未知运动或净空未确认时停止硬件验证；保留准确错误与日志，不放宽限位、延长等待或自动重试。

## 验收

- 任务转换直接消费已存在的实时 TCP/pitch/yaw，客户端不做 FK/逆映射，不重复工具偏移；示例不进入默认运行路径。
- 任务提交仍为 Base 米坐标与绝对 `pitch=0°`，PTP/1500ms/CLOSE；结果判断、STOP、实测停稳和 B 冻结请求语义不变。
- 检测工具不再有静态左右姿态选项，框与鼠标均使用一帧真实位姿；无可用遥测不显示伪造 Base。
- 桥/Planner 按需启动且复用优先，无重复串口/Planner，所有权清理准确；工具自身不发运动服务请求。
- 内参缺失不显示 3D/Base，外参无效不显示 Base，2D/熟度/可用深度不因此退出。
- 指定包构建、语法、定向差异和纯数学检查有实际记录；只读遥测与操作员 GUI/硬件验收未执行的部分必须明确列出。
- **当前证据**：只有文件/接口/调用链和 Git 基线的只读核验；本阶段没有构建、测试或运行时结果。既有文档所记上一轮模拟/构建通过不算本计划验证结果。

## 风险

- 启动基线已有大量相关改动，尤其遥测接口迁移已经发生；执行者必须在其上做最小补齐，不能把已有成果重复实现或覆盖。
- 同名话题类型从 JointState 变为 ArmTelemetry，必须构建接口与消费者并按安全程序重启；reload presets 不能迁移运行中 Python/类型。未验证的旧进程可能导致通信不匹配。
- 实时 XYZ+pitch 仍不足以完整旋转相机点，必须使用同帧模型 Q1；不增加 roll，是现有四自由度/零 roll 模型边界。
- 遥测基于模型 FK，不是独立外部位置测量，集中使用只消除客户端重复计算，不保证标定精度提升。相机固定外参来源未独立核验，已有约 10 cm 偏差仍需实测排查。
- 接收新鲜度不等于图像时刻同步；本次成立的前提是整个观测及冻结前同一静止姿态。运动中的查看器 Base 结果不能宣称可靠定位。
- 通信 launch 本身没有 Home/Look 动作，但打开串口、心跳或 MCU 复位的实机影响尚未现场验证；启动 Planner 也会暴露控制入口，外部客户端运动不属于查看器授权。必须保证安全、停稳及过程所有权。
- ROS graph 发现存在延迟；端点存在却没消息不等于节点缺少，不能因此开第二个桥。进程不明/冲突时告警并停止真实坐标显示。
- 之前执行中 `/dev/ttyACM0` 消失、读取 EOF 和执行超时 STOP 的问题仍未确认根因。本计划不修复 USB、电源、固件或机械碰撞；只保留真实错误。
- 串口桥跨消息装载顺序和 STOP 优先级仍未经端到端确认；计划通过不代表采摘闭环已可运行。自动化验收不调用运动服务，不新增非导航重试或重复恢复。
