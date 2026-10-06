# mission 相机目标换算与 planner 对接

## 当前路径与边界

```text
FruitDecision（RGB 光学坐标，米，保留图像 header）
  -> mission 选中目标
  -> CameraTargetPlannerClient
       camera -> 夹爪中心 E（mm）
       planner ArmTelemetry -> 已校正模型偏航、真实 TCP xyz/pitch
       E -> 机械臂固定基座 B（mm）
       按区域应用末端相对偏置 target_offset_ac_m / target_offset_b_m（米，换算为 mm）
       B 坐标 / 1000 -> MoveArmCartesian（米）
  -> /arm_planner_node/move_cartesian
  -> 等待 response.success / message / trajectory_id
```

正式 `MissionContext` 和单相机调试客户端共用同一入口，不再发送旧 `ArmTarget`。
正式 mission 的左右观察、二维码、收臂、恢复动作也通过该入口调用
`/arm_planner_node/execute_preset`，与目标运动共用结果校验及未知结果时的 STOP 处理。
FK 和反馈逆映射在 `fruit_arm_planner` 中完成，mission 直接读取遥测，不重复减偏置、FK、IK 或轨迹规划；固定夹爪下偏由 planner 的 geometry 统一定义。

用户确认观察窗口期间底盘与机械臂物理静止。**整个多帧观测窗口及观测结束到提交请求之间必须维持同一观测姿态**，才可用提交前的新鲜实测 TCP 遥测转换目标；不缓存/插值历史姿态。若以后改为运动中检测，需要重新设计同步，不能继续沿用此假设。

`/arm_planner_node/joint_states` 的消息类型现为 `fruit_arm_interfaces/msg/ArmTelemetry`，话题名不变。`hardware_joint_state` 保留原始硬件角，`model_joint_state` 已消除 direction/offset 并还原相对肘角，单位 rad。mission 只用同一帧模型 Q1 和 `tcp_position/tcp_pitch_deg`（m、度）做 E -> B，不再自行逆 mapping 或 FK；输入校验仍要求原始角在硬件限位内，模型角/TCP 有限且反馈新鲜。原始 ArmStatus 订阅保留给停稳、安全恢复和硬件角驻留服务，不把其角度误当模型角。

## 新外参评估（单位 mm）

三个原点：E = 夹爪抓取中心，S = D435i 底部螺纹孔表面圆心，C = RGB 光学原点。
E 和 S 的轴同向：局部 X 前、Y 左、Z 上；这些轴随末端运动，并非永远与基座/地面平行。
C 的轴为 X 右、Y 下、Z 前。

用户确认螺纹孔 S 在抓取中心 E 的后方 75.413 mm、末端局部上方
66.726 mm。建模图中，S 比腕轴高 36.726 mm，E 比腕轴低 30 mm，
因此 E 指向 S 的局部 Z 为 `+(36.726 + 30) = +66.726 mm`；
这些是末端局部尺寸，不是运动后的基座轴投影。对应的双向平移为：

```text
S -> E = [75.413, 0, -66.726]
E -> S = [-75.413, 0, 66.726]
S -> C = [10.144894, 32.458360, 12.615321]
E -> C = [-65.268106, 32.458360, 79.341321]
```

后两个机身/光学数据及旋转沿用
`tmp/vison/Engineer/APPLICATION/camera_target_transform_config.h`。
旧文件将其称为 official，但本次未独立核验厂家资料或当前设备标定；不能把继承值称为本次实机标定结果。
若相机型号、机身光学标定或安装旋转改变，需要复核这些固定值。

方向反转、平移相加正确，前提是已确认 E/S 轴平行同向。测量原点已经是夹爪中心，**不能再减旧固件的 117 mm 工具长度**。

在上述前提下：

```text
P_E_mm = R_E_C * P_C_mm + t_E_C_mm

R_E_C =
[-0.00239767   0.00237706   0.99999400]
[-0.99999200   0.00320883  -0.00240529]
[-0.00321453  -0.99999200   0.00236935]

t_E_C_mm = [-65.268106, 32.458360, 79.341321]
```

旋转矩阵通过正交性和 determinant=+1 的舍入容差校验。`T_E_C` 的含义是**把 C 系点转换到 E 系**，不要因命名中的 E/C 次序而误求逆。

第二步（与现有 FK 的向上俯仰为正约定一致）：

```text
R_B_E = Rz(yaw) * Ry(-pitch)
P_B_mm = R_B_E * P_E_mm + 1000 * planner_TCP_position_m
```

所以相机到夹爪的固定矩阵本身并不足以提交 planner；必须加上观测姿态的第二步。

第三步由 mission 的 `fruit_mission.yaml` 分区域配置末端相对偏置（米）：
A/C 采用 `target_offset_ac_m`；B 区分 Up/Down 采用 `target_offset_b_up_m` 与 `target_offset_b_down_m`，数值以源码配置为准。
局部坐标系定义：+X 为夹爪正前方深入，+Y 为夹爪左侧，+Z 为夹爪上方，各分量保留输入符号，不再根据 Base Y 自动变号。
B 根据未叠加偏置的目标基座高度 $Z$ 划分层级（配置于 `b_height`: `min_m: -0.01`, `split_m: 0.12`, `max_m: 0.30`）：
$[-0.01, 0.12)$ m 为 Down 层，$[0.12, 0.30]$ m 为 Up 层；越界目标直接跳过，不发运动及 Store 指令。
偏置计算使用补偿前目标方向 yaw。A/C 使用最终抓取请求 pitch，B 两层固定使用 0° 构建偏置旋转矩阵：

```text
yaw_offset = atan2(base_y, base_x)
pitch_request = motion.pitch_b_up_deg / motion.pitch_b_down_deg（B 按层确定）或 motion.pitch_deg（A/C），null 时使用观测 pitch
pitch_offset = 0°（B）或 pitch_request（A/C）
R_offset = Rz(yaw_offset) * Ry(-pitch_offset)
P_request_mm = P_B_mm + R_offset * (1000 * offset_local_m)
```

A/C 沿用历史数值但物理含义调整为末端局部坐标（局部左移 3 cm、局部下移 1.5 cm），非零局部 Y 在偏置后会导致最终 yaw 发生微调；
B 采用水平参考下的深入与下移，不随抓取 pitch 旋转。因此对同一视觉点和实测姿态，修改 B 抓取 pitch 不改变补偿后的 XYZ。
正式任务使用 `MissionContext._current_area`，单点调试客户端使用 `--zone` 参数。
各项配置均须存在并独立验证三个有限数，缺失或非法直接报错，不跨区跨层回退。
这是视觉表面点到抓取点的末端相对补偿，不能用它代替或重复应用 planner 的刚性 `geometry.tool_z_offset_m`。
日志同时保留区域、层（B 为 up/down）、配置键、局部偏置、偏置参考 yaw/pitch、偏置前 Base 坐标和最终请求点。
源码修改后需重新构建 `fruit_mission` 并重启 `fruit_mission_node` 或 `camera_arm_target_client`。
预设重载服务只重载 `arm_presets.yaml` 中的 presets，不刷新客户端参数。

此次修正了 E -> S 的局部 Z 符号。原值 `-66.726 mm` 把螺纹孔放在了抓取中心下方；
末端俯仰约 -91.45°、偏航约 90° 时，局部 Z 几乎沿基座 +Y，符号错误造成约
133.41 mm 的 Y 偏差。对同一帧光学点 `(0.008528585, 0.062558666, 0.314)` m
和实测 TCP 姿态复算，水果 Base Y 从 0.131326 m 修正为 0.264736 m，
与尺量约 0.26 m 接近。这是已采集数据的离线复算，仍需重启检测工具后现场复核。
只修改外参平移，不修改实时 TCP、关节映射或旋转矩阵，也不再额外叠加 30 mm 工具偏移。

## 配置和动作语义

配置：`/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml`。
`fruit_mission.yaml` 和 `script_ros_clients.yaml` 用 `camera_extrinsic_config` 指定源文件。
静态平移均为毫米；ROS 感知和 `MoveArmCartesian` 仍为米，入口乘 1000，基座坐标提交前除 1000。

- `arm_config_path`：读取 planner 的 joint_mapping 限位、状态新鲜度和驻留时间；geometry/FK 在 planner 内处理。修改标定后须同时重启 planner 和 mission，保持一致。
- `camera_frame`：只接受该 RGB 光学 frame 的点；不通过改标签伪造坐标转换。
- `joint_state_topic`：原字段名保留，指向现有 ArmTelemetry 话题；缺失、过期、frame 不是 `arm_base_link`、原始/模型角或 TCP 非有限、关节名称错误或硬件角越限时不提交请求。不得把消息内模型角再减一次偏置。
- `motion.pitch_deg: 0.0`：A/C 抓取的绝对俯仰角（度），0° 表示末端 a4 段水平。
- `motion.pitch_b_up_deg: 0.0`：B 上层抓取的绝对俯仰角（度）。
- `motion.pitch_b_down_deg: -30.0`：B 下层抓取的绝对俯仰角（度）。
- `b_height`：B 区分层边界（米），`min_m: -0.01`、`split_m: 0.12`、`max_m: 0.30`。
- 各项 pitch 均要求有限数或 null；null 使用实测观察 pitch。只影响视觉目标抓取，不修改 Look、Store、Home 或其他 preset。
- `motion.duration_ms: 1500`：抓取探出耗时 1500ms。
- `motion.linear_path: false`：关节空间 PTP 平滑规划（false 为 PTP，true 为笛卡尔 LIN）。
- 视觉目标末点使用 CLOSE，保留转换出的 XYZ，不沿用旧固件的 Z=-100mm、pitch=-90°，不编造接近或提拉坐标。

抓取流程为目标 CLOSE → 驻留 → 对应区域/侧 Store（包含 OPEN、驻留）→ Home。
`MoveArmCartesian` 没有驻留字段，因此 CLOSE 完成后复用 `move_joints`：以新鲜、有限、在限位内的实测原始四轴角为目标，HOLD 保持 `planning.default_dwell_ms`，仍由 planner 生成并执行轨迹，不在 mission 中生成轨迹或用 sleep 假冒执行完成。该驻留值须为正数。
Store/Home 都必须返回成功才记录本次工序完成；这不是视觉证实果实实际入筐。

planner 的 FK/IK 现在包含 `geometry.tool_z_offset_m` 固定结构偏移（当前 `-0.03 m`，末端局部 Z 方向，随腕部旋转）。FK 原点就是此处外参测量定义的真实夹爪中心 E，不再使用旧的 12 cm 伸出段端点作为 E。`pitch=0°` 仍指 a4 段水平，不是腕轴到抓取点的斜线水平。旧 `planning.cartesian_offset_m` 输入平移已撤掉，mission 提交的基座 XYZ 即 TCP 目标，不在相机换算中再补偿 3 cm。已测量的 E/S/C 外参数值保持不变。

只修改 `camera_extrinsic.yaml` 的相机外参时，重启使用它的 mission/相机调试客户端及
`test_camera_detector.py`，这些客户端只在启动时加载外参，无需因此重启 planner 或机械臂控制台。
planner geometry 修改仍须同时重启 planner、相关客户端及控制台，不会自动热重载；
geometry 必须通过共享解算器的尺寸/有限数校验，局部 Z 偏移允许为负。

机械臂服务执行和异步 Look 均不设前端期限或计时器，直接等待 planner 返回下位机结果。
收到完成才成功；硬件报错、主动 STOP/取消、请求异常或节点退出返回失败，未知结果仍请求
STOP。`service_timeout_sec` 和 planner 的 `execution_timeout_buffer_sec` 已删除。
端点发现、抓取驻留所需的新鲜反馈、识别和扫码仍各用独立的有限等待；端点最多 10 秒。
STOP 服务成功仅表示已发布停止命令，不证明硬件停稳；恢复不设置独立停稳等待或固定
sleep，处理 STOP 派发应答后只尝试一次 Home。预设入口直接检查导航终态及当前新鲜、
无错误、有限、在限位内的四轴低速反馈，不满足即跳过；planner 仍拒绝忙状态及 STOP 后
未恢复 IDLE 的请求。不自动清除 EMERGENCY，也不把 future 完成当作运动成功。

## 观察与 A/B/C 编排

- 导航与相应 Look 并行；导航成功、Look 成功后开窗。观察期间至目标冻结/提交之前，底盘和机械臂静止由现场物理保证，不增加开窗前或窗口内的独立静止等待。运动入口仍检查原始 ArmStatus 的新鲜、有限、限位内低速反馈；关节阈值取 mission `stationary.joint_speed_rps`，不把非有限速度伪造为零。
- A：导航与 `left_look_a` 并行 → OPEN/HOLD → 左侧识别/抓取/`left_store_a`/Home → `right_look_a`/OPEN/HOLD → 右侧对应工序。
- B：每树只观察一次，按既有像素评分选择最多两个匹配且成熟的目标。在首次观察姿态下，一次性转换并保存两个基座坐标/绝对 pitch。第二抓直接用已保存请求，不重新 Look、识别或换算；第一抓失败也推进下一冻结目标，实际运动仍受反馈与 planner 保护。
- C：二维码任务顺序、路线与左右侧使用现有规划结果，导航时提前 Look，不增加到点方向复核。导航失败跳过当前项，不在未到达的位置抓取。
- 局部失败跳过当前项，A 左右侧和 B 冻结目标逐项继续，不跨层传播 `safe_to_continue`。失败只做一次必要 Home；状态未知仍取消导航并请求 STOP，未获有效实测反馈时跳过危险动作。除既有导航重试外，没有识别/抓取/姿态重试。底盘已接受 goal 后等待终态，不再以 10 秒执行期限中断；goal 应答与取消仍独立限时。
- 二维码导航、姿态或识别失败均使用 `config/qr_fallback.yaml`，仍推进软件流程；物理动作受反馈及保护姿态门控。扫码直接执行 `qr_scan`，QR 解析只按当前窗口 B/C 区域，不维护扫码顺序状态机。
- 旧正常 retract 边界使用 `shenzhi`：A 每点、B/C 扫码、B 每树、C 每项结束及区域出口，重合出口不重复调用。失败仍一次 Home，不追加保护动作；必要保护位未确认时不放行转场。新鲜静止的已确认 shenzhi 可以用于导航交接及启动并行 Look。

## 离线算例（不控制硬件）

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 src/fruit_mission/fruit_mission/camera_target_transform.py 0 0 500
```

当前外参输出：

```text
camera_mm      = [0, 0, 500]
end_effector_mm = [434.728894, 31.255715, 80.525996]
```

光学原点 `(0,0,0)` 的几何转换结果应等于 t_E_C；独立数学函数允许该点，实际控制入口拒绝非正深度。
端到端控制前还须在安全现场用已知位置点验证毫米/米、左右方向、俯仰方向和夹爪原点。离线数值/模拟服务检查不能替代实机标定。

最小纯数学无运动校验脚本（核验相机、TCP与Base坐标几何）：

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
                   (434.728894, 31.255715, 80.525996), atol=1e-6)
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

C++ 独立头文件为 `include/fruit_mission/camera_target_transform.hpp`，函数
`fruit_mission::transformCameraToEndEffectorMm(camera_mm, rotation, translation_mm)`；
只使用标准库，R/t 显式传入，避免复制一套硬编码标定。当前 mission 为 Python，运行链只用 Python 实现。

纯数学离线单元测试（验证相对末端偏置与姿态旋转计算，不驱动硬件）：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 -m unittest discover -s src/fruit_mission/test -p test_camera_target_transform.py -v
```

## 构建与剩余集成限制

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_mission
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

需要先构建/安装 `fruit_arm_interfaces`、`fruit_arm_planner`。`start_fruit_mission.sh`
现在启动 planner-only launch，不额外启动串口桥；已有 planner 时拒绝重复启动。
启动检查覆盖 move_cartesian、move_joints、execute_preset、STOP 服务及轨迹 Header/Point/Mode 订阅、ArmStatus 发布端点，不再等待旧 task/callback 端点。每项启动等待不超过 10 秒；就绪超时告警，不因外围节点退出直接结束 mission。

正式 mission 统一直接使用 `arm_presets.yaml` 姿态定义，删除中间映射字典：
观察入口 `start_observation` 直接调用 `<side>_look_<zone>`，放果调用 `<side>_store_<zone>`（left/right，a/b/c），复归 `home`，扫码 `qr_scan`，正常转场保护 `shenzhi`。任务配置不含 `arm_presets` 映射键。这里姿态名称与 planner YAML 统一；各姿态的真实关节轨迹均在
planner 的 `arm_presets.yaml` 中定义与维护。不补假姿态。缺失、越限或硬件执行失败均返回失败。
真实姿态及已定义名称以源码 YAML 为准；操作者须检查并补齐所用预设。客户端不覆盖已有坐标，不补空轨迹或回退到 A 区姿态。区域话题供感知模型选择；QR 区域由窗口指定，不发送下位机区域命令。

独立 `camera_arm_target_client` 与 `test_camera_arm_target.sh` 不依赖导航，只接受 `--zone A|B|C --left|--right`（可加 `--debug`），旧 `--model` 删除。区域同时决定模型（A/C 蔬菜、B 水果）和 Look/Store 后缀（a/b/c）。每次先 Home，再区域 Look、实测静止 OPEN/HOLD、harvest 窗口、基座解算、CLOSE/HOLD、区域 Store，不加入跨区 shenzhi。任何工序失败不报整个流程成功。该脚本会驱动硬件，不是自动测试。

- 模型选择使用可靠、持久区域发布端，先等待 `fruit_perception_node` 的实际区域订阅端，再发布一次并等待 DDS acknowledgment；仅扫码节点订阅或串口桥就绪不能视为感知区域端就绪。DDS 确认是传输层交付，不是推理就绪回执。
- 不再支持品种过滤参数。B 等待 `targets_ready`，只从有限置信度、有效正深度的 `*_ripe` 目标中选最高置信度者；无成熟目标时等待超时，不回退到无类型信息的 `target_point` 或未成熟目标。
- Look 完成后直接进入共享 OPEN/HOLD：每次从新鲜、有限、限位内、低速的四轴反馈冻结硬件角并调用 `move_joints`（驻留时长取 planner `default_dwell_ms`）。不重复做独立 5 秒静止等待。张爪失败不开始识别/目标运动；张爪在采图之前，避免采图后改变手眼姿态。未修改 Look 预设或标定。
- 机械臂运动无限等待执行结果；端点、识别、区域交付等有限等待仍要求有限正数，大于 10 秒截断到 10 秒，较短值保留。请求异常仍请求 STOP，不将发布等同于硬件完成或停稳。

### 独立检测工具与实时 TCP (test_camera_detector.py)

独立查看工具 `test_camera_detector.py` 同样全面接入真实 TCP 遥测：
- 启动：`./test_camera_detector.py --model A|B`。静态观察预设选择 `--preset` 与快捷键 `p` 已删除，基座映射姿态完全由实时遥测决定。
- 依赖管理与通信所有权：工具在启动时检测 ROS 图中是否存在 `serial_controller`（或状态发布端）及 `arm_planner_node`（或遥测发布端）。已有则复用，缺少才按需调用对应 launch。**工具会按需打开串口通信以取得实时 TCP，但工具本身绝不自动调用任何运动、复位或张爪服务**。退出时仅清理工具自身启动的后台进程，不关闭或发送 STOP 给已存在的外部节点。若在启动后的遥测等待期间发生中断（如 Ctrl-C）或异常，函数内部具备局部清理保护，立即终止已启动的自有进程（先停 Planner 再停串口桥），防止孤儿进程残留；成功返回后由主流程统一负责自有进程清理。
- 端点冲突拒绝与 N/A 规则：工具通过类型化发布端检查（`get_publishers_info_by_topic`）校验遥测与状态发布端。若存在重复发布者（>1）或发布端消息类型不匹配（非 `ArmTelemetry` 或非 `ArmStatus`），则明确告警并拒绝按“无反馈”启动通信，切断位姿更新并将 Base 坐标置为 N/A。若未收到有效 TCP 或遥测过期（>0.25s），顶部横幅显示 `TCP: N/A`，目标框与鼠标探针处 Base 坐标均显示为 `Base: N/A`，2D 检测与深度测量照常工作；若 `CameraInfo` 内参缺失或无效，不显示 3D Cam 与 Base 坐标；若外参加载失败，不显示 Base 坐标。
- 静止观测约束：换算仍依赖静止观测前置条件，不宣称运动中动态定位。

退出路径请求 STOP/取消导航；杀进程不代表停稳。预设尚未补齐、串口顺序未经验证，也不能声称比赛整机可运行。
原串口桥的跨报文轨迹装载顺序也仍需端到端核验。

上一轮转换模块的离线检查：fruit_mission 构建、Python 语法/新增模块及单相机客户端 flake8、C++14 头文件编译及与 Python 的毫米算例一致性均通过。
历史验证曾在隔离 ROS 域 137、专用 `/offline_camera_check/*` 服务名下模拟四轴反馈和 planner 服务，核对基座坐标、俯仰角、失败应答和无效输入拒绝；其中旧服务超时 STOP 场景不再代表当前策略。
未启动真实机械臂服务或比赛任务，没有发送实机运动命令；未新增测试文件。
