# mission 相机目标换算与 planner 对接

## 当前路径与边界

```text
FruitDecision（RGB 光学坐标，米，保留图像 header）
  -> mission 选中目标
  -> CameraTargetPlannerClient
       camera -> 夹爪中心 E（mm）
       实测硬件关节角 -> JointTransformer -> RobotKinematics FK
       E -> 机械臂固定基座 B（mm）
       B 坐标 / 1000 -> MoveArmCartesian（米）
  -> /arm_planner_node/move_cartesian
  -> 等待 response.success / message / trajectory_id
```

正式 `MissionContext` 和单相机调试客户端共用同一入口，不再发送旧 `ArmTarget`。
正式 mission 的左右观察、二维码、收臂、恢复动作也通过该入口调用
`/arm_planner_node/execute_preset`，与目标运动共用结果校验及未知结果时的 STOP 处理。
`fruit_arm_planner` 源码和配置不因本任务而修改；FK 和硬件角映射直接复用其现有实现，不重复实现 IK 或轨迹规划。

用户确认 DETECT 时机械臂静止。**整个多帧观测窗口及观测结束到提交请求之间必须维持同一观测姿态**，才可用提交前的新鲜实测关节角做 FK；不缓存/插值历史姿态。若以后改为运动中检测，需要重新设计同步，不能继续沿用此假设。

`joint_states` 来自下位机反馈，经 planner 发布，单位 rad；`joint_3` 是小臂绝对角，不是相对肘角。先通过 `hardware_to_algo` 消除 direction/offset，再做 FK。FK 输出原点为夹爪作用中心，俯仰为 `q2+q3+q4`，偏航为 `q1`。

## 新外参评估（单位 mm）

三个原点：E = 夹爪抓取中心，S = D435i 底部螺纹孔表面圆心，C = RGB 光学原点。
E 和 S 的轴同向：局部 X 前、Y 左、Z 上；这些轴随末端运动，并非永远与基座/地面平行。
C 的轴为 X 右、Y 下、Z 前。

用户确认的测量是 **S 指向 E**：

```text
S -> E = [75.413, 0, 66.726]
E -> S = [-75.413, 0, -66.726]
S -> C = [10.144894, 32.458360, 12.615321]
E -> C = [-65.268106, 32.458360, -54.110679]
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

t_E_C_mm = [-65.268106, 32.458360, -54.110679]
```

旋转矩阵通过正交性和 determinant=+1 的舍入容差校验。`T_E_C` 的含义是**把 C 系点转换到 E 系**，不要因命名中的 E/C 次序而误求逆。

第二步（与现有 FK 的向上俯仰为正约定一致）：

```text
R_B_E = Rz(yaw) * Ry(-pitch)
P_B_mm = R_B_E * P_E_mm + 1000 * FK_position_m
```

所以相机到夹爪的固定矩阵本身并不足以提交 planner；必须加上观测姿态的第二步。

## 配置和动作语义

配置：`/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml`。
`fruit_mission.yaml` 和 `script_ros_clients.yaml` 用 `camera_extrinsic_config` 指定源文件。
静态平移均为毫米；ROS 感知和 `MoveArmCartesian` 仍为米，入口乘 1000，基座坐标提交前除 1000。

- `arm_config_path`：读取 planner 的 geometry、joint_mapping 和状态新鲜度；修改标定后须同时重启 planner 和 mission，保持一致。
- `camera_frame`：只接受该 RGB 光学 frame 的点；不通过改标签伪造坐标转换。
- `joint_state_topic`：四轴实测反馈。缺失、过期、非有限数、关节名称错误或越限时不提交请求。
- `motion.pitch_deg: null`：默认保持观测姿态的绝对末端俯仰；数值则指定目标俯仰角（度）。
- `motion.duration_ms: 0`：使用 planner 的默认耗时。
- `motion.linear_path: false`：PTP；true 为 LIN，含义同 planner API。
- 夹爪固定 HOLD，保留转换出的 XYZ，不沿用旧固件的 Z=-100mm、pitch=-90°，不额外接近、夹取、提拉或收臂。

当前功能是**移动到目标点**，不是完整水果抓取工序。任务结果记录 `target_reached`；到位不代表夹取成功。

planner 仍会叠加自身 `planning.cartesian_offset_m`。本次核对时其值是 `[0, 0, 0.03] m`，所以实际 IK 目标比 mission 发出的坐标高 30 mm。这里不自动抵消、不修改该配置；如希望严格到几何目标，现场应确认该偏置是否需要。外参 YAML 修改需重启 mission 客户端，不会自动热重载。

`service_timeout_sec` 是客户端等待上限，应覆盖轨迹发送、执行及 planner 自身超时余量。超时或请求异常会尝试调用 `/arm_planner_node/stop`，仍返回失败；STOP 服务成功仅表示已发布停止命令，不证明硬件停稳。不得因调用成功、future 完成或发布成功而自行推断到位；必须检查 `response.success`。

## 离线算例（不控制硬件）

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
python3 src/fruit_mission/fruit_mission/camera_target_transform.py 0 0 500
```

当前外参输出：

```text
camera_mm      = [0, 0, 500]
end_effector_mm = [434.728894, 31.255715, -52.926004]
```

光学原点 `(0,0,0)` 的几何转换结果应等于 t_E_C；独立数学函数允许该点，实际控制入口拒绝非正深度。
端到端控制前还须在安全现场用已知位置点验证毫米/米、左右方向、俯仰方向和夹爪原点。离线数值/模拟服务检查不能替代实机标定。

C++ 独立头文件为 `include/fruit_mission/camera_target_transform.hpp`，函数
`fruit_mission::transformCameraToEndEffectorMm(camera_mm, rotation, translation_mm)`；
只使用标准库，R/t 显式传入，避免复制一套硬编码标定。当前 mission 为 Python，运行链只用 Python 实现。

## 构建与剩余集成限制

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_mission
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

需要先构建/安装 `fruit_arm_interfaces`、`fruit_arm_planner`。`start_fruit_mission.sh`
现在启动 planner-only launch，不额外启动串口桥；已有 planner 时拒绝重复启动。
启动检查覆盖 move_cartesian、execute_preset、STOP 服务及轨迹 Header/Point/Mode 订阅、ArmStatus 发布端点，不再等待旧 task/callback 端点。

正式 mission 的 `config/fruit_mission.yaml` 使用 `arm_presets` 名称映射：
`observation_left=left_look`、`observation_right=right_look`、`qr=qr_look`、
`retract=retract`、`home=home`。这里仅配置字符串；各姿态的真实关节轨迹仍须在
planner 的 `arm_presets.yaml` 中完成，不补假姿态。缺失、越限、执行失败或超时均返回失败。
区域只通过现有持久 ROS 话题通知感知/扫码节点，不再发送下位机区域命令。
A 区每个作业点左右观察结束后必须确认 retract 完成，才允许导航到下一点；
A 区结束收臂同样不允许超时后继续。观察/目标运动结果不明时停止任务，不叠加恢复运动。
只有单纯感知失败且观察姿态已确认时，A 区才通过 home 预设恢复。

独立 `camera_arm_target_client` / 区域调试工具的旧姿态与回调依赖未在本次迁移，
不应据此声称这些工具已经兼容新协议。预设尚未补齐、串口顺序未经验证，也不能声称比赛整机可运行。
原串口桥的跨报文轨迹装载顺序也仍需端到端核验。

本次离线检查：fruit_mission 构建、Python 语法/新增模块及单相机客户端 flake8、C++14 头文件编译及与 Python 的毫米算例一致性均通过。
另外在隔离 ROS 域 137、专用 `/offline_camera_check/*` 服务名下模拟了四轴反馈和 planner 服务，验证基座坐标与独立矩阵/连杆几何计算一致、俯仰角映射正确、失败应答不报成功、错误 frame/NaN 深度不下发，以及服务超时请求 STOP。
未启动真实机械臂服务或比赛任务，没有发送实机运动命令；未新增测试文件。
