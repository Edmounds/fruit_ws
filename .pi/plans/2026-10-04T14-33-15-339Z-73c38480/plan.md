## 目标

为所有使用 D435i 深度相机目标点的机械臂请求增加一个可现场手动调节的 mission 配置偏置。现有流程先把 RGB 光学坐标点转换为 `arm_base_link` 的 Base 坐标，再应用偏置，最后提交 `MoveArmCartesian`；偏置不改变相机外参、planner 的 `geometry.tool_z_offset_m`、IK、轨迹或下位机协议。

最终数学语义为：

- 配置文件为 `src/fruit_mission/config/fruit_mission.yaml`，键为 `target_offset_m`，单位为米，初始值建议 `[0.0, 0.03, -0.03]`。
- Base X 直接加 `0.0`。
- Base Z 直接加 `-0.03` m。
- 配置中的 Y 值 `0.03` m 表示横向幅值：转换后的目标 `base_y > 0` 时加 `+0.03` m，`base_y < 0` 时加 `-0.03` m；不读取或区分 `left/right` 参数，不增加近零阈值。
- A、B、C 中凡是由深度相机计算目标并通过共享相机目标转换入口交给机械臂的请求都使用它；B 一次观测冻结的每个目标分别应用一次；独立 `camera_arm_target_client` 与正式 mission 使用同一配置和同一规则。

## 范围

包含：

- `src/fruit_mission/config/fruit_mission.yaml`：增加可调偏置配置。
- `src/fruit_mission/fruit_mission/config.py`：增加 mission 偏置读取和有限三元组校验，并接入正式配置校验。
- `src/fruit_mission/fruit_mission/camera_target_transform.py`：增加纯数学的 Base XYZ 偏置应用函数，集中表达 Y 按 Base Y 自动变号的规则。
- `src/fruit_mission/fruit_mission/camera_target_planner_client.py`：在 `transform_end_to_base_mm()` 完成后、写入 `MoveArmCartesian.Request` 前应用一次传入的偏置；保持现有相机外参、实时 TCP、静止门控、planner 服务和失败/STOP 处理。
- `src/fruit_mission/fruit_mission/mission_context.py`：把已加载的 mission 偏置传给共享 `CameraTargetPlannerClient`，覆盖正式 A/B/C 通过共享入口的目标请求。
- `src/fruit_mission/fruit_mission/camera_arm_target_client.py`：读取 `script_ros_clients.yaml` 现有 `config_path` 所指向的 `fruit_mission.yaml`，把同一偏置传给共享客户端；调试程序继续使用 `camera_extrinsic_config` 读取外参。
- 受影响说明：`src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`、`src/fruit_mission/README.md`、`docs/PROJECT_ARCHITECTURE.md`，必要时同步独立调试入口的配置说明。

不包含：

- 不修改 `src/fruit_arm_planner/`、`arm_presets.yaml`、planner 的 TCP 局部结构偏移、IK/轨迹规划或服务接口。
- 不修改相机外参 `camera_extrinsic.yaml`，不新增左右方向重判定、方向重试或新的识别流程。
- 不新增 D 区、收获汇总、坏产物处理、收集交换或返回起点流程。
- 不增加偏置绝对值上限；必要的配置类型、长度、有限数校验仍保留。
- 不新增自动化测试文件，不运行会控制机械臂的交互工具或运动服务。

## 决策

1. 偏置属于 Base 坐标补偿，应用顺序固定为：相机光学点 → 夹爪中心局部坐标 → `arm_base_link` Base 坐标 → 偏置 → `MoveArmCartesian` 请求。
2. Y 不是固定带符号向量，而是配置的正幅值；实际符号由转换后的 Base 目标 Y 决定。这样左侧 Base Y 为正时向左补偿，右侧 Base Y 为负时向右补偿，调用方无需传递 `left/right`。
3. 正式 mission 和独立调试程序都读取同一个 `fruit_mission.yaml` 偏置。现有 `script_ros_clients.yaml` 已有 `config_path` 指向该文件，优先复用这个配置入口，避免复制第二份可调数值。
4. 所有当前共享 `CameraTargetPlannerClient.target_request()` 的目标统一生效。A 的单目标、B 的一次观测后冻结的多个目标、以及调试程序都在同一个入口应用一次；任何 C 目标若使用该入口也自动生效。当前代码没有因此新增 C 的深度识别或采摘流程。
5. 目标不可达、越限、请求失败、超时或未知运动结果继续使用现有 planner 响应校验、STOP、一次安全 Home 和 mission 推进规则。配置本身只要求三元素有限数，不以补偿大小替代 planner 的可达性与安全校验。
6. 配置与代码变更按现有源码配置加载边界处理；不修改 `build/`、`install/`、`log/`、生成接口或其他工作树。执行前保留启动时已有用户改动，并重新核对本任务相关 diff 的边界。

## 执行步骤

1. **复核基线和共享调用边界。**
   - 文件/符号：`src/fruit_mission/fruit_mission/mission_context.py::MissionContext.__init__`、`camera_target_request`，`actions.py` 中 A/B/C 的目标调用，`camera_arm_target_client.py::run_sequence`，`script_ros_clients.yaml::config_path`，以及 `camera_target_planner_client.py::CameraTargetPlannerClient.target_request`。
   - 改动：只确认正式 mission、B 冻结目标和调试程序的实际入口；区分启动前脏改动，不覆盖已有修改。确认 C 当前是否已经调用共享深度目标入口；若实现边界与已确认需求不一致，先停止，不新增 C 流程。
   - 依赖：后续配置和代码改动依赖本步骤确认调用路径。
   - 检查命令：只读 `git status`/`git diff`，并用 `ffgrep` 或等效源码阅读核对上述符号；不启动 ROS 节点、不执行运动。
   - 期望：确认所有需要补偿的现有调用均汇聚到共享客户端，且调试配置已有可复用的 mission 配置路径。
   - 停止条件：发现目标转换存在第二套未调查的实现、C 有独立深度目标通路、配置路径不能读取 mission YAML、或已有改动与本范围重叠而无法安全保留。

2. **增加 mission 配置和统一校验。**
   - 文件/符号：`src/fruit_mission/config/fruit_mission.yaml` 的 `fruit_mission.ros__parameters`；`src/fruit_mission/fruit_mission/config.py` 的 `load_config()`、`validate_config()`，新增最小的偏置读取/校验辅助符号。
   - 改动：加入 `target_offset_m: [0.0, 0.03, -0.03]` 及明确注释，说明单位、Base 坐标、Y 为幅值且按目标 Base Y 取符号。校验值必须为长度 3 的非 bool、有限数值；不增加绝对值上限。正式 mission 的 `load_config()` 和调试读取路径复用同一校验逻辑，错误信息包含配置键和源文件路径。
   - 依赖：步骤 1 完成；步骤 3 的共享数学函数和步骤 4/5 的客户端传参依赖此键语义。
   - 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/config.py`；离线加载 YAML 并验证初始三元组为 `(0.0, 0.03, -0.03)`，不启动 ROS。
   - 期望：合法配置加载成功；缺失、非列表、长度不是 3、bool、NaN/Inf 的偏置明确失败；任意有限幅值不因人为上限失败。
   - 停止条件：校验绕过了正式配置加载、调试读取必须重复一套不一致校验、或现有 QR/航点校验被改动。

3. **在纯坐标数学层集中实现偏置规则。**
   - 文件/符号：`src/fruit_mission/fruit_mission/camera_target_transform.py`，新增一个接收 Base mm 坐标和 mission 偏置 m 的纯函数，例如 `apply_target_offset_mm()`；复用现有 `_vector3()` 的有限数检查。
   - 改动：将偏置转换为 mm 后返回新的 Base 三元组；X/Z 直接相加，Y 使用正幅值乘以 Base Y 的符号。函数不得读取 ROS、相机外参、左右侧参数或 planner 状态，不改变现有 `transform_camera_to_end_effector_mm()` 与 `transform_end_to_base_mm()` 的公式。
   - 依赖：步骤 2 确定偏置格式；步骤 4 通过该函数接入客户端。
   - 检查命令：在 runtime setup 后运行一次临时 heredoc 的纯数学检查，使用同一目标 Base 坐标分别取正/负 Y，确认输出为 `(+0.13)` / `(-0.13)` m 对应的横向结果，Z 均减 `0.03` m，X 不变；同时传入 NaN/错误长度确认抛出 `ValueError`。该检查不启动 ROS、不发送运动。
   - 期望：左右两组输入只因 Base Y 符号不同而得到相反 Y 补偿；不发生单位混用或坐标旋转；所有非法输入被拒绝。
   - 停止条件：出现固定左/右参数依赖、把偏置加在相机坐标或末端局部坐标、Y 变号结果错误、或纯函数无法保持现有有限数校验。

4. **接入正式 mission 与独立调试客户端。**
   - 文件/符号：`src/fruit_mission/fruit_mission/camera_target_planner_client.py::CameraTargetPlannerClient.__init__`、`target_request()`；`src/fruit_mission/fruit_mission/mission_context.py::MissionContext.__init__`；`src/fruit_mission/fruit_mission/camera_arm_target_client.py::CameraArmTargetClient.__init__`/`run_sequence()`。
   - 改动：让共享客户端接收已校验的 mission 偏置；在现有 `transform_end_to_base_mm()` 返回后立即调用步骤 3 的函数，再把修正后的 Base mm 除以 1000 写入 `MoveArmCartesian.Request.x/y/z`。日志同时记录原始 Base 坐标、偏置和最终请求坐标，便于现场调参，但不改变运动结果判断。正式 mission 从已加载参数传入；调试节点声明/读取现有 `config_path`（默认同一 `fruit_mission.yaml`），调用统一读取/校验，再与 `camera_extrinsic_config` 分开传给共享客户端。保留 B 冻结请求的单次计算和现有 A/B/C 失败推进、安全 STOP/Home 逻辑。
   - 依赖：步骤 2、3 完成；步骤 1 已确认没有第二套深度目标转换路径。
   - 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/config.py src/fruit_mission/fruit_mission/camera_target_transform.py src/fruit_mission/fruit_mission/camera_target_planner_client.py src/fruit_mission/fruit_mission/mission_context.py src/fruit_mission/fruit_mission/camera_arm_target_client.py`；`ament_flake8` 只检查上述受影响 Python 文件。
   - 期望：两条客户端在相同相机点、实测 TCP 和姿态下生成相同修正后的 Base 请求；调试程序仍能独立读取外参和 mission 偏置；偏置只应用一次；相机帧校验、深度校验、静止门控、planner 响应校验和 STOP 行为不变。
   - 停止条件：需要改 planner 或接口、正式 mission 与调试结果不一致、偏置被重复应用、日志把发布/回调当成运动成功、或出现任何安全门控回归。

5. **同步文档并完成定向构建/审阅。**
   - 文件：`src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md` 的配置和端到端公式、调试客户端说明；`src/fruit_mission/README.md` 的 mission/调试配置边界；`docs/PROJECT_ARCHITECTURE.md` 的相机目标与 A/B/C 共用行为。仅在现有说明需要声明参数来源时更新 `src/fruit_mission/config/script_ros_clients.yaml`，不复制偏置值。
   - 改动：记录 `target_offset_m` 的米单位、Base 坐标应用顺序、`[0.0, 0.03, -0.03]` 的初始含义、Y 按转换后 Base Y 自动变号、A/B/C 和调试入口共用，以及修改后重启加载配置的要求；明确 planner 的局部 TCP 偏移仍由 planner YAML 管理，避免重复补偿。文档不得宣称实机抓取已验证。
   - 依赖：步骤 4 的最终代码语义。
   - 检查命令：`bash -n test_camera_arm_target.sh`（若脚本未改仍作为入口回归检查）；`git diff --check -- src/fruit_mission/config/fruit_mission.yaml src/fruit_mission/fruit_mission/config.py src/fruit_mission/fruit_mission/camera_target_transform.py src/fruit_mission/fruit_mission/camera_target_planner_client.py src/fruit_mission/fruit_mission/mission_context.py src/fruit_mission/fruit_mission/camera_arm_target_client.py src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md src/fruit_mission/README.md docs/PROJECT_ARCHITECTURE.md`；从干净 runtime 执行 `colcon build --base-paths src --packages-up-to fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`，再按规则重新 source setup。
   - 期望：定向语法、lint、空白检查和 `fruit_mission` 构建通过；构建不修改或提交生成目录；文档公式与实际代码一致；最终 diff 只包含本任务范围及保留的已有用户改动。
   - 停止条件：runtime setup 校验失败、构建/lint/语法失败、配置仍从 install 副本读取、文档与实现不一致、发现 planner/下位机被改动，或最终 diff 混入无关文件。不得用 `PYTHONPATH`、手工修改生成物或硬件运动来绕过失败。

## 验收

- 读取正式 `fruit_mission.yaml` 后能得到有限三元组 `[0.0, 0.03, -0.03]` m；调试客户端通过同一 `config_path` 读取同一值。
- 对相同的相机点、观测 TCP 和姿态，现有换算得到 Base 点 `P_base` 后，最终请求满足：X 不变，Z 为 `P_base.z - 0.03` m，Y 为 `P_base.y + 0.03` m（Base Y 为正）或 `P_base.y - 0.03` m（Base Y 为负）。
- A、B、C 所有实际使用共享深度目标入口的请求以及调试程序均自动应用一次偏置；B 的多个冻结目标互不覆盖、互不重复补偿。
- 修改 YAML 数值后，正式 mission 和调试程序重启即可共同使用新值；不需要修改 planner 或相机外参。
- 非法偏置配置被拒绝；没有新增绝对值上限；目标不可达和运动异常仍由现有 planner/misson 安全流程处理。
- 定向 compileall、ament_flake8、纯数学离线检查、`git diff --check` 和 `fruit_mission` 定向构建均达到步骤 2/3/4/5 的期望结果。验证不启动硬件运动。

## 风险

- 现场若把 `0.03` 填成毫米数 `30`，会产生 30 m 级补偿；文档和 YAML 注释必须反复明确单位为米，调参只改 `fruit_mission.yaml`。
- Base Y 符号依赖观测姿态和实时 TCP/模型偏航的正确性；若外参、姿态反馈或 Base 轴定义错误，自动变号会把偏置加向错误一侧。现场必须先用日志核对原始 Base Y 和最终请求，再进行低速安全验证。
- planner 的 `geometry.tool_z_offset_m` 已经是随腕部姿态旋转的结构 TCP 偏移；本任务的 Base Z 补偿是目标表面到夹爪中心的任务调参，若把两者混为一谈会重复补偿。
- 目标点在 Base Y 近零时没有业务定义；计划不加入近零阈值或左右参数，按用户确认该情况不发生。若实机出现该情况，应停止现场调参并重新确认符号规则，不临时增加策略。
- 偏置可能把原本可达的点推到工作空间或关节限位外；现有 planner 会返回失败，mission 继续既定跳过/恢复流程，但现场仍需逐步验证初始值。
- 当前源码中 C 的路线/任务主要由二维码和既有路线规划驱动；本任务不会凭空增加 C 的深度目标流程。若后续新增 C 深度目标，必须复用共享入口并同步文档，避免产生第二套补偿。
