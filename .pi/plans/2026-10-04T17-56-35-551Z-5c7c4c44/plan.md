## 目标

让机械臂运动结果由下位机主动回传的 `ArmStatus` 决定：严格关联本次 `trajectory_id` 的 `COMPLETED` 才算成功，`FAILED`/`EMERGENCY` 立即报告失败并记录状态信息；不再用轨迹时长或 mission 的 10 秒运动 future 超时判断运动失败。

单项运动失败时放弃失败轨迹，按“`STOP` → `zero` → `home`”执行一次安全恢复；恢复成功后推进 A/B/C 主流程的下一项。恢复未确认时可以推进软件任务/导航，但不得盲发下一条机械臂运动。真实 `EMERGENCY` 保持急停锁定，不自动清锁或强行发送 `zero/home`。

## 范围

- 修改 `src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py`：核对并固化 `ArmPlannerNode.execute_trajectory_sync()` 的状态驱动等待、当前任务 ID 严格匹配和失败返回；删除仍残留的运动超时参数加载/校验。保留新鲜状态、实测四轴起点、轨迹占用和 STOP/ESTOP 安全控制。
- 修改 `src/fruit_arm_planner/config/arm_presets.yaml`：删除仅用于轨迹完成超时判断的 `planning.execution_timeout_buffer_sec`；保留现有 `zero`、`home` 预设及其操作者配置，不新增假姿态。
- 修改 `src/fruit_mission/fruit_mission/camera_target_planner_client.py`：让 Look、QR、目标抓取、Harvest、Home 共用的 service future 等待由 planner 的状态结果驱动；删除 `_motion_deadline`、`_check_motion_timeout()` 和 `service_timeout_sec` 运动结果判断。保留 `endpoint_timeout_sec` 的服务发现等待、状态新鲜度、异常处理和显式 STOP。失败响应日志保留 `trajectory_id`、planner `message` 以及当前 ArmStatus 的 `error_code` 信息。
- 修改 `src/fruit_mission/fruit_mission/mission_context.py`：在 `recover_home()` 中实现一次 `STOP` 后等待 STOP 之后的新鲜、有限、低速、可恢复状态，再按 `zero`、`home` 顺序调用并确认完成；EMERGENCY 分支继续锁定并禁止恢复运动；恢复失败只阻止后续不安全 arm 命令，不结束主流程。保持一次恢复、不重试原失败轨迹。
- 修改 `src/fruit_mission/fruit_mission/actions.py`、必要时 `Logic.py`：沿用现有局部失败后 `recover_home()` 和任务游标推进路径，确保 FAILED、planner 服务失败和 zero/home 恢复失败都记录原因并继续 A/B/C；不把失败伪装成成功，不添加运动重试或方向重查。
- 修改 `src/fruit_mission/config/camera_extrinsic.yaml`：删除 `service_timeout_sec`，保留端点等待配置及其他坐标/运动参数。
- 同步文档：
  - `src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md`：说明状态结果、严格 `trajectory_id`/新状态匹配、FAILED/EMERGENCY 处理和“无轨迹时长超时”；说明 STOP 后由 mission 按 `zero → home` 恢复。
  - `src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md`：更新 service 执行结果和 mission 调用等待约定，删除运动 future 超时判断表述。
  - `src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md`：删除 `execution_timeout_buffer_sec` 的配置说明，补充状态驱动等待与恢复边界。
  - `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`：删除 `service_timeout_sec` 和运动超时即 STOP 的旧契约，记录状态驱动等待、FAILED 后 STOP、`zero → home` 恢复、EMERGENCY 锁定及恢复失败后的 arm 门禁。
  - 如实现检查发现 `src/fruit_mission/README.md` 或其他受影响说明仍描述旧运动超时，按同一契约做最小同步。
- 不修改 service/msg 接口定义，不增加测试依赖，不新增 D 区、坏果处理、换筐、返航或第二套 arm controller。
- 执行阶段使用独立 worktree 和任务分支。当前主工作区已有大量用户/其他任务改动，必须保留；不得在主工作区直接覆盖、批量暂存或代提交这些改动。

## 决策

- 运动完成/失败的唯一业务来源是新收到且 `trajectory_id` 严格匹配本次请求的 `ArmStatus`；不接受任意任务 ID 的状态，避免旧任务状态误判当前请求。
- `STATUS_COMPLETED` 返回成功；匹配的 `STATUS_FAILED` 或 `STATUS_EMERGENCY` 返回失败，并记录 `trajectory_id`、`message`、`error_code`。`RUNNING`、旧状态和普通 `IDLE` 不得宣告当前运动完成。
- 删除轨迹时长 `execution_timeout_buffer_sec` 和 mission 的运动 `service_timeout_sec`。运动 service 可以无限等待下位机终态；状态断流不自动转换为运动失败，也不允许因此盲发下一条 arm 运动。端点发现等待、状态新鲜度和 STOP 后停稳确认属于独立安全/通信门槛。
- FAILED 后不恢复失败轨迹、不重试原动作。由 mission 恢复路径发送一次 STOP，等待 STOP 之后的新鲜有效低速可恢复反馈，再执行 `zero`，确认完成后执行 `home`，两步都成功才把恢复视为完成。
- EMERGENCY 表示硬件急停锁定：不自动清锁、不发送 `zero/home`；主流程可以继续其软件推进，但后续 arm 动作必须被门禁跳过，直到外部解除并重新获得有效状态。
- zero/home 恢复失败或停稳未确认时，任务游标仍可推进、导航仍可按现有流程继续；任何需要机械臂的下一动作必须先通过新鲜有效反馈和安全状态检查，不能使用历史/零值关节或盲发运动。
- 正常成功动作后的既有单个 `home` 归位保持不变；`zero → home` 只用于局部失败后的安全恢复。
- 失败信息记录并推进任务，不伪装为成功；现有一次安全恢复和 A/B/C 局部失败继续原则优先于等待某个失败目标重试。
- 执行阶段必须在 worktree 内构建/检查；合并前检查主分支干净且无并发写入，做 base/main/feature 三方冲突分析，只允许明确解决后的安全 fast-forward。禁止 `reset --hard`、`checkout --ours/--theirs`、强制合并、批量 `git add` 或代提交既有改动。

## 执行步骤

1. **建立隔离 worktree 并记录基线。**
   - 文件/符号：无业务文件改动；工作区路径为 `/home/rc4/fruit_ws`，目标 worktree 放在被 Git 忽略的 `.worktrees/` 下，分支使用任务专用名称。
   - 内容：记录当前主分支 HEAD、启动前 baseline 和主工作区脏状态；确认 worktree 不会发现/编译其他 `.worktrees` 包。
   - 依赖：后续所有编辑、构建和检查必须在该 worktree 中进行；先确认 `.worktrees/<branch>` 被 Git 忽略。
   - 检查命令：`git status --short`、`git rev-parse HEAD`、`git check-ignore .worktrees/<branch>`；期望主工作区脏改动保持原样，worktree 从已记录基线创建且干净。
   - 失败停止条件：无法创建隔离 worktree、无法确定基线、主工作区状态出现新的非本任务写入或 worktree 未被忽略时停止并报告。

2. **核对 planner 状态链路并完成最小修改。**
   - 文件/符号：`src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py` 的 `_on_arm_status()`、`_load_config_from_yaml()`、`execute_trajectory_sync()`、`stop()`/`emergency_stop()`。
   - 内容：保留状态接收时间和严格 `status.trajectory_id == traj_id`、`received_at > dispatched_at` 条件；确认只有 COMPLETED 成功、FAILED/EMERGENCY 失败；移除未使用的 `execution_timeout_buffer_sec` 读取和校验；不移除状态新鲜反馈、实测起点、STOP/ESTOP 或 motion ownership。
   - 依赖：先完成步骤 1；必须以当前 worktree 实际内容为准，不复制主工作区整文件。
   - 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_arm_planner/fruit_arm_planner`；`ament_flake8 src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py`；期望语法和目标文件 lint 通过，且不再存在 planner 对轨迹时长的运动完成判断。
   - 失败停止条件：状态 ID 匹配、STOP/ESTOP、安全起点或失败返回语义无法在当前代码中保持时停止，不用删除安全检查来通过检查。

3. **移除 planner 配置残留并同步接口/通信说明。**
   - 文件：`src/fruit_arm_planner/config/arm_presets.yaml`、`src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md`、`src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md`、`src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md`。
   - 内容：删除 `planning.execution_timeout_buffer_sec` 及旧的“轨迹时长+余量超时”描述；说明运动结果由匹配 ArmStatus 终态决定，状态断流不会伪造完成/失败；补充 FAILED 后由 mission 执行 STOP→zero→home，EMERGENCY 不自动恢复。
   - 依赖：步骤 2 确认 planner 的实际状态契约后执行；文档只描述现有字段，不发明下位机未定义错误码。
   - 检查命令：`git diff --check -- src/fruit_arm_planner/config/arm_presets.yaml src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md src/fruit_arm_planner/docs/04_SETUP_AND_DEBUGGING.md`；期望无空白错误且文档与代码一致。
   - 失败停止条件：发现文档仍要求超时宣告运动失败，或配置删除导致 planner 启动配置校验失败时停止并修正，不保留相互矛盾的契约。

4. **改造共享 mission arm client 的等待与错误记录。**
   - 文件/符号：`src/fruit_mission/fruit_mission/camera_target_planner_client.py` 的初始化配置校验、`_start_motion()`、`_check_motion_timeout()`、`finish_motion()`、`request_stop()`、`_on_status()`；`src/fruit_mission/config/camera_extrinsic.yaml`。
   - 内容：移除 `_motion_deadline`、运动 deadline timer 及 `service_timeout_sec` 读取；`finish_motion()` 无限等待 planner service future 终态，响应失败时记录/返回失败原因，不把 future 10 秒未完成当作运动失败；保留 endpoint service discovery 的独立上限、异常处理、显式 STOP 和 EMERGENCY 安全门禁；让 Look/QR/抓取/Harvest/Home 的共用路径一致。
   - 依赖：步骤 2 的 planner service 必须先确定为状态驱动返回；步骤 3 的配置字段同步完成。
   - 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission`；`ament_flake8 src/fruit_mission/fruit_mission/camera_target_planner_client.py`；对配置和文档执行 `git diff --check`；期望 mission Python 可编译、目标 lint 通过，代码不再创建/使用运动 deadline。
   - 失败停止条件：共享客户端仍有任何运动 future deadline、超时后错误调用 STOP，或异常路径可能把未知运动当作已完成时停止。

5. **实现失败恢复 `STOP → zero → home` 与 arm 安全门禁。**
   - 文件/符号：`src/fruit_mission/fruit_mission/mission_context.py` 的 `recover_home()`、`execute_arm_preset()`、`start_observation()` 及相关状态辅助；`src/fruit_mission/fruit_mission/actions.py` 的 `observe_and_pick()`、`_grasp_and_collect()`、`pick_b_tree()`、`pick_c()`；必要时 `Logic.py` 仅做最小推进修正。
   - 内容：失败后只执行一次 STOP；用 STOP 之后新收到的有效四轴反馈、低速和可恢复状态确认安全，再按配置/现有 preset 名称执行 `zero` 后 `home`，每一步等待实际 planner 完成；EMERGENCY 直接保持锁定；zero/home 失败记录原因并推进软件流程，但将 arm 后续动作挡住直到安全状态重新确认；所有局部 arm 失败不终止 A/B/C，不重试原轨迹。
   - 依赖：步骤 4 的共享 client 已能无限等待 planner 的真实结果；必须复用 planner-only service API，不直接发布原始轨迹/串口。
   - 检查命令：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission`；`ament_flake8 src/fruit_mission/fruit_mission/mission_context.py src/fruit_mission/fruit_mission/actions.py`；期望语法/lint 通过，任务分支仍推进且无重复抓取/姿态重试。
   - 失败停止条件：出现直接盲发下一条 arm 运动、EMERGENCY 自动清锁、失败轨迹重试、恢复循环或把失败记录成成功时停止。

6. **同步 mission 文档并审查影响面。**
   - 文件：`src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`，必要时 `src/fruit_mission/README.md`；同时复查 `src/fruit_mission/config/fruit_mission.yaml`、启动入口和客户端调用方。
   - 内容：删除 `service_timeout_sec` 运动超时契约；记录下位机状态终态、错误码/trajectory_id 记录、失败后的 STOP→zero→home、EMERGENCY 锁定、恢复失败后继续软件流程但禁止盲发 arm。
   - 依赖：步骤 5 行为确定后执行；只做本任务相关的最小文档改动。
   - 检查命令：`git diff --check -- src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md src/fruit_mission/README.md src/fruit_mission/config/camera_extrinsic.yaml`；期望文档、配置和调用代码无旧超时语义冲突。
   - 失败停止条件：任何受影响文档继续指导用 10 秒判断运动完成，或配置仍要求已删除字段时停止。

7. **执行 worktree 内构建与只读 ROS 检查。**
   - 文件/范围：`fruit_arm_planner`、`fruit_mission` 及其接口依赖；不运行 `src/fruit_arm_planner/test/cartesian_console.py`，不调用运动服务。
   - 内容：在 `/home/rc4/fruit_ws` 外的任务 worktree 中按项目 runtime setup 构建；由于 workspace 有 ROS 包依赖，使用项目规定的包级构建和目标 Python 检查；只读核对接口/话题存在性，不发送运动命令。
   - 依赖：步骤 2–6 完成且 diff 已审查。
   - 测试命令：
     - `cd <task-worktree> && source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash`
     - `colcon build --base-paths src --packages-up-to fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`
     - `source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash`
     - `/usr/bin/python3 -m compileall -q src/fruit_arm_planner/fruit_arm_planner src/fruit_mission/fruit_mission`
     - `ros2 interface show fruit_hardware_interfaces/msg/ArmStatus`
     - `ros2 interface show fruit_arm_interfaces/srv/ExecuteArmPreset`
     - 只读：`ros2 topic list`、`ros2 service list`（不调用 service、不发布 topic）。
   - 期望结果：目标包构建成功，Python 编译成功，接口显示包含状态字段和 preset service；只读 ROS 命令不产生运动。
   - 失败停止条件：runtime setup 验证失败、构建失败、接口类型不一致、发现旧进程/旧消息类型冲突，或检查命令会发送运动时停止并报告具体原因。

8. **提交任务分支并进行合并前三方冲突审查。**
   - 文件/范围：仅暂存步骤 2–6 实际修改的明确文件/hunks；不包含主工作区已有改动、生成目录、`.worktrees/` 或无关未跟踪文件。
   - 内容：完成验证后创建任务里程碑提交；记录任务分支 tip SHA、测试结果和基线 SHA。合并前重新检查 `main` 工作树、并发写入和最新 HEAD；对每个重叠文件比较共同基线、main、任务分支三方内容，按行为和接口逐项解决，禁止整文件覆盖。
   - 依赖：步骤 7 全部通过；主分支必须干净且无并发 writer 才能考虑 fast-forward。
   - 检查命令：`git status --short`、`git diff --check`、`git diff <baseline>...HEAD`、`git diff --stat`；在主分支集成前执行 `git merge-base <main> <task-branch>` 和只读冲突预检。主分支干净时才执行明确的 `git merge --ff-only <task-branch>`；否则保持 Pending Integration。
   - 期望结果：任务提交只含本需求改动；冲突均有三方依据；main 未被强制覆盖，若脏或有并发写入则不合并。
   - 失败停止条件：主分支有任何未提交/未跟踪交付物、主分支在验证期间移动、存在无法判断的行为冲突、或需要强制合并时停止并交接，不声称已合并。

## 验收

- planner 不再读取/校验 `execution_timeout_buffer_sec`，mission 不再使用 `service_timeout_sec` 作为运动完成 deadline；代码、YAML、文档三者一致。
- 每个 motion service 只在严格匹配当前 `trajectory_id` 的新 `ArmStatus` 上返回：COMPLETED 成功，FAILED/EMERGENCY 失败；旧 ID、RUNNING、普通 IDLE 不会误报完成。
- FAILED 路径不重放原轨迹，能记录 trajectory_id、planner message、error_code，按一次 STOP→zero→home 恢复；正常成功路径的单个 home 不被改成 zero→home。
- EMERGENCY 路径不自动清锁、不发 zero/home；恢复未确认时主流程可推进，但不会盲发后续 arm 运动。
- A/B/C 局部运动失败不会结束全流程，不增加非导航运动重试，不改变既有确定性路线/side 逻辑。
- 目标包构建、compileall、目标 flake8、`git diff --check` 和只读 ROS interface/topic/service 检查通过；不执行交互式硬件控制和运动服务。
- 合并前完成主分支清洁检查与 base/main/feature 三方冲突审查；若主分支脏或有并发 writer，交付状态为 **Pending Integration**，保留任务 worktree、分支和提交 SHA。

## 风险

- 下位机可能在 FAILED 后不转 IDLE；恢复路径会按 STOP 后新状态和低速门槛判断，若仍无法确认安全则不发 zero/home，继续软件流程但跳过后续 arm 动作，避免未知状态下运动。
- 取消运动超时后，若下位机状态永久断流，planner service 可能无限等待；这是“状态决定结果”的直接代价。不得把状态断流伪装成完成或失败，也不得因等待未知而盲发下一动作；只能由显式 STOP/外部终止或后续安全门禁处理。
- `ArmStatus` 的实际终态保持时间、FAILED 后状态转换、错误码扩展语义和 STOP 后反馈必须由实机/下位机负责人最终核对；当前计划只使用接口已定义的状态和错误码，不解释未定义值。
- `zero` 和 `home` 是现有配置预设，实际姿态/可达性仍由操作者 YAML 和硬件验收负责；计划不替换或编造关节角。
- 当前工作区有大量既有脏改动，可能与本任务文件重叠；若三方行为意图无法区分，停止合并并向用户报告冲突文件和证据，不采用 ours/theirs 或 reset 覆盖。
- 无限等待会使服务调用线程长期占用；planner 已有单运动 ownership，mission 继续沿用现有 executor/回调模型，禁止新增嵌套 spin 或第二控制器。
