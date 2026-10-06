## 目标

为 `/home/rc4/fruit_ws/test_camera_arm_target.sh` 启动的相机机械臂调试流程增加抓取后的放果工序。用户已逐轮确认并在 Q5 确认整体理解一致：

- 保留现有 Home → 对应侧 Look → 实测停稳 → OPEN/HOLD → 识别 → 基座坐标换算 → 目标 CLOSE/HOLD。
- 仅在目标抓取及 CLOSE 后 HOLD 驻留成功后，左侧调用完整 `left_store`，右侧调用完整 `right_store`。
- 复用 Store 内已有放果点 OPEN、驻留和返回 Home，不额外调用 OPEN 或 Home。
- `--model A|B` 均采用这一流程；成功意味着 planner 报告整个 Store 执行完成，不是视觉确认果实入筐。

本文件是后续实现和 review 的交接依据。当前为只规划阶段，未编辑业务代码，未执行程序、构建、测试或运动命令。完成计划仍须等待扩展的人类执行审批。

## 范围

实现和直接相关说明限定为以下文件，保留其中已有用户改动：

1. `test_camera_arm_target.sh`：`usage()` 中的流程说明。
2. `src/fruit_mission/fruit_mission/camera_arm_target_client.py`：`CameraArmTargetClient.run_sequence()`；必要的模块/类说明与 `main()` 内左右选项帮助文字。
3. `README.md`：相机与机械臂配合调试段落及对应流程说明。
4. `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`：独立相机调试客户端段落。
5. `docs/PROJECT_ARCHITECTURE.md`：7.3 中 `test_camera_arm_target.sh` 单步联调说明。
6. `src/fruit_mission/README.md`：仅修正直接涉及该调试入口的描述，使其与实际独立客户端及新增 Store 流程一致。

只读参考已核查：`src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md`、`src/fruit_arm_planner/config/arm_presets.yaml`、`src/fruit_arm_planner/fruit_arm_planner/trajectory_planner.py` 的相关规划逻辑、`src/fruit_mission/fruit_mission/camera_target_planner_client.py`、`src/fruit_mission/config/{camera_extrinsic,script_ros_clients}.yaml`、`src/fruit_mission/{setup.py,package.xml}`、`scripts/setup_fruit_runtime.bash`、`docs/RULE.md`。

非目标：不修改 planner、姿态 YAML、共享 `CameraTargetPlannerClient`、正式 A/B/C 比赛编排、模型选择/目标筛选、坐标换算、服务接口、串口实现或超时；不新增重试、恢复流程、提拉姿态、导航、二维码、语音、依赖或测试文件；不更改启动/进程所有权及驻留策略，不清理无关脏改动，不提交或切换分支。

## 决策

1. **执行时机，用户 Q1：抓取成功后。** 不是 Look 刚到位后立即放果，也不在抓取后额外返回 Look。
2. **左右范围，用户 Q2：两侧都增加。** `store_preset = f'{direction}_store'` 与现有 `look_preset` 的方向语义一致，不新增 CLI 参数。
3. **放果动作和结束位置，用户 Q3：复用完整 Store。** 当前源码 YAML 两个 Store 各有四个 2000 ms 航点，第二点 `gripper: 1`，其余 HOLD，末点为 Home 的 TCP 位置；未显式指定 OPEN 驻留时，共享规划器采用 `planning.default_dwell_ms: 500`。因此每个 Store 名义时长约 8500 ms。姿态、夹爪时序和标定由现有 YAML 保持所有权。
4. **失败处理，用户 Q4：沿用调试工具失败即停止。** 前置失败不调用 Store。Store 缺失、拒绝、执行失败或超时记录具体失败原因并 `return 1`；不重试、不补发 OPEN、不自动恢复 Home。调用现有 `execute_preset()`，保留共享客户端对 `response.success` 的校验、超时取消与 STOP，以及 `destroy_node()` 的既有 STOP。脚本保持失败日志和后台进程驻留供检查。
5. **实施位置，用户 Q5：允许修改真实 Python 客户端及直接相关说明。** Shell 的 `main()` 仅负责启动栈和调用 `ros2 run fruit_mission camera_arm_target_client`；动作追加在客户端 `run_sequence()`，不在 shell 再发第二套机械臂服务。
6. **成功判定：** 只有 grasp/CLOSE HOLD 和对应 Store 的 planner 服务都报告成功后才能 `return 0`，届时替换原有“停止在目标点/no harvest”日志。不要仅凭请求提交、future 完成或 STOP 发布宣称动作成功或停稳。
7. **超时和安全边界，用户 Q5：保持现状。** 共享客户端每项端点/服务等待独立计时且最多 10 秒；当前抓取前停稳等待 5 秒等较短等待不延长。Store 整体是一次 preset 操作，不拆段、不改超时或用 sleep 掩盖问题。未知运动沿用取消/STOP；无实测停稳不得自行启动恢复运动。
8. **验证边界，用户 Q5：自动验证不控制硬件。** 使用语法检查、定向 lint、构建、帮助输出与人工 diff 审阅。本改动只复用一项现有服务调用，不新增算法或协议，遵守项目规则不新建测试，也不做源码扫描断言或深度 mock。实机放果另由操作者现场验证。

## 执行步骤

1. **复核基线并修改实际动作入口。**
   - 依赖：计划经扩展人类审批后方可开始。
   - 文件/符号：读取当前 `test_camera_arm_target.sh:main()`、`CameraArmTargetClient.run_sequence()`、`CameraTargetPlannerClient.execute_preset()/finish_motion()` 和源码 YAML 的 `left_store/right_store`；通过 Git status/diff 复核已有脏改动，不能把其当成本任务成果。
   - 改动：在 `run_sequence()` 中已有 `move_to_base_target(req, self.wait_until, stationary_rps)` 成功分支后构造对应 Store 名称，记录执行日志，调用 `self._camera_planner.execute_preset(store_preset, self.wait_until)`；失败记录预设名和返回原因并返回 1；成功记录抓取及 Store 工序完成再返回 0。删除被替代的“抓取后停在目标点”成功文案。复用当前错误处理写法，不新增抽象或改共享类。
   - 检查命令（后续执行终端先 `cd /home/rc4/fruit_ws` 并 `source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash`）：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/camera_arm_target_client.py`。
   - 期望：语法检查退出 0；人工审阅确认新增调用位于抓取成功分支之后，所有失败仍返回非零。
   - 停止条件：基线/预设语义已改变、存在无法隔离的重叠用户编辑、出现未确认的恢复需求或语法失败；先解决本次修改内的问题，不能覆盖用户改动或扩大范围。

2. **同步入口帮助和受影响文档。**
   - 依赖：步骤 1 的最终执行顺序已确定。
   - 文件/符号：`test_camera_arm_target.sh:usage()`，Python `main()` 左右选项帮助及直接受影响的模块/类说明；范围列出的四份 Markdown 的相机机械臂调试段落。
   - 改动：说明左右观察、识别抓取后执行对应 Store，预设内放果并回 Home；保留模型、参数、检测筛选、独立超时和驻留行为说明。根 README 流程须与实现一致包含已有起始 Home。修正 `src/fruit_mission/README.md` 当前声称该入口复用正式比赛积木的相关段落，描述实际独立单点客户端；不改正式比赛工序段落。以 Store 放果工序替换该入口“原地停止/不执行 Harvest”的旧描述，避免暗示正式比赛 Harvest 名称映射也发生变化。
   - 检查命令（已准备的同一终端）：`bash -n test_camera_arm_target.sh`；`git diff --check -- test_camera_arm_target.sh src/fruit_mission/fruit_mission/camera_arm_target_client.py README.md src/fruit_mission/README.md src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md docs/PROJECT_ARCHITECTURE.md`。
   - 期望：均退出 0；人工 diff 审阅确认说明与步骤 1 一致，未改变 CLI 解析、启动/清理逻辑和无关文档。
   - 停止条件：语法错误、本次引入的空白错误、说明与代码不一致或 diff 混入无关改动；若错误来自原有未提交工作，记录区分，不擅自清理。

3. **完成定向检查和包构建，记录真实证据。**
   - 依赖：步骤 1、2 完成。每个新终端均从工作区加载干净 runtime；setup 验证失败立即停止，不通过手工 PYTHONPATH 绕过。
   - 文件/符号：检查修改过的 Python 客户端及 shell；复用现有 `setup.py` 的 `camera_arm_target_client` entry point，包注册/manifest 不需修改。
   - 命令：
     ```bash
     cd /home/rc4/fruit_ws
     source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
     /usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/camera_arm_target_client.py
     bash -n test_camera_arm_target.sh
     ament_flake8 src/fruit_mission/fruit_mission/camera_arm_target_client.py
     colcon build --base-paths src --packages-select fruit_mission \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
     source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
     ros2 run fruit_mission camera_arm_target_client --help
     bash test_camera_arm_target.sh --help
     ```
   - 期望：语法、lint 和构建成功；两项 `--help` 正常退出并呈现更新说明。客户端 argparse 的帮助路径在 `rclpy.init` 之前退出；shell 的帮助在启动栈之前退出，均不会发出运动命令。
   - 停止条件：runtime 校验、lint、构建或帮助检查失败。若原因是已有依赖未构建/旧接口不一致或其他用户修改，明确记录原因并暂停需要该环境的验证，不通过改接口、依赖、生成文件或无关源码消除失败。不得声称未执行的检查通过。

4. **交付审阅，并分离人工硬件验收。**
   - 依赖：步骤 3 完成或已明确记录阻塞项。
   - 文件/符号：仅复核范围内最终 diff、`run_sequence()` 的成功/失败路径及 review 文档；不新增测试代码。
   - 静态检查命令：再次针对步骤 2 所列文件运行 `git diff --check -- ...`，仅在新增修改需要时重复相关检查；审阅确认共享客户端、YAML、比赛编排无本任务修改。
   - 人工硬件验收命令，仅供操作者在安全场地使用，不由代理作为自动检查运行：
     ```bash
     cd /home/rc4/fruit_ws
     source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
     ./test_camera_arm_target.sh --model B --left
     # Ctrl-C 退出并确认停稳、进程清理后，再进行下一次：
     ./test_camera_arm_target.sh --model B --right
     # A 模型也分别检查两侧：
     ./test_camera_arm_target.sh --model A --left
     ./test_camera_arm_target.sh --model A --right
     ```
   - 期望：每次仅抓取一个有效目标；抓取及 HOLD 成功后进入对应 Store，现场观察到果实释放并回 Home，日志给出对应 preset 和 planner 轨迹执行结果；脚本继续驻留直到 Ctrl-C。
   - 失败验收：人工在隔离且安全的环境观察前置失败不进入 Store、Store 失败不打印整体成功、超时保留原取消/STOP；不在真实运行中故意制造运动故障。未获安全验证环境时记录这些路径为人工代码审阅结论，不能称作运行测试通过。
   - 停止条件：姿态不可达/越限、果实无法安全脱离、碰撞风险、反馈无效、紧急状态或运动结果未知，立即停止验收，依靠既有 STOP/急停和现场操作确认停稳；不重试、不补动作、不延长超时。硬件验收未做时在交付中明确标为未完成。

## 验收

- 左右两侧和 A/B 模型遵循用户确认顺序；没有抓取前 Store，也没有抓取后额外 Look。
- `run_sequence()` 仅在抓取及 HOLD 成功后调用一次对应完整 Store，仍通过 planner 服务确认执行完成。
- Store 的放果 OPEN/驻留/回 Home 均由现有 YAML 完成，不新增重复夹爪或 Home 请求。
- 前置失败无新增 Store 调用；Store 失败返回非零；成功日志仅在 Store 成功后出现；原取消、STOP、退出清理和进程驻留行为保留。
- shell 帮助、Python 帮助及直接受影响文档与实现一致；没有覆盖无关用户修改。
- 语法、定向 lint、包构建和帮助检查记录实际命令、结果及任何环境阻塞。当前只有读取和问答证据，上述检查全部尚未执行。
- 人工硬件验收独立记录；未执行时不得宣称水果已可靠入筐或完整实机闭环已验证。

## 风险

- 工作区已有大量未提交改动，目标 shell、Python 客户端、相关文档和姿态 YAML 都处于 modified 状态；必须按现有内容叠加最小修改，保留已有工作。
- 当前 Store 名义时长约 8.5 秒，客户端 10 秒等待只剩约 1.5 秒用于传输及完成回执，planner 自身另有 2 秒 execution timeout buffer。该差异可能触发客户端提前 STOP；用户已确认本次不延长超时或改预设，需要如实报告实际超时。
- 已存在的 Store 姿态及 PTP 中转不提供无碰撞保证，也没有本轮实机可达性/放果验证证据；复用配置不意味着物理安全和果实可靠入筐已验证。
- 原串口桥可靠队列跨消息类型的轨迹装载顺序、STOP 优先级和下位机实际执行仍需现有工程验收，不能靠新增 sleep 宣称解决。
- 服务成功仅说明匹配轨迹已报告 COMPLETED，不能证实水果夹牢、已脱柄或实际释放；不在本任务中增添感知确认或提拉动作。
- 此入口是人工硬件调试工具；不得把完整脚本或运动服务当成自动测试。未知运动时 STOP 发布不是停稳确认，禁止自行继续运动。
