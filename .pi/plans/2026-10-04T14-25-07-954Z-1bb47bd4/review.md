## 范围

本轮为只读 review：读取了既有 `plan.md`、`progress.md`，使用 `workflow_git` 核对启动 baseline、当前 diff 和 status，并读取相关实现、接口和已有测试；没有编辑业务文件、执行代码、构建、运行测试或下发硬件命令。

交接文件存在范围差异：`plan.md` 是最初获确认的 Store 扩展方案（六个业务/文档文件，并明确不改超时、共享客户端和比赛编排）；`progress.md` 及会话交接记录了后续机械臂等待、导航推进超时、静止门禁和恢复分支裁剪。本轮审查 Store 入口及这些已记录后续改动的直接调用链，不把进度记录当成新增执行授权。后续修复须由扩展审批，且应先对齐交接文档。

保留已确认需求与决策：

- 顺序为 Home → 对应侧 Look → OPEN/HOLD → 识别与基座换算 → 目标 CLOSE/HOLD 成功 → 一次完整 `left_store`/`right_store`。适用于 A/B 模型；Store 自带 OPEN、驻留、回 Home，不补发 OPEN/Home、不重试、不额外回 Look。
- 调试客户端前置失败不进入 Store；Store 失败返回非零，只有完整 Store 的 planner 服务报告成功后才报告整体成功。
- 后续确认取消机械臂执行时长看门狗、导航推进看门狗，简化普通到位等待和重复恢复分支；保留识别超时和二维码超时/mission 固定计划兜底。未知运动的取消、STOP、有效反馈和恢复前实测停稳仍是独立安全边界。

归因依据：

- baseline 已包含独立相机客户端、模型选择/筛选、起始 Home、ArmTelemetry/TCP 迁移、几何/映射/外参修改，以及 `left_store/right_store` YAML。它们不是本次执行新增成果，不擅自归因。
- baseline 中没有抓取后的 Store 调用；当前 `camera_arm_target_client.py:298` 起的对应侧 Store 调用新增且位置正确。
- baseline 的 `_gripper_at_measured_pose()` 通过 `wait_until(is_stationary, ...)` 获取新鲜静止反馈；当前改为 `sleep(0.2)` 加仅检查状态存在。
- baseline 的 `actions.navigate()` 失败时调用一次 `recover_home()`，后者清理 `_pending_observation`；当前删除了该调用。
- baseline 的 `recover_home()` 在 STOP 后检查新收到的有效 IDLE/低速反馈；当前删除了该检查并改成固定睡眠。
- 其他既有脏改动、音频、检测 GUI、以及未在本任务交接中列明的目标补偿改动，不纳入本轮缺陷归因和修复范围。

已读取 Shell `main()/cleanup()`、调试客户端 `run_sequence()/wait_until()`、共享客户端 `open_gripper()/move_to_base_target()/finish_motion()`、planner 的运动接收/关节 PTP/预设规划、mission 导航与观察交接、A/B 调用处、相关文档及 `test_axis_planner.py`。Store 的预设调用和失败返回本身未发现 P0/P1；下列问题来自后续裁剪对关键路径的影响。

## P0

无。

## P1

### R1：夹爪 OPEN/HOLD 使用未校验的新鲜度的缓存角，可能执行额外关节回摆

- **优先级：P1。**
- **位置：** `src/fruit_mission/fruit_mission/camera_target_planner_client.py:201`，`_gripper_at_measured_pose()`，重点为第 205–213 行；调用处为 `open_gripper()` 和 `move_to_base_target()`。调试入口 `camera_arm_target_client.py:247` 附近还先睡眠 0.3 秒。
- **触发条件：** Look/目标运动服务完成回包已被客户端处理，但客户端最后处理的 ArmStatus 仍是该运动途中或更早的反馈。调试客户端仅由 `wait_until()` 的 `rclpy.spin_once()` 处理回调，`time.sleep()` 期间不处理新消息；服务响应与状态订阅来自不同 ROS 端点，服务成功不保证客户端缓存已更新为终点反馈。
- **影响：** 所谓原地 OPEN/HOLD 直接把旧四轴角作为新 `MoveArmJoints` 目标。planner 使用自己的新鲜实测角作为起点，并向这个旧目标做 PTP，因此可能在观察前或 CLOSE 后发生非预期回摆，破坏观测姿态、驻留和抓取；Store 的前置抓取流程由此不可靠。当前只检查 `status is not None`，也不再过滤反馈年龄、任务状态、错误码、有限速度或停稳条件。
- **证据：** baseline 的此函数调用 `wait_until(lambda: self.is_stationary(max_joint_speed), endpoint_timeout_sec)` 后才取角；当前第 205 行睡眠后直接读 `_status`。`is_stationary()` 第 128 行起仍实现了新鲜度、有效角、状态、错误码和有限低速检查，但此路径不再使用。planner `_srv_move_joints_cb()` 第 540 行起把请求角当目标传给 `plan_joint_point_to_point(start_q=start, end_q=target_q_hw)`，不会把客户端旧角自动修正为原地 HOLD。源码 `status_max_age_sec=0.25`，观察后的 0.3 秒睡眠本身就超过该窗口。
- **结论限制：** 已确认代码允许该时序；没有实机回摆或掉果的运行证据，不声称事故已经发生。

### R2：导航失败遗留并行 Look，下一目标抛异常而终止比赛流程

- **优先级：P1。**
- **位置：** `src/fruit_mission/fruit_mission/actions.py:13`，`navigate()` 的失败分支；关联 `mission_context.py:188` 的 `navigate_named()`、第 597 行的 `start_observation()` 和 `Logic.py:33` 的 `run_a()` / 第 56 行的 `run_b()`。
- **触发条件：** 有 `side` 的导航已执行 `start_observation(side)`，随后导航服务不可用、目标被拒绝、导航结果失败或观察失败导致 `_send_waypoint_names()` 返回 False。
- **影响：** 当前 `navigate()` 直接返回 False，当前采摘不会消费该 Look，`_pending_observation` 仍为非空。下一 A 点或 B 树再次调用 `start_observation()`，第 601–602 行抛出 `RuntimeError('Previous observation has not been consumed')`。`FruitMissionNode.run()/main()` 没有处理这个 RuntimeError，整场任务退出，违反局部失败后继续 A/B/C 的关键要求。
- **证据：** `navigate_named()` 在第 203 行附近先启动 Look，随后 `_send_waypoint_names()` 失败直接返回；`start_observation()` 最后保存 `(side, future)`，即使 future 为 None 也非空。baseline 的 `actions.navigate()` 失败时调用 `recover_home()`，该函数第一步清空 `_pending_observation`；当前删除了调用，未把清理移入共用导航失败路径。A1 失败后 `Logic.run_a()` 直接进入 A2；B 导航失败后同样直接进入下一树。
- **结论限制：** 此问题按现有调用链可确定，不依赖实机卡死；模拟 FollowWaypoints 拒绝目标即可验收。

### R3：恢复 Home 在 STOP 后未获得有效低速反馈就重新提交运动

- **优先级：P1。**
- **位置：** `src/fruit_mission/fruit_mission/mission_context.py:655`，`recover_home()`，重点为第 663–670 行；关联 planner `_on_arm_status()` 和 `_run_motion()`。
- **触发条件：** `_arm_home_confirmed=False`，调用恢复并发出非阻塞 STOP，导航取消已完成或底盘原本无导航。0.3 秒后机械臂仍未提供新的有效停稳反馈，或虽报告 IDLE，但速度为 NaN/Inf/超过阈值。
- **影响：** 客户端仍提交 Home。planner 的运动准入仅检查新鲜有效位置和 IDLE/COMPLETED；`_on_arm_status()` 只依据有效位置与 IDLE 清除 `_require_idle`，不验证速度。因而不能依靠 planner 补回这里删除的低速/有限数确认，恢复运动可能在物理停稳未知时启动。STOP 发布成功和固定等待都不是实测停稳。
- **证据：** baseline 在记录 `stopped_after` 后，用 `is_stationary(joint_speed_rps, stopped_after, require_idle=True)` 确认 STOP 后的新反馈，失败则禁止 Home；当前替换为 `time.sleep(0.3)` 和无条件 `send_return_initial_pose()`。接口文档和当前 `CAMERA_TARGET_TRANSFORM.md` 仍明确要求 STOP 后新收到的无错误 IDLE、有限且在限位内的四轴低速反馈；planner 第 295 行附近和第 337 行起的实现没有速度门控。
- **结论限制：** 不认定每次恢复均会发生危险；问题是已删除未知运动恢复的必需准入条件，NaN 速度的 IDLE 帧可作为确定的拒绝场景验收。

## 修复步骤

以下仅为待审批修复方案，不在本轮执行。只修 R1–R3，不重加执行时长/导航推进看门狗，不改变二维码兜底、识别期限、Store 配置、标定、目标补偿或比赛路线。

1. **先恢复共享夹爪请求的实测反馈校验（R1）。** 修改 `camera_target_planner_client.py:_gripper_at_measured_pose()`，复用已有 `is_stationary()` 和调用方 `wait_until()`，处理 ROS 回调并取得服务完成后更新的新鲜、有限、在限位内、无错误低速反馈，再冻结四轴角并提交 OPEN/HOLD。删除作为有效反馈替代物的 `sleep(0.2)`；缺失/无效反馈不能下发该关节运动。保留无机械臂执行时长超时的用户决策；反馈有效性与动作执行看门狗分开。相应更新调试客户端和 `CAMERA_TARGET_TRANSFORM.md` 的实际等待说明，不新增反馈替代值或第二套控制器。
2. **恢复未知运动的安全 Home 交接（R3）。** 修改 `mission_context.py:recover_home()`，保存 STOP 前的接收时间边界，在处理回调后要求 `is_stationary(..., received_after=stopped_after, require_idle=True)` 成立才发一次 Home；保留 EMERGENCY 锁和导航取消。没有有效停稳证据时返回 False 并保持运动禁止，不能靠睡眠放行。复用原有短且独立的 STOP 反馈确认期限（最多 10 秒），这不是机械臂轨迹执行超时；不循环尝试 Home。同步恢复说明，使接口文档与实现一致。
3. **把失败导航的观察清理放在一个共用路径（R2）。** 在 `actions.py:navigate()` 的失败分支复用一次修复后的 `recover_home()`，或将等效的待消费 Look 清理、STOP/导航取消及一次安全 Home 放入 `MissionContext.navigate_named()` 的失败返回路径，二者只选一个，避免双重恢复。不能只置空 future 而遗留执行中的 Look，也不能删除 `start_observation()` 的占用校验。确保恢复失败仍能推进软件流程且不授权后续危险动作；保持 C 区已有清理不重复。
4. **同步交接与针对性文档。** 对齐 `plan.md/progress.md` 对 Store 扩展和后续裁剪的范围及验证状态。只改 `src/fruit_mission/README.md`、`src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md` 等直接受影响的反馈/恢复说明；保留已确认 Store 顺序及超时裁剪方向，不清理无关文档或 baseline 脏改动。
5. **修复获批后执行定向静态检查。** 每个终端先准备项目 runtime：

   ```bash
   cd /home/rc4/fruit_ws
   source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
   /usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/camera_target_planner_client.py src/fruit_mission/fruit_mission/mission_context.py src/fruit_mission/fruit_mission/actions.py
   ament_flake8 src/fruit_mission/fruit_mission/camera_target_planner_client.py src/fruit_mission/fruit_mission/mission_context.py src/fruit_mission/fruit_mission/actions.py
   bash -n test_camera_arm_target.sh
   colcon build --base-paths src --packages-select fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
   source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
   ros2 run fruit_mission camera_arm_target_client --help
   bash test_camera_arm_target.sh --help
   git diff --check -- src/fruit_mission/fruit_mission/camera_target_planner_client.py src/fruit_mission/fruit_mission/mission_context.py src/fruit_mission/fruit_mission/actions.py test_camera_arm_target.sh
   ```

   期望：退出码均为 0；帮助路径不启动机械臂。若最终修改调试客户端，也把该文件加入 compileall/lint。runtime 校验失败即停，保留并区分原有问题。
6. **补充真实 ROS 接口的离线时序验收。** 如审批包含这项验证，仅在忽略的 `tmp/` 中放一次性检查脚本 `review_camera_store_check.py`，使用实际 rclpy 的 service/action/topic 模拟端点和公开入口 `open_gripper()/move_to_base_target()/recover_home()/navigate()`；不 monkey-patch 私有状态、构造 ROS 类替身或编写源码扫描断言。使用隔离 ROS domain，不启动串口、传感器驱动或真实运动节点。该脚本当前不存在，下列为修复后验证命令，不是已通过证据：

   ```bash
   cd /home/rc4/fruit_ws
   source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
   ROS_DOMAIN_ID=214 /usr/bin/python3 tmp/review_camera_store_check.py
   ```

   必须确认该 domain 没有硬件端点后运行。脚本应通过公开 ROS 接口覆盖下一节列出的 R1–R3 场景，非预期请求/异常时退出非零；不加入包的单元测试目录，不执行完整实机脚本。

## 验收

本轮真实证据是静态读取与 baseline/diff 对照。保留上一执行阶段的记录：compileall、六个 Python 文件 ament_flake8、`fruit_arm_planner/fruit_mission/fruit_navigation` 定向构建及两项 CLI 帮助成功；导航纯算法 pytest 为 `16 passed`。该测试文件测试几何和 `ProgressWatchdog` 数学行为，不覆盖相机/机械臂反馈时序、恢复交接或并行 Look 的失败清理。它们不能证明 R1–R3 已解决。

历史全仓 `git diff --check` 报 `arm_presets.yaml:91/103` 尾空白，限定 mission/navigation/planner-node 范围检查通过；对应 YAML 空白已在 baseline，不归因给本次执行，也不顺带清理。本轮未重跑这些命令。实机放果、无碰撞路径、串口跨消息轨迹装载/STOP 优先级、物理入筐均未完成验收。

待修复后验收标准：

- **R1：** 服务完成先于终点 ArmStatus 时，OPEN/HOLD 不使用旧的运动途中角。新反馈到达后请求角等于该次有效反馈；缺失、陈旧、NaN/Inf、错误状态或非静止反馈不得产生夹爪关节请求。正常 CLOSE/HOLD 成功后只调用一次对应完整 Store。
- **R2：** 第一个 A 点或 B 树的导航在启动 Look 后被拒绝/失败时，当前项跳过并清理待消费观察；下一项目不会抛出 `Previous observation has not been consumed`。同一失败最多一次安全 Home，恢复不安全时仍保持物理运动禁止。
- **R3：** STOP 服务成功但尚无 STOP 后的有效停稳反馈时，Home 请求数为 0。旧 IDLE、带错误 IDLE、NaN/Inf/过快速度 IDLE 均不能放行；新鲜无错误、有限且在限位内的低速 IDLE 到达后才允许一次 Home。EMERGENCY 不自动解锁。
- **需求保持：** A/B 模型和左右选项不变；前置失败不调用 Store，Store 失败不报告整体成功；OPEN/驻留/回 Home 留在既有 Store YAML；识别/二维码等待与固定计划保留，执行时长和导航推进看门狗维持删除。
- **交付：** 保存真实检查命令、退出码和隔离场景结果；未做硬件验收时明确记录未完成。当前状态为发现 0 个 P0、3 个 P1，等待扩展人类审批修复。
