# 单相机常开与观察窗口重构审查

**结论：不建议按“无问题、验收完备”通过。P0：0；P1：3；P2：1。核心黑盒验收尚未完整完成。**

本轮只读审查，未运行构建、测试、ROS 或硬件命令，未编辑业务文件。所有修复和补跑均未执行，等待扩展的人类审批。压缩摘要中的“P0/P1 为零、验证完备”结论不被当前代码及覆盖支持，以本文为交接依据。

## 范围

依据同目录 `plan.md`、`baseline.md`、`progress.md`，核对 perception/QR 生产者、Mission/手动客户端消费者、A/B/C 编排、planner 保护调用、launch/源码配置、启动就绪及现有测试产物。重点文件：

- `src/fruit_perception/src/{fruit_perception_node,b_target_selector}.cpp`、`msg/FruitDecision.msg`、CMake、launch/config。
- `src/fruit_qr_planner/fruit_qr_planner/qr_node.py`。
- `src/fruit_mission/fruit_mission/{mission_context,camera_arm_target_client,camera_target_planner_client,actions,Logic,mission_node,stack_readiness_client}.py`、任务/客户端 YAML。
- `start_fruit_mission.sh`、`test_camera_arm_target.sh`及受影响参考文档。
- `tmp/check_observation_windows.py`、`tmp/observation_windows_check.json`、gtest XML、`log/latest_test/fruit_perception/stdout.log`。

### 保留的确认决策

1. 相机常开、模型持续推理，保留 busy 丢帧；不是增加推理布尔开关。
2. 正式任务导航与 Look/QR 成功后开窗；扫码不提前缓存，失败用 mission 固定兜底继续。
3. 删除观察前 `stable_stop`，不添加窗口内运动监测；依赖用户对观察至目标冻结期间物理静止的保证。保留未知运动 STOP、取消、EMERGENCY 锁、新鲜有限反馈、恢复前停稳与一次安全 Home。
4. ID、图像截止时间、区域/用途/期望类型组成窗口上下文；关闭在客户端立即生效。
5. B 一树一次观察，首次动作前冻结全部基座请求，第二抓不重新观察/转换。
6. 手动工具保留 Home→Look→机械臂静止→OPEN/HOLD→观察→CLOSE/HOLD，成功后留在目标点，不增加存放动作。
7. 仅离线验收，不运行真实 mission/运动服务，不承诺硬件稳定性、识别率、时延或热负载。

归因边界：启动前已有的关节映射/TCP 遥测、Wis'adel 语音及正常结束计数播报、导航修改不列为本次新增缺陷。用户已有 `left_store/right_store` 配置应保留；本报告关注此次是否新增调试流程调用，不能仅据 HEAD 总 diff 撤销既有用户工作。

## P0

无已证实 P0。

## P1

### R1：非对象 QR JSON 会逃出回调并终止整场任务

- **位置**：`src/fruit_mission/fruit_mission/mission_context.py:1004–1008`，`_receive_plan` 及其末尾异常处理。
- **证据**：`json.loads` 后只检查活动窗口，再直接 `plan.get(...)`，未确认顶层为 dict。异常处理仅捕获 `ValueError/TypeError/KeyError`。在窗口开启时收到合法 JSON `[]`、`null` 或 `1`，会触发未捕获 `AttributeError`。此次 diff 将原 `plan['zone']` 改为 `.get`；原非对象索引引发的 `TypeError` 在既有处理范围内，这是本次回归。
- **影响**：计划订阅回调→`rclpy.spin_once`→等待→mission 主流程；`mission_node.py::main` 只捕获 KeyboardInterrupt。坏消息会结束 A/B/C，而非忽略后用固定兜底继续。此为只读语言行为/调用链推导，未运行故障注入。
- **最小修复**：在任何 `.get` 前检查顶层对象类型，不合法记录并返回 None；不要对整场流程新增宽泛异常吞噬。新增元数据校验见 R4。

### R2：手动单点工具增加了计划未批准的存放/释放运动

- **位置**：`src/fruit_mission/fruit_mission/camera_arm_target_client.py:293–302`，`run_sequence`。
- **证据**：CLOSE/HOLD 成功后新增 `execute_preset(f'{direction}_store', ...)`。`baseline.md` 中客户端原结尾明确“Stopping at target position as requested (no harvest)”并立即返回；`plan.md` 决策 9 要求留在目标点。脚本帮助与包 README 已改为 store/放果/Home，文档同步不等于授权。
- **影响**：按原单点调试约定启动后，抓取结束还会额外位移及释放夹爪；未配置/失败的 store 也成为新的失败条件，超出此次相机重构范围。
- **最小修复**：删除新增 store 调用及相应新增帮助/成功文案，恢复 CLOSE/HOLD 后停止。保留原预设配置、正式任务 Harvest/Home。若用户在别处另行批准 store，先将确认写入本任务决策，不能凭已有预设存在推定授权。

### R3：B 树已确认空结果，Mission 却等满识别超时

- **位置**：`src/fruit_mission/fruit_mission/mission_context.py:923–925`，`wait_b_targets`。
- **证据**：`_on_decision` 收到 `targets_ready=true` 缓存列表，空列表也是有效完成结果。`BTargetSelector::update` 达到 8 帧且没有候选时返回 `{true, []}`，随后 ready 锁定并持续返回空列表。此次 diff 把原 `return targets` 改成 `if targets: return targets`。
- **影响**：没有目标的已完成观察不能立即交给 `_select_b_targets` 正常播报/跳过，而等待默认 10 秒后误报“识别超时”。选择器已锁为空，继续等帧也不会改善结果。这是确定的流程/时延回归，违背减少无效等待的目标。
- **最小修复**：当前窗口完成结果校验后直接返回过滤列表，包括 `[]`；保留 None 表示未完成/超时，保留坐标校验，不新增识别重试。

## P2

### R4：最终消费端缺少已确认的窗口元数据校验

- **位置**：`mission_context.py:970–989` 的 `_on_decision`、`:1010–1019` 的 `_receive_plan`；`camera_arm_target_client.py:66–74` 的 `_on_decision` 和 `_wait_for_target`。
- **证据**：两个客户端只按 ID 接受 FruitDecision，原等待中的窗前帧检查被删除；手动客户端未保存开始时间。Mission 未冻结供消费检查的 area/purpose。QR 时间直接从 JSON 取值比较，未校验严格整数/非 bool/纳秒范围，缺失默认零。例如合法任务内容和同 ID 搭配 `image_stamp_sec:NaN`，Python 比较均为 false，可绕过截止判定。
- **影响/定级**：正常生产者已有窗口/时间校验，不声称其正常输出必然过期；但计划要求最终消费边界独立拒绝不可信元数据，不能以生产者通常正确代替。R1 是此边界已可导致退出的高优先级问题，此项记录剩余契约缺口。
- **最小修复**：复用开窗命令保留最少上下文；decision 入缓存前检查 ID、harvest 用途及合法窗后 header 时间；QR检查 qr 用途、当前区域等于 plan zone、ID及合法窗后时间。窗口 ID 绑定区域，不要求给 FruitDecision 再加区域字段/接口包。关闭复用原上下文，避免硬编码 `purpose='harvest'` 或手动 `area='A'` 改写 QR/B 上下文。不增加 echo/监测/重试层。

## 修复步骤

以下全部待人类审批，未执行：

1. 在隔离 worktree 再冻结当前用户改动；store 如有并发修改先确认归属，不重置工作区，不撤销既有语音/映射/预设。
2. 最小修复 R1/R4 公共消费边界，不加依赖/消息包/旧模式兼容，不恢复 stable_stop。
3. R3 删除非空条件；R2按确认计划删除新增 store 调用并同步说明，另获授权则先更新决策。
4. 扩充既有 tmp 黑盒脚本，只用公开 ROS 接口、无硬件运动，不新增源码断言/私有 ROS mock。补非对象输入、完成空列表消息证据、C-only、非法命令不生效、RGB/Depth 独立截断场景。成熟目标/两目标及锁定隔离缺可信输入时明确 BLOCKED，不能悄悄减少要求。
5. 按 plan 干净 runtime 和受影响包命令重建、补跑既有算法、针对性语法/风格/ABI及黑盒，记录退出状态、输入/输出 ID和实际图像时间。每操作等待≤10 秒。回交当前 main 后复核，只清理本任务 worktree/分支。

## 验收

### 已有可读取证据

| 项目 | 证据与结论 | 边界 |
| --- | --- | --- |
| 感知算法 | 四份 2026-10-04 23:36:45–46 XML：DetectionParser 9、ImageProcessing 5、TargetLocalization 3、DetectionOutput 3，failures/errors均0；latest_test stdout四项return code 0 | **20个gtest用例、4个CTest目标通过**。原“24/24”须区分聚合口径，不应写成24个独立算法用例。 |
| 黑盒 | JSON报告2026-10-04 23:37:14、DOMAIN138、14/14、all_passed=true；已读对应断言 | 只证明现有14项断言，不是计划所有覆盖完成。 |
| 构建/风格/ABI | progress记载主工作区build、compileall、ament风格、bash -n、diff --check、ldd通过 | 保留执行阶段记录；本轮未补跑，修复后须刷新。 |
| 组件/配置 | launch中camera/perception共容器且各intra-process=true；源码YAML开启color/depth/sync/align/rgbd；新消息含ID，旧模式目标在diff删除 | 静态架构核对，不是硬件连续出流实测。 |
| 安全链 | 共享客户端有限/限位/新鲜反馈、结果处理/超时STOP，Mission恢复前STOP后新反馈停稳/取消/EMERGENCY锁仍在 | 只读核对，不是串口装载、STOP优先级或实机验证。 |

构建目录仍有03:07:35旧 `test_perception_mode.gtest.xml`（6个历史策略用例），当前CTest清单不含该目标，不算本次重新执行；本轮未修改生成目录。

### 核心覆盖缺口

1. **成熟/两目标证据**：所有感知输入是 `make_synthetic_rgbd` 纯灰图，A/B每场景仅单帧，没有A/C成熟锁定或B双目标ready，不能证明新窗清除旧锁定。plan要求缺可信素材标为阻塞，空帧不能替代。
2. **异步与RGB/Depth**：没有推理在途时开/关/换窗，也未分别测试RGB窗前/Depth窗后及相反情况。命令后先等0.2秒再发图，不覆盖竞争。
3. **最终消费者**：脚本只启动perception/QR，没有mission旧锁存计划、旧ID、窗前结果、错区域/用途、非对象QR JSON和非法时间拒绝证据，不能称R1/R4已覆盖。
4. **C-only**：所谓 independent C是在同一进程已经成功B之后检查，未证明B缺失时独立C成功。
5. **非法命令实际效果**：malformed仅断言 `poll() is None`，不发后续帧证明非法命令未开窗/改窗，只证明当时没退出。图像encoding/step/长度/UTF-8黑盒拒绝也缺失。
6. **上下文**：未覆盖C期望类型和区域/expected话题交叉到达。新窗口实现/说明还接受D，已确认接口只含A/B/C；将新窗口契约收回批准范围，不随意删除既有常态D模型支持。

### 未完成步骤和交接边界

- R1/R2/R3/R4修复：未执行。
- 核心缺口补验、修复后构建/风格/ABI及最终回交：未执行；可信成熟/两目标输入是否可用尚未确认。
- `progress.md` 步骤7/9的“全部完成”保留为原执行声明，不替代上述缺失覆盖；本报告纠正其验收解释。
- 实机出流、时延/热负载、现场准确率、轨迹装载/STOP优先级、真实采摘仍不在离线通过范围。
- review文档完成；**ready=true仅表示等待人类审批，不授权修复、补跑或业务编辑**。
