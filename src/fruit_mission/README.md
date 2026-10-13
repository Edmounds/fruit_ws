# fruit_mission：可人工审核的比赛编排

## 从哪里审核

打开 [`fruit_mission/Logic.py`](fruit_mission/Logic.py)：

- `main(ctx)`：整场 **A → 扫 B 码 → B1～B8 作业 → 扫 C 码 → 按二维码顺序执行 C 作业 → C_top_waypoint_1 → 采摘汇总播报**。
- `run_a(ctx)`：A1、A2、A3、A4 逐点展开；每点先导航并并行启动左 Look，
  然后调用左、右 `pick_a`。导航与采摘是独立积木，不用循环隐藏固定路线。
- `run_b(ctx, plan)`：固定按 B1～B8 导航，使用二维码给每棵树指定的目标品种和左右侧，
  调用一次 `pick_b_tree` 完成该树工序。树间转场边走边切对侧 Look，单树结束后不执行伸直，B 区全部完成后前往 C 区途中边走边伸直。
- `run_c(ctx, plan)`：按二维码项序导航，用路线规划器确定的侧调用 `pick_c`；
  采摘完成后不追加伸直，下一段导航按是否经过 top 点选择姿态。导航失败记录并跳过，不在未到达的位置抓取。
- 分区间固定通道点直接列在调用处，不再通过隐藏的 transition 名称展开。

只要改变比赛顺序、路线、左右侧或目标，就编辑这里。航点坐标和车体朝向仍由源码
`src/navigation/fruit_nav_bringup/waypoints/odom_waypoints.yaml` 提供，使用 `odom`。
A 固定路线要求 A1～A4 都存在且类型为 goal；新增/删减采摘点时同步修改
`Logic.py` 和 `waypoint_route.py` 的必需点校验，不能期待 YAML 自动增加工序。

## 重复动作放在哪里

[`fruit_mission/actions.py`](fruit_mission/actions.py) 是重复工序实现，不是另一套
主流程，也不是只给旧大类套一层壳：

| 积木 | 参数与职责 |
| --- | --- |
| `navigate(ctx, waypoints, side=None, preset=None)` | 明确导航点；有 side 时导航与 Look 并行，有 preset 时导航与指定机械臂预设（如 A 结束前往 B 扫码、以及 B 结束前往 C 扫码的 `shenzhi`）并行。失败清理待消费动作，安全恢复并返回 False。 |
| `pick_a(ctx, side=...)` | 区域 Look/OPEN → 实测停稳 → harvest 窗口 → 成熟目标 → CLOSE/驻留 → 区域 Store/OPEN/驻留。正常不追加 Home，无重复抓取。 |
| `pick_b_tree(ctx, tree_name=..., target=..., side=...)` | 每树观察一次，按二维码品种、成熟度和基座空间限制筛选，再按检测置信度选最多两个目标。首次抓取前冻结基座请求，再逐个抓取/Store。同树成功抓取之间不回 Home，树末尾回 Home。 |
| `navigate_c(ctx, task)` | 复用现有 C 几何规划，记录路线和 camera_side；失败的记录在恢复前由分区流程保存。 |
| `navigate_c_route(ctx, names, yaws, side=None)` | C 路线经过任一 top 点时，导航开始异步启动 `shenzhi`，通过最后一个 top 点后，剩余导航并行启动目标侧 Look；收尾不启动 Look。 |
| `pick_c(ctx, task, side=...)` | 当前二维码项的单次识别/采摘、结果记录和播报；不增加方向复核。 |
| `scan_b_qr / scan_c_qr(ctx, arrived=...)` | 已到点且 QR 姿态执行完成后开启 QR 扫码窗口；到点/扫码失败均用 mission 自有固定计划，保存、播报并恢复。B 扫码后回 Home，再导航并切首树 Look；B 完成后前往 C 扫码点途中边走边伸直。 |

`arrived` 使用前一个 `navigate` 的返回值；导航失败已经执行过一次安全恢复，
扫码函数只使用兜底计划，不再为同一导航失败重复尝试 Home。

`MissionContext` 只保留 ROS 反馈、导航发送、感知窗口、planner 原语及安全交接；
品种判断和语音格式在 `decisions.py`，C 路线数学继续在 `waypoint_route.py`。
区域切换通过 `/fruit_perception_node/set_parameters_atomically` 设置 `zone` 并核对返回结果。
`/fruit_mission/current_area` 仍发布任务状态，但感知不再订阅。
参数失败只告警并推进流程，不重试。区域不匹配的 harvest 窗口不产生抓取目标。
`start_fruit_mission.sh` 调用 `fruit_mission.launch.py`，不调用感知调试入口。
顶层 launch include 串口、planner-only、导航、感知和 RobViz 基础 launch。
感知 launch 只拥有相机、检测和 QR。正式入口不复用导航、感知、planner 或串口桥。
独立 RobViz 可复用，退出时不停止该外部实例。
它先调用 `kill_fruit_processes.sh --competition` 清理旧栈和运动客户端，再启动自己的组件。
启动前由操作者保证机器人安全静止。重启导航会重建启动相对 `odom` 原点。
清理使用明确进程身份，并排除调用者及祖先进程。
独立可视化、录点器和录包保留。旧启动脚本管理的附属进程可能随父启动脚本退出。
仍有冲突节点，或 `fruit-perception.service` 会自动重启旧栈时，入口退出并提示处理。
`--reuse-navigation` 已删除，`--flow`、`--open-loop` 和 `--debug` 保留。
`--open-loop A|C|AC|none` 按区域选择开环抓取，默认 `A`。
`A` 表示 A 开环、C 闭环；`C` 表示 A 闭环、C 开环；`AC` 表示两区开环；
`none` 表示两区闭环。B 区保持原流程，`--flow` 只选择任务入口。
单独传 `--open-loop` 或传非法值会在启动前报错。
ROS 参数 `open_loop` 同样使用上述字符串，launch、节点及源码配置默认均为 `A`。
开环区域调用已有 `left_pick_openloop_a` 或 `right_pick_openloop_a`，
仍要求成熟度和品种匹配，但不等待三维深度目标。闭环区域保留原视觉目标抓取，C 区转场伸直按路线触发。
左右观察姿态直接切换，不穿插正常 Home 或新过渡动作。
独立依赖并行初始化。顶层 launch 的唯一门控复用 `stack_readiness_client stack`。
各项保留原 10 秒超时，TF 保留 5 秒，超时告警后启动 mission。
不使用固定 sleep，外围节点退出不终止 mission。
调试录包仍由 shell 在相机检查后后台启动，不阻塞 launch 的 mission 门控。
使用 `--debug` 时，入口还会在启动 ROS 子系统前记录内核 USB 事件和 udev 事件，文件分别为
当前运行目录的 `kernel.log` 和 `udev.log`。退出时监控进程与其他调试进程一起停止。
mission 退出触发本次 launch 的整栈关闭。Ctrl-C 也关闭本次拥有的子进程。
shell 保留 STOP 请求和录包、USB 监控清理。进程关闭和 STOP 发布都不是硬件停稳证明。
单点调试入口传递 `--zone`，客户端使用同一参数服务，不再等待区域话题订阅者。
独立单点调试入口 `test_camera_arm_target.sh`（`camera_arm_target_client`）不依赖导航，只接受 `--zone A|B|C --left|--right`（可加 `--debug`），不再接受 `--model`。区域同时选择模型（A/C 蔬菜、B 水果）和 `<side>_look_<zone>` / `<side>_store_<zone>`（B 根据目标基座高度自动选择 `_store_b_up` 或 `_store_b_down`）；例如 `--zone C --left` 使用 `left_look_c`、`left_store_c`。工序为 Home→区域 Look→OPEN/HOLD→识别→CLOSE/HOLD→区域 Store，不加入跨区保护动作。它是人工硬件工具，不能作为自动测试运行。
原 `mission_flow.py` 和作为判断帮助文件的 `logic.py` 已移除，避免两个主流程来源。

调用处只显示比赛参数。超时在 `config/fruit_mission.yaml`；预设名称直接遵循
planner YAML，姿态、速度、标定在对应 planner/相机源码 YAML，
不把这些值复制进比赛主流程。导航与 Look 继续并行。
共用客户端等待 planner 的本次轨迹完成后，确认匹配轨迹的新鲜实测低速反馈。
停稳确认最多 2 秒，满足即继续，不固定 sleep。
初次动作提交前也限时取得有效静止反馈，超时报告 `arm_pose_unavailable`。
识别和后续运动不再因旧缓存的瞬时抽检而跳过。
正常 Look 的 OPEN 由预设负责，不追加冗余 OPEN/HOLD 或 Home。
底盘终态仍使用导航结果，实际底盘停稳仍须现场确认。
恢复在 STOP 确认后等待新鲜 `STATUS_IDLE`，planner 保留运动占用与 STOP 后 IDLE 保护。
A 左右调用顺序直接在 `Logic.py`，不再使用
`a_observation_order` 配置。

窗口结果在入缓存前核对 ID、用途及合法窗后图像时间；扫码计划另核对区域，非对象 JSON 与非法时间忽略后继续等待或使用既有固定兜底。
姿态统一遵循 planner 的 `arm_presets.yaml` 名称定义，无中间映射：观察调用 `left_look_a/b/c` 或 `right_look_a/b/c`，放果调用 `left_store_a/b/c` 或 `right_store_a/b/c`，扫码调用 `qr_scan_b/c`，复归调用 `home`。
Look 已明确拒绝且未下发轨迹（`trajectory_id=0`），并有有效静止反馈时，不再取消导航。
到点后消费失败结果，恢复一次 Home，再推进任务。
请求异常、取消、planner 被占用或已下发轨迹的失败仍保留必要停止和导航取消。
移除了无调用者的旧 `go_to`、`go_through` 和 YAML 顺序游标。
已删除无调用者的旧路线查询 API。当前 A/B/C 路线规划和结果记录保留。
期望类型只随观察窗口发布，不再维护重复品种话题。

实际姿态顺序以 `Logic.py` / `actions.py` 为准。B 扫码及单树结束后回 Home，树间导航同时切对侧 Look；B→C 转场使用 `shenzhi`。C 开环和闭环采摘完成后均不追加固定伸直。C 扫码后的伸直交接保持不变。原有必要 `shenzhi` 未确认时仍禁止导航，Home 不冒充保护位；软件继续推进。

所有 C 区作业入口及 `full` 收尾共用 top 路段姿态编排。
规划路线包含 `C_top_waypoint_1` 或 `C_top_waypoint_2` 时，开始导航前异步下发一次 `shenzhi`。
不计算预计到达时间，也不在到达 top 点前等待机械臂完成。
两个 top 点及中间路段不执行 Look，不为第二个 top 点重复下发伸直。
通过本段路线最后一个 top 点后，消费伸直结果，再在剩余导航中并行切目标侧 Look。
普通伸直拒绝只告警，继续原规划路线，不因此回 Home、重试或绕行。
未经过 top 点的作业路线仍直接并行 Look，原航点、yaw 和二维码任务顺序保持不变。
运动交接仍使用原有反馈和占用检查；硬件故障、急停、取消或未知运动仍受现有导航停止保护。

真实姿态及已定义名称以源码 YAML 为准。本任务只引用名字，不覆盖操作者已有坐标，不补空预设或假成功；现场运行前须补齐所用预设。QR 解析按窗口区域直接选择 B/C，无自动 B→C 状态推进；窗口 ID、图像时间和固定兜底保留。
B 树收到 `targets_ready=true` 的空列表表示完成但无果实，立即按空树跳过；仅未收到完成结果才算识别超时。关闭窗口保留开启时上下文。

正式任务和相机单点调试共用 `fruit_mission.yaml` 的末端相对偏置配置：
`target_offset_ac_m`、`target_offset_b_up_m` 与 `target_offset_b_down_m`（米），数值以源码配置为准。
局部 +X 沿夹爪正前方深入，+Y 沿夹爪左侧，+Z 沿夹爪上方。
偏置使用补偿前目标方向 yaw（`atan2(base_y, base_x)`）进行旋转叠加。
A/C 使用最终请求 pitch，B 两层固定使用 0° 作为偏置旋转参考，不随 B 抓取 pitch 改变 XYZ。
不根据 Base Y 自动翻转符号。三组配置的局部 Y 默认均为 0，可按 A/C、B Up、B Down 分别标定；
B 采用局部深入与下移。正式任务按当前区域与高度自动选择，调试客户端按 `--zone` 与高度选择，
入口统一验证各配置项三个有限数，不回退、不叠加 planner 工具几何偏移。

正式任务和 `test_camera_arm_target.sh` 共用 `config/camera_extrinsic.yaml` 的抓取与高度参数：
`motion.pitch_deg: 0.0` 用于 A/C，`motion.pitch_b_up_deg: 0.0` 与 `motion.pitch_b_down_deg: -30.0` 用于 B 的 Up/Down 分层。
`b_limits` 配置 `min_m: -0.01`、`split_m: 0.12`、`max_m: 0.30`，目标在未补偿 Base Z 下进行分类：
`[-0.01, 0.12)` 为 Down 层，`[0.12, 0.30]` 为 Up 层，越界目标跳过且不发运动/Store 指令。
同组 `max_distance_m: 1.0` 限制未补偿基座坐标的三维距离 `sqrt(x²+y²+z²) <= 1.0m`，包含边界。
B 沿用 8 帧观察预热及单目标证据门槛，异类或未确认候选不否定其他已确认目标。
第 8 帧结束判断，无合格候选则返回空结果，不延长窗口等待第二个目标。
Mission 等待感知的 `targets_ready`，不再用 2.5 秒空帧提前退出代替 B 的 8 帧判断。缺失结果仍受识别超时限制。
先排除不匹配、不成熟及空间越界目标，再按检测置信度中位数降序选最多两个；同分保留原顺序。
B 放果预设按抓取时的分层动态选用 `<side>_store_b_up` 或 `<side>_store_b_down`。
这些参数只影响视觉目标抓取，不覆盖任何 preset。
相机到基座的换算仍使用实测观察姿态。修改参数后重启 mission 或调试客户端。预设重载仅更新 presets，不改变客户端配置。

## 比赛和单区调试共用同一套函数

节点启动时从明确的入口表选择一次函数；`main()` 没有模式分支，不做动态导入或流程图引擎。
`mission_flow` / `--flow` 保留现有名字，GUI 和启动脚本不需要换一套用法：

| 选择 | Logic.py 入口 |
| --- | --- |
| `full`（默认） | `main` |
| `a` | `debug_a`：`run_a` 后到既有 A 出口点 |
| `b` | `debug_b`：真实扫 B 码 → `run_b` → 既有 B 调试出口 |
| `c` | `debug_c`：真实扫 C 码 → `run_c` |
| `ac` | `run_ac`：保留原 AC 入口，复用同一套动作 |

起始播报、初始一次安全 Home、结束关闭感知和结束播报由节点生命周期统一处理。
仅 `full` 在 C 区任务结束后，从当前位置沿现有 C 区图规划前往 `C_top_waypoint_1`，
到点后对齐该航点 YAML 保存的 yaw，再播报采摘汇总。优先使用实时 TF，
TF 不可用时沿用既有上一目标位置兜底；保留路线距离限制与导航安全门控。
该收尾沿途异步执行一次 `shenzhi`，不切换 Look，不因导航失败恢复 Home，也不增加 mission 重试。
开环和闭环 C 区收尾均使用同一 top 路段编排，不伪造 Home 或伸直成功状态。
仍保留原有运动提交、反馈、保护姿态门控和上一导航终态检查。
重试由导航层负责；规划或导航最终失败时告警，仍直接播报汇总。
`a`、`b`、`c`、`ac` 的结束位置保持不变；不增加返回起点流程。
启动会命令真实硬件，以下只供人工安全联调，不是自动验证命令：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
./start_fruit_mission.sh --flow b  # B-only 同样要扫码，不直接套固定调试计划
./start_fruit_mission.sh --flow ac                 # 默认 A 开环、C 闭环
./start_fruit_mission.sh --flow ac --open-loop C   # A 闭环、C 开环
./start_fruit_mission.sh --flow ac --open-loop AC  # A/C 都开环
./start_fruit_mission.sh --flow ac --open-loop none  # A/C 都闭环
```

源码修改后需重新构建 `fruit_mission` 并重启节点。配置继续从源码路径读取，
不用修改 install 中的副本。

独立导航和单点调试脚本通过 `stack_readiness_client bridge-start-required` 检查串口桥：
标准输出为 `true`（缺失，需要启动）或 `false`（已存在，复用）。节点、
ArmStatus 或 ready 发布者均可识别已有桥，`ready=false` 不代表缺失；
重复节点/发布者或错误消息类型会让检查失败。检查只读，不命令硬件。
单点客户端的 `--require-stack-absent` 允许已有串口桥，但仍拒绝已有感知容器、
planner 和 mission；复用桥不由当前脚本清理，也不更改其配置。

## 失败不阻断比赛，但绝不授权未知运动

- 普通观察、采摘、二维码或外围节点失败，跳过当前操作，继续 A/B/C 软件流程。
- `pick_a` 返回 `success / reason_code / matched_type`。左侧失败也调用同点右侧，
  B 第一抓失败恢复一次 Home，确认成功后继续第二个冻结目标；恢复失败则跳过剩余目标。
  C 无论本项结果如何都推进下一项，不重新观察或反复抓取。
- 后续导航/Look/抓取仍受既有反馈和运动占用门控；软件推进不是运动许可。
  保留导航取消、STOP、紧急状态锁和运动入口的新鲜实测静止检查，恢复只尝试一次 Home。
- B 区树间 Home 未确认时，调用处不提交下一树导航或对侧 Look，只推进软件任务。
- 机械臂执行无限等待下位机结果，可由主动 STOP/取消或节点退出中断。
  导航每个路点包含转向和直行，最多执行 10 秒；上层不重发，底层重试配置为 0。
  Mission 对整个目标的结果等待预算为路点数 × 10 秒，超时取消并恢复。goal 应答、取消、端点、识别、
  扫码等仍用独立的有限等待，超过 10 秒最多 10 秒，较短的不加长。
- 只用上位机 `fruit_arm_planner` 服务，不发原始轨迹/串口。服务执行完成才算工序成功，
  不把发布、STOP 已发送或语音播报当成硬件完成。

这次是软件编排重构，不是实机采摘验收；串口可靠队列的轨迹装载顺序、STOP 优先级、
真实夹爪和姿态仍须按 planner 文档在安全条件下验证。
