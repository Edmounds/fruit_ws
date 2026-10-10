# fruit_mission：可人工审核的比赛编排

## 从哪里审核

打开 [`fruit_mission/Logic.py`](fruit_mission/Logic.py)：

- `main(ctx)`：整场 **A → 扫 B 码 → B1～B8 作业 → 扫 C 码 → 按二维码顺序执行 C 作业**。
- `run_a(ctx)`：A1、A2、A3、A4 逐点展开；每点先导航并并行启动左 Look，
  然后调用左、右 `pick_a`。导航与采摘是独立积木，不用循环隐藏固定路线。
- `run_b(ctx, plan)`：固定按 B1～B8 导航，使用二维码给每棵树指定的目标品种和左右侧，
  调用一次 `pick_b_tree` 完成该树工序。
- `run_c(ctx, plan)`：按二维码项序导航，用路线规划器确定的侧调用 `pick_c`；
  导航失败记录并跳过，不在未到达的位置抓取。
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
| `navigate(ctx, waypoints, side=None)` | 明确导航点；有 side 时导航与 Look 并行。失败清理待消费 Look，安全恢复并返回 False。 |
| `pick_a(ctx, side=...)` | 区域 Look 完成 → 实测静止后 OPEN/HOLD → harvest 窗口 → 成熟目标 → CLOSE/驻留 → 区域 Store/OPEN/驻留 → Home。无导航、无重复抓取。 |
| `pick_b_tree(ctx, tree_name=..., target=..., side=...)` | 每树观察一次、张爪后开启 harvest 窗口比较二维码品种、选择最多两个成熟目标，首次抓取前冻结全部基座请求并关闭窗口，再逐个抓取/区域 Store/Home。 |
| `navigate_c(ctx, task)` | 复用现有 C 几何规划，记录路线和 camera_side；失败的记录在恢复前由分区流程保存。 |
| `pick_c(ctx, task, side=...)` | 当前二维码项的单次识别/采摘、结果记录和播报；不增加方向复核。 |
| `scan_b_qr / scan_c_qr(ctx, arrived=...)` | 已到点且 QR 姿态执行完成后开启 QR 扫码窗口；到点/扫码失败均用 mission 自有固定计划，保存、播报并恢复。B扫码后进入B1～B8，B完成后再前往C扫码点。 |

`arrived` 使用前一个 `navigate` 的返回值；导航失败已经执行过一次安全恢复，
扫码函数只使用兜底计划，不再为同一导航失败重复尝试 Home。

`MissionContext` 只保留 ROS 反馈、导航发送、感知窗口、planner 原语及安全交接；
品种判断和语音格式在 `decisions.py`，C 路线数学继续在 `waypoint_route.py`。
区域切换通过 `/fruit_perception_node/set_parameters_atomically` 设置 `zone` 并核对返回结果。
`/fruit_mission/current_area` 仍发布任务状态，但感知不再订阅。
参数失败只告警并推进流程，不重试。区域不匹配的 harvest 窗口不产生抓取目标。
`start_fruit_mission.sh` 继续直接启动感知 launch，不调用调试入口，并启动或复用
RobViz 预览桥。比赛入口会并行启动导航、感知和 planner；调试录包在感知就绪后后台启动。
任务退出时先停止 mission 和录包，再依次停止感知相机、机械臂 planner 和导航，释放相机订阅后再下电设备。
单点调试入口传递 `--zone`，客户端使用同一参数服务，不再等待区域话题订阅者。
独立单点调试入口 `test_camera_arm_target.sh`（`camera_arm_target_client`）不依赖导航，只接受 `--zone A|B|C --left|--right`（可加 `--debug`），不再接受 `--model`。区域同时选择模型（A/C 蔬菜、B 水果）和 `<side>_look_<zone>` / `<side>_store_<zone>`（B 根据目标基座高度自动选择 `_store_b_up` 或 `_store_b_down`）；例如 `--zone C --left` 使用 `left_look_c`、`left_store_c`。工序为 Home→区域 Look→OPEN/HOLD→识别→CLOSE/HOLD→区域 Store，不加入跨区保护动作。它是人工硬件工具，不能作为自动测试运行。
原 `mission_flow.py` 和作为判断帮助文件的 `logic.py` 已移除，避免两个主流程来源。

调用处只显示比赛参数。超时在 `config/fruit_mission.yaml`；预设名称直接遵循
planner YAML，姿态、速度、标定在对应 planner/相机源码 YAML，
不把这些值复制进比赛主流程。开窗前及窗口内的底盘与机械臂静止由现场物理保证，
移除了原开窗前的 `stable_stop` 阻塞等待；未知运动结果时保留 STOP 与取消。
恢复在 STOP 确认后等待新鲜 `STATUS_IDLE`，Home 完成后等待新鲜低速反馈；Home/QR 等预设入口继续检查当前新鲜有效的低速反馈，
不满足即跳过，planner 仍保留运动占用与 STOP 后 IDLE 保护。A 左右调用顺序直接在 `Logic.py`，不再使用
`a_observation_order` 配置。

窗口结果在入缓存前核对 ID、用途及合法窗后图像时间；扫码计划另核对区域，非对象 JSON 与非法时间忽略后继续等待或使用既有固定兜底。
姿态统一遵循 planner 的 `arm_presets.yaml` 名称定义，无中间映射：观察调用 `left_look_a/b/c` 或 `right_look_a/b/c`，放果调用 `left_store_a/b/c` 或 `right_store_a/b/c`，扫码直接调用 `qr_scan`，复归调用 `home`。Look 完成后复用共享客户端等待新鲜静止反馈，执行 OPEN/HOLD 后才开窗；张爪失败走既有一次 Home，不再即时检查旧 RUNNING 缓存而提前拒绝。

旧正常 `retract` 边界改用 `shenzhi`：A 每点结束、B/C 扫码后、B 每树结束、C 每项结束，以及对应区域出口；最后一个点/树/项与出口重合时不重复执行。失败操作只沿用已有一次 Home，不追加保护动作。边界仍要求保护位，未确认 `shenzhi` 不放行后续导航，即使恢复 Home 成功也不冒充保护位；软件继续推进。已完成且当前实测静止的 `shenzhi` 可用于导航交接或直接启动并行 Look，不强制先回 Home。

真实姿态及已定义名称以源码 YAML 为准。本任务只引用名字，不覆盖操作者已有坐标，不补空预设或假成功；现场运行前须补齐所用预设。QR 解析按窗口区域直接选择 B/C，无自动 B→C 状态推进；窗口 ID、图像时间和固定兜底保留。
B 树收到 `targets_ready=true` 的空列表表示完成但无果实，立即按空树跳过；仅未收到完成结果才算识别超时。关闭窗口保留开启时上下文。

正式任务和相机单点调试共用 `fruit_mission.yaml` 的末端相对偏置配置：
`target_offset_ac_m`、`target_offset_b_up_m` 与 `target_offset_b_down_m`（米），数值以源码配置为准。
局部 +X 沿夹爪正前方深入，+Y 沿夹爪左侧，+Z 沿夹爪上方。
偏置使用补偿前目标方向 yaw（`atan2(base_y, base_x)`）进行旋转叠加。
A/C 使用最终请求 pitch，B 两层固定使用 0° 作为偏置旋转参考，不随 B 抓取 pitch 改变 XYZ。
不根据 Base Y 自动翻转符号。A/C 保留原数值但物理含义变为局部坐标，非零局部 Y 不保证最终 yaw 一致；
B 采用局部深入与下移。正式任务按当前区域与高度自动选择，调试客户端按 `--zone` 与高度选择，
入口统一验证各配置项三个有限数，不回退、不叠加 planner 工具几何偏移。

正式任务和 `test_camera_arm_target.sh` 共用 `config/camera_extrinsic.yaml` 的抓取与高度参数：
`motion.pitch_deg: 0.0` 用于 A/C，`motion.pitch_b_up_deg: 0.0` 与 `motion.pitch_b_down_deg: -30.0` 用于 B 的 Up/Down 分层。
`b_height` 配置 `min_m: -0.01`、`split_m: 0.12`、`max_m: 0.30`，目标在未补偿 Base Z 下进行分类：
`[-0.01, 0.12)` 为 Down 层，`[0.12, 0.30]` 为 Up 层，越界目标跳过且不发运动/Store 指令。
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
当前全流程在 C 区任务结束后结束；回起点动作等现场验证出最终终点后再加入。
启动会命令真实硬件，以下只供人工安全联调，不是自动验证命令：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
./start_fruit_mission.sh --flow b  # B-only 同样要扫码，不直接套固定调试计划
```

源码修改后需重新构建 `fruit_mission` 并重启节点。配置继续从源码路径读取，
不用修改 install 中的副本。

启动脚本统一通过 `stack_readiness_client bridge-start-required` 检查串口桥：
标准输出为 `true`（缺失，需要启动）或 `false`（已存在，复用）。节点、
ArmStatus 或 ready 发布者均可识别已有桥，`ready=false` 不代表缺失；
重复节点/发布者或错误消息类型会让检查失败。检查只读，不命令硬件。
单点客户端的 `--require-stack-absent` 允许已有串口桥，但仍拒绝已有感知容器、
planner 和 mission；复用桥不由当前脚本清理，也不更改其配置。

## 失败不阻断比赛，但绝不授权未知运动

- 普通观察、采摘、二维码或外围节点失败，跳过当前操作，继续 A/B/C 软件流程。
- `pick_a` 返回 `success / reason_code / matched_type`。左侧失败也调用同点右侧，
  B 第一抓失败也推进下一冻结目标；不跨层传播 `safe_to_continue` 或跳过剩余目标。
  C 无论本项结果如何都推进下一项，不重新观察或反复抓取。
- 后续导航/Look/抓取仍受既有反馈和运动占用门控；软件推进不是运动许可。
  保留导航取消、STOP、紧急状态锁和运动入口的新鲜实测静止检查，恢复只尝试一次 Home。
- 机械臂执行无限等待下位机结果，可由主动 STOP/取消或节点退出中断。
  已接受的导航 goal 同样等待终态，不设推进看门狗；goal 应答、取消、端点、识别、
  扫码等仍用独立的有限等待，超过 10 秒最多 10 秒，较短的不加长。
- 只用上位机 `fruit_arm_planner` 服务，不发原始轨迹/串口。服务执行完成才算工序成功，
  不把发布、STOP 已发送或语音播报当成硬件完成。

这次是软件编排重构，不是实机采摘验收；串口可靠队列的轨迹装载顺序、STOP 优先级、
真实夹爪和姿态仍须按 planner 文档在安全条件下验证。
