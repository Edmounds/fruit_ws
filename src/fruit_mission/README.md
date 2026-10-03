# fruit_mission：可人工审核的比赛编排

## 从哪里审核

打开 [`fruit_mission/Logic.py`](fruit_mission/Logic.py)：

- `main(ctx)`：整场 **A → 扫 B 码 → 扫 C 码 → C 作业 → B 作业**。
- `run_a(ctx)`：A1、A2、A3、A4 逐点展开；每点先导航并并行启动左 Look，
  然后调用左、右 `pick_a`。导航与采摘是独立积木，不用循环隐藏固定路线。
- `run_b(ctx, plan)`：按二维码树序导航，传入树位置、目标品种、左右侧，
  调用一次 `pick_b_tree` 完成该树工序。
- `run_c(ctx, plan)`：按二维码项序导航，用路线规划器确定的侧调用 `pick_c`；
  导航失败记录并跳过，不在未到达的位置抓取。
- 分区间固定通道点直接列在调用处，不再通过隐藏的 transition 名称展开。

只要改变比赛顺序、路线、左右侧或目标，就编辑这里。航点坐标和车体朝向仍由源码
`src/fruit_waypoint_recorder/waypoints/odom_waypoints.yaml` 提供，使用 `odom`。
A 固定路线要求 A1～A4 都存在且类型为 goal；新增/删减采摘点时同步修改
`Logic.py` 和 `waypoint_route.py` 的必需点校验，不能期待 YAML 自动增加工序。

## 重复动作放在哪里

[`fruit_mission/actions.py`](fruit_mission/actions.py) 是重复工序实现，不是另一套
主流程，也不是只给旧大类套一层壳：

| 积木 | 参数与职责 |
| --- | --- |
| `navigate(ctx, waypoints, side=None)` | 明确导航点；有 side 时导航与 Look 并行。失败清理待消费 Look，安全恢复并返回 False。 |
| `pick_a(ctx, side=...)` | 当前点单侧 Look 完成 → 实测停稳 → 识别成熟目标 → CLOSE/驻留 → Harvest/OPEN/驻留 → Home。无导航、无重复抓取。 |
| `pick_b_tree(ctx, waypoint=..., target=..., side=...)` | 每树观察一次、比较二维码品种、选择最多两个成熟目标，先冻结全部基座请求，再逐个抓取/Harvest/Home。 |
| `navigate_c(ctx, task)` | 复用现有 C 几何规划，记录路线和 camera_side；失败的记录在恢复前由分区流程保存。 |
| `pick_c(ctx, task, side=...)` | 当前二维码项的单次识别/采摘、结果记录和播报；不增加方向复核。 |
| `scan_b_qr / scan_c_qr(ctx, arrived=...)` | 已到点才执行 QR 姿态、停稳和扫码；到点/扫码失败均用 mission 自有固定计划，保存、播报并恢复。 |

`arrived` 使用前一个 `navigate` 的返回值；导航失败已经执行过一次安全恢复，
扫码函数只使用兜底计划，不再为同一导航失败重复尝试 Home。

`MissionContext` 只保留 ROS 反馈、导航发送、感知窗口、planner 原语及安全交接；
品种判断和语音格式在 `decisions.py`，C 路线数学继续在 `waypoint_route.py`。
原单点调试入口也调用相同的 `observe_and_pick` / `pick_b_tree` 积木；B 无可采目标
或任一选中目标工序失败时不返回假成功。它是人工硬件工具，不能作为自动测试运行。
原 `mission_flow.py` 和作为判断帮助文件的 `logic.py` 已移除，避免两个主流程来源。

调用处只显示比赛参数。真实 Look/QR/Harvest/Home 预设名称、超时和停稳阈值仍在
`config/fruit_mission.yaml`；姿态、速度、标定在对应 planner/相机源码 YAML，
不把这些值复制进比赛主流程。A 左右调用顺序现在直接在 `Logic.py`，不再使用
`a_observation_order` 配置。

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
启动会命令真实硬件，以下只供人工安全联调，不是自动验证命令：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
./start_fruit_mission.sh --flow b  # B-only 同样要扫码，不直接套固定调试计划
```

源码修改后需重新构建 `fruit_mission` 并重启节点。配置继续从源码路径读取，
不用修改 install 中的副本。

## 失败不阻断比赛，但绝不授权未知运动

- 普通观察、采摘、二维码或外围节点失败，跳过当前操作，继续 A/B/C 软件流程。
- `pick_a` 返回 `success / reason_code / matched_type / safe_to_continue`。
  左侧普通失败且安全恢复后仍可做右侧；不能确认安全则不做同点右侧。
- B 第一抓普通失败且安全恢复后可做第二抓；不能确认安全则跳过该树剩余目标，
  不重新观察或反复抓取。C 无论本项结果如何都推进下一项。
- 后续导航/Look/抓取仍受既有反馈和运动占用门控；软件推进不是运动许可。
  保留导航取消、STOP、紧急状态锁和新鲜实测静止确认，恢复是一遍安全 Home。
- 各操作独立超时，原来超过 10 秒的等待最多 10 秒，原来更短的不加长。
- 只用上位机 `fruit_arm_planner` 服务，不发原始轨迹/串口。服务执行完成才算工序成功，
  不把发布、STOP 已发送或语音播报当成硬件完成。

这次是软件编排重构，不是实机采摘验收；串口可靠队列的轨迹装载顺序、STOP 优先级、
真实夹爪和姿态仍须按 planner 文档在安全条件下验证。
