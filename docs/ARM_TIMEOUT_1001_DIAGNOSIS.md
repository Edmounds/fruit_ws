# 机械臂 1001 排查记录

排查日期：2026-10-06。范围：对照运行日志、FT 下位机源码和现有串口桥，进行只读排查与离线通信验证。

## 结论

已复现轨迹发送链路丢点：planner 连续发布轨迹点，但 ROS 发布队列和桥订阅队列的 depth 均为 10。桥未及时消费时，KEEP_LAST 覆盖旧点。串口 ACK 不能恢复这些点，因为它们尚未进入桥的可靠发送队列。

FT 下位机收到跳号点后立即失败，并把点号错误映射成 `1001`。这一机制足以解释本次约 52 ms 内出现的失败，也是当前最有证据的原因。

本次日志没有下位机首次内部错误快照或逐点接收记录，因此尚不能断言轨迹 `88087330` 的唯一原因。没有证据表明此次失败由 120 秒执行期限触发。

## 运行证据

会话：`log/runtime/2026-10-06_22-17-47_camera_arm_target_1853935/`。

`arm_planner.log` 记录：

```text
1791296285.186956323 Dispatching preset(left_store_b_up): traj_id=88087330, points=24, duration=4500ms
1791296285.238666029 Hardware reported failure: task_status=3, error_code=1001
```

两条日志相差 `51.710 ms`。随后上位机发送 STOP，`zero` 和 `home` 均执行成功。

失败前，4、6、9 点的轨迹均成功。恢复时的 14 点轨迹也成功。队列覆盖取决于调度速度，所以点数超过 10 后并非每次都失败。

## 120 秒配置的实际含义

核对源码版本：`tmp/vison`，分支 `FT`，提交 `f77db8bcd687e3e7307447df94c630c7a25451b1`。

`Engineer/APPLICATION/arm/arm_host_trajectory.c` 定义：

```c
#define HOST_ARM_TRAJECTORY_EXECUTION_MARGIN_MS 120000u
```

`HostTrajectoryStart()` 只在收齐轨迹点且硬件 READY 后设置期限：

```text
execution_deadline_tick = now_ms
                        + 最后一点 time_from_start
                        + 120000 ms
                        + 末点夹爪非 HOLD 时的 12000 ms 余量
```

对 4500 ms、末点 HOLD 的轨迹，期限为开始执行后 `124.5 秒`。它不是接收点位的等待时间，也不会推迟输入校验失败或硬件就绪检查失败。

源码确实使用了 120 秒余量。设备实际烧录版本仍需核对。FT 提交时间为 22:24:46，晚于本次 22:17 会话，不能仅凭该提交证明设备当时已运行相同代码。

## 为什么 1001 不一定是超时

`HostTrajectoryPublicErrorCode()` 仅对 RANGE 和 ESTOP 返回 `1002`、`1003`。其他多数内部错误统一返回 `1001`，包括：

- `POINT_ID`：点号跳跃，或重复点内容不一致。
- `TRAJECTORY_ID`：点的轨迹 ID 与当前缓存不一致。
- `TIME`：首点时间不是 0，或时间没有严格递增。
- `NOT_READY`、`NOT_READY_TO_START`：接收或启动状态不符合要求。
- `EXECUTION`：硬件未就绪、参考命令被拒绝，或最终执行期限到达。

内部点号校验要求 `packet->point_id == received_point_count`。期待点 0 时收到点 14，会立即调用 `HostTrajectorySetError(POINT_ID)`，进入 FAILED。

失败后继续收到点，会把内部 `error_code` 改成 `NOT_READY`。因此调试器最终看到的错误可能已覆盖首次错误，应捕获首次 FAILED 的快照。

## 离线复现

验证使用已安装的 `auto_serial_bridge/serial_node`、独立 ROS 域 `221`、命名空间 `/arm_timeout_probe` 和 PTY。每个可靠包都即时回 ACK，心跳正常回包。没有连接机器人串口，也没有发送硬件运动命令。

共运行 72 次传输。两组分别在发布前构造消息，以及像 planner 一样在循环内逐点构造消息。每组包含正常运行和短暂暂停桥进程的场景。

正常运行时的完整传输次数：

| 发送点数 | 发布前构造消息 | 循环内构造消息 |
| --- | --- | --- |
| 4 | 3/3 | 3/3 |
| 9 | 3/3 | 3/3 |
| 14 | 0/3 | 1/3 |
| 24 | 0/3 | 2/3 |
| 27 | 0/3 | 2/3 |
| 64 | 1/3 | 0/3 |

一次 24 点传输的串口结果为 `14..23`。循环内构造消息时，也出现 `2, 6, 7, 10, 11, ..., 23`，其中 Header 在点之前到达。即使串口 CRC 和 ACK 正常，完整点序列也没有到达串口。

暂停桥进程后，24 点传输稳定只保留最后 10 个点，即 `14..23`，符合 KEEP_LAST 深度 10 的覆盖行为。

本地复现脚本与结果保存在 `tmp/arm_timeout_2026-10-06/`，该目录被 Git 忽略。运行方式：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 tmp/arm_timeout_2026-10-06/probe_bridge.py
/usr/bin/python3 tmp/arm_timeout_2026-10-06/probe_bridge.py --inline
```

## 故障发生的位置

```text
planner 生成并校验完整轨迹
  -> 连续 publish 24 个点
  -> ROS KEEP_LAST(depth=10) 覆盖未消费的点
  -> 桥可靠发送队列仅收到剩余点
  -> 串口发送并正常 ACK
  -> 下位机发现 point_id 跳号
  -> FAILED，公开 error_code=1001
```

相关实现：

- `src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py`：`hardware_qos.depth=10`，循环连续发布点。
- `src/auto_serial_bridge/config/protocol.yaml`：`config.qos_depth: 10`。
- 构建生成的 `generated_bindings.hpp`：TrajectoryPoint 订阅使用深度 10，然后调用 `reliable_send()`。
- `src/auto_serial_bridge/include/auto_serial_bridge/reliable_sender.hpp`：可靠重传保护进入该队列的消息。
- FT 的 `Engineer/MODULE/protocol/protocol.c`：业务回调拒绝点后仍回协议 ACK。ACK 不表示点已被轨迹缓存接受。

上位机在记录 Dispatching 前已通过 IK、映射、限位、连续点号和时间校验。FT 的 HOST 执行路径直接使用关节轨迹，不再次做笛卡尔 IK。当前证据优先指向传输链路，不能据此保证所有姿态标定和实际运动均正确。

下位机容量为 64 点，本次 24 点没有超过容量。现有上位机仍允许更多点，后续修复需对齐两端容量。

## 后续修复与确认

1. 优先修复轨迹点 ROS 队列覆盖。两端队列至少容纳下位机支持的一条完整轨迹，并对齐 64 点容量。
2. 重新生成并构建桥，再重复 PTY 验证。确认串口输出连续的 `0..N-1`。
3. 核对 Header 与 Point 的传输顺序。现有可靠队列按消息 ID 分组，不能把发布顺序当成跨消息加载保证。
4. 保留 STOP、ESTOP 和输入校验。增加睡眠或延长执行期限不能恢复已覆盖的点。
5. 请下位机作者捕获首次失败时的 `g_arm_host_trajectory_debug`。至少记录 `state`、`error_code`、`trajectory_id`、`expected_point_count`、`received_point_count`、`execution_start_tick`、`execution_deadline_tick` 和被拒绝的点号。
6. 区分公开故障码的内部原因，避免继续把所有拒绝错误解释成执行超时。

另一个协议差异：FT 拒绝途经点携带 OPEN/CLOSE，返回 `1002`。上位机目前允许该组合。这不是本次 `1001` 的直接解释，但需要两端明确统一语义。

本次仅保存排查结果，未修改业务代码、固件或协议配置，未进行实机运动验证。

## 2026-10-06 修复与离线验证

用户随后授权实施上位机修复。planner 的机械臂硬件命令发布队列调整为 64，桥的 `config.qos_depth` 调整为 64。planner 共用轨迹校验限制单条轨迹为 `2..64` 点，超限时在发布 Header 前拒绝，不截断轨迹。

相关接口、planner 和串口桥共 4 个包重新构建通过。桥生成绑定在运行时公布的轨迹点订阅深度为 64。队列修改不改变业务 payload 或协议哈希。

复用 PTY 验证脚本，使用发布深度 64，再次执行 72 次传输。两种消息构造方式、正常运行与暂停桥场景中的 4、9、14、24、27、64 点轨迹全部完整输出。所有轨迹都满足 Header 先于 Point、点号连续、时间严格递增、CRC 正确和途经点标记正确。

结果文件位于 `tmp/arm_timeout_2026-10-06/probe_results_depth64.json` 和 `probe_results_inline_depth64.json`。原始深度 10 的失败结果仍保留。脚本的修复前版本保存为同目录下的带日期备份。

复测命令：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
/usr/bin/python3 tmp/arm_timeout_2026-10-06/probe_bridge.py --depth 64
/usr/bin/python3 tmp/arm_timeout_2026-10-06/probe_bridge.py --depth 64 --inline
```

现有运行进程需重启才会采用新队列。下位机故障码映射、首次内部错误记录及烧录版本未修改。Header 顺序通过了本次离线验证，但按消息 ID 分组的可靠传输仍不构成通用跨消息顺序保证。未自动调用运动服务或进行实机运动验证。

构建时还确认了版本差异：当前桥生成器分别计算深度 10 和 64 的哈希，结果均为 `0xB063CA5B`，证明 QoS 修改本身不改变哈希。FT 仓库的协议头仍为 `0x3EE64F27`。桥已有的生成器修改未纳入本次提交，当前握手关闭。实机版本核对需同时检查实际固件与桥生成产物，不能仅依据这两个哈希认定接收成功。
