# KISS 项目设计审查

## 结论与范围

**主要问题不是 ROS 包太多，而是重复状态、重复协议和错误的职责归属。**
优先统一运行所有权和结果契约，再合并重复实现，最后拆分大模块。
不要为了减少文件数量，把比赛策略、运动控制和串口传输合成一个节点。

本次检查覆盖自研模块的主要调用链、启动入口、接口定义和依赖清单。
重点包括 mission、感知、QR、机械臂 planner、导航、RobViz 和串口桥。
没有逐行审计供应商驱动、Point-LIO 算法、前端构建产物和模型文件。
没有构建、启动节点或操作硬件，以下是静态设计结论，不是实机验收结果。

审查从 `efe3f40` 开始，复核了期间新增的 `18cb078`。
工作区已有操作者改动，串口桥独立仓库也有未提交改动。
结论以读取时的工作区内容为准，不把父仓库提交号当作完整部署版本。
本次只新增本文，不修改实现、标定或预设。

## 1. 优先处理：职责边界已经影响正确性

### 1.1 感知 launch 不应该拥有串口桥

证据：[fruit_perception.launch.py](../src/fruit_perception/launch/fruit_perception.launch.py)、
[start_fruit_mission.sh](../start_fruit_mission.sh)。

- 感知 launch 同时启动相机、推理、QR、串口桥和 RobViz 图像节点。
- 正式任务把串口桥放在 `perception` 子进程组里。
- 退出顺序先停止 `perception`，之后才停止 planner 和 navigation。
- 入口确实提前请求了一次 STOP，但这不等于硬件停稳确认。

这使“停止相机”和“关闭底盘、机械臂通信”成为同一个生命周期动作。
日志也把串口通信归入 `perception.log`，增加故障定位成本。

**最小建议：**由整机入口单独启动串口桥。
感知 launch 保留相机、推理和 QR，预览由入口按需组合。
关闭时先结束运动客户端，再处理控制节点，最后关闭通信。
保留现有停止、取消和反馈确认，不能只调整进程顺序就宣称安全。

### 1.2 运行所有权重复，启动依赖被误用为停止所有权

证据：[runtime_manager.py](../src/fruit_robviz_bridge/fruit_robviz_bridge/runtime_manager.py)、
[runtime_tasks.yaml](../src/fruit_robviz_bridge/config/runtime_tasks.yaml)。

Shell 入口负责子进程组、清理和硬件栈所有权。
`RuntimeManager` 又维护依赖、进程识别、就绪判断和停止流程。
二者对“存在”“就绪”“由我启动”没有统一契约。
例如，匹配启动脚本进程就可返回 ready，并不代表内部 ROS 服务可用。

更具体的问题：`stop('arm_planner_node')` 先停止依赖 `serial_controller`。
随后才执行 planner 的 STOP 命令和进程停止。
该路径不同于 `stop('arm')` 的组合任务顺序，不能笼统说所有 arm 停止路径都相同。
复用的串口桥也不应因为是依赖，就自动成为当前任务的停止对象。

**最小建议：**统一“任务拥有的进程”和“任务依赖的服务”。
让运行 API 调用同一套入口及停止流程，不再另行推导硬件退出顺序。
复用现有运行 helper，不新建通用依赖图引擎或第三套 supervisor。

### 1.3 控制 API 的访问边界不足

证据：[runtime_server.py](../src/fruit_robviz_bridge/fruit_robviz_bridge/runtime_server.py)。

运行 API 默认监听 `0.0.0.0:8001`，能够启动真实硬件任务。
`do_POST()` 没有服务端身份验证或来源拒绝检查。
`_cors_headers()` 仅决定响应头，不能阻止服务器执行请求。
命令白名单限制可执行命令，但不限制谁可以启动机器人。
实际网络可达范围还取决于机器防火墙，本次没有验证。

**最小建议：**明确可信访问范围，并在服务端执行访问控制。
保留命令白名单，按部署要求限制监听地址、网络入口或验证调用身份。
不要把运行控制服务并入普通预览服务来减少端口。

### 1.4 正式任务和调试工具的恢复契约已经分叉

证据：[MissionContext.recover_home](../src/fruit_mission/fruit_mission/mission_context.py)、
[CameraArmTargetClient.safe_recover_home](../src/fruit_mission/fruit_mission/camera_arm_target_client.py)。

- 正式任务取消导航，必要时 STOP，确认反馈后只尝试一次 Home。
- 调试工具执行 STOP、等待 IDLE、`zero`、`home`。
- 调试工具还固定使用 `0.05` 的低速阈值。
- 正式任务从 mission 配置读取低速阈值。

这不只是代码重复，同名“安全恢复”代表不同的物理动作。
调试成功不能证明正式恢复路径成功。

**最小建议：**先让操作者确认 `zero` 是否有真实机构需求。
再把恢复动作和反馈确认收敛到共享客户端，调用方只补充导航取消。
不能擅自删掉 `zero`，也不能把额外恢复动作复制到正式任务。

## 2. 可以合并：合并相同能力，不合并不同策略

| 位置 | 已确认的重复 | 最小合并方向 |
| --- | --- | --- |
| `mission_context.py` | `wait_camera_target` 与 `wait_camera_decision` 重复空视野超时、分类早退和结果归因 | 保留两个入口，共用分类判断和等待核心，明确是否要求三维目标 |
| `mission_context.py` | Look 与通用 preset 各有 pending、start、finish、failure 路径 | Look 作为具名 preset，复用提交和完成处理，保留侧向一致性校验 |
| mission 与相机调试客户端 | 各自构造窗口 JSON、保存窗口上下文和核对窗后帧 | 共用小型窗口客户端，不把正式比赛与人工调试流程合成一个类 |
| `camera_target_selector.cpp` 与 `b_target_selector.cpp` | 重复距离、中位数、滑动观测关联、类型投票和离群统计 | 提取共同数学及关联代码，保留单目标锁定和 B 多目标冻结策略 |
| `waypoint_recorder_node.py` | 下一段导航与指定路线导航重复 goal 提交、状态初始化、回调注册 | 路线构造各自保留，共用一个 goal 提交函数 |
| `robviz_web_server.py` | template 和普通航点保存重复备份、临时文件、fsync、replace | 校验和序列化各自保留，共用可靠文件写入步骤 |

对应实现：
[mission_context.py](../src/fruit_mission/fruit_mission/mission_context.py)、
[camera_arm_target_client.py](../src/fruit_mission/fruit_mission/camera_arm_target_client.py)、
[camera_target_selector.cpp](../src/fruit_perception/src/camera_target_selector.cpp)、
[b_target_selector.cpp](../src/fruit_perception/src/b_target_selector.cpp)、
[waypoint_recorder_node.py](../src/fruit_robviz_bridge/fruit_robviz_bridge/waypoint_recorder_node.py)、
[robviz_web_server.py](../src/fruit_robviz_bridge/fruit_robviz_bridge/robviz_web_server.py)。

不要为这些合并引入基类、策略插件或模板框架。
合并后的代码如果需要大量区域分支，说明合并范围过大。
备份、版本冲突检查、文件锁和原子写入不是可删除的“臃肿代码”。

## 3. 应当拆分：同一模块里放了不同变化原因

### 3.1 `MissionContext` 是最明显的大杂烩

[mission_context.py](../src/fruit_mission/fruit_mission/mission_context.py) 约 1405 行。
它同时负责导航 Action、TF、路线、窗口、识别等待、QR 校验、机械臂交接、语音和文件记录。
问题不是行数本身，而是修改识别策略也必须进入运动交接模块。
`actions.py` 直接读取 context 私有字段，context 又读取 planner client 私有字段。
这些访问说明现有边界没有封装完整的能力。

**最小建议：**先合并重复等待，再抽出感知窗口/结果接收和导航 Action 客户端。
context 保留任务状态和底盘/机械臂交接，语音继续调用已有 speaker。
不要拆成十几个 manager，也不要把安全门控分散到多个状态机。
`Logic_abc.py / Logic_acb.py → actions.py` 的主流程结构已经清楚，应当保留。

### 3.2 图像配置客户端混入了基座坐标预览

[robviz_config_client.py](../src/fruit_robviz_bridge/scripts/robviz_config_client.py) 同时承担：

- HTTP 图像设置到 ROS 参数的转换和重载。
- 相机检测到机械臂基座坐标的转换。
- arm/status/odom 历史缓存和停稳判断。
- planner、mission、navigation、recorder 四份 YAML 的读取。

构造函数直接调用 `_init_base_preview()`。
预览配置异常会阻止图像设置客户端启动，即使配置重载本身不需要这些数据。

**最小建议：**把基座预览职责移到独立模块。
是否独立成节点，取决于是否需要独立启停和失败隔离。
仅拆文件可以改善依赖，但不能自动实现运行时失败隔离。
不要新建一套坐标算法，继续复用已有纯数学函数。

### 3.3 ROS 录点器反向依赖 HTTP 服务模块

`waypoint_recorder_node.py` 从 `robviz_web_server.py` 导入航点读写和版本函数。
`robviz_config_client.py` 也从该 HTTP 模块导入图像设置校验。
复用是正确的，但共享能力放错了位置。

**最小建议：**抽出航点文件读写和图像设置校验两个普通模块。
HTTP handler 和 ROS 节点共同调用它们。
暂时不需要拆分 ROS package，也不需要“通用配置中心”。

### 3.4 感知节点应简化内部结构，不应切碎图像处理链

[fruit_perception_node.cpp](../src/fruit_perception/src/fruit_perception_node.cpp) 约 1041 行。
节点包含模型装载、异步推理、窗口处理、候选组装、发布、标注和统计。
它还用 `streams_`、`subscriptions_` 表达当前唯一的 `camera`。
两个模型的 request slot 有实际用途，多流集合目前只有一个调用来源。

**最小建议：**收敛单相机状态，按需要抽出模型执行和结果组装函数。
保留相机与 C++ raw-image 消费者的同容器进程内通信。
不要为了“解耦”，把预处理、测深和标注拆成跨进程图像节点。

## 4. 接口和配置：有重复的事实来源

### 4.1 错误代码和中文文案混在一起

[CameraTargetPlannerClient.finish_motion](../src/fruit_mission/fruit_mission/camera_target_planner_client.py)
有时返回 `arm_pose_unavailable`，有时返回中文解释或翻译后的 planner message。
这些值随后被存入 `reason_code`。
[actions.py](../src/fruit_mission/fruit_mission/actions.py) 却用英文代码作精确字典匹配。
例如通信未就绪的中文返回值不会命中 `arm_endpoints_not_ready` 分支。

**最小建议：**返回稳定的 `reason_code`，另附 `message` 和必要的 `trajectory_id`。
日志与播报在显示边界翻译，业务判断不匹配人类文案。
先修共享客户端结果契约，不在每个调用方增加字符串补丁。

### 4.2 观察窗口是跨语言接口，却藏在 String JSON 里

mission、调试客户端、QR Python 和感知 C++ 各自处理窗口字段。
感知 C++ 甚至使用 OpenCV `FileStorage` 解析控制 JSON。
因此同一协议需要多份类型、时间和区域检查。

**最小建议：**短期共用 Python 窗口处理，保留所有边界校验。
若继续维护跨语言协议，可把窗口定义为现有 `fruit_interfaces` 中的正式 msg。
这属于接口迁移，需要同时重建消费者，不适合夹在普通清理提交里。
即使改成 msg，也必须保留窗口 ID、用途、时间和区域的语义校验。

另外，期望品种同时存在于窗口 `expected_type` 和独立锁存 topic。
当前 harvest 使用窗口字段，窗外不生成抓取目标。
独立 topic 仍会触发选择器 generation 更新，增加隐藏状态变化。
核对外部消费者后，可只保留窗口内的期望品种。

### 4.3 磁盘配置与运行配置不是同一个事实

共享相机客户端从 planner YAML 读取限位、新鲜度和 dwell。
RobViz 基座预览也直接读取 planner YAML。
planner 已有只读 [get_configuration](../src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md) 服务，返回实际内存配置。
磁盘改动未重启时，客户端可能拿到未应用的配置。

**最小建议：**连接运行节点的客户端复用运行配置快照。
离线编辑和预览继续读取磁盘，并明确显示它是待应用配置。
无需把所有包的配置合并成一份 YAML。
`camera_extrinsic.yaml` 还包含抓取 pitch、高度分层和服务配置，应明确区分标定与抓取参数。
物理标定和区域策略可以分组，不必为每组新增文件。

## 5. 可删除候选：先确认没有外部使用

| 候选 | 静态证据 | 删除边界 |
| --- | --- | --- |
| 导航 `progress_timeout_sec`、`progress_min_delta` | 当前服务器只声明、读取和校验，没有用于进度判断 | 删除无效选项及误导说明，不借此新增看门狗 |
| planner `mode_start_delay_ms` 和注释中的启动 Mode 代码 | 当前发布 Header/Points，启动 Mode 被注释；无用配置项已清除 | 先确认固件自动启动契约，不擅自改通信时序 |
| `waypoint_route.py` 的 `ordered_names_to`、`a_goal_names`、QR 名称属性等旧便利 API | 本次工作区搜索只找到定义 | 确认外部脚本没有调用后删除 |
| mission 安装的 C++ 相机变换头文件 | 本次搜索只有定义和文档，实际客户端用 Python | 确认没有外部 C++ 客户端后删除，不新增绑定框架 |
| `FruitTypes.msg` | 仍注册生成，本次未找到类型消费者 | 确认外部订阅者和录包要求后删除 |
| QR 每项 `stages`、计划 `scan_index` | 解析器输出，正式流程不按它们执行 | 若只是历史状态机描述，移到文档，不把它发展成流程引擎 |

这些是删除候选，不是已经证明可以无条件删除的公共接口。

## 6. 串口复杂度应解决契约，不应继续向客户端扩散

已读取独立仓库的 `serial_controller.cpp` 和 `reliable_sender.hpp`。
物理 TX 队列是 FIFO，可靠队列却按消息 ID 分别等待 ACK。
前者不能证明 Header 和 Points 具有跨类型 ACK 顺序。
STOP 也不能因为是控制模式，就被假定拥有发送优先级。

**建议方向：**先和固件确认完整装载、启动及停止语义。
再在现有传输层解决顺序、停止优先级和断联后的未确认命令处理。
不在 mission 增加 sleep、重发抓取或第二套串口控制。
planner 等待匹配轨迹终态，客户端确认实测停稳，两者不是无意义的重复。

当前总架构文档仍描述 Header → Points → Mode。
当前 planner 实现和包级调试文档描述完整点位后自动启动。
应先确认真实契约，再统一说明，不能靠“删代码”替代验收。

总架构文档仍描述 A 每点先左后右，而当前 `Logic_abc.py` 已采用交替侧向。
文档中的 B 每树 `shenzhi` 也与当前连续树间转场不一致。
建议总架构只维护稳定边界，具体动作顺序集中到两个 Logic 文件和包级说明。

## 7. 不建议合并的边界

- **mission 与 planner：**比赛顺序和运动规划有不同职责。
- **planner 与串口桥：**运动语义和可靠传输不应该混成一个控制器。
- **感知与 mission：**检测稳定性属于感知，二维码匹配和采摘决策属于 mission。
- **Point-LIO、loam_interface 与导航控制器：**估计、坐标适配和速度控制边界合理。
- **A/C 与 B 选择器策略：**单目标锁定与双目标冻结不是同一种输出契约。
- **RobViz 编辑/预览与运行控制服务：**已有进程边界值得保留，控制面需要更严格的访问约束。

QR 包较小，但解码、规则解析和任务执行分开是合理的。
它目前持续跨进程订阅 raw image，即使无窗口也保持订阅。
可先评估按窗口订阅是否满足首帧和时间约束。
如资源测量证明需要，再把解码适配为同容器 C++ component。
不必因此删除 QR 包，也不能未经确认改变图像传输架构。

4-DOF 解析 IK、现有轨迹规划和 `planning_api.py` 复用方向符合 KISS。
无需引入 MoveIt、行为树引擎或插件框架。

## 8. 推荐处理顺序

1. 确认运行 API 访问控制、硬件进程所有权和停止顺序。
2. 确认正式/调试恢复差异及串口实际启动契约。
3. 修正共享客户端的稳定结果代码，统一关键文档。
4. 删除确认无消费者的参数、旧 API 和重复期望品种通道。
5. 合并识别等待、preset 完成处理、录点 goal 提交和可靠文件写入。
6. 抽出窗口客户端、导航客户端、航点文件模块和基座预览模块。
7. 仅在维护收益明确时迁移 typed msg，或调整 QR 图像消费方式。

每项单独验证，不做全项目一次性重构。
算法改动复用已有数学检查，通信改动复用协议检查。
流程和恢复改动用明确的离线执行记录及人工安全联调验证。
不新增源码字符串匹配测试，也不把硬件控制工具当自动测试运行。

**KISS 的目标是减少要同时理解的状态和契约，不是减少文件、服务或安全检查的数量。**
