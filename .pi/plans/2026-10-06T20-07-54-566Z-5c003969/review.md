# RobViz 只读复审

结论：**1 项 P0，8 项 P1。阻塞部署和实车导航验收。**

本轮没有修改业务代码，没有启动 ROS 节点，没有发送运动或保存请求。
除只读 OpenCV 配置解析探针外，没有重跑构建和功能测试。
旧报告已保存为 `review.backup-20261008T072200Z.md`，未提交该备份。

## 范围

### 基线、差异及归属

已读取本目录的 `plan.md`、`progress.md`、`baseline.md` 和原 `review.md`。
`baseline.md` 的 diff/status 为空，不能证明启动前工作树干净。
本次环境没有 `workflow_git` 和 `workflow_document` 工具。
使用只读 Git 命令核对实际状态、差异和提交历史。
报告通过文件编辑工具写入，不能通过 `workflow_document ready=true` 触发扩展审批。

| 仓库 | 计划基线 | 当前源码及状态 |
| --- | --- | --- |
| `/home/rc4/fruit_ws` | `main`，`3e37476857bb12cf19f738b6e90d2a65adaa8d59` | 审查开始时 HEAD 未变。Realsense 子模块脏状态及两个未跟踪计划目录属于已有状态。 |
| `/home/rc4/fruit_ws/.worktrees/robviz-web` | 同上 | `task/robviz-web`，`4b18861dfaefe062b7f46392ac75d228a79d78b6`。六个已跟踪文件有未提交修改。 |
| `/home/rc4/robviz` | `02fdfdc0242f0f5e634f3dd01dd283be3756858d` | `task/waypoints-web`，`6ce930234e4e40ff61a2377283165e11b800a72a`。三个已跟踪文件有修改，另有未跟踪 `tests/unit/waypointsMaker.test.ts`。 |

机器人相对计划基线涉及 33 个文件。前端涉及 12 个已跟踪文件，另有上述测试文件。
审查范围包括功能提交和当前未提交内容，不只检查最后一次补丁。
不把这些既有修改归因于本轮，也不提交或回滚它们。

重点读取：recorder 命令、采样、Action、保存/重载调用链，HTTP API，图像组件、配置客户端、感知 launch。
前端读取 SettingsPage、Editor、Maker、WorkerBridge、MarkerRenderer 和相关测试。
机器人文件均从任务 worktree 读取。生产标定文件按组件实际配置路径读取。

下文机器人相对路径以 `/home/rc4/fruit_ws/.worktrees/robviz-web` 为根。
前端相对路径以 `/home/rc4/robviz` 为根。
便于阅读，使用以下文件简称：

- `node.py`：`src/fruit_waypoint_recorder/fruit_waypoint_recorder/waypoint_recorder_node.py`。
- `server.py`：`src/fruit_waypoint_recorder/fruit_waypoint_recorder/waypoint_editor_server.py`。
- `recorder.py`：`src/fruit_waypoint_recorder/fruit_waypoint_recorder/recorder.py`。
- `image.cpp`：`src/fruit_robviz_bridge/src/image_bridge_component.cpp`。
- `config_client.py`：`src/fruit_robviz_bridge/scripts/robviz_config_client.py`。

### 保留的用户决策

- RGB 与 Native Depth 均使用 JPEG `CompressedImage`，不使用 WebRTC 或跨进程 raw-image 消费者。
- RGB 显示框、类型、置信度、相机 D/Cam XYZ 和实测 Arm Base XYZ。
- Native Depth 使用 JET，仅用于观察，不替代比赛数值测距。
- RGB 默认 640×360、10 FPS、1.5 Mbps。Depth 默认 424×240、10 FPS、0.8 Mbps。
- 两路独立限速，仅计算 JPEG data。预算必须为有限正数，0 不表示暂停。
- RobViz 拥有图像默认配置。Web 触发 ROS 重载，ROS 主动 GET，不后台轮询。
- 首次应用前不推流。HTTP 成功不等于 ROS 生效，错误回应不得显示成功。
- Base 使用采集时连续实测停稳窗口。复用现有 Python 变换，失效时显示 N/A。
- 通用 Editor 与项目 Maker 分开。Maker 复用 recorder 和既有逐段路线逻辑。
- 正式文件按模板，调试文件自由命名。文件 ID 使用 allowlist。
- 保存保留版本检查、锁、备份、扩展字段、fsync 和原子替换。
- 丢弃未保存修改必须确认。Editor 草稿、磁盘和 recorder 内存必须明确区分。
- Action 成功才推进游标。未知运动不能放行下一段，取消成功不等于物理停稳。
- 浏览器断开不取消已接受导航，重连不重发旧命令。
- 保留 RViz 源码，仅移除日常入口依赖。8000 托管静态产物，不使用 Vite 生产服务。
- 保留独立公开仓库 `Edmounds/robviz-fruit` 的已确认替代方案，不擅自公开机器人仓库。
- 不修改比赛策略、机械臂控制合同、标定或串口协议，不新增依赖。

## P0

### Q1：未知 Action 状态或取消前旧反馈能够解除导航所有权

**优先级：P0，运动安全。**

**位置：** `node.py:962-998`，`_on_test_result()`、`_finish_test_goal()`。
停稳路径为 `node.py:599-624`、`1045-1070`，`_is_standstill()`、`_start_standstill_wait()`、`_poll_standstill()`。

**触发条件：** 已接受 goal 的结果 future 抛异常，或取消终态后没有新的里程计观测。

**证据：** 结果异常直接进入 `_finish_test_goal()`，清除 active 和 goal handle。
异常分支没有确认 Action 终态，也没有实测停稳证明。
`_is_standstill()` 使用 `now - effective_start`，不是最后观测时刻减去窗口起点。
设取消时刻为 0，最后低速观测为 -0.01，此前已有停稳窗口。
在 0.31 秒检查时，旧观测年龄 0.32 秒，小于默认 0.5 秒，函数返回 True。
取消之后无需收到任何新反馈。
`_run_next_test_segment()` 没有独立停稳准入，可以在 active 清除后派发下一段。

**影响：** 运动状态未知时显示 idle，丢失当前 goal 所有权，并允许继续操作。
取消前低速观测不能证明取消之后的物理停稳。

**修复：** 结果异常保留未知状态和 goal 所有权，保留显式取消能力。
以有效观测终点计算窗口，取消后必须收到覆盖确认窗口的新反馈。
不得用当前时间补足观测窗口，不得把结果异常解释成安全终态。

## P1

### Q2：录点长时间持锁，默认采样必然让反馈过期

**优先级：P1，核心录点路径。**

**位置：** `node.py:299-306`、`378-450`、`465-528`、`568-624`，命令、采样和里程计调用链。

**触发条件：** 使用默认 1.0 秒采样和 0.5 秒 odometry 新鲜度执行 record。

**证据：** `_on_command()` 持有 RLock，同步执行整个 `_sample_current_pose()` 循环。
`_on_odometry()` 更新缓存也需要同一锁。
另一个 executor 线程无法更新反馈，同线程重入不解决这个问题。
录点循环约 0.5 秒后因旧 odometry 过期而退出，无法完成默认连续采样。
两线程 executor 还可能同时被采样和等待锁的 odometry 回调占用。

**影响：** 机器人实际持续发布正常反馈时，Web 录点仍失败。

**修复：** 短时持锁取得采样所有权，释放锁后采样，再短时持锁提交结果。
保留 busy 原子准入，不扩大新鲜度或缩短采样来掩盖锁问题。

### Q3：Base 使用 latest 姿态和处理时刻，不符合图像采集时语义

**优先级：P1，核心坐标显示。**

**位置：** `image.cpp:205-217`、`324-392`、`696-842`，`fresh_for_image()` 和 `base_point_for()`。
来源为 `src/fruit_perception/src/fruit_perception_node.cpp::finish_inference()`。

**触发条件：** RGB 推理存在正常延迟，反馈继续更新，或停稳窗口在采集之后才满足。

**证据：** RGB 保留采集 header，但推理结束后才发布标注图。
组件只保存 latest 遥测和 latest odometry，却拒绝晚于 image stamp 的样本。
例如采集时刻 100.0，处理时刻 100.2，latest 反馈为 100.18，无法匹配采集时姿态。
窗口使用 `Clock::now() - standstill_start`，不是 image stamp 对应的已观测窗口。
停稳后 0.28 秒采集、0.35 秒处理的图像，可能被当作满足 0.3 秒窗口。
`base_point_for()` 未检查当前 odometry 接收年龄，并接受空 `child_frame_id`。
坐标转换还在 C++ 重写，未复用已确认的 Python 纯数学实现。

**影响：** 正常异步推理下 Base 不可用，或对采集时未满足条件的图像显示有效坐标。

**修复：** 按 S3 使用现有 Python 变换生成同戳 Base 元数据。
从有界反馈历史选择采集时有效样本，证明此前连续窗口，并检查 frame 和新鲜度。
缺少证据时显示 N/A，不插值运动姿态，不扩大时延阈值。

### Q4：航点内容与版本分开读取，版本保护可以被绕过

**优先级：P1，文档保存及路线同步。**

**位置：** `server.py:237-246`、`401-466`、`877-896`。
recorder 位于 `node.py:190-198`、`778-795`、`815-835`。

**触发条件：** 一个客户端加载文件，另一个客户端同时原子保存该文件。

**证据：** HTTP 先解析内容，再独立打开文件计算 version，未取得覆盖整个读取的文档锁。
recorder 也先 load，再独立计算 version。
交错顺序为：A 读旧内容，B 保存新内容，A 随后读到 B 的新 hash。
A 的旧草稿携带新 hash，保存时通过版本检查。
recorder 的旧路线同样可能携带新版本，绕过派发前磁盘比较。

**影响：** 覆盖其他客户端的新修改，或运行与磁盘不一致的旧路线。
保存备份仍存在，但版本冲突保护失效。

**修复：** 从同一字节快照解析内容并计算 hash，或用共用文档锁覆盖整个读取。
HTTP、recorder 初始化、切换和重载共用该合同。

### Q5：配置确认不是精确版本比较，并发重载可以超时或回退

**优先级：P1，带宽控制关键路径。**

**位置：** 前端 `src/app/SettingsPage.tsx:93-111`，`reloadRos()`。
`config_client.py:48-59`、`132-164`。`image.cpp:457-494`，`on_parameters()`。

**触发条件：** 页面等待 version 2，服务返回 version 20，或两个客户端同时重载。

**证据：** 前端使用 `message.includes('version ' + version)`，20 会匹配 2。
重载服务为 Reentrant，两线程都可以阻塞于 `done.wait()`。
参数响应回调依赖同一个 executor，两个并发请求可使响应无法及时调度。
组件接受任意正 settings_version，不拒绝旧版本回写。
旧 GET 请求晚派发时能够覆盖已应用的新配置。

**影响：** 页面虚报配置生效，正常多客户端操作超时，或带宽设置倒退。
超时也不能证明远端没有应用设置。

**修复：** 精确解析并比较版本，增加最小并发准入，不阻塞参数响应线程。
拒绝同一配置会话中的旧版本回写，并明确 HTTP 重启后的版本语义。
超时显示未确认，不宣称旧配置必然未变。

### Q6：MarkerArray 没有当前快照，晚连接页面缺少航点和名称

**优先级：P1，核心 Web 可视化。**

**位置：** `node.py:249-253`、`1081-1092`，marker publisher 和 `_publish_state()`。
前端 `src/renderers/MarkerRenderer.ts::update()`。

**触发条件：** recorder 启动后才打开浏览器，或重连、重建 3D 面板。

**证据：** state/prompt 使用 TRANSIENT_LOCAL，MarkerArray 使用普通 depth=1，即 VOLATILE。
marker 只随 `_publish_state()` 发布，没有周期快照或新订阅者重发路径。
MarkerRenderer 依赖收到的 MarkerArray，不从 String state 重建场景。

**影响：** 无额外操作时，已有点和名称不能自动进入新 3D 场景。
这违反 S6 的新订阅者获取完整点位要求。

**修复：** 让晚订阅者取得完整 MarkerArray，优先使用兼容桥接订阅的锁存 QoS。
若桥接不支持，提供最小快照重发，不要求用户执行修改或导航命令。

### Q7：按需和 FPS 门控过晚，关闭预览仍触发上游绘图

**优先级：P1，明确的高效预览要求。**

**位置：** `image.cpp:307-317`、`536-579`、`626-693`、`848-874`。
上游为 `src/fruit_perception/src/fruit_perception_node.cpp:850-865`。

**触发条件：** 全部压缩图订阅者退出，首次配置未成功，或 raw 输入频率高于预览 FPS。

**证据：** raw annotated/depth 订阅永久存在，没有取消路径。
上游因 annotated raw 订阅仍 clone、draw_annotations 并发布。
存在压缩订阅者时，转换、缩放、JET、文字先执行，settings/FPS 检查最后才执行。

**影响：** 关闭面板不能停止额外上游工作，降低 FPS 不能限制前置图像处理负载。
本轮没有测量比赛吞吐，不声称已发生吞吐退化。

**修复：** 按实际订阅需求管理不必要 raw 订阅，退出时清理缓存。
把首次配置和 FPS 准入放在昂贵工作之前。
保留同容器、有界负载和 latest-wins，不降低比赛推理频率。

### Q8：重载确认使用 truthiness，加载失败也会丢失旧内存

**优先级：P1，未保存录点数据保护。**

**位置：** `node.py:778-795`、`815-835`，文件切换和重载。
`recorder.py:278-337`，`TestPointStore.load()`。
前端 `src/plugins/waypoints-maker/index.tsx::loadFile()/reloadFile()`。

**触发条件：** 有未保存修改，收到 `confirm_discard: "false"`。
或文件 pose 格式有效，但名称/类型不符合正式模板。

**证据：** reload 未验证 force/confirm_discard 为 bool，字符串 `"false"` 为真。
切换文件没有后端 dirty 丢弃确认。
store.load 先替换 points、cursor 和 undo，之后才验证正式模板。
切换还先更新 file_id/formal。验证失败仅报告错误，不恢复旧内存。

**影响：** 未确认命令或被拒绝的加载能够丢失未保存录点和撤销历史。

**修复：** 严格校验确认布尔值，切换同样要求后端确认。
先在临时存储完成全部解析和校验，再一次性替换当前状态。
失败保留文件 ID、点、版本、游标、dirty 和 undo。

### Q9：OpenCV FileStorage 无法读取实际 YAML，双图像组件无法初始化

**优先级：P1，双图像启动关键路径。**

**位置：** `image.cpp:108-114`、`162-168`、`234-243`。
生产接入为 `src/fruit_perception/launch/fruit_perception.launch.py` 的 ImageBridgeComponent。
实际配置路径由 `src/fruit_robviz_bridge/config/robviz_bridge.yaml` 指定。

**触发条件：** 组件读取现有 `camera_extrinsic.yaml` 和 `arm_presets.yaml`。

**证据：** 构造函数无条件使用 `cv::FileStorage(..., READ)` 读取两个普通 YAML。
本轮在验证过的系统 OpenCV 4.5.4 环境运行只读探针。
两个实际配置文件均抛出同一 native persistence 错误：

```text
OpenCV(4.5.4) ./modules/core/src/persistence.cpp:682:
(-5:Bad argument) Input file is invalid in function 'open'
```

探针日志：`/tmp/pi-unified-exec-4-afc77e91.log`。
Python 绑定调用同一 FileStorage 读取实现。本轮没有加载真实组件。
构造函数没有捕获并替代此读取失败，因此源码启动路径被阻塞。

**影响：** 组件初始化失败，两路 JPEG 预览无法提供。
构建成功不能发现配置加载错误。

**修复：** 与 Q3 一起按 S3 使用标准 YAML 读取和现有 Python 变换。
向组件传递已校验、只读的限位或同戳元数据，避免用 OpenCV persistence 读取普通 YAML。
不得为预览修改全局标定格式，不增加第二套默认标定或变换算法。

## 修复步骤

**以下仅为待审批步骤，本轮未执行。只修 Q1 至 Q9。**

1. 核对两个任务目录、最新 main、index、未跟踪文件及重叠修改。
   保留当前修复，不自动 stash、回滚或提交其他人的修改。
   使用独立 build/install/log，避免 worktree 与部署源码共用 CMake cache。
2. 修改 `node.py`，先修 Q1/Q2。
   采样长循环移出锁，保留采样所有权和原子 busy 准入。
   结果异常保留未知运动状态，停稳窗口只使用实际反馈覆盖时间。
   先完成下面的无执行器采样和取消验收，再进行其他导航改动。
3. 修改 `server.py`、`recorder.py`、`node.py` 和 Maker，修 Q4/Q8。
   共用内容/hash 快照读取，先完整校验再替换内存。
   严格校验丢弃确认，Maker 切换命令传递真实布尔确认。
   在 tmp 文档副本进行冲突及失败回滚验收，不写生产航点。
4. 修改 `config_client.py`、`image.cpp` 及必要接线，联合修 Q3/Q9。
   按已批准 S3 复用 YAML 读取和坐标函数。
   使用采集时有界反馈窗口，生成同戳 Base 元数据。
   先验收实际配置可加载，再验收延迟图像和 N/A 条件。
5. 修改 SettingsPage、`config_client.py` 和 `image.cpp`，修 Q5。
   精确版本确认，拒绝忙时并发，防止旧配置覆盖新配置。
   明确服务重启版本会话和超时未确认状态。
   双客户端、乱序回应和版本 2/20 场景必须通过。
6. 修改 `node.py` marker QoS 或最小快照重发，修 Q6。
   核对实际 Foxglove durability，验证晚连接和重建面板。
7. 修改 `image.cpp` raw 订阅生命周期和前置准入，修 Q7。
   验证无订阅者、未配置和 FPS 拒绝帧不会执行额外昂贵工作。
   再完成 JPEG 预算、延迟、内存和比赛吞吐对比。
8. 更新 bridge/recorder README 和受影响 Web 文档，记录实际测试及失败证据。
   定向检查全部通过后，分别提交任务拥有的文件。
   未授权前不合并、推送、部署、重启服务或进行实车运动。

## 验收

### 本轮实际证据

- 两个任务 HEAD 未变化，实际 dirty 状态已核对。
- 两个任务目录的 `git diff --check` 通过，仅证明补丁空白检查。
- 只读 FileStorage 探针确认两个实际 YAML 加载失败，见 Q9。
- 未运行 compileall、colcon、npm、HTTP 保存、录点或导航验收。
- 未修改、提交或部署任务业务代码。

### 保留的历史证据及限制

| 历史记录 | 结果 | 限制 |
| --- | --- | --- |
| progress.md 中 frontend lint/build | 记录通过，保留大 chunk 警告 | 本轮未重跑，不覆盖生产浏览器或 ROS 并发。 |
| frontend tests | 31 文件、161 用例通过 | 新 Maker 用例只检查 JSON 解析，不覆盖真实录点或导航。 |
| bridge/recorder 定向 build | 记录两个包成功 | 不证明组件配置加载或实际图像通路。 |
| compileall/flake8/uncrustify/ABI | 记录通过，ABI 为系统 OpenCV | 不覆盖采样锁、时间窗口和实际部署。 |
| verify_server_fixes.py | 记录 CSP 和图像配置快照检查通过 | 不覆盖航点并发快照、浏览器和 ROS 重载。 |
| verify_recorder_lifecycle.py | 历史日志 PASS | 旧报告指出使用私有状态及 MagicMock，不覆盖真实 executor。 |
| colcon test-result | 聚合 114 tests，1 skipped | 旧报告核对含历史 XML，不能声称本轮重跑 114 个测试。 |
| packages-up-to 构建 | 旧报告记录 5 个包 source/cache 冲突 | 后续 packages-select 成功不能替代完整受影响链验证。 |

原报告的更详细历史证据保留在 dated backup。
不删除失败记录，不把源码阅读当作测试通过。

### 审批后定向命令

每个 ROS/build/调试终端先执行：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

隔离构建，不能覆盖主 build/install：

```bash
colcon --log-base tmp/robviz-review-log build \
  --base-paths .worktrees/robviz-web/src \
  --build-base tmp/robviz-review-build \
  --install-base tmp/robviz-review-install \
  --packages-select fruit_perception fruit_robviz_bridge fruit_waypoint_recorder fruit_nav_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source tmp/robviz-review-install/local_setup.bash
/usr/bin/python3 -m compileall -q \
  .worktrees/robviz-web/src/fruit_robviz_bridge/scripts \
  .worktrees/robviz-web/src/fruit_waypoint_recorder/fruit_waypoint_recorder
ament_flake8 .worktrees/robviz-web/src/fruit_waypoint_recorder/fruit_waypoint_recorder/waypoint_recorder_node.py
ament_flake8 .worktrees/robviz-web/src/fruit_waypoint_recorder/fruit_waypoint_recorder/waypoint_editor_server.py
ament_flake8 .worktrees/robviz-web/src/fruit_robviz_bridge/scripts/robviz_config_client.py
ament_uncrustify .worktrees/robviz-web/src/fruit_robviz_bridge/src/image_bridge_component.cpp
/usr/bin/python3 -m pytest -q \
  .worktrees/robviz-web/src/fruit_mission/test/test_camera_target_transform.py \
  .worktrees/robviz-web/src/fruit_waypoint_recorder/test/test_recorder.py \
  .worktrees/robviz-web/src/navigation/fruit_navigation/test/test_axis_planner.py
ldd tmp/robviz-review-install/fruit_robviz_bridge/lib/libfruit_robviz_bridge_component.so
ldd tmp/robviz-review-install/fruit_perception/lib/libfruit_perception_component.so
```

期望：受影响包、语法、定向 lint 和已有算法测试通过。
ABI 不出现 `/usr/local/lib/libopencv*`。依赖缺失即停止核对，不绕过 runtime 验证。

```bash
cd /home/rc4/robviz
npm run lint
npm run build
npm test -- --run
```

期望：现有前端检查通过，不把 build 通过当作运行验收。

### 无执行器黑盒验收

实施阶段可在 tmp 建立最小公开端点检查，不连接真实导航或机械臂执行器。
示例入口待编写，本轮没有创建或执行：

```bash
/usr/bin/python3 tmp/check_robviz_review.py --scenario <场景名>
```

| 场景名 | 问题 | 期望结果 |
| --- | --- | --- |
| navigation-unknown | Q1 | accepted 后结果异常保持 blocked，保留取消能力。 |
| cancel-window | Q1 | 仅取消前低速反馈不能解除 busy。终态及新连续反馈满足窗口后才解除。 |
| recorder-sampling | Q2 | 持续 20 Hz odom/TF 完成默认 1 秒采样。移动或断流拒绝录点。 |
| document-snapshot | Q4 | 并发保存时内容与 hash 对应，旧草稿返回冲突，不覆盖新修改。 |
| reload-validation | Q8 | 字符串确认被拒绝。无确认切换和模板校验失败均保留旧内存及 undo。 |
| config-load | Q9 | 实际 YAML 可加载，不改生产标定文件。组件初始化且端点正常。 |
| base-image-time | Q3 | 延迟图像匹配采集时姿态。0.28 秒窗口仍 N/A，断流或空 child frame 无效。 |
| config-concurrency | Q5 | 精确拒绝版本 2/20 错配。并发不饿死响应，旧请求不回退配置。 |
| marker-late-join | Q6 | 晚连接、重连、重建 3D 获得点、名称和朝向，不发送修改命令。 |
| preview-demand | Q7 | 无订阅者、未配置和 FPS 拒绝帧不执行额外昂贵工作。恢复后只处理新帧。 |

保留原延迟接受、旧 run_id、重复命令、取消拒绝、浏览器断开和成功游标验收。
不新增源码扫描测试、深层 ROS mock 或私有状态测试。

### 图像、生产页面及安全验收

- 两路 JPEG、颜色、标签、框和同戳 Base 满足计划 A1/A2。
- 使用实际同容器部署验证，独立节点运行不能替代该证据。
- `check_preview.py` 当前 depth=1 默认 reliable，与输出 SensorDataQoS best-effort 不兼容。
  执行原计划检查前先核对并匹配 QoS，零帧不能作为“组件未发布”的证据。
  这作为验收限制记录，不增加 P0/P1 问题数量。
- 每组测量 60 秒，JPEG 总字节不超过 `Mbps × 1,000,000 / 8 × 61`。
- 保留至少 10 分钟内存和帧年龄检查，不接受旧帧排队。
- 对比预览关闭与双路开启时比赛推理吞吐和 processing_ms。
- 生产 8000 页面验证 CSP、Worker、Foxglove、深浅主题、键盘和窄屏。
- 实车导航必须另行授权，由操作者确认现场空间、控制权和硬件急停。
- 不自动执行机械臂运动，不因浏览器关闭而取消目标。

### 未完成项和交接

**Pending Integration。当前被 Q1 至 Q9 阻塞。**

保留功能提交 `4b18861dfaefe062b7f46392ac75d228a79d78b6` 和 `6ce930234e4e40ff61a2377283165e11b800a72a`。
后续修复仍未提交，不代表经过运行验收。
计划仍标 planning，progress 的“已完成”阶段不替代上述限制及真实验收证据。

仍待完成：修复审批、定向验证、真实图像与遥测、录点、导航取消、生产浏览器和性能验收。
还需核对 debug_waypoints.yaml、RobViz dist、实际 `/etc` unit、dated backup 和进程所有权。
任务分支、worktree、未提交修改和原计划全部保留。
未合并、未推送、未部署，未重启服务。
原 Vite 及临时服务进程状态本轮未核查。

扩展交接数据：`ready=true`，`p0=1`，`p1=8`。
摘要：未知运动放行、录点锁、Base 时序、航点快照、配置并发/版本、晚连接标记、按需预览、重载数据保护及 YAML 启动阻塞。
这些字段仅记录期望交接数据，不代表已经调用不可用的 `workflow_document`。
下一步为恢复该工具并由扩展询问修复审批，不自行进入实现。
