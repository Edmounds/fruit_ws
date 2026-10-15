# 单相机常开与观察窗口操作参考

全赛区仅使用单 RealSense D435i 深度相机 `camera`。历史动态开关模式（`CameraModeManager`、`OFF/QR/DETECT` 状态机、RPC 参数服务启停及模式回声等待）已彻底移除。
当前相机硬件视频流（Color、Aligned Depth、RGBD）保持常开，OpenVINO 检测与熟度分类模型常驻内存并持续推理。
抓取证据累积与二维码解码严格由任务时机驱动的**观察窗口**（Observation Window）进行隔离与采纳。

每个新终端先加载项目运行环境：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
```

## 观察窗口契约 (/fruit_mission/observation_window)

感知模型由动态字符串参数 `zone` 选择，仅接受 A/B/C。
A/C 使用蔬菜模型，B 使用水果模型。`/fruit_mission/current_area` 仅记录任务状态，不再切换感知模型。
观察窗口也不切换模型。harvest 窗口的 `area` 必须匹配 `zone`。
切区会关闭旧窗口并丢弃旧证据。参数设置成功后再开新窗口。

```bash
./start_fruit_perception.sh --zone A
# 另一个已加载运行环境的终端：
ros2 param set /fruit_perception_node zone B
```

观察窗口命令以严格 JSON 格式发布在 `/fruit_mission/observation_window`（话题类型为 `std_msgs/msg/String`，QoS 为 Reliable / Volatile / Depth 10）。

### 1. 开启观察窗口 (valid=1)

```bash
# 示例：开启 A 区采摘观察窗口（期望 ripe 成熟目标，截断时间设为当前时间）
ros2 topic pub --once /fruit_mission/observation_window std_msgs/msg/String \
  "{data: '{\"window_id\":\"manual-a-01\",\"valid\":1,\"start_sec\":1000,\"start_nanosec\":0,\"area\":\"A\",\"purpose\":\"harvest\",\"expected_type\":\"ripe\"}'}"

# 示例：开启 B 区扫码窗口
ros2 topic pub --once /fruit_mission/observation_window std_msgs/msg/String \
  "{data: '{\"window_id\":\"manual-b-qr-01\",\"valid\":1,\"start_sec\":1000,\"start_nanosec\":0,\"area\":\"B\",\"purpose\":\"qr\",\"expected_type\":\"\"}'}"
```

字段说明：
- `window_id`：非空字符串，任务层每次开窗生成的唯一 UUID。
- `valid`：严格整数 `1`（开窗）或 `0`（关窗），拒绝布尔值或浮点数。
- `start_sec` / `start_nanosec`：ROS 图像时间戳截止线。凡是曝光时间早于此截断时间的历史帧或旧在途帧均被丢弃，不计入本次证据链。
- `area`：`"A"`、`"B"` 或 `"C"`；观察窗口不接受 D。
- `purpose`：`"harvest"`（累积采摘目标证据）或 `"qr"`（二维码解码）。
- `expected_type`：期望采摘标签（如 `"ripe"`、`"pumpkin_ripe"` 等），空字符串表示不指定。

期望类型只由观察窗口提供。已删除重复 `/fruit_mission/expected_fruit_type` 话题。

### 2. 关闭观察窗口 (valid=0)

```bash
# 关闭指定窗口
ros2 topic pub --once /fruit_mission/observation_window std_msgs/msg/String \
  "{data: '{\"window_id\":\"manual-a-01\",\"valid\":0,\"start_sec\":1000,\"start_nanosec\":0,\"area\":\"A\",\"purpose\":\"harvest\",\"expected_type\":\"ripe\"}'}"
```

关窗命令下发后，感知节点与扫码节点立即终止当前窗口的数据采纳并重置跟踪器候选。
任务客户端先立即撤销本地采纳，再发布关窗命令；关窗保留原 ID、开始时间、区域、用途和期望类型，仅改 `valid=0`。
Mission 和单点客户端在缓存结果前检查当前窗口 ID 及合法窗后图像时间；二维码计划还须与窗口的 qr 用途和 B/C 区域匹配。
二维码 JSON 必须为对象，秒/纳秒必须为整数（拒绝 bool、浮点、NaN、缺失字段），纳秒范围为 0～999999999。

## 持续推理与窗外数据特性

1. **持续推理与按需标注**：即便处于窗口之外，深度相机仍以固定帧率出流，OpenVINO 核显持续进行 2D/3D 检测与标注图生成，供监视器与调试工具观察。
- 水果与蔬菜检测器均可发布 `bad`，包括 `types`、`detections`、预览检测和标注图。`bad` 不做熟度分类，按空间归属否决对应果实。完成后的 `targets` 可含不可采收的 `bad`，不授权坏果摘除。
2. **证据严格隔离**：
   - 处于窗口之外时，`/fruit_perception/decision` 发布的决策消息中 `observation_window_id` 为空（`""`），`has_target=false`，`targets_ready=false`，`targets` 列表为空。**窗外的原始检测绝不授权抓取**。
- 只有有效 harvest 窗口内的窗后新帧才累积证据。A/B/C 共用逐果实投票，完整观察 10 个处理帧，空帧计数，重复 RGB 时间戳不计数。最多票胜出，熟度概率须达到 0.70；平票比较平均检测置信度，不要求过半、最低次数或末帧支持。结果带对应窗口 ID，任务层只接受完成的 `targets_ready/targets`。
- 一次有效关联的 `bad` 在本窗口内保持否决，漏检不解除。A/C 先评分确定主目标，再判断是否可采，不用旁边好果放行固定预设。未在 10 秒内完成观察则跳过，不以历史成熟标签兜底。
3. **二维码到站即扫**：
   - `fruit_qr_planner` 仅在 `purpose="qr"` 的窗口内对窗后帧进行解码，同一窗口仅发布一次确认计划（`confirm_frames=1`，单次等待上限 10 秒）。
   - 输出的 `/fruit/b_pick_plan` 和 `/fruit/c_pick_plan` 计划中携带 `observation_window_id`、`image_stamp_sec`、`image_stamp_nanosec`。
   - B 扫码失败后直接使用任务固定兜底计划并继续执行，不阻断后续 C 扫码窗口开启。
   - 删除旧 B→C→COMPLETED 顺序状态机；只按当前 QR 窗口的 B/C 区域解析，不订阅区域通知来自动推进扫码阶段。C 可以先扫，同一区域可在新窗口再次扫码。

## 任务编排与静止边界

1. **到位即开窗**：正式 mission 中，导航到站后直接执行 `qr_scan` 并等完成再开 QR 窗口；区域 Look（`<side>_look_a/b/c`）完成后，以共享实测静止校验执行 OPEN/HOLD，再开 harvest 窗口。不切摄像头，也不向下位机切换比赛任务状态。
2. **物理静止保证**：依据用户明确保证，移除了开窗前与窗口内的 `stable_stop` 阻塞等待与持续监测配置；由现场物理保证观察到抓取目标冻结期间机构完全静止。
3. **安全恢复保护**：运动结果未知时的 STOP 发布、导航取消、EMERGENCY 锁定、恢复前 STOP 之后的机械臂停稳确认（`is_stationary` <= 0.05 rps）以及一次 Home 复位保护完整保留。
4. **B 区一次观察**：B 区每树仅观察一次，在首抓前于同一静止姿态下完成最多 2 个成熟目标的筛选与基座坐标冻结并关闭窗口，第二抓复用已冻结请求，不重新观察。
5. **保护姿态转场**：旧正常收臂边界直接调用 `shenzhi`；未定义或执行失败不算保护到位，必要转场保持禁止。失败仍一次 Home，不追加姿态重试。真实姿态以源码 YAML 为准，操作者须检查并补齐所用预设；客户端不添加假轨迹。

## 常用调试命令

RobViz 预览复用 `/fruit_perception/camera/preview_detections` 的同戳光学检测。
Python 节点发布同戳 `/fruit_perception/camera/base_preview_detections`。
这两路元数据均不是抓取授权，不改变观察窗口或 `/fruit_perception/decision`。
Base 要求采集时连续实测停稳，无效时为 N/A。
图像组件保持同容器 intra-process raw 数据路径。
RGB 和 Native Depth 的 JPEG 预算相互独立，原始 RGB 预览共用标注 RGB 传输设置。
配置重载成功消息精确确认版本和 HTTP 启动会话。
RobViz “图像采集”页支持一键/空格保存 NUC 原始帧为无损 PNG 至 `/home/rc4/fruit_ws/data/yolo_images/camera`，无自动连拍，无重试。

```bash
# 启动感知与扫码（不启动串口桥与任务）
ros2 launch fruit_perception fruit_perception.launch.py zone:=A

# 需要预览时单独启动 RobViz，或使用 start_fruit_perception.sh --zone A
ros2 launch fruit_robviz_bridge fruit_robviz_bridge.launch.py

# 打印当前决策消息
ros2 topic echo /fruit_perception/decision

# 打印二维码计划（Transient Local 锁存）
ros2 topic echo --qos-durability transient_local --once /fruit/b_pick_plan
ros2 topic echo --qos-durability transient_local --once /fruit/c_pick_plan
```

注：由于 `FruitDecision.msg` 增加了 `observation_window_id` 字段，历史旧包回放无法直接兼容，且所有节点需在更新接口后统一编译并重启。
