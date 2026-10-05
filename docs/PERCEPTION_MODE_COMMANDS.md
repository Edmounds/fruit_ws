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
2. **证据严格隔离**：
   - 处于窗口之外时，`/fruit_perception/decision` 发布的决策消息中 `observation_window_id` 为空（`""`），`has_target=false`，`targets_ready=false`，`targets` 列表为空。**窗外的原始检测绝不授权抓取**。
   - 只有在有效 harvest 窗口内且帧时间戳满足条件时，滑动窗口目标跟踪器（A/C 区 `CameraTargetSelector` 8 帧窗口，B 区 `BTargetSelector` 8 帧窗口）才累积证据；达到锁定条件后输出带有对应 `observation_window_id` 的可抓取目标。
3. **二维码到站即扫**：
   - `fruit_qr_planner` 仅在 `purpose="qr"` 的窗口内对窗后帧进行解码，同一窗口仅发布一次确认计划（`confirm_frames=1`，单次等待上限 10 秒）。
   - 输出的 `/fruit/b_pick_plan` 和 `/fruit/c_pick_plan` 计划中携带 `observation_window_id`、`image_stamp_sec`、`image_stamp_nanosec`。
   - B 扫码失败后直接使用任务固定兜底计划并继续执行，不阻断后续 C 扫码窗口开启。

## 任务编排与静止边界

1. **到位即开窗**：正式 mission 中，底盘导航到站且对应的 Look / QR 预设动作执行完成后，直接开启对应的观察窗口。
2. **物理静止保证**：依据用户明确保证，移除了开窗前与窗口内的 `stable_stop` 阻塞等待与持续监测配置；由现场物理保证观察到抓取目标冻结期间机构完全静止。
3. **安全恢复保护**：运动结果未知时的 STOP 发布、导航取消、EMERGENCY 锁定、恢复前 STOP 之后的机械臂停稳确认（`is_stationary` <= 0.05 rps）以及一次 Home 复位保护完整保留。
4. **B 区一次观察**：B 区每树仅观察一次，在首抓前于同一静止姿态下完成最多 2 个成熟目标的筛选与基座坐标冻结并关闭窗口，第二抓复用已冻结请求，不重新观察。

## 常用调试命令

```bash
# 启动感知与扫码（不启动串口桥与任务）
ros2 launch fruit_perception fruit_perception.launch.py start_serial_bridge:=false

# 打印当前决策消息
ros2 topic echo /fruit_perception/decision

# 打印二维码计划（Transient Local 锁存）
ros2 topic echo --qos-durability transient_local --once /fruit/b_pick_plan
ros2 topic echo --qos-durability transient_local --once /fruit/c_pick_plan
```

注：由于 `FruitDecision.msg` 增加了 `observation_window_id` 字段，历史旧包回放无法直接兼容，且所有节点需在更新接口后统一编译并重启。
