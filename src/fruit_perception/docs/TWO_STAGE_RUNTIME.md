# 单 camera 两阶段模型运行

本文维护当前单 camera 两阶段运行契约，并保留模型接入时的性能记录。历史验收只覆盖输入、运行和资源消耗，
不代表现场识别准确率或采摘验收。硬件为 NUC11PAHi5（i5-1135G7 / Iris Xe），运行库为 OpenVINO 2024.6.0、系统 OpenCV 4.5.4。

## 模型与配置

`config/fruit_perception.yaml` 的 `model_root` 指向源码中的
`src/fruit_perception/model`，必须是绝对路径。已清理旧的未归档或未使用权重，不自动回退旧权重或 CPU。

- A/C：`vegetable_type/best.onnx`，类别为 bad、onion、pepper、pumpkin、tomato。
- B/D：`fruit_type/best.onnx`，类别为 apple、pear。
- 检测：640×640 letterbox，RGB /255，按品种 NMS；bad 不分类、不发布。
- 分类：原始 RGB 框向外扩展每边 10%，裁到图像范围，任一边小于 8 像素则丢弃。
- `ripeness_model=edge`（默认）：六个 `ripeness_edge/<crop>/best.onnx`，224 方形拉伸、ImageNet 归一化，logits 使用稳定 Softmax。
- `ripeness_model=yolo`：`ripeness/unified/best.onnx`，短边缩放到 224、中心裁剪、/255；输出已经有 Softmax。
- YOLO 的缩放使用 OpenCV 线性插值，并非 PIL/torchvision 抗锯齿缩放的逐像素复刻；相同真实 ROI 的跨端概率一致性和准确率留待后续识别验收，当前仅证明运行契约与性能。
- 分类器 reshape 为静态 batch=1，逐 ROI 推理；不限制检测数，不为了帧率跳过分类。
- 内部结果按模型品种组成熟度标签；对外 A 区保留 `ripe/unripe`，C 区辣椒映射为 `chili_ripe/chili_unripe`，其他品种使用 `<crop>_ripe/unripe`（斜杠表示两个字符串）。检测置信度仍作为 ROS confidence，分类概率保存在内部结果/DEBUG 日志。GUI 物种检测标签如 `pepper` 不等于正式 ROS 标签。
- 分类失败丢弃该 ROI，不假造 unripe。开窗/关窗及区域切换后旧在途帧不累积为当前窗口证据。

模型固定 `GPU.0` 和 FP16 推理提示，检测使用可调 `performance_hint=latency|throughput`，分类使用 LATENCY；不使用含 CPU 的 AUTO/HETERO/MULTI。保留输入 f32 契约，内部 FP16 不重写原 ONNX。
本机设置 GPU `queue_throttle=LOW`，减少驱动在 CPU 上的忙轮询；离线基准中检测+2 ROI 的进程 CPU 从约 104% 降到约 25%，吞吐约下降 3%。不是把模型切到 CPU，也不是降低输入尺寸。
检测器和统一分类器检查 ONNX 中的类别元数据；Edge ONNX 没有类别元数据，其类别方向依据原归档契约（0=ripe、1=unripe），更换权重时必须重新核实，不能仅凭形状认定类别方向。

深度几何和目标选择器复用：A/C 单目标，B/D 最多两个稳定候选。当前 mission 使用 A 裸熟度、C `chili_*` 契约，视觉目标已调用 planner；D 只有感知路由，没有 D 区任务。左右 USB 相机代码、配置和旧 SIDES/TREE 模式已删除。

## 快速一致性确认

固定 35/50 帧批次已改为源码 YAML 配置的短滚动窗口，详见
[参数与结果契约](../README.md#short-window-target-confirmation)。起始值为 A/C、B 各 8 个处理帧，
同一空间目标至少 6 次支持相同的「品种 + 熟度」标签、支持比例至少 0.75，确认期间允许连续漏 2 帧。
支持观测的熟度概率至少 0.70；不足该值仍发布原分类标签用于诊断，但不算采收确认依据。
检测阈值保持 0.50，ROS `confidence` 仍是检测分数，不是熟度概率。

A 区对外仍发布裸熟度，但内部不再丢掉品种，所以品种跳变不能混成「稳定成熟」。
A/C 证据足够即提前输出；当前观测必须支持已确认标签，锁定后在匹配帧重复发布冻结均值，避免只发一次结果被客户端错过。
B 区经过短窗口预观察，当前存活候选全部确认且品种一致后一次冻结最多两个目标；
不能让少数高分错误品种覆盖多数正确观测，也不能在第二个已可见目标仍未确认时抢先冻结第一个。
持续不确定由现有客户端独立超时处理，不增加重试。

这些数值是可调起始值，不是实景标定。清晰 A/C 最快在第 6 个处理帧确认，清晰 B 在第 8 个处理帧确认；
不能把相机 30 FPS 当成实际两阶段处理速度，亦不能据此保证 1 秒识别。
相机视频流常开、持续推理，仅在到位后观察窗口累积确认证据；保留运动服务、未知运动 STOP 及恢复安全链。

## 构建和启动

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --packages-select fruit_perception --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source scripts/setup_fruit_runtime.bash
ros2 launch fruit_perception fruit_perception.launch.py \
  ripeness_model:=edge start_serial_bridge:=false perception_log_level:=info
# 比较统一分类器：停掉上一次 launch 后使用 ripeness_model:=yolo
```

模型选择只在启动时生效。相机视频流与模型推理常开并持续运行；根据 `/fruit_mission/observation_window` 观察窗口采纳与累积证据，窗口外不输出采摘目标。
相机和感知 component 在同一容器并启用进程内通信。仅感知验收时禁止启动任务或串口桥。
窗口接口及操作见[观察窗口参考](../../../docs/PERCEPTION_MODE_COMMANDS.md)。

性能日志每五秒输出一次（需把感知节点 logger 设为 INFO）：

```text
camera performance: engine=... area=... fps=... processing_ms=...
detector_ms=... classification_ms=... rois_per_frame=...
```

`processing_ms` 为接到 RGBD 到结果发布的节点处理耗时，不是曝光到发布的端到端时延；端到端时延应另按结果 header 时间戳测量。`classification_ms` 还包含框解析、NMS 和 ROI 处理。零 ROI 的 FPS 不能称为两阶段 FPS；应同时报告每帧实际分类数。CPU 百分比以单个逻辑核为 100%，本机八线程；GPU 推理仍会使用 CPU 做采集、预处理、ROS 及驱动调度。

## 性能对比

### 实机 camera 实测（Intel NUC i5-1135G7 / Iris Xe GPU.0 / FP16）

以下为原模型接入记录的 RealSense D435i RGBD 流测量（约 15.6 FPS），不是当前 YAML
配置帧率或本次重测结果。表中未记录自然 ROI 数，分类耗时很小，不能据此认定完整两阶段负载或零丢帧：

| 引擎 | 模式 / 区域 | 输入 FPS | 决策发布 FPS | 节点单帧处理耗时 | 检测耗时 | 分类/后处理耗时 | 容器 CPU (1核=100%) | 容器 RSS 内存 |
|---|---|---|---|---|---|---|---|---|
| **Edge** | DETECT / A | 15.6 | 15.6 | 24.3 ~ 24.8 ms | 24.2 ~ 24.6 ms | 0.11 ~ 0.13 ms | ~32% (整机约 4%) | ~919 MB |
| **Edge** | DETECT / B | 15.6 | 15.6 | 24.5 ~ 25.1 ms | 24.4 ~ 25.0 ms | 0.11 ~ 0.12 ms | ~32% (整机约 4%) | ~919 MB |
| **YOLO** | DETECT / A | 15.6 | 15.6 | 24.5 ~ 24.9 ms | 24.4 ~ 24.7 ms | 0.12 ~ 0.14 ms | ~32% (整机约 4%) | ~551 MB |

### 离线固定 ROI 极限基准（同源 1280×720 RGB，queue_throttle=LOW）

| 场景 | Edge 耗时 / FPS | YOLO 耗时 / FPS | 差距分析 |
|---|---|---|---|
| 单品熟度分类 (224×224) | 2.7 ~ 2.8 ms / ROI | 3.1 ~ 3.3 ms / ROI | Edge 快约 0.4 ms/ROI |
| 检测 (640×640) 单独 | 23.5 ms / 42.6 FPS | 23.5 ms / 42.5 FPS | 检测器一致 |
| 检测 + 2 个 ROI 分类 | 29.6 ms / 33.8 FPS | 30.5 ms / 32.8 FPS | **整体吞吐差距仅约 3% (~1 FPS)** |
| 内存占用 | 需驻留 6 个单品模型 (~919 MB) | 仅需 1 个统一模型 (~551 MB) | YOLO 节省约 368 MB 内存 |

**记录能支持的结论**：固定 2 ROI 的离线基准中，两引擎吞吐相差约 3%；该结果不等于
真实场景端到端 FPS。相机表中发布频率接近输入频率，未证明自然多 ROI 负载下零丢帧。
当前默认 Edge；内存受限时可在停止上一次 launch 后以 `ripeness_model:=yolo` 启动比较。
现场准确率、相同 ROI 的跨端概率一致性及实际采摘仍需单独验收，不能由性能推断。

## 配置依据

- [OpenVINO 2024 GPU Device](https://docs.openvino.ai/2024/openvino-workflow/running-inference/inference-devices-and-modes/gpu-device.html)：GPU.0 对应核显，优先 FP16 和静态形状，缓存主要缩短模型启动编译时间。
- [Performance Hints](https://docs.openvino.ai/2024/openvino-workflow/running-inference/optimize-inference/high-level-performance-hints.html)：吞吐提示必须匹配真实并发请求数。当前只有一个在途 camera 帧，不能靠增加 streams 宣称帧率提升；需比较实测，而不是用离线批吞吐代替实时性能。
- 本机 Iris Xe 不是带 XMX 的 Arc GPU，不套用其他硬件的宣传数字。本轮不做 INT8 校准、不改 640/224、不降低摄像头分辨率。
