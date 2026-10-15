# 单 camera 两阶段模型运行

本文维护当前单 camera 两阶段运行契约，并保留模型接入时的性能记录。历史验收只覆盖输入、运行和资源消耗，
不代表现场识别准确率或采摘验收。硬件为 NUC11PAHi5（i5-1135G7 / Iris Xe），运行库为 OpenVINO 2024.6.0、系统 OpenCV 4.5.4。

## 模型与配置

`config/fruit_perception.yaml` 的 `model_root` 指向源码中的
`src/fruit_perception/model`，必须是绝对路径。已清理旧的未归档或未使用权重，不自动回退旧权重或 CPU。

- A/C：`vegetable_type/best.onnx`，类别为 bad、onion、pepper、pumpkin、tomato。
- B：`fruit_type/best.onnx`，类别索引为 0=apple、1=pear、2=bad。
- 两个品种检测器使用工作区 `models/` 中 2026-10-10 导出的新权重。七个熟度分类权重未变。
- 检测：640×640 letterbox，RGB /255，按品种 NMS。
- 两个检测器的 `bad` 均不做熟度分类。标注图、预览检测及 `FruitDecision.types/detections` 发布裸标签 `bad`，A 区也保留该标签。
- `bad` 框参与逐果实空间归属和窗口内否决，可选深度点仅供诊断。完成投票后的 `targets` 可含 `bad`，但不能授权抓取。不增加坏果摘除流程。
- 分类：直接使用第一阶段品种检测框映射回原图的 RGB ROI，不外扩；裁到图像范围，任一边小于 8 像素则丢弃。
- `ripeness_model=edge`（默认）：六个 `ripeness_edge/<crop>/best.onnx`，224 方形拉伸、ImageNet 归一化，logits 使用稳定 Softmax。
- `ripeness_model=yolo`：`ripeness/unified/best.onnx`，短边缩放到 224、中心裁剪、/255；输出已经有 Softmax。
- YOLO 的缩放使用 OpenCV 线性插值，并非 PIL/torchvision 抗锯齿缩放的逐像素复刻；相同真实 ROI 的跨端概率一致性和准确率留待后续识别验收，当前仅证明运行契约与性能。
- 分类器 reshape 为静态 batch=1，逐 ROI 推理；不限制检测数，不为了帧率跳过分类。
- 结果保留品种和成熟度标签，辣椒映射为 `chili_ripe/chili_unripe`，其他品种使用 `<crop>_ripe/unripe`（斜杠表示两个字符串）。ROS confidence 仍为检测分数。标注图独立显示原始品种、熟度百分比、检测分数与中心深度。
- 分类失败丢弃该 ROI，不假造 unripe。开窗/关窗及区域切换后旧在途帧不累积为当前窗口证据。

模型固定 `GPU.0` 和 FP16 推理提示，检测使用可调 `performance_hint=latency|throughput`，分类使用 LATENCY；不使用含 CPU 的 AUTO/HETERO/MULTI。保留输入 f32 契约，内部 FP16 不重写原 ONNX。
GPU 的 `COMPILATION_NUM_THREADS` 固定为 1，避免本机新检测器与 Edge 组合启动时复现的 Intel OpenCL LLVM/SPIR-V 编译崩溃。
该设置只限制模型编译线程，不改变 GPU 推理设备或推理线程。默认仍使用 Edge，不修改系统驱动。
本机设置 GPU `queue_throttle=LOW`，减少驱动在 CPU 上的忙轮询；离线基准中检测+2 ROI 的进程 CPU 从约 104% 降到约 25%，吞吐约下降 3%。不是把模型切到 CPU，也不是降低输入尺寸。
检测器和统一分类器检查 ONNX 中的类别元数据；Edge ONNX 没有类别元数据，其类别方向依据原归档契约（0=ripe、1=unripe），更换权重时必须重新核实，不能仅凭形状认定类别方向。

深度几何和目标选择器复用：A/C 单目标，B 最多两个稳定候选。动态字符串参数 `zone` 只接受 A/B/C。
当前 mission 在 A 区接受任意品种的成熟投票结果，C 使用指定品种，视觉目标已调用 planner。
感知不再订阅 `/fruit_mission/current_area`，也不按观察窗口切换模型。
切区关闭旧窗口并清除旧证据。新的 harvest 窗口区域必须匹配 `zone`，否则不采纳。
左右 USB 相机代码、配置和旧 SIDES/TREE 模式已删除。

## 快速一致性确认

A/B/C 均完整观察 10 个新处理帧，包括空检测帧，详见
[参数与结果契约](../README.md#short-window-target-confirmation)。每个果实分别统计「品种 + 熟度」票数。
熟度概率达到 0.70 才投票，检测阈值保持 0.50。
最多票胜出，不要求过半或最低次数。平票比较支持帧的平均检测置信度，再平票为 `unknown`。
不要求最后一帧支持获胜标签，也不按最短秒数等待。结果冻结后重复发布，直到关闭或切换窗口。

第一阶段 `bad` 中心落在果实框内，且交叠面积覆盖至少 50% 的 `bad` 框时，否决对应果实。
归属歧义时否决相关果实，不影响无关目标。一次有效证据在本窗口内保持，短暂漏检不能解除。
A/C 先按原置信度与水平居中评分确定主目标，再判断坏果、成熟度和品种，不用旁边好果放行固定预设。
开环不要求深度。视觉定位保留有效深度、异常点剔除及 mission 空间检查。
B 逐果实输出投票结果，再由 mission 筛选最多两个采收目标。

任务层仅接受完成的 `targets_ready/targets`，删除单帧成熟放行、三帧不匹配早退及历史成熟标签兜底。
10 秒内未完成观察则跳过，不增加重试。单个有效成熟票也可能胜出，这是操作者选择的行为，并非稳定性保证。
当前不加载第三阶段模型，后续 ROI 缺陷检测可设置同一个逐果实 `bad` 标志，不改变采收否决语义。
相机持续推理，仅在到位后的新窗口累积证据。所有运动服务、STOP 和恢复安全链保持不变。

### A1 离线回放检查

使用 `rosbags/20261015_185305/20261015_185305_0.mcap` 的原始 RGB 与现有 GPU 模型重新推理。
按窗口 ID 定位，而非假定日志显示时区：右侧 `358139ba-f7b3-41a1-b3db-c4637a9c0bf8`，
图像截止时间为 `1792061629.916847683`。扩展该观察至 10 帧，不重放原关窗或任何运动话题。

- 左侧：前 5 帧为 `pumpkin_unripe`，后续空帧，第 10 帧输出 `pumpkin_unripe`。
- 右侧：前两帧为 `tomato_ripe`，第 3 帧同时出现 `bad`，第 10 帧输出 `bad`。
- 两组均未在第 10 帧前输出完成结果。右侧后续缺陷漏检没有解除否决。

检查使用独立 ROS domain 的 localhost 感知节点，不启动相机驱动、mission、planner 或串口桥。
Bag 未记录相机内参，故使用零深度及仅用于输入校验的占位内参，明确禁止产生三维抓取点。
原录制已执行旧抓取动作，延长后的图像可能包含原机械臂运动。本检查不证明新策略下的现场图像完全相同。
该结果只验证离线二维推理与否决，不是三维定位或实机采摘验收。

投票、空间归属与坐标统计的可运行数学检查：

```bash
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon build --base-paths src --packages-select fruit_perception --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fruit_perception
colcon test-result --test-result-base build/fruit_perception --verbose
```

## 构建和启动

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --packages-select fruit_perception --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source scripts/setup_fruit_runtime.bash
ros2 launch fruit_perception fruit_perception.launch.py \
  zone:=A ripeness_model:=edge perception_log_level:=info
ros2 param set /fruit_perception_node zone B
# 比较统一分类器：停掉上一次 launch 后使用 ripeness_model:=yolo
```

`ripeness_model` 只在启动时生效。`zone` 支持运行时参数切换，无需重启相机或重新编译模型。
相机视频流与模型推理常开并持续运行；观察窗口只控制证据采纳，窗口外不输出采摘目标。
无桌面调试使用 `./start_fruit_perception.sh --zone A|B|C`，在 RobViz 查看标注 JPEG。
未提供 `--zone` 时脚本退出。C++ 标注显示熟度框颜色、百分比、检测分数、中心深度和准星。
相机默认 `initial_reset: true`，启动重置设备状态，但不能保证修复硬件 MIPI 故障。
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
