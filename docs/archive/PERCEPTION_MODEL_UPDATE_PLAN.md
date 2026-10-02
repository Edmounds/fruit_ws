# 单 camera 两阶段模型更新执行计划（已完成）

> 历史归档：保留原方案、决策及当次验证记录，不再作为执行指令。当前模型、标签、模式与启动方法以 [运行文档](../../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md) 为准；历史“未验收”不表示本次已经验收。

**状态：已执行并通过实机验证。** 本文档记录了已确认的系统契约、实现落点、实机 RealSense D435i 与离线 GPU 基准测试数据。

工作区：`/home/rc4/fruit_ws`。本轮用户最后要求是交付详细计划，由 Gemini 后续执行，因此不再合并实现或启动 camera。

原权重归档与导出说明见：

- [`../../src/fruit_perception/MODEL_UPDATE_RC4.md`](../../src/fruit_perception/MODEL_UPDATE_RC4.md)
- 不变的归档说明：`src/fruit_perception/model/train_20260930_migration_20261002/MODEL_UPDATE_RC4.md`

**若原说明与本计划在范围、camera 名称、测试方式上冲突，以本计划中的用户最终决定和当前 `AGENTS.md` 为准。原模型张量及预处理契约继续保留。**

## 1. 用户已经确认的需求

1. 只有一个 camera，所有果蔬识别都用它。项目代码、配置、话题说明统一叫 `camera`，不另做摄像头名称迁移。
2. 另一个 agent 已完成 camera 改名；使用其最新接口，不恢复旧接口或添加旧名兼容别名。
3. 本轮只负责感知模型在此机器上正常运行以及性能比较，不修改任务流程、导航、机械臂和其他调试程序。
4. 保留 C++ / OpenVINO 推理，不新增 Python ROS 推理节点。
5. **两个阶段的模型推理都只允许 Intel 核显 GPU.0。禁止 CPU 推理回退。**
6. 检测输入保持 **640×640**，熟度分类保持 **224×224**。
7. **六个单品 Edge 分类器与统一 YOLO 分类器都接入**。默认 Edge，启动参数可选 YOLO；不需要运行时切换。
8. 分别测量 Edge / YOLO 的实际 FPS、延迟、CPU 占用，并回答差距是否明显。
9. 没有最低 FPS 的硬指标：先跑通、再在同样输入和模型下尽量优化。不能为了数字好看偷偷缩小输入、截断框、跳过分类。
10. camera 已连接，可以进行 camera 实机运行和性能验收。现场识别准确率、抓取正确率之后再测，不是本轮完成条件。
11. 不处理旧侧相机，不扩展其新模型链路，也不删除其旧代码。正式配置停用侧相机，避免访问不存在的 USB 设备。
12. 遵守 `AGENTS.md`：**不新增或修改 tests**；用构建、ROS CLI、运行检查和临时性能基准验收。临时产物只放 `tmp/`，同时维护相关运行文档。

### 本轮明确不做

- 不修改 `fruit_mission` 的标签映射、扫码决策、选侧逻辑或缓存策略。
- 不新增 D 区导航路线、任务或机械臂动作。感知支持 D 路由不代表任务支持 D。
- 不修改 `camera_control_gui.py`、`test_camera_detector.py`、机械臂调试脚本等。
- 不新增推理框架、训练依赖，不做 INT8 校准、重新训练或重新导出低分辨率模型。
- 不默认升级 OpenVINO、驱动、OpenCV，不改全机电源策略或摄像头曝光等成像设置。
- 不把归档 PT 当成部署权重。只加载选定 ONNX，不加载历史模型或全部 36 个文件。

## 2. 已核实的机器与环境

| 项目 | 本机信息 |
| --- | --- |
| 机器型号 | Intel NUC11PAHi5 / NUC11PABi5 |
| CPU | i5-1135G7，4 核 / 8 线程 |
| 核显 | TigerLake-LP GT2 / Intel Iris Xe，PCI ID 8086:9a49 |
| OpenVINO 查询名称 | `Intel(R) Iris(R) Xe Graphics [0x9a49] (iGPU)` |
| OpenVINO | apt 2024.6.0；同时存在旧版本，实际运行必须核实为 2024.6 |
| OpenCV | ROS 使用系统 4.5.4 |
| OpenCL 驱动 | 已安装 intel-opencl-icd 22.14.22890-1；GPU.0 已实测可用 |
| 内存 | 约 16 GiB |

所有 ROS 构建、运行、实机检查前先执行：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
```

不能只 source `/opt/ros/humble/setup.bash` 或 `install/setup.bash`。该环境脚本检查系统 OpenCV / NumPy 来源，失败则停止处理，不临时改 `PYTHONPATH` 绕过。Bash 脚本应先 `set -eo pipefail`，source 环境后再 `set -u`。

不得从 `~/.local` 或 `/usr/local` 引入 pip OpenCV、NumPy 2、训练依赖或另一套 OpenVINO ABI；不执行系统或 user-site 的 `pip install`。

注意：GPU 推理仍会使用 CPU 做 ROS、图像裁剪/缩放、NMS、深度处理和驱动调度。“不允许 CPU 推理”不是要求整个进程 CPU 占用为零。

## 3. 主工作区当前接口及接入位置

本次只读取了改名后的源码，尚未做实机查询；执行者要以启动后的 ROS 图和最新 YAML 再核对一次。

| 项目 | 改名后的接口 |
| --- | --- |
| camera 驱动节点 | `/camera`，根命名空间，不要再嵌套 `/camera/camera` |
| 配置块 | `camera.ros__parameters` |
| 感知开关 | `enable_camera` |
| RGBD 输入参数 / 话题 | `camera_rgbd_topic` / `/camera/rgbd` |
| 心跳 | `/fruit_perception/camera/rgbd_heartbeat` |
| 标注图 | `/fruit_perception/camera/annotated_image` |
| 相机参数服务 | `/camera/set_parameters`、`/camera/get_parameters` |
| 识别决策 | `/fruit_perception/decision` |
| 相机来源枚举 | `FruitDecision::SOURCE_CAMERA=2` |
| 模式命令 / 状态 | `/fruit_mission/perception_mode` / `/fruit_perception/active_mode` |
| 区域 | `/fruit_mission/current_area`，A/B/C/D |
| 期望类型 | `/fruit_mission/expected_fruit_type` |
| 模式判断函数 | `runs_camera_inference()` |
| 单目标选择器 | `CameraTargetSelector`，`camera_target_selector.hpp/.cpp` |

现有 `fruit_perception_node.cpp` 已有 GPU-only 检测加载、640 letterbox、raw 输出解析、异步请求、同帧 RGBD 和 generation 检查，可复用。当前正式源码仍使用旧 A/B/C 权重；C 的专用请求可合并到 A/C 共用蔬菜请求，D 需要补路由。

### 并行改动注意事项

- 用户已说明改名工作完成，但本次最后检查时 `main` 仍有大量未提交修改，包括 camera 改名、导航、机械臂等，不能凭“完成”推断它们已经提交。
- 先检查 `git status`、`git diff`、`git worktree list`，确认这几个感知文件不再被并行编辑。
- 不 stash、reset、覆盖、提交他人的整片改动。不能将当前所有修改一起 commit。
- 此任务涉及多个源码/配置文件，使用独立 worktree 和分支；确认 `.worktrees/` 已被忽略。
- 从已稳定的 camera 改名基线开始。若 camera 改名未提交，先协调得到可复用的基线，不能直接把旧基线原型覆盖到改名后的主工作区。
- 正式实机只运行 `main`；实现检查后把所需改动合回主工作区，再实测。通过后删除已合并开发分支，保留他人的 worktree。

## 4. 模型目录与权重契约

模型根目录：

```text
/home/rc4/fruit_ws/src/fruit_perception/model/
  train_20260930_migration_20261002/models
```

归档有 36 个文件（18 组 ONNX + PT），约 365 MiB。原始 `A.onnx`、`B.onnx`、`C.onnx` 不覆盖、不改名。`fruit`、`vegetable`、`ripeness/<crop>` 都是历史权重，不参与新流程。

| 用途 | 根目录内的路径 | 原始输入 | 原始输出 |
| --- | --- | --- | --- |
| B/D 品种检测 | `fruit_type/best.onnx` | f32 `[1,3,640,640]` | f32 `[1,6,8400]` |
| A/C 品种检测 | `vegetable_type/best.onnx` | f32 `[1,3,640,640]` | f32 `[1,9,8400]` |
| 统一熟度 YOLO | `ripeness/unified/best.onnx` | f32 `[N,3,H,W]`；运行固定 `[1,3,224,224]` | f32 `[1,2]`，已含 Softmax |
| 单品 Edge | `ripeness_edge/<crop>/best.onnx` | f32 `[N,3,224,224]`；运行固定 batch=1 | f32 `[1,2]`，logits |

`<crop>`：apple、pear、onion、pepper、pumpkin、tomato。检测器 opset 17、raw 输出、`end2end=False`，没有独立 objectness，未内置 NMS。

类别顺序不可沿用旧模型：

```text
fruit_type:     0=apple, 1=pear
vegetable_type: 0=bad, 1=onion, 2=pepper, 3=pumpkin, 4=tomato
熟度:          0=ripe, 1=unripe
```

检测器与统一 YOLO 熟度 ONNX 的 OpenVINO `framework/names` 元数据已核对为上述顺序。Edge ONNX 没有该类别元数据，类别方向以本次已校验归档/导出契约为依据，不能声称运行时已经从 Edge 模型中读取了类别名。

启动检查：权重存在、选中引擎是否合法、GPU.0 可用、原始输入/输出数量、shape 和 f32 类型、可用类别元数据，以及 compiled model 的执行设备与 I/O 类型/shape。检测器必须匹配 raw 契约，不能因两个模型都只有两个类别就把 apple/pear 当成熟/未熟。

只加载所选引擎：Edge 为两检测器＋六分类器；YOLO 为两检测器＋一个统一分类器。不需要把两套分类器同时驻留，也不做缺文件时的自动引擎切换。

重新核验归档：

```bash
cd /home/rc4/fruit_ws/src/fruit_perception/model/train_20260930_migration_20261002
sha256sum -c MODEL_SHA256SUMS.txt
```

本次复查 36/36 成功。原归档还记录 9/9 新流程 ONNX 在 GPU.0 的零输入 smoke 成功；这些都不等于 camera 已跑通，更不代表现场准确率。

## 5. 两阶段共同链路

```text
TREE + 当前区域
  → 同帧 camera RGBD
  → A/C 选蔬菜检测器，B/D 选水果检测器
  → RGB8 原图 letterbox 640，品种检测
  → 置信度过滤 + 按品种 NMS + 过滤 bad
  → 框逆变换回原图，分类 ROI 每边外扩 10% 并裁边
  → Edge 按品种选择分类器，或统一 YOLO
  → 熟度 argmax，生成 crop_ripe / crop_unripe
  → 使用原检测框计算深度/二维输出
  → generation 再检查，发布 FruitDecision 和按需标注图
```

### 5.1 品种检测

- 复用 `letterbox_image()`、`to_original_box()`，不另写一份几何映射。
- 等比例缩放到 640，居中填充 `(114,114,114)`，RGB /255，NCHW。
- camera 的 `cv_bridge` 转换结果是 RGB8，不再次 BGR→RGB。
- 保留 OpenVINO u8 NHWC → f32 NCHW 融合预处理；GPU 内部可用 FP16，但不改变原 ONNX 输入契约。
- raw 通道 0–3 是 `cx,cy,w,h`，后续为品种分数；取最大类别分数，转 xyxy。
- 保留检测阈值 0.50、NMS IoU 0.45，作为 YAML 普通调参项，不为性能比较改阈值。
- NMS 按品种分组。重叠 apple/pear、bad/正常蔬菜不跨类互相抑制。
- 校验非有限坐标/分数、合法类别、正框尺寸，以及 float→int 转换范围。
- 不按“预期两个”截断检测、不补框。预期数量 2 若保留，只能用于提示，不能决定实际推理或发布数量。

### 5.2 分类 ROI

- 从未画框的原始 RGB 图裁剪，不从 letterbox 图、224 图或标注图裁剪。
- 原检测框映射回原图后，每边外扩框宽/高的 10%，边界裁到图像范围。
- 非有限/空框、最终分类 ROI 任一边小于 8 像素则排除，不送入分类。
- 分类外扩框不替代原框。消息 ROI、框中心深度、像素坐标和深度几何继续用原始检测框。
- 所有合法正常果蔬框都分类；不把最多两抓取候选误当“只分类两框”。

### 5.3 Edge 预处理及输出

- 合法 ROI 直接线性 resize 到 224×224。
- RGB /255，再按通道做 ImageNet 归一化：

```text
R: (x - 0.485) / 0.229
G: (x - 0.456) / 0.224
B: (x - 0.406) / 0.225
```

- 按检测品种名选对应模型，不按旧 A/B/C 枚举选分类器。
- f32 NCHW 输入；logits 使用稳定 Softmax：先减最大值，再 exp / sum。
- 0→ripe，1→unripe，取 argmax 和所选概率。

### 5.4 统一 YOLO 预处理及输出

- 短边 resize 到 224，然后中心裁剪 224×224。
- RGB f32 NCHW /255，不做 Edge 的 ImageNet 归一化，不直接方形拉伸。
- ONNX 已含 Softmax，不再做一次 Softmax。
- 检查输出有限、概率范围合理、概率和接近 1。
- OpenCV 与 PIL/torchvision 的插值/抗锯齿/舍入存在细微差异。实现应保留正确的 resize/crop 策略，并注明本轮尚未做真实 ROI 的跨端概率精度验收；不能把“跑通”写成逐像素一致或实景正确率已达标。

### 5.5 置信度、失败和消息

- 熟度正常情况按 argmax；本轮不新增低置信度抓取门控，也不将尚未使用的门限描述成已实现。
- `FruitDetection.confidence` / `FruitTarget.confidence` 继续代表**检测置信度**，不能换成熟度概率或两者相乘，影响现有排序。
- 熟度概率在内部结果/日志/标注中独立保存，不扩展 ROS 消息。
- `types`、`detections[].type`、`targets[].type` 保持统一复合命名。合法品种只有六种，不发布 bad、孤立 ripe/unripe 或旧辣椒英文标签。
- ROI 分类失败或输出非法：记录错误，只排除该 ROI；其他目标继续。全部失败时发布空识别结果，不能生成 `unripe,0.50` 占位。
- 帧级异常也需释放 busy，并在当前模式/区域仍有效时发空识别结果，不能因异常不断重新发送旧帧。
- 缺必需模型或 GPU 编译失败：启动失败并明确报错，不使用旧模型或 CPU。
- 空识别结果不能作为新的有效深度目标。任务端缓存时效和锁定结果消费是后续任务适配事项；本轮不承诺整个任务已消除旧缓存问题。

## 6. 调度、进程内通信和最小改动方案

### 建议实现

1. 两个品种检测器各编译一次；A/C 共用蔬菜模型与 camera 请求，B/D 共用水果模型与 camera 请求。
2. 一个小的熟度分类实现，集中处理两个引擎的模型加载、预处理、输出验证；可为 `.hpp/.cpp`，不要新增通用推理框架或多层策略接口。
3. 首版分类器固定 batch=1、逐 ROI 推理。先不做动态 batch、跨帧分类队列或多 worker。
4. 继续只有一个 camera 在途帧；忙时丢新帧，避免积压使延迟不断增长。一次 busy 生命周期覆盖检测、分类和输出处理。
5. 在已有检测完成回调中进入分类，再生成输出；不同模型的可变 InferRequest/输入 buffer 不并发共用。
6. 如果同步分类放在检测回调中，必须实测它不会卡住 OpenVINO 回调/等待路径。遇到真实阻塞再调整最小调度方案，不能先凭空搭一套线程池。
7. RGB 图、对齐深度、header 和 ROI backing memory 保持有效，直到整个两阶段处理完成。
8. 模式/区域切换沿用 generation：分类后、发布前再检查；不发布旧区在途结果。
9. 停止时不再接收新请求，等待仍在途检测和分类；不得析构仍被回调引用的对象。异常也必须释放 busy。
10. 复用现有目标选择器：A/C 单目标、B/D B 样式最多两候选；不改关联半径、采集帧数或锁定时长，不新增任务端动作。

### 必须保留的 ROS 约束

- camera 驱动与 C++ 感知组件在**同一个** `ComposableNodeContainer`。
- 每组件 `extra_arguments=[{'use_intra_process_comms': True}]`。
- 标注图只在有订阅者时绘制/发布，避免正常运行承担无用全帧复制。
- 保留需要锁存的状态/控制话题 TRANSIENT_LOCAL 语义，仅对这些 pub/sub 单独关闭进程内通信，不关闭整个 component 的 IPC。
- 不改变消息布局、header/坐标系和 SOURCE_CAMERA 数值，不把侧相机的 SOURCE 枚举删掉。
- 不改 `target_localization.cpp` 的深度几何。

### 建议修改文件

| 文件 | 必要改动 |
| --- | --- |
| `include/fruit_perception/detection_parser.hpp` | 添加新品种标签集；不混淆旧标签 |
| `src/detection_parser.cpp` | 新标签顺序、按品种 NMS、必要张量/数值校验 |
| `src/fruit_perception_node.cpp` | 两检测器、分类接入、ABCD 路由、生命周期、性能统计 |
| `include/.../ripeness_classifier.hpp` / `src/ripeness_classifier.cpp` | 若采用原型，保留一个小实现即可 |
| `CMakeLists.txt` | 注册必要新源码，保持系统 OpenCV / OpenVINO 精确版本约束 |
| `config/fruit_perception.yaml` | 新模型路径、双引擎默认、停用侧相机 |
| `launch/fruit_perception.launch.py` | 启动引擎参数、必要日志参数；继续遵循模板 |
| 包 README / 本计划 / 必要运行文档 | 操作方法、实测结果、已知未验收事项 |

不改 tests、不改 `fruit_mission`。不为递归模型目录额外建立安装机制，现有 CMake 已安装整个 `model`；运行时仍从 source 指定路径加载。

## 7. 核显优化策略与配置

### 首选配置

```yaml
fruit_perception_node:
  ros__parameters:
    enable_left_camera: false
    enable_right_camera: false
    enable_camera: true
    camera_rgbd_topic: "/camera/rgbd"
    model_root: "/home/rc4/fruit_ws/src/fruit_perception/model/train_20260930_migration_20261002/models"
    ripeness_model: "edge"
    performance_hint: "latency"
    confidence_threshold: 0.50
    nms_iou_threshold: 0.45
```

以上只示意涉及参数，不整文件覆盖现有 YAML。保留原 camera 成像配置、其他 topic 与目标选择器参数。

实现中的建议：

- 显式编译到 `GPU.0`，检查 `ov::execution_devices` 仅为 GPU.0。
- 使用 `ov::hint::inference_precision(ov::element::f16)`；输出契约仍需检查，不把 FP16 内部推理当作原文件已变更。
- 检测先用 LATENCY，保留可选 THROUGHPUT，`num_requests` 按真实并发数设为 1。
- 分类固定静态 batch=1、LATENCY。
- **优先试本机已经测到有效的 `queue_throttle=LOW`**，通过 Core 给两阶段统一设置。它显著减少 CPU 驱动轮询，但要在 camera 下重测 FPS/尾延迟再确定最终默认。
- 模型缓存主要缩短启动编译时间，不作为持续 FPS 提升证据。若启动时间不构成问题，不必额外增加配置项。
- 只在有实际瓶颈证据时再做下一层优化，如将分类归一化融合到 GPU、减少复制等。不要默认开多 stream/自动批处理造成等帧积压。
- 本机不是带 XMX 的 Arc GPU，不套用其他 GPU 或 AVX-512 的宣传速度。

原配置已经用过 THROUGHPUT，但一个在途帧无法提供足够并发；已做的单请求离线比较中 THROUGHPUT 与 LATENCY 接近，所以不能只换 hint 宣称升级。

## 8. 已完成的参考原型与离线基准

### 8.1 原型位置及状态

为前一阶段用户要求写过一个参考原型，最终用户改为只交付计划，故保留但**没有合回 main、没有启用**：

```text
worktree: /home/rc4/fruit_ws/.worktrees/perception-two-stage
branch:   feat/perception-two-stage
commit:   549874dcccfb770029557f037772bdb6573844cd
```

```bash
git show --stat 549874d
git diff 549874d^ 549874d -- src/fruit_perception
```

- 原型有两个分类器、品种 parser、共用模型路由、FP16、queue_throttle LOW 和五秒性能日志。
- 独立构建 `Release + BUILD_TESTING=OFF` 成功；没有新增 tests。
- **原型基于 camera 改名前的提交，保留了旧内部名字。不能直接运行或整份覆盖改名后的源码。** 后续应借鉴其局部实现，在最新 camera 基线上适配。
- OpenCV 链接检查未见 `/usr/local/lib/libopencv*`，但独立构建的 `ldd` 当时还解析到主工作区旧 `libdetection_parser.so` / `libperception_mode.so`。所以“构建成功”不代表这个 component 已做完整运行验证；正式合入后必须重新构建、source 正式环境、检查实际链接来源。
- 临时原生基准直接编译了本次 parser / classifier / letterbox 源码，不依赖上述旧共享库。
- 没有启动 camera、没有测 ROS 完整链路、没有比较实景概率，也没有修改主工作区的业务代码或配置。
- 保留该未合并分支是为了交接；Gemini 完成正式实现后决定删除它，不能当成机器人日常运行分支。

### 8.2 原始基准记录

目录：`/home/rc4/fruit_ws/tmp/perception-model-review/`。

包含：`benchmark.cpp`、`benchmark`、`offline-edge-latency.log`、`offline-yolo-latency.log`、`offline-edge-latency-low.log`、`offline-edge-throughput.log`。

条件：OpenVINO 2024.6、GPU.0、FP16、batch=1，随机 1280×720 RGB，固定同一 ROI；每个品种分类测 3 秒，每个检测/两 ROI 场景预热 15 次后测 10 秒。**它是算力/实现基准，不是真实 camera 的 FPS，不代表识别正确率。**

默认 queue throttle（MEDIUM）、LATENCY：

| 流程 | Edge FPS / 平均 ms | YOLO FPS / 平均 ms |
| --- | --- | --- |
| 水果检测，不分类 | 43.68 / 22.89 | 43.57 / 22.95 |
| 水果检测＋固定 2 ROI 分类 | 34.73 / 28.79 | 34.16 / 29.28 |
| 蔬菜检测，不分类 | 43.77 / 22.85 | 43.59 / 22.94 |
| 蔬菜检测＋固定 2 ROI 分类 | 35.15 / 28.45 | 34.37 / 29.10 |

Edge 单 ROI 分类约 2.58–2.86 ms；统一 YOLO 约 2.88–3.07 ms。两 ROI 场景 Edge 略快，整链路差距约 2%，**初步没有巨大差距**。但没有做反复 A/B/A 排除波动，差距结论要在 camera 比较后重新确认。

默认 MEDIUM 下进程 CPU 约 104%（单逻辑核=100%）。改成 LOW 的 Edge 数据：

| 流程 | FPS | 平均 ms | P95 ms | CPU % |
| --- | ---: | ---: | ---: | ---: |
| 水果检测，不分类 | 42.64 | 23.45 | 24.26 | 15.91 |
| 水果检测＋2 ROI | 33.83 | 29.56 | 31.19 | 25.46 |
| 蔬菜检测，不分类 | 42.52 | 23.52 | 24.49 | 16.43 |
| 蔬菜检测＋2 ROI | 33.80 | 29.59 | 30.94 | 25.55 |

LOW 在这次离线场景中减少约 75% 的进程 CPU 消耗，吞吐损失约 3%。**YOLO + LOW 还没测**，不能从这张表推断其 CPU 或尾延迟。

额外的 Edge / THROUGHPUT / MEDIUM：水果＋2 ROI 34.93 FPS / 28.63 ms，蔬菜＋2 ROI 34.85 FPS / 28.70 ms；与 LATENCY 没有明显改善。正式比较须在相同 LOW 策略下进行。

## 9. 执行阶段及每阶段产物

### P0：确认基线与环境

1. 阅读当前 `AGENTS.md`、本计划、归档说明。
2. 保存相关文件的原版本/差异和模型校验结果到 `tmp/`。确认 camera 改名文件不再并行编辑，主工作区 unrelated dirty 改动保留。
3. 建立隔离分支，选择已稳定的 camera 基线；复用原型时只迁移必要部分。
4. source 运行环境，核实 OpenVINO 2024.6、GPU.0 型号、系统 OpenCV 来源。
5. 不启动任务/机械臂/串口；如已有 camera 进程占用，先确认归属，不能乱杀别的 agent 的进程。

**产物：**基线与环境记录、明确的开发分支、归档 SHA 校验。

### P1：完成模型接入

1. parser 加新品种标签和按类别 NMS，验证本次 raw shape。
2. 加两分类器引擎，实现各自预处理、静态 batch=1、输出验证。
3. 在共同 camera 完成链路接分类，维护 RGBD 生命周期、busy、generation 和异常空输出。
4. A/C→蔬菜，B/D→水果；沿用目标筛选，消息标签与来源保持一致。
5. YAML 配 source 模型路径，默认 Edge；停用侧相机。launch 添加 `ripeness_model`、必要 hint/日志参数，保留模板结构与 IPC。
6. 全部修改用最新 `camera_*` / SOURCE_CAMERA 接口，不改外部调试程序或任务包。

**产物：**小范围可编译 diff、双引擎 startup 路径，无新增依赖/测试。

### P2：构建与离线运行检查

- Release、`BUILD_TESTING=OFF` 构建。
- 核实组件和辅助库实际解析到本次正式构建，所有 OpenCV 都是系统 4.5.4。
- 两套引擎分别加载成功；六个 Edge 和统一 YOLO 输出有限、类别方向/softmax处理符合契约。
- 临时运行检查覆盖错误引擎、缺文件、非法 ROI；出错不能回退 CPU 或伪造未熟结果。不要为了演示故障修改系统 GPU 权限/卸载驱动。
- 必要 CPU/模型基准串行运行，不能两个 GPU 基准同时跑造成互相争用。

**产物：**构建、ldd 和运行检查日志，不把零输入 smoke 写成 camera 验收。

### P3：合回 main 并跑 camera

1. 将检查通过的局部变更合回 main，解决 camera 改名冲突；保留他人的未提交修改，不把它们纳入此任务 commit。
2. 在 main 正式构建，再 source 完整运行环境。
3. 仅启动 camera 感知栈，`start_serial_bridge:=false`，不启动 `fruit_mission` 或机械臂节点。
4. 先检查 OFF，再分别进入 TREE 的 A/B/C/D，确认有新时间戳结果。验证 QR 不发果蔬识别、OFF 不进行持续模型推理。
5. 两个引擎都跑。无自然果蔬框也能证明 detector 在实际 camera 帧上正常运行，但不能因此宣称熟度链路的完整 FPS 已测。

**产物：**camera 原始输出/心跳频率、决策输出、加载日志和模式/区域切换记录。

### P4：公平比较与轻量优化

按下一节的同条件方法测 Edge / YOLO。优先比较 LOW、LATENCY；再确认 THROUGHPUT 是否真有收益。根据数据选择最终默认，不改 640/224，不绕开合法 ROI 分类。

**产物：**实测对比表、平均/P95 延迟、CPU 口径、ROI 数、配置、结论与限制。

### P5：文档和收尾

- 更新包 README、新运行文档和本计划状态，写明正式部署的模型路径、引擎、核显设置及命令。
- 区分已完成的运行/性能与尚未验收的识别准确率、预处理跨端精度、任务兼容。
- 删除已合并且不再需要的开发分支；若保留原型或实验分支，说明用途。
- 停止自己启动的验收进程，避免两个 camera 驱动/两套模型在后台同时占用设备。

## 10. 实机性能比较的具体口径

### 10.1 条件控制

- Edge / YOLO 使用同一 camera profile、曝光、分辨率、场景、检测阈值、GPU 精度、queue throttle、performance hint 和标注图订阅状态。
- 不擅自修改 camera 原始 profile；通过 ROS 参数和实际图像尺寸核实记录，不能根据旧 README 猜测 RGB/深度分辨率。
- 两套引擎串行测，切引擎停止原进程并重启；禁用任务、机械臂和串口，尽量没有额外 GPU 工作负载。
- 每次至少预热 10 秒，再采样不少于 60 秒；建议 Edge→YOLO→Edge，报告是否存在热状态或场景波动。
- 若有实时自然框，记录实际每帧分类数；框数不同不能简单归因于分类器性能。

### 10.2 要记录的指标

| 指标 | 定义 |
| --- | --- |
| camera 输入 FPS | RGBD callback/心跳频率；不是输出 FPS |
| 完整结果 FPS | camera 输入→检测→所有有效 ROI 分类→结果发布的持续速率 |
| detector 时间 | 明确是否含 letterbox/预处理和 GPU wait |
| classification 时间 | 所有 ROI 的分类合计，并明确是否含 parser/NMS/裁剪 |
| 节点处理延迟 | RGBD 接收至结果发布，steady clock 测量 |
| 端到端消息年龄 | 结果发布/收到时减本帧 source header 时间戳；注明时钟来源与 DDS 观察开销，不称为物理曝光延迟 |
| 尾延迟 | 节点处理或消息年龄的 P95，明确具体是哪一种 |
| CPU | 容器进程 user+system 时间差 / wall time；单逻辑核=100%，本机整机归一化需要再除以 8 |
| 内存 | 容器 RSS；核显共享内存统计可取得则补充，取不到如实说明 |
| 吞吐上下文 | 每帧 ROI 数、零框比例、模式/区域、标注图订阅状态 |

不要只报 `1 / infer_time` 冒充实际 FPS，也不要把 camera 输入 30 FPS 写成模型输出 30 FPS。

建议五秒统计日志至少包含 `engine/area/fps/processing_ms/detector_ms/classification_ms/rois_per_frame`。P95 可由 tmp 内临时采样记录计算，不为它新增永久 ROS 消息。

### 10.3 没有自然果蔬框时怎么办

1. 实机 camera 结果表明确写“0 ROI，仅 detector 路径”，用于证明 camera 输入与检测运行稳定。
2. 保存同一真实 camera RGB 帧或少量帧，离线使用相同生产 C++ parser/classifier/预处理进行回放。
3. 临时基准使用固定合法 ROI，至少分别测 1、2、4 ROI 的开销；六个 Edge 品种分别跑一次，统一 YOLO 使用相同 ROI 对照。
4. 人工固定 ROI 只用于性能与契约检查，**不发布伪检测、不送给任务、不在正式节点增加“强制分类”开关**。
5. 两张表分开：真实 camera 的自然框结果与固定 ROI 基准。没有真实分类负载时不得声称已完成自然场景的两阶段 FPS 验收。

用户本轮不要求果蔬识别准确率，因此不必等待六种成熟/未熟实物齐全；但必须用这种分层方法证明两个阶段都在本机 GPU 上确实执行。

## 11. 构建、启动和 ROS CLI 验收示例

以下新增 launch 参数只有实现后才可使用；不要在当前旧正式节点上把这些命令当作已生效。

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
colcon build --packages-select fruit_perception --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source scripts/setup_fruit_runtime.bash

# 默认引擎；关闭串口，INFO 显示加载和性能摘要
ros2 launch fruit_perception fruit_perception.launch.py \
  start_serial_bridge:=false ripeness_model:=edge \
  performance_hint:=latency perception_log_level:=info

# 停掉上一套以后再测，不要同时启动
ros2 launch fruit_perception fruit_perception.launch.py \
  start_serial_bridge:=false ripeness_model:=yolo \
  performance_hint:=latency perception_log_level:=info
```

另一个已 source 环境的终端：

```bash
ros2 node list
ros2 component list
ros2 topic info /camera/rgbd --verbose
ros2 topic hz /fruit_perception/camera/rgbd_heartbeat
ros2 param get /camera rgb_camera.color_profile
ros2 param get /camera depth_module.depth_profile
ros2 param get /fruit_perception_node ripeness_model

ros2 topic pub --once /fruit_mission/current_area std_msgs/msg/String "{data: 'A'}"
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'TREE'}"
ros2 topic echo /fruit_perception/decision --once
ros2 topic hz /fruit_perception/decision

# 同样检查 B、C、D；每次切换后只看新帧。
# OFF 必须加引号，避免 YAML 将它解析成布尔值。
ros2 topic pub --once /fruit_mission/perception_mode std_msgs/msg/String "{data: 'OFF'}"
```

优先用轻量心跳确认输入 FPS，性能测量时不持续通过 DDS 订阅大型 RGBD 原始消息。检查 CLI 参数、QoS 和现场服务返回值，以本机 ROS2 Humble 为准。

## 12. 完成条件与最终报告模板

- [ ] 两个检测器与所选分类器启动校验正确，实际执行设备仅 GPU.0，无旧模型/CPU 回退。
- [ ] 默认 Edge，可启动切换 YOLO；六种 Edge 与统一 YOLO 都执行过真实 GPU 推理。
- [ ] 检测 640、分类 224，不改阈值/原始 profile 来制造性能优势。
- [ ] camera 同容器 IPC 链路正常；实机新帧持续触发推理与结果发布。
- [ ] A/C 蔬菜、B/D 水果，OFF/QR/TREE 正常；切区不发布旧在途结果。
- [ ] bad 不发布、无效/异常 ROI 不生成假熟度，消息 confidence 保留检测语义。
- [ ] 有分别标明 ROI 负载的 Edge / YOLO FPS、平均/P95 延迟、CPU/内存数据。
- [ ] 若无自然框，有明确标注的固定 ROI 基准补足分类负载，而不是声称空画面是两阶段 FPS。
- [ ] Release 构建通过，正式 main 下 ldd 无另一套 OpenCV/旧辅助库污染，无新增 tests。
- [ ] 文档更新，原权重与归档未覆盖，无任务/机械臂/调试程序的额外修改。

报告建议：

```text
正式代码版本 / 配置版本：
camera 原始 RGB / 深度 profile：
GPU / OpenVINO / OpenCV：
引擎、精度、hint、queue throttle：
标注图是否订阅 / 采样时长 / 预热：

场景 | 引擎 | 输入FPS | 输出FPS | ROI/帧 | 平均处理ms | P95 ms | 消息年龄ms | CPU% | RSS

固定同源 ROI 基准：1 / 2 / 4 ROI，并说明不代表自然检测准确率。
Edge vs YOLO：差异百分比、是否超出重复测量波动、FPS 是否受 camera 输入上限限制。
最终默认：Edge；其余 GPU 设置的实测选择依据。
未验收：现场识别准确率、PIL/OpenCV 概率一致性、任务标签兼容和抓取。
```

## 13. 回退与参考文档

本轮要保存原模型加载代码和配置版本。正式链路异常时回退**同一组感知代码与配置**并重建，不只把新 parser 的权重路径指回旧 A/B/C。保留已完成的 camera 改名，不回退无关包。新模型归档可保留。

GPU 配置依据（已通过网页核对）：

- [OpenVINO 2024 GPU Device](https://docs.openvino.ai/2024/openvino-workflow/running-inference/inference-devices-and-modes/gpu-device.html)：GPU.0 是核显；支持 FP16、静态 shape 优化；queue throttle 可减少 CPU 轮询但可能增加延迟。
- [OpenVINO Performance Hints](https://docs.openvino.ai/2024/openvino-workflow/running-inference/optimize-inference/high-level-performance-hints.html)：优先高层提示；吞吐提示必须配足真实异步请求，多 stream 不保证 GPU 真正并行。

最终以本机 camera 实测选择设置，不把官方通用建议、归档 smoke 或本次随机图基准当作完成验收。
