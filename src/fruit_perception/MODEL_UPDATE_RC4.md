# RC4 果蔬模型更新与识别逻辑迁移说明

> **历史交付说明，不是当前部署指南。**两阶段模型已接入，左右 USB 相机及 SIDES/TREE、FruitDecision.source 已删除。
> 当前模式为 OFF/QR/DETECT；A 对外为 ripe/unripe，C 辣椒为 chili_*，不是下文历史方案拟议的全区 pepper_* 标签。
> 当前配置、命令及验收边界见 [TWO_STAGE_RUNTIME.md](docs/TWO_STAGE_RUNTIME.md)。下文“当前/目标/后续”均指原交付时点，不要重复实施或执行其旧构建命令。模型归档内原说明及校验和不修改。

## 1. 本次交付与边界

本说明依据本地 `/Users/cqc/fruit_train` 的正式权重、`infer.py`，以及 RC4 上实际源代码核对后编写。模型训练批次为 `20260930-161733-b6562708`。

**本次只交付说明和模型归档，不修改 RC4 的代码、YAML、消息定义，不构建功能包、不重启、不启用新模型。文件同步完成不代表识别逻辑已更新。**

已确认的下一步实现目标：

- 继续使用现有 C++ / OpenVINO GPU 推理，不新增 Python ROS 节点。
- 一阶段识别品种，二阶段识别熟度；默认六个单品 Edge 分类器，可切换统一 YOLO 分类器。
- 统一只用 camera。侧相机与旧模式 SIDES/TREE 已彻底删除，统一使用 OFF / QR / DETECT 相机状态管理。
- FruitDecision 中已移除 source 字段和来源常量。
- A/C 使用蔬菜品种检测器，B/D 使用水果品种检测器，四区均通过 camera 识别。
- ABCD 识别预期数量暂定 **2**，只作提示，不截断检测结果、不按数量补目标。
- 抓取策略暂不改：A/C 仍单目标锁定，B/D 使用 B 的多目标候选策略，上限 2；这不意味着每次抓取两个。
- 所有赛区对外统一 `<品种>_<熟度>`，例如 `pepper_ripe`，不再只发布 `ripe`，不再使用 `chili`。
- 分类异常不能伪造成 `unripe`，异常目标不得进入抓取候选。

## 2. 文件位置与版本

RC4 功能包源目录：`/home/rc4/fruit_ws/src/fruit_perception`。

本次独立归档目录：

```text
fruit_perception/
├── MODEL_UPDATE_RC4.md
└── model/train_20260930_migration_20261002/
    ├── MODEL_UPDATE_RC4.md
    ├── MODEL_SHA256SUMS.txt
    ├── OPENVINO_SMOKE_RESULTS.json
    └── models/                     # 本地 models 的完整目录镜像
        ├── fruit/{best.onnx,best.pt}
        ├── vegetable/{best.onnx,best.pt}
        ├── fruit_type/{best.onnx,best.pt}
        ├── vegetable_type/{best.onnx,best.pt}
        ├── ripeness/{best.onnx,best.pt}
        ├── ripeness/unified/{best.onnx,best.pt}
        ├── ripeness/<crop>/{best.onnx,best.pt}
        └── ripeness_edge/<crop>/{best.onnx,best.pt}
```

`<crop>` 为 `apple, pear, onion, pepper, pumpkin, tomato`。旧 `model/A.onnx`、`B.onnx`、`C.onnx` 和原配置保持不变；不要把新模型改名覆盖它们。

全量归档为 **36 个模型文件（18 组 ONNX + PT），382,835,840 字节，约 365.10 MiB**。

下一步新加载路径以归档目录里的 `models` 为根。两份统一熟度权重 `ripeness/best.*` 与 `ripeness/unified/best.*` 本次内容相同，优先使用后者。`fruit`、`vegetable` 和 `ripeness/<crop>` 是历史权重，只归档，不参与新两阶段流程。Edge 的 `.pt` 是 torchvision 的 state_dict，不是 YOLO 模型，不能交给 `YOLO()` 或 OpenVINO 直接加载。

## 3. 本次启用候选模型的真实契约

以下形状为 ONNX 原始输入/输出，不是 OpenVINO 预处理融合后的输入。

| 用途 | 相对 `models/` 路径 | 架构 | f32 输入 | 输出 |
|---|---|---|---|---|
| B/D 品种检测 | `fruit_type/best.onnx` | YOLO26s | `images [1,3,640,640]` | `output0 [1,6,8400]` |
| A/C 品种检测 | `vegetable_type/best.onnx` | YOLO26s | `images [1,3,640,640]` | `output0 [1,9,8400]` |
| 可选统一熟度 | `ripeness/unified/best.onnx` | YOLO11n-cls | `images [N,3,H,W]`，实际用 224×224 | `output0 [N,2]`，已含 Softmax |
| 默认单品熟度 | `ripeness_edge/<crop>/best.onnx` | MobileNetV3-Small | `images [N,3,224,224]` | `output [N,2]`，logits |

以上 ONNX 均为 opset 17。检测器是 **raw 输出，end2end=False，未内置 NMS**，不能当 `[1,300,6]` 解析，也没有独立 objectness 通道。

类别顺序必须按模型本身使用，不沿用旧 A/B/C 的枚举：

```text
fruit_type:     0=apple, 1=pear
vegetable_type: 0=bad, 1=onion, 2=pepper, 3=pumpkin, 4=tomato
熟度模型:       0=ripe, 1=unripe
```

原始检测输出前四通道是 `cx, cy, w, h`，随后是各品种分数。选择最大类别分数，置信度过滤、转换 xyxy、执行 NMS。`bad` 不做熟度分类，不进入正常果蔬发布/抓取链路。

本地 Ultralytics 默认 `agnostic_nms=False`，按类别执行 NMS；RC4 当前一次 `cv::dnn::NMSBoxes()` 会跨类别抑制。迁移时复用框解码，但 NMS 应按品种分组，避免重叠 apple/pear 或 bad/正常蔬菜互相抑制。

离线评估参考：水果 mAP50=94.38%、mAP50-95=80.41%；蔬菜 mAP50=98.23%、mAP50-95=83.13%。统一熟度在 220 张测试图上准确率 100%；六个 Edge 各自在 19/24/30/59/48/40 张测试图上为 100%。这是有限测试集结果，不是 RC4 实景正确率或速度承诺；评估来源为本地候选批次 `metrics.json`。

## 4. 与 RC4 现状的关键差异

RC4 当前依赖 OpenVINO **2024.6.0**、OpenCV **4.5.4**，只允许 GPU 推理，不自动回退 CPU。

| 项目 | RC4 当前 | 目标 |
|---|---|---|
| A | A.onnx 检测熟度，发布 ripe/unripe | vegetable_type → 熟度 → 品种_熟度 |
| B | B.onnx 直接检测 apple/pear_熟度 | fruit_type → 熟度 → 品种_熟度 |
| C | C.onnx 直接检测蔬菜_熟度 | vegetable_type → 熟度 → 品种_熟度 |
| D | 接收赛区字符串，但没有推理请求路由 | 与 B 共用水果检测与候选策略 |
| 相机 | SIDES 仅 A，TREE 使用 camera | 新链路仅 camera，SIDES 不适配 |
| 辣椒 | chili_ripe / chili_unripe | pepper_ripe / pepper_unripe |

现有 `initialize_model()` 只接受检测器 `[1,3,640,640]` 和旧标签集，**不可直接用于 224 分类模型**。现有 parser 的 raw 解码可复用，但旧水果标签数 4、蔬菜标签数 8，必须更新为新品种标签数 2、5。旧 A 标签数恰好也是 2，不能因形状相同而把 apple/pear 错当 ripe/unripe。

## 5. 推荐最小识别流程

```text
TREE + 当前区域 A/B/C/D
  → 接收同一帧 camera RGBD
  → A/C 选择 vegetable_type，B/D 选择 fruit_type
  → 原图 RGB8 → 640 letterbox → 品种检测 → NMS → 过滤 bad
  → 检测框映射回原图 → 原图 ROI 每边外扩框宽/高的 10%
  → 默认按品种选择 Edge；可选统一 YOLO
  → 熟度 argmax → 组成 品种_熟度
  → 发布识别与深度信息 → 原有目标筛选/锁定
```

### 5.1 一阶段预处理与坐标

复用 `letterbox_image()`、`to_original_box()`：保持宽高比缩放到 640，居中填充 `(114,114,114)`，RGB、f32 NCHW、除 255，不做 ImageNet 归一化。现有 OpenVINO u8 NHWC → f32 NCHW 的融合预处理可以保留。

RC4 已通过 `cv_bridge` 得到 **RGB8**。本地 `infer.py` 的 OpenCV 帧是 BGR，才需要转 RGB；RC4 不可照抄 `[:, :, ::-1]` 再反转一次。

先把框还原到原图，再从未画框的原始 RGB 图裁剪；外扩框用于分类，不替代原始检测框。深度取点继续使用原始框中心与同帧对齐深度，不使用外扩框或 224 图的坐标。边界裁剪到图像范围；非有限坐标、空框、裁剪后任一边小于 8 像素的 ROI 不分类。

### 5.2 默认 Edge 预处理

对合法 ROI 直接 `cv::resize` 到 224×224，线性插值，RGB 转 f32 NCHW：

```text
x = RGB / 255
x[R] = (x[R] - 0.485) / 0.229
x[G] = (x[G] - 0.456) / 0.224
x[B] = (x[B] - 0.406) / 0.225
```

按 detector 的 **品种名** 选择对应模型。结果是 logits，执行数值稳定 Softmax：`exp(z-max(z))/sum(exp(z-max(z)))`；再 argmax，0→ripe、1→unripe，熟度置信度是所选概率。

本地支持同品种 ROI 动态 batch。为了最小改动，RC4 首版可先把分类输入 reshape 为 `[1,3,224,224]`，逐 ROI 推理；验证正确后再按品种分组批处理，不必为了动态 batch 改写整个调度器。

### 5.3 可选统一 YOLO 预处理

不能复用 Edge 的归一化或直接方形拉伸：本地 Ultralytics 分类预处理是 **短边 resize 到 224 → 中心裁剪 224×224 → RGB f32 NCHW /255**，默认 mean=(0,0,0)、std=(1,1,1)。ONNX 已含 Softmax，不再重复 Softmax。

核对依据为本地安装包的 `classify_transforms()` 和 `ClassificationPredictor.preprocess()`。C++ 的 resize 舍入、中心裁剪与 PIL/torchvision 插值可能有细微差异，必须用相同 ROI 比对两端概率后才能称为一致。不要把 Edge 的 ImageNet 均值方差套到统一 YOLO 上。

### 5.4 参数、置信度与失败处理

- 默认引擎 `edge`，参数仅接受 `edge/yolo`。第一版可启动时选择；运行时切换不是本轮必需功能，切换前必须确保目标模型已加载。
- 保留 RC4 检测阈值 0.50、NMS IoU 0.45，避免额外改变抓取评分；本地 `infer.py` 的检测默认值是 0.35，后续实景对比时显式使用相同阈值。
- 熟度分类输出仍按 argmax 二分类。本地 `--ripeness-conf=0.50` 虽然存在，但 `process_frame()` 没使用该门限；不要把本地参数当成 RC4 门控依据。RC4 目标确认现在单独使用源码 YAML 的 `camera_target_maturity_confidence_threshold`（起始值 0.70），低于该概率的分类结果可以发布诊断，但不支持采收确认，详见 `docs/TWO_STAGE_RUNTIME.md`。
- 保留 `FruitDetection.confidence` / `FruitTarget.confidence` 为 **检测置信度**，防止悄悄改变锁定排序。熟度置信度在内部结果、日志或标注中单独保留；本轮不扩展 ROS 消息。
- 启动时检查所选权重存在、类别契约、输入/输出维度、f32 类型和 GPU 可用性。所选引擎必需模型缺失或编译失败，应明确拒绝启动新链路，不能默默回退旧模型/CPU。
- 某 ROI 无效、推理失败或概率非有限时，记录错误并排除该目标；其他合法目标可继续发布，无目标时发布空结果。不得照搬本地缺 Edge 模型时伪造 `unripe,0.50` 的占位逻辑。
- 不把空结果误当“保留上一次目标”。具体任务端的缓存时效和清空行为需要验收，不能因模型更换而使用过期抓取点。

## 6. RC4 代码更新落点（本次不执行）

尽量在共同链路插入分类，不分别复制 A/B/C/D 实现：

1. `include/fruit_perception/detection_parser.hpp`、`src/detection_parser.cpp`：增加/替换新品种标签集，复用 raw YOLO 解码、按品种执行 NMS，过滤 bad；保留张量形状、坐标和分数校验。
2. `src/fruit_perception_node.cpp`：加载两检测器和所选分类器；分类模型单独做 224 输入检查，不强塞进旧 640 初始化器。`select_camera_request()` 统一 A/C→蔬菜、B/D→水果；在 `finish_inference()` 检测后、`make_detection_outputs()` 与目标选择前完成 ROI 分类和标签组装。
3. 继续复用 `image_processing.cpp` 的 letterbox/逆变换，分类加最小 ROI 预处理即可；`target_localization.cpp` 不改深度几何。
4. 继续使用已有 OpenVINO runtime，不引入 ONNX Runtime / PyTorch 运行依赖。编译模型可共享，**可变 InferRequest/输入缓存不能并发共用**；ROI 指针和 RGBD 必须保持有效，直到两阶段完成。
5. 两阶段结束后再释放 `stream.busy`、发布结果。沿用切区/切模式的 generation 检查，在分类后、发布前丢弃旧区域在途帧；异常也必须释放 busy，停止时等待所有仍在途的请求。
6. A/C 保留单目标选择器；B/D 复用 B 的最多 2 候选策略，不改关联半径、采集帧数、锁定时长。两类熟度都可作为识别结果，但实际抓取仍按任务成熟目标规则过滤，不因标签更新自动抓未熟果。
7. `config/fruit_perception.yaml`、必要时 `launch/fruit_perception.launch.py`：后续配置新模型根路径和 `ripeness_model=edge`，停用左右相机、保留 camera；不再用 A/B/C 旧模型参数表示新逻辑。现有 CMake 已安装整个 `model` 目录，递归模型路径无需另建安装机制。

模型路径应从包资源目录或显式绝对路径解析，不能依赖启动工作目录。旧模型可保留归档，但不要在新链路中自动兼容回退。

## 7. 对外标签与 fruit_mission 联动

保留 `FruitDetection` / `FruitTarget` 的消息结构、已有 topic、header/坐标系。`FruitDecision` 已删除 `source` 字段及来源枚举常量。

合法标签为：`apple_ripe/unripe`、`pear_ripe/unripe`、`onion_ripe/unripe`、`pepper_ripe/unripe`、`pumpkin_ripe/unripe`、`tomato_ripe/unripe`（此处斜杠表示两个标签，不是实际字符串）。`types`、`detections[].type`、`targets[].type` 必须使用同一命名规则；不可输出 bad、chili 或孤立 ripe/unripe。

只更新感知包还不足以让现有任务流程直接运行：

- `fruit_mission/fruit_mission/decisions.py`（原 `logic.py`，比赛编排现为 `Logic.py`）：`PRODUCE_TYPE_PREFIXES` 中“辣椒”从 `chili` 改为 `pepper`，连动反向语音映射、`expected_ripe_type()` 及测试。
- `/fruit_mission/expected_fruit_type` 当前是精确字符串过滤；发送端和接收端同步为新复合标签。A 区不能继续发单独 `ripe`，否则检测出 `onion_ripe` 也会被排除；没有指定品种时按合法熟度后缀筛选，不能用包含 `ripe` 的子串判断（unripe 也包含 ripe）。
- 现有 `choose_maturity_side()` 只认字符串 `ripe`，现有任务流程仍消费左右相机。camera-only 需要任务端改为消费 `_latest_camera_types` / 深度目标，并在需要推理时进入 TREE，不是 QR 或 SIDES。**不继续扩展侧相机逻辑，也不在本轮删除它。**
- `mission_context.py::_on_decision()` 已有 camera 结果入口，可复用；核对无目标、异常和模式切换后的缓存失效。
- 感知支持 D 不等于导航任务已有 D 流程。后续任务端负责发 current_area=D、进入 TREE，以及是否消费 B 样式 targets；本说明不新增 D 导航路线、扫码任务或机械臂动作。
- 同步更新 `fruit_mission/test/test_logic.py`、任务流程相关测试和感知包标签/路由测试；不改无关硬件接口。

## 8. 校验、验收与回退

### 8.1 归档校验（不运行 ROS）

在 RC4：

```bash
cd /home/rc4/fruit_ws/src/fruit_perception/model/train_20260930_migration_20261002
sha256sum -c MODEL_SHA256SUMS.txt
```

清单仅覆盖全部模型文件。`OPENVINO_SMOKE_RESULTS.json` 记录九个新流程 ONNX 的 RC4 GPU 编译、batch=1 零输入推理结果；它不代表摄像头端到端验收，也不验证历史权重的可运行性。动态分类器的首版检查先固定到 batch=1，原 ONNX 文件不重写。

本次实测（2026-10-02）：**36/36 模型文件 SHA256 一致，9/9 新流程 ONNX 均通过 OpenVINO 2024.6.0 的 GPU.0 编译及 batch=1 零输入推理，输出形状正确且数值有限；统一 YOLO 概率和为 1。** 旧 A/B/C 权重、配置、核心代码与 CMake 的基线哈希未变。未测试摄像头、两阶段预处理一致性、真实分类准确率及动态多 batch 性能。

### 8.2 后续实现验收

1. 先单测两检测器类别顺序和 raw 输出，尤其蔬菜 class 0=bad、class 2=pepper；验证 NMS 和框逆变换。
2. 同一真实 ROI 对比本地与 RC4：Edge RGB/ImageNet 预处理、统一 YOLO 的短边缩放/中心裁剪、类别方向、概率和标签一致；分别测试六个品种及 ripe/unripe。
3. RGBD 端到端测试 A/C 蔬菜、B/D 水果；只使用 camera，OFF 不推理、QR 不发布果蔬识别、TREE 正常推理；全区预期数量=2 仅提示。
4. 测试多框、零框、bad、贴边/小 ROI、缺权重、GPU 不可用、分类异常。异常不能生成假未熟目标，未熟识别不能自动变成可抓取目标。
5. 测试切区/切模式时在途帧不会串区，停止不挂起；框、RGB/深度时间戳、点坐标不变。
6. 验证任务端收到 pepper_ripe 和 A 区复合标签能正确匹配/播报，空结果或超时不沿用旧抓取点；B/D 上限=2，A/C 单目标锁定不变。
7. 真机记录两阶段耗时、FPS、显存/内存和概率差异，不能引用本地“零延迟”或“AVX-512”注释当成 RC4 实测结论。

实现后再构建测试（本次不执行）：

```bash
cd /home/rc4/fruit_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fruit_perception fruit_mission --symlink-install
colcon test --packages-select fruit_perception fruit_mission
colcon test-result --verbose
source install/setup.bash
```

启用前保存配置版本，再按既有启动方式启动，确认日志使用新路径、Edge、GPU 和正确类别数。不要直接编辑 install 下的生成文件。

### 8.3 回退

本次未改旧文件和配置，无需停机回退，也不要重启节点“使归档生效”。后续上线失败时回退**同一版本的感知代码、任务标签适配和配置**，恢复旧 A/B/C 路径后重新构建；不要只回退权重却保留新 parser/新标签。新归档可以保留，不影响旧节点。
