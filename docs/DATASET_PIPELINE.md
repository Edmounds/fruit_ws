# 水果数据集流水线

本项目使用 `src.dataset_pipeline` 将多个 YOLO 检测数据集统一成可训练的 12 类水果数据集。流水线不会修改原始目录，所有结果保存在 `fruit_database/builds/<版本名>/`。

## 采集训练图像

在工作空间根目录加载统一的 ROS 运行环境，然后选择一台相机启动彩色图像采集 GUI：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash


# 左侧 USB 相机：/dev/left_cam
python3 collect_cam_img.py --cam 1

# 右侧 USB 相机：/dev/right_cam
python3 collect_cam_img.py --cam 2

# D435i 彩色相机
python3 collect_cam_img.py --cam 3
```

GUI 左侧显示相机的实时原始彩色画面，右侧“截图”按钮在收到第一帧后启用。
截图保存为未缩放、无标注的无损彩色 PNG，目录分别为：

```text
/home/rc4/fruit_ws/img_output/cam1/
/home/rc4/fruit_ws/img_output/cam2/
/home/rc4/fruit_ws/img_output/cam3/
```

文件名使用本地时间戳，例如 `20260823_143012_527.png`。脚本会优先复用目标话题上
已经运行的相机 publisher；没有 publisher 时，只启动 `--cam` 选中的相机。关闭窗口或
按 `Ctrl-C` 后，脚本只停止由自己启动的相机进程，不会停止复用的现有相机进程。

## 类别定义

| ID | 类别 |
|---:|---|
| 0 | `apple_ripe` |
| 1 | `apple_unripe` |
| 2 | `chili_ripe` |
| 3 | `chili_unripe` |
| 4 | `pumpkin_ripe` |
| 5 | `pumpkin_unripe` |
| 6 | `onion_ripe` |
| 7 | `onion_unripe` |
| 8 | `pear_ripe` |
| 9 | `pear_unripe` |
| 10 | `tomato_ripe` |
| 11 | `tomato_unripe` |

## 当前清洗规则

- `Fruit` 中的 `lajiao` 统一为 `chili`，`nangua` 统一为 `pumpkin`。
- `fruit.yolov11` 中拼错的 `pumpking` 统一为 `pumpkin`。
- `fruit.yolov11` 的 `pear` 没有成熟度。包含该类别的 40 张图片整张排除，不能只删梨框，否则图片中的梨会变成漏标目标。
- `IMG_20250321_201856_jpg.rf.fxQyeazyGJ2Yc3JzVhid` 有红辣椒但标签为空，整张排除。
- `序列 010624_jpg.rf.Njka9gf4x4d9VB7YbFY2` 是确认过的无水果背景图，空标签允许保留。
- `classes.txt` 不是 YOLO 图片标签，扫描时忽略。
- 同一原图的 Roboflow 增强版本按来源分组后再做 `80/10/10` 划分，禁止同一组跨越 train、val、test。

## 安装和命令

当前脚本只依赖 Pillow 和 PyYAML：

```bash
python3 -m pip install -r requirements-dataset.txt
```

只检查，不生成数据：

```bash
python3 -m src.dataset_pipeline --config configs/datasets.yaml audit
```

生成清洗数据集：

```bash
python3 -m src.dataset_pipeline --config configs/datasets.yaml build --run-name v1
```

一次生成清洗层和均衡增强层：

```bash
python3 -m src.dataset_pipeline --config configs/datasets.yaml all --run-name v1
```

为已经存在的 clean 构建补充 balanced 层：

```bash
python3 -m src.dataset_pipeline --config configs/datasets.yaml balance \
  --run fruit_database/builds/v1
```

同名版本默认禁止覆盖。确认需要重建时，在具体子命令后添加 `--force`。全局的 `--seed` 必须放在子命令之前，例如：

```bash
python3 -m src.dataset_pipeline --seed 20260725 all --run-name v2
```

## 输出结构

```text
fruit_database/builds/v1/
├── clean/
│   ├── images/{train,val,test}/
│   ├── labels/{train,val,test}/
│   └── data.yaml
├── balanced/
│   ├── images/{train,val,test}/
│   ├── labels/{train,val,test}/
│   └── data.yaml
└── reports/
    ├── audit.json
    ├── validation.json
    ├── class_counts.csv
    ├── removed_items.csv
    ├── duplicate_groups.csv
    ├── split_manifest.csv
    └── augmentation_manifest.csv
```

`clean/data.yaml` 用于评估原始清洗数据，`balanced/data.yaml` 用于正式训练。验证集和测试集在两个层中相同，只有 balanced 的训练集增加离线增强图片。

## 均衡和增强

均衡目标是让训练集中 12 个类别的标注框数量精确等于当前最大类别。为避免多目标图片同时增加多个类别，补齐样本只从对应类别的单目标图片生成。

默认增强包括：

- `-15°` 到 `+15°` 旋转；
- 水平翻转；
- `0.90-1.10` 缩放和最大 `5%` 平移；
- 高斯模糊；
- 亮度、对比度、饱和度和 Gamma 增益。

几何增强会同步转换标注框。框可见面积不足、尺寸过小或越界时会重新抽取参数。所有随机参数和来源记录在 `augmentation_manifest.csv`。

## 添加新数据集

1. 将新数据集放在工作区内，保持 `images/` 和 `labels/` 的 YOLO 目录结构。
2. 在 `configs/datasets.yaml` 的 `sources` 中增加配置，不要修改脚本。
3. 为新数据集的每个原始类别 ID 填写统一类别名。
4. 无法可靠映射的类别使用 `__drop_image__`，这样会排除整张图片而不是只删除目标框。
5. 已确认的背景空标签放入 `allow_empty_stems`；已确认的漏标或错误图片放入 `exclude_stems`。
6. 先运行 `audit` 检查删除原因和数量，再用新的 `run-name` 执行 `all`。

新增数据后必须新建版本，不要向旧构建目录直接复制文件。这样 split、类别统计和增强数量才能根据所有数据重新计算。
