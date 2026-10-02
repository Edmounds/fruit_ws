# 训练图像采集与数据环境

当前工作区提供 `collect_cam_img.py` 图像采集工具。旧 `src.dataset_pipeline`、
`configs/datasets.yaml`、`requirements-dataset.txt` 和 `fruit_database/` 已不存在，
因此本页不再提供不可执行的清洗、划分、均衡增强命令。

## 采集训练图像

在工作区根目录加载 ROS 运行环境，启动单 camera 彩色图像采集 GUI：

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
python3 collect_cam_img.py
# 可选：自定义保存目录
python3 collect_cam_img.py --output-dir /home/rc4/fruit_ws/data/yolo_images/camera
```

默认目录为 `/home/rc4/fruit_ws/data/yolo_images/camera/`，文件名使用本地时间戳，
例如 `frame_20260823_143012_527.jpg`。两个示例是替代用法，不应同时启动。

脚本优先复用图像话题的现有 publisher；没有 publisher 时自动启动 camera 组件。
关闭窗口或按 `Ctrl-C` 后，只停止自己启动的相机进程，不停止复用的已有进程。
窗口默认显示到机器人实际连接的桌面。预览复用同一个 Canvas 图像对象；保存时
检查写入返回值，写入失败显示错误，不提示已保存。

## 标签与训练边界

- 当前模型为“物种检测 + 熟度分类”，旧 12 类检测编号不是当前 ONNX 类别契约。
  模型路径、预处理与类别方向见[两阶段运行文档](../src/fruit_perception/docs/TWO_STAGE_RUNTIME.md)。
- 原始数据应保留；划分训练/验证/测试集时，同一原图及其增强版本不能跨集合泄漏。
- 标注、训练、模型导出使用工作区之外的独立 venv，并在其中维护 requirements/lock。
  禁止在系统 Python 或 `~/.local` 执行 pip 安装。
- ROS 图像采集使用系统 OpenCV 4.5.4 / NumPy 1.x；训练的 pip OpenCV、NumPy 2.x、
  Ultralytics 不得进入 ROS 进程。退出训练环境后重新 source 项目运行环境。
- 本仓库当前没有可核实的数据集清洗流水线入口；如需恢复，先确认工具所在项目和配置，
  不按旧文档凭空创建模块或安装已不存在的 requirements。
