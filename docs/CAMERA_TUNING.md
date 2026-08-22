# 摄像头实时调参

`test_camera_detector.py` 会同时打开 YOLO 检测画面和摄像头参数窗口。参数修改后立即作用于当前摄像头，可直接观察检测框和置信度的变化。

## 启动

所有命令都在工作空间根目录执行，并先加载 ROS 运行环境：

```bash
cd /home/rc4/fruit_ws
source setup_fruit_runtime.bash
```

按要调整的摄像头选择一条命令：

```bash
# 左侧 USB 摄像头：/dev/left_cam
python3 test_camera_detector.py --cam1 --model B

# 右侧 USB 摄像头：/dev/right_cam
python3 test_camera_detector.py --cam2 --model C

# D435i 彩色摄像头
python3 test_camera_detector.py --cam3 --model A
```

`--model` 必须是 `A`、`B` 或 `C`，需要测试其他模型时替换示例中的字母。

脚本默认把窗口显示到机器人连接的桌面屏幕。检测窗口按 `q` 或 `Esc`，或者关闭参数窗口，即可退出。
正常退出会停止脚本自己启动的相机进程。如果窗口或父脚本被强制终止，独立的
`test_camera_detector_*_container` 可能继续占用摄像头；返回工作空间根目录执行
`./kill_fruit_processes.sh` 可清理该容器及其 `ros2 component standalone` 包装进程。

## 常用参数

参数窗口只显示当前摄像头实际支持的项目和范围：

| 参数 | 作用 |
| --- | --- |
| Auto exposure | 自动曝光 |
| Exposure | 曝光时间；过大会造成运动拖影 |
| Gain | 图像增益；过大会增加噪点 |
| Auto white balance | 自动白平衡 |
| White balance | 手动调整色温 |
| Brightness | 亮度 |
| Contrast | 对比度 |
| Saturation | 饱和度 |
| Sharpness | 锐度 |
| Gamma | 中间亮度层次 |
| Backlight compensation | 逆光补偿 |
| Power line frequency | 灯光防闪烁频率 |

拖动滑块、切换勾选框或选择频率后，参数会立即写入当前摄像头。手动修改 `Exposure` 或 `White balance` 时，对应的自动模式会自动关闭。

## 推荐调节顺序

1. 把机器人和目标放到比赛时的实际距离，保持检测画面持续可见。
2. 室内灯光稳定时先关闭自动曝光，调低 `Exposure` 到运动时没有明显拖影，再用 `Gain` 补足亮度。
3. 颜色偏差明显时关闭自动白平衡，再调 `White balance`，使不同类别的颜色接近训练数据。
4. 少量调整亮度、对比度、饱和度和锐度，同时观察漏检、误检和置信度，不要只追求画面鲜艳。
5. 室内交流灯出现横纹或闪烁时调整 `Power line frequency`；中国大陆通常使用 `50 Hz` 对应的设备选项。
6. 在目标静止、机器人运动和背景干扰三种情况下都确认效果，再保存参数。

一次只调整一个主要参数，便于判断检测变化来自哪里。曝光和增益通常比锐度、Gamma 等后处理参数更优先。

## 保存和恢复

确认效果后点击参数窗口底部的 `Save`。成功保存时，窗口状态栏会显示备份文件名，终端也会输出完整路径。

当前值写入源工作空间配置：

```text
/home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml
```

每次保存前，原配置会完整备份到：

```text
/home/rc4/fruit_ws/tmp/fruit_perception_YYYYMMDD_HHMMSS_ffffff.yaml.bak
```

需要恢复时，先停止感知程序，找到所需备份并覆盖配置文件，例如：

```bash
cp /home/rc4/fruit_ws/tmp/fruit_perception_YYYYMMDD_HHMMSS_ffffff.yaml.bak \
  /home/rc4/fruit_ws/src/fruit_perception/config/fruit_perception.yaml
```

保存只持久化已经成功应用到摄像头的值。未点击 `Save` 时，本次实时调整只对当前运行有效，重新启动后仍使用配置文件中的值。
