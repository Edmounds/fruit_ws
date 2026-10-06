---
name: wisadel-helium-voice
description: Generate Wis'adel (维什戴尔) character voice lines with demonic helium pitch-shifting (魔性鬼畜版氦气变声) for the fruit robot in fruit_ws. Use whenever you need the robot to speak, play voice prompts, announce fruit harvesting status, celebrate missions, or sound alerts.
---

# Wis'adel Demonic Helium Voice Generator (维什戴尔魔性鬼畜氦气语音)

## 概述

本技能为 `fruit_ws` 水果采摘机器人项目专用，用于为机器人生成明日方舟**维什戴尔（Wis'adel / W）**标志性的**魔性鬼畜版氦气变声**语音。

底层基于在 152 轨官方竹达彩奈（Ayana Taketatsu）纯净干声上微调训练的 **GPT-SoVITS v2** 模型，并通过声学共振峰拉升与时间伸缩算法（WSOLA）实时生成高亢、尖锐、极具小恶魔调侃感的鬼畜音效，直接输出为机器人可播放的标准 32kHz WAV 音频。

---

## 快速上手

在 `/home/rc4/fruit_ws` 目录下直接调用现成脚本：

```bash
cd /home/rc4/fruit_ws

# 1. 生成魔性鬼畜版氦气语音（默认模式，极力推荐！）
python3 scripts/generate_wisadel_voice.py \
  --text "果实采摘完成！哈哈哈哈，这就搞定了？真没劲！" \
  --mode gremlin \
  --output sounds/fruit_picked.wav

# 2. 生成经典纯正氦气版（保持原语速，像吸了氦气气球一本正经说话）
python3 scripts/generate_wisadel_voice.py \
  --text "喂，前面的障碍物，赶快给本小姐让开！" \
  --mode classic \
  --output sounds/obstacle_alert.wav

# 3. 生成后立即在机器人扬声器上播放试听（添加 -p / --play 参数）
python3 scripts/generate_wisadel_voice.py \
  --text "采摘任务开始，看好了，可别眨眼喔！" \
  --play
```

---

## 参数说明

`scripts/generate_wisadel_voice.py` 支持以下命令行参数：

| 参数 | 缩写 | 默认值 | 可选值 / 说明 |
| :--- | :--- | :--- | :--- |
| `--text` | `-t` | *(必填)* | 需要合成的文本内容（完美支持中文、英文、数字、中英混读） |
| `--output`| `-o` | `sounds/wisadel_helium.wav` | 本地生成的 WAV 文件保存路径 |
| `--mode` | `-m` | `gremlin` | **`gremlin`**（魔性鬼畜版，共振峰上移 1.42x，语速加快 15%）<br>**`classic`**（经典纯正版，共振峰上移 1.35x，保持原速）<br>**`dry`**（微调原声，无变声） |
| `--lang` | `-l` | `zh` | `zh`（中文/混读）、`en`（英文）、`ja`（日文）、`auto`（自动检测） |
| `--play` | `-p` | `False` | 生成完毕后是否立即使用 `aplay` / `paplay` 播放 |

---

## 常用采摘机器人台词预设表

| 场景 | 推荐台词 | 推荐模式 | 建议保存路径 |
| :--- | :--- | :--- | :--- |
| **任务启动** | `采摘任务开始，看好了，可别眨眼喔！` | `gremlin` | `sounds/mission_start.wav` |
| **目标锁定** | `发现成熟苹果！锁定目标，看我一击必中！` | `gremlin` | `sounds/target_locked.wav` |
| **障碍物警报** | `喂，前面的障碍物，赶快给本小姐让开！Boom！` | `gremlin` | `sounds/obstacle_warn.wav` |
| **采摘成功** | `果实采摘完成！哈哈哈哈，这就搞定了？真没劲！` | `gremlin` | `sounds/fruit_success.wav` |
| **全场通关** | `所有果实收集完毕！博士，今天的活干完了，快去买冰淇淋！` | `gremlin` | `sounds/all_clear.wav` |

一键批量预生成常用库示例：
```bash
python3 scripts/generate_wisadel_voice.py -t "采摘任务开始，看好了，可别眨眼喔！" -o sounds/mission_start.wav
python3 scripts/generate_wisadel_voice.py -t "果实采摘完成！哈哈哈哈，这就搞定了？真没劲！" -o sounds/fruit_success.wav
python3 scripts/generate_wisadel_voice.py -t "喂，前面的障碍物，赶快给本小姐让开！Boom！" -o sounds/obstacle_warn.wav
```

---

## 在 ROS 2 节点中集成与播放

### 1. Bash / 启动脚本中播放
在 ROS 2 任务启动或采摘完成回调脚本中：
```bash
aplay /home/rc4/fruit_ws/sounds/fruit_success.wav &
```

### 2. Python 节点中调用
在 `src/fruit_mission/` 或其他 Python 节点中异步播放：
```python
import subprocess

def play_wisadel_voice(wav_name: str):
    wav_path = f"/home/rc4/fruit_ws/sounds/{wav_name}"
    subprocess.Popen(["aplay", "-q", wav_path])

# 在采摘成功动作完成时调用：
play_wisadel_voice("fruit_success.wav")
```

---

## 架构与声学原理备忘

1. **分布式 GPU 算力协同**：
   * `fruit_ws` (rc4-remote) 通过免密 SSH 远程调度 RTX 3060 GPU 服务器 (`ubuntu-remote:~/GPT-SoVITS`)。
   * GPU 端基于微调权重（`wisadel_v2_ft-e15.ckpt` 与 `wisadel_v2_ft_e8_s624.pth`）高速完成自回归 Token 与变分声码器推理（约 2~3 秒）。
2. **氦气声学共振峰升频（FFmpeg Filter）**：
   * 人吸入氦气后，声带基频几乎不变，但共振峰（Formants）整体上浮。
   * 单纯 Pitch Shift 仅类似飚高音；真正的“氦气音”必须使用**重采样压缩 + WSOLA 时间拉伸**：
     * **魔性鬼畜版 (`gremlin`)**：`asetrate=32000*1.42,atempo=1.15/1.42,aresample=32000`
     * **经典纯正版 (`classic`)**：`asetrate=32000*1.35,atempo=1/1.35,aresample=32000`
