# 工作记录

## 执行记录

### 步骤 1：隔离实施并重新确认基线
- 检查工作区现有未提交修改（用户脏改动包括 README.md, docs/PROJECT_ARCHITECTURE.md 等）。
- 确认 `.worktrees/wisadel-prebuilt-voice` 被 git ignore 覆盖。
- 创建隔离开发分支与 worktree：
  `git worktree add -b codex/wisadel-prebuilt-voice .worktrees/wisadel-prebuilt-voice main`
- 验证 `ubuntu-remote` GPU 服务器 SSH 与 Python 环境可达。
- 检查已有素材 `sounds/fruit_success.wav` 格式为 16-bit mono 32000 Hz PCM。

### 步骤 2：完成统一文案与素材清单
- 创建素材清单配置文件 `src/fruit_mission/config/wisadel_voice.yaml`，共包含 161 条短语条目（涵盖起点、流程结束、6种果蔬品种、1~32采摘数量词、B区树号前缀、B区二维码整项短语、C区项号、C区位置名、C区前缀、识别动作、成熟/未成熟类型、各种跳过与失败提示短句）。
- 修改 `decisions.py`：
  - 新增 `format_b_tree_name(position_id)`：规范为“第N棵树”（一至八）。
  - 更新 `format_b_announcement` 和 `format_c_announcement` 使用分号分隔独立项。
  - 新增 `format_produce_count(produce, count)` 供汇总播报。
- 修改 `Logic.py`：统一 B 区树号标签与导航失败播报文案。
- 修改 `actions.py`：统一树号标签为 `format_b_tree_name`，移除 Piper 专用的 `_qr_length_scale`。
- 修改 `mission_node.py`：起点台词规范为“我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘”。

### 步骤 3：替换播放器并提供最小离线检查工具
- 编写离线工具 `scripts/prepare_wisadel_voice.py`，支持 `check`、`generate`、`render` 子命令，提供标准 PCM 验证与贪婪最长匹配分词拼接能力。
- 替换 `src/fruit_mission/fruit_mission/tts.py`：
  - 用 `RecordedSpeaker` 替换 `PiperSpeaker`（保留 `PiperSpeaker` 作为别名）。
  - 基于标准库 `wave` 进行离线 PCM 拼接，写入原子临时文件后替换。
  - 单 worker 线程 FIFO 串行播放；多项长摘要支持分项排队与独立限时。
  - `close()` 支持立即终止当前 `aplay` 子进程并清空待播队列。
- 运行 `/usr/bin/python3 scripts/prepare_wisadel_voice.py check --manifest src/fruit_mission/config/wisadel_voice.yaml --text-only`：全部代表场景文本分词通过。

### 步骤 4：直接批量生成并核验素材
- 远程在 `ubuntu-remote` 上调用 GPT-SoVITS 以 `gremlin`（魔性鬼畜版氦气变声）音色批量合成 161 条音频文件。
- 全部生成的音频文件保存于 `src/fruit_mission/sounds/wisadel/`。
- 运行 `/usr/bin/python3 scripts/prepare_wisadel_voice.py check`：全部 161 个 WAV 文件均存在且符合 32kHz 单声道 16-bit PCM 规范。
- 运行 `/usr/bin/python3 scripts/prepare_wisadel_voice.py render --output-dir /home/rc4/fruit_ws/tmp/wisadel_voice_preview`，生成 10 个代表场景试听文件（intro 5.10s, flow_finish 1.89s, b_qr 15.39s, c_qr 29.15s, b_success 13.13s, b_skip 23.93s, c_success 16.68s, c_skip 26.53s, summary_zero 3.86s, summary_multi 10.40s）。

### 步骤 5 & 步骤 6：接入 mission、添加计数与主流程最后的汇总函数、删除旧 Piper 配置
- 在 `actions.py::_grasp_and_collect` 中，仅在 `home.completed` 确认成功时调用 `ctx.record_pick(matched_type)`。
- 在 `mission_context.py` 中增加 `self._pick_counts = Counter()`、`record_pick()` 及 `announce_summary()`；移除 `length_scale` 参数。
- 在 `mission_node.py::FruitMissionNode.run` 中，正常流程结束感知关闭后直接调用 `ctx.announce_summary()`；`finally` 中仅做安全清理与立即停播，异常或 Ctrl-C 退出不汇总。
- 修改 `config/fruit_mission.yaml`：移除旧 piper 路径与 `qr_length_scale`，更新为 `manifest_path` 和 `aplay_executable`。
- 修改 `config.py`：移除 `qr_length_scale` 校验。
- 修改 `start_fruit_mission.sh`：将 Piper 可执行文件与模型检查替换为素材清单 `VOICE_MANIFEST` 与 `APLAY_EXECUTABLE` 可用性检查。
- 代码语法与风格检查：
  - `/usr/bin/python3 -m compileall -q` 通过。
  - `ament_flake8` 针对所有修改的 7 个 Python 文件检查通过（No problems found）。
  - `colcon build --base-paths src --packages-select fruit_mission` 编译成功。

### 步骤 7：同步文档并合入主工作区
- 同步更新文档：`docs/VOICE_FLOW.md`、`docs/PROJECT_ARCHITECTURE.md::7.4`、根 `README.md`、`src/fruit_mission/README.md`、`docs/RULE.md`。
- 将 worktree 分支的修改无冲突合并至主工作区，保留工作区所有原有的未提交修改。
- 移除临时分支与 worktree `codex/wisadel-prebuilt-voice`。

### 步骤 8：安全删除旧 Piper 本机环境与最终核验
- 核对目标绝对路径：`/home/rc4/fruit_ws/.venv/piper-tts` 与 `/home/rc4/fruit_ws/.cache/piper-tts`。
- 检查运行进程与打开文件：确认无活跃 Piper 进程或引用。
- 执行显式删除：`rm -rf /home/rc4/fruit_ws/.venv/piper-tts /home/rc4/fruit_ws/.cache/piper-tts`。
- 删除后校验：`test ! -e` 确认两个目录已彻底清理。
- 再次执行完整离线检查与编译：
  - `python3 scripts/prepare_wisadel_voice.py check`：通过。
  - `python3 scripts/prepare_wisadel_voice.py render --scenario summary_multi`：成功生成试听音频。
  - `colcon build --base-paths src --packages-select fruit_mission`：通过。

## 待人工确认事项
- 交付后人工试听：用户可在连接扬声器的机器人终端运行 `python3 scripts/prepare_wisadel_voice.py render --scenario <name> --play` 试听场景效果。
