## 目标

使用 `wisadel-helium-voice` 技能预生成维什戴尔 **gremlin** 音频，以本地 WAV 拼接完全替换比赛期间的 Piper 实时合成。比赛运行不联网、不访问 GPU、不调用 TTS 模型。

新增用户明确要求的采摘结束按品种计数汇总：正常主流程最后直接调用汇总播报函数，例如“本次采摘记录：南瓜两个，苹果三个”。不建立汇总调度器。

本计划位于 `/home/rc4/fruit_ws/.pi/plans/2026-10-04T12-26-03-933Z-ecaa6ce2/plan.md`。用户已通过最终 questionnaire 确认理解；当前仍是规划阶段，执行授权必须由扩展另行审批。

## 范围

- 替换已有起点、B/C 二维码、B 逐树判断与失败、C 逐点结果与失败、结束播报；不新增 A 逐点播报、障碍警报或角色庆祝词。
- 新增 A/B/C 共用的**内存按品种计数**和正常结束汇总。用户明确变更原项目“不做收获汇总”的范围限制，但仅增加这个函数，不增加换筐、返回起点或新的结束判定。
- 删除旧 Piper 后端、配置、调用、启动检查和过时文档；新方案验收后删除专用本机虚拟环境及模型缓存。
- 保留并复用现有未跟踪的 `scripts/generate_wisadel_voice.py`、wisadel 技能和 `sounds/fruit_success.wav`。最后一项是已有 wisadel 素材，不是旧 Piper 音频，不覆盖、不误删。
- 不改路线、二维码兜底、目标选择、左右侧、抓取重试、机械臂姿态、运动 API、STOP、实测静止门槛或通信协议。不训练模型、不调整远端服务、不安装依赖。
- 不新增单元测试、源代码扫描测试、私有状态 mock 或实际运动自动测试。使用离线音频检查工具及人工试听。

## 决策

### 用户确认的最终需求

1. 正式音色为 `gremlin`，中文；允许统一短句，但保留位置、顺序、品种、成熟度、匹配、跳过、故障和工序完成事实。
2. 固定短句与完整品种/位置短语预生成，不逐字拼接。二维码按项分段，保持完整顺序。运行期只拼接本地素材。
3. 允许直接使用现有脚本通过 `ubuntu-remote` 批量生成，交付后统一人工试听；不要求批量前试听审批，不自动播放。
4. 起点改为“我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘”，生成读音使用“若博卡普”，日志保留 RoboCup 写法。
5. 比赛期间单后台 FIFO 队列、不重叠。B 摘要异步；C 保留既有提前量语义和调用等待最多 10 秒。长摘要分项播放，每个播放操作独立不超过 10 秒，不把整段摘要截成前 10 秒。
6. 缺失/损坏素材、未知文案或设备故障：整条事实播报失败并告警，比赛继续；不播残缺句、不在线合成、不回退 Piper、不做额外重试。
7. 计数只发生在 CLOSE、驻留、Harvest/OPEN、Home 全部确认完成后。A 使用实际识别品种；B 使用已经匹配、被选中的实际品种；C 使用实际识别品种。每个成功目标加一，失败/跳过不加。B 两个目标分别计数。
8. 汇总只在当前所选 `full/ac/a/b/c` 流程正常遍历结束后调用，作为主流程最后的播报。只播非零品种；全部为零说“本次没有采摘记录”。无需新增持久化统计数据库、JSON 导出或赛后播放流程。
9. 最新决定覆盖前几轮“正常立即停播”和“只生成汇总文件”：**正常结束直接串行调用汇总函数，播完关闭；Ctrl-C/异常立即停播、不汇总**。不增加汇总优先级、抢占、结束清队列或复杂校验。普通播报继续 FIFO；汇总使用同一播放器的同步短段调用。
10. WAV 和清单随源码版本管理，运行读取源码路径。
11. 验证替换成功且没有旧进程使用后，删除 `/home/rc4/fruit_ws/.venv/piper-tts` 与 `/home/rc4/fruit_ws/.cache/piper-tts`，不删除其他虚拟环境、缓存或 worktree。

### 已核对的事实与最小实现

- `src/fruit_mission/fruit_mission/tts.py`：`PiperSpeaker`、`_SpeechJob`、`announce/announce_and_wait/close/_worker_loop/_speak` 已有队列和等待实现，可复用，不创建第二节点或后端选择框架。旧播放用 `subprocess.run(aplay)`，目前不能由 `close` 立即终止；仅改音频进程所有权，不触动机器人 STOP。
- `MissionContext.__init__` 创建 PiperSpeaker；`announce/announce_and_wait` 传递 `length_scale`。`actions.py::_qr_length_scale` 和 YAML 的 `qr_length_scale` 是 Piper 专用，全部删除，保留 `lead_time_sec`。
- `decisions.py::format_b_announcement/format_c_announcement/format_position_name/spoken_model_type` 是文案公共路径。采用清单内有限短语的确定性最长匹配与标点停顿，不加入自由文本推理或复杂语音 DSL。未能完整解析时拒绝整条播报。
- `actions.py::_grasp_and_collect` 是共同工序路径，且只有其成功结果证明 Home 完成。计数放在此成功出口；由 `observe_and_pick` 传入实际 `selection.matched_type`，B 由冻结请求携带对应被选中目标的 `model_type`。不复制工序成功判断。
- `mission_node.py::FruitMissionNode.run` 当前在 `finally` 异步播“任务流程结束”。改为正常路径最后 `ctx.announce_summary()`，异常路径仅保留必要感知关闭与既有安全清理。汇总函数可先播“任务流程结束”，再播品种数据；该表述不意味着全部目标采摘成功。
- `start_fruit_mission.sh` 的 `mission_runtime_paths` 内嵌 Python 和紧随其后的 `PIPER_EXECUTABLE/MODEL_PATH/MODEL_CONFIG` 检查必须同步替换。
- 已读取：`docs/RULE.md`、`docs/VOICE_FLOW.md`、mission README/config/manifest/setup/launch、根 README、架构文档相关部分、生成脚本和技能。`setup.py` 已自动包含 `config/*.yaml`；声音从源码读取，无需新增 ROS 可执行入口或修改 launch。
- 已确认本机 `.venv/piper-tts` 是 Python 3.10 专用 venv，`bin/piper` 存在；`.cache/piper-tts` 仅列出 `zh_CN-huayan-medium`，其中有 `.onnx` 和 `.onnx.json`。`tmp/fruit_mission` 当前不存在，不假定有可删除的 `tts_latest.wav`。
- 历史 worktree 中也有旧 Piper 引用；本任务只迁移主版本。删除环境前必须确认没有旧 worktree 进程正在使用，不能以修改历史 worktree 来扩大范围。

### 拟新增数据与工具

- `src/fruit_mission/config/wisadel_voice.yaml`：短语文本、合成读音（仅需要时）、相对 WAV 路径、二维码分项和离线代表场景。品种沿用现有 `PRODUCE_TYPE_PREFIXES`，不引入第二套比赛类型来源。
- `src/fruit_mission/sounds/wisadel/*.wav`：统一 32 kHz、单声道、16-bit PCM，无背景音乐。先核验技能输出实际格式，不能凭文档声称符合；格式不符停止批量并报告。
- `scripts/prepare_wisadel_voice.py`：唯一新增离线工具，提供 `generate/check/render` 子命令。生成复用现有脚本，检查/渲染复用新的播放器纯音频拼接路径。无 ROS 节点、无运动请求；默认不播放。`render --play` 仅供人工声音验收。
- `tts.py` 替换为 `RecordedSpeaker`，保留同文件的公共队列方式；PCM 拼接使用标准库 `wave`，不增加 FFmpeg 本地依赖、不做实时音色变换或在线语速变换。去掉旧 `length_scale` 参数，gremlin 语速在素材生成时固定。
- 初始素材包括固定台词、六种品种/成熟状态、未知目标/未知果实/其他目标、位置与树号、顺序、匹配/跳过/故障短句、汇总引导、数量短语。当前 A 最多 8 次、B 最多 16 次、C 最多 8 次工序，数量短语覆盖 0～32 足够当前范围；该数量库不是新增任务策略或测试断言，未来扩大采摘范围时同步扩充素材。

## 执行步骤

以下全部为**获批后的步骤**，当前未运行任何命令。每个命令终端先：

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

Shell 脚本遵守 `set -eo pipefail` → source → `set -u`。运行环境校验失败即停止，不用 PYTHONPATH 绕过。没有命令执行能力时报告阻塞，不以其他工具绕过。

### 1. 隔离实施并重新确认基线

- **文件/符号**：计划目录、`.gitignore`（仅检查）、所有拟修改文件；重点保留已脏的根 `README.md` 与 `docs/PROJECT_ARCHITECTURE.md` 中其他改动。
- **改动**：本任务包含播放器、mission 统计和启动链路，按高风险工作使用短期独立 worktree。使用用户当前内容为基线，仅修改本计划范围；不能直接在干净 HEAD 上覆盖当前脏文档。已有未跟踪生成脚本/技能作为用户文件保留并复用。若无法隔离这些既有改动，停下询问，不 stash/reset 用户工作。
- **依赖**：审批后第一个步骤。
- **检查命令**：`git status --short`、`git diff -- README.md docs/PROJECT_ARCHITECTURE.md`、`git check-ignore .worktrees/wisadel-prebuilt-voice`，并按项目 using-git-worktrees 流程创建隔离目录。后续源码检查在 worktree 中运行，但 ROS 运行环境仍 source 根工作区脚本，离线工具显式指定 worktree 清单。
- **期望**：记录新基线；worktree 可安全合并，用户脏改动没有被包含成任务改动或覆盖。
- **停止条件**：目录未被忽略且尚未取得调整位置/忽略规则的同意、重叠改动无法隔离、运行环境不合规。

### 2. 完成统一文案与素材清单

- **文件/符号**：新增 `config/wisadel_voice.yaml`；修改 `decisions.py::format_b_announcement/format_c_announcement`，新增最小 B 树号文案公共帮助函数；`actions.py` 与 `Logic.py::run_b` 仅统一播报标签，不改 navigation 参数/控制分支；`mission_node.py::run` 起点台词。
- **改动**：B 使用明确树号，不让任意航点名直接进入素材匹配；C 摘要明确“第几项、几号位置、品种”，分项边界可直接切分；B 摘要明确树号与品种。所有成熟/不成熟、匹配与失败原语义保留。清单包含全部当前语音分支，以及汇总所需品种与数量短语。RoboCup 日志与合成读音分离。
- **依赖**：步骤 1；文案稳定后才能批量生成。
- **检查命令**：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/decisions.py src/fruit_mission/fruit_mission/actions.py src/fruit_mission/fruit_mission/Logic.py src/fruit_mission/fruit_mission/mission_node.py`；步骤 3 工具就绪后再运行 `prepare_wisadel_voice.py check --manifest <worktree>/src/fruit_mission/config/wisadel_voice.yaml --text-only`。
- **期望**：真实格式函数生成的 B/C 计划与代表分支可以完整映射到清单，文本日志仍表达原事件事实；文本检查不读源码、不模拟 ROS。
- **停止条件**：必须删掉事实才能适配素材、出现无法覆盖的任意航点/数字或顺序歧义、语法失败。此时修正文案/词库，不回退实时 TTS。

### 3. 替换播放器并提供最小离线检查工具

- **文件/符号**：`tts.py::PiperSpeaker` 替换为 `RecordedSpeaker`；复用 `_SpeechJob`、队列、`announce/announce_and_wait` 和 `lead_time_sec`；新增工具 `scripts/prepare_wisadel_voice.py::main` 及 `generate/check/render` 子命令。
- **改动**：删除全部 Piper 命令与参数；有限清单短语完整解析后拼 PCM，标点采用短停顿，不交叉淡化、不混音。单 worker 串行播放，一条普通事实句任一素材缺失就整条失败。二维码全任务预先解析/核验再按项播放，每项操作独立限时，保留总时长估算供 C 提前量使用。素材路径限定于清单指定源码声音目录，拒绝逃逸路径和非有效 PCM；不存在素材/设备仅告警，不能使 mission 构造因外围缺失失败。拼接文件先写临时文件，成功后替换，避免播放半文件。用可终止的 `aplay` 子进程支持 Ctrl-C/异常立即关闭，终止等待有短上限，不阻塞机器人清理。汇总不增加另一个队列、优先级或重试。
- **依赖**：步骤 2 清单；真实音频验收依赖步骤 4。
- **检查命令**：`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission/tts.py scripts/prepare_wisadel_voice.py`；`/usr/bin/python3 scripts/prepare_wisadel_voice.py check --manifest <manifest> --text-only`。
- **期望**：无需 ROS、GPU 或播放器即可核验全部清单文案；空文本为无操作，未知短语整条失败；新增非平凡逻辑留下可运行离线工具而非禁止的单元测试。
- **停止条件**：需要新依赖才能拼接、单 worker 外出现并发声音、无法终止音频、路径/格式验证被省略、离线检查会发送运动请求。

### 4. 直接批量生成并核验素材

- **文件/符号**：复用 `scripts/generate_wisadel_voice.py::main`、技能规定的 `ubuntu-remote:~/GPT-SoVITS/generate_wisadel_audio.py`；新增 `sounds/wisadel/*.wav`，更新清单与代表场景渲染结果。
- **改动**：工具逐条调用既有脚本，显式 `--mode gremlin --lang zh --output <清单路径>`，不用 `--play`。逐文件核验 PCM 元数据；不覆盖有效既有用户素材。失败即停止本轮批量，保留已完成条目供显式再次运行，不加入自动重试策略。远端状态仅在获批执行阶段检查，不假定其在线。
- **依赖**：步骤 2、3；批量前不等待额外试听审批。
- **检查命令**：`ssh -o BatchMode=yes -o ConnectTimeout=10 ubuntu-remote 'test -x ~/GPT-SoVITS/.venv/bin/python && test -r ~/GPT-SoVITS/generate_wisadel_audio.py'`；`/usr/bin/python3 scripts/prepare_wisadel_voice.py generate --manifest <manifest> --mode gremlin`；`/usr/bin/python3 scripts/prepare_wisadel_voice.py check --manifest <manifest>`；`/usr/bin/python3 scripts/prepare_wisadel_voice.py render --manifest <manifest> --output-dir /home/rc4/fruit_ws/tmp/wisadel_voice_preview`。
- **期望**：清单所有 WAV 有效、统一格式、无空文件；代表普通句和每个 QR/汇总播放分段实际时长≤10秒；完整摘要可以长于10秒但不会丢项。长任务若后台执行须使用 `on_exit: wake`，不忙轮询。
- **停止条件**：远端不可达/脚本不存在、格式与计划不符、合成或传输失败、必需短句超过10秒。不得改远端服务、安装库或通过拉长比赛超时解决。

### 5. 接入 mission 并删除旧 Piper 配置/启动检查

- **文件/符号**：`mission_context.py::__init__/announce/announce_and_wait`；`actions.py::scan_b_qr/scan_c_qr/_qr_length_scale`；`config.py::validate_config`；`config/fruit_mission.yaml::tts`；`start_fruit_mission.sh::mission_runtime_paths` 与相关检查。
- **改动**：只保留 RecordedSpeaker，删除 length_scale 与 `_qr_length_scale`；YAML 只留源码清单路径、`aplay_executable`、拼接 WAV 路径与必要独立超时，保留 `c_qr_announce_lead_sec=10.0`。启动脚本移除 Piper 模型/可执行文件要求，改成素材清单和 aplay 可用性告警，不因外围语音文件缺失终止 A/B/C。config 只验证配置结构/数值有限且正值，文件不可用按播报失败处理。无需修改 `setup.py/package.xml/launch`，除非实查发现缺项；不新增依赖。
- **依赖**：步骤 3、4。
- **检查命令**：`bash -n start_fruit_mission.sh`；`/usr/bin/python3 -m compileall -q src/fruit_mission/fruit_mission scripts/prepare_wisadel_voice.py`；`ament_flake8` 仅传本次修改 Python 文件；`colcon build --base-paths src --packages-select fruit_mission --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`（在任务 worktree 中，不发现其他 worktree 的包）。
- **期望**：构建与语法通过；没有 Piper 执行或 length_scale 调用残留；生成时可联网，比赛播放不联网。
- **停止条件**：启动脚本仍读取被删参数、运行会调用模型、构建因基线/环境失败。区分既有失败与本次失败，不顺手修其他包。

### 6. 添加计数与主流程最后的汇总函数

- **文件/符号**：`MissionContext.__init__` 内存计数及新增 `record_pick/announce_summary`；`actions.py::observe_and_pick/_grasp_and_collect/pick_b_tree`；`mission_node.py::FruitMissionNode.run`；`decisions.py` 最小汇总文案格式帮助函数。
- **改动**：使用 `collections.Counter`，不创建统计服务/数据库。实际品种沿共用路径传入，`_grasp_and_collect` 完成 Home 后唯一地记录一次；每个 B 请求保留其 model_type，对应成功项独立计数。正常流程完成、感知关闭后直接调用 `announce_summary()`，按已有品种顺序逐个短段同步播；故障段告警并继续下一段。不在 `finally` 汇总；异常/取消走既有安全清理并停播。结束语与汇总成为最后的正常播报。数量读“两个”等完整短语，非零项才播，全零使用固定台词。不得为采摘失败补计数或重试抓取。
- **依赖**：步骤 5；清单在步骤 2 已含汇总素材。
- **检查命令**：`/usr/bin/python3 scripts/prepare_wisadel_voice.py check --manifest <manifest>`；`/usr/bin/python3 scripts/prepare_wisadel_voice.py render --manifest <manifest> --scenario summary --output-dir /home/rc4/fruit_ws/tmp/wisadel_voice_preview`；针对修改文件 `compileall` 和 `ament_flake8`，重新构建 `fruit_mission`。
- **期望**：离线实际格式/拼接路径覆盖全零、单品种、多品种和可用数量短语边界；生成可试听 WAV。代码审核确认成功出口记录、失败出口不记录、主流程正常末尾汇总；不使用 mock ROS 或测试冻结具体任务数。
- **停止条件**：计数来源是期望目标而非实际识别、同一目标重复计数、Home 失败仍计入、Ctrl-C 仍汇总、为了播报改变运动控制或加共享工序预算。

### 7. 同步文档、合并并人工声音验收

- **文件/符号**：`docs/VOICE_FLOW.md`、`docs/PROJECT_ARCHITECTURE.md::7.4`、根 `README.md::语音播报排查`、`src/fruit_mission/README.md`、`docs/RULE.md::当前项目范围`；新素材与清单纳入版本管理。
- **改动**：文档同步有限词库、gremlin、本地拼接、QR 分项、提前量、独立超时、失败继续、结束品种汇总和计数证据边界。仅将 RULE 中“不做收获汇总”改为本次确认的正常结束工序记录汇总，D/换筐/返航裁剪不变。删除有效说明中的 Piper 操作示例，不动其他已脏章节。合并已验证 worktree 结果到 main，保留用户改动，随后删除短期分支/worktree。
- **依赖**：步骤 4～6 软件验证完成；用户选择交付后统一试听。
- **检查命令**：`git diff --check -- <本次变更文件>`；`/usr/bin/python3 scripts/prepare_wisadel_voice.py check --manifest /home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml`；人工在机器人扬声器执行 `render --manifest <manifest> --scenario intro --output-dir /home/rc4/fruit_ws/tmp/wisadel_voice_preview --play`，再依次试听 B/C 摘要、最长位置/品种、失败句、零/多品种汇总场景。离线工具使用真实 RecordedSpeaker 路径，以损坏/缺失素材场景和播放中 Ctrl-C 检查失败继续及停播，不启动 mission 或硬件服务。
- **期望**：裁判能听清数字、顺序、品种与“不成熟/跳过/完成”的区别；无明显接缝、重叠或截断；Ctrl-C终止声音；文档与行为一致；交付在 main 而非仅留 worktree。
- **停止条件**：人工尚未试听则标记“声音效果待验收”，不得宣称实机声音合格；若音色/读音不清，停止清理旧环境并报告需修改的素材，不擅自切 classic。

### 8. 删除旧 Piper 本机文件并做最终检查

- **文件/符号**：仅 `/home/rc4/fruit_ws/.venv/piper-tts/`、`/home/rc4/fruit_ws/.cache/piper-tts/`；当前不存在的 `tmp/fruit_mission/tts_latest.wav` 仅在执行阶段再次核对为旧 Piper 产物后删除。保留其他 tmp 数据、wisadel生成入口、技能及原 `sounds/fruit_success.wav`。
- **改动**：这是用户授权的机器文件清理，放在软件与音频验收后。重新核对目录解析结果、进程命令及进程打开文件；正常停止仍使用旧环境的音频进程，不能干扰机器人运动或任意杀任务。确认路径非外部链接目标且没有其他活跃使用者后，仅删除这两个专用目录。
- **依赖**：步骤 7 验收通过且已合并；本步骤不在规划阶段执行。
- **检查命令**：`realpath /home/rc4/fruit_ws/.venv/piper-tts /home/rc4/fruit_ws/.cache/piper-tts`；`ps -eo pid,args` 并核对相关 `/proc/<pid>/exe`、命令行和打开文件；确认后按显式绝对路径删除上述两个目录。删除后 `test ! -e /home/rc4/fruit_ws/.venv/piper-tts && test ! -e /home/rc4/fruit_ws/.cache/piper-tts`，再运行离线 `check` 与 `render --scenario summary`，及 `git status --short`。
- **期望**：两个旧目录不再存在，新播报完全不依赖它们；最终 diff 只有本任务变更，用户其他改动仍在。
- **停止条件**：路径解析与核对目录不符、目录出现新用途、旧进程/worktree仍在使用或无法确认、试听/构建未通过。报告未完成清理，不能盲删或删除其他环境。

## 验收

- **现有证据**：只读事实核对与多轮需求确认完成；没有运行代码、SSH、音频生成、播放、构建、测试或删除操作。不得将上面的期望写成测试通过。
- **软件**：仅本地 PCM 拼接；Piper 后端/参数/启动依赖删除；QR 项序完整，异步与提前量语义保留，单 worker不重叠。有效配置下素材/设备缺失不阻断比赛，不播残缺事实。
- **数据**：工序成功后按实际品种唯一计数，失败/跳过不计。正常主流程最后直接汇总，非零品种“品种+N个”，全零固定句；异常不汇总。
- **时间与停止**：每段拼接/播放/同步等待独立≤10秒，较短原有等待不加长。Ctrl-C停播不阻碍机器人STOP；不得以语音完成替代运动反馈。
- **声音**：全部真实 WAV 元数据与分段时长通过离线检查；用户在机器人扬声器人工确认 gremlin 的读音、顺序、品种与失败语义清晰。
- **交付**：清单/WAV/离线工具/文档在主版本；旧专用Piper环境和缓存经安全核对后删除。未通过的试听、清理或运行验证逐项保留为未完成项，不宣称已运营完整采摘闭环。

## 风险

- gremlin音调高且快，裁判可能听不清。用户明确选此模式，不能擅自换音色；通过离线短句与交付试听判断，必要时请求调整。
- 现有远端模型与实际输出尚未运行核验。网络、输出格式和模型状态是执行前置，失败停止生成，不影响已部署比赛流程。
- 纯拼接可能有接缝和长静音；不引入音频工程框架。优先使用完整短语和简短停顿，具体有问题的素材再经试听修订。
- 汇总的“几个”是**已确认采摘放置工序记录**，不是传感器验证的真实入筐数量。Home失败的目标即使物理已入筐也不会记入；未新增物理收获检测。
- FIFO积压或播放故障可能使同步汇总某段触及10秒上限。按用户要求不增加复杂调度或额外重试；告警后继续后续段，不能无条件保证故障设备仍能完整播出。正常设备与无超时的实际声音验收需完整听到汇总。
- 旧worktree仍有Piper引用，删除专用环境会使这些旧版本无法播放。只在无活跃使用者且主版本验收通过后删除；不迁移或修改那些历史实验。
- 当前用户已有多处脏改动和未跟踪wisadel文件；必须隔离实施、保留基线，尤其不能覆盖根README与架构文档中的既有修改。
- 不安装依赖、不运行机械臂控制工具；若执行阶段缺少命令工具、GPU访问或人工扬声器试听条件，对应步骤标记阻塞并交接。