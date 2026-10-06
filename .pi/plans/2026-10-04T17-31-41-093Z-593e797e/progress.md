# 工作记录

## 执行记录

### 步骤 1：联网调研并验证规则入口（已完成）
- **调研工具验证**：
  - 发现 `/home/rc4/.pi/agent/skills/anysearch/runtime.conf` 存在异机历史路径，但本机成功验证 Node.js `v24.18.0` 并调用本机路径 `/home/rc4/.pi/agent/skills/anysearch/scripts/anysearch_cli.js` 正常联网。
- **Git 官方机制查证 (`git-merge` 官方文档)**：
  - URL: `https://git-scm.com/docs/git-merge`
  - 核心机制：Git 的 Pre-merge 检查在检测到工作区未提交改动与待合并文件重叠时会拒绝操作；但在存在非关键未提交文件或冲突中断时，任何 `--abort`、`reset` 或不当恢复可能破坏工作区。用户遇到的“未提交代码被强制覆盖”，主要是由于 Agent 缺少阶段 Commit、盲目执行强制检出（如 `checkout -f`、`restore`、`checkout --ours/theirs` 或跨工作区整文件拷贝覆盖）所致。
- **多 Agent Worktree 协作实践调研**：
  - 参考来源：
    - `https://nx.dev/blog/git-worktrees-ai-agents`（Nx Blog: How Git Worktrees Changed My AI Agent Workflow）
    - `https://claudelab.net/en/articles/claude-code/claude-code-git-conflict-auto-resolution-guide`（Claude Lab: Git 冲突分析与自动化指引）
  - 核心业界实践：
    1. **阶段性提交保全**：Agent 在独立 worktree 中完成小阶段必须立即 commit，使代码进入 Git 版本树保护，防止上下文丢失。
    2. **隔离环境适配最新基准**：主分支（main）必须保持保护；合并时严禁在 main 上随意操作。应先在 Agent 自己的 worktree 分支中引入最新 main，进行三方冲突分析（base, ours, theirs）并运行验证。
    3. **主分支未提交/并发时的安全交接**：当 main 分支处于脏状态（存在未提交工作）或有其他会话正在编辑时，严禁使用 stash 移走或强行覆盖；应将已通过检查的提交保存在分支中，输出完整的待合并交接说明（分支名、Commit SHA、基线版本、验证限制、后续集成指南），等待安全时机再由协调会话快进集成。

### 步骤 2：重新核对文件与 Git 归属，保全既有工作（已完成）
- **检查命令与状态核实**：
  - `git -C /home/rc4/.pi/agent rev-parse --show-toplevel` 与 `git -C /home/rc4/.codex rev-parse --show-toplevel` 均返回非 git 仓库，符合规划预期。
  - 在修改前，已将 `/home/rc4/.pi/agent/APPEND_SYSTEM.md` 和 `/home/rc4/.codex/AGENTS.md` 保存完整备份至当前目录：`backup_APPEND_SYSTEM.md` 与 `backup_codex_AGENTS.md`。
  - 在 `/home/rc4/fruit_ws` 运行 `git status --porcelain=v1 --untracked-files=all`，确认当前工作树存在大量未提交修改（包括 `AGENTS.md` 已有的遥测接口变动 `ArmTelemetry` 等）。
  - `git branch --show-current` 显示当前分支为 `main`。
  - `git worktree list --porcelain` 检出当前已有 26 个并存 worktree 分支，充分印证了并发多会话协作的场景。
  - 明确了本任务对 `fruit_ws/AGENTS.md` 的修改边界：仅定位在 `## Prohibited Actions` 中的部署分支和 worktree 交付两条规则，与用户已有的遥测/TCP 描述互不重叠。

### 步骤 3：追加两份用户级规则（已完成）
- **修改文件**：
  - `/home/rc4/.pi/agent/APPEND_SYSTEM.md`：在文件末尾新增 `## Git Workflow & Multi-Agent Collaboration` 章节。
  - `/home/rc4/.codex/AGENTS.md`：在文件末尾新增 `## 5. Git Workflow & Multi-Agent Collaboration` 章节。
- **规则内容落实**：
  1. **阶段性提交**（Milestone Commits）：小阶段通过验证即 commit，鼓励多提交；只提交自身所属文件，严禁 `git add .` / `git commit -a`，严禁 `--no-verify`，失败或归属不清时停机报告。
  2. **任务压缩限制**（Task Squash on Unshared Branch）：仅允许在私有分支完成大任务后压缩为一个清晰提交；严禁重写 main 分支或共享分支历史；在 main 直接开发的会话保持阶段提交原样。
  3. **保护 main 最新基准**（Protect main as Truth Baseline）：worktree 适配 main，严禁覆盖 main；严禁整文件覆盖与机械的 `--ours/theirs` / `reset --hard` / `checkout -f`；执行 3-way 冲突分析并验证，遇意图冲突主动询问。
  4. **主分支脏或并发时暂缓与交接**（Dirty main or Active Writers → Pause Integration & Handoff）：严禁强行合入，严禁自动 stash main，严禁代提他人改动。保全分支提交与 worktree，输出含 6 大要素的待集成交接信息，严禁假报完成。
  5. **串行快进集成**（Serial Fast-Forward Integration）：main 干净且确认无写入时，先在隔离分支适配并验证，再在 main 执行 `git merge --ff-only`；main 若移动则重走验证流程。
- **差异审查**：通过 `diff -u` 对照备份，两文件原有内容 100% 完整保留，新增条款精炼准确无语法瑕疵。

### 步骤 4：对齐 fruit_ws 的项目规则（已完成）
- **修改文件**：`/home/rc4/fruit_ws/AGENTS.md`
- **定位区间**：`## Prohibited Actions` 下的 `Robot deployment uses main only` 与 `Preserve unrelated uncommitted work`。
- **对齐内容**：
  - 保留仅在 main 上部署运行的限制；将“分支合并后删除”修正为“仅在验证通过、且 main 干净且无活跃并发写入时安全集成，然后再删除本地分支”。
  - 将 worktree 的交付要求修正为：“遵循全局 Git 协作规则：在任务分支上进行阶段性 Commit；当 main 存在未提交修改或有活跃写入者时，严禁强行合入或覆盖 main；此时必须安全暂缓，保持分支和 worktree 完整，并提供包含 tip commit SHA、baseline SHA、验证证据与快进集成步骤的持久交接”。
- **校验与隔离**：
  - 运行 `git diff --check -- AGENTS.md` 返回 0（无格式或空白问题）。
  - 对照 `baseline.md`，启动前的已有改动（四轴角遥测 `ArmTelemetry`、工具 Z 偏移量）逐字保留，本任务增量完全隔离且无冲突。

### 步骤 5：新会话只读验证规则加载与场景理解（已完成）
- **规则加载验证**：
  - 使用只读非交互沙箱命令 `codex exec --sandbox read-only --ephemeral` 在 `fruit_ws` 目录执行规则核实。
  - 新会话准确完整输出了第 5 节 `Git Workflow & Multi-Agent Collaboration` 全部条款，证明全局规则已成功注入并生效。
- **场景理解测试**：
  - 测试输入：“你在 worktree 中完成编码与测试后准备合入 main，但发生 main 存在其他开发者/Agent 未提交的修改。按规则你必须采取什么具体步骤？哪些行为被严禁？”
  - 验证结果：新会话准确列出全部刚性动作（在分支独立 commit、记录 baseline SHA 与 tip SHA、检查 main 状态后暂缓集成、保持 worktree 与分支完整并输出 6 项持久交接清单、安全时再快进集成）与严禁行为（禁止合并脏 main、禁止 auto-stash、禁止替别人 commit、禁止覆盖或 reset、禁止 ours/theirs 选边、禁止 force-merge、禁止整文件拷贝覆盖、禁止假报已合并），完全符合预期。
- **仓库只读状态取证**：
  - `git log -1 --format=fuller` 记录当前 main 最新基线为 `c23d6381514c3ac9f68e518fa428c7bb64e3fabf`。
  - `git for-each-ref` 记录了当前未合并的 11 个分支引用。

### 步骤 6：版本状态核验与交付（已完成）
- **约束遵从**：
  - 根据系统与用户指令要求“不自动提交/推送/部署”，且当前 `fruit_ws` 工作树中 `AGENTS.md` 包含用户原有的四轴角遥测修改，本任务严格执行修改隔离，不擅自执行全局 `git commit` 或 `git push`。
- **生效情况总览**：
  1. `/home/rc4/.pi/agent/APPEND_SYSTEM.md`：全局生效（非 Git 文件，已就地生效并由 Pi 在后续会话直接载入）。
  2. `/home/rc4/.codex/AGENTS.md`：全局生效（非 Git 文件，已就地生效并在步骤 5 经 Codex 实际沙箱运行验证）。
  3. `/home/rc4/fruit_ws/AGENTS.md`：项目级文件（已对齐两处禁止规则，`git diff --check` 通过，无空白/语法错误，待用户统一合并或提交）。

### 步骤 7：进阶优化接入与系统修复（已完成）
- **Git 全局冲突样式配置**：
  - 核实当前系统 Git 版本为 `2.34.1`。
  - 成功执行全局配置：`git config --global merge.conflictStyle diff3`。
  - 效果：发生代码冲突时，强制展示 `||||||| base`（共同祖先版本），使 Agent 能够看到原代码全貌，杜绝靠猜合并。
- **规则细节优化对齐**：
  - 在 `/home/rc4/.pi/agent/APPEND_SYSTEM.md` 与 `/home/rc4/.codex/AGENTS.md` 中：
    1. 强化 3-way 冲突分析时必须查看 `||||||| base` 理解原本意图后再行适配；
    2. 引入 `git merge-tree` 进行无损预检的指引；
    3. 补全 Worktree 生命周期闭环：快进集成并验证成功后，必须清理已完成的 worktree 及分支（`git worktree remove` + `git worktree prune`），杜绝僵尸 worktree 泛滥。
