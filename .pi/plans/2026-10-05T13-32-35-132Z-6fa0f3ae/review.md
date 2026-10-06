## 范围

结论：在本计划涉及的改动及现有调用链中，未发现有证据支持的 P0/P1 问题。

审查依据：本目录 `plan.md` 的最终确认需求，以及 `progress.md` 的执行记录。最终需求覆盖最初的观测姿态偏置方案。

审查对象：

- 仓库：`/home/rc4/fruit_ws`。
- Worktree：`/home/rc4/fruit_ws/.worktrees/target-offset-ac-b`。
- 分支：`task/target-offset-ac-b`。
- 实际任务 tip：`fee45c56e889dbb0a94146e87e8783f39c047f83`，已读取分支 ref 与 reflog 核对。
- main 基线：`cf9a1df962774b4277a50cbbebc1742b7e22b765`。
- 任务尚未合并，状态保持 **Pending Integration**。

完整读取计划中的九个任务文件：

1. `src/fruit_mission/config/fruit_mission.yaml`
2. `src/fruit_mission/fruit_mission/config.py`
3. `src/fruit_mission/fruit_mission/camera_target_transform.py`
4. `src/fruit_mission/fruit_mission/camera_target_planner_client.py`
5. `src/fruit_mission/fruit_mission/mission_context.py`
6. `src/fruit_mission/fruit_mission/camera_arm_target_client.py`
7. `src/fruit_mission/README.md`
8. `src/fruit_mission/docs/CAMERA_TARGET_TRANSFORM.md`
9. `src/fruit_mission/test/test_camera_target_transform.py`

同时检查 `mission_node.py`、`Logic.py`、`actions.py`、`camera_extrinsic.yaml` 和 planner API 文档，以核对配置、区域、姿态和 B 目标冻结调用链。

已使用 `workflow_git` 读取 baseline、diff、status。工具返回主工作区状态，不包含隔离分支的已提交改动。因此另行读取任务 worktree、分支 ref 和 reflog。

baseline 中已有的相机调参、planner 预设、感知配置、图片和技能文件不属于本次执行证据。主工作区 diff 中其他编排改动也不作为本任务问题归因。

保留以下用户确认决策：

- A/C 使用 `target_offset_ac_m: [0.0, 0.03, -0.015]`。
- B 使用 `target_offset_b_m: [0.03, 0.0, -0.015]`。
- 局部 +X 前、+Y 左、+Z 上，各分量保留符号。
- 相机转换使用新鲜实测观测 yaw/pitch。
- 偏置使用原始 Base 目标方向 yaw 和最终请求 pitch。
- `motion.pitch_deg` 为 null 时，偏置与请求均使用观测 pitch。
- A/C 非零局部 Y 可以改变最终 yaw，不增加耦合求解。
- 正式入口显式传 `_current_area`，调试入口显式传 `zone`。
- 现有配置加载入口要求两套新键，不兼容旧键，不跨区回退。
- B 在首次抓取前冻结请求，第二抓不重新换算。
- 不修改外参、FK/IK、预设、比赛顺序或恢复策略。

本轮只读取文件并保存 review，未执行代码、测试、构建、Git 写操作或硬件命令。

## P0

无。

## P1

无。

现有正式与调试调用链均先加载并验证两套偏置，再传入共享客户端。区域选择和请求 pitch 与已确认计划一致。

数学实现复用 `transform_end_to_base_mm`，以原始 Base 点为平移原点。其结果符合：

```text
P_request_mm = P_base_mm
             + Rz(atan2(base_y, base_x)) * Ry(-pitch_request)
             * (1000 * offset_local_m)
```

未发现 Base Y 自动变号、重复 TCP 平移、毫米/米混用或旧接口调用遗漏。

当前抓取 pitch 为 0°，因此偏置不跟随倾斜的观察 pitch。这是已确认决策，不作为缺陷报告。

## 修复步骤

无。没有 P0/P1 修复项，不执行代码修改。

尚未完成的步骤保持不变：最新 main 与外部改动核对、集成验证、fast-forward 集成、main 部署构建和任务 worktree 清理。本轮审查不授权这些操作。

## 验收

### 静态核对

- YAML 中两套向量与最终计划一致。
- `load_config` 和 `load_target_offsets` 拒绝缺少新键或非法向量的配置。
- A/C 选择 AC 向量，B 选择 B 向量。
- `target_request` 拒绝非法区域。
- 观测姿态只负责相机点转换，偏置采用请求 pitch。
- 旋转保留有符号局部 Y，不自动翻转。
- B 请求在执行循环之前冻结。
- 现有 frame、深度、遥测新鲜度、硬件限位及 STOP 处理保留。
- 测试使用公共数学函数和解析期望值，没有 ROS 私有状态 mock。

### 已有执行证据

本轮读取以下原始日志，未重新执行检查：

| 检查 | 原始日志 | 读取结果 |
| --- | --- | --- |
| Python 语法 | `/tmp/pi-unified-exec-18-1972294b.log` | `compileall passed` |
| 定向 lint | `/tmp/pi-unified-exec-20-15a21f6b.log` | 六个文件，`No problems found` |
| 纯数学测试 | `/tmp/pi-unified-exec-21-521f16ae.log` | 10 项，`OK` |
| 隔离构建及 diff 检查 | `/tmp/pi-unified-exec-22-f41346dd.log` | 一个包构建完成，检查通过 |

`progress.md` 另记录源配置断言通过，退出码为 0。本轮未取得该配置检查的独立原始日志，因此只保留为历史记录。

测试中的左右目标、负 pitch 耦合和局部 Y 算例与计划期望值一致。软件测试证据不代表实机验收。

### 限制与交接

- 没有重新运行测试或验证当前主 install。
- 任务仍在隔离分支，main 仍保留旧 Base 偏置实现。
- 未启动真实任务，未验证实际抓取位置或轨迹安全。
- 集成后仍需验证最新 main、构建并重启相关客户端。
- 不将本次 review 通过解释为已合并、已部署或实机采摘成功。
