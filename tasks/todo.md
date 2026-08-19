# ROS-native Script Communication Checklist

## Preparation

- [x] Review and approve `tasks/plan.md`.
- [x] Record each script's current command, arguments, logs, exit behavior, and
      owned child processes as compatibility invariants.
- [x] Preserve unrelated dirty-worktree changes.
- [x] Capture the reviewed D435i parser fixes as the implementation baseline.
- [x] Confirm `.worktrees/` is ignored.
- [x] Create an isolated `codex/` feature worktree.

## Typed Client Foundation

- [x] Add `src/fruit_mission/config/script_ros_clients.yaml`.
- [x] Add the shared bridge-ready, endpoint, publish, and callback-wait client.
- [x] Declare only required ROS dependencies and console entry points.
- [x] Build and import-check the foundation in `setup_fruit_runtime.bash`.

## D435i Arm Target

- [x] Add the typed D435i arm-target client.
- [x] Use fresh `FruitDetections` for both class and position.
- [x] Preserve five-sample averaging and target selection.
- [x] Publish current area and wait for callback ID 5.
- [x] Publish one ArmTarget and wait for callback ID 3.
- [x] Reduce `test_d435i_arm_target.sh` to lifecycle orchestration.
- [x] Preserve its argument-free invocation, logs, resident inspection mode,
      exit behavior, and Ctrl-C cleanup.
- [ ] Verify success and `point=NONE` timeout behavior on the robot.
- [ ] Stop the migration if the original real-robot workflow does not complete.

## Current Area

- [ ] Add the typed current-area client.
- [ ] Reuse an existing ready bridge without taking ownership of it.
- [ ] If a bridge exists but is not ready, wait/fail without starting another.
- [ ] If no bridge exists, temporarily launch the source-path hardware bridge.
- [ ] Wait for callback ID 5 before announcing current area.
- [ ] Keep the transient-local publisher alive long enough for delivery.
- [ ] Remove direct serial-console use from `zone_witch.sh`.
- [ ] Stop only a bridge process group started by `zone_witch.sh`.
- [ ] Preserve `--A|--B|--C|--D` and one-command operation.
- [ ] Verify absent, starting, and already-ready bridge cases.

## Mission Readiness

- [ ] Add the ROS graph/topic readiness client.
- [ ] Replace node-list text parsing with graph APIs.
- [ ] Replace action-list parsing with `ActionClient.wait_for_server()`.
- [ ] Keep GICP diagnostic acceptance equivalent to current behavior.
- [ ] Replace serial log matching with `/auto_serial_bridge/ready`.
- [ ] Replace topic-info counts with graph endpoint counts.
- [ ] Replace active-mode echo parsing with a typed subscription.
- [ ] Leave the already typed GICP diagnostic subscription behavior unchanged.
- [ ] Compare each new read-only readiness result with the old check before
      removing the old check.
- [ ] Preserve run directories, child logs, startup order, exit behavior, and
      Ctrl-C cleanup.
- [ ] Verify cold start, missing subsystem failure, and cleanup behavior.

## Final Audit

- [ ] Audit remaining `ros2` calls and direct `/dev/stm32` access.
- [ ] Document normal ROS ownership and emergency direct-serial usage.
- [ ] Run Bash syntax, Python compile/import, diff, and package build checks.
- [ ] Run supervised D435i, area, and mission-start smoke checks.
- [ ] Re-run the original documented shell commands from a fresh terminal.
- [ ] Confirm no tests were added or run.
- [ ] Review the final diff for unrelated changes.
- [ ] Merge into `main`.
- [ ] Remove the feature worktree and local feature branch.
