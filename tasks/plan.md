# Implementation Plan: ROS-native Script Communication

## Overview

Replace shell-level ROS topic text parsing and direct serial access in normal
project scripts with purpose-built `rclpy` clients. Shell scripts will remain
responsible for environment setup, device checks, launching processes, log
collection, signals, and cleanup. ROS messages, callbacks, readiness state,
and graph discovery will stay typed inside ROS instead of crossing a text
boundary.

The main execution path will become:

```text
shell lifecycle wrapper
        |
        v
rclpy one-shot client
        |
        +--> perception topics
        +--> /auto_serial_bridge/ready
        +--> command/arm topics --> serial bridge --> lower controller
        +<-- execution callback topic <------------- lower controller
```

No new test files or test-suite runs are planned, per project instructions.
Verification will use syntax/import checks, package builds, ROS graph
inspection, and supervised hardware runs.

Migration is vertical and script-by-script. A new ROS client is added and
verified before its shell wrapper is switched. The next script is not migrated
until the previous script completes its original real-robot workflow. Each
accepted script migration is kept as a separate Git commit so it can be rolled
back independently without disturbing the other scripts.

## Scope

### Migrate

- `test_d435i_arm_target.sh`: all topic publication, subscription, endpoint
  checks, target sampling, and lower-controller callback handling.
- `zone_witch.sh`: current-area command and current-area announcement.
- `start_fruit_mission.sh`: node/action/topic readiness, serial readiness, and
  active perception-mode readiness. Keep its existing typed GICP diagnostic
  subscription behavior unchanged.

### Keep as CLI or direct system access

- `ros2 launch`, `ros2 run`, `ros2 component`, and `ros2 bag record`: these
  manage ROS processes or tools and do not parse business messages.
- `v4l2-ctl`, RealSense discovery, process inspection, signals, and log
  collection: these are not ROS topic communication.
- `lower_controller_console.py`: keep as an explicit emergency wire-protocol
  diagnostic tool. Normal scripts must not invoke it, and it must not run at
  the same time as `auto_serial_bridge`.
- Existing nodes and tools that already use `rclpy`/`rclcpp`, including camera
  tuning, camera GUI, mission execution, and robot debug monitoring.

## Architecture Decisions

- Add small one-shot ROS clients to the existing `fruit_mission` package. This
  package already owns the relevant perception, current-area, hardware-command,
  and execution-callback contracts.
- Keep the public shell entry points and their current command-line contracts:
  `test_d435i_arm_target.sh` remains argument-free, `zone_witch.sh` keeps
  `--A|--B|--C|--D`, and `start_fruit_mission.sh` remains directly executable.
- Preserve lifecycle behavior, exit status meaning, log locations, Ctrl-C
  cleanup, and the D435i script's resident post-run inspection mode.
- Put script-client parameters in
  `src/fruit_mission/config/script_ros_clients.yaml`; do not hard-code normal
  topic names or timeouts in shell.
- Invoke clients with `ros2 run fruit_mission ...` while passing the config
  file from the workspace `src` path.
- Use `/auto_serial_bridge/ready` (`std_msgs/msg/Bool`, reliable,
  transient-local, depth 1) instead of searching serial logs for `RUNNING`.
- Use ROS graph APIs (`get_node_names_and_namespaces`, `count_publishers`,
  `count_subscribers`) and `ActionClient.wait_for_server()` instead of parsing
  `ros2 node/action/topic` text.
- Use typed `FruitDecision` messages as the only source for target type,
  detections, and 3D position. Do not synchronize separate perception topics.
- Subscribe and establish graph endpoints before publishing a hardware command.
  Publish each hardware command once; the serial bridge's reliable protocol is
  responsible for ACK/retry. Wait for the matching typed
  `ExecutionCallback` status.
- Keep the serial protocol boundary strict: clients know ROS message fields and
  callback IDs, but never encode frames, calculate checksums, process transport
  ACKs, or read raw serial bytes.
- Preserve current target selection and five-sample averaging behavior. Reject
  detections without `has_point`, non-finite coordinates, or messages older
  than the TREE-mode transition.
- Do not refactor `MissionContext` in this change. It is production mission
  logic and already communicates through typed ROS APIs; changing it would add
  risk unrelated to migrating scripts.

## Compatibility Invariants

- A script is not considered migrated merely because it builds. Its original
  end-to-end operation must complete on the robot before the old implementation
  is removed from that script's commit.
- Do not change script names, executable bits, accepted arguments, required
  environment setup, or documented one-command entry points.
- Do not keep a permanent hidden direct-serial fallback. During implementation,
  the previous behavior remains recoverable through the immediately preceding
  Git commit; final normal operation has one clear ROS path.
- A script may launch a dependency it previously owned. It must track ownership
  and stop only the process group it started, never an already-running shared
  stack.
- Existing-but-not-ready and completely-absent dependencies are different
  states. Never start a duplicate node merely because an existing node has not
  reached ready state yet.
- Every failure path must stop before publishing the next hardware command and
  must return a nonzero status while retaining useful logs.
- Current-area publication must remain alive long enough for matched subscribers
  to receive the transient-local update before a one-shot client exits.

## QoS Contract

| Topic | Direction | QoS |
| --- | --- | --- |
| `/auto_serial_bridge/ready` | subscribe | reliable, transient-local, depth 1 |
| `/fruit_mission/perception_mode` | publish | reliable, volatile, depth 1 |
| `/fruit_perception/active_mode` | subscribe | reliable, transient-local, depth 1 |
| `/fruit_mission/current_area` | publish | reliable, transient-local, depth 1 |
| `/fruit_perception/decision` | subscribe | reliable, volatile, depth 10 |
| `/fruit_hardware_bridge/state_machine/command` | publish | reliable, volatile, depth 10 |
| `/fruit_hardware_bridge/arm/target` | publish | reliable, volatile, depth 10 |
| `/fruit_hardware_bridge/execution/callback` | subscribe | reliable, volatile, depth 10 |
| `/diagnostics` | subscribe | existing publisher-compatible diagnostic QoS |

## Dependency Order

```text
Configuration and common callback client
        |
        +--> D435i arm-target client --> test_d435i_arm_target.sh
        |
        +--> current-area client -----> zone_witch.sh
        |
        +--> readiness client --------> start_fruit_mission.sh
        |
        v
Cross-script audit and supervised hardware verification
```

## Task 0: Isolate the implementation safely

**Description:** Preserve the dirty main workspace and establish an isolated
feature worktree because this change spans several scripts and package files.
The already reviewed `test_d435i_arm_target.sh` changes must be captured as a
separate baseline before the worktree is created, so the later merge does not
overwrite overlapping local edits.

**Acceptance criteria:**

- The current unrelated changes remain untouched.
- `.worktrees/` is confirmed ignored before creating the worktree.
- The feature branch uses the `codex/` prefix and is based on the intended
  current baseline.

**Verification:**

- Compare `git status --short` before and after setup.
- Confirm the worktree branch contains the two existing DDS-output parser
  fixes.

**Dependencies:** None.

**Estimated scope:** Small, Git-only preparation.

## Task 1: Add configuration and a typed hardware callback client

**Description:** Add the shared parameters and the minimal reusable ROS logic
for waiting on bridge readiness/endpoints, publishing one hardware command,
and waiting for the matching completed/failed callback. Keep this helper
limited to script clients; do not restructure mission production code.

**Acceptance criteria:**

- Topic names, sample counts, and timeouts come from
  `script_ros_clients.yaml`.
- A command is not published until bridge-ready, outbound subscriber, and
  callback publisher are visible.
- Completed, failed, and timeout outcomes return distinct nonzero/zero process
  results without parsing console text.

**Verification:**

- Source `setup_fruit_runtime.bash` and build only `fruit_mission` plus changed
  dependencies with `colcon build`.
- Import the new modules with system Python in the project ROS runtime.
- With the bridge absent, confirm the client times out cleanly without opening
  `/dev/stm32` itself.

**Dependencies:** Task 0.

**Files likely touched:**

- `src/fruit_mission/config/script_ros_clients.yaml`
- `src/fruit_mission/fruit_mission/script_hardware_client.py`
- `src/fruit_mission/package.xml`

**Estimated scope:** Medium, 2-3 files.

## Task 2: Migrate the D435i arm-target workflow

**Description:** Implement one ROS client that performs the complete current-A,
TREE-mode, D435i sampling, ArmTarget, and callback sequence. Reduce the shell
script to stack lifecycle management and one client invocation.

**Acceptance criteria:**

- No `ros2 topic echo`, `ros2 topic pub`, CSV/YAML parsing, or callback listener
  process remains in `test_d435i_arm_target.sh`.
- The client derives target type and coordinates from fresh typed
  `FruitDecision`, waits for one fresh valid target, and publishes one
  `ArmTarget`, and waits for callback ID 3.
- The existing behavior of leaving the perception stack resident for inspection
  after success or failure is preserved.
- The existing command (`./test_d435i_arm_target.sh`), startup messages,
  timeout failures, Ctrl-C cleanup, and run-log location remain usable without
  new mandatory arguments.

**Verification:**

- Run shell syntax and Python compilation/import checks.
- Supervised robot run: observe bridge ready, current-area callback ID 5,
  active mode TREE, five valid samples, ArmTarget TX packet ID `0x13`, and
  completed callback ID 3.
- Confirm DDS diagnostic text may appear in logs but cannot influence control
  flow or typed message values.

**Dependencies:** Task 1.

**Files likely touched:**

- `src/fruit_mission/fruit_mission/d435i_arm_target_client.py`
- `src/fruit_mission/setup.py`
- `test_d435i_arm_target.sh`
- `src/fruit_mission/config/script_ros_clients.yaml`

**Estimated scope:** Medium, 4 files.

## Checkpoint A: D435i path

- `fruit_mission` builds in the clean ROS runtime.
- The shell contains no structured ROS CLI output parsing.
- A valid D435i point reaches the lower controller and callback ID 3 completes.
- A `point=NONE` stream exits as a target timeout and sends no ArmTarget.

## Task 3: Migrate the manual area command

**Description:** Replace `zone_witch.sh` direct serial-console piping and
one-shot CLI publication with a typed current-area ROS client. Preserve its
one-command operation: reuse an already-ready bridge, wait for an existing
bridge that is still starting, or temporarily launch the existing
`src/fruit_hardware_bridge/launch/fruit_hardware_bridge.launch.py` only when no
bridge ready publisher exists.

**Acceptance criteria:**

- `zone_witch.sh` no longer opens `/dev/stm32` or invokes
  `lower_controller_console.py`.
- Area A/B/C/D is published as `StateMachineCommand.TASK_CURRENT_AREA`, waits
  for completed callback ID 5, then publishes the latched current-area string.
- `--A|--B|--C|--D` remains the public interface; invalid input fails before
  starting ROS or sending a command.
- An already-running bridge is reused and never stopped by the script. A bridge
  started by the script is stopped on success, failure, or Ctrl-C. An
  existing-but-not-ready bridge never causes a duplicate bridge to start.

**Verification:**

- With the bridge absent, confirm the script starts one bridge, waits for ready,
  completes the area command, and stops only that bridge.
- With a bridge already ready, confirm the script neither starts nor stops a
  bridge.
- With a bridge publisher present but ready false, confirm the script waits or
  exits clearly and never creates a second serial owner.
- With the bridge ready, run one supervised area change and observe packet ID
  `0x11`, callback ID 5 completed, and the latched current-area topic update.

**Dependencies:** Task 1.

**Files likely touched:**

- `src/fruit_mission/fruit_mission/current_area_client.py`
- `src/fruit_mission/setup.py`
- `zone_witch.sh`
- `src/fruit_mission/config/script_ros_clients.yaml`

**Estimated scope:** Medium, 4 files.

## Task 4: Replace mission startup text readiness checks

**Description:** Add a one-shot readiness client for ROS graph state,
FollowWaypoints availability, serial ready state, bridge endpoints, and active
perception mode. Keep launch, process monitoring, logs, cleanup, and the already
typed embedded GICP diagnostic subscription unchanged in the first migration;
moving working typed ROS code is outside this stability-focused change.

**Acceptance criteria:**

- `start_fruit_mission.sh` no longer parses output from `ros2 node list`,
  `ros2 action list`, `ros2 topic info`, or `ros2 topic echo`.
- Serial readiness comes from `/auto_serial_bridge/ready`, not log text.
- Navigation readiness still requires the current accepted GICP diagnostic;
  perception readiness still requires the same nodes, endpoints, and active
  mode OFF before mission launch.
- Startup order, run-directory format, child logs, `wait -n` behavior, signal
  handling, and failure cleanup remain unchanged.

**Verification:**

- Run a supervised cold start and confirm mission launch occurs only after all
  typed/graph readiness conditions pass.
- Before deleting each old text check, run the new read-only check alongside it
  once and confirm both report the same ready/not-ready state.
- Stop one required subsystem and confirm startup fails at the corresponding
  readiness condition with a specific diagnostic message.
- Confirm child process cleanup and retained log paths behave exactly as before.

**Dependencies:** Task 1.

**Files likely touched:**

- `src/fruit_mission/fruit_mission/stack_readiness_client.py`
- `src/fruit_mission/setup.py`
- `start_fruit_mission.sh`
- `src/fruit_mission/config/script_ros_clients.yaml`

**Estimated scope:** Medium, 4 files.

## Checkpoint B: Startup and area paths

- `zone_witch.sh` uses only the ROS bridge path in normal operation.
- `start_fruit_mission.sh` has no ROS CLI text readiness parsing.
- Cold-start, unavailable-bridge, and missing-navigation cases report the
  correct reason and clean up their child process groups.

## Task 5: Audit, document, and merge

**Description:** Re-run the project-script audit, document the normal ROS path
and emergency direct-serial boundary, complete final runtime checks, and merge
the worktree branch back into the user's active `main` branch.

**Acceptance criteria:**

- No normal script parses typed topic payloads from console text or opens the
  lower-controller serial port directly.
- Remaining ROS CLI uses are limited to launch/run/component/bag and explicitly
  justified human diagnostics.
- Documentation says `lower_controller_console.py` is emergency-only and must
  not contend with `auto_serial_bridge` for `/dev/stm32`.
- The feature branch is merged into `main`, then the worktree and local feature
  branch are removed after checks pass.

**Verification:**

- Use `rg` to audit remaining `ros2` calls and direct `/dev/stm32` access.
- Run `bash -n`, Python compilation/import checks, `git diff --check`, and the
  relevant `colcon build` in the clean project runtime.
- Perform one final supervised D435i command, area command, and mission startup
  smoke run.
- Re-run the exact documented user commands, without new flags, from a fresh
  terminal after sourcing the project runtime.
- Do not add or run tests.

**Dependencies:** Tasks 2, 3, and 4.

**Files likely touched:**

- `docs/LOWER_CONTROLLER_COMMANDS.md`
- `docs/PERCEPTION_MODE_COMMANDS.md`
- Files changed by Tasks 1-4

**Estimated scope:** Medium, documentation plus verification and Git cleanup.

## Risks and Mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| `zone_witch.sh` loses one-command operation | High | Reuse an existing bridge or temporarily launch the source-path `fruit_hardware_bridge` launch; track ownership and stop only the bridge started by the script. |
| QoS mismatch hides a latched state | High | Copy QoS from the publishers and existing `MissionContext`; verify late-joining subscribers receive ready/active/current-area state. |
| A stale detection is used after changing camera mode | High | Record the TREE transition time and accept only newer detection headers. |
| A hardware command is published before callbacks are observable | High | Create subscriptions first and wait for both outbound and callback graph endpoints before publishing once. |
| A failed callback is mistaken for completion | High | Treat only `STATUS_COMPLETED` as success; surface `STATUS_FAILED` separately. |
| Existing dirty work is overwritten | High | Use a baseline checkpoint and isolated worktree; merge only after a clean diff review. |
| A broad rewrite makes all scripts fail together | High | Migrate and verify one vertical script workflow per commit; do not proceed after a failed real-robot checkpoint. |
| Fast DDS SHM warnings continue | Low for parsing, potentially separate transport concern | Typed clients do not parse stdout. Diagnose SHM ownership/port configuration separately; do not delete shared-memory files or disable transport as part of this migration. |

## Success Criteria

- Business control flow depends on typed ROS messages, graph state, and process
  exit codes, never human-readable ROS CLI output.
- The only normal owner of `/dev/stm32` is `auto_serial_bridge`.
- D435i targeting, area reporting, and mission startup preserve their current
  externally visible behavior while producing specific typed timeout/failure
  outcomes.
- The same shell filenames and documented arguments work from a fresh terminal;
  no extra manual node startup is required for a command that was previously
  self-contained.
- All changes are built and verified on the robot from
  `setup_fruit_runtime.bash`, merged into `main`, and the temporary worktree is
  removed.
