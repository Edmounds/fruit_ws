# Project Rules

> Mandatory rules for AI agents coding, checking, or refactoring this workspace. Preserve existing user changes and keep edits within the requested scope.

This ROS 2 robot competes in the Agricultural Robot / Fruit-Picking Robot event of the 2026 RoboCup China selection competition. Read [docs/RULE.md](docs/RULE.md) for competition work; [the original rules](docs/2026水果采摘.pdf) and the judges' final interpretation take precedence. Current scope is A/B/C; do not add D, bad-produce removal, collection exchange, harvest summaries, or return-to-start flows without a scope change.

**The upper computer (NUC running ROS 2) now controls the arm through `fruit_arm_planner`, the arm kinematics and control package. All arm-control clients must use the interfaces documented in [src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md](src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md).** The upper computer computes IK, joint mapping and trajectories; the lower controller executes those trajectories, reports status and enforces hardware protection. Do not delegate Cartesian IK or task sequencing back to the lower controller.

## Core Directories and Responsibility Boundaries

| Path | Responsibility / boundary |
| --- | --- |
| `src/fruit_arm_planner/` | Upper-computer 4-DOF FK/IK, joint mapping, PTP/LIN planning, presets and arm-control services. No competition sequencing or serial frame handling. |
| `src/fruit_arm_interfaces/srv/` | Authoritative `MoveArmCartesian`, `MoveArmJoints`, `ExecuteArmPreset` service definitions; edit source, not generated bindings. |
| `src/fruit_hardware_interfaces/msg/` | Authoritative trajectory/control/status messages; no mission policy. |
| `src/fruit_mission/` | A/B/C sequencing, perception decisions and client orchestration; use planner APIs, not raw trajectories or serial bytes. |
| `src/fruit_perception/` | Single-camera detection, depth targets and C++ components; no arm actuation. |
| `src/fruit_qr_planner/` | B/C QR plans from camera images; no harvesting sequence execution. |
| `src/navigation/` | Point-LIO odometry, `loam_interface` TF, `fruit_navigation` waypoint action and `fruit_nav_bringup` integration. Current navigation uses `odom`, not map localization. |
| `src/fruit_hardware_bridge/` | Hardware/serial bringup; do not move IK or competition decisions here. |
| `src/auto_serial_bridge/` | Independently cloned, parent-Git-ignored package: protocol generation, framing, ACK/retries and heartbeat only. Changes here are not captured by parent commits. |
| `src/driver/` | Vendor drivers; `realsense-ros` is an SSH Git submodule. Livox source currently has `COLCON_IGNORE`; do not remove it casually. |
| `src/joy_linux/` | Ordinary in-repository ROS 2 package; never turn it into a submodule or nested Git repository. |
| `src/fruit_waypoint_recorder/`, `src/fruit_waypoint_rviz_plugin/` | Waypoint capture/editing and RViz tools; not motion controllers. |
| `scripts/`, root `start_*.sh`, `*.service` | Runtime environment and launch entry points; reuse `scripts/setup_fruit_runtime.bash`. |
| `docs/`, `src/*/docs/`, package `README.md` | Persistent engineering reference; update affected documentation with behavior changes. |
| `build/`, `install/`, `log/`, `rosbags/` | Generated/runtime artifacts, not source. Put temporary scripts, captures and analysis in ignored `tmp/`. |

## Build and Test

Stack: ROS 2 Humble, colcon, `ament_cmake` (C++/rosidl) and `ament_python` (setuptools). Dependencies belong in `package.xml` and are installed with apt/rosdep, not system pip. Perception uses C++17, system OpenCV 4.5.4 and OpenVINO Runtime 2024.6; `loam_interface` uses C++14. No workspace-wide JS/Rust/Go build, standalone Python type-checker, or universal formatter is configured.

Run commands from `/home/rc4/fruit_ws`, outside training venv/Conda. In **every** build, launch, debugging and ROS CLI terminal:

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

The setup enforces `/usr/bin/python3`, Cyclone DDS (`rmw_cyclonedds_cpp`), disabled Python user site, and apt OpenCV 4.5.4 / NumPy 1.x from `/usr/lib/python3/dist-packages`. Stop if validation fails; do not bypass it with `PYTHONPATH`. Never mix old Fast DDS processes with Cyclone DDS; stop old processes and run `ros2 daemon stop` when switching.

### Build and checks

```bash
# Full active source workspace; never discover packages in .worktrees/.
colcon list --base-paths src
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Arm planner plus its workspace dependencies.
colcon build --base-paths src --packages-up-to fruit_arm_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash

# Python syntax check (not static type checking); C++ checking is in the build.
/usr/bin/python3 -m compileall -q src/fruit_arm_planner/fruit_arm_planner src/fruit_arm_planner/launch

# Style checks: pass only affected files/directories, not the whole dirty repo.
ament_flake8 src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py
ament_uncrustify src/fruit_perception/src/fruit_perception_node.cpp

# Automatic C++ formatting for ament-style files, only when formatting is requested.
ament_uncrustify --reformat src/fruit_perception/src/fruit_perception_node.cpp
git diff --check -- AGENTS.md

# After building any changed OpenCV-linked component, inspect its ABI dependencies.
ldd install/fruit_perception/lib/libfruit_perception_component.so
```

Do not apply uncrustify to packages with their own formatter settings: `src/navigation/loam_interface/.clang-format` and `.clang-tidy` govern that package, whose CMake requests clang-format 21 for linting. That executable is currently unavailable; report the limitation instead of claiming a pass. No Python autoformatter is installed in the verified clean runtime; do not add one just to reformat unrelated code. `ldd` must show no `/usr/local/lib/libopencv*` dependencies.

### Existing tests only

**Do not write new tests.** Running existing tests is allowed. YAML checks must validate parsing, required fields, types/ranges and necessary cross-field relations, not freeze tunable thresholds, resolutions, switches, offsets or topic names. Stable protocol contracts must be checked against their authoritative definitions, not duplicated from deployment YAML.

```bash
# Full existing suite: first enable/build CMake test targets.
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
colcon test --base-paths src --return-code-on-test-failure
colcon test-result --all --verbose

# One existing Python file, or one test function within it.
/usr/bin/python3 -m pytest -q src/navigation/fruit_navigation/test/test_axis_planner.py
/usr/bin/python3 -m pytest -q src/navigation/fruit_navigation/test/test_axis_planner.py::test_normalize_angle_returns_minus_pi_to_pi

# One registered C++ test target, after building fruit_perception with BUILD_TESTING=ON.
colcon test --base-paths src --packages-select fruit_perception \
  --ctest-args -R '^test_perception_mode$' --output-on-failure \
  --return-code-on-test-failure
```

These are execution recipes, not a claim that the entire current workspace passes. `src/fruit_arm_planner/test/cartesian_console.py` is an interactive **hardware-control tool**, not a unit test. Do not run it or motion services as an automated validation step.

## Coding Conventions

### ROS 2 structure and runtime

- Prefer ROS CLI to custom probes, e.g. `ros2 topic echo /fruit_hardware_bridge/arm/status` or `ros2 service list`. Create packages with `ros2 pkg create --destination-directory src --build-type ament_python <package_name>` (use `ament_cmake` for C++).
- Every runnable package needs `launch/*.launch.py`. Follow `template.launch.py`'s configuration/node-list/argument/action sections, including `use_sim_time`. **Override its install-share config lookup**: configuration, presets and map/waypoint data must load from source paths.
- Store package configuration in `src/<package>/config/`. Good: `'/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml'`; bad: `get_package_share_directory('fruit_arm_planner') + '/config/arm_presets.yaml'` for runtime config. Installed executables/generated interfaces are still normal ROS artifacts.
- Preserve local naming: Python `ArmPlannerNode`, methods/config keys `move_cartesian`, ROS service types `MoveArmCartesian`. Good: `MoveArmCartesian.Request()` with `pitch_deg`, `duration_ms`; bad: an untyped ad-hoc dictionary with ambiguous `angle`/`time` units.
- In nodes, use `self.get_logger().info(...)` / `.error(...)` and `RCLCPP_INFO(get_logger(), ...)` / `RCLCPP_ERROR(...)`, not `print(...)` or `std::cout` for ROS diagnostics. Interactive consoles may print prompts; keep per-frame/per-point telemetry behind debug/trace logging.
- Follow existing error contracts: invalid planner config raises `RuntimeError` with the source path; invalid values raise `ValueError` or return a failed planning result. Motion services return `success`, `message`, `trajectory_id`; never report successful motion on publish alone or hide a failed result with `except Exception: pass`.
- Use `set -eo pipefail`, then source the runtime setup, **then** `set -u`; Humble setup can read unset `AMENT_TRACE_SETUP_FILES`.
- GUI scripts default to the robot's connected desktop, including over SSH. Set the following before creating Tkinter/other windows; Python single-file tools must set the equivalent `os.environ` values themselves:

```bash
export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export XAUTHORITY="$XDG_RUNTIME_DIR/gdm/Xauthority"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
```

### Arm solver and control contract

- **Control the arm through `fruit_arm_planner` on the upper computer.** Read [03_INTERFACES_AND_API.md](src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md) before adding or changing any arm client. Use `/arm_planner_node/move_cartesian`, `/arm_planner_node/move_joints`, `/arm_planner_node/execute_preset`, plus documented home/retract/stop/emergency-stop services. Do not bypass the planner with raw `TrajectoryHeader`, `TrajectoryPoint`, `ArmControlMode`, legacy `ArmTarget`, or direct serial commands.
- Keep algorithms in `kinematics.py`, bidirectional mapping/limits in `joint_transformer.py`, interpolation/preset stitching in `trajectory_planner.py`, and ROS state/dispatch in `arm_planner_node.py`, under `src/fruit_arm_planner/fruit_arm_planner/`. Reuse these implementations; do not add a second IK/trajectory implementation to a console or mission client.
- XYZ is in metres in the fixed arm-root frame; convert camera targets upstream. `move_cartesian` has no frame field and does not transform frames. The current `cmd_cartesian` callback also ignores `PointStamped.header.frame_id`; labelling camera coordinates `arm_base_link` does not convert them.
- `MoveArmJoints.joints_deg` is four **raw hardware angles in degrees**; Cartesian `pitch_deg` is absolute end-effector pitch. Internal/wire joint angles and `JointState.position` are radians; trajectory time is milliseconds. Console `joints` and preset `link_angles` mean link angles; console `motors` and preset `joints` mean raw hardware angles. Never apply calibration offsets twice.
- Q3 hardware angle represents absolute forearm pitch: map `q2_algo + q3_algo` through Q3 direction/offset, not `q3_algo` alone. Use `JointTransformer` for both directions; do not infer end-effector pitch from uncorrected hardware `joint_3 + joint_4`.
- `linear_path=false` selects joint-space PTP; `true` selects Cartesian LIN with per-point IK/limits. `cmd_*` topics are debugging inputs without per-request results; normal clients use services. `planning.cartesian_offset_m` affects Cartesian service/debug targets only, not preset waypoints or joint requests.
- `config/arm_presets.yaml` owns geometry, mapping/limits, planning and presets. Do not hard-code its current calibration or assume home/retract preset names exist. `/arm_planner_node/reload_presets` validates and atomically replaces **presets only**, retaining the old set on error/busy; geometry/mapping/planning changes require restarting planner and console. Python code changes require rebuilding and restarting the node.
- Preserve one-motion ownership, fresh finite in-limit four-axis feedback, and rejection of stale/missing feedback or unreachable targets. Never substitute zero/history for the measured start. Every trajectory starts at `t=0` with frozen measured hardware joints and HOLD, has at least two points, contiguous point IDs and strictly increasing uint32 millisecond timestamps within the uint16 point-count limit.
- Dispatch Header → all Points → start Mode; match a newly received `ArmStatus.trajectory_id` and `STATUS_COMPLETED` before returning motion success. Failure/emergency/interruption/timeout return failure; timeout publishes STOP. STOP/ESTOP service success means **published**, not confirmed hardware standstill. Preserve feedback/stop processing via the existing reentrant callback group and multithreaded executor; do not nest `spin_until_future_complete` in an already executing client callback.
- **Integration caveat:** mission and its camera debug client share `CameraTargetPlannerClient` for camera-to-arm conversion and `move_cartesian` target motion (HOLD, not a complete grasp). Formal mission observation/QR/retract/recovery now use `execute_preset` with names from mission config; area reporting is ROS-only. A-point and area-exit navigation require completed retract, never a timeout bypass. `start_fruit_mission.sh` owns planner-only launch and checks current services/trajectory endpoints. Preset trajectories are not yet complete; the independent camera/area debug clients still depend on removed legacy `StateMachineCommand` / `ExecutionCallback`. This is not proof of an operational mission. `auto_serial_bridge` queues reliable traffic by message ID; ROS publish order or `mode_start_delay_ms` does not guarantee all points are loaded before Mode. Verify end-to-end ordering before claiming an operational harvesting loop; do not mask the gap with longer sleeps.

Planner entry points (after runtime setup); the debug launch owns a serial bridge, so never duplicate an already running bridge:

```bash
# Planner only, with the bridge managed separately:
ros2 launch fruit_arm_planner fruit_arm_planner.launch.py log_level:=info
# Alternative: bridge + planner for a manual hardware session (not both launches):
ros2 launch fruit_arm_planner fruit_arm_debug.launch.py log_level:=debug
# In another prepared terminal, for manual hardware control only:
/usr/bin/python3 src/fruit_arm_planner/test/cartesian_console.py --log-level info
# Preset-only reload; this does not command motion:
ros2 service call /arm_planner_node/reload_presets std_srvs/srv/Trigger '{}'
```

### Sensors, images and ABI

- Project names: `camera` = the single D435i depth camera; `lidar` = MID-360. Good: `/camera/*`, `/fruit_perception/camera/*`, `lidar_link`, `lidar_imu_link`; bad: reintroducing left/right USB cameras or merging the two lidar frames. Preserve vendor package/API names, `/livox/lidar`, `/livox/imu`, and driver optical frames.
- Point-LIO `extrinsic_T` / `extrinsic_R` describe MID-360 ranging-to-internal-IMU extrinsics, **not** `base_link -> lidar_link` mounting. Keep mounting transforms in `src/navigation/fruit_nav_bringup/config/reality/sensor_frames.yaml`.
- Camera driver and C++ raw-image consumers must be ROS 2 components in the same `ComposableNodeContainer`, each with `extra_arguments=[{'use_intra_process_comms': True}]`. Confirm the limitation and obtain approval before cross-process image transport for a third-party node lacking components.
- Humble intra-process communication requires VOLATILE durability; check history/depth/durability for every new endpoint. For required TRANSIENT_LOCAL control/state endpoints, preserve latching and pass `rclcpp::PublisherOptions` / `SubscriptionOptions` with `use_intra_process_comm = rclcpp::IntraProcessSetting::Disable` **only on that endpoint**. Never change latched data to VOLATILE or disable intra-process communication for the entire component.
- All components in a container must share system OpenCV 4.5.4 ABI. Keep the exact system-path `find_package(OpenCV 4.5.4 EXACT ... NO_DEFAULT_PATH)` contract in `src/fruit_perception/CMakeLists.txt`. Use system `cv_bridge` for images; direct fixed-encoding parsing must validate `encoding`, `step` and data length.

## Prohibited Actions

- Do not hand-edit or commit `build/`, `install/`, `log/`, rosidl outputs, or `build/auto_serial_bridge/include/auto_serial_bridge/generated/*`. Change `src/auto_serial_bridge/config/protocol.yaml` / source interfaces and rebuild instead; protocol YAML changes require regeneration, not just restarting.
- Do not write new tests, remove safety validation/STOP handling, introduce a second arm controller, or add dependencies (e.g. MoveIt) without agreement. Reuse existing code first; ask whether an edge case actually occurs before adding speculative fallback modules or defensive policies.
- Never run `pip install` or `pip install --user` in system Python or `~/.local`. Training/annotation/export belongs in an independent venv **outside this workspace**, with its own requirements/lock file. Keep pip OpenCV, NumPy 2.x, Ultralytics and training OpenVINO tools out of ROS. Do not load `/usr/local` OpenCV 4.10, `cv2`, or related incompatible libraries during ROS build/runtime.
- Do not add credentials (`.env`, private keys, tokens) to source or logs. Existing machine-local exception, retained at the user's explicit request: the robot's sudo password is `222331`; do not copy it into scripts, command lines or other files.
- Robot deployment uses `main` only. Short-lived development branches must be checked, merged back to `main`, then deleted locally; explicitly report any retained unmerged experiment. Do not switch branches to operate different robot versions.
- Preserve unrelated uncommitted work; use a worktree if overlapping changes cannot be isolated. Small config/docs/fixes may stay in the current workspace. Large multi-package/high-risk work requires an isolated worktree (`using-git-worktrees` workflow); concurrent editing agents each need their own branch/worktree. Prefer `.worktrees/`, first checking `git check-ignore .worktrees/<branch>`. Validate and merge worktree results into the user's active branch; never leave deliverables only there.

## Coordinated Updates

- Any behavior/config change → update the affected `docs/*.md`, package `README.md` or `src/<package>/docs/*.md` in the same task. Read implementation/manifests when examples disagree; do not copy stale documentation values into code.
- `src/fruit_arm_planner/fruit_arm_planner/{kinematics,joint_transformer,trajectory_planner}.py` or `config/arm_presets.yaml` → update relevant `docs/01_ALGORITHM_AND_KINEMATICS.md`, `04_SETUP_AND_DEBUGGING.md` and `05_PHYSICAL_ARM_MAPPING_REVIEW.md` under that package. Keep planner and console calibration semantics aligned; restart both after mapping changes.
- Arm service/control changes or `src/fruit_arm_interfaces/srv/*.srv` → update `src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md`, node callbacks, console and affected mission clients. Rebuild interfaces and consumers together; never document publishing as execution success.
- `src/fruit_hardware_interfaces/msg/*.msg` or `src/auto_serial_bridge/config/protocol.yaml` → align rosidl lists, ROS dependencies, planner/bridge bindings and lower-controller wire layout; rebuild generated protocol output. Update `src/fruit_arm_planner/docs/02_COMMUNICATION_PROTOCOL.md` and `docs/LOWER_CONTROLLER_COMMANDS.md`; confirm undocumented firmware status/error codes instead of inventing meanings.
- Sensor topics/frames, perception modes or models → align `src/fruit_perception/config/`, its launch/container, `src/fruit_qr_planner/`, mission consumers, root launch/debug/recording scripts and `docs/{PERCEPTION_MODE_COMMANDS,CAMERA_TUNING,PROJECT_ARCHITECTURE}.md` as affected. Navigation/TF/waypoint changes → update `docs/{ODOMETRY_NAVIGATION,WAYPOINT_RECORDING}.md` and source waypoint files as affected.
- Launch/executable/dependency changes → align `launch/*.launch.py`, `package.xml`, `setup.py` entry points/data files or CMake registration/install rules, plus affected root `start_*.sh` / services. Keep the clean-runtime setup and source-config rules intact.
