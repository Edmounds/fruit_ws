# Project Rules

> Mandatory rules for AI agents coding, checking, or refactoring this workspace. Preserve existing user changes and keep edits within the requested scope.

This ROS 2 robot competes in the Agricultural Robot / Fruit-Picking Robot event of the 2026 RoboCup China competition. Read [docs/RULE.md](docs/RULE.md) for competition details. Current scope covers stages A, B, and C only.

**Competition flow must continue past local failures.** Skip failed observations, picks, or QR reads and advance the stage. Reuse deterministic route logic without speculative retries. Recovery uses one safe Home pose. Waits over 10 seconds are capped at 10 seconds.

**The upper computer controls the arm through `fruit_arm_planner`.** Clients must use services documented in [src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md](src/fruit_arm_planner/docs/03_INTERFACES_AND_API.md). The upper computer computes IK, joint mapping, and trajectories. The lower controller executes trajectories and enforces hardware protection. Never bypass the planner with raw serial bytes or raw trajectories.

## Core Directories and Boundaries

| Path | Responsibility / Boundary |
| --- | --- |
| `src/fruit_arm_planner/` | Upper-computer 4-DOF kinematics, joint mapping, PTP/LIN planning, presets, and arm services. |
| `src/fruit_interfaces/` | Trajectory, control, status messages, and service definitions. Edit source interfaces only. |
| `src/fruit_mission/` | A/B/C sequencing and perception decisions. Call planner APIs. |
| `src/fruit_perception/` | Camera detection, depth target extraction, and C++ components. |
| `src/fruit_qr_planner/` | QR code recognition and target sequencing. |
| `src/navigation/` | Point-LIO odometry, loam TF, and axis waypoint navigation in `odom` frame. |
| `src/auto_serial_bridge/` | Protocol framing, CRC check, retries, and heartbeats. Independently managed package. |
| `src/driver/` | Hardware drivers (`realsense-ros`, Livox SDK). Keep Livox `COLCON_IGNORE`. |
| `src/joy_linux/` | Standard ROS 2 joystick package. |
| `src/fruit_robviz_bridge/` | RobViz preview bridge, waypoint recorder, and visualization HTTP server. |
| `scripts/`, `start_*.sh` | Launch and runtime scripts using `scripts/setup_fruit_runtime.bash`. |
| `docs/`, `src/*/docs/` | Persistent technical documentation. |
| `build/`, `install/`, `log/` | Generated build and runtime artifacts. Never edit or commit manually. |

## Build and Environment

Stack: ROS 2 Humble, colcon, C++17, Python 3.10. Cyclone DDS (`rmw_cyclonedds_cpp`) is mandatory.

Source the runtime environment before running any ROS command:

```bash
cd /home/rc4/fruit_ws
source /home/rc4/fruit_ws/scripts/setup_fruit_runtime.bash
```

### Build Commands

```bash
# Build entire workspace
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Build specific package
colcon build --base-paths src --packages-up-to fruit_arm_planner --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Python syntax check
/usr/bin/python3 -m compileall -q src/fruit_arm_planner/fruit_arm_planner

# Style check on affected files only
ament_flake8 <affected_file.py>
ament_uncrustify <affected_file.cpp>
```

### Diagnostic Logging

Unified runtime logs are written to `/home/rc4/fruit_ws/log/runtime/latest/`:
- `orchestrator.log`: Execution arguments and process lifecycles.
- `arm_planner.log`: Trajectory planning and kinematics diagnostics.
- `perception.log` & `client.log`: Perception pipelines and mission requests.
- `navigation.log`: Odometry and waypoint navigation telemetry.

### Testing Rules

Tests are restricted to pure mathematical algorithms and communication protocols:
- Allowed: Kinematic calculations, TF projections, coordinate math, CRC validation.
- Prohibited: AST source-code string matching, private-method mocking, hardcoded competition sequences, and recursive repository linters.

## Coding Conventions

### ROS 2 and Configuration

- Load package configurations from source paths (e.g. `/home/rc4/fruit_ws/src/.../config/file.yaml`), not from install-share paths.
- Use ROS loggers (`self.get_logger()` in Python, `RCLCPP_INFO` in C++). Do not use `print()` or `std::cout`.
- Motion services must return explicit success flags and trajectory IDs. Never infer success solely from publishing.
- For GUI scripts, set `DISPLAY=:0` and standard session bus environment variables.

### Arm Control Contract

- Always use `fruit_arm_planner` services: `/arm_planner_node/move_cartesian`, `/arm_planner_node/move_joints`, and `/arm_planner_node/execute_preset`.
- Cartesian XYZ coordinates are in meters in the arm-root frame (`arm_base_link`).
- `MoveArmJoints.joints_deg` accepts hardware motor angles in degrees.
- Cartesian pitch represents absolute end-effector pitch in degrees.
- Detailed kinematics and calibration formulas are maintained in `src/fruit_arm_planner/docs/01_ALGORITHM_AND_KINEMATICS.md` and `05_PHYSICAL_ARM_MAPPING_REVIEW.md`.

### Sensors and Vision

- Single D435i camera uses the `/camera/*` namespace.
- Point-LIO extrinsics specify MID-360 range-to-IMU parameters, not chassis mounting transforms.
- Maintain system OpenCV 4.5.4 ABI compatibility across all vision components. Never link against `/usr/local/lib/libopencv*`.

### Git Workflow

- Automatically commit task changes upon completing a task. Stage only task-related files explicitly.
- Preserve existing user modifications in the working tree.
- Do not run continuous unrequested `git status` or `git diff` checks during small edits.
- For complex, risky refactors or concurrent multi-agent tasks, follow `using-git-worktrees` (setup under `.worktrees/<branch-name>`) and `finishing-git-worktrees` (rebase cleanly against `main`, fast-forward merge via `git merge --ff-only`, and clean up worktree).

## Prohibited Actions

- **Never modify `src/fruit_arm_planner/config/arm_presets.yaml` without explicit operator instructions.** Presets are calibrated physically.
- Do not manually edit or commit files under `build/`, `install/`, `log/`, or generated serial protocol headers.
- Do not introduce MoveIt, secondary controllers, or speculative fallback policies.
- Never run `pip install` in the system Python environment. Keep ROS and ML dependencies strictly separated.
- Never use bulk staging (`git add .`, `git add -A`, `git commit -a`). Stage only task-related files explicitly.
- Never discard existing user modifications in the working tree.

## Coordinated Updates

- Update corresponding documentation in `docs/` or `src/*/docs/` when changing node behaviors or interfaces.
- Interface updates in `src/fruit_interfaces/` or `protocol.yaml` require rebuilding both ROS messages and serial code.
- Launch file modifications must preserve the clean runtime setup and source configuration paths.
