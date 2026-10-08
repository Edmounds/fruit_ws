# fruit_interfaces

Unified ROS 2 interfaces package for the fruit-picking robot, merging `fruit_arm_interfaces` and `fruit_hardware_interfaces`.

## Messages (`msg/`)

- `ArmTelemetry.msg`: Arm joint angles, TCP position, pitch, and execution status.
- `TrajectoryHeader.msg`: Trajectory declaration with ID and point count.
- `TrajectoryPoint.msg`: Trajectory waypoint with 4 joint angles, timing, gripper action, and via-point flag.
- `ArmControlMode.msg`: Arm control mode selection (`TRAJECTORY`, `STOP`, `ESTOP`).
- `ArmStatus.msg`: Arm status telemetry from lower controller (status, progress, joints, velocities, gripper state, error code).

## Services (`srv/`)

- `MoveArmCartesian.srv`: Cartesian motion service with pitch, duration, gripper action, and linear path flag.
- `MoveArmJoints.srv`: Joint-space motion service with 4 raw hardware angles in degrees.
- `ExecuteArmPreset.srv`: Fixed preset sequence execution service (`home`, `left_look`, etc.).

## Migration & Compatibility Notes

- **Package Namespace**: All 8 message and service types are now namespaced under `fruit_interfaces/msg/*` and `fruit_interfaces/srv/*`.
- **Breaking Changes for ROS Clients**: Legacy nodes, external clients, or rosbags referencing `fruit_arm_interfaces` or `fruit_hardware_interfaces` will not automatically resolve. All workspace consumers must be updated and rebuilt together.
- **Unchanged Contracts**: Topic paths, service endpoints (e.g. `/arm_planner_node/move_cartesian`, `/fruit_hardware_bridge/arm/*`), field names, types, order, units, QoS settings, and service result semantics remain strictly identical.
- **Wire Protocol & Firmware**: Serial packet IDs, framing, CRC, and MCU data layouts are completely unchanged (`PROTOCOL_HASH 0xB063CA5B`). Lower-controller firmware does not require recompilation or flashing.
