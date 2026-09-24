# fruit_hardware_bridge

Provides the project launch entry point for the lower-controller serial bridge.
The package is reserved for future semantic hardware interfaces; it does not
currently translate perception results or implement mission behavior.

## Protocol topics

- Arm control mode: `/fruit_hardware_bridge/arm/control_mode`
  (`fruit_hardware_interfaces/msg/ArmControlMode`)
- Arm trajectory header: `/fruit_hardware_bridge/arm/trajectory_header`
  (`fruit_hardware_interfaces/msg/TrajectoryHeader`)
- Arm trajectory waypoint: `/fruit_hardware_bridge/arm/trajectory_point`
  (`fruit_hardware_interfaces/msg/TrajectoryPoint`)
- Arm telemetry status: `/fruit_hardware_bridge/arm/status`
  (`fruit_hardware_interfaces/msg/ArmStatus`)
- Chassis velocity: `/cmd_vel`
  (`geometry_msgs/msg/Twist`)

The serial bridge handles reliable delivery for trajectory declarations, waypoints,
and control mode commands. Telemetry status is received periodically at 20Hz.

## Runtime

The package launch is available for serial-only debugging and allows a
different device. It loads the protocol configuration directly from the source
workspace:

```bash
ros2 launch fruit_hardware_bridge fruit_hardware_bridge.launch.py port:=/dev/ttyUSB0
```
