# fruit_hardware_bridge

Provides the project launch entry point for the lower-controller serial bridge.
The package is reserved for future semantic hardware interfaces; it does not
currently translate perception results or implement mission behavior.

## Protocol topics

- Command: `/fruit_hardware_bridge/state_machine/command`
  (`fruit_hardware_interfaces/msg/StateMachineCommand`)
- Arm target: `/fruit_hardware_bridge/arm/target`
  (`fruit_hardware_interfaces/msg/ArmTarget`)
- Execution callback: `/fruit_hardware_bridge/execution/callback`
  (`fruit_hardware_interfaces/msg/ExecutionCallback`)

The serial bridge handles reliable delivery for discrete commands. A mission
node must publish each command once and wait for the matching completed callback.

Normal project scripts communicate with the lower controller only through these
ROS topics. `lower_controller_console.py` is reserved for emergency protocol
diagnostics and must not run while `auto_serial_bridge` owns `/dev/stm32`.

## Runtime

The main perception launch starts the camera pipeline and serial node together:

```bash
ros2 launch fruit_perception fruit_perception.launch.py
```

This package launch remains available for serial-only debugging and allows a
different device. It loads the protocol configuration directly from the source
workspace:

```bash
ros2 launch fruit_hardware_bridge fruit_hardware_bridge.launch.py port:=/dev/ttyUSB0
```
