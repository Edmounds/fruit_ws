# fruit_hardware_bridge

Converts the highest-confidence D435i fruit label into the two-byte lower-controller
representation consumed by `auto_serial_bridge`.

## Topics

- Input: `/fruit_perception/d435i/types` (`fruit_perception/msg/FruitTypes`)
- Output: `/fruit_hardware_bridge/d435i/result` (`std_msgs/msg/UInt8MultiArray`)
- Output layout: `data[0] = fruit_id`, `data[1] = status`

An empty or unknown detection publishes `[0, 0]`. Otherwise only the first label,
which is the highest-confidence detection, is converted.

## Runtime

The main perception launch starts the D435i pipeline, this result bridge, and
the serial node together. It uses `/dev/ttyACM0`:

```bash
ros2 launch fruit_perception fruit_perception.launch.py
```

The package launch remains available for serial-only debugging and allows a
different device:

```bash
ros2 launch fruit_hardware_bridge fruit_hardware_bridge.launch.py port:=/dev/ttyUSB0
```
