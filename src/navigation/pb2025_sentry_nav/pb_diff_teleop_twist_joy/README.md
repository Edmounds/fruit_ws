# pb_diff_teleop_twist_joy

PS4 teleoperation for a differential-drive base. Hold L1 for the normal 0.4 m/s and 0.8 rad/s
limits, or R1 for the 0.6 m/s and 1.0 rad/s limits. The node publishes only `linear.x` and
`angular.z` on `cmd_vel_teleop`, and publishes an immediate zero command when enable is released.
