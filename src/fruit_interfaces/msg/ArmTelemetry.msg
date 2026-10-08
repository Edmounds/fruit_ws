# One valid measured hardware snapshot, inverse mapping and FK in the arm-root frame.
std_msgs/Header header

# Original hardware angles (rad) and velocities (rad/s), without calibration removal.
sensor_msgs/JointState hardware_joint_state

# Solver q1/q2/q3/q4 (rad): yaw, absolute shoulder, relative elbow, relative wrist.
# Calibration offsets/directions and Q3 shoulder coupling are removed.
sensor_msgs/JointState model_joint_state

# Real gripper TCP, including geometry.tool_z_offset_m. XYZ in metres.
geometry_msgs/Point tcp_position
# Absolute pitch of the a4 segment in degrees, not the wrist-to-TCP diagonal.
float64 tcp_pitch_deg

# Unchanged controller metadata; values follow fruit_interfaces/ArmStatus.
uint32 trajectory_id
uint8 task_status
float32 progress
uint8 gripper_state
uint16 error_code
