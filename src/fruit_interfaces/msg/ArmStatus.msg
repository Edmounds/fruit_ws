uint8 STATUS_IDLE=0
uint8 STATUS_RUNNING=1
uint8 STATUS_COMPLETED=2
uint8 STATUS_FAILED=3
uint8 STATUS_EMERGENCY=4

uint8 GRIPPER_STATE_HOLD=0
uint8 GRIPPER_STATE_OPEN=1
uint8 GRIPPER_STATE_CLOSE=2

uint16 ERROR_NONE=0
uint16 ERROR_TIMEOUT=1001
uint16 ERROR_LIMIT_EXCEEDED=1002
uint16 ERROR_ESTOP_TRIGGERED=1003

uint32 trajectory_id
uint8 task_status
float32 progress
float32 joint_1
float32 joint_2
float32 joint_3
float32 joint_4
float32 joint_1_velocity
float32 joint_2_velocity
float32 joint_3_velocity
float32 joint_4_velocity
uint8 gripper_state
uint16 error_code
