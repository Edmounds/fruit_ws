uint8 TASK_GRIPPER=0
uint8 TASK_CAMERA_GIMBAL=1
# Task ID 2 is the common A/B/C pick-direction command. The lower controller
# interprets the direction together with the latest TASK_CURRENT_AREA value.
uint8 TASK_PICK_DIRECTION=2
# Legacy name kept for source compatibility.
uint8 TASK_AC_PICK=2
uint8 TASK_QR_RECOGNITION_POSE=4
uint8 TASK_CURRENT_AREA=5
uint8 TASK_RETURN_INITIAL_POSE=6
uint8 TASK_ARM_RETRACT_POSE=7

uint8 GRIPPER_GRIP=0
uint8 GRIPPER_OPEN=1
uint8 GIMBAL_LOOK_DOWN=0
uint8 GIMBAL_LOOK_UP=1
# These status values are the left/right pick directions.
uint8 PICK_DIRECTION_LEFT=0
uint8 PICK_DIRECTION_RIGHT=1
# Legacy names kept for source compatibility; AC_PICK_BOTH is no longer a
# valid status for Task ID 2.
uint8 AC_PICK_LEFT=0
uint8 AC_PICK_RIGHT=1
uint8 AC_PICK_BOTH=2
uint8 QR_RECOGNITION_POSE=0
uint8 AREA_A=0
uint8 AREA_B=1
uint8 AREA_C=2
uint8 AREA_D=3
uint8 RETURN_INITIAL_POSE=0
uint8 ARM_RETRACT_POSE=0

uint8 task_id
uint8 task_status
