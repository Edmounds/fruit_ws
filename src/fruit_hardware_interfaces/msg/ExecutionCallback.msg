uint8 CALLBACK_GRIPPER=0
uint8 CALLBACK_CAMERA_GIMBAL=1
# Callback ID 2 reports completion of the common A/B/C pick-direction action.
uint8 CALLBACK_PICK_DIRECTION=2
# Legacy name kept for source compatibility.
uint8 CALLBACK_AC_PICK=2
uint8 CALLBACK_ARM_TARGET=3
uint8 CALLBACK_QR_RECOGNITION_POSE=4
uint8 CALLBACK_CURRENT_AREA=5
uint8 CALLBACK_RETURN_INITIAL_POSE=6
uint8 CALLBACK_ARM_RETRACT_POSE=7

uint8 STATUS_COMPLETED=0
uint8 STATUS_EXECUTING=1
uint8 STATUS_FAILED=2

uint8 callback_id
uint8 callback_status
