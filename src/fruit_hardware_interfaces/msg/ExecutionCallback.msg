uint8 CALLBACK_GRIPPER=0
uint8 CALLBACK_CAMERA_GIMBAL=1
# Callback ID 2 reports completion of a left/right observation-position action.
# Keep the legacy constant name for source compatibility.
uint8 CALLBACK_AC_PICK=2
uint8 CALLBACK_ARM_TARGET=3
uint8 CALLBACK_QR_RECOGNITION_POSE=4
uint8 CALLBACK_CURRENT_AREA=5

uint8 STATUS_COMPLETED=0
uint8 STATUS_EXECUTING=1

uint8 callback_id
uint8 callback_status
