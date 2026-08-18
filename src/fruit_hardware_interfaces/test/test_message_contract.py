from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _constants(message_name):
    message = PACKAGE_ROOT / "msg" / message_name
    constants = {}
    for line in message.read_text(encoding="utf-8").splitlines():
        declaration, separator, value = line.partition("=")
        if separator:
            _, name = declaration.split()
            constants[name] = int(value)
    return constants


def test_state_machine_command_defines_ac_pick_directions():
    constants = _constants("StateMachineCommand.msg")

    assert constants["TASK_AC_PICK"] == 2
    assert constants["AC_PICK_LEFT"] == 0
    assert constants["AC_PICK_RIGHT"] == 1
    assert constants["AC_PICK_BOTH"] == 2


def test_state_machine_command_defines_qr_pose_and_current_area():
    constants = _constants("StateMachineCommand.msg")

    assert constants["TASK_QR_RECOGNITION_POSE"] == 4
    assert constants["TASK_CURRENT_AREA"] == 5
    assert constants["QR_RECOGNITION_POSE"] == 0
    assert constants["AREA_A"] == 0
    assert constants["AREA_B"] == 1
    assert constants["AREA_C"] == 2
    assert constants["AREA_D"] == 3


def test_state_machine_command_defines_return_initial_pose():
    constants = _constants("StateMachineCommand.msg")

    assert constants["TASK_RETURN_INITIAL_POSE"] == 6
    assert constants["RETURN_INITIAL_POSE"] == 0


def test_execution_callback_separates_ac_pick_and_arm_target():
    constants = _constants("ExecutionCallback.msg")

    assert constants["CALLBACK_AC_PICK"] == 2
    assert constants["CALLBACK_ARM_TARGET"] == 3


def test_execution_callback_defines_qr_pose_and_current_area():
    constants = _constants("ExecutionCallback.msg")

    assert constants["CALLBACK_QR_RECOGNITION_POSE"] == 4
    assert constants["CALLBACK_CURRENT_AREA"] == 5
    assert constants["STATUS_COMPLETED"] == 0
    assert constants["STATUS_EXECUTING"] == 1


def test_execution_callback_defines_return_initial_pose():
    constants = _constants("ExecutionCallback.msg")

    assert constants["CALLBACK_RETURN_INITIAL_POSE"] == 6
