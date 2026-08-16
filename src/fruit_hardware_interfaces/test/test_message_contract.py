from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _constants(message_name):
    message = PACKAGE_ROOT / "msg" / message_name
    return {
        name: int(value)
        for line in message.read_text(encoding="utf-8").splitlines()
        if "=" in line
        for _, assignment in [line.split(maxsplit=1)]
        for name, value in [assignment.split("=", maxsplit=1)]
    }


def test_state_machine_command_defines_ac_pick_directions():
    constants = _constants("StateMachineCommand.msg")

    assert constants["TASK_AC_PICK"] == 2
    assert constants["AC_PICK_LEFT"] == 0
    assert constants["AC_PICK_RIGHT"] == 1
    assert constants["AC_PICK_BOTH"] == 2


def test_execution_callback_separates_ac_pick_and_arm_target():
    constants = _constants("ExecutionCallback.msg")

    assert constants["CALLBACK_AC_PICK"] == 2
    assert constants["CALLBACK_ARM_TARGET"] == 3
