from fruit_mission.logic import PickExecutionResult
from fruit_mission.logic import SideObservation
from fruit_mission.mission_flow import run_a_area
from fruit_mission.mission_flow import run_c_area
from fruit_mission.mission_flow import run_mission
from fruit_mission.mission_flow import scan_b_qr_and_announce
from fruit_mission.mission_flow import scan_c_qr_and_build_table


class FakeContext:
    def __init__(self):
        self.config = {
            "stable_stop_sec": 0.0,
            "side_observation_timeout_sec": 0.0,
            "preferred_side": "left",
            "a_waypoints": ["a_001"],
            "b_qr_waypoint": "b_qr",
            "c_qr_waypoint": "c_qr",
            "c_positions": {
                str(index): {
                    "waypoint": f"c_{index:03d}",
                    "preferred_side": "left" if index % 2 else "right",
                }
                for index in range(1, 13)
            },
        }
        self.actions = []
        self.observations = []
        self.go_to_results = []
        self.area_results = []
        self.mode_results = []
        self.qr_pose_results = []
        self.closed_loop_results = []
        self.b_plan = {
            "zone": "B",
            "task_count": 2,
            "tasks": [
                {"sequence": 1, "position_id": 1, "target": "苹果"},
                {"sequence": 2, "position_id": 2, "target": "梨子"},
            ],
        }
        self.c_plan = {
            "zone": "C",
            "task_count": 2,
            "tasks": [
                {"sequence": 1, "position_id": 4, "target": "洋葱"},
                {"sequence": 2, "position_id": 3, "target": "南瓜"},
            ],
        }
        self.saved = []

    def go_to(self, waypoint):
        self.actions.append(("go_to", waypoint))
        return self.go_to_results.pop(0) if self.go_to_results else True

    def stable_stop(self):
        self.actions.append(("stable_stop",))

    def report_current_area(self, area):
        self.actions.append(("area", area))
        return self.area_results.pop(0) if self.area_results else True

    def send_qr_pose(self):
        self.actions.append(("qr_pose",))
        return self.qr_pose_results.pop(0) if self.qr_pose_results else True

    def set_perception_mode(self, mode):
        self.actions.append(("mode", mode))
        return self.mode_results.pop(0) if self.mode_results else True

    def wait_b_plan(self):
        self.actions.append(("wait_b_plan",))
        return self.b_plan

    def wait_c_plan(self):
        self.actions.append(("wait_c_plan",))
        return self.c_plan

    def save_c_distribution(self, distribution):
        self.saved.append(distribution)

    def announce(self, text):
        self.actions.append(("announce", text))
        return True

    def observe_sides(self, _timeout_sec):
        return self.observations.pop(0)

    def execute_closed_loop_pick(self, side, expected_type=None):
        self.actions.append(("closed_loop_pick", side, expected_type))
        if self.closed_loop_results:
            return self.closed_loop_results.pop(0)
        return PickExecutionResult(True, "picked", expected_type)


def test_scan_c_builds_distribution_and_announces_order():
    ctx = FakeContext()

    distribution = scan_c_qr_and_build_table(ctx)

    assert distribution["positions"]["4"]["target_name"] == "洋葱"
    assert distribution["positions"]["3"]["target_name"] == "南瓜"
    assert ctx.saved[-1] is distribution
    assert ("go_to", "c_qr") in ctx.actions
    assert ("mode", "QR") in ctx.actions
    assert any("C区顺序" in item[1] for item in ctx.actions if item[0] == "announce")


def test_scan_b_failure_stops_mission_before_c_scan():
    ctx = FakeContext()
    ctx.observations = [SideObservation(left=[], right=[])]
    ctx.b_plan = None

    run_mission(ctx)

    assert ("wait_b_plan",) in ctx.actions
    assert ("go_to", "c_qr") not in ctx.actions
    assert ctx.actions[-1] == ("mode", "OFF")


def test_run_a_area_skips_waypoint_when_navigation_fails():
    ctx = FakeContext()
    ctx.go_to_results = [False]

    assert run_a_area(ctx)

    assert not any(action[0] == "closed_loop_pick" for action in ctx.actions)
    assert any("导航失败" in action[1] for action in ctx.actions if action[0] == "announce")


def test_scan_b_aborts_before_waiting_plan_when_qr_pose_fails():
    ctx = FakeContext()
    ctx.qr_pose_results = [False]

    assert scan_b_qr_and_announce(ctx) is None

    assert ("wait_b_plan",) not in ctx.actions
    assert any("二维码识别姿态" in action[1] for action in ctx.actions if action[0] == "announce")


def test_run_c_area_advances_after_skip_and_keeps_qr_order():
    ctx = FakeContext()
    distribution = scan_c_qr_and_build_table(ctx)
    ctx.actions.clear()
    ctx.observations = [
        SideObservation(left=["apple_ripe"], right=[]),
        SideObservation(left=["pumpkin_ripe"], right=["onion_ripe"]),
    ]

    run_c_area(ctx, distribution)

    assert ("go_to", "c_004") in ctx.actions
    assert ("go_to", "c_003") in ctx.actions
    assert ctx.actions.index(("go_to", "c_004")) < ctx.actions.index(("go_to", "c_003"))
    assert ("closed_loop_pick", "left", "pumpkin_ripe") in ctx.actions
    assert distribution["positions"]["4"]["result"] == "type_mismatch"
    assert distribution["positions"]["3"]["result"] == "picked"


def test_run_c_area_announces_distinct_skip_reasons():
    ctx = FakeContext()
    ctx.c_plan = {
        "zone": "C",
        "task_count": 3,
        "tasks": [
            {"sequence": 1, "position_id": 4, "target": "洋葱"},
            {"sequence": 2, "position_id": 3, "target": "南瓜"},
            {"sequence": 3, "position_id": 1, "target": "西红柿"},
        ],
    }
    distribution = scan_c_qr_and_build_table(ctx)
    ctx.actions.clear()
    ctx.observations = [
        SideObservation(left=[], right=[]),
        SideObservation(left=["pumpkin_unripe"], right=[]),
        SideObservation(left=["apple_ripe"], right=[]),
    ]

    run_c_area(ctx, distribution)

    announcements = [action[1] for action in ctx.actions if action[0] == "announce"]
    assert any("未识别到目标" in text for text in announcements)
    assert any("目标南瓜未成熟" in text for text in announcements)
    assert any("识别到其他类型" in text for text in announcements)
    assert distribution["positions"]["4"]["result"] == "no_detection"
    assert distribution["positions"]["3"]["result"] == "target_unripe"
    assert distribution["positions"]["1"]["result"] == "type_mismatch"


def test_run_c_area_records_closed_loop_failure_and_advances():
    ctx = FakeContext()
    distribution = scan_c_qr_and_build_table(ctx)
    ctx.actions.clear()
    ctx.observations = [
        SideObservation(left=[], right=["onion_ripe"]),
        SideObservation(left=["pumpkin_ripe"], right=[]),
    ]
    ctx.closed_loop_results = [
        PickExecutionResult(False, "arm_callback_timeout", "onion_ripe"),
        PickExecutionResult(True, "picked", "pumpkin_ripe"),
    ]

    run_c_area(ctx, distribution)

    assert ("closed_loop_pick", "right", "onion_ripe") in ctx.actions
    assert ("closed_loop_pick", "left", "pumpkin_ripe") in ctx.actions
    assert distribution["positions"]["4"]["result"] == "arm_callback_timeout"
    assert distribution["positions"]["3"]["result"] == "picked"
