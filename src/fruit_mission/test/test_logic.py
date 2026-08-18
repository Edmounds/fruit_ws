import json

from fruit_mission.logic import (
    SideObservation,
    build_c_distribution,
    choose_side,
    expected_ripe_type,
    format_b_announcement,
    format_c_announcement,
    merge_seen_types,
    ordered_c_tasks,
)


def _c_positions():
    return {
        index: {
            "waypoint": f"c_{index:03d}",
            "preferred_side": "left" if index % 2 else "right",
        }
        for index in range(1, 13)
    }


def _c_plan():
    return {
        "zone": "C",
        "task_count": 8,
        "tasks": [
            {"sequence": 1, "position_id": 4, "target": "洋葱"},
            {"sequence": 2, "position_id": 3, "target": "南瓜"},
            {"sequence": 3, "position_id": 1, "target": "西红柿"},
            {"sequence": 4, "position_id": 10, "target": "辣椒"},
            {"sequence": 5, "position_id": 8, "target": "苹果"},
            {"sequence": 6, "position_id": 9, "target": "梨子"},
            {"sequence": 7, "position_id": 2, "target": "洋葱"},
            {"sequence": 8, "position_id": 11, "target": "南瓜"},
        ],
    }


def test_maps_chinese_names_to_model_ripe_types():
    assert expected_ripe_type("苹果") == "apple_ripe"
    assert expected_ripe_type("梨子") == "pear_ripe"
    assert expected_ripe_type("南瓜") == "pumpkin_ripe"
    assert expected_ripe_type("西红柿") == "tomato_ripe"
    assert expected_ripe_type("辣椒") == "chili_ripe"
    assert expected_ripe_type("洋葱") == "onion_ripe"


def test_pickable_target_requires_ripe_matching_type():
    observation = SideObservation(left=["pumpkin_unripe"], right=["pumpkin_ripe"])

    decision = choose_side(observation, "pumpkin_ripe", preferred_side="left")

    assert decision.should_pick
    assert decision.side == "right"
    assert decision.matched_type == "pumpkin_ripe"


def test_type_mismatch_and_empty_observation_are_skipped():
    mismatch = choose_side(
        SideObservation(left=["apple_ripe"], right=[]),
        "pear_ripe",
    )
    empty = choose_side(SideObservation(left=[], right=[]))

    assert not mismatch.should_pick
    assert not empty.should_pick


def test_skip_reason_distinguishes_empty_unripe_and_type_mismatch():
    empty = choose_side(SideObservation(left=[], right=[]), "pumpkin_ripe")
    unripe = choose_side(
        SideObservation(left=["pumpkin_unripe"], right=[]),
        "pumpkin_ripe",
    )
    mismatch = choose_side(
        SideObservation(left=["apple_ripe"], right=[]),
        "pumpkin_ripe",
    )

    assert empty.reason_code == "no_detection"
    assert unripe.reason_code == "target_unripe"
    assert mismatch.reason_code == "type_mismatch"


def test_a_area_both_sides_pick_one_preferred_side():
    observation = SideObservation(left=["apple_ripe"], right=["pear_ripe"])

    decision = choose_side(observation, preferred_side="right")

    assert decision.should_pick
    assert decision.side == "right"
    assert decision.matched_type == "pear_ripe"


def test_seen_types_keep_transient_detections_when_later_frames_are_empty():
    seen = merge_seen_types([], ["apple_ripe"])

    assert merge_seen_types(seen, []) == ["apple_ripe"]
    assert merge_seen_types(seen, ["pear_ripe", "apple_ripe"]) == [
        "apple_ripe",
        "pear_ripe",
    ]


def test_c_distribution_has_twelve_positions_and_qr_targets():
    distribution = build_c_distribution(_c_plan(), _c_positions())
    positions = distribution["positions"]

    assert set(positions) == {str(index) for index in range(1, 13)}
    assert positions["4"]["task_index"] == 1
    assert positions["4"]["target_name"] == "洋葱"
    assert positions["4"]["expected_ripe_type"] == "onion_ripe"
    assert positions["5"]["target_name"] is None


def test_c_tasks_keep_qr_order_not_position_order():
    tasks = ordered_c_tasks(_c_plan())

    assert [task["position_id"] for task in tasks] == [4, 3, 1, 10, 8, 9, 2, 11]


def test_c_distribution_is_json_serializable_for_runtime_logs():
    distribution = build_c_distribution(_c_plan(), _c_positions())

    encoded = json.dumps(distribution, ensure_ascii=False)

    assert "洋葱" in encoded


def test_announcements_include_ordered_targets():
    b_plan = {
        "tasks": [
            {"position_id": 1, "target": "苹果"},
            {"position_id": 2, "target": "梨子"},
        ]
    }

    assert format_b_announcement(b_plan) == "B区顺序：一号苹果，二号梨子"
    assert "四号位置，目标洋葱" in format_c_announcement(_c_plan())
