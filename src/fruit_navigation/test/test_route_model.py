import math

import pytest

from fruit_navigation.route_model import (
    AnchorSession,
    PoseData,
    build_c_route,
    generate_ab_points,
    generate_c_candidates,
    mission_route,
    parse_c_pick_plan,
    quaternion_from_yaw,
)


def pose(name, x, y, yaw=0.0):
    q = quaternion_from_yaw(yaw)
    return PoseData(
        name=name,
        x=x,
        y=y,
        z=0.0,
        qx=q["x"],
        qy=q["y"],
        qz=q["z"],
        qw=q["w"],
        yaw=yaw,
    )


def test_generate_ab_points_uses_first_last_and_thirds():
    points = generate_ab_points("A", pose("A_FIRST", 0.0, 0.0), pose("A_LAST", 0.0, 3.0))

    assert [point.name for point in points] == ["A1", "A2", "A3", "A4"]
    assert [point.y for point in points] == [0.0, 1.0, 2.0, 3.0]


def test_generate_ab_points_rejects_opposite_yaw():
    with pytest.raises(ValueError):
        generate_ab_points(
            "B",
            pose("B_FIRST", 0.0, 0.0, 0.0),
            pose("B_LAST", 1.0, 0.0, math.pi),
            yaw_tolerance=0.2,
        )


def test_generate_c_candidates_from_four_corners_in_expected_numbering():
    candidates = generate_c_candidates(
        pose("C1", 0.0, 0.0),
        pose("C4", 0.0, 3.0),
        pose("C9", 2.0, 0.0),
        pose("C12", 2.0, 3.0),
    )

    assert list(candidates) == [f"C{index}" for index in range(1, 13)]
    assert (candidates["C1"].x, candidates["C1"].y) == (0.0, 0.0)
    assert (candidates["C5"].x, candidates["C5"].y) == (1.0, 0.0)
    assert (candidates["C8"].x, candidates["C8"].y) == (1.0, 3.0)
    assert (candidates["C12"].x, candidates["C12"].y) == (2.0, 3.0)


def test_parse_c_pick_plan_preserves_qr_order_and_rejects_bad_ids():
    plan = {
        "zone": "C",
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

    tasks = parse_c_pick_plan(plan)

    assert [task.position_id for task in tasks] == [4, 3, 1, 10, 8, 9, 2, 11]
    assert [task.sequence for task in tasks] == list(range(1, 9))

    plan["tasks"][-1]["position_id"] = 4
    with pytest.raises(ValueError):
        parse_c_pick_plan(plan)


def test_build_c_route_includes_c_qr_then_selected_candidates():
    candidates = generate_c_candidates(
        pose("C1", 0.0, 0.0),
        pose("C4", 0.0, 3.0),
        pose("C9", 2.0, 0.0),
        pose("C12", 2.0, 3.0),
    )
    tasks = parse_c_pick_plan(
        {
            "zone": "C",
            "tasks": [
                {"sequence": i, "position_id": pid, "target": str(pid)}
                for i, pid in enumerate([4, 3, 1, 10, 8, 9, 2, 11], start=1)
            ],
        }
    )

    route = build_c_route(pose("C_QR", -1.0, -1.0), candidates, tasks)

    assert [point.name for point in route] == [
        "C_QR",
        "C4",
        "C3",
        "C1",
        "C10",
        "C8",
        "C9",
        "C2",
        "C11",
    ]


def test_anchor_session_records_eleven_prompts_and_generates_route():
    session = AnchorSession()
    for index, name in enumerate(session.required_anchor_names):
        assert session.next_prompt().endswith(name)
        session.append(pose(name, float(index), float(index)))

    document = session.to_document()

    assert document["schema_version"] == 1
    assert list(document["anchors"]) == list(session.required_anchor_names)
    assert len(document["generated"]["A"]) == 4
    assert len(document["generated"]["B"]) == 4
    assert len(document["generated"]["C"]) == 12
    assert session.next_prompt() == "All anchors recorded"


def test_mission_route_uses_home_only_as_return_target():
    anchors = {
        "HOME": pose("HOME", 0.0, 0.0),
        "B_QR": pose("B_QR", 1.0, 0.0),
        "C_QR": pose("C_QR", 2.0, 0.0),
    }
    generated = {
        "A": [pose(f"A{i}", float(i), 1.0) for i in range(1, 5)],
        "B": [pose(f"B{i}", float(i), 2.0) for i in range(1, 5)],
        "C": {f"C{i}": pose(f"C{i}", float(i), 3.0) for i in range(1, 13)},
    }
    tasks = parse_c_pick_plan(
        {
            "zone": "C",
            "tasks": [
                {"sequence": i, "position_id": pid, "target": str(pid)}
                for i, pid in enumerate([4, 3, 1, 10, 8, 9, 2, 11], start=1)
            ],
        }
    )

    route = mission_route(anchors, generated, tasks)

    assert route[0].name == "A1"
    assert route[-1].name == "HOME"
    assert "C_QR" in [point.name for point in route]
