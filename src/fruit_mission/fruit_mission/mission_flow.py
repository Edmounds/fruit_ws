"""Readable A/B/C mission flow functions."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from fruit_mission.logic import choose_side
from fruit_mission.logic import expected_ripe_type
from fruit_mission.logic import format_b_announcement
from fruit_mission.logic import format_c_announcement
from fruit_mission.logic import format_position_name
from fruit_mission.logic import PickDecision
from fruit_mission.logic import SideObservation
from fruit_mission.logic import spoken_model_type
from fruit_mission.logic import without_tree_fruits


def run_mission(ctx) -> None:
    """Run the first A -> B QR -> C QR -> C mission slice."""
    try:
        ctx.announce("水果采摘机器人准备开始比赛。")
        if not run_a_area(ctx):
            return
        if not _send_arm_retract(ctx, "A区结束机械臂收缩姿态动作超时，任务停止。"):
            return
        if scan_b_qr_and_announce(ctx) is None:
            return
        c_distribution = scan_c_qr_and_build_table(ctx)
        if c_distribution is not None:
            run_c_area(ctx, c_distribution)
    finally:
        ctx.set_perception_mode("OFF")


def run_a_area(ctx) -> bool:
    """Run A-zone stop-and-pick points with side cameras."""
    if not ctx.report_current_area("A"):
        ctx.announce("A区当前区域上报失败，任务停止。")
        return False
    if not ctx.set_perception_mode("SIDES"):
        ctx.announce("A区侧相机模式切换失败，任务停止。")
        return False
    for waypoint in ctx.route.a_goal_names():
        if not ctx.go_to(waypoint):
            ctx.announce("A区导航到当前采摘点失败，跳过。")
            continue
        ctx.stable_stop()
        observation = ctx.observe_sides(float(ctx.config["side_observation_timeout_sec"]))
        filtered_observation = without_tree_fruits(observation)
        decisions = _a_pick_decisions(
            filtered_observation,
            preferred_side=str(ctx.config.get("preferred_side", "left")),
        )
        if not decisions:
            ctx.announce(_a_skip_text(filtered_observation))
            continue
        for index, decision in enumerate(decisions):
            result = ctx.execute_closed_loop_pick(decision.side, decision.matched_type)
            if result.success:
                ctx.announce(
                    f"A区识别到{spoken_model_type(decision.matched_type)}，抓取完成。"
                )
            else:
                next_text = (
                    "继续下一侧。"
                    if index + 1 < len(decisions)
                    else "继续下一个采摘点。"
                )
                ctx.announce(
                    f"A区抓取失败，{_pick_failure_text(result.reason_code)}。{next_text}"
                )
        if not ctx.set_perception_mode("SIDES"):
            ctx.announce("A区侧相机模式恢复失败，任务停止。")
            return False
    return True


def _a_pick_decisions(
    observation: SideObservation,
    preferred_side: str,
) -> list[PickDecision]:
    first = choose_side(observation, preferred_side=preferred_side)
    if not first.should_pick or first.side is None:
        return []

    remaining = SideObservation(
        left=[] if first.side == "left" else observation.left,
        right=[] if first.side == "right" else observation.right,
    )
    second = choose_side(remaining, preferred_side=preferred_side)
    if second.should_pick and second.side is not None:
        return [first, second]
    return [first]


def _a_skip_text(observation: SideObservation) -> str:
    seen_types = list(observation.left) + list(observation.right)
    if not seen_types:
        return "A区没有识别到目标，跳过。"
    if all(item.endswith("_unripe") for item in seen_types):
        return "A区识别到的目标都不成熟，跳过。"
    return "A区没有识别到成熟目标，跳过。"


def _send_arm_retract(ctx, failure_text: str) -> bool:
    if ctx.send_arm_retract_pose():
        return True
    ctx.announce(failure_text)
    return False


def scan_b_qr_and_announce(ctx) -> Optional[Dict[str, Any]]:
    """Scan B QR and announce its ordered plan."""
    if not ctx.go_to(ctx.route.b_qr_name):
        ctx.announce("B区二维码点导航失败，任务停止。")
        return None
    ctx.stable_stop()
    if not ctx.report_current_area("B"):
        ctx.announce("B区当前区域上报失败，任务停止。")
        return None
    if not ctx.set_perception_mode("QR"):
        ctx.announce("B区二维码模式切换失败，任务停止。")
        return None
    if not ctx.send_qr_pose():
        ctx.announce("B区二维码识别姿态动作超时，任务停止。")
        return None
    b_plan = ctx.wait_b_plan()
    if b_plan is None:
        ctx.announce("B区二维码计划等待超时。")
        return None
    _announce_and_wait(ctx, format_b_announcement(b_plan))
    if not _send_arm_retract(ctx, "B区二维码后机械臂收缩姿态动作超时，任务停止。"):
        return None
    return b_plan


def scan_c_qr_and_build_table(ctx) -> Optional[Dict[str, Any]]:
    """Scan C QR, save the planned task list, and announce task order."""
    if not ctx.go_to(ctx.route.c_qr_name):
        ctx.announce("C区二维码点导航失败，任务停止。")
        return None
    ctx.stable_stop()
    if not ctx.report_current_area("C"):
        ctx.announce("C区当前区域上报失败，任务停止。")
        return None
    if not ctx.set_perception_mode("QR"):
        ctx.announce("C区二维码模式切换失败，任务停止。")
        return None
    if not ctx.send_qr_pose():
        ctx.announce("C区二维码识别姿态动作超时，任务停止。")
        return None
    c_plan = ctx.wait_c_plan()
    if c_plan is None:
        ctx.announce("C区二维码计划等待超时。")
        return None

    distribution = ctx.route.build_c_distribution(c_plan)
    ctx.save_c_distribution(distribution)
    _announce_and_wait(ctx, format_c_announcement(c_plan))
    if not _send_arm_retract(ctx, "C区二维码后机械臂收缩姿态动作超时，任务停止。"):
        return None
    return distribution


def _announce_and_wait(ctx, text: str) -> bool:
    announce_and_wait = getattr(ctx, "announce_and_wait", None)
    if callable(announce_and_wait):
        return bool(announce_and_wait(text))
    return bool(ctx.announce(text))


def run_c_area(ctx, c_distribution: Mapping[str, Any]) -> bool:
    """Execute C-zone tasks strictly in QR order."""
    if not ctx.set_perception_mode("SIDES"):
        ctx.announce("C区侧相机模式切换失败，任务停止。")
        return False
    for task in ordered_c_tasks_from_distribution(c_distribution):
        position_id = int(task["position_id"])
        target_name = str(task["target_name"])
        movement_points = list(task["movement_points"])
        camera_side = str(task.get("camera_side", task["preferred_side"]))
        if not _go_through_c_route(ctx, movement_points):
            task["result"] = "navigation_failed"
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                "导航失败，跳过。"
            )
            continue
        ctx.stable_stop()

        observation = ctx.observe_sides(float(ctx.config["side_observation_timeout_sec"]))
        task["last_observation"] = {
            "left": observation.left,
            "right": observation.right,
        }
        filtered_observation = _observation_for_camera_side(observation, camera_side)
        filtered_observation = without_tree_fruits(filtered_observation)
        decision = choose_side(
            filtered_observation,
            expected_ripe_type(target_name),
            preferred_side=camera_side,
        )
        if not decision.should_pick:
            task["result"] = decision.reason_code
            ctx.announce(
                _c_skip_text(
                    position_id,
                    target_name,
                    decision.reason_code,
                    filtered_observation,
                )
            )
            continue

        result = ctx.execute_closed_loop_pick(decision.side, expected_ripe_type(target_name))
        if result.success:
            task["result"] = "picked"
            observed = spoken_model_type(decision.matched_type)
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                f"识别到{observed}，抓取完成。"
            )
        else:
            task["result"] = result.reason_code
            observed = spoken_model_type(decision.matched_type)
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                f"识别到{observed}，但{_pick_failure_text(result.reason_code)}。"
                "继续下一项。"
            )
        if not _send_arm_retract(
            ctx,
            f"C区{format_position_name(position_id)}，机械臂收缩姿态动作超时，任务停止。",
        ):
            ctx.save_c_distribution(c_distribution)
            return False
        if not ctx.set_perception_mode("SIDES"):
            ctx.announce("C区侧相机模式恢复失败，任务停止。")
            ctx.save_c_distribution(c_distribution)
            return False

    ctx.save_c_distribution(c_distribution)
    if not _send_arm_retract(ctx, "C区结束机械臂收缩姿态动作超时，任务停止。"):
        return False
    return True


def _go_through_c_route(ctx, movement_points: list[str]) -> bool:
    go_through = getattr(ctx, "go_through", None)
    if callable(go_through):
        return bool(go_through(movement_points))
    return bool(ctx.go_to(movement_points))


def ordered_c_tasks_from_distribution(
    c_distribution: Mapping[str, Any],
) -> list[Mapping[str, Any]]:
    """Return planned C tasks in QR order."""
    tasks = list(c_distribution["tasks"])
    tasks.sort(key=lambda item: int(item["sequence"]))
    return tasks


def _observation_for_camera_side(
    observation: SideObservation,
    camera_side: str,
) -> SideObservation:
    if camera_side == "left":
        return SideObservation(left=observation.left, right=[])
    if camera_side == "right":
        return SideObservation(left=[], right=observation.right)
    raise ValueError("camera_side must be left or right")


def _c_skip_text(
    position_id: int,
    target_name: str,
    reason_code: str,
    observation: SideObservation,
) -> str:
    prefix = f"C区{format_position_name(position_id)}，应该采{target_name}。"
    observed = _first_observed_text(observation, target_name, reason_code)
    if reason_code == "no_detection":
        return prefix + "没有识别到目标，跳过。"
    if reason_code == "target_unripe":
        return prefix + f"识别到{observed or '未成熟' + target_name}，跳过。"
    if reason_code == "type_mismatch":
        return prefix + f"识别到{observed or '其他目标'}，不是{target_name}，跳过。"
    if observed:
        return prefix + f"识别到{observed}，不是成熟{target_name}，跳过。"
    return prefix + f"没有识别到成熟{target_name}，跳过。"


def _first_observed_text(
    observation: SideObservation,
    target_name: str,
    reason_code: str,
) -> Optional[str]:
    seen_types = list(observation.left) + list(observation.right)
    if not seen_types:
        return None
    if reason_code == "target_unripe":
        prefix = expected_ripe_type(target_name).rsplit("_", 1)[0]
        target_unripe_type = f"{prefix}_unripe"
        if target_unripe_type in seen_types:
            return spoken_model_type(target_unripe_type)
    return spoken_model_type(seen_types[0])


def _pick_failure_text(reason_code: str) -> str:
    messages = {
        "sides_mode_timeout": "侧向相机识别模式切换失败",
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "tree_mode_timeout": "深度相机识别模式切换失败",
        "d435i_target_timeout": "深度相机没有识别到可抓取目标",
        "arm_callback_timeout": "机械臂抓取没有返回完成信号",
    }
    return messages.get(reason_code, "闭环抓取失败")
