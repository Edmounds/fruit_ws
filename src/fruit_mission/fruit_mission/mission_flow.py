"""Readable A/B/C mission flow functions."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from fruit_mission.logic import choose_maturity_side
from fruit_mission.logic import expected_ripe_type
from fruit_mission.logic import format_b_announcement
from fruit_mission.logic import format_c_announcement
from fruit_mission.logic import format_position_name
from fruit_mission.logic import PickDecision
from fruit_mission.logic import SideObservation
from fruit_mission.logic import spoken_model_type


def run_mission(ctx) -> None:
    """Run the selected field flow using the configured waypoint names."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    mission_complete = False
    try:
        ctx.announce("Bobocup队开始比赛")
        if flow not in {"full", "ac", "a", "b", "c"}:
            ctx.announce(f"未知任务流程{flow}，任务停止。")
            return

        if flow in {"full", "ac", "a"}:
            if not run_a_area(ctx) or not _send_arm_retract(
                ctx, "A区结束机械臂收缩姿态动作超时，任务停止。"
            ):
                return
            if flow == "a":
                mission_complete = ctx.navigate_named(["to_B_QR_waypoint"])
                return

            b_plan = scan_b_qr_and_announce(ctx, announce_before_plan=False)
            if b_plan is None:
                return
            c_distribution = scan_c_qr_and_build_table(ctx)
            if c_distribution is None or not run_c_area(ctx, c_distribution):
                return
            if flow == "ac":
                mission_complete = True
                return

            if not ctx.navigate_named(
                ctx.route.transition_names("c_to_b_first_tree")
            ):
                ctx.announce("C区返回B区导航失败，任务停止。")
                return
            mission_complete = run_b_area(ctx, b_plan)
            return

        if flow == "b":
            b_plan = scan_b_qr_and_announce(ctx, announce_before_plan=True)
            if b_plan is None:
                return
            if not ctx.navigate_named(
                ctx.route.transition_names("b_to_first_tree")
            ):
                ctx.announce("B区第一个树位导航失败，任务停止。")
                return
            if not run_b_area(ctx, b_plan):
                return
            mission_complete = ctx.navigate_named(
                ctx.route.transition_names("b_to_c_qr_end")
            )
            return

        c_distribution = scan_c_qr_and_build_table(ctx)
        if c_distribution is not None:
            mission_complete = run_c_area(ctx, c_distribution)
    finally:
        ctx.set_perception_mode("OFF")
        if mission_complete:
            ctx.announce("任务结束")


def run_a_area(ctx) -> bool:
    """Run A-zone stop-and-pick points with side cameras."""
    if not ctx.report_current_area("A"):
        return False
    if not ctx.set_perception_mode("SIDES"):
        return False
    for waypoint in ctx.route.a_goal_names():
        if not ctx.go_to(waypoint):
            continue
        ctx.stable_stop()
        observation = ctx.observe_sides(float(ctx.config["side_observation_timeout_sec"]))
        decisions = _a_pick_decisions(
            observation,
            preferred_side=str(ctx.config.get("preferred_side", "left")),
        )
        if not decisions:
            continue
        for decision in decisions:
            ctx.execute_closed_loop_pick(decision.side)
        if not ctx.set_perception_mode("SIDES"):
            return False
    return ctx.set_perception_mode("OFF")


def _a_pick_decisions(
    observation: SideObservation,
    preferred_side: str,
) -> list[PickDecision]:
    first = choose_maturity_side(observation, preferred_side)
    if not first.should_pick or first.side is None:
        return []

    remaining = SideObservation(
        left=[] if first.side == "left" else observation.left,
        right=[] if first.side == "right" else observation.right,
    )
    second = choose_maturity_side(remaining, preferred_side)
    if second.should_pick and second.side is not None:
        return [first, second]
    return [first]


def _send_arm_retract(
    ctx,
    failure_text: str,
) -> bool:
    if ctx.send_arm_retract_pose():
        return True
    ctx.announce(failure_text)
    return False


def scan_b_qr_and_announce(
    ctx,
    *,
    announce_before_plan: bool = True,
) -> Optional[Dict[str, Any]]:
    """Scan B QR and announce its ordered plan."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    route_names = (
        [ctx.route.b_qr_name]
        if flow == "b"
        else ctx.route.transition_names("a_to_b_qr")
    )
    if not ctx.navigate_named(route_names):
        if announce_before_plan:
            ctx.announce("B区二维码点导航失败，任务停止。")
        return None
    ctx.stable_stop()
    if not ctx.report_current_area("B"):
        if announce_before_plan:
            ctx.announce("B区当前区域上报失败，任务停止。")
        return None
    if not ctx.set_perception_mode("QR"):
        if announce_before_plan:
            ctx.announce("B区二维码模式切换失败，任务停止。")
        return None
    if not ctx.send_qr_pose():
        if announce_before_plan:
            ctx.announce("B区二维码识别姿态动作超时，任务停止。")
        return None
    b_plan = ctx.wait_b_plan()
    if b_plan is None:
        if announce_before_plan:
            ctx.announce("B区二维码计划等待超时。")
        return None
    save_b_plan = getattr(ctx, "save_b_plan", None)
    if callable(save_b_plan):
        save_b_plan(b_plan)
    _announce_and_wait(ctx, format_b_announcement(b_plan))
    if not _send_arm_retract(ctx, "B区二维码后机械臂收缩姿态动作超时，任务停止。"):
        return None
    return b_plan


def scan_c_qr_and_build_table(ctx) -> Optional[Dict[str, Any]]:
    """Scan C QR, save the planned task list, and announce task order."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    route_names = (
        [ctx.route.c_qr_name]
        if flow == "c"
        else ctx.route.transition_names("b_to_c_qr")
    )
    if not ctx.navigate_named(route_names):
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


def run_b_area(ctx, b_plan: Mapping[str, Any]) -> bool:
    """Execute B tree tasks in the QR order, using vehicle-side directions."""
    if not ctx.report_current_area("B"):
        return False
    tasks = list(b_plan.get("tasks", []))
    tasks.sort(key=lambda item: int(item["sequence"]))
    b_names = ctx.route.b_goal_names()
    for task in tasks:
        position_id = int(task["position_id"])
        if not 1 <= position_id <= len(b_names):
            ctx.announce(f"B区第{position_id}棵树编号无效，跳过。")
            continue
        goal_name = b_names[position_id - 1]
        if not ctx.navigate_named([goal_name]):
            ctx.announce(f"B区{goal_name}导航失败，跳过。")
            continue
        ctx.stable_stop()
        target_name = str(task["target"])
        expected_type = expected_ripe_type(target_name)
        side = "right" if position_id % 2 == 1 else "left"
        result = ctx.execute_b_tree_pick(side, expected_type)
        picked = int(result.get("picked", 0))
        if result.get("success"):
            ctx.announce(f"B区{goal_name}，向{side}完成{picked}个目标抓取。")
        else:
            ctx.announce(
                f"B区{goal_name}，向{side}未完成目标抓取，继续下一棵。"
            )
        if not _send_arm_retract(
            ctx, f"B区{goal_name}机械臂收缩姿态动作超时，任务停止。"
        ):
            return False
    return _send_arm_retract(ctx, "B区结束机械臂收缩姿态动作超时，任务停止。")


def run_c_area(ctx, c_distribution: Mapping[str, Any]) -> bool:
    """Execute C-zone tasks strictly in QR order."""
    for task in ordered_c_tasks_from_distribution(c_distribution):
        position_id = int(task["position_id"])
        target_name = str(task["target_name"])
        expected_type = str(
            task.get("expected_ripe_type") or expected_ripe_type(target_name)
        )
        navigation = _navigate_c_task(ctx, task)
        task["navigation_status"] = navigation.get("status")
        for key in (
            "navigation_start",
            "navigation_start_source",
            "goal_name",
            "position_side",
            "graph_path",
            "movement_points",
            "navigation_yaws",
            "route_distance",
            "camera_side",
            "preferred_side",
        ):
            if key in navigation:
                task[key] = navigation[key]

        if navigation["status"] == "planning_failed":
            task["result"] = "navigation_planning_failed"
            task["navigation_failure_reason"] = navigation.get("reason")
            task["navigation_failure_detail"] = navigation.get("detail")
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                "无法根据当前位置规划安全路线，C区任务停止。"
            )
            ctx.save_c_distribution(c_distribution)
            _send_arm_retract(ctx, "C区导航规划失败，机械臂收缩姿态动作超时。")
            return False

        if navigation["status"] == "navigation_failed":
            task["result"] = "navigation_failed"
            task["navigation_failure_reason"] = navigation.get("reason")
            task["navigation_failure_detail"] = navigation.get("detail")
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                "导航失败，跳过并从当前位置规划下一项。"
            )
            ctx.save_c_distribution(c_distribution)
            if not bool(navigation.get("recovery_available", True)):
                ctx.announce("C区当前位置无法用于重新规划，C区任务停止。")
                _send_arm_retract(ctx, "C区导航恢复失败，机械臂收缩姿态动作超时。")
                return False
            continue

        camera_side = str(navigation["camera_side"])
        ctx.stable_stop()
        task["last_observation"] = {
            "source": "d435i",
            "expected_type": expected_type,
        }
        result = ctx.execute_closed_loop_pick(camera_side, expected_type)
        if result.success:
            task["result"] = "picked"
            observed = spoken_model_type(result.matched_type)
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                f"识别到{observed}，抓取完成。"
            )
        else:
            task["result"] = result.reason_code
            if result.reason_code in {
                "no_detection",
                "target_unripe",
                "type_mismatch",
            }:
                ctx.announce(
                    _c_skip_text(
                        position_id,
                        target_name,
                        result.reason_code,
                        result.matched_type,
                    )
                )
            else:
                observed = spoken_model_type(result.matched_type)
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

    ctx.save_c_distribution(c_distribution)
    if not _send_arm_retract(ctx, "C区结束机械臂收缩姿态动作超时，任务停止。"):
        return False
    return True


def _navigate_c_task(ctx, task: Mapping[str, Any]) -> Dict[str, Any]:
    navigate = getattr(ctx, "navigate_c_position", None)
    if callable(navigate):
        return dict(navigate(int(task["position_id"])))

    movement_points = list(task.get("movement_points", []))
    go_through = getattr(ctx, "go_through", None)
    success = (
        bool(go_through(movement_points))
        if callable(go_through)
        else bool(ctx.go_to(movement_points))
    )
    return {
        "status": "succeeded" if success else "navigation_failed",
        "reason": None if success else "follow_waypoints_failed",
        "movement_points": movement_points,
        "camera_side": task.get("camera_side", task.get("preferred_side", "left")),
    }


def ordered_c_tasks_from_distribution(
    c_distribution: Mapping[str, Any],
) -> list[Dict[str, Any]]:
    """Return planned C tasks in QR order."""
    tasks: list[Dict[str, Any]] = list(c_distribution["tasks"])
    tasks.sort(key=lambda item: int(item["sequence"]))
    return tasks


def _c_skip_text(
    position_id: int,
    target_name: str,
    reason_code: str,
    matched_type: Optional[str],
) -> str:
    prefix = f"C区{format_position_name(position_id)}，应该采{target_name}。"
    observed = spoken_model_type(matched_type) if matched_type else None
    if reason_code == "no_detection":
        return prefix + "没有识别到目标，跳过。"
    if reason_code == "target_unripe":
        return prefix + f"识别到{observed or '未成熟' + target_name}，跳过。"
    if reason_code == "type_mismatch":
        return prefix + f"识别到{observed or '其他目标'}，不是{target_name}，跳过。"
    if observed:
        return prefix + f"识别到{observed}，不是成熟{target_name}，跳过。"
    return prefix + f"没有识别到成熟{target_name}，跳过。"


def _pick_failure_text(reason_code: str) -> str:
    messages = {
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "tree_mode_timeout": "深度相机识别模式切换失败",
        "d435i_target_timeout": "深度相机没有识别到可抓取目标",
        "arm_callback_timeout": "机械臂抓取没有返回完成信号",
    }
    return messages.get(reason_code, "闭环抓取失败")
