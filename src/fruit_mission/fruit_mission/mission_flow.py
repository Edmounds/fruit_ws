"""Readable A/B/C mission flow functions."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from fruit_mission.logic import choose_maturity_side
from fruit_mission.logic import decide_b_tree_targets
from fruit_mission.logic import expected_ripe_type
from fruit_mission.logic import format_b_announcement
from fruit_mission.logic import format_c_announcement
from fruit_mission.logic import format_position_name
from fruit_mission.logic import PickDecision
from fruit_mission.logic import SideObservation
from fruit_mission.logic import spoken_model_type
from fruit_qr_planner.planner import build_b_plan
from fruit_qr_planner.planner import build_c_plan


def run_mission(ctx) -> None:
    """Run the selected field flow using the configured waypoint names."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    mission_complete = False
    try:
        _announce_and_wait(
            ctx,
            "我们是重庆大学 bobocup 水果采摘队伍，现在开始采摘",
        )
        if flow not in {"full", "ac", "a", "b", "c"}:
            ctx.announce(f"未知任务流程{flow}，任务停止。")
            return

        if flow in {"full", "ac", "a"}:
            if not run_a_area(ctx) or not _send_arm_retract(
                ctx,
                "A区结束机械臂收缩姿态动作失败，任务停止。",
                continue_on_timeout=True,
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
        if not ctx.navigate_named([waypoint]):
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
            result = ctx.execute_closed_loop_pick(decision.side)
            if not result.success and not result.safe_to_continue:
                return False
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
    *,
    continue_on_timeout: bool = False,
) -> bool:
    if ctx.send_arm_retract_pose(continue_on_timeout=continue_on_timeout):
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
        fallback_targets = ctx.config.get("b_qr_fallback_targets")
        if not fallback_targets:
            if announce_before_plan:
                ctx.announce("B区二维码计划等待超时。")
            return None
        ctx.node.get_logger().warning(
            "B区二维码计划等待超时，使用配置的兜底顺序"
        )
        b_plan = build_b_plan("\n".join(str(target) for target in fallback_targets))
    save_b_plan = getattr(ctx, "save_b_plan", None)
    if callable(save_b_plan):
        save_b_plan(b_plan)
    _announce_qr(
        ctx,
        format_b_announcement(b_plan),
    )
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
        fallback_targets = ctx.config.get("c_qr_fallback_targets")
        fallback_positions = ctx.config.get("c_qr_fallback_positions")
        if not fallback_targets or not fallback_positions:
            ctx.announce("C区二维码计划等待超时。")
            return None
        ctx.node.get_logger().warning(
            "C区二维码计划等待超时，使用配置的兜底顺序"
        )
        fallback_lines = [str(target) for target in fallback_targets]
        fallback_lines.append(
            ",".join(str(position) for position in fallback_positions)
        )
        c_plan = build_c_plan("\n".join(fallback_lines))

    distribution = ctx.route.build_c_distribution(c_plan)
    ctx.save_c_distribution(distribution)
    _announce_and_wait(
        ctx,
        format_c_announcement(c_plan),
        lead_time_sec=float(ctx.config.get("c_qr_announce_lead_sec", 4.0)),
        length_scale=_qr_length_scale(ctx),
    )
    if not _send_arm_retract(ctx, "C区二维码后机械臂收缩姿态动作超时，任务停止。"):
        return None
    return distribution


def _announce_and_wait(
    ctx,
    text: str,
    *,
    lead_time_sec: float = 0.0,
    length_scale: Optional[float] = None,
) -> bool:
    announce_and_wait = getattr(ctx, "announce_and_wait", None)
    if callable(announce_and_wait):
        if lead_time_sec > 0.0:
            try:
                return bool(
                    announce_and_wait(
                        text,
                        lead_time_sec=lead_time_sec,
                        length_scale=length_scale,
                    )
                )
            except TypeError:
                pass
        if length_scale is not None:
            try:
                return bool(announce_and_wait(text, length_scale=length_scale))
            except TypeError:
                pass
        return bool(announce_and_wait(text))
    return bool(ctx.announce(text))


def _announce_qr(ctx, text: str) -> bool:
    """Queue a compact QR summary with the configured faster speech rate."""
    length_scale = _qr_length_scale(ctx)
    try:
        return bool(ctx.announce(text, length_scale=length_scale))
    except TypeError:
        return bool(ctx.announce(text))


def _qr_length_scale(ctx) -> float:
    return float(ctx.config.get("tts", {}).get("qr_length_scale", 0.8))


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
        observation = ctx.observe_b_tree(side)
        if not observation.get("success"):
            reason_code = str(observation.get("reason_code"))
            reason = _b_observation_failure_text(reason_code)
            ctx.announce(f"B区{goal_name}，{reason}，跳过。")
        else:
            targets = list(observation.get("targets", []))
            selected_target = _announce_b_tree_decision(
                ctx,
                goal_name,
                target_name,
                expected_type,
                targets,
            )
            if selected_target is not None:
                pick_result = ctx.execute_b_tree_target(selected_target)
                if not pick_result.get("success"):
                    reason_code = str(pick_result.get("reason_code"))
                    reason = _b_arm_failure_text(reason_code)
                    ctx.announce(f"B区{goal_name}，{reason}，执行收臂。")
        if not _send_arm_retract(
            ctx, f"B区{goal_name}机械臂收缩姿态动作超时，任务停止。"
        ):
            return False
    return _send_arm_retract(ctx, "B区结束机械臂收缩姿态动作超时，任务停止。")


def _announce_b_tree_decision(
    ctx,
    goal_name: str,
    target_name: str,
    expected_type: str,
    targets: list[Any],
) -> Optional[Any]:
    decision = decide_b_tree_targets(
        target_name,
        expected_type,
        (
            (
                target.model_type,
                target.pixel_x,
                target.pixel_y,
                target.image_width,
            )
            for target in targets
        ),
    )
    if decision.reason_code == "no_detection":
        ctx.announce(f"B区{goal_name}，没有识别到果实，跳过。")
        return None
    if decision.reason_code == "inconsistent_types":
        ctx.announce(f"B区{goal_name}，识别结果不一致，跳过。")
        return None
    if decision.reason_code == "type_mismatch":
        ctx.announce(
            f"B区{goal_name}，识别到{decision.observed_name}，"
            "和二维码不一致，跳过。"
        )
        return None

    if decision.target_index is not None:
        ctx.announce(
            f"B区{goal_name}，识别到{decision.observed_name}，和二维码一致，"
            f"开始抓成熟{target_name}。"
        )
    else:
        ctx.announce(
            f"B区{goal_name}，识别到{decision.observed_name}，和二维码一致，"
            f"没有成熟{target_name}，跳过。"
        )
        return None
    return targets[decision.target_index]


def _b_observation_failure_text(reason_code: str) -> str:
    messages = {
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "tree_mode_timeout": "深度相机识别模式切换失败",
        "d435i_target_timeout": "识别超时",
    }
    return messages.get(reason_code, "识别失败")


def _b_arm_failure_text(reason_code: str) -> str:
    messages = {
        "arm_callback_failed": "机械臂执行报错",
        "arm_callback_timeout": "机械臂执行通信超时",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
    }
    return messages.get(reason_code, "机械臂执行失败")


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
            "arrival_yaw",
            "arrival_yaw_source",
            "route_rejoin",
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
                "导航重试。"
            )
            ctx.save_c_distribution(c_distribution)
            if not bool(navigation.get("recovery_available", True)):
                ctx.announce("C区当前位置无法用于重新规划，C区任务停止。")
                _send_arm_retract(ctx, "C区导航恢复失败，机械臂收缩姿态动作超时。")
                return False

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
                    f"识别到{observed}，{_pick_failure_text(result.reason_code)}。"
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
        return prefix + f"识别到{observed or '不成熟' + target_name}，跳过。"
    if reason_code == "type_mismatch":
        return prefix + f"识别到{observed or '其他目标'}，跳过。"
    if observed:
        return prefix + f"识别到{observed}，跳过。"
    return prefix + f"没有识别到成熟{target_name}，跳过。"


def _pick_failure_text(reason_code: str) -> str:
    messages = {
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "tree_mode_timeout": "深度相机识别模式切换失败",
        "d435i_target_timeout": "深度相机没有识别到可抓取目标",
        "arm_callback_failed": "机械臂执行报错",
        "arm_callback_timeout": "机械臂执行通信超时",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
    }
    return messages.get(reason_code, "闭环抓取失败")
