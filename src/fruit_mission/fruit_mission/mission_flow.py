"""Readable A/B/C mission flow functions."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from fruit_mission.logic import decide_b_tree_targets
from fruit_mission.logic import expected_ripe_type
from fruit_mission.logic import format_b_announcement
from fruit_mission.logic import format_c_announcement
from fruit_mission.logic import format_position_name
from fruit_mission.logic import spoken_model_type
from fruit_qr_planner.planner import build_b_plan
from fruit_qr_planner.planner import build_c_plan


def run_mission(ctx) -> None:
    """Run the selected field flow using the configured waypoint names."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    try:
        _announce_and_wait(
            ctx,
            "我们是重庆大学 bobocup 水果采摘队伍，现在开始采摘",
        )
        if flow not in {"full", "ac", "a", "b", "c"}:
            ctx.announce(f"未知任务流程{flow}，任务停止。")
            return
        ctx.recover_home()

        if flow in {"full", "ac", "a"}:
            run_a_area(ctx)
            if flow == "a":
                ctx.navigate_named(["to_B_QR_waypoint"])
                return

            b_plan = scan_b_qr_and_announce(ctx)
            c_distribution = scan_c_qr_and_build_table(ctx)
            run_c_area(ctx, c_distribution)
            if flow == "ac":
                return

            ctx.navigate_named(ctx.route.transition_names("c_to_b_first_tree"))
            run_b_area(ctx, b_plan)
            return

        if flow == "b":
            b_plan = scan_b_qr_and_announce(ctx)
            ctx.navigate_named(ctx.route.transition_names("b_to_first_tree"))
            run_b_area(ctx, b_plan)
            ctx.navigate_named(ctx.route.transition_names("b_to_c_qr_end"))
            return

        c_distribution = scan_c_qr_and_build_table(ctx)
        run_c_area(ctx, c_distribution)
    finally:
        ctx.set_perception_mode("OFF")
        ctx.announce("任务流程结束")


def run_a_area(ctx) -> bool:
    """Run A-zone stop-and-pick points with single camera."""
    ctx.report_current_area("A")
    observation_order = ctx.config.get("a_observation_order", ["left", "right"])
    for waypoint in ctx.route.a_goal_names():
        if not ctx.navigate_named([waypoint], observation_side=observation_order[0]):
            ctx.recover_home()
            continue
        for side in observation_order:
            result = ctx.execute_closed_loop_pick(side, "ripe")
            if not result.success and not result.safe_to_continue:
                ctx.node.get_logger().warning(f'A区{waypoint}当前动作跳过: {result.reason_code}')
                break
    ctx.set_perception_mode('OFF')
    return True


def scan_b_qr_and_announce(ctx) -> Dict[str, Any]:
    """Scan B QR and announce its ordered plan."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    route_names = (
        [ctx.route.b_qr_name]
        if flow == "b"
        else ctx.route.transition_names("a_to_b_qr")
    )
    ctx.report_current_area('B')
    ready = ctx.navigate_named(route_names) and ctx.send_qr_pose() and ctx.stable_stop()
    b_plan = ctx.wait_b_plan() if ready and ctx.set_perception_mode('QR') else None
    if b_plan is None:
        fallback_targets = ctx.config['qr_fallback']['b_targets']
        ctx.node.get_logger().warning(
            "B区二维码计划等待超时，使用配置的兜底顺序"
        )
        b_plan = build_b_plan("\n".join(str(target) for target in fallback_targets))
    ctx.save_b_plan(b_plan)
    _announce_qr(
        ctx,
        format_b_announcement(b_plan),
    )
    ctx.set_perception_mode('OFF')
    ctx.recover_home()
    return b_plan


def scan_c_qr_and_build_table(ctx) -> Dict[str, Any]:
    """Scan C QR, save the planned task list, and announce task order."""
    flow = str(ctx.config.get("mission_flow", "full")).strip().lower()
    route_names = (
        [ctx.route.c_qr_name]
        if flow == "c"
        else ctx.route.transition_names("b_to_c_qr")
    )
    ctx.report_current_area('C')
    ready = ctx.navigate_named(route_names) and ctx.send_qr_pose() and ctx.stable_stop()
    c_plan = ctx.wait_c_plan() if ready and ctx.set_perception_mode('QR') else None
    if c_plan is None:
        fallback_targets = ctx.config['qr_fallback']['c_targets']
        fallback_positions = ctx.config['qr_fallback']['c_positions']
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
    ctx.set_perception_mode('OFF')
    ctx.recover_home()
    return distribution


def _announce_and_wait(
    ctx,
    text: str,
    *,
    lead_time_sec: float = 0.0,
    length_scale: Optional[float] = None,
) -> bool:
    return bool(ctx.announce_and_wait(
        text, lead_time_sec=lead_time_sec, length_scale=length_scale))


def _announce_qr(ctx, text: str) -> bool:
    """Queue a compact QR summary with the configured faster speech rate."""
    return bool(ctx.announce(text, length_scale=_qr_length_scale(ctx)))


def _qr_length_scale(ctx) -> float:
    return float(ctx.config.get("tts", {}).get("qr_length_scale", 0.8))


def run_b_area(ctx, b_plan: Mapping[str, Any]) -> bool:
    """Execute B tree tasks in the QR order, using vehicle-side directions."""
    ctx.report_current_area("B")
    tasks = list(b_plan.get("tasks", []))
    tasks.sort(key=lambda item: int(item["sequence"]))
    b_names = ctx.route.b_goal_names()
    for task in tasks:
        position_id = int(task["position_id"])
        if not 1 <= position_id <= len(b_names):
            ctx.announce(f"B区第{position_id}棵树编号无效，跳过。")
            continue
        goal_name = b_names[position_id - 1]
        side = "right" if position_id % 2 == 1 else "left"
        if not ctx.navigate_named([goal_name], observation_side=side):
            ctx.announce(f"B区{goal_name}导航失败，跳过。")
            ctx.recover_home()
            continue
        target_name = str(task["target"])
        expected_type = expected_ripe_type(target_name)
        observation = ctx.observe_b_tree(side)
        if not observation.get("success"):
            reason_code = str(observation.get("reason_code"))
            reason = _b_observation_failure_text(reason_code)
            ctx.announce(f"B区{goal_name}，{reason}，跳过。")
        else:
            targets = list(observation.get("targets", []))
            selected_targets = _announce_b_tree_decision(
                ctx,
                goal_name,
                target_name,
                expected_type,
                targets,
            )
            requests = ctx.freeze_b_targets(selected_targets)
            for request in requests:
                pick_result = ctx.execute_b_tree_target(request, side)
                if not pick_result.get("success"):
                    reason_code = str(pick_result.get("reason_code"))
                    reason = _b_arm_failure_text(reason_code)
                    if not pick_result.get("safe_to_continue", True):
                        ctx.announce(f"B区{goal_name}，{reason}，跳过剩余目标。")
                        break
                    ctx.announce(f"B区{goal_name}，{reason}，继续下一个目标。")
            if not requests:
                ctx.recover_home()
    ctx.set_perception_mode('OFF')
    return True


def _announce_b_tree_decision(
    ctx,
    goal_name: str,
    target_name: str,
    expected_type: str,
    targets: list[Any],
) -> list[Any]:
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
        return []
    if decision.reason_code == "inconsistent_types":
        ctx.announce(f"B区{goal_name}，识别结果不一致，跳过。")
        return []
    if decision.reason_code == "type_mismatch":
        ctx.announce(
            f"B区{goal_name}，识别到{decision.observed_name}，"
            "和二维码不一致，跳过。"
        )
        return []

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
        return []
    return [targets[index] for index in decision.target_indices]


def _b_observation_failure_text(reason_code: str) -> str:
    messages = {
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "detect_mode_timeout": "深度相机识别模式切换失败",
        "camera_target_timeout": "识别超时",
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
    ctx.report_current_area('C')
    for task in ordered_c_tasks_from_distribution(c_distribution):
        position_id = int(task["position_id"])
        target_name = str(task["target_name"])
        expected_type = str(
            task.get("expected_ripe_type") or expected_ripe_type(target_name)
        )
        navigation = dict(ctx.navigate_c_position(position_id))
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
                "当前项导航规划失败，跳过。"
            )
            ctx.save_c_distribution(c_distribution)
            ctx.recover_home()
            continue

        if navigation["status"] == "navigation_failed":
            task["result"] = "navigation_failed"
            task["navigation_failure_reason"] = navigation.get("reason")
            task["navigation_failure_detail"] = navigation.get("detail")
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                "导航失败，跳过。"
            )
            ctx.save_c_distribution(c_distribution)
            ctx.recover_home()
            continue

        camera_side = str(navigation["camera_side"])
        task["last_observation"] = {
            "source": "camera",
            "expected_type": expected_type,
        }
        result = ctx.execute_closed_loop_pick(camera_side, expected_type)
        if result.success:
            task["result"] = result.reason_code
            observed = spoken_model_type(result.matched_type)
            ctx.announce(
                f"C区{format_position_name(position_id)}，应该采{target_name}。"
                f"识别到{observed}，采摘放置工序完成。"
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
        ctx.save_c_distribution(c_distribution)

    ctx.save_c_distribution(c_distribution)
    ctx.set_perception_mode('OFF')
    return True


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
        "detect_mode_timeout": "深度相机识别模式切换失败",
        "camera_target_timeout": "深度相机没有识别到可抓取目标",
        "arm_callback_failed": "机械臂执行报错",
        "arm_callback_timeout": "机械臂执行通信超时",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
    }
    return messages.get(reason_code, "闭环抓取失败")
