"""Repeated competition procedures; ROS dispatch and safety stay in MissionContext."""

from fruit_mission.camera_target_planner_client import store_preset_name
from fruit_mission.decisions import PickExecutionResult
from fruit_mission.decisions import expected_ripe_type
from fruit_mission.decisions import format_b_announcement
from fruit_mission.decisions import format_c_announcement
from fruit_mission.decisions import format_position_name
from fruit_mission.decisions import spoken_model_type
from fruit_mission.waypoint_route import c_top_transit_end
from fruit_qr_planner.planner import build_b_plan, build_c_plan


def navigate(ctx, waypoints, *, side=None, preset=None):
    """Navigate with optional concurrent Look or preset; failed handoffs get one safe Home."""
    if side is not None and ctx._current_area == "B" and not ctx._arm_home_confirmed:
        ctx.node.get_logger().warning('B区转场 Home 未确认，跳过当前导航和 Look')
        return False
    if (
        side is None
        and preset is None
        and not (ctx._arm_home_confirmed or ctx._arm_shenzhi_confirmed)
    ):
        if not ctx.recover_home():
            ctx.node.get_logger().warning('安全 Home 未确认，跳过当前导航')
            return False
    if ctx.navigate_named(waypoints, observation_side=side, arm_preset=preset):
        return True
    ctx.node.get_logger().warning(f"导航失败，跳过当前动作: {' -> '.join(waypoints)}")
    ctx.recover_home()
    return False


def enter_shenzhi(ctx, *, completed):
    """Normal work ends in protection; failed work keeps its Home handoff."""
    if ctx.is_open_loop and ctx._current_area in ("A", "C"):
        return True
    if not completed:
        # All local failure paths recover Home before reaching this boundary.
        # Home is the measured safe handoff for the next fixed route point.
        return False
    ctx.require_shenzhi()
    motion = ctx.execute_arm_preset('shenzhi')
    if not motion.completed:
        ctx.node.get_logger().warning(f'保护姿态失败，禁止后续转场: {motion.reason_code}')
        ctx.recover_home()
    return motion.completed


def scan_b_qr(ctx, *, arrived):
    """qr_scan_b -> QR window -> plan/fallback -> announcement -> Home."""
    ctx.report_current_area("B")
    ready = arrived and ctx.execute_arm_preset('qr_scan_b').completed
    plan = None
    if ready:
        window_id = ctx.open_observation_window("B", "qr")
        try:
            plan = ctx.wait_b_plan(window_id)
        finally:
            ctx.close_observation_window()
    decoded = plan is not None
    if not decoded:
        ctx.node.get_logger().warning("B区二维码获取失败，使用配置的兜底顺序")
        plan = build_b_plan("\n".join(ctx.config["qr_fallback"]["b_targets"]))
    ctx.save_b_plan(plan)
    ctx.announce(format_b_announcement(plan))
    if not arrived:  # Failed navigation already recovered once.
        return plan
    if not decoded:
        ctx.recover_home()
    else:
        motion = ctx.execute_arm_preset('home')
        if not motion.completed:
            ctx.recover_home()
    return plan


def scan_c_qr(ctx, *, arrived):
    """qr_scan_c -> QR window -> plan/fallback -> task table -> shenzhi."""
    ctx.report_current_area("C")
    ready = arrived and ctx.execute_arm_preset('qr_scan_c').completed
    plan = None
    if ready:
        window_id = ctx.open_observation_window("C", "qr")
        try:
            plan = ctx.wait_c_plan(window_id)
        finally:
            ctx.close_observation_window()
    decoded = plan is not None
    if not decoded:
        ctx.node.get_logger().warning("C区二维码获取失败，使用配置的兜底顺序")
        fallback = ctx.config["qr_fallback"]
        lines = list(fallback["c_targets"])
        lines.append(",".join(str(position) for position in fallback["c_positions"]))
        plan = build_c_plan("\n".join(lines))
    distribution = ctx.route.build_c_distribution(plan)
    ctx.save_c_distribution(distribution)
    ctx.announce(format_c_announcement(plan))
    if arrived and not decoded:
        ctx.recover_home()
    enter_shenzhi(ctx, completed=decoded)
    return distribution


def pick_a(ctx, *, side):
    """One side at the current A point, with no navigation or grasp retries."""
    result = observe_and_pick(ctx, side=side, expected_type="ripe")
    if not result.success:
        ctx.node.get_logger().warning(f"A区当前动作跳过: {result.reason_code}")
    return result


def observe_and_pick(ctx, *, side, expected_type, label=None):
    """Look (open gripper) -> harvest window -> fresh target -> CLOSE/Store/Home."""
    look = ctx.finish_observation(side)
    if not look.completed:
        ctx.recover_home()
        return PickExecutionResult(False, look.reason_code)
    area = ctx._current_area or "A"
    window_id = ctx.open_observation_window(area, "harvest", expected_type)
    try:
        if ctx.is_open_loop and area in ("A", "C"):
            selection = ctx.wait_camera_decision(window_id, expected_type)
        else:
            selection = ctx.wait_camera_target(window_id, expected_type)
    finally:
        ctx.close_observation_window()

    if ctx.is_open_loop and area in ("A", "C"):
        if selection.reason_code != "matched":
            ctx.stop_dataset_capture()
            ctx.node.get_logger().info(
                f"{area}区开环跳过抓取: {selection.reason_code} (type={selection.matched_type})"
            )
            return PickExecutionResult(
                False, selection.reason_code, selection.matched_type)
        if label:
            observed = spoken_model_type(selection.matched_type)
            ctx.announce(f"{label}识别到{observed}，开始抓取。")
        preset_name = f"{side}_pick_openloop_{area.lower()}"
        ctx.node.get_logger().info(f"{area}区开环执行抓取预设: {preset_name}")
        ctx.stop_dataset_capture()
        motion = ctx.execute_arm_preset(preset_name)
        if not motion.completed:
            ctx.recover_home()
            return PickExecutionResult(False, motion.reason_code, selection.matched_type)
        ctx.record_pick(selection.matched_type)
        ctx.announce("采摘完成。")
        return PickExecutionResult(True, "store_completed", selection.matched_type)

    if selection.target is None:
        ctx.stop_dataset_capture()
        if area != "A":
            ctx.recover_home()
        return PickExecutionResult(
            False, selection.reason_code, selection.matched_type)
    try:
        request, layer = ctx.camera_target_request(selection.target.point)
    except ValueError as error:
        ctx.stop_dataset_capture()
        ctx.node.get_logger().warning(str(error))
        if area != "A":
            ctx.recover_home()
        return PickExecutionResult(
            False, "camera_target_invalid", selection.matched_type)
    if label:
        observed = spoken_model_type(selection.matched_type)
        ctx.announce(f"{label}识别到{observed}，开始抓取。")
    store_preset = store_preset_name(side, area, layer)
    return _grasp_and_collect(
        ctx, request, store_preset=store_preset, matched_type=selection.matched_type)


def _grasp_and_collect(ctx, request, *, store_preset, matched_type=None):
    """One CLOSE/dwell -> side Store/OPEN; failure recovers once."""
    ctx.stop_dataset_capture()
    motion = ctx.move_to_base_target(request)
    if motion.completed:
        motion = ctx.execute_arm_preset(store_preset)
    if not motion.completed:
        ctx.recover_home()
        return PickExecutionResult(False, motion.reason_code, matched_type)
    ctx.record_pick(matched_type)
    ctx.announce("采摘完成。")
    return PickExecutionResult(
        True, "store_completed", matched_type,
    )


def pick_b_tree(ctx, *, target, side, tree_name=None, waypoint=None):
    """Observe (open gripper) once; freeze up to two matching ripe targets before either grasp."""
    label = tree_name or (f"{waypoint}号" if waypoint else "一号")
    look = ctx.finish_observation(side)
    if not look.completed:
        ctx.recover_home()
        ctx.announce(f"{label}识别失败，跳过。")
        return False
    window_id = ctx.open_observation_window("B", "harvest", target)
    try:
        targets = ctx.wait_b_targets(window_id)
    finally:
        ctx.close_observation_window()
    if targets is None:
        ctx.stop_dataset_capture()
        ctx.recover_home()
        ctx.announce(f"{label}没有识别到果实，跳过。")
        return False
    selected = _select_b_targets(ctx, label, target, targets)
    requests = []
    for item in selected:
        try:
            req, layer = ctx.camera_target_request(item.point)
            store_preset = store_preset_name(side, "B", layer)
            requests.append((req, store_preset, item.model_type))
            if len(requests) == 2:
                break
        except ValueError as error:
            ctx.node.get_logger().warning(f"Skipping invalid B target: {error}")
    if not requests:
        ctx.stop_dataset_capture()
        motion = ctx.execute_arm_preset("home")
        if not motion.completed:
            ctx.recover_home()
        return False
    expected_type = expected_ripe_type(target)
    observed = spoken_model_type(expected_type)
    ctx.announce(f"{label}识别到{observed}，开始抓取。")
    success = True
    for request, store_preset, model_type in requests:
        result = _grasp_and_collect(
            ctx, request, store_preset=store_preset, matched_type=model_type)
        success = result.success and success
        if not result.success:
            if not ctx._arm_home_confirmed:
                ctx.announce(f"{label}机械臂执行失败，跳过。")
                return False
            ctx.announce(f"{label}继续下一个目标。")
    if result.success:  # The last failed grasp already recovered Home once.
        motion = ctx.execute_arm_preset("home")
        if not motion.completed:
            return ctx.recover_home() and success
    return success


def _select_b_targets(ctx, label, target, targets):
    if not targets:
        ctx.announce(f"{label}没有识别到果实，跳过。")
        return []
    expected_type = expected_ripe_type(target)
    selected = sorted(
        (item for item in targets if item.model_type == expected_type),
        key=lambda item: item.confidence, reverse=True)
    if not selected:
        first_type = targets[0].model_type
        observed = spoken_model_type(first_type)
        ctx.announce(f"{label}识别到{observed}，跳过。")
    return selected


def navigate_c_route(ctx, names, yaws, *, side=None):
    """Run shenzhi through the top connector, then Look along the remaining route."""
    transit_end = c_top_transit_end(names)
    if transit_end:
        ctx.start_arm_preset('shenzhi')
        motion = None
        if ctx._pending_arm_preset[1] is None:
            # No request was submitted. Preserve the existing safe pose handoff.
            motion = ctx.finish_arm_preset()
        if not ctx._send_waypoint_names(names[:transit_end], yaws[:transit_end]):
            return False
        ctx._current_waypoint_name = names[transit_end - 1]
        ctx._current_route_yaw = yaws[transit_end - 1]
        if motion is None:
            motion = ctx.finish_arm_preset()
        if not motion.completed:
            ctx.node.get_logger().warning(
                f'C区 top 路段伸直失败，继续原导航路线: {motion.reason_code}')
    if transit_end == len(names):
        return True
    if side is not None:
        ctx.start_observation(side)
    return ctx._send_waypoint_names(names[transit_end:], yaws[transit_end:])


def navigate_c(ctx, task):
    """Reuse C geometry/side planning and record one navigation result."""
    navigation = ctx.navigate_c_position(int(task["position_id"]))
    task["navigation_status"] = navigation["status"]
    for key in (
        "navigation_start", "navigation_start_source", "goal_name", "position_side",
        "graph_path", "movement_points", "navigation_yaws", "route_distance",
        "camera_side", "preferred_side", "arrival_yaw", "arrival_yaw_source", "route_rejoin",
    ):
        if key in navigation:
            task[key] = navigation[key]
    if navigation["status"] != "succeeded":
        planning_failed = navigation["status"] == "planning_failed"
        task["result"] = "navigation_planning_failed" if planning_failed else "navigation_failed"
        task["navigation_failure_reason"] = navigation.get("reason")
        task["navigation_failure_detail"] = navigation.get("detail")
        pos_label = format_position_name(int(task["position_id"]))
        ctx.announce(f"{pos_label}导航失败，跳过。")
    return navigation


def pick_c(ctx, task, *, side):
    """One C task at its reached position; update its result and announce."""
    position = int(task["position_id"])
    target = str(task["target_name"])
    expected_type = str(task.get("expected_ripe_type") or expected_ripe_type(target))
    task["last_observation"] = {"source": "camera", "expected_type": expected_type}
    pos_label = format_position_name(position)
    result = observe_and_pick(
        ctx, side=side, expected_type=expected_type, label=pos_label)
    task["result"] = result.reason_code
    if result.success:
        return result
    if result.reason_code in {"no_detection", "camera_target_timeout"}:
        ctx.announce(f"{pos_label}没有识别到目标，跳过。")
    elif result.matched_type:
        observed = spoken_model_type(result.matched_type)
        ctx.announce(f"{pos_label}识别到{observed}，跳过。")
    else:
        ctx.announce(f"{pos_label}识别失败，跳过。")
    return result
