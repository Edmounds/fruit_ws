"""Repeated competition procedures; ROS dispatch and safety stay in MissionContext."""

from fruit_mission.camera_target_planner_client import store_preset_name
from fruit_mission.decisions import PickExecutionResult
from fruit_mission.decisions import decide_b_tree_targets
from fruit_mission.decisions import expected_ripe_type
from fruit_mission.decisions import format_b_announcement
from fruit_mission.decisions import format_c_announcement
from fruit_mission.decisions import format_position_name
from fruit_mission.decisions import spoken_model_type
from fruit_qr_planner.planner import build_b_plan, build_c_plan


def navigate(ctx, waypoints, *, side=None, preset=None):
    """Navigate with optional concurrent Look or preset; failed handoffs get one safe Home."""
    if (
        side is None
        and preset is None
        and not (ctx._arm_home_confirmed or ctx._arm_shenzhi_confirmed)
    ):
        ctx.recover_home()
    if ctx.navigate_named(waypoints, observation_side=side, arm_preset=preset):
        return True
    ctx.node.get_logger().warning(f"导航失败，跳过当前动作: {' -> '.join(waypoints)}")
    if ctx._current_area != "A":
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
    """qr_scan -> QR window -> plan/fallback -> announcement -> shenzhi."""
    ctx.report_current_area("B")
    ready = arrived and ctx.execute_arm_preset('qr_scan').completed
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
    if arrived and not decoded:  # Failed navigation already recovered once.
        ctx.recover_home()
    else:
        motion = ctx.execute_arm_preset('home')
        if not motion.completed:
            ctx.recover_home()
    return plan


def scan_c_qr(ctx, *, arrived):
    """qr_scan -> QR window -> plan/fallback -> task table -> shenzhi."""
    ctx.report_current_area("C")
    ready = arrived and ctx.execute_arm_preset('qr_scan').completed
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
    ctx.announce_and_wait(
        format_c_announcement(plan),
        lead_time_sec=float(ctx.config.get("c_qr_announce_lead_sec", 4.0)),
    )
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


def observe_and_pick(ctx, *, side, expected_type):
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
            ctx.node.get_logger().info(
                f"{area}区开环跳过抓取: {selection.reason_code} (type={selection.matched_type})"
            )
            return PickExecutionResult(
                False, selection.reason_code, selection.matched_type)
        preset_name = f"{side}_pick_openloop_a"
        ctx.node.get_logger().info(f"{area}区开环执行抓取预设: {preset_name}")
        motion = ctx.execute_arm_preset(preset_name)
        if not motion.completed:
            ctx.recover_home()
            return PickExecutionResult(False, motion.reason_code, selection.matched_type)
        ctx.record_pick(selection.matched_type)
        return PickExecutionResult(True, "store_completed", selection.matched_type)

    if selection.target is None:
        if area != "A":
            ctx.recover_home()
        return PickExecutionResult(
            False, selection.reason_code, selection.matched_type)
    try:
        request, layer = ctx.camera_target_request(selection.target.point)
    except ValueError as error:
        ctx.node.get_logger().warning(str(error))
        if area != "A":
            ctx.recover_home()
        return PickExecutionResult(
            False, "camera_target_invalid", selection.matched_type)
    store_preset = store_preset_name(side, area, layer)
    return _grasp_and_collect(
        ctx, request, store_preset=store_preset, matched_type=selection.matched_type)


def _grasp_and_collect(ctx, request, *, store_preset, matched_type=None):
    """One CLOSE/dwell -> side Store/OPEN; failure recovers once."""
    motion = ctx.move_to_base_target(request)
    if motion.completed:
        motion = ctx.execute_arm_preset(store_preset)
    if not motion.completed:
        ctx.recover_home()
        return PickExecutionResult(False, motion.reason_code, matched_type)
    ctx.record_pick(matched_type)
    return PickExecutionResult(
        True, "store_completed", matched_type,
    )


def pick_b_tree(ctx, *, target, side, tree_name=None, waypoint=None):
    """Observe (open gripper) once; freeze up to two matching ripe targets before either grasp."""
    label = tree_name or waypoint or "第一棵树"
    look = ctx.finish_observation(side)
    if not look.completed:
        reason = _b_observation_failure_text(look.reason_code)
        ctx.recover_home()
        ctx.announce(f"B区{label}，{reason}，跳过。")
        return False
    window_id = ctx.open_observation_window("B", "harvest", target)
    try:
        targets = ctx.wait_b_targets(window_id)
    finally:
        ctx.close_observation_window()
    if targets is None:
        ctx.recover_home()
        ctx.announce(f"B区{label}，识别超时，跳过。")
        return False
    selected = _select_b_targets(ctx, label, target, targets)
    requests = []
    for item in selected:
        try:
            req, layer = ctx.camera_target_request(item.point)
            store_preset = store_preset_name(side, "B", layer)
            requests.append((req, store_preset, item.model_type))
        except ValueError as error:
            ctx.node.get_logger().warning(f"Skipping invalid B target: {error}")
    if not requests:
        motion = ctx.execute_arm_preset("home")
        if not motion.completed:
            ctx.recover_home()
        return False
    success = True
    for request, store_preset, model_type in requests:
        result = _grasp_and_collect(
            ctx, request, store_preset=store_preset, matched_type=model_type)
        success = result.success and success
        if not result.success:
            reason = _b_arm_failure_text(result.reason_code)
            ctx.announce(f"B区{label}，{reason}，继续下一个目标。")
    motion = ctx.execute_arm_preset("home")
    if not motion.completed:
        ctx.recover_home()
    return success


def _select_b_targets(ctx, label, target, targets):
    if len(targets) > 2:
        ctx.node.get_logger().warning(
            f"B区{label}检测到 {len(targets)} 个目标 (>2)，将按评分只抓取前两个成熟目标。"
        )
    decision = decide_b_tree_targets(
        target, expected_ripe_type(target),
        ((item.model_type, item.pixel_x, item.pixel_y, item.image_width) for item in targets),
    )
    if decision.reason_code == "no_detection":
        ctx.announce(f"B区{label}，没有识别到果实，跳过。")
        return []
    if decision.reason_code == "inconsistent_types":
        ctx.announce(f"B区{label}，识别结果不一致，跳过。")
        return []
    if decision.reason_code == "type_mismatch":
        ctx.announce(f"B区{label}，识别到{decision.observed_name}，和二维码不一致，跳过。")
        return []
    if decision.target_index is None:
        ctx.announce(
            f"B区{label}，识别到{decision.observed_name}，和二维码一致，"
            f"没有成熟{target}，跳过。")
        return []
    ctx.announce(
        f"B区{label}，识别到{decision.observed_name}，和二维码一致，开始抓成熟{target}。")
    return [targets[index] for index in decision.target_indices]


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
        reason = "当前项导航规划失败" if planning_failed else "导航失败"
        ctx.announce(
            f"C区{format_position_name(int(task['position_id']))}，"
            f"应该采{task['target_name']}。{reason}，跳过。")
    return navigation


def pick_c(ctx, task, *, side):
    """One C task at its reached position; update its result and announce."""
    position = int(task["position_id"])
    target = str(task["target_name"])
    expected_type = str(task.get("expected_ripe_type") or expected_ripe_type(target))
    task["last_observation"] = {"source": "camera", "expected_type": expected_type}
    result = observe_and_pick(ctx, side=side, expected_type=expected_type)
    task["result"] = result.reason_code
    prefix = f"C区{format_position_name(position)}，应该采{target}。"
    observed = spoken_model_type(result.matched_type)
    if result.success:
        ctx.announce(prefix + f"识别到{observed}，采摘放置工序完成。")
    elif result.reason_code in {"no_detection", "target_unripe", "type_mismatch"}:
        ctx.announce(_c_skip_text(position, target, result.reason_code, result.matched_type))
    else:
        ctx.announce(prefix + f"识别到{observed}，{_pick_failure_text(result.reason_code)}。")
    return result


def _b_observation_failure_text(reason_code):
    return {
        "arm_callback_failed": "机械臂执行失败",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
        "arm_pose_unavailable": "机械臂执行失败",
        "camera_target_timeout": "识别超时",
    }.get(reason_code, "识别失败")


def _b_arm_failure_text(reason_code):
    return {
        "arm_callback_failed": "机械臂执行报错",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
    }.get(reason_code, "机械臂执行失败")


def _c_skip_text(position, target, reason_code, matched_type):
    prefix = f"C区{format_position_name(position)}，应该采{target}。"
    observed = spoken_model_type(matched_type) if matched_type else None
    if reason_code == "no_detection":
        return prefix + "没有识别到目标，跳过。"
    if reason_code == "target_unripe":
        return prefix + f"识别到{observed or '不成熟' + target}，跳过。"
    if reason_code == "type_mismatch":
        return prefix + f"识别到{observed or '其他目标'}，跳过。"
    if observed:
        return prefix + f"识别到{observed}，跳过。"
    return prefix + f"没有识别到成熟{target}，跳过。"


def _pick_failure_text(reason_code):
    return {
        "camera_target_timeout": "深度相机没有识别到可抓取目标",
        "arm_callback_failed": "机械臂执行报错",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
    }.get(reason_code, "闭环抓取失败")
