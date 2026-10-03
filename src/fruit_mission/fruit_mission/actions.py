"""Repeated competition procedures; ROS dispatch and safety stay in MissionContext."""

from fruit_mission.decisions import PickExecutionResult
from fruit_mission.decisions import decide_b_tree_targets
from fruit_mission.decisions import expected_ripe_type
from fruit_mission.decisions import format_b_announcement
from fruit_mission.decisions import format_c_announcement
from fruit_mission.decisions import format_position_name
from fruit_mission.decisions import spoken_model_type
from fruit_qr_planner.planner import build_b_plan, build_c_plan


def navigate(ctx, waypoints, *, side=None):
    """Navigate with optional concurrent Look; failed handoffs get one safe Home."""
    if ctx.navigate_named(waypoints, observation_side=side):
        return True
    ctx.node.get_logger().warning(f"导航失败，跳过当前动作: {' -> '.join(waypoints)}")
    ctx.recover_home()
    return False


def scan_b_qr(ctx, *, arrived):
    """QR pose -> standstill -> QR plan/fallback -> announcement -> Home."""
    ctx.report_current_area("B")
    ready = arrived and ctx.send_qr_pose() and ctx.stable_stop()
    plan = ctx.wait_b_plan() if ready and ctx.set_perception_mode("QR") else None
    if plan is None:
        ctx.node.get_logger().warning("B区二维码获取失败，使用配置的兜底顺序")
        plan = build_b_plan("\n".join(ctx.config["qr_fallback"]["b_targets"]))
    ctx.save_b_plan(plan)
    ctx.announce(format_b_announcement(plan), length_scale=_qr_length_scale(ctx))
    ctx.set_perception_mode("OFF")
    if arrived:  # Failed navigation already performed its one recovery.
        ctx.recover_home()
    return plan


def scan_c_qr(ctx, *, arrived):
    """QR pose -> standstill -> QR plan/fallback -> task table -> Home."""
    ctx.report_current_area("C")
    ready = arrived and ctx.send_qr_pose() and ctx.stable_stop()
    plan = ctx.wait_c_plan() if ready and ctx.set_perception_mode("QR") else None
    if plan is None:
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
        length_scale=_qr_length_scale(ctx),
    )
    ctx.set_perception_mode("OFF")
    if arrived:
        ctx.recover_home()
    return distribution


def pick_a(ctx, *, side):
    """One side at the current A point, with no navigation or grasp retries."""
    result = _observe_and_pick(ctx, side=side, expected_type="ripe")
    if not result.success and not result.safe_to_continue:
        ctx.node.get_logger().warning(f"A区当前动作跳过: {result.reason_code}")
    return result


def _observe_and_pick(ctx, *, side, expected_type):
    """Look -> measured standstill -> fresh target -> CLOSE/Harvest/Home."""
    ctx.set_expected_fruit_type(expected_type)
    look = ctx.finish_observation(side)
    if not look.completed:
        return PickExecutionResult(False, look.reason_code, safe_to_continue=ctx.recover_home())
    if not ctx.stable_stop():
        return PickExecutionResult(
            False, "stationary_timeout", safe_to_continue=ctx.recover_home())
    fresh_after = ctx.node.get_clock().now().nanoseconds
    if not ctx.set_perception_mode("DETECT"):
        return PickExecutionResult(
            False, "detect_mode_timeout", safe_to_continue=ctx.recover_home())
    selection = ctx.wait_camera_target(fresh_after, expected_type)
    if selection.target is None:
        return PickExecutionResult(
            False, selection.reason_code, selection.matched_type, ctx.recover_home())
    try:
        request = ctx.camera_target_request(selection.target.point)
    except ValueError as error:
        ctx.node.get_logger().warning(str(error))
        return PickExecutionResult(
            False, "camera_target_invalid", selection.matched_type, ctx.recover_home())
    result = _grasp_and_collect(ctx, request, side=side)
    return PickExecutionResult(
        result.success, result.reason_code, selection.matched_type, result.safe_to_continue)


def _grasp_and_collect(ctx, request, *, side):
    """One CLOSE/dwell -> side Harvest/OPEN -> Home; failure recovers once."""
    ctx.set_perception_mode("OFF")
    grasp = ctx.move_to_base_target(request)
    if not grasp.completed:
        return PickExecutionResult(False, grasp.reason_code, safe_to_continue=ctx.recover_home())
    harvest = ctx.execute_arm_preset(f"harvest_{side}")
    if not harvest.completed:
        return PickExecutionResult(
            False, harvest.reason_code, safe_to_continue=ctx.recover_home())
    home = ctx.execute_arm_preset("home")
    return PickExecutionResult(
        home.completed, "harvest_completed" if home.completed else home.reason_code,
        safe_to_continue=home.completed,
    )


def pick_b_tree(ctx, *, waypoint, target, side):
    """Observe once; freeze up to two matching ripe targets before either grasp."""
    look = ctx.finish_observation(side)
    if not look.completed:
        reason = _b_observation_failure_text(look.reason_code)
        ctx.recover_home()
        ctx.announce(f"B区{waypoint}，{reason}，跳过。")
        return
    if not ctx.stable_stop():
        ctx.recover_home()
        ctx.announce(f"B区{waypoint}，识别失败，跳过。")
        return
    if not ctx.set_perception_mode("DETECT"):
        ctx.recover_home()
        ctx.announce(f"B区{waypoint}，深度相机识别模式切换失败，跳过。")
        return
    targets = ctx.wait_b_targets(ctx.node.get_clock().now().nanoseconds)
    if targets is None:
        ctx.recover_home()
        ctx.announce(f"B区{waypoint}，识别超时，跳过。")
        return
    selected = _select_b_targets(ctx, waypoint, target, targets)
    requests = []
    for item in selected:
        try:
            requests.append(ctx.camera_target_request(item.point))
        except ValueError as error:
            ctx.node.get_logger().warning(f"Skipping invalid B target: {error}")
    if not requests:
        ctx.recover_home()
        return
    for request in requests:
        result = _grasp_and_collect(ctx, request, side=side)
        if not result.success:
            reason = _b_arm_failure_text(result.reason_code)
            if not result.safe_to_continue:
                ctx.announce(f"B区{waypoint}，{reason}，跳过剩余目标。")
                break
            ctx.announce(f"B区{waypoint}，{reason}，继续下一个目标。")


def _select_b_targets(ctx, waypoint, target, targets):
    decision = decide_b_tree_targets(
        target, expected_ripe_type(target),
        ((item.model_type, item.pixel_x, item.pixel_y, item.image_width) for item in targets),
    )
    if decision.reason_code == "no_detection":
        ctx.announce(f"B区{waypoint}，没有识别到果实，跳过。")
        return []
    if decision.reason_code == "inconsistent_types":
        ctx.announce(f"B区{waypoint}，识别结果不一致，跳过。")
        return []
    if decision.reason_code == "type_mismatch":
        ctx.announce(f"B区{waypoint}，识别到{decision.observed_name}，和二维码不一致，跳过。")
        return []
    if decision.target_index is None:
        ctx.announce(
            f"B区{waypoint}，识别到{decision.observed_name}，和二维码一致，"
            f"没有成熟{target}，跳过。")
        return []
    ctx.announce(
        f"B区{waypoint}，识别到{decision.observed_name}，和二维码一致，开始抓成熟{target}。")
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
    result = _observe_and_pick(ctx, side=side, expected_type=expected_type)
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


def _qr_length_scale(ctx):
    return float(ctx.config.get("tts", {}).get("qr_length_scale", 0.8))


def _b_observation_failure_text(reason_code):
    return {
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "detect_mode_timeout": "深度相机识别模式切换失败",
        "camera_target_timeout": "识别超时",
    }.get(reason_code, "识别失败")


def _b_arm_failure_text(reason_code):
    return {
        "arm_callback_failed": "机械臂执行报错",
        "arm_callback_timeout": "机械臂执行通信超时",
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
        "side_pose_timeout": "机械臂转向采摘位置超时",
        "detect_mode_timeout": "深度相机识别模式切换失败",
        "camera_target_timeout": "深度相机没有识别到可抓取目标",
        "arm_callback_failed": "机械臂执行报错",
        "arm_callback_timeout": "机械臂执行通信超时",
        "arm_endpoints_not_ready": "机械臂通信端点未就绪",
    }.get(reason_code, "闭环抓取失败")
