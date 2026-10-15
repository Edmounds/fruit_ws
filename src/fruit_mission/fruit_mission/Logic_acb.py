"""ACB composition with an early B QR scan and a mid-mission harvest summary."""

from fruit_mission.actions import navigate, pick_a, run_b, run_c, scan_b_qr, scan_c_qr


def main(ctx):
    """Work A left, scan B, work A right and C, report counts, then work B."""
    ctx.config["open_loop"] = "A"
    ctx.report_current_area("A")
    for name in ("A_left1", "A_left2", "A_left3", "A_left4"):
        if navigate(ctx, [name], side="left"):
            pick_a(ctx, side="left")

    straight = ctx.execute_arm_preset("shenzhi")
    if not straight.completed:
        ctx.node.get_logger().warning("ACB 伸直失败，跳过 B 扫码转场")
        ctx.recover_home()
    arrived = straight.completed and navigate(ctx, ["B_QR"])
    b_plan = scan_b_qr(
        ctx, arrived=arrived, preset="qr_scan_a_to_b", return_home=False)

    ctx.config["open_loop"] = "none"
    ctx.report_current_area("A")
    # This is the only safe exit from qr_scan_a_to_b, even without a QR result.
    look = ctx.execute_arm_preset("right_look_a")
    if not look.completed:
        ctx.node.get_logger().warning(
            f"ACB 扫码后右 Look 失败: {look.reason_code}，回 Home 后继续"
        )
        ctx.recover_home()
    for name in ("A_right4", "A_right3", "A_right2", "A_right1"):
        if navigate(ctx, [name], side="right"):
            pick_a(ctx, side="right")

    start = ctx._lookup_map_pose() or ctx._previous_target_pose()
    arrived = False
    if start is not None:
        names = ["to_C_waypoint_1", "C_QR"]
        try:
            yaws = ctx.route.navigation_yaws_from_waypoints(*start, names)
        except ValueError as error:
            ctx.node.get_logger().warning(f"ACB 转场朝向规划失败，跳过 C 区: {error}")
        else:
            # Use the minimum-turn approach yaw before rotating towards C.
            if navigate(ctx, names[:1], preset="shenzhi", yaws=yaws[:1]):
                # Rotation at the reached connector and travel to C_QR each get 10 s.
                arrived = navigate(ctx, names)
    else:
        ctx.node.get_logger().warning("ACB 转场起点不可用，跳过 C 区")
    c_plan = scan_c_qr(ctx, arrived=arrived)
    if arrived:
        run_c(ctx, c_plan)

        if ctx.navigate_c_waypoint("C_top_waypoint_2"):
            ctx.announce_summary(completed=False)
        else:
            ctx.node.get_logger().warning("ACB 汇总航点导航失败，跳过中途汇总")

        if not ctx.navigate_c_waypoint("C_entrance_waypoint_2"):
            ctx.node.get_logger().warning("ACB C区出口导航失败，继续既定B区转场")
    else:
        ctx.node.get_logger().warning("ACB 未到达 C_QR，跳过 C 区作业、汇总和出口导航")
    if navigate(ctx, ["to_B_lower_waypoint_1"], preset="shenzhi"):
        # B's first Look needs Home, not shenzhi.
        ctx.recover_home()
    run_b(ctx, b_plan, first_side="left")
