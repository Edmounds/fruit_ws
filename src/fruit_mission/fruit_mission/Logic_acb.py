"""ACB composition with an early B QR scan and a mid-mission harvest summary."""

from fruit_mission.actions import navigate, pick_a, run_b, run_c, scan_b_qr, scan_c_qr


def main(ctx):
    """Work A left, scan B, work A right and C, report counts, then work B."""
    ctx.config["open_loop"] = "A"
    ctx.report_current_area("A")
    for name in ("A_left1", "A_left2", "A_left3", "A_left4"):
        if navigate(ctx, [name], side="left"):
            pick_a(ctx, side="left")

    arrived = navigate(ctx, ["B_QR"], preset="shenzhi")
    b_plan = scan_b_qr(
        ctx, arrived=arrived, preset="qr_scan_a_to_b", return_home=False)

    ctx.config["open_loop"] = "none"
    ctx.report_current_area("A")
    for name in ("A_right4", "A_right3", "A_right2", "A_right1"):
        if navigate(ctx, [name], side="right"):
            pick_a(ctx, side="right")

    arrived = navigate(ctx, ["to_C_waypoint_1", "C_QR"], preset="shenzhi")
    c_plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, c_plan)

    if ctx.navigate_c_waypoint("C_top_waypoint_2"):
        ctx.announce_summary(completed=False)
    else:
        ctx.node.get_logger().warning("ACB 汇总航点导航失败，跳过中途汇总")
        ctx.recover_home()

    if not ctx.navigate_c_waypoint("C_entrance_waypoint_2"):
        ctx.node.get_logger().warning("ACB C区出口导航失败，继续既定B区转场")
        ctx.recover_home()
    if navigate(ctx, ["to_B_lower_waypoint_1"], preset="shenzhi"):
        # B's first Look needs Home, not shenzhi. Failure already recovered once.
        ctx.recover_home()
    run_b(ctx, b_plan, first_side="left")
