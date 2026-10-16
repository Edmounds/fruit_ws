"""ACBD composition: ACB flow through B8, then transition to D area (D1..D7)."""

from fruit_mission.actions import (
    navigate,
    pick_a,
    pick_d_tree,
    run_b,
    run_c,
    scan_b_qr,
    scan_c_qr,
)


def _d_transition_straight(ctx):
    if not (ctx._arm_home_confirmed or ctx._arm_shenzhi_confirmed):
        ctx.node.get_logger().warning("ACBD 安全姿态未确认，跳过伸直")
        return
    if ctx._arm_shenzhi_confirmed:
        return
    straight = ctx.execute_arm_preset("shenzhi")
    if not straight.completed:
        ctx.node.get_logger().warning("ACBD 伸直失败，恢复当前侧安全 Home 后继续路线")
        ctx.recover_home()


def main(ctx):
    """Work A left, scan B, work A right and C, report counts, work B, then work D."""
    ctx.config["open_loop"] = "none"
    ctx.report_current_area("A")
    for name in ("A_left1", "A_left2", "A_left3", "A_left4"):
        if navigate(ctx, [name], side="left"):
            pick_a(ctx, side="left")

    straight = ctx.execute_arm_preset("shenzhi")
    if not straight.completed:
        ctx.node.get_logger().warning("ACBD 伸直失败，跳过 B 扫码转场")
        ctx.recover_home()
    arrived = straight.completed and navigate(ctx, ["B_QR"])
    b_plan = scan_b_qr(
        ctx, arrived=arrived, preset="qr_scan_a_to_b", return_home=False)

    ctx.report_current_area("A")
    # This is the only safe exit from qr_scan_a_to_b, even without a QR result.
    look = ctx.execute_arm_preset("right_look_a")
    if not look.completed:
        ctx.node.get_logger().warning(
            f"ACBD 扫码后右 Look 失败: {look.reason_code}，回 Home 后继续"
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
            ctx.node.get_logger().warning(f"ACBD 转场朝向规划失败，跳过 C 区: {error}")
        else:
            # Use the minimum-turn approach yaw before rotating towards C.
            if navigate(ctx, names[:1], preset="shenzhi", yaws=yaws[:1]):
                # Rotation at the reached connector and travel to C_QR each get 10 s.
                arrived = navigate(ctx, names)
    else:
        ctx.node.get_logger().warning("ACBD 转场起点不可用，跳过 C 区")
    c_plan = scan_c_qr(ctx, arrived=arrived)
    if arrived:
        run_c(ctx, c_plan)

        if ctx.navigate_c_waypoint("C_top_waypoint_1"):
            ctx.announce_summary(completed=False)
        else:
            ctx.node.get_logger().warning("ACBD 汇总航点导航失败，跳过中途汇总")

        if not ctx.navigate_c_waypoint("C_entrance_waypoint_2"):
            ctx.node.get_logger().warning("ACBD C区出口导航失败，继续既定B区转场")
    else:
        ctx.node.get_logger().warning("ACBD 未到达 C_QR，跳过 C 区作业、汇总和出口导航")
    navigate(ctx, ["to_B_lower_waypoint_1"], preset="shenzhi")
    run_b(ctx, b_plan, first_side="left", concurrent_first_look=False)

    # 切换至 D 区
    ctx.report_current_area("D")

    # B8 -> D1: 右侧 Home -> 伸直 -> 经入口航点导航至 D1（右侧）
    _d_transition_straight(ctx)
    for wp in (
        "D_entrance_waypoint_1",
        "D_entrance_waypoint_2",
        "D_entrance_waypoint_3",
        "D_entrance_waypoint_4",
        "D_entrance_waypoint_5",
    ):
        if not navigate(ctx, [wp]):
            ctx.node.get_logger().warning(f"ACBD D区入口航点 {wp} 导航失败，继续下一个航点")

    # D1 作业点（右侧）
    if navigate(ctx, ["D1"]):
        pick_d_tree(ctx, side="right", waypoint="D1")
    else:
        ctx.node.get_logger().warning("ACBD D1 导航失败，跳过")

    # D1 -> D2: 右侧 Home -> 伸直 -> 经连接点导航至 D2（右侧）
    _d_transition_straight(ctx)
    if not navigate(ctx, ["D1_to_D2_waypoint_1"]):
        ctx.node.get_logger().warning("ACBD D1到D2连接点导航失败，继续前往D2")
    if navigate(ctx, ["D2"]):
        pick_d_tree(ctx, side="right", waypoint="D2")
    else:
        ctx.node.get_logger().warning("ACBD D2 导航失败，跳过")

    # D2 -> D3: 右侧 Home -> 伸直 -> 导航至 D3（右侧）
    _d_transition_straight(ctx)
    if navigate(ctx, ["D3"]):
        pick_d_tree(ctx, side="right", waypoint="D3")
    else:
        ctx.node.get_logger().warning("ACBD D3 导航失败，跳过")

    # D3 -> D4: 右侧 Home -> 伸直 -> 导航至 D4（右侧）
    _d_transition_straight(ctx)
    if navigate(ctx, ["D4"]):
        pick_d_tree(ctx, side="right", waypoint="D4")
    else:
        ctx.node.get_logger().warning("ACBD D4 导航失败，跳过")

    # D4 -> D5: 右侧 Home -> 伸直 -> 经两个 down 航点导航至 D5（左侧）
    _d_transition_straight(ctx)
    for wp in ("D_down_waypoint_1", "D_down_waypoint_2"):
        if not navigate(ctx, [wp]):
            ctx.node.get_logger().warning(f"ACBD D区转场航点 {wp} 导航失败，继续下一个航点")
    if navigate(ctx, ["D5"]):
        pick_d_tree(ctx, side="left", waypoint="D5")
    else:
        ctx.node.get_logger().warning("ACBD D5 导航失败，跳过")

    # D5 -> D6: 左侧 Home -> 伸直 -> 导航至 D6（左侧）
    _d_transition_straight(ctx)
    if navigate(ctx, ["D6"]):
        pick_d_tree(ctx, side="left", waypoint="D6")
    else:
        ctx.node.get_logger().warning("ACBD D6 导航失败，跳过")

    # D6 -> D7: 左侧 Home -> 伸直 -> 导航至 D7（左侧）
    _d_transition_straight(ctx)
    if navigate(ctx, ["D7"]):
        pick_d_tree(ctx, side="left", waypoint="D7")
    else:
        ctx.node.get_logger().warning("ACBD D7 导航失败，跳过")

    # D7 结束：停在 left_to_home_d，不再伸直，安全结束
