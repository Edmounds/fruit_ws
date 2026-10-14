"""Competition composition: edit routes and action order here, not in ROS callbacks."""

from fruit_mission.actions import navigate
from fruit_mission.actions import pick_a
from fruit_mission.actions import run_b
from fruit_mission.actions import run_c
from fruit_mission.actions import scan_b_qr
from fruit_mission.actions import scan_c_qr


def main(ctx):
    """Full competition: A -> B QR and work -> C QR and work."""
    run_a(ctx)

    arrived = navigate(ctx, ["to_B_QR_waypoint", "B_QR"], preset="shenzhi")
    b_plan = scan_b_qr(ctx, arrived=arrived)
    run_b(ctx, b_plan)

    arrived = navigate(ctx, [
        "to_C_waypoint_1", "C_QR",
    ], preset="shenzhi")
    c_plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, c_plan)
    if not ctx.navigate_c_waypoint("C_top_waypoint_1"):
        ctx.node.get_logger().warning("结束航点导航失败，直接播报采摘汇总")


def run_a(ctx):
    """Alternating zigzag observation and picking across Zone A waypoints."""
    ctx.report_current_area("A")
    side = "left"
    a_waypoints = ["A1", "A2", "A3", "A4"]

    for wp in a_waypoints:
        other_side = "right" if side == "left" else "left"
        if navigate(ctx, [wp], side=side):
            pick_a(ctx, side=side)
            pick_a(ctx, side=other_side)
            side = other_side
        else:
            ctx.node.get_logger().warning(
                f"A区点位 {wp} 导航失败，跳过并保持当前侧 {side}"
            )


def debug_a(ctx):
    """A-only entry, ending at the same existing A exit waypoint."""
    run_a(ctx)
    navigate(ctx, ["to_B_QR_waypoint"], preset="shenzhi")


def debug_b(ctx):
    """B-only entry: scan the real QR before the same B work function."""
    arrived = navigate(ctx, ["B_QR"])
    plan = scan_b_qr(ctx, arrived=arrived)
    run_b(ctx, plan)
    navigate(ctx, ["to_C_waypoint_1"], preset="shenzhi")


def debug_c(ctx):
    """C-only entry: scan the real QR before the same C work function."""
    arrived = navigate(ctx, ["C_QR"])
    plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, plan)


def run_ac(ctx):
    """Existing AC entry, composed from the same blocks without mode branches."""
    run_a(ctx)
    arrived = navigate(ctx, ["to_B_QR_waypoint", "B_QR"], preset="shenzhi")
    scan_b_qr(ctx, arrived=arrived)
    arrived = navigate(ctx, [
        "to_C_waypoint_1", "C_QR",
    ], preset="shenzhi")
    plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, plan)
