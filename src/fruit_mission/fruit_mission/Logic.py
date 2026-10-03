"""Competition composition: edit routes and action order here, not in ROS callbacks."""

from fruit_mission.actions import navigate
from fruit_mission.actions import navigate_c
from fruit_mission.actions import pick_a
from fruit_mission.actions import pick_b_tree
from fruit_mission.actions import pick_c
from fruit_mission.actions import scan_b_qr
from fruit_mission.actions import scan_c_qr


def main(ctx):
    """Full competition: A -> B QR -> C QR -> C -> B."""
    run_a(ctx)

    arrived = navigate(ctx, ["to_B_QR_waypoint", "B_QR"])
    b_plan = scan_b_qr(ctx, arrived=arrived)

    arrived = navigate(ctx, [
        "to_C_waypoint_1", "to_C_waypoint_2", "to_C_waypoint_3", "C_QR",
    ])
    c_plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, c_plan)

    navigate(ctx, [
        "C_entrance_waypoint_2", "to_B_waypoint_1", "to_B_waypoint_2",
        "toB_waypoint_3", "B1",
    ])
    run_b(ctx, b_plan)


def run_a(ctx):
    """Fixed A route; navigation and single-side picking are separate blocks."""
    ctx.report_current_area("A")

    if navigate(ctx, ["A1"], side="left"):
        if pick_a(ctx, side="left").safe_to_continue:
            pick_a(ctx, side="right")

    if navigate(ctx, ["A2"], side="left"):
        if pick_a(ctx, side="left").safe_to_continue:
            pick_a(ctx, side="right")

    if navigate(ctx, ["A3"], side="left"):
        if pick_a(ctx, side="left").safe_to_continue:
            pick_a(ctx, side="right")

    if navigate(ctx, ["A4"], side="left"):
        if pick_a(ctx, side="left").safe_to_continue:
            pick_a(ctx, side="right")

    ctx.set_perception_mode("OFF")


def run_b(ctx, plan):
    """QR tree order; observe/freeze/pick up to two fruit inside one action."""
    ctx.report_current_area("B")
    waypoints = ctx.route.b_goal_names()
    for task in sorted(plan["tasks"], key=lambda item: int(item["sequence"])):
        position = int(task["position_id"])
        if not 1 <= position <= len(waypoints):
            ctx.announce(f"B区第{position}棵树编号无效，跳过。")
            continue
        side = "right" if position % 2 == 1 else "left"
        waypoint = waypoints[position - 1]
        if navigate(ctx, [waypoint], side=side):
            pick_b_tree(ctx, waypoint=waypoint, target=str(task["target"]), side=side)
        else:
            ctx.announce(f"B区{waypoint}导航失败，跳过。")
    ctx.set_perception_mode("OFF")


def run_c(ctx, plan):
    """QR task order; use the route planner's side without direction rechecks."""
    ctx.report_current_area("C")
    for task in sorted(plan["tasks"], key=lambda item: int(item["sequence"])):
        navigation = navigate_c(ctx, task)
        if navigation["status"] != "succeeded":
            ctx.save_c_distribution(plan)
            ctx.recover_home()
            continue
        pick_c(ctx, task, side=navigation["camera_side"])
        ctx.save_c_distribution(plan)
    ctx.save_c_distribution(plan)
    ctx.set_perception_mode("OFF")


def debug_a(ctx):
    """A-only entry, ending at the same existing A exit waypoint."""
    run_a(ctx)
    navigate(ctx, ["to_B_QR_waypoint"])


def debug_b(ctx):
    """B-only entry: scan the real QR before the same B work function."""
    arrived = navigate(ctx, ["B_QR"])
    plan = scan_b_qr(ctx, arrived=arrived)
    navigate(ctx, ["to_C_waypoint_1", "B1"])
    run_b(ctx, plan)
    navigate(ctx, ["to_C_waypoint_2", "to_C_waypoint_3"])


def debug_c(ctx):
    """C-only entry: scan the real QR before the same C work function."""
    arrived = navigate(ctx, ["C_QR"])
    plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, plan)


def run_ac(ctx):
    """Existing AC entry, composed from the same blocks without mode branches."""
    run_a(ctx)
    arrived = navigate(ctx, ["to_B_QR_waypoint", "B_QR"])
    scan_b_qr(ctx, arrived=arrived)
    arrived = navigate(ctx, [
        "to_C_waypoint_1", "to_C_waypoint_2", "to_C_waypoint_3", "C_QR",
    ])
    plan = scan_c_qr(ctx, arrived=arrived)
    run_c(ctx, plan)
