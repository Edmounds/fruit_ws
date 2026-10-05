"""Competition composition: edit routes and action order here, not in ROS callbacks."""

from fruit_mission.actions import navigate
from fruit_mission.actions import enter_shenzhi
from fruit_mission.actions import navigate_c
from fruit_mission.actions import pick_a
from fruit_mission.actions import pick_b_tree
from fruit_mission.actions import pick_c
from fruit_mission.actions import scan_b_qr
from fruit_mission.actions import scan_c_qr
from fruit_mission.decisions import format_b_tree_name


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
        left = pick_a(ctx, side="left")
        right = pick_a(ctx, side="right")
        enter_shenzhi(ctx, completed=left.success and right.success)
    else:
        enter_shenzhi(ctx, completed=False)

    if navigate(ctx, ["A2"], side="left"):
        left = pick_a(ctx, side="left")
        right = pick_a(ctx, side="right")
        enter_shenzhi(ctx, completed=left.success and right.success)
    else:
        enter_shenzhi(ctx, completed=False)

    if navigate(ctx, ["A3"], side="left"):
        left = pick_a(ctx, side="left")
        right = pick_a(ctx, side="right")
        enter_shenzhi(ctx, completed=left.success and right.success)
    else:
        enter_shenzhi(ctx, completed=False)

    if navigate(ctx, ["A4"], side="left"):
        left = pick_a(ctx, side="left")
        right = pick_a(ctx, side="right")
        enter_shenzhi(ctx, completed=left.success and right.success)
    else:
        enter_shenzhi(ctx, completed=False)


def run_b(ctx, plan):
    """QR tree order; observe/freeze/pick up to two fruit inside one action."""
    ctx.report_current_area("B")
    waypoints = ctx.route.b_goal_names()
    for task in sorted(plan["tasks"], key=lambda item: int(item["sequence"])):
        position = int(task["position_id"])
        tree_name = format_b_tree_name(position)
        if not 1 <= position <= len(waypoints):
            ctx.announce(f"B区{tree_name}编号无效，跳过。")
            continue
        side = "right" if position % 2 == 1 else "left"
        waypoint = waypoints[position - 1]
        if navigate(ctx, [waypoint], side=side):
            completed = pick_b_tree(
                ctx,
                tree_name=tree_name,
                target=str(task["target"]),
                side=side,
                waypoint=waypoint,
            )
            enter_shenzhi(ctx, completed=completed)
        else:
            ctx.announce(f"B区{tree_name}导航失败，跳过。")
            enter_shenzhi(ctx, completed=False)


def run_c(ctx, plan):
    """QR task order; use the route planner's side without direction rechecks."""
    ctx.report_current_area("C")
    for task in sorted(plan["tasks"], key=lambda item: int(item["sequence"])):
        navigation = navigate_c(ctx, task)
        if navigation["status"] != "succeeded":
            ctx.save_c_distribution(plan)
            ctx.recover_home()
            enter_shenzhi(ctx, completed=False)
            continue
        result = pick_c(ctx, task, side=navigation["camera_side"])
        enter_shenzhi(ctx, completed=result.success)
        ctx.save_c_distribution(plan)
    ctx.save_c_distribution(plan)


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
