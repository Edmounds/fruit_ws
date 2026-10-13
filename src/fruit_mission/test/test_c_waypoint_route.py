"""Pure C route geometry checks, without ROS or hardware commands."""

import math
import unittest

from fruit_mission.waypoint_route import (
    C_ENTRANCE_1,
    C_ENTRANCE_2,
    C_QR,
    C_TOP_1,
    C_TOP_2,
    ENTRY_GOAL_CHAIN,
    OTHER_GOAL_CHAIN,
    MissionWaypointRoute,
    WaypointPose,
)


def rectangle_route(length=4.0, width=1.0):
    """Create a two-lane geometric fixture with a nonzero destination yaw."""
    coordinates = [
        (C_QR, -0.2, 0.5 * width, 0.0),
        (C_ENTRANCE_1, 0.0, 0.0, 0.0),
        (C_ENTRANCE_2, 0.0, width, math.pi),
        (C_TOP_1, length, 0.0, math.pi / 2.0),
        (C_TOP_2, length, width, math.pi),
    ]
    for index, name in enumerate(ENTRY_GOAL_CHAIN, start=1):
        coordinates.append((name, length * index / 5.0, 0.0, 0.0))
    for index, name in enumerate(OTHER_GOAL_CHAIN, start=1):
        coordinates.append((name, length * (5 - index) / 5.0, width, math.pi))
    points = [
        WaypointPose(
            name, "goal" if name in (C_QR,) + ENTRY_GOAL_CHAIN + OTHER_GOAL_CHAIN
            else "waypoint",
            order, x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0),
        )
        for order, (name, x, y, yaw) in enumerate(coordinates)
    ]
    return MissionWaypointRoute(points, required_flow="c")


class TestCWaypointRoute(unittest.TestCase):
    """Verify lane routing, rejoin geometry, and saved final yaw."""

    def test_same_lane_and_saved_yaw(self):
        route = rectangle_route()
        names, yaws = route.plan_c_waypoint(2.0, 0.0, math.pi, C_TOP_1, 0.5)
        self.assertEqual(names, [C_TOP_1])
        self.assertAlmostEqual(yaws[-1], math.pi / 2.0)

    def test_other_lane_uses_top_connector(self):
        route = rectangle_route()
        names, yaws = route.plan_c_waypoint(2.0, 1.0, 0.0, C_TOP_1, 0.5)
        self.assertEqual(names, [C_TOP_2, C_TOP_1])
        self.assertAlmostEqual(yaws[0], 0.0)
        self.assertAlmostEqual(yaws[-1], math.pi / 2.0)

    def test_off_lane_rejoins_before_navigation(self):
        route = rectangle_route()
        names, yaws = route.plan_c_waypoint(2.0, -0.15, 0.0, C_TOP_1, 0.5)
        rejoin = route.pose(names[0])
        self.assertAlmostEqual(rejoin.x, 2.0)
        self.assertAlmostEqual(rejoin.y, 0.0)
        self.assertEqual(names[-1], C_TOP_1)
        self.assertEqual(len(names), len(yaws))
        self.assertAlmostEqual(yaws[-1], math.pi / 2.0)

    def test_already_at_target_still_requests_yaw(self):
        route = rectangle_route()
        names, yaws = route.plan_c_waypoint(4.0, 0.0, 0.0, C_TOP_1, 0.5)
        self.assertEqual(names, [C_TOP_1])
        self.assertAlmostEqual(yaws[-1], math.pi / 2.0)

    def test_qr_exit_uses_an_entrance(self):
        route = rectangle_route()
        qr = route.pose(C_QR)
        names, yaws = route.plan_c_waypoint(
            qr.x, qr.y, 0.0, C_TOP_1, 0.5, allow_qr_exit=True,
        )
        self.assertIn(names[0], (C_ENTRANCE_1, C_ENTRANCE_2))
        self.assertEqual(names[-1], C_TOP_1)
        self.assertAlmostEqual(yaws[-1], math.pi / 2.0)

    def test_invalid_start_or_target_is_rejected(self):
        route = rectangle_route()
        for x, y, yaw, limit in (
            (2.0, -1.0, 0.0, 0.5),
            (math.nan, 0.0, 0.0, 0.5),
            (2.0, 0.0, math.inf, 0.5),
            (2.0, 0.0, 0.0, 0.0),
        ):
            with self.subTest(start=(x, y, yaw), limit=limit):
                with self.assertRaises(ValueError):
                    route.plan_c_waypoint(x, y, yaw, C_TOP_1, limit)
        with self.assertRaises(ValueError):
            route.plan_c_waypoint(2.0, 0.0, 0.0, C_QR, 0.5)

    def test_pick_routes_keep_final_pose_and_rejoin(self):
        route = rectangle_route()
        for position in range(1, 13):
            with self.subTest(position=position):
                plan = route.plan_c_navigation(2.0, -0.15, 0.0, position, 0.5)
                pose = route.pose(plan.goal_name)
                self.assertEqual(plan.navigation_names[-1], plan.goal_name)
                self.assertAlmostEqual(
                    plan.navigation_yaws[-1], 2.0 * math.atan2(pose.qz, pose.qw),
                )
                self.assertAlmostEqual(plan.rejoin_point[0], 2.0)
                self.assertAlmostEqual(plan.rejoin_point[1], 0.0)


if __name__ == "__main__":
    unittest.main()
