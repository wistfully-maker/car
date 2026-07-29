#!/usr/bin/env python3

import math
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from ucar_waypoint_nav.waypoint_logic import (  # noqa: E402
    RouteProgress,
    Waypoint,
)


def pass_through(x, y, yaw=0.0):
    return Waypoint(
        name="pass",
        kind="pass_through",
        x=x,
        y=y,
        yaw=yaw,
        switch_radius=0.35,
        exit_radius=0.45,
        heading_tolerance=math.radians(40.0),
        minimum_pass_speed=0.08,
    )


def terminal(x, y, yaw=0.0):
    return Waypoint(
        name="terminal",
        kind="terminal",
        x=x,
        y=y,
        yaw=yaw,
        position_tolerance=0.15,
        yaw_tolerance=0.15,
        settle_time=0.5,
    )


class RouteProgressTests(unittest.TestCase):
    def test_switches_before_intermediate_goal_without_stop(self):
        manager = RouteProgress(
            [pass_through(1.0, 0.0), terminal(2.0, 0.0)]
        )
        decision = manager.update(
            x=0.72,
            y=0.0,
            yaw=0.0,
            linear_speed=0.20,
            now=1.0,
        )
        self.assertTrue(decision.send_next_goal)
        self.assertFalse(decision.publish_stop)
        self.assertEqual(manager.current_index, 1)

    def test_rejects_stopped_intermediate_waypoint(self):
        manager = RouteProgress(
            [pass_through(1.0, 0.0), terminal(2.0, 0.0)]
        )
        decision = manager.update(
            x=0.75,
            y=0.0,
            yaw=0.0,
            linear_speed=0.0,
            now=1.0,
        )
        self.assertFalse(decision.send_next_goal)
        self.assertEqual(
            decision.error, "pass-through speed below minimum"
        )

    def test_heading_must_match_outgoing_corridor(self):
        manager = RouteProgress(
            [pass_through(1.0, 0.0, yaw=math.pi / 2), terminal(1.0, 2.0)]
        )
        decision = manager.update(
            x=0.75,
            y=0.0,
            yaw=0.0,
            linear_speed=0.2,
            now=1.0,
        )
        self.assertFalse(decision.send_next_goal)
        self.assertEqual(decision.error, "")

    def test_intermediate_switch_happens_only_once(self):
        manager = RouteProgress(
            [
                pass_through(1.0, 0.0),
                pass_through(2.0, 0.0),
                terminal(3.0, 0.0),
            ]
        )
        first = manager.update(0.7, 0.0, 0.0, 0.2, 1.0)
        repeated = manager.update(0.7, 0.0, 0.0, 0.2, 1.1)
        self.assertTrue(first.send_next_goal)
        self.assertFalse(repeated.send_next_goal)
        self.assertEqual(manager.current_index, 1)

    def test_terminal_requires_stable_pose_and_zero_speed(self):
        manager = RouteProgress([terminal(1.0, 0.0)])
        moving = manager.update(0.95, 0.0, 0.02, 0.10, 1.0)
        entered = manager.update(0.95, 0.0, 0.02, 0.0, 1.1)
        settled = manager.update(0.95, 0.0, 0.02, 0.0, 1.7)
        self.assertFalse(moving.arrived)
        self.assertFalse(entered.arrived)
        self.assertTrue(settled.arrived)


if __name__ == "__main__":
    unittest.main()
