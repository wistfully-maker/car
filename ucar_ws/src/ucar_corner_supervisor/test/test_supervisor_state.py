#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from ucar_corner_supervisor.corner_geometry import (
    CornerObservation,
    Supervisor,
    SupervisorConfig,
)


def corner(distance=0.2, exit_heading=math.pi / 2.0):
    return CornerObservation(
        distance=distance,
        turn_angle=math.pi / 2.0,
        exit_heading=exit_heading,
        point=(distance, 0.0),
    )


class SupervisorStateTests(unittest.TestCase):
    def setUp(self):
        self.supervisor = Supervisor(SupervisorConfig())

    def update(self, now=0.0, **overrides):
        arguments = {
            "goal_active": True,
            "raw_fresh": True,
            "tf_valid": True,
            "yaw": 0.0,
            "corner": None,
            "raw_command": (0.20, 0.08, 0.10),
        }
        arguments.update(overrides)
        return self.supervisor.update(now=now, **arguments)

    def test_idle_without_active_goal_outputs_zero(self):
        result = self.update(goal_active=False)
        self.assertEqual(result.state, "IDLE")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_following_clamps_lateral_velocity(self):
        result = self.update()
        self.assertEqual(result.state, "FOLLOWING")
        self.assertEqual(result.command, (0.20, 0.02, 0.10))

    def test_distant_corner_does_not_trigger_or_change_forward_motion(self):
        result = self.update(corner=corner(distance=0.6))
        self.assertEqual(result.state, "FOLLOWING")
        self.assertEqual(result.command, (0.20, 0.02, 0.10))

    def test_near_corner_stops_translation_and_turns_in_place(self):
        result = self.update(corner=corner(distance=0.24))
        self.assertEqual(result.state, "TURNING")
        self.assertEqual(result.command[:2], (0.0, 0.0))
        self.assertAlmostEqual(result.command[2], 0.35)

    def test_unreliable_near_corner_enters_blocked(self):
        result = self.update(
            corner=corner(distance=0.24), corner_confident=False
        )
        self.assertEqual(result.state, "BLOCKED")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_unsafe_sweep_blocks_turn_entry(self):
        result = self.update(
            corner=corner(distance=0.24), sweep_safe=False
        )
        self.assertEqual(result.state, "BLOCKED")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_new_obstacle_during_turn_enters_blocked(self):
        self.update(corner=corner(distance=0.24))
        result = self.update(now=0.1, sweep_safe=False)
        self.assertEqual(result.state, "BLOCKED")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_blocked_latches_until_goal_is_cleared(self):
        self.update(corner=corner(distance=0.24), sweep_safe=False)
        still_blocked = self.update(now=1.0, corner=None, sweep_safe=True)
        cleared = self.update(now=1.1, goal_active=False)
        self.assertEqual(still_blocked.state, "BLOCKED")
        self.assertEqual(cleared.state, "IDLE")

    def test_turning_uses_minimum_angular_speed_near_target(self):
        self.update(corner=corner(distance=0.24))
        result = self.update(now=0.1, yaw=math.radians(80))
        self.assertEqual(result.state, "TURNING")
        self.assertAlmostEqual(result.command[2], 0.18)

    def test_heading_must_stay_aligned_for_hold_period(self):
        self.update(corner=corner(distance=0.24))
        first = self.update(now=0.1, yaw=math.radians(85))
        middle = self.update(now=0.25, yaw=math.radians(88))
        completed = self.update(now=0.41, yaw=math.radians(89))
        self.assertEqual(first.state, "EXIT_ALIGN")
        self.assertEqual(middle.state, "EXIT_ALIGN")
        self.assertEqual(completed.state, "FOLLOWING")
        self.assertEqual(first.command, (0.0, 0.0, 0.0))

    def test_alignment_hold_resets_when_heading_leaves_tolerance(self):
        self.update(corner=corner(distance=0.24))
        self.update(now=0.1, yaw=math.radians(85))
        result = self.update(now=0.2, yaw=math.radians(70))
        self.assertEqual(result.state, "TURNING")

    def test_turn_timeout_enters_error_and_stops(self):
        self.update(corner=corner(distance=0.24))
        result = self.update(now=8.01, yaw=0.0)
        self.assertEqual(result.state, "ERROR")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_stale_raw_command_during_turn_enters_error_and_stops(self):
        self.update(corner=corner(distance=0.24))
        result = self.update(now=0.6, raw_fresh=False)
        self.assertEqual(result.state, "ERROR")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_stale_raw_command_stops_without_leaving_following(self):
        result = self.update(raw_fresh=False)
        self.assertEqual(result.state, "FOLLOWING")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_stale_raw_command_cannot_trigger_a_near_corner(self):
        result = self.update(
            raw_fresh=False, corner=corner(distance=0.20)
        )
        self.assertEqual(result.state, "FOLLOWING")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_missing_tf_during_goal_enters_error(self):
        result = self.update(tf_valid=False)
        self.assertEqual(result.state, "ERROR")
        self.assertEqual(result.command, (0.0, 0.0, 0.0))

    def test_same_corner_is_suppressed_until_release(self):
        self.update(corner=corner(distance=0.24))
        self.update(now=0.1, yaw=math.radians(89))
        completed = self.update(
            now=0.41, yaw=math.radians(90), corner=corner(distance=0.20)
        )
        held = self.update(
            now=0.5, yaw=math.radians(90), corner=corner(distance=0.30)
        )
        released = self.update(
            now=0.6, yaw=math.radians(90), corner=corner(distance=0.50)
        )
        self.assertEqual(completed.state, "FOLLOWING")
        self.assertEqual(held.state, "FOLLOWING")
        self.assertEqual(released.state, "FOLLOWING")
        retriggered = self.update(
            now=0.7, yaw=math.radians(90), corner=corner(distance=0.20)
        )
        self.assertEqual(retriggered.state, "TURNING")

    def test_same_corner_releases_at_exact_release_distance(self):
        self.update(corner=corner(distance=0.24))
        self.update(now=0.1, yaw=math.radians(89))
        self.update(now=0.41, yaw=math.radians(90))
        self.update(
            now=0.5, yaw=math.radians(90), corner=corner(distance=0.45)
        )
        result = self.update(
            now=0.6, yaw=math.radians(90), corner=corner(distance=0.20)
        )
        self.assertEqual(result.state, "TURNING")


if __name__ == "__main__":
    unittest.main()
