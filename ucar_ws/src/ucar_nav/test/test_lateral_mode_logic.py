#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "lateral_mode_logic.py"
SPEC = importlib.util.spec_from_file_location("lateral_mode_logic", str(SCRIPT))
logic = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(logic)


STRAIGHT = [(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)]
JAGGED = [(0.0, 0.0), (0.2, 0.01), (0.4, -0.01), (0.8, 0.0)]
RIGHT_ANGLE = [
    (0.0, 0.0),
    (0.4, 0.0),
    (0.8, 0.0),
    (0.8, 0.4),
    (0.8, 0.8),
]


def metrics(points):
    return logic.analyze_path(points, lookahead_distance=1.6, resample_spacing=0.1)


class PathAnalysisTests(unittest.TestCase):
    def test_straight_path_has_small_turn_angle(self):
        self.assertLess(metrics(STRAIGHT).turn_angle_deg, 10.0)

    def test_small_jagged_noise_does_not_look_like_a_corner(self):
        self.assertLess(metrics(JAGGED).turn_angle_deg, 10.0)

    def test_right_angle_path_is_detected(self):
        result = metrics(RIGHT_ANGLE)
        self.assertGreater(result.turn_angle_deg, 45.0)
        self.assertAlmostEqual(math.pi / 2.0, result.exit_heading_rad, delta=0.15)

    def test_duplicate_points_are_ignored(self):
        result = metrics([(0.0, 0.0), (0.0, 0.0), (0.8, 0.0)])
        self.assertLess(result.turn_angle_deg, 10.0)
        self.assertAlmostEqual(0.8, result.analyzed_length, places=6)

    def test_short_path_is_safe(self):
        result = metrics([(0.0, 0.0), (0.2, 0.0)])
        self.assertEqual(0.0, result.turn_angle_deg)
        self.assertEqual(0.0, result.exit_heading_rad)


class LateralModeStateMachineTests(unittest.TestCase):
    def setUp(self):
        self.machine = logic.LateralModeStateMachine(
            enter_angle_deg=45.0,
            exit_angle_deg=10.0,
            heading_exit_tolerance_deg=10.0,
            exit_hold_time=0.5,
        )
        self.corner = logic.PathMetrics(90.0, math.pi / 2.0, 1.0)
        self.straight = logic.PathMetrics(0.0, math.pi / 2.0, 1.0)

    def test_enters_corner_immediately(self):
        self.assertEqual(
            "CORNER", self.machine.update(0.0, self.corner, 0.0)
        )

    def test_heading_error_keeps_corner_mode(self):
        self.machine.update(0.0, self.corner, 0.0)
        self.assertEqual(
            "CORNER", self.machine.update(1.0, self.straight, 0.0)
        )

    def test_exits_after_straight_and_aligned_hold(self):
        self.machine.update(0.0, self.corner, 0.0)
        self.assertEqual(
            "CORNER",
            self.machine.update(1.0, self.straight, math.pi / 2.0),
        )
        self.assertEqual(
            "STRAIGHT",
            self.machine.update(1.5, self.straight, math.pi / 2.0),
        )

    def test_failed_exit_condition_resets_hold_timer(self):
        self.machine.update(0.0, self.corner, 0.0)
        self.machine.update(1.0, self.straight, math.pi / 2.0)
        self.machine.update(1.3, self.straight, 0.0)
        self.assertEqual(
            "CORNER",
            self.machine.update(1.6, self.straight, math.pi / 2.0),
        )
        self.assertEqual(
            "STRAIGHT",
            self.machine.update(2.1, self.straight, math.pi / 2.0),
        )

    def test_invalid_plan_returns_to_straight(self):
        self.machine.update(0.0, self.corner, 0.0)
        self.assertEqual(
            "STRAIGHT",
            self.machine.update(0.1, self.corner, 0.0, plan_valid=False),
        )


class ModeParameterTests(unittest.TestCase):
    def test_straight_parameters(self):
        self.assertEqual(
            {
                "max_vel_x": 0.45,
                "max_vel_x_backwards": 0.10,
                "max_vel_y": 0.02,
                "acc_lim_y": 0.20,
            },
            logic.mode_parameters(
                "STRAIGHT", 0.45, 0.10, 0.02, 0.20, 0.20, 0.02, 0.18, 0.60
            ),
        )

    def test_corner_parameters(self):
        self.assertEqual(
            {
                "max_vel_x": 0.20,
                "max_vel_x_backwards": 0.02,
                "max_vel_y": 0.18,
                "acc_lim_y": 0.60,
            },
            logic.mode_parameters(
                "CORNER", 0.45, 0.10, 0.02, 0.20, 0.20, 0.02, 0.18, 0.60
            ),
        )

    def test_unknown_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            logic.mode_parameters(
                "UNKNOWN", 0.45, 0.10, 0.02, 0.20, 0.20, 0.02, 0.18, 0.60
            )


if __name__ == "__main__":
    unittest.main()
