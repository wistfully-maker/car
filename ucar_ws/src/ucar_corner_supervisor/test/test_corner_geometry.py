#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from corner_geometry import find_first_corner, normalize_angle, resample_polyline


class CornerGeometryTests(unittest.TestCase):
    def test_normalize_angle_wraps_to_signed_pi(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-1.5 * math.pi), 0.5 * math.pi)

    def test_resample_polyline_uses_nearly_uniform_spacing(self):
        points = [(0.0, 0.0), (0.13, 0.0), (0.57, 0.0)]
        sampled = resample_polyline(points, 0.05)
        distances = [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(sampled, sampled[1:])
        ]
        self.assertGreater(len(sampled), 10)
        self.assertTrue(all(distance <= 0.051 for distance in distances))
        self.assertAlmostEqual(sampled[-1][0], 0.57)

    def test_straight_path_has_no_corner(self):
        points = [(index * 0.05, 0.0) for index in range(31)]
        self.assertIsNone(
            find_first_corner(points, search_distance=1.2, min_angle=math.radians(45))
        )

    def test_finds_ninety_degree_corner_and_exit_heading(self):
        points = (
            [(index * 0.05, 0.0) for index in range(17)]
            + [(0.8, index * 0.05) for index in range(1, 17)]
        )
        corner = find_first_corner(
            points,
            search_distance=1.2,
            min_angle=math.radians(45),
            spacing=0.05,
            direction_window=0.20,
        )
        self.assertIsNotNone(corner)
        self.assertAlmostEqual(corner.distance, 0.8, delta=0.08)
        self.assertAlmostEqual(corner.turn_angle, math.pi / 2.0, delta=0.08)
        self.assertAlmostEqual(corner.exit_heading, math.pi / 2.0, delta=0.08)

    def test_ignores_gentle_bend_below_angle_threshold(self):
        points = [
            (index * 0.05, 0.15 * math.sin(index * 0.05))
            for index in range(31)
        ]
        self.assertIsNone(
            find_first_corner(
                points,
                search_distance=1.2,
                min_angle=math.radians(45),
                direction_window=0.20,
            )
        )

    def test_ignores_corner_beyond_search_distance(self):
        points = (
            [(index * 0.05, 0.0) for index in range(31)]
            + [(1.5, index * 0.05) for index in range(1, 11)]
        )
        self.assertIsNone(
            find_first_corner(
                points,
                search_distance=1.2,
                min_angle=math.radians(45),
                direction_window=0.20,
            )
        )


if __name__ == "__main__":
    unittest.main()
