#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from ucar_corner_supervisor.path_corners import extract_corner_plan


def extract(points, minimum_length=0.15):
    return extract_corner_plan(
        points,
        simplify_tolerance=0.04,
        min_corner_angle=math.radians(45.0),
        min_segment_length=minimum_length,
        max_fit_residual=0.08,
        same_turn_merge_distance=0.20,
    )


class PathCornerTests(unittest.TestCase):
    def test_straight_path_has_no_corners(self):
        self.assertEqual(extract([(0, 0), (0.5, 0), (1.0, 0)]), [])

    def test_single_right_angle_uses_stable_corridor_headings(self):
        corners = extract([(0, 0), (1, 0), (1, 1)])
        self.assertEqual(len(corners), 1)
        self.assertAlmostEqual(corners[0].entry_heading, 0.0, delta=0.03)
        self.assertAlmostEqual(
            corners[0].exit_heading, math.pi / 2.0, delta=0.03
        )
        self.assertTrue(corners[0].confident)

    def test_grid_jitter_does_not_change_corridor_headings(self):
        points = [
            (0.0, 0.00),
            (0.2, 0.01),
            (0.4, -0.01),
            (0.6, 0.01),
            (0.8, 0.00),
            (1.0, 0.00),
            (1.01, 0.2),
            (0.99, 0.4),
            (1.01, 0.6),
            (1.0, 0.8),
        ]
        corners = extract(points)
        self.assertEqual(len(corners), 1)
        self.assertAlmostEqual(corners[0].entry_heading, 0.0, delta=0.08)
        self.assertAlmostEqual(
            corners[0].exit_heading, math.pi / 2.0, delta=0.08
        )

    def test_short_connector_preserves_opposite_turns(self):
        points = [(0, 0), (1, 0), (1, 0.25), (2, 0.25)]
        corners = extract(points)
        self.assertEqual(len(corners), 2)
        self.assertGreater(corners[0].turn_angle, 0.0)
        self.assertLess(corners[1].turn_angle, 0.0)
        self.assertAlmostEqual(
            corners[0].exit_heading, math.pi / 2.0, delta=0.03
        )
        self.assertAlmostEqual(corners[1].exit_heading, 0.0, delta=0.03)

    def test_short_connector_preserves_same_direction_turns(self):
        points = [(0, 0), (1, 0), (1, 0.25), (0.7, 0.25)]
        corners = extract(points)
        self.assertEqual(len(corners), 2)
        self.assertGreater(corners[0].turn_angle, 0.0)
        self.assertGreater(corners[1].turn_angle, 0.0)

    def test_too_short_connector_marks_both_adjacent_fits_unreliable(self):
        points = [(0, 0), (1, 0), (1, 0.10), (2, 0.10)]
        corners = extract(points)
        self.assertEqual(len(corners), 2)
        self.assertFalse(corners[0].confident)
        self.assertFalse(corners[1].confident)

    def test_corner_distances_are_ordered_along_path(self):
        points = [(0, 0), (1, 0), (1, 0.3), (2, 0.3)]
        corners = extract(points)
        self.assertLess(corners[0].path_distance, corners[1].path_distance)


if __name__ == "__main__":
    unittest.main()
