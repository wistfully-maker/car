#!/usr/bin/env python3

import math
import pathlib
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from ucar_waypoint_nav.waypoint_derivation import (  # noqa: E402
    DerivedWaypoint,
    build_waypoint_document,
    derive_sparse_waypoints,
    write_preview_ppm,
)


class WaypointDerivationTests(unittest.TestCase):
    def test_selects_at_most_three_clear_pass_through_points(self):
        path = (
            [(column, 8) for column in range(1, 9)]
            + [(8, row) for row in range(7, 1, -1)]
            + [(column, 2) for column in range(7, 1, -1)]
            + [(2, row) for row in range(3, 11)]
            + [(column, 10) for column in range(3, 12)]
        )
        clearance = [[2.0] * 14 for _ in range(14)]
        clearance[5][8] = 5.0
        clearance[2][5] = 5.0
        clearance[7][2] = 5.0

        result = derive_sparse_waypoints(
            path=path,
            clearance=clearance,
            resolution=0.10,
            direction_window=0.30,
            minimum_turn=math.radians(35.0),
            minimum_spacing=0.40,
            maximum_count=3,
        )

        self.assertGreaterEqual(len(result), 2)
        self.assertLessEqual(len(result), 3)
        self.assertTrue(all(point.kind == "pass_through" for point in result))
        self.assertTrue(all(point.clearance >= 0.20 for point in result))
        self.assertEqual(
            [point.path_index for point in result],
            sorted(point.path_index for point in result),
        )

    def test_straight_path_needs_no_intermediate_waypoint(self):
        path = [(column, 3) for column in range(12)]
        clearance = [[4.0] * 12 for _ in range(7)]
        result = derive_sparse_waypoints(
            path,
            clearance,
            resolution=0.10,
            direction_window=0.30,
            minimum_turn=math.radians(35.0),
            minimum_spacing=0.40,
            maximum_count=3,
        )
        self.assertEqual(result, [])

    def test_removes_intermediate_waypoint_too_close_to_terminal(self):
        path = (
            [(column, 5) for column in range(8)]
            + [(7, row) for row in range(6, 10)]
            + [(column, 9) for column in range(8, 12)]
        )
        clearance = [[3.0] * 12 for _ in range(12)]
        result = derive_sparse_waypoints(
            path,
            clearance,
            resolution=0.10,
            direction_window=0.20,
            minimum_turn=math.radians(35.0),
            minimum_spacing=0.30,
            maximum_count=3,
            terminal_spacing=0.60,
        )
        self.assertTrue(
            all(
                (len(path) - 1 - point.path_index) * 0.10 >= 0.60
                for point in result
            )
        )

    def test_document_appends_confirmed_terminal_pose(self):
        document = build_waypoint_document(
            map_sha256="abc123",
            derived=[
                DerivedWaypoint(
                    column=4,
                    row=5,
                    path_index=10,
                    yaw=0.5,
                    clearance=0.35,
                    turn_angle=1.2,
                )
            ],
            pixel_to_world=lambda column, row: (1.0, 2.0),
            terminal=(-1.40219, -0.627908, -3.0878),
        )
        self.assertEqual(document["map_sha256"], "abc123")
        self.assertEqual(document["waypoints"][0]["kind"], "pass_through")
        self.assertEqual(document["waypoints"][-1]["kind"], "terminal")
        self.assertAlmostEqual(document["waypoints"][-1]["yaw"], -3.0878)

    def test_preview_writer_marks_path_and_waypoints(self):
        pixels = [[254] * 4 for _ in range(3)]
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "preview.ppm"
            write_preview_ppm(
                output,
                pixels,
                path=[(0, 0), (1, 0)],
                waypoints=[(1, 0)],
                start=(0, 0),
                goal=(3, 2),
            )
            content = output.read_bytes()
        self.assertTrue(content.startswith(b"P6\n4 3\n255\n"))
        self.assertGreater(len(content), len(b"P6\n4 3\n255\n") + 12)


if __name__ == "__main__":
    unittest.main()
