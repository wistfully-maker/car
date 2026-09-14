#!/usr/bin/env python3

import pathlib
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from ucar_waypoint_nav.map_geometry import (  # noqa: E402
    MapGeometry,
    astar,
    clearance_map,
    inflate_obstacles,
    load_map,
    read_pgm,
)


class MapGeometryTests(unittest.TestCase):
    def test_world_pixel_round_trip_uses_bottom_left_map_origin(self):
        geometry = MapGeometry(
            width=480,
            height=256,
            resolution=0.05,
            origin=(-12.2, -12.2),
        )
        pixel = geometry.world_to_pixel(-1.40219, -0.627908)
        world = geometry.pixel_to_world(*pixel)
        self.assertAlmostEqual(world[0], -1.40219, delta=0.05)
        self.assertAlmostEqual(world[1], -0.627908, delta=0.05)

    def test_inflation_blocks_cells_within_euclidean_radius(self):
        occupied = [[False] * 7 for _ in range(7)]
        occupied[3][3] = True
        inflated = inflate_obstacles(occupied, radius_cells=2)
        self.assertTrue(inflated[3][5])
        self.assertTrue(inflated[4][4])
        self.assertFalse(inflated[1][1])

    def test_astar_routes_around_wall_without_cutting_blocked_corner(self):
        occupied = [[False] * 6 for _ in range(6)]
        for row in range(5):
            occupied[row][2] = True
        path = astar(occupied, start=(0, 0), goal=(5, 0))
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (5, 0))
        self.assertTrue(all(not occupied[row][column] for column, row in path))
        self.assertIn((2, 5), path)

    def test_read_pgm_supports_binary_p5_with_comment(self):
        content = b"P5\n# fixture\n3 2\n255\n" + bytes([0, 1, 2, 3, 4, 5])
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "map.pgm"
            path.write_bytes(content)
            width, height, maximum, pixels = read_pgm(path)
        self.assertEqual((width, height, maximum), (3, 2, 255))
        self.assertEqual(pixels, [[0, 1, 2], [3, 4, 5]])

    def test_read_pgm_preserves_whitespace_valued_first_binary_pixel(self):
        content = b"P5\n2 1\n255\n" + bytes([10, 200])
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "map.pgm"
            path.write_bytes(content)
            _, _, _, pixels = read_pgm(path)
        self.assertEqual(pixels, [[10, 200]])

    def test_load_map_applies_ros_occupancy_thresholds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "fixture.pgm").write_text(
                "P2\n3 1\n255\n0 205 254\n", encoding="ascii"
            )
            (root / "fixture.yaml").write_text(
                "\n".join(
                    [
                        "image: fixture.pgm",
                        "resolution: 0.05",
                        "origin: [-1.0, -2.0, 0.0]",
                        "negate: 0",
                        "occupied_thresh: 0.65",
                        "free_thresh: 0.196",
                    ]
                ),
                encoding="utf-8",
            )
            map_data = load_map(root / "fixture.yaml")
        self.assertEqual(map_data.geometry.origin, (-1.0, -2.0))
        self.assertEqual(map_data.occupied, [[True, True, False]])

    def test_clearance_map_reports_distance_from_nearest_obstacle(self):
        occupied = [[False] * 5 for _ in range(5)]
        occupied[2][2] = True
        clearance = clearance_map(occupied)
        self.assertEqual(clearance[2][2], 0.0)
        self.assertAlmostEqual(clearance[2][4], 2.0)
        self.assertAlmostEqual(clearance[4][4], 2.0 * 2.0**0.5)


if __name__ == "__main__":
    unittest.main()
