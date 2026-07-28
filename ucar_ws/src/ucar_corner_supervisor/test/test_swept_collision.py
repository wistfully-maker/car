#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from ucar_corner_supervisor.swept_collision import (
    GridMap,
    check_rotation_sweep,
)


FOOTPRINT = [
    (0.171, -0.128),
    (0.171, 0.128),
    (-0.171, 0.128),
    (-0.171, -0.128),
]


def empty_grid():
    return GridMap(
        width=100,
        height=100,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-2.5,
        data=[0] * 10000,
    )


def set_cost(grid, x, y, cost):
    column = int(math.floor((x - grid.origin_x) / grid.resolution))
    row = int(math.floor((y - grid.origin_y) / grid.resolution))
    grid.data[row * grid.width + column] = cost


class SweptCollisionTests(unittest.TestCase):
    def test_empty_grid_allows_quarter_turn(self):
        result = check_rotation_sweep(
            empty_grid(),
            pose=(0.0, 0.0, 0.0),
            target_yaw=math.pi / 2.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertTrue(result.safe)
        self.assertIsNone(result.blocking_cell)

    def test_wall_in_outer_corner_sweep_blocks_turn(self):
        grid = empty_grid()
        set_cost(grid, 0.14, 0.17, 254)
        result = check_rotation_sweep(
            grid,
            pose=(0.0, 0.0, 0.0),
            target_yaw=math.pi / 2.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertFalse(result.safe)
        self.assertIsNotNone(result.blocking_cell)

    def test_wall_outside_sweep_is_safe(self):
        grid = empty_grid()
        set_cost(grid, 0.45, 0.45, 254)
        result = check_rotation_sweep(
            grid,
            pose=(0.0, 0.0, 0.0),
            target_yaw=math.pi / 2.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertTrue(result.safe)

    def test_unknown_cell_under_footprint_is_unsafe(self):
        grid = empty_grid()
        set_cost(grid, 0.0, 0.0, -1)
        result = check_rotation_sweep(
            grid,
            pose=(0.0, 0.0, 0.0),
            target_yaw=0.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertFalse(result.safe)

    def test_footprint_outside_grid_is_unsafe(self):
        grid = empty_grid()
        result = check_rotation_sweep(
            grid,
            pose=(2.49, 0.0, 0.0),
            target_yaw=math.pi / 2.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertFalse(result.safe)
        self.assertEqual(result.reason, "footprint_outside_grid")

    def test_clockwise_turn_checks_full_sweep(self):
        grid = empty_grid()
        set_cost(grid, 0.14, -0.17, 254)
        result = check_rotation_sweep(
            grid,
            pose=(0.0, 0.0, 0.0),
            target_yaw=-math.pi / 2.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertFalse(result.safe)


if __name__ == "__main__":
    unittest.main()
