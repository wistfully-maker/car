#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from ucar_corner_supervisor.swept_collision import (
    GridMap,
    apply_grid_update,
    can_reuse_sweep_cache,
    check_rotation_sweep,
    needs_rotation_sweep,
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
    def test_sweep_is_only_needed_near_a_corner_during_an_active_goal(self):
        self.assertFalse(
            needs_rotation_sweep(False, "IDLE", 0.1, 0.25)
        )
        self.assertFalse(
            needs_rotation_sweep(True, "FOLLOWING", 0.4, 0.25)
        )
        self.assertTrue(
            needs_rotation_sweep(True, "FOLLOWING", 0.25, 0.25)
        )
        self.assertTrue(
            needs_rotation_sweep(True, "TURNING", None, 0.25)
        )
        self.assertTrue(
            needs_rotation_sweep(True, "BLOCKED", None, 0.25)
        )

    def test_sweep_cache_requires_fresh_unchanged_costmap(self):
        self.assertTrue(
            can_reuse_sweep_cache(True, 12, 12, False, 0.05, 0.20)
        )
        self.assertFalse(
            can_reuse_sweep_cache(True, 12, 13, False, 0.05, 0.20)
        )
        self.assertFalse(
            can_reuse_sweep_cache(False, 12, 12, False, 0.05, 0.20)
        )
        self.assertFalse(
            can_reuse_sweep_cache(True, 12, 12, True, 0.05, 0.20)
        )
        self.assertFalse(
            can_reuse_sweep_cache(True, 12, 12, False, 0.20, 0.20)
        )

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
        set_cost(grid, 0.17, 0.07, 254)
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

    def test_cell_intersecting_footprint_edge_is_unsafe(self):
        grid = empty_grid()
        set_cost(grid, 0.17, 0.0, 254)
        result = check_rotation_sweep(
            grid,
            pose=(0.0, 0.0, 0.0),
            target_yaw=0.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertFalse(result.safe)

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
        set_cost(grid, 0.17, -0.07, 254)
        result = check_rotation_sweep(
            grid,
            pose=(0.0, 0.0, 0.0),
            target_yaw=-math.pi / 2.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=253,
        )
        self.assertFalse(result.safe)

    def test_rotated_grid_origin_maps_world_footprint_correctly(self):
        grid = GridMap(
            width=20,
            height=20,
            resolution=0.1,
            origin_x=1.0,
            origin_y=2.0,
            origin_yaw=math.pi / 2.0,
            data=[0] * 400,
        )
        grid.data[5 * grid.width + 5] = 100
        result = check_rotation_sweep(
            grid,
            pose=(0.45, 2.55, 0.0),
            target_yaw=0.0,
            footprint=FOOTPRINT,
            angle_step=math.radians(3.0),
            lethal_threshold=100,
        )
        self.assertFalse(result.safe)

    def test_incremental_update_changes_exact_grid_region(self):
        grid = GridMap(
            width=4,
            height=3,
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            data=[0] * 12,
        )
        apply_grid_update(
            grid,
            x=1,
            y=1,
            width=2,
            height=2,
            data=[10, 11, 20, 21],
        )
        self.assertEqual(grid.data, [0, 0, 0, 0, 0, 10, 11, 0, 0, 20, 21, 0])

    def test_incremental_update_rejects_out_of_bounds_region(self):
        grid = empty_grid()
        with self.assertRaises(ValueError):
            apply_grid_update(
                grid,
                x=99,
                y=99,
                width=2,
                height=2,
                data=[1, 2, 3, 4],
            )


if __name__ == "__main__":
    unittest.main()
