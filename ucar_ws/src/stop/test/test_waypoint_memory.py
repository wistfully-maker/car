"""Unit tests for ROS-independent workshop waypoint memory and routing."""

import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from stop_integration.waypoint_memory import WaypointMemory, build_phase2_route


class WaypointMemoryTests(unittest.TestCase):
    def setUp(self):
        self.memory = WaypointMemory(3)

    def test_records_any_canonical_workshop_before_simulation_target_is_known(self):
        self.memory.record("电子产品生产车间", 0)
        self.memory.record("日用品加工车间", 1)

        self.assertEqual({0}, self.memory.candidates("电子产品生产车间"))
        self.assertEqual({1}, self.memory.candidates("日用品加工车间"))
        self.assertEqual(0, self.memory.unique_waypoint("电子产品生产车间"))

    def test_duplicate_observation_is_idempotent(self):
        self.memory.record("食品加工车间", 2)
        self.memory.record("食品加工车间", 2)

        self.assertEqual({2}, self.memory.candidates("食品加工车间"))

    def test_conflicting_observations_are_not_treated_as_a_unique_waypoint(self):
        self.memory.record("食品加工车间", 0)
        self.memory.record("食品加工车间", 2)

        self.assertEqual({0, 2}, self.memory.candidates("食品加工车间"))
        self.assertIsNone(self.memory.unique_waypoint("食品加工车间"))

    def test_scanned_waypoints_are_independent_from_label_observations(self):
        self.memory.record("食品加工车间", 0)
        self.memory.mark_scanned(1)

        self.assertEqual({1}, self.memory.scanned_waypoints)
        self.assertEqual({0}, self.memory.candidates("食品加工车间"))

    def test_reset_forgets_observations_and_scan_progress(self):
        self.memory.record("食品加工车间", 0)
        self.memory.mark_scanned(0)

        self.memory.reset()

        self.assertEqual(set(), self.memory.candidates("食品加工车间"))
        self.assertEqual(set(), self.memory.scanned_waypoints)

    def test_rejects_invalid_waypoint_indexes(self):
        for invalid_index in (-1, 3, True, "1"):
            with self.subTest(index=invalid_index):
                with self.assertRaises(ValueError):
                    self.memory.record("食品加工车间", invalid_index)
                with self.assertRaises(ValueError):
                    self.memory.mark_scanned(invalid_index)


class Phase2RouteTests(unittest.TestCase):
    def test_known_simulation_workshop_is_visited_first_then_remaining_point(self):
        cases = (
            (1, 0, [0, 2]),  # physical at 2, simulation at 1
            (0, 2, [2, 1]),  # physical at 1, simulation at 3
            (2, 1, [1, 0]),  # physical at 3, simulation at 2
        )
        for physical, preferred, expected in cases:
            with self.subTest(physical=physical, preferred=preferred):
                self.assertEqual(
                    expected,
                    build_phase2_route(physical, preferred, waypoint_count=3),
                )

    def test_unknown_or_conflicting_memory_uses_forward_then_wrapped_fallback(self):
        cases = (
            (1, [2, 0]),
            (0, [1, 2]),
            (2, [0, 1]),
        )
        for physical, expected in cases:
            with self.subTest(physical=physical):
                self.assertEqual(
                    expected,
                    build_phase2_route(physical, None, waypoint_count=3),
                )

    def test_physical_waypoint_is_never_reused_as_preferred_simulation_point(self):
        self.assertEqual(
            [2, 0],
            build_phase2_route(1, 1, waypoint_count=3),
        )

    def test_route_rejects_invalid_arguments(self):
        with self.assertRaises(ValueError):
            build_phase2_route(-1, None, waypoint_count=3)
        with self.assertRaises(ValueError):
            build_phase2_route(0, 3, waypoint_count=3)
        with self.assertRaises(ValueError):
            build_phase2_route(0, None, waypoint_count=1)


if __name__ == "__main__":
    unittest.main()
