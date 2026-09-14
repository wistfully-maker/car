"""Phase F: lock the front-sector lidar safety helper contract.

Angular sectors in radians, robust trimmed-median statistic, strict filtering
of NaN/infinity/zero/out-of-range values, configurable safety and target stop
distances, and a clean disabled mode.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import lidar_safety
from ucar_delivery.lidar_safety import LidarSafety


def make_ranges(count=64, angle_min=-math.pi, angle_increment=0.1, fill=3.0):
    return [fill] * count


class FrontDistanceTests(unittest.TestCase):
    def setUp(self):
        self.safety = LidarSafety(
            {
                "enabled": True,
                "front_sector_deg": 60.0,
                "min_range": 0.05,
                "max_range": 8.0,
                "safety_stop_distance": 0.25,
                "target_stop_distance": 0.30,
            }
        )

    def test_front_distance_is_median_of_sector(self):
        ranges = make_ranges()
        # 前向索引约 31.4 -> 扇形 ±30° 覆盖索引 ~26..36
        for i in range(26, 37):
            ranges[i] = 0.5
        distance = self.safety.front_distance(
            ranges, -math.pi, 0.1
        )
        self.assertAlmostEqual(0.5, distance, delta=0.05)

    def test_angles_outside_sector_are_ignored(self):
        ranges = make_ranges()
        for i in range(0, 20):
            ranges[i] = 0.1
        for i in range(26, 37):
            ranges[i] = 2.0
        distance = self.safety.front_distance(ranges, -math.pi, 0.1)
        self.assertAlmostEqual(2.0, distance, delta=0.05)

    def test_nan_inf_zero_and_negative_filtered(self):
        ranges = make_ranges()
        for i in range(26, 37):
            ranges[i] = 0.5
        ranges[28] = float("nan")
        ranges[29] = float("inf")
        ranges[30] = 0.0
        ranges[31] = -1.0
        distance = self.safety.front_distance(ranges, -math.pi, 0.1)
        self.assertAlmostEqual(0.5, distance, delta=0.05)

    def test_out_of_range_values_filtered(self):
        ranges = make_ranges()
        for i in range(26, 37):
            ranges[i] = 9.0
        ranges[28] = 0.5
        distance = self.safety.front_distance(ranges, -math.pi, 0.1)
        self.assertAlmostEqual(0.5, distance, delta=0.05)

    def test_median_robust_to_outlier(self):
        ranges = make_ranges()
        for i in range(26, 37):
            ranges[i] = 0.5
        ranges[26] = 0.05
        ranges[27] = 0.06
        distance = self.safety.front_distance(ranges, -math.pi, 0.1)
        self.assertEqual(0.5, distance)

    def test_all_invalid_returns_none(self):
        ranges = [float("nan")] * 64
        self.assertIsNone(
            self.safety.front_distance(ranges, -math.pi, 0.1)
        )
        self.assertIsNone(
            self.safety.front_distance([], -math.pi, 0.1)
        )

    def test_sector_uses_degrees_config(self):
        safety = LidarSafety(
            {
                "enabled": True,
                "front_sector_deg": 20.0,
                "min_range": 0.05,
                "max_range": 8.0,
                "safety_stop_distance": 0.25,
                "target_stop_distance": 0.30,
            }
        )
        ranges = make_ranges()
        for i in range(26, 37):
            ranges[i] = 0.5
        for i in range(30, 34):
            ranges[i] = 2.0
        # 20° 扇形只含 i=30..33（2.0），扇形外（0.5）被排除
        distance = safety.front_distance(ranges, -math.pi, 0.1)
        self.assertAlmostEqual(2.0, distance, delta=0.05)


class DangerTests(unittest.TestCase):
    def setUp(self):
        self.safety = LidarSafety(
            {
                "enabled": True,
                "front_sector_deg": 60.0,
                "min_range": 0.05,
                "max_range": 8.0,
                "safety_stop_distance": 0.25,
                "target_stop_distance": 0.30,
            }
        )

    def test_danger_below_safety_distance(self):
        self.assertTrue(self.safety.danger(0.20))
        self.assertFalse(self.safety.danger(0.30))
        self.assertFalse(self.safety.danger(None))

    def test_disabled_never_dangerous(self):
        safety = LidarSafety(
            {
                "enabled": False,
                "front_sector_deg": 60.0,
                "min_range": 0.05,
                "max_range": 8.0,
                "safety_stop_distance": 0.25,
                "target_stop_distance": 0.30,
            }
        )
        self.assertFalse(safety.danger(0.01))
        self.assertIsNone(safety.front_distance([0.1], 0.0, 1.0))

    def test_config_validation(self):
        with self.assertRaises(ValueError):
            LidarSafety(
                {
                    "enabled": True,
                    "front_sector_deg": 400.0,
                    "min_range": 0.05,
                    "max_range": 8.0,
                    "safety_stop_distance": 0.25,
                    "target_stop_distance": 0.30,
                }
            )
        with self.assertRaises(ValueError):
            LidarSafety(
                {
                    "enabled": True,
                    "front_sector_deg": 60.0,
                    "min_range": 0.05,
                    "max_range": 8.0,
                    "safety_stop_distance": 0.0,
                    "target_stop_distance": 0.30,
                }
            )


if __name__ == "__main__":
    unittest.main()
