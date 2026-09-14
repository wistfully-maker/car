import math
import unittest

from qr_item_search.scan_schedule import build_pass_angles


class BuildPassAnglesTest(unittest.TestCase):
    def test_45_degree_first_pass_angles(self):
        self.assertEqual(
            [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
            build_pass_angles(45.0, 0.0),
        )

    def test_45_degree_offset_pass_angles(self):
        self.assertEqual(
            [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
            build_pass_angles(45.0, 22.5),
        )

    def test_90_degree_angles(self):
        self.assertEqual(
            [0.0, 90.0, 180.0, 270.0],
            build_pass_angles(90.0, 0.0),
        )

    def test_30_degree_angles(self):
        self.assertEqual(
            [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0,
             210.0, 240.0, 270.0, 300.0, 330.0],
            build_pass_angles(30.0, 0.0),
        )

    def test_offset_wraps_into_0_360_range(self):
        angles = build_pass_angles(45.0, 22.5)
        for angle in angles:
            self.assertGreaterEqual(angle, 0.0)
            self.assertLess(angle, 360.0)
        self.assertEqual(337.5, angles[-1])

    def test_step_must_divide_360(self):
        for step in (100.0, 7.0, 0.0, -45.0, math.nan, math.inf):
            with self.subTest(step=step):
                with self.assertRaises(ValueError):
                    build_pass_angles(step, 0.0)

    def test_rejects_invalid_offset(self):
        for offset in (-1.0, 360.0, math.nan):
            with self.subTest(offset=offset):
                with self.assertRaises(ValueError):
                    build_pass_angles(45.0, offset)


if __name__ == "__main__":
    unittest.main()
