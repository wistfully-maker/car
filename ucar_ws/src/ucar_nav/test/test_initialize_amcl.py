#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "initialize_amcl.py"
SPEC = importlib.util.spec_from_file_location("initialize_amcl", str(SCRIPT))
helper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(helper)


class InitializeAmclTests(unittest.TestCase):
    def test_zero_yaw_quaternion(self):
        x, y, z, w = helper.yaw_to_quaternion(0.0)
        self.assertEqual((0.0, 0.0), (x, y))
        self.assertAlmostEqual(0.0, z)
        self.assertAlmostEqual(1.0, w)

    def test_pi_yaw_quaternion(self):
        _, _, z, w = helper.yaw_to_quaternion(math.pi)
        self.assertAlmostEqual(1.0, abs(z))
        self.assertLess(abs(w), 1e-6)

    def test_covariance_uses_ros_pose_indices(self):
        covariance = helper.build_covariance(0.1, 0.2, 0.3)
        self.assertEqual(36, len(covariance))
        self.assertEqual(0.1, covariance[0])
        self.assertEqual(0.2, covariance[7])
        self.assertEqual(0.3, covariance[35])
        self.assertEqual(
            0.0,
            sum(
                value
                for index, value in enumerate(covariance)
                if index not in (0, 7, 35)
            ),
        )


if __name__ == "__main__":
    unittest.main()
