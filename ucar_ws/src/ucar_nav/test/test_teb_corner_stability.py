#!/usr/bin/env python3
import pathlib
import unittest

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


class TebCornerStabilityTests(unittest.TestCase):
    def setUp(self):
        path = PACKAGE / "config/local_planners/teb_corner_safe.yaml"
        self.teb = yaml.safe_load(path.read_text(encoding="utf-8"))[
            "TebLocalPlannerROS"
        ]

    def test_optimizer_stays_within_robot_compute_budget(self):
        self.assertEqual(2, self.teb["no_inner_iterations"])
        self.assertEqual(1, self.teb["no_outer_iterations"])

    def test_straights_discourage_but_do_not_disable_lateral_motion(self):
        self.assertEqual(0.02, self.teb["max_vel_y"])
        self.assertEqual(0.20, self.teb["acc_lim_y"])
        self.assertGreaterEqual(self.teb["weight_kinematics_nh"], 20.0)

    def test_corner_profile_prefers_forward_motion(self):
        self.assertLessEqual(self.teb["max_vel_x_backwards"], 0.10)
        self.assertGreaterEqual(
            self.teb["weight_kinematics_forward_drive"], 20.0
        )

    def test_corner_profile_keeps_clearance_from_walls(self):
        self.assertGreaterEqual(self.teb["min_obstacle_dist"], 0.15)
        self.assertGreaterEqual(self.teb["inflation_dist"], 0.30)


if __name__ == "__main__":
    unittest.main()
