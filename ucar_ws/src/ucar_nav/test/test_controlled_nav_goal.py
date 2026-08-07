#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "controlled_nav_goal.py"
SPEC = importlib.util.spec_from_file_location("controlled_nav_goal", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ControlledNavGoalTests(unittest.TestCase):
    def test_computes_forward_goal_in_map_frame(self):
        x, y, yaw = MODULE.compute_relative_goal(2.0, -3.0, math.pi, 0.5)
        self.assertAlmostEqual(x, 1.5)
        self.assertAlmostEqual(y, -3.0)
        self.assertAlmostEqual(yaw, math.pi)

    def test_rejects_nonpositive_or_long_distance(self):
        for distance in (0.0, -0.1, 0.51):
            with self.subTest(distance=distance):
                with self.assertRaises(ValueError):
                    MODULE.validate_distance(distance)

    def test_parses_named_waterplus_waypoint(self):
        xml_text = """
        <Waterplus>
          <Waypoint>
            <Name>1</Name>
            <Pos_x>-1.4</Pos_x><Pos_y>-0.6</Pos_y><Pos_z>0</Pos_z>
            <Ori_x>0</Ori_x><Ori_y>0</Ori_y>
            <Ori_z>1</Ori_z><Ori_w>0</Ori_w>
          </Waypoint>
        </Waterplus>
        """
        pose = MODULE.parse_waypoint_xml(xml_text, "1")
        self.assertEqual(pose, (-1.4, -0.6, 0.0, 0.0, 0.0, 1.0, 0.0))

    def test_rejects_missing_waypoint(self):
        with self.assertRaises(ValueError):
            MODULE.parse_waypoint_xml("<Waterplus/>", "1")


if __name__ == "__main__":
    unittest.main()
