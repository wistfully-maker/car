#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RosAssetTests(unittest.TestCase):
    def test_manager_uses_move_base_action_without_publishing_cmd_vel(self):
        source = (
            ROOT / "scripts/waypoint_route_manager.py"
        ).read_text(encoding="utf-8")
        self.assertIn('SimpleActionClient("/move_base", MoveBaseAction)', source)
        self.assertIn('lookup_transform("map", "base_link"', source)
        self.assertIn('"/task/pickup_navigation_goal"', source)
        self.assertIn('"/task/pickup_arrived"', source)
        self.assertIn('"/ucar_waypoint_nav/state"', source)
        self.assertIn('"/ucar_waypoint_nav/diagnostic"', source)
        self.assertIn('rospy.Service("/ucar_waypoint_nav/start"', source)
        self.assertIn('rospy.Service("/ucar_waypoint_nav/cancel"', source)
        self.assertNotIn('Publisher("/cmd_vel"', source)
        self.assertNotIn("cancel_goal()\n        self._send", source)

    def test_launch_selects_one_global_planner_and_only_teb_locally(self):
        launch_path = ROOT / "launch/waypoint_teb_navigation.launch"
        ET.parse(str(launch_path))
        text = launch_path.read_text(encoding="utf-8")
        self.assertIn('name="global_planner" default="global_planner"', text)
        self.assertIn("global_planner/GlobalPlanner", text)
        self.assertIn("navfn/NavfnROS", text)
        self.assertIn("teb_local_planner/TebLocalPlannerROS", text)
        self.assertNotIn("corner_supervisor", text)
        self.assertNotIn("DWAPlannerROS", text)

    def test_teb_baseline_has_online_pass_through_settings(self):
        config = yaml.safe_load(
            (ROOT / "config/teb.yaml").read_text(encoding="utf-8")
        )["TebLocalPlannerROS"]
        self.assertTrue(config["free_goal_vel"])
        self.assertEqual(config["no_inner_iterations"], 5)
        self.assertEqual(config["no_outer_iterations"], 4)
        self.assertEqual(config["footprint_model"]["type"], "polygon")
        self.assertEqual(config["max_vel_x"], 0.35)
        self.assertGreater(config["max_vel_y"], 0.0)

    def test_controller_requests_ten_hertz(self):
        config = yaml.safe_load(
            (ROOT / "config/controller.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(config["controller_frequency"], 10.0)


if __name__ == "__main__":
    unittest.main()
