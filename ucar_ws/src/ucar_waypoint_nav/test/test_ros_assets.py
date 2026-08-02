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

    def test_lidar_loc_launch_replaces_only_amcl_localization(self):
        original_path = ROOT / "launch/waypoint_teb_navigation.launch"
        lidar_path = ROOT / "launch/waypoint_teb_lidar_loc.launch"

        original_text = original_path.read_text(encoding="utf-8")
        self.assertIn('name="amcl" pkg="amcl" type="amcl"', original_text)

        ET.parse(str(lidar_path))
        lidar_text = lidar_path.read_text(encoding="utf-8")
        self.assertIn('name="lidar_loc" pkg="jie_ware" type="lidar_loc"', lidar_text)
        self.assertNotIn('pkg="amcl"', lidar_text)
        self.assertIn('name="move_base" pkg="move_base" type="move_base"', lidar_text)
        self.assertIn('type="waypoint_route_manager.py"', lidar_text)
        for required_param in (
            'name="base_frame" value="base_link"',
            'name="odom_frame" value="odom"',
            'name="laser_frame" value="laser_frame"',
            'name="laser_topic" value="scan"',
        ):
            self.assertIn(required_param, lidar_text)

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

    def test_diagnostic_script_captures_navigation_evidence(self):
        source = (
            ROOT / "scripts/capture_navigation_run.sh"
        ).read_text(encoding="utf-8")
        for required in (
            "rosparam dump",
            "/scan",
            "/tf",
            "/amcl_pose",
            "/odom",
            "/cmd_vel",
            "/move_base/local_costmap/costmap",
            "/ucar_waypoint_nav/diagnostic",
            "rev-parse HEAD",
            "sha256sum",
        ):
            self.assertIn(required, source)

    def test_readme_documents_full_operation_cycle(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        for required in (
            "一键启动",
            "分步启动",
            "global_planner:=global_planner",
            "global_planner:=navfn",
            "/ucar_waypoint_nav/start",
            "/ucar_waypoint_nav/cancel",
            "AMCL",
            "pickup_waypoints.yaml",
            "teb.yaml",
            "trajectory is not feasible",
            "关闭",
        ):
            self.assertIn(required, readme)


if __name__ == "__main__":
    unittest.main()
