#!/usr/bin/env python3
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch"


def load_yaml(relative_path):
    with (PACKAGE / relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def launch_text(name):
    return (LAUNCH / name).read_text(encoding="utf-8")


class NavigationConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        required = (
            "launch/robot_base_bringup.launch",
            "launch/navigation_stack.launch",
            "launch/ucar_navigation.launch",
            "config/amcl/amcl_omni.yaml",
            "config/costmap/common.yaml",
            "config/costmap/global.yaml",
            "config/costmap/local.yaml",
            "config/local_planners/dwa_safe.yaml",
            "config/global_planner.yaml",
            "config/move_base.yaml",
            "scripts/capture_nav_diagnostics.sh",
            "README.md",
            "HANDOFF.md",
        )
        for relative_path in required:
            self.assertTrue(
                (PACKAGE / relative_path).is_file(),
                relative_path,
            )

    def test_launch_files_are_valid_xml(self):
        for path in LAUNCH.glob("*.launch"):
            ET.parse(path)

    def test_navigation_stack_loads_only_dwa(self):
        text = launch_text("navigation_stack.launch")
        self.assertIn(
            'value="dwa_local_planner/DWAPlannerROS"',
            text,
        )
        self.assertIn("dwa_safe.yaml", text)
        self.assertNotIn("teb_local_planner", text.lower())
        self.assertNotIn("teb_safe.yaml", text.lower())

    def test_hardware_and_navigation_launches_are_separated(self):
        hardware = launch_text("robot_base_bringup.launch")
        navigation = launch_text("navigation_stack.launch")
        self.assertIn("base_driver.launch", hardware)
        self.assertIn("ydlidar.launch", hardware)
        self.assertNotIn('pkg="move_base"', hardware)
        self.assertNotIn("base_driver.launch", navigation)
        self.assertNotIn("ydlidar.launch", navigation)
        self.assertNotIn("usb_cam", navigation)

    def test_common_costmap_uses_safe_polygon_and_laser(self):
        common = load_yaml("config/costmap/common.yaml")
        footprint = common["footprint"]
        self.assertEqual(4, len(footprint))
        self.assertEqual(
            [
                [0.191, -0.148],
                [0.191, 0.148],
                [-0.191, 0.148],
                [-0.191, -0.148],
            ],
            footprint,
        )
        source = common["obstacle_layer"]["laser_scan_sensor"]
        self.assertEqual("laser_frame", source["sensor_frame"])
        self.assertEqual("scan", source["topic"])
        self.assertTrue(source["marking"])
        self.assertTrue(source["clearing"])

    def test_local_costmap_has_real_safety_gradient(self):
        local = load_yaml("config/costmap/local.yaml")["local_costmap"]
        self.assertEqual("odom", local["global_frame"])
        self.assertLessEqual(local["transform_tolerance"], 0.5)
        inflation = local["inflation_layer"]
        corner_radius = math.hypot(0.191, 0.148)
        self.assertGreater(inflation["inflation_radius"], corner_radius)
        self.assertGreaterEqual(inflation["inflation_radius"], 0.35)

    def test_dwa_safe_profile_is_holonomic_and_conservative(self):
        dwa = load_yaml(
            "config/local_planners/dwa_safe.yaml"
        )["DWAPlannerROS"]
        self.assertGreater(dwa["vy_samples"], 0)
        self.assertGreater(dwa["max_vel_y"], 0)
        self.assertLess(dwa["min_vel_y"], 0)
        self.assertLessEqual(dwa["max_vel_x"], 0.25)
        self.assertLessEqual(dwa["max_vel_y"], 0.15)
        self.assertLessEqual(dwa["max_vel_theta"], 0.5)
        self.assertGreaterEqual(dwa["occdist_scale"], 0.15)
        self.assertGreaterEqual(dwa["stop_time_buffer"], 0.4)

    def test_move_base_disables_uncommanded_recovery_rotation(self):
        move_base = load_yaml("config/move_base.yaml")
        self.assertFalse(move_base["recovery_behavior_enabled"])
        self.assertFalse(move_base["clearing_rotation_allowed"])

    def test_docs_keep_backups_outside_catkin_source(self):
        for relative_path in ("README.md", "HANDOFF.md"):
            text = (PACKAGE / relative_path).read_text(encoding="utf-8")
            self.assertNotIn("ucar_ws/src/ucar_nav.backup", text)
            self.assertIn("ucar_nav_backups", text)


if __name__ == "__main__":
    unittest.main()
