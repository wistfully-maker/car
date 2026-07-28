#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class RosAssetTests(unittest.TestCase):
    def test_node_is_valid_python_and_contains_required_interfaces(self):
        source = (PACKAGE_DIR / "scripts/corner_supervisor_node.py").read_text(
            encoding="utf-8"
        )
        ast.parse(source)
        for interface in (
            "/move_base/cmd_vel_raw",
            "/move_base/NavfnROS/plan",
            "/move_base/status",
            "/cmd_vel",
            "~state",
            "getSystemState",
        ):
            self.assertIn(interface, source)

    def test_config_contains_all_control_parameters(self):
        config = (PACKAGE_DIR / "config/corner_supervisor.yaml").read_text(
            encoding="utf-8"
        )
        for parameter in (
            "controller_rate",
            "raw_command_timeout",
            "path_search_distance",
            "corner_trigger_distance",
            "corner_release_distance",
            "min_corner_angle_deg",
            "path_resample_spacing",
            "following_max_lateral",
            "turn_max_angular",
            "turn_min_angular",
            "turn_kp",
            "heading_tolerance_deg",
            "heading_hold_time",
            "turn_timeout",
        ):
            self.assertIn(parameter + ":", config)

    def test_launch_starts_only_the_supervisor_node(self):
        root = ET.parse(
            str(PACKAGE_DIR / "launch/corner_supervisor.launch")
        ).getroot()
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["pkg"], "ucar_corner_supervisor")
        self.assertEqual(nodes[0].attrib["type"], "corner_supervisor_node.py")
        self.assertEqual(root.findall("include"), [])


if __name__ == "__main__":
    unittest.main()
