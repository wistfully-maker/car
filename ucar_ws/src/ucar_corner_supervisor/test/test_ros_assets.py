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
            "/move_base/local_costmap/costmap",
            "/move_base/local_costmap/costmap_updates",
            "/cmd_vel",
            "~state",
            "getSystemState",
            "extract_corner_plan",
            "check_rotation_sweep",
            "apply_grid_update",
            "corner_confidence",
            "sweep_safe",
            "sweep_needed",
            "sweep_reason",
        ):
            self.assertIn(interface, source)

    def test_config_contains_all_control_parameters(self):
        config = (PACKAGE_DIR / "config/corner_supervisor.yaml").read_text(
            encoding="utf-8"
        )
        for parameter in (
            "controller_rate",
            "raw_command_timeout",
            "corner_trigger_distance",
            "corner_release_distance",
            "min_corner_angle_deg",
            "following_max_lateral",
            "turn_max_angular",
            "turn_min_angular",
            "turn_kp",
            "heading_tolerance_deg",
            "heading_hold_time",
            "turn_timeout",
            "path_simplify_tolerance",
            "min_stable_segment_length",
            "max_fit_residual",
            "same_turn_merge_distance",
            "path_resample_spacing",
            "corner_release_margin",
            "completed_corner_match_distance",
            "completed_corner_match_heading_deg",
            "costmap_timeout",
            "costmap_update_topic",
            "lethal_cost_threshold",
            "sweep_angle_step_deg",
            "sweep_check_rate",
            "footprint",
        ):
            self.assertIn(parameter + ":", config)
        self.assertIn("min_corner_angle_deg: 70.0", config)
        self.assertIn("corner_trigger_distance: 0.20", config)
        self.assertIn("following_max_lateral: 0.05", config)
        self.assertIn("sweep_check_rate: 5.0", config)

    def test_launch_starts_only_the_supervisor_node(self):
        root = ET.parse(
            str(PACKAGE_DIR / "launch/corner_supervisor.launch")
        ).getroot()
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["pkg"], "ucar_corner_supervisor")
        self.assertEqual(nodes[0].attrib["type"], "corner_supervisor_node.py")
        self.assertEqual(root.findall("include"), [])

    def test_integration_launch_wraps_navigation_without_legacy_controller(self):
        root = ET.parse(
            str(PACKAGE_DIR / "launch/navigation_with_corner_supervisor.launch")
        ).getroot()
        xml = ET.tostring(root, encoding="unicode")
        self.assertIn('from="/cmd_vel"', xml)
        self.assertIn('to="/move_base/cmd_vel_raw"', xml)
        self.assertIn('name="enable_lateral_mode_controller"', xml)
        self.assertIn('value="false"', xml)
        self.assertIn("navigation_stack.launch", xml)
        self.assertIn("corner_supervisor.launch", xml)


if __name__ == "__main__":
    unittest.main()
