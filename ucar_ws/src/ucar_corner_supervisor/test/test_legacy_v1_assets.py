#!/usr/bin/env python3

import ast
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "src"))

from ucar_corner_supervisor.legacy_v1_geometry import (  # noqa: E402
    Supervisor,
    SupervisorConfig,
    find_first_corner,
)


class LegacyV1Tests(unittest.TestCase):
    def test_frozen_config_keeps_first_version_values(self):
        config = (
            PACKAGE_DIR / "config/corner_supervisor_legacy_v1.yaml"
        ).read_text(encoding="utf-8")
        for exact_value in (
            "path_search_distance: 1.2",
            "min_corner_angle_deg: 45.0",
            "direction_window: 0.20",
            "corner_trigger_distance: 0.25",
            "following_max_lateral: 0.02",
            "turn_max_angular: 0.35",
        ):
            self.assertIn(exact_value, config)

    def test_legacy_node_is_valid_and_uses_frozen_geometry(self):
        source = (
            PACKAGE_DIR
            / "scripts/corner_supervisor_legacy_v1_node.py"
        ).read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn(
            "ucar_corner_supervisor.legacy_v1_geometry",
            source,
        )
        self.assertIn('"source_commit": "19e0ce7"', source)

    def test_legacy_integration_remaps_move_base_velocity(self):
        root = ET.parse(
            str(
                PACKAGE_DIR
                / "launch/navigation_with_corner_supervisor_legacy_v1.launch"
            )
        ).getroot()
        xml = ET.tostring(root, encoding="unicode")
        self.assertIn('from="/cmd_vel"', xml)
        self.assertIn('to="/move_base/cmd_vel_raw"', xml)
        self.assertIn(
            "corner_supervisor_legacy_v1.launch",
            xml,
        )
        self.assertIn(
            'name="enable_lateral_mode_controller"',
            xml,
        )
        self.assertIn('value="false"', xml)

    def test_v2_alias_selects_current_integration_launch(self):
        root = ET.parse(
            str(
                PACKAGE_DIR
                / "launch/navigation_with_corner_supervisor_v2.launch"
            )
        ).getroot()
        xml = ET.tostring(root, encoding="unicode")
        self.assertIn(
            "navigation_with_corner_supervisor.launch",
            xml,
        )
        self.assertIn("supervisor_config", xml)

    def test_operator_guide_covers_complete_two_version_workflow(self):
        guide = (PACKAGE_DIR / "README_V1_V2.md").read_text(
            encoding="utf-8"
        )
        for required in (
            "navigation_with_corner_supervisor_legacy_v1.launch",
            "navigation_with_corner_supervisor_v2.launch",
            "source /home/ucar/ucar_ws/devel/setup.bash",
            "robot_base_bringup.launch",
            "/initialpose",
            "36 元素协方差",
            "/move_base/clear_costmaps",
            "/move_base_simple/goal",
            "/move_base/cancel",
            "/home/ucar/waypoints.xml",
            "corner_supervisor_legacy_v1.yaml",
            "corner_supervisor.yaml",
            "teb_corner_safe.yaml",
            "trajectory is not feasible",
            "new node registered with same name",
            "Ctrl-C",
            "catkin_make --pkg ucar_corner_supervisor",
        ):
            self.assertIn(required, guide)

    def test_legacy_geometry_detects_and_executes_first_corner(self):
        points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        corner = find_first_corner(
            points,
            search_distance=1.2,
            min_angle=math.radians(45.0),
        )
        self.assertIsNotNone(corner)
        supervisor = Supervisor(SupervisorConfig())
        result = supervisor.update(
            now=0.0,
            goal_active=True,
            raw_fresh=True,
            tf_valid=True,
            yaw=0.0,
            corner=corner,
            raw_command=(0.2, 0.0, 0.0),
        )
        self.assertEqual(result.state, "FOLLOWING")


if __name__ == "__main__":
    unittest.main()
