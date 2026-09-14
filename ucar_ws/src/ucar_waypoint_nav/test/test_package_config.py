#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_package_name_and_runtime_dependencies(self):
        root = ET.parse(str(ROOT / "package.xml")).getroot()
        self.assertEqual(root.findtext("name"), "ucar_waypoint_nav")
        dependencies = {node.text for node in root.findall("exec_depend")}
        self.assertTrue(
            {
                "rospy",
                "actionlib",
                "move_base_msgs",
                "tf2_ros",
                "std_srvs",
            }
            <= dependencies
        )

    def test_python_scripts_are_installed(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/derive_sparse_waypoints.py", cmake)
        self.assertIn("scripts/waypoint_route_manager.py", cmake)


if __name__ == "__main__":
    unittest.main()
