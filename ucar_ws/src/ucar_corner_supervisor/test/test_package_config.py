#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative_path in ("package.xml", "CMakeLists.txt"):
            self.assertTrue(
                (PACKAGE_DIR / relative_path).is_file(),
                "{} is missing".format(relative_path),
            )

    def test_package_manifest_has_runtime_dependencies(self):
        root = ET.parse(str(PACKAGE_DIR / "package.xml")).getroot()
        dependencies = {
            element.text for element in root.findall("depend") if element.text
        }
        self.assertEqual(root.findtext("name"), "ucar_corner_supervisor")
        self.assertTrue(
            {
                "geometry_msgs",
                "nav_msgs",
                "rospy",
                "tf2_ros",
                "actionlib_msgs",
                "std_msgs",
            }.issubset(dependencies)
        )

    def test_python_scripts_are_installed(self):
        cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("catkin_install_python", cmake)
        self.assertIn("scripts/corner_supervisor_node.py", cmake)


if __name__ == "__main__":
    unittest.main()
