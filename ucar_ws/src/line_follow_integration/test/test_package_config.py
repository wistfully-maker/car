import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml", "CMakeLists.txt", "setup.py",
            "config/phase3.yaml", "launch/phase3.launch",
            "src/line_follow_integration/__init__.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_phase3_defaults_are_exact(self):
        config = yaml.safe_load(
            (ROOT / "config/phase3.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual({
            "frame_id": "map",
            "x": 0.5167081260031493,
            "y": -3.125171302690142,
            "qz": -0.7044294777312331,
            "qw": 0.7097739857893512,
        }, config["line_start_goal"])
        self.assertEqual("/usb_cam/image_raw", config["camera"]["input_topic"])
        self.assertEqual("/line_follow/image_raw", config["camera"]["line_topic"])
        self.assertEqual([640, 480, 15.0, "center_4_3"], [
            config["camera"]["output_width"],
            config["camera"]["output_height"],
            config["camera"]["output_fps"],
            config["camera"]["crop_mode"],
        ])
        self.assertEqual({
            "navigation": 300.0,
            "image_ready": 5.0,
            "image_max_age": 0.5,
            "image_recovery_grace": 3.0,
            "direction": 30.0,
            "line_follow": 120.0,
        }, config["timeouts"])

    def test_launch_does_not_own_shared_hardware(self):
        root = ET.parse(ROOT / "launch/phase3.launch").getroot()
        text = ET.tostring(root, encoding="unicode")
        for forbidden in ("usb_cam_node", "base_driver", "ydlidar", "map_server", "amcl"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
