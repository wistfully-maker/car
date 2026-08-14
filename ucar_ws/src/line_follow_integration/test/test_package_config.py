import unittest
import subprocess
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

    def test_rosrun_entrypoints_are_executable(self):
        for script in (ROOT / "scripts").glob("*.py"):
            with self.subTest(script=script.name):
                mode = subprocess.check_output(
                    ["git", "-C", str(ROOT), "ls-files", "-s", "--",
                     str(script)],
                    text=True,
                ).split()[0]
                self.assertEqual(
                    "100755", mode,
                    "%s must be executable in the Git archive" % script.name,
                )

    def test_phase3_defaults_are_exact(self):
        config = yaml.safe_load(
            (ROOT / "config/phase3.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual({
            "frame_id": "map",
            "x": 0.4167081260031493,
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


class OperatorDocumentationTests(unittest.TestCase):
    def test_operator_visible_contracts_are_documented(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        for required in (
            "/task/line_navigation_goal",
            "/task/line_navigation_arrived",
            "/task/line_follow/start",
            "/task/line_follow/status",
            "/cmd_vel/line_follow",
            "LINE_FOLLOW",
            "red_light",
            "left_turn",
            "right_turn",
            "straight",
            "任务完成",
            "/home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml",
            "D:\\program_sec\\智能车\\.worktrees\\phase3-line-follow-integration\\ucar_ws\\src\\line_follow_integration\\config\\phase3.yaml",
        ):
            self.assertIn(required, readme)

    def test_readme_states_operation_truths(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        for statement in (
            "重启根 launch",
            "只有最终停车线检测才是成功",
            "30 秒无方向结果时选择 straight",
            "1020x720",
            "YOLO 使用原始图像",
            "640x480",
            "start_all_yolo.launch",
        ):
            self.assertIn(statement, readme)


if __name__ == "__main__":
    unittest.main()
