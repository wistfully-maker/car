import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parent.parent


class PackageConfigTest(unittest.TestCase):
    def test_launch_file_has_expected_nodes_and_parameters(self):
        root = ET.parse(
            str(PACKAGE_ROOT / "launch" / "qr_item_search.launch")
        ).getroot()
        arguments = {
            element.get("name"): element.get("default")
            for element in root.findall("arg")
        }
        self.assertEqual("/usb_cam/image_raw", arguments["image_topic"])
        self.assertEqual(
            "[0.0, 1.5708, 3.1416]",
            arguments["wall_yaw_offsets"],
        )

        nodes = {
            element.get("type"): element
            for element in root.findall("node")
        }
        scanner = nodes["qr_scanner_node.py"]
        controller = nodes["item_search_controller_node.py"]
        self.assertEqual("qr_item_search", scanner.get("pkg"))
        self.assertEqual("qr_item_search", controller.get("pkg"))

        scanner_parameters = {
            element.get("name"): element.get("value")
            for element in scanner.findall("param")
        }
        self.assertEqual("$(arg image_topic)", scanner_parameters["image_topic"])
        self.assertEqual("2", scanner_parameters["required_frames"])
        self.assertEqual("1.0", scanner_parameters["connect_timeout"])
        self.assertEqual("2.0", scanner_parameters["read_timeout"])
        self.assertEqual("2", scanner_parameters["http_retries"])

        controller_parameters = {
            element.get("name"): element.get("value")
            for element in controller.findall("param")
        }
        expected = {
            "yaw_kp": "1.2",
            "max_angular_speed": "0.30",
            "min_angular_speed": "0.11",
            "yaw_tolerance": "0.035",
            "settle_seconds": "0.8",
            "scan_timeout": "4.0",
            "turn_timeout": "8.0",
        }
        self.assertEqual(expected, controller_parameters)
        wall_offsets = controller.find("rosparam")
        self.assertEqual("wall_yaw_offsets", wall_offsets.get("param"))
        self.assertEqual("true", wall_offsets.get("subst_value"))
        self.assertEqual("$(arg wall_yaw_offsets)", wall_offsets.text.strip())

    def test_package_declares_runtime_dependencies(self):
        root = ET.parse(str(PACKAGE_ROOT / "package.xml")).getroot()
        dependencies = {
            element.text.strip()
            for tag in ("depend", "exec_depend")
            for element in root.findall(tag)
        }
        self.assertTrue(
            {"tf", "python3-pyzbar", "python3-requests"}.issubset(
                dependencies
            )
        )

    def test_maintainer_email_has_a_qualified_domain(self):
        root = ET.parse(str(PACKAGE_ROOT / "package.xml")).getroot()
        email = root.find("maintainer").get("email")
        local, separator, domain = email.partition("@")

        self.assertTrue(local)
        self.assertEqual("@", separator)
        self.assertIn(".", domain)

    def test_cmake_installs_scripts_and_launch_with_one_test_block(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/qr_scanner_node.py", cmake)
        self.assertIn("scripts/item_search_controller_node.py", cmake)
        self.assertIn("install(DIRECTORY launch/", cmake)
        self.assertIn("${CATKIN_PACKAGE_SHARE_DESTINATION}/launch", cmake)
        self.assertEqual(1, cmake.count("if(CATKIN_ENABLE_TESTING)"))

    def test_reset_ros_interfaces_use_latched_int32(self):
        scanner_tree = ast.parse(
            (PACKAGE_ROOT / "scripts" / "qr_scanner_node.py").read_text(
                encoding="utf-8"
            )
        )
        controller_tree = ast.parse(
            (
                PACKAGE_ROOT / "scripts" / "item_search_controller_node.py"
            ).read_text(encoding="utf-8")
        )
        scanner_call = self._topic_call(
            scanner_tree,
            "Subscriber",
            "/qr_item_search/reset",
        )
        controller_call = self._topic_call(
            controller_tree,
            "Publisher",
            "/qr_item_search/reset",
        )
        self.assertEqual("Int32", scanner_call.args[1].id)
        self.assertEqual("Int32", controller_call.args[1].id)
        latch = next(
            keyword.value
            for keyword in controller_call.keywords
            if keyword.arg == "latch"
        )
        self.assertIs(True, latch.value)

    @staticmethod
    def _topic_call(tree, method, topic):
        matches = []
        for node in ast.walk(tree):
            if not (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == method
                and len(node.args) >= 2
            ):
                continue
            topic_node = node.args[0]
            if isinstance(topic_node, (ast.Str, ast.Constant)):
                value = (
                    topic_node.s
                    if isinstance(topic_node, ast.Str)
                    else topic_node.value
                )
                if value == topic:
                    matches.append(node)
        if len(matches) != 1:
            raise AssertionError(
                "expected one {} for {}, found {}".format(
                    method,
                    topic,
                    len(matches),
                )
            )
        return matches[0]


if __name__ == "__main__":
    unittest.main()
