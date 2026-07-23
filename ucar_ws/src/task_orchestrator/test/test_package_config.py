import unittest
import ast
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml",
            "CMakeLists.txt",
            "setup.py",
            "src/task_orchestrator/__init__.py",
            "config/orchestrator.yaml",
            "launch/task_orchestrator.launch",
            "scripts/task_orchestrator_node.py",
            "scripts/voice_task_adapter_node.py",
            "scripts/tts_bridge_node.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_ros_adapter_declares_exact_string_topics(self):
        source = (ROOT / "scripts/task_orchestrator_node.py").read_text(
            encoding="utf-8"
        )
        tree = ast.parse(source)
        publisher_topics = set()
        subscriber_topics = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if not isinstance(function, ast.Attribute) or not node.args:
                continue
            if not isinstance(node.args[0], ast.Constant):
                continue
            topic = node.args[0].value
            if function.attr == "Publisher":
                publisher_topics.add(topic)
                self.assertEqual("String", node.args[1].id)
            elif function.attr == "Subscriber":
                subscriber_topics.add(topic)
                self.assertEqual("String", node.args[1].id)

        self.assertEqual(
            {
                "/task/status",
                "/task/pickup_navigation_goal",
                "/qr_item_search/start",
                "/qr_item_search/stop",
                "/llm/classify/request",
                "/voice/speak",
                "/task/delivery_navigation_goal",
            },
            publisher_topics,
        )
        self.assertEqual(
            {
                "/voice/task_request",
                "/task/dependencies_ready",
                "/task/pickup_arrived",
                "/qr_item_search/result",
                "/llm/classify/result",
                "/voice/speak_done",
                "/task/delivery_arrived",
                "/task/cancel",
            },
            subscriber_topics,
        )

    def test_launch_loads_parameters_inside_node_namespace(self):
        root = ET.parse(ROOT / "launch/task_orchestrator.launch").getroot()
        nodes = {node.attrib["name"]: node for node in root.findall("node")}
        self.assertEqual(
            {"task_orchestrator", "voice_task_adapter", "tts_bridge"},
            set(nodes),
        )
        for node in nodes.values():
            rosparam = node.find("rosparam")
            self.assertIsNotNone(rosparam)
            self.assertEqual("load", rosparam.attrib["command"])

    def test_ros_adapter_uses_protocol_layer_and_public_core_api(self):
        source = (ROOT / "scripts/task_orchestrator_node.py").read_text(
            encoding="utf-8"
        )
        for parser in (
            "parse_task_request",
            "parse_arrival",
            "parse_qr_result",
            "parse_llm_result",
            "parse_speech_done",
            "parse_cancel",
        ):
            self.assertIn(parser, source)
        self.assertNotIn("orch._transition", source)
        self.assertNotIn("orch._publish_status", source)
        self.assertIn("expected_state", source)

        tree = ast.parse(source)
        ready_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "on_dependencies_ready"
        ]
        self.assertEqual(1, len(ready_calls))
        self.assertEqual([], ready_calls[0].args)

    def test_voice_adapter_declares_exact_string_topics(self):
        source = (ROOT / "scripts/voice_task_adapter_node.py").read_text(
            encoding="utf-8"
        )
        tree = ast.parse(source)
        publisher_topics = set()
        subscriber_topics = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if not isinstance(function, ast.Attribute) or not node.args:
                continue
            if not isinstance(node.args[0], ast.Constant):
                continue
            topic = node.args[0].value
            if function.attr == "Publisher":
                publisher_topics.add(topic)
                self.assertEqual("String", node.args[1].id)
            elif function.attr == "Subscriber":
                subscriber_topics.add(topic)
                self.assertEqual("String", node.args[1].id)

        self.assertIn("/voice/task_request", publisher_topics)
        self.assertIn("/question", subscriber_topics)

    def test_tts_bridge_declares_exact_string_topics_and_parser(self):
        source = (ROOT / "scripts/tts_bridge_node.py").read_text(
            encoding="utf-8"
        )
        tree = ast.parse(source)
        publisher_topics = set()
        subscriber_topics = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if not isinstance(function, ast.Attribute) or not node.args:
                continue
            if not isinstance(node.args[0], ast.Constant):
                continue
            topic = node.args[0].value
            if function.attr == "Publisher":
                publisher_topics.add(topic)
                self.assertEqual("String", node.args[1].id)
            elif function.attr == "Subscriber":
                subscriber_topics.add(topic)
                self.assertEqual("String", node.args[1].id)
        self.assertEqual({"/voice/speak_done"}, publisher_topics)
        self.assertEqual({"/voice/speak"}, subscriber_topics)
        self.assertIn("parse_speak_request", source)

    def test_cmake_installs_only_scripts_that_exist(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for script in (
            "task_orchestrator_node.py",
            "voice_task_adapter_node.py",
            "tts_bridge_node.py",
        ):
            if ("scripts/%s" % script) in cmake:
                self.assertTrue((ROOT / "scripts" / script).is_file(), script)


if __name__ == "__main__":
    unittest.main()
