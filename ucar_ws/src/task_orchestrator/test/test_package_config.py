import unittest
import ast
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def string_literal(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if type(node).__name__ == "Str":
        return node.s
    return None


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml",
            "CMakeLists.txt",
            "setup.py",
            "README.md",
            "src/task_orchestrator/__init__.py",
            "config/orchestrator.yaml",
            "launch/task_orchestrator.launch",
            "scripts/task_orchestrator_node.py",
            "scripts/voice_task_adapter_node.py",
            "scripts/tts_bridge_node.py",
            "scripts/fast_nav_adapter_node.py",
            "test/manual_simulation.md",
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
            topic = string_literal(node.args[0])
            if topic is None:
                continue
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
            topic = string_literal(node.args[0])
            if topic is None:
                continue
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
            topic = string_literal(node.args[0])
            if topic is None:
                continue
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
            "fast_nav_adapter_node.py",
        ):
            if ("scripts/%s" % script) in cmake:
                self.assertTrue((ROOT / "scripts" / script).is_file(), script)

    def test_fast_nav_adapter_declares_ros_contract(self):
        source = (ROOT / "scripts/fast_nav_adapter_node.py").read_text(
            encoding="utf-8"
        )
        tree = ast.parse(source)
        publishers = set()
        subscribers = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            function = node.func
            if not isinstance(function, ast.Attribute):
                continue
            topic = string_literal(node.args[0])
            if function.attr == "Publisher" and topic:
                publishers.add(topic)
            elif function.attr == "Subscriber" and topic:
                subscribers.add(topic)
        self.assertEqual({"/task/pickup_arrived"}, publishers)
        self.assertEqual(
            {"/task/pickup_navigation_goal", "/task/cancel", "/odom"},
            subscribers,
        )
        for required in (
            "actionlib.SimpleActionClient",
            '"/move_base"',
            "MoveBaseAction",
            "MoveBaseGoal",
            "FastNavSession",
            "StopDetector",
            "validate_waypoint",
            "math.hypot",
            "quaternion_from_euler",
            "threading.RLock",
            "rospy.on_shutdown",
        ):
            self.assertIn(required, source)

    def test_fast_nav_dependencies_and_installation_are_declared(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/fast_nav_adapter_node.py", cmake)
        root = ET.parse(ROOT / "package.xml").getroot()
        for dependency in (
            "actionlib",
            "actionlib_msgs",
            "move_base_msgs",
            "nav_msgs",
            "geometry_msgs",
        ):
            self.assertIsNotNone(root.find("build_depend[.='%s']" % dependency))
            self.assertIsNotNone(root.find("exec_depend[.='%s']" % dependency))

    def test_fast_nav_configuration_has_explicit_defaults(self):
        config = (ROOT / "config/orchestrator.yaml").read_text(encoding="utf-8")
        for required in (
            "fast_nav_adapter:",
            "action_timeout: 300.0",
            "settle_time: 0.5",
            "linear_stop_threshold: 0.03",
            "angular_stop_threshold: 0.05",
        ):
            self.assertIn(required, config)
        self.assertNotIn("pickup_goal:", config)
        self.assertNotIn("replace_with_deployed_map_sha256", config)

    def test_manual_simulation_uses_current_protocol(self):
        manual = (ROOT / "test/manual_simulation.md").read_text(
            encoding="utf-8"
        )
        normalized = manual.replace('\\"', '"')
        for required in (
            "physical_target_category",
            "simulation_target_category",
            '"status": "ready"',
            '"status": "arrived"',
            '"order": 1',
            '"item_name": "手机"',
            '"physical":',
            '"simulation":',
            '"target_workshop"',
            '"selected_item"',
            '"reason": "operator_cancel"',
        ):
            self.assertIn(required, normalized)
        for obsolete in (
            '"physical_category"',
            '"simulation_category"',
            '"sequence"',
            '"classification"',
            '"destination"',
            '"item":',
        ):
            self.assertNotIn(obsolete, normalized)


if __name__ == "__main__":
    unittest.main()
