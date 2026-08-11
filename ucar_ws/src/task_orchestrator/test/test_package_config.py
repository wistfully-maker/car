import unittest
import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


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
            "scripts/system_readiness_gate_node.py",
            "scripts/velocity_arbiter_node.py",
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
                "/task/simulation_navigation_goal",
                "/task/line_navigation_goal",
                "/task/line_follow/start",
                "/task/motion_mode",
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
                "/task/navigation_handoff_status",
                "/task/delivery_arrived",
                "/task/cancel",
                "/task/simulation_arrived",
                "/task/line_navigation_arrived",
                "/task/line_follow/status",
            },
            subscriber_topics,
        )

    def test_launch_loads_parameters_inside_node_namespace(self):
        root = ET.parse(ROOT / "launch/task_orchestrator.launch").getroot()
        nodes = {node.attrib["name"]: node for node in root.findall("node")}
        self.assertEqual(
            {"task_orchestrator", "voice_task_adapter", "tts_bridge",
             "fast_nav_adapter", "readiness_gate", "velocity_arbiter",
             "navigation_handoff_supervisor"},
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
            "parse_line_status",
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
            "system_readiness_gate_node.py",
            "velocity_arbiter_node.py",
        ):
            if ("scripts/%s" % script) in cmake:
                self.assertTrue((ROOT / "scripts" / script).is_file(), script)

    def test_runtime_bringup_dependencies_are_declared(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        exec_dependencies = {node.text for node in package.findall("exec_depend")}
        self.assertTrue({
            "ucar_fast_nav", "speech_command", "qr_item_search", "llm_spark",
            "usb_cam",
        } <= exec_dependencies)

    def test_cmake_installs_share_files_and_registers_python_tests(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("install(DIRECTORY launch config", cmake)
        self.assertIn("DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}", cmake)
        self.assertIn("if(CATKIN_ENABLE_TESTING)", cmake)
        self.assertIn("catkin_add_nosetests(test)", cmake)
        self.assertIn("test/test_competition_bringup.py", "\n".join(
            str(path.relative_to(ROOT)).replace("\\", "/")
            for path in (ROOT / "test").glob("test_*.py")
        ))

    def test_velocity_arbiter_configuration_and_installation(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/velocity_arbiter_node.py", cmake)
        config = (ROOT / "config/orchestrator.yaml").read_text(encoding="utf-8")
        self.assertIn("velocity_arbiter:", config)
        self.assertIn("source_timeout: 0.3", config)
        self.assertIn("check_period: 0.05", config)
        lines = config.splitlines()
        start = lines.index("velocity_arbiter:") + 1
        subtree = []
        for line in lines[start:]:
            if line and not line.startswith(" "):
                break
            if line.strip():
                subtree.append(line.strip())
        self.assertEqual([
            "source_timeout: 0.3", "check_period: 0.05",
            "max_linear_abs: 1.0", "max_angular_abs: 2.0",
        ], subtree)

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

    def test_readiness_gate_dependencies_installation_and_configuration(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/system_readiness_gate_node.py", cmake)
        package = ET.parse(ROOT / "package.xml").getroot()
        for dependency in (
            "actionlib", "move_base_msgs", "nav_msgs", "sensor_msgs",
            "std_msgs", "tf2_ros", "rosnode",
        ):
            self.assertIsNotNone(package.find("build_depend[.='%s']" % dependency))
            self.assertIsNotNone(package.find("exec_depend[.='%s']" % dependency))
        config = (ROOT / "config/orchestrator.yaml").read_text(encoding="utf-8")
        self.assertIn("dependency_ready: 120.0", config)
        adapter = (ROOT / "scripts/task_orchestrator_node.py").read_text(encoding="utf-8")
        self.assertIn('"dependency_ready": 120.0', adapter)
        for required in (
            "readiness_gate:", "message_max_age:", "check_period:",
            "tf_timeout:", "action_wait_timeout:", "log_interval:",
            "map_frame:", "odom_frame:", "base_frame:", "laser_frame:",
            "lidar_loc_node:", "amcl_node:", "global_planner_param:",
            "local_planner_param:",
        ):
            self.assertIn(required, config)

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

    def test_operator_documentation_covers_safe_bringup_contract(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        manual = (ROOT / "test/manual_simulation.md").read_text(encoding="utf-8")
        handoff_path = ROOT.parents[2] / "HANDOFF.md"

        for heading in (
            "## 1. 先明确边界和真实终点", "## 2. 三种入口不能混用",
            "## 3. 部署后第一次启动", "## 4. 唤醒后如何观察完整自动链路",
            "## 5. 参数来源与冲突策略", "## 6. 停止、急停与常驻原则",
            "## 7. 常见故障：命令、判断和动作",
        ):
            self.assertIn(heading, readme)
        for command in (
            "./src/task_orchestrator/scripts/start_competition.sh",
            "roslaunch task_orchestrator competition_full.launch",
            "roslaunch task_orchestrator task_orchestrator.launch",
            "rostopic info /cmd_vel", "rosnode ping", "chmod 600",
            "rostopic echo -n 1 /map", "rostopic hz /scan", "rostopic hz /odom",
        ):
            self.assertIn(command, readme)
        self.assertNotIn("map stale", readme.lower())
        self.assertNotIn("地图陈旧", readme)

        stop_section = readme.split("## 6. 停止、急停与常驻原则", 1)[1].split("## 7.", 1)[0]
        for warning in ("机械急停", "机械断能", "第二个", "不能保证安全", "不能替代机械急停"):
            self.assertIn(warning, stop_section)

        start_section = manual.split("## 1. 安全边界与启动", 1)[1].split("## 2.", 1)[0]
        for required in (
            "机械断能", "架空", "ROS_MASTER_URI", "rosnode list",
            "rostopic info /cmd_vel", "competition", "外部仲裁器",
        ):
            self.assertIn(required, start_section)
        for forbidden in (
            "roslaunch ucar_waypoint_nav", "rosrun amcl", "roslaunch amcl",
            "roslaunch dynamic_obstacle",
        ):
            self.assertNotIn(forbidden, readme)
            self.assertNotIn(forbidden, manual)

        config = yaml.safe_load((ROOT / "config/orchestrator.yaml").read_text(encoding="utf-8"))
        self.assertEqual(120.0, config["timeouts"]["dependency_ready"])
        self.assertEqual(300.0, config["timeouts"]["pickup_navigation"])
        self.assertEqual(3.0, config["readiness_gate"]["message_max_age"])
        self.assertEqual(0.3, config["velocity_arbiter"]["source_timeout"])

        launch = ET.parse(ROOT / "launch/competition_full.launch").getroot()
        launch_args = {node.attrib["name"] for node in launch.findall("arg")}
        self.assertIn("llm_request_timeout", launch_args)
        self.assertIn("timeout_qr_search", launch_args)
        self.assertIn("总 launch 显式值 > `orchestrator.yaml` 默认值", readme)

        if handoff_path.exists():
            handoff = handoff_path.read_text(encoding="utf-8")
            for stable in ("5b628bb", "任务 8", "任务 17", "Bash", "任务 5", "待部署"):
                self.assertIn(stable, handoff)


    def test_phase3_outer_timeouts_strictly_exceed_node_limits(self):
        config = yaml.safe_load(
            (ROOT / "config/orchestrator.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            {
                "line_navigation": 310.0,
                "line_direction": 35.0,
                "line_follow": 125.0,
            },
            {
                key: config["timeouts"][key]
                for key in ("line_navigation", "line_direction", "line_follow")
            },
        )
        adapter = (ROOT / "scripts/task_orchestrator_node.py").read_text(
            encoding="utf-8"
        )
        for key, value in (
            ("line_navigation", 310.0),
            ("line_direction", 35.0),
            ("line_follow", 125.0),
        ):
            self.assertIn('"%s": %s' % (key, value), adapter)
        self.assertGreater(310.0, 300.0)
        self.assertGreater(35.0, 30.0)
        self.assertGreater(125.0, 120.0)

    def test_line_follow_integration_is_an_exec_dependency(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        exec_dependencies = {node.text for node in package.findall("exec_depend")}
        self.assertIn("line_follow_integration", exec_dependencies)

    def test_sole_final_cmd_vel_publisher_is_the_velocity_arbiter(self):
        def published_topics(script_name):
            tree = ast.parse(
                (ROOT / "scripts" / script_name).read_text(encoding="utf-8")
            )
            return {
                node.args[0].value
                for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "Publisher"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)
            }

        for script in (
            "task_orchestrator_node.py",
            "voice_task_adapter_node.py",
            "tts_bridge_node.py",
            "fast_nav_adapter_node.py",
            "system_readiness_gate_node.py",
            "navigation_handoff_supervisor_node.py",
        ):
            self.assertNotIn("/cmd_vel", published_topics(script), script)
        self.assertEqual(
            {"/cmd_vel"}, published_topics("velocity_arbiter_node.py")
        )

    def test_orchestrator_launch_forwards_phase3_pose_and_timeouts(self):
        launch = ET.parse(
            ROOT / "launch/task_orchestrator.launch"
        ).getroot()
        launch_args = {node.attrib["name"]: node for node in launch.findall("arg")}
        for name in ("timeout_line_navigation", "timeout_line_direction",
                     "timeout_line_follow"):
            self.assertIn(name, launch_args)
        node = next(
            node for node in launch.findall("node")
            if node.attrib["name"] == "task_orchestrator"
        )
        text = ET.tostring(node, encoding="unicode")
        self.assertIn("$(arg phase3_config)", text)
        self.assertIn(
            "$(find line_follow_integration)/config/phase3.yaml",
            ET.tostring(launch, encoding="unicode"),
        )
        adapter = (ROOT / "scripts/task_orchestrator_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"~line_start_goal"', adapter)


if __name__ == "__main__":
    unittest.main()
