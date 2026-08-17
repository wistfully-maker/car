"""Phase H: lock ucar_delivery ROS wiring and launch contracts.

Proves: exact protocol-v1 topic sets, no second camera, no camera settings
changes, a single final /cmd_vel owner, simulation-completion stub disabled
by default in production launches, launch-overridable parameters, exact
protocol topics and valid launch XML.
"""

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def string_literal(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if node.__class__.__name__ == "Str":
        return node.s
    return None


def collect_topics(source, publisher_name=None, subscriber_name=None):
    tree = ast.parse(source)
    publishers = set()
    subscribers = set()

    def collect_strings(node):
        found = []
        literal = string_literal(node)
        if literal is not None:
            found.append(literal)
        if isinstance(node, ast.Call):
            for arg in node.args:
                found.extend(collect_strings(arg))
        return found

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not isinstance(function, ast.Attribute):
            continue
        for arg in node.args:
            for topic in collect_strings(arg):
                if not topic.startswith("/"):
                    continue
                if publisher_name and function.attr == publisher_name:
                    publishers.add(topic)
                elif subscriber_name and function.attr == subscriber_name:
                    subscribers.add(topic)
    return publishers, subscribers


def parse_launch(relative):
    return ET.parse(ROOT / relative).getroot()


class PackageFileTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml",
            "CMakeLists.txt",
            "setup.py",
            "src/ucar_delivery/__init__.py",
            "src/ucar_delivery/protocol.py",
            "src/ucar_delivery/mission.py",
            "src/ucar_delivery/nav_supervisor.py",
            "src/ucar_delivery/sign_detector.py",
            "src/ucar_delivery/frame_detector.py",
            "src/ucar_delivery/lidar_safety.py",
            "src/ucar_delivery/parking_controller.py",
            "src/ucar_delivery/velocity_mux.py",
            "config/delivery.yaml",
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
            "scripts/delivery_mission_node.py",
            "scripts/sign_adapter_node.py",
            "scripts/frame_detector_node.py",
            "scripts/parking_controller_node.py",
            "scripts/delivery_velocity_mux_node.py",
            "scripts/mock_goal_publisher.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_launch_files_are_valid_xml(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            parse_launch(relative)

    def test_cmake_installs_only_scripts_that_exist(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for script in (
            "delivery_mission_node.py",
            "sign_adapter_node.py",
            "frame_detector_node.py",
            "parking_controller_node.py",
            "delivery_velocity_mux_node.py",
            "mock_goal_publisher.py",
        ):
            self.assertIn("scripts/%s" % script, cmake)
            self.assertTrue((ROOT / "scripts" / script).is_file(), script)

    def test_package_declares_dependencies(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        declared = {node.text for node in package.findall("depend")}
        for dependency in (
            "rospy", "std_msgs", "geometry_msgs", "sensor_msgs",
            "nav_msgs", "move_base_msgs", "actionlib", "tf2_ros",
        ):
            self.assertIn(dependency, declared)

    def test_package_has_no_orchestrator_runtime_dependency(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        exec_dependencies = {
            node.text for node in package.findall("exec_depend")
        }
        self.assertNotIn("task_orchestrator", exec_dependencies)


class CameraOwnershipTests(unittest.TestCase):
    def test_no_launch_starts_a_second_camera(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn('<node pkg="usb_cam"', text, relative)
            self.assertNotIn("camera_node", text, relative)

    def test_no_camera_settings_are_changed(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            for forbidden in (
                "width", "height", "fps", "pixel_format", "exposure",
                "white_balance", "camera_info_url", "framerate",
            ):
                self.assertNotIn(forbidden, text, "%s in %s" % (forbidden, relative))

    def test_delivery_nodes_subscribe_to_the_shared_camera_topic(self):
        for script in ("scripts/sign_adapter_node.py",
                       "scripts/frame_detector_node.py"):
            source = (ROOT / script).read_text(encoding="utf-8")
            self.assertIn("/usb_cam/image_raw", source)
            self.assertIn("rospy.Subscriber", source)


class CmdVelOwnershipTests(unittest.TestCase):
    def test_only_mux_node_publishes_final_cmd_vel(self):
        for script in (
            "scripts/delivery_mission_node.py",
            "scripts/sign_adapter_node.py",
            "scripts/frame_detector_node.py",
            "scripts/parking_controller_node.py",
            "scripts/mock_goal_publisher.py",
        ):
            source = (ROOT / script).read_text(encoding="utf-8")
            self.assertNotIn('Publisher(\n            "/cmd_vel"', source)
            self.assertNotIn('"/cmd_vel", Twist', source)
            self.assertNotIn('"/cmd_vel")', source)
        mux = (ROOT / "scripts/delivery_velocity_mux_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/cmd_vel"', mux)
        self.assertIn("rospy.Publisher", mux)

    def test_parking_commands_only_on_parking_topic(self):
        source = (ROOT / "scripts/parking_controller_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/cmd_vel/delivery_parking"', source)
        self.assertIn("cmd_vel_parking", source)

    def test_mission_node_publishes_only_manual_topic_for_motion(self):
        source = (ROOT / "scripts/delivery_mission_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/cmd_vel/delivery_manual"', source)
        self.assertIn("cmd_vel_manual", source)
        self.assertNotIn('"/cmd_vel/delivery_parking"', source)
        self.assertNotIn('"/cmd_vel/navigation"', source)
        self.assertNotIn('"/cmd_vel/qr"', source)


class ScopeIsolationTests(unittest.TestCase):
    """Task 1 scope regression: standalone ucar_delivery must not include the
    competition orchestrator full-flow launch (defect C baseline).

    The Git-level scope check (git diff --name-only 2fab15b -- task_orchestrator
    must be empty) is executed by the repair plan and recorded in handoff, not
    inside this suite, because unit tests here do not depend on Git.
    """

    def test_no_delivery_launch_includes_competition_full(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            root = parse_launch(relative)
            included = [
                item.attrib.get("file", "")
                for item in root.iter("include")
            ]
            self.assertNotIn(
                "competition_full.launch", " ".join(included), relative
            )


class ProtocolTopicTests(unittest.TestCase):
    def test_mission_node_declares_exact_topics(self):
        source = (ROOT / "scripts/delivery_mission_node.py").read_text(
            encoding="utf-8"
        )
        publishers, subscribers = collect_topics(source, "Publisher", "Subscriber")
        self.assertEqual(
            {
                "/ucar_delivery/motion_mode",
                "/task/delivery_status",
                "/task/delivery_arrived",
                "/task/simulation_arrived",
                "/task/delivery_sign_start",
                "/task/delivery_frame_start",
                "/task/delivery_parking_start",
                "/cmd_vel/delivery_manual",
            },
            publishers,
        )
        self.assertEqual(
            {
                "/task/delivery_navigation_goal",
                "/task/simulation_navigation_goal",
                "/task/delivery_sign_found",
                "/task/delivery_frame_observation",
                "/task/delivery_parking_progress",
                "/task/delivery_parking_result",
                "/task/cancel",
                "/odom",
                "/scan",
            },
            subscribers,
        )

    def test_protocol_v1_topic_names_exact(self):
        source = (ROOT / "scripts/delivery_mission_node.py").read_text(
            encoding="utf-8"
        )
        for topic in (
            "/task/delivery_navigation_goal",
            "/task/simulation_navigation_goal",
            "/task/delivery_arrived",
            "/task/simulation_arrived",
        ):
            self.assertIn(topic, source)

    def test_no_sim_completion_stub_remains_in_package(self):
        self.assertFalse((ROOT / "scripts/simulation_stub_node.py").is_file())
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
            "CMakeLists.txt",
            "config/delivery.yaml",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("simulation_stub", text, relative)
            self.assertNotIn("sim_trigger", text, relative)
            self.assertNotIn("sim_complete", text, relative)


class ParameterContractTests(unittest.TestCase):
    def test_critical_parameters_overrideable_from_launch(self):
        delivery = (ROOT / "launch/delivery.launch").read_text(encoding="utf-8")
        for parameter in (
            "move_base_action", "camera_topic", "scan_topic", "odom_topic",
            "cmd_vel_manual", "cmd_vel_parking", "motion_mode",
            "sign_backend",
        ):
            self.assertIn(parameter, delivery)
            self.assertIn("$(arg %s)" % parameter, delivery)

    def test_mission_configuration_from_yaml(self):
        config = yaml.safe_load(
            (ROOT / "config/delivery.yaml").read_text(encoding="utf-8")
        )
        self.assertIn("mission", config)
        self.assertIn("viewpoints", config)
        self.assertIn("physical", config["viewpoints"])
        self.assertIn("simulation", config["viewpoints"])
        self.assertEqual(
            config["mission"]["viewpoint_count"],
            len(config["viewpoints"]["physical"]),
        )
        self.assertEqual(
            config["mission"]["viewpoint_count"],
            len(config["viewpoints"]["simulation"]),
        )
        for phase in ("physical", "simulation"):
            for viewpoint in config["viewpoints"][phase]:
                for key in ("x", "y", "yaw"):
                    self.assertIn(key, viewpoint)

    def test_parking_and_lidar_configuration_present(self):
        config = yaml.safe_load(
            (ROOT / "config/delivery.yaml").read_text(encoding="utf-8")
        )
        self.assertFalse(config["lidar_safety"]["enabled"])
        self.assertIn("k_yaw", config["parking_controller"])
        self.assertIn("approach_speed", config["parking_controller"])
        self.assertIn("chassis_capability", config["parking_controller"])
        parking = config["parking_controller"]
        for key in (
            "frame_reacquire_timeout", "near_zone_front_y",
            "near_zone_loss_is_fatal",
        ):
            self.assertIn(key, parking)
        self.assertIn("sign_detector", config)
        self.assertIn("frame_detector", config)
        frame = config["frame_detector"]
        for key in (
            "roi_x_min", "roi_x_max", "roi_y_min", "roi_y_max",
            "acquire_confirm_frames", "lost_grace_frames",
            "min_geometry_confidence", "near_zone_front_y",
            "near_zone_loss_is_fatal",
        ):
            self.assertIn(key, frame)

    def test_staging_pose_configuration_present(self):
        config = yaml.safe_load(
            (ROOT / "config/delivery.yaml").read_text(encoding="utf-8")
        )
        staging = config["staging_pose"]
        self.assertTrue(staging["enabled"])
        for key in (
            "camera_lidar_yaw_offset_deg", "bearing_margin_deg",
            "min_valid_points", "min_inliers", "max_fit_residual",
            "max_range_spread", "staging_distance", "min_staging_travel",
            "max_staging_travel", "estimation_confirmations",
            "estimation_consistency_xy", "estimation_consistency_yaw_deg",
            "staging_estimation_max_retries", "staging_navigation_max_retries",
            "tf_timeout", "max_scan_age",
        ):
            self.assertIn(key, staging)

    def test_staging_navigation_settle_gate_configuration_present(self):
        config = yaml.safe_load(
            (ROOT / "config/delivery.yaml").read_text(encoding="utf-8")
        )
        supervisor = config["nav_supervisor"]
        for key in (
            "settle_timeout", "odom_timeout",
            "settle_velocity_threshold", "settle_angular_threshold",
            "settle_stable_duration",
        ):
            self.assertIn(key, supervisor)
        self.assertIn("camera_center_x", config["sign_detector"])
        self.assertIn("camera_focal_px", config["sign_detector"])
        self.assertIn("staging_estimate", config["timeouts"])
        self.assertIn("staging_navigation", config["timeouts"])

    def test_vehicle_backend_config_matches_deployed_contract(self):
        config = yaml.safe_load(
            (ROOT / "config/delivery.yaml").read_text(encoding="utf-8")
        )
        sign = config["sign_detector"]
        self.assertEqual("yolo_biao.infer", sign["vehicle_backend_module"])
        self.assertEqual("ocr.ocr_infer", sign["vehicle_ocr_module"])
        self.assertEqual(
            "电子产品生产车间",
            sign["target_aliases"]["电子产品加工车间"],
        )

    def test_readme_parameter_table_covers_field_tuning_keys(self):
        # 现场调参项必须出现在 README 参数表（单位/默认值/安全方向），
        # 防止文档与配置漂移。
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        table = readme.split("## 9. 全部 launch/YAML 参数")[1]
        table = table.split("## 10.")[0]
        for key in (
            "staging_pose.camera_lidar_yaw_offset_deg",
            "staging_pose.staging_distance",
            "staging_pose.min_staging_travel",
            "staging_pose.max_staging_travel",
            "staging_pose.estimation_confirmations",
            "staging_pose.estimation_consistency_xy",
            "staging_pose.estimation_consistency_yaw_deg",
            "staging_pose.staging_estimation_max_retries",
            "staging_pose.staging_navigation_max_retries",
            "staging_pose.max_scan_age",
            "staging_pose.tf_timeout",
            "nav_supervisor.settle_timeout",
            "nav_supervisor.odom_timeout",
            "nav_supervisor.settle_velocity_threshold",
            "nav_supervisor.settle_angular_threshold",
            "nav_supervisor.settle_stable_duration",
            "sign_detector.camera_center_x",
            "sign_detector.camera_focal_px",
            "frame_detector.roi_x_min",
            "frame_detector.max_line_thickness",
            "frame_detector.min_geometry_confidence",
            "frame_detector.morph_iterations",
            "frame_detector.use_hsv",
            "frame_detector.acquire_confirm_frames",
            "frame_detector.lost_grace_frames",
            "frame_detector.near_zone_front_y",
            "parking_controller.frame_reacquire_timeout",
            "parking_controller.near_zone_front_y",
            "parking_controller.near_zone_loss_is_fatal",
        ):
            self.assertIn(key, table, key)
        # timeouts 是分组描述行（无独立行）：默认值与单位需出现在描述中
        self.assertIn("staging_estimate 30", table)
        self.assertIn("staging_navigation 120", table)

    def test_topics_section_matches_node_defaults(self):
        config = yaml.safe_load(
            (ROOT / "config/delivery.yaml").read_text(encoding="utf-8")
        )
        topics = config["topics"]
        self.assertEqual("/task/delivery_navigation_goal", topics["delivery_goal"])
        self.assertEqual("/task/simulation_navigation_goal", topics["simulation_goal"])
        self.assertEqual("/task/delivery_arrived", topics["delivery_arrived"])
        self.assertEqual("/task/simulation_arrived", topics["simulation_arrived"])
        self.assertEqual("/cmd_vel/delivery_manual", topics["cmd_vel_manual"])
        self.assertEqual("/cmd_vel/delivery_parking", topics["cmd_vel_parking"])
        self.assertEqual("/cmd_vel", topics["cmd_vel"])
        self.assertEqual(
            "/cmd_vel/delivery_navigation", topics["cmd_vel_navigation"]
        )


class NavigationIntegrationTests(unittest.TestCase):
    SRC = ROOT.parents[0]

    def test_navigation_stack_exposes_cmd_vel_topic_remap(self):
        nav_launch = self.SRC / "ucar_nav" / "launch" / "navigation_stack.launch"
        text = nav_launch.read_text(encoding="utf-8")
        self.assertIn('cmd_vel_topic" default="/cmd_vel"', text)
        self.assertIn('<remap from="/cmd_vel" to="$(arg cmd_vel_topic)"/>', text)

    def test_ucar_nav_has_no_other_repair_changes(self):
        # 静态锁定：除 cmd_vel_topic arg/remap 外，navigation_stack.launch
        # 不允许出现其他修复性改动；Git 级检查由计划执行并记录在 handoff。
        nav = self.SRC / "ucar_nav" / "launch" / "navigation_stack.launch"
        text = nav.read_text(encoding="utf-8")
        self.assertNotIn("cmd_vel_topic", text.replace(
            '<arg name="cmd_vel_topic" default="/cmd_vel"/>', ""
        ).replace('<remap from="/cmd_vel" to="$(arg cmd_vel_topic)"/>', ""))


class StandaloneLaunchTests(unittest.TestCase):
    """Task 5: the standalone root launch owns exactly one navigation stack,
    exactly one mux and zero hardware/competition modules."""

    def text(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8")

    def test_no_launch_references_competition_or_other_modules(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            text = self.text(relative)
            for forbidden in (
                "competition_full.launch",
                "qr_item_search",
                "llm_spark",
                "speech_command",
                "pickup_navigation.launch",
                "gazebo",
            ):
                self.assertNotIn(forbidden, text, "%s contains %s" % (relative, forbidden))

    def test_standalone_includes_one_navigation_stack_and_delivery(self):
        root = parse_launch("launch/delivery_standalone.launch")
        included = [
            item.attrib.get("file", "")
            for item in root.iter("include")
        ]
        self.assertEqual(
            1, len([f for f in included if f.endswith("navigation_stack.launch")])
        )
        self.assertEqual(
            1, len([f for f in included if f.endswith("delivery.launch")])
        )

    def test_move_base_output_isolated_via_cmd_vel_topic(self):
        text = self.text("launch/delivery_standalone.launch")
        self.assertIn(
            'cmd_vel_topic" default="/cmd_vel/delivery_navigation"', text
        )
        self.assertIn('name="cmd_vel_topic" value="$(arg cmd_vel_topic)"', text)

    def test_delivery_launch_starts_only_delivery_components(self):
        root = parse_launch("launch/delivery.launch")
        types = [node.attrib["type"] for node in root.findall("node")]
        self.assertEqual(
            [
                "delivery_mission_node.py",
                "sign_adapter_node.py",
                "frame_detector_node.py",
                "parking_controller_node.py",
            ],
            types,
        )
        # 每个被引用的 arg 都已在 delivery.launch 中声明
        declared = {arg.attrib["name"] for arg in root.findall("arg")}
        referenced = set()
        for node in root.findall("node"):
            for param in node.findall("param"):
                value = param.attrib.get("value", "")
                if value.startswith("$(arg "):
                    referenced.add(value[len("$(arg "):-1])
        self.assertLessEqual(referenced, declared)

    def test_standalone_starts_exactly_one_mux(self):
        root = parse_launch("launch/delivery_standalone.launch")
        mux = [
            node for node in root.findall("node")
            if "velocity_mux" in node.attrib.get("type", "")
        ]
        self.assertEqual(1, len(mux))
        self.assertIn(
            "delivery_velocity_mux_node.py", mux[0].attrib["type"]
        )

    def test_no_hardware_or_competition_node_started(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            root = parse_launch(relative)
            for node in root.findall("node"):
                pkg = node.attrib.get("pkg", "")
                self.assertIn(pkg, ("ucar_delivery", "ucar_nav"), relative)
                self.assertNotIn("usb_cam", pkg, relative)
                self.assertNotIn("lidar", pkg, relative)
                self.assertNotIn("base", pkg, relative)

    def test_no_camera_settings_are_touched(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
        ):
            text = self.text(relative)
            for forbidden in (
                "width", "height", "fps", "pixel_format", "exposure",
                "white_balance", "camera_info_url", "framerate",
            ):
                self.assertNotIn(forbidden, text, "%s in %s" % (forbidden, relative))

    def test_no_sim_stub_or_orchestrator_reference(self):
        for relative in (
            "launch/delivery.launch",
            "launch/delivery_standalone.launch",
            "CMakeLists.txt",
        ):
            text = self.text(relative)
            self.assertNotIn("simulation_stub", text, relative)
            self.assertNotIn("task_orchestrator", text, relative)


if __name__ == "__main__":
    unittest.main()
