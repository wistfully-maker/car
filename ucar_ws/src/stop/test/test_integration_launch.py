import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

STOP_ROOT = Path(__file__).resolve().parents[1]
ORCH_ROOT = STOP_ROOT.parent / "task_orchestrator"

MISSION_INTEGRATION = STOP_ROOT / "launch" / "mission_integration.launch"
COMPETITION = ORCH_ROOT / "launch" / "competition_full.launch"
ORCHESTRATOR = ORCH_ROOT / "launch" / "task_orchestrator.launch"
LEGACY_WRAPPER = ORCH_ROOT / "launch" / "legacy_navigation_include.launch"


class MissionIntegrationLaunchTests(unittest.TestCase):
    def test_python_nodes_use_catkin_wrappers(self):
        cmake = (STOP_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("catkin_install_python(PROGRAMS", cmake)

    def test_mission_integration_parses_as_valid_xml(self):
        ET.parse(MISSION_INTEGRATION)

    def test_never_starts_shared_hardware_or_speech(self):
        text = MISSION_INTEGRATION.read_text(encoding="utf-8")
        for banned in ("ucar_controller", "ydlidar", "usb_cam", "speech_command"):
            self.assertNotIn(banned, text)
        root = ET.parse(MISSION_INTEGRATION).getroot()
        pkgs = {node.attrib.get("pkg") for node in root.iter("node")}
        self.assertNotIn("usb_cam", pkgs)

    def test_single_owner_for_map_localization_and_move_base(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        nodes = list(root.iter("node"))
        self.assertEqual(
            1, len([n for n in nodes if n.attrib.get("pkg") == "map_server"])
        )
        self.assertEqual(
            1, len([n for n in nodes if n.attrib.get("pkg") == "move_base"])
        )
        amcl_direct = [n for n in nodes if n.attrib.get("pkg") == "amcl"]
        amcl_includes = [
            inc for inc in root.iter("include")
            if "amcl" in inc.attrib.get("file", "")
        ]
        self.assertEqual(1, len(amcl_direct) + len(amcl_includes))

    def test_stop_move_base_publishes_to_isolated_topic(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        move_base = next(
            n for n in root.iter("node") if n.attrib.get("pkg") == "move_base"
        )
        remaps = {
            item.attrib["from"]: item.attrib["to"]
            for item in move_base.findall("remap")
        }
        self.assertEqual("/cmd_vel/stop_navigation", remaps["cmd_vel"])
        self.assertFalse(any(
            item.attrib.get("name") == "cmd_vel_topic"
            for item in move_base.findall("param")
        ))

    def test_no_node_except_competition_arbiter_publishes_final_cmd_vel(self):
        text = MISSION_INTEGRATION.read_text(encoding="utf-8")
        self.assertNotIn('value="/cmd_vel"', text)
        self.assertNotIn('from="/cmd_vel"', text)
        mission = (STOP_ROOT / "scripts" / "mission_orchestrator.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("'/cmd_vel/stop_manual'", mission)
        self.assertNotIn("Publisher('/cmd_vel'", mission)
        mux = (STOP_ROOT / "scripts" / "stop_velocity_mux_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/cmd_vel/stop"', mux)
        self.assertNotIn('"/cmd_vel"', mux.replace('"/cmd_vel/stop"', ""))

    def test_vehicle_parameter_defaults_are_exposed_unchanged(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        args = {
            a.attrib.get("name"): a.attrib.get("default")
            for a in root.findall("arg")
        }
        for name, default in (
            ("max_rotations", "6"),
            ("target_distance", "0.20"),
            ("x_align_tolerance", "0.50"),
            ("y_align_tolerance", "12"),
            ("auto_start_delay", "3.0"),
            ("phase2_ack_timeout", "60.0"),
        ):
            self.assertEqual(default, args[name], name)
        for name in ("initial_pose_x", "initial_pose_y", "initial_pose_yaw"):
            self.assertIn(name, args)

    def test_integration_launch_starts_mission_mux_and_adapter(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        types = {node.attrib.get("type") for node in root.iter("node")}
        for executable in (
            "mission_orchestrator.py",
            "stop_velocity_mux_node.py",
            "stop_protocol_adapter_node.py",
            "ocr_native_node.py",
        ):
            self.assertIn(executable, types)


class RootCompositionTests(unittest.TestCase):
    def test_navigation_stacks_are_only_supervisor_owned(self):
        root = ET.parse(COMPETITION).getroot()
        includes = [inc.attrib.get("file", "") for inc in root.iter("include")]
        self.assertFalse(any("mission_integration" in f for f in includes))
        self.assertFalse(any("legacy_navigation_include" in f for f in includes))

    def test_root_owns_shared_hardware_in_handoff_mode(self):
        root = ET.parse(COMPETITION).getroot()
        hardware = [
            g for g in root.findall("group")
            if g.attrib.get("if", "").startswith("$(eval")
            and ("start_base" in g.attrib.get("if", "")
                 or "start_lidar" in g.attrib.get("if", ""))
        ]
        self.assertEqual(2, len(hardware))
        files = {
            inc.attrib["file"]
            for group in hardware
            for inc in group.findall("include")
        }
        self.assertEqual(
            {
                "$(find ucar_controller)/launch/base_driver.launch",
                "$(find ydlidar)/launch/ydlidar.launch",
            },
            files,
        )

    def test_supervisor_owns_both_navigation_launches(self):
        root = ET.parse(ORCHESTRATOR).getroot()
        nodes = {node.attrib["name"]: node for node in root.findall("node")}
        self.assertIn("navigation_handoff_supervisor", nodes)
        node = nodes["navigation_handoff_supervisor"]
        self.assertEqual(
            "navigation_handoff_supervisor_node.py", node.attrib["type"]
        )
        params = {
            p.attrib["name"]: p.attrib["value"]
            for p in node.findall("param")
        }
        self.assertEqual(
            "$(arg legacy_nav_launch)",
            params["navigation_handoff_supervisor/legacy_nav_launch"],
        )
        self.assertEqual(
            "$(arg stop_integration_launch)",
            params["navigation_handoff_supervisor/stop_integration_launch"],
        )
        args = {a.attrib["name"]: a.attrib.get("default")
                for a in root.findall("arg")}
        self.assertIn("$(find stop)/launch/mission_integration.launch",
                      args["stop_integration_launch"])
        self.assertIn("legacy_navigation_include.launch",
                      args["legacy_nav_launch"])

    def test_legacy_wrapper_passes_isolated_velocity_and_no_hardware(self):
        root = ET.parse(LEGACY_WRAPPER).getroot()
        include = root.find("include")
        self.assertIn("pickup_navigation.launch", include.attrib["file"])
        args = {a.attrib["name"]: a.attrib["value"]
                for a in include.findall("arg")}
        self.assertEqual("$(arg start_base)", args["start_base"])
        self.assertEqual("$(arg start_lidar)", args["start_lidar"])
        self.assertEqual("/cmd_vel/navigation", args["cmd_vel_topic"])
        defaults = {a.attrib["name"]: a.attrib.get("default")
                    for a in root.findall("arg")}
        self.assertEqual("false", defaults["start_base"])
        self.assertEqual("false", defaults["start_lidar"])


if __name__ == "__main__":
    unittest.main()
