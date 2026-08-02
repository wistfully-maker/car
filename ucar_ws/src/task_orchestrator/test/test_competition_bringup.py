import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPETITION = ROOT / "launch" / "competition_full.launch"
ORCHESTRATOR = ROOT / "launch" / "task_orchestrator.launch"


class CompetitionBringupTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(COMPETITION).getroot()
        self.text = COMPETITION.read_text(encoding="utf-8")

    def test_declares_every_independent_start_switch(self):
        expected = {
            "start_fast_nav", "start_base", "start_lidar", "start_camera",
            "start_fast_nav_adapter", "start_readiness_gate", "start_speech",
            "start_qr", "start_llm", "start_orchestrator",
            "start_velocity_arbiter",
        }
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in self.root.findall("arg")}
        self.assertTrue(expected <= set(args))
        for name in expected:
            self.assertIn("$(arg %s)" % name, self.text)
        self.assertEqual("true", args["start_velocity_arbiter"])

    def test_documents_external_contracts_and_fail_fast_boundaries(self):
        for marker in (
            "EXTERNAL_CONTRACT",
            "ucar_fast_nav/pickup_navigation.launch",
            "speech_command/speech_command.launch",
            "qr_item_search/qr_item_search.launch",
            "llm_spark/llm_spark.launch",
            "usb_cam/usb_cam_node",
            "fail fast",
            "does not start a camera",
            "external equivalent sole arbiter",
        ):
            self.assertIn(marker, self.text)

    def test_uses_real_external_launch_interfaces(self):
        includes = {include.attrib["file"]: include
                    for include in self.root.iter("include")}
        nav = includes["$(arg fast_nav_launch)"]
        self.assertEqual(
            {"start_base": "$(arg start_base)",
             "start_lidar": "$(arg start_lidar)",
             "cmd_vel_topic": "/cmd_vel/navigation"},
            {arg.attrib["name"]: arg.attrib["value"]
             for arg in nav.findall("arg")},
        )
        self.assertIn("$(find ucar_fast_nav)/launch/pickup_navigation.launch",
                      self.text)
        self.assertIn("$(find speech_command)/launch/speech_command.launch",
                      self.text)
        self.assertIn("$(find qr_item_search)/launch/qr_item_search.launch",
                      self.text)
        self.assertIn("$(find llm_spark)/launch/llm_spark.launch", self.text)

    def test_base_and_lidar_are_only_fast_nav_downstream_switches(self):
        nav_group = next(group for group in self.root.findall("group")
                         if group.attrib.get("if") == "$(arg start_fast_nav)")
        nav_args = {arg.attrib["name"]: arg.attrib["value"]
                    for arg in nav_group.find("include").findall("arg")}
        self.assertEqual("$(arg start_base)", nav_args["start_base"])
        self.assertEqual("$(arg start_lidar)", nav_args["start_lidar"])
        outside = "".join(ET.tostring(group, encoding="unicode")
                          for group in self.root.findall("group")
                          if group is not nav_group)
        self.assertNotIn("$(arg start_base)", outside)
        self.assertNotIn("$(arg start_lidar)", outside)

    def test_has_one_shared_camera_and_remaps_qr_velocity(self):
        cameras = [node for node in self.root.iter("node")
                   if node.attrib.get("pkg") == "usb_cam"]
        self.assertEqual(1, len(cameras))
        qr_groups = [group for group in self.root.findall("group")
                     if group.attrib.get("if") == "$(arg start_qr)"]
        self.assertEqual(1, len(qr_groups))
        qr = qr_groups[0]
        self.assertEqual(
            {"/cmd_vel": "/cmd_vel/qr"},
            {r.attrib["from"]: r.attrib["to"] for r in qr.findall("remap")},
        )
        image_args = [arg for arg in qr.iter("arg")
                      if arg.attrib.get("name") == "image_topic"]
        self.assertEqual(["/usb_cam/image_raw"],
                         [arg.attrib.get("value") for arg in image_args])

    def test_delegates_waypoint_loading_exclusively_to_vendor_launch(self):
        waypoint = [p for p in self.root.iter("rosparam")
                    if p.attrib.get("ns") == "/ucar_fast_nav"]
        self.assertEqual([], waypoint)
        nav_includes = [include for include in self.root.iter("include")
                        if include.attrib.get("file") == "$(arg fast_nav_launch)"]
        self.assertEqual(1, len(nav_includes))
        defaults = {arg.attrib["name"]: arg.attrib.get("default")
                    for arg in self.root.findall("arg")}
        self.assertEqual(
            "$(find ucar_fast_nav)/launch/pickup_navigation.launch",
            defaults["fast_nav_launch"],
        )
        self.assertNotIn("pickup_goal:", self.text)

    def test_starts_auxiliary_nodes_with_private_full_config(self):
        expected = {
            "fast_nav_adapter": "fast_nav_adapter_node.py",
            "readiness_gate": "system_readiness_gate_node.py",
            "velocity_arbiter": "velocity_arbiter_node.py",
        }
        task_root = ET.parse(ORCHESTRATOR).getroot()
        nodes = {node.attrib.get("name"): node for node in task_root.findall("node")}
        for name, executable in expected.items():
            self.assertEqual(executable, nodes[name].attrib.get("type"))
            rosparam = nodes[name].find("rosparam")
            self.assertEqual("load", rosparam.attrib.get("command"))
            self.assertEqual("$(find task_orchestrator)/config/orchestrator.yaml",
                             rosparam.attrib.get("file"))
        enabled = {
            "fast_nav_adapter": ("start_fast_nav_adapter", "enable_fast_nav_adapter"),
            "readiness_gate": ("start_readiness_gate", "enable_readiness_gate"),
            "velocity_arbiter": ("start_velocity_arbiter", "enable_velocity_arbiter"),
        }
        for _, (start_arg, enable_arg) in enabled.items():
            groups = [g for g in self.root.findall("group")
                      if g.attrib.get("if") == "$(arg %s)" % start_arg]
            self.assertEqual(1, len(groups))
            values = {a.attrib["name"]: a.attrib["value"]
                      for a in groups[0].find("include").findall("arg")}
            self.assertEqual("true", values[enable_arg])

    def test_bans_legacy_navigation_and_extra_velocity_outputs(self):
        for banned in ("ucar_waypoint_nav", "amcl", "dynamic_obstacle",
                       "/cmd_vel/avoidance", "/cmd_vel/line"):
            self.assertNotIn(banned, self.text)
        self.assertEqual(1, self.text.count('to="/cmd_vel/qr"'))
        self.assertIn('value="/cmd_vel/navigation"', self.text)
        self.assertEqual("true", next(
            arg.attrib["default"] for arg in self.root.findall("arg")
            if arg.attrib["name"] == "start_velocity_arbiter"
        ))

    def test_task_launch_auxiliary_nodes_are_opt_in(self):
        root = ET.parse(ORCHESTRATOR).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("arg")}
        for name in ("enable_fast_nav_adapter", "enable_readiness_gate",
                     "enable_velocity_arbiter"):
            self.assertEqual("false", args[name])
        nodes = {node.attrib["name"]: node for node in root.findall("node")}
        self.assertEqual("$(arg enable_fast_nav_adapter)",
                         nodes["fast_nav_adapter"].attrib.get("if"))
        self.assertEqual("$(arg enable_readiness_gate)",
                         nodes["readiness_gate"].attrib.get("if"))
        self.assertEqual("$(arg enable_velocity_arbiter)",
                         nodes["velocity_arbiter"].attrib.get("if"))


if __name__ == "__main__":
    unittest.main()
