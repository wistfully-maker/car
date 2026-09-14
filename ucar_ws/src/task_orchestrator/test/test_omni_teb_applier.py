import importlib.util
import math
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from task_orchestrator.omni_teb_params import teb_parameters

CONFIG_YAML = ROOT / "config" / "omni_fast_nav_teb.yaml"
LEGACY_WRAPPER = ROOT / "launch" / "legacy_navigation_include.launch"

VALID_CONFIG = {
    "holonomic_robot": True,
    "max_vel_x": 0.80,
    "max_vel_x_backwards": 0.30,
    "max_vel_y": 0.45,
    "max_vel_theta": 1.20,
    "acc_lim_x": 0.50,
    "acc_lim_y": 0.50,
    "acc_lim_theta": 1.50,
    "weight_kinematics_nh": 1.0,
    "weight_kinematics_forward_drive": 1.0,
    "weight_optimaltime": 1.0,
}


class OmniTebParamsTests(unittest.TestCase):
    def test_valid_config_passes_through_unchanged(self):
        self.assertEqual(VALID_CONFIG, teb_parameters(dict(VALID_CONFIG)))

    def test_missing_field_is_rejected(self):
        for field in VALID_CONFIG:
            broken = dict(VALID_CONFIG)
            del broken[field]
            with self.assertRaises(ValueError):
                teb_parameters(broken)

    def test_non_holonomic_config_is_rejected(self):
        broken = dict(VALID_CONFIG)
        broken["holonomic_robot"] = False
        with self.assertRaises(ValueError):
            teb_parameters(broken)

    def test_non_positive_or_non_numeric_velocity_is_rejected(self):
        for field in ("max_vel_y", "acc_lim_x", "weight_optimaltime"):
            for bad in (0.0, -0.1, True, "fast"):
                broken = dict(VALID_CONFIG)
                broken[field] = bad
                with self.assertRaises(ValueError):
                    teb_parameters(broken)

    def test_hard_caps_fail_closed(self):
        for field, cap in (
            ("max_vel_x", 1.0),
            ("max_vel_x_backwards", 0.5),
            ("max_vel_y", 0.6),
            ("max_vel_theta", 1.5),
            ("acc_lim_x", 1.0),
            ("acc_lim_y", 0.8),
            ("acc_lim_theta", 2.0),
        ):
            broken = dict(VALID_CONFIG)
            broken[field] = cap + 0.01
            with self.assertRaises(ValueError):
                teb_parameters(broken)

    def test_non_mapping_is_rejected(self):
        for bad in (None, [], 42, "x"):
            with self.assertRaises(ValueError):
                teb_parameters(bad)


class OmniFastNavConfigFileTests(unittest.TestCase):
    def test_config_file_exists_and_parses(self):
        self.assertTrue(CONFIG_YAML.exists())
        yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8"))

    def test_config_values_match_measured_contract(self):
        cfg = yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8"))
        self.assertEqual(VALID_CONFIG, cfg["pickup_teb"])

    def test_applier_script_is_installed_by_cmake(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/omni_teb_applier_node.py", cmake)


class LegacyWrapperModeSwitchTests(unittest.TestCase):
    def test_declares_navigation_mode_default_differential(self):
        root = ET.parse(LEGACY_WRAPPER).getroot()
        args = {
            a.attrib["name"]: a.attrib.get("default")
            for a in root.findall("arg")
        }
        self.assertEqual("differential", args["navigation_mode"])

    def test_omni_group_loads_config_and_starts_applier(self):
        root = ET.parse(LEGACY_WRAPPER).getroot()
        groups = {
            ("if", g.attrib["if"]) if "if" in g.attrib
            else ("unless", g.attrib["unless"]): g
            for g in root.findall("group")
        }
        omni = groups[("if", "$(eval arg('navigation_mode') == 'omni')")]
        files = {r.attrib["file"] for r in omni.iter("rosparam")}
        self.assertEqual(
            {"$(find task_orchestrator)/config/omni_fast_nav_teb.yaml"},
            files,
        )
        nodes = list(omni.iter("node"))
        self.assertEqual(1, len(nodes))
        self.assertEqual(
            "omni_teb_applier_node.py", nodes[0].attrib["type"]
        )
        params = {
            p.attrib["name"]: p.attrib["value"]
            for p in nodes[0].findall("param")
        }
        self.assertEqual(
            "/omni_fast_nav/pickup_teb", params["config_path"]
        )

    def test_no_applier_outside_omni_mode(self):
        root = ET.parse(LEGACY_WRAPPER).getroot()
        top_level_nodes = [
            n for n in root.findall("node")
            if n.attrib.get("type") == "omni_teb_applier_node.py"
        ]
        self.assertEqual([], top_level_nodes)
        for group in root.findall("group"):
            if "if" in group.attrib and "omni" in group.attrib["if"]:
                continue
            self.assertFalse(any(
                n.attrib.get("type") == "omni_teb_applier_node.py"
                for n in group.iter("node")
            ))

    def test_vendor_include_arguments_unchanged(self):
        root = ET.parse(LEGACY_WRAPPER).getroot()
        include = root.find("include")
        self.assertIn("pickup_navigation.launch", include.attrib["file"])
        args = {
            a.attrib["name"]: a.attrib["value"]
            for a in include.findall("arg")
        }
        self.assertEqual("$(arg start_base)", args["start_base"])
        self.assertEqual("$(arg start_lidar)", args["start_lidar"])
        self.assertEqual("/cmd_vel/navigation", args["cmd_vel_topic"])


class ApplierNodeTests(unittest.TestCase):
    """Node wiring: reads config path, applies once, publishes status."""

    def _load_node_module(self):
        for name in ("rospy", "dynamic_reconfigure",
                     "dynamic_reconfigure.client", "std_msgs",
                     "std_msgs.msg"):
            if name not in sys.modules:
                sys.modules[name] = types.ModuleType(name)
        sys.modules["dynamic_reconfigure"].client = sys.modules[
            "dynamic_reconfigure.client"
        ]
        sys.modules["std_msgs.msg"].String = type(
            "String",
            (),
            {"__init__": lambda self, data=None: setattr(self, "data", data)},
        )
        spec = importlib.util.spec_from_file_location(
            "omni_teb_applier_node",
            ROOT / "scripts" / "omni_teb_applier_node.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_node_applies_parameters_once_and_publishes_status(self):
        module = self._load_node_module()
        applied = {}
        statuses = []

        class FakeClient:
            def __init__(self, *args, **kwargs):
                pass

            def update_configuration(self, target):
                applied.update(target)

        class FakePublisher:
            def publish(self, message):
                statuses.append(message.data)

        params = {
            "~config_path": "/omni_fast_nav/pickup_teb",
            "~reconfigure_namespace": "/move_base/TebLocalPlannerROS",
            "~timeout": 5.0,
            "~retry_period": 0.1,
        }
        params_req = {
            "/omni_fast_nav/pickup_teb": dict(VALID_CONFIG),
        }

        module.rospy.get_param = lambda key, default=None: params_req.get(
            key, params.get(key, default)
        )
        module.rospy.is_shutdown = lambda: False
        module.rospy.get_time = lambda: 100.0
        module.rospy.sleep = lambda _seconds: None
        module.rospy.loginfo = lambda *args, **kwargs: None
        module.rospy.logerr = lambda *args, **kwargs: None
        module.rospy.logwarn = lambda *args, **kwargs: None
        module.rospy.logwarn_throttle = lambda *args, **kwargs: None
        module.rospy.signal_shutdown = lambda reason=None: None
        module.dynamic_reconfigure.client.Client = FakeClient

        node = module.OmniTebApplier.__new__(module.OmniTebApplier)
        node.config_path = "/omni_fast_nav/pickup_teb"
        node.reconfigure_namespace = "/move_base/TebLocalPlannerROS"
        node.timeout = 5.0
        node.retry_period = 0.1
        node.status_pub = FakePublisher()
        node._apply_once()

        self.assertEqual(VALID_CONFIG, applied)
        self.assertEqual(["applied"], statuses)


if __name__ == "__main__":
    unittest.main()
