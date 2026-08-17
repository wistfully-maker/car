import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

STOP_ROOT = Path(__file__).resolve().parents[1]
MISSION_INTEGRATION = STOP_ROOT / "launch" / "mission_integration.launch"
OMNI_DIR = STOP_ROOT / "config" / "omni"
OMNI_FILES = (
    OMNI_DIR / "move_base_params.yaml",
    OMNI_DIR / "costmap_common_params.yaml",
    OMNI_DIR / "global_planner_params.yaml",
    OMNI_DIR / "teb_local_planner_params.yaml",
    OMNI_DIR / "local_costmap_params.yaml",
    OMNI_DIR / "global_costmap_params.yaml",
)
UCAR_NAV_FILES = (
    "$(find ucar_nav)/launch/config/move_base/move_base_params.yaml",
    "$(find ucar_nav)/launch/config/move_base/costmap_common_params.yaml",
    "$(find ucar_nav)/launch/config/move_base/global_planner_params.yaml",
    "$(find ucar_nav)/launch/config/move_base/teb_local_planner_params.yaml",
    "$(find ucar_nav)/launch/config/move_base/local_costmap_params.yaml",
    "$(find ucar_nav)/launch/config/move_base/global_costmap_params.yaml",
)
OMNI_LAUNCH_FILES = tuple(
    "$(find stop)/config/omni/" + p.name for p in OMNI_FILES
)


class OmniConfigSetTests(unittest.TestCase):
    def test_all_six_omni_config_files_exist(self):
        for path in OMNI_FILES:
            self.assertTrue(path.exists(), path.name)

    def test_omni_config_files_parse_as_yaml(self):
        for path in OMNI_FILES:
            yaml.safe_load(path.read_text(encoding="utf-8"))

    def test_omni_teb_velocity_and_weight_contract(self):
        cfg = yaml.safe_load(
            (OMNI_DIR / "teb_local_planner_params.yaml").read_text(
                encoding="utf-8"
            )
        )["TebLocalPlannerROS"]
        expected = {
            "holonomic_robot": True,
            "max_vel_x": 0.60,
            "max_vel_x_backwards": 0.30,
            "max_vel_y": 0.45,
            "max_vel_theta": 1.20,
            "acc_lim_x": 0.50,
            "acc_lim_y": 0.50,
            "acc_lim_theta": 1.50,
            "weight_kinematics_nh": 1.0,
            "weight_kinematics_forward_drive": 1.0,
            "weight_optimaltime": 1.0,
            "weight_obstacle": 60.0,
            "costmap_obstacles_behind_robot_dist": 1.5,
            "min_turning_radius": 0.0,
        }
        for key, value in expected.items():
            self.assertEqual(value, cfg[key], key)

    def test_omni_teb_keeps_baseline_goal_and_optimizer_contract(self):
        cfg = yaml.safe_load(
            (OMNI_DIR / "teb_local_planner_params.yaml").read_text(
                encoding="utf-8"
            )
        )["TebLocalPlannerROS"]
        for key, value in (
            ("xy_goal_tolerance", 0.1),
            ("yaw_goal_tolerance", 0.2),
            ("free_goal_vel", True),
            ("min_obstacle_dist", 0.15),
            ("inflation_dist", 0.5),
            ("no_inner_iterations", 5),
            ("no_outer_iterations", 4),
            ("penalty_epsilon", 0.1),
        ):
            self.assertEqual(value, cfg[key], key)

    def test_omni_inflation_contract(self):
        common = yaml.safe_load(
            (OMNI_DIR / "costmap_common_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(0.28, common["inflation_layer"]["inflation_radius"])
        self.assertEqual(
            3.0, common["inflation_layer"]["cost_scaling_factor"]
        )
        local = yaml.safe_load(
            (OMNI_DIR / "local_costmap_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            0.28, local["local_costmap"]["inflation_layer"]["inflation_radius"]
        )
        self.assertEqual(
            3.0, local["local_costmap"]["inflation_layer"]["cost_scaling_factor"]
        )

    def test_omni_footprint_matches_vehicle_baseline(self):
        common = yaml.safe_load(
            (OMNI_DIR / "costmap_common_params.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            [[0.171, -0.128], [0.171, 0.128], [-0.171, 0.128], [-0.171, -0.128]],
            common["footprint"],
        )


class MissionIntegrationModeSwitchTests(unittest.TestCase):
    def _groups(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        groups = {}
        for g in root.findall("group"):
            key = ("if", g.attrib["if"]) if "if" in g.attrib else (
                "unless", g.attrib["unless"])
            groups[key] = g
        return groups

    def _files(self, group):
        return {r.attrib["file"] for r in group.iter("rosparam")}

    def test_declares_navigation_mode_default_differential(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        args = {
            a.attrib["name"]: a.attrib.get("default")
            for a in root.findall("arg")
        }
        self.assertEqual("differential", args["navigation_mode"])

    def test_omni_group_selects_stop_omni_configs(self):
        groups = self._groups()
        omni = groups[("if", "$(eval arg('navigation_mode') == 'omni')")]
        self.assertEqual(set(OMNI_LAUNCH_FILES), self._files(omni))
        common_ns = [
            r for r in omni.iter("rosparam")
            if "costmap_common_params.yaml" in r.attrib["file"]
        ]
        self.assertEqual(2, len(common_ns))
        self.assertEqual(
            {"/move_base/global_costmap", "/move_base/local_costmap"},
            {r.attrib["ns"] for r in common_ns},
        )

    def test_non_omni_group_falls_back_to_original_ucar_nav_configs(self):
        groups = self._groups()
        fallback = groups[
            ("unless", "$(eval arg('navigation_mode') == 'omni')")
        ]
        self.assertEqual(set(UCAR_NAV_FILES), self._files(fallback))
        self.assertIn(
            ("unless", "$(eval arg('navigation_mode') == 'omni')"),
            groups,
        )

    def test_move_base_planner_and_velocity_remap_unchanged(self):
        root = ET.parse(MISSION_INTEGRATION).getroot()
        move_base = next(
            n for n in root.iter("node") if n.attrib.get("pkg") == "move_base"
        )
        params = {
            p.attrib["name"]: p.attrib["value"]
            for p in move_base.findall("param")
        }
        self.assertEqual(
            "teb_local_planner/TebLocalPlannerROS",
            params["base_local_planner"],
        )
        remaps = {
            r.attrib["from"]: r.attrib["to"]
            for r in move_base.findall("remap")
        }
        self.assertEqual("/cmd_vel/stop_navigation", remaps["cmd_vel"])


if __name__ == "__main__":
    unittest.main()
