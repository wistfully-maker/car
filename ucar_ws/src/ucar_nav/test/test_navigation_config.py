#!/usr/bin/env python3
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch"


def load_yaml(relative_path):
    with (PACKAGE / relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def launch_text(name):
    return (LAUNCH / name).read_text(encoding="utf-8")


class NavigationConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        required = (
            "launch/robot_base_bringup.launch",
            "launch/lidar_bringup.launch",
            "launch/navigation_stack.launch",
            "launch/other_team_baseline.launch",
            "launch/ucar_navigation.launch",
            "config/amcl/amcl_omni.yaml",
            "config/costmap/common.yaml",
            "config/costmap/common_teb.yaml",
            "config/costmap/global.yaml",
            "config/costmap/global_teb.yaml",
            "config/costmap/local.yaml",
            "config/costmap/local_teb.yaml",
            "config/local_planners/dwa_safe.yaml",
            "config/baselines/other_team/amcl.yaml",
            "config/baselines/other_team/global_planner.yaml",
            "config/baselines/other_team/teb.yaml",
            "config/baselines/other_team/costmap_common.yaml",
            "config/baselines/other_team/local_costmap.yaml",
            "config/baselines/other_team/global_costmap.yaml",
            "config/baselines/other_team/move_base.yaml",
            "config/baselines/other_team/first_run_limits.yaml",
            "config/global_planner.yaml",
            "config/move_base.yaml",
            "config/move_base_teb.yaml",
            "scripts/capture_nav_diagnostics.sh",
            "README.md",
            "HANDOFF.md",
        )
        for relative_path in required:
            self.assertTrue(
                (PACKAGE / relative_path).is_file(),
                relative_path,
            )

    def test_corner_profile_files_exist(self):
        required = (
            "config/profiles/navfn_teb_corner.yaml",
            "config/profiles/navfn_dwa_corner.yaml",
            "config/profiles/legacy_0717_teb.yaml",
            "config/profiles/legacy_0721_dwa.yaml",
        )
        for relative_path in required:
            self.assertTrue(
                (PACKAGE / relative_path).is_file(),
                relative_path,
            )

    def test_corner_profiles_keep_common_forward_and_turn_limits(self):
        teb = load_yaml("config/local_planners/teb_corner_safe.yaml")[
            "TebLocalPlannerROS"
        ]
        dwa = load_yaml("config/local_planners/dwa_corner_safe.yaml")[
            "DWAPlannerROS"
        ]
        for planner in (teb, dwa):
            self.assertEqual(0.45, planner["max_vel_x"])
            self.assertEqual(0.60, planner["max_vel_theta"])
        self.assertEqual(0.02, teb["max_vel_y"])
        self.assertEqual(0.20, teb["acc_lim_y"])
        self.assertEqual(0.20, dwa["max_vel_y"])

    def test_navfn_corner_profiles_reference_the_expected_planners(self):
        teb_profile = load_yaml("config/profiles/navfn_teb_corner.yaml")
        dwa_profile = load_yaml("config/profiles/navfn_dwa_corner.yaml")
        for profile in (teb_profile, dwa_profile):
            self.assertEqual("maps/map.yaml", profile["map"])
            self.assertEqual("navfn/NavfnROS", profile["global_planner"])
            self.assertIn("config/global_planners/navfn.yaml", profile["parameter_files"])
        self.assertEqual(
            "teb_local_planner/TebLocalPlannerROS",
            teb_profile["local_planner"],
        )
        self.assertEqual(
            "dwa_local_planner/DWAPlannerROS",
            dwa_profile["local_planner"],
        )

    def test_launch_files_are_valid_xml(self):
        for path in LAUNCH.glob("*.launch"):
            ET.parse(path)

    def test_package_declares_teb_runtime_dependency(self):
        root = ET.parse(PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text
            for element in root
            if element.tag in ("depend", "exec_depend")
        }
        self.assertIn("teb_local_planner", dependencies)

    def test_package_declares_lateral_controller_dependencies(self):
        root = ET.parse(PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text
            for element in root
            if element.tag in (
                "depend",
                "build_depend",
                "build_export_depend",
                "exec_depend",
            )
        }
        for dependency in ("dynamic_reconfigure", "geometry_msgs", "tf2_ros"):
            self.assertIn(dependency, dependencies)

    def test_cmake_installs_navigation_helpers(self):
        text = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for script in (
            "scripts/lateral_mode_logic.py",
            "scripts/teb_lateral_mode_controller.py",
            "scripts/initialize_amcl.py",
        ):
            self.assertIn(script, text)

    def test_navigation_stack_defaults_to_navfn_teb_corner(self):
        text = launch_text("navigation_stack.launch")
        self.assertIn(
            '<arg name="navigation_profile" default="navfn_teb_corner"/>',
            text,
        )
        self.assertIn(
            'value="teb_local_planner/TebLocalPlannerROS"',
            text,
        )
        self.assertIn("teb_corner_safe.yaml", text)

    def test_navigation_stack_exposes_four_explicit_profiles(self):
        text = launch_text("navigation_stack.launch")
        for profile in (
            "navfn_teb_corner",
            "navfn_dwa_corner",
            "legacy_0717_teb",
            "legacy_0721_dwa",
        ):
            self.assertIn(
                "arg('navigation_profile') == '{}'".format(profile),
                text,
            )

    def test_only_navfn_teb_profile_starts_lateral_controller(self):
        root = ET.parse(LAUNCH / "navigation_stack.launch").getroot()
        launch_args = {
            item.attrib["name"]: item.attrib["default"]
            for item in root.findall("arg")
        }
        self.assertEqual("true", launch_args["enable_lateral_mode_controller"])
        groups = root.findall("group")
        teb_group = next(
            group
            for group in groups
            if "navfn_teb_corner" in group.attrib.get("if", "")
        )
        includes = teb_group.findall("include")
        self.assertEqual(1, len(includes))
        self.assertEqual(
            "$(arg enable_lateral_mode_controller)",
            includes[0].attrib.get("if"),
        )
        for group in groups:
            if group is not teb_group:
                self.assertEqual([], group.findall("include"))

    def test_teb_safe_profile_matches_current_safety_limits(self):
        teb = load_yaml(
            "config/local_planners/teb_safe.yaml"
        )["TebLocalPlannerROS"]
        self.assertEqual("polygon", teb["footprint_model"]["type"])
        self.assertEqual(
            [
                [0.171, -0.128],
                [0.171, 0.128],
                [-0.171, 0.128],
                [-0.171, -0.128],
            ],
            teb["footprint_model"]["vertices"],
        )
        self.assertFalse(teb["enable_homotopy_class_planning"])
        self.assertEqual(30, teb["max_samples"])
        self.assertEqual(1.2, teb["max_global_plan_lookahead_dist"])
        self.assertEqual(1.0, teb["weight_kinematics_nh"])
        self.assertEqual(1.0, teb["weight_kinematics_forward_drive"])
        self.assertEqual(2.0, teb["weight_max_vel_y"])
        self.assertEqual(0.20, teb["max_vel_x"])
        self.assertEqual(0.08, teb["max_vel_x_backwards"])
        self.assertEqual(0.08, teb["max_vel_y"])
        self.assertEqual(0.40, teb["max_vel_theta"])
        self.assertEqual(0.50, teb["acc_lim_x"])
        self.assertEqual(0.50, teb["acc_lim_y"])
        self.assertEqual(0.80, teb["acc_lim_theta"])
        self.assertEqual(0.20, teb["dt_ref"])
        self.assertEqual(0.05, teb["min_obstacle_dist"])
        self.assertEqual(0.18, teb["inflation_dist"])
        self.assertEqual(0.20, teb["global_plan_viapoint_sep"])
        self.assertEqual(3, teb["feasibility_check_no_poses"])
        self.assertEqual(2, teb["no_inner_iterations"])
        self.assertEqual(1, teb["no_outer_iterations"])
        self.assertEqual(0.05, teb["penalty_epsilon"])
        self.assertEqual(100.0, teb["weight_obstacle"])
        self.assertEqual(0.5, teb["weight_inflation"])
        self.assertEqual(3.0, teb["weight_viapoint"])

    def test_other_team_baseline_preserves_source_parameters(self):
        root = "config/baselines/other_team/"
        planner = load_yaml(root + "global_planner.yaml")["GlobalPlanner"]
        teb = load_yaml(root + "teb.yaml")["TebLocalPlannerROS"]
        move_base = load_yaml(root + "move_base.yaml")
        limits = load_yaml(root + "first_run_limits.yaml")[
            "TebLocalPlannerROS"
        ]

        self.assertEqual(0, planner["orientation_mode"])
        self.assertEqual(253, planner["lethal_cost"])
        self.assertEqual("point", teb["footprint_model"]["type"])
        self.assertEqual(1100.0, teb["weight_kinematics_nh"])
        self.assertEqual(
            1000.0,
            teb["weight_kinematics_forward_drive"],
        )
        self.assertEqual(15.0, move_base["controller_frequency"])
        self.assertEqual(5.0, move_base["planner_frequency"])
        self.assertEqual(0.20, limits["max_vel_x"])
        self.assertEqual(0.12, limits["max_vel_x_backwards"])
        self.assertEqual(0.40, limits["max_vel_theta"])

    def test_other_team_baseline_launch_is_isolated_and_safety_limited(self):
        text = launch_text("other_team_baseline.launch")
        self.assertIn("teb_local_planner/TebLocalPlannerROS", text)
        self.assertIn("global_planner/GlobalPlanner", text)
        self.assertIn("config/baselines/other_team/teb.yaml", text)
        self.assertIn(
            "config/baselines/other_team/first_run_limits.yaml",
            text,
        )
        self.assertIn(
            '<arg name="apply_first_run_limits" default="true"/>',
            text,
        )
        self.assertNotIn("base_driver.launch", text)
        self.assertNotIn("ydlidar.launch", text)

    def test_teb_uses_nominal_footprint_and_lower_rate_costmap(self):
        common = load_yaml("config/costmap/common_teb.yaml")
        self.assertEqual(
            [
                [0.171, -0.128],
                [0.171, 0.128],
                [-0.171, 0.128],
                [-0.171, -0.128],
            ],
            common["footprint"],
        )
        self.assertEqual(0.25, common["inflation_layer"]["inflation_radius"])
        local = load_yaml("config/costmap/local_teb.yaml")["local_costmap"]
        self.assertEqual("map", local["global_frame"])
        self.assertEqual(5.0, local["update_frequency"])
        self.assertEqual(1.0, local["publish_frequency"])
        move_base = load_yaml("config/move_base_teb.yaml")
        self.assertEqual(5.0, move_base["controller_frequency"])
        self.assertEqual(
            0.0,
            move_base["planner_frequency"],
            "static global maps should not replace the TEB plan every 0.5 s",
        )

    def test_launch_loads_planner_specific_costmap_profiles(self):
        text = launch_text("navigation_stack.launch")
        self.assertIn("config/costmap/common_teb.yaml", text)
        self.assertIn("config/costmap/local_teb.yaml", text)
        self.assertIn("config/costmap/common.yaml", text)
        self.assertIn("config/costmap/local.yaml", text)
        self.assertIn("config/move_base_teb.yaml", text)
        self.assertIn("config/costmap/global_teb.yaml", text)

    def test_teb_global_costmap_does_not_duplicate_static_walls_from_laser(self):
        global_costmap = load_yaml(
            "config/costmap/global_teb.yaml"
        )["global_costmap"]
        plugin_names = {
            plugin["name"] for plugin in global_costmap["plugins"]
        }
        self.assertEqual({"static_layer", "inflation_layer"}, plugin_names)

    def test_hardware_and_navigation_launches_are_separated(self):
        hardware = launch_text("robot_base_bringup.launch")
        navigation = launch_text("navigation_stack.launch")
        self.assertIn("base_driver.launch", hardware)
        self.assertIn("lidar_bringup.launch", hardware)
        self.assertNotIn("ydlidar.launch", hardware)
        self.assertNotIn('pkg="move_base"', hardware)
        self.assertNotIn("base_driver.launch", navigation)
        self.assertNotIn("ydlidar.launch", navigation)
        self.assertNotIn("usb_cam", navigation)

    def test_lidar_bringup_exposes_extrinsic_arguments(self):
        root = ET.parse(LAUNCH / "lidar_bringup.launch").getroot()
        launch_args = {
            item.attrib["name"]: item.attrib["default"]
            for item in root.findall("arg")
        }
        self.assertEqual("-0.11", launch_args["laser_x"])
        self.assertEqual("0.165", launch_args["laser_z"])
        self.assertEqual("-0.07", launch_args["laser_yaw"])
        self.assertEqual("true", launch_args["start_lidar"])
        nodes = root.findall("node")
        lidar_nodes = [
            node for node in nodes
            if node.attrib.get("pkg") == "ydlidar"
            and node.attrib.get("type") == "ydlidar_node"
        ]
        self.assertEqual(1, len(lidar_nodes))
        static_nodes = [
            node for node in nodes
            if node.attrib.get("pkg") == "tf"
            and node.attrib.get("type") == "static_transform_publisher"
        ]
        self.assertEqual(1, len(static_nodes))
        args = static_nodes[0].attrib["args"].split()
        self.assertEqual(
            [
                "$(arg", "laser_x)", "0.0", "$(arg", "laser_z)",
                "$(arg", "laser_yaw)", "0.0", "0.0",
                "/base_link", "/laser_frame", "40",
            ],
            args,
        )

    def test_base_command_timeout_exceeds_slowest_control_period(self):
        root = ET.parse(LAUNCH / "robot_base_bringup.launch").getroot()
        timeout_param = root.find(
            "param[@name='/base_driver/cmd_timeout']"
        )
        self.assertIsNotNone(timeout_param)
        timeout = float(timeout_param.attrib["value"])
        teb_period = 1.0 / load_yaml(
            "config/move_base_teb.yaml"
        )["controller_frequency"]
        self.assertGreaterEqual(timeout, 2.0 * teb_period)

    def test_common_costmap_uses_safe_polygon_and_laser(self):
        common = load_yaml("config/costmap/common.yaml")
        footprint = common["footprint"]
        self.assertEqual(4, len(footprint))
        self.assertEqual(
            [
                [0.191, -0.148],
                [0.191, 0.148],
                [-0.191, 0.148],
                [-0.191, -0.148],
            ],
            footprint,
        )
        source = common["obstacle_layer"]["laser_scan_sensor"]
        self.assertEqual("laser_frame", source["sensor_frame"])
        self.assertEqual("scan", source["topic"])
        self.assertTrue(source["marking"])
        self.assertTrue(source["clearing"])

    def test_local_costmap_has_real_safety_gradient(self):
        local = load_yaml("config/costmap/local.yaml")["local_costmap"]
        self.assertEqual("odom", local["global_frame"])
        self.assertLessEqual(local["transform_tolerance"], 0.5)
        inflation = local["inflation_layer"]
        corner_radius = math.hypot(0.191, 0.148)
        self.assertGreater(inflation["inflation_radius"], corner_radius)
        self.assertGreaterEqual(inflation["inflation_radius"], 0.35)

    def test_global_planner_can_route_through_inscribed_narrow_passage(self):
        planner = load_yaml("config/global_planner.yaml")["GlobalPlanner"]
        self.assertEqual(254, planner["lethal_cost"])
        self.assertEqual(0, planner["orientation_mode"])
        self.assertFalse(planner["use_grid_path"])

    def test_dwa_safe_profile_is_holonomic_and_conservative(self):
        dwa = load_yaml(
            "config/local_planners/dwa_safe.yaml"
        )["DWAPlannerROS"]
        self.assertGreater(dwa["vy_samples"], 0)
        self.assertGreater(dwa["max_vel_y"], 0)
        self.assertLess(dwa["min_vel_y"], 0)
        self.assertLessEqual(dwa["max_vel_x"], 0.25)
        self.assertLessEqual(dwa["max_vel_y"], 0.15)
        self.assertLessEqual(dwa["max_vel_theta"], 0.5)
        self.assertGreaterEqual(dwa["occdist_scale"], 0.15)
        self.assertGreaterEqual(dwa["stop_time_buffer"], 0.4)
        control_period = 1.0 / dwa["controller_frequency"]
        self.assertLessEqual(
            dwa["min_vel_trans"], dwa["acc_lim_x"] * control_period
        )
        self.assertLessEqual(
            dwa["min_vel_theta"], dwa["acc_lim_theta"] * control_period
        )
        self.assertGreaterEqual(dwa["min_vel_trans"], 0.10)
        self.assertGreaterEqual(dwa["min_vel_theta"], 0.18)
        self.assertGreaterEqual(dwa["acc_lim_trans"], 1.0)
        self.assertLessEqual(
            dwa["vx_samples"] * dwa["vy_samples"] * dwa["vth_samples"],
            1024,
        )
        self.assertLessEqual(dwa["sim_time"], 1.5)

    def test_move_base_disables_uncommanded_recovery_rotation(self):
        move_base = load_yaml("config/move_base.yaml")
        self.assertFalse(move_base["recovery_behavior_enabled"])
        self.assertFalse(move_base["clearing_rotation_allowed"])
        self.assertGreaterEqual(move_base["oscillation_timeout"], 15.0)
        self.assertLessEqual(move_base["oscillation_distance"], 0.05)

    def test_amcl_updates_during_low_speed_short_motion(self):
        amcl = load_yaml("config/amcl/amcl_omni.yaml")
        self.assertLessEqual(amcl["update_min_d"], 0.05)
        self.assertLessEqual(amcl["update_min_a"], 0.10)

    def test_dwa_short_motion_can_reset_oscillation_flags(self):
        dwa = load_yaml(
            "config/local_planners/dwa_safe.yaml"
        )["DWAPlannerROS"]
        self.assertLessEqual(dwa["oscillation_reset_dist"], 0.05)
        self.assertLessEqual(dwa["oscillation_reset_angle"], 0.10)

    def test_docs_keep_backups_outside_catkin_source(self):
        for relative_path in ("README.md", "HANDOFF.md"):
            text = (PACKAGE / relative_path).read_text(encoding="utf-8")
            self.assertNotIn("ucar_ws/src/ucar_nav.backup", text)
            self.assertIn("ucar_nav_backups", text)


if __name__ == "__main__":
    unittest.main()
