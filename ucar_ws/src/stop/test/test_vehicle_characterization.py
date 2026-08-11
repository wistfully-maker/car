"""Freeze the vehicle-proven stop algorithm before any integration edit.

Two layers:
1. VehicleCharacterizationTests — must pass against the pristine snapshot and
   stay green after the minimal integration seam (waypoints, warehouse map,
   Stage 0-6 assignments, Phase 1 backup/180°/costmap clear, Phase 2 jump,
   retry parameters, result strings, and OCR model hashes from the manifest).
2. MissionSeamContractTests — RED before the seam, GREEN after: no auto-start,
   activation seam, cancel seam, internal phase event, protocol task input.
"""

import ast
import hashlib
import sys
import unittest
from pathlib import Path

STOP_ROOT = Path(__file__).resolve().parents[1]
MISSION_SOURCE = (STOP_ROOT / "scripts" / "mission_orchestrator.py").read_text(
    encoding="utf-8"
)
MISSION_TREE = ast.parse(MISSION_SOURCE)
MANIFEST = STOP_ROOT / "VEHICLE_SNAPSHOT.sha256"

# 与快照一致、本次集成不允许改动的文件（模型字节必须逐位一致）。
FROZEN_MANIFEST_FILES = (
    "scripts/models/ppocrv4_det.rknn",
    "scripts/models/ppocrv4_rec.rknn",
    "scripts/models/ppocr_keys_v1.txt",
    "scripts/ocr/__init__.py",
    "scripts/ocr/ppocr_det.py",
    "scripts/ocr/ppocr_rec.py",
    "scripts/ocr/ppocr_system.py",
    "scripts/ocr/rknn_executor.py",
    "scripts/ocr/utils/db_postprocess.py",
    "scripts/ocr/utils/operators.py",
    "scripts/ocr/utils/rec_postprocess.py",
    "scripts/ocr/utils/__init__.py",
    "scripts/ocr_native_node.py",
    "scripts/precision_park.py",
    "scripts/scan_and_park.py",
    "msg/BoundingBox.msg",
    "msg/BoundingBoxes.msg",
)


def function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


def assigned_literal(tree, target_name):
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == target_name:
                    return ast.literal_eval(node.value)
    return None


def calls(function, func_name):
    return [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == func_name
    ]


def call_with_args(function, func_name):
    """返回 (func_name, [字面量实参]) 列表，用于断言固定调用点。"""
    result = []
    for node in calls(function, func_name):
        try:
            args = [ast.literal_eval(arg) for arg in node.args]
        except (ValueError, TypeError):
            args = None
        result.append((func_name, args))
    return result


class VehicleCharacterizationTests(unittest.TestCase):
    def test_find_points_list_is_exactly_the_vehicle_waypoints(self):
        waypoints = assigned_literal(MISSION_TREE, "FIND_POINTS_LIST")
        self.assertEqual(
            [
                (-0.812845, -2.44196, 0.0, 1.0),
                (0.771455, -2.44196, 0.0, 1.0),
                (1.7843, -2.43094, 0.0, 1.0),
            ],
            waypoints,
        )

    def test_warehouse_map_matches_vehicle_aliases(self):
        self.assertEqual(
            {
                "食品": "食品加工车间",
                "电子": "电子产品生产车间",
                "日用": "日用品加工车间",
                "电子产品": "电子产品生产车间",
            },
            assigned_literal(MISSION_TREE, "WAREHOUSE_MAP"),
        )

    def test_stage_zero_to_six_assignments_are_intact(self):
        boxes = function(MISSION_TREE, "boxes_callback")
        lidar = function(MISSION_TREE, "LidarCallback")
        done = function(MISSION_TREE, "mission_done")
        self.assertIsNotNone(boxes)
        self.assertIsNotNone(lidar)
        self.assertIsNotNone(done)
        for marker in (
            "rotate_speed(camera_deg, 0.5)",     # Stage 0 首次对准
            "lidar_processing_flag = True",      # Stage 1 启用 LiDAR
            "rotate_speed(angle, 0.3)",          # Stage 3 PCA 旋转对正
            "cmd.linear.y = 0.3 if camera_deg > 0 else -0.3",  # Stage 4 Y 微调
            "cmd.linear.x = 0.10",               # Stage 2 慢速逼近
            "cmd.linear.x = 0.13",               # Stage 5 X 微调
            "search_item_stage = 6",             # 完成阶段
            "dist_forward_item > board_center_to_park_dist + 0.05",
        ):
            with self.subTest(marker=marker):
                self.assertIn(marker, MISSION_SOURCE)

    def test_phase1_backup_turn_and_costmap_clear_are_intact(self):
        switch = function(MISSION_TREE, "switch_to_phase2")
        self.assertIsNotNone(switch)
        self.assertIn("cmd.linear.x = -0.12", MISSION_SOURCE)
        self.assertIn("rospy.Duration(4.5)", MISSION_SOURCE)
        self.assertEqual(
            [("rotate_speed", [180, 1.5])],
            call_with_args(switch, "rotate_speed"),
        )
        self.assertIn("/move_base/clear_costmaps", MISSION_SOURCE)

    def test_phase2_double_pointer_jump_is_intact(self):
        switch = function(MISSION_TREE, "switch_to_phase2")
        self.assertIsNotNone(switch)
        self.assertIn("current_point_index = sim_point_index", MISSION_SOURCE)

    def test_vehicle_parameters_and_retry_defaults_are_intact(self):
        self.assertEqual(2, assigned_literal(MISSION_TREE, "max_navigation_retries"))
        self.assertEqual(6, assigned_literal(MISSION_TREE, "rotate_num_threshold"))
        self.assertEqual(
            0.20, assigned_literal(MISSION_TREE, "board_center_to_park_dist")
        )
        self.assertEqual(
            0.50, assigned_literal(MISSION_TREE, "x_align_tolerance")
        )
        self.assertEqual(12, assigned_literal(MISSION_TREE, "y_align_tolerance"))
        self.assertEqual(1.5, assigned_literal(MISSION_TREE, "lidar_offset_deg"))
        for param in ("target_distance", "0.20", "x_align_tolerance", "0.50",
                      "y_align_tolerance", "12", "max_rotations", "6",
                      "initial_pose_x", "initial_pose_yaw"):
            self.assertIn(param, MISSION_SOURCE)

    def test_navigation_result_handling_is_intact(self):
        goal = function(MISSION_TREE, "goal_callback")
        self.assertIsNotNone(goal)
        self.assertIn("msg.status.status == 3", MISSION_SOURCE)   # SUCCEEDED
        self.assertIn("msg.status.status == 4", MISSION_SOURCE)   # ABORTED
        self.assertIn("msg.status.status == 5", MISSION_SOURCE)   # REJECTED
        self.assertIn("navigation_failed_count", MISSION_SOURCE)
        self.assertIn("max_navigation_retries", MISSION_SOURCE)

    def test_mission_result_strings_are_unchanged(self):
        for marker in ("phase1_done", '"done"', "failed:not_found"):
            self.assertIn(marker, MISSION_SOURCE)

    def test_go_to_find_point_skips_close_waypoints(self):
        self.assertIn("dist < 0.3", MISSION_SOURCE)

    def test_waypoint_index_advances_only_after_scan_exhaustion(self):
        go_source = ast.get_source_segment(
            MISSION_SOURCE, function(MISSION_TREE, "go_to_find_point")
        )
        goal_source = ast.get_source_segment(
            MISSION_SOURCE, function(MISSION_TREE, "goal_callback")
        )
        boxes_source = ast.get_source_segment(
            MISSION_SOURCE, function(MISSION_TREE, "boxes_callback")
        )

        self.assertNotIn("current_point_index += 1", go_source)
        self.assertNotIn("current_point_index += 1", goal_source)

        exhausted_source = boxes_source[
            boxes_source.index('rospy.loginfo("  Exhausted, next waypoint")'):
        ]
        self.assertIn("current_point_index += 1", exhausted_source)
        self.assertLess(
            exhausted_source.index("current_point_index += 1"),
            exhausted_source.index("go_to_find_point()"),
        )

    def test_sim_workshop_records_the_active_scan_waypoint(self):
        boxes_source = ast.get_source_segment(
            MISSION_SOURCE, function(MISSION_TREE, "boxes_callback")
        )
        self.assertIn("sim_point_index = current_point_index", boxes_source)

    def test_frozen_snapshot_files_match_vehicle_manifest(self):
        expected = {}
        for line in MANIFEST.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            digest, relative = line.split(None, 1)
            expected[relative] = digest
        for relative in FROZEN_MANIFEST_FILES:
            with self.subTest(file=relative):
                self.assertIn(relative, expected)
                actual = hashlib.sha256(
                    (STOP_ROOT / relative).read_bytes()
                ).hexdigest()
                self.assertEqual(
                    expected[relative],
                    actual,
                    "%s 与车端快照不一致，禁止改动已验证模型/算法文件" % relative,
                )


class MissionSeamContractTests(unittest.TestCase):
    """集成车缝契约：先写失败，改造后转绿。"""

    def test_activation_seam_calls_start_mission(self):
        activate = function(MISSION_TREE, "activate")
        self.assertIsNotNone(activate)
        self.assertEqual(1, len(calls(activate, "start_mission")))

    def test_auto_start_is_removed(self):
        self.assertNotIn("_wait_and_start", MISSION_SOURCE)
        self.assertIsNone(function(MISSION_TREE, "_wait_and_start"))

    def test_cancel_seam_stops_motion_and_emits_failure_once(self):
        cancel = function(MISSION_TREE, "cancel_mission")
        self.assertIsNotNone(cancel)
        self.assertIn("is_searching_item = False", MISSION_SOURCE)
        self.assertIn("failed:cancelled", MISSION_SOURCE)

    def test_internal_tts_is_removed_from_mission_done(self):
        done = function(MISSION_TREE, "mission_done")
        self.assertIsNotNone(done)
        self.assertEqual([], calls(done, "speak"))

    def test_phase_events_use_private_integration_topic(self):
        self.assertIn("/stop/mission_event", MISSION_SOURCE)

    def test_mission_waits_for_protocol_task_input(self):
        self.assertIn("/task/stop_mission_goal", MISSION_SOURCE)
        self.assertIn("MissionGate", MISSION_SOURCE)

    def test_cancel_subscription_is_wired(self):
        self.assertIn("/task/cancel", MISSION_SOURCE)

    def test_manual_velocity_is_isolated(self):
        self.assertIn("/cmd_vel/stop_manual", MISSION_SOURCE)

    def test_motion_mode_is_published_for_stop_mux(self):
        self.assertIn("/stop/motion_mode", MISSION_SOURCE)
        self.assertIn("_set_mode", MISSION_SOURCE)

    def test_phase2_waits_for_global_ack(self):
        self.assertIn("/stop/mission_ack", MISSION_SOURCE)
        self.assertIn("phase2_pending", MISSION_SOURCE)
        self.assertIn("update_simulation", MISSION_SOURCE)


if __name__ == "__main__":
    unittest.main()
