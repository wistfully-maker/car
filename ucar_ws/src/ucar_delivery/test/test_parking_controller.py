"""Phase F: lock the closed-loop white-frame parking controller contract.

Per-state deterministic behavior, command signs, saturation, deadbands, rate
limiting, consecutive-frame transitions, line loss, stale camera data, lidar
disabled mode, wall safety stop, final stability verification, timeouts,
cancel and shutdown zero-velocity guarantees.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import parking_controller
from ucar_delivery.parking_controller import ParkingController
from ucar_delivery.frame_detector import FrameObservation

TIME = 1000.0

TIMEOUTS = {
    "align": 30.0,
    "center": 30.0,
    "approach": 60.0,
    "final_stop": 20.0,
    "verify": 15.0,
}

DEFAULT_CONFIG = {
    "k_yaw": 0.8,
    "k_lateral": 0.5,
    "max_linear": 0.15,
    "max_angular": 0.4,
    "max_linear_y": 0.0,
    "angular_deadband": 2.0,
    "center_deadband_px": 8.0,
    "yaw_deadband_px": 6.0,
    "min_linear": 0.01,
    "min_angular": 0.01,
    "accel_linear": 0.3,
    "accel_angular": 1.2,
    "approach_speed": 0.08,
    "consecutive_frames": 3,
    "max_frame_age": 0.5,
    "chassis_capability": "differential",
    "lidar_enabled": False,
    "safety_stop_distance": 0.25,
    "target_stop_distance": 0.30,
    "verify_duration": 1.0,
    "velocity_threshold": 0.02,
    "frame_reacquire_timeout": 1.0,
    "near_zone_front_y": 380.0,
    "near_zone_loss_is_fatal": True,
}


class Harness:
    def __init__(self, config=None, timeouts=None, camera_yaw_sign=1.0):
        merged = dict(DEFAULT_CONFIG)
        merged.update(config or {})
        self.outputs = []
        self.now = [TIME]
        self.controller = ParkingController(
            self.outputs,
            lambda: self.now[0],
            dict(TIMEOUTS, **(timeouts or {})),
            merged,
        )

    def actions(self, name):
        return [payload for action, payload in self.outputs if action == name]

    def twist(self):
        actions = self.actions("publish_twist")
        return actions[-1] if actions else None

    def observation(self, **overrides):
        payload = {
            "timestamp": self.now[0],
            "frame_detected": True,
            "confidence": 1.0,
            "near_center_x": 320.0,
            "far_center_x": 320.0,
            "left_boundary": 200.0,
            "right_boundary": 440.0,
            "front_boundary_y": 300.0,
            "visible_boundary_count": 4,
        }
        payload.update(overrides)
        return FrameObservation(**payload)

    def start(self, state=None):
        self.controller.start()
        if state == "align":
            return
        # 直接推到位以便测试单个状态
        while self.controller.state != "CENTER_FRAME" and self.controller.state not in (
            "FAILED", "COMPLETE",
        ):
            self.controller.update(self.observation(), None, self.now[0])
            self.now[0] += 0.05
        if state == "center":
            return
        while self.controller.state != "APPROACH_FRAME" and self.controller.state not in (
            "FAILED", "COMPLETE",
        ):
            self.controller.update(self.observation(), None, self.now[0])
            self.now[0] += 0.05
        if state == "approach":
            return
        while self.controller.state != "FINAL_STOP" and self.controller.state not in (
            "FAILED", "COMPLETE",
        ):
            self.controller.update(self.observation(), 0.26, self.now[0])
            self.now[0] += 0.05
        if state == "final_stop":
            return
        while self.controller.state != "VERIFY_STOP" and self.controller.state not in (
            "FAILED", "COMPLETE",
        ):
            self.controller.update(self.observation(), 0.26, self.now[0])
            self.controller.on_odometry_velocity(0.0, 0.0, 0.0, self.now[0])
            self.now[0] += 0.05


class AlignStateTests(unittest.TestCase):
    def test_yaw_error_produces_correct_sign(self):
        h = Harness()
        h.controller.start()
        # far > near -> yaw_error > 0 -> 需要右转（angular.z < 0）
        h.controller.update(
            h.observation(near_center_x=300.0, far_center_x=360.0),
            None,
            h.now[0],
        )
        twist = h.twist()
        self.assertIsNotNone(twist)
        self.assertAlmostEqual(0.0, twist["linear_x"], places=6)
        self.assertLess(twist["angular_z"], 0.0)

    def test_opposite_yaw_error_opposite_sign(self):
        h = Harness()
        h.controller.start()
        h.controller.update(
            h.observation(near_center_x=360.0, far_center_x=300.0),
            None,
            h.now[0],
        )
        self.assertGreater(h.twist()["angular_z"], 0.0)

    def test_angular_saturation(self):
        h = Harness()
        h.controller.start()
        h.controller.update(
            h.observation(near_center_x=100.0, far_center_x=540.0),
            None,
            h.now[0],
        )
        twist = h.twist()
        self.assertEqual(-DEFAULT_CONFIG["max_angular"], twist["angular_z"])

    def test_deadband_zeroes_small_yaw(self):
        h = Harness()
        h.controller.start()
        h.controller.update(
            h.observation(near_center_x=320.0, far_center_x=321.0),
            None,
            h.now[0],
        )
        self.assertEqual(0.0, h.twist()["angular_z"])

    def test_consecutive_frames_required_to_leave_align(self):
        h = Harness()
        h.controller.start()
        for _ in range(2):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("ALIGN_FRAME", h.controller.state)
        h.controller.update(h.observation(), None, h.now[0])
        self.assertEqual("CENTER_FRAME", h.controller.state)

    def test_angular_rate_limited(self):
        h = Harness()
        h.controller.start()
        h.controller.update(
            h.observation(near_center_x=100.0, far_center_x=540.0),
            None,
            h.now[0],
        )
        h.now[0] += 0.05
        h.controller.update(
            h.observation(near_center_x=540.0, far_center_x=100.0),
            None,
            h.now[0],
        )
        # 方向翻转受 accel_angular * dt 限制，不能瞬间从 -max 变 +max
        twist = h.twist()
        self.assertGreater(twist["angular_z"], -DEFAULT_CONFIG["max_angular"])
        self.assertLess(twist["angular_z"], DEFAULT_CONFIG["max_angular"])


class CenterStateTests(unittest.TestCase):
    def test_lateral_error_produces_correct_sign(self):
        h = Harness()
        h.start("center")
        # 纯横向偏移：框在左 -> lateral_error < 0 -> 左转（angular > 0）
        h.controller.update(
            h.observation(near_center_x=280.0, far_center_x=280.0,
                          lateral_error=-40.0),
            None,
            h.now[0],
        )
        twist = h.twist()
        self.assertGreater(twist["angular_z"], 0.0)

    def test_centered_frame_advances_to_approach(self):
        h = Harness()
        h.controller.start()
        for _ in range(3):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("CENTER_FRAME", h.controller.state)
        for _ in range(3):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("APPROACH_FRAME", h.controller.state)

    def test_holonomic_lateral_velocity_only_when_enabled(self):
        h = Harness(config={"chassis_capability": "holonomic", "max_linear_y": 0.2})
        h.start("center")
        h.controller.update(
            h.observation(near_center_x=280.0, far_center_x=280.0,
                          lateral_error=-40.0),
            None,
            h.now[0],
        )
        twist = h.twist()
        self.assertNotEqual(0.0, twist["linear_y"])

    def test_differential_never_emits_lateral_velocity(self):
        h = Harness()
        h.start("center")
        h.controller.update(
            h.observation(near_center_x=280.0, far_center_x=280.0,
                          lateral_error=-40.0),
            None,
            h.now[0],
        )
        self.assertEqual(0.0, h.twist()["linear_y"])


class ApproachStateTests(unittest.TestCase):
    def test_approach_moves_slowly_forward(self):
        h = Harness()
        h.start("approach")
        for _ in range(10):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        twist = h.twist()
        self.assertEqual(DEFAULT_CONFIG["approach_speed"], twist["linear_x"])

    def test_approach_advances_to_final_stop_at_target_distance(self):
        h = Harness()
        h.start("approach")
        for _ in range(3):
            h.controller.update(h.observation(), 0.28, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("FINAL_STOP", h.controller.state)

    def test_far_zone_line_loss_enters_reacquire_with_zero(self):
        h = Harness()
        h.controller.start()
        for _ in range(6):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        # 远区（front_y=300 < near_zone_front_y=380）丢框 → 零速 + 有限重获
        h.controller.update(
            h.observation(frame_detected=False, front_boundary_y=300.0),
            None, h.now[0],
        )
        self.assertEqual(ParkingController.REACQUIRE, h.controller.state)
        zero = h.actions("publish_zero")
        self.assertTrue(zero)
        self.assertEqual([], h.actions("controller_result"))  # 尚未失败

    def test_far_zone_loss_recovers_within_timeout(self):
        h = Harness()
        h.controller.start()
        for _ in range(6):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("APPROACH_FRAME", h.controller.state)
        h.controller.update(
            h.observation(frame_detected=False, front_boundary_y=300.0),
            None, h.now[0],
        )
        self.assertEqual(ParkingController.REACQUIRE, h.controller.state)
        # 宽限内重获 → 回到丢框前状态继续（不失败）
        h.now[0] += 0.1
        h.controller.update(h.observation(), None, h.now[0])
        self.assertEqual("APPROACH_FRAME", h.controller.state)
        self.assertEqual([], h.actions("controller_result"))

    def test_reacquire_timeout_fails_with_zero(self):
        h = Harness()
        h.controller.start()
        for _ in range(6):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        h.controller.update(
            h.observation(frame_detected=False, front_boundary_y=300.0),
            None, h.now[0],
        )
        h.now[0] += 1.1  # 超过 frame_reacquire_timeout=1.0
        h.controller.tick(h.now[0])
        self.assertEqual("FAILED", h.controller.state)
        results = h.actions("controller_result")
        self.assertEqual("frame_lost", results[-1]["status"])
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )

    def test_stale_camera_data_stops_immediately(self):
        h = Harness()
        h.controller.start()
        for _ in range(6):
            h.controller.update(h.observation(), None, h.now[0])
            h.now[0] += 0.05
        h.now[0] += 2.0
        h.controller.update(
            h.observation(timestamp=h.now[0] - 1.2), None, h.now[0]
        )
        # 陈旧帧是数据质量问题：立即失败，不进重获
        self.assertEqual("FAILED", h.controller.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )


class LidarSafetyTests(unittest.TestCase):
    def test_danger_stops_immediately_with_zero(self):
        h = Harness(config={"lidar_enabled": True})
        h.controller.start()
        h.controller.update(h.observation(), 0.10, h.now[0])
        self.assertEqual("FAILED", h.controller.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )
        results = h.actions("controller_result")
        self.assertEqual("lidar_danger", results[-1]["status"])

    def test_lidar_disabled_ignores_front_range(self):
        h = Harness(config={"lidar_enabled": False})
        h.controller.start()
        h.controller.update(h.observation(), 0.10, h.now[0])
        self.assertEqual("ALIGN_FRAME", h.controller.state)
        self.assertEqual([], h.actions("controller_result"))

    def test_invalid_scan_never_treated_as_safe(self):
        # lidar enabled 时无有效扫描（None）不能视为安全：立即零速失败
        h = Harness(config={"lidar_enabled": True})
        h.controller.start()
        h.controller.update(h.observation(), None, h.now[0])
        self.assertEqual("FAILED", h.controller.state)
        results = h.actions("controller_result")
        self.assertEqual("lidar_invalid", results[-1]["status"])
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )


class FinalStopTests(unittest.TestCase):
    def test_final_stop_completes_to_verify(self):
        h = Harness()
        h.start("final_stop")
        self.assertEqual("FINAL_STOP", h.controller.state)
        for _ in range(10):
            h.controller.update(h.observation(), 0.26, h.now[0])
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("VERIFY_STOP", h.controller.state)

    def test_verify_requires_stable_window_and_near_zero_velocity(self):
        h = Harness()
        h.start("final_stop")
        # FINAL_STOP -> VERIFY_STOP 需要稳定
        for _ in range(10):
            h.controller.update(h.observation(), 0.26, h.now[0])
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("VERIFY_STOP", h.controller.state)
        # 速度非零 -> 不完成
        h.controller.on_odometry_velocity(0.05, 0.0, 0.0, h.now[0])
        h.now[0] += 1.2
        h.controller.tick(h.now[0])
        self.assertEqual("VERIFY_STOP", h.controller.state)
        # 零速度持续 -> 完成
        for _ in range(30):
            h.controller.update(h.observation(), 0.26, h.now[0])
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        results = h.actions("controller_result")
        self.assertEqual("ok", results[-1]["status"])

    def test_verify_requires_terminal_condition(self):
        # 对齐 + 零速，但终停条件丢失（雷达距离远离目标）→ 稳定计时清零
        h = Harness()
        h.start("final_stop")
        for _ in range(10):
            h.controller.update(h.observation(), 0.26, h.now[0])
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("VERIFY_STOP", h.controller.state)
        for _ in range(30):
            h.controller.update(h.observation(), 0.5, h.now[0])
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("VERIFY_STOP", h.controller.state)
        results = h.actions("controller_result")
        self.assertFalse(any(r["status"] == "ok" for r in results))


class ParkingSafetyBoundaryTests(unittest.TestCase):
    """Task 6: standalone safety boundaries at the state-machine level.

    Close-range line loss becomes a safe unconfirmed failure instead of a
    blind approach; line loss during verification never reports verified;
    verified additionally requires alignment, not just zero velocity.
    """

    def drive_to_verify(self, h):
        h.start("final_stop")
        for _ in range(10):
            h.controller.update(h.observation(), 0.26, h.now[0])
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("VERIFY_STOP", h.controller.state)

    def test_line_loss_near_target_fails_instead_of_blind_approach(self):
        h = Harness()
        h.start("approach")
        # 前白线已接近视觉停止阈值（420.0），仍差最后几像素
        for _ in range(2):
            h.controller.update(
                h.observation(front_boundary_y=419.0), None, h.now[0]
            )
            h.now[0] += 0.05
        self.assertEqual("APPROACH_FRAME", h.controller.state)
        # 近区（front_y=419 >= near_zone_front_y=380）丢框 → 立即失败
        h.controller.update(
            h.observation(frame_detected=False, front_boundary_y=419.0),
            None, h.now[0],
        )
        self.assertEqual("FAILED", h.controller.state)
        results = h.actions("controller_result")
        self.assertEqual("frame_lost_near", results[-1]["status"])
        # 零命令必须先于失败结果，且之后不再有运动命令
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )
        self.assertEqual(actions[-1], "controller_result")

    def test_near_zone_loss_flag_is_fatal_without_current_geometry(self):
        h = Harness()
        h.start("approach")
        h.controller.update(
            h.observation(
                frame_detected=False,
                front_boundary_y=None,
                near_zone=False,
                near_zone_loss=True,
            ),
            None,
            h.now[0],
        )
        self.assertEqual("FAILED", h.controller.state)
        self.assertEqual(
            "frame_lost_near",
            h.actions("controller_result")[-1]["status"],
        )
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )

    def test_line_loss_during_verify_never_reports_verified(self):
        h = Harness()
        self.drive_to_verify(h)
        h.controller.update(
            h.observation(frame_detected=False), None, h.now[0]
        )
        self.assertEqual("FAILED", h.controller.state)
        results = h.actions("controller_result")
        self.assertTrue(results)
        self.assertFalse(any(r["status"] == "ok" for r in results))

    def test_verify_requires_alignment_not_only_zero_velocity(self):
        h = Harness()
        self.drive_to_verify(h)
        # 零速但未对齐（yaw_error 大）：不完成
        for _ in range(30):
            h.controller.update(
                h.observation(near_center_x=280.0, far_center_x=360.0),
                0.26, h.now[0],
            )
            h.controller.on_odometry_velocity(0.0, 0.0, 0.0, h.now[0])
            h.now[0] += 0.05
        self.assertEqual("VERIFY_STOP", h.controller.state)
        results = h.actions("controller_result")
        self.assertFalse(any(r["status"] == "ok" for r in results))


class TerminationTests(unittest.TestCase):
    def test_timeout_fails_with_zero(self):
        h = Harness()
        h.controller.start()
        h.now[0] += 31.0
        h.controller.tick(h.now[0])
        self.assertEqual("FAILED", h.controller.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )

    def test_cancel_fails_with_zero(self):
        h = Harness()
        h.controller.start()
        h.controller.cancel("operator")
        self.assertEqual("FAILED", h.controller.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero"), actions.index("controller_result")
        )
        results = h.actions("controller_result")
        self.assertEqual("cancelled", results[-1]["status"])

    def test_shutdown_guarantees_zero(self):
        h = Harness()
        h.controller.start()
        h.controller.shutdown()
        self.assertEqual("FAILED", h.controller.state)
        actions = [name for name, _ in h.outputs]
        self.assertEqual(actions[-2], "publish_zero")

    def test_align_timeout_reported(self):
        h = Harness()
        h.controller.start()
        h.now[0] += 31.0
        h.controller.tick(h.now[0])
        results = h.actions("controller_result")
        self.assertIn("ALIGN", results[-1]["message"])


if __name__ == "__main__":
    unittest.main()
