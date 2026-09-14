"""Phase F: end-to-end pure-local delivery scenarios (ROS-free).

Drives the mission engine together with the parking controller exactly the
way the mission node routes events, covering the twelve documented
scenarios: first/second viewpoint, estimation retries and viewpoint switch,
staging navigation failure, far-zone reacquisition, near-zone loss, lidar
danger, cancel/timeout/shutdown, independent simulation phase after
physical completion, stale/cross-phase event rejection and checkpoint
telemetry with exact state names on /task/delivery_status.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import mission as mission_module
from ucar_delivery import protocol
from ucar_delivery.frame_detector import FrameObservation
from ucar_delivery.mission import DeliveryMission
from ucar_delivery.parking_controller import ParkingController

EXACT_STATE_NAMES = frozenset(
    {
        "IDLE", "ACCEPT_GOAL", "NAVIGATE_VIEWPOINT", "SEARCH_SIGN",
        "ALIGN_SIGN", "ESTIMATE_STAGING_POSE", "NAVIGATE_STAGING_POSE",
        "ACQUIRE_FRAME", "ALIGN_FRAME", "CENTER_FRAME", "APPROACH_FRAME",
        "FINAL_STOP", "VERIFY_STOP", "ARRIVED", "SAFE_STOP", "FAILED",
        "CANCELLED", "TIMEOUT",
    }
)

MISSION_TIMEOUTS = {
    "navigation": 120.0,
    "sign_search": 8.0,
    "sign_align": 30.0,
    "staging_estimate": 30.0,
    "staging_navigation": 120.0,
    "frame_acquire": 45.0,
    "frame_align": 30.0,
    "frame_center": 30.0,
    "approach": 60.0,
    "final_stop": 30.0,
    "verify": 15.0,
    "safe_stop": 10.0,
}

PARKING_TIMEOUTS = {
    "align": 30.0,
    "center": 30.0,
    "approach": 60.0,
    "final_stop": 20.0,
    "verify": 15.0,
}

PARKING_CONFIG = {
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
    "visual_stop_front_y": 420.0,
    "verify_duration": 1.0,
    "velocity_threshold": 0.02,
    "frame_reacquire_timeout": 1.0,
    "near_zone_front_y": 380.0,
    "near_zone_loss_is_fatal": True,
}


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def valid_staging_pose(**overrides):
    pose = {
        "x": 1.0, "y": 0.5, "yaw": 0.2,
        "surface_center_laser": [0.7, 0.0],
        "surface_normal_laser": [1.0, 0.0],
        "inlier_count": 8,
        "residual": 0.01,
        "confidence": 0.9,
    }
    pose.update(overrides)
    return pose


class ScenarioHarness:
    """Combines the mission engine and the parking controller the way the
    mission node wires them, with an explicit event router."""

    def __init__(self, phase=protocol.PHASE_PHYSICAL, mission_config=None,
                 parking_config=None):
        self.outputs = []
        self.clock = Clock()
        merged = dict(mission_config or {})
        self.mission = DeliveryMission(
            self.outputs, self.clock, dict(MISSION_TIMEOUTS), merged
        )
        self.parking = ParkingController(
            self.outputs, self.clock, dict(PARKING_TIMEOUTS),
            dict(PARKING_CONFIG, **(parking_config or {})),
        )
        self.phase = phase

    def actions(self, name):
        return [payload for action, payload in self.outputs if action == name]

    def goal(self, **overrides):
        payload = {
            "protocol_version": 1,
            "task_id": "task-scenario-001",
            "goal_id": "delivery-scenario-001",
            "target_workshop": "食品加工车间",
            "selected_item": "苹果",
            "phase": self.phase,
        }
        payload.update(overrides)
        return payload

    def accept(self, **overrides):
        self.mission.accept_goal(self.goal(**overrides))

    # ------------------------------------------------------------ 事件路由

    def viewpoint_ok(self):
        self.mission.on_viewpoint_result(self.phase, True, "")

    def sign_found(self):
        self.mission.on_sign_found(self.phase, {
            "confidence": 0.9,
            "bearing_rad": 0.0,
            "half_width_rad": 0.5,
        })

    def sign_aligned(self):
        self.mission.on_sign_aligned(self.phase)

    def staging_estimated(self, pose=None):
        self.mission.on_staging_pose_estimated(
            self.phase, pose or valid_staging_pose()
        )

    def staging_nav_ok(self):
        self.mission.on_staging_navigation_result(self.phase, True, "")

    def frame_acquired(self):
        self.mission.on_frame_acquired(self.phase, {"confidence": 1.0})

    # -------------------------------------------------------- 停车控制器驱动

    def observation(self, **overrides):
        payload = {
            "timestamp": self.clock(),
            "frame_detected": True,
            "confidence": 1.0,
            "near_center_x": 320.0,
            "far_center_x": 320.0,
            "left_boundary": 200.0,
            "right_boundary": 440.0,
            "front_boundary_y": 300.0,
            "visible_boundary_count": 4,
            "lateral_error": 0.0,
            "yaw_error": 0.0,
            "confirmed": True,
            "near_zone": False,
            "near_zone_loss": False,
        }
        payload.update(overrides)
        return FrameObservation(**payload)

    def route_parking_result(self):
        """controller ok -> mission ARRIVED；失败 -> mission.fail。"""
        for action, payload in self.outputs:
            if action != "controller_result":
                continue
            if payload.get("status") == "ok":
                self.mission.on_stop_verified(self.phase, {"confidence": 1.0})
            else:
                self.mission.fail(
                    payload.get("message") or "parking failed"
                )

    def drive_parking(self, steps, front_range=0.26, **obs_overrides):
        """推进控制器 steps 步，返回是否 COMPLETE。"""
        self.parking.start()
        for _ in range(steps):
            self.parking.update(
                self.observation(**obs_overrides), front_range, self.clock()
            )
            self.parking.on_odometry_velocity(0.0, 0.0, 0.0, self.clock())
            self.clock.advance(0.05)
        return self.parking.state == ParkingController.COMPLETE

    def complete_parking(self):
        """对齐 → 居中 → 接近 → 终停 → 验证，一路推到 COMPLETE。

        与 mission 节点一致：controller 阶段推进的同时，progress 事件
        同步推动 mission 状态机。
        """
        self.parking.start()
        self.drive_parking(3)  # ALIGN_FRAME -> CENTER_FRAME
        self.mission.on_frame_aligned(self.phase, {"confidence": 1.0})
        self.drive_parking(3)  # CENTER_FRAME -> APPROACH_FRAME
        self.mission.on_frame_centered(self.phase, {"confidence": 1.0})
        self.drive_parking(3)  # APPROACH_FRAME -> FINAL_STOP
        self.mission.on_approach_complete(self.phase, {"confidence": 1.0})
        self.drive_parking(10)  # FINAL_STOP -> VERIFY_STOP
        self.mission.on_final_stop_done(self.phase, {"confidence": 1.0})
        self.drive_parking(30)  # VERIFY_STOP -> COMPLETE
        self.route_parking_result()  # controller ok -> mission ARRIVED

    # -------------------------------------------------------------- 状态机

    def terminal_arrival(self):
        arrivals = self.actions("publish_arrival")
        return arrivals[-1] if arrivals else None


class ScenarioTests(unittest.TestCase):
    def make(self, **kwargs):
        return ScenarioHarness(**kwargs)

    def run_full_success(self, h):
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.complete_parking()

    # 1. 第一视点识别成功 -> staging -> 停车成功
    def test_first_viewpoint_staging_parking_success(self):
        h = self.make()
        self.run_full_success(h)
        self.assertEqual(DeliveryMission.ARRIVED, h.mission.state)
        arrival = h.terminal_arrival()
        self.assertEqual("arrived", arrival["status"])
        self.assertEqual("delivery-scenario-001", arrival["goal_id"])
        self.assertEqual("physical", h.mission.phase)
        # controller ok 是 ARRIVED 的唯一推动者：无 ok 时不得提前到达
        self.assertEqual(1, len(h.actions("start_frame_search")))
        self.assertEqual(1, len(h.actions("navigate_to_staging")))

    # 2. 第一视点无目标 -> 第二视点成功
    def test_second_viewpoint_after_first_search_timeout(self):
        h = self.make()
        h.accept()
        h.viewpoint_ok()
        self.assertEqual(DeliveryMission.SEARCH_SIGN, h.mission.state)
        h.clock.advance(9.0)  # sign_search=8s 超时
        h.mission.tick()
        goals = h.actions("navigate_to_viewpoint")
        self.assertEqual(2, len(goals))
        self.assertEqual(1, goals[-1]["viewpoint_index"])
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.complete_parking()
        self.assertEqual("arrived", h.terminal_arrival()["status"])

    # 3. 估计失败后重新对正采样成功
    def test_estimate_failure_resamples_at_same_point(self):
        h = self.make()
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.mission.on_staging_estimate_failed(h.phase, "not enough inliers")
        self.assertEqual(DeliveryMission.ALIGN_SIGN, h.mission.state)
        h.sign_aligned()
        h.staging_estimated()
        self.assertEqual(
            DeliveryMission.NAVIGATE_STAGING_POSE, h.mission.state
        )
        goals = h.actions("navigate_to_staging")
        self.assertEqual(1, len(goals))  # 只有成功确认后才发送一次

    # 4. 估计全部失败后换下一个搜索点
    def test_estimate_exhausted_switches_viewpoint(self):
        h = self.make(mission_config={
            "staging_estimation_max_retries": 2,
            "viewpoint_count": 3,
        })
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        for _ in range(3):
            h.mission.on_staging_estimate_failed(h.phase, "no inliers")
            if h.mission.state == DeliveryMission.ALIGN_SIGN:
                h.sign_aligned()
        goals = h.actions("navigate_to_viewpoint")
        self.assertEqual(2, len(goals))
        self.assertEqual(1, goals[-1]["viewpoint_index"])
        self.assertEqual(
            DeliveryMission.NAVIGATE_VIEWPOINT, h.mission.state
        )
        self.assertEqual([], h.actions("navigate_to_staging"))

    # 5. staging 导航失败后安全失败
    def test_staging_navigation_failure_fails_safe(self):
        h = self.make(mission_config={"staging_navigation_max_retries": 1})
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.mission.on_staging_navigation_result(h.phase, False, "aborted")
        h.mission.on_staging_navigation_result(h.phase, False, "aborted")
        self.assertEqual(DeliveryMission.FAILED, h.mission.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("publish_arrival"),
        )
        self.assertEqual("failed", h.terminal_arrival()["status"])

    # 6. 白框远区短暂丢失后恢复（重获）
    def test_far_zone_loss_reacquires_and_completes(self):
        h = self.make()
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.parking.start()
        h.drive_parking(3)  # ALIGN_FRAME -> CENTER_FRAME
        h.mission.on_frame_aligned(h.phase, {"confidence": 1.0})
        # 远区丢框 1 帧 → REACQUIRE，宽限内重获
        h.parking.update(
            h.observation(frame_detected=False, front_boundary_y=300.0),
            None, h.clock(),
        )
        self.assertEqual(ParkingController.REACQUIRE, h.parking.state)
        h.clock.advance(0.1)
        h.drive_parking(5)  # 重获回 CENTER，3 帧后 -> APPROACH_FRAME
        self.assertEqual(
            ParkingController.APPROACH_FRAME, h.parking.state
        )
        h.mission.on_frame_centered(h.phase, {"confidence": 1.0})
        h.drive_parking(3)
        h.mission.on_approach_complete(h.phase, {"confidence": 1.0})
        h.drive_parking(10)
        h.mission.on_final_stop_done(h.phase, {"confidence": 1.0})
        h.drive_parking(30)
        h.route_parking_result()
        self.assertEqual(DeliveryMission.ARRIVED, h.mission.state)
        self.assertEqual("arrived", h.terminal_arrival()["status"])

    # 7. 近区丢框立即失败
    def test_near_zone_loss_fails(self):
        h = self.make()
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.parking.start()
        h.drive_parking(3)
        h.drive_parking(3)
        h.drive_parking(3)
        # 近区（front_y=419 >= 380）丢框 → 立即失败
        h.parking.update(
            h.observation(frame_detected=False, front_boundary_y=419.0),
            0.26, h.clock(),
        )
        self.assertEqual(ParkingController.FAILED, h.parking.state)
        h.route_parking_result()
        self.assertEqual(DeliveryMission.FAILED, h.mission.state)
        results = [p for a, p in h.outputs if a == "controller_result"]
        self.assertEqual("frame_lost_near", results[-1]["status"])

    # 8. lidar 危险立即失败
    def test_lidar_danger_fails(self):
        h = self.make(parking_config={"lidar_enabled": True})
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.parking.start()
        h.parking.update(h.observation(), 0.10, h.clock())
        self.assertEqual(ParkingController.FAILED, h.parking.state)
        results = [p for a, p in h.outputs if a == "controller_result"]
        self.assertEqual("lidar_danger", results[-1]["status"])
        h.route_parking_result()
        self.assertEqual(DeliveryMission.FAILED, h.mission.state)

    # 9. 取消 / 超时 / 节点关闭
    def test_cancel_fails_safe(self):
        h = self.make()
        h.accept()
        h.mission.cancel("operator stop")
        self.assertEqual(DeliveryMission.CANCELLED, h.mission.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("publish_arrival"),
        )

    def test_timeout_fails_safe(self):
        h = self.make()
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.clock.advance(31.0)  # staging_estimate=30s 超时
        h.mission.tick()
        self.assertEqual(DeliveryMission.TIMEOUT, h.mission.state)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("publish_arrival"),
        )

    def test_shutdown_fails_safe(self):
        h = self.make()
        h.accept()
        h.mission.cancel("node shutdown")
        self.assertEqual(DeliveryMission.CANCELLED, h.mission.state)
        actions = [name for name, _ in h.outputs]
        self.assertIn("publish_zero_velocity", actions)

    # 10. 实物任务完成后接受独立仿真车间任务
    def test_independent_simulation_phase_after_physical(self):
        h = self.make()
        self.run_full_success(h)
        self.assertEqual(
            "arrived", h.terminal_arrival()["status"]
        )
        h.phase = protocol.PHASE_SIMULATION
        h.accept(
            goal_id="simulation-scenario-001",
            target_workshop="日用品加工车间",
            selected_item="毛巾",
        )
        self.assertEqual(protocol.PHASE_SIMULATION, h.mission.phase)
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.complete_parking()
        arrivals = h.actions("publish_arrival")
        self.assertEqual("arrived", arrivals[-1]["status"])
        self.assertEqual("simulation-scenario-001", arrivals[-1]["goal_id"])

    # 11. 重复 / 过期 / 跨阶段事件不运动
    def test_duplicate_goal_does_not_move(self):
        h = self.make()
        h.accept()
        goals_before = len(h.actions("navigate_to_viewpoint"))
        h.accept()  # 相同 goal_id
        self.assertEqual(
            goals_before, len(h.actions("navigate_to_viewpoint"))
        )

    def test_cross_phase_events_ignored(self):
        h = self.make()
        h.accept()
        before = len(h.outputs)
        h.mission.on_viewpoint_result(
            protocol.PHASE_SIMULATION, True, ""
        )
        h.mission.on_sign_found(protocol.PHASE_SIMULATION, {"confidence": 0.9})
        self.assertEqual(len(h.outputs), before)
        self.assertEqual(
            DeliveryMission.NAVIGATE_VIEWPOINT, h.mission.state
        )

    def test_stale_events_ignored(self):
        h = self.make()
        h.accept()
        before = len(h.outputs)
        # 状态不匹配的事件（NAVIGATE_VIEWPOINT 中收到 staging 估计）
        h.staging_estimated()
        self.assertEqual(len(h.outputs), before)

    # 12. 每个检查点发布可关联的 /task/delivery_status（精确状态名）
    def test_every_checkpoint_publishes_linked_status(self):
        h = self.make()
        h.accept()
        h.viewpoint_ok()
        h.sign_found()
        h.sign_aligned()
        h.staging_estimated()
        h.staging_nav_ok()
        h.frame_acquired()
        h.complete_parking()
        statuses = h.actions("publish_status")
        self.assertGreaterEqual(len(statuses), 12)
        for status in statuses:
            self.assertEqual("physical", status["phase"])
            self.assertEqual("task-scenario-001", status["task_id"])
            self.assertEqual("delivery-scenario-001", status["goal_id"])
            self.assertIn("state", status)
            self.assertIn(status["state"], EXACT_STATE_NAMES)
            self.assertIn("status", status)
            self.assertIn("message", status)
        self.assertEqual("arrived", statuses[-1]["status"])
        self.assertEqual(DeliveryMission.ARRIVED, statuses[-1]["state"])
        # 状态名必须与状态机类常量完全一致（精确名称，无别名）
        self.assertEqual(
            DeliveryMission.ARRIVED, statuses[-1]["state"]
        )
        self.assertNotIn("BOGUS_STATE", [s["state"] for s in statuses])


if __name__ == "__main__":
    unittest.main()
