"""Phase A: lock the reusable dual-phase delivery mission state machine.

The same engine instance runs both the physical and the simulation phase.
Every failure, timeout and cancellation must pass through SAFE_STOP and emit
a guaranteed zero-velocity command before reaching a terminal state.
Cross-phase events must be ignored.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import mission, protocol

DEFAULT_TIMEOUTS = {
    "navigation": 120.0,
    "sign_search": 60.0,
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


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Harness:
    def __init__(self, phase=protocol.PHASE_PHYSICAL, timeouts=None, **config):
        self.outputs = []
        self.clock = Clock()
        self.config = dict(config)
        self.engine = mission.DeliveryMission(
            self.outputs, self.clock, timeouts or dict(DEFAULT_TIMEOUTS), self.config
        )
        self.phase = phase

    def actions(self, action):
        return [payload for name, payload in self.outputs if name == action]

    def goal(self, **overrides):
        payload = {
            "protocol_version": 1,
            "task_id": "task-test-001",
            "goal_id": "delivery-test-001",
            "target_workshop": "食品加工车间",
            "selected_item": "苹果",
            "phase": self.phase,
        }
        payload.update(overrides)
        return payload

    def accept(self, **overrides):
        self.engine.accept_goal(self.goal(**overrides))

    def run_full_success(self):
        self.accept()
        self.engine.on_viewpoint_result(self.phase, True, "")
        self.engine.on_sign_found(self.phase, {"confidence": 0.9})
        self.engine.on_sign_aligned(self.phase)
        self.engine.on_staging_pose_estimated(
            self.phase, self.valid_staging_pose()
        )
        self.engine.on_staging_navigation_result(self.phase, True, "")
        self.engine.on_frame_acquired(self.phase, {"confidence": 0.8})
        self.engine.on_frame_aligned(self.phase, {"confidence": 0.8})
        self.engine.on_frame_centered(self.phase, {"confidence": 0.8})
        self.engine.on_approach_complete(self.phase, {"confidence": 0.8})
        self.engine.on_final_stop_done(self.phase, {"confidence": 0.8})
        self.engine.on_stop_verified(self.phase, {"confidence": 0.8})

    def run_success_arrival(self):
        self.run_full_success()
        return self.actions("publish_arrival")

    def valid_staging_pose(self, **overrides):
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


def navigation_actions(h):
    return [
        (name, payload)
        for name, payload in h.outputs
        if name == "navigate_to_viewpoint"
    ]


class AcceptanceTests(unittest.TestCase):
    def test_accept_goal_starts_first_viewpoint_navigation(self):
        h = Harness()
        h.accept()
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)
        self.assertEqual(h.engine.phase, "physical")
        goals = navigation_actions(h)
        self.assertEqual(len(goals), 1)
        self.assertEqual(goals[0][1]["viewpoint_index"], 0)
        self.assertEqual(goals[0][1]["phase"], "physical")
        self.assertEqual(goals[0][1]["task_id"], "task-test-001")
        self.assertEqual(goals[0][1]["goal_id"], "delivery-test-001")

    def test_simulation_phase_goal_accepted(self):
        h = Harness(phase=protocol.PHASE_SIMULATION)
        h.accept(goal_id="simulation-delivery-test-001")
        self.assertEqual(h.engine.phase, "simulation")
        goals = navigation_actions(h)
        self.assertEqual(goals[0][1]["phase"], "simulation")

    def test_motion_mode_navigation_on_accept(self):
        h = Harness()
        h.accept()
        modes = h.actions("publish_motion_mode")
        self.assertIn("NAVIGATION", modes)

    def test_duplicate_goal_ignored(self):
        h = Harness()
        h.accept()
        goals_before = len(navigation_actions(h))
        h.accept()
        self.assertEqual(len(navigation_actions(h)), goals_before)
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)

    def test_new_goal_while_active_rejected(self):
        h = Harness()
        h.accept()
        goals_before = len(navigation_actions(h))
        h.accept(task_id="task-other")
        self.assertEqual(len(navigation_actions(h)), goals_before)
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)
        self.assertEqual(h.engine.goal["task_id"], "task-test-001")

    def test_new_goal_after_mission_end_accepted(self):
        h = Harness()
        h.run_full_success()
        arrivals = h.actions("publish_arrival")
        self.assertEqual(arrivals[-1]["status"], "arrived")
        h.accept(
            goal_id="second-goal",
            task_id="task-test-001",
            target_workshop="电子产品生产车间",
            selected_item="手机",
        )
        self.assertEqual(h.engine.goal["goal_id"], "second-goal")
        self.assertEqual(h.engine.phase, "physical")

    def test_unknown_workshop_goal_rejected_before_engine(self):
        h = Harness()
        with self.assertRaises(protocol.ProtocolError):
            h.engine.accept_goal(
                h.goal(target_workshop="不存在的车间")
            )
        self.assertEqual(h.engine.state, mission.DeliveryMission.IDLE)


class ViewpointTraversalTests(unittest.TestCase):
    phase = protocol.PHASE_PHYSICAL
    def test_next_viewpoint_after_search_exhausts_local_viewpoint(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        goals = navigation_actions(h)
        self.assertEqual(len(goals), 2)
        self.assertEqual(goals[1][1]["viewpoint_index"], 1)
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)

    def test_viewpoint_navigation_retry_within_budget(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, False, "aborted")
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)
        goals = navigation_actions(h)
        self.assertEqual(len(goals), 2)
        self.assertEqual(goals[1][1]["viewpoint_index"], 0)
        self.assertEqual(goals[1][1]["attempt"], 2)

    def test_viewpoint_navigation_retries_exhausted_fails(self):
        h = Harness(viewpoint_max_retries=2)
        h.accept()
        h.engine.on_viewpoint_result(self.phase, False, "aborted")
        h.engine.on_viewpoint_result(self.phase, False, "aborted")
        h.engine.on_viewpoint_result(self.phase, False, "aborted")
        self.assertEqual(h.engine.state, mission.DeliveryMission.FAILED)
        arrivals = h.actions("publish_arrival")
        self.assertEqual(arrivals[-1]["status"], "failed")

    def test_all_viewpoints_exhausted_fails(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        self.assertEqual(h.engine.state, mission.DeliveryMission.FAILED)
        arrivals = h.actions("publish_arrival")
        self.assertEqual(arrivals[-1]["message"], "sign not found at any viewpoint")


class SuccessPathTests(unittest.TestCase):
    phase = protocol.PHASE_PHYSICAL
    def test_full_physical_success_flow(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        self.assertEqual(h.engine.state, mission.DeliveryMission.SEARCH_SIGN)
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        self.assertEqual(h.engine.state, mission.DeliveryMission.ALIGN_SIGN)
        h.engine.on_sign_aligned(self.phase)
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.ESTIMATE_STAGING_POSE
        )
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_STAGING_POSE
        )
        h.engine.on_staging_navigation_result(self.phase, True, "")
        self.assertEqual(h.engine.state, mission.DeliveryMission.ACQUIRE_FRAME)
        h.engine.on_frame_acquired(self.phase, {"confidence": 0.8})
        self.assertEqual(h.engine.state, mission.DeliveryMission.ALIGN_FRAME)
        h.engine.on_frame_aligned(self.phase, {"confidence": 0.8})
        self.assertEqual(h.engine.state, mission.DeliveryMission.CENTER_FRAME)
        h.engine.on_frame_centered(self.phase, {"confidence": 0.8})
        self.assertEqual(h.engine.state, mission.DeliveryMission.APPROACH_FRAME)
        h.engine.on_approach_complete(self.phase, {"confidence": 0.8})
        self.assertEqual(h.engine.state, mission.DeliveryMission.FINAL_STOP)
        h.engine.on_final_stop_done(self.phase, {"confidence": 0.8})
        self.assertEqual(h.engine.state, mission.DeliveryMission.VERIFY_STOP)
        h.engine.on_stop_verified(self.phase, {"confidence": 0.8})
        self.assertEqual(h.engine.state, mission.DeliveryMission.ARRIVED)
        arrivals = h.actions("publish_arrival")
        self.assertEqual(len(arrivals), 1)
        self.assertEqual(arrivals[0]["status"], "arrived")
        self.assertEqual(arrivals[0]["task_id"], "task-test-001")
        self.assertEqual(arrivals[0]["goal_id"], "delivery-test-001")

    def test_full_simulation_success_flow(self):
        h = Harness(phase=protocol.PHASE_SIMULATION)
        h.accept(goal_id="simulation-delivery-test-001")
        h.engine.on_viewpoint_result(h.phase, True, "")
        h.engine.on_sign_found(h.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(h.phase)
        h.engine.on_staging_pose_estimated(h.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(h.phase, True, "")
        h.engine.on_frame_acquired(h.phase, {"confidence": 0.8})
        h.engine.on_frame_aligned(h.phase, {"confidence": 0.8})
        h.engine.on_frame_centered(h.phase, {"confidence": 0.8})
        h.engine.on_approach_complete(h.phase, {"confidence": 0.8})
        h.engine.on_final_stop_done(h.phase, {"confidence": 0.8})
        h.engine.on_stop_verified(h.phase, {"confidence": 0.8})
        arrivals = h.actions("publish_arrival")
        self.assertEqual(arrivals[-1]["goal_id"], "simulation-delivery-test-001")
        self.assertEqual(arrivals[-1]["status"], "arrived")

    def test_sign_detection_preserves_capture(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        detection = {
            "confidence": 0.93,
            "capture_timestamp": 123.4,
            "bbox": [10, 20, 30, 40],
            "observed_pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
            "detection_yaw": 0.2,
        }
        h.engine.on_sign_found(self.phase, detection)
        self.assertEqual(h.engine.detection, detection)
        aligns = h.actions("align_sign")
        self.assertEqual(len(aligns), 1)

    def test_same_engine_reused_for_simulation_after_physical(self):
        h = Harness()
        h.run_full_success()
        self.assertEqual(h.engine.state, mission.DeliveryMission.ARRIVED)
        h.accept(
            phase=protocol.PHASE_SIMULATION,
            goal_id="simulation-delivery-test-001",
            target_workshop="日用品加工车间",
            selected_item="毛巾",
        )
        h.engine.on_viewpoint_result(protocol.PHASE_SIMULATION, True, "")
        h.engine.on_sign_found(protocol.PHASE_SIMULATION, {"confidence": 0.9})
        h.engine.on_sign_aligned(protocol.PHASE_SIMULATION)
        h.engine.on_staging_pose_estimated(
            protocol.PHASE_SIMULATION, h.valid_staging_pose()
        )
        h.engine.on_staging_navigation_result(protocol.PHASE_SIMULATION, True, "")
        h.engine.on_frame_acquired(protocol.PHASE_SIMULATION, {"confidence": 0.8})
        h.engine.on_frame_aligned(protocol.PHASE_SIMULATION, {"confidence": 0.8})
        h.engine.on_frame_centered(protocol.PHASE_SIMULATION, {"confidence": 0.8})
        h.engine.on_approach_complete(protocol.PHASE_SIMULATION, {"confidence": 0.8})
        h.engine.on_final_stop_done(protocol.PHASE_SIMULATION, {"confidence": 0.8})
        h.engine.on_stop_verified(protocol.PHASE_SIMULATION, {"confidence": 0.8})
        arrivals = h.actions("publish_arrival")
        self.assertEqual(arrivals[-1]["goal_id"], "simulation-delivery-test-001")


class CrossPhaseIsolationTests(unittest.TestCase):
    phase = protocol.PHASE_PHYSICAL
    def test_physical_events_ignored_during_simulation_phase(self):
        h = Harness()
        h.run_full_success()
        h.accept(
            phase=protocol.PHASE_SIMULATION,
            goal_id="simulation-delivery-test-001",
            target_workshop="日用品加工车间",
            selected_item="毛巾",
        )
        before = len(h.outputs)
        h.engine.on_sign_found(protocol.PHASE_PHYSICAL, {"confidence": 0.9})
        h.engine.on_frame_acquired(protocol.PHASE_PHYSICAL, {"confidence": 0.8})
        self.assertEqual(len(h.outputs), before)
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT
        )

    def test_simulation_events_ignored_during_physical_phase(self):
        h = Harness()
        h.accept()
        before = len(h.outputs)
        h.engine.on_sign_found(protocol.PHASE_SIMULATION, {"confidence": 0.9})
        h.engine.on_viewpoint_result(protocol.PHASE_SIMULATION, True, "")
        self.assertEqual(len(h.outputs), before)


class FailurePathTests(unittest.TestCase):
    phase = protocol.PHASE_PHYSICAL
    def assert_safe_stop_before_terminal(self, h, terminal_state):
        actions = [name for name, _ in h.outputs]
        self.assertIn("publish_zero_velocity", actions)
        self.assertIn("publish_safe_stop", actions)
        self.assertLess(
            actions.index("publish_safe_stop"),
            actions.index("publish_arrival"),
        )
        self.assertEqual(h.engine.state, terminal_state)
        arrivals = h.actions("publish_arrival")
        self.assertEqual(arrivals[-1]["status"], "failed")

    def test_sign_not_found_fails(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.clock.advance(61.0)
        h.engine.tick()
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.FAILED)

    def test_sign_align_timeout_fails(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.clock.advance(31.0)
        h.engine.tick()
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.TIMEOUT)

    def test_frame_acquire_timeout_fails(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(self.phase, True, "")
        h.clock.advance(46.0)
        h.engine.tick()
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.TIMEOUT)

    def test_approach_timeout_fails(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(self.phase, True, "")
        h.engine.on_frame_acquired(self.phase, {"confidence": 0.8})
        h.engine.on_frame_aligned(self.phase, {"confidence": 0.8})
        h.engine.on_frame_centered(self.phase, {"confidence": 0.8})
        h.clock.advance(61.0)
        h.engine.tick()
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.TIMEOUT)

    def test_explicit_fail_goes_through_safe_stop(self):
        h = Harness()
        h.accept()
        h.engine.fail("frame lost near wall")
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.FAILED)

    def test_lidar_danger_fails(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(self.phase, True, "")
        h.engine.on_frame_acquired(self.phase, {"confidence": 0.8})
        h.engine.on_lidar_danger(self.phase, 0.12)
        self.assertEqual(h.engine.state, mission.DeliveryMission.FAILED)
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_safe_stop"), actions.index("publish_arrival")
        )
        self.assertEqual(h.engine.last_stop_reason, "lidar_safety")

    def test_cancel_goes_through_safe_stop(self):
        h = Harness()
        h.accept()
        h.engine.cancel("operator cancelled")
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.CANCELLED)

    def test_cancel_ignored_when_terminal(self):
        h = Harness()
        h.run_full_success()
        before = len(h.outputs)
        h.engine.cancel("late cancel")
        self.assertEqual(len(h.outputs), before)
        self.assertEqual(h.engine.state, mission.DeliveryMission.ARRIVED)

    def test_different_goal_resets_failed_mission(self):
        h = Harness()
        h.accept()
        h.engine.fail("test failure")
        self.assertEqual(h.engine.state, mission.DeliveryMission.FAILED)
        h.accept(
            goal_id="after-failure-goal",
            task_id="task-test-002",
            target_workshop="电子产品生产车间",
            selected_item="手机",
        )
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)
        self.assertEqual(h.engine.goal["goal_id"], "after-failure-goal")
        goals = navigation_actions(h)
        self.assertEqual(goals[-1][1]["goal_id"], "after-failure-goal")

    def test_repeated_terminal_goal_does_not_restart(self):
        h = Harness()
        h.accept()
        h.engine.fail("test failure")
        goals_before = len(navigation_actions(h))
        h.accept()  # 相同 (phase, task_id, goal_id)
        self.assertEqual(h.engine.state, mission.DeliveryMission.FAILED)
        self.assertEqual(len(navigation_actions(h)), goals_before)

    def test_repeated_arrived_goal_republishes_without_restart(self):
        h = Harness()
        h.run_full_success()
        arrivals_before = len(h.actions("publish_arrival"))
        goals_before = len(navigation_actions(h))
        h.accept()  # 相同 identity 的已到达目标
        self.assertEqual(h.engine.state, mission.DeliveryMission.ARRIVED)
        self.assertEqual(len(h.actions("publish_arrival")), arrivals_before)

        self.assertEqual(len(navigation_actions(h)), goals_before)

    def test_reset_then_different_goal_still_works(self):
        h = Harness()
        h.accept()
        h.engine.fail("test failure")
        h.engine.reset()
        h.accept(goal_id="after-reset-goal")
        self.assertEqual(h.engine.goal["goal_id"], "after-reset-goal")


class MotionModeMappingTests(unittest.TestCase):
    """Task 2: standalone mode vocabulary.

    NAVIGATE_VIEWPOINT -> NAVIGATION; SEARCH_SIGN / ALIGN_SIGN ->
    VISUAL_SEARCH; ACQUIRE_FRAME through VERIFY_STOP -> PARKING; terminal
    and SAFE_STOP -> IDLE. The pure mission module must stay independent of
    task_orchestrator.
    """

    phase = protocol.PHASE_PHYSICAL

    def test_search_sign_maps_to_visual_search(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        modes = h.actions("publish_motion_mode")
        self.assertIn("VISUAL_SEARCH", modes)
        self.assertNotIn("PARKING", modes)

    def test_align_sign_maps_to_visual_search(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        modes = h.actions("publish_motion_mode")
        self.assertIn("VISUAL_SEARCH", modes)
        self.assertNotIn("PARKING", modes)

    def test_acquire_frame_and_later_states_map_to_parking(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(self.phase, True, "")
        h.engine.on_frame_acquired(self.phase, {"confidence": 0.8})
        modes = h.actions("publish_motion_mode")
        self.assertEqual("PARKING", modes[-1])
        self.assertIn("PARKING", modes)

    def test_mission_module_has_no_orchestrator_import(self):
        source = Path(mission.__file__).read_text(encoding="utf-8")
        self.assertNotIn("task_orchestrator", source)


class ParkingModeTests(unittest.TestCase):
    phase = protocol.PHASE_PHYSICAL
    def test_motion_mode_parking_after_frame_acquired(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(self.phase, True, "")
        h.engine.on_frame_acquired(self.phase, {"confidence": 0.8})
        modes = h.actions("publish_motion_mode")
        self.assertIn("PARKING", modes)

    def test_motion_mode_back_to_idle_on_arrival(self):
        h = Harness()
        h.run_full_success()
        modes = h.actions("publish_motion_mode")
        self.assertNotEqual(modes[-1], "PARKING")

    def test_parking_commands_gated_by_state(self):
        h = Harness()
        h.accept()
        self.assertEqual(h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT)
        self.assertFalse(h.engine.visual_parking_active)


class StagingContractTests(unittest.TestCase):
    """任务 A：标牌对正后必须先经动态观察点估计与导航，才能启动白框搜索。

    ESTIMATE_STAGING_POSE / NAVIGATE_STAGING_POSE 是新增合同状态：
    - 对正完成不得直接进入 ACQUIRE_FRAME；
    - 动态点未确认不得发送 staging goal；
    - staging 导航成功且停稳后才启动白框搜索；
    - 估计失败在当前点重新对正采样，耗尽后换下一个搜索点；
    - staging 导航失败有界重试，耗尽后安全失败；
    - 所有失败/取消/超时先零速。
    """

    phase = protocol.PHASE_PHYSICAL

    def assert_safe_stop_before_terminal(self, h, terminal_state):
        actions = [name for name, _ in h.outputs]
        self.assertIn("publish_zero_velocity", actions)
        self.assertIn("publish_safe_stop", actions)
        self.assertLess(
            actions.index("publish_safe_stop"),
            actions.index("publish_arrival"),
        )
        self.assertEqual(h.engine.state, terminal_state)

    def to_estimate(self, h):
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.ESTIMATE_STAGING_POSE
        )

    def test_sign_aligned_enters_staging_estimate_not_acquire(self):
        h = Harness()
        self.to_estimate(h)
        self.assertNotEqual(h.engine.state, mission.DeliveryMission.ACQUIRE_FRAME)
        self.assertEqual([], h.actions("start_frame_search"))
        self.assertIn("start_staging_estimate", [a for a, _ in h.outputs])

    def test_staging_pose_not_confirmed_sends_no_staging_goal(self):
        h = Harness()
        self.to_estimate(h)
        self.assertEqual([], h.actions("navigate_to_staging"))

    def test_invalid_staging_pose_ignored(self):
        h = Harness()
        self.to_estimate(h)
        h.engine.on_staging_pose_estimated(self.phase, None)
        h.engine.on_staging_pose_estimated(
            self.phase, {"x": float("nan"), "y": 0.0, "yaw": 0.0}
        )
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.ESTIMATE_STAGING_POSE
        )
        self.assertEqual([], h.actions("navigate_to_staging"))

    def test_confirmed_staging_pose_starts_staging_navigation(self):
        h = Harness()
        self.to_estimate(h)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_STAGING_POSE
        )
        goals = h.actions("navigate_to_staging")
        self.assertEqual(1, len(goals))
        self.assertEqual(1.0, goals[0]["pose"]["x"])
        # 白框搜索必须等 staging 导航成功之后
        self.assertEqual([], h.actions("start_frame_search"))

    def test_staging_navigation_success_before_frame_search(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_STAGING_POSE
        )
        self.assertEqual([], h.actions("start_frame_search"))
        h.engine.on_staging_navigation_result(self.phase, True, "")
        self.assertEqual(h.engine.state, mission.DeliveryMission.ACQUIRE_FRAME)
        self.assertEqual(1, len(h.actions("start_frame_search")))

    def test_staging_navigation_failure_retries_bounded(self):
        h = Harness(staging_navigation_max_retries=2)
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.engine.on_staging_navigation_result(self.phase, False, "aborted")
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_STAGING_POSE
        )
        self.assertEqual(2, len(h.actions("navigate_to_staging")))
        h.engine.on_staging_navigation_result(self.phase, False, "aborted")
        h.engine.on_staging_navigation_result(self.phase, False, "aborted")
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.FAILED)

    def test_staging_estimate_failure_resamples_at_current_point(self):
        h = Harness(staging_estimation_max_retries=3)
        self.to_estimate(h)
        h.engine.on_staging_estimate_failed(self.phase, "not enough inliers")
        self.assertEqual(h.engine.state, mission.DeliveryMission.ALIGN_SIGN)
        aligns = h.actions("align_sign")
        self.assertEqual(2, len(aligns))  # 初始对正 + 重新对正
        h.engine.on_sign_aligned(self.phase)
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.ESTIMATE_STAGING_POSE
        )
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_STAGING_POSE
        )

    def test_staging_estimate_retries_exhausted_switches_viewpoint(self):
        h = Harness(staging_estimation_max_retries=2)
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_estimate_failed(self.phase, "no inliers")
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_estimate_failed(self.phase, "no inliers")
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_estimate_failed(self.phase, "no inliers")
        goals = navigation_actions(h)
        self.assertEqual(2, len(goals))
        self.assertEqual(goals[-1][1]["viewpoint_index"], 1)
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT
        )

    def test_staging_estimate_timeout_fails_safe(self):
        h = Harness()
        self.to_estimate(h)
        h.clock.advance(31.0)
        h.engine.tick()
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.TIMEOUT)

    def test_staging_navigation_timeout_fails_safe(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        h.clock.advance(121.0)
        h.engine.tick()
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.TIMEOUT)

    def test_staging_pose_event_out_of_state_ignored(self):
        h = Harness()
        h.accept()
        before = len(h.outputs)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        self.assertEqual(len(h.outputs), before)
        self.assertEqual(
            h.engine.state, mission.DeliveryMission.NAVIGATE_VIEWPOINT
        )

    def test_staging_navigation_maps_to_navigation_mode(self):
        h = Harness()
        h.accept()
        h.engine.on_viewpoint_result(self.phase, True, "")
        h.engine.on_sign_found(self.phase, {"confidence": 0.9})
        h.engine.on_sign_aligned(self.phase)
        h.engine.on_staging_pose_estimated(self.phase, h.valid_staging_pose())
        modes = h.actions("publish_motion_mode")
        self.assertEqual("NAVIGATION", modes[-1])
        self.assertEqual([], h.actions("start_frame_search"))

    def test_staging_estimate_maps_to_visual_search_mode(self):
        h = Harness()
        self.to_estimate(h)
        modes = h.actions("publish_motion_mode")
        self.assertEqual("VISUAL_SEARCH", modes[-1])

    def test_staging_cancel_goes_through_safe_stop(self):
        h = Harness()
        self.to_estimate(h)
        h.engine.cancel("operator cancelled")
        self.assert_safe_stop_before_terminal(h, mission.DeliveryMission.CANCELLED)


if __name__ == "__main__":
    unittest.main()
