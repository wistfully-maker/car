"""Phase C: lock the move_base navigation supervisor contract.

The supervisor owns one navigation attempt's action lifecycle: goal sending,
success/preemption/abort handling, bounded cancellation, timeouts, target
detection interruption and a guaranteed zero-velocity exit on every path.
Pure logic driven by a fake action client: no ROS import.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import nav_supervisor
from ucar_delivery.nav_supervisor import NavSupervisor

SUCCEEDED = "SUCCEEDED"
ABORTED = "ABORTED"
PREEMPTED = "PREEMPTED"

TIMEOUTS = {
    "action_timeout": 120.0,
    "cancel_timeout": 10.0,
    "settle_time": 0.5,
}

STAGING_SETTLE_CONFIG = {
    "settle_timeout": 2.0,
    "odom_timeout": 0.3,
    "settle_velocity_threshold": 0.02,
    "settle_angular_threshold": 0.05,
    "settle_stable_duration": 0.5,
}


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Harness:
    def __init__(self, timeouts=None, config=None):
        self.outputs = []
        self.clock = Clock()
        self.sup = NavSupervisor(
            self.outputs,
            self.clock,
            dict(TIMEOUTS, **(timeouts or {})),
            config or {},
        )

    def actions(self, action):
        return [payload for name, payload in self.outputs if name == action]

    def goal(self, **overrides):
        payload = {
            "protocol_version": 1,
            "phase": "physical",
            "task_id": "task-test-001",
            "goal_id": "delivery-test-001",
            "viewpoint_index": 0,
            "attempt": 1,
        }
        payload.update(overrides)
        return payload

    def begin(self, **overrides):
        self.sup.begin(self.goal(**overrides))

    def staging_goal(self, **overrides):
        payload = {
            "protocol_version": 1,
            "phase": "physical",
            "task_id": "task-test-001",
            "goal_id": "delivery-test-001",
            "goal_kind": "staging",
            "pose": {"x": 1.2, "y": -0.4, "yaw": 0.3},
        }
        payload.update(overrides)
        return payload

    def begin_staging(self, **overrides):
        self.sup.begin(self.staging_goal(**overrides))

    def complete_path(self):
        self.begin()
        self.sup.on_status(SUCCEEDED, self.clock())
        self.clock.advance(0.6)
        self.sup.tick()


class GoalSendingTests(unittest.TestCase):
    def test_begin_sends_exactly_one_goal_with_payload(self):
        h = Harness()
        h.begin(viewpoint_index=2, attempt=3)
        goals = h.actions("send_goal")
        self.assertEqual(1, len(goals))
        payload = goals[0]
        self.assertEqual(2, payload["viewpoint_index"])
        self.assertEqual(3, payload["attempt"])
        self.assertEqual("task-test-001", payload["task_id"])
        self.assertEqual("delivery-test-001", payload["goal_id"])
        self.assertEqual("physical", payload["phase"])
        self.assertEqual(NavSupervisor.NAVIGATING, h.sup.state)

    def test_begin_while_active_is_rejected(self):
        h = Harness()
        h.begin()
        count = len(h.actions("send_goal"))
        h.begin()
        self.assertEqual(count, len(h.actions("send_goal")))
        self.assertEqual(NavSupervisor.NAVIGATING, h.sup.state)

    def test_status_before_begin_is_ignored(self):
        h = Harness()
        h.sup.on_status(SUCCEEDED, h.clock())
        self.assertEqual([], h.actions("session_result"))
        self.assertEqual(NavSupervisor.IDLE, h.sup.state)


class StagingGoalTests(unittest.TestCase):
    """任务 C：导航会话必须区分 viewpoint 与 staging 目标，会话结果携带
    goal_kind 供 mission 节点路由到不同回调。"""

    def test_begin_staging_sends_goal_kind_in_payload(self):
        h = Harness()
        h.begin_staging()
        goals = h.actions("send_goal")
        self.assertEqual(1, len(goals))
        self.assertEqual("staging", goals[0]["goal_kind"])
        self.assertEqual(1.2, goals[0]["pose"]["x"])
        self.assertEqual("physical", goals[0]["phase"])
        self.assertEqual("delivery-test-001", goals[0]["goal_id"])

    def test_staging_session_result_carries_goal_kind(self):
        h = Harness(config=STAGING_SETTLE_CONFIG)
        h.begin_staging()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.clock.advance(0.6)
        h.sup.tick()
        self.assertEqual([], h.actions("session_result"))
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        h.clock.advance(0.1)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("staging", results[0]["goal_kind"])
        self.assertTrue(results[0]["ok"])


class StagingSettleGateTests(unittest.TestCase):
    def test_staging_success_without_odometry_does_not_complete(self):
        h = Harness(config=STAGING_SETTLE_CONFIG)
        h.begin_staging()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.clock.advance(1.0)
        h.sup.tick()
        self.assertEqual(NavSupervisor.SETTLING, h.sup.state)
        self.assertEqual([], h.actions("session_result"))

    def test_moving_odometry_resets_stable_window(self):
        h = Harness(config=STAGING_SETTLE_CONFIG)
        h.begin_staging()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.3)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.03, 0.0, 0.0, h.clock())
        h.sup.tick()
        self.assertEqual([], h.actions("session_result"))

        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        self.assertEqual([], h.actions("session_result"))
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        self.assertTrue(h.actions("session_result")[-1]["ok"])

    def test_stale_odometry_resets_stable_window(self):
        h = Harness(config=STAGING_SETTLE_CONFIG)
        h.begin_staging()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.4)  # odom_timeout=0.3
        h.sup.tick()
        self.assertEqual([], h.actions("session_result"))
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.2)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.clock.advance(0.1)
        h.sup.on_odometry_velocity(0.0, 0.0, 0.0, h.clock())
        h.sup.tick()
        self.assertTrue(h.actions("session_result")[-1]["ok"])

    def test_staging_settle_timeout_fails_safely(self):
        h = Harness(config=STAGING_SETTLE_CONFIG)
        h.begin_staging()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.clock.advance(2.1)
        h.sup.tick()
        result = h.actions("session_result")[-1]
        self.assertFalse(result["ok"])
        self.assertTrue(result["retryable"])
        self.assertEqual("settle_timed_out", result["status"])

    def test_staging_abort_reports_retryable(self):
        h = Harness()
        h.begin_staging()
        h.sup.on_status(ABORTED, h.clock(), "staging goal blocked")
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("staging", results[0]["goal_kind"])
        self.assertFalse(results[0]["ok"])
        self.assertTrue(results[0]["retryable"])

    def test_default_goal_kind_is_viewpoint(self):
        h = Harness()
        h.begin()
        goals = h.actions("send_goal")
        self.assertEqual("viewpoint", goals[0]["goal_kind"])


class SuccessPathTests(unittest.TestCase):
    def test_success_waits_for_settle_then_reports_ok(self):
        h = Harness()
        h.begin()
        h.sup.on_status(SUCCEEDED, h.clock())
        self.assertEqual(NavSupervisor.SETTLING, h.sup.state)
        h.clock.advance(0.4)
        h.sup.tick()
        self.assertEqual(NavSupervisor.SETTLING, h.sup.state)
        self.assertEqual([], h.actions("session_result"))
        h.clock.advance(0.2)
        h.sup.tick()
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("succeeded", results[0]["status"])
        self.assertTrue(results[0]["ok"])
        self.assertEqual("", results[0]["message"])

    def test_success_emits_zero_velocity_before_result(self):
        h = Harness()
        h.begin()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.clock.advance(0.6)
        h.sup.tick()
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("session_result"),
        )


class FailurePathTests(unittest.TestCase):
    def test_aborted_reports_failure_with_message(self):
        h = Harness()
        h.begin()
        h.sup.on_status(ABORTED, h.clock(), "goal blocked by obstacle")
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("aborted", results[0]["status"])
        self.assertFalse(results[0]["ok"])
        self.assertEqual("goal blocked by obstacle", results[0]["message"])
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("session_result"),
        )

    def test_preempted_without_pending_cancel_reports_failure(self):
        h = Harness()
        h.begin()
        h.sup.on_status(PREEMPTED, h.clock(), "preempted externally")
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertFalse(results[0]["ok"])
        self.assertEqual("preempted", results[0]["status"])

    def test_navigation_timeout_cancels_then_times_out(self):
        h = Harness()
        h.begin()
        h.clock.advance(121.0)
        h.sup.tick()
        self.assertEqual(NavSupervisor.CANCELLING, h.sup.state)
        self.assertEqual(1, len(h.actions("cancel_goal")))
        h.clock.advance(11.0)
        h.sup.tick()
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("timed_out", results[0]["status"])
        self.assertFalse(results[0]["ok"])

    def test_active_progress_resets_action_timeout(self):
        h = Harness()
        h.begin()
        h.clock.advance(100.0)
        h.sup.on_status("ACTIVE", h.clock())
        h.clock.advance(100.0)
        h.sup.on_status("ACTIVE", h.clock())
        h.clock.advance(100.0)
        h.sup.tick()
        self.assertEqual(NavSupervisor.NAVIGATING, h.sup.state)
        self.assertEqual([], h.actions("session_result"))

    def test_cancel_timeout_force_terminates(self):
        h = Harness()
        h.begin()
        h.sup.cancel("operator")
        h.clock.advance(11.0)
        h.sup.tick()
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertFalse(results[0]["ok"])
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("session_result"),
        )


class CancellationTests(unittest.TestCase):
    def test_cancel_requests_cancellation_and_waits(self):
        h = Harness()
        h.begin()
        h.sup.cancel("operator stop")
        self.assertEqual(NavSupervisor.CANCELLING, h.sup.state)
        cancels = h.actions("cancel_goal")
        self.assertEqual(1, len(cancels))
        self.assertEqual("operator stop", cancels[0]["reason"])
        self.assertEqual([], h.actions("session_result"))

    def test_cancel_completes_after_terminal_status(self):
        h = Harness()
        h.begin()
        h.sup.cancel("operator stop")
        h.sup.on_status(PREEMPTED, h.clock(), "cancelled by client")
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("cancelled", results[0]["status"])
        self.assertFalse(results[0]["ok"])

    def test_cancel_while_idle_is_noop(self):
        h = Harness()
        h.sup.cancel("late cancel")
        self.assertEqual(NavSupervisor.IDLE, h.sup.state)
        self.assertEqual([], h.actions("session_result"))

    def test_cancel_during_settle_reports_cancelled(self):
        h = Harness()
        h.begin()
        h.sup.on_status(SUCCEEDED, h.clock())
        h.clock.advance(0.2)
        h.sup.cancel("operator")
        self.assertEqual(NavSupervisor.CANCELLING, h.sup.state)
        h.clock.advance(11.0)
        h.sup.tick()
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("cancelled", results[0]["status"])


class DetectionInterruptionTests(unittest.TestCase):
    def test_detection_cancels_active_goal(self):
        h = Harness()
        h.begin()
        h.sup.on_detection(h.clock())
        self.assertEqual(NavSupervisor.CANCELLING, h.sup.state)
        self.assertEqual(1, len(h.actions("cancel_goal")))

    def test_detection_interruption_reports_interrupted(self):
        h = Harness()
        h.begin()
        h.sup.on_detection(h.clock())
        h.sup.on_status(PREEMPTED, h.clock(), "preempted after detection")
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("interrupted", results[0]["status"])
        self.assertTrue(results[0]["retryable"])
        actions = [name for name, _ in h.outputs]
        self.assertLess(
            actions.index("publish_zero_velocity"),
            actions.index("session_result"),
        )

    def test_detection_after_completion_is_ignored(self):
        h = Harness()
        h.complete_path()
        before = len(h.outputs)
        h.sup.on_detection(h.clock())
        self.assertEqual(len(h.outputs), before)


class ShutdownTests(unittest.TestCase):
    def test_shutdown_active_guarantees_zero_velocity(self):
        h = Harness()
        h.begin()
        h.sup.shutdown()
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        results = h.actions("session_result")
        self.assertEqual(1, len(results))
        self.assertEqual("stopped", results[0]["status"])
        actions = [name for name, _ in h.outputs]
        self.assertEqual(actions[-2], "publish_zero_velocity")
        self.assertEqual(actions[-1], "session_result")

    def test_shutdown_idle_is_safe(self):
        h = Harness()
        h.sup.shutdown()
        self.assertEqual(NavSupervisor.COMPLETE, h.sup.state)
        self.assertEqual([], h.actions("session_result"))


class SafetyGuaranteeTests(unittest.TestCase):
    def test_zero_velocity_before_result_on_every_path(self):
        for path in (
            "success",
            "abort",
            "cancel",
            "timeout",
            "detection",
        ):
            with self.subTest(path=path):
                h = Harness()
                h.begin()
                if path == "success":
                    h.sup.on_status(SUCCEEDED, h.clock())
                    h.clock.advance(0.6)
                    h.sup.tick()
                elif path == "abort":
                    h.sup.on_status(ABORTED, h.clock(), "boom")
                elif path == "cancel":
                    h.sup.cancel("operator")
                    h.sup.on_status(PREEMPTED, h.clock(), "cancel")
                elif path == "timeout":
                    h.clock.advance(121.0)
                    h.sup.tick()
                    h.clock.advance(11.0)
                    h.sup.tick()
                elif path == "detection":
                    h.sup.on_detection(h.clock())
                    h.sup.on_status(PREEMPTED, h.clock(), "detected")
                actions = [name for name, _ in h.outputs]
                self.assertLess(
                    actions.index("publish_zero_velocity"),
                    actions.index("session_result"),
                    "zero velocity must precede result on %s path" % path,
                )

    def test_retry_reuses_session_after_completion(self):
        h = Harness()
        h.begin()
        h.sup.on_status(ABORTED, h.clock(), "retry me")
        h.sup.begin(h.goal(attempt=2))
        goals = h.actions("send_goal")
        self.assertEqual(2, len(goals))
        self.assertEqual(2, goals[-1]["attempt"])
        self.assertEqual(NavSupervisor.NAVIGATING, h.sup.state)


if __name__ == "__main__":
    unittest.main()
