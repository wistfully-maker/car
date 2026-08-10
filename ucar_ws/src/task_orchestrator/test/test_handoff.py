import sys
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.handoff import (
    ActionGoalCancelled,
    ActionGoalCancelFailed,
    LegacyStackExited,
    LegacyStackStillRunning,
    NavigationHandoff,
    OdomNotStopped,
    OdomStale,
    OdomStopped,
    StopStackNotReady,
    StopStackReady,
    StopStackStarted,
    StopStackStartFailed,
)


def make_config(**overrides):
    config = {
        "cancel_retries": 3,
        "cancel_timeout": 3.0,
        "stop_stable_duration": 0.75,
        "odom_max_age": 0.5,
        "legacy_exit_retries": 5,
        "legacy_exit_timeout": 10.0,
        "readiness_retries": 30,
        "readiness_poll_period": 1.0,
        "total_timeout": 90.0,
    }
    config.update(overrides)
    return config


class FakeClock:
    def __init__(self):
        self.now = [0.0]

    def __call__(self):
        return self.now[0]


def identity_payload(action):
    return action[1]["task_id"], action[1]["goal_id"]


class HandoffHappyPathTests(unittest.TestCase):
    def test_exact_state_sequence_and_zero_before_each_transition(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(), clock)
        self.assertEqual("IDLE", machine.state)

        effects = machine.start(task_id="task-1", goal_id="delivery-1")
        self.assertEqual("CANCELLING", machine.state)
        self.assertEqual(
            [
                ("publish_zero", None),
                ("cancel_goals", {"task_id": "task-1", "goal_id": "delivery-1"}),
            ],
            effects,
        )

        effects = machine.observe(ActionGoalCancelled())
        self.assertEqual("VERIFYING_STOP", machine.state)
        self.assertEqual(
            [
                ("publish_zero", None),
                ("wait_stopped", {"task_id": "task-1", "goal_id": "delivery-1"}),
            ],
            effects,
        )

        # 新鲜近零 odom 连续累积 stop_stable_duration 后才切换。
        clock.now[0] = 0.1
        self.assertEqual([], machine.observe(OdomStopped(stamp=0.1)))
        clock.now[0] = 0.5
        self.assertEqual([], machine.observe(OdomStopped(stamp=0.5)))
        self.assertEqual("VERIFYING_STOP", machine.state)
        clock.now[0] = 0.9
        effects = machine.observe(OdomStopped(stamp=0.9))
        self.assertEqual("STOPPING_LEGACY", machine.state)
        self.assertEqual(
            [
                ("publish_zero", None),
                ("stop_owned_legacy", {"task_id": "task-1", "goal_id": "delivery-1"}),
            ],
            effects,
        )

        effects = machine.observe(LegacyStackExited())
        self.assertEqual("STARTING_STOP", machine.state)
        self.assertEqual(
            [
                ("publish_zero", None),
                ("start_owned_stop", {"task_id": "task-1", "goal_id": "delivery-1"}),
            ],
            effects,
        )

        effects = machine.observe(StopStackStarted())
        self.assertEqual("WAITING_STOP_READY", machine.state)
        self.assertEqual(
            [
                ("publish_zero", None),
                ("wait_stop_ready", {"task_id": "task-1", "goal_id": "delivery-1"}),
            ],
            effects,
        )

        effects = machine.observe(StopStackReady())
        self.assertEqual("READY", machine.state)
        self.assertEqual(
            [
                ("publish_zero", None),
                ("release_task", {"task_id": "task-1", "goal_id": "delivery-1"}),
            ],
            effects,
        )

    def test_single_odom_sample_never_transitions(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(), clock)
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        self.assertEqual([], machine.observe(OdomStopped(stamp=0.1)))
        self.assertEqual("VERIFYING_STOP", machine.state)


class HandoffIdentityTests(unittest.TestCase):
    def test_duplicate_start_is_ignored(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        first = machine.start(task_id="task-1", goal_id="delivery-1")
        second = machine.start(task_id="task-2", goal_id="other-1")
        self.assertEqual([], second)
        self.assertEqual("CANCELLING", machine.state)
        self.assertEqual(("task-1", "delivery-1"), identity_payload(first[1]))

    def test_observations_before_start_are_ignored(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        self.assertEqual([], machine.observe(ActionGoalCancelled()))
        self.assertEqual([], machine.observe(StopStackReady()))
        self.assertEqual("IDLE", machine.state)

    def test_wrong_identity_events_are_ignored(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        machine.start(task_id="task-1", goal_id="delivery-1")
        for event in (
            ActionGoalCancelled(task_id="stale-task"),
            ActionGoalCancelled(goal_id="stale-goal"),
            OdomStopped(stamp=0.1, task_id="stale-task"),
            StopStackReady(task_id="stale-task", goal_id="delivery-1"),
        ):
            with self.subTest(event=event):
                output_count = 0
                self.assertEqual([], machine.observe(event))
        self.assertEqual("CANCELLING", machine.state)


class HandoffOdomTests(unittest.TestCase):
    def _reach_verify_stop(self, machine):
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())

    def test_gap_beyond_odom_max_age_resets_accumulation(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(), clock)
        self._reach_verify_stop(machine)
        clock.now[0] = 0.1
        machine.observe(OdomStopped(stamp=0.1))
        clock.now[0] = 0.3
        machine.observe(OdomStopped(stamp=0.3))
        clock.now[0] = 1.0  # gap 0.7 > odom_max_age 0.5
        effects = machine.observe(OdomStopped(stamp=1.0))
        self.assertEqual("VERIFYING_STOP", machine.state)
        self.assertEqual("publish_zero", effects[0][0])
        self.assertEqual("publish_diagnostic", effects[1][0])
        self.assertEqual("odom_stale", effects[1][1]["reason"])
        # 重置后重新累积到足够时长仍能切换。
        clock.now[0] = 1.2
        machine.observe(OdomStopped(stamp=1.2))
        clock.now[0] = 1.6
        machine.observe(OdomStopped(stamp=1.6))
        clock.now[0] = 2.0
        machine.observe(OdomStopped(stamp=2.0))
        self.assertEqual("STOPPING_LEGACY", machine.state)

    def test_odom_clock_rollback_resets_accumulation(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(), clock)
        self._reach_verify_stop(machine)
        clock.now[0] = 1.0
        machine.observe(OdomStopped(stamp=1.0))
        clock.now[0] = 0.5  # stamp 回退
        effects = machine.observe(OdomStopped(stamp=0.5))
        self.assertEqual("VERIFYING_STOP", machine.state)
        self.assertEqual("publish_zero", effects[0][0])
        self.assertEqual("odom_clock_rollback", effects[1][1]["reason"])

    def test_moving_vehicle_resets_accumulation(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(), clock)
        self._reach_verify_stop(machine)
        clock.now[0] = 0.1
        machine.observe(OdomStopped(stamp=0.1))
        clock.now[0] = 0.5
        effects = machine.observe(OdomNotStopped())
        self.assertEqual("VERIFYING_STOP", machine.state)
        self.assertEqual("vehicle_moving", effects[1][1]["reason"])
        clock.now[0] = 0.7
        machine.observe(OdomStopped(stamp=0.7))
        clock.now[0] = 1.1
        machine.observe(OdomStopped(stamp=1.1))
        clock.now[0] = 1.5
        machine.observe(OdomStopped(stamp=1.5))
        # 重置后 1.5 - 0.7 = 0.8 >= 0.75 才能切换；0.7 前的样本不计入。
        self.assertEqual("STOPPING_LEGACY", machine.state)

    def test_invalid_odom_stamp_is_stale(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        self._reach_verify_stop(machine)
        effects = machine.observe(OdomStopped(stamp=float("nan")))
        self.assertEqual("VERIFYING_STOP", machine.state)
        self.assertEqual("odom_stale", effects[1][1]["reason"])


class HandoffRetryTests(unittest.TestCase):
    def test_cancel_retry_then_success(self):
        machine = NavigationHandoff(make_config(cancel_retries=3), FakeClock())
        machine.start("task-1", "delivery-1")
        effects = machine.observe(ActionGoalCancelFailed())
        self.assertEqual("CANCELLING", machine.state)
        self.assertEqual("publish_zero", effects[0][0])
        self.assertEqual("publish_diagnostic", effects[1][0])
        self.assertEqual("retrying", effects[1][1]["reason"])
        self.assertEqual(1, effects[1][1]["retry"])
        self.assertEqual("cancel_goals", effects[2][0])
        machine.observe(ActionGoalCancelled())
        self.assertEqual("VERIFYING_STOP", machine.state)

    def test_cancel_retries_exhausted_enter_failed(self):
        machine = NavigationHandoff(
            make_config(cancel_retries=2), FakeClock()
        )
        machine.start("task-1", "delivery-1")
        for _ in range(2):
            machine.observe(ActionGoalCancelFailed())
        effects = machine.observe(ActionGoalCancelFailed())
        self.assertEqual("FAILED", machine.state)
        self.assertEqual("publish_zero", effects[0][0])
        self.assertEqual("publish_diagnostic", effects[1][0])
        self.assertEqual("handoff_failed", effects[2][0])
        self.assertEqual("cancel", effects[2][1]["stage"])

    def test_legacy_exit_retry_then_success(self):
        machine = NavigationHandoff(make_config(legacy_exit_retries=2), FakeClock())
        self._reach_stopping_legacy(machine)
        effects = machine.observe(LegacyStackStillRunning())
        self.assertEqual("STOPPING_LEGACY", machine.state)
        self.assertEqual("verify_legacy_absent", effects[2][0])
        self.assertEqual(1, effects[1][1]["retry"])
        machine.observe(LegacyStackExited())
        self.assertEqual("STARTING_STOP", machine.state)

    def test_legacy_exit_retries_exhausted_enter_failed(self):
        machine = NavigationHandoff(
            make_config(legacy_exit_retries=1), FakeClock()
        )
        self._reach_stopping_legacy(machine)
        machine.observe(LegacyStackStillRunning())
        effects = machine.observe(LegacyStackStillRunning())
        self.assertEqual("FAILED", machine.state)
        self.assertEqual("legacy_exit", effects[2][1]["stage"])

    def test_readiness_retry_then_success(self):
        machine = NavigationHandoff(make_config(readiness_retries=2), FakeClock())
        self._reach_waiting_readiness(machine)
        effects = machine.observe(StopStackNotReady())
        self.assertEqual("WAITING_STOP_READY", machine.state)
        self.assertEqual("wait_stop_ready", effects[2][0])
        self.assertEqual("readiness", effects[1][1]["stage"])
        machine.observe(StopStackReady())
        self.assertEqual("READY", machine.state)

    def test_readiness_retries_exhausted_enter_failed(self):
        machine = NavigationHandoff(
            make_config(readiness_retries=1), FakeClock()
        )
        self._reach_waiting_readiness(machine)
        machine.observe(StopStackNotReady())
        effects = machine.observe(StopStackNotReady())
        self.assertEqual("FAILED", machine.state)
        self.assertEqual("readiness", effects[2][1]["stage"])

    def test_stop_stack_start_failure_enters_failed(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        self._reach_starting_stop(machine)
        effects = machine.observe(StopStackStartFailed())
        self.assertEqual("FAILED", machine.state)
        self.assertEqual("start_stop", effects[2][1]["stage"])

    def _reach_stopping_legacy(self, machine):
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        machine.observe(OdomStopped(stamp=0.1))
        machine.observe(OdomStopped(stamp=0.5))
        machine.observe(OdomStopped(stamp=0.9))

    def _reach_starting_stop(self, machine):
        self._reach_stopping_legacy(machine)
        machine.observe(LegacyStackExited())

    def _reach_waiting_readiness(self, machine):
        self._reach_starting_stop(machine)
        machine.observe(StopStackStarted())


class HandoffDeadlineTests(unittest.TestCase):
    def test_total_timeout_enters_failed(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(total_timeout=90.0), clock)
        machine.start("task-1", "delivery-1")
        clock.now[0] = 91.0
        effects = machine.tick()
        self.assertEqual("FAILED", machine.state)
        self.assertEqual("publish_zero", effects[0][0])
        self.assertEqual("total_timeout", effects[2][1]["reason"])

    def test_tick_within_deadline_keeps_state(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(total_timeout=90.0), clock)
        machine.start("task-1", "delivery-1")
        clock.now[0] = 89.0
        self.assertEqual([], machine.tick())
        self.assertEqual("CANCELLING", machine.state)

    def test_tick_clock_rollback_restarts_deadline(self):
        clock = FakeClock()
        clock.now[0] = 100.0
        machine = NavigationHandoff(make_config(total_timeout=90.0), clock)
        machine.start("task-1", "delivery-1")
        clock.now[0] = 50.0  # 回退 50 秒
        effects = machine.tick()
        self.assertEqual("CANCELLING", machine.state)
        self.assertEqual("clock_rollback", effects[1][1]["reason"])
        clock.now[0] = 139.0  # 重置后的 90 秒以内
        self.assertEqual([], machine.tick())
        clock.now[0] = 140.5  # 重置后超过 90 秒
        machine.tick()
        self.assertEqual("FAILED", machine.state)

    def test_tick_is_noop_in_terminal_states(self):
        clock = FakeClock()
        machine = NavigationHandoff(make_config(), clock)
        self.assertEqual([], machine.tick())
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        clock.now[0] = 1000.0
        machine.tick()
        self.assertEqual("FAILED", machine.state)
        self.assertEqual([], machine.tick())


class HandoffStateGatingTests(unittest.TestCase):
    def test_stale_cancel_failure_in_verify_stop_is_ignored(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        self.assertEqual([], machine.observe(ActionGoalCancelFailed()))
        self.assertEqual("VERIFYING_STOP", machine.state)

    def test_stale_start_failure_in_other_states_is_ignored(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        self.assertEqual([], machine.observe(StopStackStartFailed()))
        self.assertEqual("VERIFYING_STOP", machine.state)

    def test_stale_readiness_event_in_legacy_stop_is_ignored(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        machine.observe(OdomStopped(stamp=0.1))
        machine.observe(OdomStopped(stamp=0.5))
        machine.observe(OdomStopped(stamp=0.9))
        self.assertEqual([], machine.observe(StopStackNotReady()))
        self.assertEqual("STOPPING_LEGACY", machine.state)


class HandoffTerminalTests(unittest.TestCase):
    def test_ready_is_terminal_and_idempotent(self):
        machine = NavigationHandoff(make_config(), FakeClock())
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelled())
        machine.observe(OdomStopped(stamp=0.1))
        machine.observe(OdomStopped(stamp=0.5))
        machine.observe(OdomStopped(stamp=0.9))
        machine.observe(LegacyStackExited())
        machine.observe(StopStackStarted())
        machine.observe(StopStackReady())
        self.assertEqual("READY", machine.state)
        self.assertEqual([], machine.observe(StopStackReady()))
        self.assertEqual([], machine.observe(StopStackNotReady()))
        self.assertEqual("READY", machine.state)

    def test_failed_is_terminal_and_idempotent(self):
        machine = NavigationHandoff(
            make_config(cancel_retries=1), FakeClock()
        )
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelFailed())
        machine.observe(ActionGoalCancelFailed())
        self.assertEqual("FAILED", machine.state)
        self.assertEqual([], machine.observe(ActionGoalCancelled()))
        self.assertEqual([], machine.tick())
        self.assertEqual("FAILED", machine.state)

    def test_restart_after_failed(self):
        machine = NavigationHandoff(
            make_config(cancel_retries=1), FakeClock()
        )
        machine.start("task-1", "delivery-1")
        machine.observe(ActionGoalCancelFailed())
        machine.observe(ActionGoalCancelFailed())
        effects = machine.start(task_id="task-2", goal_id="delivery-2")
        self.assertEqual("CANCELLING", machine.state)
        self.assertEqual("cancel_goals", effects[1][0])
        self.assertEqual("task-2", effects[1][1]["task_id"])


class HandoffConfigTests(unittest.TestCase):
    def test_accepts_exact_defaults(self):
        NavigationHandoff(make_config(), FakeClock())

    def test_rejects_missing_non_positive_or_non_finite_values(self):
        for field in (
            "cancel_retries",
            "cancel_timeout",
            "stop_stable_duration",
            "odom_max_age",
            "legacy_exit_retries",
            "legacy_exit_timeout",
            "readiness_retries",
            "readiness_poll_period",
            "total_timeout",
        ):
            for value in (0, -1, float("nan"), float("inf"), "3", None):
                with self.subTest(field=field, value=value):
                    config = make_config(**{field: value})
                    with self.assertRaises(ValueError):
                        NavigationHandoff(config, FakeClock())
            with self.subTest(field=field):
                del make_config()[field]
                config = make_config()
                del config[field]
                with self.assertRaises(ValueError):
                    NavigationHandoff(config, FakeClock())

    def test_rejects_non_integer_retry_counts(self):
        for field in ("cancel_retries", "legacy_exit_retries", "readiness_retries"):
            with self.subTest(field=field):
                config = make_config(**{field: 2.5})
                with self.assertRaises(ValueError):
                    NavigationHandoff(config, FakeClock())


if __name__ == "__main__":
    unittest.main()
