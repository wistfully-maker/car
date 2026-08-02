import json
import math
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.fast_nav_logic import FastNavSession, StopDetector
from task_orchestrator.protocol import ProtocolError, parse_arrival


def waypoint(**changes):
    value = {
        "frame_id": "map",
        "map_sha256": "abc123",
        "x": 1.25,
        "y": -0.5,
        "yaw": 1.57,
        "position_tolerance": 0.15,
        "yaw_tolerance": 0.2,
    }
    value.update(changes)
    return value


def goal(task_id="task-1", goal_id="goal-1", **changes):
    value = {
        "protocol_version": 1,
        "task_id": task_id,
        "goal_id": goal_id,
    }
    value.update(changes)
    return value


class FastNavSessionTests(unittest.TestCase):
    def setUp(self):
        self.session = FastNavSession(waypoint())

    def test_accepts_protocol_v1_goal_and_preserves_identity(self):
        self.assertTrue(self.session.accept_goal(json.dumps(goal())))
        self.assertEqual(
            {"task_id": "task-1", "goal_id": "goal-1"},
            self.session.active_identity,
        )
        self.assertEqual(waypoint(), self.session.waypoint)

    def test_rejects_bad_protocol_or_missing_or_empty_identity(self):
        invalid = (
            goal(protocol_version=2),
            {"protocol_version": 1, "goal_id": "goal-1"},
            {"protocol_version": 1, "task_id": "task-1"},
            goal(task_id=""),
            goal(goal_id="   "),
        )
        for message in invalid:
            with self.subTest(message=message):
                with self.assertRaises(ProtocolError):
                    self.session.accept_goal(message)

    def test_duplicate_active_identity_does_not_create_new_session(self):
        self.assertTrue(self.session.accept_goal(goal()))
        self.assertFalse(self.session.accept_goal(goal(extra="ignored")))
        self.assertEqual(
            {"task_id": "task-1", "goal_id": "goal-1"},
            self.session.active_identity,
        )

    def test_new_identity_replaces_old_and_old_result_is_stale(self):
        self.session.accept_goal(goal())
        self.assertTrue(self.session.accept_goal(goal("task-2", "goal-2")))
        self.assertIsNone(
            self.session.complete("task-1", "goal-1", succeeded=True)
        )
        result = self.session.complete("task-2", "goal-2", succeeded=True)
        self.assertEqual("arrived", result["status"])

    def test_success_terminal_is_emitted_once_and_matches_arrival_protocol(self):
        self.session.accept_goal(goal())
        result = self.session.complete("task-1", "goal-1", succeeded=True)
        self.assertEqual(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": "goal-1",
                "status": "arrived",
                "message": "",
            },
            result,
        )
        self.assertEqual(result, parse_arrival(json.dumps(result), "task-1", "goal-1"))
        self.assertIsNone(
            self.session.complete("task-1", "goal-1", succeeded=True)
        )

    def test_failure_terminal_requires_reason_and_is_emitted_once(self):
        self.session.accept_goal(goal())
        with self.assertRaises(ProtocolError):
            self.session.complete("task-1", "goal-1", succeeded=False, message=" ")
        result = self.session.complete(
            "task-1", "goal-1", succeeded=False, message=" action aborted "
        )
        self.assertEqual("failed", result["status"])
        self.assertEqual("action aborted", result["message"])
        self.assertEqual("task-1", result["task_id"])
        self.assertEqual("goal-1", result["goal_id"])
        self.assertIsNone(
            self.session.complete(
                "task-1", "goal-1", succeeded=False, message="again"
            )
        )

    def test_cancel_active_session_emits_failed_once_with_original_identity(self):
        self.session.accept_goal(goal())
        result = self.session.cancel("task-1", " operator cancel ")
        self.assertEqual("failed", result["status"])
        self.assertEqual("operator cancel", result["message"])
        self.assertEqual("goal-1", result["goal_id"])
        self.assertIsNone(self.session.cancel("task-1", "again"))

    def test_cancel_for_stale_task_is_ignored(self):
        self.session.accept_goal(goal())
        self.assertIsNone(self.session.cancel("task-old", "operator cancel"))
        self.assertEqual("task-1", self.session.active_identity["task_id"])

    def test_concurrent_completes_emit_exactly_one_terminal(self):
        self.session.accept_goal(goal())
        first_entered = threading.Event()
        second_entered = threading.Event()
        release_first = threading.Event()
        call_count = [0]
        count_lock = threading.Lock()
        original_builder = __import__(
            "task_orchestrator.fast_nav_logic", fromlist=["build_arrival"]
        ).build_arrival

        def synchronized_builder(*args, **kwargs):
            with count_lock:
                call_count[0] += 1
                call_number = call_count[0]
            if call_number == 1:
                first_entered.set()
                self.assertTrue(release_first.wait(timeout=2.0))
            else:
                second_entered.set()
            return original_builder(*args, **kwargs)

        results = []
        with patch(
            "task_orchestrator.fast_nav_logic.build_arrival",
            synchronized_builder,
        ):
            threads = [
                threading.Thread(
                    target=lambda: results.append(
                        self.session.complete(
                            "task-1", "goal-1", succeeded=True
                        )
                    )
                )
                for _ in range(2)
            ]
            for thread in threads:
                thread.start()
            self.assertTrue(first_entered.wait(timeout=2.0))
            self.assertFalse(second_entered.wait(timeout=0.1))
            release_first.set()
            for thread in threads:
                thread.join(timeout=3.0)

        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(1, sum(result is not None for result in results))

    def test_old_complete_cannot_clear_new_goal_during_interleaving(self):
        self.session.accept_goal(goal())
        builder_entered = threading.Event()
        allow_builder = threading.Event()
        original_builder = __import__(
            "task_orchestrator.fast_nav_logic", fromlist=["build_arrival"]
        ).build_arrival

        def blocked_builder(*args, **kwargs):
            builder_entered.set()
            self.assertTrue(allow_builder.wait(timeout=2.0))
            return original_builder(*args, **kwargs)

        results = []
        accept_started = threading.Event()
        accept_finished = threading.Event()

        def accept_new_goal():
            accept_started.set()
            self.session.accept_goal(goal("task-2", "goal-2"))
            accept_finished.set()

        with patch(
            "task_orchestrator.fast_nav_logic.build_arrival", blocked_builder
        ):
            thread = threading.Thread(
                target=lambda: results.append(
                    self.session.complete("task-1", "goal-1", succeeded=True)
                )
            )
            thread.start()
            self.assertTrue(builder_entered.wait(timeout=2.0))
            accept_thread = threading.Thread(target=accept_new_goal)
            accept_thread.start()
            self.assertTrue(accept_started.wait(timeout=2.0))
            self.assertFalse(accept_finished.wait(timeout=0.1))
            allow_builder.set()
            thread.join(timeout=3.0)
            accept_thread.join(timeout=3.0)

        self.assertFalse(thread.is_alive())
        self.assertFalse(accept_thread.is_alive())
        self.assertEqual(
            {"task_id": "task-2", "goal_id": "goal-2"},
            self.session.active_identity,
        )

    def test_waypoint_is_isolated_from_input_and_property_mutation(self):
        original = waypoint(metadata={"owner": "caller"})
        session = FastNavSession(original)
        original["x"] = 999
        original["metadata"]["owner"] = "changed"
        exposed = session.waypoint
        exposed["x"] = -999
        self.assertEqual(1.25, session.waypoint["x"])
        self.assertNotIn("metadata", session.waypoint)

    def test_waypoint_validates_required_text_numeric_and_tolerance_fields(self):
        for field in ("frame_id", "map_sha256"):
            with self.subTest(field=field):
                with self.assertRaises(ProtocolError):
                    FastNavSession(waypoint(**{field: " "}))
        for field in ("x", "y", "yaw", "position_tolerance", "yaw_tolerance"):
            with self.subTest(field=field):
                with self.assertRaises(ProtocolError):
                    FastNavSession(waypoint(**{field: math.inf}))
        for field in ("position_tolerance", "yaw_tolerance"):
            for value in (0, -0.1):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ProtocolError):
                        FastNavSession(waypoint(**{field: value}))
        with self.assertRaises(ProtocolError):
            FastNavSession({"frame_id": "map"})

    def test_waypoint_huge_integer_raises_protocol_error(self):
        with self.assertRaises(ProtocolError):
            FastNavSession(waypoint(x=10**1000))


class StopDetectorTests(unittest.TestCase):
    def test_requires_both_velocities_within_threshold_for_settle_time(self):
        detector = StopDetector(0.03, 0.05, 0.5)
        self.assertFalse(detector.update(0.03, 0.05, 10.0))
        self.assertFalse(detector.update(-0.02, -0.04, 10.49))
        self.assertTrue(detector.update(0.01, 0.01, 10.5))

    def test_either_velocity_over_limit_restarts_timer(self):
        detector = StopDetector(0.03, 0.05, 0.5)
        self.assertFalse(detector.update(0.0, 0.0, 1.0))
        self.assertFalse(detector.update(0.031, 0.0, 1.4))
        self.assertFalse(detector.update(0.0, 0.0, 1.5))
        self.assertFalse(detector.update(0.0, -0.051, 1.9))
        self.assertFalse(detector.update(0.0, 0.0, 2.0))
        self.assertTrue(detector.update(0.0, 0.0, 2.5))

    def test_time_going_backwards_safely_restarts_timer(self):
        detector = StopDetector(0.03, 0.05, 0.5)
        self.assertFalse(detector.update(0.0, 0.0, 5.0))
        self.assertFalse(detector.update(0.0, 0.0, 4.0))
        self.assertFalse(detector.update(0.0, 0.0, 4.49))
        self.assertTrue(detector.update(0.0, 0.0, 4.5))

    def test_rejects_invalid_configuration_and_samples(self):
        for args in ((-0.1, 0.05, 0.5), (0.03, -0.1, 0.5), (0.03, 0.05, 0)):
            with self.subTest(args=args):
                with self.assertRaises(ValueError):
                    StopDetector(*args)
        detector = StopDetector(0.03, 0.05, 0.5)
        for sample in ((math.nan, 0, 1), (0, math.inf, 1), (0, 0, math.nan)):
            with self.subTest(sample=sample):
                with self.assertRaises(ValueError):
                    detector.update(*sample)

    def test_huge_integer_configuration_and_sample_raise_value_error(self):
        with self.assertRaises(ValueError):
            StopDetector(10**1000, 0.05, 0.5)
        detector = StopDetector(0.03, 0.05, 0.5)
        with self.assertRaises(ValueError):
            detector.update(10**1000, 0.0, 1.0)

    def test_reset_discards_previous_settle_window_and_time_history(self):
        detector = StopDetector(0.03, 0.05, 0.5)
        self.assertFalse(detector.update(0.0, 0.0, 10.0))
        detector.reset()
        self.assertFalse(detector.update(0.0, 0.0, 20.0))
        self.assertFalse(detector.update(0.0, 0.0, 20.49))
        self.assertTrue(detector.update(0.0, 0.0, 20.5))


if __name__ == "__main__":
    unittest.main()
