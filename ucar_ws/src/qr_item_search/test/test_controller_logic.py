import ast
import json
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock

from qr_item_search.controller_logic import SearchController


class RecordingOutputs:
    def __init__(self):
        self.speeds = []
        self.states = []
        self.walls = []
        self.scan_enabled = []
        self.reset_count = 0
        self.reset_ids = []

    def publish_speed(self, value):
        self.speeds.append(value)

    def publish_state(self, value):
        self.states.append(value)

    def publish_wall(self, value):
        self.walls.append(value)

    def publish_scan_enabled(self, value):
        self.scan_enabled.append(value)

    def publish_reset(self, search_id):
        self.reset_count += 1
        self.reset_ids.append(search_id)


class SearchControllerTest(unittest.TestCase):
    def setUp(self):
        self.outputs = RecordingOutputs()
        self.controller = SearchController(
            wall_yaw_offsets=[0.0, math.pi / 2],
            outputs=self.outputs,
            settle_seconds=0.5,
            scan_timeout=2.0,
            turn_timeout=3.0,
        )
        self.controller.update_yaw(0.0)

    def enter_scanning(self):
        self.controller.start(now=0.0)
        self.controller.tick(now=0.0)
        self.controller.tick(now=0.5)
        self.assertEqual("SCANNING", self.controller.state)

    def test_first_wall_match_succeeds(self):
        self.enter_scanning()

        self.controller.handle_observation(
            json.dumps({"search_id": 1, "wall_index": 0, "status": "success"}),
            now=0.6,
        )
        self.controller.match_decision(True, now=0.7)

        self.assertEqual("SUCCESS", self.controller.state)
        self.assertEqual(0.0, self.outputs.speeds[-1])
        self.assertFalse(self.outputs.scan_enabled[-1])

    def test_failed_observation_immediately_turns_to_next_wall(self):
        self.enter_scanning()

        self.controller.handle_observation(
            json.dumps(
                {
                    "search_id": 1,
                    "wall_index": 0,
                    "status": "invalid_payload",
                }
            ),
            now=0.6,
        )

        self.assertEqual("TURNING", self.controller.state)
        self.assertEqual(1, self.controller.wall_index)
        self.assertEqual(0.0, self.outputs.speeds[-1])

    def test_scan_timeout_advances_to_next_wall(self):
        self.enter_scanning()

        self.controller.tick(now=2.5)

        self.assertEqual("TURNING", self.controller.state)
        self.assertEqual(1, self.controller.wall_index)

    def test_turn_timeout_enters_error_and_stops(self):
        self.controller.update_yaw(0.2)
        self.controller.start(now=0.0)

        self.controller.tick(now=3.0)

        self.assertEqual("ERROR", self.controller.state)
        self.assertEqual(0.0, self.outputs.speeds[-1])

    def test_turning_uses_configured_minimum_effective_speed(self):
        controller = SearchController(
            wall_yaw_offsets=[0.05],
            outputs=self.outputs,
            kp=1.2,
            max_speed=0.30,
            min_speed=0.11,
            tolerance=0.035,
        )
        controller.update_yaw(0.0)
        controller.start(now=0.0)

        controller.tick(now=0.1)

        self.assertEqual(0.11, self.outputs.speeds[-1])

    def test_start_without_odometry_enters_error(self):
        controller = SearchController([0.0], self.outputs)

        controller.start(now=0.0)

        self.assertEqual("ERROR", controller.state)
        self.assertEqual(0.0, self.outputs.speeds[-1])

    def test_stale_wall_observation_is_ignored(self):
        self.enter_scanning()

        self.controller.handle_observation(
            json.dumps({"search_id": 1, "wall_index": 1, "status": "success"}),
            now=0.6,
        )

        self.assertEqual("SCANNING", self.controller.state)

    def test_invalid_observation_json_enters_error(self):
        self.enter_scanning()

        self.controller.handle_observation("{bad json", now=0.6)

        self.assertEqual("ERROR", self.controller.state)
        self.assertEqual(0.0, self.outputs.speeds[-1])

    def test_stop_returns_idle_with_zero_speed(self):
        self.controller.update_yaw(0.2)
        self.controller.start(now=0.0)
        self.controller.tick(now=0.1)

        self.controller.stop(now=0.2)

        self.assertEqual("IDLE", self.controller.state)
        self.assertEqual(0.0, self.outputs.speeds[-1])

    def test_start_publishes_scanner_reset(self):
        self.controller.start(now=0.0)

        self.assertEqual(1, self.outputs.reset_count)
        self.assertEqual([1], self.outputs.reset_ids)
        self.assertEqual(0, self.outputs.walls[-1])
        self.assertEqual("TURNING", self.outputs.states[-1])

    def test_rejects_invalid_configuration(self):
        invalid = (
            {"wall_yaw_offsets": []},
            {"wall_yaw_offsets": [math.nan]},
            {"kp": 0.0},
            {"max_speed": 0.0},
            {"min_speed": -0.1},
            {"min_speed": 0.31, "max_speed": 0.30},
            {"tolerance": -1.0},
            {"settle_seconds": -1.0},
            {"scan_timeout": 0.0},
            {"turn_timeout": math.inf},
        )
        for override in invalid:
            parameters = {
                "wall_yaw_offsets": [0.0],
                "outputs": self.outputs,
            }
            parameters.update(override)
            with self.subTest(override=override):
                with self.assertRaises(ValueError):
                    SearchController(**parameters)

    def test_repeated_start_while_active_is_ignored(self):
        self.assertTrue(self.controller.start(now=0.0))

        self.assertFalse(self.controller.start(now=0.1))

        self.assertEqual("TURNING", self.controller.state)
        self.assertEqual(1, self.outputs.reset_count)

    def test_non_finite_time_is_rejected_without_state_change(self):
        for now in (math.nan, math.inf, -math.inf):
            with self.subTest(now=now):
                with self.assertRaises(ValueError):
                    self.controller.start(now)
                self.assertEqual("IDLE", self.controller.state)

    def test_time_rollback_enters_error_and_stops(self):
        self.controller.start(now=2.0)

        self.assertFalse(self.controller.tick(now=1.0))

        self.assertEqual("ERROR", self.controller.state)
        self.assertEqual(0.0, self.outputs.speeds[-1])

    def test_malformed_current_observation_enters_error(self):
        malformed = (
            {"search_id": 1, "wall_index": True, "status": "success"},
            {"search_id": 1, "wall_index": -1, "status": "success"},
            {"search_id": 1, "wall_index": 0, "status": 1},
            {"search_id": 1, "wall_index": 0, "status": "unknown"},
            {"search_id": True, "wall_index": 0, "status": "success"},
        )
        for payload in malformed:
            with self.subTest(payload=payload):
                outputs = RecordingOutputs()
                controller = SearchController(
                    [0.0], outputs, settle_seconds=0.0
                )
                controller.update_yaw(0.0)
                controller.start(0.0)
                controller.tick(0.0)
                controller.tick(0.0)

                controller.handle_observation(json.dumps(payload), 0.1)

                self.assertEqual("ERROR", controller.state)

    def test_stop_serializes_after_inflight_tick(self):
        entered = threading.Event()
        release = threading.Event()
        outputs = RecordingOutputs()
        original_publish_speed = outputs.publish_speed

        def blocking_speed(value):
            original_publish_speed(value)
            if value != 0.0:
                entered.set()
                release.wait(1.0)

        outputs.publish_speed = blocking_speed
        controller = SearchController([0.5], outputs)
        controller.update_yaw(0.2)
        controller.start(0.0)
        tick_thread = threading.Thread(target=controller.tick, args=(0.1,))
        tick_thread.start()
        self.assertTrue(entered.wait(1.0))
        stop_started = threading.Event()

        def stop_controller():
            stop_started.set()
            controller.stop(0.2)

        stop_thread = threading.Thread(target=stop_controller)
        stop_thread.start()
        self.assertTrue(stop_started.wait(1.0))
        self.assertTrue(stop_thread.is_alive())

        release.set()
        tick_thread.join(1.0)
        stop_thread.join(1.0)

        self.assertEqual("IDLE", controller.state)
        self.assertEqual(0.0, outputs.speeds[-1])

    def test_nonzero_speed_failure_enters_error_and_attempts_zero(self):
        class FailingOutputs(RecordingOutputs):
            def publish_speed(self, value):
                self.speeds.append(value)
                if value != 0.0:
                    raise RuntimeError("motor publisher failed")

        outputs = FailingOutputs()
        errors = []
        controller = SearchController([0.5], outputs, error_handler=errors.append)
        controller.update_yaw(0.2)
        controller.start(0.0)

        controller.tick(0.1)

        self.assertEqual("ERROR", controller.state)
        self.assertEqual([0.0, 0.0, 0.3, 0.0], outputs.speeds)
        self.assertEqual(1, len(errors))

    def test_one_state_publisher_failure_does_not_block_other_outputs(self):
        class PartlyFailingOutputs(RecordingOutputs):
            def publish_scan_enabled(self, value):
                raise RuntimeError("scan publisher failed")

        outputs = PartlyFailingOutputs()
        errors = []
        controller = SearchController([0.0], outputs, error_handler=errors.append)
        initial_error_count = len(errors)

        controller.stop(0.0)

        self.assertEqual("ERROR", outputs.states[-1])
        self.assertEqual(0, outputs.walls[-1])
        self.assertEqual(0.0, outputs.speeds[-1])
        self.assertEqual(initial_error_count + 2, len(errors))

    def test_shutdown_zero_failure_does_not_escape(self):
        outputs = RecordingOutputs()
        controller = SearchController([0.0], outputs)
        outputs.publish_speed = Mock(side_effect=RuntimeError("down"))

        controller.shutdown()

    def test_old_search_observation_is_ignored(self):
        self.enter_scanning()

        self.controller.handle_observation(
            json.dumps(
                {"search_id": 0, "wall_index": 0, "status": "success"}
            ),
            0.6,
        )

        self.assertEqual("SCANNING", self.controller.state)

    def test_reset_publish_failure_prevents_start_and_enters_error(self):
        class ResetFailingOutputs(RecordingOutputs):
            def publish_reset(self, search_id):
                self.reset_ids.append(search_id)
                raise RuntimeError("reset down")

        outputs = ResetFailingOutputs()
        controller = SearchController([0.0], outputs)
        controller.update_yaw(0.0)

        self.assertFalse(controller.start(0.0))

        self.assertEqual("ERROR", controller.state)
        self.assertEqual(0.0, outputs.speeds[-1])
        self.assertEqual("ERROR", outputs.states[-1])

    def test_control_plane_failure_during_entry_enters_error(self):
        for failed_method in (
            "publish_speed",
            "publish_scan_enabled",
            "publish_wall",
        ):
            with self.subTest(failed_method=failed_method):
                outputs = RecordingOutputs()
                controller = SearchController([0.0], outputs)
                controller.update_yaw(0.0)

                def fail(*args):
                    raise RuntimeError("control publisher down")

                setattr(outputs, failed_method, fail)
                self.assertFalse(controller.start(0.0))
                self.assertEqual("ERROR", controller.state)
                self.assertEqual(0.0, outputs.speeds[-1])

    def test_error_degradation_output_failures_do_not_recurse(self):
        class AllFailingOutputs(RecordingOutputs):
            def __init__(self):
                super().__init__()
                self.calls = []

            def _fail(self, name):
                self.calls.append(name)
                raise RuntimeError(name)

            def publish_reset(self, search_id):
                self._fail("reset")

            def publish_speed(self, value):
                self._fail("speed")

            def publish_scan_enabled(self, value):
                self._fail("enabled")

            def publish_wall(self, value):
                self._fail("wall")

            def publish_state(self, value):
                self._fail("state")

        outputs = RecordingOutputs()
        controller = SearchController([0.0], outputs)
        controller.update_yaw(0.0)
        failing = AllFailingOutputs()
        controller._outputs = failing

        self.assertFalse(controller.start(0.0))

        self.assertEqual(
            ["reset", "speed", "enabled", "wall", "state"],
            failing.calls,
        )
        self.assertEqual("ERROR", controller.state)

    def test_no_now_calls_sample_clock_in_lock_order(self):
        first_clock_entered = threading.Event()
        release_first_clock = threading.Event()
        calls = []

        def controlled_clock():
            calls.append(len(calls) + 1)
            if len(calls) == 1:
                first_clock_entered.set()
                release_first_clock.wait(1.0)
            return float(len(calls))

        outputs = RecordingOutputs()
        controller = SearchController(
            [0.5],
            outputs,
            clock=controlled_clock,
        )
        controller.update_yaw(0.0)
        first = threading.Thread(target=controller.start)
        second = threading.Thread(target=controller.tick)
        first.start()
        self.assertTrue(first_clock_entered.wait(1.0))
        second.start()
        self.assertEqual([1], calls)

        release_first_clock.set()
        first.join(1.0)
        second.join(1.0)

        self.assertEqual([1, 2], calls)
        self.assertNotEqual("ERROR", controller.state)

    def test_clock_failure_or_non_finite_value_enters_safe_error(self):
        cases = (
            lambda: math.nan,
            lambda: math.inf,
            Mock(side_effect=RuntimeError("clock failed")),
        )
        for clock in cases:
            with self.subTest(clock=clock):
                outputs = RecordingOutputs()
                controller = SearchController([0.0], outputs, clock=clock)
                controller.update_yaw(0.0)

                self.assertFalse(controller.start())

                self.assertEqual("ERROR", controller.state)
                self.assertEqual(0.0, outputs.speeds[-1])

    def test_reset_publisher_is_latched_in_ros_adapter_ast(self):
        script = (
            Path(__file__).resolve().parent.parent
            / "scripts"
            / "item_search_controller_node.py"
        )
        tree = ast.parse(script.read_text(encoding="utf-8"))
        matching_calls = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not (
                isinstance(node.func, ast.Attribute)
                and node.func.attr == "Publisher"
                and node.args
                and isinstance(node.args[0], (ast.Str, ast.Constant))
            ):
                continue
            topic = (
                node.args[0].s
                if isinstance(node.args[0], ast.Str)
                else node.args[0].value
            )
            if topic == "/qr_item_search/reset":
                matching_calls.append(node)

        self.assertEqual(1, len(matching_calls))
        latch = next(
            (
                keyword.value
                for keyword in matching_calls[0].keywords
                if keyword.arg == "latch"
            ),
            None,
        )
        self.assertIsInstance(latch, (ast.NameConstant, ast.Constant))
        self.assertIs(True, latch.value)


if __name__ == "__main__":
    unittest.main()
