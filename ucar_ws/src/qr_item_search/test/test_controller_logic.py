import json
import math
import threading
import unittest
from unittest.mock import Mock

from qr_item_search.controller_logic import SearchController


class Outputs:
    def __init__(self):
        self.speeds, self.states, self.controls, self.results = [], [], [], []
    def publish_speed(self, value): self.speeds.append(value)
    def publish_state(self, value): self.states.append(value)
    def publish_scanner_control(self, value): self.controls.append(value)
    def publish_result(self, value): self.results.append(value)


def start_json(task="task", search="search"):
    return json.dumps({"protocol_version": 1, "task_id": task, "search_id": search, "expected_count": 3})


def event(kind, **fields):
    value = {"protocol_version": 1, "task_id": "task", "search_id": "search",
             "event": kind, "scanner_session": "session-a"}
    value.update(fields)
    return json.dumps(value)


class SearchControllerTest(unittest.TestCase):
    def setUp(self):
        self.out = Outputs()
        self.c = SearchController(self.out, camera_timeout=100.0)
        self.c.update_yaw(0.0, 0.0)

    def start_search(self, now=0.0):
        self.assertTrue(self.c.start(start_json(), now))
        return now

    def quality(self, now, decoded=False):
        return self.c.handle_scanner_event(event(
            "quality", brightness=50., overexposed=0., sharpness=50.,
            decoded=decoded, detected_yaw=0.), now)

    def detected(self, order, now, url=None, yaw=0.0):
        return self.c.handle_scanner_event(event(
            "detected", order=order, url=url or "https://%d" % order,
            detected_yaw=yaw, item_name="", message=""), now)

    def resolved(self, order, now, url=None, name=None, yaw=0.0):
        return self.c.handle_scanner_event(event(
            "resolved", order=order, url=url or "https://%d" % order,
            detected_yaw=yaw, item_name=name or "item%d" % order, message=""), now)

    def turn(self, t, start_deg, end_deg):
        """Simulate turning from start_deg to end_deg relative angle."""
        cur = math.radians(start_deg)
        target = math.radians(end_deg)
        direction = 1.0 if target >= cur else -1.0
        while direction * (target - cur) > 1e-9:
            t += 0.02
            cur += direction * 0.01
            self.c.update_yaw(cur, t, angular_speed=0.5)
            self.c.tick(t)
        return t

    def settle(self, t, deg):
        self.c.update_yaw(math.radians(deg), t, angular_speed=0.0)
        t += 0.25
        self.c.update_yaw(math.radians(deg), t, angular_speed=0.0)
        return t

    def station_cycle(self, t, start_deg, target_deg):
        """From a window open at time t, run window expiry, turn and settle;
        returns the time when the next window opens."""
        self.c.tick(t + 0.6 + 1e-6)
        t += 0.6
        t = self.turn(t, start_deg, target_deg)
        return self.settle(t, target_deg)

    def walk_pass(self, t, first_angle, step=45.0):
        """Walk one full pass; returns (t_after_last_window, last_deg)."""
        angles = [first_angle + index * step for index in range(8)]
        for index in range(8):
            if index + 1 < 8:
                t = self.station_cycle(t, angles[index], angles[index + 1])
            else:
                self.c.tick(t + 0.6 + 1e-6)
                t += 0.6
        return t, angles[-1]

    def test_initial_outputs_are_safe(self):
        self.assertEqual([0.0], self.out.speeds)
        self.assertEqual("IDLE", self.out.states[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_start_initial_station_scans_without_moving(self):
        self.start_search()
        self.c.tick(0.1)
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertTrue(self.out.controls[-1]["enabled"])
        control = self.out.controls[-1]
        self.assertEqual(0, control["pass_index"])
        self.assertEqual(0, control["station_index"])
        self.assertNotIn("capture_id", control)
        self.assertNotIn("capture_after", control)

    def test_shutdown_publishes_zero_before_slow_metrics_write_finishes(self):
        entered = threading.Event()
        release = threading.Event()

        def slow_metrics(_record):
            entered.set()
            self.assertTrue(release.wait(2.0))

        out = Outputs()
        controller = SearchController(out, camera_timeout=100.0, metrics=slow_metrics)
        controller.update_yaw(0.0, 0.0)
        self.assertTrue(controller.start(start_json(), 0.0))
        controller.tick(0.61)
        controller.update_yaw(math.radians(1.0), 0.62, angular_speed=0.5)
        controller.tick(0.62)
        self.assertNotEqual(0.0, out.speeds[-1])

        worker = threading.Thread(target=controller.shutdown)
        worker.start()
        self.assertTrue(entered.wait(1.0))
        self.assertEqual(0.0, out.speeds[-1])
        release.set()
        worker.join(2.0)
        self.assertFalse(worker.is_alive())

    def test_scanner_stays_enabled_across_dwell_and_turning(self):
        self.start_search()
        enabled = [value for value in self.out.controls if value["enabled"]]
        self.assertEqual(1, len(enabled))
        self.assertTrue(all(value["enhanced"] for value in enabled))
        self.c.tick(0.61)
        self.assertEqual("TURNING", self.c.state)
        self.assertTrue(self.out.controls[-1]["enabled"])
        self.c.update_yaw(math.radians(1.0), 0.62)
        self.c.tick(0.62)
        self.assertEqual(0.5, self.out.speeds[-1])
        self.assertTrue(self.out.controls[-1]["enabled"])

    def test_window_expiry_turns_to_next_station(self):
        self.start_search()
        self.c.tick(0.6)
        self.assertEqual("TURNING", self.c.state)
        self.assertAlmostEqual(math.radians(45.0), self.c._target_rel)
        self.assertTrue(self.out.controls[-1]["enabled"])
        self.assertEqual(1, self.out.controls[-1]["station_index"])

    def test_cruise_then_approach_speed(self):
        self.start_search()
        self.c.tick(0.6)
        self.c.update_yaw(math.radians(5.0), 0.7)
        self.c.tick(0.7)
        self.assertAlmostEqual(0.5, self.out.speeds[-1])
        self.c.update_yaw(math.radians(37.0), 0.8)
        self.c.tick(0.8)
        self.assertAlmostEqual(0.2, self.out.speeds[-1])

    def test_within_tolerance_zero_and_settling(self):
        self.start_search()
        self.c.tick(0.6)
        self.c.update_yaw(math.radians(43.5), 0.7)
        self.c.tick(0.7)
        self.assertEqual("SETTLING", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_settling_requires_continuous_low_angular_speed(self):
        self.start_search()
        self.c.tick(0.6)
        self.c.update_yaw(math.radians(44.0), 0.7)
        self.c.tick(0.7)
        self.assertEqual("SETTLING", self.c.state)
        self.c.update_yaw(math.radians(44.0), 0.8, angular_speed=0.05)
        self.assertEqual("SETTLING", self.c.state)
        self.c.update_yaw(math.radians(44.0), 1.05, angular_speed=0.0)
        self.assertEqual("SETTLING", self.c.state)
        self.c.update_yaw(math.radians(44.0), 1.30, angular_speed=0.0)
        self.assertEqual("SCANNING", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertTrue(self.out.controls[-1]["enabled"])
        self.assertEqual(0, self.out.controls[-1]["pass_index"])
        self.assertEqual(1, self.out.controls[-1]["station_index"])

    def test_detected_does_not_end_dwell_and_http_proceeds(self):
        self.start_search()
        self.assertTrue(self.detected(1, 0.1))
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.assertEqual(1, len(self.c._detected))
        self.assertTrue(self.out.controls[-1]["enabled"])
        self.resolved(1, 0.3)
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.assertEqual("item1", self.c._resolved[1])
        self.c.tick(0.61)
        self.assertEqual("TURNING", self.c.state)

    def test_three_urls_stop_and_wait_http(self):
        self.start_search()
        self.detected(1, 0.1)
        self.detected(2, 0.2)
        self.detected(3, 0.3)
        self.assertEqual("WAITING_HTTP", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertTrue(self.out.controls[-1]["enabled"])
        self.c.tick(0.5)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_three_urls_resolved_completes(self):
        self.start_search()
        for order in (1, 2, 3):
            self.detected(order, 0.1 + order / 100)
        for index, order in enumerate((3, 1, 2)):
            self.resolved(order, 0.3 + index / 100)
        self.assertEqual("COMPLETE", self.c.state)
        self.assertEqual("complete", self.out.results[-1]["status"])
        self.assertEqual([1, 2, 3], [item["order"] for item in self.out.results[-1]["items"]])

    def test_offset_pass_after_incomplete_first_pass(self):
        self.start_search()
        t, last_deg = self.walk_pass(0.0, 0.0)
        self.assertEqual("TURNING", self.c.state)
        self.assertAlmostEqual(math.radians(360.0 + 22.5), self.c._target_rel)
        self.assertEqual(1, self.c._pass_index)
        self.assertEqual(0, self.c._station_index)
        self.assertAlmostEqual(last_deg, 315.0)

    def test_second_pass_exhausted_reports_not_found(self):
        self.start_search()
        t, _ = self.walk_pass(0.0, 0.0)
        t = self.turn(t, 315.0, 360.0 + 22.5)
        t = self.settle(t, 360.0 + 22.5)
        t, _ = self.walk_pass(t, 360.0 + 22.5)
        self.assertEqual("NOT_FOUND", self.c.state)
        self.assertEqual("not_found", self.out.results[-1]["status"])
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_settling_timeout_errors(self):
        self.start_search()
        self.c.tick(0.6)
        self.c.update_yaw(math.radians(44.0), 0.7)
        self.c.tick(0.7)
        self.c.tick(0.8)
        self.assertEqual("SETTLING", self.c.state)
        self.c.update_yaw(math.radians(44.0), 3.0)
        self.c.tick(3.71)
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual("settling timeout", self.out.results[-1]["message"])
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_heading_timeout_errors(self):
        self.start_search()
        self.c.tick(0.6)
        self.c.update_yaw(math.radians(5.0), 0.7)
        self.c.tick(1.8)
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_camera_timeout_errors_during_window(self):
        out = Outputs()
        c = SearchController(out)
        c.update_yaw(0.0, 0.0)
        self.assertTrue(c.start(start_json(), 0.0))
        c.update_yaw(0.0, 1.05)
        c.tick(1.1)
        self.assertEqual("ERROR", c.state)
        self.assertEqual("camera timed out", out.results[-1]["message"])
        self.assertEqual(0.0, out.speeds[-1])

    def test_waiting_http_does_not_camera_timeout(self):
        self.start_search()
        for order in (1, 2, 3):
            self.detected(order, 0.1 + order / 100)
        self.c.tick(10)
        self.assertEqual("WAITING_HTTP", self.c.state)

    def test_total_timeout_finishes_not_found(self):
        self.start_search()
        self.c.tick(60.0)
        self.assertEqual("NOT_FOUND", self.c.state)
        self.assertEqual("not_found", self.out.results[-1]["status"])

    def test_resolve_error_in_waiting_http_requests_retry_once(self):
        self.start_search()
        for order in (1, 2, 3):
            self.detected(order, 0.1 + order / 100)
        self.c.handle_scanner_event(event("resolve_error", order=1, url="https://1",
                                          detected_yaw=0.0, item_name="", message="offline"), 0.3)
        self.assertEqual("WAITING_HTTP", self.c.state)
        retry_controls = [value for value in self.out.controls if value["retry_failed"]]
        self.assertEqual(1, len(retry_controls))
        self.assertTrue(retry_controls[0]["enabled"])
        self.c.tick(0.4)
        self.assertEqual("WAITING_HTTP", self.c.state)

    def test_unresolved_http_timeout_reports_reason(self):
        self.start_search()
        for order in (1, 2, 3):
            self.detected(order, 0.1 + order / 100)
        self.c.handle_scanner_event(event("resolve_error", order=1, url="https://1",
                                          detected_yaw=0.0, item_name="", message="offline"), 0.3)
        self.c.tick(60.0)
        self.assertEqual("NOT_FOUND", self.c.state)
        self.assertIn("unresolved", self.out.results[-1]["message"])

    def test_duplicate_detection_is_idempotent(self):
        self.start_search()
        self.detected(1, 0.1)
        self.detected(1, 0.2)
        self.assertEqual(1, len(self.c._detected))

    def test_conflicting_detection_enters_error(self):
        self.start_search()
        self.detected(1, 0.1)
        self.detected(1, 0.2, url="https://different")
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_same_url_different_order_enters_error(self):
        self.start_search()
        self.detected(1, 0.1, url="https://same")
        self.detected(2, 0.2, url="https://same")
        self.assertEqual("ERROR", self.c.state)

    def test_resolution_requires_detection(self):
        self.start_search()
        self.resolved(1, 0.2)
        self.assertEqual("ERROR", self.c.state)

    def test_stale_identity_is_ignored(self):
        self.start_search()
        raw = json.loads(event("quality", brightness=1., overexposed=0., sharpness=1.,
                               decoded=False, detected_yaw=0.))
        raw["search_id"] = "old"
        self.assertFalse(self.c.handle_scanner_event(json.dumps(raw), 0.1))
        self.assertEqual("INITIAL_SCAN", self.c.state)

    def test_stale_stop_is_ignored(self):
        self.start_search()
        raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "other",
                          "reason": "operator"})
        self.assertFalse(self.c.stop(raw, 0.2))
        self.assertEqual("INITIAL_SCAN", self.c.state)

    def test_stop_identity_finishes_stopped(self):
        self.start_search()
        raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search",
                          "reason": "operator"})
        self.assertTrue(self.c.stop(raw, 0.2))
        self.assertEqual("STOPPED", self.c.state)
        self.assertEqual("stopped", self.out.results[-1]["status"])
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_shutdown_always_stops(self):
        self.start_search()
        self.c.tick(0.1)
        self.c.shutdown()
        self.assertEqual(0., self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual("controller shutdown", self.out.results[-1]["message"])

    def test_fast_shutdown_prevents_later_tick_from_moving(self):
        self.start_search()
        self.c.tick(0.6)
        barrier = threading.Barrier(2)
        shutdown = threading.Thread(target=lambda: (self.c.shutdown(), barrier.wait()))
        tick = threading.Thread(target=lambda: (barrier.wait(), self.c.tick(0.8)))
        shutdown.start(); tick.start(); shutdown.join(1); tick.join(1)
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual(0., self.out.speeds[-1])
        shutdown_index = len(self.out.speeds) - 1
        self.assertFalse(any(value != 0.0 for value in self.out.speeds[shutdown_index:]))

    def test_shutdown_latch_rejects_new_search_after_active_shutdown(self):
        self.start_search()
        self.c.shutdown()
        self.assertFalse(self.c.start(start_json(search="new"), 0.2))
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual(0., self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_idle_shutdown_preserves_idle_and_rejects_start(self):
        out = Outputs(); c = SearchController(out)
        c.shutdown()
        self.assertEqual("IDLE", c.state)
        self.assertFalse(c.start(start_json(), 0))
        self.assertEqual("IDLE", c.state)
        self.assertFalse(out.controls[-1]["enabled"])

    def test_duplicate_active_start_is_idempotent(self):
        self.start_search()
        self.assertFalse(self.c.start(start_json(), 0.1))
        self.assertEqual("INITIAL_SCAN", self.c.state)

    def test_different_active_start_is_rejected(self):
        self.start_search()
        self.assertFalse(self.c.start(start_json(search="other"), 0.1))
        self.assertEqual("INITIAL_SCAN", self.c.state)

    def test_new_search_resets_progress(self):
        self.start_search()
        self.detected(1, 0.1)
        self.resolved(1, 0.2)
        self.c.stop(json.dumps({"protocol_version": 1, "task_id": "task",
                                "search_id": "search", "reason": "done"}), 0.3)
        self.assertTrue(self.c.start(start_json(search="next"), 0.4))
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.assertEqual({}, self.c._detected)
        self.assertEqual({}, self.c._resolved)
        self.assertEqual(0, self.c._pass_index)
        self.assertEqual(0, self.c._station_index)

    def test_start_without_yaw_errors_safely(self):
        c = SearchController(Outputs())
        self.assertFalse(c.start(start_json(), 0))
        self.assertEqual("ERROR", c.state)

    def test_stale_heading_start_errors(self):
        c = SearchController(Outputs()); c.update_yaw(0, 0)
        self.assertFalse(c.start(start_json(), 1.1))
        self.assertEqual("ERROR", c.state)

    def test_invalid_protocol_start_does_not_move(self):
        self.assertFalse(self.c.start("bad", 0.1))
        self.assertEqual(0., self.out.speeds[-1])

    def test_time_rollback_errors(self):
        self.start_search()
        self.assertFalse(self.c.tick(-0.1))
        self.assertEqual("ERROR", self.c.state)

    def test_nonfinite_time_and_yaw_rejected(self):
        with self.assertRaises(ValueError): self.c.update_yaw(True, 0.1)
        with self.assertRaises(ValueError): self.c.tick(math.inf)
        with self.assertRaises(ValueError):
            self.c.update_yaw(0.0, 0.1, angular_speed=math.nan)

    def test_invalid_configuration_rejected(self):
        for kwargs in ({"step_angle_deg": 0}, {"cruise_angular_speed": True},
                       {"approach_angular_speed": -1.0}, {"approach_zone_deg": -1},
                       {"yaw_tolerance_deg": -1}, {"settled_angular_speed": -0.1},
                       {"settled_duration": math.inf}, {"scan_window": 0},
                       {"offset_angle_deg": -1}, {"max_passes": 0},
                       {"search_total_timeout": -1}, {"settling_timeout": 0},
                       {"heading_timeout": 0}, {"camera_timeout": math.inf},
                       {"step_angle_deg": 360.0}, {"step_angle_deg": 100.0},
                       {"yaw_tolerance_deg": 20.0, "approach_zone_deg": 10.0}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    SearchController(Outputs(), **kwargs)

    def test_malformed_current_event_errors(self):
        self.start_search()
        self.c.handle_scanner_event("{bad", 0.1)
        self.assertEqual("ERROR", self.c.state)

    def test_publisher_nonzero_failure_enters_error_and_zero(self):
        out = Outputs()
        def speed(value):
            out.speeds.append(value)
            if value: raise RuntimeError("motor")
        out.publish_speed = speed
        c = SearchController(out, camera_timeout=100.0); c.update_yaw(0, 0)
        c.start(start_json(), 0); c.tick(0.6); c.tick(0.61)
        self.assertEqual("ERROR", c.state)
        self.assertEqual(0., out.speeds[-1])

    def test_clock_error_enters_safe_error(self):
        c = SearchController(Outputs(), clock=Mock(side_effect=RuntimeError("clock")))
        self.assertFalse(c.tick())
        self.assertEqual("ERROR", c.state)

    def test_concurrent_stop_and_tick_leave_zero_speed(self):
        self.start_search()
        stop_raw = json.dumps({"protocol_version": 1, "task_id": "task",
                               "search_id": "search", "reason": "stop"})
        barrier = threading.Barrier(2)
        first = threading.Thread(target=lambda: (barrier.wait(), self.c.tick(0.6)))
        second = threading.Thread(target=lambda: (barrier.wait(), self.c.stop(stop_raw, 0.6)))
        first.start(); second.start(); first.join(1); second.join(1)
        self.assertEqual("STOPPED", self.c.state)
        self.assertEqual(0., self.out.speeds[-1])

    def test_start_enabled_control_failure_degrades_to_safe_error(self):
        out, errors = Outputs(), Mock()
        original = out.publish_scanner_control
        def control(value):
            if value["enabled"]: raise RuntimeError("control")
            original(value)
        out.publish_scanner_control = control
        c = SearchController(out, error_handler=errors, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        self.assertEqual("ERROR", c.state)
        self.assertEqual(0., out.speeds[-1])
        self.assertFalse(out.controls[-1]["enabled"])
        errors.assert_called()

    def test_initial_control_has_all_protocol_fields(self):
        self.assertEqual({
            "protocol_version": 1, "task_id": "", "search_id": "",
            "enabled": False, "enhanced": False, "detected_yaw": 0.0,
            "retry_failed": False,
        }, self.out.controls[0])

    def test_late_current_events_ignored_after_terminal_states(self):
        self.start_search()
        self.c.tick(60.0)
        self.assertFalse(self.c.handle_scanner_event(event(
            "resolved", order=1, url="https://1", detected_yaw=0.0,
            item_name="late", message=""), 61))
        self.assertEqual("NOT_FOUND", self.c.state)

    def test_scanner_session_restart_fails_active_search(self):
        hello = lambda session: json.dumps({"protocol_version": 1, "event": "scanner_started",
                                             "scanner_session": session, "task_id": "", "search_id": ""})
        self.assertFalse(self.c.handle_scanner_event(hello("session-1"), 0.0))
        self.start_search()
        self.assertFalse(self.c.handle_scanner_event(hello("session-2"), 0.1))
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual("scanner restarted", self.out.results[-1]["message"])
        self.assertTrue(self.c.start(start_json(search="new-search"), 0.2))

    def test_active_ordinary_event_requires_nonempty_scanner_session(self):
        self.start_search()
        payload = json.loads(event("quality", brightness=1., overexposed=0.,
                                   sharpness=1., decoded=False, detected_yaw=0.0))
        payload.pop("scanner_session", None)
        self.assertFalse(self.c.handle_scanner_event(json.dumps(payload), 0.1))
        self.assertEqual("ERROR", self.c.state)

    def test_debug_snapshot_reports_state_and_progress(self):
        self.start_search()
        self.detected(1, 0.1)
        snapshot = self.c.debug_snapshot()
        self.assertEqual("INITIAL_SCAN", snapshot["state"])
        self.assertEqual(1, snapshot["found_count"])
        self.assertEqual(3, snapshot["expected_count"])
        self.assertEqual("task", snapshot["task_id"])
        self.assertEqual("search", snapshot["search_id"])
        self.assertIsInstance(snapshot["relative_yaw_deg"], float)

    def test_debug_snapshot_reports_unsettled_until_scanning_starts(self):
        self.start_search()
        self.c.tick(0.61)
        self.c.update_yaw(math.radians(45.0), 0.62, angular_speed=0.2)
        self.c.tick(0.62)
        self.assertEqual("SETTLING", self.c.state)
        self.assertFalse(self.c.debug_snapshot()["settled"])

        self.c.update_yaw(math.radians(45.0), 0.70, angular_speed=0.0)
        self.c.update_yaw(math.radians(45.0), 0.95, angular_speed=0.0)
        self.assertEqual("SCANNING", self.c.state)
        self.assertTrue(self.c.debug_snapshot()["settled"])

    def test_terminal_shutdown_preserves_state_and_last_result(self):
        cases = []

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        for order in (1, 2, 3):
            c.handle_scanner_event(event("detected", order=order, url="https://%d" % order,
                                         detected_yaw=0.0), 0.1 + order / 100)
        for index, order in enumerate((1, 2, 3)):
            c.handle_scanner_event(event("resolved", order=order, url="https://%d" % order,
                                         detected_yaw=0.0, item_name=str(order), message=""),
                                   0.3 + index / 100)
        cases.append((c, out, "COMPLETE"))

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        c.stop(json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search",
                           "reason": "stop"}), 0.1)
        cases.append((c, out, "STOPPED"))

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0); c.tick(60)
        cases.append((c, out, "NOT_FOUND"))

        for c, out, state in cases:
            with self.subTest(state=state):
                result = out.results[-1]
                count = len(out.results)
                c.shutdown()
                self.assertEqual(state, c.state)
                self.assertEqual(count, len(out.results))
                self.assertIs(result, out.results[-1])
                self.assertFalse(c.start(start_json(search="new"), 61))

    def test_metrics_record_at_terminal(self):
        records = []
        c = SearchController(Outputs(), camera_timeout=100.0, metrics=records.append)
        c.update_yaw(0, 0)
        c.start(start_json(), 0)
        for order in (1, 2, 3):
            c.handle_scanner_event(event("detected", order=order, url="https://%d" % order,
                                         detected_yaw=0.0), 0.1 + order / 100)
        for index, order in enumerate((1, 2, 3)):
            c.handle_scanner_event(event("resolved", order=order, url="https://%d" % order,
                                         detected_yaw=0.0, item_name=str(order), message=""),
                                   0.3 + index / 100)
        self.assertEqual("COMPLETE", c.state)
        self.assertEqual(1, len(records))
        record = records[0]
        self.assertEqual("complete", record["terminal_status"])
        self.assertEqual("search", record["search_id"])
        self.assertEqual("task", record["task_id"])
        self.assertEqual(45.0, record["config"]["step_angle_deg"])
        self.assertIn("total_seconds", record)
        self.assertIn("state_seconds", record)
        self.assertIn("stations", record)
        self.assertEqual(3, len(record["items"]))

    def test_metrics_station_record_fields(self):
        records = []
        c = SearchController(Outputs(), camera_timeout=100.0, metrics=records.append)
        c.update_yaw(0, 0)
        c.start(start_json(), 0)
        c.tick(0.6 + 1e-6)
        c.tick(60.0)
        self.assertEqual("NOT_FOUND", c.state)
        self.assertEqual(1, len(records))
        station = records[0]["stations"][0]
        for key in ("pass", "index", "target_yaw_deg", "actual_yaw_deg", "yaw_error_deg",
                    "settling_seconds", "dwell_seconds", "frame_count", "quality", "decoded_url"):
            self.assertIn(key, station)
        self.assertEqual(0, station["pass"])
        self.assertEqual(0, station["index"])
        self.assertEqual(0.0, station["target_yaw_deg"])
        self.assertEqual(0.0, station["actual_yaw_deg"])
        self.assertIsNone(station["decoded_url"])
        self.assertEqual(0, station["frame_count"])

    def test_metrics_failure_does_not_break_terminal(self):
        errors = Mock()
        c = SearchController(Outputs(), camera_timeout=100.0,
                             metrics=Mock(side_effect=RuntimeError("disk")),
                             error_handler=errors)
        c.update_yaw(0, 0)
        c.start(start_json(), 0)
        c.tick(60.0)
        self.assertEqual("NOT_FOUND", c.state)
        self.assertEqual(0.0, c.out.speeds[-1] if hasattr(c, "out") else 0.0)
        errors.assert_called()

    def test_metrics_not_called_without_task(self):
        metrics = Mock()
        c = SearchController(Outputs(), metrics=metrics)
        c.shutdown()
        metrics.assert_not_called()

    def test_metrics_recorded_after_stop(self):
        records = []
        c = SearchController(Outputs(), camera_timeout=100.0, metrics=records.append)
        c.update_yaw(0, 0)
        c.start(start_json(), 0)
        c.stop(json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search",
                           "reason": "stop"}), 0.2)
        self.assertEqual(1, len(records))
        self.assertEqual("stopped", records[0]["terminal_status"])

    def test_all_terminal_states_finish_with_safe_outputs(self):
        terminal_outputs = {}

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        for order in (1, 2, 3):
            c.handle_scanner_event(event("detected", order=order, url="https://%d" % order,
                                         detected_yaw=0.0), 0.1 + order / 100)
        for index, order in enumerate((1, 2, 3)):
            c.handle_scanner_event(event("resolved", order=order, url="https://%d" % order,
                                         detected_yaw=0.0, item_name=str(order), message=""),
                                   0.3 + index / 100)
        terminal_outputs["COMPLETE"] = out

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0); c.tick(60)
        terminal_outputs["NOT_FOUND"] = out

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.start(start_json(), 0)
        terminal_outputs["ERROR"] = out

        out = Outputs(); c = SearchController(out, camera_timeout=100.0)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        c.stop(json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search",
                           "reason": "stop"}), 0.1)
        terminal_outputs["STOPPED"] = out

        for state, out in terminal_outputs.items():
            with self.subTest(state=state):
                self.assertEqual(0., out.speeds[-1])
                self.assertFalse(out.controls[-1]["enabled"])
                self.assertFalse(out.controls[-1]["enhanced"])


if __name__ == "__main__":
    unittest.main()
