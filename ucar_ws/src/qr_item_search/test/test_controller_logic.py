import json
import math
import threading
import unittest
from unittest.mock import Mock

from qr_item_search.controller_logic import SearchController
from qr_item_search.sweep_coverage import CoverageMap, Interval


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
    value = {"protocol_version": 1, "task_id": "task", "search_id": "search", "event": kind}
    value.update(fields)
    return json.dumps(value)


class SearchControllerTest(unittest.TestCase):
    def setUp(self):
        self.out = Outputs()
        self.coverage = CoverageMap(sector_count=4, minimum_frames=1, margin=0)
        self.c = SearchController(self.out, coverage=self.coverage)
        self.c.update_yaw(0.0, 0.0)

    def start(self):
        self.assertTrue(self.c.start(start_json(), 0.0))

    def quality(self, now=.1, yaw=.1, decoded=False):
        return self.c.handle_scanner_event(event("quality", brightness=50., overexposed=0., sharpness=50.,
                                                 decoded=decoded, detected_yaw=yaw), now)

    def detected(self, order, url=None, yaw=.2, now=.2):
        return self.c.handle_scanner_event(event("detected", order=order, url=url or "https://%d" % order,
                                                 detected_yaw=yaw, item_name="", message=""), now)

    def resolved(self, order, url=None, name=None, yaw=.2, now=.3):
        return self.c.handle_scanner_event(event("resolved", order=order, url=url or "https://%d" % order,
                                                 detected_yaw=yaw, item_name=name or "item%d" % order, message=""), now)

    def test_initial_outputs_are_safe(self):
        self.assertEqual([0.0], self.out.speeds)
        self.assertEqual("IDLE", self.out.states[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_start_then_tick_publishes_fast_speed(self):
        self.start(); self.c.tick(.1)
        self.assertEqual("FAST_SWEEP", self.c.state)
        self.assertEqual(.4, self.out.speeds[-1])
        self.assertEqual("searching", self.out.results[-1]["status"])

    def test_fast_sweep_uses_configured_380_degree_angle(self):
        self.start()
        for index in range(1, 7):
            self.c.update_yaw(index, index / 10); self.quality(index / 10 + .001)
        self.assertEqual("FAST_SWEEP", self.c.state)
        self.c.update_yaw(6.64, .7); self.c.tick(.7)
        self.assertEqual("TARGETED_RESCAN", self.c.state)

    def test_third_detection_stops_and_waits_http(self):
        self.start()
        for order in (1, 2, 3): self.detected(order, now=.1 + order / 100)
        self.assertEqual("WAITING_HTTP", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_reverse_resolution_finishes_sorted(self):
        self.start()
        for order in (1, 2, 3): self.detected(order, yaw=order, now=.1 + order / 100)
        for index, order in enumerate((3, 1, 2)): self.resolved(order, yaw=order, now=.2 + index / 100)
        self.assertEqual("COMPLETE", self.c.state)
        self.assertEqual([1, 2, 3], [item["order"] for item in self.out.results[-1]["items"]])

    def test_quality_records_coverage(self):
        self.start(); self.quality(decoded=True)
        self.assertEqual(1, self.coverage.sectors[0].frame_count)
        self.assertTrue(self.coverage.sectors[0].decoded)

    def test_invalid_quality_enters_error(self):
        self.start()
        self.c.handle_scanner_event(event("quality", brightness=True, overexposed=0., sharpness=1.,
                                          decoded=False, detected_yaw=0.), .1)
        self.assertEqual("ERROR", self.c.state)

    def test_stale_identity_is_ignored(self):
        self.start()
        raw = json.loads(event("quality", brightness=1., overexposed=0., sharpness=1., decoded=False, detected_yaw=0.))
        raw["search_id"] = "old"
        self.assertFalse(self.c.handle_scanner_event(json.dumps(raw), .1))
        self.assertEqual("FAST_SWEEP", self.c.state)

    def test_duplicate_detection_is_idempotent(self):
        self.start(); self.detected(1); self.detected(1, now=.21)
        self.assertEqual("FAST_SWEEP", self.c.state)

    def test_conflicting_detection_enters_error(self):
        self.start(); self.detected(1)
        self.detected(1, url="https://different", now=.21)
        self.assertEqual("ERROR", self.c.state)

    def test_same_url_different_order_enters_error(self):
        self.start(); self.detected(1, url="https://same")
        self.detected(2, url="https://same", now=.21)
        self.assertEqual("ERROR", self.c.state)

    def test_resolution_requires_detection(self):
        self.start(); self.resolved(1)
        self.assertEqual("ERROR", self.c.state)

    def test_resolve_error_in_waiting_resumes_enhanced_retry(self):
        self.start()
        for order in (1, 2, 3): self.detected(order, now=.1 + order / 100)
        self.c.handle_scanner_event(event("resolve_error", order=1, url="https://1", detected_yaw=.2,
                                          item_name="", message="offline"), .3)
        self.assertEqual("TARGETED_RESCAN", self.c.state)
        self.assertTrue(self.out.controls[-1]["enhanced"])
        self.assertTrue(self.out.controls[-1]["retry_failed"])

    def test_retry_flag_only_on_first_control(self):
        self.test_resolve_error_in_waiting_resumes_enhanced_retry()
        self.c.update_yaw(.1, .4)
        self.assertFalse(self.out.controls[-1]["retry_failed"])

    def test_total_timeout_finishes_not_found(self):
        self.start(); self.c.tick(40.0)
        self.assertEqual("NOT_FOUND", self.c.state)
        self.assertEqual("not_found", self.out.results[-1]["status"])

    def test_heading_timeout_errors(self):
        self.start(); self.c.tick(1.1)
        self.assertEqual("ERROR", self.c.state)

    def test_camera_timeout_errors(self):
        self.start(); self.c.update_yaw(.1, .5); self.c.tick(1.1)
        self.assertEqual("ERROR", self.c.state)

    def test_waiting_http_does_not_camera_timeout(self):
        self.start()
        for order in (1, 2, 3): self.detected(order, now=.1 + order / 100)
        self.c.tick(10)
        self.assertEqual("WAITING_HTTP", self.c.state)

    def test_stop_identity_finishes_stopped(self):
        self.start()
        raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "operator"})
        self.assertTrue(self.c.stop(raw, .2))
        self.assertEqual("STOPPED", self.c.state)
        self.assertEqual("stopped", self.out.results[-1]["status"])

    def test_stale_stop_is_ignored(self):
        self.start()
        raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "other", "reason": "operator"})
        self.assertFalse(self.c.stop(raw, .2))
        self.assertEqual("FAST_SWEEP", self.c.state)

    def test_shutdown_always_stops(self):
        self.start(); self.c.tick(.1); self.c.shutdown()
        self.assertEqual(0., self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_duplicate_active_start_is_idempotent(self):
        self.start()
        self.assertFalse(self.c.start(start_json(), .1))
        self.assertEqual("FAST_SWEEP", self.c.state)

    def test_different_active_start_is_rejected(self):
        self.start()
        self.assertFalse(self.c.start(start_json(search="other"), .1))
        self.assertEqual("FAST_SWEEP", self.c.state)

    def test_new_search_resets_coverage(self):
        self.start(); self.quality(); self.c.stop(json.dumps(
            {"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "done"}), .2)
        self.c.start(start_json(search="next"), .3)
        self.assertEqual(0, sum(sector.frame_count for sector in self.coverage.sectors))

    def test_start_without_yaw_errors_safely(self):
        c = SearchController(Outputs())
        self.assertFalse(c.start(start_json(), 0))
        self.assertEqual("ERROR", c.state)

    def test_stale_heading_start_errors(self):
        c = SearchController(Outputs()); c.update_yaw(0, 0)
        self.assertFalse(c.start(start_json(), 1.1))
        self.assertEqual("ERROR", c.state)

    def test_invalid_protocol_start_does_not_move(self):
        self.assertFalse(self.c.start("bad", .1))
        self.assertEqual(0., self.out.speeds[-1])

    def test_time_rollback_errors(self):
        self.start(); self.assertFalse(self.c.tick(-.1))
        self.assertEqual("ERROR", self.c.state)

    def test_nonfinite_time_and_yaw_rejected(self):
        with self.assertRaises(ValueError): self.c.update_yaw(True, .1)
        with self.assertRaises(ValueError): self.c.tick(math.inf)

    def test_invalid_configuration_rejected(self):
        for kwargs in ({"fast_angular_speed": 0}, {"targeted_angular_speed": True},
                       {"minimum_effective_speed": .5}, {"fast_sweep_angle": 0},
                       {"yaw_tolerance": -1}, {"heading_timeout": 0},
                       {"camera_timeout": math.inf}, {"search_total_timeout": -1}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError): SearchController(Outputs(), **kwargs)

    def test_malformed_current_event_errors(self):
        self.start(); self.c.handle_scanner_event("{bad", .1)
        self.assertEqual("ERROR", self.c.state)

    def test_targeted_approach_and_sweep(self):
        coverage = Mock()
        coverage.reset = Mock()
        coverage.rescan_intervals.return_value = [Interval(1., 2., 0)]
        c = SearchController(self.out, coverage=coverage, fast_sweep_angle=.5)
        c.update_yaw(0, 0); c.start(start_json(), 0); c.update_yaw(.6, .1); c.tick(.1)
        self.assertEqual("TARGETED_RESCAN", c.state)
        c.tick(.11); self.assertGreater(self.out.speeds[-1], 0)
        c.update_yaw(1., .2); c.tick(.2)
        c.update_yaw(2., .3); c.tick(.3)
        self.assertEqual("NOT_FOUND", c.state)

    def test_publisher_nonzero_failure_enters_error_and_zero(self):
        out = Outputs()
        def speed(value):
            out.speeds.append(value)
            if value: raise RuntimeError("motor")
        out.publish_speed = speed
        c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0); c.tick(.1)
        self.assertEqual("ERROR", c.state)
        self.assertEqual(0., out.speeds[-1])

    def test_clock_error_enters_safe_error(self):
        c = SearchController(Outputs(), clock=Mock(side_effect=RuntimeError("clock")))
        self.assertFalse(c.tick())
        self.assertEqual("ERROR", c.state)

    def test_concurrent_stop_and_tick_leave_zero_speed(self):
        self.start()
        stop_raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"})
        barrier = threading.Barrier(2)
        first = threading.Thread(target=lambda: (barrier.wait(), self.c.tick(.2)))
        second = threading.Thread(target=lambda: (barrier.wait(), self.c.stop(stop_raw, .2)))
        first.start(); second.start(); first.join(1); second.join(1)
        self.assertEqual("STOPPED", self.c.state)
        self.assertEqual(0., self.out.speeds[-1])

    def test_terminal_and_waiting_states_tick_zero(self):
        self.start()
        for order in (1, 2, 3): self.detected(order, now=.1 + order / 100)
        self.c.tick(.5)
        self.assertEqual(0., self.out.speeds[-1])

    def test_blocking_error_publisher_does_not_hold_state_lock(self):
        entered, release = threading.Event(), threading.Event()
        out = Outputs()
        def result(value): entered.set(); release.wait(1); out.results.append(value)
        out.publish_result = result
        c = SearchController(out)
        worker = threading.Thread(target=c.start, args=(start_json(), 0))
        worker.start(); self.assertTrue(entered.wait(1))
        update = threading.Thread(target=c.update_yaw, args=(0, .1))
        update.start(); update.join(.2)
        self.assertFalse(update.is_alive())
        release.set(); worker.join(1)

    def test_initial_control_has_all_protocol_fields(self):
        self.assertEqual({
            "protocol_version": 1, "task_id": "", "search_id": "",
            "enabled": False, "enhanced": False, "detected_yaw": 0.0,
            "retry_failed": False,
        }, self.out.controls[0])

    def test_same_search_id_ignores_task_for_active_and_completed_idempotency(self):
        self.start()
        self.assertFalse(self.c.start(start_json(task="other"), .1))
        for order in (1, 2, 3): self.detected(order, yaw=order, now=.11 + order / 100)
        for index, order in enumerate((1, 2, 3)): self.resolved(order, yaw=order, now=.2 + index / 100)
        result = self.out.results[-1]
        self.assertFalse(self.c.start(start_json(task="other"), .4))
        self.assertIs(result, self.out.results[-1])

    def test_stale_identity_ignores_older_event_and_stop_times(self):
        self.start(); self.c.tick(.5)
        stale = json.loads(event("quality", brightness=1., overexposed=0., sharpness=1.,
                                 decoded=False, detected_yaw=0.))
        stale["search_id"] = "old"
        self.assertFalse(self.c.handle_scanner_event(json.dumps(stale), -.1))
        stop = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "old", "reason": "old"})
        self.assertFalse(self.c.stop(stop, -.2))
        self.assertEqual("FAST_SWEEP", self.c.state)

    def test_target_sector_zero_maps_near_two_pi(self):
        coverage = Mock(); coverage.reset = Mock()
        coverage.rescan_intervals.return_value = [Interval(0., .2, 0)]
        c = SearchController(self.out, coverage=coverage, fast_sweep_angle=5.5)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        for index in range(1, 7):
            c.update_yaw(float(index), index / 10)
            c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                         decoded=False, detected_yaw=index), index / 10 + .001)
        c.tick(.7)
        self.assertGreater(c._intervals[0][0], 6.0)

    def test_targeted_approach_can_command_negative_speed(self):
        coverage = Mock(); coverage.reset = Mock()
        coverage.rescan_intervals.return_value = [Interval(-1., -.5, 0)]
        c = SearchController(self.out, coverage=coverage, fast_sweep_angle=.5)
        c.update_yaw(0, 0); c.start(start_json(), 0); c.update_yaw(.6, .1)
        c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                     decoded=False, detected_yaw=.6), .11)
        c.tick(.12); c.tick(.13)
        self.assertLess(self.out.speeds[-1], 0)

    def test_partial_not_found_uses_detection_order_and_renumbers(self):
        coverage = Mock(); coverage.reset = Mock(); coverage.rescan_intervals.return_value = []
        c = SearchController(self.out, coverage=coverage, fast_sweep_angle=.5)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        for order in (1, 2):
            c.handle_scanner_event(event("detected", order=order, url="https://%d" % order,
                                         detected_yaw=float(order)), .1 + order / 100)
        for index, order in enumerate((2, 1)):
            c.handle_scanner_event(event("resolved", order=order, url="https://%d" % order,
                                         detected_yaw=float(order), item_name=str(order), message=""), .2 + index / 100)
        c.update_yaw(.6, .3)
        c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                     decoded=False, detected_yaw=.6), .31)
        c.tick(.32); c.tick(.33)
        self.assertEqual(["https://1", "https://2"], [item["url"] for item in self.out.results[-1]["items"]])
        self.assertEqual([1, 2], [item["order"] for item in self.out.results[-1]["items"]])

    def test_blocked_nonzero_publish_is_followed_by_stop_zero(self):
        entered, release = threading.Event(), threading.Event()
        out = Outputs()
        def speed(value):
            out.speeds.append(value)
            if value: entered.set(); release.wait(1)
        out.publish_speed = speed
        c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0)
        tick = threading.Thread(target=c.tick, args=(.1,)); tick.start(); self.assertTrue(entered.wait(1))
        raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"})
        stop = threading.Thread(target=c.stop, args=(raw, .2)); stop.start()
        release.set(); tick.join(1); stop.join(1)
        self.assertEqual(0., out.speeds[-1])

    def test_targeted_rescan_advances_multiple_intervals_in_given_order(self):
        coverage = Mock(); coverage.reset = Mock()
        coverage.rescan_intervals.return_value = [Interval(.5, .6, 0), Interval(1., 1.1, 1)]
        c = SearchController(self.out, coverage=coverage, fast_sweep_angle=.2)
        c.update_yaw(0, 0); c.start(start_json(), 0); c.update_yaw(.3, .1)
        c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                     decoded=False, detected_yaw=.3), .11)
        c.tick(.12)
        self.assertEqual([(.5, .6), (1., 1.1)], c._intervals)
        c.update_yaw(.5, .2); c.tick(.2)
        c.update_yaw(.7, .3); c.tick(.3)
        self.assertEqual(1, c._interval_index)
        c.update_yaw(1., .4); c.tick(.4)
        c.update_yaw(1.2, .5); c.tick(.5)
        self.assertEqual("NOT_FOUND", c.state)

    def test_missing_scanner_identity_is_malformed_current_event(self):
        for payload in (
            {"protocol_version": 1, "event": "quality"},
            {"protocol_version": 1, "task_id": "task", "event": "quality"},
            {"protocol_version": 1, "search_id": "search", "event": "quality"},
            {"protocol_version": 1, "task_id": "", "search_id": "search", "event": "quality"},
        ):
            with self.subTest(payload=payload):
                out = Outputs(); c = SearchController(out)
                c.update_yaw(0, 0); c.start(start_json(), 0)
                self.assertFalse(c.handle_scanner_event(json.dumps(payload), .1))
                self.assertEqual("ERROR", c.state)
                self.assertEqual(0., out.speeds[-1])

    def test_partial_not_found_sorts_by_numeric_detection_order(self):
        coverage = Mock(); coverage.reset = Mock(); coverage.rescan_intervals.return_value = []
        out = Outputs(); c = SearchController(out, coverage=coverage, fast_sweep_angle=.5)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        for index, order in enumerate((2, 1)):
            c.handle_scanner_event(event("detected", order=order, url="https://%d" % order,
                                         detected_yaw=float(order)), .1 + index / 100)
        for index, order in enumerate((2, 1)):
            c.handle_scanner_event(event("resolved", order=order, url="https://%d" % order,
                                         detected_yaw=float(order), item_name=str(order), message=""), .2 + index / 100)
        c.update_yaw(.6, .3)
        c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                     decoded=False, detected_yaw=.6), .31)
        c.tick(.32); c.tick(.33)
        self.assertEqual(["https://1", "https://2"], [item["url"] for item in out.results[-1]["items"]])
        self.assertEqual([1, 2], [item["order"] for item in out.results[-1]["items"]])

    def test_all_terminal_states_finish_with_safe_outputs(self):
        terminal_outputs = {}

        out = Outputs(); c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0)
        for order in (1, 2, 3):
            c.handle_scanner_event(event("detected", order=order, url="https://%d" % order,
                                         detected_yaw=float(order)), .1 + order / 100)
        for index, order in enumerate((1, 2, 3)):
            c.handle_scanner_event(event("resolved", order=order, url="https://%d" % order,
                                         detected_yaw=float(order), item_name=str(order), message=""), .2 + index / 100)
        terminal_outputs["COMPLETE"] = out

        out = Outputs(); c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0); c.tick(40)
        terminal_outputs["NOT_FOUND"] = out

        out = Outputs(); c = SearchController(out); c.start(start_json(), 0)
        terminal_outputs["ERROR"] = out

        out = Outputs(); c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0)
        c.stop(json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"}), .1)
        terminal_outputs["STOPPED"] = out

        for state, out in terminal_outputs.items():
            with self.subTest(state=state):
                self.assertEqual(0., out.speeds[-1])
                self.assertFalse(out.controls[-1]["enabled"])
                self.assertFalse(out.controls[-1]["enhanced"])

    def test_early_resolve_error_enters_targeted_when_third_url_arrives(self):
        self.start()
        self.detected(1, now=.1)
        self.c.handle_scanner_event(event("resolve_error", order=1, url="https://1", detected_yaw=.2,
                                          item_name="", message="offline"), .11)
        self.detected(2, now=.12); self.detected(3, now=.13)
        self.assertEqual("TARGETED_RESCAN", self.c.state)
        self.assertTrue(self.out.controls[-1]["retry_failed"])
        self.c.update_yaw(.1, .14)
        self.assertFalse(self.out.controls[-1]["retry_failed"])

    def test_fast_sweep_finish_with_early_error_enters_targeted_retry(self):
        coverage = Mock(); coverage.reset = Mock(); coverage.rescan_intervals.return_value = [Interval(0, .2, 0)]
        out = Outputs(); c = SearchController(out, coverage=coverage, fast_sweep_angle=.5)
        c.update_yaw(0, 0); c.start(start_json(), 0)
        c.handle_scanner_event(event("detected", order=1, url="https://1", detected_yaw=.2), .1)
        c.handle_scanner_event(event("resolve_error", order=1, url="https://1", detected_yaw=.2,
                                     item_name="", message="offline"), .11)
        c.update_yaw(.6, .2)
        c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                     decoded=False, detected_yaw=.6), .21)
        c.tick(.22)
        self.assertEqual("TARGETED_RESCAN", c.state)
        self.assertTrue(out.controls[-1]["retry_failed"])

    def test_late_current_events_are_ignored_after_all_terminal_states(self):
        terminal = []
        out = Outputs(); c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0)
        c.stop(json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"}), .1)
        terminal.append(c)
        out = Outputs(); c = SearchController(out); c.update_yaw(0, 0); c.start(start_json(), 0); c.tick(40)
        terminal.append(c)
        for c in terminal:
            state = c.state
            self.assertFalse(c.handle_scanner_event(event("resolved", order=1, url="https://1",
                                                           detected_yaw=1., item_name="late", message=""), 41))
            self.assertEqual(state, c.state)

    def test_start_enabled_control_failure_degrades_to_safe_error(self):
        out, errors = Outputs(), Mock()
        original = out.publish_scanner_control
        def control(value):
            if value["enabled"]: raise RuntimeError("control")
            original(value)
        out.publish_scanner_control = control
        c = SearchController(out, error_handler=errors); c.update_yaw(0, 0); c.start(start_json(), 0)
        self.assertEqual("ERROR", c.state)
        self.assertEqual(0., out.speeds[-1])
        self.assertFalse(out.controls[-1]["enabled"])
        errors.assert_called()

    def test_target_mode_control_failure_degrades_to_safe_error(self):
        out, errors = Outputs(), Mock()
        original = out.publish_scanner_control
        def control(value):
            if value["enabled"] and value["enhanced"]: raise RuntimeError("target control")
            original(value)
        out.publish_scanner_control = control
        c = SearchController(out, fast_sweep_angle=.5, error_handler=errors)
        c.update_yaw(0, 0); c.start(start_json(), 0); c.update_yaw(.6, .1)
        c.handle_scanner_event(event("quality", brightness=1., overexposed=0., sharpness=50.,
                                     decoded=False, detected_yaw=.6), .11)
        c.tick(.12)
        self.assertEqual("ERROR", c.state)
        self.assertEqual(0., out.speeds[-1])
        self.assertFalse(out.controls[-1]["enabled"])

    def test_blocked_old_enabled_control_then_stop_finishes_consistently(self):
        entered, release = threading.Event(), threading.Event()
        out = Outputs()
        original = out.publish_scanner_control
        def control(value):
            if value["enabled"]:
                entered.set(); release.wait(1)
            original(value)
        out.publish_scanner_control = control
        c = SearchController(out); c.update_yaw(0, 0)
        starter = threading.Thread(target=c.start, args=(start_json(), 0)); starter.start()
        self.assertTrue(entered.wait(1))
        raw = json.dumps({"protocol_version": 1, "task_id": "task", "search_id": "search", "reason": "stop"})
        stopper = threading.Thread(target=c.stop, args=(raw, .1)); stopper.start()
        release.set(); starter.join(1); stopper.join(1)
        self.assertEqual("STOPPED", out.states[-1])
        self.assertFalse(out.controls[-1]["enabled"])


if __name__ == "__main__":
    unittest.main()
