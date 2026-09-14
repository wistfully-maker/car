import json
import math
import unittest

from qr_item_search.controller_logic import SearchController


class Outputs:
    def __init__(self):
        self.speeds, self.states, self.controls, self.results = [], [], [], []
    def publish_speed(self, value): self.speeds.append(value)
    def publish_state(self, value): self.states.append(value)
    def publish_scanner_control(self, value): self.controls.append(value)
    def publish_result(self, value): self.results.append(value)


def start_json(task="task", search="search"):
    return json.dumps({"protocol_version": 1, "task_id": task,
                       "search_id": search, "expected_count": 3})


def event(kind, **fields):
    value = {"protocol_version": 1, "task_id": "task", "search_id": "search",
             "event": kind, "scanner_session": "session-a"}
    value.update(fields)
    return json.dumps(value)


class ContinuousScanningTest(unittest.TestCase):
    """The scanner stays enabled for the whole search, not per station."""

    def setUp(self):
        self.out = Outputs()
        self.c = SearchController(self.out, camera_timeout=100.0)
        self.c.update_yaw(0.0, 0.0)

    def start_search(self, now=0.0):
        self.assertTrue(self.c.start(start_json(), now))

    def detected(self, order, now, url=None, yaw=0.0):
        return self.c.handle_scanner_event(event(
            "detected", order=order, url=url or "https://%d" % order,
            detected_yaw=yaw, item_name="", message=""), now)

    def resolved(self, order, now, name=None):
        return self.c.handle_scanner_event(event(
            "resolved", order=order, url="https://%d" % order,
            detected_yaw=0.0, item_name=name or "item%d" % order, message=""), now)

    def test_scanner_enabled_throughout_search(self):
        self.start_search()
        self.c.tick(0.61)
        self.assertEqual("TURNING", self.c.state)
        self.c.update_yaw(math.radians(1.0), 0.7)
        self.c.tick(0.7)
        self.c.update_yaw(math.radians(44.0), 1.0)
        self.c.tick(1.0)
        self.assertEqual("SETTLING", self.c.state)
        self.c.update_yaw(math.radians(44.0), 1.2, angular_speed=0.0)
        self.c.update_yaw(math.radians(44.0), 1.45, angular_speed=0.0)
        self.assertEqual("SCANNING", self.c.state)
        self.c.tick(2.05 + 1e-6)
        self.assertEqual("TURNING", self.c.state)
        self.assertTrue(all(value["enabled"] for value in self.out.controls[1:]),
                        "every control during the search must stay enabled")

    def test_url_during_turning_is_accepted_and_http_proceeds(self):
        self.start_search()
        self.c.tick(0.61)
        self.c.update_yaw(math.radians(1.0), 0.7)
        self.c.tick(0.7)
        self.assertTrue(self.detected(1, 0.8))
        self.assertEqual("TURNING", self.c.state)
        self.assertEqual(1, len(self.c._detected))
        self.resolved(1, 0.9)
        self.assertEqual("TURNING", self.c.state)
        self.assertEqual("item1", self.c._resolved[1])
        self.assertNotEqual(0.0, self.out.speeds[-1])

    def test_detected_during_dwell_does_not_end_station(self):
        self.start_search()
        self.assertTrue(self.detected(1, 0.3))
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.c.tick(0.4)
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.c.tick(0.61)
        self.assertEqual("TURNING", self.c.state)

    def test_third_url_in_turning_stops_and_waits_http(self):
        self.start_search()
        self.c.tick(0.61)
        self.c.update_yaw(math.radians(1.0), 0.7)
        self.c.tick(0.7)
        for order in (1, 2, 3):
            self.assertTrue(self.detected(order, 0.8 + order / 100))
        self.assertEqual("WAITING_HTTP", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertTrue(self.out.controls[-1]["enabled"])
        self.c.tick(1.0)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_third_url_in_settling_stops_and_waits_http(self):
        self.start_search()
        self.c.tick(0.61)
        self.c.update_yaw(math.radians(44.0), 0.7)
        self.c.tick(0.7)
        self.assertEqual("SETTLING", self.c.state)
        for order in (1, 2, 3):
            self.assertTrue(self.detected(order, 0.8 + order / 100))
        self.assertEqual("WAITING_HTTP", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_third_url_in_dwell_stops_and_waits_http(self):
        self.start_search()
        for order in (1, 2, 3):
            self.assertTrue(self.detected(order, 0.1 + order / 100))
        self.assertEqual("WAITING_HTTP", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])

    def test_resolved_completes_and_disables_scanner(self):
        self.start_search()
        for order in (1, 2, 3):
            self.detected(order, 0.1 + order / 100)
        for index, order in enumerate((3, 1, 2)):
            self.resolved(order, 0.3 + index / 100)
        self.assertEqual("COMPLETE", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_stop_disables_scanner_and_zero(self):
        self.start_search()
        raw = json.dumps({"protocol_version": 1, "task_id": "task",
                          "search_id": "search", "reason": "stop"})
        self.assertTrue(self.c.stop(raw, 0.2))
        self.assertEqual("STOPPED", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_shutdown_disables_scanner_and_zero(self):
        self.start_search()
        self.c.tick(0.61)
        self.c.shutdown()
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_not_found_disables_scanner_and_zero(self):
        self.start_search()
        self.c.tick(60.0)
        self.assertEqual("NOT_FOUND", self.c.state)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_start_control_has_station_context_without_capture_gating(self):
        self.start_search()
        control = self.out.controls[-1]
        self.assertTrue(control["enabled"])
        self.assertTrue(control["enhanced"])
        self.assertEqual(0, control["pass_index"])
        self.assertEqual(0, control["station_index"])
        self.assertNotIn("capture_id", control)
        self.assertNotIn("capture_after", control)

    def test_control_carries_fresh_station_context_at_boundaries(self):
        self.start_search()
        self.c.tick(0.61)
        self.assertEqual("TURNING", self.c.state)
        self.assertEqual(1, self.out.controls[-1]["station_index"])
        self.c.update_yaw(math.radians(44.0), 0.7)
        self.c.tick(0.7)
        self.c.update_yaw(math.radians(44.0), 0.9, angular_speed=0.0)
        self.c.update_yaw(math.radians(44.0), 1.15, angular_speed=0.0)
        self.assertEqual("SCANNING", self.c.state)
        self.assertEqual(1, self.out.controls[-1]["station_index"])


if __name__ == "__main__":
    unittest.main()
