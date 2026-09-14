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


class CameraHeartbeatTest(unittest.TestCase):
    """Camera liveness is proven by raw frame heartbeats, not by decoding."""

    def setUp(self):
        self.out = Outputs()
        self.c = SearchController(self.out, camera_timeout=1.0)
        self.c.update_yaw(0.0, 0.0)
        self.assertTrue(self.c.start(start_json(), 0.0))

    def frame_seen(self, now):
        return self.c.handle_scanner_event(event("frame_seen"), now)

    def quality(self, now, decoded=False):
        return self.c.handle_scanner_event(event(
            "quality", brightness=50., overexposed=0., sharpness=50.,
            decoded=decoded, detected_yaw=0.), now)

    def test_initial_scan_heartbeat_prevents_false_timeout(self):
        self.frame_seen(0.1)
        self.c.update_yaw(0.0, 0.5)
        self.c.tick(1.1)
        self.assertNotEqual("ERROR", self.c.state)
        self.assertEqual([], [r["message"] for r in self.out.results
                              if r["status"] == "error"])

    def test_turning_longer_than_camera_timeout_no_false_alarm(self):
        self.c.tick(0.61)
        for t in (0.8, 1.3, 1.8, 2.3, 2.8):
            self.c.update_yaw(math.radians(5.0), t)
            self.frame_seen(t)
            self.c.tick(t)
        self.assertEqual("TURNING", self.c.state)
        self.assertEqual([], [r["message"] for r in self.out.results
                              if r["status"] == "error"])

    def test_next_station_entry_does_not_immediately_timeout(self):
        self.c.tick(0.61)
        for t in (0.8, 1.3, 1.8, 2.3):
            self.c.update_yaw(math.radians(5.0), t)
            self.frame_seen(t)
            self.c.tick(t)
        self.c.update_yaw(math.radians(44.0), 2.7)
        self.c.tick(2.7)
        self.assertEqual("SETTLING", self.c.state)
        self.c.update_yaw(math.radians(44.0), 2.9, angular_speed=0.0)
        self.c.update_yaw(math.radians(44.0), 3.15, angular_speed=0.0)
        self.assertEqual("SCANNING", self.c.state)
        self.c.tick(3.155)
        self.assertEqual("SCANNING", self.c.state)
        self.assertEqual([], [r["message"] for r in self.out.results
                              if r["status"] == "error"])

    def test_real_heartbeat_stall_enters_error_and_zero(self):
        self.frame_seen(0.1)
        self.c.update_yaw(0.0, 0.5)
        self.c.tick(0.6)
        self.assertEqual(0.0, self.out.speeds[-1])
        self.c.tick(1.2)
        self.assertEqual("ERROR", self.c.state)
        self.assertEqual("camera timed out", self.out.results[-1]["message"])
        self.assertEqual(0.0, self.out.speeds[-1])
        self.assertFalse(self.out.controls[-1]["enabled"])

    def test_decoder_block_does_not_cause_camera_timeout(self):
        self.quality(0.1)
        self.c.tick(0.61)
        for t in (0.8, 1.3, 1.8, 2.3):
            self.c.update_yaw(math.radians(5.0), t)
            self.frame_seen(t)
            self.c.tick(t)
        self.c.update_yaw(math.radians(44.0), 2.7)
        self.c.tick(2.7)
        self.c.update_yaw(math.radians(44.0), 2.9, angular_speed=0.0)
        self.c.update_yaw(math.radians(44.0), 3.15, angular_speed=0.0)
        self.c.tick(3.2)
        self.assertEqual("SCANNING", self.c.state)
        self.assertEqual([], [r["message"] for r in self.out.results
                              if r["status"] == "error"])

    def test_stale_search_heartbeat_is_ignored(self):
        raw = json.loads(event("frame_seen"))
        raw["search_id"] = "old-search"
        self.assertFalse(self.c.handle_scanner_event(json.dumps(raw), 0.1))
        self.c.tick(0.3)
        self.assertEqual("INITIAL_SCAN", self.c.state)
        self.assertIsNone(self.c._last_frame_seen)


if __name__ == "__main__":
    unittest.main()
