import json
import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.gazebo_tcp_bridge import (
    BridgeProtocolError,
    GazeboBridgeSession,
    JsonLineDecoder,
    encode_line,
)


START = {
    "protocol_version": 1,
    "task_id": "task-1",
    "goal_id": "gazebo-1",
    "selected_item": "手机",
    "target_category": "电子产品",
    "target_workshop": "电子产品生产车间",
}


class JsonLineCodecTests(unittest.TestCase):
    def test_fragmented_utf8_and_multiple_frames_round_trip(self):
        decoder = JsonLineDecoder(max_frame_bytes=8192)
        first = encode_line(START)
        second = encode_line(dict(START, goal_id="gazebo-2"))
        split = len(first) // 2
        self.assertEqual([], decoder.feed(first[:split]))
        frames = decoder.feed(first[split:] + second)
        self.assertEqual([START, dict(START, goal_id="gazebo-2")], frames)

    def test_rejects_non_object_invalid_json_and_oversize_frame(self):
        for payload in (b"[]\n", b"not-json\n"):
            with self.subTest(payload=payload):
                with self.assertRaises(BridgeProtocolError):
                    JsonLineDecoder(32).feed(payload)
        with self.assertRaises(BridgeProtocolError):
            JsonLineDecoder(8).feed(b"123456789")


class GazeboBridgeSessionTests(unittest.TestCase):
    def test_connected_start_is_sent_and_matching_success_completes_once(self):
        session = GazeboBridgeSession(allowed_client_ip="10.0.0.8")
        self.assertTrue(session.connect("10.0.0.8"))
        self.assertEqual(START, session.start(dict(START))["send"])
        self.assertIsNone(session.start(dict(START)))
        complete = session.receive({
            "protocol_version": 1,
            "task_id": "task-1",
            "goal_id": "gazebo-1",
            "status": "success",
        })
        self.assertEqual("success", complete["complete"]["status"])
        self.assertIsNone(session.receive(complete["complete"]))

    def test_disconnected_start_and_active_disconnect_fail_softly(self):
        session = GazeboBridgeSession()
        failure = session.start(dict(START))["complete"]
        self.assertEqual("failure", failure["status"])
        self.assertEqual("desktop_not_connected", failure["reason"])

        self.assertTrue(session.connect("10.0.0.9"))
        session.start(dict(START, goal_id="gazebo-2"))
        failure = session.disconnect()["complete"]
        self.assertEqual("connection_lost", failure["reason"])
        self.assertIsNone(session.disconnect())

    def test_rejects_unapproved_peer_and_ignores_stale_result(self):
        session = GazeboBridgeSession(allowed_client_ip="10.0.0.8")
        self.assertFalse(session.connect("10.0.0.7"))
        self.assertTrue(session.connect("10.0.0.8"))
        session.start(dict(START))
        self.assertIsNone(session.receive({
            "protocol_version": 1,
            "task_id": "stale",
            "goal_id": "gazebo-1",
            "status": "success",
        }))

    def test_validates_start_fields_and_failure_reason(self):
        session = GazeboBridgeSession()
        session.connect("127.0.0.1")
        for invalid in (
            dict(START, protocol_version=1.0),
            dict(START, target_category="unknown"),
            dict(START, target_workshop="wrong"),
            dict(START, selected_item=""),
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises(BridgeProtocolError):
                    session.start(invalid)
        session.start(dict(START))
        with self.assertRaises(BridgeProtocolError):
            session.receive({
                "protocol_version": 1.0,
                "task_id": "task-1",
                "goal_id": "gazebo-1",
                "status": "success",
            })
        with self.assertRaises(BridgeProtocolError):
            session.receive({
                "protocol_version": 1,
                "task_id": "task-1",
                "goal_id": "gazebo-1",
                "status": "failure",
                "reason": "",
            })


if __name__ == "__main__":
    unittest.main()
