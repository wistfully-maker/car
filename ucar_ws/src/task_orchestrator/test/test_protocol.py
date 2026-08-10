import json
import math
import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.protocol import (
    ProtocolError,
    parse_arrival,
    parse_cancel,
    parse_dependencies_ready,
    parse_llm_result,
    parse_navigation_handoff_status,
    parse_qr_result,
    parse_speak_request,
    parse_speech_done,
    parse_task_request,
)


def encode(value):
    return json.dumps(value, ensure_ascii=False)


class ProtocolTests(unittest.TestCase):
    def test_navigation_handoff_status_is_correlated_and_bounded(self):
        ready = {
            "protocol_version": 1, "task_id": "task-001",
            "goal_id": "delivery-001", "status": "ready", "message": "",
        }
        parsed = parse_navigation_handoff_status(
            encode(ready), "task-001", "delivery-001"
        )
        self.assertEqual("ready", parsed["status"])
        failed = dict(ready, status="failed", message="move_base unavailable")
        self.assertEqual("move_base unavailable", parse_navigation_handoff_status(
            encode(failed), "task-001", "delivery-001"
        )["message"])
        for invalid in (
            dict(ready, status="switching"),
            dict(ready, status="failed", message=""),
            dict(ready, goal_id="stale"),
        ):
            with self.assertRaises(ProtocolError):
                parse_navigation_handoff_status(
                    encode(invalid), "task-001", "delivery-001"
                )

    def test_speak_request_requires_identity_and_text(self):
        message = {
            "protocol_version": 1,
            "task_id": "task-001",
            "speech_id": "speech-001",
            "text": "播报内容",
        }
        self.assertEqual(
            "播报内容",
            parse_speak_request(encode(message))["text"],
        )
        for field in ("task_id", "speech_id", "text"):
            invalid = dict(message)
            invalid[field] = " "
            with self.subTest(field=field):
                with self.assertRaises(ProtocolError):
                    parse_speak_request(encode(invalid))

    def test_dependencies_ready_validates_task_identity(self):
        message = {
            "protocol_version": 1,
            "task_id": "task-001",
            "status": "ready",
        }
        self.assertEqual(
            "ready",
            parse_dependencies_ready(
                encode(message), "task-001"
            )["status"],
        )
        with self.assertRaises(ProtocolError):
            parse_dependencies_ready(encode(message), "stale-task")
        message["status"] = "starting"
        with self.assertRaises(ProtocolError):
            parse_dependencies_ready(encode(message), "task-001")

    def test_parses_valid_task_request(self):
        result = parse_task_request(
            encode(
                {
                    "protocol_version": 1,
                    "task_id": "task-001",
                    "physical_target_category": "食品",
                    "simulation_target_category": "日用品",
                    "raw_text": "取得食品，并领取仿真环境中的日用品",
                }
            )
        )
        self.assertEqual("task-001", result["task_id"])
        self.assertEqual("食品", result["physical_target_category"])

    def test_task_request_rejects_invalid_protocol_or_category(self):
        for version, category in ((True, "食品"), (2, "食品"), (1, "玩具")):
            with self.subTest(version=version, category=category):
                with self.assertRaises(ProtocolError):
                    parse_task_request(
                        encode(
                            {
                                "protocol_version": version,
                                "task_id": "task-001",
                                "physical_target_category": category,
                                "simulation_target_category": "日用品",
                                "raw_text": "test",
                            }
                        )
                    )

    def test_arrival_validates_identity_and_failure_message(self):
        arrived = {
            "protocol_version": 1,
            "task_id": "task-001",
            "goal_id": "goal-001",
            "status": "arrived",
            "message": "",
        }
        self.assertEqual(
            "arrived",
            parse_arrival(encode(arrived), "task-001", "goal-001")["status"],
        )
        with self.assertRaises(ProtocolError):
            parse_arrival(encode(arrived), "stale-task", "goal-001")
        arrived["task_id"] = " task-001 "
        with self.assertRaises(ProtocolError):
            parse_arrival(encode(arrived), "task-001", "goal-001")
        arrived["task_id"] = "task-001"
        arrived["status"] = "failed"
        with self.assertRaises(ProtocolError):
            parse_arrival(encode(arrived), "task-001", "goal-001")

    def test_parses_complete_qr_result_with_three_distinct_items(self):
        result = parse_qr_result(
            encode(self._qr_message()),
            "task-001",
            "search-001",
        )
        self.assertEqual(["手机", "毛巾", "苹果"], [
            item["item_name"] for item in result["items"]
        ])

    def test_qr_result_rejects_stale_identity_and_invalid_complete_items(self):
        message = self._qr_message()
        with self.assertRaises(ProtocolError):
            parse_qr_result(encode(message), "task-001", "stale-search")

        invalid_messages = []
        two_items = self._qr_message()
        two_items["items"] = two_items["items"][:2]
        invalid_messages.append(two_items)
        duplicate_order = self._qr_message()
        duplicate_order["items"][1]["order"] = 1
        invalid_messages.append(duplicate_order)
        duplicate_name = self._qr_message()
        duplicate_name["items"][1]["item_name"] = "手机"
        invalid_messages.append(duplicate_name)
        duplicate_url = self._qr_message()
        duplicate_url["items"][1]["url"] = "https://example/3"
        invalid_messages.append(duplicate_url)
        nonconsecutive_order = self._qr_message()
        nonconsecutive_order["items"][2]["order"] = 4
        invalid_messages.append(nonconsecutive_order)
        invalid_yaw = self._qr_message()
        invalid_yaw["items"][0]["detected_yaw"] = math.nan
        invalid_messages.append(invalid_yaw)
        invalid_stamp = self._qr_message()
        invalid_stamp["stamp"] = math.inf
        invalid_messages.append(invalid_stamp)

        for invalid in invalid_messages:
            with self.subTest(invalid=invalid):
                with self.assertRaises(ProtocolError):
                    parse_qr_result(
                        encode(invalid),
                        "task-001",
                        "search-001",
                    )

    def test_qr_searching_accepts_partial_but_not_complete_items(self):
        message = self._qr_message()
        message.update(
            status="searching",
            items=message["items"][:1],
            message="",
        )
        self.assertEqual(
            "searching",
            parse_qr_result(
                encode(message), "task-001", "search-001"
            )["status"],
        )
        message["items"] = self._qr_message()["items"]
        with self.assertRaises(ProtocolError):
            parse_qr_result(encode(message), "task-001", "search-001")

    def test_qr_failure_requires_message(self):
        message = self._qr_message()
        message.update(status="not_found", items=[], message="")
        with self.assertRaises(ProtocolError):
            parse_qr_result(encode(message), "task-001", "search-001")
        message["message"] = "items not found"
        self.assertEqual(
            "not_found",
            parse_qr_result(
                encode(message), "task-001", "search-001"
            )["status"],
        )

    def test_parses_and_validates_successful_llm_result(self):
        result = parse_llm_result(
            encode(self._llm_message()),
            self._llm_context(),
        )
        self.assertEqual("苹果", result["physical"]["selected_item"])
        self.assertEqual("毛巾", result["simulation"]["selected_item"])

    def test_llm_result_rejects_untrusted_selection(self):
        mutations = (
            ("physical", "selected_order", 99),
            ("physical", "selected_item", "香蕉"),
            ("physical", "category", "日用品"),
            ("physical", "workshop", "错误车间"),
            ("simulation", "category", "食品"),
        )
        for section, field, value in mutations:
            message = self._llm_message()
            message[section][field] = value
            with self.subTest(section=section, field=field):
                with self.assertRaises(ProtocolError):
                    parse_llm_result(
                        encode(message),
                        self._llm_context(),
                    )

    def test_llm_error_requires_message(self):
        message = self._llm_message()
        message.update(status="error", message="")
        message.pop("physical")
        message.pop("simulation")
        with self.assertRaises(ProtocolError):
            parse_llm_result(encode(message), self._llm_context())
        message["message"] = "classification failed"
        self.assertEqual(
            "error",
            parse_llm_result(encode(message), self._llm_context())["status"],
        )

    def test_speech_done_validates_identity_status_and_message(self):
        message = {
            "protocol_version": 1,
            "task_id": "task-001",
            "speech_id": "speech-001",
            "status": "success",
            "message": "",
        }
        self.assertEqual(
            "success",
            parse_speech_done(
                encode(message), "task-001", "speech-001"
            )["status"],
        )
        with self.assertRaises(ProtocolError):
            parse_speech_done(
                encode(message), "task-001", "stale-speech"
            )
        message.update(status="error", message="")
        with self.assertRaises(ProtocolError):
            parse_speech_done(
                encode(message), "task-001", "speech-001"
            )

    def test_cancel_validates_task_identity_and_reason(self):
        message = {
            "protocol_version": 1,
            "task_id": "task-001",
            "reason": "operator_cancel",
        }
        self.assertEqual(
            "operator_cancel",
            parse_cancel(encode(message), "task-001")["reason"],
        )
        with self.assertRaises(ProtocolError):
            parse_cancel(encode(message), "stale-task")
        message["reason"] = " "
        with self.assertRaises(ProtocolError):
            parse_cancel(encode(message), "task-001")
        message["reason"] = " operator_cancel "
        with self.assertRaises(ProtocolError):
            parse_cancel(encode(message), "task-001")

    @staticmethod
    def _qr_message():
        return {
            "protocol_version": 1,
            "task_id": "task-001",
            "search_id": "search-001",
            "stamp": 123.5,
            "status": "complete",
            "items": [
                {
                    "order": 1,
                    "item_name": "手机",
                    "url": "https://example/3",
                    "detected_yaw": 0.0,
                },
                {
                    "order": 2,
                    "item_name": "毛巾",
                    "url": "https://example/2",
                    "detected_yaw": 1.27,
                },
                {
                    "order": 3,
                    "item_name": "苹果",
                    "url": "https://example/1",
                    "detected_yaw": 3.39,
                },
            ],
            "message": "",
        }

    @staticmethod
    def _llm_context():
        return {
            "task_id": "task-001",
            "request_id": "llm-001",
            "physical_target_category": "食品",
            "simulation_target_category": "日用品",
            "candidates": [
                {"order": 1, "item_name": "手机"},
                {"order": 2, "item_name": "毛巾"},
                {"order": 3, "item_name": "苹果"},
            ],
        }

    @staticmethod
    def _llm_message():
        return {
            "protocol_version": 1,
            "task_id": "task-001",
            "request_id": "llm-001",
            "status": "success",
            "physical": {
                "selected_order": 3,
                "selected_item": "苹果",
                "category": "食品",
                "workshop": "食品加工车间",
            },
            "simulation": {
                "selected_order": 2,
                "selected_item": "毛巾",
                "category": "日用品",
                "workshop": "日用品加工车间",
            },
            "message": "",
        }


if __name__ == "__main__":
    unittest.main()
