# -*- coding: utf-8 -*-
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from llm_spark.protocol import (  # noqa: E402
    ProtocolError,
    build_error_result,
    build_prompt,
    build_success_result,
    parse_request,
)


VALID_REQUEST = {
    "protocol_version": 1,
    "task_id": "task-001",
    "request_id": "request-001",
    "physical_target_category": "食品",
    "simulation_target_category": "日用品",
    "candidates": [
        {"order": 1, "item_name": "手机"},
        {"order": 2, "item_name": "毛巾"},
        {"order": 3, "item_name": "苹果"},
    ],
}

VALID_MODEL_RESULT = {
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
}


class ProtocolTests(unittest.TestCase):
    def test_parses_protocol_v1_dual_target_request(self):
        request = parse_request(json.dumps(VALID_REQUEST, ensure_ascii=False))
        self.assertEqual("task-001", request["task_id"])
        self.assertEqual(3, len(request["candidates"]))

    def test_rejects_invalid_categories_and_candidate_orders(self):
        invalid = dict(VALID_REQUEST)
        invalid["physical_target_category"] = "玩具"
        with self.assertRaises(ProtocolError):
            parse_request(json.dumps(invalid, ensure_ascii=False))

        invalid = dict(VALID_REQUEST)
        invalid["candidates"] = [
            {"order": 2, "item_name": "毛巾"},
            {"order": 3, "item_name": "苹果"},
        ]
        with self.assertRaises(ProtocolError):
            parse_request(json.dumps(invalid, ensure_ascii=False))

    def test_prompt_contains_both_targets_and_candidates(self):
        request = parse_request(json.dumps(VALID_REQUEST, ensure_ascii=False))
        prompt = build_prompt(request)
        self.assertIn("实物目标母类：食品", prompt)
        self.assertIn("仿真目标母类：日用品", prompt)
        self.assertIn("1. 手机", prompt)
        self.assertIn('"physical"', prompt)
        self.assertIn('"simulation"', prompt)

    def test_builds_identity_preserving_success_result(self):
        request = parse_request(json.dumps(VALID_REQUEST, ensure_ascii=False))
        result = build_success_result(request, VALID_MODEL_RESULT)
        self.assertEqual(1, result["protocol_version"])
        self.assertEqual("task-001", result["task_id"])
        self.assertEqual("request-001", result["request_id"])
        self.assertEqual("success", result["status"])
        self.assertEqual("苹果", result["physical"]["selected_item"])
        self.assertEqual("毛巾", result["simulation"]["selected_item"])

    def test_rejects_selection_not_matching_candidate_or_target(self):
        request = parse_request(json.dumps(VALID_REQUEST, ensure_ascii=False))
        invalid = json.loads(json.dumps(VALID_MODEL_RESULT, ensure_ascii=False))
        invalid["physical"]["selected_item"] = "香蕉"
        with self.assertRaises(ProtocolError):
            build_success_result(request, invalid)

        invalid = json.loads(json.dumps(VALID_MODEL_RESULT, ensure_ascii=False))
        invalid["simulation"]["workshop"] = "电子产品生产车间"
        with self.assertRaises(ProtocolError):
            build_success_result(request, invalid)

    def test_error_result_keeps_task_and_request_identity(self):
        result = build_error_result(VALID_REQUEST, "network timeout")
        self.assertEqual({
            "protocol_version": 1,
            "task_id": "task-001",
            "request_id": "request-001",
            "status": "error",
            "message": "network timeout",
        }, result)


if __name__ == "__main__":
    unittest.main()
