import json
import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.voice_adapter import VoiceTaskAdapterLogic
from task_orchestrator.protocol import parse_task_request


class VoiceTaskAdapterLogicTests(unittest.TestCase):
    def setUp(self):
        self.now = [100.0]
        self.ids = iter(("task-1", "task-2", "task-3"))
        self.logic = VoiceTaskAdapterLogic(
            clock=lambda: self.now[0],
            id_factory=lambda: next(self.ids),
            debounce_seconds=2.0,
        )

    def test_builds_protocol_v1_dual_category_request(self):
        result = self.logic.build(
            " 取得食品，并领取仿真环境中需要的日用品 "
        )
        self.assertEqual(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "physical_target_category": "食品",
                "simulation_target_category": "日用品",
                "raw_text": "取得食品，并领取仿真环境中需要的日用品",
            },
            result,
        )
        self.assertEqual(result, parse_task_request(json.dumps(result)))

    def test_rejects_empty_or_ambiguous_text(self):
        for text in ("", "   ", "只取得食品", "食品日用品电子产品"):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    self.logic.build(text)

    def test_debounces_only_identical_successful_text(self):
        text = "取得食品，并领取仿真环境中的日用品"
        self.assertIsNotNone(self.logic.build(text))
        self.now[0] += 1.0
        self.assertIsNone(self.logic.build(text))

        different = "取得电子产品，并领取仿真环境中的食品"
        self.assertEqual("task-2", self.logic.build(different)["task_id"])

        self.now[0] += 3.0
        self.assertEqual("task-3", self.logic.build(text)["task_id"])


if __name__ == "__main__":
    unittest.main()
