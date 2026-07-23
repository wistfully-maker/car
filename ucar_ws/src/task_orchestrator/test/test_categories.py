import unittest
import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.categories import (
    category_config,
    format_result_speech,
)


class CategoryTests(unittest.TestCase):
    def test_all_competition_categories_have_labels_and_workshops(self):
        self.assertEqual("食品加工车间", category_config("食品")["workshop"])
        self.assertEqual("日用品大类", category_config("日用品")["label"])
        self.assertEqual("电子产品生产车间", category_config("电子产品")["workshop"])

    def test_unknown_category_is_rejected(self):
        with self.assertRaises(ValueError):
            category_config("玩具")

    def test_speech_uses_exact_competition_template(self):
        text = format_result_speech(
            physical_item="苹果",
            physical_category="食品",
            simulation_item="毛巾",
            simulation_category="日用品",
        )
        self.assertEqual(
            "取得苹果属于食品大类应放置在食品加工车间，"
            "仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间",
            text,
        )


if __name__ == "__main__":
    unittest.main()
