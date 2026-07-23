import unittest
import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.voice_parser import parse_categories


class VoiceParserTests(unittest.TestCase):
    def test_extracts_two_categories_in_order(self):
        result = parse_categories(
            "小飞小飞，前往物品领取区，取得食品，放置在对应仓库，"
            "并领取仿真环境中需要的日用品放置在对应仓库"
        )
        self.assertEqual(("食品", "日用品"), result)

    def test_supports_electronic_products(self):
        result = parse_categories(
            "取得电子产品，并领取仿真环境中需要的食品"
        )
        self.assertEqual(("电子产品", "食品"), result)

    def test_rejects_one_category_only(self):
        with self.assertRaises(ValueError):
            parse_categories("前往物品领取区，取得食品")

    def test_rejects_ambiguous_extra_category(self):
        with self.assertRaises(ValueError):
            parse_categories("取得食品和日用品，仿真环境需要电子产品")


if __name__ == "__main__":
    unittest.main()
