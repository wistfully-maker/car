#!/usr/bin/env python3

from pathlib import Path
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class ReadmeTests(unittest.TestCase):
    def test_readme_documents_required_operator_actions(self):
        readme = (PACKAGE_DIR / "README.md").read_text(encoding="utf-8")
        for required_text in (
            "move_base/cmd_vel_raw",
            "roslaunch ucar_corner_supervisor corner_supervisor.launch",
            "rosnode kill /ucar_corner_supervisor",
            "rostopic info /cmd_vel",
            "rostopic echo /ucar_corner_supervisor/state",
            "不得同时运行",
            "参数",
            "回滚",
        ):
            self.assertIn(required_text, readme)


if __name__ == "__main__":
    unittest.main()
