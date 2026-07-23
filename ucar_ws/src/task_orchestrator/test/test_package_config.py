import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml",
            "CMakeLists.txt",
            "setup.py",
            "src/task_orchestrator/__init__.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)


if __name__ == "__main__":
    unittest.main()