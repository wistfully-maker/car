"""Regression tests for Python modules required by installed ROS scripts."""

import ast
import unittest
from pathlib import Path


STOP_ROOT = Path(__file__).resolve().parents[1]
SETUP_TREE = ast.parse((STOP_ROOT / "setup.py").read_text(encoding="utf-8"))


def setup_keyword(name):
    for node in ast.walk(SETUP_TREE):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name) and func.id == "generate_distutils_setup":
                for keyword in node.keywords:
                    if keyword.arg == name:
                        return ast.literal_eval(keyword.value)
    raise AssertionError("generate_distutils_setup has no %s keyword" % name)


class PythonPackagingTests(unittest.TestCase):
    def test_catkin_devel_space_exposes_native_ocr_modules(self):
        packages = setup_keyword("packages")
        package_dir = setup_keyword("package_dir")

        self.assertIn("ocr", packages)
        self.assertIn("ocr.utils", packages)
        self.assertEqual("scripts/ocr", package_dir["ocr"])
        self.assertEqual("scripts/ocr/utils", package_dir["ocr.utils"])


if __name__ == "__main__":
    unittest.main()
