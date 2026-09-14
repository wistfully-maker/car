import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_package_files_exist(self):
        for relative in (
            "setup.py",
            "src/llm_spark/__init__.py",
            "src/llm_spark/protocol.py",
            "scripts/spark_llm_node.py",
            "launch/llm_spark.launch",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_node_has_no_embedded_api_password(self):
        source = (ROOT / "scripts/spark_llm_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("SPARK_API_PASSWORD", source)
        self.assertNotIn('~api_password",\n            "', source)


if __name__ == "__main__":
    unittest.main()
