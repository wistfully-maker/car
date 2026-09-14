import json
import os
import tempfile
import unittest
from unittest import mock
from unittest.mock import Mock

from qr_item_search.run_metrics import METRICS_FILENAME, append_run_record


class AppendRunRecordTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.mkdtemp(prefix="qr_metrics_")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.temp, ignore_errors=True)

    def path(self):
        return os.path.join(self.temp, METRICS_FILENAME)

    def read_lines(self):
        with open(self.path(), encoding="utf-8") as handle:
            return [line for line in handle.read().splitlines() if line]

    def test_creates_dir_and_appends_one_json_line(self):
        nested = os.path.join(self.temp, "a", "b")
        append_run_record(nested, {"status": "complete", "total_seconds": 1.5})
        with open(os.path.join(nested, METRICS_FILENAME), encoding="utf-8") as handle:
            lines = [line for line in handle.read().splitlines() if line]
        self.assertEqual(1, len(lines))
        self.assertEqual({"status": "complete", "total_seconds": 1.5}, json.loads(lines[0]))

    def test_keeps_chinese_text_unescaped(self):
        append_run_record(self.temp, {"item_name": "香蕉"})
        line = self.read_lines()[0]
        self.assertIn("香蕉", line)

    def test_appends_multiple_records_without_overwriting(self):
        append_run_record(self.temp, {"run": 1})
        append_run_record(self.temp, {"run": 2})
        lines = self.read_lines()
        self.assertEqual(2, len(lines))
        self.assertEqual([{"run": 1}, {"run": 2}], [json.loads(line) for line in lines])

    def test_failure_only_warns(self):
        warning = Mock()
        with mock.patch("qr_item_search.run_metrics.os.makedirs",
                        side_effect=OSError("denied")):
            append_run_record(self.temp, {"run": 1}, warning=warning)
        warning.assert_called()
        self.assertFalse(os.path.exists(self.path()))

    def test_empty_dir_only_warns(self):
        warning = Mock()
        append_run_record("", {"run": 1}, warning=warning)
        warning.assert_called()


if __name__ == "__main__":
    unittest.main()
