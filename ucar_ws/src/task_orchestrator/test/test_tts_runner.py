import subprocess
import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.tts_runner import TtsBridgeLogic, run_tts


class Result:
    def __init__(self, returncode=0, stderr=b""):
        self.returncode = returncode
        self.stderr = stderr


class TtsRunnerTests(unittest.TestCase):
    def test_passes_exact_argument_list_without_shell(self):
        calls = []

        def runner(command, timeout, **kwargs):
            calls.append((command, timeout, kwargs))
            return Result()

        result = run_tts(
            "取得苹果属于食品大类",
            ["python3", "/tmp/tts_http.py"],
            12.5,
            runner=runner,
        )
        self.assertEqual({"status": "success", "message": ""}, result)
        self.assertEqual(
            ["python3", "/tmp/tts_http.py", "取得苹果属于食品大类"],
            calls[0][0],
        )
        self.assertEqual(12.5, calls[0][1])
        self.assertNotIn("shell", calls[0][2])

    def test_nonzero_exit_and_timeout_are_errors(self):
        def failed_runner(command, timeout, **kwargs):
            return Result(3, "合成失败".encode("utf-8"))

        failed = run_tts("文本", ["tts"], 5, runner=failed_runner)
        self.assertEqual("error", failed["status"])
        self.assertIn("exit code 3", failed["message"])
        self.assertIn("合成失败", failed["message"])

        def timeout_runner(command, timeout, **kwargs):
            raise subprocess.TimeoutExpired(command, timeout)

        timed_out = run_tts("文本", ["tts"], 5, runner=timeout_runner)
        self.assertEqual("error", timed_out["status"])
        self.assertIn("timeout", timed_out["message"])

    def test_rejects_invalid_inputs_without_starting_runner(self):
        calls = []

        def runner(*args, **kwargs):
            calls.append(args)
            return Result()

        for text, command, timeout in (
            ("", ["tts"], 1),
            ("text", [], 1),
            ("text", "tts", 1),
            ("text", ["tts"], 0),
        ):
            with self.subTest(text=text, command=command, timeout=timeout):
                result = run_tts(text, command, timeout, runner=runner)
                self.assertEqual("error", result["status"])
        self.assertEqual([], calls)


class TtsBridgeLogicTests(unittest.TestCase):
    def test_builds_protocol_done_message(self):
        calls = []

        def runner(text, command, timeout):
            calls.append((text, command, timeout))
            return {"status": "success", "message": ""}

        logic = TtsBridgeLogic(["tts"], 30.0, runner=runner)
        result = logic.handle(
            {
                "task_id": "task-1",
                "speech_id": "speech-1",
                "text": "播报内容",
            }
        )
        self.assertEqual(
            {
                "protocol_version": 1,
                "task_id": "task-1",
                "speech_id": "speech-1",
                "status": "success",
                "message": "",
            },
            result,
        )
        self.assertEqual([("播报内容", ["tts"], 30.0)], calls)

    def test_duplicate_republishes_cached_result_without_replaying(self):
        calls = []

        def runner(text, command, timeout):
            calls.append(text)
            return {"status": "error", "message": "speaker unavailable"}

        logic = TtsBridgeLogic(["tts"], 30.0, runner=runner)
        request = {
            "task_id": "task-1",
            "speech_id": "speech-1",
            "text": "播报内容",
        }
        first = logic.handle(request)
        second = logic.handle(dict(request, text="不应播放的新文本"))
        self.assertEqual(first, second)
        self.assertEqual(["播报内容"], calls)


if __name__ == "__main__":
    unittest.main()
