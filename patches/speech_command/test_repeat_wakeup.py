import re
import unittest
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parent / "AIUITester.cpp"
).read_text(encoding="utf-8")


def function_body(signature):
    match = re.search(
        re.escape(signature) + r"\s*\(\s*\)\s*\{(?P<body>.*?)\n\}",
        SOURCE,
        re.DOTALL,
    )
    if not match:
        raise AssertionError("function not found: " + signature)
    return match.group("body")


class RepeatWakeupTests(unittest.TestCase):
    def test_stop_records_that_aiui_requires_restart(self):
        self.assertIn("std::atomic<bool> aiui_stopped", SOURCE)
        self.assertIn("aiui_stopped.store(true)", function_body("void gStop"))

    def test_wakeup_restarts_stopped_service_before_wakeup(self):
        body = function_body("void gWakeup")
        self.assertIn("aiui_stopped.exchange(false)", body)
        self.assertLess(
            body.index("AIUIConstant::CMD_START"),
            body.index("AIUIConstant::CMD_WAKEUP"),
        )

    def test_wakeup_does_not_restart_an_already_running_service(self):
        body = function_body("void gWakeup")
        self.assertRegex(
            body,
            r"if\s*\(\s*aiui_stopped\.exchange\(false\)\s*\)",
        )

    def test_uart_partial_frame_buffer_survives_across_callbacks(self):
        uart_source = SOURCE[SOURCE.index("void uart_rec("):]
        self.assertIn(
            "static unsigned char recv_buf[RECV_BUF_LEN]",
            uart_source,
        )

    def test_uart_stream_resynchronizes_to_header_inside_chunk(self):
        uart_source = SOURCE[SOURCE.index("void uart_rec("):]
        self.assertIn("find_sync_head", uart_source)
        self.assertRegex(
            uart_source,
            r"uart_rec\s*\(\s*msg\s*\+\s*sync_offset",
        )

    def test_aiui_callback_uses_competition_command_gate(self):
        self.assertIn('#include "competition_command_gate.h"', SOURCE)
        self.assertIn("competition_gate.accept(fullText)", SOURCE)
        self.assertIn("CommandKind::INCOMPLETE_TASK", SOURCE)
        self.assertIn("CommandKind::COMPLETE_TASK", SOURCE)
        self.assertIn("CommandKind::DUPLICATE_TASK", SOURCE)
        self.assertIn("CommandKind::NOISE", SOURCE)

    def test_new_stopped_session_resets_competition_gate(self):
        wake_body = function_body("void gWakeup")
        stopped_branch = re.search(
            r"if\s*\(\s*aiui_stopped\.exchange\(false\)\s*\)"
            r"\s*\{(?P<body>.*?)\n\s*\}",
            wake_body,
            re.DOTALL,
        )
        self.assertIsNotNone(stopped_branch)
        self.assertIn(
            "competition_gate.reset()",
            stopped_branch.group("body"),
        )

    def test_complete_competition_task_bypasses_original_qa(self):
        branch = re.search(
            r"if\s*\(\s*decision\.kind\s*==\s*"
            r"CommandKind::COMPLETE_TASK\s*\)"
            r"\s*\{(?P<body>.*?)\n\s*\}",
            SOURCE,
            re.DOTALL,
        )
        self.assertIsNotNone(branch)
        body = branch.group("body")
        self.assertIn("sign_conversation_cloud = 1", body)
        self.assertIn("gStop()", body)
        self.assertNotIn("FindDocument", body)
        self.assertNotIn("_serial.write", body)

    def test_cloud_iat_path_contains_no_original_qa_or_echo(self):
        iat_path = SOURCE[
            SOURCE.index('if (sub == "iat"'):
            SOURCE.index('else \t\tif (sub == "asr"')
        ]
        self.assertNotIn("FindDocument", iat_path)
        self.assertNotIn("QA_list_", iat_path)
        self.assertNotIn("_serial.write", iat_path)
        self.assertNotIn("gTTS(", iat_path)


if __name__ == "__main__":
    unittest.main()
