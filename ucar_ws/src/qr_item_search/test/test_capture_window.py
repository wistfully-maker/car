import math
import os
import tempfile
import threading
import time
import unittest
from unittest import mock
from unittest.mock import Mock

import cv2
import numpy as np

from qr_item_search.keyframes import KeyframeSaver, sanitize_filename
from qr_item_search.qr_decode import UniqueQrDecoder
from qr_item_search.scanner_logic import ScannerLogic


def make_frame(width=64, height=48, gray=128):
    return np.full((height, width, 3), gray, dtype=np.uint8)


class ContinuousScanWindowTest(unittest.TestCase):
    """With search-wide scanning, capture fields are parsed for protocol
    compatibility but never reject a frame from the same search."""

    def setUp(self):
        self.decoder = Mock()
        self.resolver = Mock()
        self.publisher = Mock()
        self.quality = Mock(return_value={"brightness": 1.0, "overexposed": 0.0, "sharpness": 2.0})
        self.variants = Mock(side_effect=lambda frame, enhanced: [frame])
        self.keyframes = Mock()
        self.failed = Mock()
        self.logic = ScannerLogic(self.decoder, self.resolver, self.publisher,
                                  self.quality, self.variants, worker_count=1,
                                  keyframe_sink=self.keyframes,
                                  failed_station_sink=self.failed)
        self.logic.reset_search("task", "search")

    def process(self, frame, stamp):
        self.assertTrue(self.logic.submit_frame(frame, stamp=stamp))
        return self.logic.process_latest_frame()

    def test_capture_fields_do_not_reject_same_search_frames(self):
        self.decoder.process.return_value = ["https://a.test"]
        self.logic.set_control(True, True, 0.0, capture_id=2, capture_after=10.0,
                               pass_index=0, station_index=1)
        self.logic.submit_frame("pre-window-stamp", stamp=9.0)
        self.assertTrue(self.logic.process_latest_frame())
        self.logic.set_control(True, True, 0.0, capture_id=3, capture_after=12.0,
                               pass_index=0, station_index=2)
        self.logic.submit_frame("next-station", stamp=11.0)
        self.assertTrue(self.logic.process_latest_frame())
        self.assertEqual(2, self.decoder.process.call_count)
        self.assertEqual(1, self.logic.jobs.qsize())

    def test_frames_without_stamp_are_accepted(self):
        self.decoder.process.return_value = ["https://a.test"]
        self.logic.set_control(True, True, 0.0, capture_id=1, capture_after=10.0)
        self.assertTrue(self.logic.submit_frame("no-stamp"))
        self.assertTrue(self.logic.process_latest_frame())
        self.quality.assert_called_once_with("no-stamp")

    def test_no_gating_when_capture_fields_absent(self):
        self.decoder.process.return_value = ["https://a.test"]
        self.logic.set_control(True, False, 0.0)
        self.assertTrue(self.process("frame", 1.0))
        self.quality.assert_called_once_with("frame")

    def test_new_url_invokes_keyframe_sink_with_station_context(self):
        self.decoder.process.return_value = ["https://a.test", "https://b.test"]
        self.logic.set_control(True, True, 1.5, pass_index=0, station_index=3)
        self.assertTrue(self.process("frame", 10.1))
        args = self.keyframes.call_args[0]
        self.assertEqual("frame", args[0])
        self.assertEqual(["https://a.test", "https://b.test"], args[1])
        self.assertEqual("task", args[4])
        self.assertEqual("search", args[5])
        self.assertEqual((0, 3), args[6])

    def test_sink_error_does_not_break_processing(self):
        self.decoder.process.return_value = ["https://a.test"]
        self.keyframes.side_effect = RuntimeError("disk")
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.assertTrue(self.process("frame", 10.1))
        self.assertEqual(1, self.logic.jobs.qsize())

    def test_station_change_without_url_saves_last_frame(self):
        self.decoder.process.return_value = []
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.logic.submit_frame("dwell-frame", stamp=10.3)
        self.assertTrue(self.logic.process_latest_frame())
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=1)
        self.failed.assert_called_once()
        frame, task, search, meta = self.failed.call_args[0]
        self.assertEqual("dwell-frame", frame)
        self.assertEqual(("task", "search"), (task, search))
        self.assertEqual((0, 0), meta)

    def test_station_change_with_url_skips_failed_save(self):
        self.decoder.process.return_value = ["https://a.test"]
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.process("frame", 10.1)
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=1)
        self.failed.assert_not_called()
        self.keyframes.assert_called_once()

    def test_disable_without_url_saves_last_frame(self):
        self.decoder.process.return_value = []
        self.logic.set_control(True, True, 0.0, pass_index=1, station_index=2)
        self.logic.submit_frame("last-frame", stamp=10.3)
        self.assertTrue(self.logic.process_latest_frame())
        self.logic.set_control(False, False, 0.0)
        self.failed.assert_called_once()
        frame, task, search, meta = self.failed.call_args[0]
        self.assertEqual("last-frame", frame)
        self.assertEqual((1, 2), meta)

    def test_failed_sink_error_is_reported_not_raised(self):
        self.decoder.process.return_value = []
        self.failed.side_effect = RuntimeError("boom")
        errors = Mock()
        self.logic._error_handler = errors
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.logic.submit_frame("f", stamp=10.1)
        self.logic.process_latest_frame()
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=1)
        errors.assert_called()

    def test_reset_search_clears_window_context(self):
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.logic.reset_search("next", "next")
        self.assertIsNone(self.logic._window_meta)
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.decoder.process.return_value = ["https://a.test"]
        self.assertTrue(self.process("frame", 20.5))

    def test_strict_capture_parameters_still_validated(self):
        for kwargs in ({"capture_id": True}, {"capture_id": 0}, {"capture_after": math.nan},
                       {"capture_after": True}, {"pass_index": -1}, {"pass_index": 1.0},
                       {"station_index": -1}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    self.logic.set_control(True, True, 0.0, **kwargs)

    def test_previous_search_result_still_dropped(self):
        entered, release = threading.Event(), threading.Event()

        def slow_decode(_frame):
            entered.set()
            release.wait(2.0)
            return ["https://old.example"]

        self.decoder.process.side_effect = slow_decode
        self.logic.set_control(True, True, 0.0, pass_index=0, station_index=0)
        self.logic.submit_frame("x", stamp=10.0)
        worker = threading.Thread(target=self.logic.process_latest_frame)
        worker.start()
        self.assertTrue(entered.wait(1.0))
        self.logic.reset_search("new-task", "new-search")
        release.set()
        worker.join(2.0)
        self.assertEqual(0, self.logic.jobs.qsize())
        events = [call[0][0]["event"] for call in self.publisher.call_args_list]
        self.assertNotIn("detected", events)


class KeyframeSaverTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.mkdtemp(prefix="qr_keyframes_")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.temp, ignore_errors=True)

    def wait_for_files(self, count, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            names = os.listdir(self.temp)
            if len(names) >= count:
                return names
            time.sleep(0.02)
        return os.listdir(self.temp)

    @staticmethod
    def _imread_unicode(path):
        data = np.fromfile(path, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def wait_for_readable_image(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for name in os.listdir(self.temp):
                image = self._imread_unicode(os.path.join(self.temp, name))
                if image is not None:
                    return image, name
            time.sleep(0.02)
        raise AssertionError("no readable keyframe appeared in %s" % self.temp)

    def test_sanitize_filename(self):
        self.assertEqual("search_01", sanitize_filename("search/01"))
        self.assertEqual("search", sanitize_filename(""))
        self.assertEqual("ab_", sanitize_filename("ab?"))
        self.assertEqual("search", sanitize_filename(None))

    def test_saves_success_keyframe_with_metadata_name(self):
        saver = KeyframeSaver(self.temp)
        saver.save("success", make_frame(), [], "task-1", "search/甲", 0, 4)
        names = self.wait_for_files(1)
        self.assertEqual(1, len(names))
        self.assertIn("success_", names[0])
        self.assertIn("search_", names[0])
        self.assertIn("_p0_s4_", names[0])
        saver.shutdown()

    def test_saves_failed_station_frame_and_rect_annotation(self):
        saver = KeyframeSaver(self.temp)
        rect = (10, 10, 20, 20)
        saver.save("failed", make_frame(), [rect], "task", "search", 1, 0)
        image, name = self.wait_for_readable_image()
        self.assertIn("failed_", name)
        self.assertEqual((48, 64, 3), image.shape)
        saver.shutdown()

    def test_missing_dir_is_created_automatically(self):
        warning = Mock()
        saver = KeyframeSaver(os.path.join(self.temp, "missing", "nested"),
                              warning=warning)
        saver.save("success", make_frame(), [], "task", "search", 0, 0)
        names = self.wait_for_files(1)
        self.assertEqual(1, len(names))
        self.assertFalse(warning.called)
        saver.shutdown()

    def test_none_frame_is_ignored(self):
        warning = Mock()
        saver = KeyframeSaver(self.temp, warning=warning)
        saver.save("success", None, [], "task", "search", 0, 0)
        self.assertEqual([], os.listdir(self.temp))
        saver.shutdown()

    def test_enqueue_failure_only_warns(self):
        warning = Mock()
        saver = KeyframeSaver(self.temp, warning=warning)
        with mock.patch("qr_item_search.keyframes.os.makedirs",
                        side_effect=OSError("denied")):
            saver.save("success", make_frame(), [], "task", "search", 0, 0)
        warning.assert_called()
        saver.shutdown()

    def test_invalid_kind_rejected(self):
        saver = KeyframeSaver(self.temp)
        with self.assertRaises(ValueError):
            saver.save("other", make_frame(), [], "task", "search", 0, 0)
        saver.shutdown()


if __name__ == "__main__":
    unittest.main()
