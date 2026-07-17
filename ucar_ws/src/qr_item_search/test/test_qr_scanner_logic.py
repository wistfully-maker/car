import queue
import threading
import unittest
from unittest.mock import Mock

from qr_item_search.qr_payload import InvalidPayload, InvalidQrUrl
from qr_item_search.scanner_logic import ScannerJob, ScannerLogic


class StatefulBlockingDecoder:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.candidate = None
        self.events = []
        self.observed_candidates = []
        self.block_next = True

    def process(self, image):
        if self.block_next:
            self.block_next = False
            self.entered.set()
            self.release.wait(1.0)
            self.candidate = "stale"
            self.events.append("process_end")
            return None
        self.observed_candidates.append(self.candidate)
        return None

    def reset_wall(self):
        self.events.append("reset_wall")
        self.candidate = None

    def reset_search(self):
        self.events.append("reset_search")
        self.candidate = None


class ScannerLogicTest(unittest.TestCase):
    def setUp(self):
        self.decoder = Mock()
        self.resolver = Mock()
        self.publisher = Mock()
        self.logic = ScannerLogic(
            decoder=self.decoder,
            resolver=self.resolver,
            publisher=self.publisher,
        )

    def test_disabled_scanner_ignores_images(self):
        self.logic.handle_image(object())

        self.decoder.process.assert_not_called()

    def test_success_uses_wall_snapshot_from_enqueue_time(self):
        self.decoder.process.return_value = "https://example.test/item"
        self.resolver.resolve.return_value = "香蕉"
        self.logic.set_wall_index(2)
        self.logic.set_enabled(True)

        self.assertTrue(self.logic.handle_image(object()))
        self.logic.set_wall_index(3)
        self.assertTrue(self.logic.work_once(block=False))

        self.publisher.assert_called_once_with(
            {
                "status": "success",
                "search_id": 0,
                "wall_index": 2,
                "url": "https://example.test/item",
                "item_name": "香蕉",
                "message": "",
            }
        )

    def test_maps_resolver_exceptions_to_status(self):
        cases = (
            (InvalidQrUrl("bad"), "invalid_url"),
            (InvalidPayload("bad"), "invalid_payload"),
            (RuntimeError("offline"), "http_error"),
        )
        for error, status in cases:
            with self.subTest(status=status):
                decoder = Mock()
                decoder.process.return_value = "https://example.test/item"
                resolver = Mock()
                resolver.resolve.side_effect = error
                publisher = Mock()
                logic = ScannerLogic(decoder, resolver, publisher)
                logic.set_wall_index(0)
                logic.set_enabled(True)
                logic.handle_image(object())

                logic.work_once(block=False)

                payload = publisher.call_args[0][0]
                self.assertEqual(status, payload["status"])
                self.assertEqual(str(error), payload["message"])

    def test_disabling_resets_wall_candidate(self):
        self.logic.set_enabled(True)

        self.logic.set_enabled(False)

        self.decoder.reset_wall.assert_called_once_with()

    def test_reset_search_clears_decoder_seen_values(self):
        self.logic.reset_search(3)

        self.decoder.reset_search.assert_called_once_with()

    def test_reset_search_rejects_invalid_search_id(self):
        for search_id in (-1, True, 1.5, None):
            with self.subTest(search_id=search_id):
                with self.assertRaises(ValueError):
                    self.logic.reset_search(search_id)

    def test_busy_scanner_does_not_enqueue_duplicate(self):
        self.decoder.process.return_value = "https://example.test/item"
        self.logic.set_wall_index(0)
        self.logic.set_enabled(True)

        self.assertTrue(self.logic.handle_image(object()))
        self.assertFalse(self.logic.handle_image(object()))

        self.decoder.process.assert_called_once()
        self.assertEqual(1, self.logic.jobs.qsize())

    def test_full_queue_does_not_raise_and_recovers_busy_state(self):
        jobs = queue.Queue(maxsize=1)
        jobs.put_nowait(ScannerJob(0, "existing"))
        logic = ScannerLogic(
            decoder=self.decoder,
            resolver=self.resolver,
            publisher=self.publisher,
            jobs=jobs,
        )
        self.decoder.process.return_value = "https://example.test/item"
        logic.set_wall_index(0)
        logic.set_enabled(True)

        self.assertFalse(logic.handle_image(object()))

        self.assertFalse(logic.busy)

    def test_worker_clears_busy_and_marks_job_done(self):
        self.decoder.process.return_value = "https://example.test/item"
        self.resolver.resolve.return_value = "香蕉"
        self.logic.set_wall_index(0)
        self.logic.set_enabled(True)
        self.logic.handle_image(object())

        self.logic.work_once(block=False)

        self.assertFalse(self.logic.busy)
        self.logic.jobs.join()

    def test_missing_wall_index_ignores_image(self):
        self.logic.set_enabled(True)

        self.assertFalse(self.logic.handle_image(object()))

        self.decoder.process.assert_not_called()

    def test_rejects_invalid_wall_index(self):
        for wall_index in (-1, 1.5, True, None):
            with self.subTest(wall_index=wall_index):
                with self.assertRaises(ValueError):
                    self.logic.set_wall_index(wall_index)

    def test_disable_does_not_wait_for_decode_and_discards_result(self):
        entered = threading.Event()
        release = threading.Event()

        def blocking_decode(image):
            entered.set()
            release.wait(1.0)
            return "https://example.test/item"

        self.decoder.process.side_effect = blocking_decode
        self.logic.set_wall_index(0)
        self.logic.set_enabled(True)
        decode_thread = threading.Thread(
            target=self.logic.handle_image,
            args=(object(),),
        )
        decode_thread.start()
        self.assertTrue(entered.wait(1.0))
        self.assertTrue(self.logic.busy)
        self.assertFalse(self.logic.handle_image(object()))

        disable_thread = threading.Thread(
            target=self.logic.set_enabled,
            args=(False,),
        )
        disable_thread.start()
        disable_thread.join(0.2)
        completed_without_decode = not disable_thread.is_alive()
        release.set()
        decode_thread.join(1.0)

        self.assertTrue(completed_without_decode)
        self.assertFalse(decode_thread.is_alive())
        self.decoder.process.assert_called_once()
        self.assertEqual(0, self.logic.jobs.qsize())
        self.assertFalse(self.logic.busy)

    def test_wall_change_does_not_wait_for_decode_and_discards_result(self):
        entered = threading.Event()
        release = threading.Event()

        def blocking_decode(image):
            entered.set()
            release.wait(1.0)
            return "https://example.test/item"

        self.decoder.process.side_effect = blocking_decode
        self.logic.set_wall_index(0)
        self.logic.set_enabled(True)
        decode_thread = threading.Thread(
            target=self.logic.handle_image,
            args=(object(),),
        )
        decode_thread.start()
        self.assertTrue(entered.wait(1.0))

        wall_thread = threading.Thread(
            target=self.logic.set_wall_index,
            args=(1,),
        )
        wall_thread.start()
        wall_thread.join(0.2)
        completed_without_decode = not wall_thread.is_alive()
        release.set()
        decode_thread.join(1.0)

        self.assertTrue(completed_without_decode)
        self.assertFalse(decode_thread.is_alive())
        self.assertEqual(0, self.logic.jobs.qsize())
        self.assertFalse(self.logic.busy)

    def test_decode_exception_publishes_error_and_releases_busy(self):
        self.decoder.process.side_effect = UnicodeDecodeError(
            "utf-8", b"\xff", 0, 1, "invalid"
        )
        self.logic.set_wall_index(2)
        self.logic.set_enabled(True)

        self.assertFalse(self.logic.handle_image(object()))

        self.publisher.assert_called_once()
        payload = self.publisher.call_args[0][0]
        self.assertEqual("decode_error", payload["status"])
        self.assertEqual(2, payload["wall_index"])
        self.assertFalse(self.logic.busy)

    def test_publisher_exception_is_not_republished_and_worker_continues(self):
        self.decoder.process.side_effect = [
            "https://example.test/a",
            "https://example.test/b",
        ]
        self.resolver.resolve.side_effect = ["苹果", "香蕉"]
        self.publisher.side_effect = [RuntimeError("publisher down"), None]
        error_handler = Mock()
        logic = ScannerLogic(
            self.decoder,
            self.resolver,
            self.publisher,
            error_handler=error_handler,
        )
        logic.set_wall_index(0)
        logic.set_enabled(True)

        logic.handle_image(object())
        logic.work_once(block=False)
        logic.handle_image(object())
        logic.work_once(block=False)

        self.assertEqual(2, self.publisher.call_count)
        self.assertEqual(2, self.resolver.resolve.call_count)
        error_handler.assert_called_once()

    def test_wall_reset_runs_after_blocking_decode_and_clears_candidate(self):
        decoder = StatefulBlockingDecoder()
        logic = ScannerLogic(decoder, self.resolver, self.publisher)
        logic.set_wall_index(0)
        decoder.events = []
        logic.set_enabled(True)
        decode_thread = threading.Thread(
            target=logic.handle_image,
            args=(object(),),
        )
        decode_thread.start()
        self.assertTrue(decoder.entered.wait(1.0))

        wall_thread = threading.Thread(
            target=logic.set_wall_index,
            args=(1,),
        )
        wall_thread.start()
        wall_thread.join(0.2)
        callback_returned = not wall_thread.is_alive()
        decoder.release.set()
        decode_thread.join(1.0)

        self.assertTrue(callback_returned)
        self.assertEqual(["process_end", "reset_wall"], decoder.events)
        self.assertFalse(logic.busy)
        logic.handle_image(object())
        self.assertEqual([None], decoder.observed_candidates)

    def test_search_reset_supersedes_pending_wall_reset(self):
        decoder = StatefulBlockingDecoder()
        logic = ScannerLogic(decoder, self.resolver, self.publisher)
        logic.set_wall_index(0)
        decoder.events = []
        logic.set_enabled(True)
        decode_thread = threading.Thread(
            target=logic.handle_image,
            args=(object(),),
        )
        decode_thread.start()
        self.assertTrue(decoder.entered.wait(1.0))

        logic.set_wall_index(1)
        reset_thread = threading.Thread(target=logic.reset_search, args=(4,))
        reset_thread.start()
        reset_thread.join(0.2)
        callback_returned = not reset_thread.is_alive()
        decoder.release.set()
        decode_thread.join(1.0)

        self.assertTrue(callback_returned)
        self.assertEqual(["process_end", "reset_search"], decoder.events)
        logic.handle_image(object())
        self.assertEqual([None], decoder.observed_candidates)

    def test_queue_full_warning_can_reenter_logic_without_deadlock(self):
        jobs = queue.Queue(maxsize=1)
        jobs.put_nowait(ScannerJob(0, "existing"))
        holder = {}
        warning_called = threading.Event()

        def reentrant_warning(message):
            holder["logic"].accepting_images
            warning_called.set()

        logic = ScannerLogic(
            self.decoder,
            self.resolver,
            self.publisher,
            jobs=jobs,
            warning=reentrant_warning,
        )
        holder["logic"] = logic
        self.decoder.process.return_value = "https://example.test/item"
        logic.set_wall_index(0)
        logic.set_enabled(True)
        callback_thread = threading.Thread(
            target=logic.handle_image,
            args=(object(),),
        )
        callback_thread.daemon = True

        callback_thread.start()
        callback_thread.join(0.2)

        self.assertFalse(callback_thread.is_alive())
        self.assertTrue(warning_called.is_set())

    def test_reset_search_discards_inflight_resolver_result(self):
        entered = threading.Event()
        release = threading.Event()

        def blocking_resolve(url):
            entered.set()
            release.wait(1.0)
            return "stale item"

        self.decoder.process.return_value = "https://example.test/item"
        self.resolver.resolve.side_effect = blocking_resolve
        self.logic.set_wall_index(0)
        self.logic.set_enabled(True)
        self.logic.handle_image(object())
        worker = threading.Thread(target=self.logic.work_once)
        worker.start()
        self.assertTrue(entered.wait(1.0))

        self.logic.reset_search(1)
        release.set()
        worker.join(1.0)

        self.publisher.assert_not_called()
        self.assertFalse(self.logic.busy)


if __name__ == "__main__":
    unittest.main()
