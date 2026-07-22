import math
import queue
import threading
import unittest
from unittest.mock import Mock

from qr_item_search.scanner_logic import ScannerLogic


class ScannerLogicTest(unittest.TestCase):
    def setUp(self):
        self.decoder = Mock()
        self.resolver = Mock()
        self.publisher = Mock()
        self.quality = Mock(return_value={"brightness": 1.0, "overexposed": 0.0, "sharpness": 2.0})
        self.variants = Mock(side_effect=lambda frame, enhanced: [frame])
        self.logic = ScannerLogic(self.decoder, self.resolver, self.publisher,
                                  self.quality, self.variants, worker_count=1)
        self.logic.reset_search(" task ", " search ")
        self.logic.set_control(True, False, 1.5)

    def _process(self, frame="frame"):
        self.assertTrue(self.logic.submit_frame(frame))
        return self.logic.process_latest_frame()

    def test_latest_frame_overwrites_and_disabled_rejects(self):
        self.logic.submit_frame("first")
        self.logic.submit_frame("latest")
        self.logic.process_latest_frame()
        self.quality.assert_called_once_with("latest")
        self.logic.set_control(False, False, 0)
        self.assertFalse(self.logic.submit_frame("ignored"))

    def test_disabling_discards_pending_latest_frame(self):
        self.assertTrue(self.logic.submit_frame("pending"))
        self.logic.set_control(False, False, 0)
        self.assertFalse(self.logic.process_latest_frame())
        self.quality.assert_not_called()

    def test_detects_three_urls_in_order_and_http_pending_allows_next_frame(self):
        self.decoder.process.side_effect = [["https://a", "https://b", "https://c"], ["https://d"]]
        self._process("one")
        self._process("two")
        self.assertEqual([1, 2, 3], [job.order for job in list(self.logic.jobs.queue)])
        detected = [call[0][0] for call in self.publisher.call_args_list if call[0][0]["event"] == "detected"]
        self.assertEqual(["https://a", "https://b", "https://c"], [event["url"] for event in detected])

    def test_reset_allows_three_urls_again_after_capacity_reached(self):
        self.decoder.process.side_effect = [["https://1", "https://2", "https://3", "https://4"],
                                            ["https://4", "https://5", "https://6"]]
        self._process()
        self.assertEqual(3, self.logic.jobs.qsize())
        self.logic.reset_search("task2", "search2")
        self.logic.set_control(True, False, 0)
        self._process()
        self.assertEqual(3, self.logic.jobs.qsize())

    def test_invalid_url_does_not_consume_order(self):
        self.decoder.process.return_value = ["bad", "https://good"]
        self._process()
        self.assertEqual(1, self.logic.jobs.get_nowait().order)
        events = [call[0][0]["event"] for call in self.publisher.call_args_list]
        self.assertIn("invalid_url", events)

    def test_duplicate_url_only_uses_one_order(self):
        self.decoder.process.return_value = ["https://same", "https://same"]
        self._process()
        self.assertEqual(1, self.logic.jobs.qsize())
        self.assertEqual(1, self.logic.jobs.get_nowait().order)

    def test_quality_and_enhanced_short_circuit(self):
        self.logic.set_control(True, True, 2)
        self.quality.return_value = {"brightness": 1.0, "overexposed": 0.0, "sharpness": 2.0}
        self.variants.side_effect = None
        self.variants.return_value = ["one", "two", "three"]
        self.decoder.process.side_effect = [[], ["https://a"]]
        self._process()
        self.assertEqual(["one", "two"], [call[0][0] for call in self.decoder.process.call_args_list])
        quality = [call[0][0] for call in self.publisher.call_args_list if call[0][0]["event"] == "quality"][0]
        self.assertEqual("quality", quality["event"])
        self.assertEqual(1.0, quality["brightness"])

    def test_queue_full_warns_without_deadlock(self):
        jobs = queue.Queue(maxsize=1)
        jobs.put(object())
        warning = Mock()
        logic = ScannerLogic(self.decoder, self.resolver, self.publisher, self.quality,
                             self.variants, jobs=jobs, warning=warning, worker_count=1)
        logic.reset_search("task", "search")
        logic.set_control(True, False, 0)
        jobs.put_nowait(object())
        self.decoder.process.return_value = ["https://a"]
        logic.submit_frame("x")
        logic.process_latest_frame()
        warning.assert_called_once()

    def test_worker_publishes_order_independent_of_completion_and_error(self):
        self.decoder.process.return_value = ["https://a", "https://b"]
        self._process()
        self.resolver.resolve.side_effect = ["A", RuntimeError("offline")]
        self.logic.work_once(block=False)
        self.logic.work_once(block=False)
        events = [call[0][0] for call in self.publisher.call_args_list]
        self.assertEqual("resolved", events[-2]["event"])
        self.assertEqual("resolve_error", events[-1]["event"])
        self.assertEqual([1, 2], [events[-2]["order"], events[-1]["order"]])

    def test_reset_discards_stale_decode_and_resolver_results(self):
        entered, release = threading.Event(), threading.Event()
        def slow_decode(frame):
            entered.set(); release.wait(1); return ["https://a"]
        self.decoder.process.side_effect = slow_decode
        self.logic.submit_frame("x")
        thread = threading.Thread(target=self.logic.process_latest_frame)
        thread.start(); self.assertTrue(entered.wait(1))
        self.logic.reset_search("new-task", "new-search")
        release.set(); thread.join(1)
        self.assertEqual(0, self.logic.jobs.qsize())

    def test_retry_failed_once(self):
        self.decoder.process.return_value = ["https://a"]
        self._process(); self.resolver.resolve.side_effect = RuntimeError("bad")
        self.logic.work_once(block=False)
        self.logic.set_control(True, False, 0, retry_failed=True)
        self.assertEqual(1, self.logic.jobs.qsize())

    def test_retry_preserves_original_job_without_duplicate_detected_event(self):
        self.decoder.process.return_value = ["https://a"]
        self._process(); original = self.logic.jobs.get_nowait(); self.logic.jobs.task_done()
        self.resolver.resolve.side_effect = RuntimeError("bad")
        self.logic.jobs.put_nowait(original); self.logic.work_once(block=False)
        detected_before = len([call for call in self.publisher.call_args_list if call[0][0]["event"] == "detected"])
        self.logic.set_control(True, False, 99, retry_failed=True)
        retry = self.logic.jobs.get_nowait(); self.logic.jobs.task_done()
        self.assertEqual(original, retry)
        self.assertEqual(detected_before, len([call for call in self.publisher.call_args_list if call[0][0]["event"] == "detected"]))
        self.logic.set_control(True, False, 99, retry_failed=True)
        self.assertEqual(0, self.logic.jobs.qsize())

    def test_full_retry_is_deferred_without_changing_original_job(self):
        jobs = queue.Queue(maxsize=1)
        logic = ScannerLogic(self.decoder, self.resolver, self.publisher, self.quality, self.variants, jobs=jobs, worker_count=1)
        logic.reset_search("task", "search"); logic.set_control(True, False, 7)
        self.decoder.process.return_value = ["https://a"]
        logic.submit_frame("x"); logic.process_latest_frame()
        original = jobs.get_nowait(); jobs.task_done(); jobs.put_nowait(original)
        self.resolver.resolve.side_effect = RuntimeError("bad"); logic.work_once(block=False)
        jobs.put_nowait(object())
        logic.set_control(True, False, 99, retry_failed=True)
        jobs.get_nowait(); jobs.task_done()
        logic.process_latest_frame()
        retry = jobs.get_nowait(); jobs.task_done()
        self.assertEqual(original, retry)

    def test_errors_do_not_deadlock_and_workers_are_daemon(self):
        error_handler = Mock()
        logic = ScannerLogic(self.decoder, self.resolver, lambda event: (_ for _ in ()).throw(RuntimeError()),
                             self.quality, self.variants, worker_count=2, error_handler=error_handler)
        logic.reset_search("task", "search"); logic.set_control(True, False, 0)
        self.quality.side_effect = RuntimeError("quality")
        logic.submit_frame("x"); logic.process_latest_frame()
        stop = threading.Event()
        threads = logic.run_workers(stop.is_set)
        self.assertEqual(2, len(threads)); self.assertTrue(all(thread.daemon for thread in threads))
        stop.set()

    def test_strict_parameters(self):
        for count in (0, True, 1.0):
            with self.subTest(count=count):
                with self.assertRaises(ValueError):
                    ScannerLogic(self.decoder, self.resolver, self.publisher, self.quality, self.variants, worker_count=count)
        with self.assertRaises(ValueError): self.logic.reset_search(" ", "search")
        for args in ((1, False, 0), (True, 0, 0), (True, False, True), (True, False, math.inf)):
            with self.subTest(args=args):
                with self.assertRaises(ValueError): self.logic.set_control(*args)

    def test_quality_publishes_after_detection_with_valid_decoded_flag(self):
        self.decoder.process.return_value = ["bad", "https://a"]
        self._process()
        events = [call[0][0] for call in self.publisher.call_args_list]
        self.assertEqual("quality", events[-1]["event"])
        self.assertTrue(events[-1]["decoded"])
        self.assertEqual(1.5, events[-1]["detected_yaw"])

    def test_invalid_variant_does_not_short_circuit_later_variants(self):
        self.logic.set_control(True, True, 0)
        self.variants.side_effect = None; self.variants.return_value = ["bad", "good"]
        self.decoder.process.side_effect = [["bad"], ["https://a"]]
        self._process()
        self.assertEqual(2, self.decoder.process.call_count)

    def test_reset_disables_and_clears_latest_frame(self):
        self.logic.submit_frame("old")
        self.logic.reset_search("new", "new")
        self.assertFalse(self.logic.submit_frame("ignored"))
        self.assertFalse(self.logic.process_latest_frame())

    def test_deferred_observation_gets_first_order_after_queue_slot_opens(self):
        jobs = queue.Queue(maxsize=1); jobs.put_nowait(object())
        logic = ScannerLogic(self.decoder, self.resolver, self.publisher, self.quality, self.variants, jobs=jobs, worker_count=1)
        logic.reset_search("task", "search"); logic.set_control(True, False, 0)
        jobs.put_nowait(object()); self.decoder.process.return_value = ["https://a"]
        logic.submit_frame("frame"); logic.process_latest_frame()
        jobs.get_nowait(); jobs.task_done()
        logic.process_latest_frame()
        self.assertEqual(1, jobs.get_nowait().order)

    def test_work_once_marks_task_done_on_resolver_error(self):
        self.decoder.process.return_value = ["https://a"]; self._process()
        self.resolver.resolve.side_effect = RuntimeError("down")
        self.logic.work_once(block=False)
        self.assertEqual(0, self.logic.jobs.unfinished_tasks)

    def test_decoder_error_allows_following_frame(self):
        self.decoder.process.side_effect = [RuntimeError("decode"), ["https://a"]]
        self._process(); self._process()
        self.assertEqual(1, self.logic.jobs.qsize())

    def test_publisher_error_calls_error_handler(self):
        errors = Mock()
        logic = ScannerLogic(self.decoder, self.resolver, Mock(side_effect=RuntimeError("publish")), self.quality, self.variants, worker_count=1, error_handler=errors)
        logic.reset_search("task", "search"); logic.set_control(True, False, 0)
        self.decoder.process.return_value = ["https://a"]
        logic.submit_frame("x"); logic.process_latest_frame()
        errors.assert_called()

    def test_slow_resolver_reset_does_not_publish_or_record_failure(self):
        entered, release = threading.Event(), threading.Event()
        self.decoder.process.return_value = ["https://a"]; self._process()
        def slow(url): entered.set(); release.wait(1); raise RuntimeError("old")
        self.resolver.resolve.side_effect = slow
        worker = threading.Thread(target=self.logic.work_once); worker.start(); self.assertTrue(entered.wait(1))
        self.logic.reset_search("next", "next"); release.set(); worker.join(1)
        self.assertFalse(self.logic._failed)

    def test_workers_exit_when_shutdown_is_true(self):
        stopped = threading.Event(); stopped.set()
        threads = self.logic.run_workers(stopped.is_set)
        for thread in threads: thread.join(.5)
        self.assertTrue(all(not thread.is_alive() for thread in threads))

    def test_reset_during_slow_decode_drops_old_result_and_resets_decoder(self):
        entered, release = threading.Event(), threading.Event()
        def slow(frame): entered.set(); release.wait(1); return ["https://old"]
        self.decoder.process.side_effect = slow
        self.logic.submit_frame("x"); worker = threading.Thread(target=self.logic.process_latest_frame)
        worker.start(); self.assertTrue(entered.wait(1)); self.logic.reset_search("next", "next")
        release.set(); worker.join(1)
        self.assertEqual(0, self.logic.jobs.qsize())
        self.assertTrue(self.decoder.reset_search.called)

    def test_blocking_publisher_does_not_block_reset(self):
        entered, release = threading.Event(), threading.Event()
        def publish(event): entered.set(); release.wait(1)
        logic = ScannerLogic(self.decoder, self.resolver, publish, self.quality, self.variants, worker_count=1)
        logic.reset_search("task", "search"); logic.set_control(True, False, 0)
        self.decoder.process.return_value = ["https://a"]
        logic.submit_frame("x"); worker = threading.Thread(target=logic.process_latest_frame)
        worker.start(); self.assertTrue(entered.wait(1))
        reset = threading.Thread(target=logic.reset_search, args=("next", "next")); reset.start(); reset.join(.2)
        self.assertFalse(reset.is_alive())
        release.set(); worker.join(1)

    def test_concurrent_resolvers_may_publish_reverse_completion_with_fixed_orders(self):
        self.decoder.process.return_value = ["https://a", "https://b"]; self._process()
        a_entered, release_a, b_done = threading.Event(), threading.Event(), threading.Event()
        def resolve(url):
            if url.endswith("a"):
                a_entered.set(); release_a.wait(1); return "A"
            b_done.set(); return "B"
        self.resolver.resolve.side_effect = resolve
        first = threading.Thread(target=self.logic.work_once, kwargs={"block": False})
        second = threading.Thread(target=self.logic.work_once, kwargs={"block": False})
        first.start(); self.assertTrue(a_entered.wait(1)); second.start(); self.assertTrue(b_done.wait(1)); release_a.set()
        first.join(1); second.join(1)
        resolved = [call[0][0] for call in self.publisher.call_args_list if call[0][0]["event"] == "resolved"]
        self.assertEqual([2, 1], [event["order"] for event in resolved])

    def test_worker_cannot_resolve_before_detected_event(self):
        events = []
        logic = ScannerLogic(self.decoder, self.resolver, events.append, self.quality, self.variants, worker_count=1)
        logic.reset_search("task", "search"); logic.set_control(True, False, 0)
        self.decoder.process.return_value = ["https://a"]; self.resolver.resolve.return_value = "item"
        stop = threading.Event(); logic.run_workers(stop.is_set)
        logic.submit_frame("x"); logic.process_latest_frame()
        for _ in range(20):
            if any(event["event"] == "resolved" for event in events): break
            threading.Event().wait(.02)
        stop.set()
        sequence = [event["event"] for event in events if event["event"] in ("detected", "resolved")]
        self.assertEqual(["detected", "resolved"], sequence)

    def test_retry_requested_before_blocking_failure_requeues_original_once(self):
        entered, release = threading.Event(), threading.Event()
        self.decoder.process.return_value = ["https://a"]; self._process()
        original = self.logic.jobs.get_nowait(); self.logic.jobs.task_done(); self.logic.jobs.put_nowait(original)
        def fail(url): entered.set(); release.wait(1); raise RuntimeError("bad")
        self.resolver.resolve.side_effect = fail
        worker = threading.Thread(target=self.logic.work_once); worker.start(); self.assertTrue(entered.wait(1))
        self.logic.set_control(True, False, 0, retry_failed=True); release.set(); worker.join(1)
        retry = self.logic.jobs.get_nowait(); self.logic.jobs.task_done()
        self.assertEqual(original, retry)
        self.logic.set_control(True, False, 0, retry_failed=True)
        self.assertEqual(0, self.logic.jobs.qsize())


    def test_full_queue_reserves_only_first_three_unique_urls(self):
        jobs = queue.Queue(maxsize=1)
        logic = ScannerLogic(self.decoder, self.resolver, self.publisher, self.quality,
                             self.variants, jobs=jobs, warning=Mock(), worker_count=1)
        logic.reset_search("task", "search"); logic.set_control(True, False, 0)
        jobs.put_nowait(object())
        self.decoder.process.return_value = ["https://1", "https://2", "https://3", "https://4"]
        logic.submit_frame("x"); logic.process_latest_frame()
        self.assertEqual({"https://1", "https://2", "https://3"}, logic._reserved_urls)
        self.assertEqual(3, len(logic._deferred))

    def test_rejects_invalid_expected_count(self):
        for value in (0, True, 1.0):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ScannerLogic(self.decoder, self.resolver, self.publisher, self.quality,
                                 self.variants, expected_count=value)


if __name__ == "__main__":
    unittest.main()
