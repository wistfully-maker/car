"""Concurrent latest-frame QR scanning and payload resolution."""
import math
import queue
import threading
from dataclasses import dataclass

from qr_item_search.qr_payload import InvalidPayload, InvalidQrUrl, validate_url


@dataclass(frozen=True)
class ScannerJob:
    generation: int
    task_id: str
    search_id: str
    order: int
    url: str
    detected_yaw: float


class ScannerLogic:
    def __init__(self, decoder, resolver, event_publisher, quality_function, variant_function,
                 jobs=None, worker_count=3, warning=None, error_handler=None):
        if type(worker_count) is not int or worker_count < 1:
            raise ValueError("worker_count must be a positive integer")
        self._decoder, self._resolver, self._publisher = decoder, resolver, event_publisher
        self._quality, self._variants = quality_function, variant_function
        self._jobs = jobs if jobs is not None else queue.Queue(maxsize=max(6, worker_count * 2))
        self._worker_count = worker_count
        self._warning, self._error_handler = warning or (lambda _: None), error_handler or (lambda _: None)
        self._lock, self._decoder_lock = threading.Lock(), threading.Lock()
        self._generation = 0
        self._task_id = self._search_id = None
        self._enabled = self._enhanced = False
        self._yaw, self._latest, self._next_order = 0.0, None, 1
        self._failed, self._retried, self._deferred, self._retry_deferred = {}, set(), {}, {}
        self._retry_requested = False
        self._pending_decoder_reset = 0

    @property
    def jobs(self): return self._jobs

    def reset_search(self, task_id, search_id):
        task_id, search_id = self._text(task_id, "task_id"), self._text(search_id, "search_id")
        with self._lock:
            self._generation += 1
            self._task_id, self._search_id = task_id, search_id
            self._enabled = self._enhanced = False
            self._yaw, self._latest, self._next_order = 0.0, None, 1
            self._failed, self._retried, self._deferred, self._retry_deferred = {}, set(), {}, {}
            self._retry_requested = False
            self._pending_decoder_reset = self._generation
            self._drain_jobs_locked()

    def set_control(self, enabled, enhanced, detected_yaw, retry_failed=False):
        if type(enabled) is not bool or type(enhanced) is not bool or type(retry_failed) is not bool:
            raise ValueError("control flags must be bool")
        if isinstance(detected_yaw, bool) or not isinstance(detected_yaw, (int, float)) or not math.isfinite(detected_yaw):
            raise ValueError("detected_yaw must be finite")
        retry_jobs = []
        with self._lock:
            self._enabled, self._enhanced, self._yaw = enabled, enhanced, float(detected_yaw)
            rising = retry_failed and not self._retry_requested
            self._retry_requested = retry_failed
            if rising:
                for url, job in self._failed.items():
                    if url not in self._retried:
                        self._retried.add(url); retry_jobs.append(job)
        for job in retry_jobs: self._requeue_retry(job)
        self._flush_deferred()

    def submit_frame(self, frame):
        with self._lock:
            if not self._enabled or self._task_id is None: return False
            self._latest = (frame, self._generation, self._task_id, self._search_id, self._enhanced, self._yaw)
            return True

    def process_latest_frame(self):
        self._flush_deferred()
        with self._lock:
            snapshot, self._latest = self._latest, None
        if snapshot is None: return False
        frame, gen, task, search, enhanced, yaw = snapshot
        if not self._current(gen, task, search): return False
        try: quality = self._quality(frame)
        except Exception as error: self._report_error(error); return False
        valid_urls, invalids = [], []
        try:
            for variant in self._variants(frame, enhanced):
                values = self._decode_variant(variant, gen, task, search)
                if values is None: return False
                variant_valid = []
                for raw in values:
                    try: variant_valid.append(validate_url(raw))
                    except InvalidQrUrl as error: invalids.append((raw, str(error)))
                if variant_valid:
                    valid_urls.extend(variant_valid); break
        except Exception as error: self._report_error(error); return False
        if not self._current(gen, task, search): return False
        seen = set()
        for raw, message in invalids:
            self._publish_current(self._event("invalid_url", task, search, url=str(raw), detected_yaw=yaw, item_name="", message=message), gen, task, search)
        for url in valid_urls:
            if url not in seen:
                seen.add(url); self._enqueue_or_defer(gen, task, search, url, yaw)
        fields = self._quality_fields(quality)
        fields.update(detected_yaw=yaw, decoded=bool(seen))
        self._publish_current(self._event("quality", task, search, **fields), gen, task, search)
        return True

    def work_once(self, block=True, timeout=None):
        try: job = self._jobs.get(block=block, timeout=timeout)
        except queue.Empty: return False
        try:
            try:
                item = self._resolver.resolve(job.url)
                if not isinstance(item, str) or not item.strip(): raise InvalidPayload("empty resolved item")
                event = self._job_event("resolved", job, item_name=item.strip())
            except Exception as error:
                event = self._job_event("resolve_error", job, message=str(error))
                with self._lock:
                    if self._current_locked(job.generation, job.task_id, job.search_id): self._failed[job.url] = job
            self._publish_current(event, job.generation, job.task_id, job.search_id)
        finally:
            self._jobs.task_done()
        self._flush_deferred()
        return True

    def run_workers(self, is_shutdown):
        threads = []
        for _ in range(self._worker_count):
            thread = threading.Thread(target=self._worker_loop, args=(is_shutdown,), daemon=True)
            thread.start(); threads.append(thread)
        return threads

    def _worker_loop(self, is_shutdown):
        while not is_shutdown(): self.work_once(timeout=.2)

    def _decode_variant(self, variant, gen, task, search):
        with self._decoder_lock:
            self._reset_decoder_if_needed()
            if not self._current(gen, task, search): return None
            try: values = self._decoder.process(variant)
            except Exception:
                if not self._current(gen, task, search): self._reset_decoder_if_needed(force=True)
                raise
            if not self._current(gen, task, search):
                self._reset_decoder_if_needed(force=True); return None
            return values

    def _reset_decoder_if_needed(self, force=False):
        with self._lock: pending = self._pending_decoder_reset
        if pending or force:
            self._decoder.reset_search()
            with self._lock:
                if pending: self._pending_decoder_reset = 0

    def _enqueue_or_defer(self, gen, task, search, url, yaw):
        deferred = False
        with self._lock:
            if not self._current_locked(gen, task, search): return
            if url in self._deferred: return
            job = ScannerJob(gen, task, search, self._next_order, url, yaw)
            try: self._jobs.put_nowait(job)
            except queue.Full:
                self._deferred[url] = (gen, task, search, yaw); deferred = True
            if deferred:
                job = None
            else:
                self._next_order += 1
        if deferred:
            self._warn("scanner job queue is full; observation deferred")
            return
        self._publish_current(self._job_event("detected", job), gen, task, search)

    def _flush_deferred(self):
        queued, warned = [], False
        with self._lock:
            for url, job in list(self._retry_deferred.items()):
                if not self._current_locked(job.generation, job.task_id, job.search_id):
                    del self._retry_deferred[url]; continue
                try: self._jobs.put_nowait(job)
                except queue.Full: warned = True; break
                del self._retry_deferred[url]
            for url, (gen, task, search, yaw) in list(self._deferred.items()):
                if not self._current_locked(gen, task, search): del self._deferred[url]; continue
                job = ScannerJob(gen, task, search, self._next_order, url, yaw)
                try: self._jobs.put_nowait(job)
                except queue.Full: warned = True; break
                self._next_order += 1; del self._deferred[url]; queued.append(job)
        if warned: self._warn("scanner job queue is full; observation deferred")
        for job in queued: self._publish_current(self._job_event("detected", job), job.generation, job.task_id, job.search_id)

    def _requeue_retry(self, job):
        deferred = False
        with self._lock:
            if not self._current_locked(job.generation, job.task_id, job.search_id): return
            try: self._jobs.put_nowait(job)
            except queue.Full:
                self._retry_deferred[job.url] = job; deferred = True
        if deferred: self._warn("scanner job queue is full; retry deferred")

    def _drain_jobs_locked(self):
        while True:
            try: self._jobs.get_nowait(); self._jobs.task_done()
            except queue.Empty: return

    def _current(self, gen, task, search):
        with self._lock: return self._current_locked(gen, task, search)
    def _current_locked(self, gen, task, search):
        return gen == self._generation and task == self._task_id and search == self._search_id
    @staticmethod
    def _text(value, name):
        if not isinstance(value, str) or not value.strip(): raise ValueError("%s must be non-empty" % name)
        return value.strip()
    @staticmethod
    def _quality_fields(value):
        if isinstance(value, dict): return {key: value[key] for key in ("brightness", "overexposed", "sharpness")}
        return {key: getattr(value, key) for key in ("brightness", "overexposed", "sharpness")}
    @staticmethod
    def _event(event, task, search, **fields):
        fields.update(event=event, task_id=task, search_id=search); return fields
    def _job_event(self, event, job, item_name="", message=""):
        return self._event(event, job.task_id, job.search_id, order=job.order, url=job.url, detected_yaw=job.detected_yaw, item_name=item_name, message=message)
    def _publish_current(self, event, gen, task, search):
        if not self._current(gen, task, search): return
        try: self._publisher(event)
        except Exception as error: self._report_error(error)
    def _warn(self, message):
        try: self._warning(message)
        except Exception as error: self._report_error(error)
    def _report_error(self, error):
        try: self._error_handler(error)
        except Exception: pass
