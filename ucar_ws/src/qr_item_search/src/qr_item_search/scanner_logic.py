import queue
import threading
from collections import namedtuple

from qr_item_search.qr_payload import InvalidPayload, InvalidQrUrl


ScannerJob = namedtuple(
    "ScannerJob",
    ("wall_index", "url", "search_id"),
)
ScannerJob.__new__.__defaults__ = (0,)
FrameToken = namedtuple(
    "FrameToken",
    ("generation", "search_id", "wall_index"),
)


class FrameAdapter:
    def __init__(self, logic, convert):
        self._logic = logic
        self._convert = convert

    def handle(self, message):
        token = self._logic.reserve_frame()
        if token is None:
            return False
        try:
            image = self._convert(message)
        except Exception as error:
            self._logic.fail_reserved_frame(token, str(error))
            return False
        return self._logic.process_reserved_frame(token, image)


class ScannerLogic:
    _NO_RESET = 0
    _WALL_RESET = 1
    _SEARCH_RESET = 2

    def __init__(
        self,
        decoder,
        resolver,
        publisher,
        jobs=None,
        warning=None,
        error_handler=None,
    ):
        self._decoder = decoder
        self._resolver = resolver
        self._publisher = publisher
        self._warning = warning if warning is not None else lambda message: None
        self._error_handler = (
            error_handler
            if error_handler is not None
            else lambda error: self._warning(
                "scanner publisher failed: {}".format(error)
            )
        )
        self._jobs = jobs if jobs is not None else queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self._enabled = False
        self._busy = False
        self._decoding = False
        self._active_token = None
        self._resetting = False
        self._pending_reset = self._NO_RESET
        self._wall_index = -1
        self._generation = 0
        self._search_id = 0

    @property
    def busy(self):
        with self._lock:
            return self._decoding or self._resetting or self._busy

    @property
    def accepting_images(self):
        with self._lock:
            return (
                self._enabled
                and not self._decoding
                and not self._resetting
                and not self._busy
                and self._wall_index >= 0
            )

    @property
    def jobs(self):
        return self._jobs

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        with self._lock:
            changed = enabled != self._enabled
            self._enabled = enabled
            if changed:
                self._generation += 1
            run_reset = (
                self._schedule_reset_locked(self._WALL_RESET)
                if not enabled
                else False
            )
        if run_reset:
            self._drain_immediate_resets()

    def set_wall_index(self, wall_index):
        if type(wall_index) is not int or wall_index < 0:
            raise ValueError("wall_index must be a non-negative integer")
        with self._lock:
            changed = wall_index != self._wall_index
            self._wall_index = wall_index
            if changed:
                self._generation += 1
            run_reset = (
                self._schedule_reset_locked(self._WALL_RESET)
                if changed
                else False
            )
        if run_reset:
            self._drain_immediate_resets()

    def reset_search(self, search_id):
        if type(search_id) is not int or search_id < 0:
            raise ValueError("search_id must be a non-negative integer")
        with self._lock:
            self._generation += 1
            self._search_id = search_id
            run_reset = self._schedule_reset_locked(self._SEARCH_RESET)
        if run_reset:
            self._drain_immediate_resets()

    def handle_image(self, image):
        token = self.reserve_frame()
        if token is None:
            return False
        return self.process_reserved_frame(token, image)

    def reserve_frame(self):
        with self._lock:
            if (
                not self._enabled
                or self._decoding
                or self._resetting
                or self._busy
                or self._wall_index < 0
            ):
                return None
            self._decoding = True
            token = FrameToken(
                self._generation,
                self._search_id,
                self._wall_index,
            )
            self._active_token = token
            return token

    def process_reserved_frame(self, token, image):
        with self._lock:
            if token != self._active_token:
                return False
        try:
            url = self._decoder.process(image)
        except Exception as error:
            return self._complete_reserved_frame(token, None, error)
        return self._complete_reserved_frame(token, url, None)

    def fail_reserved_frame(self, token, message):
        with self._lock:
            if token != self._active_token:
                return False
        return self._complete_reserved_frame(
            token,
            None,
            RuntimeError(message),
        )

    def _complete_reserved_frame(self, token, url, decode_error):
        warning_message = None
        error_payload = None
        accepted = False
        while True:
            with self._lock:
                reset_kind = self._pending_reset
                if reset_kind != self._NO_RESET:
                    self._pending_reset = self._NO_RESET
                else:
                    current = (
                        token == self._active_token
                        and self._enabled
                        and self._generation == token.generation
                        and self._search_id == token.search_id
                        and self._wall_index == token.wall_index
                    )
                    if decode_error is not None and current:
                        error_payload = self._payload(
                            ScannerJob(
                                token.wall_index,
                                "",
                                token.search_id,
                            ),
                            "decode_error",
                            "",
                            str(decode_error),
                        )
                    elif url and current:
                        job = ScannerJob(
                            token.wall_index,
                            url,
                            token.search_id,
                        )
                        self._busy = True
                        try:
                            self._jobs.put_nowait(job)
                            accepted = True
                        except queue.Full:
                            self._busy = False
                            warning_message = (
                                "scanner job queue is full; "
                                "dropping QR observation"
                            )
                    self._decoding = False
                    self._active_token = None
                    break
            self._run_decoder_reset(reset_kind)

        if warning_message is not None:
            try:
                self._warning(warning_message)
            except Exception:
                pass
        if error_payload is not None:
            self._safe_publish(error_payload)
        return accepted

    def work_once(self, block=True, timeout=None):
        try:
            job = self._jobs.get(block=block, timeout=timeout)
        except queue.Empty:
            return False

        try:
            try:
                item_name = self._resolver.resolve(job.url)
                payload = self._payload(job, "success", item_name, "")
            except InvalidQrUrl as error:
                payload = self._payload(job, "invalid_url", "", str(error))
            except InvalidPayload as error:
                payload = self._payload(
                    job, "invalid_payload", "", str(error)
                )
            except Exception as error:
                payload = self._payload(job, "http_error", "", str(error))
            with self._lock:
                current = job.search_id == self._search_id
            if current:
                self._safe_publish(payload)
        finally:
            with self._lock:
                self._busy = False
            self._jobs.task_done()
        return True

    def run_worker(self, is_shutdown):
        while not is_shutdown():
            self.work_once(timeout=0.2)

    def _schedule_reset_locked(self, reset_kind):
        self._pending_reset = max(self._pending_reset, reset_kind)
        if not self._decoding and not self._resetting:
            self._resetting = True
            return True
        return False

    def _drain_immediate_resets(self):
        while True:
            with self._lock:
                reset_kind = self._pending_reset
                self._pending_reset = self._NO_RESET
            self._run_decoder_reset(reset_kind)
            with self._lock:
                if self._pending_reset == self._NO_RESET:
                    self._resetting = False
                    return

    def _run_decoder_reset(self, reset_kind):
        try:
            if reset_kind == self._SEARCH_RESET:
                self._decoder.reset_search()
            elif reset_kind == self._WALL_RESET:
                self._decoder.reset_wall()
        except Exception as error:
            try:
                self._error_handler(error)
            except Exception:
                pass

    def _safe_publish(self, payload):
        try:
            self._publisher(payload)
        except Exception as error:
            try:
                self._error_handler(error)
            except Exception:
                pass

    @staticmethod
    def _payload(job, status, item_name, message):
        return {
            "status": status,
            "search_id": job.search_id,
            "wall_index": job.wall_index,
            "url": job.url,
            "item_name": item_name,
            "message": message,
        }
