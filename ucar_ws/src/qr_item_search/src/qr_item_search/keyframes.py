"""Asynchronous keyframe saving for successful detections and failed stations.

The saver owns a background thread so JPEG encoding and disk writes never
block the ROS image callback or the QR decoder.  Failures are reported as
warnings only.
"""
import os
import queue
import re
import threading
import time


def sanitize_filename(value, fallback="search"):
    """Keep filename-safe characters; empty or exotic values fall back."""
    if not isinstance(value, str):
        return fallback
    cleaned = re.sub(r"[^A-Za-z0-9._-]", "_", value)
    return cleaned[:80] or fallback


class KeyframeSaver:
    def __init__(self, output_dir, jpeg_quality=85, warning=None, error_handler=None, clock=None):
        if not isinstance(output_dir, str) or not output_dir.strip():
            raise ValueError("output_dir must be a non-empty string")
        self._output_dir = output_dir
        self._jpeg_quality = int(jpeg_quality)
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self._warning = warning or (lambda _: None)
        self._error_handler = error_handler or (lambda _: None)
        self._clock = clock or time.time
        self._queue = queue.Queue(maxsize=32)
        self._shutdown = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def save(self, kind, frame, rects, task_id, search_id, pass_index, station_index):
        """Enqueue a frame for asynchronous saving; never raises."""
        if frame is None:
            return
        if kind not in ("success", "failed"):
            raise ValueError("kind must be success or failed")
        try:
            stamp = float(self._clock())
            name = "{}_{}_p{}_s{}_{:.3f}.jpg".format(
                kind, sanitize_filename(search_id),
                int(pass_index or 0), int(station_index or 0), stamp)
            os.makedirs(self._output_dir, exist_ok=True)
            self._queue.put_nowait((os.path.join(self._output_dir, name), frame, tuple(rects or ())))
        except Exception as error:
            self._warning("keyframe enqueue failed: %s" % error)

    def shutdown(self):
        """Stop the worker thread after draining pending jobs."""
        if self._shutdown.is_set():
            return
        self._shutdown.set()
        try:
            self._queue.put(None, timeout=2.0)
        except queue.Full:
            self._warning("keyframe queue full during shutdown; dropping pending saves")
        self._thread.join(timeout=5.0)

    def _run(self):
        import cv2

        while True:
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            path, frame, rects = item
            try:
                encoded = frame
                if rects:
                    encoded = frame.copy()
                    for rect in rects:
                        x, y, width, height = (int(value) for value in rect)
                        cv2.rectangle(encoded, (x, y), (x + width, y + height), (0, 255, 0), 2)
                ok, buffer = cv2.imencode(
                    ".jpg", encoded, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
                if not ok:
                    raise ValueError("JPEG encoding failed")
                with open(path, "wb") as handle:
                    handle.write(buffer.tobytes())
            except Exception as error:
                self._warning("keyframe write failed: %s" % error)
            finally:
                self._queue.task_done()
