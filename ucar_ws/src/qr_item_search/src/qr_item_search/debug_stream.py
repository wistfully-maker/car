"""MJPEG debug stream served over HTTP using only the Python standard library.

The server holds the latest JPEG and never performs I/O inside the ROS
image callback: clients are served by their own handler threads, and
slow clients can only block themselves.
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "qr-item-search-boundary"

INDEX_HTML = (
    b"<html><head><title>qr_item_search debug</title></head>"
    b"<body><h1>qr_item_search debug stream</h1>"
    b"<img src=\"/stream.mjpg\" style=\"max-width:90vw\"/></body></html>"
)


def draw_overlay(image, state):
    """Draw state, yaw, settled flag, progress and image quality on a frame."""
    import cv2

    lines = ["state: %s" % state.get("state", "?")]
    lines.append("yaw rel: %.1f deg   target: %.1f deg" % (
        state.get("relative_yaw_deg", 0.0), state.get("target_yaw_deg", 0.0)))
    lines.append("settled: %s   found: %s/%s" % (
        state.get("settled", False), state.get("found_count", 0),
        state.get("expected_count", 3)))
    brightness = state.get("brightness")
    if brightness is not None:
        lines.append("brightness: %.0f   overexposed: %.2f   sharpness: %.0f" % (
            brightness, state.get("overexposed", 0.0), state.get("sharpness", 0.0)))
    overlay = image.copy()
    cursor = 20
    for line in lines:
        cv2.putText(overlay, line, (10, cursor), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cursor += 18
    return overlay


class MjpegStreamServer:
    def __init__(self, host="0.0.0.0", port=8080):
        self._condition = threading.Condition()
        self._latest = None
        self._httpd = ThreadingHTTPServer((host, port), self._handler_factory())
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def _handler_factory(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                server._dispatch(self)

            def log_message(self, format, *args):
                pass

        return Handler

    @property
    def address(self):
        return self._httpd.server_address

    def set_latest(self, jpeg):
        """Store the newest JPEG; never blocks on client I/O."""
        if not isinstance(jpeg, bytes) or not jpeg:
            raise ValueError("latest JPEG must be non-empty bytes")
        with self._condition:
            self._latest = jpeg
            self._condition.notify_all()

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._httpd.shutdown()
        self._httpd.server_close()

    def _dispatch(self, handler):
        path = handler.path.split("?", 1)[0]
        if path == "/":
            self._send_index(handler)
        elif path == "/snapshot.jpg":
            self._send_snapshot(handler)
        elif path == "/stream.mjpg":
            self._send_stream(handler)
        else:
            handler.send_error(404)

    def _send_index(self, handler):
        handler.send_response(200)
        handler.send_header("Content-Type", "text/html; charset=utf-8")
        handler.send_header("Content-Length", str(len(INDEX_HTML)))
        handler.end_headers()
        handler.wfile.write(INDEX_HTML)

    def _send_snapshot(self, handler):
        with self._condition:
            latest = self._latest
        if latest is None:
            handler.send_error(404, "no frame available yet")
            return
        handler.send_response(200)
        handler.send_header("Content-Type", "image/jpeg")
        handler.send_header("Content-Length", str(len(latest)))
        handler.end_headers()
        handler.wfile.write(latest)

    def _send_stream(self, handler):
        handler.send_response(200)
        handler.send_header("Content-Type",
                            "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
        handler.end_headers()
        last = None
        while True:
            with self._condition:
                while self._latest is last:
                    self._condition.wait(timeout=2.0)
                jpeg = self._latest
                last = self._latest
            try:
                handler.wfile.write(
                    b"--%b\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                    % (BOUNDARY.encode(), len(jpeg)))
                handler.wfile.write(jpeg)
                handler.wfile.write(b"\r\n")
                handler.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
