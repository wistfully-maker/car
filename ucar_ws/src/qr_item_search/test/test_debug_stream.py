import http.client
import time
import unittest

import numpy as np

from qr_item_search.debug_stream import BOUNDARY, MjpegStreamServer, draw_overlay


def sample_jpeg(size=200):
    return b"\xff\xd8\xff\xe0" + b"x" * size


class MjpegStreamServerTest(unittest.TestCase):
    def setUp(self):
        self.server = MjpegStreamServer("127.0.0.1", 0).start()

    def tearDown(self):
        self.server.stop()

    def url(self):
        return "127.0.0.1", self.server.address[1]

    def request(self, path):
        host, port = self.url()
        conn = http.client.HTTPConnection(host, port, timeout=5)
        conn.request("GET", path)
        response = conn.getresponse()
        body = response.read()
        conn.close()
        return response.status, response.getheader("Content-Type"), body

    def test_index_serves_html(self):
        status, content_type, body = self.request("/")
        self.assertEqual(200, status)
        self.assertIn("text/html", content_type)
        self.assertIn(b"stream.mjpg", body)

    def test_snapshot_404_before_frames(self):
        status, _, _ = self.request("/snapshot.jpg")
        self.assertEqual(404, status)

    def test_snapshot_serves_latest_jpeg(self):
        jpeg = sample_jpeg()
        self.server.set_latest(jpeg)
        status, content_type, body = self.request("/snapshot.jpg")
        self.assertEqual(200, status)
        self.assertEqual("image/jpeg", content_type)
        self.assertEqual(jpeg, body)

    def test_unknown_path_404(self):
        status, _, _ = self.request("/other")
        self.assertEqual(404, status)

    def test_stream_serves_mjpeg_content_type_and_first_frame(self):
        jpeg = sample_jpeg()
        self.server.set_latest(jpeg)
        host, port = self.url()
        conn = http.client.HTTPConnection(host, port, timeout=5)
        conn.request("GET", "/stream.mjpg")
        response = conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertIn("multipart/x-mixed-replace", response.getheader("Content-Type"))
        self.assertIn(BOUNDARY, response.getheader("Content-Type"))
        chunk = b""
        deadline = time.time() + 5
        while jpeg not in chunk and time.time() < deadline:
            piece = response.read1(1024)
            if not piece:
                break
            chunk += piece
        conn.close()
        self.assertIn(b"--%s" % BOUNDARY.encode(), chunk)
        self.assertIn(jpeg, chunk)

    def test_set_latest_replaces_frame_and_is_fast_for_slow_client(self):
        conn = http.client.HTTPConnection(*self.url(), timeout=5)
        conn.request("GET", "/stream.mjpg")
        response = conn.getresponse()
        started = time.time()
        for _ in range(5):
            self.server.set_latest(sample_jpeg())
        elapsed = time.time() - started
        self.assertLess(elapsed, 1.0)
        conn.close()

    def test_set_latest_rejects_empty_or_non_bytes(self):
        with self.assertRaises(ValueError):
            self.server.set_latest(b"")
        with self.assertRaises(ValueError):
            self.server.set_latest("not bytes")


class DrawOverlayTest(unittest.TestCase):
    def test_draws_state_lines_without_resizing(self):
        image = np.full((120, 160, 3), 128, dtype=np.uint8)
        state = {"state": "SCANNING", "relative_yaw_deg": 45.0, "target_yaw_deg": 45.0,
                 "settled": True, "found_count": 2, "expected_count": 3,
                 "brightness": 128.0, "overexposed": 0.1, "sharpness": 50.0}
        result = draw_overlay(image, state)
        self.assertEqual(image.shape, result.shape)
        self.assertTrue(np.any(result != image))

    def test_tolerates_missing_quality_fields(self):
        image = np.full((60, 80, 3), 128, dtype=np.uint8)
        result = draw_overlay(image, {"state": "IDLE"})
        self.assertEqual(image.shape, result.shape)


if __name__ == "__main__":
    unittest.main()
