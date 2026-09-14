import sys
import unittest
from pathlib import Path

import numpy as np


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from line_follow_integration.camera_transform import center_crop_4_3, transform_line_frame


class CameraTransformTests(unittest.TestCase):
    def test_1020x720_center_crop_is_960x720(self):
        frame = np.zeros((720, 1020, 3), dtype=np.uint8)
        frame[:, :, 0] = np.arange(1020, dtype=np.uint16) % 256
        cropped = center_crop_4_3(frame)
        self.assertEqual((720, 960, 3), cropped.shape)
        np.testing.assert_array_equal(frame[:, 30:990], cropped)

    def test_transform_returns_640x480_without_mutating_input(self):
        frame = np.random.default_rng(7).integers(0, 256, (720, 1020, 3), dtype=np.uint8)
        original = frame.copy()
        output = transform_line_frame(frame, 640, 480, "center_4_3")
        self.assertEqual((480, 640, 3), output.shape)
        np.testing.assert_array_equal(original, frame)

    def test_invalid_shape_and_mode_fail_closed(self):
        with self.assertRaises(ValueError):
            center_crop_4_3(np.zeros((10, 10), dtype=np.uint8))
        with self.assertRaises(ValueError):
            transform_line_frame(np.zeros((10, 10, 3), dtype=np.uint8), 640, 480, "stretch")


if __name__ == "__main__":
    unittest.main()
