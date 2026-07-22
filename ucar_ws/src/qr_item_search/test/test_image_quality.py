import math
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

import qr_item_search.image_quality as image_quality
from qr_item_search.image_quality import decode_variants, measure_quality


class MeasureQualityTest(unittest.TestCase):
    def test_measures_black_and_white_frames(self):
        black = measure_quality(np.zeros((12, 12), dtype=np.uint8))
        white = measure_quality(np.full((12, 12), 255, dtype=np.uint8))

        self.assertEqual(0.0, black.brightness)
        self.assertEqual(0.0, black.overexposed)
        self.assertEqual(255.0, white.brightness)
        self.assertEqual(1.0, white.overexposed)

    def test_gray_and_bgr_inputs_have_consistent_basic_measurements(self):
        gray = np.full((12, 12), 100, dtype=np.uint8)
        bgr = np.dstack((gray, gray, gray))

        gray_quality = measure_quality(gray)
        bgr_quality = measure_quality(bgr)

        self.assertEqual(gray_quality.brightness, bgr_quality.brightness)
        self.assertEqual(gray_quality.overexposed, bgr_quality.overexposed)
        self.assertTrue(math.isfinite(gray_quality.sharpness))
        self.assertTrue(math.isfinite(bgr_quality.sharpness))

    def test_measures_threshold_pixels_and_exact_laplacian_variance(self):
        pixels = np.array([[249, 250, 255]], dtype=np.uint8)
        quality = measure_quality(pixels)
        edge = np.array([[0, 0, 255], [0, 0, 255], [0, 0, 255]], dtype=np.uint8)

        self.assertEqual(float(pixels.mean()), quality.brightness)
        self.assertEqual(2.0 / 3.0, quality.overexposed)
        self.assertEqual(
            float(cv2.Laplacian(edge, cv2.CV_64F).var()),
            measure_quality(edge).sharpness,
        )

    def test_rejects_invalid_input(self):
        invalid = (None, [], np.array([], dtype=np.uint8),
                   np.zeros((3, 3, 4), dtype=np.uint8),
                   np.zeros((3,), dtype=np.uint8),
                   np.zeros((3, 3), dtype=np.float32))
        for image in invalid:
            with self.subTest(image_type=type(image)):
                with self.assertRaises(ValueError):
                    measure_quality(image)


class DecodeVariantsTest(unittest.TestCase):
    def test_enhanced_variants_are_ordered_and_preserve_original_object(self):
        image = np.full((40, 40, 3), 100, dtype=np.uint8)
        variants = list(decode_variants(image, True))

        self.assertEqual(4, len(variants))
        self.assertIs(image, variants[0])
        self.assertEqual((40, 40), variants[1].shape)
        self.assertEqual((40, 40), variants[2].shape)
        self.assertEqual((40, 40), variants[3].shape)

    def test_enhanced_variants_use_required_opencv_pipeline(self):
        image = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        expected_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        clahe_result = np.full((1, 2), 11, dtype=np.uint8)
        threshold_result = np.full((1, 2), 22, dtype=np.uint8)
        clahe = Mock()
        clahe.apply.return_value = clahe_result

        with patch.object(image_quality.cv2, "createCLAHE", return_value=clahe) as create_clahe, \
             patch.object(image_quality.cv2, "adaptiveThreshold", return_value=threshold_result) as threshold:
            variants = list(decode_variants(image, True))

        np.testing.assert_array_equal(expected_gray, variants[1])
        self.assertIs(clahe_result, variants[2])
        self.assertIs(threshold_result, variants[3])
        create_clahe.assert_called_once_with(clipLimit=2.0, tileGridSize=(8, 8))
        clahe.apply.assert_called_once()
        np.testing.assert_array_equal(expected_gray, clahe.apply.call_args[0][0])
        threshold.assert_called_once_with(
            clahe_result,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            5,
        )

    def test_disabled_enhancement_yields_only_original_object(self):
        image = np.zeros((8, 8), dtype=np.uint8)
        variants = list(decode_variants(image, False))
        self.assertEqual([image], variants)
        self.assertIs(image, variants[0])

    def test_rejects_non_boolean_enhanced_and_invalid_image(self):
        image = np.zeros((8, 8), dtype=np.uint8)
        for enhanced in (0, 1, None):
            with self.subTest(enhanced=enhanced):
                with self.assertRaises(ValueError):
                    list(decode_variants(image, enhanced))
        with self.assertRaises(ValueError):
            list(decode_variants(np.zeros((8, 8), dtype=np.float32), True))


if __name__ == "__main__":
    unittest.main()
