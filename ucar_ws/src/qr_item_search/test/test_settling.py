import math
import unittest

from qr_item_search.settling import SettlingDetector


class SettlingDetectorTest(unittest.TestCase):
    def test_high_speed_resets_timer(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.20)
        self.assertFalse(detector.update(0.0, 0.0))
        self.assertFalse(detector.update(0.01, 0.10))
        self.assertFalse(detector.update(0.10, 0.15))
        self.assertFalse(detector.update(0.01, 0.25))
        self.assertFalse(detector.update(0.01, 0.35))

    def test_continuous_low_speed_for_required_duration_settles(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.20)
        self.assertFalse(detector.update(0.0, 0.0))
        self.assertFalse(detector.update(0.01, 0.10))
        self.assertTrue(detector.update(0.01, 0.20))

    def test_zero_required_duration_settles_immediately(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.0)
        self.assertTrue(detector.update(0.0, 1.0))

    def test_threshold_is_strictly_greater(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.0)
        self.assertTrue(detector.update(0.03, 1.0))
        self.assertFalse(detector.update(0.04, 2.0))

    def test_reset_restarts_timing(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.20)
        self.assertFalse(detector.update(0.0, 0.0))
        self.assertTrue(detector.update(0.0, 0.20))
        detector.reset()
        self.assertFalse(detector.update(0.0, 1.0))

    def test_time_moving_backwards_raises(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.20)
        detector.update(0.0, 1.0)
        with self.assertRaises(ValueError):
            detector.update(0.0, 0.9)

    def test_non_finite_inputs_raise(self):
        detector = SettlingDetector(speed_threshold=0.03, required_duration=0.20)
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    detector.update(value, 0.0)
                with self.assertRaises(ValueError):
                    detector.update(0.0, value)

    def test_rejects_invalid_construction(self):
        with self.assertRaises(ValueError):
            SettlingDetector(speed_threshold=-1.0, required_duration=0.2)
        with self.assertRaises(ValueError):
            SettlingDetector(speed_threshold=0.03, required_duration=-0.1)


if __name__ == "__main__":
    unittest.main()
