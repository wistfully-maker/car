"""Phase D: lock the configurable workshop sign adapter contract.

The adapter normalizes target names, filters by confidence, requires OCR
confirmation, needs consecutive valid frames, rejects stale images, preserves
the capture pose and degrades safely when the vehicle inference backend fails.
The backend is a narrow interface; tests use a deterministic fake.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ucar_delivery import sign_detector
from ucar_delivery.sign_detector import SignDetector, normalize_target


DEFAULT_CONFIG = {
    "min_confidence": 0.6,
    "confirm_frames": 3,
    "ocr_confirm_frames": 1,
    "max_image_age": 1.0,
    "target_aliases": {},
}


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeBackend:
    """Deterministic backend: scripted sequence or a fixed auto candidate."""

    def __init__(self, sequence=None, auto=None):
        self._sequence = list(sequence) if sequence is not None else None
        self._auto = auto
        self.calls = 0
        self.failures = 0

    def detect(self, image, timestamp):
        self.calls += 1
        if self._auto is not None:
            item = self._auto
        elif self._sequence:
            item = self._sequence.pop(0)
        else:
            return None
        if isinstance(item, Exception):
            self.failures += 1
            raise item
        return item

    @property
    def exhausted(self):
        return bool(self._sequence) is not None and not self._sequence


def candidate(**overrides):
    payload = {
        "timestamp": 1000.0,
        "target": "食品加工车间",
        "confidence": 0.9,
        "bbox": [100, 80, 120, 60],
        "detection_yaw": 0.15,
        "ocr_text": "食品加工车间",
        "ocr_confidence": 0.88,
    }
    payload.update(overrides)
    return payload


_NO_POSE = object()


class Harness:
    def __init__(self, config=None, backend=None):
        self.clock = Clock()
        merged = dict(DEFAULT_CONFIG)
        merged.update(config or {})
        if backend is None:
            backend = FakeBackend(auto=candidate())
        self.backend = backend
        self.detector = SignDetector(self.clock, merged, self.backend)
        self.detector.set_target("食品加工车间")

    def frame(self, candidate=None, stamp=None, pose=_NO_POSE):
        if pose is _NO_POSE:
            robot_pose = {"x": 1.0, "y": 2.0, "yaw": 0.3}
        else:
            robot_pose = pose
        image_stamp = self.clock() if stamp is None else stamp
        if candidate is not None:
            return self.detector.on_candidate(candidate, image_stamp, robot_pose)
        return self.detector.process_frame(None, image_stamp, robot_pose)


class NormalizationTests(unittest.TestCase):
    def test_normalize_strips_whitespace_and_case(self):
        self.assertEqual("食品加工车间", normalize_target(" 食品加工车间 "))
        self.assertEqual("foodworkshop", normalize_target(" Food Workshop "))
        self.assertEqual("毛巾车间", normalize_target("毛 巾 车 间"))
        self.assertEqual("", normalize_target("   "))
        self.assertIsNone(normalize_target(None))

    def test_aliases_map_to_canonical_target(self):
        h = Harness(config={"target_aliases": {"food_ws": "食品加工车间"}})
        for _ in range(3):
            result = h.frame(candidate(target="food_ws", ocr_text="食品加工车间"))
        self.assertIsNotNone(result)
        self.assertEqual("食品加工车间", result["target_name"])

    def test_aliases_apply_to_ocr_text_as_well_as_candidate_target(self):
        alias = "电子产品加工车间"
        canonical = "电子产品生产车间"
        h = Harness(config={"target_aliases": {alias: canonical}})
        h.detector.set_target(canonical)
        for _ in range(3):
            result = h.frame(candidate(target=alias, ocr_text=alias))
        self.assertIsNotNone(result)
        self.assertTrue(result["ocr_confirmed"])


class ConfidenceFilterTests(unittest.TestCase):
    def test_below_min_confidence_rejected(self):
        h = Harness(config={"min_confidence": 0.8})
        for _ in range(4):
            result = h.frame(candidate(confidence=0.79))
        self.assertIsNone(result)

    def test_at_min_confidence_accepted(self):
        h = Harness(config={"min_confidence": 0.6})
        for _ in range(3):
            result = h.frame(candidate(confidence=0.6, ocr_text="食品加工车间"))
        self.assertIsNotNone(result)

    def test_missing_confidence_rejected(self):
        h = Harness()
        for _ in range(4):
            result = h.frame(candidate(confidence=None))
        self.assertIsNone(result)

    def test_missing_bbox_rejected(self):
        # vehicle 后端合同：候选必须带 bbox（staging 估计依赖 bearing）
        h = Harness()
        for _ in range(4):
            result = h.frame(candidate(bbox=None))
        self.assertIsNone(result)
        self.assertIsNotNone(h.detector.last_error)

    def test_malformed_bbox_rejected(self):
        h = Harness()
        for bad in ([1, 2, 3], "x1,y1,x2,y2", [1, 2, 3, "y"]):
            result = h.frame(candidate(bbox=bad))
            self.assertIsNone(result)
            self.assertIsNotNone(h.detector.last_error)


class ConsecutiveFrameTests(unittest.TestCase):
    def test_confirmation_requires_consecutive_frames(self):
        h = Harness()
        self.assertIsNone(h.frame())
        self.assertIsNone(h.frame())
        result = h.frame()
        self.assertIsNotNone(result)
        self.assertEqual(3, h.backend.calls)

    def test_one_bad_frame_resets_streak(self):
        h = Harness()
        h.frame()
        h.frame(candidate(target="日用品加工车间", ocr_text="日用品加工车间"))
        h.frame()
        result = h.frame()
        self.assertIsNone(result)
        result = h.frame()
        self.assertIsNotNone(result)

    def test_confirm_frames_configured(self):
        h = Harness(config={"confirm_frames": 5})
        for _ in range(4):
            self.assertIsNone(h.frame())
        result = h.frame()
        self.assertIsNotNone(result)

    def test_target_change_resets_streak(self):
        h = Harness()
        h.frame()
        h.frame()
        h.detector.set_target("日用品加工车间")
        result = h.frame(candidate(target="日用品加工车间", ocr_text="日用品加工车间"))
        self.assertIsNone(result)


class OcrConfirmationTests(unittest.TestCase):
    def test_ocr_mismatch_does_not_confirm(self):
        h = Harness()
        for _ in range(5):
            result = h.frame(candidate(ocr_text="电子产品生产车间"))
        self.assertIsNone(result)

    def test_ocr_text_normalized_before_compare(self):
        h = Harness()
        for _ in range(3):
            result = h.frame(candidate(ocr_text=" 食品加工 车间 "))
        self.assertIsNotNone(result)

    def test_missing_ocr_text_never_confirms(self):
        h = Harness()
        for _ in range(5):
            result = h.frame(candidate(ocr_text=None))
        self.assertIsNone(result)

    def test_ocr_confirm_frames_requires_two_ocr_frames(self):
        h = Harness(config={"ocr_confirm_frames": 2})
        h.frame(candidate(ocr_text="食品加工车间"))
        h.frame(candidate(ocr_text="其他文字"))
        h.frame(candidate(ocr_text="其他文字"))
        result = h.frame(candidate(ocr_text="其他文字"))
        self.assertIsNone(result)
        h.frame(candidate(ocr_text="食品加工车间"))
        result = h.frame(candidate(ocr_text="食品加工车间"))
        self.assertIsNotNone(result)

    def test_confirmed_sign_marks_ocr_confirmed(self):
        h = Harness()
        for _ in range(3):
            result = h.frame()
        self.assertTrue(result["ocr_confirmed"])


class StaleImageTests(unittest.TestCase):
    def test_stale_image_rejected(self):
        h = Harness()
        h.clock.advance(2.0)
        result = h.frame(stamp=h.clock() - 1.5)
        self.assertIsNone(result)

    def test_fresh_image_accepted(self):
        h = Harness()
        for _ in range(3):
            result = h.frame(stamp=h.clock())
        self.assertIsNotNone(result)


class CapturePoseTests(unittest.TestCase):
    def test_confirmed_sign_preserves_capture_data(self):
        h = Harness()
        for _ in range(3):
            result = h.frame(
                candidate(confidence=0.93, detection_yaw=0.42, bbox=[1, 2, 3, 4]),
                pose={"x": 5.0, "y": 6.0, "yaw": 1.1},
            )
        self.assertIsNotNone(result)
        self.assertEqual(0.93, result["confidence"])
        self.assertEqual(0.42, result["detection_yaw"])
        self.assertEqual([1, 2, 3, 4], result["bbox"])
        self.assertEqual({"x": 5.0, "y": 6.0, "yaw": 1.1}, result["observed_pose"])
        self.assertEqual("食品加工车间", result["target_name"])
        self.assertIn("timestamp", result)

    def test_capture_pose_may_be_absent(self):
        h = Harness()
        for _ in range(3):
            result = h.frame(pose=None)
        self.assertIsNotNone(result)
        self.assertIsNone(result["observed_pose"])


class BackendFailureTests(unittest.TestCase):
    def test_backend_exception_recorded_and_safe(self):
        h = Harness(backend=FakeBackend([RuntimeError("backend crashed")]))
        result = h.frame()
        self.assertIsNone(result)
        self.assertEqual(1, h.backend.failures)
        self.assertIsNotNone(h.detector.last_error)
        result2 = h.frame()
        self.assertIsNone(result2)

    def test_recovery_after_backend_failure(self):
        h = Harness(
            backend=FakeBackend([RuntimeError("boom"), candidate(), candidate(), candidate()])
        )
        h.frame()
        h.frame()
        h.frame()
        result = h.frame()
        self.assertIsNotNone(result)

    def test_invalid_candidate_rejected(self):
        h = Harness()
        for bad in ("text", 42, []):
            result = h.detector.on_candidate(
                bad, h.clock(), {"x": 0.0, "y": 0.0, "yaw": 0.0}
            )
            self.assertIsNone(result)
            self.assertIsNotNone(h.detector.last_error)

    def test_no_detection_is_normal_not_an_error(self):
        h = Harness()
        result = h.frame()
        self.assertIsNone(result)
        self.assertIsNone(h.detector.last_error)

    def test_reset_clears_error_and_streak(self):
        h = Harness(backend=FakeBackend([RuntimeError("boom")]))
        h.frame()
        self.assertIsNotNone(h.detector.last_error)
        h.detector.reset()
        self.assertIsNone(h.detector.last_error)
        h.backend._sequence = [candidate(), candidate(), candidate()]
        for _ in range(3):
            result = h.frame()
        self.assertIsNotNone(result)


if __name__ == "__main__":
    unittest.main()
