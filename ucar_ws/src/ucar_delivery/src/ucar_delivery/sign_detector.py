"""Configurable workshop sign adapter (ROS-free).

Wraps a narrow vehicle-inference backend (YOLO + OCR) with deterministic
logic: target normalization and aliases, confidence filtering, OCR
confirmation, consecutive-frame confirmation, stale-image rejection, capture
pose preservation and safe degradation on backend failure.
"""

import math

WHITESPACE = " \t\r\n\v\f\u00a0\u3000"


def normalize_target(value):
    """Normalize a target name for comparison (whitespace + case)."""
    if not isinstance(value, str):
        return None
    return "".join(ch for ch in value if ch not in WHITESPACE).lower()


class SignDetector:
    def __init__(self, clock, config, backend):
        self._clock = clock
        self._backend = backend
        self._min_confidence = float(config.get("min_confidence", 0.6))
        self._confirm_frames = int(config.get("confirm_frames", 3))
        self._ocr_confirm_frames = int(config.get("ocr_confirm_frames", 1))
        self._max_image_age = float(config.get("max_image_age", 1.0))
        self._aliases = dict(config.get("target_aliases", {}))
        self._target = None
        self._target_normalized = None
        self._streak = 0
        self._ocr_streak = 0
        self.last_error = None

    def set_target(self, target_name):
        """Switch the workshop sign to search for; resets all streaks."""
        self._target = target_name
        self._target_normalized = normalize_target(target_name)
        self._streak = 0
        self._ocr_streak = 0

    def reset(self):
        """Clear streaks and recorded backend errors."""
        self._streak = 0
        self._ocr_streak = 0
        self.last_error = None

    def _canonical(self, raw_target):
        text = normalize_target(raw_target)
        if text is None:
            return None
        for alias, canonical in self._aliases.items():
            if text == normalize_target(alias):
                return canonical
        return raw_target.strip()

    def process_frame(self, image, image_stamp, robot_pose):
        """Run one image through the backend and update confirmation state.

        Returns a confirmed-sign dict or None. Stale images are rejected
        before touching the backend.
        """
        if self._target_normalized is None:
            return None
        now = self._clock()
        if now - image_stamp > self._max_image_age:
            return None
        try:
            candidate = self._backend.detect(image, image_stamp)
        except Exception as exc:  # backend must never take the mission down
            self.last_error = str(exc)
            return None
        self.last_error = None
        return self.on_candidate(candidate, image_stamp, robot_pose)

    def on_candidate(self, candidate, image_stamp, robot_pose):
        """Validate one candidate and update the confirmation streak.

        Public so tests can feed raw backend output deterministically.
        """
        if self._target_normalized is None:
            return None
        if not isinstance(candidate, dict):
            self.last_error = "invalid candidate: %r" % (candidate,)
            return None
        try:
            timestamp = float(candidate.get("timestamp"))
            confidence = float(candidate.get("confidence"))
        except (TypeError, ValueError):
            self.last_error = "candidate must carry finite numeric fields"
            return None
        if not (math.isfinite(timestamp) and math.isfinite(confidence)):
            self.last_error = "candidate fields must be finite"
            return None
        raw_target = candidate.get("target")
        canonical = self._canonical(raw_target)
        if canonical is None:
            self.last_error = "candidate target missing"
            return None
        if confidence < self._min_confidence:
            return None
        bbox = candidate.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            # vehicle 后端合同：候选必须带 bbox（staging 估计依赖由 bbox
            # 推导的 bearing/half-width），缺失即不可信。
            self._streak = 0
            self._ocr_streak = 0
            self.last_error = "candidate bbox must be a 4-element sequence"
            return None
        if not all(
            isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value)
            for value in bbox
        ):
            self._streak = 0
            self._ocr_streak = 0
            self.last_error = "candidate bbox must carry finite numbers"
            return None
        if normalize_target(canonical) != self._target_normalized:
            self._streak = 0
            self._ocr_streak = 0
            return None

        ocr_text = candidate.get("ocr_text")
        ocr_canonical = self._canonical(ocr_text)
        ocr_matched = (
            ocr_canonical is not None
            and normalize_target(ocr_canonical) == self._target_normalized
        )
        self._streak += 1
        if ocr_matched:
            self._ocr_streak += 1
        else:
            self._ocr_streak = 0
        if self._streak < self._confirm_frames:
            return None
        if self._ocr_streak < self._ocr_confirm_frames:
            return None
        return {
            "timestamp": timestamp,
            "target_name": self._target,
            "confidence": confidence,
            "bbox": candidate.get("bbox"),
            "detection_yaw": candidate.get("detection_yaw"),
            "ocr_text": ocr_text,
            "ocr_confidence": candidate.get("ocr_confidence"),
            "ocr_confirmed": ocr_matched,
            "observed_pose": dict(robot_pose) if robot_pose else None,
        }
