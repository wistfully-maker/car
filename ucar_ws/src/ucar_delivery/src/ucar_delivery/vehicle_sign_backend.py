"""Narrow vehicle YOLO + OCR composition contract.

The vehicle-specific modules stay outside this package.  This adapter makes
their required boundary explicit and ensures OCR is actually executed for the
YOLO crop instead of trusting an OCR-looking field supplied by the YOLO fake.
"""

import inspect
import math


class VehicleBackendError(RuntimeError):
    """Raised when a vehicle inference result violates the narrow contract."""


class VehicleSignBackend:
    """Compose one YOLO detector and one OCR detector behind ``detect``."""

    def __init__(self, yolo_detector, ocr_detector):
        yolo_detect = getattr(yolo_detector, "detect", None)
        ocr_detect = getattr(ocr_detector, "detect", None)
        yolo_predict = getattr(yolo_detector, "predict", None)
        ocr_predict = getattr(ocr_detector, "predict", None)

        if callable(yolo_detect) and callable(ocr_detect):
            self._mode = "detect"
            try:
                inspect.signature(yolo_detect).bind(object(), 0.0)
            except (TypeError, ValueError):
                raise VehicleBackendError(
                    "YoloDetector.detect must accept (image, timestamp)"
                )
            try:
                inspect.signature(ocr_detect).bind(
                    object(), [0, 0, 1, 1], 0.0
                )
            except (TypeError, ValueError):
                raise VehicleBackendError(
                    "OcrDetector.detect must accept (image, bbox, timestamp)"
                )
        elif callable(yolo_predict) and callable(ocr_predict):
            self._mode = "predict"
            try:
                inspect.signature(yolo_predict).bind(object())
            except (TypeError, ValueError):
                raise VehicleBackendError(
                    "YoloDetector.predict must accept (image)"
                )
            try:
                inspect.signature(ocr_predict).bind(object())
            except (TypeError, ValueError):
                raise VehicleBackendError(
                    "RapidOcrInfer.predict must accept (image)"
                )
        else:
            raise VehicleBackendError(
                "vehicle modules must expose either the detect pair or "
                "the predict pair"
            )
        self._yolo = yolo_detector
        self._ocr = ocr_detector

    @staticmethod
    def _parse_ocr_result(result):
        if isinstance(result, dict):
            text = result.get("text", result.get("ocr_text"))
            confidence = result.get(
                "confidence", result.get("ocr_confidence")
            )
        elif isinstance(result, (list, tuple)) and len(result) == 2:
            text, confidence = result
        elif isinstance(result, str):
            text, confidence = result, None
        else:
            raise VehicleBackendError("OCR result has unsupported shape")
        if not isinstance(text, str) or not text.strip():
            raise VehicleBackendError("OCR result must carry non-empty text")
        return text, confidence

    def detect(self, image, timestamp):
        if self._mode == "predict":
            return self._detect_with_vehicle_predict(image, timestamp)

        candidate = self._yolo.detect(image, timestamp)
        if candidate is None:
            return None
        if not isinstance(candidate, dict):
            raise VehicleBackendError("YOLO candidate must be an object")
        bbox = candidate.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            raise VehicleBackendError("YOLO candidate must carry bbox")
        ocr_result = self._ocr.detect(image, bbox, timestamp)
        text, confidence = self._parse_ocr_result(ocr_result)
        merged = dict(candidate)
        merged["ocr_text"] = text
        merged["ocr_confidence"] = confidence
        return merged

    @staticmethod
    def _image_bounds(image):
        shape = getattr(image, "shape", None)
        if not isinstance(shape, tuple) or len(shape) < 2:
            raise VehicleBackendError("vehicle image must expose height and width")
        height, width = int(shape[0]), int(shape[1])
        if height <= 0 or width <= 0:
            raise VehicleBackendError("vehicle image must be non-empty")
        return height, width

    @staticmethod
    def _parse_box(box, width, height):
        try:
            if len(box) < 5:
                raise ValueError
            x1, y1, x2, y2, confidence = (float(value) for value in box[:5])
        except (TypeError, ValueError, OverflowError):
            raise VehicleBackendError("YOLO predict box must carry x1 y1 x2 y2 confidence")
        if not all(math.isfinite(value) for value in (
                x1, y1, x2, y2, confidence)):
            raise VehicleBackendError("YOLO predict box must carry finite values")
        left = max(0, min(width, int(math.floor(x1))))
        top = max(0, min(height, int(math.floor(y1))))
        right = max(0, min(width, int(math.ceil(x2))))
        bottom = max(0, min(height, int(math.ceil(y2))))
        if right <= left or bottom <= top:
            raise VehicleBackendError("YOLO predict box is empty after clipping")
        return left, top, right, bottom, confidence

    def _detect_with_vehicle_predict(self, image, timestamp):
        boxes = self._yolo.predict(image)
        if boxes is None:
            return None
        try:
            boxes = list(boxes)
        except TypeError:
            raise VehicleBackendError("YOLO predict result must be iterable")
        if not boxes:
            return None

        height, width = self._image_bounds(image)
        parsed = [self._parse_box(box, width, height) for box in boxes]
        parsed.sort(key=lambda box: box[4], reverse=True)
        for left, top, right, bottom, confidence in parsed:
            crop = image[top:bottom, left:right]
            result = self._ocr.predict(crop)
            if result is None:
                continue
            try:
                text, ocr_confidence = self._parse_ocr_result(result)
            except VehicleBackendError:
                # RapidOCR returns an empty text when this YOLO crop contains
                # no readable workshop label. That is a normal non-detection;
                # another candidate may still be valid.
                if result == "" or (
                    isinstance(result, (list, tuple))
                    and len(result) == 2
                    and (result[0] is None or result[0] == "")
                ):
                    continue
                raise
            return {
                "timestamp": float(timestamp),
                "target": text,
                "confidence": confidence,
                "bbox": [float(left), float(top), float(right), float(bottom)],
                "ocr_text": text,
                "ocr_confidence": ocr_confidence,
            }
        return None
