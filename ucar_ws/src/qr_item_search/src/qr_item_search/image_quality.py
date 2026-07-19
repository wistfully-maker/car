"""Image validation, quality measurements, and QR decode variants."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameQuality:
    brightness: float
    overexposed: float
    sharpness: float


def _validate_image(image):
    if not isinstance(image, np.ndarray):
        raise ValueError("image must be a numpy array")
    if image.dtype != np.uint8:
        raise ValueError("image must use uint8 pixels")
    if image.size == 0:
        raise ValueError("image must not be empty")
    if image.ndim == 2:
        return image
    if image.ndim == 3 and image.shape[2] == 3:
        return image
    raise ValueError("image must be a 2D grayscale or 3-channel BGR array")


def _to_gray(image):
    return image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def measure_quality(image):
    """Return brightness, overexposure ratio, and Laplacian sharpness."""
    image = _validate_image(image)
    gray = _to_gray(image)
    return FrameQuality(
        brightness=float(gray.mean()),
        overexposed=float((gray >= 250).mean()),
        sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    )


def decode_variants(image, enhanced):
    """Yield the original frame and, optionally, three grayscale enhancements."""
    image = _validate_image(image)
    if type(enhanced) is not bool:
        raise ValueError("enhanced must be a bool")
    yield image
    if not enhanced:
        return
    gray = _to_gray(image)
    yield gray
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    yield clahe
    yield cv2.adaptiveThreshold(
        clahe,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        5,
    )
