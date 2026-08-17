"""Three-sided white parking frame detector (ROS-free, pure numpy).

Finds the white parking frame on a dark (blue) floor: a near horizontal line,
a front horizontal line, and left/right vertical side lines. Works without
OpenCV so it can be unit-tested anywhere numpy exists.

Single-frame geometry (`WhiteFrameDetector.detect`) is wrapped by the
multi-frame confirmation gate (`FrameConfirmGate`) which needs consecutive
acquire frames and tolerates a bounded loss grace period; a loss inside the
near zone is flagged immediately for the fail-closed parking controller.
"""

from dataclasses import dataclass

import numpy as np

DEFAULT_CONFIG = {
    "gray_threshold": 180,
    "min_line_pixels": 60,
    "min_col_pixels": 60,
    "min_boundaries": 2,
    "roi_x_min": 0,
    "roi_x_max": 640,
    "roi_y_min": 0,
    "roi_y_max": 480,
    # 横线行跨度超过该值视为大片白色（杂物/过曝），一致性扣分
    "max_line_thickness": 20,
    "min_geometry_confidence": 0.65,
    # 开闭运算迭代次数（0 = 关闭形态学；细线在开运算下会损失边缘像素，
    # 因此默认关闭，需要去噪时显式开启）
    "morph_iterations": 0,
    # 可选 HSV 白色约束：低饱和度 + 高明度（白线），滤掉彩色杂物
    "use_hsv": False,
    "hsv_sat_max": 60.0,
    "hsv_value_min": 180.0,
    # 透视一致性：远端宽度相对近端宽度的允许放大比例/像素容差
    "perspective_tolerance": 0.2,
    "perspective_tolerance_px": 15.0,
}


@dataclass
class FrameObservation:
    timestamp: float = 0.0
    frame_detected: bool = False
    confidence: float = 0.0
    near_center_x: float = None
    far_center_x: float = None
    left_boundary: float = None
    right_boundary: float = None
    front_boundary_y: float = None
    visible_boundary_count: int = 0
    yaw_error: float = 0.0
    lateral_error: float = 0.0
    # 横线白色 run 宽度与行跨度（透视/厚度一致性用）
    near_width: float = None
    far_width: float = None
    near_thickness: int = 0
    far_thickness: int = 0
    # 多帧滞回确认后的状态（FrameConfirmGate 填充）
    confirmed: bool = False
    near_zone: bool = False
    near_zone_loss: bool = False


def _segments(profile, minimum, gap=2):
    """Group profile indices above `minimum` into contiguous runs."""
    indices = np.where(profile >= minimum)[0]
    if indices.size == 0:
        return []
    groups = []
    start = previous = int(indices[0])
    for index in indices[1:]:
        if index - previous > gap:
            groups.append((start, previous))
            start = int(index)
        previous = int(index)
    groups.append((start, previous))
    return groups


def _line_center(white, start, end, minimum):
    """Center x of the longest white run inside the row band.

    Side lines also cross the band, so the longest contiguous run (the
    horizontal line itself) is used instead of a raw mean.
    """
    block_columns = white[start:end + 1, :].any(axis=0).astype(np.uint8)
    runs = [
        (seg_start, seg_end)
        for seg_start, seg_end in _segments(block_columns, 1)
        if seg_end - seg_start + 1 >= minimum
    ]
    if not runs:
        return None
    longest = max(runs, key=lambda run: run[1] - run[0])
    return (longest[0] + longest[1]) / 2.0


def _line_run_width(white, start, end):
    """Width of the widest white run inside the row band (pixels)."""
    block_columns = white[start:end + 1, :].any(axis=0).astype(np.uint8)
    runs = _segments(block_columns, 1)
    if not runs:
        return None
    longest = max(runs, key=lambda run: run[1] - run[0])
    return longest[1] - longest[0] + 1


def _side_assign(col_segments, image_center_x):
    """Assign vertical segments to left/right boundaries.

    With a single vertical segment its image side decides whether it is the
    left or the right boundary; a frame seen from inside cannot legitimately
    produce both boundaries from one segment.
    """
    left_line = None
    right_line = None
    if not col_segments:
        return left_line, right_line
    if len(col_segments) >= 2:
        left_line = col_segments[0]
        right_line = col_segments[-1]
        return left_line, right_line
    only = col_segments[0]
    center = (only[0] + only[1]) / 2.0
    if center < image_center_x:
        left_line = only
    else:
        right_line = only
    return left_line, right_line


def _erode(mask):
    out = mask.copy()
    out[1:, :] &= mask[:-1, :]
    out[:-1, :] &= mask[1:, :]
    out[:, 1:] &= mask[:, :-1]
    out[:, :-1] &= mask[:, 1:]
    return out


def _dilate(mask):
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _morph_open(mask, iterations):
    for _ in range(iterations):
        mask = _erode(mask)
    for _ in range(iterations):
        mask = _dilate(mask)
    return mask


def _rgb_to_hsv_mask(image, sat_max, value_min):
    """White mask from an RGB image: low saturation and high value.

    Pure numpy conversion; the input image is never modified.
    """
    rgb = image.astype(np.float32) / 255.0
    maximum = rgb.max(axis=2)
    minimum = rgb.min(axis=2)
    delta = maximum - minimum
    value = maximum
    with np.errstate(divide="ignore", invalid="ignore"):
        saturation = np.where(maximum > 0.0, delta / np.maximum(maximum, 1e-9), 0.0)
    sat_ok = saturation * 255.0 <= sat_max
    value_ok = value * 255.0 >= value_min
    return sat_ok & value_ok


class WhiteFrameDetector:
    def __init__(self, config=None):
        merged = dict(DEFAULT_CONFIG)
        merged.update(config or {})
        self._config = merged
        self._threshold = int(merged["gray_threshold"])
        self._min_line_pixels = int(merged["min_line_pixels"])
        self._min_col_pixels = int(merged["min_col_pixels"])
        self._min_boundaries = int(merged["min_boundaries"])
        self._roi_x_min = int(merged["roi_x_min"])
        self._roi_x_max = int(merged["roi_x_max"])
        self._roi_y_min = int(merged["roi_y_min"])
        self._roi_y_max = int(merged["roi_y_max"])
        self._max_line_thickness = int(merged["max_line_thickness"])
        self._min_confidence = float(merged["min_geometry_confidence"])
        self._morph_iterations = int(merged["morph_iterations"])
        self._use_hsv = bool(merged["use_hsv"])

    def _white_mask(self, image):
        """Boolean white mask over the full image, never mutating input."""
        if self._use_hsv and image.ndim == 3:
            return _rgb_to_hsv_mask(
                image, float(self._config["hsv_sat_max"]),
                float(self._config["hsv_value_min"]),
            )
        gray = np.asarray(image, dtype=np.uint8)
        if gray.ndim != 2 or gray.size == 0:
            return None
        return gray > self._threshold

    def detect(self, image, timestamp):
        """Detect the parking frame in one image (single-frame geometry).

        Returns a FrameObservation with full-image coordinates. The input
        image is never modified. `confirmed`/`near_zone`/`near_zone_loss`
        fields are filled by FrameConfirmGate, not here.
        """
        white = self._white_mask(image)
        if white is None:
            return FrameObservation(timestamp=timestamp)
        if self._morph_iterations > 0:
            white = _morph_open(white, self._morph_iterations)

        roi = (
            slice(
                max(0, self._roi_y_min),
                min(white.shape[0], self._roi_y_max),
            ),
            slice(
                max(0, self._roi_x_min),
                min(white.shape[1], self._roi_x_max),
            ),
        )
        if roi[0].start >= roi[0].stop or roi[1].start >= roi[1].stop:
            return FrameObservation(timestamp=timestamp)
        white_roi = white[roi]
        roi_y0 = roi[0].start
        roi_x0 = roi[1].start

        row_profile = white_roi.sum(axis=1)
        col_profile = white_roi.sum(axis=0)
        image_center_x = white_roi.shape[1] / 2.0 + roi_x0

        row_segments = _segments(row_profile, self._min_line_pixels)
        col_segments = _segments(col_profile, self._min_col_pixels)

        near_line = None
        far_line = None
        if row_segments:
            near_line = (
                row_segments[-1][0] + roi_y0,
                row_segments[-1][1] + roi_y0,
            )
            upper = [segment for segment in row_segments[:-1]
                     if segment[1] < row_segments[-1][0]]
            if upper:
                far_line = (
                    upper[-1][0] + roi_y0,
                    upper[-1][1] + roi_y0,
                )

        left_line, right_line = _side_assign(
            [
                (segment[0] + roi_x0, segment[1] + roi_x0)
                for segment in col_segments
            ],
            image_center_x,
        )

        visible = sum(
            segment is not None
            for segment in (near_line, far_line, left_line, right_line)
        )
        observation = FrameObservation(
            timestamp=timestamp,
            frame_detected=False,
            confidence=0.0,
            visible_boundary_count=visible,
        )
        if visible < self._min_boundaries:
            return observation

        if near_line is not None:
            near_center = _line_center(
                white_roi,
                near_line[0] - roi_y0,
                near_line[1] - roi_y0,
                self._min_line_pixels,
            )
            if near_center is not None:
                observation.near_center_x = near_center + roi_x0
            observation.lateral_error = (
                (observation.near_center_x or 0.0) - image_center_x
            )
            observation.near_width = _line_run_width(
                white_roi,
                near_line[0] - roi_y0,
                near_line[1] - roi_y0,
            )
            observation.near_thickness = near_line[1] - near_line[0] + 1
        if far_line is not None:
            far_center = _line_center(
                white_roi,
                far_line[0] - roi_y0,
                far_line[1] - roi_y0,
                self._min_line_pixels,
            )
            if far_center is not None:
                observation.far_center_x = far_center + roi_x0
            observation.front_boundary_y = (far_line[0] + far_line[1]) / 2.0
            observation.far_width = _line_run_width(
                white_roi,
                far_line[0] - roi_y0,
                far_line[1] - roi_y0,
            )
            observation.far_thickness = far_line[1] - far_line[0] + 1
        if left_line is not None:
            observation.left_boundary = (left_line[0] + left_line[1]) / 2.0
        if right_line is not None:
            observation.right_boundary = (right_line[0] + right_line[1]) / 2.0
        if (
            observation.far_center_x is not None
            and observation.near_center_x is not None
        ):
            observation.yaw_error = (
                observation.far_center_x - observation.near_center_x
            )

        observation.confidence = self._geometry_confidence(observation)
        observation.frame_detected = (
            observation.confidence >= self._min_confidence
        )
        return observation

    def _geometry_confidence(self, observation):
        """Deterministic geometric confidence in [0, 1].

        Base score comes from the visible boundary count; consistency
        checks (far line above near line, side lines enclosing the near
        line, perspective narrowing, line thickness) add or subtract fixed
        amounts. Inconsistent combinations stay below the detection
        threshold by design.
        """
        visible = observation.visible_boundary_count
        near = observation.near_center_x
        far = observation.far_center_x
        left = observation.left_boundary
        right = observation.right_boundary
        center_x = 320.0
        if near is not None:
            center_x = near - observation.lateral_error
        score = 0.2 * visible

        if near is not None and far is not None:
            # 两条横线：远线在上方（检测器已按 y 排序），加近宽远窄的透视分
            score += 0.2
            near_width = observation.near_width
            far_width = observation.far_width
            if near_width is not None and far_width is not None:
                tolerance = (
                    float(self._config["perspective_tolerance"]) * near_width
                    + float(self._config["perspective_tolerance_px"])
                )
                if far_width <= near_width + tolerance:
                    score += 0.1
                else:
                    # 透视违反是强信号：远端显著宽于近端
                    score -= 0.6
        if left is not None and right is not None:
            if not (left < center_x < right):
                # 两条边线必须分居图像中心两侧（同侧组合不可信）
                score -= 0.5
            if near is not None and left < near < right:
                score += 0.1
            elif near is not None:
                score -= 0.4
        elif (left is not None or right is not None) and near is not None:
            only = left if left is not None else right
            if only < near - 10 or only > near + 10:
                score += 0.05
        if observation.front_boundary_y is not None and near is not None:
            score += 0.05

        # 大片白色（线过厚 = 杂物/过曝）扣分
        if observation.near_thickness > self._max_line_thickness:
            score -= 0.55
        if observation.far_thickness > self._max_line_thickness:
            score -= 0.3
        return max(0.0, min(1.0, score))


class FrameConfirmGate:
    """Multi-frame confirmation and bounded-loss hysteresis (ROS-free).

    `confirmed` becomes true only after `acquire_confirm_frames` consecutive
    detections and stays true through up to `lost_grace_frames` lost frames.
    A loss while the front line is inside the near zone is flagged
    immediately on `near_zone_loss` so the parking controller can fail
    closed instead of blind-approaching.
    """

    DEFAULT_CONFIG = {
        "acquire_confirm_frames": 3,
        "lost_grace_frames": 2,
        "near_zone_front_y": 380.0,
    }

    def __init__(self, config=None):
        merged = dict(self.DEFAULT_CONFIG)
        merged.update(config or {})
        self._acquire = max(1, int(merged["acquire_confirm_frames"]))
        self._lost_grace = max(0, int(merged["lost_grace_frames"]))
        self._near_zone_front_y = float(merged["near_zone_front_y"])
        self._detected_streak = 0
        self._lost_streak = 0
        self._confirmed = False
        self._last_confirmed_near_zone = False

    def reset(self):
        self._detected_streak = 0
        self._lost_streak = 0
        self._confirmed = False
        self._last_confirmed_near_zone = False

    def update(self, observation, timestamp=None):
        """Feed one single-frame observation; returns the confirmed copy."""
        detected = bool(observation.frame_detected)
        near_zone = (
            observation.front_boundary_y is not None
            and observation.front_boundary_y >= self._near_zone_front_y
        )
        near_zone_loss = False
        if detected:
            self._detected_streak += 1
            self._lost_streak = 0
            if self._detected_streak >= self._acquire:
                self._confirmed = True
            if self._confirmed:
                # A fresh confirmed frame is authoritative: a far-zone
                # reacquisition clears any prior near-zone memory.
                self._last_confirmed_near_zone = near_zone
        else:
            self._detected_streak = 0
            if self._confirmed:
                # The current empty frame has no usable front_boundary_y.
                # Preserve the last confirmed geometry through the grace
                # window so a near-zone disappearance is still fail-fast.
                near_zone_loss = self._last_confirmed_near_zone
                self._lost_streak += 1
                if self._lost_streak > self._lost_grace:
                    self._confirmed = False
                    self._last_confirmed_near_zone = False
            near_zone = self._confirmed and self._last_confirmed_near_zone

        out = FrameObservation(
            timestamp=observation.timestamp,
            frame_detected=detected,
            confidence=observation.confidence,
            near_center_x=observation.near_center_x,
            far_center_x=observation.far_center_x,
            left_boundary=observation.left_boundary,
            right_boundary=observation.right_boundary,
            front_boundary_y=observation.front_boundary_y,
            visible_boundary_count=observation.visible_boundary_count,
            yaw_error=observation.yaw_error,
            lateral_error=observation.lateral_error,
            confirmed=self._confirmed,
            near_zone=near_zone,
            near_zone_loss=near_zone_loss,
        )
        return out
