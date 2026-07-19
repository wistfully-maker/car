"""Quality-aware angular coverage bookkeeping for a QR search sweep."""

import math
from dataclasses import dataclass

from qr_item_search.yaw_control import normalize_angle


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("%s must be a finite number" % name)
    return float(value)


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("%s must be a positive integer" % name)
    return value


@dataclass
class Sector:
    frame_count: int = 0
    brightness_sum: float = 0.0
    overexposed_sum: float = 0.0
    sharpness_sum: float = 0.0
    decoded: bool = False


@dataclass(frozen=True)
class Interval:
    start: float
    end: float
    priority: int


class YawTracker:
    """Accumulates adjacent yaw deltas on a continuous angular axis."""

    def __init__(self):
        self._last_yaw = None
        self._accumulated = 0.0

    @property
    def accumulated(self):
        return self._accumulated

    def reset(self, yaw):
        self._last_yaw = _finite_number(yaw, "yaw")
        self._accumulated = 0.0
        return self._accumulated

    def update(self, yaw):
        yaw = _finite_number(yaw, "yaw")
        if self._last_yaw is None:
            return self.reset(yaw)
        delta = yaw - self._last_yaw
        self._accumulated += normalize_angle(delta)
        self._last_yaw = yaw
        return self._accumulated


class CoverageMap:
    """Divide a revolution into sectors and identify areas needing another pass."""

    def __init__(
        self,
        sector_count=24,
        minimum_frames=2,
        overexposed_threshold=0.25,
        sharpness_threshold=30.0,
        margin=math.radians(10.0),
    ):
        self.sector_count = _positive_integer(sector_count, "sector_count")
        self.minimum_frames = _positive_integer(minimum_frames, "minimum_frames")
        self.overexposed_threshold = _finite_number(
            overexposed_threshold, "overexposed_threshold"
        )
        self.sharpness_threshold = _finite_number(sharpness_threshold, "sharpness_threshold")
        self.margin = _finite_number(margin, "margin")
        if not 0.0 <= self.overexposed_threshold <= 1.0:
            raise ValueError("overexposed_threshold must be between zero and one")
        if self.sharpness_threshold < 0.0:
            raise ValueError("sharpness_threshold must be non-negative")
        if self.margin < 0.0:
            raise ValueError("margin must be non-negative")
        self.sectors = [Sector() for _ in range(self.sector_count)]

    @property
    def sector_width(self):
        return math.tau / self.sector_count

    def sector_index(self, yaw):
        yaw = _finite_number(yaw, "yaw")
        return int((yaw % math.tau) / self.sector_width) % self.sector_count

    def record(self, yaw, brightness, overexposed, sharpness, decoded):
        brightness = _finite_number(brightness, "brightness")
        overexposed = _finite_number(overexposed, "overexposed")
        sharpness = _finite_number(sharpness, "sharpness")
        if not 0.0 <= overexposed <= 1.0:
            raise ValueError("overexposed must be between zero and one")
        if not isinstance(decoded, bool):
            raise ValueError("decoded must be a bool")
        sector = self.sectors[self.sector_index(yaw)]
        sector.frame_count += 1
        sector.brightness_sum += brightness
        sector.overexposed_sum += overexposed
        sector.sharpness_sum += sharpness
        sector.decoded = sector.decoded or decoded

    def _quality_is_poor(self, sector):
        if not sector.frame_count:
            return False
        return (
            sector.overexposed_sum / sector.frame_count >= self.overexposed_threshold
            or sector.sharpness_sum / sector.frame_count < self.sharpness_threshold
        )

    def _candidate_priorities(self):
        candidates = {}
        for index, sector in enumerate(self.sectors):
            if sector.frame_count < self.minimum_frames:
                candidates[index] = 0
        for index, sector in enumerate(self.sectors):
            if self._quality_is_poor(sector):
                candidates.setdefault(index, 1)
        for index, sector in enumerate(self.sectors):
            if sector.decoded:
                candidates.setdefault((index - 1) % self.sector_count, 2)
                candidates.setdefault((index + 1) % self.sector_count, 2)
        candidates.setdefault(0, 3)
        return candidates

    def _candidate_runs(self, candidates):
        selected = [index in candidates for index in range(self.sector_count)]
        runs = []
        index = 0
        while index < self.sector_count:
            if not selected[index]:
                index += 1
                continue
            start = index
            while index < self.sector_count and selected[index]:
                index += 1
            runs.append([start, index])
        if len(runs) > 1 and selected[0] and selected[-1]:
            first = runs.pop(0)
            last = runs.pop(-1)
            runs.insert(0, [last[0], first[1] + self.sector_count])
        return runs

    @staticmethod
    def _merge_overlapping_intervals(intervals):
        merged = []
        for interval in sorted(intervals, key=lambda item: item.start):
            if not merged or interval.start > merged[-1].end:
                merged.append(interval)
                continue
            previous = merged[-1]
            merged[-1] = Interval(
                start=previous.start,
                end=max(previous.end, interval.end),
                priority=min(previous.priority, interval.priority),
            )
        return merged

    def rescan_intervals(self):
        candidates = self._candidate_priorities()
        intervals = []
        for start_sector, end_sector in self._candidate_runs(candidates):
            priority = min(
                candidates[index % self.sector_count]
                for index in range(start_sector, end_sector)
            )
            intervals.append(
                Interval(
                    start=start_sector * self.sector_width - self.margin,
                    end=end_sector * self.sector_width + self.margin,
                    priority=priority,
                )
            )
        intervals = self._merge_overlapping_intervals(intervals)
        return sorted(intervals, key=lambda interval: (interval.priority, interval.start))
