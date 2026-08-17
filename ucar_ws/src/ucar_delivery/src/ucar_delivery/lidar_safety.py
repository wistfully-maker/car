"""Front-sector lidar safety helper (ROS-free).

Computes the front distance from a LaserScan-like range array using a
configurable angular sector around the forward axis, strict filtering of
NaN/infinity/zero/out-of-range values, and a robust median statistic.
"""

import math

import numpy as np


class LidarSafety:
    def __init__(self, config):
        self._enabled = bool(config.get("enabled", False))
        sector_deg = float(config.get("front_sector_deg", 60.0))
        if not (0.0 < sector_deg <= 360.0):
            raise ValueError("front_sector_deg must be in (0, 360]")
        self._half_sector = math.radians(sector_deg) / 2.0
        self._min_range = float(config.get("min_range", 0.05))
        self._max_range = float(config.get("max_range", 8.0))
        if not self._max_range > self._min_range:
            raise ValueError("max_range must exceed min_range")
        self._safety_stop_distance = float(config.get("safety_stop_distance", 0.25))
        if not self._safety_stop_distance > 0.0:
            raise ValueError("safety_stop_distance must be positive")
        self._target_stop_distance = float(config.get("target_stop_distance", 0.30))
        if not self._target_stop_distance >= self._safety_stop_distance:
            raise ValueError(
                "target_stop_distance must be >= safety_stop_distance"
            )

    @property
    def enabled(self):
        return self._enabled

    @property
    def target_stop_distance(self):
        return self._target_stop_distance

    def front_distance(self, ranges, angle_min, angle_increment):
        """Median range inside the front sector, or None when disabled/empty."""
        if not self._enabled:
            return None
        if ranges is None:
            return None
        valid = []
        for index, value in enumerate(ranges):
            angle = angle_min + index * angle_increment
            if abs(angle) > self._half_sector:
                continue
            if (
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(value)
                and self._min_range <= value <= self._max_range
            ):
                valid.append(float(value))
        if not valid:
            return None
        return float(np.median(valid))

    def danger(self, front_distance):
        """True when the front distance violates the safety stop distance."""
        if not self._enabled or front_distance is None:
            return False
        return front_distance < self._safety_stop_distance
