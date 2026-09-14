"""Odometry-based settling detection for stop-and-scan stations."""

import math


class SettlingDetector:
    def __init__(self, speed_threshold, required_duration):
        if isinstance(speed_threshold, bool) or not math.isfinite(float(speed_threshold)):
            raise ValueError("speed_threshold must be a finite number")
        if isinstance(required_duration, bool) or not math.isfinite(float(required_duration)):
            raise ValueError("required_duration must be a finite number")
        self.speed_threshold = float(speed_threshold)
        self.required_duration = float(required_duration)
        if self.speed_threshold < 0:
            raise ValueError("speed_threshold must be non-negative")
        if self.required_duration < 0:
            raise ValueError("required_duration must be non-negative")
        self._since = None

    def reset(self):
        self._since = None

    def update(self, angular_speed, now):
        angular_speed = float(angular_speed)
        now = float(now)
        if not math.isfinite(angular_speed) or not math.isfinite(now):
            raise ValueError("settling inputs must be finite")
        if abs(angular_speed) > self.speed_threshold:
            self._since = None
            return False
        if self._since is None:
            self._since = now
            return self.required_duration == 0.0
        if now < self._since:
            raise ValueError("time moved backwards")
        return now - self._since >= self.required_duration
