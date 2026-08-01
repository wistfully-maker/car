import math


def shortest_angular_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class AngleAccumulator(object):
    def __init__(self, target, tolerance):
        if target <= 0.0:
            raise ValueError("target must be positive")
        if tolerance < 0.0 or tolerance >= target:
            raise ValueError("tolerance must be non-negative and smaller than target")
        self.target = target
        self.tolerance = tolerance
        self.accumulated = 0.0
        self._last_yaw = None

    def start(self, yaw):
        self.accumulated = 0.0
        self._last_yaw = yaw

    def update(self, yaw):
        if self._last_yaw is None:
            self.start(yaw)
            return self.accumulated
        self.accumulated += shortest_angular_delta(yaw, self._last_yaw)
        self._last_yaw = yaw
        return self.accumulated

    @property
    def done(self):
        return self.accumulated >= self.target - self.tolerance
