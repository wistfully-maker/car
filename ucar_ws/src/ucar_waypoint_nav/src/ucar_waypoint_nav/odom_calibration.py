import math


def shortest_angular_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class AngleAccumulator(object):
    def __init__(self, target, tolerance, direction=1):
        if target <= 0.0:
            raise ValueError("target must be positive")
        if tolerance < 0.0 or tolerance >= target:
            raise ValueError("tolerance must be non-negative and smaller than target")
        if direction not in (-1, 1):
            raise ValueError("direction must be -1 or 1")
        self.target = target
        self.tolerance = tolerance
        self.direction = direction
        self.accumulated = 0.0
        self._last_yaw = None

    def start(self, yaw):
        self.accumulated = 0.0
        self._last_yaw = yaw

    def update(self, yaw):
        if self._last_yaw is None:
            self.start(yaw)
            return self.accumulated
        self.accumulated += self.direction * shortest_angular_delta(yaw, self._last_yaw)
        self._last_yaw = yaw
        return self.accumulated

    @property
    def done(self):
        return self.accumulated >= self.target - self.tolerance
