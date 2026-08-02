"""ROS-independent two-source velocity arbiter."""

import math

from task_orchestrator.motion_mode import IDLE, NAVIGATION, QR_SEARCH


class VectorValue:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class TwistValue:
    def __init__(self, linear=None, angular=None):
        self.linear = linear or VectorValue()
        self.angular = angular or VectorValue()


def validate_positive_finite(name, value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError("%s must be a positive finite number" % name)
    return float(value)


def _finite_timestamp(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("timestamp must be a finite number")
    return float(value)


def _normalized(message, max_linear_abs, max_angular_abs):
    try:
        groups = []
        for vector_name, limit in (("linear", max_linear_abs), ("angular", max_angular_abs)):
            raw = getattr(message, vector_name)
            values = [getattr(raw, axis) for axis in ("x", "y", "z")]
            if any(type(value) not in (int, float) or not math.isfinite(value)
                   or abs(value) > limit for value in values):
                return None
            groups.append(VectorValue(*values))
        return TwistValue(*groups)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


class VelocityArbiter:
    SOURCES = frozenset(("navigation", "qr"))
    ACTIVE_SOURCE = {NAVIGATION: "navigation", QR_SEARCH: "qr"}

    def __init__(self, source_timeout=0.3, max_linear_abs=1.0,
                 max_angular_abs=2.0):
        self.source_timeout = validate_positive_finite("source_timeout", source_timeout)
        self.max_linear_abs = validate_positive_finite("max_linear_abs", max_linear_abs)
        self.max_angular_abs = validate_positive_finite("max_angular_abs", max_angular_abs)
        self.mode = IDLE
        self._last_time = None
        self._last_source_time = None
        self._stopped = False

    @staticmethod
    def _zero():
        return TwistValue()

    def _observe_time(self, timestamp):
        timestamp = _finite_timestamp(timestamp)
        rolled_back = self._last_time is not None and timestamp < self._last_time
        self._last_time = timestamp
        if rolled_back:
            self._last_source_time = None
            self._stopped = False
        return timestamp, rolled_back

    def set_mode(self, mode, timestamp):
        _, rolled_back = self._observe_time(timestamp)
        valid = mode in (IDLE, NAVIGATION, QR_SEARCH)
        safe_mode = mode if valid else IDLE
        changed = safe_mode != self.mode
        self.mode = safe_mode
        if changed or rolled_back:
            self._last_source_time = None
            self._stopped = False
        return self._zero() if changed or rolled_back or not valid else None

    def accept(self, source, message, timestamp):
        if source not in self.SOURCES:
            raise ValueError("unknown velocity source: %s" % source)
        timestamp, rolled_back = self._observe_time(timestamp)
        if rolled_back:
            return self._zero()
        if self.ACTIVE_SOURCE.get(self.mode) != source:
            return None
        copied = _normalized(message, self.max_linear_abs, self.max_angular_abs)
        if copied is None:
            if self._stopped:
                return None
            self._last_source_time = None
            self._stopped = True
            return self._zero()
        self._last_source_time = timestamp
        self._stopped = False
        return copied

    def tick(self, timestamp):
        timestamp, rolled_back = self._observe_time(timestamp)
        if rolled_back:
            return self._zero()
        if self.mode == IDLE or self._last_source_time is None or self._stopped:
            return None
        if timestamp - self._last_source_time <= self.source_timeout:
            return None
        self._stopped = True
        return self._zero()
