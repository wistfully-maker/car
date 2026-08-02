import json
import math
import threading

from task_orchestrator.protocol import ProtocolError, load_object, require_text


def _finite_number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("%s must be finite" % field)
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise ProtocolError("%s must be finite" % field)
    if not math.isfinite(number):
        raise ProtocolError("%s must be finite" % field)
    return number


def validate_waypoint(value):
    """Return a validated, normalized pickup waypoint."""
    if not isinstance(value, dict):
        raise ProtocolError("waypoint must be an object")

    result = {
        "frame_id": require_text(value.get("frame_id"), "frame_id"),
        "map_sha256": require_text(value.get("map_sha256"), "map_sha256"),
    }
    for field in ("x", "y", "yaw", "position_tolerance", "yaw_tolerance"):
        result[field] = _finite_number(value.get(field), field)
    for field in ("position_tolerance", "yaw_tolerance"):
        if result[field] <= 0:
            raise ProtocolError("%s must be greater than zero" % field)
    return result


def build_arrival(task_id, goal_id, succeeded, message=""):
    """Build a protocol-v1 arrival terminal payload."""
    clean_task_id = require_text(task_id, "task_id")
    clean_goal_id = require_text(goal_id, "goal_id")
    if task_id != clean_task_id or goal_id != clean_goal_id:
        raise ProtocolError("identity must not contain surrounding whitespace")
    if not isinstance(succeeded, bool):
        raise ProtocolError("succeeded must be boolean")
    if not isinstance(message, str):
        raise ProtocolError("message must be text")
    clean_message = message.strip()
    if not succeeded and not clean_message:
        raise ProtocolError("message must be non-empty")
    return {
        "protocol_version": 1,
        "task_id": clean_task_id,
        "goal_id": clean_goal_id,
        "status": "arrived" if succeeded else "failed",
        "message": clean_message,
    }


class FastNavSession:
    """Track one active navigation identity and suppress stale terminals."""

    def __init__(self, waypoint):
        self._lock = threading.RLock()
        self._waypoint = validate_waypoint(waypoint)
        self._active_identity = None

    @property
    def waypoint(self):
        with self._lock:
            return dict(self._waypoint)

    @property
    def active_identity(self):
        with self._lock:
            if self._active_identity is None:
                return None
            return dict(self._active_identity)

    def accept_goal(self, message):
        goal = self._parse_goal(message)
        identity = {"task_id": goal["task_id"], "goal_id": goal["goal_id"]}
        with self._lock:
            if identity == self._active_identity:
                return False
            self._active_identity = identity
            return True

    def complete(self, task_id, goal_id, succeeded, message=""):
        with self._lock:
            if not self._matches(task_id, goal_id):
                return None
            payload = build_arrival(task_id, goal_id, succeeded, message)
            self._active_identity = None
            return payload

    def cancel(self, task_id, reason):
        with self._lock:
            if self._active_identity is None:
                return None
            if task_id != self._active_identity["task_id"]:
                return None
            return self.complete(
                self._active_identity["task_id"],
                self._active_identity["goal_id"],
                succeeded=False,
                message=reason,
            )

    def _matches(self, task_id, goal_id):
        return self._active_identity == {
            "task_id": task_id,
            "goal_id": goal_id,
        }

    @staticmethod
    def _parse_goal(message):
        if isinstance(message, dict):
            try:
                raw_json = json.dumps(message)
            except (TypeError, ValueError) as exc:
                raise ProtocolError("message must be JSON-compatible: %s" % exc)
        else:
            raw_json = message
        goal = load_object(raw_json)
        for field in ("task_id", "goal_id"):
            raw_value = goal.get(field)
            clean_value = require_text(raw_value, field)
            if raw_value != clean_value:
                raise ProtocolError(
                    "%s must not contain surrounding whitespace" % field
                )
            goal[field] = clean_value
        return goal


class StopDetector:
    """Detect continuous low linear and angular speed using explicit time."""

    def __init__(
        self, linear_threshold, angular_threshold, settle_time
    ):
        self.linear_threshold = self._config_number(
            linear_threshold, "linear_threshold", allow_zero=True
        )
        self.angular_threshold = self._config_number(
            angular_threshold, "angular_threshold", allow_zero=True
        )
        self.settle_time = self._config_number(
            settle_time, "settle_time", allow_zero=False
        )
        self._stopped_since = None
        self._last_timestamp = None

    def reset(self):
        self._stopped_since = None
        self._last_timestamp = None

    def update(self, linear_speed, angular_speed, timestamp):
        linear = self._sample_number(linear_speed, "linear_speed")
        angular = self._sample_number(angular_speed, "angular_speed")
        now = self._sample_number(timestamp, "timestamp")

        if self._last_timestamp is not None and now < self._last_timestamp:
            self._stopped_since = None
        self._last_timestamp = now

        stopped = (
            abs(linear) <= self.linear_threshold
            and abs(angular) <= self.angular_threshold
        )
        if not stopped:
            self._stopped_since = None
            return False
        if self._stopped_since is None:
            self._stopped_since = now
            return False
        return now - self._stopped_since >= self.settle_time

    @staticmethod
    def _sample_number(value, field):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("%s must be finite" % field)
        try:
            number = float(value)
        except (OverflowError, ValueError):
            raise ValueError("%s must be finite" % field)
        if not math.isfinite(number):
            raise ValueError("%s must be finite" % field)
        return number

    @classmethod
    def _config_number(cls, value, field, allow_zero):
        number = cls._sample_number(value, field)
        if number < 0 or (not allow_zero and number == 0):
            comparison = "non-negative" if allow_zero else "positive"
            raise ValueError("%s must be %s" % (field, comparison))
        return number
