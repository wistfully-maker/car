import json
import math
import threading
import time

from qr_item_search.search_state import SearchMachine
from qr_item_search.yaw_control import angular_command, normalize_angle


class SearchController:
    _STARTABLE_STATES = {"IDLE", "SUCCESS", "NOT_FOUND", "ERROR"}
    _OBSERVATION_STATUSES = {
        "success",
        "invalid_url",
        "invalid_payload",
        "http_error",
        "decode_error",
    }

    def __init__(
        self,
        wall_yaw_offsets,
        outputs,
        kp=1.2,
        max_speed=0.30,
        tolerance=0.035,
        settle_seconds=0.8,
        scan_timeout=4.0,
        turn_timeout=8.0,
        error_handler=None,
        clock=None,
    ):
        self._offsets = self._validate_offsets(wall_yaw_offsets)
        self._validate_controls(
            kp,
            max_speed,
            tolerance,
            settle_seconds,
            scan_timeout,
            turn_timeout,
        )
        self._outputs = outputs
        self._error_handler = (
            error_handler if error_handler is not None else lambda error: None
        )
        self._clock = clock if clock is not None else time.monotonic
        self._kp = kp
        self._max_speed = max_speed
        self._tolerance = tolerance
        self._settle_seconds = settle_seconds
        self._scan_timeout = scan_timeout
        self._turn_timeout = turn_timeout
        self._machine = SearchMachine(len(self._offsets))
        self._current_yaw = None
        self._entry_yaw = None
        self._state_started = 0.0
        self._last_now = None
        self._search_id = 0
        self._lock = threading.RLock()
        self._output_errors = []
        with self._lock:
            self._publish_entry_outputs_locked()
        self._drain_output_errors()

    @property
    def state(self):
        with self._lock:
            return self._machine.state

    @property
    def wall_index(self):
        with self._lock:
            return self._machine.wall_index

    def update_yaw(self, yaw):
        if not self._is_finite_number(yaw):
            raise ValueError("yaw must be finite")
        with self._lock:
            self._current_yaw = yaw
        self._drain_output_errors()

    def start(self, now=None):
        return self._run_timed(now, lambda resolved_now: self._start_locked())

    def tick(self, now=None):
        return self._run_timed(now, self._tick_locked)

    def handle_observation(self, raw_payload, now=None):
        return self._run_timed(
            now,
            lambda resolved_now: self._handle_observation_locked(
                raw_payload,
                resolved_now,
            ),
        )

    def match_decision(self, matched, now=None):
        return self._run_timed(
            now,
            lambda resolved_now: self._match_decision_locked(
                matched,
                resolved_now,
            ),
        )

    def stop(self, now=None):
        return self._run_timed(now, self._stop_locked)

    def shutdown(self):
        with self._lock:
            self._emit_locked("publish_speed", 0.0)
        self._drain_output_errors()

    def _run_timed(self, now, action):
        with self._lock:
            if now is None:
                try:
                    resolved_now = self._clock()
                except Exception as error:
                    self._output_errors.append(error)
                    self._degrade_to_error_locked()
                    result = False
                    resolved_now = None
                else:
                    if not self._is_finite_number(resolved_now):
                        self._degrade_to_error_locked()
                        result = False
                        resolved_now = None
            else:
                if not self._is_finite_number(now):
                    raise ValueError("now must be finite")
                resolved_now = now

            if resolved_now is None:
                pass
            elif (
                self._last_now is not None
                and resolved_now < self._last_now
            ):
                self._machine.fail()
                self._enter_state_locked(self._last_now)
                result = False
            else:
                self._last_now = resolved_now
                result = action(resolved_now)
        self._drain_output_errors()
        return result

    def _start_locked(self):
        if self._machine.state not in self._STARTABLE_STATES:
            return False
        if self._current_yaw is None:
            self._machine.fail()
            self._enter_state_locked(self._last_now)
            return False
        next_search_id = self._search_id + 1
        if not self._emit_locked("publish_reset", next_search_id):
            self._degrade_to_error_locked()
            return False
        self._entry_yaw = self._current_yaw
        self._search_id = next_search_id
        self._machine.start()
        return self._enter_state_locked(self._last_now)

    def _tick_locked(self, now):
        elapsed = now - self._state_started
        if self._machine.state == "TURNING":
            if self._current_yaw is None or elapsed >= self._turn_timeout:
                self._machine.fail()
                self._enter_state_locked(now)
                return False
            target = normalize_angle(
                self._entry_yaw + self._offsets[self._machine.wall_index]
            )
            speed, reached = angular_command(
                self._current_yaw,
                target,
                self._kp,
                self._max_speed,
                self._tolerance,
            )
            if not self._emit_locked("publish_speed", speed) and speed != 0.0:
                self._machine.fail()
                self._enter_state_locked(now)
                return False
            if reached:
                self._machine.turn_reached()
                return self._enter_state_locked(now)
        elif (
            self._machine.state == "SETTLING"
            and elapsed >= self._settle_seconds
        ):
            self._machine.settled()
            return self._enter_state_locked(now)
        elif (
            self._machine.state == "SCANNING"
            and elapsed >= self._scan_timeout
        ):
            self._machine.scan_timeout()
            return self._enter_state_locked(now)
        return True

    def _handle_observation_locked(self, raw_payload, now):
        if self._machine.state != "SCANNING":
            return False
        try:
            payload = json.loads(raw_payload)
        except (TypeError, ValueError):
            return self._malformed_observation_locked(now)
        if not isinstance(payload, dict):
            return self._malformed_observation_locked(now)
        wall_index = payload.get("wall_index")
        status = payload.get("status")
        search_id = payload.get("search_id")
        if (
            type(search_id) is not int
            or search_id < 0
            or type(wall_index) is not int
            or wall_index < 0
            or type(status) is not str
            or status not in self._OBSERVATION_STATUSES
        ):
            return self._malformed_observation_locked(now)
        if search_id != self._search_id:
            return False
        if wall_index != self._machine.wall_index:
            return False
        if status == "success":
            self._machine.observation_succeeded()
        else:
            self._machine.observation_failed()
        return self._enter_state_locked(now)

    def _malformed_observation_locked(self, now):
        self._machine.fail()
        self._enter_state_locked(now)
        return False

    def _match_decision_locked(self, matched, now):
        if self._machine.state != "WAITING_MATCH":
            return False
        self._machine.match_decision(matched)
        return self._enter_state_locked(now)

    def _stop_locked(self, now):
        self._machine.stop()
        return self._enter_state_locked(now)

    def _enter_state_locked(self, now):
        self._state_started = now
        speed_ok = self._emit_locked("publish_speed", 0.0)
        enabled_ok = self._emit_locked(
            "publish_scan_enabled",
            self._machine.state == "SCANNING",
        )
        wall_ok = self._emit_locked(
            "publish_wall",
            self._machine.wall_index,
        )
        self._emit_locked("publish_state", self._machine.state)
        if not (speed_ok and enabled_ok and wall_ok):
            self._degrade_to_error_locked()
            return False
        return True

    def _degrade_to_error_locked(self):
        self._machine.fail()
        self._emit_locked("publish_speed", 0.0)
        self._emit_locked("publish_scan_enabled", False)
        self._emit_locked("publish_wall", self._machine.wall_index)
        self._emit_locked("publish_state", "ERROR")

    def _publish_entry_outputs_locked(self):
        self._emit_locked("publish_speed", 0.0)
        self._emit_locked(
            "publish_scan_enabled",
            self._machine.state == "SCANNING",
        )
        self._emit_locked("publish_wall", self._machine.wall_index)
        self._emit_locked("publish_state", self._machine.state)

    def _emit_locked(self, method_name, *args):
        try:
            getattr(self._outputs, method_name)(*args)
            return True
        except Exception as error:
            self._output_errors.append(error)
            return False

    def _drain_output_errors(self):
        with self._lock:
            errors = self._output_errors
            self._output_errors = []
        for error in errors:
            try:
                self._error_handler(error)
            except Exception:
                pass

    @staticmethod
    def _is_finite_number(value):
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )

    @classmethod
    def _validate_offsets(cls, offsets):
        try:
            values = list(offsets)
        except TypeError:
            raise ValueError("wall_yaw_offsets must be a non-empty sequence")
        if not values or any(
            not cls._is_finite_number(value) for value in values
        ):
            raise ValueError("wall_yaw_offsets must contain finite numbers")
        return values

    @classmethod
    def _validate_controls(
        cls,
        kp,
        max_speed,
        tolerance,
        settle_seconds,
        scan_timeout,
        turn_timeout,
    ):
        values = (
            kp,
            max_speed,
            tolerance,
            settle_seconds,
            scan_timeout,
            turn_timeout,
        )
        if any(not cls._is_finite_number(value) for value in values):
            raise ValueError("controller parameters must be finite numbers")
        if kp <= 0 or max_speed <= 0 or tolerance < 0:
            raise ValueError("invalid yaw control parameters")
        if settle_seconds < 0 or scan_timeout <= 0 or turn_timeout <= 0:
            raise ValueError("invalid controller timing")
