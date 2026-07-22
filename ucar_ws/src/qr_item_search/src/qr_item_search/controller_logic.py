"""Protocol-v1 continuous QR search controller."""
import json
import math
import threading
import time

from qr_item_search.protocol import ProtocolError, build_search_result, parse_start_request, parse_stop_request
from qr_item_search.search_state import SearchMachine
from qr_item_search.sweep_coverage import CoverageMap, YawTracker
from qr_item_search.yaw_control import directed_angular_command


class SearchController:
    def __init__(self, outputs, fast_angular_speed=.40, targeted_angular_speed=.20,
                 minimum_effective_speed=.11, fast_sweep_angle=6.632251,
                 yaw_tolerance=.035, heading_timeout=1.0, camera_timeout=1.0,
                 search_total_timeout=40.0, coverage=None, error_handler=None, clock=None):
        values = (fast_angular_speed, targeted_angular_speed, minimum_effective_speed,
                  fast_sweep_angle, yaw_tolerance, heading_timeout, camera_timeout,
                  search_total_timeout)
        if any(not self._finite(value) for value in values):
            raise ValueError("controller parameters must be finite")
        if (fast_angular_speed <= 0 or targeted_angular_speed <= 0 or
                minimum_effective_speed < 0 or
                minimum_effective_speed > min(fast_angular_speed, targeted_angular_speed) or
                fast_sweep_angle <= 0 or yaw_tolerance < 0 or
                heading_timeout <= 0 or camera_timeout <= 0 or search_total_timeout <= 0):
            raise ValueError("invalid controller parameters")
        self._outputs, self._fast, self._target = outputs, float(fast_angular_speed), float(targeted_angular_speed)
        self._minimum, self._sweep_angle, self._tolerance = float(minimum_effective_speed), float(fast_sweep_angle), float(yaw_tolerance)
        self._heading_timeout, self._camera_timeout, self._total_timeout = heading_timeout, camera_timeout, search_total_timeout
        self._coverage, self._errors, self._clock = coverage or CoverageMap(), error_handler or (lambda _: None), clock or time.monotonic
        self._machine, self._tracker, self._lock = SearchMachine(), YawTracker(), threading.RLock()
        self._action_lock, self._speed_lock, self._control_epoch = threading.Lock(), threading.Lock(), 0
        self._yaw = self._relative = self._last_heading = self._last_now = None
        self._task = self._search = None
        self._started = self._last_image = None
        self._detected, self._resolved, self._resolve_errors = {}, {}, {}
        self._intervals, self._interval_index, self._target_phase = [], 0, "approach"
        self._last_result = None
        self._retry_edge_sent = False
        self._shutdown_latched = False
        self._scanner_session = None
        self._emit([("publish_speed", 0.0), ("publish_state", "IDLE"),
                    ("publish_scanner_control", self._control(False, False, False))], 0)

    @property
    def state(self):
        with self._lock: return self._machine.state

    def update_yaw(self, yaw, now=None):
        yaw = self._number(yaw, "yaw"); now = self._resolve_now(now)
        if now is None: return False
        with self._lock:
            if not self._serial(now): return self._fail_locked(now, "time moved backwards")
            self._yaw, self._last_heading = yaw, now
            actions = []
            if self._machine.state in self._machine.ACTIVE:
                self._relative = self._tracker.update(yaw)
                actions.append(("publish_scanner_control", self._control(
                    self._machine.state in ("FAST_SWEEP", "TARGETED_RESCAN"),
                    self._machine.state == "TARGETED_RESCAN", False)))
            epoch = self._control_epoch
        return self._emit(actions, epoch)

    def start(self, raw_json, now=None):
        with self._lock:
            if self._shutdown_latched:
                return False
        now = self._resolve_now(now)
        if now is None: return False
        try: request = parse_start_request(raw_json)
        except ProtocolError:
            self._emit([("publish_speed", 0.0)]); return False
        with self._lock:
            if self._shutdown_latched:
                return False
            if not self._serial(now): return self._finish_error_locked(now, "time moved backwards")
            same = request.search_id == self._search
            if same and self._machine.state in self._machine.ACTIVE:
                actions = [("publish_state", self._machine.state)]
                if self._last_result: actions.append(("publish_result", self._last_result))
                result = False
            elif same and self._last_result is not None:
                actions, result = [("publish_state", self._machine.state), ("publish_result", self._last_result)], False
            elif self._machine.state not in self._machine.RESTARTABLE:
                actions, result = [], False
            elif self._yaw is None or self._last_heading is None or now - self._last_heading > self._heading_timeout:
                self._task, self._search = request.task_id, request.search_id
                return self._finish_error_locked(now, "heading unavailable")
            else:
                self._task, self._search = request.task_id, request.search_id
                self._control_epoch += 1
                self._machine.start(); self._tracker.reset(self._yaw); self._relative = 0.0
                self._coverage.reset()
                self._detected, self._resolved, self._resolve_errors = {}, {}, {}
                self._retry_edge_sent = False
                self._intervals, self._interval_index, self._target_phase = [], 0, "approach"
                self._started, self._last_image = now, None
                searching = self._result(now, "searching", "", [])
                self._last_result = searching
                actions = [("publish_speed", 0.0), ("publish_result", searching),
                           ("publish_state", "FAST_SWEEP"),
                           ("publish_scanner_control", self._control(True, False, False))]
                result = True
            epoch = self._control_epoch
        self._emit(actions, epoch); return result

    def tick(self, now=None):
        now = self._resolve_now(now)
        if now is None: return False
        with self._lock:
            if not self._serial(now): return self._finish_error_locked(now, "time moved backwards")
            state = self._machine.state
            if state not in self._machine.ACTIVE:
                actions = [("publish_speed", 0.0)]
            elif now - self._started >= self._total_timeout:
                return self._finish_not_found_locked(now, "search timed out")
            elif state in ("FAST_SWEEP", "TARGETED_RESCAN") and (
                    self._last_heading is None or now - self._last_heading > self._heading_timeout):
                return self._finish_error_locked(now, "heading timed out")
            elif state in ("FAST_SWEEP", "TARGETED_RESCAN") and (
                    (self._last_image is None and now - self._started > self._camera_timeout) or
                    (self._last_image is not None and now - self._last_image > self._camera_timeout)):
                return self._finish_error_locked(now, "camera timed out")
            elif state == "WAITING_HTTP":
                actions = [("publish_speed", 0.0), ("publish_scanner_control", self._control(False, False, False))]
            elif state == "FAST_SWEEP":
                if self._relative >= self._sweep_angle:
                    actions = self._enter_targeted_rescan_locked(now)
                else: actions = [("publish_speed", self._fast, self._control_epoch)]
            else:
                actions = self._target_tick_locked(now)
            epoch = self._control_epoch
        ok = self._emit(actions, epoch)
        if not ok and any(action[0] == "publish_speed" and action[1] != 0 for action in actions):
            with self._lock: return self._finish_error_locked(now, "speed publisher failed")
        return ok

    def handle_scanner_event(self, raw_json, now=None):
        try: payload = json.loads(raw_json)
        except (TypeError, ValueError):
            now = self._resolve_now(now)
            if now is None: return False
            with self._lock: return self._finish_error_locked(now, "malformed scanner event")
        if not isinstance(payload, dict):
            now = self._resolve_now(now)
            if now is None: return False
            with self._lock: return self._finish_error_locked(now, "malformed scanner event")
        if payload.get("event") == "scanner_started":
            now = self._resolve_now(now)
            if now is None: return False
            with self._lock:
                if not self._serial(now): return self._finish_error_locked(now, "time moved backwards")
                session = payload.get("scanner_session")
                valid = (type(payload.get("protocol_version")) is int
                         and payload.get("protocol_version") == 1
                         and isinstance(session, str) and bool(session.strip()))
                if not valid:
                    if self._machine.state in self._machine.ACTIVE:
                        return self._finish_error_locked(now, "malformed scanner event")
                    return False
                previous = self._scanner_session
                self._scanner_session = session.strip()
                if (previous is not None and previous != self._scanner_session
                        and self._machine.state in self._machine.ACTIVE):
                    return self._finish_error_locked(now, "scanner restarted")
                return False
        with self._lock:
            task_id, search_id = payload.get("task_id"), payload.get("search_id")
            valid_identity = (
                isinstance(task_id, str) and bool(task_id.strip()) and
                isinstance(search_id, str) and bool(search_id.strip())
            )
            malformed_identity = not valid_identity and self._machine.state in self._machine.ACTIVE
            if valid_identity and (task_id != self._task or search_id != self._search):
                return False
            if valid_identity and self._machine.state not in self._machine.ACTIVE:
                return False
        now = self._resolve_now(now)
        if now is None: return False
        with self._lock:
            if not self._serial(now): return self._finish_error_locked(now, "time moved backwards")
            if malformed_identity:
                return self._finish_error_locked(now, "malformed scanner event")
            if payload.get("protocol_version") != 1 or not isinstance(payload.get("event"), str):
                return self._finish_error_locked(now, "malformed scanner event")
            try: actions = self._scanner_event_locked(payload, now)
            except Exception:
                return self._finish_error_locked(now, "malformed scanner event")
            if actions is None:
                return False
            epoch = self._control_epoch
        self._emit(actions, epoch); return True

    def stop(self, raw_json, now=None):
        try: request = parse_stop_request(raw_json)
        except ProtocolError: return False
        with self._lock:
            if request.task_id != self._task or request.search_id != self._search: return False
        now = self._resolve_now(now)
        if now is None: return False
        with self._lock:
            if not self._serial(now): return self._finish_error_locked(now, "time moved backwards")
            self._machine.stop()
            self._control_epoch += 1
            result = self._result(now, "stopped", request.reason, self._partial_items())
            self._last_result = result
            actions = self._terminal_actions(result)
            epoch = self._control_epoch
        self._emit(actions, epoch); return True

    def shutdown(self):
        with self._lock:
            self._shutdown_latched = True
            self._control_epoch += 1
            epoch = self._control_epoch
            was_active = self._machine.state in self._machine.ACTIVE
            if was_active:
                self._machine.fail()
            actions = [
                ("publish_speed", 0.0),
                ("publish_scanner_control", self._control(False, False, False)),
                ("publish_state", self._machine.state),
            ]
            if was_active and self._task:
                stamp = self._last_now if self._last_now is not None else 0.0
                result = self._result(stamp, "error", "controller shutdown", self._partial_items())
                self._last_result = result
                actions.append(("publish_result", result))
        self._emit(actions, epoch)

    def _scanner_event_locked(self, p, now):
        kind = p["event"]
        if kind == "quality":
            b, o, s, y = (self._number(p[key], key) for key in ("brightness", "overexposed", "sharpness", "detected_yaw"))
            if not isinstance(p.get("decoded"), bool): raise ValueError()
            self._coverage.record(y, b, o, s, p["decoded"]); self._last_image = now
            return []
        if kind == "invalid_url":
            if not isinstance(p.get("url"), str) or not isinstance(p.get("message"), str):
                raise ValueError()
            self._number(p.get("detected_yaw"), "detected_yaw")
            return []
        if kind == "detected":
            if self._machine.state == "WAITING_HTTP":
                return None
            order, url, yaw = self._item_identity(p)
            existing = self._detected.get(order)
            candidate = {"order": order, "url": url, "detected_yaw": yaw}
            if existing and existing != candidate: raise ValueError()
            if any(item["url"] == url and key != order for key, item in self._detected.items()): raise ValueError()
            self._detected[order] = candidate
            if len(self._detected) == 3 and self._machine.state in ("FAST_SWEEP", "TARGETED_RESCAN"):
                self._control_epoch += 1
                self._machine.urls_collected()
                if self._resolve_errors:
                    return self._enter_targeted_rescan_locked(now)
                return [("publish_speed", 0.0), ("publish_state", "WAITING_HTTP"),
                        ("publish_scanner_control", self._control(False, False, False))]
            return []
        if kind in ("resolved", "resolve_error"):
            order, url, yaw = self._item_identity(p)
            detected = self._detected.get(order)
            if detected is None or detected["url"] != url or detected["detected_yaw"] != yaw: raise ValueError()
            if kind == "resolved":
                name = self._text(p.get("item_name"), "item_name")
                self._resolved[order] = name
                self._resolve_errors.pop(order, None)
                if len(self._resolved) == 3:
                    self._control_epoch += 1
                    self._machine.items_resolved()
                    result = self._result(now, "complete", "", self._complete_items())
                    self._last_result = result
                    return self._terminal_actions(result)
            else:
                message = self._text(p.get("message"), "message")
                self._resolve_errors[order] = message
                if self._machine.state == "WAITING_HTTP":
                    return self._enter_targeted_rescan_locked(now)
            return []
        raise ValueError()

    def _target_tick_locked(self, now):
        if self._interval_index >= len(self._intervals):
            return self._rescan_not_found_actions_locked(now)
        start, end = self._intervals[self._interval_index]
        if self._target_phase == "approach":
            speed, reached = directed_angular_command(start - self._relative, self._target, self._tolerance, self._minimum)
            if reached: self._target_phase = "sweep"; speed = 0.0
            return [("publish_speed", speed, self._control_epoch), ("publish_scanner_control", self._control(True, True, False))]
        if self._relative >= end - self._tolerance:
            self._interval_index += 1; self._target_phase = "approach"
            if self._interval_index >= len(self._intervals):
                return self._rescan_not_found_actions_locked(now)
            return [("publish_speed", 0.0)]
        return [("publish_speed", self._target, self._control_epoch), ("publish_scanner_control", self._control(True, True, False))]

    def _rescan_not_found_actions_locked(self, now):
        self._machine.rescan_finished()
        self._control_epoch += 1
        result = self._result(now, "not_found", "items not found", self._partial_items())
        self._last_result = result
        return self._terminal_actions(result)

    def _begin_rescan_locked(self):
        current = self._relative
        self._intervals = []
        for interval in self._coverage.rescan_intervals():
            shift = round((current - interval.start) / math.tau) * math.tau
            self._intervals.append((interval.start + shift, interval.end + shift))
        self._interval_index, self._target_phase = 0, "approach"

    def _enter_targeted_rescan_locked(self, now):
        if self._machine.state == "FAST_SWEEP":
            self._machine.fast_sweep_finished()
        elif self._machine.state == "WAITING_HTTP":
            self._machine.resume_rescan()
        self._control_epoch += 1
        self._begin_rescan_locked()
        retry = bool(self._resolve_errors) and not self._retry_edge_sent
        if retry: self._retry_edge_sent = True
        return [("publish_speed", 0.0), ("publish_state", "TARGETED_RESCAN"),
                ("publish_scanner_control", self._control(True, True, retry))]

    def _finish_not_found_locked(self, now, message):
        self._machine.timeout() if self._machine.state in self._machine.ACTIVE else None
        self._control_epoch += 1
        result = self._result(now, "not_found", message, self._partial_items())
        self._last_result = result; actions = self._terminal_actions(result)
        self._emit_without_state_lock(actions, self._control_epoch); return False

    def _finish_error_locked(self, now, message):
        self._machine.fail()
        self._control_epoch += 1
        items = self._partial_items() if self._task else []
        actions = [("publish_speed", 0.0), ("publish_state", "ERROR"),
                   ("publish_scanner_control", self._control(False, False, False))]
        if self._task:
            result = self._result(now, "error", message, items); self._last_result = result
            actions.append(("publish_result", result))
        self._emit_without_state_lock(actions, self._control_epoch); return False

    def _fail_locked(self, now, message): return self._finish_error_locked(now, message)
    def _terminal_actions(self, result):
        return [("publish_speed", 0.0), ("publish_state", self._machine.state),
                ("publish_scanner_control", self._control(False, False, False)), ("publish_result", result)]
    def _control(self, enabled, enhanced, retry, identity=True):
        result = {"protocol_version": 1, "enabled": enabled, "enhanced": enhanced,
                  "detected_yaw": self._relative or 0.0, "retry_failed": retry}
        result.update(task_id=self._task or "", search_id=self._search or "")
        return result
    def _result(self, now, status, message, items):
        return build_search_result(self._task, self._search, now, status, items, message)
    def _complete_items(self):
        return [dict(self._detected[i], item_name=self._resolved[i]) for i in (1, 2, 3)]
    def _partial_items(self):
        items = []
        found_orders = [order for order in sorted(self._detected) if order in self._resolved]
        for new_order, old_order in enumerate(found_orders, 1):
            value = self._detected[old_order]
            items.append({"order": new_order, "url": value["url"], "detected_yaw": value["detected_yaw"],
                          "item_name": self._resolved[old_order]})
        return items
    def _item_identity(self, p):
        order = p.get("order")
        if type(order) is not int or not 1 <= order <= 3: raise ValueError()
        return order, self._text(p.get("url"), "url"), self._number(p.get("detected_yaw"), "detected_yaw")
    def _resolve_now(self, now):
        if now is not None: return self._number(now, "now")
        try: return self._number(self._clock(), "clock")
        except Exception as error:
            self._report(error)
            with self._lock: self._finish_error_locked(self._last_now or 0.0, "clock failed")
            return None
    def _serial(self, now):
        if self._last_now is not None and now < self._last_now: return False
        self._last_now = now; return True
    def _emit(self, actions, expected_epoch=None):
        if not actions:
            return True
        if expected_epoch is None:
            with self._lock: expected_epoch = self._control_epoch
        ok = True
        critical_failure = None
        with self._action_lock:
            for action in actions:
                name, value = action[:2]
                if not (name == "publish_speed" and value == 0.0):
                    with self._lock:
                        if expected_epoch != self._control_epoch:
                            continue
                if name == "publish_speed":
                    speed_epoch = action[2] if len(action) == 3 else expected_epoch
                    if not self._emit_speed_guarded(value, speed_epoch): ok = False
                    continue
                try:
                    getattr(self._outputs, name)(value)
                except Exception as error:
                    ok = False; self._report(error)
                    if name == "publish_scanner_control" and value.get("enabled") is True:
                        critical_failure = "scanner control publisher failed"
        if critical_failure is not None:
            self._degrade_output_failure(expected_epoch, critical_failure)
        return ok
    def _emit_speed_guarded(self, value, expected_epoch):
        with self._speed_lock:
            if value != 0.0:
                with self._lock:
                    if (expected_epoch != self._control_epoch or
                            self._machine.state not in ("FAST_SWEEP", "TARGETED_RESCAN")):
                        return True
            try:
                self._outputs.publish_speed(value)
                return True
            except Exception as error:
                self._report(error)
                return False
    def _emit_without_state_lock(self, actions, expected_epoch):
        self._lock.release()
        try:
            return self._emit(actions, expected_epoch)
        finally:
            self._lock.acquire()
    def _degrade_output_failure(self, expected_epoch, message):
        with self._lock:
            if expected_epoch != self._control_epoch:
                return
            self._machine.fail(); self._control_epoch += 1
            epoch = self._control_epoch
            actions = [("publish_speed", 0.0), ("publish_scanner_control", self._control(False, False, False)),
                       ("publish_state", "ERROR")]
            if self._task:
                result = self._result(self._last_now or 0.0, "error", message, self._partial_items())
                self._last_result = result; actions.append(("publish_result", result))
        self._emit(actions, epoch)
    def _report(self, error):
        try: self._errors(error)
        except Exception: pass
    @staticmethod
    def _finite(value): return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    @classmethod
    def _number(cls, value, name):
        if not cls._finite(value): raise ValueError("%s must be finite" % name)
        return float(value)
    @staticmethod
    def _text(value, name):
        if not isinstance(value, str) or not value.strip(): raise ValueError("%s must be non-empty" % name)
        return value.strip()
