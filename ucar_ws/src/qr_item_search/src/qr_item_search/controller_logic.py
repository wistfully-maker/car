"""Protocol-v1 fixed-angle stop-and-scan QR search controller."""
import json
import math
import threading
import time

from qr_item_search.protocol import ProtocolError, build_search_result, parse_start_request, parse_stop_request
from qr_item_search.scan_schedule import build_pass_angles
from qr_item_search.search_state import SearchMachine
from qr_item_search.settling import SettlingDetector
from qr_item_search.sweep_coverage import YawTracker
from qr_item_search.yaw_control import normalize_angle, staged_angular_command

SCAN_STATES = frozenset({"INITIAL_SCAN", "SCANNING"})
MOTION_STATES = frozenset({"INITIAL_SCAN", "TURNING", "SETTLING", "SCANNING", "OFFSET_PASS"})


class SearchController:
    def __init__(self, outputs, step_angle_deg=45.0, cruise_angular_speed=0.50,
                 approach_angular_speed=0.20, approach_zone_deg=10.0,
                 yaw_tolerance_deg=2.0, settled_angular_speed=0.03,
                 settled_duration=0.20, scan_window=0.60, offset_angle_deg=22.5,
                 max_passes=2, search_total_timeout=60.0, settling_timeout=3.0,
                 heading_timeout=1.0, camera_timeout=1.0,
                 error_handler=None, clock=None, metrics=None):
        values = (step_angle_deg, cruise_angular_speed, approach_angular_speed,
                  approach_zone_deg, yaw_tolerance_deg, settled_angular_speed,
                  settled_duration, scan_window, offset_angle_deg, max_passes,
                  search_total_timeout, settling_timeout, heading_timeout,
                  camera_timeout)
        if any(not self._finite(value) for value in values):
            raise ValueError("controller parameters must be finite")
        if (step_angle_deg <= 0 or cruise_angular_speed <= 0 or
                approach_angular_speed <= 0 or approach_zone_deg < 0 or
                yaw_tolerance_deg < 0 or settled_angular_speed < 0 or
                settled_duration < 0 or scan_window <= 0 or offset_angle_deg < 0 or
                type(max_passes) is not int or max_passes < 1 or
                search_total_timeout <= 0 or settling_timeout <= 0 or
                heading_timeout <= 0 or camera_timeout <= 0):
            raise ValueError("invalid controller parameters")
        if yaw_tolerance_deg > approach_zone_deg:
            raise ValueError("yaw_tolerance_deg must not exceed approach_zone_deg")
        if step_angle_deg >= 360.0:
            raise ValueError("step_angle_deg must be smaller than 360")
        self._step_angle_deg = float(step_angle_deg)
        self._offset_angle_deg = float(offset_angle_deg)
        build_pass_angles(self._step_angle_deg, 0.0)
        build_pass_angles(self._step_angle_deg, self._offset_angle_deg)
        self._cruise = float(cruise_angular_speed)
        self._approach = float(approach_angular_speed)
        self._approach_zone = math.radians(approach_zone_deg)
        self._tolerance = math.radians(yaw_tolerance_deg)
        self._settling_speed = float(settled_angular_speed)
        self._settling_duration = float(settled_duration)
        self._scan_window = float(scan_window)
        self._max_passes = max_passes
        self._total_timeout = float(search_total_timeout)
        self._settling_timeout = float(settling_timeout)
        self._heading_timeout = float(heading_timeout)
        self._camera_timeout = float(camera_timeout)
        self._outputs = outputs
        self._errors = error_handler or (lambda _: None)
        self._clock = clock or time.monotonic
        self._metrics = metrics
        self._machine = SearchMachine()
        self._tracker = YawTracker()
        self._settling = SettlingDetector(self._settling_speed, self._settling_duration)
        self._lock = threading.RLock()
        self._action_lock = threading.Lock()
        self._speed_lock = threading.Lock()
        self._control_epoch = 0
        self._yaw = self._relative = self._last_heading = self._last_now = None
        self._task = self._search = None
        self._started = self._last_frame_seen = None
        self._detected = {}
        self._resolved = {}
        self._resolve_errors = {}
        self._retry_edge_sent = False
        self._schedule = []
        self._pass_index = 0
        self._station_index = 0
        self._target_rel = 0.0
        self._dwell_open = False
        self._dwell_started = None
        self._dwell_frame_count = 0
        self._dwell_quality = None
        self._last_quality = None
        self._settling_started = None
        self._stations = []
        self._station = None
        self._prev_station = None
        self._state_seconds = {}
        self._state_entered = {}
        self._last_result = None
        self._shutdown_latched = False
        self._scanner_session = None
        self._emit([("publish_speed", 0.0), ("publish_state", "IDLE"),
                    ("publish_scanner_control", self._control(False, False, False))], 0)

    @property
    def state(self):
        with self._lock:
            return self._machine.state

    def debug_snapshot(self):
        with self._lock:
            quality = self._last_quality or {}
            return {
                "protocol_version": 1,
                "task_id": self._task or "",
                "search_id": self._search or "",
                "state": self._machine.state,
                "relative_yaw_deg": round(math.degrees(self._relative or 0.0), 2),
                "target_yaw_deg": round(math.degrees(self._target_rel or 0.0), 2),
                "settled": self._machine.state == "SCANNING",
                "found_count": len(self._detected),
                "expected_count": 3,
                "pass_index": self._pass_index,
                "station_index": self._station_index,
                "brightness": quality.get("brightness"),
                "overexposed": quality.get("overexposed"),
                "sharpness": quality.get("sharpness"),
            }

    def update_yaw(self, yaw, now=None, angular_speed=0.0):
        yaw = self._number(yaw, "yaw")
        angular_speed = self._number(angular_speed, "angular_speed")
        now = self._resolve_now(now)
        if now is None:
            return False
        with self._lock:
            if not self._serial(now):
                return self._fail_locked(now, "time moved backwards")
            self._yaw = yaw
            self._last_heading = now
            actions = []
            if self._machine.state in MOTION_STATES:
                self._relative = self._tracker.update(yaw)
                if self._machine.state == "SETTLING":
                    if self._settling.update(angular_speed, now):
                        self._leave_current_state(now)
                        self._machine.settled()
                        self._control_epoch += 1
                        self._state_entered["SCANNING"] = now
                        actions.append(("publish_state", "SCANNING"))
                        self._start_dwell(now, actions)
            epoch = self._control_epoch
        return self._emit(actions, epoch)

    def start(self, raw_json, now=None):
        with self._lock:
            if self._shutdown_latched:
                return False
        now = self._resolve_now(now)
        if now is None:
            return False
        try:
            request = parse_start_request(raw_json)
        except ProtocolError:
            self._emit([("publish_speed", 0.0)])
            return False
        with self._lock:
            if self._shutdown_latched:
                return False
            if not self._serial(now):
                return self._finish_error_locked(now, "time moved backwards")
            same = request.search_id == self._search
            if same and self._machine.state in self._machine.ACTIVE:
                actions = [("publish_state", self._machine.state)]
                if self._last_result:
                    actions.append(("publish_result", self._last_result))
                result = False
            elif same and self._last_result is not None:
                actions = [("publish_state", self._machine.state),
                           ("publish_result", self._last_result)]
                result = False
            elif self._machine.state not in self._machine.RESTARTABLE:
                actions, result = [], False
            elif self._yaw is None or self._last_heading is None or now - self._last_heading > self._heading_timeout:
                self._task, self._search = request.task_id, request.search_id
                return self._finish_error_locked(now, "heading unavailable")
            else:
                self._task, self._search = request.task_id, request.search_id
                self._control_epoch += 1
                self._machine.start()
                self._tracker.reset(self._yaw)
                self._relative = 0.0
                self._detected, self._resolved, self._resolve_errors = {}, {}, {}
                self._retry_edge_sent = False
                self._pass_index, self._station_index = 0, 0
                self._schedule = build_pass_angles(self._step_angle_deg, 0.0)
                self._target_rel = math.radians(self._schedule[0])
                self._settling.reset()
                self._settling_started = None
                self._started, self._last_frame_seen = now, None
                self._dwell_open = False
                self._dwell_started = None
                self._stations = []
                self._station = None
                self._prev_station = None
                self._state_seconds = {}
                self._state_entered = {"INITIAL_SCAN": now}
                searching = self._result(now, "searching", "", [])
                self._last_result = searching
                actions = [("publish_speed", 0.0), ("publish_result", searching),
                           ("publish_state", "INITIAL_SCAN")]
                self._start_dwell(now, actions)
                result = True
            epoch = self._control_epoch
        self._emit(actions, epoch)
        return result

    def tick(self, now=None):
        now = self._resolve_now(now)
        if now is None:
            return False
        with self._lock:
            if not self._serial(now):
                return self._finish_error_locked(now, "time moved backwards")
            state = self._machine.state
            if state not in self._machine.ACTIVE:
                actions = [("publish_speed", 0.0)]
            elif now - self._started >= self._total_timeout:
                return self._finish_not_found_locked(now, "search timed out")
            elif state in MOTION_STATES and (
                    self._last_heading is None or now - self._last_heading > self._heading_timeout):
                return self._finish_error_locked(now, "heading timed out")
            elif state in self._machine.ACTIVE and (
                    (self._last_frame_seen is None and now - self._started > self._camera_timeout) or
                    (self._last_frame_seen is not None and now - self._last_frame_seen > self._camera_timeout)):
                return self._finish_error_locked(now, "camera timed out")
            elif state == "WAITING_HTTP":
                actions = [("publish_speed", 0.0)]
            elif state in SCAN_STATES:
                if now - self._dwell_started >= self._scan_window:
                    self._end_dwell_metrics(now)
                    actions = self._advance_actions_locked(now)
                    if self._machine.state in self._machine.ACTIVE:
                        actions.append(("publish_scanner_control", self._scan_control()))
                else:
                    actions = [("publish_speed", 0.0)]
            elif state == "TURNING":
                error = self._target_rel - (self._relative or 0.0)
                speed, reached = staged_angular_command(
                    error, self._cruise, self._approach, self._approach_zone, self._tolerance)
                if reached:
                    self._leave_current_state(now)
                    self._machine.heading_reached()
                    self._control_epoch += 1
                    self._state_entered["SETTLING"] = now
                    self._settling.reset()
                    self._settling_started = now
                    actions = [("publish_speed", 0.0), ("publish_state", "SETTLING")]
                else:
                    actions = [("publish_speed", speed, self._control_epoch)]
            elif state == "SETTLING":
                actions = [("publish_speed", 0.0)]
                if now - self._settling_started >= self._settling_timeout:
                    return self._finish_error_locked(now, "settling timeout")
            else:
                actions = [("publish_speed", 0.0)]
            epoch = self._control_epoch
        ok = self._emit(actions, epoch)
        if not ok and any(action[0] == "publish_speed" and action[1] != 0 for action in actions):
            with self._lock:
                return self._finish_error_locked(now, "speed publisher failed")
        return ok

    def handle_scanner_event(self, raw_json, now=None):
        try:
            payload = json.loads(raw_json)
        except (TypeError, ValueError):
            now = self._resolve_now(now)
            if now is None:
                return False
            with self._lock:
                return self._finish_error_locked(now, "malformed scanner event")
        if not isinstance(payload, dict):
            now = self._resolve_now(now)
            if now is None:
                return False
            with self._lock:
                return self._finish_error_locked(now, "malformed scanner event")
        now = self._resolve_now(now)
        if now is None:
            return False
        with self._lock:
            session = payload.get("scanner_session")
            valid_session = isinstance(session, str) and bool(session.strip())
            previous_session = self._scanner_session
            normalized_session = session.strip() if valid_session else None
            session_fault = None
            if not valid_session:
                if self._machine.state in self._machine.ACTIVE:
                    session_fault = "malformed scanner event"
            else:
                self._scanner_session = normalized_session
                if (previous_session is not None and previous_session != normalized_session
                        and self._machine.state in self._machine.ACTIVE):
                    session_fault = "scanner restarted"

            if payload.get("event") == "scanner_started":
                valid_hello = (type(payload.get("protocol_version")) is int
                               and payload.get("protocol_version") == 1 and valid_session)
                if not self._serial(now):
                    return self._finish_error_locked(now, "time moved backwards")
                if not valid_hello:
                    if self._machine.state in self._machine.ACTIVE:
                        return self._finish_error_locked(now, "malformed scanner event")
                    return False
                if session_fault is not None:
                    return self._finish_error_locked(now, session_fault)
                return False

            task_id, search_id = payload.get("task_id"), payload.get("search_id")
            valid_identity = (
                isinstance(task_id, str) and bool(task_id.strip()) and
                isinstance(search_id, str) and bool(search_id.strip())
            )
            malformed_identity = not valid_identity and self._machine.state in self._machine.ACTIVE
            if (session_fault is None and valid_identity
                    and (task_id != self._task or search_id != self._search)):
                return False
            if (session_fault is None and valid_identity
                    and self._machine.state not in self._machine.ACTIVE):
                return False
            if not self._serial(now):
                return self._finish_error_locked(now, "time moved backwards")
            if session_fault is not None:
                return self._finish_error_locked(now, session_fault)
            if malformed_identity:
                return self._finish_error_locked(now, "malformed scanner event")
            if payload.get("protocol_version") != 1 or not isinstance(payload.get("event"), str):
                return self._finish_error_locked(now, "malformed scanner event")
            try:
                actions = self._scanner_event_actions_locked(payload, now)
            except Exception:
                return self._finish_error_locked(now, "malformed scanner event")
            if actions is None:
                return False
            epoch = self._control_epoch
        self._emit(actions, epoch)
        return True

    def stop(self, raw_json, now=None):
        try:
            request = parse_stop_request(raw_json)
        except ProtocolError:
            return False
        with self._lock:
            if request.task_id != self._task or request.search_id != self._search:
                return False
        now = self._resolve_now(now)
        if now is None:
            return False
        with self._lock:
            if not self._serial(now):
                return self._finish_error_locked(now, "time moved backwards")
            self._leave_current_state(now)
            self._machine.stop()
            self._control_epoch += 1
            self._state_entered["STOPPED"] = now
            self._end_dwell_metrics(now)
            result = self._result(now, "stopped", request.reason, self._partial_items())
            self._last_result = result
            self._publish_zero_before_metrics_locked()
            self._emit_metrics(now, "stopped", request.reason)
            actions = self._terminal_actions(result)
            epoch = self._control_epoch
        self._emit(actions, epoch)
        return True

    def shutdown(self):
        with self._lock:
            self._shutdown_latched = True
            self._control_epoch += 1
            epoch = self._control_epoch
            was_active = self._machine.state in self._machine.ACTIVE
            if was_active:
                self._leave_current_state(self._last_now or 0.0)
                self._machine.fail()
                self._state_entered["ERROR"] = self._last_now or 0.0
                self._end_dwell_metrics(self._last_now or 0.0)
            actions = [
                ("publish_speed", 0.0),
                ("publish_scanner_control", self._control(False, False, False)),
                ("publish_state", self._machine.state),
            ]
            self._publish_zero_before_metrics_locked()
            if was_active and self._task:
                stamp = self._last_now if self._last_now is not None else 0.0
                result = self._result(stamp, "error", "controller shutdown", self._partial_items())
                self._last_result = result
                self._emit_metrics(stamp, "error", "controller shutdown")
                actions.append(("publish_result", result))
        self._emit(actions, epoch)

    def _scanner_event_actions_locked(self, p, now):
        kind = p["event"]
        if kind == "frame_seen":
            self._last_frame_seen = now
            return []
        if kind == "quality":
            b, o, s, y = (self._number(p[key], key)
                          for key in ("brightness", "overexposed", "sharpness", "detected_yaw"))
            if not isinstance(p.get("decoded"), bool):
                raise ValueError()
            self._last_quality = {"brightness": b, "overexposed": o, "sharpness": s}
            if self._dwell_open:
                self._dwell_frame_count += 1
                self._dwell_quality = dict(self._last_quality)
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
            if existing and existing != candidate:
                raise ValueError()
            if any(item["url"] == url and key != order for key, item in self._detected.items()):
                raise ValueError()
            self._detected[order] = candidate
            self._credit_decoded_url(url)
            if len(self._detected) == 3:
                self._leave_current_state(now)
                self._machine.urls_collected()
                self._control_epoch += 1
                self._state_entered["WAITING_HTTP"] = now
                self._end_dwell_metrics(now)
                actions = [("publish_speed", 0.0), ("publish_state", "WAITING_HTTP"),
                           ("publish_scanner_control", self._scan_control())]
                if self._resolve_errors and not self._retry_edge_sent:
                    self._retry_edge_sent = True
                    actions.append(("publish_scanner_control", self._scan_control(retry=True)))
                return actions
            return []
        if kind in ("resolved", "resolve_error"):
            order, url, yaw = self._item_identity(p)
            detected = self._detected.get(order)
            if detected is None or detected["url"] != url or detected["detected_yaw"] != yaw:
                raise ValueError()
            if kind == "resolved":
                name = self._text(p.get("item_name"), "item_name")
                self._resolved[order] = name
                self._resolve_errors.pop(order, None)
                if len(self._resolved) == 3:
                    self._leave_current_state(now)
                    self._machine.items_resolved()
                    self._control_epoch += 1
                    self._state_entered["COMPLETE"] = now
                    self._end_dwell_metrics(now)
                    result = self._result(now, "complete", "", self._complete_items())
                    self._last_result = result
                    self._publish_zero_before_metrics_locked()
                    self._emit_metrics(now, "complete", "")
                    return self._terminal_actions(result)
            else:
                message = self._text(p.get("message"), "message")
                self._resolve_errors[order] = message
                if self._machine.state == "WAITING_HTTP" and not self._retry_edge_sent:
                    self._retry_edge_sent = True
                    return [("publish_scanner_control", self._scan_control(retry=True))]
            return []
        raise ValueError()

    def _start_dwell(self, now, actions):
        if self._prev_station is not None:
            self._stations.append(self._prev_station)
            self._prev_station = None
        self._dwell_started = now
        self._dwell_frame_count = 0
        self._dwell_quality = None
        self._dwell_open = True
        target_deg = self._schedule[self._station_index]
        actual_deg = math.degrees(self._relative or 0.0) % 360.0
        settling_seconds = None
        if self._settling_started is not None:
            settling_seconds = round(now - self._settling_started, 3)
        self._station = {
            "pass": self._pass_index,
            "index": self._station_index,
            "target_yaw_deg": target_deg,
            "actual_yaw_deg": round(actual_deg, 3),
            "yaw_error_deg": round(math.degrees(
                normalize_angle(math.radians(target_deg - actual_deg))), 3),
            "settling_seconds": settling_seconds,
            "dwell_seconds": None,
            "frame_count": 0,
            "quality": None,
            "decoded_url": None,
        }
        actions.append(("publish_scanner_control", self._scan_control()))

    def _end_dwell_metrics(self, now):
        if self._dwell_open:
            self._dwell_open = False
            if self._station is not None:
                self._station["dwell_seconds"] = round(now - self._dwell_started, 3)
                self._station["frame_count"] = self._dwell_frame_count
                self._station["quality"] = self._dwell_quality
                self._prev_station = self._station
                self._station = None
        if self._prev_station is not None:
            self._stations.append(self._prev_station)
            self._prev_station = None

    def _credit_decoded_url(self, url):
        if self._station is not None and self._station["decoded_url"] is None:
            self._station["decoded_url"] = url
            return
        if self._prev_station is not None and self._prev_station["decoded_url"] is None:
            self._prev_station["decoded_url"] = url

    def _advance_actions_locked(self, now):
        if len(self._detected) >= 3:
            self._leave_current_state(now)
            self._machine.urls_collected()
            self._control_epoch += 1
            self._state_entered["WAITING_HTTP"] = now
            actions = [("publish_speed", 0.0), ("publish_state", "WAITING_HTTP"),
                       ("publish_scanner_control", self._scan_control())]
            if self._resolve_errors and not self._retry_edge_sent:
                self._retry_edge_sent = True
                actions.append(("publish_scanner_control", self._scan_control(retry=True)))
            return actions
        if self._station_index + 1 < len(self._schedule):
            self._leave_current_state(now)
            self._machine.scan_finished()
            self._station_index += 1
            self._target_rel = self._station_target(self._pass_index, self._station_index)
            self._control_epoch += 1
            self._state_entered["TURNING"] = now
            return [("publish_state", "TURNING")]
        if self._pass_index < self._max_passes - 1:
            self._leave_current_state(now)
            self._machine.pass_finished()
            self._pass_index += 1
            self._station_index = 0
            self._schedule = build_pass_angles(self._step_angle_deg, self._offset_angle_deg)
            self._target_rel = self._station_target(self._pass_index, 0)
            self._machine.offset_pass_started()
            self._control_epoch += 1
            self._state_entered["TURNING"] = now
            return [("publish_state", "TURNING")]
        self._leave_current_state(now)
        self._machine.scan_exhausted()
        self._control_epoch += 1
        self._state_entered["NOT_FOUND"] = now
        result = self._result(now, "not_found", "items not found", self._partial_items())
        self._last_result = result
        self._emit_metrics(now, "not_found", "items not found")
        return self._terminal_actions(result)

    def _station_target(self, pass_index, station_index):
        return math.radians(pass_index * 360.0 + self._schedule[station_index])

    def _finish_not_found_locked(self, now, message):
        if self._machine.state in self._machine.ACTIVE:
            self._leave_current_state(now)
            self._machine.timeout()
            self._state_entered["NOT_FOUND"] = now
        if self._resolve_errors and self._machine.state == "NOT_FOUND":
            details = "; ".join("%s (%s)" % (url, msg)
                                for url, msg in sorted(self._resolve_errors.items()))
            message = "items unresolved: " + details
        self._control_epoch += 1
        self._end_dwell_metrics(now)
        result = self._result(now, "not_found", message, self._partial_items())
        self._last_result = result
        self._publish_zero_before_metrics_locked()
        self._emit_metrics(now, "not_found", message)
        actions = self._terminal_actions(result)
        self._emit_without_state_lock(actions, self._control_epoch)
        return False

    def _finish_error_locked(self, now, message):
        self._leave_current_state(now)
        self._machine.fail()
        self._control_epoch += 1
        self._state_entered["ERROR"] = now
        self._end_dwell_metrics(now)
        items = self._partial_items() if self._task else []
        actions = [("publish_speed", 0.0), ("publish_state", "ERROR"),
                   ("publish_scanner_control", self._control(False, False, False))]
        if self._task:
            result = self._result(now, "error", message, items)
            self._last_result = result
            actions.append(("publish_result", result))
        self._publish_zero_before_metrics_locked()
        self._emit_metrics(now, "error", message)
        self._emit_without_state_lock(actions, self._control_epoch)
        return False

    def _fail_locked(self, now, message):
        return self._finish_error_locked(now, message)

    def _leave_current_state(self, now):
        state = self._machine.state
        entered = self._state_entered.get(state, now)
        elapsed = max(0.0, now - entered)
        self._state_seconds[state] = self._state_seconds.get(state, 0.0) + elapsed

    def _run_record(self, now, status, message):
        state_seconds = dict(self._state_seconds)
        state = self._machine.state
        entered = self._state_entered.get(state)
        if entered is not None:
            state_seconds[state] = state_seconds.get(state, 0.0) + max(0.0, now - entered)
        return {
            "schema": "qr_stop_scan/v1",
            "task_id": self._task,
            "search_id": self._search,
            "config": {
                "step_angle_deg": self._step_angle_deg,
                "cruise_angular_speed": self._cruise,
                "approach_angular_speed": self._approach,
                "approach_zone_deg": round(math.degrees(self._approach_zone), 4),
                "yaw_tolerance_deg": round(math.degrees(self._tolerance), 4),
                "settled_angular_speed": self._settling_speed,
                "settled_duration": self._settling_duration,
                "scan_window": self._scan_window,
                "offset_angle_deg": self._offset_angle_deg,
                "max_passes": self._max_passes,
                "search_total_timeout": self._total_timeout,
                "settling_timeout": self._settling_timeout,
                "heading_timeout": self._heading_timeout,
                "camera_timeout": self._camera_timeout,
            },
            "started_at": self._started,
            "finished_at": now,
            "total_seconds": round(now - self._started, 3) if self._started is not None else None,
            "terminal_status": status,
            "message": message,
            "state_seconds": {key: round(value, 3) for key, value in state_seconds.items()},
            "stations": list(self._stations),
            "items": self._partial_items(),
        }

    def _emit_metrics(self, now, status, message):
        if self._metrics is None or self._task is None:
            return
        try:
            self._metrics(self._run_record(now, status, message))
        except Exception as error:
            self._report(error)

    def _publish_zero_before_metrics_locked(self):
        """Make terminal safety independent from optional filesystem I/O."""
        self._emit_without_state_lock(
            [("publish_speed", 0.0)], self._control_epoch)

    def _terminal_actions(self, result):
        return [("publish_speed", 0.0), ("publish_state", self._machine.state),
                ("publish_scanner_control", self._control(False, False, False)),
                ("publish_result", result)]

    def _control(self, enabled, enhanced, retry=False, pass_index=None, station_index=None):
        result = {"protocol_version": 1, "enabled": enabled, "enhanced": enhanced,
                  "detected_yaw": self._relative or 0.0, "retry_failed": retry}
        if pass_index is not None:
            result["pass_index"] = pass_index
            result["station_index"] = station_index
        result.update(task_id=self._task or "", search_id=self._search or "")
        return result

    def _scan_control(self, retry=False):
        return self._control(True, True, retry=retry,
                             pass_index=self._pass_index, station_index=self._station_index)

    def _result(self, now, status, message, items):
        return build_search_result(self._task, self._search, now, status, items, message)

    def _complete_items(self):
        return [dict(self._detected[i], item_name=self._resolved[i]) for i in (1, 2, 3)]

    def _partial_items(self):
        items = []
        found_orders = [order for order in sorted(self._detected) if order in self._resolved]
        for new_order, old_order in enumerate(found_orders, 1):
            value = self._detected[old_order]
            items.append({"order": new_order, "url": value["url"],
                          "detected_yaw": value["detected_yaw"],
                          "item_name": self._resolved[old_order]})
        return items

    def _item_identity(self, p):
        order = p.get("order")
        if type(order) is not int or not 1 <= order <= 3:
            raise ValueError()
        return order, self._text(p.get("url"), "url"), self._number(p.get("detected_yaw"), "detected_yaw")

    def _resolve_now(self, now):
        if now is not None:
            return self._number(now, "now")
        try:
            return self._number(self._clock(), "clock")
        except Exception as error:
            self._report(error)
            with self._lock:
                self._finish_error_locked(self._last_now or 0.0, "clock failed")
            return None

    def _serial(self, now):
        if self._last_now is not None and now < self._last_now:
            return False
        self._last_now = now
        return True

    def _emit(self, actions, expected_epoch=None):
        if not actions:
            return True
        if expected_epoch is None:
            with self._lock:
                expected_epoch = self._control_epoch
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
                    if not self._emit_speed_guarded(value, speed_epoch):
                        ok = False
                    continue
                try:
                    getattr(self._outputs, name)(value)
                except Exception as error:
                    ok = False
                    self._report(error)
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
                            self._machine.state != "TURNING"):
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
            self._leave_current_state(self._last_now or 0.0)
            self._machine.fail()
            self._control_epoch += 1
            self._state_entered["ERROR"] = self._last_now or 0.0
            epoch = self._control_epoch
            actions = [("publish_speed", 0.0),
                       ("publish_scanner_control", self._control(False, False, False)),
                       ("publish_state", "ERROR")]
            if self._task:
                result = self._result(self._last_now or 0.0, "error", message, self._partial_items())
                self._last_result = result
                self._emit_metrics(self._last_now or 0.0, "error", message)
                actions.append(("publish_result", result))
        self._emit(actions, epoch)

    def _report(self, error):
        try:
            self._errors(error)
        except Exception:
            pass

    @staticmethod
    def _finite(value):
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)

    @classmethod
    def _number(cls, value, name):
        if not cls._finite(value):
            raise ValueError("%s must be finite" % name)
        return float(value)

    @staticmethod
    def _text(value, name):
        if not isinstance(value, str) or not value.strip():
            raise ValueError("%s must be non-empty" % name)
        return value.strip()
