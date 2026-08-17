"""Reusable dual-phase delivery mission engine (ROS-free).

One engine instance serves both the physical and the simulation phase. A
phase is started with accept_goal(); every incoming event carries its phase
and is ignored when it does not match the active phase. All failure, timeout
and cancellation paths transition through SAFE_STOP, emit a guaranteed
zero-velocity command and only then reach a terminal state.
"""

import math

from ucar_delivery import protocol


class DeliveryMission:
    IDLE = "IDLE"
    ACCEPT_GOAL = "ACCEPT_GOAL"
    NAVIGATE_VIEWPOINT = "NAVIGATE_VIEWPOINT"
    SEARCH_SIGN = "SEARCH_SIGN"
    ALIGN_SIGN = "ALIGN_SIGN"
    ESTIMATE_STAGING_POSE = "ESTIMATE_STAGING_POSE"
    NAVIGATE_STAGING_POSE = "NAVIGATE_STAGING_POSE"
    ACQUIRE_FRAME = "ACQUIRE_FRAME"
    ALIGN_FRAME = "ALIGN_FRAME"
    CENTER_FRAME = "CENTER_FRAME"
    APPROACH_FRAME = "APPROACH_FRAME"
    FINAL_STOP = "FINAL_STOP"
    VERIFY_STOP = "VERIFY_STOP"
    ARRIVED = "ARRIVED"
    SAFE_STOP = "SAFE_STOP"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    TIMEOUT = "TIMEOUT"

    _ACTIVE_STATES = frozenset(
        (
            NAVIGATE_VIEWPOINT,
            SEARCH_SIGN,
            ALIGN_SIGN,
            ESTIMATE_STAGING_POSE,
            NAVIGATE_STAGING_POSE,
            ACQUIRE_FRAME,
            ALIGN_FRAME,
            CENTER_FRAME,
            APPROACH_FRAME,
            FINAL_STOP,
            VERIFY_STOP,
        )
    )
    _PARKING_STATES = frozenset(
        (
            SEARCH_SIGN,
            ALIGN_SIGN,
            ACQUIRE_FRAME,
            ALIGN_FRAME,
            CENTER_FRAME,
            APPROACH_FRAME,
            FINAL_STOP,
            VERIFY_STOP,
        )
    )
    _FAILED_TERMINALS = (FAILED, CANCELLED, TIMEOUT)
    _TERMINAL_STATES = frozenset((ARRIVED,) + _FAILED_TERMINALS)

    _TIMEOUT_KEYS = {
        NAVIGATE_VIEWPOINT: "navigation",
        SEARCH_SIGN: "sign_search",
        ALIGN_SIGN: "sign_align",
        ESTIMATE_STAGING_POSE: "staging_estimate",
        NAVIGATE_STAGING_POSE: "staging_navigation",
        ACQUIRE_FRAME: "frame_acquire",
        ALIGN_FRAME: "frame_align",
        CENTER_FRAME: "frame_center",
        APPROACH_FRAME: "approach",
        FINAL_STOP: "final_stop",
        VERIFY_STOP: "verify",
        SAFE_STOP: "safe_stop",
    }

    DEFAULT_CONFIG = {
        "viewpoint_count": 3,
        "viewpoint_max_retries": 2,
        "staging_estimation_max_retries": 3,
        "staging_navigation_max_retries": 2,
    }

    def __init__(self, outputs, clock, timeouts, config=None):
        self._outputs = outputs
        self._clock = clock
        self._timeouts = dict(timeouts)
        merged = dict(self.DEFAULT_CONFIG)
        merged.update(config or {})
        self._config = merged
        self.state = self.IDLE
        self.phase = None
        self.goal = None
        self.deadline = None
        self.viewpoint_index = 0
        self.viewpoint_attempt = 0
        self.detection = None
        self.last_observation = None
        self.last_stop_reason = None
        self.last_status = None
        # 最近一个终端目标的身份 (phase, task_id, goal_id)：重复投递该目标
        # 只重发终端状态/结果，绝不重新移动。
        self._terminal_identity = None

    @property
    def visual_parking_active(self):
        return self.state in self._PARKING_STATES

    def _emit(self, action, payload):
        self._outputs.append((action, payload))

    def _transition(self, state):
        self.state = state
        timeout_key = self._TIMEOUT_KEYS.get(state)
        if timeout_key is None:
            self.deadline = None
        else:
            self.deadline = self._clock() + self._timeouts[timeout_key]
        mode = self._motion_mode_for_state(state)
        if mode is not None:
            self._emit("publish_motion_mode", mode)

    def _motion_mode_for_state(self, state):
        if state in (self.NAVIGATE_VIEWPOINT, self.NAVIGATE_STAGING_POSE):
            return "NAVIGATION"
        if state in (self.SEARCH_SIGN, self.ALIGN_SIGN,
                     self.ESTIMATE_STAGING_POSE):
            return "VISUAL_SEARCH"
        if state in self._PARKING_STATES:
            return "PARKING"
        if state in self._TERMINAL_STATES or state == self.SAFE_STOP:
            return "IDLE"
        return None

    def _publish_status(self, status, message="", remember=True):
        payload = {
            "protocol_version": protocol.PROTOCOL_VERSION,
            "phase": self.phase,
            "task_id": self.goal["task_id"],
            "goal_id": self.goal["goal_id"],
            "state": self.state,
            "status": status,
            "message": message,
        }
        if remember:
            self.last_status = dict(payload)
        self._emit("publish_status", payload)

    def _republish_status(self):
        if self.last_status is not None:
            self._emit("publish_status", dict(self.last_status))

    def _matches_phase(self, phase):
        return phase == self.phase

    def _require_active(self, phase):
        return self._matches_phase(phase) and self.state in self._ACTIVE_STATES

    # ------------------------------------------------------------------ goal

    def accept_goal(self, goal):
        if goal.get("phase") not in protocol.PHASES:
            raise protocol.ProtocolError(
                "unsupported delivery phase: %s" % goal.get("phase")
            )
        target = protocol.ALLOWED_WORKSHOPS
        if goal.get("target_workshop") not in target:
            raise protocol.ProtocolError(
                "unsupported target_workshop: %s" % goal.get("target_workshop")
            )
        if self.goal is not None and self.state not in self._TERMINAL_STATES:
            # 活动任务：相同 goal_id 只重发状态；不同 goal 被忽略
            if self.goal.get("goal_id") == goal.get("goal_id"):
                self._republish_status()
            return
        if self.state in self._TERMINAL_STATES:
            # 终端任务：相同身份只重发终端状态/结果（绝不移动）；
            # 不同的 (phase, task_id, goal_id) 走重置路径并正常启动。
            identity = (goal["phase"], goal.get("task_id"), goal.get("goal_id"))
            if identity == self._terminal_identity:
                self._republish_status()
                return
        self._reset_mission(goal)
        self._transition(self.ACCEPT_GOAL)
        self._transition(self.NAVIGATE_VIEWPOINT)
        self._publish_status("running")
        self._start_viewpoint()

    def _reset_mission(self, goal):
        self.phase = goal["phase"]
        self.goal = goal
        self.detection = None
        self.last_observation = None
        self.last_stop_reason = None
        self.viewpoint_index = 0
        self.viewpoint_attempt = 0
        self.staging_pose = None
        self.staging_estimate_attempt = 0
        self.staging_navigation_attempt = 0
        self.last_status = None

    def reset(self):
        if self.state not in self._TERMINAL_STATES:
            return
        self.state = self.IDLE
        self.phase = None
        self.goal = None
        self.deadline = None
        self.last_status = None
        self._terminal_identity = None

    # ------------------------------------------------------------- viewpoint

    def _start_viewpoint(self):
        self.viewpoint_attempt += 1
        self._emit(
            "navigate_to_viewpoint",
            {
                "protocol_version": protocol.PROTOCOL_VERSION,
                "phase": self.phase,
                "task_id": self.goal["task_id"],
                "goal_id": self.goal["goal_id"],
                "viewpoint_index": self.viewpoint_index,
                "attempt": self.viewpoint_attempt,
            },
        )

    def _next_viewpoint_or_fail(self, message="sign not found at any viewpoint"):
        if self.viewpoint_index + 1 < int(self._config["viewpoint_count"]):
            self.viewpoint_index += 1
            self.viewpoint_attempt = 0
            self.staging_pose = None
            self.staging_estimate_attempt = 0
            self.staging_navigation_attempt = 0
            self._transition(self.NAVIGATE_VIEWPOINT)
            self._publish_status("running")
            self._start_viewpoint()
            return True
        self.fail(message)
        return False

    def on_viewpoint_result(self, phase, ok, message):
        if not self._require_active(phase):
            return
        if self.state != self.NAVIGATE_VIEWPOINT:
            return
        if ok:
            self._transition(self.SEARCH_SIGN)
            self._publish_status("running")
            self._emit("cancel_navigation", {"reason": "viewpoint reached"})
            self._emit("start_sign_search", self._context())
            return
        retry_budget = int(self._config["viewpoint_max_retries"])
        if self.viewpoint_attempt < retry_budget:
            self._transition(self.NAVIGATE_VIEWPOINT)
            self._start_viewpoint()
            return
        self.fail(
            "navigation to viewpoint %d failed after %d attempts%s"
            % (
                self.viewpoint_index,
                self.viewpoint_attempt,
                ": %s" % message if message else "",
            )
        )

    # ----------------------------------------------------------------- sign

    def _context(self):
        return {
            "protocol_version": protocol.PROTOCOL_VERSION,
            "phase": self.phase,
            "task_id": self.goal["task_id"],
            "goal_id": self.goal["goal_id"],
        }

    def on_sign_found(self, phase, detection):
        if not self._require_active(phase):
            return
        if self.state not in (self.SEARCH_SIGN, self.NAVIGATE_VIEWPOINT):
            return
        self.detection = dict(detection)
        if self.state == self.NAVIGATE_VIEWPOINT:
            self._emit("cancel_navigation", {"reason": "sign detected"})
        self._emit("stop_sign_search", self._context())
        self._transition(self.ALIGN_SIGN)
        self._publish_status("running")
        self._emit("align_sign", dict(self._context(), detection=self.detection))

    def on_sign_aligned(self, phase):
        if not self._require_active(phase):
            return
        if self.state != self.ALIGN_SIGN:
            return
        self._transition(self.ESTIMATE_STAGING_POSE)
        self._publish_status("running")
        self._emit("start_staging_estimate", self._context())

    # -------------------------------------------------------------- staging

    def _finite_pose(self, pose):
        if not isinstance(pose, dict):
            return False
        try:
            values = (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))
        except (KeyError, TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values)

    def on_staging_pose_estimated(self, phase, pose):
        if not self._require_active(phase):
            return
        if self.state != self.ESTIMATE_STAGING_POSE:
            return
        # 动态点未通过节点层一致性确认或字段不合法时，禁止发送 staging goal。
        if not self._finite_pose(pose):
            return
        self.staging_pose = dict(pose)
        self._transition(self.NAVIGATE_STAGING_POSE)
        self._publish_status("running")
        self._emit(
            "navigate_to_staging",
            dict(self._context(), pose=dict(self.staging_pose)),
        )

    def on_staging_estimate_failed(self, phase, reason):
        if not self._require_active(phase):
            return
        if self.state != self.ESTIMATE_STAGING_POSE:
            return
        budget = int(self._config["staging_estimation_max_retries"])
        if self.staging_estimate_attempt < budget:
            # 规则 3：在当前点重新对正并采样，最多 staging_estimation_max_retries 次。
            self.staging_estimate_attempt += 1
            self._transition(self.ALIGN_SIGN)
            self._publish_status("running")
            self._emit(
                "align_sign",
                dict(self._context(), detection=self.detection),
            )
            return
        # 规则 4：当前点估计重试耗尽，换下一个搜索点，不得盲目前进。
        self._next_viewpoint_or_fail(
            "staging estimation failed at viewpoint %d"
            % self.viewpoint_index
        )

    def on_staging_navigation_result(self, phase, ok, message):
        if not self._require_active(phase):
            return
        if self.state != self.NAVIGATE_STAGING_POSE:
            return
        if ok:
            self._transition(self.ACQUIRE_FRAME)
            self._publish_status("running")
            self._emit("start_frame_search", self._context())
            return
        budget = int(self._config["staging_navigation_max_retries"])
        if self.staging_navigation_attempt < budget:
            # 规则 5：允许 staging_navigation_max_retries 次，耗尽后安全失败。
            self.staging_navigation_attempt += 1
            self._transition(self.NAVIGATE_STAGING_POSE)
            self._emit(
                "navigate_to_staging",
                dict(self._context(), pose=dict(self.staging_pose or {})),
            )
            return
        self.fail(
            "navigation to staging pose failed after %d attempts%s"
            % (
                self.staging_navigation_attempt,
                ": %s" % message if message else "",
            )
        )

    # ---------------------------------------------------------------- frame

    def _on_frame_event(self, phase, current_state, observation, action,
                        next_state):
        if not self._require_active(phase):
            return
        if self.state != current_state:
            return
        self.last_observation = dict(observation)
        self._transition(next_state)
        self._publish_status("running")
        self._emit(action, dict(self._context(), observation=self.last_observation))

    def on_frame_acquired(self, phase, observation):
        self._on_frame_event(
            phase, self.ACQUIRE_FRAME, observation, "align_frame",
            self.ALIGN_FRAME,
        )

    def on_frame_aligned(self, phase, observation):
        self._on_frame_event(
            phase, self.ALIGN_FRAME, observation, "center_frame",
            self.CENTER_FRAME,
        )

    def on_frame_centered(self, phase, observation):
        self._on_frame_event(
            phase, self.CENTER_FRAME, observation, "start_approach",
            self.APPROACH_FRAME,
        )

    def on_approach_complete(self, phase, observation):
        self._on_frame_event(
            phase, self.APPROACH_FRAME, observation, "final_stop",
            self.FINAL_STOP,
        )

    def on_final_stop_done(self, phase, observation):
        self._on_frame_event(
            phase, self.FINAL_STOP, observation, "verify_stop",
            self.VERIFY_STOP,
        )

    def on_stop_verified(self, phase, observation):
        if not self._require_active(phase):
            return
        if self.state != self.VERIFY_STOP:
            return
        self.last_observation = dict(observation)
        self._transition(self.ARRIVED)
        self._terminal_identity = (
            self.phase, self.goal["task_id"], self.goal["goal_id"]
        )
        self._publish_status("arrived")
        arrival = protocol.build_arrival(
            self.phase,
            self.goal["task_id"],
            self.goal["goal_id"],
            "arrived",
            "",
        )
        self._emit("publish_arrival", arrival)

    def on_lidar_danger(self, phase, front_range):
        if not self._require_active(phase):
            return
        self._safe_stop("lidar_safety", self.FAILED)

    # ----------------------------------------------------------- termination

    def _safe_stop(self, reason, terminal_state):
        if self.state == self.SAFE_STOP:
            return
        self.last_stop_reason = reason
        self._transition(self.SAFE_STOP)
        self._emit("publish_zero_velocity", self._context())
        self._emit("publish_safe_stop", dict(self._context(), reason=reason))
        self._terminate(terminal_state, reason)

    def _terminate(self, terminal_state, reason):
        self._transition(terminal_state)
        self._terminal_identity = (
            self.phase, self.goal["task_id"], self.goal["goal_id"]
        )
        self._publish_status("failed", reason)
        arrival = protocol.build_arrival(
            self.phase,
            self.goal["task_id"],
            self.goal["goal_id"],
            "failed",
            reason,
        )
        self._emit("publish_arrival", arrival)

    def fail(self, reason):
        if self.state not in self._ACTIVE_STATES:
            return
        self._safe_stop(reason, self.FAILED)

    def cancel(self, reason):
        if self.state not in self._ACTIVE_STATES:
            return
        self._safe_stop(reason or "cancelled", self.CANCELLED)

    def tick(self):
        if self.state not in self._ACTIVE_STATES or self.deadline is None:
            return
        if self._clock() < self.deadline:
            return
        if self.state == self.SAFE_STOP:
            self._terminate(self.TIMEOUT, "safe stop timed out")
            return
        timed_out_state = self.state
        if self.state == self.SEARCH_SIGN:
            self.last_stop_reason = "sign_search_timeout"
            if self._next_viewpoint_or_fail():
                self._publish_status(
                    "running",
                    "no sign at viewpoint %d" % self.viewpoint_index,
                    remember=False,
                )
            return
        self._safe_stop(
            "timeout in state %s" % timed_out_state, self.TIMEOUT
        )
