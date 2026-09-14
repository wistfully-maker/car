"""move_base navigation supervisor (ROS-free).

Owns one navigation attempt's action lifecycle: goal sending, success,
preemption and abort handling, bounded cancellation, action timeouts, target
detection interruption and a guaranteed zero-velocity exit on every terminal
path. The ROS node adapts actionlib callbacks to this class.
"""


class NavSupervisor:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    SETTLING = "SETTLING"
    CANCELLING = "CANCELLING"
    COMPLETE = "COMPLETE"

    REQUIRED_TIMEOUTS = ("action_timeout", "cancel_timeout", "settle_time")

    def __init__(self, outputs, clock, timeouts, config):
        self._outputs = outputs
        self._clock = clock
        for key in self.REQUIRED_TIMEOUTS:
            if key not in timeouts:
                raise ValueError("missing supervisor timeout: %s" % key)
        self._action_timeout = float(timeouts["action_timeout"])
        self._cancel_timeout = float(timeouts["cancel_timeout"])
        self._settle_time = float(timeouts["settle_time"])
        config = config or {}
        self._settle_timeout = float(config.get("settle_timeout", 5.0))
        self._odom_timeout = float(config.get("odom_timeout", 0.5))
        self._settle_velocity_threshold = float(
            config.get("settle_velocity_threshold", 0.02)
        )
        self._settle_angular_threshold = float(
            config.get("settle_angular_threshold", 0.05)
        )
        self._settle_stable_duration = float(
            config.get("settle_stable_duration", 0.5)
        )
        self.state = self.IDLE
        self._payload = None
        self._deadline = None
        self._cancel_reason = None
        self._settle_not_before = None
        self._stable_since = None
        self._odom_velocity = None
        self._odom_stamp = None

    def _emit(self, action, payload):
        self._outputs.append((action, payload))

    def begin(self, payload):
        if self.state not in (self.IDLE, self.COMPLETE):
            return
        self._payload = dict(payload)
        # 会话必须携带目标种类：mission 节点按 goal_kind 路由会话结果。
        self._payload.setdefault("goal_kind", "viewpoint")
        self._cancel_reason = None
        self._settle_not_before = None
        self._stable_since = None
        self._odom_velocity = None
        self._odom_stamp = None
        self.state = self.NAVIGATING
        self._deadline = self._clock() + self._action_timeout
        self._emit("send_goal", self._payload)

    def _finish(self, status, message, ok, retryable):
        self.state = self.COMPLETE
        self._deadline = None
        self._settle_not_before = None
        self._stable_since = None
        self._emit("publish_zero_velocity", self._context())
        self._emit(
            "session_result",
            dict(
                self._context(),
                status=status,
                ok=ok,
                message=message,
                retryable=retryable,
            ),
        )

    def _context(self):
        payload = self._payload or {}
        return {
            "phase": payload.get("phase"),
            "task_id": payload.get("task_id"),
            "goal_id": payload.get("goal_id"),
            "viewpoint_index": payload.get("viewpoint_index"),
            "attempt": payload.get("attempt"),
            # "viewpoint"（搜索点）或 "staging"（动态观察点）：会话结果
            # 由 mission 节点按此路由到不同回调。
            "goal_kind": payload.get("goal_kind", "viewpoint"),
        }

    def _cancel_pending(self, status, message, retryable):
        return (status, message, retryable)

    def on_status(self, status, now, message=""):
        if self.state == self.NAVIGATING:
            if status in ("PENDING", "ACTIVE"):
                self._deadline = now + self._action_timeout
                return
            if status == "SUCCEEDED":
                self.state = self.SETTLING
                self._settle_not_before = now + self._settle_time
                if self._is_staging():
                    self._deadline = now + self._settle_timeout
                    self._stable_since = None
                else:
                    self._deadline = self._settle_not_before
                return
            if status == "ABORTED":
                self._finish("aborted", message, False, True)
                return
            if status == "PREEMPTED":
                self._finish("preempted", message or "navigation preempted",
                             False, True)
                return
            return
        if self.state == self.CANCELLING:
            if status in ("SUCCEEDED", "ABORTED", "PREEMPTED"):
                reason = self._cancel_reason or ("cancelled", "cancelled", False)
                self._finish(reason[0], reason[1], False, reason[2])
            return
        if self.state == self.SETTLING:
            if status in ("SUCCEEDED",):
                return
        # Unknown statuses and out-of-state events are ignored.

    def _is_staging(self):
        return (self._payload or {}).get("goal_kind") == "staging"

    def _refresh_staging_stability(self, now):
        if self._odom_velocity is None or self._odom_stamp is None:
            self._stable_since = None
            return False
        age = now - self._odom_stamp
        vx, vy, vth = self._odom_velocity
        stable = (
            0.0 <= age <= self._odom_timeout
            and abs(vx) <= self._settle_velocity_threshold
            and abs(vy) <= self._settle_velocity_threshold
            and abs(vth) <= self._settle_angular_threshold
        )
        if stable:
            if self._stable_since is None:
                self._stable_since = now
        else:
            self._stable_since = None
        return stable

    def on_odometry_velocity(self, vx, vy, vth, stamp):
        stamp = float(stamp)
        if (
            self._odom_stamp is not None
            and stamp - self._odom_stamp > self._odom_timeout
        ):
            self._stable_since = None
        self._odom_velocity = (float(vx), float(vy), float(vth))
        self._odom_stamp = stamp
        if self.state == self.SETTLING and self._is_staging():
            self._refresh_staging_stability(stamp)

    def on_detection(self, now):
        if self.state != self.NAVIGATING:
            return
        self._cancel_reason = self._cancel_pending(
            "interrupted", "sign detected during navigation", True
        )
        self.state = self.CANCELLING
        self._deadline = now + self._cancel_timeout
        self._emit(
            "cancel_goal", {"reason": "sign detected", "interrupt": True}
        )

    def cancel(self, reason, now=None):
        if self.state not in (self.NAVIGATING, self.SETTLING):
            return
        self._cancel_reason = self._cancel_pending(
            "cancelled", reason or "cancelled", False
        )
        self.state = self.CANCELLING
        self._deadline = self._clock() + self._cancel_timeout
        self._emit("cancel_goal", {"reason": reason or "cancelled"})

    def tick(self, now=None):
        now = self._clock() if now is None else now
        if self.state == self.NAVIGATING:
            if self._deadline is not None and now >= self._deadline:
                self._cancel_reason = self._cancel_pending(
                    "timed_out", "navigation action timed out", True
                )
                self.state = self.CANCELLING
                self._deadline = now + self._cancel_timeout
                self._emit(
                    "cancel_goal", {"reason": "navigation action timed out"}
                )
            return
        if self.state == self.SETTLING:
            if not self._is_staging():
                if self._deadline is not None and now >= self._deadline:
                    self._finish("succeeded", "", True, False)
                return
            stable = self._refresh_staging_stability(now)
            if (
                stable and self._settle_not_before is not None
                and now >= self._settle_not_before
                and self._stable_since is not None
                and now - self._stable_since >= self._settle_stable_duration
            ):
                self._finish("succeeded", "", True, False)
                return
            if self._deadline is not None and now >= self._deadline:
                self._finish(
                    "settle_timed_out",
                    "staging odometry did not settle before timeout",
                    False,
                    True,
                )
            return
        if self.state == self.CANCELLING:
            if self._deadline is not None and now >= self._deadline:
                reason = self._cancel_reason or ("timed_out", "cancel timed out", False)
                self._finish(reason[0], reason[1], False, reason[2])
            return

    def shutdown(self):
        if self.state == self.IDLE:
            self.state = self.COMPLETE
            return
        if self.state == self.COMPLETE:
            return
        self._finish("stopped", "supervisor shutdown", False, False)
