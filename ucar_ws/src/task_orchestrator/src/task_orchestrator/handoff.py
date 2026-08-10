"""ROS-independent bounded navigation-stack handoff state machine.

本模块不导入 rospy/roslaunch/subprocess，也不依赖任何 ROS 消息类型。
ROS 适配器（navigation_handoff_supervisor_node）负责把真实 action 取消、
odom、进程退出和就绪探测翻译成本模块的观察事件，并把本模块产生的
effect 翻译成真实的动作。

状态顺序：

    IDLE -> CANCELLING -> VERIFYING_STOP -> STOPPING_LEGACY
         -> STARTING_STOP -> WAITING_STOP_READY -> READY
    任意阶段耗尽重试或超过总时限 -> FAILED

每个状态切换都先发出 ``publish_zero``，之后才发生命周期 effect。
有界操作（取消、旧栈退出、新栈就绪）各自拥有重试次数；odom 近零验证
没有独立重试次数，只受总时限约束（连续新鲜近零样本达到
``stop_stable_duration`` 才通过）。FAILED 后等待人工处理或重新
``start()`` 本次任务，绝不带着不确定定位继续运动。
"""

import math


class HandoffObservation:
    """带可选身份的事件基类；身份不匹配的观察一律忽略。"""

    def __init__(self, task_id=None, goal_id=None):
        self.task_id = task_id
        self.goal_id = goal_id


class ActionGoalCancelled(HandoffObservation):
    """遗留 action goal 已取消。"""


class ActionGoalCancelFailed(HandoffObservation):
    """遗留 action goal 取消失败（触发有界重试）。"""


class OdomStopped(HandoffObservation):
    """收到一条近零速度的 odom 样本，stamp 为样本时间。"""

    def __init__(self, stamp, task_id=None, goal_id=None):
        super().__init__(task_id, goal_id)
        self.stamp = stamp


class OdomNotStopped(HandoffObservation):
    """odom 显示车辆仍在运动，近零累积清零。"""


class OdomStale(HandoffObservation):
    """odom 样本陈旧或时间回退，近零累积清零。"""


class LegacyStackExited(HandoffObservation):
    """旧导航栈进程组已确认退出。"""


class LegacyStackStillRunning(HandoffObservation):
    """旧导航栈进程组仍在运行（触发有界重试）。"""


class StopStackStarted(HandoffObservation):
    """stop 导航栈进程组已启动。"""


class StopStackStartFailed(HandoffObservation):
    """stop 导航栈进程组启动失败；旧栈已停，直接进入 FAILED。"""


class StopStackReady(HandoffObservation):
    """map/AMCL/TF/action/OCR/camera/scan 全部就绪。"""


class StopStackNotReady(HandoffObservation):
    """新栈尚未就绪（触发有界重试）。"""


_CONFIG_FIELDS = (
    "cancel_retries",
    "cancel_timeout",
    "stop_stable_duration",
    "odom_max_age",
    "legacy_exit_retries",
    "legacy_exit_timeout",
    "readiness_retries",
    "readiness_poll_period",
    "total_timeout",
)
_RETRY_FIELDS = ("cancel_retries", "legacy_exit_retries", "readiness_retries")


def validate_config(config):
    for field in _CONFIG_FIELDS:
        value = config.get(field)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError("%s must be a positive finite number" % field)
    for field in _RETRY_FIELDS:
        value = config[field]
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("%s must be an integer" % field)
    return config


class NavigationHandoff:
    IDLE = "IDLE"
    CANCELLING = "CANCELLING"
    VERIFYING_STOP = "VERIFYING_STOP"
    STOPPING_LEGACY = "STOPPING_LEGACY"
    STARTING_STOP = "STARTING_STOP"
    WAITING_STOP_READY = "WAITING_STOP_READY"
    READY = "READY"
    FAILED = "FAILED"

    _TERMINAL_STATES = frozenset((IDLE, READY, FAILED))

    def __init__(self, config, clock):
        self._config = validate_config(dict(config))
        self._clock = clock
        self.state = self.IDLE
        self._task_id = None
        self._goal_id = None
        self._start_time = None
        self._last_clock = None
        self._retries = {}
        self._odom_first_stamp = None
        self._odom_last_stamp = None

    def start(self, task_id, goal_id):
        """以任务身份启动交接；重复启动返回空 effect 列表。"""
        if self.state not in self._TERMINAL_STATES:
            return []
        self._task_id = task_id
        self._goal_id = goal_id
        now = self._clock()
        self._start_time = now
        self._last_clock = now
        self._retries = {}
        self._odom_first_stamp = None
        self._odom_last_stamp = None
        self.state = self.CANCELLING
        return [
            ("publish_zero", None),
            ("cancel_goals", self._identity_payload()),
        ]

    def observe(self, observation):
        if self.state in self._TERMINAL_STATES or not self._matches(observation):
            return []
        handler = self._OBSERVERS.get(type(observation))
        if handler is None:
            return []
        return handler(self, observation)

    def tick(self):
        """总时限检查；只在活动状态推进，时钟回退时重置时限基准。"""
        if self.state in self._TERMINAL_STATES:
            return []
        now = self._clock()
        if now < self._last_clock:
            self._start_time = now
            self._last_clock = now
            return [
                ("publish_zero", None),
                ("publish_diagnostic", self._diagnostic("clock_rollback")),
            ]
        self._last_clock = now
        if now - self._start_time > self._config["total_timeout"]:
            return self._fail("total_timeout", "total_timeout")
        return []

    def _matches(self, observation):
        if observation.task_id is not None and observation.task_id != self._task_id:
            return False
        if observation.goal_id is not None and observation.goal_id != self._goal_id:
            return False
        return True

    def _identity_payload(self):
        return {"task_id": self._task_id, "goal_id": self._goal_id}

    def _diagnostic(self, reason):
        payload = self._identity_payload()
        payload["state"] = self.state
        payload["stage"] = self._retry_stage(self.state)
        payload["reason"] = reason
        payload["retry"] = self._retries.get(self.state, 0)
        payload["max_retries"] = self._retry_limit(self.state)
        return payload

    def _retry_stage(self, state):
        return {
            self.CANCELLING: "cancel",
            self.STOPPING_LEGACY: "legacy_exit",
            self.WAITING_STOP_READY: "readiness",
        }.get(state, "handoff")

    def _retry_limit(self, state):
        limits = {
            self.CANCELLING: self._config["cancel_retries"],
            self.STOPPING_LEGACY: self._config["legacy_exit_retries"],
            self.WAITING_STOP_READY: self._config["readiness_retries"],
        }
        return limits.get(state, 0)

    def _retry(self, observation, stage, retry_effect, reason):
        self._retries[self.state] = self._retries.get(self.state, 0) + 1
        if self._retries[self.state] > self._retry_limit(self.state):
            return self._fail(reason, stage)
        return [
            ("publish_zero", None),
            ("publish_diagnostic", self._diagnostic("retrying")),
            (retry_effect, self._identity_payload()),
        ]

    def _fail(self, reason, stage):
        self.state = self.FAILED
        diagnostic = self._diagnostic(reason)
        diagnostic["stage"] = stage
        payload = self._identity_payload()
        payload["reason"] = reason
        payload["stage"] = stage
        return [
            ("publish_zero", None),
            ("publish_diagnostic", diagnostic),
            ("handoff_failed", payload),
        ]

    def _transition(self, state):
        self.state = state
        self._odom_first_stamp = None
        self._odom_last_stamp = None

    def _on_action_goal_cancelled(self, _observation):
        if self.state != self.CANCELLING:
            return []
        self._transition(self.VERIFYING_STOP)
        return [
            ("publish_zero", None),
            ("wait_stopped", self._identity_payload()),
        ]

    def _on_action_goal_cancel_failed(self, _observation):
        if self.state != self.CANCELLING:
            return []
        return self._retry(
            None, "cancel", "cancel_goals", "cancel_exhausted"
        )

    def _on_odom_stopped(self, observation):
        if self.state != self.VERIFYING_STOP:
            return []
        stamp = observation.stamp
        if (
            isinstance(stamp, bool)
            or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp)
        ):
            return self._odom_miss("odom_stale")
        last = self._odom_last_stamp
        if last is not None and stamp < last:
            return self._odom_miss("odom_clock_rollback")
        if last is not None and stamp - last > self._config["odom_max_age"]:
            return self._odom_miss("odom_stale")
        if self._odom_first_stamp is None:
            self._odom_first_stamp = stamp
        self._odom_last_stamp = stamp
        if (
            stamp - self._odom_first_stamp
            >= self._config["stop_stable_duration"]
        ):
            self._transition(self.STOPPING_LEGACY)
            return [
                ("publish_zero", None),
                ("stop_owned_legacy", self._identity_payload()),
            ]
        return []

    def _odom_miss(self, reason):
        self._odom_first_stamp = None
        self._odom_last_stamp = None
        return [
            ("publish_zero", None),
            ("publish_diagnostic", self._diagnostic(reason)),
        ]

    def _on_odom_not_stopped(self, _observation):
        if self.state != self.VERIFYING_STOP:
            return []
        return self._odom_miss("vehicle_moving")

    def _on_odom_stale(self, _observation):
        if self.state != self.VERIFYING_STOP:
            return []
        return self._odom_miss("odom_stale")

    def _on_legacy_stack_exited(self, _observation):
        if self.state != self.STOPPING_LEGACY:
            return []
        self._transition(self.STARTING_STOP)
        return [
            ("publish_zero", None),
            ("start_owned_stop", self._identity_payload()),
        ]

    def _on_legacy_stack_still_running(self, _observation):
        if self.state != self.STOPPING_LEGACY:
            return []
        return self._retry(
            None, "legacy_exit", "verify_legacy_absent", "legacy_exit_exhausted"
        )

    def _on_stop_stack_started(self, _observation):
        if self.state != self.STARTING_STOP:
            return []
        self._transition(self.WAITING_STOP_READY)
        return [
            ("publish_zero", None),
            ("wait_stop_ready", self._identity_payload()),
        ]

    def _on_stop_stack_start_failed(self, _observation):
        if self.state != self.STARTING_STOP:
            return []
        return self._fail("stop_stack_start_failed", "start_stop")

    def _on_stop_stack_ready(self, _observation):
        if self.state != self.WAITING_STOP_READY:
            return []
        self._transition(self.READY)
        return [
            ("publish_zero", None),
            ("release_task", self._identity_payload()),
        ]

    def _on_stop_stack_not_ready(self, _observation):
        if self.state != self.WAITING_STOP_READY:
            return []
        return self._retry(
            None, "readiness", "wait_stop_ready", "readiness_exhausted"
        )

    _OBSERVERS = {
        ActionGoalCancelled: _on_action_goal_cancelled,
        ActionGoalCancelFailed: _on_action_goal_cancel_failed,
        OdomStopped: _on_odom_stopped,
        OdomNotStopped: _on_odom_not_stopped,
        OdomStale: _on_odom_stale,
        LegacyStackExited: _on_legacy_stack_exited,
        LegacyStackStillRunning: _on_legacy_stack_still_running,
        StopStackStarted: _on_stop_stack_started,
        StopStackStartFailed: _on_stop_stack_start_failed,
        StopStackReady: _on_stop_stack_ready,
        StopStackNotReady: _on_stop_stack_not_ready,
    }
