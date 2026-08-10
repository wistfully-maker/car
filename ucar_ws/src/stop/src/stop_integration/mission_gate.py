"""ROS-independent stop mission identity, dedupe and terminal-result cache.

门控只做校验与记录，本身绝不触发运动；是否启动由上层调用
``activate()`` 决定。一个时刻只允许一个活动任务；终态结果按
``task_id`` 缓存，重复终态目标只重发缓存结果。
"""


class MissionError(ValueError):
    """任务描述违反集成协议时抛出。"""


class Acceptance:
    """``accept()`` 的结果：启动、现有任务、缓存终态或错误。"""

    def __init__(self, start=False, mission=None, terminal=None, error=None):
        self.start = bool(start)
        self.mission = dict(mission) if mission else {}
        self.terminal = dict(terminal) if terminal else None
        self.error = error


def _require_text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise MissionError("%s must be non-empty" % field)
    return value.strip()


class MissionGate:
    def __init__(self):
        self._active = None
        self._terminal = None

    @property
    def active_task_id(self):
        return self._active["task_id"] if self._active else None

    def accept(self, mission):
        try:
            normalized = self._validate(mission)
        except MissionError as exc:
            return Acceptance(error=str(exc))
        task_id = normalized["task_id"]
        if self._active is not None:
            if self._active["task_id"] == task_id:
                # 重复活动身份：不重启，返回现有任务。
                return Acceptance(mission=dict(self._active))
            return Acceptance(
                error="active mission %s; cross-task update rejected"
                % self._active["task_id"]
            )
        if self._terminal is not None and self._terminal["task_id"] == task_id:
            # 重复终态身份：只重发缓存结果，不重复运动。
            return Acceptance(terminal=dict(self._terminal))
        self._active = normalized
        return Acceptance(start=True, mission=dict(normalized))

    def record_result(self, task_id, status, message=""):
        """记录终态结果并释放活动任务。"""
        self._terminal = {
            "task_id": task_id,
            "status": status,
            "message": message,
        }
        self._active = None

    def _validate(self, mission):
        if not isinstance(mission, dict):
            raise MissionError("mission must be an object")
        version = mission.get("protocol_version")
        if (
            isinstance(version, bool)
            or not isinstance(version, int)
            or version != 1
        ):
            raise MissionError("unsupported protocol version")
        normalized = {"protocol_version": 1}
        normalized["task_id"] = _require_text(mission.get("task_id"), "task_id")
        normalized["physical_goal_id"] = _require_text(
            mission.get("physical_goal_id"), "physical_goal_id"
        )
        normalized["simulation_goal_id"] = _require_text(
            mission.get("simulation_goal_id"), "simulation_goal_id"
        )
        normalized["physical"] = self._validate_target(
            mission.get("physical"), "physical"
        )
        simulation = mission.get("simulation")
        if simulation is not None:
            normalized["simulation"] = self._validate_target(
                simulation, "simulation"
            )
        return normalized

    @staticmethod
    def _validate_target(target, field):
        if not isinstance(target, dict):
            raise MissionError("%s must be an object" % field)
        return {
            "target_workshop": _require_text(
                target.get("target_workshop"), "%s.target_workshop" % field
            ),
            "selected_item": _require_text(
                target.get("selected_item"), "%s.selected_item" % field
            ),
        }
