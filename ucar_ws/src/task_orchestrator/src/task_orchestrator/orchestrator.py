"""ROS-independent state machine for one complete U-CAR task."""

import math

from task_orchestrator.categories import (
    format_delivery_speech,
    format_result_speech,
    format_simulation_delivery_speech,
)
from task_orchestrator.motion_mode import motion_mode_for_state


class TaskOrchestrator:
    IDLE = "IDLE"
    CHECKING_DEPENDENCIES = "CHECKING_DEPENDENCIES"
    NAVIGATING_TO_PICKUP = "NAVIGATING_TO_PICKUP"
    WAITING_QR = "WAITING_QR"
    WAITING_LLM = "WAITING_LLM"
    WAITING_SPEECH = "WAITING_SPEECH"
    WAITING_DELIVERY_SPEECH = "WAITING_DELIVERY_SPEECH"
    WAITING_SIMULATION_SPEECH = "WAITING_SIMULATION_SPEECH"
    DELIVERY_HANDED_OFF = "DELIVERY_HANDED_OFF"
    NAVIGATING_TO_WORKSHOP = "NAVIGATING_TO_WORKSHOP"
    NAVIGATING_TO_SIM_WORKSHOP = "NAVIGATING_TO_SIM_WORKSHOP"
    WAITING_GAZEBO = "WAITING_GAZEBO"
    NAVIGATING_LINE_START = "NAVIGATING_LINE_START"
    WAITING_LINE_DIRECTION = "WAITING_LINE_DIRECTION"
    LINE_FOLLOWING = "LINE_FOLLOWING"
    WAITING_FINAL_SPEECH = "WAITING_FINAL_SPEECH"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"
    CANCELLED = "CANCELLED"

    _ACTIVE_STATES = frozenset(
        (
            CHECKING_DEPENDENCIES,
            NAVIGATING_TO_PICKUP,
            WAITING_QR,
            WAITING_LLM,
            WAITING_SPEECH,
            WAITING_DELIVERY_SPEECH,
            WAITING_SIMULATION_SPEECH,
            DELIVERY_HANDED_OFF,
            NAVIGATING_TO_WORKSHOP,
            NAVIGATING_TO_SIM_WORKSHOP,
            WAITING_GAZEBO,
            NAVIGATING_LINE_START,
            WAITING_LINE_DIRECTION,
            LINE_FOLLOWING,
            WAITING_FINAL_SPEECH,
        )
    )
    _TERMINAL_STATES = frozenset(
        (IDLE, COMPLETE, ERROR, CANCELLED)
    )
    _TIMEOUT_KEYS = {
        CHECKING_DEPENDENCIES: "dependency_ready",
        NAVIGATING_TO_PICKUP: "pickup_navigation",
        WAITING_QR: "qr_search",
        WAITING_LLM: "llm_classification",
        WAITING_SPEECH: "speech",
        WAITING_DELIVERY_SPEECH: "speech",
        WAITING_SIMULATION_SPEECH: "speech",
        DELIVERY_HANDED_OFF: "delivery_navigation",
        NAVIGATING_TO_WORKSHOP: "delivery_navigation",
        NAVIGATING_TO_SIM_WORKSHOP: "simulation_navigation",
        WAITING_GAZEBO: "gazebo",
        NAVIGATING_LINE_START: "line_navigation",
        WAITING_LINE_DIRECTION: "line_direction",
        LINE_FOLLOWING: "line_follow",
        WAITING_FINAL_SPEECH: "speech",
    }

    def __init__(self, outputs, clock, id_factory, timeouts,
                 simulation_phase_enabled=False, gazebo_phase_enabled=False,
                 line_start_goal=None):
        self._outputs = outputs
        self._clock = clock
        self._id_factory = id_factory
        self._timeouts = dict(timeouts)
        self.simulation_phase_enabled = bool(simulation_phase_enabled)
        self.gazebo_phase_enabled = bool(gazebo_phase_enabled)
        self._line_start_goal = self._validate_line_start_goal(line_start_goal)
        self.state = self.IDLE
        self.task = None
        self.deadline = None
        self.last_status = None
        self._emit("publish_motion_mode", motion_mode_for_state(self.state))

    @staticmethod
    def _validate_line_start_goal(line_start_goal):
        if line_start_goal is None:
            raise ValueError("line_start_goal is required for Phase 3")
        try:
            pose = dict(line_start_goal)
        except (TypeError, ValueError):
            raise ValueError("line_start_goal must be an object")
        for field in ("frame_id", "x", "y", "qz", "qw"):
            if field not in pose:
                raise ValueError("line_start_goal requires %s" % field)
        for field in ("x", "y", "qz", "qw"):
            value = pose[field]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError("line_start_goal %s must be finite" % field)
        if not isinstance(pose["frame_id"], str) or not pose["frame_id"].strip():
            raise ValueError("line_start_goal frame_id must be non-blank text")
        return pose

    def _emit(self, action, payload):
        self._outputs.append((action, payload))

    def _transition(self, state):
        self.state = state
        self._emit("publish_motion_mode", motion_mode_for_state(state))
        timeout_key = self._TIMEOUT_KEYS.get(state)
        if timeout_key is None:
            self.deadline = None
        else:
            self.deadline = self._clock() + self._timeouts[timeout_key]

    def _publish_status(self, status, message="", remember=True):
        payload = {
            "protocol_version": 1,
            "task_id": self.task["task_id"],
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

    def _matches(self, message, identity_field=None, task_field=None):
        if self.task is None:
            return False
        if message.get("task_id") != self.task.get("task_id"):
            return False
        if identity_field is None:
            return True
        return message.get(identity_field) == self.task.get(task_field)

    def _publish_qr_stop(self, reason):
        search_id = self.task.get("search_id")
        if search_id is None:
            return
        self._emit(
            "publish_qr_stop",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "search_id": search_id,
                "reason": reason,
            },
        )

    def _fail(self, message):
        failed_state = self.state
        if failed_state == self.WAITING_QR:
            self._publish_qr_stop("orchestrator_error")
        self._transition(self.ERROR)
        self._publish_status("error", message)

    def on_task_request(self, message):
        task_id = message["task_id"]
        if (
            self.task is not None
            and self.task["task_id"] == task_id
            and self.state != self.IDLE
        ):
            self._republish_status()
            return
        if self.state in self._ACTIVE_STATES:
            self._publish_status(
                "busy",
                "active task %s" % self.task["task_id"],
                remember=False,
            )
            return
        if self.state not in self._TERMINAL_STATES:
            return

        self.task = {
            "task_id": task_id,
            "physical_target_category": message[
                "physical_target_category"
            ],
            "simulation_target_category": message[
                "simulation_target_category"
            ],
            "raw_text": message["raw_text"],
        }
        self.last_status = None
        self._transition(self.CHECKING_DEPENDENCIES)
        self._publish_status("accepted")

    def on_dependencies_ready(self):
        if self.state != self.CHECKING_DEPENDENCIES:
            return
        goal_id = self._id_factory()
        self.task["pickup_goal_id"] = goal_id
        self._transition(self.NAVIGATING_TO_PICKUP)
        self._publish_status("running")
        self._emit(
            "publish_pickup_goal",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": goal_id,
                "target": "物品领取区观察点",
            },
        )

    def on_pickup_arrived(self, message):
        if self.state != self.NAVIGATING_TO_PICKUP:
            return
        if not self._matches(message, "goal_id", "pickup_goal_id"):
            return
        if message["status"] == "failed":
            self._fail(message["message"])
            return

        search_id = self._id_factory()
        self.task["search_id"] = search_id
        self._transition(self.WAITING_QR)
        self._publish_status("running")
        self._emit(
            "publish_qr_start",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "search_id": search_id,
                "expected_count": 3,
            },
        )

    def on_qr_result(self, message):
        if self.state != self.WAITING_QR:
            if self._matches(message, "search_id", "search_id"):
                self._republish_status()
            return
        if not self._matches(message, "search_id", "search_id"):
            return
        status = message["status"]
        if status == "searching":
            return
        if status != "complete":
            self._fail(message.get("message") or "QR search failed")
            return

        self.task["qr_items"] = list(message["items"])
        # 提前启动导航交接（与 LLM 并行）：交接不需要车间信息，
        # 只以 task_id/goal_id 关联；stop 任务目标等 LLM 完成后由本机
        # 直接发布 /task/stop_mission_goal。
        delivery_goal_id = self._id_factory()
        self.task["delivery_goal_id"] = delivery_goal_id
        self.task["handoff_ready"] = False
        self._emit(
            "publish_handoff_goal",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": delivery_goal_id,
            },
        )
        request_id = self._id_factory()
        self.task["request_id"] = request_id
        candidates = [
            {"order": item["order"], "item_name": item["item_name"]}
            for item in self.task["qr_items"]
        ]
        self._transition(self.WAITING_LLM)
        self._publish_status("running")
        self._emit(
            "publish_llm_request",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "request_id": request_id,
                "physical_target_category": self.task[
                    "physical_target_category"
                ],
                "simulation_target_category": self.task[
                    "simulation_target_category"
                ],
                "candidates": candidates,
            },
        )

    def on_llm_result(self, message):
        if self.state != self.WAITING_LLM:
            if self._matches(message, "request_id", "request_id"):
                self._republish_status()
            return
        if not self._matches(message, "request_id", "request_id"):
            return
        if message["status"] != "success":
            self._fail(message.get("message") or "LLM classification failed")
            return

        self.task["physical"] = dict(message["physical"])
        self.task["simulation"] = dict(message["simulation"])
        speech_id = self._id_factory()
        self.task["speech_id"] = speech_id
        text = format_result_speech(
            self.task["physical"]["selected_item"],
            self.task["physical_target_category"],
            self.task["simulation"]["selected_item"],
            self.task["simulation_target_category"],
        )
        self._transition(self.WAITING_SPEECH)
        self._publish_status("running")
        self._emit(
            "publish_speech",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "speech_id": speech_id,
                "text": text,
            },
        )

    def on_speech_done(self, message):
        if self.state not in (
            self.WAITING_SPEECH,
            self.WAITING_DELIVERY_SPEECH,
            self.WAITING_SIMULATION_SPEECH,
            self.WAITING_FINAL_SPEECH,
        ):
            if self._matches(message, "speech_id", "speech_id"):
                self._republish_status()
            return
        if not self._matches(message, "speech_id", "speech_id"):
            return
        if message["status"] != "success":
            self._fail(message.get("message") or "speech playback failed")
            return

        if self.state == self.WAITING_DELIVERY_SPEECH:
            if self.simulation_phase_enabled:
                self._publish_simulation_goal()
            else:
                self._complete()
            return
        if self.state == self.WAITING_SIMULATION_SPEECH:
            # 第二阶段仿真播报成功后进入第三阶段，不再直接完成。
            self._publish_line_navigation_goal()
            return
        if self.state == self.WAITING_FINAL_SPEECH:
            self._complete()
            return

        goal_id = self.task["delivery_goal_id"]
        # 交接已在 QR 完成时提前启动（与 LLM 并行）。交接未就绪时停在
        # DELIVERY_HANDED_OFF 等待 ready（motion mode 保持 IDLE）；
        # 已就绪则立即放行 stop 任务目标。
        self._transition(self.DELIVERY_HANDED_OFF)
        self._publish_status("running")
        if self.task.get("handoff_ready"):
            self._publish_stop_mission_goal()

    def _publish_stop_mission_goal(self):
        goal_id = self.task["delivery_goal_id"]
        self._transition(self.NAVIGATING_TO_WORKSHOP)
        self._publish_status("running")
        self._emit(
            "publish_stop_mission_goal",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": goal_id,
                "target_workshop": self.task["physical"]["workshop"],
                "selected_item": self.task["physical"]["selected_item"],
                "physical_goal_id": goal_id,
                "physical": {
                    "target_workshop": self.task["physical"]["workshop"],
                    "selected_item": self.task["physical"]["selected_item"],
                },
            },
        )

    def on_delivery_arrived(self, message):
        if self.state != self.NAVIGATING_TO_WORKSHOP:
            return
        if not self._matches(message, "goal_id", "delivery_goal_id"):
            return
        if message["status"] == "failed":
            self._fail(message["message"])
            return
        # 实物停车播报成功后才允许第二阶段继续。
        self._start_speech(
            self.WAITING_DELIVERY_SPEECH,
            format_delivery_speech(
                self.task["physical"]["selected_item"],
                self.task["physical"]["workshop"],
            ),
        )

    def _start_speech(self, state, text, status_message=""):
        speech_id = self._id_factory()
        self.task["speech_id"] = speech_id
        self._transition(state)
        self._publish_status("running", status_message)
        self._emit(
            "publish_speech",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "speech_id": speech_id,
                "text": text,
            },
        )

    def _publish_line_navigation_goal(self):
        goal_id = self._id_factory()
        self.task["line_navigation_goal_id"] = goal_id
        self._transition(self.NAVIGATING_LINE_START)
        self._publish_status("running")
        self._emit(
            "publish_line_navigation_goal",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": goal_id,
                "pose": dict(self._line_start_goal),
            },
        )

    def on_line_navigation_arrived(self, message):
        if self.state != self.NAVIGATING_LINE_START:
            if self._matches(message, "goal_id", "line_navigation_goal_id"):
                self._republish_status()
            return
        if not self._matches(message, "goal_id", "line_navigation_goal_id"):
            return
        if message["status"] == "failed":
            self._fail(message["message"])
            return

        goal_id = self._id_factory()
        self.task["line_follow_goal_id"] = goal_id
        self._transition(self.WAITING_LINE_DIRECTION)
        self._publish_status("running")
        self._emit(
            "publish_line_follow_start",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": goal_id,
            },
        )

    def on_line_status(self, message):
        if self.state not in (self.WAITING_LINE_DIRECTION, self.LINE_FOLLOWING):
            if self._matches(message, "goal_id", "line_follow_goal_id"):
                self._republish_status()
            return
        if not self._matches(message, "goal_id", "line_follow_goal_id"):
            return
        if self.state == self.WAITING_LINE_DIRECTION:
            # 红灯等待和过早的成功不推进；关联失败必须立即终止任务。
            if message["status"] == "failure":
                self._fail(message.get("reason") or "line follow failed")
            elif message["status"] == "direction_selected":
                self._transition(self.LINE_FOLLOWING)
                self._publish_status("running")
            return
        if message["status"] == "success":
            self._start_speech(self.WAITING_FINAL_SPEECH, "任务完成")
        elif message["status"] == "failure":
            self._fail(message.get("reason") or "line follow failed")

    def _publish_simulation_goal(self):
        goal_id = self._id_factory()
        self.task["simulation_goal_id"] = goal_id
        self._transition(self.NAVIGATING_TO_SIM_WORKSHOP)
        self._publish_status("running")
        self._emit(
            "publish_simulation_navigation_goal",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": goal_id,
                "target_workshop": self.task["simulation"]["workshop"],
                "selected_item": self.task["simulation"]["selected_item"],
            },
        )

    def _complete(self):
        self._transition(self.COMPLETE)
        self._publish_status("complete")

    def on_navigation_handoff_status(self, message):
        # 交接在 QR 完成时提前启动，ready 可能在 LLM/TTS 期间到达；
        # 只有 DELIVERY_HANDED_OFF 且已 ready 才放行 stop 任务目标。
        if self.state not in (
            self.WAITING_LLM,
            self.WAITING_SPEECH,
            self.DELIVERY_HANDED_OFF,
        ):
            return
        if not self._matches(message, "goal_id", "delivery_goal_id"):
            return
        if message["status"] == "failed":
            self._fail(message.get("message") or "navigation handoff failed")
            return
        self.task["handoff_ready"] = True
        if self.state == self.DELIVERY_HANDED_OFF:
            self._publish_stop_mission_goal()

    def on_simulation_arrived(self, message):
        if self.state != self.NAVIGATING_TO_SIM_WORKSHOP:
            return
        if not self._matches(message, "goal_id", "simulation_goal_id"):
            return
        if message["status"] == "failed":
            self._fail(message["message"])
            return
        if self.gazebo_phase_enabled:
            goal_id = self._id_factory()
            self.task["gazebo_goal_id"] = goal_id
            self._transition(self.WAITING_GAZEBO)
            self._publish_status("running")
            self._emit(
                "publish_gazebo_start",
                {
                    "protocol_version": 1,
                    "task_id": self.task["task_id"],
                    "goal_id": goal_id,
                    "selected_item": self.task["simulation"]["selected_item"],
                    "target_category": self.task["simulation"]["category"],
                    "target_workshop": self.task["simulation"]["workshop"],
                },
            )
            return
        self._finish_gazebo_gate()

    def _finish_gazebo_gate(self, diagnostic=""):
        self._start_speech(
            self.WAITING_SIMULATION_SPEECH,
            format_simulation_delivery_speech(
                self.task["simulation"]["selected_item"],
                self.task["simulation"]["workshop"],
            ),
            diagnostic,
        )

    def on_gazebo_complete(self, message):
        if self.state != self.WAITING_GAZEBO:
            return
        if not self._matches(message, "goal_id", "gazebo_goal_id"):
            return
        diagnostic = ""
        if message["status"] == "failure":
            diagnostic = "gazebo failure: %s" % message.get("reason", "unknown")
        self._finish_gazebo_gate(diagnostic)

    def on_cancel(self, message):
        if self.state not in self._ACTIVE_STATES:
            return
        if not self._matches(message):
            return
        if self.state == self.WAITING_QR:
            self._publish_qr_stop(message.get("reason") or "cancelled")
        self._transition(self.CANCELLED)
        self._publish_status("cancelled", message.get("reason") or "cancelled")

    def on_internal_error(self, message):
        if self.state in self._ACTIVE_STATES:
            self._fail(message or "internal orchestrator error")

    def tick(self):
        if self.state not in self._ACTIVE_STATES or self.deadline is None:
            return
        if self._clock() < self.deadline:
            return
        timed_out_state = self.state
        if timed_out_state == self.WAITING_GAZEBO:
            self._finish_gazebo_gate("gazebo timeout")
            return
        self._fail("timeout in state %s" % timed_out_state)
