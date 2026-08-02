"""ROS-independent state machine for one complete U-CAR task."""

from task_orchestrator.categories import format_result_speech
from task_orchestrator.motion_mode import motion_mode_for_state


class TaskOrchestrator:
    IDLE = "IDLE"
    CHECKING_DEPENDENCIES = "CHECKING_DEPENDENCIES"
    NAVIGATING_TO_PICKUP = "NAVIGATING_TO_PICKUP"
    WAITING_QR = "WAITING_QR"
    WAITING_LLM = "WAITING_LLM"
    WAITING_SPEECH = "WAITING_SPEECH"
    NAVIGATING_TO_WORKSHOP = "NAVIGATING_TO_WORKSHOP"
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
            NAVIGATING_TO_WORKSHOP,
        )
    )
    _TERMINAL_STATES = frozenset((IDLE, COMPLETE, ERROR, CANCELLED))
    _TIMEOUT_KEYS = {
        CHECKING_DEPENDENCIES: "dependency_ready",
        NAVIGATING_TO_PICKUP: "pickup_navigation",
        WAITING_QR: "qr_search",
        WAITING_LLM: "llm_classification",
        WAITING_SPEECH: "speech",
        NAVIGATING_TO_WORKSHOP: "delivery_navigation",
    }

    def __init__(self, outputs, clock, id_factory, timeouts):
        self._outputs = outputs
        self._clock = clock
        self._id_factory = id_factory
        self._timeouts = dict(timeouts)
        self.state = self.IDLE
        self.task = None
        self.deadline = None
        self.last_status = None
        self._emit("publish_motion_mode", motion_mode_for_state(self.state))

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
        if self.state != self.WAITING_SPEECH:
            if self._matches(message, "speech_id", "speech_id"):
                self._republish_status()
            return
        if not self._matches(message, "speech_id", "speech_id"):
            return
        if message["status"] != "success":
            self._fail(message.get("message") or "speech playback failed")
            return

        goal_id = self._id_factory()
        self.task["delivery_goal_id"] = goal_id
        self._transition(self.NAVIGATING_TO_WORKSHOP)
        self._publish_status("running")
        self._emit(
            "publish_delivery_goal",
            {
                "protocol_version": 1,
                "task_id": self.task["task_id"],
                "goal_id": goal_id,
                "target_workshop": self.task["physical"]["workshop"],
                "selected_item": self.task["physical"]["selected_item"],
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
        self._transition(self.COMPLETE)
        self._publish_status("complete")

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
        self._fail("timeout in state %s" % timed_out_state)
