#!/usr/bin/env python3
"""ROS String/JSON adapter for the pure task orchestration core."""

import json
import threading
import uuid

import rospy
from std_msgs.msg import String

from task_orchestrator.orchestrator import TaskOrchestrator
from task_orchestrator.motion_mode import IDLE, NAVIGATION, QR_SEARCH
from task_orchestrator.protocol import (
    ProtocolError,
    parse_arrival,
    parse_cancel,
    parse_dependencies_ready,
    parse_llm_result,
    parse_qr_result,
    parse_speech_done,
    parse_task_request,
)


DEFAULT_TIMEOUTS = {
    "dependency_ready": 120.0,
    "pickup_navigation": 300.0,
    "qr_search": 120.0,
    "llm_classification": 120.0,
    "speech": 60.0,
    "delivery_navigation": 300.0,
    "simulation_navigation": 300.0,
    "cancel_ack": 15.0,
}


def _make_publishers():
    return {
        "publish_motion_mode": rospy.Publisher(
            "/task/motion_mode", String, queue_size=1, latch=True
        ),
        "publish_status": rospy.Publisher(
            "/task/status", String, queue_size=10
        ),
        "publish_pickup_goal": rospy.Publisher(
            "/task/pickup_navigation_goal", String, queue_size=10
        ),
        "publish_qr_start": rospy.Publisher(
            "/qr_item_search/start", String, queue_size=10
        ),
        "publish_qr_stop": rospy.Publisher(
            "/qr_item_search/stop", String, queue_size=10
        ),
        "publish_llm_request": rospy.Publisher(
            "/llm/classify/request", String, queue_size=10
        ),
        "publish_speech": rospy.Publisher(
            "/voice/speak", String, queue_size=10
        ),
        "publish_delivery_goal": rospy.Publisher(
            "/task/delivery_navigation_goal", String, queue_size=10
        ),
        "publish_simulation_navigation_goal": rospy.Publisher(
            "/task/simulation_navigation_goal", String, queue_size=10
        ),
    }


def _dispatch(outputs, publishers):
    while outputs:
        action, payload = outputs.pop(0)
        publisher = publishers.get(action)
        if publisher is None:
            rospy.logerr("unknown orchestrator action: %s", action)
            continue
        try:
            if action == "publish_motion_mode":
                publisher.publish(_motion_mode_message(payload))
            else:
                publisher.publish(
                    String(data=json.dumps(payload, ensure_ascii=False))
                )
        except Exception as exc:
            rospy.logerr("failed to publish orchestrator action %s: %s", action, exc)


def _motion_mode_message(payload):
    if not isinstance(payload, str) or payload not in (
        IDLE,
        NAVIGATION,
        QR_SEARCH,
    ):
        rospy.logerr("invalid motion mode payload; publishing safe IDLE")
        payload = IDLE
    return String(data=payload)


def _run_callback(lock, outputs, publishers, operation):
    """Serialize one core mutation and its complete publication batch."""
    with lock:
        start = len(outputs)
        operation()
        batch = outputs[start:]
        del outputs[start:]
        _dispatch(batch, publishers)
        return batch


def _task_context(orchestrator):
    task = orchestrator.task
    return {
        "task_id": task["task_id"],
        "request_id": task["request_id"],
        "physical_target_category": task["physical_target_category"],
        "simulation_target_category": task[
            "simulation_target_category"
        ],
        "candidates": [
            {"order": item["order"], "item_name": item["item_name"]}
            for item in task["qr_items"]
        ],
    }


def _callback(
    orchestrator,
    outputs,
    publishers,
    parser,
    handler,
    callback_lock,
    expected_state=None,
):
    def receive(message):
        def mutate():
            expected_states = expected_state if isinstance(expected_state, tuple) else (expected_state,)
            if expected_state is not None and orchestrator.state not in expected_states:
                rospy.logwarn("ignored message for inactive state %s (current: %s)",
                              expected_state, orchestrator.state)
                return
            try:
                parsed = parser(message.data)
                handler(parsed)
            except ProtocolError as exc:
                rospy.logwarn("ignored invalid or stale message: %s", exc)
            except Exception as exc:
                rospy.logerr("orchestrator callback failed: %s", exc)
                orchestrator.on_internal_error(str(exc))
        _run_callback(callback_lock, outputs, publishers, mutate)

    return receive


def _ready_callback(orchestrator, outputs, publishers, callback_lock):
    def receive(message):
        def mutate():
            if orchestrator.state != TaskOrchestrator.CHECKING_DEPENDENCIES:
                rospy.logwarn("ignored dependencies-ready outside dependency check")
                return
            try:
                parse_dependencies_ready(message.data, orchestrator.task["task_id"])
                orchestrator.on_dependencies_ready()
            except (ProtocolError, KeyError, TypeError) as exc:
                rospy.logwarn("ignored invalid dependencies-ready message: %s", exc)
            except Exception as exc:
                rospy.logerr("dependencies-ready callback failed: %s", exc)
                orchestrator.on_internal_error(str(exc))
        _run_callback(callback_lock, outputs, publishers, mutate)

    return receive


def _subscribe(orchestrator, outputs, publishers, callback_lock):
    rospy.Subscriber(
        "/voice/task_request",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            parse_task_request,
            orchestrator.on_task_request,
            callback_lock,
        ),
    )
    rospy.Subscriber(
        "/task/dependencies_ready",
        String,
        _ready_callback(orchestrator, outputs, publishers, callback_lock),
    )
    rospy.Subscriber(
        "/task/pickup_arrived",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_arrival(
                raw,
                orchestrator.task["task_id"],
                orchestrator.task["pickup_goal_id"],
            ),
            orchestrator.on_pickup_arrived,
            callback_lock,
            TaskOrchestrator.NAVIGATING_TO_PICKUP,
        ),
    )
    rospy.Subscriber(
        "/qr_item_search/result",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_qr_result(
                raw,
                orchestrator.task["task_id"],
                orchestrator.task["search_id"],
            ),
            orchestrator.on_qr_result,
            callback_lock,
            TaskOrchestrator.WAITING_QR,
        ),
    )
    rospy.Subscriber(
        "/llm/classify/result",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_llm_result(
                raw,
                _task_context(orchestrator),
            ),
            orchestrator.on_llm_result,
            callback_lock,
            TaskOrchestrator.WAITING_LLM,
        ),
    )
    rospy.Subscriber(
        "/voice/speak_done",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_speech_done(
                raw,
                orchestrator.task["task_id"],
                orchestrator.task["speech_id"],
            ),
            orchestrator.on_speech_done,
            callback_lock,
            TaskOrchestrator.WAITING_SPEECH,
        ),
    )
    rospy.Subscriber(
        "/task/delivery_arrived",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_arrival(
                raw,
                orchestrator.task["task_id"],
                orchestrator.task["delivery_goal_id"],
            ),
            orchestrator.on_delivery_arrived,
            callback_lock,
            TaskOrchestrator.DELIVERY_HANDED_OFF,
        ),
    )
    rospy.Subscriber(
        "/task/simulation_arrived",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_arrival(
                raw,
                orchestrator.task["task_id"],
                orchestrator.task["simulation_goal_id"],
            ),
            orchestrator.on_simulation_arrived,
            callback_lock,
            TaskOrchestrator.NAVIGATING_TO_SIM_WORKSHOP,
        ),
    )
    rospy.Subscriber(
        "/task/cancel",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            lambda raw: parse_cancel(
                raw,
                orchestrator.task["task_id"],
            ),
            orchestrator.on_cancel,
            callback_lock,
            (
                TaskOrchestrator.CHECKING_DEPENDENCIES,
                TaskOrchestrator.NAVIGATING_TO_PICKUP,
                TaskOrchestrator.WAITING_QR,
                TaskOrchestrator.WAITING_LLM,
                TaskOrchestrator.WAITING_SPEECH,
                TaskOrchestrator.NAVIGATING_TO_SIM_WORKSHOP,
            ),
        ),
    )


def main():
    rospy.init_node("task_orchestrator")
    outputs = []
    callback_lock = threading.RLock()
    timeouts = rospy.get_param("~timeouts", DEFAULT_TIMEOUTS)
    simulation_phase_enabled = bool(
        rospy.get_param("~simulation_phase_enabled", False)
    )
    orchestrator = TaskOrchestrator(
        outputs,
        rospy.get_time,
        lambda: uuid.uuid4().hex,
        timeouts,
        simulation_phase_enabled=simulation_phase_enabled,
    )
    publishers = _make_publishers()
    with callback_lock:
        initial = list(outputs)
        del outputs[:]
        _dispatch(initial, publishers)
    _subscribe(orchestrator, outputs, publishers, callback_lock)

    def tick(_event):
        _run_callback(callback_lock, outputs, publishers, orchestrator.tick)

    rospy.Timer(rospy.Duration(0.5), tick)
    rospy.loginfo("task_orchestrator node started")
    rospy.spin()


if __name__ == "__main__":
    main()
