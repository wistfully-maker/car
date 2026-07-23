#!/usr/bin/env python3
"""ROS String/JSON adapter for the pure task orchestration core."""

import json
import uuid

import rospy
from std_msgs.msg import String

from task_orchestrator.orchestrator import TaskOrchestrator
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
    "dependency_ready": 30.0,
    "pickup_navigation": 300.0,
    "qr_search": 120.0,
    "llm_classification": 60.0,
    "speech": 60.0,
    "delivery_navigation": 300.0,
    "cancel_ack": 15.0,
}


def _make_publishers():
    return {
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
    }


def _dispatch(outputs, publishers):
    while outputs:
        action, payload = outputs.pop(0)
        publisher = publishers.get(action)
        if publisher is None:
            rospy.logerr("unknown orchestrator action: %s", action)
            continue
        publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


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
    expected_state=None,
):
    def receive(message):
        expected_states = (
            expected_state
            if isinstance(expected_state, tuple)
            else (expected_state,)
        )
        if (
            expected_state is not None
            and orchestrator.state not in expected_states
        ):
            rospy.logwarn(
                "ignored message for inactive state %s (current: %s)",
                expected_state,
                orchestrator.state,
            )
            return
        try:
            parsed = parser(message.data)
            handler(parsed)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid or stale message: %s", exc)
            return
        except Exception as exc:
            rospy.logerr("orchestrator callback failed: %s", exc)
            orchestrator.on_internal_error(str(exc))
        _dispatch(outputs, publishers)

    return receive


def _ready_callback(orchestrator, outputs, publishers):
    def receive(message):
        if orchestrator.state != TaskOrchestrator.CHECKING_DEPENDENCIES:
            rospy.logwarn("ignored dependencies-ready outside dependency check")
            return
        try:
            parse_dependencies_ready(
                message.data,
                orchestrator.task["task_id"],
            )
            orchestrator.on_dependencies_ready()
        except (ProtocolError, KeyError, TypeError) as exc:
            rospy.logwarn("ignored invalid dependencies-ready message: %s", exc)
            return
        except Exception as exc:
            rospy.logerr("dependencies-ready callback failed: %s", exc)
            orchestrator.on_internal_error(str(exc))
        _dispatch(outputs, publishers)

    return receive


def _subscribe(orchestrator, outputs, publishers):
    rospy.Subscriber(
        "/voice/task_request",
        String,
        _callback(
            orchestrator,
            outputs,
            publishers,
            parse_task_request,
            orchestrator.on_task_request,
        ),
    )
    rospy.Subscriber(
        "/task/dependencies_ready",
        String,
        _ready_callback(orchestrator, outputs, publishers),
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
            TaskOrchestrator.NAVIGATING_TO_WORKSHOP,
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
            (
                TaskOrchestrator.CHECKING_DEPENDENCIES,
                TaskOrchestrator.NAVIGATING_TO_PICKUP,
                TaskOrchestrator.WAITING_QR,
                TaskOrchestrator.WAITING_LLM,
                TaskOrchestrator.WAITING_SPEECH,
                TaskOrchestrator.NAVIGATING_TO_WORKSHOP,
            ),
        ),
    )


def main():
    rospy.init_node("task_orchestrator")
    outputs = []
    timeouts = rospy.get_param("~timeouts", DEFAULT_TIMEOUTS)
    orchestrator = TaskOrchestrator(
        outputs,
        rospy.get_time,
        lambda: uuid.uuid4().hex,
        timeouts,
    )
    publishers = _make_publishers()
    _subscribe(orchestrator, outputs, publishers)

    def tick(_event):
        orchestrator.tick()
        _dispatch(outputs, publishers)

    rospy.Timer(rospy.Duration(0.5), tick)
    rospy.loginfo("task_orchestrator node started")
    rospy.spin()


if __name__ == "__main__":
    main()
