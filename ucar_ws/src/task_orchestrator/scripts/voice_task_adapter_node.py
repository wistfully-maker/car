#!/usr/bin/env python3
"""Convert recognized `/question` text into protocol-v1 task requests."""

import json
import uuid

import rospy
from std_msgs.msg import String

from task_orchestrator.voice_adapter import VoiceTaskAdapterLogic


class VoiceTaskAdapterNode:
    def __init__(self):
        self._publisher = rospy.Publisher(
            "/voice/task_request",
            String,
            queue_size=10,
        )
        self._logic = VoiceTaskAdapterLogic(
            clock=rospy.get_time,
            id_factory=lambda: "task-%s" % uuid.uuid4().hex,
            debounce_seconds=rospy.get_param(
                "~voice_adapter/debounce_seconds",
                2.0,
            ),
        )
        self._subscriber = rospy.Subscriber(
            "/question",
            String,
            self._on_question,
        )

    def _on_question(self, message):
        try:
            payload = self._logic.build(message.data)
        except ValueError as exc:
            rospy.logwarn("rejected voice instruction: %s", exc)
            return
        if payload is None:
            rospy.logdebug("debounced duplicate voice instruction")
            return
        self._publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        rospy.loginfo("published voice task %s", payload["task_id"])


def main():
    rospy.init_node("voice_task_adapter")
    VoiceTaskAdapterNode()
    rospy.loginfo("voice_task_adapter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
