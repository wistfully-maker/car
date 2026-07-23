#!/usr/bin/env python3
"""Bridge protocol-v1 `/voice/speak` requests to the existing TTS script."""

import json

import rospy
from std_msgs.msg import String

from task_orchestrator.protocol import ProtocolError, parse_speak_request
from task_orchestrator.tts_runner import TtsBridgeLogic


DEFAULT_COMMAND = [
    "python3",
    "/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py",
]


class TtsBridgeNode:
    def __init__(self):
        self._publisher = rospy.Publisher(
            "/voice/speak_done",
            String,
            queue_size=10,
        )
        self._logic = TtsBridgeLogic(
            command=rospy.get_param(
                "~tts_bridge/command",
                DEFAULT_COMMAND,
            ),
            timeout=rospy.get_param(
                "~tts_bridge/timeout",
                30.0,
            ),
        )
        self._subscriber = rospy.Subscriber(
            "/voice/speak",
            String,
            self._on_speak,
        )

    def _on_speak(self, message):
        try:
            request = parse_speak_request(message.data)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid speak request: %s", exc)
            return

        result = self._logic.handle(request)
        self._publisher.publish(
            String(data=json.dumps(result, ensure_ascii=False))
        )
        rospy.loginfo(
            "speech %s finished with %s",
            result["speech_id"],
            result["status"],
        )


def main():
    rospy.init_node("tts_bridge")
    TtsBridgeNode()
    rospy.loginfo("tts_bridge node started")
    rospy.spin()


if __name__ == "__main__":
    main()
