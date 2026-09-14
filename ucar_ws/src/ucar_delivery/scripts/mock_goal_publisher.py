#!/usr/bin/env python3
"""Local mock goal publisher.

Publishes one (or a repeating) protocol-v1 navigation goal for local
offline tests, exactly like:

rostopic pub -1 /task/delivery_navigation_goal std_msgs/String \
  "data: '{...}'"

Production never runs this node.
"""

import json

import rospy
from std_msgs.msg import String

from ucar_delivery import protocol


class MockGoalPublisher:
    def __init__(self):
        phase = rospy.get_param("~phase", protocol.PHASE_PHYSICAL)
        if phase not in protocol.PHASES:
            raise ValueError("unsupported mock phase: %s" % phase)
        self._phase = phase
        self._task_id = rospy.get_param("~task_id", "task-test-001")
        self._goal_id = rospy.get_param("~goal_id", None)
        self._target_workshop = rospy.get_param(
            "~target_workshop", "食品加工车间"
        )
        self._selected_item = rospy.get_param("~selected_item", "苹果")
        self._interval = rospy.get_param("~interval", 0.0)
        self._topic = rospy.get_param(
            "~topic",
            "/task/delivery_navigation_goal"
            if phase == protocol.PHASE_PHYSICAL
            else "/task/simulation_navigation_goal",
        )
        self._publisher = rospy.Publisher(self._topic, String, queue_size=10)
        self._publish()

    def _build(self):
        prefix = "delivery" if self._phase == protocol.PHASE_PHYSICAL else "simulation"
        return {
            "protocol_version": 1,
            "task_id": self._task_id,
            "goal_id": self._goal_id
            or "%s-%s-%s" % (prefix, self._task_id, "001"),
            "target_workshop": self._target_workshop,
            "selected_item": self._selected_item,
        }

    def _publish(self):
        payload = self._build()
        self._publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        rospy.loginfo("mock %s goal published: %s", self._phase, payload)
        if self._interval > 0.0:
            rospy.Timer(
                rospy.Duration(self._interval),
                lambda _event: self._publish(),
            )


def main():
    rospy.init_node("delivery_mock_goal_publisher")
    MockGoalPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
