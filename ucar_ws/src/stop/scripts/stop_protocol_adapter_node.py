#!/usr/bin/env python3
"""stop 阶段事件 -> protocol v1 到达结果映射（公有协议 topic 的权威 owner）。

输入：
    /task/stop_mission_goal        实物目标（handoff supervisor 放行）
    /task/simulation_navigation_goal  仿真目标（全局编排器，Phase 1 停车后）
    /stop/mission_event            车端私有阶段事件（phase1_done/done/failed:*）

输出：
    /task/delivery_arrived         第一次停车稳定确认后才发布
    /task/simulation_arrived       第二次停车稳定确认后才发布
    /stop/mission_ack              匹配仿真目标后放行 Phase 2（私有 seam）

重复终态事件只重发缓存结果，不重启任务；错误身份一律忽略。
"""

import json
import threading

import rospy
from std_msgs.msg import String

from stop_integration.mission_gate import MissionGate


def _parse_goal(message_data):
    try:
        value = json.loads(message_data)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid JSON: %s" % exc)
    if not isinstance(value, dict):
        raise ValueError("goal must be an object")
    version = value.get("protocol_version")
    if (
        isinstance(version, bool)
        or not isinstance(version, int)
        or version != 1
    ):
        raise ValueError("unsupported protocol version")
    result = {}
    for field in ("task_id", "goal_id", "target_workshop", "selected_item"):
        text = value.get(field)
        if not isinstance(text, str) or not text.strip():
            raise ValueError("%s must be non-empty" % field)
        result[field] = text.strip()
    return result


class StopProtocolAdapterNode:
    def __init__(self):
        self._gate = MissionGate()
        self._lock = threading.RLock()
        self._phase1_done = False
        # 终态到达结果缓存 (task_id -> [(topic, payload)])：重复事件重发，不重启。
        self._terminal_outputs = {}
        self._delivery_pub = rospy.Publisher(
            "/task/delivery_arrived", String, queue_size=10
        )
        self._simulation_pub = rospy.Publisher(
            "/task/simulation_arrived", String, queue_size=10
        )
        self._ack_pub = rospy.Publisher(
            "/stop/mission_ack", String, queue_size=10
        )
        self._publishers = {
            "/task/delivery_arrived": self._delivery_pub,
            "/task/simulation_arrived": self._simulation_pub,
        }
        rospy.Subscriber(
            "/task/stop_mission_goal", String, self._on_physical_goal, queue_size=1
        )
        rospy.Subscriber(
            "/task/simulation_navigation_goal",
            String,
            self._on_simulation_goal,
            queue_size=1,
        )
        rospy.Subscriber(
            "/stop/mission_event", String, self._on_mission_event, queue_size=10
        )

    def _publish(self, publisher, payload):
        try:
            publisher.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
        except Exception as exc:
            rospy.logerr("protocol publish failed: %s", exc)

    def _on_physical_goal(self, message):
        try:
            goal = _parse_goal(message.data)
        except ValueError as exc:
            rospy.logwarn("ignored invalid stop mission goal: %s", exc)
            return
        mission = {
            "protocol_version": 1,
            "task_id": goal["task_id"],
            "physical_goal_id": goal["goal_id"],
            "physical": {
                "target_workshop": goal["target_workshop"],
                "selected_item": goal["selected_item"],
            },
        }
        with self._lock:
            acceptance = self._gate.accept(mission)
            if acceptance.error:
                rospy.logwarn("rejected stop mission goal: %s", acceptance.error)
                return
            if acceptance.start:
                self._phase1_done = False
                rospy.loginfo("stop mission accepted: %s", goal["task_id"])
            elif acceptance.terminal is not None:
                self._republish_terminal(acceptance.terminal["task_id"])

    def _on_simulation_goal(self, message):
        try:
            goal = _parse_goal(message.data)
        except ValueError as exc:
            rospy.logwarn("ignored invalid simulation goal: %s", exc)
            return
        with self._lock:
            try:
                self._gate.update_simulation(
                    goal["task_id"],
                    goal["goal_id"],
                    {
                        "target_workshop": goal["target_workshop"],
                        "selected_item": goal["selected_item"],
                    },
                )
            except ValueError as exc:
                rospy.logwarn("ignored simulation goal: %s", exc)
                return
            self._publish(
                self._ack_pub,
                {
                    "protocol_version": 1,
                    "task_id": goal["task_id"],
                    "goal_id": goal["goal_id"],
                    "action": "start_phase2",
                    "simulation": {
                        "target_workshop": goal["target_workshop"],
                        "selected_item": goal["selected_item"],
                    },
                },
            )

    def _on_mission_event(self, message):
        event = message.data.strip()
        with self._lock:
            terminal = self._gate.terminal
            if terminal is not None and event == terminal["status"]:
                # 重复终态事件只重发缓存结果。
                self._republish_terminal(terminal["task_id"])
                return
            mission = self._gate.active_mission
            if mission is None:
                return
            task_id = mission["task_id"]
            if event == "phase1_done":
                self._phase1_done = True
                self._publish(
                    self._delivery_pub,
                    {
                        "protocol_version": 1,
                        "task_id": task_id,
                        "goal_id": mission["physical_goal_id"],
                        "status": "arrived",
                        "message": "",
                    },
                )
                return
            if event == "done":
                if "simulation_goal_id" not in mission:
                    rospy.logwarn("done event without simulation goal; ignored")
                    return
                payload = {
                    "protocol_version": 1,
                    "task_id": task_id,
                    "goal_id": mission["simulation_goal_id"],
                    "status": "arrived",
                    "message": "",
                }
                self._publish(self._simulation_pub, payload)
                self._gate.record_result(task_id, "done", "")
                self._terminal_outputs[task_id] = [
                    ("/task/simulation_arrived", dict(payload))
                ]
                return
            if event.startswith("failed:"):
                reason = event.split(":", 1)[1]
                if not self._phase1_done:
                    payload = {
                        "protocol_version": 1,
                        "task_id": task_id,
                        "goal_id": mission["physical_goal_id"],
                        "status": "failed",
                        "message": reason,
                    }
                    self._publish(self._delivery_pub, payload)
                elif "simulation_goal_id" in mission:
                    payload = {
                        "protocol_version": 1,
                        "task_id": task_id,
                        "goal_id": mission["simulation_goal_id"],
                        "status": "failed",
                        "message": reason,
                    }
                    self._publish(self._simulation_pub, payload)
                else:
                    rospy.logwarn(
                        "phase2 failure before simulation goal; no arrival id"
                    )
                    payload = None
                if payload is not None:
                    self._gate.record_result(task_id, event, reason)
                    topic = (
                        "/task/simulation_arrived"
                        if "simulation_goal_id" in mission and self._phase1_done
                        else "/task/delivery_arrived"
                    )
                    self._terminal_outputs[task_id] = [(topic, dict(payload))]
                return
            rospy.logwarn("ignored unknown mission event: %s", event)

    def _republish_terminal(self, task_id):
        for topic, payload in self._terminal_outputs.get(task_id, []):
            publisher = self._publishers.get(topic)
            if publisher is not None:
                self._publish(publisher, payload)


def main():
    rospy.init_node("stop_protocol_adapter")
    StopProtocolAdapterNode()
    rospy.loginfo("stop_protocol_adapter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
