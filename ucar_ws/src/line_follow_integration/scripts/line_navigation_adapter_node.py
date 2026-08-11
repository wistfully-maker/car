#!/usr/bin/env python3
"""Adapt line-start navigation goals to move_base and report settled arrival.

按 fast_nav_adapter_node.py 的模式实现：generation 保护、settled-odom 检测、
300 秒超时、/task/cancel、过期回调抑制与每个身份一次终态到达。
MoveBaseGoal 直接使用传入 pose 的 frame_id/x/y/qz/qw。
"""

import json
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from line_follow_integration.protocol import (
    ProtocolError,
    build_arrival,
    parse_cancel,
    parse_navigation_goal,
)


class GenerationCoordinator:
    """Serialize external action-client calls while suppressing stale work."""

    def __init__(self):
        self._lock = threading.Lock()
        self._operation_lock = threading.Lock()
        self._generation = 0

    def issue(self):
        with self._lock:
            self._generation += 1
            return self._generation

    def is_current(self, generation):
        with self._lock:
            return generation == self._generation

    def run_if_current(self, generation, operation):
        with self._operation_lock:
            if not self.is_current(generation):
                return False
            operation(lambda: self.is_current(generation))
            return self.is_current(generation)


class LineNavigationSession:
    """Track one active line-navigation identity; emit one terminal arrival."""

    def __init__(self):
        self._lock = threading.RLock()
        self._active_identity = None

    @property
    def active_identity(self):
        with self._lock:
            if self._active_identity is None:
                return None
            return dict(self._active_identity)

    def accept_goal(self, task_id, goal_id):
        identity = {"task_id": task_id, "goal_id": goal_id}
        with self._lock:
            if identity == self._active_identity:
                return False
            self._active_identity = identity
            return True

    def complete(self, task_id, goal_id, succeeded, message=""):
        with self._lock:
            if self._active_identity != {
                "task_id": task_id,
                "goal_id": goal_id,
            }:
                return None
            self._active_identity = None
            return build_arrival(task_id, goal_id, succeeded, message)

    def cancel(self, task_id, reason):
        with self._lock:
            identity = self._active_identity
            if identity is None or identity["task_id"] != task_id:
                return None
            return self.complete(
                identity["task_id"], identity["goal_id"], False, reason
            )


class SettledOdomDetector:
    """Detect continuous near-zero speed over a settle window."""

    def __init__(self, linear_threshold, angular_threshold, settle_time):
        self.linear_threshold = float(linear_threshold)
        self.angular_threshold = float(angular_threshold)
        self.settle_time = float(settle_time)
        self._stopped_since = None
        self._last_timestamp = None

    def reset(self):
        self._stopped_since = None
        self._last_timestamp = None

    def update(self, linear_speed, angular_speed, timestamp):
        if self._last_timestamp is not None and timestamp < self._last_timestamp:
            self._stopped_since = None
        self._last_timestamp = timestamp
        stopped = (
            abs(linear_speed) <= self.linear_threshold
            and abs(angular_speed) <= self.angular_threshold
        )
        if not stopped:
            self._stopped_since = None
            return False
        if self._stopped_since is None:
            self._stopped_since = timestamp
            return False
        return timestamp - self._stopped_since >= self.settle_time


def _parse_cancel(raw, expected_task_id):
    message = parse_cancel(raw)
    if message["task_id"] != expected_task_id:
        raise ProtocolError("task_id mismatch")
    return message["reason"]


class LineNavigationAdapter:
    def __init__(self):
        timeouts = rospy.get_param("~timeouts", {})
        self._action_timeout = float(timeouts.get("navigation", 300.0))
        if self._action_timeout <= 0:
            raise ValueError("navigation timeout must be positive")

        self._lock = threading.RLock()
        self._coordinator = GenerationCoordinator()
        self._session = LineNavigationSession()
        self._detector = SettledOdomDetector(
            rospy.get_param(
                "~line_navigation_adapter/linear_stop_threshold", 0.03
            ),
            rospy.get_param(
                "~line_navigation_adapter/angular_stop_threshold", 0.05
            ),
            rospy.get_param(
                "~line_navigation_adapter/settle_time", 0.5
            ),
        )
        self._phase = "idle"
        self._action_identity = None
        self._action_started = None
        self._current_pose = None
        self._publisher = rospy.Publisher(
            "/task/line_navigation_arrived", String, queue_size=10
        )
        self._client = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction
        )
        rospy.Subscriber(
            "/task/line_navigation_goal", String, self._on_goal
        )
        rospy.Subscriber("/task/cancel", String, self._on_cancel)
        rospy.Subscriber("/odom", Odometry, self._on_odom)
        self._timer = rospy.Timer(rospy.Duration(0.5), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)

    def _publish(self, payload):
        if payload is not None:
            self._publisher.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )

    def _move_base_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.orientation.z = pose["qz"]
        goal.target_pose.pose.orientation.w = pose["qw"]
        return goal

    def _on_goal(self, message):
        try:
            parsed = parse_navigation_goal(message.data)
            identity = {
                "task_id": parsed["task_id"],
                "goal_id": parsed["goal_id"],
            }
            with self._lock:
                if self._session.active_identity == identity:
                    return
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line navigation goal: %s", exc)
            return

        generation = self._coordinator.issue()

        def replace_goal(still_current):
            with self._lock:
                old_identity = self._session.active_identity
                if not self._session.accept_goal(
                    identity["task_id"], identity["goal_id"]
                ):
                    return
                self._detector.reset()
                self._phase = "navigating"
                self._action_identity = dict(identity, generation=generation)
                self._action_started = rospy.get_time()
                self._current_pose = dict(parsed["pose"])
                goal = self._move_base_goal(self._current_pose)
            if old_identity is not None:
                self._client.cancel_goal()
            if not still_current():
                return
            self._client.send_goal(
                goal,
                done_cb=lambda status, result: self._on_done(
                    generation, identity["task_id"], identity["goal_id"],
                    status, result,
                ),
            )
            if not still_current():
                self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, replace_goal)
        except Exception as exc:
            rospy.logerr("move_base line navigation failed: %s", exc)
            self._publish(
                self._fail_generation(
                    generation, "move_base line navigation failed: %s" % exc
                )
            )

    def _fail_generation(self, generation, message):
        with self._lock:
            identity = self._action_identity
            if identity is None or identity.get("generation") != generation:
                return None
            payload = self._session.complete(
                identity["task_id"], identity["goal_id"], False, message
            )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
            self._current_pose = None
            return payload

    def _on_done(self, generation, task_id, goal_id, status, _result):
        payload = None
        with self._lock:
            if self._action_identity != {
                "task_id": task_id,
                "goal_id": goal_id,
                "generation": generation,
            }:
                return
            if status == GoalStatus.SUCCEEDED:
                self._phase = "settling"
                self._detector.reset()
                return
            payload = self._session.complete(
                task_id, goal_id, False,
                "move_base finished with status %s" % status,
            )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
            self._current_pose = None
        self._publish(payload)

    def _on_odom(self, message):
        payload = None
        with self._lock:
            if self._phase != "settling":
                return
            twist = message.twist.twist
            linear_speed = (twist.linear.x ** 2 + twist.linear.y ** 2) ** 0.5
            angular_speed = abs(twist.angular.z)
            if not self._detector.update(
                linear_speed, angular_speed, rospy.get_time()
            ):
                return
            identity = self._session.active_identity
            if identity is not None:
                payload = self._session.complete(
                    identity["task_id"], identity["goal_id"], True
                )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
            self._current_pose = None
        self._publish(payload)

    def _on_cancel(self, message):
        with self._lock:
            identity = self._session.active_identity
            if identity is None:
                return
            expected_task_id = identity["task_id"]
        try:
            reason = _parse_cancel(message.data, expected_task_id)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line navigation cancel: %s", exc)
            return
        generation = self._coordinator.issue()
        result = [None]

        def cancel_current(_still_current):
            with self._lock:
                identity = self._session.active_identity
                if identity is None or identity["task_id"] != expected_task_id:
                    return
                result[0] = self._session.cancel(expected_task_id, reason)
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
            self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, cancel_current)
        except Exception as exc:
            rospy.logerr("move_base line navigation cancel failed: %s", exc)
        self._publish(result[0])

    def _on_timer(self, _event):
        with self._lock:
            if (
                self._phase not in ("navigating", "settling")
                or self._action_started is None
            ):
                return
            if rospy.get_time() - self._action_started < self._action_timeout:
                return
            timed_out_identity = self._session.active_identity
        generation = self._coordinator.issue()
        result = [None]

        def cancel_timed_out(_still_current):
            with self._lock:
                if self._session.active_identity != timed_out_identity:
                    return
                result[0] = self._session.complete(
                    timed_out_identity["task_id"],
                    timed_out_identity["goal_id"],
                    False,
                    "move_base line navigation timed out",
                )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
            self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, cancel_timed_out)
        except Exception as exc:
            rospy.logerr("move_base line navigation timeout cancel failed: %s",
                         exc)
        self._publish(result[0])

    def _on_shutdown(self):
        generation = self._coordinator.issue()

        def cancel_for_shutdown(_still_current):
            with self._lock:
                identity = self._session.active_identity
                should_cancel = identity is not None
                if identity is not None:
                    self._session.complete(
                        identity["task_id"], identity["goal_id"], False,
                        "ROS shutdown",
                    )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
            if should_cancel:
                self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, cancel_for_shutdown)
        except Exception as exc:
            rospy.logerr("move_base line navigation shutdown cancel failed: %s",
                         exc)


def main():
    rospy.init_node("line_navigation_adapter")
    LineNavigationAdapter()
    rospy.loginfo("line_navigation_adapter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
