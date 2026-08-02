#!/usr/bin/env python3
"""Adapt pickup task goals to move_base and report settled arrival."""

import json
import math
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from task_orchestrator.fast_nav_logic import (
    ActionOperationCoordinator,
    FastNavSession,
    StopDetector,
    normalize_goal,
    validate_waypoint,
)
from task_orchestrator.protocol import ProtocolError, parse_cancel


def quaternion_from_euler(roll, pitch, yaw):
    """Return an (x, y, z, w) quaternion without adding a tf dependency."""
    del roll, pitch
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class FastNavAdapter:
    def __init__(self):
        waypoint = validate_waypoint(
            rospy.get_param("/ucar_fast_nav/pickup_goal")
        )
        self._session = FastNavSession(waypoint)
        self._detector = StopDetector(
            rospy.get_param(
                "~fast_nav_adapter/linear_stop_threshold", 0.03
            ),
            rospy.get_param(
                "~fast_nav_adapter/angular_stop_threshold", 0.05
            ),
            rospy.get_param("~fast_nav_adapter/settle_time", 0.5),
        )
        self._action_timeout = float(
            rospy.get_param("~fast_nav_adapter/action_timeout", 300.0)
        )
        if not math.isfinite(self._action_timeout) or self._action_timeout <= 0:
            raise ValueError("action_timeout must be positive and finite")

        self._lock = threading.RLock()
        self._action_operations = ActionOperationCoordinator()
        self._phase = "idle"
        self._action_identity = None
        self._action_started = None
        self._publisher = rospy.Publisher(
            "/task/pickup_arrived", String, queue_size=10
        )
        self._client = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction
        )
        rospy.Subscriber(
            "/task/pickup_navigation_goal", String, self._on_goal
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

    def _move_base_goal(self):
        waypoint = self._session.waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = waypoint["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint["x"]
        goal.target_pose.pose.position.y = waypoint["y"]
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint["yaw"])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        return goal

    def _on_goal(self, message):
        try:
            parsed = normalize_goal(message.data)
            requested_identity = {
                "task_id": parsed["task_id"],
                "goal_id": parsed["goal_id"],
            }
            with self._lock:
                if self._session.active_identity == requested_identity:
                    return
        except (ProtocolError, TypeError, ValueError) as exc:
            rospy.logwarn("ignored invalid pickup navigation goal: %s", exc)
            return

        generation = self._action_operations.issue()

        def replace_goal(still_current):
            with self._lock:
                old_identity = self._session.active_identity
                if not self._session.accept_goal(parsed):
                    return
                identity = self._session.active_identity
                replacing = old_identity is not None
                self._detector.reset()
                self._phase = "navigating"
                self._action_identity = dict(identity, generation=generation)
                self._action_started = rospy.get_time()
                goal = self._move_base_goal()
            if replacing:
                self._client.cancel_goal()
            if not still_current():
                return
            task_id = identity["task_id"]
            goal_id = identity["goal_id"]
            self._client.send_goal(
                goal,
                done_cb=lambda status, result: self._on_done(
                    generation, task_id, goal_id, status, result
                ),
            )
            if not still_current():
                self._client.cancel_goal()

        try:
            self._action_operations.run_if_current(generation, replace_goal)
        except Exception as exc:
            rospy.logerr("move_base goal operation failed: %s", exc)
            payload = self._fail_generation(
                generation, "move_base goal operation failed: %s" % exc
            )
            self._publish(payload)

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
                task_id,
                goal_id,
                succeeded=False,
                message="move_base finished with status %s" % status,
            )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
        self._publish(payload)

    def _on_odom(self, message):
        payload = None
        with self._lock:
            if self._phase != "settling":
                return
            twist = message.twist.twist
            linear_speed = math.hypot(twist.linear.x, twist.linear.y)
            angular_speed = abs(twist.angular.z)
            if not self._detector.update(
                linear_speed, angular_speed, rospy.get_time()
            ):
                return
            identity = self._session.active_identity
            if identity is not None:
                payload = self._session.complete(
                    identity["task_id"], identity["goal_id"], succeeded=True
                )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
        self._publish(payload)

    def _on_cancel(self, message):
        with self._lock:
            identity = self._session.active_identity
            if identity is None:
                return
            expected_task_id = identity["task_id"]
        try:
            parsed = parse_cancel(message.data, expected_task_id)
            task_id = parsed["task_id"]
            reason = parsed["reason"]
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid navigation cancel: %s", exc)
            return
        generation = self._action_operations.issue()
        result = [None]

        def cancel_current(_still_current):
            with self._lock:
                identity = self._session.active_identity
                if identity is None or identity["task_id"] != task_id:
                    return
                result[0] = self._session.cancel(task_id, reason)
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
            self._client.cancel_goal()

        try:
            self._action_operations.run_if_current(generation, cancel_current)
        except Exception as exc:
            rospy.logerr("move_base cancel failed: %s", exc)
        payload = result[0]
        self._publish(payload)

    def _on_timer(self, _event):
        with self._lock:
            if self._phase not in ("navigating", "settling") or self._action_started is None:
                return
            if rospy.get_time() - self._action_started < self._action_timeout:
                return
            timed_out_identity = self._session.active_identity
        generation = self._action_operations.issue()
        result = [None]

        def cancel_timed_out(_still_current):
            with self._lock:
                if self._session.active_identity != timed_out_identity:
                    return
                result[0] = self._session.complete(
                    timed_out_identity["task_id"],
                    timed_out_identity["goal_id"],
                    succeeded=False,
                    message="move_base action timed out",
                )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
            self._client.cancel_goal()

        try:
            self._action_operations.run_if_current(generation, cancel_timed_out)
        except Exception as exc:
            rospy.logerr("move_base timeout cancel failed: %s", exc)
        self._publish(result[0])

    def _on_shutdown(self):
        generation = self._action_operations.issue()

        def cancel_for_shutdown(_still_current):
            with self._lock:
                identity = self._session.active_identity
                should_cancel = identity is not None
                if identity is not None:
                    self._session.complete(
                        identity["task_id"],
                        identity["goal_id"],
                        succeeded=False,
                        message="ROS shutdown",
                    )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
            if should_cancel:
                self._client.cancel_goal()

        try:
            self._action_operations.run_if_current(
                generation, cancel_for_shutdown
            )
        except Exception as exc:
            rospy.logerr("move_base shutdown cancel failed: %s", exc)


def main():
    rospy.init_node("fast_nav_adapter")
    FastNavAdapter()
    rospy.loginfo("fast_nav_adapter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
