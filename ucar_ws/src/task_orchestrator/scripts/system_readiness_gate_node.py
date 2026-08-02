#!/usr/bin/env python3
"""Publish task-scoped readiness only after the orchestrator asks for it."""

import json
import threading
from collections import deque

import actionlib
import rosnode
import rospy
import tf2_ros
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from task_orchestrator.protocol import ProtocolError, load_object, require_text
from task_orchestrator.readiness import ReadinessSnapshot, missing_requirements


class SystemReadinessGate:
    _KNOWN_STATES = frozenset((
        "IDLE", "CHECKING_DEPENDENCIES", "NAVIGATING_TO_PICKUP",
        "WAITING_QR", "WAITING_LLM", "WAITING_SPEECH",
        "NAVIGATING_TO_WORKSHOP", "COMPLETE", "ERROR", "CANCELLED",
    ))

    def __init__(self):
        param = lambda key, default: rospy.get_param("~readiness_gate/" + key, default)
        self._max_age = float(param("message_max_age", 3.0))
        self._check_period = float(param("check_period", 1.0))
        self._tf_timeout = float(param("tf_timeout", 0.05))
        self._action_timeout = float(param("action_wait_timeout", 0.05))
        self._log_interval = float(param("log_interval", 10.0))
        self._map_frame = param("map_frame", "map")
        self._odom_frame = param("odom_frame", "odom")
        self._base_frame = param("base_frame", "base_link")
        self._laser_frame = param("laser_frame", "laser_frame")
        self._lidar_loc_node = param("lidar_loc_node", "/lidar_loc")
        self._amcl_node = param("amcl_node", "/amcl")
        self._global_planner_param = param(
            "global_planner_param", "/move_base/base_global_planner"
        )
        self._local_planner_param = param(
            "local_planner_param", "/move_base/base_local_planner"
        )
        self._stamps = {"scan": None, "odom": None}
        self._map_received = False
        self._last_check_time = None
        self._active_task_id = None
        self._active_generation = 0
        self._checks_in_progress = set()
        self._published_task_ids = deque(maxlen=64)
        self._last_log_time = None
        self._lock = threading.RLock()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self._publisher = rospy.Publisher("/task/dependencies_ready", String, queue_size=10)
        rospy.Subscriber("/task/status", String, self._on_status, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(self._check_period), self._on_timer)

    def _on_scan(self, _message):
        self._stamps["scan"] = rospy.get_time()

    def _on_odom(self, _message):
        self._stamps["odom"] = rospy.get_time()

    def _on_map(self, _message):
        self._map_received = True

    def _on_status(self, message):
        try:
            payload = load_object(message.data)
            raw_task_id = payload.get("task_id")
            task_id = require_text(raw_task_id, "task_id")
            if raw_task_id != task_id:
                raise ProtocolError("task_id must not contain surrounding whitespace")
            state = require_text(payload.get("state"), "state")
            require_text(payload.get("status"), "status")
            if not isinstance(payload.get("message"), str):
                raise ProtocolError("message must be text")
        except ProtocolError:
            return
        if state not in self._KNOWN_STATES:
            return
        with self._lock:
            if state == "CHECKING_DEPENDENCIES":
                self._active_generation += 1
                self._active_task_id = task_id
                return
            if task_id == self._active_task_id:
                self._active_generation += 1
                self._active_task_id = None

    def _can_transform(self, parent, child):
        try:
            return bool(self._tf_buffer.can_transform(
                parent, child, rospy.Time(0), rospy.Duration(self._tf_timeout)
            ))
        except Exception:
            return False

    @staticmethod
    def _node_live(name):
        try:
            return bool(rosnode.rosnode_ping(name, max_count=1, verbose=False))
        except Exception:
            return False

    @staticmethod
    def _get_param(name):
        try:
            return rospy.get_param(name, None)
        except Exception:
            return None

    def _snapshot(self, now):
        try:
            action_available = bool(
                self._move_base.wait_for_server(rospy.Duration(self._action_timeout))
            )
        except Exception:
            action_available = False
        return ReadinessSnapshot(
            now=now,
            scan_stamp=self._stamps["scan"],
            odom_stamp=self._stamps["odom"],
            map_received=self._map_received,
            tf_map_odom=self._can_transform(self._map_frame, self._odom_frame),
            tf_odom_base=self._can_transform(self._odom_frame, self._base_frame),
            tf_base_laser=self._can_transform(self._base_frame, self._laser_frame),
            move_base_available=action_available,
            lidar_loc_live=self._node_live(self._lidar_loc_node),
            amcl_live=self._node_live(self._amcl_node),
            global_planner=self._get_param(self._global_planner_param),
            local_planner=self._get_param(self._local_planner_param),
        )

    def _on_timer(self, _event):
        with self._lock:
            task_id = self._active_task_id
            if task_id is None or task_id in self._published_task_ids:
                return
            token = (task_id, self._active_generation)
            if token in self._checks_in_progress:
                return
            self._checks_in_progress.add(token)

        try:
            now = rospy.get_time()
            with self._lock:
                if self._last_check_time is not None and now < self._last_check_time:
                    self._stamps["scan"] = None
                    self._stamps["odom"] = None
                self._last_check_time = now
            missing = missing_requirements(self._snapshot(now), self._max_age)
        except Exception as exc:
            now = 0.0
            missing = [
                "readiness evaluation error: %s: %s"
                % (type(exc).__name__, exc)
            ]
        finally:
            with self._lock:
                self._checks_in_progress.discard(token)
        should_log = False
        publish_error = None
        with self._lock:
            still_current = (
                self._active_task_id == task_id
                and self._active_generation == token[1]
                and task_id not in self._published_task_ids
            )
            if not still_current:
                return
            if missing:
                if (
                    self._last_log_time is None
                    or now < self._last_log_time
                    or now - self._last_log_time >= self._log_interval
                ):
                    self._last_log_time = now
                    should_log = True
            else:
                payload = {
                    "protocol_version": 1,
                    "task_id": task_id,
                    "status": "ready",
                }
                try:
                    self._publisher.publish(
                        String(data=json.dumps(payload, ensure_ascii=False))
                    )
                except Exception as exc:
                    publish_error = exc
                else:
                    self._published_task_ids.append(task_id)

        if should_log:
            rospy.logwarn("readiness gate missing: %s", "; ".join(missing))
        if publish_error is not None:
            rospy.logerr(
                "readiness ready publish failed for %s: %s",
                task_id,
                publish_error,
            )


def main():
    rospy.init_node("system_readiness_gate")
    SystemReadinessGate()
    rospy.spin()


if __name__ == "__main__":
    main()
