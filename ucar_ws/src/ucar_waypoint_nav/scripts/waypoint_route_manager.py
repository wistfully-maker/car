#!/usr/bin/env python3

"""Dispatch pass-through waypoint goals to move_base."""

import hashlib
import json
import math
import pathlib
import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros
import yaml

from ucar_waypoint_nav.protocol import (
    TaskIdentity,
    make_pickup_result,
    parse_pickup_goal,
)
from ucar_waypoint_nav.waypoint_logic import RouteProgress, Waypoint


FAILURE_STATES = {
    GoalStatus.PREEMPTED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
    GoalStatus.LOST,
}


def quaternion_from_yaw(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def load_waypoints(path):
    document = yaml.safe_load(
        pathlib.Path(path).read_text(encoding="utf-8")
    )
    waypoints = []
    payloads = document["waypoints"]
    for index, payload in enumerate(payloads):
        yaw = float(payload["yaw"])
        if str(payload["kind"]) == "pass_through":
            next_payload = payloads[index + 1]
            yaw = math.atan2(
                float(next_payload["y"]) - float(payload["y"]),
                float(next_payload["x"]) - float(payload["x"]),
            )
        waypoints.append(
            Waypoint(
                name=str(payload["name"]),
                kind=str(payload["kind"]),
                x=float(payload["x"]),
                y=float(payload["y"]),
                yaw=yaw,
                switch_radius=float(payload.get("switch_radius", 0.35)),
                exit_radius=float(payload.get("exit_radius", 0.45)),
                heading_tolerance=float(
                    payload.get("heading_tolerance", math.radians(40.0))
                ),
                minimum_pass_speed=float(
                    payload.get("minimum_pass_speed", 0.08)
                ),
                position_tolerance=float(
                    payload.get("position_tolerance", 0.15)
                ),
                yaw_tolerance=float(
                    payload.get("yaw_tolerance", 0.15)
                ),
                settle_time=float(payload.get("settle_time", 0.5)),
            )
        )
    return document, waypoints


class WaypointRouteManager:
    def __init__(self):
        self._lock = threading.RLock()
        waypoint_file = rospy.get_param("~waypoint_file")
        self._document, self._waypoints = load_waypoints(waypoint_file)
        self._verify_map_hash(rospy.get_param("~map_image"))

        self._start_xy_tolerance = float(
            rospy.get_param("~start_xy_tolerance", 0.20)
        )
        self._start_yaw_tolerance = float(
            rospy.get_param("~start_yaw_tolerance", 0.25)
        )
        self._route_timeout = float(
            rospy.get_param("~route_timeout", 300.0)
        )
        self._pass_speed_grace = float(
            rospy.get_param("~pass_speed_grace", 0.50)
        )
        self._terminal_speed_tolerance = float(
            rospy.get_param("~terminal_speed_tolerance", 0.03)
        )
        self._state = "IDLE"
        self._progress = None
        self._identity = None
        self._started_at = None
        self._low_speed_since = None
        self._speed = 0.0
        self._completed_goal_ids = set()

        self._state_pub = rospy.Publisher(
            "/ucar_waypoint_nav/state", String, queue_size=1, latch=True
        )
        self._diagnostic_pub = rospy.Publisher(
            "/ucar_waypoint_nav/diagnostic",
            String,
            queue_size=10,
        )
        self._result_pub = rospy.Publisher(
            "/task/pickup_arrived", String, queue_size=10
        )
        rospy.Subscriber(
            "/task/pickup_navigation_goal",
            String,
            self._goal_callback,
            queue_size=10,
        )
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=20)
        rospy.Service("/ucar_waypoint_nav/start", Trigger, self._start_service)
        rospy.Service("/ucar_waypoint_nav/cancel", Trigger, self._cancel_service)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        wait = float(rospy.get_param("~move_base_wait_timeout", 30.0))
        if not self._client.wait_for_server(rospy.Duration(wait)):
            raise RuntimeError("move_base action server is unavailable")
        rospy.on_shutdown(self._shutdown)
        self._publish_state("IDLE")
        self._timer = rospy.Timer(rospy.Duration(0.05), self._tick)

    def _verify_map_hash(self, map_image):
        actual = hashlib.sha256(pathlib.Path(map_image).read_bytes()).hexdigest()
        expected = str(self._document["map_sha256"])
        if actual != expected:
            raise RuntimeError(
                "map image checksum mismatch: expected {}, got {}".format(
                    expected, actual
                )
            )

    def _publish_state(self, state):
        if state != self._state:
            self._state = state
        self._state_pub.publish(String(data=state))

    def _odom_callback(self, message):
        velocity = message.twist.twist.linear
        self._speed = math.hypot(velocity.x, velocity.y)

    def _lookup_pose(self):
        transform = self._tf_buffer.lookup_transform("map", "base_link",
            rospy.Time(0), rospy.Duration(0.10))
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return translation.x, translation.y, yaw

    def _validate_start(self, pose):
        x, y, yaw = pose
        if math.hypot(x, y) > self._start_xy_tolerance:
            raise ValueError("robot is outside the configured P start pose")
        error = (yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(error) > self._start_yaw_tolerance:
            raise ValueError("robot yaw does not match the P start pose")

    def _make_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y
        goal.target_pose.pose.orientation = quaternion_from_yaw(waypoint.yaw)
        return goal

    def _send_current_goal(self):
        waypoint = self._progress.current
        self._client.send_goal(self._make_goal(waypoint))
        rospy.loginfo(
            "waypoint navigation goal %s (%s): %.3f %.3f %.3f",
            waypoint.name,
            waypoint.kind,
            waypoint.x,
            waypoint.y,
            waypoint.yaw,
        )

    def _begin(self, identity):
        if self._progress is not None:
            raise ValueError("a pickup navigation task is already active")
        if identity.goal_id in self._completed_goal_ids:
            raise ValueError("goal_id has already completed")
        pose = self._lookup_pose()
        self._validate_start(pose)
        self._identity = identity
        self._progress = RouteProgress(
            self._waypoints,
            terminal_speed_tolerance=self._terminal_speed_tolerance,
        )
        self._started_at = rospy.Time.now().to_sec()
        self._low_speed_since = None
        self._publish_state("NAVIGATING")
        self._send_current_goal()

    def _goal_callback(self, message):
        with self._lock:
            try:
                self._begin(parse_pickup_goal(message.data))
            except Exception as error:
                rospy.logerr("pickup navigation goal rejected: %s", error)

    def _start_service(self, _request):
        with self._lock:
            try:
                stamp = int(rospy.Time.now().to_nsec())
                self._begin(
                    TaskIdentity(1, "manual", "manual-{}".format(stamp))
                )
                return TriggerResponse(True, "pickup route started")
            except Exception as error:
                return TriggerResponse(False, str(error))

    def _finish(self, status, message):
        identity = self._identity
        if identity is not None:
            self._result_pub.publish(
                String(
                    data=make_pickup_result(
                        identity, status, message
                    )
                )
            )
            self._completed_goal_ids.add(identity.goal_id)
        self._progress = None
        self._identity = None
        self._started_at = None
        self._low_speed_since = None
        self._publish_state(
            "ARRIVED" if status == "arrived" else "ERROR"
        )

    def _cancel_service(self, _request):
        with self._lock:
            if self._progress is None:
                return TriggerResponse(True, "no active pickup route")
            self._client.cancel_all_goals()
            self._finish("cancelled", "cancelled by operator")
            self._publish_state("CANCELLED")
            return TriggerResponse(True, "pickup route cancelled")

    def _publish_diagnostic(self, pose, decision):
        waypoint = self._progress.current
        payload = {
            "state": self._state,
            "waypoint_index": self._progress.current_index,
            "waypoint_name": waypoint.name,
            "waypoint_kind": waypoint.kind,
            "distance": math.hypot(waypoint.x - pose[0], waypoint.y - pose[1]),
            "speed": self._speed,
            "action_state": self._client.get_state(),
            "decision_error": decision.error,
        }
        self._diagnostic_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    separators=(",", ":"),
                    sort_keys=True,
                )
            )
        )

    def _tick(self, _event):
        with self._lock:
            if self._progress is None:
                return
            now = rospy.Time.now().to_sec()
            if now - self._started_at > self._route_timeout:
                self._client.cancel_all_goals()
                self._finish("failed", "pickup route timed out")
                return
            try:
                pose = self._lookup_pose()
            except Exception as error:
                rospy.logwarn_throttle(1.0, "waypoint TF unavailable: %s", error)
                return
            decision = self._progress.update(
                x=pose[0],
                y=pose[1],
                yaw=pose[2],
                linear_speed=self._speed,
                now=now,
            )
            self._publish_diagnostic(pose, decision)

            if decision.error:
                if self._low_speed_since is None:
                    self._low_speed_since = now
                elif now - self._low_speed_since >= self._pass_speed_grace:
                    self._client.cancel_all_goals()
                    self._finish("failed", decision.error)
                return
            self._low_speed_since = None

            if decision.send_next_goal:
                self._publish_state("SWITCHING")
                self._send_current_goal()
                self._publish_state("NAVIGATING")
                return
            if decision.arrived:
                self._finish("arrived", "")
                return

            action_state = self._client.get_state()
            if action_state in FAILURE_STATES:
                self._finish(
                    "failed",
                    "move_base failed with state {}".format(action_state),
                )
            elif (
                action_state == GoalStatus.SUCCEEDED
                and self._progress.current.kind == "pass_through"
            ):
                decision = self._progress.advance_stopped_pass_through()
                if decision.send_next_goal:
                    self._publish_state("SWITCHING")
                    self._send_current_goal()
                    self._publish_state("NAVIGATING")

    def _shutdown(self):
        try:
            self._client.cancel_all_goals()
        except Exception:
            pass


def main():
    rospy.init_node("ucar_waypoint_route_manager")
    try:
        WaypointRouteManager()
    except Exception as error:
        rospy.logfatal("waypoint route manager failed to start: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
