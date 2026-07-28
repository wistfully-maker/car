#!/usr/bin/env python3

"""ROS velocity supervisor that executes large path corners in place."""

import json
import math
import threading

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rosgraph
import rospy
from std_msgs.msg import String
import tf2_ros

from corner_geometry import (
    Supervisor,
    SupervisorConfig,
    find_first_corner,
)


RAW_COMMAND_TOPIC = "/move_base/cmd_vel_raw"
GLOBAL_PLAN_TOPIC = "/move_base/NavfnROS/plan"
MOVE_BASE_STATUS_TOPIC = "/move_base/status"
OUTPUT_COMMAND_TOPIC = "/cmd_vel"
STATE_TOPIC = "~state"
DIAGNOSTIC_TOPIC = "~diagnostic"


def quaternion_yaw(rotation):
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class CornerSupervisorNode:
    def __init__(self):
        self._lock = threading.RLock()
        self._raw_command = (0.0, 0.0, 0.0)
        self._raw_stamp = None
        self._path = []
        self._path_stamp = None
        self._goal_active = False
        self._last_state = None

        self._rate = float(rospy.get_param("~controller_rate", 20.0))
        self._raw_timeout = float(
            rospy.get_param("~raw_command_timeout", 0.5)
        )
        self._path_timeout = float(rospy.get_param("~path_timeout", 0.0))
        self._search_distance = float(
            rospy.get_param("~path_search_distance", 1.2)
        )
        self._min_corner_angle = math.radians(
            float(rospy.get_param("~min_corner_angle_deg", 45.0))
        )
        self._spacing = float(
            rospy.get_param("~path_resample_spacing", 0.05)
        )
        self._direction_window = float(
            rospy.get_param("~direction_window", 0.20)
        )
        config = SupervisorConfig(
            trigger_distance=float(
                rospy.get_param("~corner_trigger_distance", 0.25)
            ),
            release_distance=float(
                rospy.get_param("~corner_release_distance", 0.45)
            ),
            following_max_lateral=float(
                rospy.get_param("~following_max_lateral", 0.02)
            ),
            turn_max_angular=float(
                rospy.get_param("~turn_max_angular", 0.35)
            ),
            turn_min_angular=float(
                rospy.get_param("~turn_min_angular", 0.18)
            ),
            turn_kp=float(rospy.get_param("~turn_kp", 0.9)),
            heading_tolerance=math.radians(
                float(rospy.get_param("~heading_tolerance_deg", 8.0))
            ),
            heading_hold_time=float(
                rospy.get_param("~heading_hold_time", 0.30)
            ),
            turn_timeout=float(rospy.get_param("~turn_timeout", 8.0)),
        )
        self._supervisor = Supervisor(config)

        self._assert_cmd_vel_is_unowned()
        self._command_publisher = rospy.Publisher(
            OUTPUT_COMMAND_TOPIC, Twist, queue_size=1
        )
        self._state_publisher = rospy.Publisher(
            STATE_TOPIC, String, queue_size=1, latch=True
        )
        self._diagnostic_publisher = rospy.Publisher(
            DIAGNOSTIC_TOPIC, String, queue_size=10
        )
        rospy.Subscriber(
            RAW_COMMAND_TOPIC, Twist, self._raw_command_callback, queue_size=1
        )
        rospy.Subscriber(
            GLOBAL_PLAN_TOPIC, Path, self._path_callback, queue_size=1
        )
        rospy.Subscriber(
            MOVE_BASE_STATUS_TOPIC,
            GoalStatusArray,
            self._status_callback,
            queue_size=1,
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.on_shutdown(self.stop)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._rate), self._control_callback
        )

    @staticmethod
    def _assert_cmd_vel_is_unowned():
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        owners = next(
            (nodes for topic, nodes in publishers if topic == OUTPUT_COMMAND_TOPIC),
            [],
        )
        if owners:
            raise RuntimeError(
                "{} already has publishers: {}. Remap move_base output to {} "
                "and stop legacy velocity controllers first.".format(
                    OUTPUT_COMMAND_TOPIC, ", ".join(owners), RAW_COMMAND_TOPIC
                )
            )

    def _raw_command_callback(self, message):
        with self._lock:
            self._raw_command = (
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
            self._raw_stamp = rospy.Time.now()

    def _path_callback(self, message):
        with self._lock:
            self._path = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses
            ]
            self._path_stamp = rospy.Time.now()

    def _status_callback(self, message):
        active_codes = (GoalStatus.PENDING, GoalStatus.ACTIVE)
        goal_active = any(
            status.status in active_codes for status in message.status_list
        )
        with self._lock:
            if goal_active and not self._goal_active:
                self._path = []
                self._path_stamp = None
            self._goal_active = goal_active

    @staticmethod
    def _remaining_path(path, x, y):
        if not path:
            return []
        nearest = min(
            range(len(path)),
            key=lambda index: (path[index][0] - x) ** 2
            + (path[index][1] - y) ** 2,
        )
        return path[nearest:]

    def _control_callback(self, _event):
        now = rospy.Time.now()
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0), rospy.Duration(0.05)
            )
            translation = transform.transform.translation
            yaw = quaternion_yaw(transform.transform.rotation)
            tf_valid = True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            translation = None
            yaw = 0.0
            tf_valid = False
            rospy.logwarn_throttle(2.0, "corner supervisor TF: %s", error)

        with self._lock:
            raw_command = self._raw_command
            raw_stamp = self._raw_stamp
            path = list(self._path)
            path_stamp = self._path_stamp
            goal_active = self._goal_active

        raw_fresh = (
            raw_stamp is not None
            and (now - raw_stamp).to_sec() <= self._raw_timeout
        )
        path_fresh = path_stamp is not None and (
            self._path_timeout <= 0.0
            or (now - path_stamp).to_sec() <= self._path_timeout
        )
        observation = None
        if tf_valid and path_fresh:
            remaining = self._remaining_path(
                path, translation.x, translation.y
            )
            observation = find_first_corner(
                remaining,
                search_distance=self._search_distance,
                min_angle=self._min_corner_angle,
                spacing=self._spacing,
                direction_window=self._direction_window,
            )

        result = self._supervisor.update(
            now=now.to_sec(),
            goal_active=goal_active,
            raw_fresh=raw_fresh,
            tf_valid=tf_valid,
            yaw=yaw,
            corner=observation,
            raw_command=raw_command,
        )
        self._publish_command(result.command)
        if result.state != self._last_state:
            rospy.loginfo("corner supervisor state: %s", result.state)
            self._state_publisher.publish(String(data=result.state))
            self._last_state = result.state
        self._diagnostic_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "state": result.state,
                        "corner_distance": (
                            observation.distance if observation else None
                        ),
                        "corner_angle_deg": (
                            math.degrees(observation.turn_angle)
                            if observation
                            else None
                        ),
                        "raw_fresh": raw_fresh,
                        "path_fresh": path_fresh,
                        "tf_valid": tf_valid,
                        "message": result.message,
                    },
                    ensure_ascii=False,
                )
            )
        )

    def _publish_command(self, command):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        self._command_publisher.publish(message)

    def stop(self):
        if hasattr(self, "_command_publisher"):
            self._publish_command((0.0, 0.0, 0.0))


def main():
    rospy.init_node("ucar_corner_supervisor")
    try:
        CornerSupervisorNode()
    except Exception as error:
        rospy.logfatal("corner supervisor failed to start: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
