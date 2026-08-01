#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ucar_waypoint_nav.odom_calibration import AngleAccumulator


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OneShotRotationCalibration(object):
    def __init__(self):
        self.speed = rospy.get_param("~speed", 0.30)
        self.target = math.radians(rospy.get_param("~target_degrees", 360.0))
        self.tolerance = math.radians(rospy.get_param("~tolerance_degrees", 1.0))
        self.max_duration = rospy.get_param("~max_duration", 30.0)
        self.odom_timeout = rospy.get_param("~odom_timeout", 1.0)
        self.rate_hz = rospy.get_param("~rate", 20.0)
        if self.speed <= 0.0:
            raise ValueError("speed must be positive for counterclockwise calibration")

        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.latest_yaw = None
        self.latest_odom_time = None
        self.subscriber = rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=10)
        rospy.on_shutdown(self.stop)

    def _odom_callback(self, msg):
        self.latest_yaw = quaternion_yaw(msg.pose.pose.orientation)
        self.latest_odom_time = rospy.Time.now()

    def stop(self):
        zero = Twist()
        for _ in range(10):
            self.publisher.publish(zero)
            rospy.sleep(0.05)

    def run(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        wait_rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.latest_yaw is None:
            if rospy.Time.now() >= deadline:
                raise RuntimeError("no /odom received within 5 seconds")
            wait_rate.sleep()

        tracker = AngleAccumulator(self.target, self.tolerance)
        tracker.start(self.latest_yaw)
        started = rospy.Time.now()
        command = Twist()
        command.angular.z = self.speed
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - started).to_sec() > self.max_duration:
                raise RuntimeError("rotation calibration exceeded max_duration")
            if self.latest_odom_time is None or (now - self.latest_odom_time).to_sec() > self.odom_timeout:
                raise RuntimeError("/odom timed out during rotation")

            tracker.update(self.latest_yaw)
            if tracker.done:
                self.stop()
                rospy.loginfo("rotation calibration complete: accumulated=%.6f rad (%.3f deg)",
                              tracker.accumulated, math.degrees(tracker.accumulated))
                return tracker.accumulated

            self.publisher.publish(command)
            rate.sleep()

        raise RuntimeError("ROS shutdown before calibration completed")


if __name__ == "__main__":
    rospy.init_node("calibrate_odom_rotation_once")
    node = OneShotRotationCalibration()
    try:
        node.run()
    except Exception as exc:
        rospy.logerr("rotation calibration failed: %s", exc)
        node.stop()
        raise
