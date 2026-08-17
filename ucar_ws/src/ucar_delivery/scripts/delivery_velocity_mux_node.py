#!/usr/bin/env python3
"""Standalone delivery velocity mux node (Task 4).

Subscribes the three isolated motion sources and the private motion mode
topic, and is the ONLY node publishing final /cmd_vel in standalone mode.
Publishes zero on initialization, stale input, invalid mode, exceptions,
emergency stop and shutdown.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ucar_delivery.velocity_mux import Command, VelocityMux, ZERO


class DeliveryVelocityMuxNode:
    def __init__(self):
        self._source_timeout = rospy.get_param(
            "~velocity_mux/source_timeout", 0.3
        )
        self._publish_rate = rospy.get_param(
            "~velocity_mux/publish_rate", 20.0
        )
        self._cmd_vel = rospy.get_param(
            "~topics/cmd_vel", "/cmd_vel"
        )
        self._motion_mode_topic = rospy.get_param(
            "~topics/motion_mode", "/ucar_delivery/motion_mode"
        )
        self._navigation_topic = rospy.get_param(
            "~topics/cmd_vel_navigation", "/cmd_vel/delivery_navigation"
        )
        self._manual_topic = rospy.get_param(
            "~topics/cmd_vel_manual", "/cmd_vel/delivery_manual"
        )
        self._parking_topic = rospy.get_param(
            "~topics/cmd_vel_parking", "/cmd_vel/delivery_parking"
        )
        self._mux = VelocityMux(rospy.get_time, self._source_timeout)
        self._pub = rospy.Publisher(self._cmd_vel, Twist, queue_size=1)
        rospy.Subscriber(
            self._navigation_topic, Twist,
            self._make_twist_callback("navigation"), queue_size=1,
        )
        rospy.Subscriber(
            self._manual_topic, Twist,
            self._make_twist_callback("manual"), queue_size=1,
        )
        rospy.Subscriber(
            self._parking_topic, Twist,
            self._make_twist_callback("parking"), queue_size=1,
        )
        rospy.Subscriber(
            self._motion_mode_topic, String, self._on_mode, queue_size=1,
        )
        rospy.Timer(
            rospy.Duration(1.0 / max(self._publish_rate, 1.0)),
            self._on_tick,
        )
        rospy.on_shutdown(self._on_shutdown)
        self._publish_zero()

    def _make_twist_callback(self, source):
        def callback(message):
            command = Command(
                linear_x=message.linear.x,
                linear_y=message.linear.y,
                angular_z=message.angular.z,
            )
            try:
                self._mux.update(source, command, rospy.get_time())
            except Exception as exc:
                rospy.logerr("velocity mux update failed: %s", exc)
                self._publish_zero()
        return callback

    def _on_mode(self, message):
        try:
            self._mux.set_mode(message.data, rospy.get_time())
        except Exception as exc:
            rospy.logerr("velocity mux mode failed: %s", exc)
            self._mux.set_mode(VelocityMux.IDLE, rospy.get_time())
        self._publish_current()

    def _on_tick(self, _event):
        try:
            self._publish_current()
        except Exception as exc:
            rospy.logerr("velocity mux tick failed: %s", exc)
            self._publish_zero()

    def _publish_current(self):
        command = self._mux.output(rospy.get_time())
        twist = Twist()
        twist.linear.x = command.linear_x
        twist.linear.y = command.linear_y
        twist.linear.z = command.linear_z
        twist.angular.x = command.angular_x
        twist.angular.y = command.angular_y
        twist.angular.z = command.angular_z
        self._pub.publish(twist)

    def _publish_zero(self):
        twist = Twist()
        self._pub.publish(twist)

    def _on_shutdown(self):
        self._mux.shutdown(rospy.get_time())
        self._publish_zero()


def main():
    rospy.init_node("delivery_velocity_mux")
    DeliveryVelocityMuxNode()
    rospy.loginfo("delivery_velocity_mux node started")
    rospy.spin()


if __name__ == "__main__":
    main()
