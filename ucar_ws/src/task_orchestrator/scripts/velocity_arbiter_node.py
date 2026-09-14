#!/usr/bin/env python3
"""Publish the sole robot cmd_vel from navigation or QR search."""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from task_orchestrator.velocity_arbiter import VelocityArbiter, validate_positive_finite


def _ros_twist(value):
    message = Twist()
    for vector_name in ("linear", "angular"):
        source = getattr(value, vector_name)
        target = getattr(message, vector_name)
        for axis in ("x", "y", "z"):
            setattr(target, axis, float(getattr(source, axis)))
    return message


class VelocityArbiterNode:
    def __init__(self):
        try:
            timeout = validate_positive_finite(
                "source_timeout", rospy.get_param("~velocity_arbiter/source_timeout", 0.3))
            period = validate_positive_finite(
                "check_period", rospy.get_param("~velocity_arbiter/check_period", 0.05))
            max_linear = validate_positive_finite(
                "max_linear_abs", rospy.get_param("~velocity_arbiter/max_linear_abs", 1.0))
            max_angular = validate_positive_finite(
                "max_angular_abs", rospy.get_param("~velocity_arbiter/max_angular_abs", 2.0))
        except ValueError as exc:
            rospy.logerr("invalid velocity_arbiter configuration: %s", exc)
            raise
        self._state_lock = threading.RLock()
        self._publish_lock = threading.Lock()
        self._epoch = 0
        self._shutdown = False
        self._publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self._arbiter = VelocityArbiter(timeout, max_linear, max_angular)
        rospy.Subscriber("/task/motion_mode", String, self._on_mode, queue_size=1)
        rospy.Subscriber("/cmd_vel/navigation", Twist,
                         lambda msg: self._on_source("navigation", msg), queue_size=1)
        rospy.Subscriber("/cmd_vel/qr", Twist,
                         lambda msg: self._on_source("qr", msg), queue_size=1)
        rospy.Subscriber("/cmd_vel/stop", Twist,
                         lambda msg: self._on_source("stop", msg), queue_size=1)
        rospy.Subscriber("/cmd_vel/line_follow", Twist,
                         lambda msg: self._on_source("line_follow", msg), queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)
        self._publish_zero(force=True)

    def _publish_decision(self, value, epoch, allow_motion=False, force=False):
        if value is None:
            return
        with self._publish_lock:
            with self._state_lock:
                if not force and (self._shutdown or epoch != self._epoch):
                    return
                if allow_motion and self._arbiter.mode == "IDLE":
                    return
            try:
                self._publisher.publish(_ros_twist(value))
            except Exception as exc:
                rospy.logerr("velocity publish failed: %s", exc)

    def _publish_zero(self, force=False):
        with self._state_lock:
            epoch = self._epoch
            zero = self._arbiter._zero()
        self._publish_decision(zero, epoch, force=force)

    def _on_mode(self, message):
        with self._state_lock:
            if self._shutdown:
                return
            old_mode = self._arbiter.mode
            output = self._arbiter.set_mode(message.data, rospy.get_time())
            if self._arbiter.mode != old_mode or output is not None:
                self._epoch += 1
            epoch = self._epoch
        self._publish_decision(output, epoch)

    def _on_source(self, source, message):
        with self._state_lock:
            if self._shutdown:
                return
            output = self._arbiter.accept(source, message, rospy.get_time())
            epoch = self._epoch
        self._publish_decision(output, epoch, allow_motion=True)

    def _on_timer(self, _event):
        with self._state_lock:
            if self._shutdown:
                return
            output = self._arbiter.tick(rospy.get_time())
            epoch = self._epoch
        self._publish_decision(output, epoch)

    def _on_shutdown(self):
        with self._state_lock:
            if self._shutdown:
                return
            self._shutdown = True
            self._epoch += 1
            epoch = self._epoch
            zero = self._arbiter._zero()
        self._publish_decision(zero, epoch, force=True)


def main():
    rospy.init_node("velocity_arbiter")
    VelocityArbiterNode()
    rospy.loginfo("velocity_arbiter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
