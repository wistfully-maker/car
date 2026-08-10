#!/usr/bin/env python3
"""Stop 栈内部速度选择器：只发布 /cmd_vel/stop，绝不发布最终 /cmd_vel。

来源：
    /cmd_vel/stop_navigation   —— stop move_base（TEB 动态避障输出）
    /cmd_vel/stop_manual       —— mission 的 PCA/白框停车/倒车手动输出
模式（/stop/motion_mode，mission 发布，latch）：
    IDLE / NAVIGATION / MANUAL；未知模式回落 IDLE。
模式切换、未知模式、输入陈旧、非法数值、时间回退、异常和 shutdown
一律先输出零速度。
"""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from stop_integration.velocity_mux import (
    VelocityMux,
    validate_positive_finite,
)


def _ros_twist(value):
    message = Twist()
    for vector_name in ("linear", "angular"):
        source = getattr(value, vector_name)
        target = getattr(message, vector_name)
        for axis in ("x", "y", "z"):
            setattr(target, axis, float(getattr(source, axis)))
    return message


class StopVelocityMuxNode:
    def __init__(self):
        try:
            timeout = validate_positive_finite(
                "source_timeout",
                rospy.get_param("~velocity_mux/source_timeout", 0.3),
            )
            period = validate_positive_finite(
                "check_period",
                rospy.get_param("~velocity_mux/check_period", 0.05),
            )
            max_linear = validate_positive_finite(
                "max_linear_abs",
                rospy.get_param("~velocity_mux/max_linear_abs", 1.0),
            )
            max_angular = validate_positive_finite(
                "max_angular_abs",
                rospy.get_param("~velocity_mux/max_angular_abs", 2.0),
            )
        except ValueError as exc:
            rospy.logerr("invalid velocity_mux configuration: %s", exc)
            raise
        self._lock = threading.RLock()
        self._shutdown = False
        self._publisher = rospy.Publisher(
            "/cmd_vel/stop", Twist, queue_size=1, latch=False
        )
        self._mux = VelocityMux(timeout, max_linear, max_angular)
        rospy.Subscriber(
            "/stop/motion_mode", String, self._on_mode, queue_size=1
        )
        rospy.Subscriber(
            "/cmd_vel/stop_navigation", Twist,
            lambda msg: self._on_source("navigation", msg), queue_size=1,
        )
        rospy.Subscriber(
            "/cmd_vel/stop_manual", Twist,
            lambda msg: self._on_source("manual", msg), queue_size=1,
        )
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)

    def _publish(self, value, force=False):
        if value is None:
            return
        with self._lock:
            if not force and self._shutdown:
                return
            try:
                self._publisher.publish(_ros_twist(value))
            except Exception as exc:
                rospy.logerr("stop velocity publish failed: %s", exc)

    def _on_mode(self, message):
        with self._lock:
            if self._shutdown:
                return
            try:
                output = self._mux.set_mode(message.data, rospy.get_time())
            except Exception as exc:
                rospy.logerr("velocity mux mode failed: %s", exc)
                output = self._mux._zero()
        self._publish(output)

    def _on_source(self, source, message):
        with self._lock:
            if self._shutdown:
                return
            try:
                output = self._mux.accept(source, message, rospy.get_time())
            except Exception as exc:
                rospy.logerr("velocity mux source %s failed: %s", source, exc)
                output = self._mux._zero()
        self._publish(output)

    def _on_timer(self, _event):
        with self._lock:
            if self._shutdown:
                return
            try:
                output = self._mux.tick(rospy.get_time())
            except Exception as exc:
                rospy.logerr("velocity mux tick failed: %s", exc)
                output = self._mux._zero()
        self._publish(output)

    def _on_shutdown(self):
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            zero = self._mux._zero()
        self._publish(zero, force=True)


def main():
    rospy.init_node("stop_velocity_mux")
    StopVelocityMuxNode()
    rospy.loginfo("stop_velocity_mux node started")
    rospy.spin()


if __name__ == "__main__":
    main()
