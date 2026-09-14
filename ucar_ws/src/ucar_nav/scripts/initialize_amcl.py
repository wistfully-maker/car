#!/usr/bin/env python3
"""Publish a configurable initial pose to AMCL once."""

import math


def yaw_to_quaternion(yaw):
    half = float(yaw) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_covariance(cov_x, cov_y, cov_yaw):
    covariance = [0.0] * 36
    covariance[0] = float(cov_x)
    covariance[7] = float(cov_y)
    covariance[35] = float(cov_yaw)
    return covariance


def main():
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rospy.init_node("initialize_amcl", anonymous=True)
    frame_id = rospy.get_param("~frame_id", "map")
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    yaw = float(rospy.get_param("~yaw", 0.0))
    cov_x = float(rospy.get_param("~covariance_x", 0.10))
    cov_y = float(rospy.get_param("~covariance_y", 0.10))
    cov_yaw = float(rospy.get_param("~covariance_yaw", 0.0685))
    timeout = float(rospy.get_param("~subscriber_timeout", 10.0))

    publisher = rospy.Publisher(
        "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True
    )
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)
    while (
        not rospy.is_shutdown()
        and publisher.get_num_connections() == 0
        and rospy.Time.now() < deadline
    ):
        rate.sleep()

    if publisher.get_num_connections() == 0:
        rospy.logerr(
            "AMCL initialization timed out: /initialpose has no subscriber"
        )
        return 2

    message = PoseWithCovarianceStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = frame_id
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    quaternion = yaw_to_quaternion(yaw)
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]
    message.pose.covariance = build_covariance(cov_x, cov_y, cov_yaw)
    publisher.publish(message)
    rospy.loginfo(
        "Published AMCL initial pose: frame=%s x=%.3f y=%.3f yaw=%.3f",
        frame_id,
        x,
        y,
        yaw,
    )
    rospy.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
