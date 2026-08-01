#!/usr/bin/env python3
"""架空测试用: 发布虚拟里程计 (机器人不动), 供 AMCL + move_base 使用"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3


def main():
    rospy.init_node('fake_odom')

    pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(30)  # 30Hz

    rospy.loginfo("[fake_odom] 发布虚拟里程计 (静止在原点)")

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        # TF: odom → base_footprint (identity)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)

        # TF: base_footprint → base_link (identity, costmap 需要)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        br.sendTransform(t2)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.w = 1.0
        # 速度为零 (机器人不动)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0
        # 协方差 (AMCL 需要非零值)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        pub.publish(odom)

        rate.sleep()


if __name__ == '__main__':
    main()
