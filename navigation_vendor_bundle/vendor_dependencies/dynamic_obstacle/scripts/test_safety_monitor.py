#!/usr/bin/env python3
"""测试脚本: 直发 TrackedObject → 验证安全监控 TTC + 紧急停车

用法:
  rosrun dynamic_obstacle test_safety_monitor.py
  rostopic echo /safety/cmd_vel
  rostopic echo /dynamic_obstacle/ttc
"""

import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from dynamic_obstacle.msg import TrackedObject, TrackedObjectArray


def main():
    rospy.init_node('test_safety_monitor')

    pub_objs = rospy.Publisher('/tracked_objects', TrackedObjectArray, queue_size=1)
    pub_odom = rospy.Publisher('/odom_test', Odometry, queue_size=1)
    # 注意: safety_monitor 订阅的是 /odom, 需要确保 odom 话题正常

    rate = rospy.Rate(10)
    t = 0.0
    rospy.loginfo("[test] 模拟行人从 (3, 0) 走向机器人 (0, 0), 速度 -0.5m/s")
    rospy.loginfo("[test] 预计 TTC 从 6s 逐渐下降到 0s, TTC<0.5s 时触发紧急停车")

    while not rospy.is_shutdown():
        # 发布 odom (机器人静止在原点)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        pub_odom.publish(odom)

        # 模拟障碍物: 从 (3, 0) 以 0.5 m/s 向原点移动
        start_dist = 3.0
        speed = -0.5  # 向原点移动
        ox = start_dist + speed * t
        oy = 0.0

        if ox > 0.05:  # 还没到达机器人位置
            obj = TrackedObject()
            obj.id = 1
            obj.x = ox
            obj.y = oy
            obj.vx = speed
            obj.vy = 0.0
            obj.size_x = 0.3
            obj.size_y = 0.3
            obj.is_confirmed = True
            obj.track_length = 10
            obj.predicted_path = [Point(x=ox + speed * i * 0.1, y=oy, z=0)
                                  for i in range(20)]

            arr = TrackedObjectArray()
            arr.header = Header(stamp=rospy.Time.now(), frame_id='map')
            arr.objects = [obj]
            pub_objs.publish(arr)

            if t < 1.0:
                rospy.loginfo(f"[test] t={t:.1f}s, 障碍物位置=({ox:.2f}, {oy:.2f}), "
                              f"预计 TTC≈{ox/abs(speed):.1f}s")

        t += 0.1
        rate.sleep()


if __name__ == '__main__':
    main()
