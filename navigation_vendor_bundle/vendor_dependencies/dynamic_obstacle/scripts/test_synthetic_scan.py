#!/usr/bin/env python3
"""测试脚本: 发布模拟 /scan_filtered, 包含一个来回移动的障碍物点。

用法:
  rosrun dynamic_obstacle test_synthetic_scan.py
  然后另开终端: rostopic echo /tracked_objects
"""

import math
import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('test_synthetic_scan')
    pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=1)
    rate = rospy.Rate(10)  # 10Hz

    scan = LaserScan()
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.radians(1.0)  # 1度分辨率
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.1
    scan.range_max = 10.0

    num_beams = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
    t = 0.0
    rospy.loginfo("[test] 开始发布模拟 /scan_filtered (移动障碍物 x=0~3m, y=-1~1m)")

    while not rospy.is_shutdown():
        # 障碍物做 8 字形运动: x = 1.5 + 1.0*sin(t), y = 0.8*sin(2t)
        ox = 1.5 + 1.0 * math.sin(t * 0.5)
        oy = 0.8 * math.sin(t * 0.5 * 2)

        # 背景: 远处墙壁 (2.5m 处一圈)
        ranges = [2.5] * num_beams

        # 在障碍物方向上插入近距离读数
        obs_angle = math.atan2(oy, ox)
        obs_dist = math.hypot(ox, oy)
        beam_idx = int((obs_angle - scan.angle_min) / scan.angle_increment)

        if 0 <= beam_idx < num_beams:
            ranges[beam_idx] = obs_dist
            # 附近几个光束也设相近值，构成一个小聚类
            for di in [-2, -1, 1, 2]:
                ni = beam_idx + di
                if 0 <= ni < num_beams:
                    offset_angle = scan.angle_min + ni * scan.angle_increment
                    ranges[ni] = obs_dist + 0.01 * abs(di)

        scan.header.stamp = rospy.Time.now()
        scan.ranges = ranges
        pub.publish(scan)

        t += 0.1
        rate.sleep()


if __name__ == '__main__':
    main()
