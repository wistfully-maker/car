#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达避障测试 — 验证: 1.雷达检测前方挡板 2.底盘横向平移机动

用法:
  rosrun car_server test_lidar_avoid.py
  手动把挡板放车前 <0.4m, 观察是否触发避障

流程:
  持续检测雷达前方±20°内 <0.4m 的点数
  >10点连续3帧 → 触发避障机动
  机动: 右移0.5m → 前进0.58m → 左移0.45m
"""

import rospy, math, numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from threading import Lock

# 雷达检测参数(对标国赛)
AVOIDANCE_ANGLE_DEG = 40.0      # 前方±20度
AVOIDANCE_DISTANCE_M = 0.4      # 触发距离
AVOIDANCE_POINT_THRESHOLD = 10  # 点数阈值
CONSECUTIVE_FRAMES = 3          # 连续确认帧数

# 机动参数(对标国赛)
STRAFE_OUT_DIST = 0.5   # 右移距离
FORWARD_DIST = 0.58     # 前进距离
STRAFE_IN_DIST = 0.45   # 左移距离
STRAFE_SPEED = 0.15     # 平移速度
FORWARD_SPEED = 0.15    # 前进速度


class LidarAvoidTester:
    def __init__(self):
        rospy.init_node('test_lidar_avoid', anonymous=True)

        self.lock = Lock()
        self.latest_scan = None
        self.latest_pose = None
        self.obstacle_detected = False
        self.consecutive = 0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)

        # 状态
        self.state = "MONITOR"       # MONITOR / MANEUVER
        self.maneuver_step = 0
        self.maneuver_start_pose = None

        rospy.loginfo("✅ 雷达避障测试启动, 把挡板放车前<0.4m触发")
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.control_loop)

    def scan_cb(self, msg):
        """雷达回调: 检测前方±20°内<0.4m的点数"""
        with self.lock:
            self.latest_scan = msg

    def odom_cb(self, msg):
        with self.lock:
            self.latest_pose = msg.pose.pose

    def check_obstacle(self, scan):
        """检测前方挡板, 返回点数"""
        if scan is None:
            return 0
        # 计算0度(正前方)索引
        center_idx = int((0.0 - scan.angle_min) / scan.angle_increment)
        angle_rad = math.radians(AVOIDANCE_ANGLE_DEG)
        idx_offset = int(angle_rad / scan.angle_increment)
        start = max(0, center_idx - idx_offset)
        end = min(len(scan.ranges), center_idx + idx_offset)

        count = 0
        for i in range(start, end):
            d = scan.ranges[i]
            if 0 < d < AVOIDANCE_DISTANCE_M:
                count += 1
        return count

    def control_loop(self, event):
        with self.lock:
            scan = self.latest_scan
            pose = self.latest_pose

        if self.state == "MONITOR":
            count = self.check_obstacle(scan)
            if count > AVOIDANCE_POINT_THRESHOLD:
                self.consecutive += 1
                rospy.loginfo(f"⚠️ 前方挡板! 点数={count} [{self.consecutive}/{CONSECUTIVE_FRAMES}]")
                if self.consecutive >= CONSECUTIVE_FRAMES:
                    rospy.loginfo("🔀 触发避障机动!")
                    self.state = "MANEUVER"
                    self.maneuver_step = 0
                    self.maneuver_start_pose = pose
            else:
                self.consecutive = 0
                if count > 0:
                    rospy.loginfo_throttle(1.0, f"前方点数={count} (未达阈值{AVOIDANCE_POINT_THRESHOLD})")

        elif self.state == "MANEUVER":
            if pose is None:
                rospy.logwarn_throttle(1.0, "等待里程计...")
                return
            self.execute_maneuver(pose)

    def execute_maneuver(self, pose):
        """三步机动: 右移→前进→左移"""
        if self.maneuver_start_pose is None:
            self.maneuver_start_pose = pose

        start = self.maneuver_start_pose.position
        cur = pose.position
        dist = math.hypot(cur.x - start.x, cur.y - start.y)

        t = Twist()

        if self.maneuver_step == 0:  # 右移
            if dist < STRAFE_OUT_DIST:
                t.linear.y = -STRAFE_SPEED
                rospy.loginfo_throttle(0.5, f"步骤0: 右移 {dist:.2f}/{STRAFE_OUT_DIST}m")
            else:
                rospy.loginfo("✅ 右移完成")
                self.maneuver_step = 1
                self.maneuver_start_pose = pose

        elif self.maneuver_step == 1:  # 前进
            if dist < FORWARD_DIST:
                t.linear.x = FORWARD_SPEED
                rospy.loginfo_throttle(0.5, f"步骤1: 前进 {dist:.2f}/{FORWARD_DIST}m")
            else:
                rospy.loginfo("✅ 前进完成")
                self.maneuver_step = 2
                self.maneuver_start_pose = pose

        elif self.maneuver_step == 2:  # 左移
            if dist < STRAFE_IN_DIST:
                t.linear.y = STRAFE_SPEED
                rospy.loginfo_throttle(0.5, f"步骤2: 左移 {dist:.2f}/{STRAFE_IN_DIST}m")
            else:
                rospy.loginfo("✅ 左移完成! 避障机动结束")
                self.maneuver_step = 3
                self.state = "DONE"

        if self.state == "DONE":
            self.cmd_pub.publish(Twist())  # 停
        else:
            self.cmd_pub.publish(t)


if __name__ == '__main__':
    try:
        tester = LidarAvoidTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
