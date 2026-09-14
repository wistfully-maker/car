#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上坡直行下坡 — 根据 IMU pitch 角调整直行速度

上坡(pitch明显) → 加大速度
平路 → 正常速度
下坡(pitch反号) → 减速

用法: rosrun car_server ramp_straight.py
先跑一次看 pitch 打印值, 确认上坡/下坡的符号, 再调阈值
"""

import rospy, math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# 参数
FLAT_SPEED = 0.2    # 平路速度
UP_SPEED = 0.4      # 上坡速度(加大动力)
DOWN_SPEED = 0.03   # 下坡速度(慢速防冲)
PITCH_THRESHOLD = 8.0  # 坡度阈值(度), 超过算坡

class RampStraight:
    def __init__(self):
        rospy.init_node('ramp_straight', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.pitch = 0.0
        self.pitch_ready = False
        rospy.loginfo("✅ 上坡直行下坡启动")
        # 30Hz控制循环
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.control_loop)

    def imu_callback(self, msg):
        """四元数转 pitch 角"""
        q = msg.orientation
        # pitch = asin(2*(w*y - z*x))  (绕y轴)
        sinp = 2.0 * (q.w*q.y - q.z*q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.degrees(math.asin(sinp))
        self.pitch_ready = True

    def control_loop(self, event):
        if not self.pitch_ready:
            return
        t = Twist()
        if self.pitch < -PITCH_THRESHOLD:
            # 上坡(你的车 pitch 负)
            t.linear.x = UP_SPEED
            rospy.loginfo_throttle(1.0, f"⬆️ 上坡 pitch={self.pitch:.1f}° 速度={UP_SPEED}")
        elif self.pitch > PITCH_THRESHOLD:
            # 下坡(你的车 pitch 正)
            t.linear.x = DOWN_SPEED
            rospy.loginfo_throttle(1.0, f"⬇️ 下坡 pitch={self.pitch:.1f}° 速度={DOWN_SPEED}")
        else:
            # 平路
            t.linear.x = FLAT_SPEED
            rospy.loginfo_throttle(1.0, f"➡️ 平路 pitch={self.pitch:.1f}° 速度={FLAT_SPEED}")
        t.angular.z = 0.0
        self.cmd_pub.publish(t)


if __name__ == '__main__':
    try:
        node = RampStraight()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
