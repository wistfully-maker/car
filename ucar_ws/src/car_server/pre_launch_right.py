#!/usr/bin/env python3
"""
前置动作: 直行1.9s(0.2m/s) → 顺时针转70° → 直行1s → 退出
"""
import rospy, math, time
from geometry_msgs.msg import Twist

STRAIGHT_TIME = 1.9      # 直行1.9秒
STRAIGHT_SPEED = 0.2     # 直行速度m/s
TURN_ANGLE = 70          # 顺时针转70度
TURN_SPEED = 1.0         # 角速度rad/s

rospy.init_node("pre_launch_right", anonymous=True)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rate = rospy.Rate(20)

# 等publisher连上
rospy.sleep(0.5)

# === 阶段1: 直行 ===
rospy.loginfo(f"🚀 直行{STRAIGHT_TIME}s, {STRAIGHT_SPEED}m/s")
t0 = rospy.Time.now()
while (rospy.Time.now()-t0).to_sec() < STRAIGHT_TIME:
    t = Twist()
    t.linear.x = STRAIGHT_SPEED
    pub.publish(t)
    rate.sleep()

# === 阶段2: 原地顺时针转45° ===
turn_dur = math.radians(TURN_ANGLE) / TURN_SPEED
rospy.loginfo(f"↩ 顺时针转{TURN_ANGLE}度 (需{turn_dur:.2f}s)")
t0 = rospy.Time.now()
while (rospy.Time.now()-t0).to_sec() < turn_dur:
    t = Twist()
    t.angular.z = -TURN_SPEED
    pub.publish(t)
    rate.sleep()

# === 阶段3: 转完直行1s ===
rospy.loginfo("🚀 转完直行1s")
t0 = rospy.Time.now()
while (rospy.Time.now()-t0).to_sec() < 1.0:
    t = Twist()
    t.linear.x = STRAIGHT_SPEED
    pub.publish(t)
    rate.sleep()

# === 停止 ===
t = Twist()
pub.publish(t)
rospy.loginfo("✅ 前置动作完成, 退出")
