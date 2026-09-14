#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("line_control_server")
    # 创建/cmd_vel发布器
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(20)
    cmd = Twist()

    while not rospy.is_shutdown():
        # =====================
        # 在这里修改速度
        cmd.linear.x  = 0.15   # 前进速度
        cmd.angular.z = 0.2    # 转向角速度
        # =====================
        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

