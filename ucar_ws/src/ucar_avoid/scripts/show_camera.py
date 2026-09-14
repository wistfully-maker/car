#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer', anonymous=True)
        self.bridge = CvBridge()
        # 订阅摄像头话题
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.loginfo("Camera viewer started. Press 'q' in the window to quit.")

    def image_callback(self, msg):
        try:
            # 将ROS图像转换为OpenCV图像（BGR格式）
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Conversion error: %s", e)
            return

        # 显示图像
        cv2.imshow("Camera View", cv_image)
        # 等待按键，1ms 超时以保持响应
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User quit")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
