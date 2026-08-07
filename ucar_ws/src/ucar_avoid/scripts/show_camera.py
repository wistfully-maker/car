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
        
        # 订阅摄像头话题（根据实际情况修改话题名）
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        
        # 启动 OpenCV 窗口线程
        cv2.startWindowThread()
        cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera View', 640, 480)
        
        rospy.loginfo("Camera viewer started. Press Ctrl+C to exit.")
    
    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow('Camera View', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn("Display error: %s", e)

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
