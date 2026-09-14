#!/usr/bin/python3
# -*- coding: UTF-8 -*-

"""
camera_driver.py —— USB 摄像头驱动节点

运行方式:
    rosrun ucar_camera camera_driver.py

功能：
    通过 OpenCV 读取 USB 摄像头（V4L2），以固定帧率将 RGB 图像发布到 ROS 话题。

ROS 接口：
    发布: <cam_topic_name> (sensor_msgs/Image)  —— 摄像头 RGB 图像（默认 /ucar_camera/image_raw）

rosparam 配置项：
    ~image_width   (int, 默认 1280)  —— 图像宽度
    ~image_height  (int, 默认 720)   —— 图像高度
    ~cam_topic_name (str, 默认 "/ucar_camera/image_raw") —— 发布话题名
    ~device_path   (str, 默认 "/dev/video0") —— 摄像头设备路径
    ~rate          (int, 默认 15)    —— 发布帧率 (Hz)
"""

import os
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import time
import threading


class UcarCamera:
    """
    USB 摄像头驱动类。

    初始化时读取 rosparam 配置，创建发布者并以固定帧率循环采集并发布图像。
    """

    def __init__(self):
        rospy.init_node("ucar_camera", anonymous=True)

        # ---- 读取 rosparam 配置 ----
        self.img_width = int(rospy.get_param('~image_width', default=1280))
        self.img_height = int(rospy.get_param('~image_height', default=720))

        self.camera_topic_name = rospy.get_param('~cam_topic_name', default="/ucar_camera/image_raw")
        self.cam_pub = rospy.Publisher(self.camera_topic_name, Image, queue_size=1)

        # ---- 构造 ROS Image 消息模板 ----
        self.image_temp = Image()
        self.image_temp.header.frame_id = 'opencv'
        self.image_temp.height = self.img_height
        self.image_temp.width = self.img_width
        self.image_temp.encoding = 'rgb8'
        self.image_temp.is_bigendian = True
        self.image_temp.step = self.img_width * 3     # 每行字节数 = 宽度 × 3 (RGB)

        # ---- 初始化摄像头 ----
        device_path = rospy.get_param('device_path', default="/dev/video0")
        self.cap = cv2.VideoCapture(device_path)
        self.cap.set(3, self.img_width)
        self.cap.set(4, self.img_height)
        # 使用 I420 (YUYV 的变体) 编码格式以兼容大多数 USB 摄像头
        codec = cv2.VideoWriter_fourcc('I', '4', '2', '0')
        self.cap.set(cv2.CAP_PROP_FOURCC, codec)

        # ---- 主循环：以固定帧率采集并发布 ----
        self.cam_pub_rate = int(rospy.get_param('~rate', default=15))
        ros_rate = rospy.Rate(self.cam_pub_rate)
        while not rospy.is_shutdown():
            ros_rate = rospy.Rate(self.cam_pub_rate)
            ret, frame_1 = self.cap.read()
            frame_1 = cv2.flip(frame_1, 1)                # 水平镜像翻转
            self.frame = cv2.cvtColor(frame_1, cv2.COLOR_BGR2RGB)  # BGR → RGB
            self.image_temp.header = Header(stamp=rospy.Time.now())
            self.image_temp.data = np.array(self.frame).tostring()
            self.cam_pub.publish(self.image_temp)
            ros_rate.sleep()


if __name__ == '__main__':
    ucar_camera = UcarCamera()
