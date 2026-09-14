#!/usr/bin/env python3
"""
YOLO常驻节点 — 加入launch文件, 持续检测并写/tmp/yolo_result.txt
"""
import cv2, time, os
import numpy as np
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL = "/home/ucar/ucar_ws/src/yolo_turn/best.pt"
CONF  = 0.5
INTERVAL = 1.5  # 每1.5秒更新一次结果
NAMES = {0: "red_light", 1: "straight", 2: "left_turn", 3: "right_turn"}

bridge = CvBridge(); frame = None
def cb(msg):
    global frame
    try: frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except: pass

rospy.init_node("yolo_server")
rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
rospy.sleep(1.5)
model = YOLO(MODEL)

result_file = "/tmp/yolo_result.txt"
rospy.loginfo("✅ YOLO服务器就绪, 每1.5s更新 /tmp/yolo_result.txt")

while not rospy.is_shutdown():
    best_name, best_conf = None, 0
    t0 = time.time()
    # 采样1.5秒取最佳结果
    while time.time()-t0 < INTERVAL and not rospy.is_shutdown():
        if frame is not None:
            results = model(frame, conf=CONF, verbose=False)
            for r in results:
                if r.boxes is not None and len(r.boxes) > 0:
                    b = max(r.boxes, key=lambda b: float(b.conf[0]))
                    cls_id = int(b.cls[0]); cf = float(b.conf[0])
                    name = NAMES.get(cls_id, "unknown")
                    if cf > best_conf: best_conf = cf; best_name = name
        rospy.sleep(0.05)
    # 写入结果
    if best_name:
        with open(result_file, "w") as f: f.write(best_name)
