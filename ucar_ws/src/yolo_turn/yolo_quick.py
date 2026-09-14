#!/usr/bin/env python3
"""YOLO单帧检测 — 取一帧图跑一次推理, 即时输出结果"""
import cv2, sys, os, time
import numpy as np
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL = os.path.join(os.path.dirname(__file__), "best.pt")
CONF  = 0.5
NAMES = {0: "red_light", 1: "straight", 2: "left_turn", 3: "right_turn"}

bridge = CvBridge()
frame = None

def callback(msg):
    global frame
    try: frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except: pass

rospy.init_node("yolo_quick", anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, callback, queue_size=1)
rospy.sleep(1.5)  # 等第一帧

model = YOLO(MODEL)
best_name = None
best_conf = 0
t0 = time.time()

# 采集3秒内置信度最高的结果
while time.time() - t0 < 3.0 and not rospy.is_shutdown():
    if frame is not None:
        results = model(frame, conf=CONF, verbose=False)
        for r in results:
            if r.boxes is not None and len(r.boxes) > 0:
                b = max(r.boxes, key=lambda b: float(b.conf[0]))
                cls_id = int(b.cls[0]); conf = float(b.conf[0])
                name = NAMES.get(cls_id, "unknown")
                if conf > best_conf:
                    best_conf = conf; best_name = name
    rospy.sleep(0.1)

result = best_name if best_name else "straight"
print(f">>> {result}")
with open("/tmp/yolo_result.txt", "w") as f:
    f.write(result)
print(f"[INFO] 结果已写入 /tmp/yolo_result.txt")
