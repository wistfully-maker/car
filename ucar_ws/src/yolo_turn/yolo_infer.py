#!/usr/bin/env python3
"""
YOLO 交通标志检测 — ROS摄像头版
每3秒输出置信度最高的方向结果
"""

import os, sys, time, cv2
import numpy as np
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL  = "best.pt"
CONF   = 0.5
WINDOW = 1.5     # 统计窗口（秒）

NAMES = {0: "red_light", 1: "straight", 2: "left_turn", 3: "right_turn"}

bridge = CvBridge()
latest_frame = None
best = None
t0 = time.time()


def callback(msg):
    global latest_frame
    try:
        latest_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        pass


def main():
    global best, t0

    rospy.init_node("yolo_infer", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback, queue_size=1)
    rospy.sleep(1)  # 等第一帧

    model = YOLO(MODEL)
    print(f"[INFO] 模型: {MODEL}, conf={CONF}, 窗口={WINDOW}s")
    print(f"[INFO] 等待摄像头话题 /usb_cam/image_raw ...")

    rate = rospy.Rate(10)  # 10fps推理

    try:
        while not rospy.is_shutdown():
            frame = latest_frame
            if frame is None:
                rate.sleep()
                continue

            results = model(frame, conf=CONF, verbose=False)
            for r in results:
                if r.boxes is not None and len(r.boxes) > 0:
                    b = max(r.boxes, key=lambda b: float(b.conf[0]))
                    cls_id = int(b.cls[0])
                    conf = float(b.conf[0])
                    name = NAMES.get(cls_id, "unknown")
                    if best is None or conf > best[1]:
                        best = (name, conf)

            elapsed = time.time() - t0
            if elapsed >= WINDOW:
                if best is not None:
                    result_str = f"{best[0]}"
                    print(f">>> {result_str} ({best[1]:.2f})")
                    # 写入文件供auto_drive读取
                    with open("/tmp/yolo_result.txt", "w") as f:
                        f.write(result_str)
                else:
                    print(f">>> 未检出")
                best = None
                t0 = time.time()

            rate.sleep()

    except KeyboardInterrupt:
        if best is not None:
            print(f">>> {best[0]} ({best[1]:.2f})")
    finally:
        print("[INFO] 退出")


if __name__ == "__main__":
    main()
