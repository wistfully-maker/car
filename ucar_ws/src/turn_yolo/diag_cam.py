#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, logging
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite
# 关键: import cv2/numpy/rknnlite 之后恢复被覆盖的日志级别名
logging._nameToLevel.update({"CRITICAL": 50, "FATAL": 50, "ERROR": 40, "WARN": 30, "WARNING": 30, "INFO": 20, "DEBUG": 10, "NOTSET": 0})

MODEL = "/home/ucar/ucar_ws/src/turn_yolo/turn_yolo_480.rknn"
NAMES = {0: "straight", 1: "left", 2: "right", 3: "red"}
rknn = RKNNLite(); rknn.load_rknn(MODEL); rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
bridge = CvBridge(); latest = [None]
def cb(m):
    try: latest[0] = bridge.imgmsg_to_cv2(m, "bgr8")
    except Exception: pass
rospy.init_node("diag", anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
rospy.sleep(2.0)
f = latest[0]
if f is None:
    print("NO FRAME (相机没在发图?)"); sys.exit(1)
print("frame shape:", f.shape, f.dtype)
cv2.imwrite("/tmp/diag_frame.jpg", f)

def letterbox(im, new=(480,480), color=(114,114,114)):
    sh = im.shape[:2]; r = min(new[0]/sh[0], new[1]/sh[1])
    nu = int(round(sh[1]*r)), int(round(sh[0]*r))
    dw, dh = (new[1]-nu[0])/2, (new[0]-nu[1])/2
    if sh[::-1] != nu: im = cv2.resize(im, nu, interpolation=cv2.INTER_LINEAR)
    return cv2.copyMakeBorder(im, int(round(dh-0.1)), int(round(dh+0.1)), int(round(dw-0.1)), int(round(dw+0.1)), cv2.BORDER_CONSTANT, value=color)

x = cv2.cvtColor(letterbox(f), cv2.COLOR_BGR2RGB)[None]
out = rknn.inference(inputs=[x], data_type="uint8", data_format="nhwc")[0]
preds = out[0].T
print("out shape:", out.shape)
scores = 1.0/(1.0+np.exp(-preds[:, 4:]))
print("per-class max sigmoid:")
for i, nm in NAMES.items():
    print("  %-8s max=%.3f" % (nm, scores[:, i].max()))
print("raw class logits (mean/min/max):")
for i, nm in NAMES.items():
    cc = preds[:, 4+i]
    print("  %-8s mean=%7.2f min=%7.2f max=%7.2f" % (nm, cc.mean(), cc.min(), cc.max()))
rknn.release()
