#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""连续抓帧，dump 每帧四类的原始 logit 与置信度，用于定位 0.50 根因。"""
import sys, time, logging
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite
logging._nameToLevel.update({"CRITICAL": 50, "FATAL": 50, "ERROR": 40, "WARN": 30,
                             "WARNING": 30, "INFO": 20, "DEBUG": 10, "NOTSET": 0})

MODEL = "/home/ucar/ucar_ws/src/turn_yolo/turn_yolo_480_cls.rknn"
NAMES = {0: "straight", 1: "left", 2: "right", 3: "red"}

rknn = RKNNLite()
rknn.load_rknn(MODEL)
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)


def letterbox(im, new=(480, 480), color=(114, 114, 114)):
    sh = im.shape[:2]
    r = min(new[0] / sh[0], new[1] / sh[1])
    nu = int(round(sh[1] * r)), int(round(sh[0] * r))
    dw, dh = (new[1] - nu[0]) / 2, (new[0] - nu[1]) / 2
    if sh[::-1] != nu:
        im = cv2.resize(im, nu, interpolation=cv2.INTER_LINEAR)
    return cv2.copyMakeBorder(im, int(round(dh - 0.1)), int(round(dh + 0.1)),
                              int(round(dw - 0.1)), int(round(dw + 0.1)),
                              cv2.BORDER_CONSTANT, value=color)


bridge = CvBridge()
n = [0]


def cb(m):
    try:
        img = bridge.imgmsg_to_cv2(m, "bgr8")
    except Exception:
        return
    x = cv2.cvtColor(letterbox(img), cv2.COLOR_BGR2RGB)[None]
    out = rknn.inference(inputs=[x], data_type="uint8", data_format="nhwc")[0]
    preds = out[0].T                           # [N,4] 纯 class logits（已 clamp）
    logits = preds
    per = logits.max(0)                        # 每类最大 logit
    sig = 1.0 / (1.0 + np.exp(-logits))
    conf = sig.max(1)
    cls = sig.argmax(1)
    i = int(conf.argmax())
    # 打印: 帧号 | 检测类 置信度 | 四类最大logit(直 左 右 红)
    print("%3d | %-8s %.2f | logits=[%6.2f %6.2f %6.2f %6.2f]" %
          (n[0], NAMES[int(cls[i])], float(conf[i]), per[0], per[1], per[2], per[3]))
    n[0] += 1


rospy.init_node("diag_logits", anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
print("开始 dump logit，Ctrl+C 停", flush=True)
rospy.spin()
