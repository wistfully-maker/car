#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""常驻实时检测：加载一次模型，持续检测相机画面，显示每帧推理耗时/FPS。"""
import sys, time, logging
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite
# 关键: import cv2/numpy/rknnlite 之后恢复被覆盖的日志级别名
logging._nameToLevel.update({"CRITICAL": 50, "FATAL": 50, "ERROR": 40, "WARN": 30,
                             "WARNING": 30, "INFO": 20, "DEBUG": 10, "NOTSET": 0})

MODEL = "/home/ucar/ucar_ws/src/turn_yolo/turn_yolo_480_cls.rknn"
NAMES = {0: "straight", 1: "left", 2: "right", 3: "red"}
CONF = 0.5
RESULT_FILE = "/tmp/yolo_result.txt"

print("加载模型...", flush=True)
rknn = RKNNLite()
rknn.load_rknn(MODEL)
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
print("模型加载完成，开始实时检测 (Ctrl+C 退出)", flush=True)


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


def detect(img):
    x = cv2.cvtColor(letterbox(img), cv2.COLOR_BGR2RGB)[None]
    out = rknn.inference(inputs=[x], data_type="uint8", data_format="nhwc")[0]
    preds = out[0].T                        # [4725, 4] 纯 class logits（已 clamp）
    scores = 1.0 / (1.0 + np.exp(-preds))   # sigmoid -> 连续置信度
    conf = scores.max(1)
    cls = scores.argmax(1)
    i = int(conf.argmax())
    if conf[i] >= CONF:
        return NAMES[int(cls[i])], float(conf[i])
    return None, float(conf[i])


bridge = CvBridge()
n_frames = [0]
t0 = [time.time()]
times = []


def cb(m):
    try:
        img = bridge.imgmsg_to_cv2(m, "bgr8")
    except Exception:
        return
    t = time.time()
    name, conf = detect(img)
    dt = (time.time() - t) * 1000
    times.append(dt)
    if name:
        with open(RESULT_FILE, "w") as f:
            f.write(name)
        print(">>> %-8s %.2f   推理 %5.1fms  %.1f FPS" % (name, conf, dt, 1000.0 / dt))
    else:
        print(">>> 未检出(%.2f)  推理 %5.1fms  %.1f FPS" % (conf, dt, 1000.0 / dt))


rospy.init_node("yolo_live", anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
rospy.spin()
rknn.release()
