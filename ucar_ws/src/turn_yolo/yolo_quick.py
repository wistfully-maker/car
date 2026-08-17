#!/usr/bin/env python3
"""
YOLOv8 红绿灯方向检测 —— 快速定帧版（RKNN/NPU 加速）
====================================
采集一小段时间(默认 3 秒)取最高置信度的方向，一次性输出结果。
加载 turn_yolo_480_cls.rknn，在 RK3588 NPU 上推理。

用法:
  python yolo_quick.py                # 订阅 ROS 摄像头
  python yolo_quick.py --image a.jpg  # 单张图片(测试)
  python yolo_quick.py --seconds 3    # 采集时间(秒)
"""

import os
import sys
import time
import argparse

import numpy as np
import cv2
from rknnlite.api import RKNNLite

BASE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(BASE, "turn_yolo_480_cls.rknn")
CONF = 0.5
IMGSZ = 480
RESULT_FILE = "/tmp/yolo_result.txt"

NAMES = {0: "straight", 1: "left", 2: "right", 3: "red"}


class Detector:
    """RKNN 检测器：封装模型加载 + 单帧推理。"""

    def __init__(self, model_path):
        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"load_rknn 失败: {ret}")
        # RK3588 三核 NPU 全开
        ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            raise RuntimeError(f"init_runtime 失败: {ret}")

    def _letterbox(self, im, new=(IMGSZ, IMGSZ), color=(114, 114, 114)):
        shape = im.shape[:2]
        r = min(new[0] / shape[0], new[1] / shape[1])
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new[1] - new_unpad[0], new[0] - new_unpad[1]
        dw, dh = dw / 2, dh / 2
        if shape[::-1] != new_unpad:
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=color)
        return im, r, (dw, dh)

    def best_detection(self, frame):
        """返回置信度最高的 (short_name, conf)；无检出返回 (None, 0)。"""
        img, ratio, (dw, dh) = self._letterbox(frame)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # [640,640,3] uint8 RGB
        img = img[None]                              # [1,640,640,3] 加 batch 维(rknn 需 4 维)
        out = self.rknn.inference(inputs=[img],
                                  data_type='uint8', data_format='nhwc')[0]   # [1, 8, 8400]

        preds = out[0].T                              # [8400, 8]
        scores = 1.0 / (1.0 + np.exp(-preds))  # raw logits -> sigmoid
        conf = scores.max(1)
        cls = scores.argmax(1)

        keep = conf >= CONF
        if not keep.any():
            return None, 0.0
        conf, cls = conf[keep], cls[keep]
        i = int(conf.argmax())
        return NAMES.get(int(cls[i]), "unknown"), float(conf[i])

    def release(self):
        self.rknn.release()


def write_result(result):
    try:
        with open(RESULT_FILE, "w") as f:
            f.write(result)
    except OSError:
        pass


def run_image(det, path):
    img = cv2.imread(path)
    if img is None:
        print(f"[ERROR] 读图失败: {path}")
        return
    name, conf = det.best_detection(img)
    result = name if name else "straight"
    print(f">>> {result} ({conf:.2f})" if name else f">>> {result} (未检出)")
    write_result(result)


def run_ros(det, seconds):
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    bridge = CvBridge()
    latest = [None]

    def cb(msg):
        try:
            latest[0] = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    import logging
    logging._nameToLevel.update({"CRITICAL": 50, "FATAL": 50, "ERROR": 40, "WARN": 30, "WARNING": 30, "INFO": 20, "DEBUG": 10, "NOTSET": 0})
    rospy.init_node("turn_yolo_quick", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
    rospy.sleep(1.5)  # 等第一帧

    best_name, best_conf = None, 0.0
    t0 = time.time()
    while time.time() - t0 < seconds and not rospy.is_shutdown():
        frame = latest[0]
        if frame is not None:
            name, conf = det.best_detection(frame)
            if name and conf > best_conf:
                best_name, best_conf = name, conf
        rospy.sleep(0.1)

    result = best_name if best_name else "straight"
    print(f">>> {result} ({best_conf:.2f})" if best_name else f">>> {result} (未检出)")
    write_result(result)


def main():
    global CONF
    p = argparse.ArgumentParser(description="红绿灯快速定帧检测 (RKNN)")
    p.add_argument("--image", default=None, help="单张图片路径")
    p.add_argument("--seconds", type=float, default=3.0, help="采集时间(秒)")
    p.add_argument("--conf", type=float, default=CONF, help="置信度阈值")
    args = p.parse_args()
    CONF = args.conf

    if not os.path.exists(MODEL):
        print(f"[ERROR] 模型不存在: {MODEL}")
        sys.exit(1)

    det = Detector(MODEL)
    try:
        if args.image:
            run_image(det, args.image)
        else:
            run_ros(det, args.seconds)
    finally:
        det.release()


if __name__ == "__main__":
    main()
