#!/usr/bin/env python3
"""
YOLOv8 红绿灯方向检测 —— 小车部署版
====================================
加载 best.pt，实时输出 red / left / right / straight。

类别映射（与训练 data.yaml 一致）:
  0 -> straight   (绿灯直行)
  1 -> left       (绿灯左转)
  2 -> right      (绿灯右转)
  3 -> red        (红灯)

输出:
  - 终端打印  >>> red / left / right / straight
  - 写入 /tmp/yolo_result.txt 供 auto_drive 读取

用法:
  python yolo_infer.py              # 订阅 ROS 摄像头 /usb_cam/image_raw
  python yolo_infer.py --camera 0   # 直读摄像头(无 ROS, 测试用)
  python yolo_infer.py --image a.jpg  # 单张图片(测试用)
"""

import os
import sys
import time
import argparse

import cv2
from ultralytics import YOLO

BASE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(BASE, "best.pt")
CONF = 0.5          # 置信度阈值
IMGSZ = 640         # ★ 与训练一致
WINDOW = 1.5        # 投票窗口(秒)，窗口内取最高置信度，抗单帧抖动
RESULT_FILE = "/tmp/yolo_result.txt"

# 训练类别 id -> 简洁结果（与检测节点 /start_follow 发布值一致）
NAMES = {0: "straight", 1: "left", 2: "right", 3: "red"}


def best_detection(model, frame):
    """对一帧做推理，返回置信度最高的 (short_name, conf)；无检出返回 (None, 0)。"""
    results = model(frame, conf=CONF, imgsz=IMGSZ, verbose=False)
    for r in results:
        boxes = r.boxes
        if boxes is None or len(boxes) == 0:
            continue
        best = max(boxes, key=lambda b: float(b.conf[0]))
        cls_id = int(best.cls[0])
        conf = float(best.conf[0])
        return NAMES.get(cls_id, "unknown"), conf
    return None, 0.0


def write_result(result):
    """把结果写到 /tmp/yolo_result.txt（供 auto_drive 读取），兼容无 /tmp 的测试环境。"""
    try:
        with open(RESULT_FILE, "w") as f:
            f.write(result)
    except OSError:
        pass


def run_image(model, path):
    img = cv2.imread(path)
    if img is None:
        print(f"[ERROR] 读图失败: {path}")
        return
    name, conf = best_detection(model, img)
    print(f">>> {name} ({conf:.2f})" if name else ">>> 未检出")
    if name:
        write_result(name)


def run_camera(model, index):
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print(f"[ERROR] 打不开摄像头 {index}")
        return
    print(f"[INFO] 直读摄像头 {index}，Ctrl+C 退出")
    best = (None, 0.0)
    t0 = time.time()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            name, conf = best_detection(model, frame)
            if name and conf > best[1]:
                best = (name, conf)
            if time.time() - t0 >= WINDOW:
                if best[0]:
                    print(f">>> {best[0]} ({best[1]:.2f})")
                    write_result(best[0])
                else:
                    print(">>> 未检出")
                best = (None, 0.0)
                t0 = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


def run_ros(model):
    """订阅 ROS 摄像头 /usb_cam/image_raw。"""
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

    rospy.init_node("turn_yolo_infer", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
    print("[INFO] 订阅 /usb_cam/image_raw ...")
    rospy.sleep(1.0)

    best = (None, 0.0)
    t0 = time.time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        frame = latest[0]
        if frame is not None:
            name, conf = best_detection(model, frame)
            if name and conf > best[1]:
                best = (name, conf)
        if time.time() - t0 >= WINDOW:
            if best[0]:
                print(f">>> {best[0]} ({best[1]:.2f})")
                write_result(best[0])
            else:
                print(">>> 未检出")
            best = (None, 0.0)
            t0 = time.time()
        rate.sleep()


def main():
    p = argparse.ArgumentParser(description="红绿灯方向检测 (best.pt)")
    p.add_argument("--camera", type=int, default=None, help="直读摄像头索引(无 ROS)")
    p.add_argument("--image", default=None, help="单张图片路径")
    p.add_argument("--conf", type=float, default=CONF, help="置信度阈值")
    args = p.parse_args()

    if not os.path.exists(MODEL):
        print(f"[ERROR] 模型不存在: {MODEL}")
        sys.exit(1)

    model = YOLO(MODEL)
    print(f"[INFO] 模型: {MODEL}")
    print(f"[INFO] 类别: 0=straight 1=left 2=right 3=red")
    print(f"[INFO] conf={args.conf}, imgsz={IMGSZ}, 窗口={WINDOW}s")

    if args.image:
        run_image(model, args.image)
    elif args.camera is not None:
        run_camera(model, args.camera)
    else:
        run_ros(model)


if __name__ == "__main__":
    main()
