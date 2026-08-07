#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Point

sys.path.append('/home/ucar/ucar_ws/src/yolo')
sys.path.append('/home/ucar/ucar_ws/src/ocr')

try:
    from yolo_infer import YoloDetector
    from infer import RapidOcrInfer
except ImportError as e:
    rospy.logerr("Failed to import YOLO or OCR module: %s", e)
    sys.exit(1)


class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node', anonymous=True)

        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.85)
        self.detection_enabled = False

        rospy.loginfo("Initializing YOLO detector...")
        self.yolo = YoloDetector()
        rospy.loginfo("Initializing OCR recognizer...")
        self.ocr = RapidOcrInfer()

        self.bridge = CvBridge()

        # 订阅
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        self.enable_sub = rospy.Subscriber('/vision/enable', Bool, self.enable_callback, queue_size=1)

        # 发布话题
        self.status_pub = rospy.Publisher('/vision/status', Bool, queue_size=1)
        self.text_pub = rospy.Publisher('/vision/detected', String, queue_size=1)
        self.conf_pub = rospy.Publisher('/vision/confidence', Float32, queue_size=1)
        self.pos_pub = rospy.Publisher('/vision/position', Point, queue_size=1)
        self.area_pub = rospy.Publisher('/vision/area', Float32, queue_size=1)

        # 峰值检测相关
        self.peak_conf = 0.0
        self.peak_detected = False
        self.peak_hold_frames = 1
        self.consecutive_decrease = 0
        self.has_detected_once = False

        self.ocr_in_progress = False

        rospy.loginfo("========================================")
        rospy.loginfo("  Vision Node Started (Peak Detection Mode)")
        rospy.loginfo("  Confidence Threshold: %.2f", self.confidence_threshold)
        rospy.loginfo("  Peak hold frames: %d", self.peak_hold_frames)
        rospy.loginfo("========================================")

    def enable_callback(self, msg):
        self.detection_enabled = msg.data
        if self.detection_enabled:
            rospy.loginfo("🔍 Vision node AWAKENED, starting detection...")
            self.ocr_in_progress = False
            self.peak_conf = 0.0
            self.peak_detected = False
            self.consecutive_decrease = 0
            self.has_detected_once = False
        else:
            rospy.loginfo("💤 Vision node SLEEPING")

    def image_callback(self, msg):
        if not self.detection_enabled:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        try:
            detections = self.yolo.predict(cv_image)
        except Exception as e:
            rospy.logerr("YOLO detection error: %s", e)
            return

        best = None
        best_conf = 0.0
        for det in detections:
            if len(det) >= 5:
                x1, y1, x2, y2, conf = det[:5]
                if conf > best_conf and conf >= self.confidence_threshold:
                    best_conf = conf
                    best = (None, [x1, y1, x2, y2], conf)

        if self.ocr_in_progress:
            return

        if best is not None:
            cls, bbox, conf = best
            x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            sign_area = (x2 - x1) * (y2 - y1)

            self.area_pub.publish(Float32(data=float(sign_area)))

            # ★★★ 峰值检测（静默模式） ★★★
            if conf > self.peak_conf:
                self.peak_conf = conf
                self.consecutive_decrease = 0
                self.has_detected_once = True
            else:
                self.consecutive_decrease += 1

                # 峰值确认
                if (self.consecutive_decrease >= self.peak_hold_frames and 
                    self.peak_conf >= self.confidence_threshold and 
                    not self.peak_detected and
                    self.has_detected_once):
                    
                    rospy.loginfo("🎯 Peak detected! conf=%.3f → STOP ROTATION!", self.peak_conf)
                    self.peak_detected = True
                    self.status_pub.publish(Bool(data=True))

            # ★★★ 如果检测到标牌，发布停车信号（用于C++停车） ★★★
            # 但这里我们只在峰值时停车，所以不在这里发布 status=true

            # 发布置信度
            self.conf_pub.publish(Float32(data=conf))

            # ★★★ OCR 识别（只在检测到标牌时执行） ★★★
            self.ocr_in_progress = True
            self.text_pub.publish(String(data=""))

            # 裁剪 ROI 进行 OCR
            h, w = cv_image.shape[:2]
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(x1 + 1, min(x2, w))
            y2 = max(y1 + 1, min(y2, h))

            if x2 > x1 and y2 > y1:
                roi = cv_image[y1:y2, x1:x2]
                try:
                    text, ocr_conf = self.ocr.predict(roi)
                    rospy.loginfo("📝 OCR: %s (%.3f)", text, ocr_conf)
                except Exception as e:
                    rospy.logerr("OCR error: %s", e)
                    text = ""
                    self.ocr_in_progress = False
                    return

                if text and str(text).strip() != "":
                    self.text_pub.publish(String(data=str(text)))
                    self.conf_pub.publish(Float32(data=conf))
                    rospy.loginfo("📤 Final: %s", text)
                else:
                    rospy.logwarn("OCR result empty")
                    self.text_pub.publish(String(data=""))
            else:
                rospy.logwarn("Invalid bbox")
                self.text_pub.publish(String(data=""))

            self.ocr_in_progress = False

        else:
            # 没有检测到标牌
            if not self.peak_detected:
                self.peak_conf = 0.0
                self.consecutive_decrease = 0
            self.status_pub.publish(Bool(data=False))


def main():
    try:
        node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Vision node crashed: %s", e)


if __name__ == '__main__':
    main()
