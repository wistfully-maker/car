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

        # 发布
        self.status_pub = rospy.Publisher('/vision/status', Bool, queue_size=1)
        self.text_pub = rospy.Publisher('/vision/detected', String, queue_size=1)
        self.conf_pub = rospy.Publisher('/vision/confidence', Float32, queue_size=1)
        self.pos_pub = rospy.Publisher('/vision/position', Point, queue_size=1)
        self.area_pub = rospy.Publisher('/vision/area', Float32, queue_size=1)

        # ★★★ 状态控制
        self.ocr_in_progress = False
        self.ocr_done = False              # OCR 是否已完成（匹配成功或失败）
        self.ocr_matched = False           # OCR 是否匹配成功（目标车间）
        self.triggered_this_scan = False   # 是否已触发峰值停车

        # 置信度历史（用于峰值检测）
        self.conf_history = []

        rospy.loginfo("========================================")
        rospy.loginfo("  Vision Node Started (OCR First Mode)")
        rospy.loginfo("  Confidence Threshold: %.2f", self.confidence_threshold)
        rospy.loginfo("  Step 1: OCR on first detection")
        rospy.loginfo("  Step 2: Peak detection & stop")
        rospy.loginfo("========================================")

    def enable_callback(self, msg):
        self.detection_enabled = msg.data
        if self.detection_enabled:
            rospy.loginfo("🔍 Vision node AWAKENED, starting detection...")
            self.ocr_in_progress = False
            self.ocr_done = False
            self.ocr_matched = False
            self.triggered_this_scan = False
            self.conf_history.clear()
        else:
            rospy.loginfo("💤 Vision node SLEEPING")

    def run_ocr(self, cv_image, bbox):
        """执行 OCR 识别并判断是否匹配目标"""
        self.ocr_in_progress = True
        
        x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
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
                self.ocr_in_progress = False
                self.ocr_done = True
                return

            if text and str(text).strip() != "":
                self.text_pub.publish(String(data=str(text)))
                rospy.loginfo("📤 Final: %s", text)
                # ★★★ 检查是否匹配目标车间（包含两个目标）
                if "食品加工车间" in text or "日用品加工车间" in text:
                    self.ocr_matched = True
                    rospy.loginfo("✅ OCR matched target warehouse: %s", text)
                else:
                    self.ocr_matched = False
                    rospy.loginfo("❌ OCR not matched (expected: 食品加工车间 or 日用品加工车间)")
            else:
                rospy.logwarn("OCR result empty")
                self.text_pub.publish(String(data=""))
                self.ocr_matched = False
        else:
            rospy.logwarn("Invalid bbox")
            self.text_pub.publish(String(data=""))
            self.ocr_matched = False

        self.ocr_in_progress = False
        self.ocr_done = True

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

            # 发布标牌中心
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            self.pos_pub.publish(Point(x=center_x, y=center_y, z=0.0))
            self.area_pub.publish(Float32(data=float(sign_area)))

            # ★★★ 第一步：首次检测到标牌 → 立即执行 OCR ★★★
            if not self.ocr_done:
                rospy.loginfo("🎯 First detection! Running OCR immediately...")
                self.run_ocr(cv_image, bbox)
                # OCR 完成后继续执行后面的峰值检测

            # ★★★ 第二步：峰值检测（只有 OCR 匹配成功后，才允许触发停车） ★★★
            if self.ocr_matched and not self.triggered_this_scan:
                # 收集置信度历史
                self.conf_history.append(conf)
                if len(self.conf_history) > 3:
                    self.conf_history.pop(0)

                # 峰值检测：置信度下降 + 下降幅度 > 0.02
                if len(self.conf_history) >= 3:
                    c0 = self.conf_history[-3]
                    c1 = self.conf_history[-2]
                    c2 = self.conf_history[-1]
                    if c1 > c0 and c2 < c1 and c1 >= self.confidence_threshold and (c1 - c2) > 0.02:
                        self.triggered_this_scan = True
                        rospy.loginfo("🔽 Confidence dropped (%.3f -> %.3f), stopping rotation!", c1, c2)
                        rospy.loginfo("🎯 Peak detected! Publishing stop signal...")
                        self.status_pub.publish(Bool(data=True))

            # 发布置信度
            self.conf_pub.publish(Float32(data=conf))

        else:
            # 无检测，重置部分状态
            self.conf_history.clear()
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
