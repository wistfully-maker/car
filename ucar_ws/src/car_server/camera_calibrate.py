#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头前瞻标定工具 — 调整摄像头角度使其与follow.py扫描区域匹配

显示三个视图(左右拼接):
  左: 原图 + 扫描区域标记线
  中: Canny预处理后的二值图(160×120放大到480)
  右: 二值图ROI区域(仅显示扫描行范围)

用法:
  rosrun car_server camera_calibrate.py
  rosrun car_server camera_calibrate.py _deal_low:=76 _deal_high:=105

查看: rqt_image_view /car_server/debug_image
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraCalibrate:
    def __init__(self):
        rospy.init_node('camera_calibrate', anonymous=True)
        self.bridge = CvBridge()
        self.dbg_pub = rospy.Publisher('/car_server/debug_image', Image, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)

        # === 可调参数(通过 _param:=value 修改) ===
        self.img_w = rospy.get_param('~img_w', 160)
        self.img_h = rospy.get_param('~img_h', 120)
        self.deal_low  = rospy.get_param('~deal_low', 76)    # 中点扫描下界
        self.deal_high = rospy.get_param('~deal_high', 105)  # 中点扫描上界
        self.eight_low  = rospy.get_param('~eight_low', 85)  # 拐角检测下界
        self.eight_high = rospy.get_param('~eight_high', 118)# 拐角检测上界
        self.stop_row_start = rospy.get_param('~stop_start', 24)
        self.stop_row_end   = rospy.get_param('~stop_end', 119)
        self.direction = rospy.get_param('~direction', '')  # left/right/straight

        self.fc = 0
        rospy.loginfo("=" * 60)
        rospy.loginfo("📷 前瞻标定工具启动")
        rospy.loginfo(f"   中点扫描: deal_low={self.deal_low} deal_high={self.deal_high}")
        rospy.loginfo(f"   拐角检测: eight_low={self.eight_low} eight_high={self.eight_high}")
        rospy.loginfo(f"   停车检测: row {self.stop_row_start}~{self.stop_row_end}")
        rospy.loginfo(f"   图像尺寸: {self.img_w}×{self.img_h}")
        rospy.loginfo("   查看: rqt_image_view /car_server/debug_image")
        rospy.loginfo("=" * 60)

    # ================================================================
    # follow.py 原样管线
    # ================================================================
    def preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 50, 150)
        k = np.ones((3, 3), np.uint8)
        d1 = cv2.dilate(edge, k, iterations=2)
        e1 = cv2.erode(d1, k, iterations=1)
        d2 = cv2.dilate(e1, k, iterations=1)
        cl = cv2.morphologyEx(d2, cv2.MORPH_CLOSE, k)
        _, bi = cv2.threshold(cl, 1, 255, cv2.THRESH_BINARY)
        return bi

    def midpoint_error(self, mask):
        hw = self.img_w // 2; half = hw; mc = 0
        rng = self.deal_high - self.deal_low
        if rng <= 0: return 0
        for y in range(self.deal_high, self.deal_low, -1):
            lr = mask[y][max(0, half - hw):half]
            L = max(0, half - hw) if not np.any(lr == 255) else np.average(np.where(lr == 255))
            rr = mask[y][half:min(self.img_w, half + hw)]
            R = min(self.img_w, half + hw) if not np.any(rr == 255) else np.average(np.where(rr == 255)) + half
            half = int((L + R) // 2); mc += half
        return hw - (mc / rng)

    # ================================================================
    # 可视化
    # ================================================================
    def _draw_zone(self, img, y_low, y_high, color, label, scale):
        """在缩放后的图上画扫描区域"""
        y1 = int(y_low * scale)
        y2 = int(y_high * scale)
        cv2.line(img, (0, y1), (img.shape[1], y1), color, 1)
        cv2.line(img, (0, y2), (img.shape[1], y2), color, 1)
        # 半透明填充
        overlay = img.copy()
        cv2.rectangle(overlay, (0, y1), (img.shape[1], y2), color, -1)
        img[:] = cv2.addWeighted(img, 0.85, overlay, 0.15, 0)
        cv2.putText(img, label, (5, y1 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            self.fc += 1

            # === 管线处理 ===
            small = cv2.resize(frame, (self.img_w, self.img_h))
            mask = self.preprocess(small)
            err = self.midpoint_error(mask)
            rh, rw = mask.shape

            # === 构建显示画面(1280×480) ===
            # 左: 原图640×480 + 标记
            # 中: 二值图640×480
            display = np.zeros((480, 1280, 3), dtype=np.uint8)

            # --- 左: 原图 ---
            left_view = frame.copy()
            scale_y = 480.0 / self.img_h  # 120→480 缩放比
            self._draw_zone(left_view, self.eight_high, self.eight_low,
                           (255, 165, 0), "CORN", scale_y * 4)  # 橙
            self._draw_zone(left_view, self.deal_high, self.deal_low,
                           (0, 255, 0), "SCAN", scale_y * 4)    # 绿
            self._draw_zone(left_view, self.stop_row_start, self.stop_row_end,
                           (0, 0, 255), "STOP", scale_y * 4)    # 红
            # 中心线
            cv2.line(left_view, (320, 0), (320, 480), (255, 255, 0), 1)

            # 文字信息
            cv2.putText(left_view, f"F{self.fc} err={err:+.1f}", (5, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.putText(left_view, f"deal[{self.deal_low}:{self.deal_high}]", (5, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
            cv2.putText(left_view, f"eight[{self.eight_low}:{self.eight_high}]", (5, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 165, 0), 1)

            display[:, 0:640] = left_view

            # --- 中: 二值图放大 ---
            mask_big = cv2.resize(mask, (640, 480))  # 160×120 → 640×480
            mask_bgr = cv2.cvtColor(mask_big, cv2.COLOR_GRAY2BGR)

            # 画扫描区域线
            self._draw_zone(mask_bgr, self.eight_high, self.eight_low,
                           (255, 165, 0), "CORN", 4.0)  # 480/120=4
            self._draw_zone(mask_bgr, self.deal_high, self.deal_low,
                           (0, 255, 0), "SCAN", 4.0)
            # 画中点
            mid_x = int((80 - err) * 4)  # error→像素
            cv2.line(mask_bgr, (mid_x, mask_bgr.shape[0]),
                    (mid_x, max(0, mask_bgr.shape[0] - 30)), (0, 0, 255), 2)

            cv2.putText(mask_bgr, "BINARY+Canny", (5, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.putText(mask_bgr, f"R={rw} H={rh}", (5, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            display[:, 640:1280] = mask_bgr[:, :]

            # --- 右侧信息(覆盖在二值图右下角) ---
            cv2.putText(display, f"err={err:+.1f}", (650, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(display, "CORN:橙 SCAN:绿 STOP:红", (650, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

            # === 发布 ===
            dbg_msg = Image()
            dbg_msg.header.stamp = msg.header.stamp
            dbg_msg.height = 480; dbg_msg.width = 1280
            dbg_msg.encoding = "bgr8"; dbg_msg.is_bigendian = False
            dbg_msg.step = 1280 * 3
            dbg_msg.data = display.astype(np.uint8).tobytes()
            self.dbg_pub.publish(dbg_msg)

            if self.fc % 30 == 0:
                rospy.loginfo(f"F{self.fc} err={err:+.1f} "
                             f"deal[{self.deal_low}:{self.deal_high}] "
                             f"eight[{self.eight_low}:{self.eight_high}]")

        except Exception as e:
            import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")


if __name__ == '__main__':
    try:
        CameraCalibrate(); rospy.spin()
    except rospy.ROSInterruptException: pass
