#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camer_height_check 完整 ROS版 — 复刻原文件全部功能

管线: HSV白线提取 → 滑动中点+中线 → 畸变矫正 → 霍夫横线检测 → 标定线

数据源: /usb_cam/image_raw
输出: /car_server/debug_image (二值化+中线 | 原图+标定线)

用法:
  rosrun car_server camer_height_ros.py
  电脑看: rqt_image_view /car_server/debug_image
"""

import rospy, cv2, numpy as np, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 参数(和 camer_height_check.py 完全一致)
DEAL_LOW = 320
DEAL_HIGH = 400
BIAODING_LINE = 200
BIAODING_LINE_2 = 440
STOP_LINE_LOW = 400
STOP_LINE_HIGH = 470

LOWER_WHITE = np.array([0, 21, 203])
UPPER_WHITE = np.array([180, 63, 255])

CAMERA_MATRIX = np.array([
    [417.02331176, 0, 317.6164318],
    [0, 417.36101837, 222.31700641],
    [0, 0, 1]
], dtype=np.float32)
DIST_COEFFS = np.array([-0.3183328, 0.09406683, 0.00304064, -0.00085934, 0], dtype=np.float32)


class CamerHeightRos:
    def __init__(self):
        rospy.init_node('camer_height_ros', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/car_server/debug_image', Image, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback, queue_size=1)
        self.lines_num = 0
        rospy.loginfo("✅ camer_height 完整ROS版启动, 看 /car_server/debug_image")

    def check_white_lines(self, image, low, high):
        region = image[low:high, :]
        white_cols = np.any(region == 255, axis=0)
        return np.sum(white_cols)

    def detect_and_binarize_white_line(self, image):
        image = cv2.resize(image, (640, 480))
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
        binary_result = cv2.threshold(mask_white, 1, 255, cv2.THRESH_BINARY)[1]
        self.lines_num = self.check_white_lines(binary_result, STOP_LINE_LOW, STOP_LINE_HIGH)
        return binary_result

    def mid(self, follow, mask):
        halfWidth = follow.shape[1] // 2
        half = halfWidth
        mid_count = 0
        range_scan = DEAL_HIGH - DEAL_LOW
        for y in range(DEAL_HIGH, DEAL_LOW, -1):
            left_roi = mask[y][max(0, half-halfWidth):half]
            if (left_roi == np.zeros_like(left_roi)).all():
                left = max(0, half-halfWidth)
            else:
                left = np.average(np.where(mask[y][0:half] == 255))
            right_roi = mask[y][half:min(follow.shape[1], half+halfWidth)]
            if (right_roi == np.zeros_like(right_roi)).all():
                right = min(follow.shape[1], half+halfWidth)
            else:
                right = np.average(np.where(mask[y][half:follow.shape[1]] == 255)) + half
            mid = int((left + right) // 2)
            half = mid
            follow[y, mid] = 255
            mid_count += mid
        return mid_count / range_scan

    def test_line(self, lines, w):
        if lines is None:
            return 0
        T = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 != x2:
                tan = (y1-y2)/(x1-x2)
                if abs(tan) < 0.01 and abs(x1-x2) > 0.2*w:
                    T += 1
        return T

    def xunxian(self, image):
        mask = self.detect_and_binarize_white_line(image)
        follow = mask.copy()
        midoutput = self.mid(follow, mask)
        h, w = 480, 640
        cx = midoutput

        # ROI裁剪 + 畸变矫正 + 霍夫横线
        search_top = int(3.5*h/5)
        search_bot = int(search_top + 20)
        mask_roi = mask.copy()
        mask_roi[0:search_top, 0:w] = 0
        mask_roi[search_bot:h, 0:w] = 0
        undistorted = cv2.undistort(mask_roi, CAMERA_MATRIX, DIST_COEFFS)
        edges = cv2.Canny(undistorted, 50, 150)
        lines = cv2.HoughLinesP(edges, 0.5, np.pi/180, 100, minLineLength=100, maxLineGap=60)
        t = self.test_line(lines, w)

        return cx, mask, follow, t

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)

            cx, mask, follow, t = self.xunxian(frame)

            # 二值化图(含中线 follow)
            follow_bgr = cv2.cvtColor(follow, cv2.COLOR_GRAY2BGR)

            # 原图 + 标定线 + 中线
            frame = cv2.resize(frame, (640, 480))
            display = frame.copy()
            cv2.line(display, (int(cx), 0), (int(cx), 480), (0, 0, 255), 2)
            cv2.line(display, (0, BIAODING_LINE), (640, BIAODING_LINE), (0, 255, 0), 2)
            cv2.line(display, (0, BIAODING_LINE_2), (640, BIAODING_LINE_2), (0, 255, 0), 2)
            cv2.line(display, (0, DEAL_LOW), (640, DEAL_LOW), (255, 0, 0), 1)
            cv2.line(display, (0, DEAL_HIGH), (640, DEAL_HIGH), (255, 0, 0), 1)
            cv2.putText(display, f"cx={cx:.0f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if t >= 1:
                cv2.putText(display, "HENGXIAN!", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 拼接: 原图 | 二值化+中线
            combined = np.hstack([display, follow_bgr]).astype(np.uint8)

            out = Image()
            out.header.stamp = msg.header.stamp
            out.height = combined.shape[0]
            out.width = combined.shape[1]
            out.encoding = "bgr8"
            out.is_bigendian = False
            out.step = combined.shape[1] * 3
            out.data = combined.tobytes()
            self.pub.publish(out)

            rospy.loginfo_throttle(2.0, f"中点cx={cx:.0f} 白线列数={self.lines_num} 横线检测={t}")
        except Exception as e:
            import traceback
            rospy.logerr_throttle(5, f"错误: {e}\n{traceback.format_exc()}")


if __name__ == '__main__':
    try:
        node = CamerHeightRos()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
