#!/usr/bin/env python3
"""
ROS视觉巡线节点 — Sobel X + IPM鸟瞰变换版
===========================================
1. 裁剪ROI → 2. IPM透视变换(鸟瞰图) → 3. Sobel X竖直边缘
   → 4. 触底连续性评分 → 5. 找左右线 → 6. PID控制
"""
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ==================== 参数 ====================
CUT_RATIO = 0.60       # ROI：只处理下方
SOBEL_THRESH = 25      # 降低以适应傍晚光线
MIN_EDGE_PIXELS = 10   # 放宽最低要求
PROCESS_WIDTH = 240

# HSV阈值（Sobel找不到线时的备用方案）
LOWER_WHITE = np.array([0, 0, 100])
UPPER_WHITE = np.array([179, 60, 255])

# ---- IPM鸟瞰变换参数（从pick_four_points.py获取）----
SRC_POINTS = np.float32([(0, 98), (150, 1), (500, 2), (638, 101)])
DST_POINTS = np.float32([(0, 0), (178, 0), (178, 638), (0, 638)])
IPM_OUT_W = 178
IPM_OUT_H = 638
IPM_MATRIX = cv2.getPerspectiveTransform(SRC_POINTS, DST_POINTS)

KP = 0.008
KD = 0.001
BASE_SPEED = 0.07
MAX_ANGULAR = 0.5


class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.frame_count = 0
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image,
                                     self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_debug = rospy.Publisher("/car_server/debug_image",
                                          Image, queue_size=1)
        rospy.loginfo("✅ 巡线节点 (Sobel X 竖直边缘版)")

    def find_line_edges(self, gray, roi_h):
        """
        Sobel X(水平梯度) + 触底连续性 → 找左右线
        Sobel X只检测竖直边缘 → 白线=竖直条带边界 ✓
                             反光=方向混乱     ✗ 被过滤
        """
        h, w = gray.shape

        # Sobel X: 只检测水平梯度(竖直边缘)
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_x = np.abs(sobel_x)
        # 阈值化：保留强竖直边缘
        edges = (sobel_x > 40).astype(np.uint8) * 255

        # ---- 每列从底部向上数连续边缘(触底才有效) ----
        col_continuity = np.zeros(w, dtype=np.int32)
        col_total = np.sum(edges, axis=0)

        for x in range(w):
            col = edges[:, x]
            cnt = 0
            # 从最底部向上数，遇0即停
            for y in range(h - 1, -1, -1):
                if col[y]:
                    cnt += 1
                else:
                    break
            col_continuity[x] = cnt

        center_x = w // 2

        # ---- 左半部分：用连续性评分找最佳列 ----
        left_best = None
        left_max = 0
        for x in range(3, center_x - 5):
            if col_continuity[x] > left_max:
                left_max = col_continuity[x]
                left_best = x

        # ---- 右半部分：用连续性评分找最佳列 ----
        right_best = None
        right_max = 0
        for x in range(w - 3, center_x + 5, -1):
            if col_continuity[x] > right_max:
                right_max = col_continuity[x]
                right_best = x

        # 至少需要连续N行才算可靠
        if left_max < 15:
            left_best = None
        if right_max < 15:
            right_best = None

        return left_best, right_best, edges, col_total, col_continuity, \
               left_max, right_max

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1

            h, w = frame.shape[:2]
            scale = PROCESS_WIDTH / w
            frame = cv2.resize(frame, (PROCESS_WIDTH, int(h * scale)))

            h, w = frame.shape[:2]
            y0 = int(h * CUT_RATIO)
            roi = frame[y0:h, :]
            roi_h, roi_w = roi.shape[:2]

            # 转灰度
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # ---- 双模式检测 ----
            # 模式1: HSV（无反光时最灵敏）
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask_hsv = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, kernel)

            # 模式2: Sobel X边缘（有反光时用）
            left_x, right_x, edges, col_total, col_cont, l_max, r_max = \
                self.find_line_edges(blur, roi_h)

            # 自适应：Sobel找不到线时回退到HSV
            left_sobel = left_x
            right_sobel = right_x
            if left_x is None and right_x is None:
                # Sobel失败，用HSV找线
                num_lbls, _, stats, cents = cv2.connectedComponentsWithStats(
                    mask_hsv, connectivity=8)
                hsv_cents = []
                for i in range(1, num_lbls):
                    if stats[i, cv2.CC_STAT_AREA] > 40:
                        hsv_cents.append((cents[i][0], cents[i][1]))
                hsv_cents.sort(key=lambda p: p[0])
                lx = rx = None
                for c in hsv_cents:
                    if c[0] < roi_w // 2:
                        lx = int(c[0])
                    else:
                        rx = int(c[0])
                        break
                if lx:
                    left_x = lx
                if rx:
                    right_x = rx
                mode = "HSV"
            else:
                mode = "SOBEL"

            center_x = roi_w // 2
            error = 0.0

            both = (left_x is not None and right_x is not None)
            single = (left_x is None) != (right_x is None)  # 只有一条线

            if both:
                track_mid = (left_x + right_x) / 2.0
                error = track_mid - center_x
                self.road_half = (right_x - left_x) // 2
            elif single and hasattr(self, 'road_half'):
                if left_x is not None:
                    error = (left_x + self.road_half) - center_x
                else:
                    error = (right_x - self.road_half) - center_x

            # 自适应KP：弯道丢线时更激进纠偏
            kp_now = KP * 2.5 if single else KP
            angular = kp_now * error + KD * (error - self.last_error)
            self.last_error = error
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

            twist = Twist()
            if left_x is None and right_x is None:
                twist.linear.x = BASE_SPEED * 0.3
                twist.angular.z = 0
            elif single:
                # 单线时降速保命
                twist.linear.x = BASE_SPEED * 0.6
                twist.angular.z = angular
            else:
                twist.linear.x = BASE_SPEED
                twist.angular.z = angular
            self.pub_cmd.publish(twist)

            # ---- 调试画面：ROI | Mask | Sobel 三栏 ----
            mask_bgr = cv2.cvtColor(mask_hsv, cv2.COLOR_GRAY2BGR)
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            debug = np.hstack((roi, mask_bgr, edges_bgr))
            cv2.putText(debug, "ROI", (5, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
            cv2.putText(debug, "MASK", (roi_w + 5, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
            cv2.putText(debug, "SOBEL", (roi_w * 2 + 5, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

            # 在ROI上画线
            for x_pos, color, label in [(left_x, (255,0,0), "L"),
                                         (right_x, (0,0,255), "R")]:
                if x_pos is not None:
                    cv2.line(debug, (x_pos, 0), (x_pos, roi_h),
                             color, 2)
                    cv2.putText(debug, label, (x_pos + 5, 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.line(debug, (center_x, 0), (center_x, roi_h), (0, 255, 0), 1)
            if left_x is not None and right_x is not None:
                md = int((left_x + right_x) / 2)
                cv2.line(debug, (md, 0), (md, roi_h), (0, 255, 255), 2)

            info = "e=%.1f a=%.3f L=%s R=%s | %s" % (
                error, angular,
                str(left_x) if left_x else "N",
                str(right_x) if right_x else "N",
                mode)
            cv2.putText(debug, info, (5, roi_h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

            debug_msg = Image()
            debug_msg.header.stamp = msg.header.stamp
            debug_msg.height = debug.shape[0]
            debug_msg.width = debug.shape[1]
            debug_msg.encoding = "bgr8"
            debug_msg.is_bigendian = False
            debug_msg.step = debug.shape[1] * 3
            debug_msg.data = debug.tobytes()
            self.pub_debug.publish(debug_msg)

            if self.frame_count % 100 == 0:
                rospy.loginfo_throttle(5, info)

        except Exception as e:
            import traceback
            rospy.logerr("出错: %s\n%s" % (e, traceback.format_exc()))


if __name__ == "__main__":
    rospy.init_node("line_follower")
    LineFollower()
    rospy.spin()
