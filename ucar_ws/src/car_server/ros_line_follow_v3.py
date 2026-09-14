#!/usr/bin/env python3
"""
ROS视觉巡线节点 V3 — 八邻域追踪版
====================================
管线: ROI底部40% → HSV白色提取 → 形态学 → 八邻域追踪 → 边线 → 中线 → PID

替换了原来 ros_line_follow.py 的 Sobel X + HSV 双模式方案

订阅: /usb_cam/image_raw
发布: /cmd_vel, /car_server/debug_image
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ==================== 参数 ====================

# ROI
CUT_RATIO = 0.60           # 只取图像底部40%

# HSV白色提取
HSV_LOWER = np.array([0, 0, 120])
HSV_UPPER = np.array([179, 60, 255])

# 形态学
CLOSE_KERNEL  = np.ones((5, 5), np.uint8)

# 八邻域方向表
SEEDS_L = [(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1)]
SEEDS_R = [(0,1),(1,1),(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1)]

# PID
KP = 0.008
KD = 0.001
BASE_SPEED = 0.07
MAX_ANGULAR = 0.5

# ==================== 八邻域追踪 ====================

def trace_boundary(image, start, seeds, max_steps=600):
    points = []
    cx, cy = start
    points.append((cx, cy))
    visited = set()

    for _ in range(max_steps):
        if (cx, cy) in visited:
            break
        visited.add((cx, cy))

        candidates = []
        for i in range(8):
            da = seeds[i]; ax = cx+da[0]; ay = cy+da[1]
            db = seeds[(i+1)%8]; bx = cx+db[0]; by = cy+db[1]
            if (0<=ax<image.shape[1] and 0<=ay<image.shape[0] and
                0<=bx<image.shape[1] and 0<=by<image.shape[0]):
                if image[ay,ax]==0 and image[by,bx]==255:
                    candidates.append((ax, ay, i))

        if not candidates: break
        best = min(candidates, key=lambda p: p[1])
        cx, cy = best[0], best[1]
        points.append((cx, cy))

    return points


def find_start_points(binary, start_row):
    h, w = binary.shape
    if start_row >= h: start_row = h - 1
    center = w // 2
    row_data = binary[start_row, :]

    L_start, R_start = None, None
    for x in range(center, 2, -1):
        if row_data[x] == 255 and row_data[x-1] == 0:
            L_start = (x, start_row); break
    for x in range(center, w-2):
        if row_data[x] == 255 and row_data[x+1] == 0:
            R_start = (x, start_row); break

    return L_start, R_start


def extract_center_line(h, left_pts, right_pts):
    l_border = np.full(h, -1, dtype=int)
    r_border = np.full(h, -1, dtype=int)
    center   = np.full(h, -1, dtype=int)

    for x, y in left_pts:
        if 0 <= y < h and l_border[y] == -1: l_border[y] = x
    for x, y in right_pts:
        if 0 <= y < h and r_border[y] == -1: r_border[y] = x
    for y in range(h):
        if l_border[y] != -1 and r_border[y] != -1:
            center[y] = (l_border[y] + r_border[y]) // 2

    return l_border, r_border, center


# ==================== ROS节点 ====================

class LineFollowerV3:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.frame_count = 0
        self.road_half = 30  # 默认半路宽

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image,
                                     self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_debug = rospy.Publisher("/car_server/debug_image",
                                          Image, queue_size=1)
        rospy.loginfo("✅ V3巡线节点 (八邻域追踪版) 已启动")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1

            h, w = frame.shape[:2]

            # === Step1: ROI裁剪 ===
            y0 = int(h * CUT_RATIO)
            roi = frame[y0:h, :]
            roi_h, roi_w = roi.shape[:2]

            # === Step2: HSV白色提取 ===
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

            # 形态学
            binary = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)

            # 去小连通域
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
                binary, connectivity=8)
            clean = np.zeros_like(binary)
            for i in range(1, num_labels):
                if stats[i, cv2.CC_STAT_AREA] >= 50:
                    clean[labels == i] = 255
            binary = clean

            # === Step3: 找起始点 ===
            L_start, R_start = None, None
            start_row = None
            for row in range(roi_h - 5, 20, -1):
                L, R = find_start_points(binary, row)
                if L and R and (R[0] - L[0]) > 20:
                    start_row = row
                    L_start, R_start = L, R
                    break

            error = 0.0
            angular = 0.0
            twist = Twist()

            if L_start is None or R_start is None:
                # 找不到起始点 → 直行减速
                twist.linear.x = BASE_SPEED * 0.3
                twist.angular.z = 0.0
                mode = "LOST"
            else:
                # === Step4: 八邻域追踪 ===
                L_pts = trace_boundary(binary, L_start, SEEDS_L)
                R_pts = trace_boundary(binary, R_start, SEEDS_R)

                # === Step5: 提取中线 ===
                l_border, r_border, center_line = extract_center_line(
                    roi_h, L_pts, R_pts)

                # === Step6: 计算误差 ===
                # 取底部20行的中线平均值
                bottom_centers = center_line[-20:]
                valid = bottom_centers[bottom_centers != -1]
                if len(valid) > 3:
                    avg_center = valid.mean()
                    error = roi_w // 2 - avg_center
                    self.road_half = (r_border[-20:][r_border[-20:]!=-1].mean() -
                                      l_border[-20:][l_border[-20:]!=-1].mean()) // 2
                    mode = "OK"
                else:
                    # 底部中线不够 → 用上半部分估算
                    all_valid = center_line[center_line != -1]
                    if len(all_valid) > 10:
                        avg_center = all_valid.mean()
                        error = roi_w // 2 - avg_center
                        mode = "EST"
                    else:
                        error = self.last_error
                        mode = "HOLD"

                # === Step7: PID ===
                angular = KP * error + KD * (error - self.last_error)
                self.last_error = error
                angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

                # === 速度控制 ===
                coverage = (center_line != -1).sum() / roi_h
                if coverage > 0.3:
                    twist.linear.x = BASE_SPEED
                elif coverage > 0.1:
                    twist.linear.x = BASE_SPEED * 0.7
                else:
                    twist.linear.x = BASE_SPEED * 0.4
                twist.angular.z = angular

            self.pub_cmd.publish(twist)

            # === 调试画面 ===
            debug = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            # 画起始行
            if start_row:
                cv2.line(debug, (0, start_row), (roi_w, start_row),
                        (255,255,0), 1)
            # 画起始点
            if L_start: cv2.circle(debug, L_start, 6, (0,255,0), -1)
            if R_start: cv2.circle(debug, R_start, 6, (0,0,255), -1)
            # 画追踪点
            if L_start:
                for x,y in L_pts[-200:]:
                    cv2.circle(debug, (x,y), 1, (0,255,0), -1)
            if R_start:
                for x,y in R_pts[-200:]:
                    cv2.circle(debug, (x,y), 1, (0,0,255), -1)
            # 画中线
            if L_start and R_start:
                for y in range(roi_h):
                    if center_line[y] != -1:
                        cv2.circle(debug, (center_line[y],y), 2,
                                  (255,255,0), -1)

            # 信息
            info = f"V3 F{self.frame_count} {mode} e={error:.0f} a={angular:.3f}"
            cv2.putText(debug, info, (5, 13),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,255,0), 1)

            debug = debug.astype(np.uint8)
            debug_msg = Image()
            debug_msg.header.stamp = msg.header.stamp
            debug_msg.height = debug.shape[0]
            debug_msg.width = debug.shape[1]
            debug_msg.encoding = "bgr8"
            debug_msg.is_bigendian = False
            debug_msg.step = debug.shape[1] * 3
            debug_msg.data = debug.tobytes()
            self.pub_debug.publish(debug_msg)

            if self.frame_count % 50 == 0:
                rospy.loginfo_throttle(5, info)

        except Exception as e:
            import traceback
            rospy.logerr("V3出错: %s\n%s" % (e, traceback.format_exc()))


if __name__ == "__main__":
    rospy.init_node("line_follower_v3")
    LineFollowerV3()
    rospy.spin()
