#!/usr/bin/env python3
"""
99%复刻UP主 FTW_left.py 的巡线逻辑
====================================
逐行对照FTW_left.py实现, 只去掉了激光雷达/里程计/状态机/停车检测

和UP主一致的核心:
  - Canny→IPM→形态学(dilate15+erode7)→ROI
  - 固定左墙巡线: CENTER_LINE_OFFSET=47像素
  - 双起点同时搜索(必须左右同时找到)
  - 胡萝卜点 LOOKAHEAD=10
  - PID: Kp=0.3 Kd=0.1 死区=15
"""

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import math, time

# ============ UP主原参数(FTW_left.py) ============

# IPM
INVERSE_IPM = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float32)

# 图像处理
GAUSSIAN = (5,5); CANNY_LO, CANNY_HI = 50, 150
DILATE_K = np.ones((15,15), np.uint8)
ERODE_K  = np.ones((7,7), np.uint8)
IPM_ROI_Y, IPM_ROI_H = 240, 240

# 起始点搜索(和UP主完全一致)
START_POINT_SCAN_STEP = 10
HORIZONTAL_SEARCH_OFFSET = 20
START_POINT_SEARCH_MIN_Y = 120

# 胡萝卜点+路径偏移(和UP主完全一致)
LOOKAHEAD_DISTANCE = 10
CENTER_LINE_OFFSET = 47  # 左墙向右偏移47像素

# PID(和UP主完全一致)
Kp, Ki, Kd = 0.3, 0.0, 0.1
ERROR_DEADZONE = 15
LINEAR_SPEED = 0.1
MAX_ANGULAR_DEG = 15.0  # 最大角速度(度/秒)

# 沿墙走方向表(和UP主完全一致)
FTW_SEEDS_LEFT = [
    (0, 1), (1, 1), (1, 0), (1, -1),
    (0, -1), (-1, -1), (-1, 0), (-1, 1)
]
FTW_SEEDS_RIGHT = [
    (-1, 0), (-1, -1), (0, -1), (1, -1),
    (1, 0), (1, 1), (0, 1), (-1, 1)
]


class UpstreamFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.integral = 0.0
        self.fc = 0
        self.last_print = time.time()
        self.max_angular = math.radians(MAX_ANGULAR_DEG)

        # IPM正向矩阵
        self.ipm_m = np.linalg.inv(INVERSE_IPM)

        # ROS
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ 复刻UP主FTW_left 启动")

    # ====== 和UP主完全一致的follow_the_wall ======
    def follow_the_wall(self, image, start_point, seeds):
        points = []
        cx, cy = start_point
        h, w = image.shape[:2]

        for _ in range(400):
            points.append((cx, cy))
            candidates = []

            for i in range(8):
                da = seeds[i]; ax = cx+da[0]; ay = cy+da[1]
                db = seeds[(i+1)%8]; bx = cx+db[0]; by = cy+db[1]
                if 0<=ax<w and 0<=ay<h and 0<=bx<w and 0<=by<h:
                    if image[ay,ax]==0 and image[by,bx]==255:
                        candidates.append((ax, ay))

            if not candidates: break
            cx, cy = min(candidates, key=lambda p: p[1])

        return points

    # ====== 和UP主完全一致的extract_final_border ======
    def extract_border(self, h, points):
        border = np.full(h, -1, dtype=int)
        found = set()
        for x, y in points:
            if y not in found:
                border[y] = x; found.add(y)
        return border

    # ====== 和UP主完全一致的图像处理 ======
    def process_image(self, frame):
        h, w = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, GAUSSIAN, 0)
        canny = cv2.Canny(blur, CANNY_LO, CANNY_HI)
        ipm = cv2.warpPerspective(canny, self.ipm_m, (w, h))
        dilated = cv2.dilate(ipm, DILATE_K)
        morphed = cv2.erode(dilated, ERODE_K)
        roi = morphed[IPM_ROI_Y:IPM_ROI_Y+IPM_ROI_H, 0:w]
        _, binary = cv2.threshold(roi, 5, 255, cv2.THRESH_BINARY)
        return binary, roi

    # ====== 和UP主完全一致的起点搜索(必须左右同时找到) ======
    def find_start_points(self, binary):
        rh, rw = binary.shape[:2]
        left_start = None; right_start = None; scan_y = None

        for y in range(rh-1, START_POINT_SEARCH_MIN_Y, -START_POINT_SCAN_STEP):
            # 左起点: 中间偏右向左扫
            sx_left = (rw//2) + HORIZONTAL_SEARCH_OFFSET
            for x in range(sx_left, 0, -1):
                if binary[y,x]==0 and binary[y,x-1]==255:
                    left_start = (x-1, y); break

            # 右起点: 中间偏左向右扫
            sx_right = (rw//2) - HORIZONTAL_SEARCH_OFFSET
            for x in range(sx_right, rw-1):
                if binary[y,x]==0 and binary[y,x+1]==255:
                    right_start = (x+1, y); break

            # 左右必须同时找到
            if left_start and right_start:
                scan_y = y; break
            else:
                left_start = None; right_start = None

        return left_start, right_start, scan_y

    # ====== 和UP主完全一致的胡萝卜点误差计算 ======
    def calc_error(self, lb, rb, base_y, rh, rw):
        """左墙巡线: 左线 + CENTER_LINE_OFFSET = 目标中线"""
        anchor_y = max(0, base_y - LOOKAHEAD_DISTANCE)

        # 在[anchor_y, base_y]范围内收集左线位置
        left_positions = []
        for y in range(anchor_y, base_y+1):
            if lb[y] != -1:
                left_positions.append(lb[y])

        if len(left_positions) < 3:
            return 0.0, False

        # 左线平均位置 + 偏移 = 目标路径
        left_avg = np.mean(left_positions)
        target_x = left_avg + CENTER_LINE_OFFSET
        target_x = max(0, min(rw-1, target_x))

        error = (rw//2) - target_x
        return error, True

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fc += 1

            # === UP主图像处理管线 ===
            binary, roi_img = self.process_image(frame)
            rh, rw = binary.shape[:2]

            # === UP主起点搜索 ===
            Ls, Rs, scan_y = self.find_start_points(binary)

            twist = Twist()
            error = 0.0; is_found = False

            if Ls and Rs:
                # === UP主八邻域追踪 ===
                Lp = self.follow_the_wall(binary, Ls, FTW_SEEDS_LEFT)
                Rp = self.follow_the_wall(binary, Rs, FTW_SEEDS_RIGHT)

                if Lp and Rp:
                    lb = self.extract_border(rh, Lp)
                    rb = self.extract_border(rh, Rp)
                    base_y = Ls[1]

                    # === UP主胡萝卜点误差(左墙+偏移) ===
                    error, is_found = self.calc_error(lb, rb, base_y, rh, rw)

            # === UP主PID(含死区) ===
            if is_found:
                if abs(error) < ERROR_DEADZONE:
                    error = 0.0

                self.integral += error
                self.integral = max(-50, min(50, self.integral))
                ang = Kp*error + Ki*self.integral + Kd*(error-self.last_error)
                self.last_error = error
                twist.linear.x = LINEAR_SPEED
                twist.angular.z = max(-self.max_angular,
                                      min(self.max_angular, ang))
                mode = "OK"
            else:
                twist.linear.x = LINEAR_SPEED * 0.3
                twist.angular.z = 0.0
                mode = "LOST"

            self.pub_cmd.publish(twist)

            # === 调试画面 ===
            dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            if scan_y:
                cv2.line(dbg, (0, scan_y), (rw, scan_y), (255,255,0), 1)
            if Ls: cv2.circle(dbg, Ls, 6, (0,255,0), -1)
            if Rs: cv2.circle(dbg, Rs, 6, (0,0,255), -1)
            if is_found:
                # 画左线+偏移目标线
                for y in range(rh):
                    if lb[y]!=-1:
                        cv2.circle(dbg, (lb[y],y), 1, (0,255,0), -1)
                        tx = int(lb[y]+CENTER_LINE_OFFSET)
                        if 0<=tx<rw:
                            cv2.circle(dbg, (tx,y), 1, (0,255,255), -1)
                for y in range(rh):
                    if rb[y]!=-1:
                        cv2.circle(dbg, (rb[y],y), 1, (0,0,255), -1)

            info = (f"UPSTREAM F{self.fc} {mode} e={error:.0f} "
                    f"off={CENTER_LINE_OFFSET}")
            cv2.putText(dbg, info, (5,12), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0),1)

            dbg = dbg.astype(np.uint8)
            dbg_msg = Image()
            dbg_msg.header.stamp = msg.header.stamp
            dbg_msg.height = dbg.shape[0]; dbg_msg.width = dbg.shape[1]
            dbg_msg.encoding = "bgr8"; dbg_msg.is_bigendian = False
            dbg_msg.step = dbg.shape[1]*3; dbg_msg.data = dbg.tobytes()
            self.pub_dbg.publish(dbg_msg)

            if self.fc%30==0:
                rospy.loginfo(f"UPSTREAM F{self.fc} {mode} e={error:.0f}")

        except Exception as e:
            import traceback; rospy.logerr(f"UPSTREAM: {e}\n{traceback.format_exc()}")


if __name__ == "__main__":
    rospy.init_node("upstream_follower")
    UpstreamFollower()
    rospy.spin()
