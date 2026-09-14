#!/usr/bin/env python3
"""
ROS巡线 V15 — 滑动窗口+多项式拟合(复刻line_fitting.py)
管线: Canny→IPM(dilate15+erode7)→滑动窗口→多项式拟合→中线→PID

优势: 不追踪单像素边缘, 窗口内统计均值, 自然免疫"一条线两边缘"问题
"""

import rospy, cv2, numpy as np, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# === IPM ===
INVERSE_IPM = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float32)
IPM_M = np.linalg.inv(INVERSE_IPM)
IPM_W, IPM_H = 640, 480

# === 管线 ===
GAUSSIAN = (5,5); CANNY_LO, CANNY_HI = 50, 150
DILATE_K = np.ones((15,15), np.uint8); ERODE_K = np.ones((7,7), np.uint8)
ROI_Y, ROI_H = 240, 240

# === PID ===
KP, KI, KD = 0.3, 0.0, 0.1
BASE_SPEED, MAX_ANGULAR = 0.1, 0.26
DEADZONE, LOOKAHEAD = 15, 10

# === 滑动窗口 ===
NWINDOWS, MARGIN, MINPIX = 20, 50, 50

# === 停车 ===
STOP_WIN_H, STOP_WIN_W, STOP_WHITE_THRESH = 3, 30, 0.60
STOP_CONSECUTIVE, STOP_DELAY = 3, 8.0


class LineFollowerV15:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0; self.integral = 0.0; self.road_half = 60
        self.fc = 0; self.mode = "INIT"
        self.stop_detected = False; self.stop_consecutive = 0
        self.stop_enter_time = None; self.is_stopped = False

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V15 滑动窗口+多项式拟合 启动")

    def sliding_window(self, binary):
        """滑动窗口找左右车道线像素"""
        rh, rw = binary.shape
        # 下半部直方图找左右峰值
        histogram = np.sum(binary[rh//2:, :], axis=0)
        midpoint = rw // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # 非零像素坐标
        nonzero = binary.nonzero()
        nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

        window_height = rh // NWINDOWS
        leftx_current, rightx_current = leftx_base, rightx_base
        left_inds, right_inds = [], []

        for window in range(NWINDOWS):
            win_y_low = rh - (window+1)*window_height
            win_y_high = rh - window*window_height
            win_xleft_low = leftx_current - MARGIN
            win_xleft_high = leftx_current + MARGIN
            win_xright_low = rightx_current - MARGIN
            win_xright_high = rightx_current + MARGIN

            good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_inds.append(good_left); right_inds.append(good_right)

            if len(good_left) > MINPIX:
                leftx_current = int(np.mean(nonzerox[good_left]))
            if len(good_right) > MINPIX:
                rightx_current = int(np.mean(nonzerox[good_right]))

        try:
            left_inds = np.concatenate(left_inds)
            right_inds = np.concatenate(right_inds)
        except ValueError:
            return None, None

        leftx = nonzerox[left_inds]; lefty = nonzeroy[left_inds]
        rightx = nonzerox[right_inds]; righty = nonzeroy[right_inds]
        return (leftx, lefty), (rightx, righty)

    def fit_poly(self, pts, rh):
        """二次多项式拟合"""
        if pts is None or len(pts[0]) < 10: return None
        x, y = pts
        try:
            fit = np.polyfit(y, x, 2)
            ploty = np.linspace(0, rh-1, rh)
            fitx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
            return fitx.astype(int)
        except:
            return None

    def pub_dbg(self, dbg, msg):
        dbg = dbg.astype(np.uint8)
        dm = Image(); dm.header.stamp = msg.header.stamp
        dm.height = dbg.shape[0]; dm.width = dbg.shape[1]
        dm.encoding = "bgr8"; dm.is_bigendian = False
        dm.step = dbg.shape[1]*3; dm.data = dbg.tobytes()
        self.pub_dbg.publish(dm)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fc += 1
            h, w = frame.shape[:2]

            # === 图像管线 ===
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, GAUSSIAN, 0)
            canny = cv2.Canny(blur, CANNY_LO, CANNY_HI)
            ipm = cv2.warpPerspective(canny, IPM_M, (IPM_W, IPM_H))
            dilated = cv2.dilate(ipm, DILATE_K)
            morphed = cv2.erode(dilated, ERODE_K)
            roi = morphed[ROI_Y:ROI_Y+ROI_H, 0:IPM_W]
            _, binary = cv2.threshold(roi, 5, 255, cv2.THRESH_BINARY)
            rh, rw = binary.shape

            # === 停车检测 ===
            if not self.stop_detected:
                wx1 = max(0, rw//2 - STOP_WIN_W//2)
                window = binary[rh-STOP_WIN_H:rh, wx1:wx1+STOP_WIN_W]
                wr = (window>0).sum()/window.size if window.size>0 else 0
                if wr >= STOP_WHITE_THRESH:
                    self.stop_consecutive += 1
                    rospy.loginfo(f"🛑 停车区{wr*100:.0f}% [{self.stop_consecutive}/{STOP_CONSECUTIVE}]")
                else:
                    self.stop_consecutive = max(0, self.stop_consecutive-1)
                if self.stop_consecutive >= STOP_CONSECUTIVE:
                    self.stop_detected = True
                    self.stop_enter_time = rospy.Time.now()
                    rospy.loginfo(f"🛑 检测到停车区! 直行{STOP_DELAY}秒...")

            if self.stop_detected and not self.is_stopped:
                if (rospy.Time.now()-self.stop_enter_time).to_sec() < STOP_DELAY:
                    t = Twist(); t.linear.x = BASE_SPEED; self.pub_cmd.publish(t)
                    dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                    cv2.putText(dbg, f"STOP STRAIGHT {STOP_DELAY-(rospy.Time.now()-self.stop_enter_time).to_sec():.1f}s",
                               (5,12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
                    self.pub_dbg(dbg, msg); return
                else:
                    self.is_stopped = True; rospy.loginfo("🅿 停车!")

            if self.is_stopped:
                t = Twist(); self.pub_cmd.publish(t)
                dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg, "PARKED", (rw//2-30, rh//2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                self.pub_dbg(dbg, msg); return

            # === 滑动窗口 + 多项式拟合 ===
            (lx, ly), (rx, ry) = self.sliding_window(binary) if (binary>0).sum()>100 else ((None,None),(None,None))
            L_fit = self.fit_poly((lx, ly), rh) if lx is not None else None
            R_fit = self.fit_poly((rx, ry), rh) if rx is not None else None

            twist = Twist(); error = 0.0
            nL = len(lx) if lx is not None else 0
            nR = len(rx) if rx is not None else 0

            # === 误差计算(胡萝卜点) ===
            look_y = max(0, rh - LOOKAHEAD)
            if L_fit is not None and R_fit is not None and nL>50 and nR>50:
                # 双线 → 中线
                center_line = (L_fit + R_fit)//2
                la_zone = center_line[look_y:]
                valid = la_zone[(la_zone>=0)&(la_zone<rw)]
                if len(valid)>5:
                    error = rw//2 - valid.mean()
                    # 更新路宽
                    self.road_half = int((R_fit[look_y:].mean() - L_fit[look_y:].mean())//2)
                    self.road_half = max(20, min(120, self.road_half))
                self.mode = "CENTER"
            elif L_fit is not None and nL>30:
                # 左线 → 左墙
                la_l = L_fit[look_y:]; valid = la_l[(la_l>=0)&(la_l<rw)]
                if len(valid)>5:
                    error = rw//2 - (valid.mean() + self.road_half)
                self.mode = "LEFT_WALL"
            elif R_fit is not None and nR>30:
                # 右线 → 右墙
                la_r = R_fit[look_y:]; valid = la_r[(la_r>=0)&(la_r<rw)]
                if len(valid)>5:
                    error = rw//2 - (valid.mean() - self.road_half)
                self.mode = "RIGHT_WALL"
            else:
                error = self.last_error; self.mode = "HOLD"

            # === PID ===
            if abs(error) < DEADZONE: error = 0.0
            self.integral += error; self.integral = max(-50, min(50, self.integral))
            ang = KP*error + KI*self.integral + KD*(error-self.last_error)
            self.last_error = error
            ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))
            twist.linear.x = BASE_SPEED; twist.angular.z = ang
            self.pub_cmd.publish(twist)

            # === 调试画面 ===
            dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            # 画拟合曲线
            if L_fit is not None:
                for y in range(0, rh, 3):
                    x = L_fit[y]
                    if 0<=x<rw: cv2.circle(dbg, (x,y), 1, (0,255,0), -1)
            if R_fit is not None:
                for y in range(0, rh, 3):
                    x = R_fit[y]
                    if 0<=x<rw: cv2.circle(dbg, (x,y), 1, (0,0,255), -1)
            # 中线
            if L_fit is not None and R_fit is not None:
                for y in range(0, rh, 3):
                    cx = (L_fit[y]+R_fit[y])//2
                    if 0<=cx<rw: cv2.circle(dbg, (cx,y), 1, (255,255,0), -1)
            cv2.line(dbg, (0, look_y), (rw, look_y), (255,255,0), 1)

            info = f"V15 F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}"
            cv2.putText(dbg, info, (5,12), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)
            self.pub_dbg(dbg, msg)

            if self.fc%30 == 0:
                rospy.loginfo(f"V15 F{self.fc} {self.mode} e={error:.0f}")

        except Exception as e:
            import traceback; rospy.logerr(f"V15: {e}\n{traceback.format_exc()}")


if __name__ == "__main__":
    rospy.init_node("line_follower_v15")
    LineFollowerV15()
    rospy.spin()
