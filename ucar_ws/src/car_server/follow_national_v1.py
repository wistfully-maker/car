#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
国赛巡线 V1 — V4巡线 + 雷达挡板避障

三回调 + 状态机架构:
  image_callback: 视觉巡线(IPM+八邻域+方向码) + 拐角检测 + 停车检测
  scan_callback:  雷达检测前方挡板
  odom_callback:  里程计姿态(测平移/前进距离)
  main_control_loop(30Hz): 状态机决策

状态机:
  INIT_ALIGN → INIT_FORWARD → FOLLOW → APPROACH → ROTATE → STRAIGHT
  → POST_STRAIGHT → CRUISE → (雷达挡板 → AVOID_MANEUVER) → STOP_DETECT → FINAL_STOP

用法:
  rosrun car_server follow_national_v1.py _direction:=left   # left/right/mid
  rosrun car_server follow_national_v1.py _direction:=right
  rosrun car_server follow_national_v1.py _direction:=mid
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from threading import Lock

# ============ 视觉参数(复用V4) ============
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

INVERSE_IPM = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float32)
IPM_M = np.linalg.inv(INVERSE_IPM)
IPM_W, IPM_H = 640, 480
DILATE_K = np.ones((15,15), np.uint8)
ERODE_K  = np.ones((7,7), np.uint8)
ROI_Y, ROI_H = 240, 240

# 状态定义
(INIT_ALIGN, INIT_FORWARD, FOLLOW, APPROACH, ROTATE, STRAIGHT,
 POST_STRAIGHT, CRUISE, AVOID_MANEUVER, STOP_DETECT, FINAL_STOP) = range(11)

STATE_NAMES = ["INIT_ALIGN","INIT_FORWARD","FOLLOW","APPROACH","ROTATE",
               "STRAIGHT","POST_STRAIGHT","CRUISE","AVOID_MANEUVER",
               "STOP_DETECT","FINAL_STOP"]

# 避障参数(对标国赛)
AVOID_ANGLE_DEG = 40.0
AVOID_DIST_M = 0.4
AVOID_POINT_THR = 10
AVOID_CONSEC = 3
STRAFE_OUT_DIST = 0.5
FORWARD_DIST = 0.58
STRAFE_IN_DIST = 0.45
STRAFE_SPEED = 0.15
FORWARD_SPEED = 0.15


class NationalLineFollower:
    def __init__(self):
        rospy.init_node('follow_national_v1', anonymous=True)
        self.direction = rospy.get_param('~direction', 'left')

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

        # 三条道参数
        if self.direction == 'left':
            self.rotate_deg = 65; self.has_corner = True; self.has_first_line = False
        elif self.direction == 'right':
            self.rotate_deg = -65; self.has_corner = True; self.has_first_line = False
        else:  # mid
            self.rotate_deg = -70; self.has_corner = False; self.has_first_line = True

        # 共享变量(带锁)
        self.lock = Lock()
        self.vision_error = 0.0
        self.is_line_found = False
        self.corner_detected = False
        self.l0_pct = 0.0; self.r0_pct = 0.0
        self.is_stop_zone = False
        self.stop_front_found = False
        self.first_line_found = False
        self.obstacle_detected = False
        self.latest_pose = None

        # 状态机
        self.state = INIT_ALIGN
        self.return_state = None  # 避障后恢复的状态

        # 对正
        self.align_smooth = 0.0; self.align_cnt = 0
        # 直行
        self.forward_start = None
        # 拐角
        self.approach_deadline = None
        self.dir_consec = 0
        # 旋转
        self.rotate_flag = 0; self.straight_start = None
        self.post_start = None
        # 巡航
        self.cruise_start = None; self.cruise_done = False
        # 停车
        self.stop_fps = 0; self.stop_voice = False
        self.first_line_fps = 0
        # 避障
        self.avoid_consec = 0
        self.maneuver_step = 0; self.maneuver_start_pose = None

        # PID
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        self.fc = 0
        rospy.loginfo(f"✅ 国赛巡线V1 [{self.direction}] 三回调+状态机")
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.main_control_loop)

    # ============ 视觉处理 ============
    def preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edge = cv2.Canny(blur, 50, 150)
        ipm = cv2.warpPerspective(edge, IPM_M, (IPM_W, IPM_H))
        dilated = cv2.dilate(ipm, DILATE_K)
        morphed = cv2.erode(dilated, ERODE_K)
        roi = morphed[ROI_Y:ROI_Y+ROI_H, 0:IPM_W]
        _, binary = cv2.threshold(roi, 5, 255, cv2.THRESH_BINARY)
        return binary

    def midpoint_error(self, mask):
        hw = 160 // 2; half = hw; mc = 0
        # 用IPM ROI(240高)下的滑动中点
        rh, rw = mask.shape
        for y in range(rh-30, rh-105, -1):
            lr = mask[y][max(0,half-80):half]
            L = max(0,half-80) if not np.any(lr==255) else np.average(np.where(lr==255))
            rr = mask[y][half:min(rw,half+80)]
            R = min(rw,half+80) if not np.any(rr==255) else np.average(np.where(rr==255))+half
            half = int((L+R)//2); mc += half
        return rw//2 - (mc/75 if mc else rw//2)

    def trace(self, img_b, start, seeds):
        pts=[]; dirs=[]; cx,cy=start; pts.append((cx,cy))
        visited=set()
        for _ in range(400):
            if (cx,cy) in visited: break
            visited.add((cx,cy))
            cand=[]
            for i in range(8):
                da=seeds[i]; ax=cx+da[0]; ay=cy+da[1]
                db=seeds[(i+1)%8]; bx=cx+db[0]; by=cy+db[1]
                if 0<=ax<img_b.shape[1] and 0<=ay<img_b.shape[0] and 0<=bx<img_b.shape[1] and 0<=by<img_b.shape[0]:
                    if img_b[ay,ax]==0 and img_b[by,bx]==255: cand.append((ax,ay,i))
            if not cand: break
            best=min(cand,key=lambda p:p[1])
            cx,cy=best[0],best[1]; pts.append((cx,cy)); dirs.append(best[2])
        return pts, dirs

    def detect_stop_line(self, bgr):
        """原图阈值185检测横白线"""
        h,w = bgr.shape[:2]; scale = h/120.0
        rs, re = int(24*scale), int(119*scale)
        crop = bgr[rs:re+1,:]; gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        _, bi = cv2.threshold(blur, 185, 255, cv2.THRESH_BINARY)
        bh,bw=bi.shape; mid=bw//2; rf=[]
        for col in range(mid-20, mid+21, 5):
            for row in range(bh-2,0,-1):
                if bi[row,col]==255 and bi[row+1,col]==0: rf.append(row); break
        return len(rf)>=6 and sum(rf)/len(rf)>int(60*scale)

    # ============ 回调 ============
    def image_callback(self, msg):
        try:
            frame = cv2.flip(self.bridge.imgmsg_to_cv2(msg,"bgr8"), 1)
            mask = self.preprocess(frame)
            err = self.midpoint_error(mask)
            self.fc += 1

            # 八邻域追踪 + 方向码
            rh, rw = mask.shape; Ls = Rs = None
            for row in range(rh-5, 20, -1):
                xs = np.where(mask[row,:]==255)[0]
                if len(xs) < 15: continue
                lx, rx = xs[0], xs[-1]
                if rx-lx < 20: continue
                for x in range(max(1,lx-5), min(rw-1,lx+5)):
                    if mask[row,x]==0 and mask[row,x-1]==255: Ls=(x-1,row); break
                if Ls is None: Ls=(lx,row)
                for x in range(max(1,rx-5), min(rw-1,rx+5)):
                    if mask[row,x]==0 and mask[row,x+1]==255: Rs=(x+1,row); break
                if Rs is None: Rs=(rx,row)
                break

            l0 = r0 = 0.0; found = False
            if Ls and Rs:
                _, Ld = self.trace(mask, Ls, SEEDS_R)
                _, Rd = self.trace(mask, Rs, SEEDS_L)
                Ln = Ld[:10] if len(Ld)>=10 else Ld
                Rn = Rd[:10] if len(Rd)>=10 else Rd
                l0 = Ln.count(0)/len(Ln) if Ln else 0
                r0 = Rn.count(0)/len(Rn) if Rn else 0
                found = True

            stop = self.detect_stop_line(frame)

            with self.lock:
                self.vision_error = err
                self.is_line_found = found
                self.l0_pct = l0; self.r0_pct = r0
                self.is_stop_zone = stop

        except Exception as e:
            rospy.logerr_throttle(5, f"视觉回调错误: {e}")

    def scan_callback(self, msg):
        """雷达检测前方挡板"""
        try:
            center_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
            idx_offset = int(math.radians(AVOID_ANGLE_DEG) / msg.angle_increment)
            start = max(0, center_idx - idx_offset)
            end = min(len(msg.ranges), center_idx + idx_offset)
            count = sum(1 for i in range(start, end) if 0 < msg.ranges[i] < AVOID_DIST_M)
            with self.lock:
                self.obstacle_detected = count > AVOID_POINT_THR
        except Exception:
            pass

    def odom_callback(self, msg):
        with self.lock:
            self.latest_pose = msg.pose.pose

    # ============ PID ============
    def _pid(self, err, fast):
        self.pid_count += 1
        if self.pid_count > 50: self.pid_count = 0; self.sum_pid = 0.0
        self.sum_pid += err
        self.last_error = max(-5.0, min(5.0, err))
        az = err*0.03 + self.sum_pid*0.0001
        lx = max(0.1, 0.45-abs(err)*0.042) if fast else max(0.1, 0.30-abs(err)*0.042)
        t = Twist(); t.linear.x = lx; t.angular.z = az
        return t

    def rotate(self, deg, spd=0.8):
        dur = abs(math.radians(deg))/spd
        cmd = Twist(); cmd.angular.z = spd if deg>0 else -spd
        t0 = rospy.Time.now()
        while rospy.Time.now()-t0 < rospy.Duration(dur):
            self.cmd_pub.publish(cmd); rospy.sleep(0.05)
        self.cmd_pub.publish(Twist())

    # ============ 主状态机 ============
    def main_control_loop(self, event):
        with self.lock:
            err = self.vision_error
            found = self.is_line_found
            l0 = self.l0_pct; r0 = self.r0_pct
            stop = self.is_stop_zone
            obstacle = self.obstacle_detected
            pose = self.latest_pose

        t = Twist()

        # ===== 避障机动(最高优先级) =====
        if self.state == AVOID_MANEUVER:
            if pose is None: return
            self._run_maneuver(pose)
            return

        # ===== 对正 =====
        if self.state == INIT_ALIGN:
            self.align_smooth = 0.7*self.align_smooth + 0.3*err
            s = self.align_smooth
            if abs(s) <= 15:
                self.align_cnt += 1
                if self.align_cnt >= 5:
                    self.state = INIT_FORWARD
                    self.forward_start = rospy.Time.now()
                    rospy.loginfo("✅ 对正完成")
                else:
                    t.angular.z = s*0.04*0.15
            else:
                self.align_cnt = 0
                t.angular.z = max(-0.30, min(0.30, s*0.04))
            self.cmd_pub.publish(t); return

        # ===== 直行 =====
        if self.state == INIT_FORWARD:
            if (rospy.Time.now()-self.forward_start).to_sec() < 0.3:
                t.linear.x = 0.20; t.angular.z = max(-0.25,min(0.25,err*0.02))
                self.cmd_pub.publish(t); return
            else:
                self.state = FOLLOW
                rospy.loginfo("✅ 直行完成 → 巡线"); return

        # ===== 停车 =====
        if self.state == FINAL_STOP:
            self.cmd_pub.publish(Twist())
            return

        if self.state == STOP_DETECT:
            if self.stop_front_found:
                self.stop_fps += 1
                if self.stop_fps >= 36 and not self.stop_voice:
                    self.cmd_pub.publish(Twist()); rospy.sleep(0.3)
                    self.stop_voice = True
                    rospy.loginfo("🔊 等待后线")
            if stop:
                if not self.stop_front_found:
                    rospy.loginfo("🛑 前白线")
                    self.stop_front_found = True
                elif self.stop_fps >= 36:
                    rospy.loginfo("🛑 后白线! 停车")
                    self.state = FINAL_STOP
                    self.cmd_pub.publish(Twist())
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")
                    return
            t = self._pid(err, True); t.linear.x = min(t.linear.x, 0.48)
            self.cmd_pub.publish(t); return

        # ===== 巡航(雷达检测挡板) =====
        if self.state == CRUISE:
            # 雷达挡板检测
            if obstacle:
                self.avoid_consec += 1
                if self.avoid_consec >= AVOID_CONSEC:
                    rospy.loginfo("🔀 检测到挡板! 进入避障")
                    self.return_state = CRUISE
                    self.state = AVOID_MANEUVER
                    self.maneuver_step = 0
                    self.maneuver_start_pose = pose
                    return
            else:
                self.avoid_consec = 0

            if not self.cruise_done:
                if self.cruise_start is None:
                    self.cruise_start = rospy.Time.now()
                el = (rospy.Time.now()-self.cruise_start).to_sec()
                if el < 8.0:
                    t = self._pid(err, True)
                    if self.fc%30==0: rospy.loginfo(f"🚫 巡航屏蔽停车 {el:.1f}s/8.0s")
                    self.cmd_pub.publish(t); return
                self.cruise_done = True
                rospy.loginfo("🔊 巡航结束，开始检测停车")
            self.state = STOP_DETECT
            return

        # ===== 摆正直行 =====
        if self.state == POST_STRAIGHT:
            if (rospy.Time.now()-self.post_start).to_sec() < 2.0:
                self.cmd_pub.publish(self._pid(err, True)); return
            else:
                self.state = CRUISE
                self.cruise_start = rospy.Time.now()
                rospy.loginfo("✅ 摆正完成 → 巡航")
                return

        # ===== 旋转后0.5s直行 =====
        if self.state == STRAIGHT:
            if (rospy.Time.now()-self.straight_start).to_sec() < 0.5:
                t.linear.x = 0.12; t.angular.z = 0.0
                self.cmd_pub.publish(t); return
            else:
                # 中道: 第一条白线后直接进停车; 左右: 进摆正
                if self.direction == 'mid':
                    self.state = STOP_DETECT
                    rospy.loginfo("✅ 中道第一条白线后 → 停车检测")
                else:
                    self.state = POST_STRAIGHT
                    self.post_start = rospy.Time.now()
                    rospy.loginfo("✅ 旋转后直行完成 → 摆正")
                return

        # ===== 旋转 =====
        if self.state == ROTATE:
            rospy.loginfo(f"🎯 旋转{self.rotate_deg}°")
            self.rotate(self.rotate_deg, 0.8)
            self.state = STRAIGHT
            self.straight_start = rospy.Time.now()
            return

        # ===== 接近拐角 =====
        if self.state == APPROACH:
            now = rospy.Time.now()
            if now < self.approach_deadline:
                t.linear.x = 0.15; t.angular.z = max(-0.15,min(0.15,err*0.02))
                self.cmd_pub.publish(t); return
            else:
                self.state = ROTATE
                return

        # ===== 巡线(FOLLOW) =====
        if self.state == FOLLOW:
            # 拐角检测(左/右道)
            if self.has_corner and found:
                if l0 > 0.40 and r0 > 0.40:
                    self.dir_consec += 1
                    if self.dir_consec >= 3:
                        self.state = APPROACH
                        self.approach_deadline = rospy.Time.now() + rospy.Duration(0.9)
                        rospy.loginfo(f"🔀 拐角! L0%={l0*100:.0f} R0%={r0*100:.0f} → 接近")
                        return
                else:
                    self.dir_consec = max(0, self.dir_consec-1)

            # 中道: 第一条白线
            if self.direction == 'mid' and stop:
                self.first_line_fps += 1
                if self.first_line_fps >= 3:
                    rospy.loginfo("🛑 中道第一条白线! → 旋转")
                    self.state = ROTATE
                    return
            else:
                self.first_line_fps = 0

            t = self._pid(err, False)
            self.cmd_pub.publish(t); return

        # 默认停止
        self.cmd_pub.publish(t)

    def _run_maneuver(self, pose):
        """三步避障机动"""
        if self.maneuver_start_pose is None:
            self.maneuver_start_pose = pose
        start = self.maneuver_start_pose.position
        cur = pose.position
        dist = math.hypot(cur.x-start.x, cur.y-start.y)

        t = Twist()
        if self.maneuver_step == 0:
            if dist < STRAFE_OUT_DIST:
                t.linear.y = -STRAFE_SPEED
            else:
                rospy.loginfo("✅ 右移完成"); self.maneuver_step = 1; self.maneuver_start_pose = pose
        elif self.maneuver_step == 1:
            if dist < FORWARD_DIST:
                t.linear.x = FORWARD_SPEED
            else:
                rospy.loginfo("✅ 前进完成"); self.maneuver_step = 2; self.maneuver_start_pose = pose
        elif self.maneuver_step == 2:
            if dist < STRAFE_IN_DIST:
                t.linear.y = STRAFE_SPEED
            else:
                rospy.loginfo("✅ 左移完成! 避障结束")
                self.maneuver_step = 0
                self.state = self.return_state if self.return_state else CRUISE
                self.avoid_consec = 0
                t = Twist()
        self.cmd_pub.publish(t)


if __name__ == '__main__':
    try:
        node = NationalLineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
