#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
国赛中道 — V7单回调架构 + 雷达避障

中道: 第一条白线旋转-70° → 避障(右移0.42→前进0.5→左移0.42) → 第二条白线停车

用法: rosrun car_server follow_national_mid.py
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

# 避障(中道: 右移→前进→左移)
AVOID_ANGLE_DEG = 20.0  # 正前方±10°, 只认车前挡板
AVOID_DIST_M = 0.25
AVOID_POINT_THR = 10
AVOID_CONSEC = 3
STRAFE_OUT_DIST = 0.42   # 右移
FORWARD_DIST = 0.5
STRAFE_IN_DIST = 0.42    # 左移
STRAFE_SPEED = 0.15
FORWARD_SPEED = 0.15


class NationalMidFollower:
    def __init__(self):
        rospy.init_node('follow_national_mid', anonymous=True)
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

        self.img_w, self.img_h = 160, 120
        self.deal_low, self.deal_high = 76, 105

        # 对正
        self.aligned = False
        self.align_thr = 15.0; self.align_kp = 0.04
        self.align_max = 0.30; self.align_need = 5
        self.align_cnt = 0; self.align_smooth = 0.0

        # 直行
        self.forward_done = False; self.forward_start = None
        self.timer_start = None; self.timer_dur = 3.0

        # PID
        self.Kp = 0.03; self.Ki = 0.00010; self.Kd = 0.0
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        # 第一条白线
        self.first_line_found = False; self.first_line_done = False
        self.first_line_fps = 0; self.first_line_fps_thr = 3

        # 旋转
        self.rotate_flag = 0; self.rotate_spd = 0.8

        # 摆正直行
        self.straight_start = None; self.post_start = None
        self.post_done = False; self.post_dur = 2.0
        self.stop_shield_start = None
        self.stop_shield_dur = 7.4

        # 第二条白线停车(V7 mid)
        self.stop_flag = False
        self.stop_front_found = False
        self.stop_rs = 24; self.stop_re = 119
        self.stop_f_thr = 60
        self.stop_speed = 0.3
        self.stop_dur = 1.9
        self.stop_front_time = None

        # 雷达/里程计
        self.obstacle_detected = False
        self.latest_pose = None

        # 避障
        self.in_avoidance = False
        self.avoid_consec = 0
        self.maneuver_step = 0; self.maneuver_start_pose = None

        self.fc = 0
        rospy.loginfo("✅ 国赛中道 V7架构 + 雷达避障")

    def preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edge = cv2.Canny(blur, 50, 150)
        k = np.ones((3,3), np.uint8)
        d1 = cv2.dilate(edge, k, iterations=2); e1 = cv2.erode(d1, k, iterations=1)
        d2 = cv2.dilate(e1, k, iterations=1); cl = cv2.morphologyEx(d2, cv2.MORPH_CLOSE, k)
        _, bi = cv2.threshold(cl, 1, 255, cv2.THRESH_BINARY)
        return bi

    def midpoint_error(self, mask):
        hw = self.img_w//2; half = hw; mc = 0
        for y in range(self.deal_high, self.deal_low, -1):
            lr = mask[y][max(0,half-hw):half]
            L = max(0,half-hw) if not np.any(lr==255) else np.average(np.where(lr==255))
            rr = mask[y][half:min(self.img_w,half+hw)]
            R = min(self.img_w,half+hw) if not np.any(rr==255) else np.average(np.where(rr==255))+half
            half = int((L+R)//2); mc += half
        return hw - (mc/(self.deal_high-self.deal_low))

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

    def _pid(self, err, eight_done):
        self.pid_count += 1
        if self.pid_count > 50: self.pid_count = 0; self.sum_pid = 0.0
        self.sum_pid += err
        az = err*self.Kp + self.sum_pid*self.Ki + (err-self.last_error)*self.Kd
        self.last_error = max(-5.0, min(5.0, err))
        lx = max(0.1, 0.45-abs(err)*0.042) if eight_done else max(0.1, 0.30-abs(err)*0.042)
        t = Twist(); t.linear.x = lx; t.angular.z = az
        return t

    def rotate(self, deg, spd=0.8):
        dur = abs(math.radians(deg))/spd
        cmd = Twist(); cmd.angular.z = spd if deg>0 else -spd
        t0 = rospy.Time.now()
        while rospy.Time.now()-t0 < rospy.Duration(dur):
            self.cmd_pub.publish(cmd); rospy.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def detect_white_line(self, bgr):
        h,w = bgr.shape[:2]; scale = h/120.0
        rs, re = int(self.stop_rs*scale), int(self.stop_re*scale)
        f_thr = int(self.stop_f_thr*scale)
        crop = bgr[rs:re+1,:]; gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        _, bi = cv2.threshold(blur, 185, 255, cv2.THRESH_BINARY)
        bh,bw=bi.shape; mid=bw//2; rf=[]
        for col in range(mid-20, mid+21, 5):
            for row in range(bh-2,0,-1):
                if bi[row,col]==255 and bi[row+1,col]==0: rf.append(row); break
        return len(rf)>=6 and sum(rf)/len(rf)>f_thr

    def detect_stop_line(self, bgr):
        if self.stop_front_found and self.stop_front_time is not None:
            if (rospy.Time.now()-self.stop_front_time).to_sec() >= self.stop_dur:
                rospy.loginfo(f"🛑 第二条白线后行驶{self.stop_dur}s，停车!")
                return True
            return False
        if self.detect_white_line(bgr):
            rospy.loginfo("🛑 第二条白线(前)!")
            self.stop_front_found = True
            self.stop_front_time = rospy.Time.now()
            return False
        return False

    def scan_callback(self, msg):
        try:
            center_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
            idx_offset = int(math.radians(AVOID_ANGLE_DEG) / msg.angle_increment)
            start = max(0, center_idx - idx_offset)
            end = min(len(msg.ranges), center_idx + idx_offset)
            count = sum(1 for i in range(start, end) if 0 < msg.ranges[i] < AVOID_DIST_M)
            self.obstacle_detected = count > AVOID_POINT_THR
        except Exception:
            pass

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def _run_maneuver(self):
        pose = self.latest_pose
        if pose is None:
            return
        if self.maneuver_start_pose is None:
            self.maneuver_start_pose = pose
        start = self.maneuver_start_pose.position
        cur = pose.position
        dist = math.hypot(cur.x-start.x, cur.y-start.y)

        t = Twist()
        if self.maneuver_step == 0:  # 右移0.42
            if dist < STRAFE_OUT_DIST:
                t.linear.y = -STRAFE_SPEED
            else:
                rospy.loginfo("✅ 右移0.42完成"); self.maneuver_step = 1; self.maneuver_start_pose = pose
        elif self.maneuver_step == 1:  # 前进0.5
            if dist < FORWARD_DIST:
                t.linear.x = FORWARD_SPEED
            else:
                rospy.loginfo("✅ 前进0.5完成"); self.maneuver_step = 2; self.maneuver_start_pose = pose
        elif self.maneuver_step == 2:  # 左移0.42
            if dist < STRAFE_IN_DIST:
                t.linear.y = STRAFE_SPEED
            else:
                rospy.loginfo("✅ 左移0.42完成! 避障结束")
                self.maneuver_step = 0
                self.in_avoidance = False
                self.avoid_consec = 0
                t = Twist()
        self.cmd_pub.publish(t)

    def image_callback(self, msg):
        if self.stop_flag: return

        if self.in_avoidance:
            self._run_maneuver()
            return

        try:
            frame = cv2.flip(self.bridge.imgmsg_to_cv2(msg,"bgr8"), 1)
            small = cv2.resize(frame, (self.img_w, self.img_h))
            mask = self.preprocess(small)
            err = self.midpoint_error(mask)
            self.fc += 1

            if not self.aligned:
                self.align_smooth = 0.7*self.align_smooth + 0.3*err
                s = self.align_smooth
                ang = max(-self.align_max, min(self.align_max, s*self.align_kp))
                if abs(s) <= self.align_thr:
                    self.align_cnt += 1
                    if self.align_cnt >= self.align_need:
                        self.aligned = True; self.align_cnt = 0
                        self.cmd_pub.publish(Twist())
                        rospy.loginfo("✅ 对正完成!"); return
                    else:
                        rospy.loginfo_throttle(1.0,f"🔧 对正 {self.align_cnt}/{self.align_need}"); return
                else:
                    self.align_cnt = 0; self.cmd_pub.publish(Twist()); return

            if not self.forward_done:
                if self.forward_start is None: self.forward_start = rospy.Time.now()
                if (rospy.Time.now()-self.forward_start).to_sec() < 0.3:
                    t = Twist(); t.linear.x = 0.20
                    t.angular.z = max(-0.25,min(0.25,err*0.02))
                    self.cmd_pub.publish(t); return
                else:
                    self.forward_done = True
                    self.timer_start = rospy.Time.now()
                    rospy.loginfo("✅ 直行完成! 等3s开始检测第一条白线"); return

            # ===== 第一条白线后(停车检测 + 雷达避障) =====
            if self.post_done:
                # 第一条白线旋转后 7.4s 内不检测停车, 但雷达检测挡板
                if self.stop_shield_start is not None:
                    el_sh = (rospy.Time.now()-self.stop_shield_start).to_sec()
                    if el_sh < self.stop_shield_dur:
                        # 雷达检测挡板(只在屏蔽停车期间)
                        if self.obstacle_detected:
                            self.avoid_consec += 1
                            if self.avoid_consec >= AVOID_CONSEC:
                                rospy.loginfo("🔀 检测到挡板! 进入避障")
                                self.in_avoidance = True
                                self.maneuver_step = 0
                                self.maneuver_start_pose = None
                                self._run_maneuver()
                                return
                        else:
                            self.avoid_consec = 0
                        self.cmd_pub.publish(self._pid(err, True))
                        if self.fc%30==0: rospy.loginfo(f"🚫 屏蔽停车 {el_sh:.1f}s/{self.stop_shield_dur}s")
                        return

                if self.detect_stop_line(frame):
                    self.stop_flag = True; self.cmd_pub.publish(Twist())
                    rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")
                    return
                if self.stop_front_found:
                    t = self._pid(err, True); t.linear.x = self.stop_speed
                else:
                    t = self._pid(err, True)
                self.cmd_pub.publish(t)
                return

            if self.post_start is not None and not self.post_done:
                el = (rospy.Time.now()-self.post_start).to_sec()
                if el < self.post_dur:
                    self.cmd_pub.publish(self._pid(err, True))
                    if self.fc%15==0: rospy.loginfo(f"🔄 直行 {el:.1f}s/{self.post_dur}s")
                    return
                else:
                    self.post_done = True
                    rospy.loginfo("✅ 直行完成! 开启停车"); return

            # ===== 旋转完成 → 直行0.5s → (第一条白线:直接停车检测) =====
            if self.rotate_flag == 1:
                self.straight_start = rospy.Time.now()
                self.rotate_flag = 2
                rospy.loginfo("✅ 旋转完成! → 直行0.5s..."); return

            if self.rotate_flag == 2:
                if (rospy.Time.now()-self.straight_start).to_sec() < 0.5:
                    self.cmd_pub.publish(self._pid(err, True)); return
                else:
                    if self.first_line_found and not self.first_line_done:
                        self.first_line_done = True
                        self.post_done = True
                        self.rotate_flag = 3
                        rospy.loginfo("✅ 第一条白线处理完毕! → 开始停车检测..."); return
                    else:
                        self.post_start = rospy.Time.now()
                        self.rotate_flag = 3
                        rospy.loginfo("✅ 直行0.5s完成! → 摆正直行2s..."); return

            # ===== 等3s + 第一条白线检测 =====
            timer_ok = (rospy.Time.now()-self.timer_start).to_sec() >= self.timer_dur if self.timer_start else False

            if not timer_ok:
                self.cmd_pub.publish(self._pid(err, False))
                if self.fc%30==0: rospy.loginfo(f"⏳ 等3s... {(rospy.Time.now()-self.timer_start).to_sec():.1f}s/{self.timer_dur}s")
                return

            if not self.first_line_found:
                if self.detect_white_line(frame):
                    self.first_line_fps += 1
                    if self.first_line_fps >= self.first_line_fps_thr:
                        rospy.loginfo("🛑 第一条白线! → 旋转-70°")
                        self.stop_shield_start = rospy.Time.now()
                        self.rotate(-70, self.rotate_spd)
                        self.first_line_found = True
                        self.rotate_flag = 1
                        return
                else:
                    self.first_line_fps = max(0, self.first_line_fps-1)
                self.cmd_pub.publish(self._pid(err, False))
                if self.fc%15==0: rospy.loginfo(f"🔍 找第一条白线... 确认={self.first_line_fps}")
                return

            self.cmd_pub.publish(self._pid(err, self.first_line_found))

        except Exception as e:
            import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")


if __name__=='__main__':
    try: NationalMidFollower(); rospy.spin()
    except rospy.ROSInterruptException: pass
