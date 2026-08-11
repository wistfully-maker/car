#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_left_v4 — 时间制接近拐角
拐角确认(L0%>40% AND R0%>40%连续3帧) → 低速前进APPROACH_TIME秒 → 旋转65° → 微调 → 摆正直行2s → 停车

用法: rosrun car_server follow_left_v4.py
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

APPROACH_TIME = 0.9   # 拐角确认后前进秒数
APPROACH_SPEED = 0.15  # 接近速度

class FollowLeftV4:
    def __init__(self):
        rospy.init_node('follow_left_v4', anonymous=True)
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)

        self.img_w, self.img_h = 160, 120
        self.deal_low, self.deal_high = 76, 105

        # 对正
        self.aligned = False
        self.align_thr = 15.0; self.align_kp = 0.04
        self.align_max = 0.30; self.align_need = 5
        self.align_cnt = 0; self.align_smooth = 0.0

        # 直行
        self.forward_done = False; self.forward_start = None

        # PID
        self.Kp = 0.03; self.Ki = 0.00010; self.Kd = 0.0
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        # 拐角检测
        self.corner_detected = False
        self.dir0_thr = 0.40; self.dir0_consec = 0; self.dir0_consec_need = 3

        # 接近
        self.approach_deadline = None

        # 旋转
        self.rotate_flag = 0; self.rotate_deg = 65; self.rotate_spd = 0.8

        # 摆正直行
        self.straight_start = None   # 旋转后0.5s直行起点
        self.post_start = None; self.post_done = False; self.post_dur = 2.0
        self.cruise_start = None  # PID巡航起点(不检测停车)
        self.cruise_dur = 4.0

        # 停车
        self.stop_flag = False
        self.stop_front_found = False; self.stop_fps = 0
        self.stop_fps_thr = 36; self.stop_voice = False
        self.stop_rs = 24; self.stop_re = 119
        self.stop_f_thr = 60; self.stop_b_thr = 60

        self.fc = 0
        rospy.loginfo(f"✅ follow_left_v4 时间制接近={APPROACH_TIME}s → 旋转65°")

    # ========== 管线(同V3) ==========
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

    def detect_stop_line(self, bgr):
        if self.stop_front_found:
            self.stop_fps += 1
            if self.stop_fps >= self.stop_fps_thr and not self.stop_voice:
                self.cmd_pub.publish(Twist()); rospy.sleep(0.3)
                self.stop_voice = True; rospy.loginfo("🔊 等待后线...")
        h,w = bgr.shape[:2]; scale = h/120.0
        rs, re = int(self.stop_rs*scale), int(self.stop_re*scale)
        ft, bt = int(self.stop_f_thr*scale), int(self.stop_b_thr*scale)
        crop = bgr[rs:re+1,:]; gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        _, bi = cv2.threshold(blur, 185, 255, cv2.THRESH_BINARY)
        bh,bw=bi.shape; mid=bw//2; rf=[]
        for col in range(mid-20, mid+21, 5):
            for row in range(bh-2,0,-1):
                if bi[row,col]==255 and bi[row+1,col]==0: rf.append(row); break
        if len(rf)>=6:
            thr = ft if self.stop_fps<=self.stop_fps_thr else bt
            if sum(rf)/len(rf)>thr:
                if not self.stop_front_found:
                    rospy.loginfo("🛑 前白线!")
                    self.rotate(-16, 0.3); rospy.sleep(0.3)  # 左道顺时针16°
                    self.stop_front_found=True; return False
                elif self.stop_fps>=self.stop_fps_thr:
                    rospy.loginfo("🛑 后白线! 停车!"); return True
        return False

    # ================================================================
    def image_callback(self, msg):
        if self.stop_flag: return
        try:
            frame = cv2.flip(self.bridge.imgmsg_to_cv2(msg,"bgr8"), 1)
            small = cv2.resize(frame, (self.img_w, self.img_h))
            mask = self.preprocess(small)
            err = self.midpoint_error(mask)
            self.fc += 1

            # ===== 对正 =====
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

            # ===== 直行 =====
            if not self.forward_done:
                if self.forward_start is None: self.forward_start = rospy.Time.now()
                if (rospy.Time.now()-self.forward_start).to_sec() < 0.3:
                    t = Twist(); t.linear.x = 0.20
                    t.angular.z = max(-0.25,min(0.25,err*0.02))
                    self.cmd_pub.publish(t); return
                else:
                    self.forward_done = True
                    rospy.loginfo("✅ 直行完成!"); return

            # ===== 巡航+PID(旋转后4s不检测停车) =====
            if self.post_done:
                if self.cruise_start is None:
                    self.cruise_start = rospy.Time.now()
                el_c = (rospy.Time.now()-self.cruise_start).to_sec()
                if el_c < self.cruise_dur:
                    self.cmd_pub.publish(self._pid(err, True))
                    if self.fc%30==0: rospy.loginfo(f"🔍 巡航中... {el_c:.1f}s/{self.cruise_dur}s")
                    return
                # 巡航结束, 开始检测停车
                if self.detect_stop_line(frame):
                    self.stop_flag = True; self.cmd_pub.publish(Twist())
                    rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")
                    return
                self.cmd_pub.publish(self._pid(err, True))
                if self.fc%30==0: rospy.loginfo("🔍 找停车线中...")
                return

            # ===== 摆正直行 =====
            if self.post_start is not None and not self.post_done:
                el = (rospy.Time.now()-self.post_start).to_sec()
                if el < self.post_dur:
                    self.cmd_pub.publish(self._pid(err, True))
                    if self.fc%15==0: rospy.loginfo(f"🔄 直行 {el:.1f}s/{self.post_dur}s")
                    return
                else:
                    self.post_done = True
                    rospy.loginfo("✅ 直行完成! 开启停车"); return

            # ===== 旋转完成 → 直行0.5s → 摆正直行2s =====
            if self.rotate_flag == 1:
                self.straight_start = rospy.Time.now()
                self.rotate_flag = 2
                rospy.loginfo("✅ 旋转65°完成! → 直行0.5s..."); return

            if self.rotate_flag == 2:
                el = (rospy.Time.now()-self.straight_start).to_sec()
                if el < 0.5:
                    t = Twist(); t.linear.x = 0.12; t.angular.z = 0.0  # 纯直行不修正
                    self.cmd_pub.publish(t); return
                else:
                    self.post_start = rospy.Time.now()
                    self.rotate_flag = 3
                    rospy.loginfo("✅ 直行0.5s完成! → 摆正直行2s..."); return

            # ===== 八邻域追踪 =====
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

            if Ls and Rs:
                _, Ld = self.trace(mask, Ls, SEEDS_R)
                _, Rd = self.trace(mask, Rs, SEEDS_L)
                Ln = Ld[:10] if len(Ld)>=10 else Ld; Rn = Rd[:10] if len(Rd)>=10 else Rd
                l0 = Ln.count(0)/len(Ln) if Ln else 0; r0 = Rn.count(0)/len(Rn) if Rn else 0

                # === 拐角检测 (AND + 连续3帧) ===
                if not self.corner_detected:
                    if l0 > self.dir0_thr and r0 > self.dir0_thr:
                        self.dir0_consec += 1
                        if self.dir0_consec >= self.dir0_consec_need:
                            self.corner_detected = True
                            self.approach_deadline = rospy.Time.now() + rospy.Duration(APPROACH_TIME)
                            rospy.loginfo(f"🔀 拐角! L0%={l0*100:.0f} R0%={r0*100:.0f} → 接近{APPROACH_TIME}s...")
                    else:
                        self.dir0_consec = max(0, self.dir0_consec-1)

                # === 接近阶段 (计时) ===
                if self.corner_detected and not self.rotate_flag:
                    now = rospy.Time.now()
                    if now < self.approach_deadline:
                        t = Twist(); t.linear.x = APPROACH_SPEED
                        t.angular.z = max(-0.15, min(0.15, err*0.02))
                        self.cmd_pub.publish(t)
                        if self.fc%15==0:
                            rem = (self.approach_deadline-now).to_sec()
                            rospy.loginfo(f"🚶 接近中... {rem:.1f}s")
                        return
                    else:
                        rospy.loginfo(f"🎯 时间到! → 旋转65°")
                        self.rotate(self.rotate_deg, self.rotate_spd)
                        self.rotate_flag = 1; return

                if self.fc%15==0:
                    state = "接近" if self.corner_detected else "检测"
                    rospy.loginfo(f"🔍[{state}] L0%={l0*100:.0f} R0%={r0*100:.0f}")

            self.cmd_pub.publish(self._pid(err, self.corner_detected))

        except Exception as e:
            import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")

if __name__=='__main__':
    try: FollowLeftV4(); rospy.spin()
    except rospy.ROSInterruptException: pass
