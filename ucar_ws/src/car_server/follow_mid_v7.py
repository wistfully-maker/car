#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_mid_v7 — 基于 mid_v4，停车机制改为「识别到前停车线后 0.3m/s 匀速 1.5s → 停车」
中道专用: 第一条白线旋转-70° → 拐角检测旋转 → 第二条白线停车

流程:
  对正 → 直行0.3s → PID等3s → 检测第一条白线 → 旋转-70°
  → 拐角检测(方向6) → 接近 → 旋转-70° → 直行0.5s → 摆正2s → PID巡航4s
  → 检测第二条白线(前停车线) → 0.3m/s 匀速 1.5s → 停车

相对 mid_v4 的改动：
  1. 去掉「30帧后线检测 + 超时重置」，改为识别到前停车线后计时：0.3m/s 匀速 1.5s 直接停车
  2. mid 无停车前旋转，检测到前线直接开始 0.3m/s 匀速
  3. 停车位置由「时间×速度」决定，不依赖后线在画面里的位置

用法: rosrun car_server follow_mid_v7.py
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

APPROACH_TIME = 0.8; APPROACH_SPEED = 0.15

class FollowMidV7:
    def __init__(self):
        rospy.init_node('follow_mid_v7', anonymous=True)
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
        self.timer_start = None; self.timer_dur = 3.0

        # PID
        self.Kp = 0.03; self.Ki = 0.00010; self.Kd = 0.0
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        # 第一条白线
        self.first_line_found = False; self.first_line_done = False
        self.first_line_fps = 0; self.first_line_fps_thr = 3  # 连续3帧确认

        # 拐角检测 (中道用方向6竖排)
        self.corner_detected = False
        self.dir6_thr = 0.50; self.dir6_consec = 0; self.dir6_consec_need = 3
        self.approach_deadline = None

        # 旋转
        self.rotate_flag = 0; self.rotate_spd = 0.8

        # 摆正直行
        self.straight_start = None; self.post_start = None
        self.post_done = False; self.post_dur = 2.0
        # 第一条白线旋转后屏蔽停车计时
        self.stop_shield_start = None   # 第一条白线旋转开始时间
        self.stop_shield_dur = 7.4      # 旋转后 7.4s 内不检测停车

        # 第二条白线(停车)：识别到前停车线后 0.3m/s 匀速 1.5s → 停车
        self.stop_flag = False
        self.stop_front_found = False
        self.stop_rs = 24; self.stop_re = 119
        self.stop_f_thr = 60
        self.stop_speed = 0.3            # 前线后匀速速度（m/s）
        self.stop_dur = 1.7              # 前线后行驶时长（秒）
        self.stop_front_time = None      # 前线时间戳

        self.fc = 0
        rospy.loginfo(f"✅ follow_mid_v7 前停车线后 {self.stop_speed}m/s 匀速 {self.stop_dur}s → 停车")

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
        self.last_error = max(-5.0, min(5.0, err))
        az = err*self.Kp + self.sum_pid*self.Ki + (err-self.last_error)*self.Kd
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
        """检测白线，返回 True=检测到"""
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
        if len(rf)>=6 and sum(rf)/len(rf)>f_thr:
            return True
        return False

    def detect_stop_line(self, bgr):
        """第二条白线: 识别到前线后 0.3m/s 匀速 1.5s → 停车"""
        if self.stop_front_found and self.stop_front_time is not None:
            if (rospy.Time.now() - self.stop_front_time).to_sec() >= self.stop_dur:
                rospy.loginfo(f"🛑 第二条白线后行驶 {self.stop_dur}s，停车！")
                return True
            return False
        if self.detect_white_line(bgr):
            rospy.loginfo("🛑 第二条白线(前)!")
            self.stop_front_found = True
            self.stop_front_time = rospy.Time.now()
            return False
        return False

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
                    self.timer_start = rospy.Time.now()
                    rospy.loginfo("✅ 直行完成! 等3s开始检测第一条白线"); return

            # ===== 第二条白线停车 =====
            if self.post_done:
                # 第一条白线旋转后 7.4s 内不检测停车
                if self.stop_shield_start is not None:
                    el_sh = (rospy.Time.now()-self.stop_shield_start).to_sec()
                    if el_sh < self.stop_shield_dur:
                        self.cmd_pub.publish(self._pid(err, True))
                        if self.fc%30==0: rospy.loginfo(f"🚫 屏蔽停车 {el_sh:.1f}s/{self.stop_shield_dur}s")
                        return
                if self.detect_stop_line(frame):
                    self.stop_flag = True; self.cmd_pub.publish(Twist())
                    rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")
                    return
                # 前线后：0.3m/s 匀速（角速度 PID 修正）；前线前：正常 PID
                if self.stop_front_found:
                    t = self._pid(err, True)
                    t.linear.x = self.stop_speed
                    rem = max(0.0, self.stop_dur - (rospy.Time.now() - self.stop_front_time).to_sec())
                    if self.fc%15==0: rospy.loginfo(f"🚶 停车线后匀速 {self.stop_speed}m/s，剩余 {rem:.1f}s")
                else:
                    t = self._pid(err, True)
                    if self.fc%30==0: rospy.loginfo("🔍 找第二条停车线...")
                self.cmd_pub.publish(t)
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
                    rospy.loginfo("✅ 直行完成! 开启巡航"); return

            # ===== 旋转完成 → 直行0.5s → (第一条白线:拐角检测 | 拐角:摆正) =====
            if self.rotate_flag == 1:
                self.straight_start = rospy.Time.now()
                self.rotate_flag = 2
                rospy.loginfo("✅ 旋转完成! → 直行0.5s..."); return

            if self.rotate_flag == 2:
                if (rospy.Time.now()-self.straight_start).to_sec() < 0.5:
                    self.cmd_pub.publish(self._pid(err, True)); return
                else:
                    if self.first_line_found and not self.first_line_done:
                        # 第一条白线旋转完成 → 直接开始停车检测
                        self.first_line_done = True
                        self.post_done = True
                        self.rotate_flag = 3
                        rospy.loginfo("✅ 第一条白线处理完毕! → 开始停车检测..."); return
                    else:
                        # 后续旋转完成 → 摆正
                        self.post_start = rospy.Time.now()
                        self.rotate_flag = 3
                        rospy.loginfo("✅ 直行0.5s完成! → 摆正直行2s..."); return

            # ================================================================
            # 中道特殊流程: 第一条白线 → 旋转 → 拐角检测 → 旋转 → 摆正
            # ================================================================
            timer_ok = (rospy.Time.now()-self.timer_start).to_sec() >= self.timer_dur if self.timer_start else False

            if not timer_ok:
                # 等3s, PID巡线
                self.cmd_pub.publish(self._pid(err, False))
                if self.fc%30==0: rospy.loginfo(f"⏳ 等3s... {(rospy.Time.now()-self.timer_start).to_sec():.1f}s/{self.timer_dur}s")
                return

            # ===== 阶段A: 检测第一条白线(还没找到) =====
            if not self.first_line_found:
                if self.detect_white_line(frame):
                    self.first_line_fps += 1
                    if self.first_line_fps >= self.first_line_fps_thr:
                        rospy.loginfo("🛑 第一条白线! → 旋转-70°")
                        self.stop_shield_start = rospy.Time.now()  # 记录旋转开始时间
                        self.rotate(-70, self.rotate_spd)
                        self.first_line_rotate_done = True
                        self.first_line_found = True
                        self.rotate_flag = 1  # 复用旋转→直行→摆正流程
                        return
                else:
                    self.first_line_fps = max(0, self.first_line_fps-1)
                self.cmd_pub.publish(self._pid(err, False))
                if self.fc%15==0: rospy.loginfo(f"🔍 找第一条白线... 确认={self.first_line_fps}")
                return

            # ===== 阶段B: 第一条白线处理完毕 → 直接进入摆正巡航停车 =====
            # (跳过拐角检测)
            if self.first_line_done:
                self.cmd_pub.publish(self._pid(err, True))
                return

            # 八邻域追踪(拐角检测, 中道当前不启用)
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

                # 方向6占比
                l6 = Ln.count(6)/len(Ln) if Ln else 0; r6 = Rn.count(6)/len(Rn) if Rn else 0

                # 拐角检测 + 立即旋转 (中道用方向6←)
                if not self.corner_detected:
                    if l6 > self.dir6_thr or r6 > self.dir6_thr:
                        self.dir6_consec += 1
                        if self.dir6_consec >= self.dir6_consec_need:
                            self.corner_detected = True
                            rospy.loginfo(f"🔀 拐角! L6%={l6*100:.0f} R6%={r6*100:.0f} → 旋转-70°")
                            self.rotate(-70, self.rotate_spd)
                            self.rotate_flag = 1; return
                    else:
                        self.dir6_consec = max(0, self.dir6_consec-1)

                if self.fc%15==0:
                    state = "检测"
                    rospy.loginfo(f"🔍[{state}] L6%={l6*100:.0f} R6%={r6*100:.0f}")

            self.cmd_pub.publish(self._pid(err, self.corner_detected))

        except Exception as e:
            import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")

if __name__=='__main__':
    try: FollowMidV7(); rospy.spin()
    except rospy.ROSInterruptException: pass
