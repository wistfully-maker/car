#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_right_v2 — 八邻域方向0(↓)拐角检测版
与follow_left.py唯一区别: 凸包检测 → 八邻域方向0占比检测

拐角触发: 近30步方向0占比 > 40% → 旋转70°
依据: 直道0≈5%, 拐角底部0≈67%

4个任务: 对正→直行0.3s→八邻域拐角检测+旋转70°→摆正直行2s→停车

用法: rosrun car_server follow_right_v2.py
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# === follow.py管线参数 ===
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

class FollowRightV2:
    def __init__(self):
        rospy.init_node('follow_right_v2', anonymous=True)
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dbg_pub = rospy.Publisher('/car_server/debug_image', Image, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)

        # === 图像参数 ===
        self.img_w, self.img_h = 160, 120
        self.deal_low, self.deal_high = 76, 105

        # === 任务1: 对正 ===
        self.aligned = False
        self.align_thr = 15.0; self.align_kp = 0.04
        self.align_max = 0.30; self.align_need = 5
        self.align_cnt = 0; self.align_smooth = 0.0

        # === 任务2: 直行 ===
        self.forward_done = False
        self.forward_start = None
        self.forward_dur = 0.3

        # === PID ===
        self.Kp = 0.03; self.Ki = 0.00010; self.Kd = 0.0
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        # === 任务3: 八邻域拐角检测 ===
        self.turn_done = False
        self.rotate_flag = 0
        self.rotate_deg = -70; self.rotate_spd = 0.8
        self.dir0_thr = 0.40        # ★ 方向0占比>40%触发
        self.dir0_consec = 0        # 连续帧计数
        self.dir0_consec_need = 3   # 连续3帧确认

        # === 任务4: 摆正直行 ===
        self.post_straight_done = False
        self.post_straight_start = None
        self.post_straight_dur = 2.0

        # === 停车 ===
        self.stop_flag = False
        self.is_stop_line_front_found = False
        self.play_voice_once = False
        self.filtered_fps_num = 0
        self.filtered_fps_num_threshold = 30
        self.stop_line_row_start = 24
        self.stop_line_row_end = 119
        self.stop_line_front_threshold = 60
        self.stop_line_back_threshold = 60

        self.fc = 0
        rospy.loginfo("=" * 50)
        rospy.loginfo("✅ follow_right_v2 右道 [八邻域方向0检测+顺时针70°]")
        rospy.loginfo(f"   拐角触发: 近30步方向0占比 > {self.dir0_thr*100:.0f}%")
        rospy.loginfo("=" * 50)

    # ================================================================
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
        hw = self.img_w // 2; half = hw; mc = 0
        rng = self.deal_high - self.deal_low
        if rng <= 0: return 0
        for y in range(self.deal_high, self.deal_low, -1):
            lr = mask[y][max(0,half-hw):half]
            L = max(0,half-hw) if not np.any(lr==255) else np.average(np.where(lr==255))
            rr = mask[y][half:min(self.img_w,half+hw)]
            R = min(self.img_w,half+hw) if not np.any(rr==255) else np.average(np.where(rr==255))+half
            half = int((L+R)//2); mc += half
        return hw - (mc/rng)

    # === 八邻域追踪 ===
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

    # === follow.py PID ===
    def _follow_pid(self, error, is_eight_finished):
        self.pid_count += 1
        if self.pid_count > 50: self.pid_count = 0; self.sum_pid = 0.0
        self.sum_pid += error
        d_pid = error - self.last_error
        angular_z = error * self.Kp + self.sum_pid * self.Ki + d_pid * self.Kd
        self.last_error = max(-5.0, min(5.0, error))
        linear_x = max(0.1, 0.45 - abs(error)*0.042) if is_eight_finished else max(0.1, 0.30 - abs(error)*0.042)
        t = Twist(); t.linear.x = linear_x; t.angular.z = angular_z
        return t

    def rotate(self, deg, spd=0.8):
        dur = abs(math.radians(deg))/spd
        cmd = Twist(); cmd.angular.z = spd if deg>0 else -spd
        rospy.loginfo(f"  旋转 {deg}°...")
        t0 = rospy.Time.now()
        while rospy.Time.now()-t0 < rospy.Duration(dur):
            self.cmd_pub.publish(cmd); rospy.sleep(0.05)
        self.cmd_pub.publish(Twist())
        rospy.loginfo("  旋转完成")

    def detect_stop_line(self, bgr_image):
        if self.is_stop_line_front_found:
            self.filtered_fps_num += 1
            if self.filtered_fps_num >= self.filtered_fps_num_threshold:
                if not self.play_voice_once:
                    self.cmd_pub.publish(Twist())
                    rospy.loginfo("🔊 过滤帧结束, 即将检测后线!")
                    rospy.sleep(0.3); self.play_voice_once = True
        h, w = bgr_image.shape[:2]
        scale = h / 120.0
        r_start = int(self.stop_line_row_start * scale)
        r_end = int(self.stop_line_row_end * scale)
        front_thresh = int(self.stop_line_front_threshold * scale)
        back_thresh = int(self.stop_line_back_threshold * scale)
        cropped = bgr_image[r_start:r_end+1, :]
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, bi = cv2.threshold(blur, 185, 255, cv2.THRESH_BINARY)
        bh, bw = bi.shape; mid = bw//2; rows_found = []
        for col in range(mid-20, mid+21, 5):
            for row in range(bh-2, 0, -1):
                if bi[row,col]==255 and bi[row+1,col]==0:
                    rows_found.append(row); break
        if len(rows_found) >= 6:
            thr = front_thresh if self.filtered_fps_num <= self.filtered_fps_num_threshold else back_thresh
            if sum(rows_found)/len(rows_found) > thr:
                if not self.is_stop_line_front_found:
                    rospy.loginfo("🛑 前白线! 微调对齐...")
                    self.rotate(20, 0.3); rospy.sleep(0.3)
                    self.is_stop_line_front_found = True
                    return False
                elif self.filtered_fps_num >= self.filtered_fps_num_threshold:
                    rospy.loginfo("🛑 后白线! 停车!")
                    return True
        return False

    def _dbg(self, mask, msg):
        if self.fc % 3 != 0: return
        try:
            d = cv2.resize(mask, (640,480)); d = cv2.cvtColor(d, cv2.COLOR_GRAY2BGR).astype(np.uint8)
            m = Image(); m.header.stamp=msg.header.stamp
            m.height=480; m.width=640; m.encoding="bgr8"
            m.is_bigendian=False; m.step=640*3; m.data=d.tobytes()
            self.dbg_pub.publish(m)
        except: pass

    # ================================================================
    def image_callback(self, msg):
        if self.turn_done or self.stop_flag: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            small = cv2.resize(frame, (self.img_w, self.img_h))
            mask = self.preprocess(small)
            err = self.midpoint_error(mask)
            self.fc += 1

            # ===== 任务1: 对正 =====
            if not self.aligned:
                self.align_smooth = 0.7*self.align_smooth + 0.3*err
                s = self.align_smooth
                ang = max(-self.align_max, min(self.align_max, s*self.align_kp))
                if abs(s) <= self.align_thr:
                    self.align_cnt += 1
                    if self.align_cnt >= self.align_need:
                        self.aligned = True; self.align_cnt = 0
                        self.cmd_pub.publish(Twist())
                        rospy.loginfo("✅ 对正完成! → 直行")
                        return
                    else:
                        rospy.loginfo_throttle(1.0, f"🔧 对正 {self.align_cnt}/{self.align_need} err={err:+.0f}")
                        self.cmd_pub.publish(Twist()); return
                else:
                    self.align_cnt = 0
                    t = Twist(); t.angular.z = ang; self.cmd_pub.publish(t); return

            # ===== 任务2: 直行 =====
            if not self.forward_done:
                if self.forward_start is None:
                    self.forward_start = rospy.Time.now()
                    rospy.loginfo("🚀 直行0.3s...")
                if (rospy.Time.now()-self.forward_start).to_sec() < self.forward_dur:
                    t = Twist(); t.linear.x = 0.20
                    t.angular.z = max(-0.25,min(0.25,err*0.02))
                    self.cmd_pub.publish(t); return
                else:
                    self.forward_done = True
                    rospy.loginfo("✅ 直行完成! → 等待拐角..."); return

            # ===== 停车线检测(任务4完成后) =====
            if self.post_straight_done:
                if self.detect_stop_line(frame):
                    self.stop_flag = True
                    self.cmd_pub.publish(Twist())
                    rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")
                    return

            # ===== 任务4: 摆正直行2s =====
            if self.post_straight_start is not None and not self.post_straight_done:
                el = (rospy.Time.now()-self.post_straight_start).to_sec()
                if el < self.post_straight_dur:
                    t = self._follow_pid(err, is_eight_finished=True)
                    self.cmd_pub.publish(t)
                    if self.fc % 15 == 0:
                        rospy.loginfo(f"🔄 直行 {el:.1f}s/{self.post_straight_dur}s err={err:+.0f}")
                    self._dbg(mask, msg); return
                else:
                    self.post_straight_done = True
                    rospy.loginfo("✅ 直行完成! 开启停车检测"); return

            # ===== 任务3: 八邻域方向0检测 =====
            # 找起点 + 追踪
            rh, rw = mask.shape; Ls = Rs = None
            for row in range(rh-5, 20, -1):
                xs = np.where(mask[row,:]==255)[0]
                if len(xs) < 15: continue
                lx, rx = xs[0], xs[-1]
                if rx-lx < 20: continue
                for x in range(max(1,lx-5), min(rw-1,lx+5)):
                    if mask[row,x]==0 and mask[row,x-1]==255: Ls=(x-1,row); break
                if Ls is None: Ls = (lx, row)
                for x in range(max(1,rx-5), min(rw-1,rx+5)):
                    if mask[row,x]==0 and mask[row,x+1]==255: Rs=(x+1,row); break
                if Rs is None: Rs = (rx, row)
                srow = row; break

            if Ls and Rs:
                _, Ld = self.trace(mask, Ls, SEEDS_R)
                _, Rd = self.trace(mask, Rs, SEEDS_L)

                # ★ 方向0占比 (近30步)
                Lr = Ld[:10] if len(Ld) >= 10 else Ld
                Rr = Rd[:10] if len(Rd) >= 10 else Rd
                l0_pct = Lr.count(0)/len(Lr) if Lr else 0
                r0_pct = Rr.count(0)/len(Rr) if Rr else 0

                # 拐角检测
                if not self.rotate_flag:
                    if (l0_pct > self.dir0_thr or r0_pct > self.dir0_thr) and (srow is not None and srow > 90):
                        self.dir0_consec += 1
                        if self.dir0_consec >= self.dir0_consec_need:
                            rospy.loginfo(f"🔀 拐角! L0%={l0_pct*100:.0f} R0%={r0_pct*100:.0f} → 旋转{self.rotate_deg}°")
                            self.rotate(self.rotate_deg, self.rotate_spd)
                            self.rotate_flag = 1; return
                    else:
                        self.dir0_consec = max(0, self.dir0_consec-1)

                if self.fc % 15 == 0:
                    marker = " ⚡" if self.dir0_consec > 0 else ""
                    rospy.loginfo(f"🔍 L0%={l0_pct*100:.0f} R0%={r0_pct*100:.0f} consec={self.dir0_consec}{marker}")

                # 旋转后微调
                if self.rotate_flag == 1:
                    if abs(err) <= 30:
                        self.post_straight_start = rospy.Time.now()
                        rospy.loginfo("✅ 旋转完成! → 摆正直行2s..."); return
                    else:
                        t = Twist(); t.angular.z = 0.3 if err>0 else -0.3
                        self.cmd_pub.publish(t); return

            # PID巡线前进
            t = self._follow_pid(err, is_eight_finished=False)
            self.cmd_pub.publish(t)
            self._dbg(mask, msg)

        except Exception as e:
            import traceback; rospy.logerr(f"ERR: {e}\n{traceback.format_exc()}")

if __name__=='__main__':
    try: FollowRightV2(); rospy.spin()
    except rospy.ROSInterruptException: pass
