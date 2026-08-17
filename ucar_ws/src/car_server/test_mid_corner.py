#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
中道测试: PID巡线 + 方向6拐角检测 → 旋转-70° → 摆正直行2s → 停车

用法: rosrun car_server test_mid_corner.py
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

class TestMidCorner:
    def __init__(self):
        rospy.init_node('test_mid_corner', anonymous=True)
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)

        self.img_w, self.img_h = 160, 120
        self.deal_low, self.deal_high = 76, 105

        # PID
        self.Kp = 0.03; self.Ki = 0.00010; self.Kd = 0.0
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        # 拐角检测(方向6)
        self.corner_detected = False
        self.dir6_thr = 0.50; self.dir6_consec = 0; self.dir6_consec_need = 3

        # 旋转
        self.rotate_flag = 0

        # 摆正直行
        self.post_start = None; self.post_done = False; self.post_dur = 2.0

        # 停车
        self.stop_flag = False
        self.stop_front_found = False; self.stop_fps = 0
        self.stop_fps_thr = 30; self.stop_voice = False
        self.stop_rs = 24; self.stop_re = 119
        self.stop_f_thr = 60; self.stop_b_thr = 60

        self.fc = 0
        rospy.loginfo("✅ 中道测试: PID巡线 + 方向6检测 → 旋转-70° → 摆正2s → 停车")

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

    def _pid(self, err):
        self.pid_count += 1
        if self.pid_count > 50: self.pid_count = 0; self.sum_pid = 0.0
        self.sum_pid += err
        self.last_error = max(-5.0, min(5.0, err))
        az = err*self.Kp + self.sum_pid*self.Ki + (err-self.last_error)*self.Kd
        lx = max(0.1, 0.30-abs(err)*0.042)
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
                    rospy.loginfo("🛑 前白线!"); self.stop_front_found=True; return False
                elif self.stop_fps>=self.stop_fps_thr:
                    rospy.loginfo("🛑 后白线! 停车!"); return True
        return False

    def image_callback(self, msg):
        if self.stop_flag: return
        try:
            frame = cv2.flip(self.bridge.imgmsg_to_cv2(msg,"bgr8"), 1)
            small = cv2.resize(frame, (self.img_w, self.img_h))
            mask = self.preprocess(small)
            err = self.midpoint_error(mask)
            self.fc += 1

            # ===== 停车 =====
            if self.post_done:
                if self.detect_stop_line(frame):
                    self.stop_flag = True; self.cmd_pub.publish(Twist())
                    rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")
                    return
                self.cmd_pub.publish(self._pid(err))
                if self.fc%30==0: rospy.loginfo("🔍 找停车线...")
                return

            # ===== 摆正直行 =====
            if self.post_start is not None and not self.post_done:
                el = (rospy.Time.now()-self.post_start).to_sec()
                if el < self.post_dur:
                    self.cmd_pub.publish(self._pid(err))
                    if self.fc%15==0: rospy.loginfo(f"🔄 直行 {el:.1f}s/{self.post_dur}s")
                    return
                else:
                    self.post_done = True
                    rospy.loginfo("✅ 直行完成! 开启停车"); return

            # ===== 旋转完成 =====
            if self.rotate_flag == 1:
                self.post_start = rospy.Time.now()
                self.rotate_flag = 2
                rospy.loginfo("✅ 旋转完成! → 摆正直行2s..."); return

            # ===== PID巡线 + 拐角检测 =====
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
                l6 = Ln.count(6)/len(Ln) if Ln else 0; r6 = Rn.count(6)/len(Rn) if Rn else 0

                if not self.corner_detected:
                    if l6 > self.dir6_thr or r6 > self.dir6_thr:
                        self.dir6_consec += 1
                        if self.dir6_consec >= self.dir6_consec_need:
                            self.corner_detected = True
                            rospy.loginfo(f"🔀 拐角! L6%={l6*100:.0f} R6%={r6*100:.0f} → 旋转-70°")
                            self.rotate(-70, 0.8)
                            self.rotate_flag = 1; return
                    else:
                        self.dir6_consec = max(0, self.dir6_consec-1)

                if self.fc%15==0:
                    rospy.loginfo(f"🔍 L6%={l6*100:.0f} R6%={r6*100:.0f} consec={self.dir6_consec}")

            self.cmd_pub.publish(self._pid(err))

        except Exception as e:
            import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")

if __name__=='__main__':
    try: TestMidCorner(); rospy.spin()
    except rospy.ROSInterruptException: pass
