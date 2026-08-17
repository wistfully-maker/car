#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仅调试: 任务1→2→3→4 (对正→直行→拐角检测+旋转→摆正直行2s→停车)

用法:
  左道: rosrun car_server test_align_turn.py _direction:=left
  右道: rosrun car_server test_align_turn.py _direction:=right
"""

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class TestAlignTurn:
    def __init__(self):
        rospy.init_node('test_align_turn', anonymous=True)
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dbg_pub = rospy.Publisher('/car_server/debug_image', Image, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)

        self.direction = rospy.get_param('~direction', 'left')  # left / right

        # === 图像 ===
        self.img_w, self.img_h = 160, 120
        self.deal_low, self.deal_high = 76, 105
        self.eight_low, self.eight_high = 85, 118

        # === PID (follow.py 原样) ===
        self.Kp = 0.03; self.Ki = 0.00010; self.Kd = 0.0
        self.sum_pid = 0.0; self.last_error = 0.0; self.pid_count = 0

        # === 任务1: 对正 (保留修复后的平滑参数, 不还原follow的激进值) ===
        self.aligned = False
        self.align_thr = 15.0; self.align_kp = 0.04
        self.align_max = 0.30; self.align_need = 5
        self.align_cnt = 0; self.align_smooth = 0.0

        # === 任务2: 直行 ===
        self.forward_done = False
        self.forward_start = None
        self.forward_dur = 0.3

        # === 任务3: 拐角 ===
        self.turn_done = False
        self.start_time = None
        self.rotate_flag = 0
        self.tracker = ContourCornerTracker()
        self.dir_idx = 3; self.dir_thr = 1
        self.max_err = 30.0
        if self.direction == 'right':
            self.rotate_deg = -70
        else:
            self.rotate_deg = 70

        # === 任务4: 旋转后摆正直行 ===
        self.post_straight_done = False
        self.post_straight_start = None
        self.post_straight_dur = 2.0   # 2秒

        self.fc = 0
        rospy.loginfo(f"🧪 调试模式 [{self.direction}] 对正→直行→拐角旋转{self.rotate_deg}°→摆正直行2s→停车")

    def preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edge = cv2.Canny(blur, 50, 150)
        k = np.ones((3,3), np.uint8)
        d1 = cv2.dilate(edge, k, iterations=2)
        e1 = cv2.erode(d1, k, iterations=1)
        d2 = cv2.dilate(e1, k, iterations=1)
        cl = cv2.morphologyEx(d2, cv2.MORPH_CLOSE, k)
        _, bi = cv2.threshold(cl, 1, 255, cv2.THRESH_BINARY)
        return bi

    def midpoint_error(self, mask):
        hw = self.img_w // 2; half = hw; mc = 0
        rng = self.deal_high - self.deal_low
        for y in range(self.deal_high, self.deal_low, -1):
            lr = mask[y][max(0,half-hw):half]
            L = max(0,half-hw) if not np.any(lr==255) else np.average(np.where(lr==255))
            rr = mask[y][half:min(self.img_w,half+hw)]
            R = min(self.img_w,half+hw) if not np.any(rr==255) else np.average(np.where(rr==255))+half
            half = int((L+R)//2); mc += half
        return hw - (mc/rng if rng>0 else hw)

    def rotate(self, deg, spd=0.8):
        dur = abs(math.radians(deg))/spd
        cmd = Twist(); cmd.angular.z = spd if deg>0 else -spd
        rospy.loginfo(f"  旋转 {deg}°...")
        t0 = rospy.Time.now()
        while rospy.Time.now()-t0 < rospy.Duration(dur):
            self.cmd_pub.publish(cmd); rospy.sleep(0.05)
        self.cmd_pub.publish(Twist())
        rospy.loginfo("  旋转完成")

    # ================================================================
    # follow.py 原版PID
    # ================================================================
    def _follow_pid(self, error, is_eight_finished):
        self.pid_count += 1
        if self.pid_count > 50:
            self.pid_count = 0; self.sum_pid = 0.0
        self.sum_pid += error
        d_pid = error - self.last_error
        angular_z = error * self.Kp + self.sum_pid * self.Ki + d_pid * self.Kd
        self.last_error = max(-5.0, min(5.0, error))
        if is_eight_finished:
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)
        else:
            linear_x = max(0.1, 0.30 - abs(error) * 0.042)
        t = Twist()
        t.linear.x = linear_x
        t.angular.z = angular_z
        return t

    def _dbg(self, mask, msg):
        if self.fc % 3 != 0: return
        try:
            d = cv2.resize(mask, (640,480))
            d = cv2.cvtColor(d, cv2.COLOR_GRAY2BGR).astype(np.uint8)
            m = Image(); m.header.stamp=msg.header.stamp
            m.height=480; m.width=640; m.encoding="bgr8"
            m.is_bigendian=False; m.step=640*3; m.data=d.tobytes()
            self.dbg_pub.publish(m)
        except: pass

    def image_callback(self, msg):
        if self.turn_done: return
        try:
            img = cv2.resize(self.bridge.imgmsg_to_cv2(msg,"bgr8"), (self.img_w,self.img_h))
            img = cv2.flip(img, 1)
            mask = self.preprocess(img)
            err = self.midpoint_error(mask)
            self.fc += 1

            # ========== 任务1: 对正 ==========
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
                        rospy.loginfo_throttle(1.0, f"🔧 对正 {self.align_cnt}/{self.align_need} raw={err:+.0f} smooth={s:+.0f}")
                        self.cmd_pub.publish(Twist())
                        return
                else:
                    self.align_cnt = 0
                    t = Twist(); t.angular.z = ang; self.cmd_pub.publish(t)
                    return

            # ========== 任务2: 直行 ==========
            if not self.forward_done:
                if self.forward_start is None:
                    self.forward_start = rospy.Time.now()
                    rospy.loginfo("🚀 直行0.3s...")
                if (rospy.Time.now()-self.forward_start).to_sec() < self.forward_dur:
                    t = Twist(); t.linear.x = 0.20
                    t.angular.z = max(-0.25,min(0.25, err*0.02))
                    self.cmd_pub.publish(t)
                    return
                else:
                    self.forward_done = True
                    self.start_time = rospy.Time.now()
                    rospy.loginfo("✅ 直行完成! → 等待拐角...")
                    return

            # ========== 任务4: 旋转后摆正直行2s ==========
            if self.post_straight_start is not None and not self.post_straight_done:
                el = (rospy.Time.now()-self.post_straight_start).to_sec()
                if el < self.post_straight_dur:
                    t = self._follow_pid(err, is_eight_finished=True)  # follow.py原版PID
                    self.cmd_pub.publish(t)
                    if self.fc % 15 == 0:
                        rospy.loginfo(f"🔄 摆正直行 {el:.1f}s/{self.post_straight_dur}s err={err:+.0f}")
                    self._dbg(mask, msg)
                    return
                else:
                    self.post_straight_done = True
                    self.turn_done = True
                    self.cmd_pub.publish(Twist())
                    rospy.loginfo("=" * 40)
                    rospy.loginfo(f"✅✅ 全部完成! 对正+直行+拐角旋转{self.rotate_deg}°+摆正直行2s")
                    rospy.loginfo("=" * 40)
                    return

            # ========== 任务3: 拐角检测+旋转 ==========
            res = self.tracker.process(mask, (self.img_w,self.img_h),
                                       self.eight_low, self.eight_high)
            if res is not None:
                Lc = res['stats']['left_dirs'][self.dir_idx]
                Rc = res['stats']['right_dirs'][self.dir_idx]
                if self.fc % 10 == 0:
                    rospy.loginfo(f"🔍 dir={self.dir_idx} L={Lc} R={Rc}")

                if (Lc > self.dir_thr or Rc > self.dir_thr) or self.rotate_flag == 1:
                    if self.rotate_flag == 0:
                        rospy.loginfo(f"🔀 拐角! L={Lc} R={Rc} → 旋转{self.rotate_deg}°")
                        self.rotate(self.rotate_deg)
                        self.rotate_flag = 1
                        return
                    else:
                        if abs(err) <= self.max_err:
                            self.cmd_pub.publish(Twist())
                            self.post_straight_start = rospy.Time.now()
                            rospy.loginfo("✅ 旋转完成! → 摆正直行2s...")
                            return
                        else:
                            t = Twist(); t.angular.z = 0.3 if err>0 else -0.3
                            self.cmd_pub.publish(t)
                            return

            # 还没检测到拐角, follow.py原版PID巡线前进
            t = self._follow_pid(err, is_eight_finished=False)
            self.cmd_pub.publish(t)
            self._dbg(mask, msg)

        except Exception as e:
            import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")


# === 凸包拐角检测器 (follow.py 原样) ===
class ContourCornerTracker:
    def __init__(self): pass

    def _find_corners(self, cnt, hi, lo):
        if len(cnt)<6: return []
        hull = cv2.convexHull(cnt, returnPoints=False)
        if hull is None or len(hull)<3: return []
        defe = cv2.convexityDefects(cnt, hull)
        if defe is None: return []
        pts = []
        for i in range(defe.shape[0]):
            flat = defe[i].flatten(); f = int(flat[2])
            p = tuple(cnt[f][0])
            if hi<=p[1]<=lo: pts.append(p)
        return pts

    def process(self, mask, size=(160,120), lo=87, hi=117):
        f = cv2.resize(mask, size)
        f = cv2.medianBlur(f, 3)
        if hi>lo: hi,lo=lo,hi
        res = cv2.findContours(f, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts = res[0] if len(res)==2 else res[1]
        cand = []
        for c in cnts:
            pts = [tuple(p[0]) for p in c if hi<=p[0][1]<=lo]
            if len(pts)>=3: cand.append((len(pts), float(np.mean([p[0] for p in pts])), c))
        if len(cand)<2: return None
        cand.sort(key=lambda x:x[0], reverse=True)
        Lc = self._find_corners(cand[0][2], hi, lo)
        Rc = self._find_corners(cand[1][2], hi, lo)
        st = {'left_dirs':{i:0 for i in range(8)}, 'right_dirs':{i:0 for i in range(8)}}
        for side, pts in (('left',Lc),('right',Rc)):
            for p in pts:
                dx=p[0]-size[0]//2; dy=p[1]-size[1]//2
                st[side+'_dirs'][int(((math.degrees(math.atan2(dy,dx))+180)/45.0)%8)]+=1
        return {'stats':st}


if __name__=='__main__':
    try:
        TestAlignTurn(); rospy.spin()
    except rospy.ROSInterruptException: pass
