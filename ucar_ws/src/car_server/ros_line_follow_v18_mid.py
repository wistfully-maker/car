#!/usr/bin/env python3
"""
ROS巡线 V18 — 中段: 直行2.5s→巡线→白线左转45°→右线兜底→白线停车
策略:
  0. 直行2.5s(0.2m/s) → 开启视觉巡线
  1. 顶部+底部双窗口检测白线 → 原地逆时针转45°(first_turn_done=True)
  2. 一拐后降速0.06, 左墙预判记忆+右线road_half冻结兜底
  3. 白线停车(和完整版一致: 多行扫描取峰值)
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

GAUSSIAN = (5,5); CANNY_LO, CANNY_HI = 50, 150
DILATE_K = np.ones((15,15), np.uint8); ERODE_K = np.ones((7,7), np.uint8)
ROI_Y, ROI_H = 240, 240
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]

KP, KI, KD = 0.3, 0.0, 0.1
BASE_SPEED, MAX_ANGULAR = 0.12, 0.26
DEADZONE = 15; LOOKAHEAD = 10

# 停车(和完整版一致)
STOP_WHITE_THRESH = 0.40; STOP_CONSECUTIVE = 1; STOP_DELAY = 4.5
# 左转线
TURN_ANGLE_DEG = 45; TURN_SPEED = 0.785
# 初始直行
INIT_STRAIGHT_TIME = 2.5; INIT_SPEED = 0.2


class LineFollowerV18Mid:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0; self.integral = 0.0; self.road_half = 60
        self.fc = 0; self.mode = "INIT"
        self.init_done = False; self.init_start = None
        self.first_turn_done = False; self.detect_consecutive = 0
        self.left_history = []; self.frozen_road_half = None
        self.stop_detected = False; self.stop_consecutive = 0
        self.stop_enter_time = None; self.is_stopped = False

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V18 中段启动")

    def _pub_dbg(self, dbg, msg):
        dbg = dbg.astype(np.uint8)
        dm = Image(); dm.header.stamp = msg.header.stamp
        dm.height = dbg.shape[0]; dm.width = dbg.shape[1]
        dm.encoding = "bgr8"; dm.is_bigendian = False
        dm.step = dbg.shape[1]*3; dm.data = dbg.tobytes()
        self.pub_dbg.publish(dm)

    def trace(self, image, start, seeds):
        pts=[]; cx,cy=start; pts.append((cx,cy))
        visited=set()
        for _ in range(400):
            if (cx,cy) in visited: break
            visited.add((cx,cy))
            cand=[]
            for i in range(8):
                da=seeds[i]; ax=cx+da[0]; ay=cy+da[1]
                db=seeds[(i+1)%8]; bx=cx+db[0]; by=cy+db[1]
                if 0<=ax<image.shape[1] and 0<=ay<image.shape[0] and \
                   0<=bx<image.shape[1] and 0<=by<image.shape[0]:
                    if image[ay,ax]==0 and image[by,bx]==255:
                        cand.append((ax,ay))
            if not cand: break
            cx,cy=min(cand,key=lambda p:p[1])
            pts.append((cx,cy))
        return pts

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            self.fc += 1; h,w = frame.shape[:2]

            if not self.init_done:
                if self.init_start is None: self.init_start = rospy.Time.now()
                if (rospy.Time.now()-self.init_start).to_sec() < INIT_STRAIGHT_TIME:
                    t=Twist(); t.linear.x=INIT_SPEED; self.pub_cmd.publish(t); return
                else: self.init_done=True; rospy.loginfo("✅ 初始直行完成")

            # === 图像管线 ===
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            blur=cv2.GaussianBlur(gray,GAUSSIAN,0)
            canny=cv2.Canny(blur,CANNY_LO,CANNY_HI)
            ipm=cv2.warpPerspective(canny,IPM_M,(IPM_W,IPM_H))
            dilated=cv2.dilate(ipm,DILATE_K); morphed=cv2.erode(dilated,ERODE_K)
            roi=morphed[ROI_Y:ROI_Y+ROI_H,0:IPM_W]
            _,binary=cv2.threshold(roi,5,255,cv2.THRESH_BINARY)
            rh,rw=binary.shape

            # === 停车检测(多行扫描取峰值) ===
            if self.first_turn_done and not self.stop_detected:
                bh = binary[rh-40:rh, rw//4:3*rw//4]
                br = (bh>0).sum(axis=1)/bh.shape[1]
                best = br.max()
                if best >= STOP_WHITE_THRESH:
                    self.stop_consecutive += 1
                    rospy.loginfo(f"🛑 停车区 {best*100:.0f}% [{self.stop_consecutive}/{STOP_CONSECUTIVE}]")
                else: self.stop_consecutive = max(0,self.stop_consecutive-1)
                if self.stop_consecutive >= STOP_CONSECUTIVE:
                    self.stop_detected=True; self.stop_enter_time=rospy.Time.now()
                    rospy.loginfo(f"🛑 检测到停车区! 直行{STOP_DELAY}秒...")

            if self.stop_detected and not self.is_stopped:
                if (rospy.Time.now()-self.stop_enter_time).to_sec() < STOP_DELAY:
                    t=Twist(); t.linear.x=BASE_SPEED; self.pub_cmd.publish(t)
                    dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                    cv2.putText(dbg,f"GO {STOP_DELAY-(rospy.Time.now()-self.stop_enter_time).to_sec():.1f}s",
                               (5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),1)
                    self._pub_dbg(dbg,msg); return
                else:
                    self.is_stopped=True; rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")

            if self.is_stopped:
                t=Twist(); self.pub_cmd.publish(t)
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,"PARKED",(rw//2-30,rh//2),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
                self._pub_dbg(dbg,msg); return

            # === 找起点 ===
            srow=None; Ls=Rs=None
            for row in range(rh-5,20,-1):
                xs=np.where(binary[row,:]==255)[0]
                if len(xs)<15: continue
                lx,rx=xs[0],xs[-1]
                if rx-lx<20: continue
                for x in range(max(1,lx-5),min(rw-1,lx+5)):
                    if binary[row,x]==0 and binary[row,x-1]==255: Ls=(x-1,row); break
                if Ls is None: Ls=(lx,row)
                for x in range(max(1,rx-5),min(rw-1,rx+5)):
                    if binary[row,x]==0 and binary[row,x+1]==255: Rs=(x+1,row); break
                if Rs is None: Rs=(rx,row)
                srow=row; break

            # === 左转线检测(顶部+底部双窗口) ===
            if not self.first_turn_done and not self.is_stopped:
                top = binary[0:50, rw//4:3*rw//4]
                tw = (top>0).sum()/top.size if top.size>0 else 0
                bot = binary[rh-40:rh, rw//4:3*rw//4]
                bw = (bot>0).sum(axis=1)/bot.shape[1]
                wr = max(tw, bw.max())
                if wr >= STOP_WHITE_THRESH:
                    self.detect_consecutive += 1
                    rospy.loginfo(f"↪ 左转线{wr*100:.0f}% [{self.detect_consecutive}/{STOP_CONSECUTIVE}]")
                else: self.detect_consecutive = max(0,self.detect_consecutive-1)
                if self.detect_consecutive >= STOP_CONSECUTIVE:
                    td=math.radians(TURN_ANGLE_DEG)/TURN_SPEED
                    rospy.loginfo(f"↪ 原地转{TURN_ANGLE_DEG}°"); t0=rospy.Time.now()
                    while (rospy.Time.now()-t0).to_sec()<td:
                        t=Twist(); t.angular.z=TURN_SPEED; self.pub_cmd.publish(t); rospy.sleep(0.05)
                    self.first_turn_done=True; self.last_error=0.0; self.integral=0.0
                    rospy.loginfo("✅ 旋转完成!")

            twist=Twist(); error=0.0

            if Ls and Rs:
                Lp=self.trace(binary,Ls,SEEDS_R); Rp=self.trace(binary,Rs,SEEDS_L)
                lb=np.full(rh,-1,dtype=int); rb=np.full(rh,-1,dtype=int)
                for x,y in Lp:
                    if 0<=y<rh and lb[y]==-1: lb[y]=x
                for x,y in Rp:
                    if 0<=y<rh and rb[y]==-1: rb[y]=x

                nL_all=(lb!=-1).sum(); nR_all=(rb!=-1).sum()
                la_zone=slice(max(0,rh-LOOKAHEAD),rh)

                # 更新road_half
                if (lb[la_zone]!=-1).sum()>5 and (rb[la_zone]!=-1).sum()>5:
                    vl=lb[la_zone][lb[la_zone]!=-1]; vr=rb[la_zone][rb[la_zone]!=-1]
                    if len(vl)>3 and len(vr)>3:
                        self.road_half=(vr.mean()-vl.mean())//2
                        self.road_half=max(20,min(200,self.road_half))

                # 误判检测
                same_line=False
                if nL_all>10 and nR_all>10:
                    vl=lb[la_zone][lb[la_zone]!=-1]; vr=rb[la_zone][rb[la_zone]!=-1]
                    if len(vl)>3 and len(vr)>3 and (vr.mean()-vl.mean())<40:
                        same_line=True

                # 墙切换
                if nL_all>20 and nR_all>20 and not same_line:
                    cl=np.full(rh,-1,dtype=int)
                    for y in range(rh):
                        if lb[y]!=-1 and rb[y]!=-1: cl[y]=(lb[y]+rb[y])//2
                    lc=cl[la_zone][cl[la_zone]!=-1]
                    error=rw//2-lc.mean() if len(lc)>3 else self.last_error
                    self.mode="CENTER"
                elif (nL_all>20 and nR_all>20 and same_line) or nL_all>20:
                    ll=lb[la_zone][lb[la_zone]!=-1]
                    error=rw//2-(ll.mean()+self.road_half) if len(ll)>3 else self.last_error
                    self.mode="SAME_LINE" if same_line else "LEFT_WALL"
                elif nR_all>10:
                    if self.frozen_road_half is None: self.frozen_road_half=self.road_half
                    rr=rb[la_zone][rb[la_zone]!=-1]
                    error=rw//2-(rr.mean()-self.frozen_road_half) if len(rr)>3 else self.last_error
                    self.mode="RIGHT_WALL"
                else:
                    self.frozen_road_half=None
                    error=self.last_error; self.mode="HOLD"

                # 一拐后左墙记忆
                if self.first_turn_done:
                    if nL_all>10:
                        ll=lb[la_zone][lb[la_zone]!=-1]
                        if len(ll)>3:
                            self.left_history.append(ll.mean())
                            if len(self.left_history)>15: self.left_history.pop(0)
                    elif nR_all<=10:  # 双线都丢
                        error=self.last_error; self.mode="HOLD"

                # PID
                if abs(error)<DEADZONE: error=0.0
                self.integral+=error; self.integral=max(-50,min(50,self.integral))
                ang=KP*error+KI*self.integral+KD*(error-self.last_error)
                self.last_error=error; ang=max(-MAX_ANGULAR,min(MAX_ANGULAR,ang))

                # 速度
                spd=BASE_SPEED
                if self.first_turn_done: spd=0.08
                twist.linear.x=spd; twist.angular.z=ang
                self.pub_cmd.publish(twist)

                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                if srow: cv2.line(dbg,(0,srow),(rw,srow),(255,255,0),1)
                for x,y in Lp: cv2.circle(dbg,(x,y),1,(0,255,0),-1)
                for x,y in Rp: cv2.circle(dbg,(x,y),1,(0,0,255),-1)
                for y in range(rh):
                    if lb[y]!=-1 and rb[y]!=-1: cv2.circle(dbg,((lb[y]+rb[y])//2,y),1,(255,255,0),-1)
                    elif lb[y]!=-1: cv2.circle(dbg,(int(lb[y]+self.road_half),y),1,(0,255,255),-1)
                    elif rb[y]!=-1: cv2.circle(dbg,(int(rb[y]-self.road_half),y),1,(0,255,255),-1)
                info=f"V18 F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}"
                cv2.putText(dbg,info,(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
            else:
                twist.linear.x=BASE_SPEED*0.25; self.pub_cmd.publish(twist)
                self.mode="LOST"
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,f"V18 F{self.fc} LOST",(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)

            self._pub_dbg(dbg,msg)
            if self.fc%30==0: rospy.loginfo(f"V18 F{self.fc} {self.mode} e={error:.0f}")

        except Exception as e:
            import traceback; rospy.logerr(f"V18: {e}\n{traceback.format_exc()}")

if __name__=="__main__":
    rospy.init_node("line_follower_v18")
    LineFollowerV18Mid()
    rospy.spin()
