#!/usr/bin/env python3
"""
ROS巡线 V20bb — 中段(右墙突变版): 直行2s→巡线→丢线→一拐→右墙→位置突变二拐→停车
策略:
  0. 直行2s(0.2m/s) → 开启视觉巡线
  1. 岔路口丢线 → 直行2s(0.15m/s)自然接近转向白线
  2. 白线检测 → 原地逆时针转45° → 强制RIGHT_WALL
  3. 第二个拐点: 右墙提供更好视野(已发现小车靠右墙时二拐更易识别)
  4. 停车: ROI下半全扫描取峰值
"""

import rospy, cv2, numpy as np, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

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

INIT_STRAIGHT = 2.0; INIT_SPEED = 0.2
LOST_STRAIGHT = 1.5; LOST_SPEED = 0.15   # 丢线直行1.5s
TURN_ANGLE = 45; TURN_SPEED = 0.785
STOP_THRESH = 0.50; STOP_CONS = 3; STOP_DELAY = 4.5


class LineFollowerV20bMid:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0; self.integral = 0.0; self.road_half = 60
        self.fc = 0; self.mode = "INIT"
        # 初始阶段
        self.init_done = False; self.init_start = None
        # 丢线处理
        self.lost_start = None
        # 第一拐
        self.turn1_done = False; self.turn1_consec = 0
        # 第二拐(方向序列检测)
        self.turn2_done = False; self.turn2_consec = 0
        self.wall_history = []    # 右墙位置记忆(二拐预判)
        # 停车
        self.stop_detected = False; self.stop_consec = 0
        self.stop_enter = None; self.is_stopped = False

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V20b 中段启动")

    def _dbg(self, dbg, msg):
        dbg = dbg.astype(np.uint8)
        dm = Image(); dm.header.stamp = msg.header.stamp
        dm.height = dbg.shape[0]; dm.width = dbg.shape[1]
        dm.encoding = "bgr8"; dm.is_bigendian = False
        dm.step = dbg.shape[1]*3; dm.data = dbg.tobytes()
        self.pub_dbg.publish(dm)

    def trace(self, img, start, seeds):
        pts=[]; dirs=[]; cx,cy=start; pts.append((cx,cy))
        visited=set()
        for _ in range(400):
            if (cx,cy) in visited: break
            visited.add((cx,cy))
            cand=[]
            for i in range(8):
                da=seeds[i]; ax=cx+da[0]; ay=cy+da[1]
                db=seeds[(i+1)%8]; bx=cx+db[0]; by=cy+db[1]
                if 0<=ax<img.shape[1] and 0<=ay<img.shape[0] and \
                   0<=bx<img.shape[1] and 0<=by<img.shape[0]:
                    if img[ay,ax]==0 and img[by,bx]==255:
                        cand.append((ax,ay,i))
            if not cand: break
            best=min(cand,key=lambda p:p[1])
            cx,cy=best[0],best[1]
            pts.append((cx,cy)); dirs.append(best[2])
        return pts, dirs

    def detect_wall_jump(self, wall_history):
        """右墙位置突变: 最近15帧中首尾差距>30像素 → 直角弯"""
        if len(wall_history) < 15: return False
        return abs(wall_history[-1] - wall_history[0]) > 30

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fc += 1; h,w = frame.shape[:2]

            # === 初始直行 ===
            if not self.init_done:
                if self.init_start is None: self.init_start = rospy.Time.now()
                if (rospy.Time.now()-self.init_start).to_sec() < INIT_STRAIGHT:
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

            # === 停车检测(下半全扫描) ===
            if self.turn1_done and not self.stop_detected:
                half = binary[rh//2:rh, rw//4:3*rw//4]
                br = (half>0).sum(axis=1)/half.shape[1]
                best = br.max()
                if best >= STOP_THRESH:
                    self.stop_consec += 1
                    rospy.loginfo(f"🛑 停车{best*100:.0f}% [{self.stop_consec}/{STOP_CONS}]")
                else: self.stop_consec = max(0,self.stop_consec-1)
                if self.stop_consec >= STOP_CONS:
                    self.stop_detected=True; self.stop_enter=rospy.Time.now()
                    rospy.loginfo(f"🛑 停车! 直行{STOP_DELAY}s")

            if self.stop_detected and not self.is_stopped:
                if (rospy.Time.now()-self.stop_enter).to_sec() < STOP_DELAY:
                    t=Twist(); t.linear.x=BASE_SPEED; self.pub_cmd.publish(t)
                    dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                    cv2.putText(dbg,f"GO {STOP_DELAY-(rospy.Time.now()-self.stop_enter).to_sec():.1f}s",
                               (5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),1)
                    self._dbg(dbg,msg); return
                else:
                    self.is_stopped=True; rospy.loginfo("🅿 停车!")
                    with open("/tmp/stop_done.txt","w") as f: f.write("parked")

            if self.is_stopped:
                t=Twist(); self.pub_cmd.publish(t)
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,"PARKED",(rw//2-30,rh//2),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
                self._dbg(dbg,msg); return

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

            # === 一拐: 丢线→直行1.5s→立即左转 ===
            if not self.turn1_done:
                if Ls is None and Rs is None:
                    if self.lost_start is None:
                        self.lost_start = rospy.Time.now()
                        rospy.logwarn("⚠️ 丢线! 直行1.5s后左转...")
                    el = (rospy.Time.now()-self.lost_start).to_sec()
                    if el < LOST_STRAIGHT:
                        t=Twist(); t.linear.x=LOST_SPEED; self.pub_cmd.publish(t)
                        dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                        cv2.putText(dbg,f"LOST STRAIGHT {LOST_STRAIGHT-el:.1f}s",
                                   (5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)
                        self._dbg(dbg,msg); return
                    else:
                        # 直行结束→立即左转
                        td=math.radians(TURN_ANGLE)/TURN_SPEED
                        rospy.loginfo(f"↪ 一拐! 左转{TURN_ANGLE}°"); t0=rospy.Time.now()
                        while (rospy.Time.now()-t0).to_sec()<td:
                            t=Twist(); t.angular.z=TURN_SPEED; self.pub_cmd.publish(t); rospy.sleep(0.05)
                        self.turn1_done=True; self.last_error=0.0; self.integral=0.0
                        self.lost_start=None
                        rospy.loginfo("✅ 左转完成, 强制RIGHT_WALL!")
                elif Ls and Rs:
                    self.lost_start = None

            twist=Twist(); error=0.0

            if Ls and Rs:
                Lp,Ld=self.trace(binary,Ls,SEEDS_R)
                Rp,Rd=self.trace(binary,Rs,SEEDS_L)
                lb=np.full(rh,-1,dtype=int); rb=np.full(rh,-1,dtype=int)
                for x,y in Lp:
                    if 0<=y<rh and lb[y]==-1: lb[y]=x
                for x,y in Rp:
                    if 0<=y<rh and rb[y]==-1: rb[y]=x

                nL_all=(lb!=-1).sum(); nR_all=(rb!=-1).sum()
                la=slice(max(0,rh-LOOKAHEAD),rh)

                # 更新路宽
                if (lb[la]!=-1).sum()>5 and (rb[la]!=-1).sum()>5:
                    vl=lb[la][lb[la]!=-1]; vr=rb[la][rb[la]!=-1]
                    if len(vl)>3 and len(vr)>3:
                        self.road_half=(vr.mean()-vl.mean())//2
                        self.road_half=max(20,min(200,self.road_half))

                # === 第二拐检测: 右墙位置突变 → 延迟直行后左转 ===
                if self.turn1_done and not self.turn2_done:
                    if len(self.wall_history) >= 15:
                        jump = self.wall_history[-1] - self.wall_history[0]
                        is_turn = abs(jump) > 30
                        if self.fc % 30 == 0:
                            rospy.loginfo(f"🔍 二拐 jump={jump:.0f}px turn={is_turn}")
                        if is_turn:
                            self.turn2_consec += 1
                            if self.turn2_consec >= 3:
                                rospy.loginfo(f"🔍 二拐确认! 直行3.5s后转...")
                                self.turn2_detected = rospy.Time.now()
                                self.turn2_consec = 0
                        else:
                            self.turn2_consec = max(0, self.turn2_consec-1)
                    # 已确认→等延迟后转
                    if hasattr(self,'turn2_detected') and self.turn2_detected:
                        el = (rospy.Time.now()-self.turn2_detected).to_sec()
                        if el < 3.5:
                            self.mode = "WAIT_TURN"
                        else:
                            rospy.loginfo("↩ 二拐左转55°!")
                            td=math.radians(55)/TURN_SPEED; t0=rospy.Time.now()
                            while (rospy.Time.now()-t0).to_sec()<td:
                                t=Twist(); t.angular.z=TURN_SPEED; self.pub_cmd.publish(t); rospy.sleep(0.05)
                            self.turn2_done=True; self.last_error=0.0; self.integral=0.0
                            del self.turn2_detected
                            rospy.loginfo("✅ 二拐完成!")

                # === 第一拐后强制右墙 + 记忆 ===
                if self.turn1_done and not self.turn2_done:
                    rr=rb[la][rb[la]!=-1]
                    if len(rr)>3:
                        self.wall_history.append(rr.mean())
                        if len(self.wall_history)>15: self.wall_history.pop(0)
                        error=rw//2-(rr.mean()-self.road_half)
                        self.mode="RIGHT_WALL"
                    elif len(self.wall_history)>=8:
                        recent=np.array(self.wall_history[-8:])
                        trend=recent[-1]-recent[0]
                        if trend<-2:   error=-40
                        elif trend>2:  error=40
                        else:          error=self.last_error
                        self.mode="MEMORY"
                    elif nL_all>10:
                        ll=lb[la][lb[la]!=-1]
                        error=rw//2-(ll.mean()+self.road_half) if len(ll)>3 else self.last_error
                        self.mode="LEFT_FB"
                    else: error=self.last_error; self.mode="HOLD"
                else:
                    # 正常巡线(CENTER/LEFT/RIGHT切换)
                    if nL_all>20 and nR_all>20:
                        cl=np.full(rh,-1,dtype=int)
                        for y in range(rh):
                            if lb[y]!=-1 and rb[y]!=-1: cl[y]=(lb[y]+rb[y])//2
                        lc=cl[la][cl[la]!=-1]
                        error=rw//2-lc.mean() if len(lc)>3 else self.last_error
                        self.mode="CENTER"
                    elif nL_all>20:
                        ll=lb[la][lb[la]!=-1]
                        error=rw//2-(ll.mean()+self.road_half) if len(ll)>3 else self.last_error
                        self.mode="LEFT_WALL"
                    elif nR_all>20:
                        rr=rb[la][rb[la]!=-1]
                        error=rw//2-(rr.mean()-self.road_half) if len(rr)>3 else self.last_error
                        self.mode="RIGHT_WALL"
                    else: error=self.last_error; self.mode="HOLD"

                # PID
                if abs(error)<DEADZONE: error=0.0
                self.integral+=error; self.integral=max(-50,min(50,self.integral))
                ang=KP*error+KI*self.integral+KD*(error-self.last_error)
                self.last_error=error; ang=max(-MAX_ANGULAR,min(MAX_ANGULAR,ang))

                spd = 0.1 if self.turn1_done else BASE_SPEED
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
                info=f"V20b F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}"
                cv2.putText(dbg,info,(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
            else:
                twist.linear.x=BASE_SPEED*0.25; self.pub_cmd.publish(twist)
                self.mode="LOST"
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,f"V20b F{self.fc} LOST",(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)

            self._dbg(dbg,msg)
            if self.fc%30==0: rospy.loginfo(f"V20b F{self.fc} {self.mode} e={error:.0f}")

        except Exception as e:
            import traceback; rospy.logerr(f"V20b: {e}\n{traceback.format_exc()}")

if __name__=="__main__":
    rospy.init_node("line_follower_v20b")
    LineFollowerV20bMid()
    rospy.spin()
