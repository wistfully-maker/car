#!/usr/bin/env python3
"""
ROS巡线 V19 — 全左墙跟随 + 拐弯预判记忆
管线: Canny→IPM(dilate15+erode7)→八邻域→全左墙→预判→PID

策略:
  1. 始终跟随左墙, OFFSET=47(对标博主)
  2. 记录最近N帧左墙位置, 分析移动趋势预判拐弯
  3. 盲区时按记忆方向继续转, 直到重见左墙
  4. 停车: 底部中心窗口检测白线
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
BASE_SPEED, MAX_ANGULAR = 0.1, 0.26
DEADZONE, LOOKAHEAD = 15, 10
OFFSET = 47  # 对标博主 CENTER_LINE_OFFSET

# 拐弯预判
TREND_WINDOW = 15     # 记录最近15帧左墙位置
TREND_THRESH = 3      # 每帧平均左移>3像素=左转趋势
MEMORY_FRAMES = 30    # 盲区时按记忆转30帧
MEMORY_SPEED = 0.3    # 盲区角速度

# 停车
STOP_WIN_H, STOP_WIN_W, STOP_THRESH = 5, 50, 0.50
STOP_CONS, STOP_DELAY = 1, 2.5


class LineFollowerV19:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0; self.integral = 0.0
        self.fc = 0; self.mode = "INIT"
        self.left_history = []
        self.turn_memory = 0
        self.trend = "NONE"
        self.stop_detected = False; self.stop_consecutive = 0
        self.stop_enter_time = None; self.is_stopped = False

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V19 全左墙+拐弯预判 启动")

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

    def _pub_dbg(self, dbg, msg):
        dbg=dbg.astype(np.uint8)
        dm=Image(); dm.header.stamp=msg.header.stamp
        dm.height=dbg.shape[0]; dm.width=dbg.shape[1]
        dm.encoding="bgr8"; dm.is_bigendian=False
        dm.step=dbg.shape[1]*3; dm.data=dbg.tobytes()
        self.pub_dbg.publish(dm)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fc += 1; h,w = frame.shape[:2]

            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            blur=cv2.GaussianBlur(gray,GAUSSIAN,0)
            canny=cv2.Canny(blur,CANNY_LO,CANNY_HI)
            ipm=cv2.warpPerspective(canny,IPM_M,(IPM_W,IPM_H))
            dilated=cv2.dilate(ipm,DILATE_K)
            morphed=cv2.erode(dilated,ERODE_K)
            roi=morphed[ROI_Y:ROI_Y+ROI_H,0:IPM_W]
            _,binary=cv2.threshold(roi,5,255,cv2.THRESH_BINARY)
            rh,rw=binary.shape

            # === 停车检测 ===
            if not self.stop_detected:
                wx1=max(0,rw//2-STOP_WIN_W//2)
                window=binary[rh-STOP_WIN_H:rh,wx1:wx1+STOP_WIN_W]
                wr=(window>0).sum()/window.size if window.size>0 else 0
                if wr>=STOP_THRESH:
                    self.stop_consecutive+=1
                    rospy.loginfo(f"🛑 停车区{wr*100:.0f}% [{self.stop_consecutive}/{STOP_CONS}]")
                else: self.stop_consecutive=max(0,self.stop_consecutive-1)
                if self.stop_consecutive>=STOP_CONS:
                    self.stop_detected=True; self.stop_enter_time=rospy.Time.now()
                    rospy.loginfo(f"🛑 检测到停车区! 直行{STOP_DELAY}秒...")

            if self.stop_detected and not self.is_stopped:
                if (rospy.Time.now()-self.stop_enter_time).to_sec()<STOP_DELAY:
                    t=Twist(); t.linear.x=BASE_SPEED; self.pub_cmd.publish(t)
                    dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                    cv2.putText(dbg,f"GO {STOP_DELAY-(rospy.Time.now()-self.stop_enter_time).to_sec():.1f}s",
                               (5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,255,255),1)
                    self._pub_dbg(dbg,msg); return
                else: self.is_stopped=True; rospy.loginfo("🅿 停车!")

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

            # 中间兜底
            if Ls is None or Rs is None:
                for row in range(rh//2,20,-1):
                    xs=np.where(binary[row,:]==255)[0]
                    if len(xs)<15: continue
                    lx,rx=xs[0],xs[-1]
                    if rx-lx<20: continue
                    Ls=(lx,row); Rs=(rx,row); srow=row; break

            twist=Twist(); error=0.0

            if Ls and Rs:
                Lp=self.trace(binary,Ls,SEEDS_R)
                Rp=self.trace(binary,Rs,SEEDS_L)

                lb=np.full(rh,-1,dtype=int); rb=np.full(rh,-1,dtype=int)
                for x,y in Lp:
                    if 0<=y<rh and lb[y]==-1: lb[y]=x
                for x,y in Rp:
                    if 0<=y<rh and rb[y]==-1: rb[y]=x

                nL_all=(lb!=-1).sum()
                lookahead_zone=slice(max(0,rh-LOOKAHEAD),rh)

                # === 拐弯预判: 记录左墙位置变化 ===
                la_l=lb[lookahead_zone][lb[lookahead_zone]!=-1]
                if len(la_l)>3:
                    left_pos=la_l.mean()
                    self.left_history.append(left_pos)
                    if len(self.left_history)>TREND_WINDOW:
                        self.left_history.pop(0)
                    # 分析趋势
                    if len(self.left_history)>=TREND_WINDOW:
                        recent=np.array(self.left_history[-TREND_WINDOW:])
                        shift_per_frame=(recent[-1]-recent[0])/TREND_WINDOW
                        if shift_per_frame<-TREND_THRESH:
                            self.trend="LEFT"; self.turn_memory=MEMORY_FRAMES
                        elif shift_per_frame>TREND_THRESH:
                            self.trend="RIGHT"; self.turn_memory=MEMORY_FRAMES
                        else:
                            self.trend="STRAIGHT"

                # === 全左墙巡线(对标博主OFFSET=47) ===
                if nL_all>10:
                    la_l2=lb[lookahead_zone][lb[lookahead_zone]!=-1]
                    if len(la_l2)>3:
                        est_center=la_l2.mean()+OFFSET
                        error=rw//2-est_center
                    else: error=self.last_error
                    self.mode="LEFT_WALL"
                elif nL_all<=10 and self.turn_memory>0:
                    # 盲区: 按记忆方向转
                    if self.trend=="LEFT":
                        error=50  # 持续左转
                        self.mode="TURN_LEFT"
                    elif self.trend=="RIGHT":
                        error=-50
                        self.mode="TURN_RIGHT"
                    else:
                        error=self.last_error; self.mode="HOLD"
                    self.turn_memory-=1
                else:
                    error=self.last_error; self.mode="HOLD"

                # PID
                if abs(error)<DEADZONE: error=0.0
                self.integral+=error; self.integral=max(-50,min(50,self.integral))
                ang=KP*error+KI*self.integral+KD*(error-self.last_error)
                self.last_error=error
                ang=max(-MAX_ANGULAR,min(MAX_ANGULAR,ang))
                twist.linear.x=BASE_SPEED; twist.angular.z=ang
                self.pub_cmd.publish(twist)

                # 调试
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                if srow: cv2.line(dbg,(0,srow),(rw,srow),(255,255,0),1)
                for x,y in Lp: cv2.circle(dbg,(x,y),1,(0,255,0),-1)
                for x,y in Rp: cv2.circle(dbg,(x,y),1,(0,0,255),-1)
                la_vals=lb[max(0,rh-LOOKAHEAD):][lb[max(0,rh-LOOKAHEAD):]!=-1]
                tgt=int(la_vals.mean()+OFFSET) if len(la_vals)>0 else rw//2
                if 0<=tgt<rw: cv2.circle(dbg,(tgt,rh-LOOKAHEAD//2),5,(0,255,255),-1)
                info=f"V19 F{self.fc} {self.mode} e={error:.0f} trend={self.trend} mem={self.turn_memory}"
                cv2.putText(dbg,info,(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
                self._pub_dbg(dbg,msg)
            else:
                # 完全找不到起点
                if self.turn_memory>0:
                    twist.linear.x=BASE_SPEED*0.5
                    twist.angular.z=0.3 if self.trend=="LEFT" else -0.3 if self.trend=="RIGHT" else 0
                    self.pub_cmd.publish(twist); self.turn_memory-=1
                else:
                    twist.linear.x=BASE_SPEED*0.3; self.pub_cmd.publish(twist)

            if self.fc%30==0:
                rospy.loginfo(f"V19 F{self.fc} {self.mode} e={error:.0f} trend={self.trend}")

        except Exception as e:
            import traceback; rospy.logerr(f"V19: {e}\n{traceback.format_exc()}")


if __name__=="__main__":
    rospy.init_node("line_follower_v19")
    LineFollowerV19()
    rospy.spin()
