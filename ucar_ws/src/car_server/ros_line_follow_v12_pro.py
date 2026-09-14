#!/usr/bin/env python3
"""
ROS巡线 V12P_pro — V12P + 方向6拐点检测: 直行→一拐→右墙→方向6计数二拐→停车
管线: Canny→IPM(dilate15+erode7)→八邻域→中线/墙线→PID

逻辑:
  启动后: 直行5秒(0.25m/s)→停车→原地左转50°→开启视觉巡线
  检测到白线: 直行8秒→停车
"""

import rospy, cv2, numpy as np, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# === UP主IPM矩阵 ===
INVERSE_IPM = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float32)
IPM_M = np.linalg.inv(INVERSE_IPM)
IPM_W, IPM_H = 640, 480

# === 管线参数 ===
GAUSSIAN = (5,5); CANNY_LO, CANNY_HI = 50, 150
DILATE_K = np.ones((15,15), np.uint8)
ERODE_K  = np.ones((7,7), np.uint8)
ROI_Y, ROI_H = 240, 240

# === 方向表(和UP主一样交换) ===
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]

# === PID(对标UP主) ===
KP, KI, KD = 0.3, 0.0, 0.1
BASE_SPEED, MAX_ANGULAR = 0.1, 0.26  # 和V17一致
DEADZONE = 15
LOOKAHEAD = 6
# 白线检测(UP主方案: ROI底部中心小窗口)
STOP_WIN_H = 3
STOP_WIN_W = 30
STOP_WHITE_THRESH = 0.60
STOP_CONSECUTIVE = 3       # 连续N帧确认
STOP_DELAY = 8.0           # 直行8秒

# 初始动作
INIT_STRAIGHT_SPEED = 0.25  # 初始直行速度(m/s)
INIT_STRAIGHT_TIME = 7.0    # 初始直行时间(秒)
TURN_ANGLE_DEG = 55         # 初始左转角度
TURN_SPEED = 0.873          # 原地转向角速度=50°/1s


class LineFollowerV12Pro:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.integral = 0.0
        self.road_half = 60
        self.fc = 0
        self.mode = "INIT"
        # 初始阶段: 0=直行, 1=左转, 2=直行过渡, 3=巡线
        self.phase = 0
        self.phase_time = None
        # 白线检测
        self.detect_consecutive = 0
        self.stop_detected = False
        # 二拐(方向6)
        self.turn2_done=False; self.turn2_consec=0
        self.turn2_detected=None
        self.stop_enter_time = None
        self.is_stopped = False

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V12P 先直行5s→左转50°→巡线→停车 启动")

    def _pub_dbg(self, dbg, msg):
        dbg_msg = Image()
        dbg_msg.header.stamp = msg.header.stamp
        dbg_msg.height = dbg.shape[0]; dbg_msg.width = dbg.shape[1]
        dbg_msg.encoding = "bgr8"; dbg_msg.is_bigendian = False
        dbg_msg.step = dbg.shape[1]*3; dbg_msg.data = dbg.tobytes()
        self.pub_dbg.publish(dbg_msg)

    def trace(self, image, start, seeds):
        pts=[]; dirs=[]; cx,cy=start; pts.append((cx,cy))
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
                        cand.append((ax,ay,i))
            if not cand: break
            best=min(cand,key=lambda p:p[1])
            cx,cy=best[0],best[1]
            pts.append((cx,cy)); dirs.append(best[2])
        return pts, dirs

    def detect_cross(self, dirs):
        """方向6≥10/30帧→拐点"""
        if len(dirs)<30: return False
        return dirs.count(6)>=10

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fc += 1
            h,w = frame.shape[:2]

            # === 初始阶段: 不跑视觉, 直接发动作指令 ===
            if self.phase < 3:
                if self.phase_time is None:
                    self.phase_time = rospy.Time.now()
                elapsed = (rospy.Time.now()-self.phase_time).to_sec()

                if self.phase == 0:
                    if elapsed < INIT_STRAIGHT_TIME:
                        t=Twist(); t.linear.x=INIT_STRAIGHT_SPEED; self.pub_cmd.publish(t)
                    else:
                        self.phase=1; self.phase_time=rospy.Time.now()
                        rospy.loginfo("⏸ 直行完成, 左转...")
                elif self.phase == 1:
                    turn_dur=math.radians(TURN_ANGLE_DEG)/TURN_SPEED
                    if elapsed < turn_dur:
                        t=Twist(); t.angular.z=TURN_SPEED; self.pub_cmd.publish(t)
                    else:
                        self.phase=2; self.phase_time=rospy.Time.now()
                        rospy.loginfo("↩ 左转完成, 直行过渡2s...")
                elif self.phase == 2:
                    if elapsed < 3.5:
                        t=Twist(); t.linear.x=0.16; self.pub_cmd.publish(t)
                    else:
                        self.phase=3; self.phase_time=None
                        self.last_error=0.0; self.integral=0.0
                        rospy.loginfo("✅ 开启视觉巡线!")
                return  # 跳过视觉处理

            # === UP主管线 ===
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, GAUSSIAN, 0)
            canny = cv2.Canny(blur, CANNY_LO, CANNY_HI)
            ipm = cv2.warpPerspective(canny, IPM_M, (IPM_W, IPM_H))
            dilated = cv2.dilate(ipm, DILATE_K)
            morphed = cv2.erode(dilated, ERODE_K)
            roi = morphed[ROI_Y:ROI_Y+ROI_H, 0:IPM_W]
            _, binary = cv2.threshold(roi, 5, 255, cv2.THRESH_BINARY)
            rh, rw = binary.shape

            # === 白线检测(巡线阶段) ===
            if self.phase == 3 and not self.stop_detected:
                wx1 = max(0, rw//2 - STOP_WIN_W//2)
                wx2 = min(rw, wx1 + STOP_WIN_W)
                wy1 = max(0, rh - STOP_WIN_H)
                window = binary[wy1:rh, wx1:wx2]
                white_ratio = (window>0).sum()/window.size if window.size>0 else 0

                if white_ratio >= STOP_WHITE_THRESH:
                    self.detect_consecutive += 1
                    rospy.loginfo(f"🛑 停车区{white_ratio*100:.0f}% [{self.detect_consecutive}/{STOP_CONSECUTIVE}]")
                else:
                    self.detect_consecutive = max(0, self.detect_consecutive-1)

                if self.detect_consecutive >= STOP_CONSECUTIVE:
                    self.stop_detected = True
                    self.stop_enter_time = rospy.Time.now()
                    rospy.loginfo(f"🛑 检测到停车区! 直行{STOP_DELAY}秒...")

            # 直行停车
            if self.stop_detected and not self.is_stopped:
                if (rospy.Time.now()-self.stop_enter_time).to_sec() < STOP_DELAY:
                    twist = Twist()
                    twist.linear.x = BASE_SPEED
                    self.pub_cmd.publish(twist)
                    dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                    cv2.putText(dbg, f"STOP STRAIGHT {STOP_DELAY-(rospy.Time.now()-self.stop_enter_time).to_sec():.1f}s",
                               (5,12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
                    dbg = dbg.astype(np.uint8)
                    self._pub_dbg(dbg, msg)
                    return
                else:
                    self.is_stopped = True
                    rospy.loginfo("🅿 停车!")

            # 最终停车
            if self.is_stopped:
                twist = Twist()
                self.pub_cmd.publish(twist)
                dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg, "PARKED", (rw//2-30, rh//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                dbg = dbg.astype(np.uint8)
                self._pub_dbg(dbg, msg)
                return

            # === 自适应找起点 ===
            srow=None; Ls=Rs=None
            # 第一轮: 从底部向上扫
            for row in range(rh-5, 20, -1):
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
            # 第二轮: 底部没找到 → 从画面中间开始找
            if Ls is None or Rs is None:
                mid_row = rh // 2
                for row in range(mid_row, 20, -1):
                    xs=np.where(binary[row,:]==255)[0]
                    if len(xs)<15: continue
                    lx,rx=xs[0],xs[-1]
                    if rx-lx<20: continue
                    Ls=(lx,row); Rs=(rx,row); srow=row; break

            twist=Twist()
            error=0.0

            # === 追踪 + 左右墙判断 ===
            if Ls and Rs:
                if getattr(self,'lost_frames',0)>=30:
                    rospy.loginfo("✅ 重新找到线!")
                self.lost_frames=0
                Lp,Ld=self.trace(binary,Ls,SEEDS_R)
                Rp,Rd=self.trace(binary,Rs,SEEDS_L)

                lb=np.full(rh,-1,dtype=int); rb=np.full(rh,-1,dtype=int)
                for x,y in Lp:
                    if 0<=y<rh and lb[y]==-1: lb[y]=x
                for x,y in Rp:
                    if 0<=y<rh and rb[y]==-1: rb[y]=x

                # 统计底部有效行(最近30行)
                bottom_zone = rh-30
                nL_bottom = (lb[bottom_zone:]!=-1).sum()
                nR_bottom = (rb[bottom_zone:]!=-1).sum()

                # 全局有效行
                nL_all = (lb!=-1).sum()
                nR_all = (rb!=-1).sum()

                # === 胡萝卜点: 从底部向上LOOKAHEAD行取中线 ===
                lookahead_y = max(0, rh - LOOKAHEAD)
                lookahead_zone = slice(lookahead_y, rh)

                # 动态更新半路宽(双线都在时)
                if nL_bottom>5 and nR_bottom>5:
                    valid_l = lb[lookahead_zone][lb[lookahead_zone]!=-1]
                    valid_r = rb[lookahead_zone][rb[lookahead_zone]!=-1]
                    if len(valid_l)>3 and len(valid_r)>3:
                        self.road_half = (valid_r.mean() - valid_l.mean()) // 2
                        self.road_half = max(20, min(60, self.road_half))  # 上限60防提前转弯

                # === 一条线两边缘误判检测 ===
                same_line = False
                if nL_all > 10 and nR_all > 10:
                    valid_l = lb[lookahead_zone][lb[lookahead_zone]!=-1]
                    valid_r = rb[lookahead_zone][rb[lookahead_zone]!=-1]
                    if len(valid_l)>3 and len(valid_r)>3:
                        if (valid_r.mean() - valid_l.mean()) < 40:
                            same_line = True  # 同一线两边缘!

                # === 左右墙切换 + 胡萝卜点误差(降阈值防HOLD) ===
                if nL_all > 10 and nR_all > 10 and not same_line:
                    # 真双线 → 中线巡线
                    cl=np.full(rh,-1,dtype=int)
                    for y in range(rh):
                        if lb[y]!=-1 and rb[y]!=-1: cl[y]=(lb[y]+rb[y])//2
                    lookahead_centers = cl[lookahead_zone][cl[lookahead_zone]!=-1]
                    if len(lookahead_centers)>3:
                        error=rw//2-lookahead_centers.mean()
                    else:
                        valid_all=cl[cl!=-1]
                        error=rw//2-valid_all.mean() if len(valid_all)>10 else self.last_error
                    self.mode="CENTER"

                elif (nL_all > 10 and nR_all > 10 and same_line) or nL_all > 10:
                    # 误判双线 或 只有左线 → 左墙巡线
                    lookahead_l = lb[lookahead_zone][lb[lookahead_zone]!=-1]
                    if len(lookahead_l)>3:
                        est_center=lookahead_l.mean()+self.road_half
                        error=rw//2-est_center
                    else:
                        error=self.last_error
                    self.mode="SAME_LINE" if same_line else "LEFT_WALL"

                elif nR_all > 10:
                    # 只有右线 → 右墙巡线
                    lookahead_r = rb[lookahead_zone][rb[lookahead_zone]!=-1]
                    if len(lookahead_r)>3:
                        est_center=lookahead_r.mean()-self.road_half
                        error=rw//2-est_center
                    else:
                        error=self.last_error
                    self.mode="RIGHT_WALL"

                else:
                    # 丢线 → 原地左转找线(首次丢线转30°, 之后保持)
                    if not hasattr(self,'lost_frames'): self.lost_frames=0
                    self.lost_frames+=1
                    if self.lost_frames < 30:  # 30帧≈30度
                        if self.lost_frames==1:
                            rospy.logwarn("⚠️ 丢线! 逆时针旋转30°搜索...")
                        twist=Twist()
                        twist.angular.z=0.52  # 0.52rad/s×1s≈30°
                        self.pub_cmd.publish(twist)
                        dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                        cv2.putText(dbg,f"LOST TURN LEFT {self.lost_frames}/30",
                                   (5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)
                        dbg=dbg.astype(np.uint8); self._pub_dbg(dbg,msg)
                        return
                    error=self.last_error
                    self.mode="HOLD"

                # === 二拐检测: 方向6≥10/30帧 ===
                if self.phase>=3 and not self.turn2_done and not self.is_stopped:
                    Lf = self.detect_cross(Ld)
                    Rf = self.detect_cross(Rd)
                    if self.fc%30==0: rospy.loginfo(f"🔍 二拐 L6={Ld.count(6)} R6={Rd.count(6)}")
                    if Lf or Rf:
                        self.turn2_consec+=1
                        if self.turn2_consec>=3:
                            rospy.loginfo("🔍 二拐确认! 直行3.5s后转...")
                            self.turn2_detected=rospy.Time.now()
                            self.turn2_consec=0
                    else: self.turn2_consec=max(0,self.turn2_consec-1)
                if self.turn2_detected:
                    el=(rospy.Time.now()-self.turn2_detected).to_sec()
                    if el<3.5: self.mode="WAIT_TURN"
                    else:
                        rospy.loginfo("↩ 二拐左转55°!")
                        td=math.radians(55)/TURN_SPEED; t0=rospy.Time.now()
                        while (rospy.Time.now()-t0).to_sec()<td:
                            t=Twist(); t.angular.z=TURN_SPEED; self.pub_cmd.publish(t); rospy.sleep(0.05)
                        self.turn2_done=True; self.last_error=0.0; self.integral=0.0
                        self.turn2_detected=None
                        rospy.loginfo("✅ 二拐完成!")

                # === PID(加死区) ===
                if abs(error) < DEADZONE:
                    error = 0.0  # 死区: 小误差不转, 减少直道摆动

                self.integral += error
                self.integral = max(-50, min(50, self.integral))  # 积分限幅
                ang = KP*error + KI*self.integral + KD*(error-self.last_error)
                self.last_error = error

                ang=max(-MAX_ANGULAR,min(MAX_ANGULAR,ang))

                # 速度
                cov=(lb!=-1).sum()/rh+(rb!=-1).sum()/rh
                twist.linear.x=BASE_SPEED if cov>0.5 else BASE_SPEED*0.7
                twist.angular.z=ang

                # 调试画面
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                if srow: cv2.line(dbg,(0,srow),(rw,srow),(255,255,0),1)
                for x,y in Lp: cv2.circle(dbg,(x,y),1,(0,255,0),-1)
                for x,y in Rp: cv2.circle(dbg,(x,y),1,(0,0,255),-1)
                # 画中线或估计中线
                for y in range(rh):
                    if lb[y]!=-1 and rb[y]!=-1:
                        cv2.circle(dbg,((lb[y]+rb[y])//2,y),1,(255,255,0),-1)
                    elif lb[y]!=-1 and self.mode=="LEFT_WALL":
                        cv2.circle(dbg,(int(lb[y]+self.road_half),y),1,(0,255,255),-1)
                    elif rb[y]!=-1 and self.mode=="RIGHT_WALL":
                        cv2.circle(dbg,(int(rb[y]-self.road_half),y),1,(0,255,255),-1)

                ph = f"P{self.phase}" if self.phase<3 else ""
                det_s = f" DET={self.detect_consecutive}" if self.detect_consecutive>0 else ""
                info=(f"V12P F{self.fc} {self.mode} e={error:.0f} "
                      f"hw={self.road_half}{det_s}")
                cv2.putText(dbg,info,(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
            else:
                twist.linear.x=BASE_SPEED*0.25; twist.angular.z=0
                self.mode="LOST"
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,f"V12P F{self.fc} LOST",(5,12),
                           cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)

            self.pub_cmd.publish(twist)

            dbg=dbg.astype(np.uint8)
            dbg_msg=Image()
            dbg_msg.header.stamp=msg.header.stamp
            dbg_msg.height=dbg.shape[0]; dbg_msg.width=dbg.shape[1]
            dbg_msg.encoding="bgr8"; dbg_msg.is_bigendian=False
            dbg_msg.step=dbg.shape[1]*3; dbg_msg.data=dbg.tobytes()
            self.pub_dbg.publish(dbg_msg)

            if self.fc%30==0:
                rospy.loginfo(f"V12P F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}")

        except Exception as e:
            import traceback; rospy.logerr(f"V12P: {e}\n{traceback.format_exc()}")


if __name__=="__main__":
    rospy.init_node("line_follower_v12_pro")
    LineFollowerV12Pro()
    rospy.spin()
