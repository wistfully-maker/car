#!/usr/bin/env python3
"""
ROS巡线 V10 — V10 + 边界间横线检测
管线: Canny→IPM(dilate15+erode7)→八邻域→中线/墙线→PID

停车逻辑(边界间填充法):
  1. 对每行, 计算lb[y]~rb[y]之间的白像素占比
  2. 连续>8行填充率>60% → 确认停车横线(连接左右边界的垂直线段)
  3. 直行STOP_DELAY秒后 → 停车
"""

import rospy, cv2, numpy as np
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
BASE_SPEED, MAX_ANGULAR = 0.07, 0.5
DEADZONE = 15            # 误差<15像素不转(死区)
LOOKAHEAD = 10           # 胡萝卜点: 从底部向上看N行
STOP_FILL_RATIO = 0.60    # 边界间填充率>60%
STOP_MIN_ROWS  = 8        # 连续N行填充率>阈值
STOP_CONSECUTIVE = 3      # 连续N帧确认
STOP_DELAY = 2.0          # 确认后直行N秒(实测后调)


class LineFollowerV10Stop:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.integral = 0.0
        self.road_half = 60
        self.fc = 0
        self.mode = "INIT"
        # 停车状态
        self.stop_detected = False      # 是否检测到停车线
        self.stop_consecutive = 0       # 连续检测计数
        self.stop_enter_time = None     # 进入停车区时间
        self.is_stopped = False         # 是否已完成停车

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V10 边界间横线检测 启动")

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
            self.fc += 1
            h,w = frame.shape[:2]

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

            # === 自适应找起点 ===
            srow=None; Ls=Rs=None
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

            twist=Twist()
            error=0.0

            # === 追踪 + 左右墙判断 ===
            if Ls and Rs:
                Lp=self.trace(binary,Ls,SEEDS_R)
                Rp=self.trace(binary,Rs,SEEDS_L)

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
                        self.road_half = max(20, min(200, self.road_half))

                # === 一条线两边缘误判检测 ===
                same_line = False
                if nL_all > 10 and nR_all > 10:
                    valid_l = lb[lookahead_zone][lb[lookahead_zone]!=-1]
                    valid_r = rb[lookahead_zone][rb[lookahead_zone]!=-1]
                    if len(valid_l)>3 and len(valid_r)>3:
                        if (valid_r.mean() - valid_l.mean()) < 40:
                            same_line = True  # 同一线两边缘!

                # === 左右墙切换 + 胡萝卜点误差 ===
                if nL_all > 20 and nR_all > 20 and not same_line:
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

                elif (nL_all > 20 and nR_all > 20 and same_line) or nL_all > 20:
                    # 误判双线 或 只有左线 → 左墙巡线
                    lookahead_l = lb[lookahead_zone][lb[lookahead_zone]!=-1]
                    if len(lookahead_l)>3:
                        est_center=lookahead_l.mean()+self.road_half
                        error=rw//2-est_center
                    else:
                        error=self.last_error
                    self.mode="SAME_LINE" if same_line else "LEFT_WALL"

                elif nR_all > 20:
                    # 只有右线 → 右墙巡线
                    lookahead_r = rb[lookahead_zone][rb[lookahead_zone]!=-1]
                    if len(lookahead_r)>3:
                        est_center=lookahead_r.mean()-self.road_half
                        error=rw//2-est_center
                    else:
                        error=self.last_error
                    self.mode="RIGHT_WALL"

                else:
                    error=self.last_error
                    self.mode="HOLD"

                # === 停车横线检测(边界间填充法, lb/rb已就绪) ===
                if not self.is_stopped:
                    detected = False
                    fill_rows = 0
                    for y in range(rh-1, 0, -1):
                        if lb[y] != -1 and rb[y] != -1:
                            between = binary[y, lb[y]:rb[y]+1]
                            if len(between) > 30:
                                fill = (between>0).sum() / len(between)
                                if fill > STOP_FILL_RATIO:
                                    fill_rows += 1
                                    if fill_rows >= STOP_MIN_ROWS:
                                        detected = True
                                        break
                                else:
                                    fill_rows = 0

                    if detected:
                        self.stop_consecutive += 1
                    else:
                        self.stop_consecutive = max(0, self.stop_consecutive - 1)

                    if self.stop_consecutive >= STOP_CONSECUTIVE and not self.stop_detected:
                        self.stop_detected = True
                        self.stop_enter_time = rospy.Time.now()
                        rospy.loginfo("🛑 检测到停车线! 直行进入...")

                # 停车后处理
                if self.stop_detected and not self.is_stopped:
                    if (rospy.Time.now()-self.stop_enter_time).to_sec() > STOP_DELAY:
                        self.is_stopped = True
                        rospy.loginfo("🅿 停车!")

                if self.is_stopped:
                    twist = Twist()
                    self.pub_cmd.publish(twist)
                    dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                    cv2.putText(dbg, "PARKED", (rw//2-30, rh//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                    dbg = dbg.astype(np.uint8)
                    dbg_msg = Image()
                    dbg_msg.header.stamp = msg.header.stamp
                    dbg_msg.height = dbg.shape[0]; dbg_msg.width = dbg.shape[1]
                    dbg_msg.encoding = "bgr8"; dbg_msg.is_bigendian = False
                    dbg_msg.step = dbg.shape[1]*3; dbg_msg.data = dbg.tobytes()
                    self.pub_dbg.publish(dbg_msg)
                    return

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

                stop_info = f" STOP={self.stop_consecutive}" if self.stop_consecutive>0 else ""
                info=(f"V10 F{self.fc} {self.mode} e={error:.0f} "
                      f"hw={self.road_half} L={nL_all} R={nR_all}{stop_info}")
                cv2.putText(dbg,info,(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
            else:
                twist.linear.x=BASE_SPEED*0.25; twist.angular.z=0
                self.mode="LOST"
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,f"V10 F{self.fc} LOST",(5,12),
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
                rospy.loginfo(f"V10 F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}")

        except Exception as e:
            import traceback; rospy.logerr(f"V10: {e}\n{traceback.format_exc()}")


if __name__=="__main__":
    rospy.init_node("line_follower_v10")
    LineFollowerV10Stop()
    rospy.spin()
