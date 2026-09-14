#!/usr/bin/env python3
"""
ROS巡线 V5 — Canny + 八邻域追踪版
管线: 底部40%ROI → Canny → 闭运算 → 八邻域 → 中线 → PID

发布: /cmd_vel, /car_server/debug_image
"""

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# === 参数 ===
CUT_RATIO = 0.60
GAUSSIAN = (5,5)
CANNY_LO, CANNY_HI = 50, 150
CLOSE_K = np.ones((5,5), np.uint8)
MIN_TRACK_W = 80

SEEDS_L = [(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1)]
SEEDS_R = [(0,1),(1,1),(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1)]

KP, KD = 0.008, 0.001
BASE_SPEED, MAX_ANGULAR = 0.07, 0.5


class LineFollowerV5:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0
        self.fc = 0
        self.road_half = 40
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V5 Canny+八邻域 启动")

    def trace(self, image, start, seeds):
        pts=[]; cx,cy=start; pts.append((cx,cy))
        visited=set()
        for _ in range(500):
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
            y0 = int(h*CUT_RATIO)
            roi = frame[y0:h,:]
            rh,rw = roi.shape[:2]

            # Canny
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, GAUSSIAN, 0)
            canny = cv2.Canny(blur, CANNY_LO, CANNY_HI)
            binary = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, CLOSE_K)

            # 找起点
            srow=None; Ls=Rs=None
            for row in range(rh-10,20,-1):
                xs=np.where(binary[row,:]==255)[0]
                if len(xs)<15: continue
                lx,rx=xs[0],xs[-1]
                if rx-lx<MIN_TRACK_W: continue
                for x in range(max(1,lx-3),min(rw-1,lx+3)):
                    if binary[row,x]==0 and binary[row,x-1]==255: Ls=(x-1,row); break
                if Ls is None: Ls=(lx,row)
                for x in range(max(1,rx-3),min(rw-1,rx+3)):
                    if binary[row,x]==0 and binary[row,x+1]==255: Rs=(x+1,row); break
                if Rs is None: Rs=(rx,row)
                srow=row; break

            twist=Twist()
            error=0.0; mode="LOST"

            if Ls and Rs:
                Lp=self.trace(binary,Ls,SEEDS_L)
                Rp=self.trace(binary,Rs,SEEDS_R)

                lb=np.full(rh,-1,dtype=int)
                rb=np.full(rh,-1,dtype=int)
                for x,y in Lp:
                    if 0<=y<rh and lb[y]==-1: lb[y]=x
                for x,y in Rp:
                    if 0<=y<rh and rb[y]==-1: rb[y]=x

                cl=np.full(rh,-1,dtype=int)
                for y in range(rh):
                    if lb[y]!=-1 and rb[y]!=-1:
                        cl[y]=(lb[y]+rb[y])//2

                # 误差: 底部有效中线的平均值
                valid=cl[-30:][cl[-30:]!=-1]
                if len(valid)>5:
                    error=rw//2-valid.mean()
                    mode="OK"
                else:
                    valid_all=cl[cl!=-1]
                    if len(valid_all)>10:
                        error=rw//2-valid_all.mean()
                        mode="EST"
                    else:
                        error=self.last_error; mode="HOLD"

                ang=KP*error+KD*(error-self.last_error)
                self.last_error=error
                ang=max(-MAX_ANGULAR,min(MAX_ANGULAR,ang))

                cov=(cl!=-1).sum()/rh
                twist.linear.x=BASE_SPEED  # 先全速测试，确认车能动后再恢复分档
                twist.angular.z=ang

                # 调试画面
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                if srow: cv2.line(dbg,(0,srow),(rw,srow),(255,255,0),1)
                for x,y in Lp: cv2.circle(dbg,(x,y),1,(0,255,0),-1)
                for x,y in Rp: cv2.circle(dbg,(x,y),1,(0,0,255),-1)
                for y in range(rh):
                    if cl[y]!=-1: cv2.circle(dbg,(cl[y],y),1,(255,255,0),-1)
                info=f"V5 F{self.fc} {mode} e={error:.0f} cov={cov*100:.0f}% L={len(Lp)} R={len(Rp)}"
                cv2.putText(dbg,info,(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
            else:
                twist.linear.x=BASE_SPEED*0.3; twist.angular.z=0
                dbg=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg,f"V5 F{self.fc} LOST",(5,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)

            self.pub_cmd.publish(twist)
            dbg=dbg.astype(np.uint8)
            dbg_msg=Image()
            dbg_msg.header.stamp=msg.header.stamp
            dbg_msg.height=dbg.shape[0]
            dbg_msg.width=dbg.shape[1]
            dbg_msg.encoding="bgr8"
            dbg_msg.is_bigendian=False
            dbg_msg.step=dbg.shape[1]*3
            dbg_msg.data=dbg.tobytes()
            self.pub_dbg.publish(dbg_msg)

            if self.fc%30==0: rospy.loginfo(f"V5 F{self.fc} {mode} e={error:.0f}")

        except Exception as e:
            import traceback; rospy.logerr(f"V5: {e}\n{traceback.format_exc()}")


if __name__=="__main__":
    rospy.init_node("line_follower_v5")
    LineFollowerV5()
    rospy.spin()
