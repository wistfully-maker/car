#!/usr/bin/env python3
"""八邻域方向检测 — 只输出方向统计，不下发控制指令"""

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

INVERSE_IPM = np.array([[-3.365493,2.608984,-357.317062],[-0.049261,1.302389,-874.095796],[0.000029,0.007556,-4.205510]], dtype=np.float32)
IPM_M = np.linalg.inv(INVERSE_IPM)
GAUSSIAN = (5,5); CANNY_LO,CANNY_HI = 50,150
DILATE_K = np.ones((15,15),np.uint8); ERODE_K = np.ones((7,7),np.uint8)
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]

bridge = CvBridge(); fc = 0

def trace(img, start, seeds):
    pts=[]; dirs=[]; cx,cy=start; pts.append((cx,cy))
    visited=set()
    for _ in range(400):
        if (cx,cy) in visited: break
        visited.add((cx,cy))
        cand=[]
        for i in range(8):
            da=seeds[i]; ax=cx+da[0]; ay=cy+da[1]
            db=seeds[(i+1)%8]; bx=cx+db[0]; by=cy+db[1]
            if 0<=ax<img.shape[1] and 0<=ay<img.shape[0] and 0<=bx<img.shape[1] and 0<=by<img.shape[0]:
                if img[ay,ax]==0 and img[by,bx]==255: cand.append((ax,ay,i))
        if not cand: break
        best=min(cand,key=lambda p:p[1])
        cx,cy=best[0],best[1]; pts.append((cx,cy)); dirs.append(best[2])
    return pts, dirs

def callback(msg):
    global fc
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        fc += 1; h,w = frame.shape[:2]

        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        blur=cv2.GaussianBlur(gray,GAUSSIAN,0)
        canny=cv2.Canny(blur,CANNY_LO,CANNY_HI)
        ipm=cv2.warpPerspective(canny,IPM_M,(640,480))
        dilated=cv2.dilate(ipm,DILATE_K); morphed=cv2.erode(dilated,ERODE_K)
        roi=morphed[240:480,0:640]
        _,binary=cv2.threshold(roi,5,255,cv2.THRESH_BINARY)
        rh,rw=binary.shape

        Ls=Rs=None
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
            break

        if Ls and Rs:
            Lp,Ld=trace(binary,Ls,SEEDS_R)
            Rp,Rd=trace(binary,Rs,SEEDS_L)
            if fc%15==0:
                # 全程统计
                Ld2=Ld.count(2); Ld6=Ld.count(6); Ld4=Ld.count(4)
                Rd0=Rd.count(0); Rd6=Rd.count(6)
                # 最近50帧
                Lr=Ld[-50:] if len(Ld)>=50 else Ld
                Rr=Rd[-50:] if len(Rd)>=50 else Rd
                pctL6=(Lr.count(4)+Lr.count(6))/len(Lr)*100 if Lr else 0
                pctR6=(Rr.count(0)+Rr.count(6))/len(Rr)*100 if Rr else 0
                rospy.loginfo(f"F{fc:4d} Lp={len(Lp):3d} Rp={len(Rp):3d} | "
                              f"L(2={Ld2:3d} 4={Ld4:2d} 6={Ld6:2d}) R(0={Rd0:2d} 6={Rd6:2d}) | "
                              f"最近50 L4+6={pctL6:.0f}% R0+6={pctR6:.0f}%")
        else:
            if fc%15==0: rospy.loginfo(f"F{fc:4d} 未找到起点")
    except Exception as e:
        rospy.logerr(f"ERR: {e}")

rospy.init_node("dir_analyzer")
rospy.Subscriber("/usb_cam/image_raw", Image, callback, queue_size=1)
rospy.loginfo("✅ 八邻域方向分析器启动, 每15帧输出方向统计")
rospy.spin()
