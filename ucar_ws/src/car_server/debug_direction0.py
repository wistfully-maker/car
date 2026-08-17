#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""八邻域方向0(↓)调试 — 实时显示方向0占比, 不控制小车

用法: rosrun car_server debug_direction0.py
查看: rqt_image_view /car_server/debug_image
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

bridge = CvBridge()
fc = 0
dbg_pub = None

def preprocess(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edge = cv2.Canny(blur, 50, 150)
    k = np.ones((3,3), np.uint8)
    d1 = cv2.dilate(edge, k, iterations=2); e1 = cv2.erode(d1, k, iterations=1)
    d2 = cv2.dilate(e1, k, iterations=1); cl = cv2.morphologyEx(d2, cv2.MORPH_CLOSE, k)
    _, bi = cv2.threshold(cl, 1, 255, cv2.THRESH_BINARY)
    return bi

def trace(img_b, start, seeds):
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

def callback(msg):
    global fc, dbg_pub
    try:
        frame = cv2.flip(bridge.imgmsg_to_cv2(msg,"bgr8"), 1)
        small = cv2.resize(frame, (160, 120))
        mask = preprocess(small)
        fc += 1

        # 八邻域追踪
        rh,rw=mask.shape; Ls=Rs=None
        for row in range(rh-5,20,-1):
            xs=np.where(mask[row,:]==255)[0]
            if len(xs)<15: continue
            lx,rx=xs[0],xs[-1]
            if rx-lx<20: continue
            for x in range(max(1,lx-5),min(rw-1,lx+5)):
                if mask[row,x]==0 and mask[row,x-1]==255: Ls=(x-1,row); break
            if Ls is None: Ls=(lx,row)
            for x in range(max(1,rx-5),min(rw-1,rx+5)):
                if mask[row,x]==0 and mask[row,x+1]==255: Rs=(x+1,row); break
            if Rs is None: Rs=(rx,row)
            break

        Ld=[]; Rd=[]
        if Ls and Rs:
            Lp,Ld = trace(mask, Ls, SEEDS_R)
            Rp,Rd = trace(mask, Rs, SEEDS_L)

        # 方向0占比
        Lr = Ld[-30:] if len(Ld)>=30 else Ld
        Rr = Rd[-30:] if len(Rd)>=30 else Rd
        l0 = Lr.count(0)/len(Lr) if Lr else 0
        r0 = Rr.count(0)/len(Rr) if Rr else 0

        # 终端输出
        if fc % 15 == 0:
            bar_l = "█"*int(l0*20); bar_r = "█"*int(r0*20)
            trig = " ⚡TRIGGER!" if (l0>0.4 or r0>0.4) else ""
            print(f"[F{fc:4d}] L0%={l0*100:3.0f}% {bar_l} | R0%={r0*100:3.0f}% {bar_r}{trig}")

        # 调试画面(640×480)
        dbg = cv2.resize(mask, (640, 480))
        dbg = cv2.cvtColor(dbg, cv2.COLOR_GRAY2BGR)

        # 画追踪点
        dir_colors = {0:(0,0,255), 1:(100,100,100), 2:(100,100,100),
                      3:(100,100,100), 4:(100,100,100), 5:(100,100,100),
                      6:(0,255,0), 7:(100,100,100)}
        if Ls and Rs:
            for pt in Lp:
                if len(pt)>=2: cv2.circle(dbg,(pt[0]*4,pt[1]*4),2,(0,255,0),-1)
            for pt in Rp:
                if len(pt)>=2: cv2.circle(dbg,(pt[0]*4,pt[1]*4),2,(255,0,0),-1)

        # 方向0高亮(红色大点)
        if Ld:
            for i, pt in enumerate(Lp[1:], 1):
                if i-1 < len(Ld) and Ld[i-1]==0:
                    cv2.circle(dbg,(pt[0]*4,pt[1]*4),4,(0,0,255),-1)
        if Rd:
            for i, pt in enumerate(Rp[1:], 1):
                if i-1 < len(Rd) and Rd[i-1]==0:
                    cv2.circle(dbg,(pt[0]*4,pt[1]*4),4,(0,0,255),-1)

        # 图例 + 数据
        cv2.putText(dbg, f"F{fc} L0%={l0*100:.0f} R0%={r0*100:.0f}", (5,15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
        cv2.putText(dbg, "GREEN:L RED:R  BIG_RED=dir0(↓)", (5,35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
        if l0>0.4 or r0>0.4:
            cv2.putText(dbg, "CORNER!", (5,55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # ROI框
        cv2.rectangle(dbg,(0,76*4),(640,105*4),(0,255,0),1)

        # 阈值线
        bar_y = 460
        cv2.line(dbg, (0,bar_y), (640,bar_y), (100,100,100), 1)
        cv2.rectangle(dbg, (0,bar_y), (int(l0*640), 480), (0,0,200), -1)
        cv2.putText(dbg, "40%", (int(0.4*640), bar_y-3),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)

        dm = Image(); dm.header.stamp=msg.header.stamp
        dm.height=480; dm.width=640; dm.encoding="bgr8"
        dm.is_bigendian=False; dm.step=640*3; dm.data=dbg.astype(np.uint8).tobytes()
        dbg_pub.publish(dm)

    except Exception as e:
        import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")

rospy.init_node("debug_direction0")
dbg_pub = rospy.Publisher('/car_server/debug_image', Image, queue_size=1)
rospy.Subscriber("/usb_cam/image_raw", Image, callback, queue_size=1)
rospy.loginfo("✅ 方向0调试启动")
rospy.loginfo("   终端: 每15帧输出L0%/R0%")
rospy.loginfo("   画面: rqt_image_view /car_server/debug_image")
rospy.loginfo("   红色大点=方向0(↓) 绿色点=L边界 蓝色点=R边界")
rospy.spin()
