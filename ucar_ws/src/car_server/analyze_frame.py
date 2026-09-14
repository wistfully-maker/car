#!/usr/bin/env python3
"""抓一帧 → 八邻域 + 凸包分析"""
import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('analyze_frame', anonymous=True)
bridge = CvBridge()
msg = rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=10)
frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
frame = cv2.flip(frame, 1)

# === follow.py 管线 ===
small = cv2.resize(frame, (160, 120))
gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 0)
edge = cv2.Canny(blur, 50, 150)
k = np.ones((3, 3), np.uint8)
d1 = cv2.dilate(edge, k, iterations=2)
e1 = cv2.erode(d1, k, iterations=1)
d2 = cv2.dilate(e1, k, iterations=1)
cl = cv2.morphologyEx(d2, cv2.MORPH_CLOSE, k)
_, bi = cv2.threshold(cl, 1, 255, cv2.THRESH_BINARY)

# === 八邻域追踪 ===
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

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
            if 0<=ax<bi.shape[1] and 0<=ay<bi.shape[0] and 0<=bx<bi.shape[1] and 0<=by<bi.shape[0]:
                if bi[ay,ax]==0 and bi[by,bx]==255: cand.append((ax,ay,i))
        if not cand: break
        best=min(cand,key=lambda p:p[1])
        cx,cy=best[0],best[1]; pts.append((cx,cy)); dirs.append(best[2])
    return pts, dirs

rh,rw=bi.shape; Ls=Rs=None
for row in range(rh-5,20,-1):
    xs=np.where(bi[row,:]==255)[0]
    if len(xs)<15: continue
    lx,rx=xs[0],xs[-1]
    if rx-lx<20: continue
    for x in range(max(1,lx-5),min(rw-1,lx+5)):
        if bi[row,x]==0 and bi[row,x-1]==255: Ls=(x-1,row); break
    if Ls is None: Ls=(lx,row)
    for x in range(max(1,rx-5),min(rw-1,rx+5)):
        if bi[row,x]==0 and bi[row,x+1]==255: Rs=(x+1,row); break
    if Rs is None: Rs=(rx,row)
    break

def c(d,k): return d.count(k)
print("=" * 70)
print("[八邻域]")
if Ls and Rs:
    _,Ld=trace(bi,Ls,SEEDS_R); _,Rd=trace(bi,Rs,SEEDS_L)
    Lr=Ld[-50:] if len(Ld)>=50 else Ld; Rr=Rd[-50:] if len(Rd)>=50 else Rd
    l6p=c(Lr,6)/len(Lr)*100 if Lr else 0; r0p=c(Rr,0)/len(Rr)*100 if Rr else 0
    print(f"  L总={len(Ld):3d}  2={c(Ld,2):3d} 4={c(Ld,4):2d} 6={c(Ld,6):2d}")
    print(f"  R总={len(Rd):3d}  0={c(Rd,0):2d} 6={c(Rd,6):2d}")
    print(f"  近50: L6%={l6p:.0f}% R0%={r0p:.0f}%")
else:
    print("  未找到起点")

print("[凸包]")
f2=cv2.medianBlur(bi,3)
res2=cv2.findContours(f2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
cnts=res2[0] if len(res2)==2 else res2[1]
print(f"  轮廓总数={len(cnts)}")
for ci,cc in enumerate(cnts):
    if len(cc)>=6:
        hull=cv2.convexHull(cc,returnPoints=False)
        if hull is not None and len(hull)>=3:
            defe=cv2.convexityDefects(cc,hull)
            if defe is not None:
                dirs={i:0 for i in range(8)}
                for i in range(defe.shape[0]):
                    flat=defe[i].flatten(); f_idx=int(flat[2])
                    p=tuple(cc[f_idx][0])
                    dx,dy=p[0]-80,p[1]-60
                    code=int(((math.degrees(math.atan2(dy,dx))+180)/45.0)%8)
                    dirs[code]+=1
                print(f"  轮廓{ci}: {len(cc)}pts defects={defe.shape[0]} dirs={dirs}")
print("=" * 70)
