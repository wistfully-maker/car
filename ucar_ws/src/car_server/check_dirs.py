"""检查八邻域追踪方向: 近处 vs 远处方向0占比"""
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

bridge = CvBridge()
rospy.init_node('check', anonymous=True)
msg = rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=5)
frame = cv2.flip(bridge.imgmsg_to_cv2(msg,'bgr8'), 1)
small = cv2.resize(frame, (160,120))
gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray,(5,5),0)
edge = cv2.Canny(blur,50,150)
k = np.ones((3,3),np.uint8)
d1=cv2.dilate(edge,k,iterations=2); e1=cv2.erode(d1,k,iterations=1)
d2=cv2.dilate(e1,k,iterations=1); cl=cv2.morphologyEx(d2,cv2.MORPH_CLOSE,k)
_,bi=cv2.threshold(cl,1,255,cv2.THRESH_BINARY)

rh,rw=bi.shape; Ls=Rs=None
for row in range(rh-5,20,-1):
    xs=np.where(bi[row,:]==255)[0]
    if len(xs)<15: continue
    lx,rx=xs[0],xs[-1]
    if rx-lx<20: continue
    for x in range(max(1,lx-5),min(rw-1,lx+5)):
        if bi[row,x]==0 and bi[row,x-1]==255: Ls=(x-1,row); break
    if Ls is None:Ls=(lx,row)
    for x in range(max(1,rx-5),min(rw-1,rx+5)):
        if bi[row,x]==0 and bi[row,x+1]==255: Rs=(x+1,row); break
    if Rs is None:Rs=(rx,row)
    break

def trace(img_b,start,seeds):
    pts=[];dirs=[];cx,cy=start;pts.append((cx,cy))
    visited=set()
    for _ in range(400):
        if (cx,cy) in visited:break
        visited.add((cx,cy))
        cand=[]
        for i in range(8):
            da=seeds[i];ax=cx+da[0];ay=cy+da[1]
            db=seeds[(i+1)%8];bx=cx+db[0];by=cy+db[1]
            if 0<=ax<img_b.shape[1] and 0<=ay<img_b.shape[0] and 0<=bx<img_b.shape[1] and 0<=by<img_b.shape[0]:
                if img_b[ay,ax]==0 and img_b[by,bx]==255:cand.append((ax,ay,i))
        if not cand:break
        best=min(cand,key=lambda p:p[1])
        cx,cy=best[0],best[1];pts.append((cx,cy));dirs.append(best[2])
    return pts,dirs

if Ls and Rs:
    Lp,Ld=trace(bi,Ls,SEEDS_R); Rp,Rd=trace(bi,Rs,SEEDS_L)
    n = min(len(Ld), len(Lp)-1)
    print(f"起点: L({Ls[0]},{Ls[1]}) R({Rs[0]},{Rs[1]})")
    print(f"总步数: L={len(Ld)} R={len(Rd)}")

    # 近处: 前10步(dirs[0:10])
    # 远处: 后10步(dirs[-10:])
    if n >= 20:
        near = Ld[:10]; far = Ld[-10:]
        print(f"前10步(近) y范围: {[Lp[i+1][1] for i in range(min(10,n))]}")
        print(f"后10步(远) y范围: {[Lp[i+1][1] for i in range(max(n-10,0),n)]}")
        print(f"前10步方向0(近): {near.count(0)}/{len(near)}={near.count(0)/len(near)*100:.0f}%")
        print(f"后10步方向0(远): {far.count(0)}/{len(far)}={far.count(0)/len(far)*100:.0f}%")
        print(f"结论: {'近处方向0高 → 拐角就在眼前' if near.count(0)>far.count(0) else '远处方向0高 → 拐角还在远处'}")
    else:
        print(f"步数不足20, Ld={Ld}")
else:
    print("未找到起点")
