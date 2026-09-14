#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
凸包+八邻域 双检测调试 — 不控制小车, 只看输出
把车手动放在不同位置(直道/弯道/岔口), 终端对比两种检测方法

用法: rosrun car_server debug_convex.py
"""

import rospy, cv2, math, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# === follow.py 凸包检测器(原样) ===
class ContourCornerTracker:
    def __init__(self): pass

    def _find_corners(self, cnt, hi, lo):
        if len(cnt) < 6: return []
        hull = cv2.convexHull(cnt, returnPoints=False)
        if hull is None or len(hull) < 3: return []
        defe = cv2.convexityDefects(cnt, hull)
        if defe is None: return []
        pts = []
        for i in range(defe.shape[0]):
            flat = defe[i].flatten()
            f = int(flat[2])
            p = tuple(cnt[f][0])
            if hi <= p[1] <= lo: pts.append(p)
        return pts

    def process(self, mask, size=(160, 120), lo=87, hi=117):
        f = cv2.resize(mask, size)
        f = cv2.medianBlur(f, 3)
        if hi > lo: hi, lo = lo, hi
        res = cv2.findContours(f, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts = res[0] if len(res) == 2 else res[1]
        cand = []
        for c in cnts:
            pts = [tuple(p[0]) for p in c if hi <= p[0][1] <= lo]
            if len(pts) >= 3:
                cand.append((len(pts), float(np.mean([p[0] for p in pts])), c))
        if len(cand) < 2: return None
        cand.sort(key=lambda x: x[0], reverse=True)
        Lc = self._find_corners(cand[0][2], hi, lo)
        Rc = self._find_corners(cand[1][2], hi, lo)
        st = {'left_dirs': {i: 0 for i in range(8)}, 'right_dirs': {i: 0 for i in range(8)}}
        for side, pts in (('left', Lc), ('right', Rc)):
            for p in pts:
                dx = p[0] - size[0] // 2
                dy = p[1] - size[1] // 2
                code = int(((math.degrees(math.atan2(dy, dx)) + 180) / 45.0) % 8)
                st[side + '_dirs'][code] += 1
        return {'stats': st, 'n_contours': len(cnts), 'n_candidates': len(cand),
                'L_corners': len(Lc), 'R_corners': len(Rc),
                'L_contour': cand[0][2], 'R_contour': cand[1][2],
                'L_hull': cv2.convexHull(cand[0][2], returnPoints=True),
                'R_hull': cv2.convexHull(cand[1][2], returnPoints=True),
                'L_defects': Lc, 'R_defects': Rc}


# === 八邻域追踪(原样) ===
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]

def trace(img, start, seeds):
    pts = []; dirs = []; cx, cy = start; pts.append((cx, cy))
    visited = set()
    for _ in range(400):
        if (cx, cy) in visited: break
        visited.add((cx, cy))
        cand = []
        for i in range(8):
            da = seeds[i]; ax = cx + da[0]; ay = cy + da[1]
            db = seeds[(i + 1) % 8]; bx = cx + db[0]; by = cy + db[1]
            if 0 <= ax < img.shape[1] and 0 <= ay < img.shape[0] and \
               0 <= bx < img.shape[1] and 0 <= by < img.shape[0]:
                if img[ay, ax] == 0 and img[by, bx] == 255:
                    cand.append((ax, ay, i))
        if not cand: break
        best = min(cand, key=lambda p: p[1])
        cx, cy = best[0], best[1]; pts.append((cx, cy)); dirs.append(best[2])
    return pts, dirs


bridge = CvBridge()
tracker = ContourCornerTracker()
dbg_pub = None  # 在 init_node 后赋值
fc = 0

def callback(msg):
    global fc
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        fc += 1

        # === follow.py 管线(160×120) ===
        small = cv2.resize(frame, (160, 120))
        small = cv2.flip(small, 1)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 50, 150)
        k = np.ones((3, 3), np.uint8)
        d1 = cv2.dilate(edge, k, iterations=2)
        e1 = cv2.erode(d1, k, iterations=1)
        d2 = cv2.dilate(e1, k, iterations=1)
        cl = cv2.morphologyEx(d2, cv2.MORPH_CLOSE, k)
        _, bi = cv2.threshold(cl, 1, 255, cv2.THRESH_BINARY)

        # === 凸包检测 ===
        res = tracker.process(bi)

        # === 八邻域追踪(在同一张二值图上) ===
        rh, rw = bi.shape
        Ls = Rs = None
        for row in range(rh - 5, 20, -1):
            xs = np.where(bi[row, :] == 255)[0]
            if len(xs) < 15: continue
            lx, rx = xs[0], xs[-1]
            if rx - lx < 20: continue
            for x in range(max(1, lx - 5), min(rw - 1, lx + 5)):
                if bi[row, x] == 0 and bi[row, x - 1] == 255: Ls = (x - 1, row); break
            if Ls is None: Ls = (lx, row)
            for x in range(max(1, rx - 5), min(rw - 1, rx + 5)):
                if bi[row, x] == 0 and bi[row, x + 1] == 255: Rs = (x + 1, row); break
            if Rs is None: Rs = (rx, row)
            break

        Ld = []; Rd = []
        if Ls and Rs:
            _, Ld = trace(bi, Ls, SEEDS_R)
            _, Rd = trace(bi, Rs, SEEDS_L)

        # === 每15帧输出 ===
        if fc % 15 == 0:
            print("=" * 70)
            print(f"=== F{fc} ===")
            print("-" * 70)

            # 八邻域输出
            if Ld or Rd:
                def cnt(d, k): return d.count(k)
                Lr = Ld[-50:] if len(Ld) >= 50 else Ld
                Rr = Rd[-50:] if len(Rd) >= 50 else Rd
                l6p = cnt(Lr,6)/len(Lr)*100 if Lr else 0
                r0p = cnt(Rr,0)/len(Rr)*100 if Rr else 0
                print(f"[八邻域] L总={len(Ld):3d}  2→{cnt(Ld,2):3d} 4→{cnt(Ld,4):2d} 6→{cnt(Ld,6):2d} | "
                      f"R总={len(Rd):3d}  0→{cnt(Rd,0):2d} 6→{cnt(Rd,6):2d} | "
                      f"近50 L6%={l6p:.0f} R0%={r0p:.0f}")
            else:
                print("[八邻域] 未找到起点")

            # 凸包输出
            if res is not None:
                L = res['stats']['left_dirs']; R = res['stats']['right_dirs']
                print(f"[凸包]   cnts={res['n_contours']} cand={res['n_candidates']} "
                      f"L角点={res['L_corners']} R角点={res['R_corners']}")
                print(f"         L: 0={L[0]} 1={L[1]} 2={L[2]} 3={L[3]} 4={L[4]} 5={L[5]} 6={L[6]} 7={L[7]}")
                print(f"         R: 0={R[0]} 1={R[1]} 2={R[2]} 3={R[3]} 4={R[4]} 5={R[5]} 6={R[6]} 7={R[7]}")
                # 重点: 方向和3的计数
                if L[3] > 0 or R[3] > 0:
                    print(f"  ⚡ 方向3触发! L[3]={L[3]} R[3]={R[3]}")
                else:
                    # 哪个方向最多?
                    Lmax = max(L, key=L.get)
                    Rmax = max(R, key=R.get)
                    print(f"         L主要方向={Lmax}({L[Lmax]}) R主要方向={Rmax}({R[Rmax]})")
            else:
                print(f"[凸包]   未检测到 (轮廓<2 或 角点<3)")

            print("-" * 70)

        # === 调试画面(每帧发布) ===
        try:
            dbg = cv2.resize(bi, (640, 480))
            dbg = cv2.cvtColor(dbg, cv2.COLOR_GRAY2BGR)

            if res is not None:
                for cnt, color in [(res['L_contour'], (0,255,0)), (res['R_contour'], (255,0,0))]:
                    if cnt is not None and len(cnt) > 0:
                        try:
                            cnt_scaled = (cnt * 4).astype(np.int32).reshape(-1,1,2)
                            cv2.drawContours(dbg, [cnt_scaled], -1, color, 1)
                        except: pass

                for hull, color in [(res['L_hull'], (0,255,255)), (res['R_hull'], (255,255,0))]:
                    if hull is not None and len(hull) > 0:
                        try:
                            h_scaled = (hull * 4).astype(np.int32).reshape(-1,1,2)
                            cv2.drawContours(dbg, [h_scaled], -1, color, 1)
                        except: pass

                for defects, color in [(res['L_defects'], (0,0,255)), (res['R_defects'], (255,0,255))]:
                    for pt in defects:
                        try:
                            sx, sy = pt[0]*4, pt[1]*4
                            dx, dy = pt[0] - 80, pt[1] - 60
                            code = int(((math.degrees(math.atan2(dy, dx)) + 180) / 45.0) % 8)
                            cv2.circle(dbg, (sx, sy), 6, color, 2)
                            cv2.putText(dbg, str(code), (sx+8, sy-8),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                        except: pass

            cv2.rectangle(dbg, (0, 76*4), (640, 105*4), (0,255,0), 1)
            cv2.rectangle(dbg, (0, 85*4), (640, 118*4), (255,165,0), 1)
            cv2.putText(dbg, f"F{fc}", (5,15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            cv2.putText(dbg, "G:cnt Y:hull R:defect", (5,35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)

            dm = Image(); dm.header.stamp = msg.header.stamp
            dm.height=480; dm.width=640; dm.encoding="bgr8"
            dm.is_bigendian=False; dm.step=640*3; dm.data=dbg.astype(np.uint8).tobytes()
            dbg_pub.publish(dm)
        except:
            pass

    except Exception as e:
        import traceback; rospy.logerr(f"{e}\n{traceback.format_exc()}")


rospy.init_node("debug_convex")
dbg_pub = rospy.Publisher('/car_server/debug_image', Image, queue_size=1)
rospy.Subscriber("/usb_cam/image_raw", Image, callback, queue_size=1)
rospy.loginfo("✅ 双检测调试启动 (不控制小车)")
rospy.loginfo("   手动把车放在直道/弯道/岔口 → 观察终端输出")
rospy.loginfo("   查看画面: rqt_image_view /car_server/debug_image")
rospy.spin()
