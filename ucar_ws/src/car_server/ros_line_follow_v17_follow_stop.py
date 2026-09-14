#!/usr/bin/env python3
"""
V17 + follow.py停车检测 + 八邻域方向统计输出

管线: Canny→IPM(dilate15+erode7)→八邻域(返回方向码)→中线/墙线→PID
新增:
  1. 停车检测: 完全替换为follow.py的双线检测(原图阈值185, 前线→对齐→后线→停车)
  2. 八邻域方向: trace()返回方向码, 每30帧输出L/R方向统计
  3. 凸包拐角: ContourCornerTracker检测岔口拐角并输出
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

# === V17管线参数 ===
GAUSSIAN = (5,5); CANNY_LO, CANNY_HI = 50, 150
DILATE_K = np.ones((15,15), np.uint8)
ERODE_K  = np.ones((7,7), np.uint8)
ROI_Y, ROI_H = 240, 240

# === 方向表(和UP主一样交换) ===
SEEDS_L = [(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1)]
SEEDS_R = [(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1),(0,1),(1,1)]

# === PID ===
KP, KI, KD = 0.25, 0.0, 0.1
BASE_SPEED, MAX_ANGULAR = 0.12, 0.22
DEADZONE = 15; LOOKAHEAD = 10

# === follow.py 停车检测参数 ===
STOP_ROW_START  = 24    # 原图行范围(对应640×480→按比例缩放)
STOP_ROW_END    = 119
STOP_THRESH_VAL = 185   # 二值化阈值(只保留纯白横线)
STOP_FRONT_ROW  = 60    # 前线判定: 白块底部平均行>60
STOP_BACK_ROW   = 60    # 后线判定
STOP_FILTER_NUM = 30    # 前→后线之间的过滤帧数
STOP_ALIGN_DEG  = 20    # 前线检测后微调角度


# ============================================================
# 凸包拐角检测器 (follow.py 原样, 用于输出方向统计)
# ============================================================
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
            f = int(flat[2])  # [start, end, farthest, distance]
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
        st = {'left_dirs': {i: 0 for i in range(8)},
              'right_dirs': {i: 0 for i in range(8)}}
        for side, pts in (('left', Lc), ('right', Rc)):
            for p in pts:
                dx = p[0] - size[0] // 2
                dy = p[1] - size[1] // 2
                code = int(((math.degrees(math.atan2(dy, dx)) + 180) / 45.0) % 8)
                st[side + '_dirs'][code] += 1
        return {'stats': st}


class LineFollowerV17FollowStop:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_error = 0.0; self.integral = 0.0; self.road_half = 47
        self.fc = 0; self.mode = "INIT"

        # === follow.py 停车状态 ===
        self.is_stop_line_front_found = False
        self.stop_filtered_fps = 0
        self.stop_filter_threshold = STOP_FILTER_NUM
        self.play_voice_once = False
        self.stop_detected = False
        self.stop_enter_time = None
        self.is_stopped = False

        # === 拐角检测器 ===
        self.corner_tracker = ContourCornerTracker()

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_dbg = rospy.Publisher("/car_server/debug_image", Image, queue_size=1)
        rospy.loginfo("✅ V17+follow停车+方向统计 启动")

    # ================================================================
    # 八邻域追踪 (改: 同时返回方向码列表)
    # ================================================================
    def trace(self, image, start, seeds):
        pts = []; dirs = []; cx, cy = start; pts.append((cx, cy))
        visited = set()
        for _ in range(400):
            if (cx, cy) in visited: break
            visited.add((cx, cy))
            cand = []
            for i in range(8):
                da = seeds[i]; ax = cx + da[0]; ay = cy + da[1]
                db = seeds[(i + 1) % 8]; bx = cx + db[0]; by = cy + db[1]
                if 0 <= ax < image.shape[1] and 0 <= ay < image.shape[0] and \
                   0 <= bx < image.shape[1] and 0 <= by < image.shape[0]:
                    if image[ay, ax] == 0 and image[by, bx] == 255:
                        cand.append((ax, ay, i))  # ★ 新增: 记录方向码i
            if not cand: break
            best = min(cand, key=lambda p: p[1])
            cx, cy = best[0], best[1]
            pts.append((cx, cy)); dirs.append(best[2])  # ★ 新增: 保存方向码
        return pts, dirs

    # ================================================================
    # follow.py 停车线检测 (完全复制, 适配640×480原图)
    # ================================================================
    def detect_stop_line(self, bgr_image):
        """follow.py的停车线检测, 在原始BGR图上运行"""
        # 前线已找到 → 帧计数
        if self.is_stop_line_front_found:
            self.stop_filtered_fps += 1
            if self.stop_filtered_fps >= self.stop_filter_threshold:
                if not self.play_voice_once:
                    twist = Twist()
                    self.pub_cmd.publish(twist)
                    rospy.loginfo("🔊 过滤帧结束, 即将检测后线!")
                    rospy.sleep(0.3)
                    self.play_voice_once = True

        h, w = bgr_image.shape[:2]
        # 按比例缩放follow.py的行范围(原160×120 → 当前分辨率)
        scale = h / 120.0
        r_start = int(STOP_ROW_START * scale)
        r_end   = int(STOP_ROW_END * scale)
        front_thresh = int(STOP_FRONT_ROW * scale)
        back_thresh  = int(STOP_BACK_ROW * scale)

        cropped = bgr_image[r_start:r_end + 1, :]
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, bi = cv2.threshold(blur, STOP_THRESH_VAL, 255, cv2.THRESH_BINARY)

        bh, bw = bi.shape
        mid = bw // 2
        left_col = mid - 20
        right_col = mid + 20
        rows_found = []

        for col in range(left_col, right_col + 1, 5):
            for row in range(bh - 2, 0, -1):
                if bi[row, col] == 255 and bi[row + 1, col] == 0:
                    rows_found.append(row)
                    break

        if len(rows_found) >= 6:
            if self.stop_filtered_fps <= self.stop_filter_threshold:
                thr = front_thresh
            else:
                thr = back_thresh

            if sum(rows_found) / len(rows_found) > thr:
                if not self.is_stop_line_front_found:
                    rospy.loginfo("🛑 找到终点停车线的前线! 微调对齐...")
                    # 微调旋转(正20°)
                    self._rotate_simple(STOP_ALIGN_DEG, 0.3)
                    rospy.sleep(0.3)
                    self.is_stop_line_front_found = True
                    return False
                elif self.stop_filtered_fps >= self.stop_filter_threshold:
                    rospy.loginfo("🛑 找到终点停车线的后线! 准备停车!")
                    return True
        return False

    def _rotate_simple(self, deg, spd):
        """简单定时旋转"""
        dur = abs(math.radians(deg)) / spd
        cmd = Twist(); cmd.angular.z = spd if deg > 0 else -spd
        t0 = rospy.Time.now()
        while rospy.Time.now() - t0 < rospy.Duration(dur):
            self.pub_cmd.publish(cmd); rospy.sleep(0.05)
        self.pub_cmd.publish(Twist())

    # ================================================================
    # 八邻域方向统计输出
    # ================================================================
    def _log_direction_stats(self, L_dirs, R_dirs):
        """每30帧输出L/R方向码统计"""
        if not L_dirs and not R_dirs: return

        def count_dirs(dirs_list):
            c = {i: 0 for i in range(8)}
            for d in dirs_list: c[d] = c.get(d, 0) + 1
            return c

        Lc = count_dirs(L_dirs) if L_dirs else {}
        Rc = count_dirs(R_dirs) if R_dirs else {}

        # 最近50帧
        Lr = L_dirs[-50:] if len(L_dirs) >= 50 else L_dirs
        Rr = R_dirs[-50:] if len(R_dirs) >= 50 else R_dirs
        pct_L2 = (Lr.count(2) / len(Lr) * 100) if Lr else 0
        pct_L6 = (Lr.count(6) / len(Lr) * 100) if Lr else 0
        pct_R2 = (Rr.count(2) / len(Rr) * 100) if Rr else 0
        pct_R0 = (Rr.count(0) / len(Rr) * 100) if Rr else 0

        # 关键方向组合(用于检测拐角)
        lp = len(L_dirs) if L_dirs else 0
        rp = len(R_dirs) if R_dirs else 0
        rospy.loginfo(f"📐 八邻域方向 F{self.fc:4d} | "
                      f"L总{lp:3d} 2→{Lc.get(2,0):3d} 4→{Lc.get(4,0):2d} 6→{Lc.get(6,0):2d} | "
                      f"R总{rp:3d} 0→{Rc.get(0,0):2d} 6→{Rc.get(6,0):2d} | "
                      f"最近50 L2%={pct_L2:.0f} L6%={pct_L6:.0f} R2%={pct_R2:.0f} R0%={pct_R0:.0f}")

    # ================================================================
    # 凸包拐角方向统计输出
    # ================================================================
    def _log_corner_stats(self, corner_result):
        """输出ContourCornerTracker的方向统计"""
        st = corner_result['stats']
        L = st['left_dirs']; R = st['right_dirs']
        rospy.loginfo(f"🔀 凸包拐角 F{self.fc:4d} | "
                      f"L(0={L[0]} 1={L[1]} 2={L[2]} 3={L[3]} 4={L[4]} 5={L[5]} 6={L[6]} 7={L[7]}) | "
                      f"R(0={R[0]} 1={R[1]} 2={R[2]} 3={R[3]} 4={R[4]} 5={R[5]} 6={R[6]} 7={R[7]})")

    # ================================================================
    # 主回调
    # ================================================================
    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.fc += 1

            # ===== V17管线 =====
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, GAUSSIAN, 0)
            canny = cv2.Canny(blur, CANNY_LO, CANNY_HI)
            ipm = cv2.warpPerspective(canny, IPM_M, (IPM_W, IPM_H))
            dilated = cv2.dilate(ipm, DILATE_K)
            morphed = cv2.erode(dilated, ERODE_K)
            roi = morphed[ROI_Y:ROI_Y + ROI_H, 0:IPM_W]
            _, binary = cv2.threshold(roi, 5, 255, cv2.THRESH_BINARY)
            rh, rw = binary.shape

            # ===== NEW: follow.py 停车线检测 (在原图上) =====
            if False and not self.stop_detected:
                if self.detect_stop_line(frame):
                    self.stop_detected = True
                    self.stop_enter_time = rospy.Time.now()
                    rospy.loginfo("🛑 检测到后线! 开始直行停车倒计时...")

            # 已停车
            if self.is_stopped:
                self.pub_cmd.publish(Twist())
                return

            # 前线已找到或已触发停车 → 直行不转向(等后线或停车)
            if False and self.stop_detected:
                # 后线检测到→立即停车
                self.is_stopped = True
                rospy.loginfo("🅿 停车!")
                self.pub_cmd.publish(Twist())
                # 保存标记
                with open("/tmp/stop_done.txt", "w") as f: f.write("parked")
                return

            # 前线已找到, 还在等待后线 → 直行不转向
            if False and self.is_stop_line_front_found and not self.stop_detected:
                twist = Twist()
                twist.linear.x = BASE_SPEED
                self.pub_cmd.publish(twist)
                dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg, f"GO_TO_BACK {self.stop_filtered_fps}/{self.stop_filter_threshold}",
                           (5, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                self._pub_dbg(dbg, msg)
                return

            # ===== V17自适应找起点 =====
            srow = None; Ls = Rs = None
            for row in range(rh - 5, 20, -1):
                xs = np.where(binary[row, :] == 255)[0]
                if len(xs) < 15: continue
                lx, rx = xs[0], xs[-1]
                if rx - lx < 20: continue
                for x in range(max(1, lx - 5), min(rw - 1, lx + 5)):
                    if binary[row, x] == 0 and binary[row, x - 1] == 255:
                        Ls = (x - 1, row); break
                if Ls is None: Ls = (lx, row)
                for x in range(max(1, rx - 5), min(rw - 1, rx + 5)):
                    if binary[row, x] == 0 and binary[row, x + 1] == 255:
                        Rs = (x + 1, row); break
                if Rs is None: Rs = (rx, row)
                srow = row; break

            twist = Twist()
            error = 0.0

            if Ls and Rs:
                # ★ trace() 现在返回方向码
                Lp, Ld = self.trace(binary, Ls, SEEDS_R)
                Rp, Rd = self.trace(binary, Rs, SEEDS_L)

                lb = np.full(rh, -1, dtype=int); rb = np.full(rh, -1, dtype=int)
                for x, y in Lp:
                    if 0 <= y < rh and lb[y] == -1: lb[y] = x
                for x, y in Rp:
                    if 0 <= y < rh and rb[y] == -1: rb[y] = x

                nL_all = (lb != -1).sum(); nR_all = (rb != -1).sum()
                la = slice(max(0, rh - LOOKAHEAD), rh)

                # 路宽更新
                if (lb[la] != -1).sum() > 5 and (rb[la] != -1).sum() > 5:
                    vl = lb[la][lb[la] != -1]; vr = rb[la][rb[la] != -1]
                    if len(vl) > 3 and len(vr) > 3:
                        self.road_half = (vr.mean() - vl.mean()) // 2
                        self.road_half = max(20, min(200, self.road_half))

                # same-line检测
                same_line = False
                if nL_all > 10 and nR_all > 10:
                    vl = lb[la][lb[la] != -1]; vr = rb[la][rb[la] != -1]
                    if len(vl) > 3 and len(vr) > 3:
                        if (vr.mean() - vl.mean()) < 40:
                            same_line = True

                # 墙切换
                if nL_all > 20 and nR_all > 20 and not same_line:
                    cl = np.full(rh, -1, dtype=int)
                    for y in range(rh):
                        if lb[y] != -1 and rb[y] != -1: cl[y] = (lb[y] + rb[y]) // 2
                    lc = cl[la][cl[la] != -1]
                    error = rw // 2 - lc.mean() if len(lc) > 3 else self.last_error
                    self.mode = "CENTER"
                elif (nL_all > 20 and nR_all > 20 and same_line) or nL_all > 20:
                    ll = lb[la][lb[la] != -1]
                    error = rw // 2 - (ll.mean() + self.road_half) if len(ll) > 3 else self.last_error
                    self.mode = "SAME_LINE" if same_line else "LEFT_WALL"
                elif nR_all > 20:
                    rr = rb[la][rb[la] != -1]
                    error = rw // 2 - (rr.mean() - self.road_half) if len(rr) > 3 else self.last_error
                    self.mode = "RIGHT_WALL"
                else:
                    error = self.last_error; self.mode = "HOLD"

                # PID
                if abs(error) < DEADZONE: error = 0.0
                self.integral += error; self.integral = max(-50, min(50, self.integral))
                ang = KP * error + KI * self.integral + KD * (error - self.last_error)
                self.last_error = error
                ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))

                cov = (lb != -1).sum() / rh + (rb != -1).sum() / rh
                twist.linear.x = BASE_SPEED if cov > 0.5 else BASE_SPEED * 0.7
                twist.angular.z = ang

                # ===== NEW: 每30帧输出八邻域方向统计 =====
                if self.fc % 30 == 0:
                    self._log_direction_stats(Ld, Rd)

                # ===== NEW: 凸包拐角方向统计 (每30帧) =====
                if self.fc % 30 == 0:
                    corner_res = self.corner_tracker.process(binary)
                    if corner_res is not None:
                        self._log_corner_stats(corner_res)
                    else:
                        rospy.loginfo(f"🔀 凸包拐角 F{self.fc:4d} | 未检测到(轮廓<2)")

                # 调试画面
                dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                if srow: cv2.line(dbg, (0, srow), (rw, srow), (255, 255, 0), 1)
                for x, y in Lp: cv2.circle(dbg, (x, y), 1, (0, 255, 0), -1)
                for x, y in Rp: cv2.circle(dbg, (x, y), 1, (0, 0, 255), -1)
                for y in range(rh):
                    if lb[y] != -1 and rb[y] != -1:
                        cv2.circle(dbg, ((lb[y] + rb[y]) // 2, y), 1, (255, 255, 0), -1)
                    elif lb[y] != -1 and self.mode in ("LEFT_WALL", "SAME_LINE"):
                        cv2.circle(dbg, (int(lb[y] + self.road_half), y), 1, (0, 255, 255), -1)
                    elif rb[y] != -1 and self.mode == "RIGHT_WALL":
                        cv2.circle(dbg, (int(rb[y] - self.road_half), y), 1, (0, 255, 255), -1)

                stop_s = f" FRONT={self.is_stop_line_front_found}" if self.is_stop_line_front_found else ""
                info = f"V17FS F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}{stop_s}"
                cv2.putText(dbg, info, (5, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
            else:
                twist.linear.x = BASE_SPEED * 0.25; twist.angular.z = 0
                self.mode = "LOST"
                dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                cv2.putText(dbg, f"V17FS F{self.fc} LOST", (5, 12),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            self.pub_cmd.publish(twist)
            self._pub_dbg(dbg, msg)

            if self.fc % 30 == 0:
                rospy.loginfo(f"V17FS F{self.fc} {self.mode} e={error:.0f} hw={self.road_half}")

        except Exception as e:
            import traceback; rospy.logerr(f"V17FS: {e}\n{traceback.format_exc()}")

    def _pub_dbg(self, dbg, msg):
        dbg = dbg.astype(np.uint8)
        dm = Image()
        dm.header.stamp = msg.header.stamp
        dm.height = dbg.shape[0]; dm.width = dbg.shape[1]
        dm.encoding = "bgr8"; dm.is_bigendian = False
        dm.step = dbg.shape[1] * 3; dm.data = dbg.tobytes()
        self.pub_dbg.publish(dm)


if __name__ == "__main__":
    rospy.init_node("line_follower_v17_follow_stop")
    LineFollowerV17FollowStop()
    rospy.spin()
