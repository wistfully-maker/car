#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
scan_and_park.py —— 旋转扫描 OCR + 激光雷达 PCA 精准停车（一体化节点）

运行方式:
    rosrun stop scan_and_park.py

功能：
    两阶段全流程自动化：

    Phase 1 — 旋转扫描找板子（替代原 item_finder 的巡航点遍历）:
        拍照 → OCR 识别 → 没找到 → 转 72° → 停下 → 拍照 → OCR → ...
        找到目标关键字后计算板子偏角 camera_deg，进入 Phase 2。

    Phase 2 — LiDAR PCA 精准停车（替代原 item_finder 的 stage 2-5）:
        1. 根据 camera_deg 旋转对准板子方向
        2. PCA 拟合板子点云 → 法向量 → 旋转车头对准板子中心
        3. 直行靠近板子（LiDAR 实时反馈距离）
        4. 二次 PCA 精细姿态对正
        5. LiDAR 正前方测距微调

ROS 接口:
    发布: /perception/detect_image (Image)    —— 图像给 OCR 节点
    发布: /cmd_vel (Twist)                    —— 底盘运动控制
    发布: /park_result (String)               —— 停车结果
    订阅: /perception/bounding_boxes (BoundingBoxes) —— OCR 识别结果
    订阅: /usb_cam/image_raw (Image)          —— 摄像头原始图像
    订阅: /scan (LaserScan)                   —— 激光雷达数据

参数:
    ~target_keyword        —— OCR 目标关键字，默认 "食品"
    ~rotate_angle          —— Phase 1 每次旋转角度（度），默认 72
    ~max_rotations         —— Phase 1 最大旋转次数，默认 5
    ~ocr_timeout           —— 单次 OCR 等待超时（秒），默认 3.0
    ~board_angle_left      —— 板子在激光坐标系中的左边界角（度），默认 -15
    ~board_angle_right     —— 板子在激光坐标系中的右边界角（度），默认 15
    ~target_distance       —— 目标停车距离（米），默认 0.20
    ~distance_tolerance    —— 距离容差（米），默认 0.03
    ~yaw_tolerance         —— 姿态对正容差（度），默认 1.0
    ~auto_start_delay      —— 启动后延迟（秒），默认 1.0
"""

import os
import sys
import math
import time
import json
import hashlib
import hmac
import base64
import threading
import urllib.request
import urllib.parse

import rospy
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from stop.msg import BoundingBoxes


# ==============================================================================
# 讯飞在线 WebSocket TTS
# ==============================================================================
_TTS_APPID = "2697a716"
_TTS_API_KEY = "424d575268594135073cbe862124f0b2"
_TTS_API_SECRET = "NmU5Yjg0YzJmOWFhODE0OTQyYjQ4YmUy"

try:
    import websocket
    _HAVE_WS = True
except ImportError:
    _HAVE_WS = False


def tts_synthesize(text):
    if not _HAVE_WS:
        return None
    import ssl
    date_str = time.strftime("%a, %d %b %Y %H:%M:%S GMT", time.gmtime())
    signature_origin = f"host: ws-api.xfyun.cn\ndate: {date_str}\nGET /v2/tts HTTP/1.1"
    signature_sha = hmac.new(_TTS_API_SECRET.encode(), signature_origin.encode(), hashlib.sha256).digest()
    signature = base64.b64encode(signature_sha).decode()
    authorization_origin = (
        f'api_key="{_TTS_API_KEY}", algorithm="hmac-sha256", '
        f'headers="host date request-line", signature="{signature}"'
    )
    authorization = base64.b64encode(authorization_origin.encode()).decode()
    url = (
        f"wss://ws-api.xfyun.cn/v2/tts"
        f"?authorization={urllib.parse.quote(authorization)}"
        f"&date={urllib.parse.quote(date_str)}"
        f"&host=ws-api.xfyun.cn"
    )
    req_data = {
        "common": {"app_id": _TTS_APPID},
        "business": {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan",
                     "speed": 50, "volume": 100, "pitch": 50, "tte": "utf8"},
        "data": {"status": 2, "text": base64.b64encode(text.encode("utf-8")).decode()},
    }
    ws = None
    try:
        ws = websocket.create_connection(url, timeout=20, sslopt={"cert_reqs": ssl.CERT_NONE})
        ws.send(json.dumps(req_data))
        audio_data = b""
        while True:
            resp = json.loads(ws.recv())
            if resp.get("code") != 0:
                rospy.logerr("TTS error: %s", resp.get("message", ""))
                return None
            b64 = resp.get("data", {}).get("audio", "")
            if b64:
                audio_data += base64.b64decode(b64)
            if resp.get("data", {}).get("status") == 2:
                break
        return audio_data if audio_data else None
    except Exception as e:
        rospy.logerr("TTS error: %s", e)
        return None
    finally:
        if ws:
            ws.close()


def tts_play(audio_bytes):
    if not audio_bytes:
        return
    import tempfile
    tmp = tempfile.NamedTemporaryFile(suffix=".pcm", delete=False)
    try:
        tmp.write(audio_bytes)
        tmp.close()
        os.system(f"aplay -q -f S16_LE -r 16000 -c 1 {tmp.name} 2>/dev/null")
    finally:
        os.unlink(tmp.name)


def tts_speak(text):
    def _run():
        audio = tts_synthesize(text)
        if audio:
            tts_play(audio)
    t = threading.Thread(target=_run)
    t.daemon = True
    t.start()


# ==============================================================================
# PCA 工具函数（来自 src item_finder.py）
# ==============================================================================

def calculate_external_normal(*points):
    if len(points) < 2:
        raise ValueError("At least 2 points required")
    pts = np.array(points)
    x, y = pts[:, 0], pts[:, 1]
    x_mean, y_mean = np.mean(x), np.mean(y)
    cov = np.cov(x - x_mean, y - y_mean)
    _, eigenvectors = np.linalg.eigh(cov)
    normal = eigenvectors[:, 0]
    a, b = normal
    C = -(a * x_mean + b * y_mean)
    if C > 1e-10:
        a, b = -a, -b
    norm = np.hypot(a, b)
    return (a / norm, b / norm), (x_mean, y_mean)


def vector_to_angle(nx, ny):
    if math.isclose(nx, 0.0, abs_tol=1e-9) and math.isclose(ny, 0.0, abs_tol=1e-9):
        raise ValueError("Zero vector")
    return math.degrees(math.atan2(ny, nx))


# ==============================================================================
# 主节点：ScanAndPark
# ==============================================================================

class ScanAndPark:
    """
    一体化节点：Phase 1 旋转扫描 OCR → Phase 2 LiDAR PCA 精准停车。
    """

    # 状态
    (IDLE, SCANNING, PRE_ALIGN,
     YAW_COARSE, APPROACH, YAW_FINE, X_FINE, DONE, FAILED) = range(9)

    def __init__(self):
        rospy.init_node("scan_and_park")

        # ======== Phase 1 参数 ========
        self.target_keyword   = rospy.get_param("~target_keyword", "食品")
        self.rotate_angle     = rospy.get_param("~rotate_angle", 72)
        self.rotate_speed     = rospy.get_param("~rotate_speed", 1.5)
        self.max_rotations    = rospy.get_param("~max_rotations", 5)
        self.ocr_timeout      = rospy.get_param("~ocr_timeout", 3.0)

        # ======== Phase 2 参数 ========
        self.board_angle_left  = rospy.get_param("~board_angle_left", -15.0)
        self.board_angle_right = rospy.get_param("~board_angle_right", 15.0)
        self.target_distance   = rospy.get_param("~target_distance", 0.20)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.03)
        self.yaw_tolerance     = rospy.get_param("~yaw_tolerance", 1.0)
        self.linear_speed      = rospy.get_param("~linear_speed", 0.15)
        self.rotate_speed_fine = rospy.get_param("~rotate_speed_fine", 0.25)
        self.auto_start_delay  = rospy.get_param("~auto_start_delay", 1.0)

        # ======== 内部状态 ========
        self.state = self.IDLE
        self.camera_deg = 0.0          # OCR 计算出的板子偏角
        self.found_text = ""
        self.found_box = None

        # Phase 1 状态
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.ocr_result = None
        self.ocr_event = threading.Event()

        # Phase 2 状态
        self.latest_scan = None
        self.scan_lock = threading.Lock()

        # ======== ROS 接口 ========
        self.bridge = cv_bridge.CvBridge()
        self.img_pub   = rospy.Publisher('/perception/detect_image', Image, queue_size=1)
        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.result_pub = rospy.Publisher('/park_result', String, queue_size=1)

        rospy.Subscriber('/perception/bounding_boxes', BoundingBoxes, self._ocr_cb)
        rospy.Subscriber('/usb_cam/image_raw', Image, self._img_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)

        rospy.loginfo("[scan_and_park] ========================================")
        rospy.loginfo("[scan_and_park] Phase1 OCR : keyword='%s' rotate=%d° x%d",
                      self.target_keyword, self.rotate_angle, self.max_rotations)
        rospy.loginfo("[scan_and_park] Phase2 LiDAR: FOV[%.0f,%.0f] target=%.2fm",
                      self.board_angle_left, self.board_angle_right, self.target_distance)
        rospy.loginfo("[scan_and_park] ========================================")

        if self.auto_start_delay > 0:
            rospy.Timer(rospy.Duration(self.auto_start_delay), self._start, oneshot=True)
        else:
            self._start(None)

    # ==================================================================
    # 回调
    # ==================================================================

    def _img_cb(self, msg):
        with self.image_lock:
            self.latest_image = msg

    def _scan_cb(self, msg):
        with self.scan_lock:
            self.latest_scan = msg

    def _ocr_cb(self, msg):
        if self.state != self.SCANNING:
            return
        image_width = 640.0
        image_cx = image_width / 2.0
        fx = 417.02
        for box in msg.bounding_boxes:
            if self.target_keyword in box.Class.strip():
                self.found_text = box.Class.strip()
                center_x = (box.xmin + box.xmax) / 2.0
                self.camera_deg = math.degrees(math.atan((center_x - image_cx) / fx))
                self.found_box = (box.xmin, box.ymin, box.xmax, box.ymax)
                self.ocr_result = msg
                self.ocr_event.set()
                return
        self.ocr_result = msg
        self.ocr_event.set()

    # ==================================================================
    # 启动
    # ==================================================================

    def _start(self, event):
        if self.state != self.IDLE:
            return
        rospy.loginfo("[scan_and_park] ==== START ====")
        t = threading.Thread(target=self._run)
        t.daemon = True
        t.start()

    def _run(self):
        try:
            # ====== Phase 1: 旋转扫描 OCR ======
            if not self._phase1_scan():
                self._fail("OCR scan: board not found")
                return

            # ====== Phase 2: 对准 → LiDAR PCA 精准停车 ======
            self._phase2_pre_align()
            self._phase2_yaw_coarse()
            self._phase2_approach()
            self._phase2_yaw_fine()
            self._phase2_x_fine()

            self.state = self.DONE
            rospy.loginfo("[scan_and_park] ==== DONE ====")
            self.result_pub.publish(String(data="done"))

        except Exception as e:
            rospy.logerr("[scan_and_park] Exception: %s", e)
            self.result_pub.publish(String(data="failed:" + str(e)))

    # ==================================================================
    # Phase 1: 旋转扫描 OCR
    # ==================================================================

    def _phase1_scan(self):
        """旋转 72°→拍照→OCR，循环直到找到板子或遍历完所有朝向。"""
        rospy.loginfo("[scan_and_park] --- Phase 1: OCR scan ---")

        for i in range(self.max_rotations + 1):
            if rospy.is_shutdown():
                return False

            rospy.sleep(0.5)  # 等摄像头稳定

            rospy.loginfo("[scan_and_park] OCR scan %d/%d", i + 1, self.max_rotations + 1)
            self.state = self.SCANNING

            found = self._ocr_capture()
            if found:
                rospy.loginfo("[scan_and_park] ==== FOUND: %s (camera_deg=%.1f°) ====",
                              self.found_text, self.camera_deg)
                tts_speak(f"找到{self.target_keyword}车间")
                return True

            if i < self.max_rotations:
                rospy.loginfo("[scan_and_park] Not found, rotating %d°", self.rotate_angle)
                self._rotate(self.rotate_angle, self.rotate_speed)

        rospy.logwarn("[scan_and_park] Phase 1 exhausted: board not found")
        tts_speak(f"未找到{self.target_keyword}车间")
        return False

    def _ocr_capture(self):
        """发布一帧图像给 OCR，等待结果返回。"""
        with self.image_lock:
            img = self.latest_image
        if img is None:
            rospy.logwarn("[scan_and_park] No camera image")
            return False

        self.ocr_event.clear()
        self.ocr_result = None
        self.found_text = ""

        self.img_pub.publish(img)
        got = self.ocr_event.wait(self.ocr_timeout)

        if not got:
            rospy.logwarn("[scan_and_park] OCR timeout")
            return False

        return bool(self.found_text)

    # ==================================================================
    # Phase 2 Step 0: 根据 camera_deg 预对准
    # ==================================================================

    def _phase2_pre_align(self):
        """根据 OCR 得到的偏角旋转车头对准板子方向。"""
        self.state = self.PRE_ALIGN
        rospy.loginfo("[scan_and_park] --- Phase 2: Pre-align camera_deg=%.1f° ---", self.camera_deg)
        self._rotate(self.camera_deg, self.rotate_speed)

    # ==================================================================
    # Phase 2 Step 1: PCA 粗对准（对准板子中心 B）
    # ==================================================================

    def _phase2_yaw_coarse(self):
        self.state = self.YAW_COARSE
        rospy.loginfo("[scan_and_park] --- YAW coarse: face board center ---")

        scan = self._get_scan(1.0)
        if scan is None:
            rospy.logwarn("[scan_and_park] YAW coarse: no scan, skip")
            return

        points, B_point = self._extract_board_points(scan)
        if points is None:
            rospy.logwarn("[scan_and_park] YAW coarse: no board points, skip")
            return

        angle = vector_to_angle(B_point[0], B_point[1])
        rospy.loginfo("[scan_and_park] YAW coarse: B=(%.3f,%.3f) angle=%.2f°",
                      B_point[0], B_point[1], angle)

        if abs(angle) >= self.yaw_tolerance:
            self._rotate(angle, self.rotate_speed)

    # ==================================================================
    # Phase 2 Step 2: 直行靠近
    # ==================================================================

    def _phase2_approach(self):
        self.state = self.APPROACH
        rospy.loginfo("[scan_and_park] --- APPROACH: target=%.2fm ---", self.target_distance)

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30.0)
        stuck_timeout = rospy.Duration(3.0)
        last_dist = None
        last_dist_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("[scan_and_park] APPROACH timeout (30s)")
                break

            scan = self._get_scan(0.3)
            if scan is None:
                rospy.sleep(0.1)
                continue

            dist = self._get_board_center_distance(scan)
            if dist is None:
                rospy.logwarn("[scan_and_park] APPROACH: lost board")
                break

            if dist <= self.target_distance:
                rospy.loginfo("[scan_and_park] APPROACH reached: %.3f <= %.2f", dist, self.target_distance)
                break

            # 卡住检测
            if last_dist is not None and abs(dist - last_dist) < 0.01:
                if (rospy.Time.now() - last_dist_time) > stuck_timeout:
                    rospy.logerr("[scan_and_park] APPROACH stuck! dist=%.3f unchanged. Chassis running?", dist)
                    break
            else:
                last_dist = dist
                last_dist_time = rospy.Time.now()

            rospy.loginfo_throttle(0.5, "[scan_and_park] APPROACH: dist=%.3f", dist)
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.sleep(0.3)

    # ==================================================================
    # Phase 2 Step 3: PCA 精细姿态对正（对准法向量）
    # ==================================================================

    def _phase2_yaw_fine(self):
        self.state = self.YAW_FINE
        rospy.loginfo("[scan_and_park] --- YAW fine: face board normal ---")

        scan = self._get_scan(1.0)
        if scan is None:
            rospy.logwarn("[scan_and_park] YAW fine: no scan")
            return

        points, _ = self._extract_board_points(scan)
        if points is None or len(points) < 2:
            rospy.logwarn("[scan_and_park] YAW fine: insufficient points")
            return

        try:
            (nx, ny), _ = calculate_external_normal(*points)
        except ValueError:
            return

        angle = vector_to_angle(nx, ny)
        rospy.loginfo("[scan_and_park] YAW fine: normal=(%.3f,%.3f) angle=%.2f°", nx, ny, angle)

        if abs(angle) >= self.yaw_tolerance:
            self._rotate(angle, self.rotate_speed_fine)

    # ==================================================================
    # Phase 2 Step 4: X 轴直行微调
    # ==================================================================

    def _phase2_x_fine(self):
        self.state = self.X_FINE
        rospy.loginfo("[scan_and_park] --- X FINE: target=%.2f tol=%.3f ---",
                      self.target_distance, self.distance_tolerance)

        rate = rospy.Rate(10)
        slow_speed = self.linear_speed * 0.5

        while not rospy.is_shutdown():
            scan = self._get_scan(0.3)
            if scan is None:
                rospy.sleep(0.1)
                continue

            n = len(scan.ranges)
            c = n // 2
            valid = [scan.ranges[i] for i in range(c - 1, c + 2)
                    if 0 < scan.ranges[i] < float('inf')]
            if not valid:
                rospy.logwarn("[scan_and_park] X FINE: no valid front distance")
                break

            dist = min(valid)
            if dist <= self.target_distance + self.distance_tolerance:
                rospy.loginfo("[scan_and_park] X FINE done: %.3f m", dist)
                break

            remaining = dist - self.target_distance
            speed = min(slow_speed, max(0.05, remaining * 0.5))
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())

    # ==================================================================
    # 辅助方法
    # ==================================================================

    def _rotate(self, angle_degrees, speed):
        if abs(angle_degrees) < 0.3:
            return
        angle_rad = math.radians(angle_degrees)
        duration = abs(angle_rad) / abs(speed)
        angular = abs(speed) * (1 if angle_degrees > 0 else -1)
        rospy.loginfo("  Rotating %.1f° @ %.2f rad/s", angle_degrees, angular)

        cmd = Twist()
        cmd.angular.z = angular
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and (rospy.Time.now() - start) < rospy.Duration(duration):
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.2)

    def _get_scan(self, timeout=1.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            with self.scan_lock:
                if self.latest_scan is not None:
                    return self.latest_scan
            if (rospy.Time.now() - start) > rospy.Duration(timeout):
                return None
            rospy.sleep(0.05)
        return None

    def _extract_board_points(self, scan):
        board_center = (self.board_angle_left + self.board_angle_right) / 2.0
        half = abs(self.board_angle_right - self.board_angle_left) / 2.0
        points = []
        B_point, B_dist = None, float('inf')

        for deg in np.arange(-half, half + 0.5, 0.5):
            angle_deg = board_center + deg
            angle_rad = math.radians(angle_deg)
            idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
            if 0 <= idx < len(scan.ranges):
                r = scan.ranges[idx]
                if scan.range_min <= r <= scan.range_max:
                    x = r * math.cos(angle_rad)
                    y = r * math.sin(angle_rad)
                    points.append((x, y))
                    if abs(deg) < 0.5 and r < B_dist:
                        B_dist = r
                        B_point = (x, y)

        if len(points) < 2:
            return None, None
        return points, B_point

    def _get_board_center_distance(self, scan):
        board_center = (self.board_angle_left + self.board_angle_right) / 2.0
        angle_rad = math.radians(board_center)
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        dists = []
        for di in [-1, 0, 1]:
            i = idx + di
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if scan.range_min <= r <= scan.range_max:
                    dists.append(r)
        return min(dists) if dists else None

    def _fail(self, reason):
        self.state = self.FAILED
        rospy.logerr("[scan_and_park] ==== FAILED: %s ====", reason)
        self.result_pub.publish(String(data="failed:" + reason))


# ==============================================================================
# 主入口
# ==============================================================================
if __name__ == "__main__":
    try:
        ScanAndPark()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
