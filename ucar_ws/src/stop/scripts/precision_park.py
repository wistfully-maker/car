#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
precision_park.py —— 激光雷达 PCA 精准停车节点（无地图，纯 LiDAR 闭环）

运行方式:
    rosrun stop precision_park.py

功能：
    不依赖地图和 move_base，纯用 LiDAR PCA + /cmd_vel 实现精准对位停车：

        Stage 1: PCA 拟合板子点云 → 法向量 → 旋转车身正对板子中心
        Stage 2: 驱动机器人靠近板子（LiDAR 实时反馈距离）
        Stage 3: 二次 PCA 姿态对正（慢速精细旋转）
        Stage 4: X 轴直行微调（LiDAR 正前方测距闭环）

    源自 src/ucar_nav/scripts/item_finder.py 的 stage 2-5 逻辑，
    去除 move_base / TF 坐标变换 / 地图依赖，全部在激光坐标系下完成。

ROS 接口:
    发布: /cmd_vel (Twist)              —— 底盘运动控制
    发布: /park_result (String)         —— 停车结果
    订阅: /scan (LaserScan)             —— 激光雷达数据

参数:
    ~board_angle_left       —— 板子在激光坐标系中的左边界角（度），默认 -15
    ~board_angle_right      —— 板子在激光坐标系中的右边界角（度），默认 15
    ~target_distance         —— 目标停车距离（米，到板子正面的距离），默认 0.20
    ~distance_tolerance      —— 距离容差（米），默认 0.03
    ~yaw_tolerance           —— 姿态对正容差（度），默认 1.0
    ~auto_start_delay        —— 启动后延迟（秒），默认 1.0
    ~rotate_speed_fast       —— 粗对准旋转速度（rad/s），默认 0.8
    ~rotate_speed_fine       —— 精细旋转速度（rad/s），默认 0.25
    ~linear_speed            —— 直行速度（m/s），默认 0.15

状态机:
    IDLE       (0) —— 等待启动
    YAW_COARSE (1) —— PCA 法向量 → 旋转对准板子中心
    APPROACH   (2) —— 直行靠近板子直到目标距离
    YAW_FINE   (3) —— 二次 PCA 精细姿态对正
    X_FINE     (4) —— LiDAR 正前方测距微调
    DONE       (5) —— 精度调整完成
"""

import math
import threading

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


# ==============================================================================
# 工具函数（来自 src item_finder.py）
# ==============================================================================

def calculate_external_normal(*points):
    """
    利用 PCA 主成分分析对板子点云进行直线拟合，计算板子法向量。

    参数:
        points: [(x1,y1), (x2,y2), ...]

    返回:
        ((nx, ny), (x_mean, y_mean))  — 法向量单位矢量、点云中心
    """
    if len(points) < 2:
        raise ValueError("At least 2 points are required to fit a line")

    pts = np.array(points)
    x = pts[:, 0]
    y = pts[:, 1]
    x_mean = np.mean(x)
    y_mean = np.mean(y)

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
    """二维方向向量 → 角度（度）。"""
    if math.isclose(nx, 0.0, abs_tol=1e-9) and math.isclose(ny, 0.0, abs_tol=1e-9):
        raise ValueError("Cannot determine angle for zero vector")
    return math.degrees(math.atan2(ny, nx))


# ==============================================================================
# 精准停车主节点
# ==============================================================================

class PrecisionPark:
    """
    LiDAR PCA 精准停车（无地图，纯 /cmd_vel 闭环控制）。

    流程:
        YAW_COARSE — PCA → 旋转车头对准板子中心
        APPROACH   — 直行靠近直到 LiDAR 测距 < target_distance
        YAW_FINE   — 二次 PCA 精细旋转对正
        X_FINE     — 正前方测距微调
    """

    IDLE, YAW_COARSE, APPROACH, YAW_FINE, X_FINE, DONE = range(6)

    def __init__(self):
        rospy.init_node("precision_park")

        # ---- ROS 参数 ----
        self.board_angle_left   = rospy.get_param("~board_angle_left", -15.0)
        self.board_angle_right  = rospy.get_param("~board_angle_right", 15.0)
        self.target_distance    = rospy.get_param("~target_distance", 0.20)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.03)
        self.yaw_tolerance      = rospy.get_param("~yaw_tolerance", 1.0)
        self.auto_start_delay   = rospy.get_param("~auto_start_delay", 1.0)
        self.rotate_speed_fast  = rospy.get_param("~rotate_speed_fast", 0.8)
        self.rotate_speed_fine  = rospy.get_param("~rotate_speed_fine", 0.25)
        self.linear_speed       = rospy.get_param("~linear_speed", 0.15)

        # ---- 状态 ----
        self.state = self.IDLE
        self.latest_scan = None
        self.scan_lock = threading.Lock()
        self.board_angle = None         # 从 /board_angle 收到的板子偏角
        self.board_angle_lock = threading.Lock()

        # ---- ROS 接口 ----
        self.cmd_pub    = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.result_pub = rospy.Publisher('/park_result', String, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)
        rospy.Subscriber('/board_angle', Float32, self._board_angle_cb)

        rospy.loginfo("[precision_park] Initialized (no-map, LiDAR-only)")
        rospy.loginfo("  board FOV   : [%.1f, %.1f] deg", self.board_angle_left, self.board_angle_right)
        rospy.loginfo("  target_dist : %.2f m  tolerance: %.3f m", self.target_distance, self.distance_tolerance)
        rospy.loginfo("  yaw_tol     : %.1f deg", self.yaw_tolerance)

        if self.auto_start_delay > 0:
            rospy.Timer(rospy.Duration(self.auto_start_delay), self._start, oneshot=True)
        else:
            self._start(None)

    # ==========================================================================
    # 回调
    # ==========================================================================

    def _scan_cb(self, msg):
        with self.scan_lock:
            self.latest_scan = msg

    def _board_angle_cb(self, msg):
        """接收 yolo8 发来的板子偏角。"""
        with self.board_angle_lock:
            if self.board_angle is None:  # 只取第一次
                self.board_angle = msg.data
                rospy.loginfo("[precision_park] Received /board_angle = %.1f°", msg.data)

    # ==========================================================================
    # 启动
    # ==========================================================================

    def _start(self, event):
        if self.state != self.IDLE:
            return
        rospy.loginfo("[precision_park] ==== START ====")
        t = threading.Thread(target=self._run)
        t.daemon = True
        t.start()

    def _run(self):
        try:
            # Step 0: 如果 yolo8 提供了板子偏角，先旋转对准
            with self.board_angle_lock:
                angle = self.board_angle
            if angle is not None:
                rospy.loginfo("[precision_park] Pre-align: rotating %.1f deg to face board", angle)
                self._rotate(angle, self.rotate_speed_fast)
            else:
                rospy.loginfo("[precision_park] No /board_angle received, using default FOV")

            # Stage 1: PCA 粗对准
            self.state = self.YAW_COARSE
            if not self._stage_yaw_align(coarse=True):
                rospy.logerr("[precision_park] Stage 1 failed: no scan data. Is LiDAR running?")
                self.result_pub.publish(String(data="failed:no_scan"))
                return

            # Stage 2: 直行靠近
            self.state = self.APPROACH
            self._stage_approach()

            # Stage 3: PCA 精对准
            self.state = self.YAW_FINE
            if not self._stage_yaw_align(coarse=False):
                rospy.logwarn("[precision_park] YAW fine skipped (no scan)")

            # Stage 4: X 微调
            self.state = self.X_FINE
            self._stage_x_fine()

            self.state = self.DONE
            rospy.loginfo("[precision_park] ==== DONE ====")
            self.result_pub.publish(String(data="done"))

        except Exception as e:
            rospy.logerr("[precision_park] Exception: %s", e)
            self.result_pub.publish(String(data="failed:" + str(e)))

    # ==========================================================================
    # Stage 1 & 3: PCA 姿态对正
    # ==========================================================================

    def _stage_yaw_align(self, coarse=True):
        """
        PCA 拟合板子点云 → 法向量 → 旋转车头对准板子中心。

        coarse=True:  对准板子中心方向（B 点方向）
        coarse=False: 对准板子法向量方向（正对板面）

        返回:
            True  对准完成（或无需对准）
            False 无雷达数据，无法对准
        """
        speed = self.rotate_speed_fast if coarse else self.rotate_speed_fine
        label = "coarse" if coarse else "fine"

        scan = self._get_scan(1.0)
        if scan is None:
            rospy.logwarn("[precision_park] YAW %s: no scan", label)
            return False

        points, B_point = self._extract_board_points(scan)
        if points is None:
            rospy.logwarn("[precision_park] YAW %s: insufficient points (%d)",
                          label, len(points) if points else 0)
            return False

        if coarse:
            # 粗对准：对准板子中心 B
            angle = vector_to_angle(B_point[0], B_point[1])
            rospy.loginfo("[precision_park] YAW coarse: B=(%.3f,%.3f) angle=%.2f deg",
                          B_point[0], B_point[1], angle)
        else:
            # 精对准：对准法向量（正对板面）
            (nx, ny), _ = calculate_external_normal(*points)
            angle = vector_to_angle(nx, ny)
            rospy.loginfo("[precision_park] YAW fine: normal=(%.3f,%.3f) angle=%.2f deg", nx, ny, angle)

        if abs(angle) < self.yaw_tolerance:
            rospy.loginfo("[precision_park] YAW %s: already aligned (%.2f < %.1f)",
                          label, angle, self.yaw_tolerance)
            return True

        self._rotate(angle, speed)
        return True

    # ==========================================================================
    # Stage 2: 直行靠近板子
    # ==========================================================================

    def _stage_approach(self):
        """
        用 LiDAR 实时检测板子中心距离，直行靠近直到距离 <= target_distance。
        闭环控制：每 100ms 读一次 LiDAR，更新距离。
        """
        rospy.loginfo("[precision_park] APPROACH: driving toward board, target=%.2f m",
                      self.target_distance)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30.0)          # 最多跑 30s
        stuck_timeout = rospy.Duration(3.0)     # 3s 距离不变视为卡住
        last_dist = None
        last_dist_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # 超时保护
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("[precision_park] APPROACH timeout (30s), stopping")
                break

            scan = self._get_scan(0.3)
            if scan is None:
                rospy.sleep(0.1)
                continue

            # 取板子中心角度附近的最小距离
            dist = self._get_board_center_distance(scan)
            if dist is None:
                rospy.logwarn("[precision_park] APPROACH: lost board, stopping")
                break

            if dist <= self.target_distance:
                rospy.loginfo("[precision_park] APPROACH reached: %.3f <= %.2f",
                              dist, self.target_distance)
                break

            # 移动检测：距离一直不变说明底盘没动或卡住了
            if last_dist is not None and abs(dist - last_dist) < 0.01:
                if (rospy.Time.now() - last_dist_time) > stuck_timeout:
                    rospy.logerr("[precision_park] APPROACH stuck! distance=%.3f unchanged for 3s. "
                                 "Is chassis driver running?", dist)
                    break
            else:
                last_dist = dist
                last_dist_time = rospy.Time.now()

            rospy.loginfo_throttle(0.5, "[precision_park] APPROACH: dist=%.3f → target=%.2f",
                                   dist, self.target_distance)

            cmd = Twist()
            cmd.linear.x = self.linear_speed
            self.cmd_pub.publish(cmd)
            rate.sleep()

        # 停止
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.3)

    # ==========================================================================
    # Stage 4: X 轴直行微调
    # ==========================================================================

    def _stage_x_fine(self):
        """
        LiDAR 正前方测距闭环，慢速微调直到距离在容差内。
        对应原始 stage 5。
        """
        rospy.loginfo("[precision_park] X_FINE: target=%.2f tol=%.3f",
                      self.target_distance, self.distance_tolerance)
        rate = rospy.Rate(10)
        slow_speed = self.linear_speed * 0.5

        while not rospy.is_shutdown():
            scan = self._get_scan(0.3)
            if scan is None:
                rospy.sleep(0.1)
                continue

            # 正前方 3 束取最小
            n = len(scan.ranges)
            center = n // 2
            valid = [scan.ranges[i] for i in range(center - 1, center + 2)
                    if 0 < scan.ranges[i] < float('inf')]

            if not valid:
                rospy.logwarn("[precision_park] X_FINE: no valid front distance")
                break

            dist = min(valid)

            if dist <= self.target_distance + self.distance_tolerance:
                rospy.loginfo("[precision_park] X_FINE done: %.3f m in tolerance", dist)
                break

            # 计算剩余距离，限速
            remaining = dist - self.target_distance
            speed = min(slow_speed, max(0.05, remaining * 0.5))
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())

    # ==========================================================================
    # 辅助方法
    # ==========================================================================

    def _get_scan(self, timeout=1.0):
        """获取最新 LaserScan，带超时。"""
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
        """
        从激光扫描中提取板子角度范围内的有效点。

        返回:
            (points_list, B_point) — B_point 是板子中心的最近点
            (None, None)          — 有效点不足
        """
        board_center_deg = (self.board_angle_left + self.board_angle_right) / 2.0
        board_half_width = abs(self.board_angle_right - self.board_angle_left) / 2.0

        points = []
        B_point = None
        B_dist = float('inf')

        step = 0.5  # 采样步长（度）
        angles = np.arange(-board_half_width, board_half_width + step, step)

        for deg_offset in angles:
            angle_deg = board_center_deg + deg_offset
            angle_rad = math.radians(angle_deg)
            idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))

            if 0 <= idx < len(scan.ranges):
                r = scan.ranges[idx]
                if scan.range_min <= r <= scan.range_max:
                    x = r * math.cos(angle_rad)
                    y = r * math.sin(angle_rad)
                    points.append((x, y))
                    # 最接近中心角度的最近点为 B
                    if abs(deg_offset) < 0.5 and r < B_dist:
                        B_dist = r
                        B_point = (x, y)

        if len(points) < 2:
            return None, None

        return points, B_point

    def _get_board_center_distance(self, scan):
        """获取板子中心角度上的最近距离。"""
        board_center_deg = (self.board_angle_left + self.board_angle_right) / 2.0
        angle_rad = math.radians(board_center_deg)
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))

        # 取中心附近 3 束的最小距离
        dists = []
        for di in [-1, 0, 1]:
            i = idx + di
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if scan.range_min <= r <= scan.range_max:
                    dists.append(r)

        return min(dists) if dists else None

    def _rotate(self, angle_degrees, speed):
        """原地旋转指定角度（开环，按时间算）。"""
        if abs(angle_degrees) < 0.3:
            return

        angle_rad = math.radians(angle_degrees)
        duration = abs(angle_rad) / abs(speed)
        angular_speed = abs(speed) * (1 if angle_degrees > 0 else -1)

        rospy.loginfo("  rotating %.1f deg @ %.2f rad/s (%.2f s)",
                      angle_degrees, angular_speed, duration)

        cmd = Twist()
        cmd.angular.z = angular_speed
        start = rospy.Time.now()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and (rospy.Time.now() - start) < rospy.Duration(duration):
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.sleep(0.2)


# ==============================================================================
# 主入口
# ==============================================================================
if __name__ == "__main__":
    try:
        PrecisionPark()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
