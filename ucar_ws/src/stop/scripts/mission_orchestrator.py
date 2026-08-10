#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mission_orchestrator.py —— 全流程任务编排（学习 item_finder.py 逻辑）

输入格式: "食品，苹果；电子，手机"
    - 真实车间: 食品，货物: 苹果
    - 仿真车间: 电子，货物: 手机

双阶段:
    Phase 1 ("real"):  找真实车间 → PCA停车 → 播报 "已将苹果放入食品加工车间"
    Phase 2 ("sim"):   找仿真车间 → PCA停车 → 播报 "仿真任务已完成，已将手机放入电子产品生产车间"

双指针优化: Phase 1 遍历时若也看到了仿真车间，记录位置，Phase 2 直接跳转

架构（回调驱动状态机，同 item_finder.py）:
    - img_callback:    到达航点后发布图像给 OCR
    - boxes_callback:  驱动 stage 0/1/3/4
    - goal_callback:   /move_base/result 导航状态
    - LidarCallback:   stage 2 PCA + stage 5 X轴微调

依赖:
    - ucar_1navigation.launch (move_base + TEB 动态避障 + AMCL)
    - ocr_native_node (RKNN PP-OCR)
    - speech_command/scripts/tts_http.py (讯飞 TTS)

运行方式:
    roslaunch stop mission.launch task_input:="食品，苹果；电子，手机"
"""

import math
import os
import sys
import subprocess
import numpy as np
import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
from std_srvs.srv import Empty
from stop.msg import BoundingBoxes

# ==============================================================================
# 全局变量（同 item_finder.py 风格）
# ==============================================================================

# 状态机
search_item_stage = 0                         # 0-6
is_searching_item = False
isNavPointReached = False
isBoxesCallbackFinished = False

# 双阶段
current_phase = "real"                        # "real" | "sim"
sim_point_index = -1                          # 双指针优化：Phase 1 中记录仿真车间所在索引

# 任务输入
real_keyword = "食品"                         # OCR 匹配关键字
real_cargo = ""                               # 货物名（播报用）
real_warehouse = "食品加工车间"               # 完整车间名（播报用）
sim_keyword = "电子"
sim_cargo = ""
sim_warehouse = "电子产品生产车间"

# 航点索引 & 旋转计数
current_point_index = 0
rotate_num = 0
rotate_num_threshold = 6

# 视觉
camera_angle_rad = 0.0
camera_deg = 0.0
camera_deg_left = 0.0
camera_deg_right = 0.0

# LiDAR
lidar_processing_flag = False
board_center_to_park_dist = 0.20

# 微调
is_x_aligned = False
is_y_aligned = False
dist_forward_item = 0.7
x_align_tolerance = 0.50
y_align_tolerance = 12
mission_done_called = False          # 防重复播报
stage2_creeping = False              # Stage 2 cmd_vel 慢速逼近标志

# 航点
FIND_POINTS_LIST = [
    (-0.812845, -2.44196, 0.0, 1.0),
    (0.771455, -2.44196, 0.0, 1.0),
    (1.7843, -2.43094, 0.0, 1.0),
]

# 摄像头内参
camera_matrix = np.array([
    [417.0233117551048, 0, 317.6164317961776],
    [0, 417.361018373727, 222.3170064096779],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([-0.3183327957970696, 0.0940668277282494,
                        0.003040644254145817, -0.0008593433351979479,
                        0], dtype=np.float32)

lidar_offset_deg = 1.5

# 导航
navigation_failed_count = 0
max_navigation_retries = 2

# ROS 发布者（全局）
goal_pub = None
img_pub = None
cmd_vel_pub = None
result_pub = None

# ==============================================================================
# TTS —— 调用 speech_command/scripts/tts_http.py
# ==============================================================================

# tts_http.py 相对于本脚本的路径（适配机器人部署路径）
_TTS_SCRIPT_CANDIDATES = [
    os.path.join(os.path.dirname(__file__), '../../speech_command/scripts/tts_http.py'),
    '/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py',
]


def _find_tts_script():
    for p in _TTS_SCRIPT_CANDIDATES:
        if os.path.exists(p):
            return p
    return _TTS_SCRIPT_CANDIDATES[-1]  # fallback


def speak(text):
    """同步调用 tts_http.py 播报（阻塞，播完才返回）"""
    script = _find_tts_script()
    rospy.loginfo("[TTS] %s", text)
    try:
        subprocess.run(
            ['python3', script, text],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=30
        )
    except Exception as e:
        rospy.logwarn("[TTS] Failed: %s", e)


# ==============================================================================
# 关键字匹配 & 车间名映射
# ==============================================================================

WAREHOUSE_MAP = {
    "食品": "食品加工车间",
    "电子": "电子产品生产车间",
    "日用": "日用品加工车间",
    "电子产品": "电子产品生产车间",
}


def get_warehouse(keyword):
    return WAREHOUSE_MAP.get(keyword, keyword + "车间")


def match_keyword(ocr_texts, keyword):
    """检查 OCR 文本列表中是否有包含 keyword 的项（同 item_finder）"""
    for t in ocr_texts:
        if keyword in t:
            return t
    return ""


def match_relaxed(ocr_texts):
    """微调阶段宽松匹配：包含车间相关字眼即可"""
    for t in ocr_texts:
        if any(c in t for c in "生产车间加工车间"):
            return t
    return ""


# ==============================================================================
# PCA 工具函数
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


def calculate_point_C(Bx, By, dx_dir, dy_dir, BC_length):
    dir_len = math.hypot(dx_dir, dy_dir)
    if dir_len == 0:
        raise ValueError("Zero vector")
    ux, uy = dx_dir / dir_len, dy_dir / dir_len
    return (Bx - ux * BC_length, By - uy * BC_length)


def vector_to_angle(nx, ny):
    if math.isclose(nx, 0.0, abs_tol=1e-9) and math.isclose(ny, 0.0, abs_tol=1e-9):
        raise ValueError("Zero vector")
    return math.degrees(math.atan2(ny, nx))


# ==============================================================================
# 底盘控制
# ==============================================================================

def rotate_speed(angle_degrees, speed):
    if math.isclose(angle_degrees, 0.0, abs_tol=1e-3):
        return
    angle_rad = math.radians(angle_degrees)
    duration = abs(angle_rad) / abs(speed)
    angular = abs(speed) * (1 if angle_degrees > 0 else -1)
    rospy.loginfo("  Rotating %.1fdeg @ %.2f rad/s", angle_degrees, angular)
    cmd = Twist()
    cmd.angular.z = angular
    start = rospy.Time.now()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and (rospy.Time.now() - start) < rospy.Duration(duration):
        cmd_vel_pub.publish(cmd)
        rate.sleep()
    cmd_vel_pub.publish(Twist())
    rospy.sleep(0.2)


def find_item_rotate():
    rotate_speed(72, 1.5)
    rospy.sleep(0.5)
    rospy.loginfo("  Rotation complete")


# ==============================================================================
# 导航
# ==============================================================================

def go_point(x, y, qz, qw):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw
    goal_pub.publish(goal)
    rospy.loginfo("  Sent goal: (%.3f, %.3f)", x, y)


def go_to_find_point():
    """前往当前阶段的下一个航点"""
    global current_point_index, isNavPointReached
    if current_point_index < len(FIND_POINTS_LIST):
        x, y, qz, qw = FIND_POINTS_LIST[current_point_index]
        rospy.loginfo("--> [%s] Go to waypoint %d/%d: (%.3f, %.3f)",
                      current_phase, current_point_index + 1,
                      len(FIND_POINTS_LIST), x, y)

        # 检查距离：太近（<0.3m）直接跳过导航，触发 OCR 扫描
        try:
            import tf2_ros as _tf2r
            _buf = _tf2r.Buffer(rospy.Duration(2.0))
            _lis = _tf2r.TransformListener(_buf)
            rospy.sleep(0.3)
            tf = _buf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            dist = math.hypot(x - rx, y - ry)
            if dist < 0.3:
                rospy.loginfo("  Already at waypoint (dist=%.2f < 0.3), skip nav", dist)
                current_point_index += 1
                isNavPointReached = True
                return
        except Exception:
            pass

        go_point(x, y, qz, qw)
    else:
        rospy.logwarn("[%s] All waypoints exhausted!", current_phase)
        handle_phase_exhausted()


def handle_phase_exhausted():
    """当前阶段所有航点遍历完毕"""
    global current_phase, is_searching_item
    if current_phase == "real":
        rospy.logwarn("[Phase 1] Real workshop not found, trying Phase 2")
        switch_to_phase2()
    else:
        rospy.logerr("[Phase 2] Sim workshop not found, mission failed")
        is_searching_item = False
        result_pub.publish(String(data="failed:not_found"))
        rospy.loginfo("未找到目标车间")


# ==============================================================================
# Phase 切换
# ==============================================================================

def switch_to_phase2():
    """Phase 1 → Phase 2 切换（同 item_finder.py play_real_item_sound）"""
    global current_phase, current_point_index, sim_point_index
    global search_item_stage, rotate_num, is_x_aligned, is_y_aligned
    global is_searching_item, mission_done_called, stage2_creeping

    mission_done_called = False
    stage2_creeping = False
    current_phase = "sim"

    # 双指针优化
    if sim_point_index >= 0:
        rospy.loginfo("[双指针] 仿真车间已在索引 %d 记录，直接跳转", sim_point_index)
        current_point_index = sim_point_index
    else:
        rospy.loginfo("[双指针] 仿真车间未记录，从当前位置继续")

    # 重置状态
    search_item_stage = 0
    rotate_num = 0
    is_x_aligned = False
    is_y_aligned = False
    is_searching_item = True
    rospy.loginfo("[Phase 2] Start searching: %s", sim_keyword)

    # ---- 先倒退离开墙面，避免全局规划失败 ----
    rospy.loginfo("[Phase 2] Backing up 0.5m to clear wall...")
    cmd = Twist()
    cmd.linear.x = -0.12
    start = rospy.Time.now()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and (rospy.Time.now() - start) < rospy.Duration(4.5):
        cmd_vel_pub.publish(cmd)
        rate.sleep()
    cmd_vel_pub.publish(Twist())
    rospy.sleep(0.3)

    # ---- 掉头180°面朝空旷方向，避免TEB振荡 ----
    rospy.loginfo("[Phase 2] Turning 180° away from wall...")
    rotate_speed(180, 1.5)
    rospy.sleep(0.3)

    try:
        rospy.wait_for_service('/move_base/clear_costmaps', 1.0)
        rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
        rospy.loginfo("[Phase 2] Costmap cleared")
    except Exception:
        pass

    go_to_find_point()


# ==============================================================================
# 任务完成处理
# ==============================================================================

def mission_done():
    """当前阶段完成（防重入）"""
    global search_item_stage, is_searching_item, current_phase
    global mission_done_called

    if mission_done_called:
        return
    mission_done_called = True

    search_item_stage = 6
    is_searching_item = False
    cmd_vel_pub.publish(Twist())

    if current_phase == "real":
        # Phase 1 完成 → 播报实车语音 → 切 Phase 2
        text = "已将{}放入{}".format(real_cargo, real_warehouse)
        rospy.loginfo("==== Phase 1 DONE: %s ====", text)
        result_pub.publish(String(data="phase1_done"))
        speak(text)
        switch_to_phase2()
    else:
        # Phase 2 完成 → 播报仿真语音 → 任务结束
        text = "仿真任务已完成，已将{}放入{}".format(sim_cargo, sim_warehouse)
        rospy.loginfo("==== Phase 2 DONE: %s ====", text)
        result_pub.publish(String(data="done"))
        speak(text)


# ==============================================================================
# ROS 回调：图像转发
# ==============================================================================

def img_callback(msg):
    global isNavPointReached, isBoxesCallbackFinished, is_searching_item
    if not is_searching_item:
        return
    if isNavPointReached or isBoxesCallbackFinished:
        if isNavPointReached:
            rospy.loginfo("[img_cb] Waypoint reached, publish image")
            isNavPointReached = False
        elif isBoxesCallbackFinished:
            rospy.loginfo("[img_cb] Stage done, publish image")
            isBoxesCallbackFinished = False
        img_pub.publish(msg)


# ==============================================================================
# ROS 回调：OCR 结果驱动状态机
# ==============================================================================

def boxes_callback(msg):
    global rotate_num, rotate_num_threshold, camera_angle_rad, camera_deg
    global camera_deg_left, camera_deg_right, lidar_processing_flag
    global is_y_aligned, isBoxesCallbackFinished, search_item_stage
    global is_searching_item, y_align_tolerance
    global current_point_index, current_phase, sim_point_index
    global stage2_creeping

    if search_item_stage == 6:
        return

    if stage2_creeping:
        return

    # ---- 提取 OCR 文本和框 ----
    slist_tool = []
    slist_rect = []
    for box in msg.bounding_boxes:
        slist_tool.append(box.Class.strip())
        slist_rect.append([box.xmin, box.ymin, box.xmax, box.ymax])

    # ---- 确定当前阶段的目标关键字 ----
    if current_phase == "real":
        cur_kw = real_keyword
        sim_kw = sim_keyword
    else:
        cur_kw = sim_keyword
        sim_kw = None

    # ---- 匹配 ----
    target_class = match_keyword(slist_tool, cur_kw)
    if target_class == "" and search_item_stage in [3, 4]:
        target_class = match_relaxed(slist_tool)

    rospy.loginfo("[boxes_cb][%s] OCR=%s, matched=%s, stage=%d",
                  current_phase, slist_tool, target_class, search_item_stage)

    # ---- 双指针优化：Phase 1 时间步检查仿真车间 ----
    if current_phase == "real" and sim_point_index < 0 and sim_kw:
        sim_match = match_keyword(slist_tool, sim_kw)
        if sim_match:
            sim_point_index = current_point_index
            rospy.loginfo("[双指针] Phase 1 在航点 %d 发现仿真车间: %s",
                          sim_point_index, sim_match)

    # ---- 找到目标 → 计算几何信息 ----
    if target_class != "":
        idx = slist_tool.index(target_class)
        bx = slist_rect[idx]

        cx = (bx[0] + bx[2]) / 2.0
        cy = (bx[1] + bx[3]) / 2.0
        pc = np.array([[[cx, cy]]], dtype=np.float32)
        ud = cv2.undistortPoints(pc, camera_matrix, dist_coeffs, P=camera_matrix)[0][0]
        x_norm = (ud[0] - camera_matrix[0, 2]) / camera_matrix[0, 0]
        camera_angle_rad = math.atan(x_norm)
        camera_deg = math.degrees(camera_angle_rad)

        if search_item_stage in [0, 1, 2, 3]:
            xl, xr = bx[0], bx[2]
            pl = np.array([[[xl, cy]]], dtype=np.float32)
            ul = cv2.undistortPoints(pl, camera_matrix, dist_coeffs, P=camera_matrix)[0][0]
            camera_deg_left = math.degrees(
                math.atan((ul[0] - camera_matrix[0, 2]) / camera_matrix[0, 0]))
            pr = np.array([[[xr, cy]]], dtype=np.float32)
            ur = cv2.undistortPoints(pr, camera_matrix, dist_coeffs, P=camera_matrix)[0][0]
            camera_deg_right = math.degrees(
                math.atan((ur[0] - camera_matrix[0, 2]) / camera_matrix[0, 0]))
            rospy.loginfo("  Board angle: L=%.1f R=%.1f center=%.1f",
                          camera_deg_left, camera_deg_right, camera_deg)

        # ==== Stage 0: 首次发现 → 旋转对准 ====
        if search_item_stage == 0:
            rospy.loginfo("[Stage 0] First sight, align to board")
            rotate_speed(camera_deg, 0.5)
            rospy.sleep(0.5)
            search_item_stage = 1
            isBoxesCallbackFinished = True
            return

        # ==== Stage 1: 二次确认 → 启用 LiDAR ====
        elif search_item_stage == 1:
            rospy.loginfo("[Stage 1] Second confirm, enable LiDAR")
            lidar_processing_flag = True
            search_item_stage = 2
            return

        # ==== Stage 3: LiDAR 旋转对正 ====
        elif search_item_stage == 3:
            rospy.loginfo("[Stage 3] LiDAR yaw align")
            lidar_processing_flag = True

        # ==== Stage 4: Y 轴横移微调 ====
        elif search_item_stage == 4:
            rospy.loginfo("[Stage 4] Y lateral: camera_deg=%.2f", camera_deg)
            if abs(camera_deg) >= y_align_tolerance:
                cmd = Twist()
                cmd.linear.y = 0.3 if camera_deg > 0 else -0.3
                cmd_vel_pub.publish(cmd)
            else:
                rospy.loginfo("  Y aligned (tol=%d)", y_align_tolerance)
                is_y_aligned = True
                search_item_stage = 5
                return
            isBoxesCallbackFinished = True

    else:
        # ---- 未找到 ----
        if search_item_stage in [3, 4]:
            rospy.loginfo("[boxes_cb] Board not visible (too close), done")
            mission_done()
        else:
            rospy.loginfo("[boxes_cb] Not found, continue")
            search_item_stage = 0
            rotate_num += 1
            if rotate_num <= rotate_num_threshold:
                rospy.loginfo("  Rotate %d/%d", rotate_num, rotate_num_threshold)
                find_item_rotate()
                isBoxesCallbackFinished = True
            else:
                rospy.loginfo("  Exhausted, next waypoint")
                go_to_find_point()
                rotate_num = 0


# ==============================================================================
# ROS 回调：导航结果
# ==============================================================================

def goal_callback(msg):
    global current_point_index, isNavPointReached, navigation_failed_count
    global max_navigation_retries, is_searching_item, search_item_stage

    if not is_searching_item:
        return

    if msg.status.status == 3:                    # SUCCEEDED
        if search_item_stage != 2:
            current_point_index += 1
        # 清除代价地图
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', 1.0)
            rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
            rospy.loginfo("  Costmap cleared")
        except Exception:
            pass
        rospy.sleep(0.5)
        if search_item_stage == 2:
            search_item_stage = 3
            rospy.loginfo("  Arrived at pickup point")
        isNavPointReached = True

    elif msg.status.status == 4:                  # ABORTED
        rospy.logwarn("  Nav ABORTED")
        navigation_failed_count += 1
        if navigation_failed_count >= max_navigation_retries:
            navigation_failed_count = 0
            go_to_find_point()
    elif msg.status.status == 5:                  # REJECTED
        rospy.logwarn("  Nav REJECTED")
        go_to_find_point()


# ==============================================================================
# ROS 回调：LiDAR
# ==============================================================================

def LidarCallback(msg):
    global lidar_processing_flag, lidar_offset_deg, camera_angle_rad
    global is_x_aligned, dist_forward_item, board_center_to_park_dist
    global camera_deg_left, camera_deg_right, isBoxesCallbackFinished
    global search_item_stage, x_align_tolerance, is_searching_item
    global stage2_creeping, rotate_num

    if not is_searching_item:
        return

    # ==== Stage 2: cmd_vel 慢速逼近（不用 move_base，避免全局规划失败）====
    if stage2_creeping:
        n = len(msg.ranges)
        c = n // 2
        front_dists = [msg.ranges[i] for i in range(c - 1, c + 2)
                       if 0 < msg.ranges[i] < float('inf')]
        if front_dists:
            dist_forward_item = min(front_dists)

        if dist_forward_item > board_center_to_park_dist + 0.05:
            cmd = Twist()
            cmd.linear.x = 0.10
            cmd_vel_pub.publish(cmd)
        else:
            cmd_vel_pub.publish(Twist())
            rospy.loginfo("[Stage 2] Approach done (dist=%.3f)", dist_forward_item)
            stage2_creeping = False
            search_item_stage = 3
            isBoxesCallbackFinished = True
        return

    # ==== Stage 5: X 轴直行微调 ====
    if search_item_stage == 5:
        n = len(msg.ranges)
        c = n // 2
        front_dists = [msg.ranges[i] for i in range(c - 1, c + 2)
                       if 0 < msg.ranges[i] < float('inf')]
        if front_dists:
            dist_forward_item = min(front_dists)
        cmd = Twist()
        if dist_forward_item >= x_align_tolerance:
            cmd.linear.x = 0.13
            cmd_vel_pub.publish(cmd)
        else:
            rospy.loginfo("[Stage 5] X aligned (dist=%.3f < %.2f)", dist_forward_item, x_align_tolerance)
            cmd_vel_pub.publish(Twist())
            is_x_aligned = True
            mission_done()

    # ==== Stage 2/3: PCA 处理 ====
    if not lidar_processing_flag:
        return

    target_angle_rad = camera_angle_rad + math.radians(lidar_offset_deg)
    camera_range = abs(camera_deg_left - camera_deg_right) / 2.0
    angles_deg = range(-int(camera_range), int(camera_range) + 1)

    valid_points = []
    B_point = None
    for deg in angles_deg:
        cur_rad = target_angle_rad + math.radians(deg)
        idx = int(round((cur_rad - msg.angle_min) / msg.angle_increment))
        if 0 <= idx < len(msg.ranges):
            r = msg.ranges[idx]
            if msg.range_min <= r <= msg.range_max:
                x = r * math.cos(cur_rad)
                y = r * math.sin(cur_rad)
                valid_points.append((x, y))
                if deg == 0:
                    B_point = (x, y)

    if len(valid_points) < 2 or B_point is None:
        rospy.logwarn("[Lidar] Insufficient points (%d)", len(valid_points))
        if search_item_stage == 2:
            # PCA 点数不够也往前走——板子斜着但只要在视野里就逼近
            rospy.logwarn("[Stage 2] PCA failed, approach blindly anyway")
            lidar_processing_flag = False
            stage2_creeping = True
        else:
            rospy.logwarn("[Lidar] Skip yaw, go to Y fine")
            search_item_stage = 4
            isBoxesCallbackFinished = True
            lidar_processing_flag = False
        return

    # ==== Stage 2: PCA 确认方向 → 直接往前走（不旋转，斜着也直走）====
    if search_item_stage == 2:
        (nx, ny), _ = calculate_external_normal(*valid_points)
        rospy.loginfo("[Stage 2] PCA: normal=(%.3f,%.3f) B=(%.3f,%.3f)",
                      nx, ny, B_point[0], B_point[1])
        rospy.loginfo("[Stage 2] Approach directly (target=%.2fm)", board_center_to_park_dist)

        cmd_vel_pub.publish(Twist())
        rospy.sleep(0.2)

        stage2_creeping = True
        lidar_processing_flag = False

    # ==== Stage 3: PCA 旋转对正 ====
    elif search_item_stage == 3:
        (nx, ny), _ = calculate_external_normal(*valid_points)
        angle = vector_to_angle(nx, ny)
        rospy.loginfo("[Stage 3] PCA align: normal=(%.3f,%.3f) angle=%.2f", nx, ny, angle)
        rotate_speed(angle, 0.3)
        search_item_stage = 4
        rospy.sleep(0.5)
        isBoxesCallbackFinished = True

    lidar_processing_flag = False


# ==============================================================================
# 启动
# ==============================================================================

def start_mission():
    """重置状态，启动 Phase 1"""
    global is_searching_item, current_phase, current_point_index, sim_point_index
    global rotate_num, search_item_stage, navigation_failed_count
    global is_x_aligned, is_y_aligned, mission_done_called, stage2_creeping

    current_phase = "real"
    current_point_index = 0
    sim_point_index = -1
    rotate_num = 0
    search_item_stage = 0
    navigation_failed_count = 0
    is_x_aligned = False
    is_y_aligned = False
    mission_done_called = False
    stage2_creeping = False
    is_searching_item = True

    rospy.loginfo("=" * 50)
    rospy.loginfo("  MISSION START")
    rospy.loginfo("  Phase 1 (real): %s -> %s, cargo=%s",
                  real_keyword, real_warehouse, real_cargo)
    rospy.loginfo("  Phase 2 (sim):  %s -> %s, cargo=%s",
                  sim_keyword, sim_warehouse, sim_cargo)
    rospy.loginfo("=" * 50)

    go_to_find_point()


# ==============================================================================
# 主入口
# ==============================================================================

if __name__ == "__main__":
    rospy.init_node("mission_orchestrator")

    # ---- 解析 task_input ----
    task_input = rospy.get_param("~task_input", "食品，苹果；电子，手机")
    parts = [p.strip() for p in task_input.replace("；", ";").split(";")]
    if len(parts) >= 1:
        rp = [p.strip() for p in parts[0].replace("，", ",").split(",")]
        real_keyword = rp[0] if len(rp) >= 1 else "食品"
        real_cargo = rp[1] if len(rp) >= 2 else ""
        real_warehouse = get_warehouse(real_keyword)
    if len(parts) >= 2:
        sp = [p.strip() for p in parts[1].replace("，", ",").split(",")]
        sim_keyword = sp[0] if len(sp) >= 1 else "电子"
        sim_cargo = sp[1] if len(sp) >= 2 else ""
        sim_warehouse = get_warehouse(sim_keyword)

    # ---- 其他参数 ----
    rotate_num_threshold = rospy.get_param("~max_rotations", 6)
    board_center_to_park_dist = rospy.get_param("~target_distance", 0.20)
    x_align_tolerance = rospy.get_param("~x_align_tolerance", 0.50)
    y_align_tolerance = rospy.get_param("~y_align_tolerance", 12)
    auto_start_delay = rospy.get_param("~auto_start_delay", 3.0)

    rospy.loginfo("=" * 50)
    rospy.loginfo("  Mission Orchestrator v2")
    rospy.loginfo("  task_input: %s", task_input)
    rospy.loginfo("  Real: kw=%s cargo=%s warehouse=%s", real_keyword, real_cargo, real_warehouse)
    rospy.loginfo("  Sim:  kw=%s cargo=%s warehouse=%s", sim_keyword, sim_cargo, sim_warehouse)
    rospy.loginfo("  Waypoints: %d", len(FIND_POINTS_LIST))
    rospy.loginfo("=" * 50)

    # ---- 发布者 ----
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    img_pub = rospy.Publisher('/perception/detect_image', Image, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    result_pub = rospy.Publisher('/mission/result', String, queue_size=1)

    # ---- 订阅者 ----
    rospy.Subscriber('/usb_cam/image_raw', Image, img_callback, queue_size=1)
    rospy.Subscriber('/perception/bounding_boxes', BoundingBoxes, boxes_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_callback)
    rospy.Subscriber('/scan', LaserScan, LidarCallback, queue_size=1)

    # ---- 自动设置初始位姿 ----
    from geometry_msgs.msg import PoseWithCovarianceStamped
    init_x  = rospy.get_param("~initial_pose_x", 0.0)
    init_y  = rospy.get_param("~initial_pose_y", 0.0)
    init_yaw = rospy.get_param("~initial_pose_yaw", 0.0)

    if init_x != 0.0 or init_y != 0.0:
        rospy.loginfo("[mission] Auto-setting initial pose: (%.3f, %.3f, yaw=%.2f)",
                      init_x, init_y, init_yaw)

        # 等 AMCL 和 move_base 先初始化
        rospy.sleep(auto_start_delay)

        init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        rospy.sleep(0.5)

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x = init_x
        pose.pose.pose.position.y = init_y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.z = math.sin(init_yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(init_yaw / 2.0)
        # 默认协方差（相信这个初始位姿比较准确）
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685

        init_pub.publish(pose)
        rospy.loginfo("[mission] Initial pose published!")
    else:
        rospy.loginfo("[mission] No initial pose configured, set in RViz if needed")

    # ---- 等待 TF 就绪后自动启动 ----
    def _wait_and_start(event=None):
        """等待 /map 坐标系可用后启动任务"""
        import tf2_ros as _tf2r
        _tf_buf = _tf2r.Buffer(rospy.Duration(5.0))
        _tf_lis = _tf2r.TransformListener(_tf_buf)
        rospy.loginfo("[mission] Waiting for /map frame from AMCL...")

        waited = 0
        while not rospy.is_shutdown():
            try:
                _tf_buf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
                rospy.loginfo("[mission] TF ready! (waited %ds)", waited)
                start_mission()
                return
            except Exception:
                if waited % 10 == 0:
                    rospy.loginfo("[mission] Waiting for /map... (%ds elapsed)", waited)
                rospy.sleep(1.0)
                waited += 1

    # 延迟足够时间让 AMCL 收到 initialpose 后再开始 TF 检查
    extra_delay = max(auto_start_delay, 5.0) if (init_x != 0.0 or init_y != 0.0) else max(auto_start_delay, 1.0)
    rospy.Timer(rospy.Duration(extra_delay), _wait_and_start, oneshot=True)

    rospy.spin()

