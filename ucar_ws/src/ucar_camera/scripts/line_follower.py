#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
line_follower.py —— 视觉巡线节点

运行方式:
    rosrun ucar_camera line_follower.py

功能：
    基于 OpenCV 图像处理的赛道巡线节点，通过 PID 控制底盘沿赛道边缘行驶。
    包含以下核心模块：

    1. 八邻域赛道边缘追踪 (EightNeighborhoodTracker)
       - 在二值化图像中搜索左右赛道边缘
       - 用于识别赛道拐点（十字路口、T 字路口等特殊点）

    2. 巡线主控 (LineFollowerNode)
       - 初始车头对正 → 正常巡线 → 拐点识别与旋转 → 倒计时 → 终点停车线检测
       - 支持 left / right / straight 三种巡线模式

    巡线流程：
       收到 /start_follow 指令 → 等待 8s →
       初始车头对正 → 八邻域倒计时 → 拐点检测与旋转 →
       倒计时等待 → 终点停车线检测（前线→后线）→ 停车播报

ROS 接口：
    发布: /cmd_vel (Twist)          —— 底盘速度控制
    发布: /finish_follow (Bool)     —— 巡线完成信号
    订阅: /usb_cam/image_raw (Image) —— 摄像头图像
    订阅: /start_follow (String)     —— 启动指令（"left" / "right" / "straight"）
"""

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
import wave
import os
import subprocess


# ==============================================================================
# 八邻域赛道边缘追踪器
# ==============================================================================

class EightNeighborhoodTracker:
    """
    八邻域赛道边缘追踪器。

    在二值化的赛道图像中，分别沿左边缘（逆时针搜索）和右边缘（顺时针搜索）
    爬取边界点序列，用于检测赛道拐点。

    参数:
        max_points: 单侧边界最大追踪点数
    """

    def __init__(self, max_points=200):
        self.max_points = max_points

        # 左边缘搜索方向（8邻域，逆时针序）
        self.seeds_L = [[0, 1], [1, 1], [1, 0], [1, -1],
                        [0, -1], [-1, -1], [-1, 0], [-1, 1]]

        # 右边缘搜索方向（8邻域，顺时针序）
        self.seeds_R = [[0, 1], [-1, 1], [-1, 0], [-1, -1],
                        [0, -1], [1, -1], [1, 0], [1, 1]]

    def _get_initial_point(self, image, initial_row):
        """
        在指定行搜索左右边缘的初始追踪起点。

        搜索策略：从图像中间分别向左右两侧扫描，
        寻找二值图像中黑→白的跳变点（0→255）。

        返回:
            (是否找到左右起点, 左起点坐标 [x,y], 右起点坐标 [x,y])
        """
        width = image.shape[1]
        initial_L = [0, 0]
        initial_R = [0, 0]
        found_L, found_R = False, False

        # 由中间向左侧寻找左边缘起点
        for i in range(width // 2, 1, -1):
            if image[initial_row, i] == 0 and image[initial_row, i - 1] == 255:
                initial_L = [i, initial_row]
                found_L = True
                break

        # 由中间向右侧寻找右边缘起点
        for i in range(width // 2, width - 2):
            if image[initial_row, i] == 0 and image[initial_row, i + 1] == 255:
                initial_R = [i, initial_row]
                found_R = True
                break

        return (found_L and found_R), initial_L, initial_R

    def _crawl_edges(self, image, start_L, start_R, deal_low, deal_high):
        """
        从给定起点沿赛道边缘爬取边界点序列。

        对左右边缘分别跟踪，记录每个位置的 8 邻域方向编号。
        退出条件：
            - 超出 deal_high ~ deal_low 的 ROI 范围
            - 左右边缘相遇（间距 < 2 像素）
            - 最近 3 帧停滞不动
            - 达到最大追踪点数

        返回:
            dict: {
                'points_L': 左边缘点集 [(x,y), ...],
                'points_R': 右边缘点集 [(x,y), ...],
                'dir_L':    左边缘各步的方向编号,
                'dir_R':    右边缘各步的方向编号,
                'stats':    {'left_dirs': {dir: count}, 'right_dirs': {dir: count}},
                'meet':     左右边缘是否相遇
            }
        """
        height, width = image.shape
        if deal_high > deal_low:
            deal_high, deal_low = deal_low, deal_high

        points_L = np.zeros((self.max_points, 2), dtype=np.uint16)
        points_R = np.zeros((self.max_points, 2), dtype=np.uint16)
        dir_L = np.zeros(self.max_points, dtype=np.uint16)
        dir_R = np.zeros(self.max_points, dtype=np.uint16)

        count_L, count_R = 0, 0
        current_L = start_L.copy()
        current_R = start_R.copy()

        active_L, active_R = True, True
        left_and_right_meet = False

        for _ in range(self.max_points):
            # ---- 左线追踪 ----
            if active_L:
                if not (deal_high <= current_L[1] <= deal_low):
                    active_L = False
                else:
                    # 构建左边缘的 8 邻域搜索窗口
                    search_fields_L = []
                    for dx, dy in self.seeds_L:
                        x, y = current_L[0] + dx, current_L[1] + dy
                        if 0 <= x < width and 0 <= y < height:
                            search_fields_L.append([x, y])
                        else:
                            search_fields_L.append([0, 0])

                    if count_L < self.max_points:
                        points_L[count_L] = current_L
                    count_L += 1

                    # 在 8 个方向上寻找黑→白跳变点
                    candidates_L = []
                    for i in range(8):
                        ni = (i + 1) % 8
                        if (0 <= search_fields_L[i][0] < width and
                            0 <= search_fields_L[i][1] < height and
                            0 <= search_fields_L[ni][0] < width and
                            0 <= search_fields_L[ni][1] < height):
                            if (image[search_fields_L[i][1], search_fields_L[i][0]] == 0 and
                                image[search_fields_L[ni][1], search_fields_L[ni][0]] == 255):
                                candidates_L.append(search_fields_L[i])
                                if count_L - 1 < len(dir_L):
                                    dir_L[count_L - 1] = i

                    if candidates_L:
                        current_L = min(candidates_L, key=lambda p: p[1])

            # ---- 右线追踪 ----
            if active_R:
                if not (deal_high <= current_R[1] <= deal_low):
                    active_R = False
                else:
                    search_fields_R = []
                    for dx, dy in self.seeds_R:
                        x, y = current_R[0] + dx, current_R[1] + dy
                        if 0 <= x < width and 0 <= y < height:
                            search_fields_R.append([x, y])
                        else:
                            search_fields_R.append([0, 0])

                    if count_R < self.max_points:
                        points_R[count_R] = current_R
                    count_R += 1

                    candidates_R = []
                    for i in range(8):
                        ni = (i + 1) % 8
                        if (0 <= search_fields_R[i][0] < width and
                            0 <= search_fields_R[i][1] < height and
                            0 <= search_fields_R[ni][0] < width and
                            0 <= search_fields_R[ni][1] < height):
                            if (image[search_fields_R[i][1], search_fields_R[i][0]] == 0 and
                                image[search_fields_R[ni][1], search_fields_R[ni][0]] == 255):
                                candidates_R.append(search_fields_R[i])
                                if count_R - 1 < len(dir_R):
                                    dir_R[count_R - 1] = i

                    if candidates_R:
                        current_R = min(candidates_R, key=lambda p: p[1])

            # ---- 退出条件检查 ----
            if not active_L and not active_R:
                break

            # 最近 3 帧停滞不动 → 退出
            if count_R >= 3 and count_L >= 3:
                last_R = points_R[count_R-3:count_R]
                last_L = points_L[count_L-3:count_L-1]
                if (all(np.array_equal(last_R[0], p) for p in last_R[1:]) or
                    all(np.array_equal(last_L[0], p) for p in last_L[1:])):
                    break

            # 左右边缘相遇 → 退出
            if abs(current_R[0] - current_L[0]) < 2 and abs(current_R[1] - current_L[1]) < 2:
                left_and_right_meet = True
                break

            # 左下死胡同修正
            if count_L > 1 and dir_L[count_L-1] == 7 and current_R[1] > current_L[1]:
                current_L = points_L[count_L-1].copy()
                count_L -= 1

        # 统计各方向的命中次数
        stats = {
            'left_dirs': {i: 0 for i in range(8)},
            'right_dirs': {i: 0 for i in range(8)}
        }
        for i in range(count_L):
            if i < len(dir_L):
                stats['left_dirs'][dir_L[i]] += 1
        for i in range(count_R):
            if i < len(dir_R):
                stats['right_dirs'][dir_R[i]] += 1

        return {
            'points_L': points_L[:count_L],
            'points_R': points_R[:count_R],
            'dir_L': dir_L[:count_L],
            'dir_R': dir_R[:count_R],
            'stats': stats,
            'meet': left_and_right_meet
        }

    def process(self, binary_mask, target_size=(160, 120), deal_low=87, deal_high=117):
        """
        完整的边缘追踪处理流程：
        缩放 → 中值滤波去噪 → 添加人工边界（防丢线） → 搜索起点 → 爬取边缘。

        参数:
            binary_mask: 输入的二值化车道线图像
            target_size: 处理分辨率 (宽, 高)
            deal_low:    ROI 下边界（靠近车体）
            deal_high:   ROI 上边界（远处）

        返回:
            追踪结果 dict，若未找到起始点则返回 None
        """
        frame = cv2.resize(binary_mask, target_size)
        filtered = cv2.medianBlur(frame, 3)
        bordered = filtered.copy()
        h, w = bordered.shape

        # ---- 添加外黑内白的人工边缘，提高极端丢线时的容错率 ----
        # 1. 左右两侧画白边（内白）
        cv2.line(bordered, (1, 0), (1, h), (255, 255, 255), 2)
        cv2.line(bordered, (w - 2, 0), (w - 2, h), (255, 255, 255), 2)
        # 2. 最外侧画黑边（外黑），确保画面边缘是死路
        cv2.line(bordered, (0, 0), (0, h), (0, 0, 0), 2)
        cv2.line(bordered, (w - 1, 0), (w - 1, h), (0, 0, 0), 2)

        # 从 ROI 底部向上逐行搜索初始起点
        found_initial = False
        start_L, start_R = [0, 0], [0, 0]

        for row in range(deal_high, deal_low - 1, -1):
            found, init_L, init_R = self._get_initial_point(bordered, row)
            if found:
                found_initial = True
                start_L, start_R = init_L, init_R
                break

        if not found_initial:
            return None

        result = self._crawl_edges(bordered, start_L, start_R, deal_low, deal_high)
        return result


# ==============================================================================
# 视觉巡线 ROS 主节点
# ==============================================================================

class LineFollowerNode:
    """
    视觉巡线主控节点。

    巡线全流程状态机：
        1. 收到 /start_follow 指令 → 等待 8s 稳定
        2. 初始车头对正（原地旋转直到偏差 < 阈值）
        3. 八邻域倒计时 → 拐点检测与旋转处理
        4. 拐点完成后启动倒计时 → 倒计时结束后开始终点停车线检测
        5. 检测到停车线前线 → 微调方向 → 过滤帧 → 检测后线 → 停车

    PID 直行速度分阶段调节：
        阶段① (八邻域):      线性速度 0.10~0.30
        阶段② (倒计时等待):   线性速度 0.10~0.45
        阶段③ (停车线前线):   线性速度 0.10~0.45
        阶段④ (停车线后线):   线性速度 0.10~0.45
    """

    def __init__(self):
        rospy.init_node('basic_line_follower', anonymous=True)

        self.bridge = CvBridge()

        # ---- ROS 接口 ----
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.finish_pub = rospy.Publisher('/finish_follow', Bool, queue_size=10)
        self.debug_pub = rospy.Publisher('/line_follow_debug', Image, queue_size=1)  # 处理后画面调试发布
        self.debug_enabled = True   # 调试开关：True=发布处理图，False=不发布

        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/start_follow', String, self.start_follow_callback, queue_size=1)

        # ---- 运行状态 ----
        self.start_follow = False                     # 是否已收到启动指令
        self.stop_flag = False                        # 是否已触发停车
        self.follow_mode = 'right'                    # 巡线模式：left / right / straight

        # ---- 图像处理参数 ----
        self.img_width = 160
        self.img_height = 120
        self.deal_low = 76                            # PID 计算的 ROI 下边界
        self.deal_high = 105                          # PID 计算的 ROI 上边界
        self.eight_deal_high = 118                    # 八邻域追踪 ROI 上边界
        self.eight_deal_low = 85                      # 八邻域追踪 ROI 下边界

        # ---- PID 控制参数 ----
        self.Kp = 0.03
        self.Ki = 0.00010
        self.Kd = 0.0000000
        self.sum_pid = 0.0
        self.last_error = 0.0
        self.pid_count = 0                            # PID 积分累计帧数（>50 帧清零防饱和）

        # ---- 初始对正 ----
        self.is_initial_aligned = False               # 初始车头是否已对正
        self.initial_align_threshold = 10.0           # 初始对正偏差阈值（像素）

        # ---- 八邻域拐点检测 ----
        self.start_time = None                        # 巡线开始时间（对正完成后设置）
        self.timer_duration = 0.0                     # 八邻域倒计时时长（由 follow_mode 决定）
        self.Is_eight_finished = False                # 八邻域拐点处理是否完成
        self.rotate_one_flag = 0                      # 拐点旋转阶段标记（0=未触发, 1=已触发）
        self.max_rotate_error_cross1 = 30.0           # 拐点旋转完成的最大允许偏差
        self.Is_play_sound = False                    # 倒计时结束语音播报标记
        self.filtered_fps_num = 0                     # 停车线过滤帧计数

        # 八邻域追踪器实例
        self.tracker = EightNeighborhoodTracker(max_points=200)

        # ---- 拐点连续帧确认 ----
        self.corner_consec = 0                        # 拐点连续命中帧数
        self.corner_consec_need = 3                   # 连续 3 帧确认拐点

        # ---- 接近阶段（拐点确认后先低速前进再旋转） ----
        self.approach_deadline = None                 # 接近阶段截止时间
        self.approach_time = 0.9                      # 接近时长（秒）
        self.approach_speed = 0.15                    # 接近速度（m/s）
        self.rotate_val = 0                           # 拐点旋转角度（确认时写入）

        # ---- 旋转后状态机 ----
        self.rotate_flag = 0                          # 0=未转 1=刚转完 2=纯直行0.5s中 3=摆正完成
        self.straight_start = None                    # 旋转后纯直行起点

        # ---- 摆正直行 2s ----
        self.post_start = None                        # 摆正直行起点
        self.post_done = False                        # 摆正直行是否完成
        self.post_dur = 2.0                           # 摆正直行时长（秒）

        # ---- 巡航 4s 屏蔽停车 ----
        self.cruise_start = None                      # 巡航起点
        self.cruise_done = False                      # 巡航是否完成
        self.cruise_dur = 4.0                         # 巡航时长（秒）

        # ---- 拐点旋转后倒计时 ----
        self.post_rotate_duration = 5.0               # 倒计时时长（秒）
        self.is_post_rotate_finished = False          # 倒计时是否完成
        self.post_rotate_start_time = None            # 倒计时开始时间
        self.is_stop_line_front_found = False         # 是否已找到停车线前线
        self.play_voice_once = False                  # 语音播报一次性标记
        self.filtered_fps_num_threshold = 30          # 过滤帧数阈值（前线→后线间隔帧数）

        # ---- 停车线检测参数 ----
        self.stop_line_row_start = 24                 # 检测 ROI 起始行（120p 基准）
        self.stop_line_row_end = 119                  # 检测 ROI 结束行
        self.stop_line_front_threshold = 60           # 前线平均行号阈值
        self.stop_line_back_threshold = 60            # 后线平均行号阈值

        rospy.loginfo("基础视觉巡线节点已启动，等待图像数据...")

    # ==========================================================================
    #  启动回调：解析巡线指令
    # ==========================================================================

    def start_follow_callback(self, msg):
        """
        /start_follow 回调。
        解析指令字符串（left / right / straight），设置对应的巡线参数。
        收到指令后等待 8 秒再启动巡线。
        """
        self.is_initial_aligned = False
        self.start_time = None

        command = msg.data.lower()

        if command == 'right':
            rospy.loginfo("右转")
            self.follow_mode = 'right'
            self.timer_duration = 0.0
            self.post_rotate_duration = 3.0
        elif command == 'left':
            rospy.loginfo("左转")
            self.follow_mode = 'left'
            self.timer_duration = 0.0
            self.post_rotate_duration = 3.0
        elif command == 'straight':
            self.follow_mode = 'straight'
            self.timer_duration = 3.0
            self.post_rotate_duration = 5.0
            rospy.loginfo("直走")
        else:
            rospy.loginfo(f"收到未知指令: {msg.data}")

        rospy.sleep(8.0)
        self.start_follow = True
        rospy.loginfo(f"收到启动指令: {msg.data}，开始巡线流程！")

    # ==========================================================================
    #  底盘控制：原地旋转
    # ==========================================================================

    def rotate_speed(self, angle_degrees, speed):
        """
        控制底盘原地旋转指定角度。

        参数:
            angle_degrees: 目标旋转角度（度）
            speed:        旋转角速度（rad/s）
        """
        angle_radians = math.radians(angle_degrees)
        rotate_cmd = Twist()

        rotate_cmd.linear.x = 0.0
        rotate_cmd.angular.z = 0.0
        self.cmd_pub.publish(rotate_cmd)
        rospy.sleep(0.1)

        rotate_cmd.angular.z = speed if angle_degrees > 0 else -speed
        rospy.loginfo(f"开始旋转 {angle_degrees} 度")

        start_time_rotate = rospy.Time.now()
        duration = abs(angle_radians) / speed

        while rospy.Time.now() - start_time_rotate < rospy.Duration(duration):
            self.cmd_pub.publish(rotate_cmd)
            rospy.sleep(0.05)

        rotate_cmd.angular.z = 0.0
        self.cmd_pub.publish(rotate_cmd)
        rospy.loginfo("大角度旋转完成")

    # ==========================================================================
    #  图像预处理：边缘检测流水线
    # ==========================================================================

    def preprocess_image(self, image):
        """
        图像预处理流水线：
        灰度化 → 高斯模糊 → Canny 边缘检测 → 膨胀 → 腐蚀 → 再膨胀 → 闭运算 → 二值化。

        返回二值化边缘图像。
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(edges, kernel, iterations=2)
        eroded = cv2.erode(dilated, kernel, iterations=1)
        dilated_again = cv2.dilate(eroded, kernel, iterations=1)
        closed_edges = cv2.morphologyEx(dilated_again, cv2.MORPH_CLOSE, kernel)

        _, binary_result = cv2.threshold(closed_edges, 1, 255, cv2.THRESH_BINARY)
        return binary_result

    # ==========================================================================
    #  PID 中线计算
    # ==========================================================================

    def calculate_midpoint_and_error(self, mask):
        """
        在二值化边缘图像中逐行计算赛道中线位置，返回等效中点和偏差。

        算法：从 ROI 底部向上逐行扫描，用上一行的中点作为当前行的中心，
        在中心左右各 half_width 范围内找到左右白色边缘点，取平均作为该行中点。

        返回:
            (等效中点 x 坐标, 偏差 = 图像宽度/2 - 等效中点)
        """
        half_width = self.img_width // 2
        half = half_width
        mid_count = 0
        range_scan = self.deal_high - self.deal_low

        for y in range(self.deal_high, self.deal_low, -1):
            left_roi = mask[y][max(0, half - half_width):half]
            left = (max(0, half - half_width)
                    if not np.any(left_roi == 255)
                    else np.average(np.where(left_roi == 255)))

            right_roi = mask[y][half:min(self.img_width, half + half_width)]
            right = (min(self.img_width, half + half_width)
                     if not np.any(right_roi == 255)
                     else np.average(np.where(right_roi == 255)) + half)

            mid = int((left + right) // 2)
            half = mid
            mid_count += mid

        mid_equivalent = mid_count / range_scan if range_scan > 0 else half_width
        error = half_width - mid_equivalent
        return mid_equivalent, error

    # ==========================================================================
    #  终点停车线检测
    # ==========================================================================

    def detect_stop_line(self, image):
        """
        检测画面中的横向白色停止线。

        算法：
        1. 截取 ROI 行范围 → 灰度 → 高斯模糊 → 二值化（阈值 185）
        2. 在图像中间区域扫描 9 列，从下往上找白→黑的跳变点
        3. 至少 6 列找到跳变点且平均行号 > 阈值 → 判定为停车线
        4. 前线检测到后启动过滤帧计数，满阈值后再检测后线 → 触发停车

        返回:
            True  —— 检测到停车线后线，已触发停车
            False —— 未检测到或仅检测到前线
        """
        # ---- 前线已找到：过滤帧倒计时 ----
        if self.is_stop_line_front_found == True:
            self.filtered_fps_num += 1
            if self.filtered_fps_num >= self.filtered_fps_num_threshold:
                if self.play_voice_once == False:
                    twist = Twist()
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    print("过滤帧结束，即将检测到终点停车线的后线，准备停车！")
                    rospy.sleep(0.3)
                    self.play_voice_once = True

        # ---- ROI 切割 + 二值化 ----
        cropped_image = image[self.stop_line_row_start:self.stop_line_row_end + 1, :]
        gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

        # 轻度高斯模糊，去除地砖反光噪点
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        _, binary_image = cv2.threshold(blurred_image, 185, 255, cv2.THRESH_BINARY)

        height, width = binary_image.shape
        middle = width // 2
        left_col = middle - 20
        right_col = middle + 20
        rows_found = []

        # 扫描 9 列（间隔 5 像素），从下往上找白→黑跳变
        for col in range(left_col, right_col + 1, 5):
            for row in range(height - 2, 0, -1):
                if binary_image[row, col] == 255 and binary_image[row + 1, col] == 0:
                    rows_found.append(row)
                    break

        # 至少 6 列找到跳变点 → 判定为连贯横线
        if len(rows_found) >= 6:
            # 前线/后线使用不同阈值
            if self.filtered_fps_num <= self.filtered_fps_num_threshold:
                stop_threshold = self.stop_line_front_threshold
            else:
                stop_threshold = self.stop_line_back_threshold

            if sum(rows_found) / len(rows_found) > stop_threshold:
                if self.is_stop_line_front_found == False:
                    print("找到终点停车线的前线")
                    # 根据方向微调车头
                    if self.follow_mode == 'right':
                        self.rotate_speed(20, 0.3)
                    elif self.follow_mode == 'left':
                        self.rotate_speed(-20, 0.3)
                    rospy.sleep(0.3)
                    self.is_stop_line_front_found = True
                    return False
                elif self.filtered_fps_num >= self.filtered_fps_num_threshold:
                    print("找到终点停车线的后端，准备停车")
                    self.stop_robot()
                    return True

        return False

    # ==========================================================================
    #  PID 控制输出
    # ==========================================================================

    def execute_pid_control(self, error):
        """
        根据偏差计算 PID 控制量，发布 /cmd_vel。

        直行速度分两档调节（error 越大 → 速度越低）：
            拐点确认前:  linear = max(0.10, 0.30 - |error| × 0.042)
            拐点确认后:  linear = max(0.10, 0.45 - |error| × 0.042)

        PID 积分每 50 帧清零一次防饱和，微分项使用截断后的误差。
        """
        self.pid_count += 1
        if self.pid_count > 50:
            self.pid_count = 0
            self.sum_pid = 0.0

        self.sum_pid += error
        d_pid = error - self.last_error
        angular_z = (error * self.Kp) + (self.sum_pid * self.Ki) + (d_pid * self.Kd)

        # 误差截断到 [-5, 5] 防止异常跳变
        self.last_error = max(-5.0, min(5.0, error))

        # 分两档计算直行速度（对齐 follow_left_v4）
        if self.Is_eight_finished == False:
            linear_x = max(0.1, 0.30 - abs(error) * 0.042)
        else:
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    # ==========================================================================
    #  停车程序
    # ==========================================================================

    def stop_robot(self):
        """
        执行停车：
        1. 发送零速度指令
        2. 发布 /finish_follow = True
        3. 播放 finish.wav 音频
        4. 关闭节点
        """
        rospy.loginfo("检测到停止线，执行停车程序！")
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = 0.0, 0.0, 0.0
        self.cmd_pub.publish(twist)
        self.finish_pub.publish(True)
        self.stop_flag = True
        wav_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'wav', 'finish.wav')
        subprocess.Popen(["aplay", wav_path])
        rospy.signal_shutdown("到达终点，巡线节点正常关闭。")

    # ==========================================================================
    #  主图像回调：巡线全流程状态机
    # ==========================================================================

    def image_callback(self, msg):
        """
        /usb_cam/image_raw 回调 —— 每帧驱动巡线全流程。

        处理顺序（按优先级）：
            0. 初始车头对正（原地旋转直到偏差 < 阈值）
            1. 终点停车线检测（仅倒计时完成后才执行）
            2. 八邻域拐点检测与旋转处理
            3. 拐点旋转后倒计时管理
            4. 常规 PID 巡线控制
        """
        if not self.start_follow or self.stop_flag:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = cv2.resize(cv_image, (self.img_width, self.img_height))
            image = cv2.flip(image, 1)

            mask = self.preprocess_image(image)
            _, vtherror = self.calculate_midpoint_and_error(mask)

            # ---- 调试发布：处理后二值化画面 ----
            if self.debug_enabled:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))

            # ==============================================================
            # 0. 初始车头对正（最高优先级）
            #    偏差大于阈值 → 原地旋转微调
            #    偏差小于阈值 → 标记对正完成，记录开始时间
            # ==============================================================
            if not self.is_initial_aligned:
                if abs(vtherror) > self.initial_align_threshold:
                    rospy.loginfo(f"初始车头未对正 (偏差: {vtherror:.2f})，正在原地旋转调整...")
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3 if vtherror > 0 else -0.3
                    self.cmd_pub.publish(twist)
                    return
                else:
                    self.is_initial_aligned = True
                    self.start_time = rospy.Time.now()
                    rospy.loginfo("初始车头已对正！开启八邻域倒计时，并开始正常巡线。")
                    return

            # ==============================================================
            # 1. 拐点检测 + 接近阶段 + 旋转
            #    (倒计时到期后 → 八邻域检测 → 连续3帧确认 → 接近0.9s → 旋转)
            # ==============================================================
            if self.Is_eight_finished == False:
                elapsed_time = (rospy.Time.now() - self.start_time).to_sec()

                if elapsed_time >= self.timer_duration:
                    # straight 模式下调低 ROI 上边界
                    if self.follow_mode == 'straight':
                        self.eight_deal_low = 70

                    # ---- 接近阶段尚未开始 → 做拐点检测 ----
                    if self.approach_deadline is None:
                        tracker_result = self.tracker.process(
                            binary_mask=mask,
                            target_size=(self.img_width, self.img_height),
                            deal_low=self.eight_deal_low,
                            deal_high=self.eight_deal_high
                        )

                        if tracker_result is not None:
                            # ---- 参数设置：根据模式选方向索引和阈值 ----
                            if self.follow_mode == 'left':
                                dir_idx = 3
                                threshold = 6
                                rotate_val = 80
                            elif self.follow_mode == 'right':
                                dir_idx = 3
                                threshold = 6
                                rotate_val = -60
                            elif self.follow_mode == 'straight':
                                dir_idx = 2
                                threshold = 30
                                rotate_val = -70
                            else:
                                return

                            # ---- 数据获取 ----
                            left_count = tracker_result['stats']['left_dirs'][dir_idx]
                            right_count = tracker_result['stats']['right_dirs'][dir_idx]

                            print(f"当前模式:{self.follow_mode} | 方向:{dir_idx} | "
                                  f"左点数:{left_count} | 右点数:{right_count} | 阈值:{threshold}")

                            # ---- 拐点判断：连续 3 帧确认 ----
                            if left_count > threshold or right_count > threshold:
                                self.corner_consec += 1
                                if self.corner_consec >= self.corner_consec_need:
                                    rospy.loginfo(f"拐点确认！连续{self.corner_consec}帧命中 "
                                                  f"(左{left_count}/右{right_count})，进入接近阶段...")
                                    self.rotate_val = rotate_val
                                    self.approach_deadline = rospy.Time.now() + rospy.Duration(self.approach_time)
                            else:
                                self.corner_consec = 0

                    # ---- 接近阶段：拐点确认后先低速前进 0.9s 再旋转 ----
                    if self.approach_deadline is not None:
                        now = rospy.Time.now()
                        if now < self.approach_deadline:
                            twist = Twist()
                            twist.linear.x = self.approach_speed
                            twist.angular.z = max(-0.15, min(0.15, vtherror * 0.02))
                            self.cmd_pub.publish(twist)
                            return
                        else:
                            rospy.loginfo(f"接近完成，旋转 {self.rotate_val} 度")
                            self.rotate_speed(self.rotate_val, 0.8)
                            self.rotate_flag = 1
                            self.Is_eight_finished = True
                            return

            # ==============================================================
            # 2. 旋转后状态机：纯直行0.5s → 摆正直行2s → 巡航4s(屏蔽停车)
            # ==============================================================

            # ---- 旋转刚完成 → 记录纯直行起点 ----
            if self.rotate_flag == 1:
                self.straight_start = rospy.Time.now()
                self.rotate_flag = 2
                rospy.loginfo("旋转完成！纯直行 0.5s...")
                return

            # ---- 纯直行 0.5s（不修正方向，避免刚转完抖动） ----
            if self.rotate_flag == 2:
                el = (rospy.Time.now() - self.straight_start).to_sec()
                if el < 0.5:
                    twist = Twist()
                    twist.linear.x = 0.12
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    return
                else:
                    self.post_start = rospy.Time.now()
                    self.rotate_flag = 3
                    rospy.loginfo("纯直行完成！摆正直行 2s...")
                    return

            # ---- 摆正直行 2s（恢复 PID 修正） ----
            if self.post_start is not None and not self.post_done:
                el = (rospy.Time.now() - self.post_start).to_sec()
                if el < self.post_dur:
                    self.execute_pid_control(vtherror)
                    return
                else:
                    self.post_done = True
                    self.cruise_start = rospy.Time.now()
                    rospy.loginfo("摆正完成！巡航 4s 屏蔽停车...")
                    return

            # ---- 巡航 4s（屏蔽停车线检测） ----
            if self.post_done and not self.cruise_done:
                el = (rospy.Time.now() - self.cruise_start).to_sec()
                if el < self.cruise_dur:
                    self.execute_pid_control(vtherror)
                    return
                else:
                    self.cruise_done = True
                    rospy.loginfo("巡航结束！开始检测终点停车线")
                    wav_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                            '..', 'wav', 'start_check_parking.wav')
                    subprocess.Popen(["aplay", wav_path])
                    return

            # ==============================================================
            # 3. 停车线检测（仅巡航结束后执行）
            # ==============================================================
            if self.cruise_done:
                if self.detect_stop_line(image):
                    self.stop_robot()
                    print(f"过滤帧数{self.filtered_fps_num}")
                    print("检测到终点停车线，执行停车程序！")
                    return

            # ==============================================================
            # 4. 常规 PID 控制输出（拐点检测阶段每帧兜底）
            # ==============================================================
            self.execute_pid_control(vtherror)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge 转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"巡线过程发生异常: {e}")


# ==============================================================================
# 主入口
# ==============================================================================
if __name__ == '__main__':
    try:
        node = LineFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
