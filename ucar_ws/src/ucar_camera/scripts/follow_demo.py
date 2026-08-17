#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
==============================================================================
ROS 视觉巡线节点 (含挡板避障绕行)
针对20届巡线赛道，从巡线的左侧赛道进入
==============================================================================

【程序简介】
本节点是一个完整的基于视觉的移动机器人巡线控制程序。它结合了传统的 PID 误差
巡线算法与基于"八邻域"的图像拓扑特征提取算法，能够实现小车的初始自动对正、
常规循迹、直角/十字拐点识别与转向，以及终点停车线的识别与自动停车。

【避障绕行状态机（纯雷达触发）】：
雷达距离 < 0.30m → 雷达法线对齐(lidar_align) → 侧向平移绕过(lateral_shift)
→ 前冲通过(forward_pass) → 反向侧向平移回赛道(return_lateral_shift) → 恢复正常巡线
左行/右行模式对称镜像。
不再使用视觉检测，由雷达持续监控前方距离；距离小于阈值时直接触发绕行，
无需后退，直接用雷达PCA拟合挡板法线、旋转对齐后侧向平移绕过。

【核心状态机与控制流程】
程序在 image_callback 中实现了一个具有高低优先级的状态机流水线：
0. 雷达避障绕行 (Priority 0 — 最高)
1. 初始车头对正 (Priority 1)
2. 延时防误判机制 & 八邻域激活 (Priority 2)
3. 拐点处理状态机 (Priority 3)
4. 终点停车检测 (Priority 4)
5. 常规 PID 巡线 (Base Priority)

运行说明：
1.要先启动小车底盘，雷达和摄像头 启动follow.launch即可
2.cd到这个脚本所在目录，运行 python3 follow_demo.py 即可开始巡线
3.新开一个终端，使用以下命令发布巡线指令，选择巡线赛道（右转、左转或直走）：
rostopic pub -1 /start_follow std_msgs/String "data: 'right'"
rostopic pub -1 /start_follow std_msgs/String "data: 'left'"
rostopic pub -1 /start_follow std_msgs/String "data: 'straight'"
日期：2026.07.25
==============================================================================
"""

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import wave
import os
import subprocess
import wave

try:
    import pyaudio
    p = pyaudio.PyAudio()
except Exception:
    pyaudio = None
    p = None
    print("警告: pyaudio 不可用，语音播报已禁用")
# ==============================================================================
# 八邻域追踪器类
# ==============================================================================
class EightNeighborhoodTracker:
    """
    八邻域赛道边缘追踪器类
    """
    def __init__(self, max_points=200):
        self.max_points = max_points
        # 左搜索方向 (逆时针)
        self.seeds_L = [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]
        # 右搜索方向 (顺时针)
        self.seeds_R = [[0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1], [1, 0], [1, 1]]

    def _get_initial_point(self, image, initial_row):
        width = image.shape[1]
        initial_L = [0, 0]
        initial_R = [0, 0]
        found_L, found_R = False, False

        # 由中间向左侧寻找起点
        for i in range(width // 2, 1, -1):
            if image[initial_row, i] == 0 and image[initial_row, i - 1] == 255:
                initial_L = [i, initial_row]
                found_L = True
                break

        # 由中间向右侧寻找起点
        for i in range(width // 2, width - 2):
            if image[initial_row, i] == 0 and image[initial_row, i + 1] == 255:
                initial_R = [i, initial_row]
                found_R = True
                break

        return (found_L and found_R), initial_L, initial_R

    def _crawl_edges(self, image, start_L, start_R, deal_low, deal_high):
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
            # ================= 左线处理 =================
            if active_L:
                if not (deal_high <= current_L[1] <= deal_low):
                    active_L = False
                else:
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

                    candidates_L = []
                    for i in range(8):
                        ni = (i + 1) % 8
                        if (0 <= search_fields_L[i][0] < width and 0 <= search_fields_L[i][1] < height and
                            0 <= search_fields_L[ni][0] < width and 0 <= search_fields_L[ni][1] < height):
                            if (image[search_fields_L[i][1], search_fields_L[i][0]] == 0 and
                                image[search_fields_L[ni][1], search_fields_L[ni][0]] == 255):
                                candidates_L.append(search_fields_L[i])
                                if count_L - 1 < len(dir_L):
                                    dir_L[count_L - 1] = i

                    if candidates_L:
                        current_L = min(candidates_L, key=lambda p: p[1])

            # ================= 右线处理 =================
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
                        if (0 <= search_fields_R[i][0] < width and 0 <= search_fields_R[i][1] < height and
                            0 <= search_fields_R[ni][0] < width and 0 <= search_fields_R[ni][1] < height):
                            if (image[search_fields_R[i][1], search_fields_R[i][0]] == 0 and
                                image[search_fields_R[ni][1], search_fields_R[ni][0]] == 255):
                                candidates_R.append(search_fields_R[i])
                                if count_R - 1 < len(dir_R):
                                    dir_R[count_R - 1] = i

                    if candidates_R:
                        current_R = min(candidates_R, key=lambda p: p[1])

            # ================= 退出条件检查 =================
            if not active_L and not active_R:
                break

            if count_R >= 3 and count_L >= 3:
                last_R = points_R[count_R-3:count_R]
                last_L = points_L[count_L-3:count_L-1]
                if all(np.array_equal(last_R[0], p) for p in last_R[1:]) or all(np.array_equal(last_L[0], p) for p in last_L[1:]):
                    break

            if abs(current_R[0] - current_L[0]) < 2 and abs(current_R[1] - current_L[1]) < 2:
                left_and_right_meet = True
                break

            if count_L > 1 and dir_L[count_L-1] == 7 and current_R[1] > current_L[1]:
                current_L = points_L[count_L-1].copy()
                count_L -= 1

        stats = {
            'left_dirs': {i: 0 for i in range(8)},
            'right_dirs': {i: 0 for i in range(8)}
        }
        for i in range(count_L):
            if i < len(dir_L): stats['left_dirs'][dir_L[i]] += 1
        for i in range(count_R):
            if i < len(dir_R): stats['right_dirs'][dir_R[i]] += 1

        return {
            'points_L': points_L[:count_L],
            'points_R': points_R[:count_R],
            'dir_L': dir_L[:count_L],
            'dir_R': dir_R[:count_R],
            'stats': stats,
            'meet': left_and_right_meet
        }

    def process(self, binary_mask, target_size=(160, 120), deal_low=87, deal_high=117):
        frame = cv2.resize(binary_mask, target_size)
        filtered = cv2.medianBlur(frame, 3)
        bordered = filtered.copy()
        h, w = bordered.shape

        # ==========================================================
        # 加入外黑内白的人工边缘，提高极端丢线时的容错率
        # ==========================================================
        # 1. 先在左右两侧画白边 (内白)
        cv2.line(bordered, (1, 0), (1, h), (255, 255, 255), 2)
        cv2.line(bordered, (w - 2, 0), (w - 2, h), (255, 255, 255), 2)

        # 2. 再在最外侧画黑边 (外黑)，覆盖上一层的溢出，确保画面最边缘是死路
        cv2.line(bordered, (0, 0), (0, h), (0, 0, 0), 2)
        cv2.line(bordered, (w - 1, 0), (w - 1, h), (0, 0, 0), 2)
        # ==========================================================

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

def play_wav(file_path, p):
    if p is None:
        return
    # 使用 with 语句确保文件安全关闭
    with wave.open(file_path, 'rb') as wf:
        stream = p.open(
            format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            output=True,
            output_device_index=0  # <--- 就是加了这一行！强制使用 es7134 声卡
        )
        chunk = 1024
        data = wf.readframes(chunk)

        while data:
            stream.write(data)
            data = wf.readframes(chunk)

        stream.stop_stream()
        stream.close()

# ==============================================================================
# 视觉巡线 ROS 主节点类
# ==============================================================================
class LineFollowerNode:
    def __init__(self):
        rospy.init_node('basic_line_follower', anonymous=True)

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.finish_pub = rospy.Publisher('/finish_follow', Bool, queue_size=10)
        self.debug_pub = rospy.Publisher('/follow_demo_debug', Image, queue_size=1)  # 处理后画面调试发布
        self.debug_enabled = True   # 调试开关：True=发布处理图，False=不发布

        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/start_follow', String, self.start_follow_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.barrier_lidar_callback, queue_size=1)

        self.start_follow = False
        self.stop_flag = False
        self.follow_mode = 'right'   #巡线赛道选择，初始化时默认是左转，在start_follow_callback中更新
        self.img_width = 160
        self.img_height = 120
        self.deal_low = 76
        self.deal_high = 105

        self.eight_deal_high = 118
        self.eight_deal_low = 85

        self.Kp = 0.03
        self.Ki = 0.00010
        self.Kd = 0.0000000
        self.sum_pid = 0.0
        self.last_error = 0.0
        self.pid_count = 0

        self.is_initial_aligned = False
        self.initial_align_threshold = 10.0

        # === 初始对正：比例控制 + 稳定帧计数 ===
        self.align_Kp = 0.025               # 比例系数：偏差1像素 → 0.025 rad/s
        self.align_max_speed = 0.8          # 最大旋转速度 (rad/s)，偏差大时转得快
        self.align_min_speed = 0.12         # 最小旋转速度 (rad/s)，避免太慢转不动
        self.align_stable_count = 0         # 连续满足阈值条件的帧数
        self.align_stable_required = 8      # 需要连续多少帧稳定才算对正完成

        # === 初始对正后的前进阶段 ===
        self.is_initial_forward_done = False        # 初始前进是否完成
        self.initial_forward_start_time = None      # 初始前进开始时间
        self.initial_forward_duration = 0.3  # 初始前进持续时间（秒），可根据需要调整

        self.start_time = None
        self.timer_duration = 0.0
        self.Is_eight_finished =  False
        self.rotate_one_flag = 0
        self.max_rotate_error_cross1 = 30.0
        self.Is_play_sound = False  #播放标注，倒计时结束播报
        self.filtered_fps_num = 0
        # 实例化八邻域追踪器
        self.tracker = EightNeighborhoodTracker(max_points=200)

        # === 新增：拐点旋转后的倒计时参数,倒计时结束后才去检测终点停车线 ===
        self.post_rotate_duration = 5.0           # 倒计时的时间
        self.is_post_rotate_finished = False      # 倒计时完成的标志，初始为False
        self.post_rotate_start_time = None        # 记录倒计时开始的时间
        self.is_stop_line_front_found = False     # 是否已经找到停车线前线的标志
        self.play_voice_once = False              # 只语音播报一次的标志
        self.filtered_fps_num_threshold = 30      # 过滤帧数的阈值，达到后才执行停车线后端的逻辑

        # 停车线检测的行范围 (以120p为基准)
        self.stop_line_row_start = 24   # 检测起始行
        self.stop_line_row_end = 119    # 检测结束行

        # 停车线平均行号阈值
        self.stop_line_front_threshold = 60   # 前线阈值
        self.stop_line_back_threshold = 60    # 后线阈值

        # ================================================================
        # 避障绕行状态机（纯雷达触发，不动原巡线逻辑）
        # 状态流转：none → lidar_align → lateral_shift
        #          → forward_pass → return_lateral_shift → done
        # 雷达距离 < 0.30m 时触发绕行，不后退，直接法线对齐后侧向平移
        # ================================================================
        self.barrier_state = 'none'              # 绕行状态: none/done/lidar_align/lateral_shift/forward_pass/return_lateral_shift
        self.barrier_detect_enabled = False      # 是否启用避障绕行（左右转模式开启，直行模式关闭）
        self.barrier_phase_start_time = None     # 绕行阶段计时起点

        # 绕行动作参数
        self.barrier_forward_speed = 0.30        # 直行速度
        self.barrier_rotate_speed = 0.6          # 旋转速度
        self.barrier_return_lateral_time = 1.9   # 侧向平移回赛道时间（秒），控制回程距离
        self.barrier_return_correct_angle = 35   # 回程后反向微调角度（度），左转模式往右转，右转模式往左转

        # === 雷达触发绕行参数 ===
        self.barrier_lidar_trigger_distance = 0.28  # 雷达触发绕行的距离阈值（米），< 此值触发绕行
        self.barrier_lidar_points = []           # 缓存雷达扫描到的挡板点云 [(x,y), ...]
        self.barrier_lidar_angle_range = 30      # 雷达扫描挡板的半角范围（度）
        self.barrier_align_distance = 0.30       # 法线对齐时期望的车与挡板距离（米）
        self.barrier_lateral_speed = 0.25        # 侧向平移速度 (m/s)
        self.barrier_lateral_time = 2.0          # 侧向平移时间（秒）
        self.barrier_forward_pass_time = 1.9     # 绕过挡板后前冲时间（秒）
        self.barrier_normal_angle = 0.0          # 雷达计算出的法线角度（临时存储）
        self.barrier_lidar_min_distance = float('inf')  # 雷达当前帧最近距离
        self.barrier_done_once = False            # 障碍物只触发一次，绕行完成后不再触发
        self.barrier_lidar_enabled = True         # 雷达是否启用（绕行时关闭，绕行后恢复）

        rospy.loginfo("基础视觉巡线节点已启动（含避障绕行），等待图像数据...")

    def start_follow_callback(self, msg):
            self.is_initial_aligned = False
            self.is_initial_forward_done = False       # 重置初始前进标志
            self.initial_forward_start_time = None      # 重置前进开始时间
            self.align_stable_count = 0                 # 重置稳定帧计数
            self.start_time = None

            # 重置绕行状态
            self.barrier_state = 'none'
            self.barrier_phase_start_time = None
            self.barrier_lidar_points = []
            self.barrier_normal_angle = 0.0
            self.barrier_lidar_min_distance = float('inf')
            self.barrier_done_once = False
            self.barrier_lidar_enabled = True

            # 获取指令并转换为小写
            command = msg.data.lower()

            # 根据指令打印对应的动作
            if command == 'right':
                rospy.loginfo("右转")
                self.follow_mode = 'right'
                self.timer_duration = 0.0
                self.post_rotate_duration = 3.0
                self.barrier_detect_enabled = True      # 右转弯道启用避障绕行
            elif command == 'left':
                rospy.loginfo("左转")
                self.follow_mode = 'left'
                self.timer_duration = 0.0
                self.post_rotate_duration = 3.0
                self.barrier_detect_enabled = True      # 左转弯道启用避障绕行
            elif command == 'straight':
                self.follow_mode = 'straight'
                self.timer_duration = 3.0
                self.post_rotate_duration = 5.0
                self.barrier_detect_enabled = True      # 直行道也启用避障绕行（与左转模式策略一致）
                rospy.loginfo("直走")
            else:
                rospy.loginfo(f"收到未知指令: {msg.data}")
            rospy.sleep(0.5)  # 已去掉——原来给操作者留的准备时间，不需要了
            self.start_follow = True
            rospy.loginfo(f"收到启动指令: {msg.data}，开始巡线流程！")

    def rotate_speed(self, angle_degrees, speed):
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

    def preprocess_image(self, image):
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

    def calculate_midpoint_and_error(self, mask):
        half_width = self.img_width // 2
        half = half_width
        mid_count = 0
        range_scan = self.deal_high - self.deal_low

        for y in range(self.deal_high, self.deal_low, -1):
            left_roi = mask[y][max(0, half - half_width):half]
            left = max(0, half - half_width) if not np.any(left_roi == 255) else np.average(np.where(left_roi == 255))

            right_roi = mask[y][half:min(self.img_width, half + half_width)]
            right = min(self.img_width, half + half_width) if not np.any(right_roi == 255) else np.average(np.where(right_roi == 255)) + half

            mid = int((left + right) // 2)
            half = mid
            mid_count += mid

        mid_equivalent = mid_count / range_scan if range_scan > 0 else half_width
        error = half_width - mid_equivalent
        return mid_equivalent, error

    def detect_stop_line(self, image):
            if self.is_stop_line_front_found == True:
                self.filtered_fps_num += 1
                if self.filtered_fps_num >= self.filtered_fps_num_threshold :
                    if self.play_voice_once == False:
                        twist = Twist()
                        twist.linear.x = 0.0
                        self.cmd_pub.publish(twist)
                        print("过滤帧结束，即将检测到终点停车线的后线，准备停车！")
                        rospy.sleep(0.3)
                        self.play_voice_once = True


            # 切割画面指定行范围
            cropped_image = image[self.stop_line_row_start:self.stop_line_row_end + 1, :]
            gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

            # 轻度高斯模糊，去除地砖上的零星反光噪点
            blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

            _, binary_image = cv2.threshold(blurred_image, 185, 255, cv2.THRESH_BINARY)

            height, width = binary_image.shape
            middle = width // 2
            left_col = middle - 20
            right_col = middle + 20
            rows_found = []

            # 扫描范围：一共会扫描 9 列
            for col in range(left_col, right_col + 1, 5):
                for row in range(height - 2, 0, -1):
                    if binary_image[row, col] == 255 and binary_image[row + 1, col] == 0:
                        rows_found.append(row)
                        break

            # 要求 9 列扫描中，至少有 6 列都找到了跳变点，才算是一条连贯的横线
            if len(rows_found) >= 6:
                # 前线/后线使用不同的平均行号阈值
                if self.filtered_fps_num <= self.filtered_fps_num_threshold:
                    stop_threshold = self.stop_line_front_threshold
                else:
                    stop_threshold = self.stop_line_back_threshold
                if sum(rows_found) / len(rows_found) > stop_threshold:
                    if self.is_stop_line_front_found == False:
                        print("找到终点停车线的前线")
                        if self.follow_mode == 'right':
                            self.rotate_speed(20, 0.3)  # 以20度、0.3速度旋转，调整车头方向
                        elif self.follow_mode == 'left':
                            self.rotate_speed(-20, 0.3)  # 以20度、0.3速度旋转，调整车头方向
                        rospy.sleep(0.3)
                        self.is_stop_line_front_found = True
                        return False
                    elif self.filtered_fps_num>=self.filtered_fps_num_threshold :
                        print("找到终点停车线的后端，准备停车")
                        self.stop_robot()
                        return  True

            return False

    def execute_barrier_detour(self):
        """
        纯雷达避障绕行状态机：
          lidar_align(雷达法线对齐) → lateral_shift(侧向平移绕过)
          → forward_pass(前冲通过) → return_lateral_shift(侧向平移回赛道) → done
        雷达持续收集点云，距离 < 0.30m 时自动触发绕行（在 image_callback 中检测）。
        不后退，直接用法线对齐 + 侧向平移绕过。
        """
        if self.barrier_state in ('none', 'done'):
            return False

        # 绕行中始终关闭雷达，避免雷达把障碍物误当停车线，也防止重复触发
        self.barrier_lidar_enabled = False

        twist = Twist()

        # -------- lidar_align: 雷达计算法线 + 旋转对齐挡板 --------
        if self.barrier_state == 'lidar_align':
            points = self.barrier_lidar_points

            if len(points) < 2:
                rospy.logwarn("[避障] 雷达挡板点不足(%d)，跳过本次绕行，恢复巡线" % len(points))
                self.barrier_state = 'none'
                self._reset_lidar_only()
                return False

            try:
                (nx, ny), (cx, cy) = self._calculate_external_normal(*points)

                # 计算法线与雷达 x 轴（正前方）的夹角
                angle = self._vector_to_angle(nx, ny)
                rospy.loginfo("[避障] 挡板法向量: (%.3f, %.3f), 夹角: %.1f°" % (nx, ny, angle))

                # 旋转小车，使车头正对挡板法线方向
                self.rotate_speed(angle, self.barrier_rotate_speed)
                rospy.sleep(0.3)

                # 使用点云中心距离作为参考，只前进不后退
                # 触发距离(0.30m)已经足够近，不需要后退；只在太远时前进靠近
                current_dist = math.hypot(cx, cy)
                rospy.loginfo("[避障] 当前距离: %.2fm, 期望距离: %.2fm" % (
                    current_dist, self.barrier_align_distance))

                # 只在距离过大时前进靠近，距离偏小时不做后退（避免压线）
                if current_dist > self.barrier_align_distance + 0.05:
                    adjust_time = (current_dist - self.barrier_align_distance) / 0.15
                    adjust_start = rospy.Time.now()
                    rospy.loginfo("[避障] 距离过大，前进调整 %.2fs" % adjust_time)
                    while (rospy.Time.now() - adjust_start).to_sec() < adjust_time:
                        twist.linear.x = 0.15
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist)
                        rospy.sleep(0.05)

                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                rospy.sleep(0.2)

                self.barrier_state = 'lateral_shift'
                self.barrier_phase_start_time = rospy.Time.now()
                self.barrier_normal_angle = angle
                rospy.loginfo("[避障] 法线对齐完成 → 侧向平移绕过")
                return True

            except Exception as e:
                rospy.logerr("[避障] 法线计算异常: %s，跳过本次绕行，恢复巡线" % str(e))
                self.barrier_state = 'none'
                self._reset_lidar_only()
                return False

        # -------- lateral_shift: 侧向平移绕过挡板 --------
        # 直行/左行: 右移绕开 (linear.y < 0); 右行: 左移绕开 (linear.y > 0)
        elif self.barrier_state == 'lateral_shift':
            elapsed = (rospy.Time.now() - self.barrier_phase_start_time).to_sec()
            if elapsed < self.barrier_lateral_time:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                twist.linear.y = -self.barrier_lateral_speed if self.follow_mode in ('left', 'straight') else self.barrier_lateral_speed
                self.cmd_pub.publish(twist)
                return True
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            rospy.sleep(0.2)
            self.barrier_state = 'forward_pass'
            self.barrier_phase_start_time = rospy.Time.now()
            rospy.loginfo("[避障] 侧向平移完成 → 前冲通过挡板区域")
            return True

        # -------- forward_pass: 前冲通过挡板区域 --------
        elif self.barrier_state == 'forward_pass':
            elapsed = (rospy.Time.now() - self.barrier_phase_start_time).to_sec()
            if elapsed < self.barrier_forward_pass_time:
                twist.linear.x = self.barrier_forward_speed
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return True
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            rospy.sleep(0.2)
            self.barrier_state = 'return_lateral_shift'
            self.barrier_phase_start_time = rospy.Time.now()
            rospy.loginfo("[避障] 前冲完成 → 侧向平移回赛道")
            return True

        # -------- return_lateral_shift: 侧向平移回赛道（开环，时间控制距离）--------
        # 直行/左行: 左移回 (linear.y > 0); 右行: 右移回 (linear.y < 0)
        # 侧移时间由 barrier_return_lateral_time 控制，侧移距离 = lateral_speed × return_lateral_time
        elif self.barrier_state == 'return_lateral_shift':
            elapsed = (rospy.Time.now() - self.barrier_phase_start_time).to_sec()
            if elapsed < self.barrier_return_lateral_time:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                twist.linear.y = self.barrier_lateral_speed if self.follow_mode in ('left', 'straight') else -self.barrier_lateral_speed
                self.cmd_pub.publish(twist)
                return True
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            rospy.sleep(0.2)
            # 回程后微调旋转：朝回程反方向转一点，摆正车头避免反向冲出
            # 左转/直行: 回程左移回 → 向右微调 (负角度)
            # 右转:      回程右移回 → 向左微调 (正角度)
            correct_angle = -self.barrier_return_correct_angle if self.follow_mode in ('left', 'straight') else self.barrier_return_correct_angle
            self.rotate_speed(correct_angle, self.barrier_rotate_speed)
            self.barrier_state = 'done'
            self._reset_after_detour()
            rospy.loginfo("[避障] 回程完成 (%.1fs) + 反向微调 %d° → 恢复正常巡线" % (
                self.barrier_return_lateral_time, correct_angle))
            return False

        return False

    def _reset_lidar_only(self):
        """仅重置雷达缓存，不清除巡线状态，保持雷达启用"""
        self.barrier_lidar_points = []
        self.barrier_lidar_min_distance = float('inf')
        self.barrier_normal_angle = 0.0

    def _reset_after_detour(self):
        """绕行完成后重置巡线状态，恢复原巡线任务"""
        self.start_time = rospy.Time.now()
        self.sum_pid = 0.0
        self.last_error = 0.0
        self.pid_count = 0
        # 重新启动倒计时
        self.is_post_rotate_finished = False
        self.post_rotate_start_time = rospy.Time.now()  # 重新开始倒计时
        self.is_stop_line_front_found = False
        self.filtered_fps_num = 0
        self.play_voice_once = False
        # 绕行完成后关闭雷达、标记已触发过一次，障碍物只出现一次
        self.barrier_lidar_enabled = False
        self.barrier_done_once = True
        # 清除雷达辅助状态
        self.barrier_lidar_points = []
        self.barrier_lidar_min_distance = float('inf')
        self.barrier_normal_angle = 0.0
        rospy.loginfo("[避障] 绕行完成，雷达已关闭，障碍物不会再触发")

    def execute_pid_control(self, error):
        self.pid_count += 1
        if self.pid_count > 50:
            self.pid_count = 0
            self.sum_pid = 0.0

        self.sum_pid += error
        d_pid = error - self.last_error
        angular_z = (error * self.Kp) + (self.sum_pid * self.Ki) + (d_pid * self.Kd)
        # self.last_error = error
        self.last_error = max(-5.0, min(5.0, error))
        if self.Is_eight_finished == False:
            # 阶段①：八邻域找特殊点
            linear_x = max(0.1, 0.30 - abs(error) * 0.042)
        elif self.is_stop_line_front_found == True:
            # 阶段④：找终点停车线后线
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)
        elif self.is_post_rotate_finished == True:
            # 阶段③：找终点停车线前线
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)
        else:
            # 阶段②：正常巡线（拐点旋转完成，等待倒计时结束）
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


    @staticmethod
    def _calculate_external_normal(*points):
        """
        利用 PCA (主成分分析) 对雷达提取的挡板点云进行直线拟合，计算出挡板的法向量。
        与 find.py 中的 calculate_external_normal 逻辑一致。
        :param points: 雷达扫描到的挡板上的坐标点集合 (x,y)
        :return: (法向量单位矢量 (nx, ny)), (点云中心点 (cx, cy))
        """
        if len(points) < 2:
            raise ValueError("At least 2 points are required to fit a line")

        pts = np.array(points)
        x = pts[:, 0]
        y = pts[:, 1]

        x_mean = np.mean(x)
        y_mean = np.mean(y)

        cov_matrix = np.cov(x - x_mean, y - y_mean)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        normal_vector = eigenvectors[:, 0]

        a, b = normal_vector
        C = -(a * x_mean + b * y_mean)

        # 统一法线方向：指向挡板外侧（正对小车）
        if C > 1e-10:
            a, b = -a, -b

        norm = np.hypot(a, b)
        return (a / norm, b / norm), (x_mean, y_mean)

    @staticmethod
    def _vector_to_angle(nx, ny):
        """
        将方向向量转换为与 x 轴的夹角（度）。
        与 find.py 中的 vector_to_angle 逻辑一致。
        """
        if math.isclose(nx, 0.0, abs_tol=1e-9) and math.isclose(ny, 0.0, abs_tol=1e-9):
            raise ValueError("Cannot determine angle for zero vector (nx=0, ny=0)")
        angle_rad = math.atan2(ny, nx)
        return math.degrees(angle_rad)

    def barrier_lidar_callback(self, msg):
        """
        雷达回调函数：当 barrier_lidar_enabled 为 True 时，
        持续收集正前方 ±barrier_lidar_angle_range 范围内的有效点云。
        同时记录最近距离，供 image_callback 中的距离触发判断使用。
        绕行过程中和绕行完成后关闭，避免重复触发或干扰停车线检测。
        """
        if not self.barrier_lidar_enabled:
            return

        points = []
        min_distance = float('inf')
        half_range = self.barrier_lidar_angle_range

        for deg in range(-half_range, half_range + 1):
            angle_rad = math.radians(deg)
            idx = int(round((angle_rad - msg.angle_min) / msg.angle_increment))

            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                if msg.range_min <= r <= msg.range_max:
                    x = r * math.cos(angle_rad)
                    y = r * math.sin(angle_rad)
                    points.append((x, y))
                    if r < min_distance:
                        min_distance = r

        self.barrier_lidar_points = points
        self.barrier_lidar_min_distance = min_distance

    def stop_robot(self):
        rospy.loginfo("检测到停止线，执行停车程序！")
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = 0.0, 0.0, 0.0
        self.cmd_pub.publish(twist)
        self.finish_pub.publish(True)
        self.stop_flag = True
        wav_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '', 'wav', 'finish.wav')
        play_wav(wav_path, p)
        rospy.signal_shutdown("到达终点，巡线节点正常关闭。")

    def image_callback(self, msg):
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

            # ==================================================
            # 0. 初始车头对正逻辑（优先级最高）
            # ==================================================
            if not self.is_initial_aligned:
                angular_speed = vtherror * self.align_Kp
                angular_speed = max(-self.align_max_speed, min(self.align_max_speed, angular_speed))
                if 0 < abs(angular_speed) < self.align_min_speed:
                    angular_speed = self.align_min_speed if angular_speed > 0 else -self.align_min_speed

                if abs(vtherror) <= self.initial_align_threshold:
                    self.align_stable_count += 1
                    if self.align_stable_count >= self.align_stable_required:
                        self.is_initial_aligned = True
                        self.align_stable_count = 0
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist)
                        rospy.loginfo("初始车头已对正！准备直线前进进入赛道...")
                        return
                    else:
                        rospy.loginfo_throttle(
                            1.0,
                            f"对正稳定中... ({self.align_stable_count}/{self.align_stable_required}) "
                            f"偏差: {vtherror:.2f}"
                        )
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist)
                        return
                else:
                    self.align_stable_count = 0
                    rospy.loginfo_throttle(
                        0.5,
                        f"初始车头未对正 (偏差: {vtherror:.2f})，"
                        f"转速: {angular_speed:.3f} rad/s"
                    )
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = angular_speed
                    self.cmd_pub.publish(twist)
                    return

            # ==================================================
            # 0.5 初始对正后，先直线前进一段，再正式开始巡线
            # ==================================================
            if not self.is_initial_forward_done:
                if self.initial_forward_start_time is None:
                    self.initial_forward_start_time = rospy.Time.now()
                    rospy.loginfo(f"车头已对正，直线前进 {self.initial_forward_duration} 秒后开始巡线...")

                elapsed_forward = (rospy.Time.now() - self.initial_forward_start_time).to_sec()
                if elapsed_forward < self.initial_forward_duration:
                    twist = Twist()
                    twist.linear.x = 0.20
                    angular_correction = vtherror * 0.02
                    twist.angular.z = max(-0.25, min(0.25, angular_correction))
                    self.cmd_pub.publish(twist)
                    return
                else:
                    self.is_initial_forward_done = True
                    self.start_time = rospy.Time.now()
                    rospy.loginfo("初始前进完成！开启八邻域倒计时，开始正常巡线。")
                    return

            # ==================================================
            # 【雷达避障检测】仅在正常巡线阶段检测，且障碍物只触发一次
            # 条件：启用绕行 + 未在绕行中 + 初始前进已完成 + 雷达启用 + 未完成过绕行
            # 雷达距离 < 0.30m 时直接触发绕行（纯雷达，不依赖视觉）
            # ==================================================
            if (self.barrier_detect_enabled and self.barrier_state == 'none'
                    and self.is_initial_forward_done and not self.barrier_done_once
                    and self.barrier_lidar_enabled):
                if self.barrier_lidar_min_distance < self.barrier_lidar_trigger_distance:
                    rospy.loginfo("[避障] 雷达检测到障碍物！距离: %.2fm < %.2fm → 触发绕行" % (
                        self.barrier_lidar_min_distance, self.barrier_lidar_trigger_distance))
                    self.barrier_state = 'lidar_align'
                    self.barrier_phase_start_time = rospy.Time.now()
                    # 立即停车
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.sleep(0.2)

            # ==================================================
            # 【新增】挡板绕行状态机（优先级高于一切巡线逻辑）
            # ==================================================
            if self.execute_barrier_detour():
                return

            # ==================================================
            # 终点停车检测
            # ==================================================
            if self.is_post_rotate_finished == True :
                if self.detect_stop_line(image):
                    self.stop_robot()
                    print(f"过滤帧数{self.filtered_fps_num}")
                    print ("检测到终点停车线，执行停车程序！")
                    return

            # ==================================================
            # 1. 定时器与拐点处理逻辑
            # ==================================================
            if self.Is_eight_finished == False:
                elapsed_time = (rospy.Time.now() - self.start_time).to_sec()

                if elapsed_time >= self.timer_duration:
                    if self.follow_mode == 'straight':
                        self.eight_deal_low=70
                    tracker_result = self.tracker.process(
                        binary_mask=mask,
                        target_size=(self.img_width, self.img_height),
                        deal_low=self.eight_deal_low,
                        deal_high=self.eight_deal_high
                    )
                    if tracker_result is not None:

                        # ================= 1. 参数设置区 =================
                        if self.follow_mode == 'left':
                            dir_idx = 3
                            threshold = 6        # left 状态的 threshold
                            rotate_val = 80
                        elif self.follow_mode == 'right':
                            dir_idx = 3
                            threshold = 6        # left 状态的 threshold
                            rotate_val = -70
                        elif self.follow_mode == 'straight':
                            dir_idx = 2
                            threshold = 30        # straight 状态的 threshold
                            rotate_val = -70
                        else:
                            return  # 如果是其他未知模式，直接跳过不处理

                        # ================= 2. 数据获取区 =================
                        left_count = tracker_result['stats']['left_dirs'][dir_idx]
                        right_count = tracker_result['stats']['right_dirs'][dir_idx]

                        print(f"当前模式:{self.follow_mode} | 方向:{dir_idx} | 左点数:{left_count} | 右点数:{right_count} | 阈值:{threshold}")

                        # ================= 3. 核心执行区 =================
                        if (left_count > threshold or right_count > threshold) or self.rotate_one_flag == 1:

                            if self.rotate_one_flag == 0:
                                rospy.loginfo(f"检测到左右点数超过 {threshold}，发现拐点！准备以 {rotate_val} 速度旋转。")
                                self.rotate_speed(rotate_val, 0.8)
                                self.rotate_one_flag = 1
                                return

                            else:
                                if abs(vtherror) <= self.max_rotate_error_cross1:
                                    twist = Twist()
                                    twist.angular.z = 0.0
                                    self.cmd_pub.publish(twist)
                                    self.Is_eight_finished = True
                                    rospy.loginfo("拐点处旋转处理完成，即将恢复常规PID巡线。")
                                    # === 启动倒计时 ===
                                    self.post_rotate_start_time = rospy.Time.now()
                                    rospy.loginfo("拐点处旋转处理完成，开启后续倒计时，即将恢复常规PID巡线。")
                                    return
                                else:
                                    twist = Twist()
                                    twist.linear.x = 0.0
                                    if vtherror > 0:
                                        twist.angular.z = 0.3
                                    else:
                                        twist.angular.z = -0.3
                                    self.cmd_pub.publish(twist)
                                    return

            # ==================================================
            # 检查拐点旋转后的倒计时是否结束
            # ==================================================
            if self.Is_eight_finished ==  True and self.post_rotate_start_time is not None :
                if not self.is_post_rotate_finished:
                    elapsed_post_time = (rospy.Time.now() - self.post_rotate_start_time).to_sec()
                    if elapsed_post_time >= self.post_rotate_duration:
                        self.is_post_rotate_finished = True
                        rospy.loginfo("旋转后倒计时结束，标志位已置为 True！开始检测终点停车线")

            # ==================================================
            # 2. 常规 PID 控制输出
            # ==================================================
            self.execute_pid_control(vtherror)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge 转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"巡线过程发生异常: {e}")


if __name__ == '__main__':
    try:
        node = LineFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
