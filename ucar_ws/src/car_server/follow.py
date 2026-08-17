#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
import wave
import pyaudio

p = pyaudio.PyAudio()

class ContourCornerTracker:
    def __init__(self, max_points=200):
        self.max_points = max_points

    @staticmethod
    def _direction_code(dx, dy):
        if dx == 0 and dy > 0:
            return 0
        if dx > 0 and dy > 0:
            return 1
        if dx > 0 and dy == 0:
            return 2
        if dx > 0 and dy < 0:
            return 3
        if dx == 0 and dy < 0:
            return 4
        if dx < 0 and dy < 0:
            return 5
        if dx < 0 and dy == 0:
            return 6
        if dx < 0 and dy > 0:
            return 7
        return None

    def _find_corner_points(self, contour, deal_high, deal_low):
        if len(contour) < 6:
            return []
        hull = cv2.convexHull(contour, returnPoints=False)
        if hull is None or len(hull) < 3:
            return []
        defects = cv2.convexityDefects(contour, hull)
        if defects is None:
            return []
        corners = []
        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            far = tuple(contour[f][0])
            if deal_high <= far[1] <= deal_low:
                corners.append(far)
        return corners

    def process(self, binary_mask, target_size=(160, 120), deal_low=87, deal_high=117):
        frame = cv2.resize(binary_mask, target_size)
        filtered = cv2.medianBlur(frame, 3)
        if deal_high > deal_low:
            deal_high, deal_low = deal_low, deal_high

        result = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = result[0] if len(result) == 2 else result[1]

        candidates = []
        for contour in contours:
            pts = [tuple(p[0]) for p in contour if deal_high <= p[0][1] <= deal_low]
            if len(pts) >= 3:
                mean_x = float(np.mean([pt[0] for pt in pts]))
                candidates.append((len(pts), mean_x, contour))

        if len(candidates) < 2:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        left_contour, right_contour = candidates[0][2], candidates[1][2]

        left_corners = self._find_corner_points(left_contour, deal_high, deal_low)
        right_corners = self._find_corner_points(right_contour, deal_high, deal_low)

        stats = {
            'left_dirs': {i: 0 for i in range(8)},
            'right_dirs': {i: 0 for i in range(8)}
        }
        for side_key, corners in (('left', left_corners), ('right', right_corners)):
            for pt in corners:
                dx = pt[0] - (target_size[0] // 2)
                dy = pt[1] - (target_size[1] // 2)
                ang = math.degrees(math.atan2(dy, dx))
                code = int(((ang + 180) / 45.0) % 8)
                stats[side_key + '_dirs'][code] += 1

        meet = abs(candidates[0][1] - candidates[1][1]) < 2

        return {
            'points_L': np.asarray(left_corners, dtype=np.uint16),
            'points_R': np.asarray(right_corners, dtype=np.uint16),
            'dir_L': np.zeros(len(left_corners), dtype=np.uint16),
            'dir_R': np.zeros(len(right_corners), dtype=np.uint16),
            'stats': stats,
            'meet': meet
        }

def play_wav(file_path, p):
    with wave.open(file_path, 'rb') as wf:
        stream = p.open(
            format=p.get_format_from_width(wf.getsampwidth()),
            channels=wf.getnchannels(),
            rate=wf.getframerate(),
            output=True,
            output_device_index=0
        )
        chunk = 1024
        data = wf.readframes(chunk)
        
        while data:
            stream.write(data)
            data = wf.readframes(chunk)
            
        stream.stop_stream()
        stream.close()

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('basic_line_follower', anonymous=True)

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.finish_pub = rospy.Publisher('/finish_follow', Bool, queue_size=10)
        
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/start_follow', String, self.start_follow_callback, queue_size=1)

        self.start_follow = False
        self.stop_flag = False
        self.follow_mode = 'right'
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
        self.align_Kp = 0.025
        self.align_max_speed = 0.8
        self.align_min_speed = 0.12
        self.align_stable_count = 0
        self.align_stable_required = 8

        self.is_initial_forward_done = False
        self.initial_forward_start_time = None
        self.initial_forward_duration = 0.3

        self.start_time = None
        self.timer_duration = 0.0
        self.Is_eight_finished = False
        self.rotate_one_flag = 0
        self.max_rotate_error_cross1 = 30.0
        self.Is_play_sound = False
        self.filtered_fps_num = 0
        self.tracker = ContourCornerTracker(max_points=200)

        self.post_rotate_duration = 5.0
        self.is_post_rotate_finished = False
        self.post_rotate_start_time = None
        self.is_stop_line_front_found = False
        self.play_voice_once = False
        self.filtered_fps_num_threshold = 30

        self.stop_line_row_start = 24
        self.stop_line_row_end = 119
        self.stop_line_front_threshold = 60
        self.stop_line_back_threshold = 60

        rospy.loginfo("基础视觉巡线节点已启动，等待图像数据...")

    def start_follow_callback(self, msg):
        self.is_initial_aligned = False
        self.is_initial_forward_done = False
        self.initial_forward_start_time = None
        self.align_stable_count = 0
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
            if self.filtered_fps_num >= self.filtered_fps_num_threshold:
                if self.play_voice_once == False:
                    twist = Twist()
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    print("过滤帧结束，即将检测到终点停车线的后线，准备停车！")
                    rospy.sleep(0.3)
                    self.play_voice_once = True

        cropped_image = image[self.stop_line_row_start:self.stop_line_row_end + 1, :]
        gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        _, binary_image = cv2.threshold(blurred_image, 185, 255, cv2.THRESH_BINARY)
        
        height, width = binary_image.shape
        middle = width // 2
        left_col = middle - 20
        right_col = middle + 20
        rows_found = []

        for col in range(left_col, right_col + 1, 5):
            for row in range(height - 2, 0, -1):
                if binary_image[row, col] == 255 and binary_image[row + 1, col] == 0:
                    rows_found.append(row)
                    break
                    
        if len(rows_found) >= 6:
            if self.filtered_fps_num <= self.filtered_fps_num_threshold:
                stop_threshold = self.stop_line_front_threshold
            else:
                stop_threshold = self.stop_line_back_threshold
            if sum(rows_found) / len(rows_found) > stop_threshold:
                if self.is_stop_line_front_found == False:
                    print("找到终点停车线的前线")
                    if self.follow_mode == 'right':
                        self.rotate_speed(20, 0.3)
                    elif self.follow_mode == 'left':
                        self.rotate_speed(-20, 0.3)
                    rospy.sleep(0.3)
                    self.is_stop_line_front_found = True
                    return False
                elif self.filtered_fps_num>=self.filtered_fps_num_threshold:
                    print("找到终点停车线的后端，准备停车")
                    self.stop_robot()
                    return True
                    
        return False

    def execute_pid_control(self, error):
        self.pid_count += 1
        if self.pid_count > 50:
            self.pid_count = 0
            self.sum_pid = 0.0

        self.sum_pid += error
        d_pid = error - self.last_error
        angular_z = (error * self.Kp) + (self.sum_pid * self.Ki) + (d_pid * self.Kd)
        self.last_error = max(-5.0, min(5.0, error))
        
        if self.Is_eight_finished == False:
            linear_x = max(0.1, 0.30 - abs(error) * 0.042)
        elif self.is_stop_line_front_found == True:
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)
        elif self.is_post_rotate_finished == True:
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)
        else:
            linear_x = max(0.1, 0.45 - abs(error) * 0.042)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

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

            if self.is_post_rotate_finished == True:
                if self.detect_stop_line(image):
                    self.stop_robot()
                    print(f"过滤帧数{self.filtered_fps_num}")
                    print ("检测到终点停车线，执行停车程序！")
                    return

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
                        if self.follow_mode == 'left':
                            dir_idx = 3
                            threshold = 1
                            rotate_val = 80
                        elif self.follow_mode == 'right':
                            dir_idx = 3
                            threshold = 1
                            rotate_val = -70
                        elif self.follow_mode == 'straight':
                            dir_idx = 2
                            threshold = 1
                            rotate_val = -70
                        else:
                            return

                        left_count = tracker_result['stats']['left_dirs'][dir_idx]
                        right_count = tracker_result['stats']['right_dirs'][dir_idx]
                        
                        print(f"当前模式:{self.follow_mode} | 方向:{dir_idx} | 左点数:{left_count} | 右点数:{right_count} | 阈值:{threshold}")
                        
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

            if self.Is_eight_finished == True and self.post_rotate_start_time is not None:
                if not self.is_post_rotate_finished:
                    elapsed_post_time = (rospy.Time.now() - self.post_rotate_start_time).to_sec()
                    if elapsed_post_time >= self.post_rotate_duration:
                        self.is_post_rotate_finished = True
                        rospy.loginfo("旋转后倒计时结束，标志位已置为 True！,开始检测终点停车线")
                        wav_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '', 'wav', 'start_check_parking.wav')

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
