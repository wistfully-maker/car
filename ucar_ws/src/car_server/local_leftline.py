#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist
import http.server
import socketserver
import threading
from io import BytesIO

# =====================【视觉参数】=====================
LOWER_WHITE = np.array([0, 0, 102])
UPPER_WHITE = np.array([101, 49, 240])
CUT_RATIO = 0.63
MIN_AREA = 40

# PID控制参数
KP = 0.004
KD = 0.0007
BASE_LINEAR_SPEED = 0.07

# =====================【通用配置】=====================
ROAD_HALF_WIDTH = 180
DEBUG_MODE = True
MAX_ANGULAR = 0.5

# ==========三岔路口控制参数【左侧路线固定左转】==========
INTERSECTION_THRESHOLD = 3
TURN_ANGULAR_SPEED = 0.33
TURN_DURATION = 1.75
INTERSECTION_COOLDOWN = 3.0
# ==============================================================

last_error = 0
in_intersection = False
turn_start_time = 0
last_intersection_time = 0
# 全局共享图像，用于网页推流
stream_frame = None

# 网页视频流服务
class StreamHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                if stream_frame is not None:
                    _, jpg = cv2.imencode('.jpg', stream_frame, [int(cv2.IMWRITE_JPEG_QUALITY),70])
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
                time.sleep(0.06)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = '''
            <html><body style="background:#000;text-align:center;">
            <h2 style="color:white;">UCAR ROI + Mask 实时画面</h2>
            <img src="/stream" width="800">
            </body></html>'''
            self.wfile.write(html.encode())

def start_stream_server():
    with socketserver.TCPServer(("",8080), StreamHandler) as httpd:
        print("🌐 视频流服务启动: http://172.20.10.4:8080")
        httpd.serve_forever()

def main():
    global last_error, DEBUG_MODE, in_intersection, turn_start_time, last_intersection_time, stream_frame
    # 启动网页推流子线程
    threading.Thread(target=start_stream_server, daemon=True).start()

    rospy.init_node("line_follow_left_node")
    pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    twist = Twist()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        rospy.logerr("摄像头打开失败！")
        return
    rospy.loginfo("【左侧赛道巡线程序启动，目标：三岔左转驶向P2】")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        now = time.time()
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("读取图像失败")
            rate.sleep()
            continue

        h_img, w_img = frame.shape[:2]
        y_start = int(h_img * CUT_RATIO)
        frame_roi = frame[y_start:h_img, :]
        roi_h, roi_w = frame_roi.shape[:2]

        # 图像预处理
        blur = cv2.GaussianBlur(frame_roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 8邻域连通域筛选白线
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        valid_centers = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > MIN_AREA:
                cx, cy = centroids[i]
                valid_centers.append([cx, cy])

        # 三岔路口双重判定：白线≥3 并且远处存在2条以上岔道斜线
        far_line_count = 0
        for (cx, cy) in valid_centers:
            if cy < roi_h * 0.55:
                far_line_count +=1

        if not in_intersection and (now - last_intersection_time > INTERSECTION_COOLDOWN):
            if len(valid_centers) >= INTERSECTION_THRESHOLD and far_line_count >=2:
                rospy.loginfo("========检测三岔路口，执行左转进入左车道========")
                in_intersection = True
                turn_start_time = now

        # 左转执行逻辑
        if in_intersection:
            twist.linear.x = BASE_LINEAR_SPEED
            twist.angular.z = TURN_ANGULAR_SPEED
            pub_twist.publish(twist)
            if now - turn_start_time > TURN_DURATION:
                rospy.loginfo("=======左转完成，恢复正常双线巡线=======")
                in_intersection = False
                last_intersection_time = now
            # 拼接用于网页推送的合成画面
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            stream_frame = np.hstack([frame_roi, mask_color])
            rate.sleep()
            continue

        # ==========正常直道巡线逻辑==========
        center_target_x = w_img / 2.0
        left_line = None
        right_line = None
        mid_x = None

        if len(valid_centers) >= 2:
            valid_centers.sort(key=lambda p: p[0])
            left_side = [p for p in valid_centers if p[0] < center_target_x]
            right_side = [p for p in valid_centers if p[0] > center_target_x]
            if left_side and right_side:
                left_line = left_side[0]
                right_line = right_side[-1]
        elif len(valid_centers) == 1:
            if valid_centers[0][0] < center_target_x:
                left_line = valid_centers[0]
            else:
                right_line = valid_centers[0]

        error = 0
        if left_line is not None and right_line is not None:
            mid_x = (left_line[0] + right_line[0]) / 2.0
            error = mid_x - center_target_x
        elif left_line is not None:
            cx, cy = left_line
            if cy > roi_h * 0.5:
                mid_x = left_line[0] + ROAD_HALF_WIDTH
                error = mid_x - center_target_x
            else:
                error = 0
        elif right_line is not None:
            cx, cy = right_line
            if cy > roi_h * 0.5:
                mid_x = right_line[0] - ROAD_HALF_WIDTH
                error = mid_x - center_target_x
            else:
                error = 0

        # PID计算转向
        angular = KP * error + KD * (error - last_error)
        last_error = error
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

        twist.linear.x = BASE_LINEAR_SPEED
        twist.angular.z = angular
        pub_twist.publish(twist)

        # 绘制标记点
        debug_img = frame_roi.copy()
        cv2.line(debug_img, (int(center_target_x), 0),
                (int(center_target_x), debug_img.shape[0]), (0, 255, 0), 1)
        if left_line is not None:
            cv2.circle(debug_img, (int(left_line[0]), int(left_line[1])),
                      8, (255, 0, 0), -1)
        if right_line is not None:
            cv2.circle(debug_img, (int(right_line[0]), int(right_line[1])),
                      8, (0, 0, 255), -1)
        if mid_x is not None:
            cv2.circle(debug_img, (int(mid_x), debug_img.shape[0]//2),
                      8, (0, 255, 255), -1)

        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        stream_frame = np.hstack([debug_img, mask_color])

        if DEBUG_MODE:
            rospy.loginfo_throttle(1,
                "左白线:%s 右白线:%s 连通块数量:%d 远处斜线:%d 偏差:%.1f 角速度:%.3f 转弯状态:%s" % (
                    "√" if left_line else "×",
                    "√" if right_line else "×",
                    len(valid_centers), far_line_count,
                    error, angular,
                    "转弯中" if in_intersection else "正常巡线"))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):
            DEBUG_MODE = not DEBUG_MODE
        rate.sleep()

    # 程序退出，小车停止
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub_twist.publish(twist)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        pub_twist.publish(twist)

