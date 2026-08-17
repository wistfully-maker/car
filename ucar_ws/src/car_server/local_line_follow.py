#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

# =====================【视觉参数 无需改动】=====================
LOWER_WHITE = np.array([0, 0, 155])
UPPER_WHITE = np.array([40, 41, 231])
CUT_RATIO = 0.60
MIN_AREA = 40

# PID参数，现场可以微调
KP = 0.005
KD = 0.0007
BASE_LINEAR_SPEED = 0.07

# =====================【新增参数】=====================
ROAD_HALF_WIDTH = 180   # 道路半宽(像素)，单线丢线时的预估距离
DEBUG_MODE = True       # 调试模式开关，正式比赛改为False
MAX_ANGULAR = 0.5       # 角速度限幅，防止急转弯
# ==============================================================

last_error = 0

def main():
    global last_error, DEBUG_MODE
    rospy.init_node("line_follow_node")
    pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    twist = Twist()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        rospy.logerr("摄像头打开失败！")
        return
    rospy.loginfo("ROS巡线程序启动成功")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("读取图像失败")
            rate.sleep()
            continue

        h_img, w_img = frame.shape[:2]
        # ROI裁剪，舍弃画面上半部分
        y_start = int(h_img * CUT_RATIO)
        frame_roi = frame[y_start:h_img, :]

        # 图像预处理
        blur = cv2.GaussianBlur(frame_roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)

        # 形态学闭运算填补白线缝隙
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ========== 8邻域连通域识别 ==========
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        valid_centers = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area > MIN_AREA:
                cx, cy = centroids[i]
                valid_centers.append([cx, cy])

        # ========== 【优化1】改进左右线识别 ==========
        center_target_x = w_img / 2.0
        left_line = None
        right_line = None
        mid_x = None  # 提前初始化，解决变量未定义报错
        
        if len(valid_centers) >= 2:
            # 按x坐标排序
            valid_centers.sort(key=lambda p: p[0])
            
            # 改进：选择最左和最右，但验证合理性
            left_candidate = valid_centers[0]
            right_candidate = valid_centers[-1]
            
            # 验证左右线是否在正确的位置（左线在左边，右线在右边）
            if left_candidate[0] < center_target_x and right_candidate[0] > center_target_x:
                left_line = left_candidate
                right_line = right_candidate
            elif len(valid_centers) > 2:
                # 如果最左最右不合理，尝试找分别在两侧的最大面积点
                left_side = [p for p in valid_centers if p[0] < center_target_x]
                right_side = [p for p in valid_centers if p[0] > center_target_x]
                if left_side and right_side:
                    left_line = left_side[0]   # 最左边
                    right_line = right_side[-1] # 最右边
        elif len(valid_centers) == 1:
            # 【优化2】单条线丢线处理
            if valid_centers[0][0] < center_target_x:
                left_line = valid_centers[0]
            else:
                right_line = valid_centers[0]

        # ========== 【优化3】改进误差计算（含丢线处理） ==========
        error = 0
        if left_line is not None and right_line is not None:
            # 双线正常：计算道路中心
            mid_x = (left_line[0] + right_line[0]) / 2.0
            error = mid_x - center_target_x
        elif left_line is not None:
            # 只有左线：预估道路中心（左线右侧固定距离）
            mid_x = left_line[0] + ROAD_HALF_WIDTH
            error = mid_x - center_target_x
        elif right_line is not None:
            # 只有右线：预估道路中心（右线左侧固定距离）
            mid_x = right_line[0] - ROAD_HALF_WIDTH
            error = mid_x - center_target_x
        # 如果两条线都看不到，error保持为0，mid_x维持None

        # PID计算角速度
        angular = KP * error + KD * (error - last_error)
        last_error = error
        
        # 【优化4】角速度限幅，防止急转弯
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

        twist.linear.x = BASE_LINEAR_SPEED
        twist.angular.z = angular
        pub_twist.publish(twist)

        # ========== 【优化5】调试窗口增强 ==========
        if DEBUG_MODE:
            # 绘制调试信息
            debug_img = frame_roi.copy()
            
            # 图像中心线（绿色）
            cv2.line(debug_img, (int(center_target_x), 0), 
                    (int(center_target_x), debug_img.shape[0]), (0, 255, 0), 1)
            
            # 左线（蓝色圆点）
            if left_line is not None:
                cv2.circle(debug_img, (int(left_line[0]), int(left_line[1])), 
                          8, (255, 0, 0), -1)
            
            # 右线（红色圆点）
            if right_line is not None:
                cv2.circle(debug_img, (int(right_line[0]), int(right_line[1])), 
                          8, (0, 0, 255), -1)
            
            # 道路中心（黄色圆点）
            if mid_x is not None:
                cv2.circle(debug_img, (int(mid_x), debug_img.shape[0]//2), 
                          8, (0, 255, 255), -1)
            
            cv2.imshow("roi", debug_img)
            cv2.imshow("mask", mask)
            
            # 终端打印调试信息
            rospy.loginfo_throttle(1, 
                "L:%s R:%s Error:%.1f Angular:%.3f" % (
                    "Y" if left_line else "N",
                    "Y" if right_line else "N",
                    error, angular))
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):  # 按D键切换调试模式
            DEBUG_MODE = not DEBUG_MODE
            if not DEBUG_MODE:
                cv2.destroyAllWindows()

        rate.sleep()

    # 停止小车
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub_twist.publish(twist)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

