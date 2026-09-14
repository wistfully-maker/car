#!/usr/bin/env python3
import cv2
import numpy as np

# 窗口名称
WIN_NAME = "HSV Threshold Tuner"
cv2.namedWindow(WIN_NAME)

# ==================== 初始阈值（和巡线代码默认保持一致） ====================
init_h_min = 0
init_h_max = 179
init_s_min = 0
init_s_max = 13
init_v_min = 121
init_v_max = 233
kernel_size = 3

# 创建滑动条
cv2.createTrackbar("H_MIN", WIN_NAME, init_h_min, 179, lambda x: None)
cv2.createTrackbar("H_MAX", WIN_NAME, init_h_max, 179, lambda x: None)
cv2.createTrackbar("S_MIN", WIN_NAME, init_s_min, 255, lambda x: None)
cv2.createTrackbar("S_MAX", WIN_NAME, init_s_max, 255, lambda x: None)
cv2.createTrackbar("V_MIN", WIN_NAME, init_v_min, 255, lambda x: None)
cv2.createTrackbar("V_MAX", WIN_NAME, init_v_max, 255, lambda x: None)
cv2.createTrackbar("Kernel", WIN_NAME, kernel_size, 9, lambda x: None)

# 打开摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

print("操作说明：")
print("1. 拖动滑块调节阈值，观察右侧Mask图像")
print("2. 白线纯白、背景无杂点为最佳效果")
print("3. 按下【q】键退出，终端会打印最终阈值，可以直接复制到巡线代码！")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # 获取滑块数值
    h_min = cv2.getTrackbarPos("H_MIN", WIN_NAME)
    h_max = cv2.getTrackbarPos("H_MAX", WIN_NAME)
    s_min = cv2.getTrackbarPos("S_MIN", WIN_NAME)
    s_max = cv2.getTrackbarPos("S_MAX", WIN_NAME)
    v_min = cv2.getTrackbarPos("V_MIN", WIN_NAME)
    v_max = cv2.getTrackbarPos("V_MAX", WIN_NAME)
    k_size = cv2.getTrackbarPos("Kernel", WIN_NAME)
    if k_size < 1:
        k_size = 1
    if k_size % 2 == 0:
        k_size += 1

    # HSV转换
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)

    # ✅修复：正确形态学调用方式
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_size, k_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 拼接原图与mask并排展示
    mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    combine = np.hstack([frame, mask_color])
    cv2.imshow(WIN_NAME, combine)

    # q退出
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\n==================== 最终阈值（直接复制！） ====================")
        print(f"LOWER_WHITE = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"UPPER_WHITE = np.array([{h_max}, {s_max}, {v_max}])")
        print(f"KERNEL_SIZE = ({k_size}, {k_size})")
        print("===============================================================")
        break

cap.release()
cv2.destroyAllWindows()

