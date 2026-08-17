#!/usr/bin/env python3
import cv2
import numpy as np
import time

# 初始阈值，和巡线程序保持一致
LOWER = np.array([0, 0, 121])
UPPER = np.array([179, 13, 233])
KERNEL_SIZE = 3

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("摄像头打开失败！")
    exit()

print("==== 终端版HSV调参工具 ====")
print("指令：")
print(" h/H：减小/增大 H_MIN")
print(" j/J：减小/增大 S_MIN")
print(" k/K：减小/增大 V_MIN")
print(" u/U：减小/增大 H_MAX")
print(" i/I：减小/增大 S_MAX")
print(" o/O：减小/增大 V_MAX")
print(" m/M：减小/增大 形态学核大小")
print(" r：重置初始阈值")
print(" q：退出并输出最终阈值\n")

step = 2  # 每次调节步长

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    blur = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)

    # 8邻域形态学处理
    k = KERNEL_SIZE
    if k < 1: k = 1
    if k % 2 == 0: k += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(k,k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 统计白色像素（赛道区域）
    white_pixel = cv2.countNonZero(mask)
    total_pixel = mask.shape[0] * mask.shape[1]
    ratio = white_pixel / total_pixel

    print(f"\rLOW={list(LOWER)} | UPPER={list(UPPER)} | kernel={k} | 白色占比:{ratio:.3f}", end="")

    # 读取键盘输入
    key = cv2.waitKeyEx(10) & 0xFF
    if key == ord('q'):
        print("\n\n==========最终阈值【直接复制】==========")
        print(f"LOWER_WHITE = np.array({list(LOWER)})")
        print(f"UPPER_WHITE = np.array({list(UPPER)})")
        print(f"KERNEL_SIZE = ({k}, {k})")
        break
    elif key == ord('h'): LOWER[0] -= step
    elif key == ord('H'): LOWER[0] += step
    elif key == ord('u'): UPPER[0] -= step
    elif key == ord('U'): UPPER[0] += step
    elif key == ord('j'): LOWER[1] -= step
    elif key == ord('J'): LOWER[1] += step
    elif key == ord('i'): UPPER[1] -= step
    elif key == ord('I'): UPPER[1] += step
    elif key == ord('k'): LOWER[2] -= step
    elif key == ord('K'): LOWER[2] += step
    elif key == ord('o'): UPPER[2] -= step
    elif key == ord('O'): UPPER[2] += step
    elif key == ord('m'): KERNEL_SIZE -= 1
    elif key == ord('M'): KERNEL_SIZE += 1
    elif key == ord('r'):
        LOWER = np.array([0, 0, 121])
        UPPER = np.array([179, 13, 233])
        KERNEL_SIZE = 3

    # 限制合法范围
    LOWER = np.clip(LOWER, [0,0,0], [179,255,255])
    UPPER = np.clip(UPPER, LOWER, [179,255,255])

cap.release()
cv2.destroyAllWindows()

