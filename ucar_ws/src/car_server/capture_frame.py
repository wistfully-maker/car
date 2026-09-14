#!/usr/bin/env python3
"""抓一帧图片保存到 /tmp/track_sample.jpg"""
import cv2, time
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
time.sleep(1)
ret, frame = cap.read()
if ret:
    cv2.imwrite('/tmp/track_sample.jpg', frame)
    print('✅ 已保存 /tmp/track_sample.jpg')
else:
    print('❌ 读取失败')
cap.release()
