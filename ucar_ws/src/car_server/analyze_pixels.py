#!/usr/bin/env python3
"""
现场像素分析工具
抓一帧图 → 在图上点几个位置 → 输出每个点的RGB/HSV值
用于区分白线 vs 反光的真实像素差异
"""
import cv2
import numpy as np
import sys
import os

# 抓图
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
for _ in range(10):  # 跳过前几帧
    cap.read()
ret, frame = cap.read()
cap.release()

if not ret:
    print("❌ 抓图失败")
    sys.exit(1)

# 保存原图
path = "/tmp/track_analyze.jpg"
cv2.imwrite(path, frame)
print(f"✅ 图片已保存: {path}")
print(f"   尺寸: {frame.shape[1]}x{frame.shape[0]}")
print()

# 分析：把图从上到下分成几个区域，每个区域统计HSV
h, w = frame.shape[:2]
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# 分区统计
regions = [
    ("画面底部1/4 (最近处)", int(h*0.75), h),
    ("画面中下部1/4",        int(h*0.50), int(h*0.75)),
    ("画面中上部1/4",        int(h*0.25), int(h*0.50)),
    ("画面顶部1/4 (远处)",   0,           int(h*0.25)),
]

for name, y0, y1 in regions:
    region = hsv[y0:y1, :, :]
    h_chan = region[:, :, 0]
    s_chan = region[:, :, 1]
    v_chan = region[:, :, 2]

    # V > 120 的"亮"像素（候选白线/反光）
    bright = v_chan > 120
    bright_count = np.sum(bright)
    total = bright.size

    print(f"【{name}】亮像素 {bright_count}/{total} ({100*bright_count/total:.1f}%)")
    if bright_count > 0:
        print(f"  亮区 H: [{np.min(h_chan[bright]):.0f} ~ {np.max(h_chan[bright]):.0f}]")
        print(f"  亮区 S: [{np.min(s_chan[bright]):.0f} ~ {np.max(s_chan[bright]):.0f}]")
        print(f"  亮区 V: [{np.min(v_chan[bright]):.0f} ~ {np.max(v_chan[bright]):.0f}]")

        # 看V分布
        v_bright = v_chan[bright]
        print(f"  亮区V分布: 10%={np.percentile(v_bright,10):.0f} "
              f"50%={np.percentile(v_bright,50):.0f} "
              f"90%={np.percentile(v_bright,90):.0f} "
              f"99%={np.percentile(v_bright,99):.0f}")

        # 有多少亮像素是"暴表"的(V=250+)
        overexposed = np.sum(v_chan > 250)
        print(f"  暴表(V>250): {overexposed} 像素 ({100*overexposed/total:.1f}%)")

        # S通道分析：纯白S低，有色反光S高
        s_bright = s_chan[bright]
        low_s = np.sum(s_bright < 30)
        print(f"  低饱和(S<30): {low_s}/{bright_count} ({100*low_s/bright_count:.1f}%)")
    print()

# 最后输出建议
print("=" * 60)
print("💡 调参建议：")
v_all = hsv[:, :, 2]
bright_all = v_all > 120
if np.sum(bright_all) > 0:
    v_b = v_all[bright_all]
    p50 = np.percentile(v_b, 50)
    p90 = np.percentile(v_b, 90)
    p99 = np.percentile(v_b, 99)
    print(f"  V中位数={p50:.0f}  V90分位={p90:.0f}  V99分位={p99:.0f}")
    print(f"  如果白线V≈{p50:.0f}~{p90:.0f}, 反光V≈{p90:.0f}~{p99:.0f}")
    print(f"  则 V上限应设为 {p90:.0f} 左右来切掉反光")
print("=" * 60)
