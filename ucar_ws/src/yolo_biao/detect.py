#!/usr/bin/env python3
"""
旋转扫描 + YOLO 检测停止
========================
替代 rostopic pub 命令。发布 /cmd_vel 旋转同时轮询 YOLO 置信度。

用法:
    python3 rotate_detect.py                   # 默认: conf>0.85 直接停
    python3 rotate_detect.py --thresh 0.80      # 自定义阈值
    python3 rotate_detect.py --speed 0.5        # 慢速旋转
    python3 rotate_detect.py --peak             # 峰值检测模式 (冲高回落后停止)
"""

import argparse, time, json, sys

try:
    from urllib.request import urlopen, URLError
except ImportError:
    from urllib2 import urlopen, URLError

import rospy
from geometry_msgs.msg import Twist

STATUS_URL = "http://127.0.0.1:8080/status"


def get_conf():
    try:
        r = urlopen(STATUS_URL, timeout=1)
        data = json.loads(r.read().decode())
        return float(data.get("conf", 0.0))
    except Exception:
        return 0.0


def main():
    parser = argparse.ArgumentParser(description="旋转扫描 + YOLO 检测停止")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--thresh", type=float, default=0.85,
                        help="置信度阈值 (默认 0.85)")
    parser.add_argument("--peak", action="store_true",
                        help="峰值检测模式 (冲高回落后停)")
    parser.add_argument("--hz", type=int, default=20)
    args = parser.parse_args()

    # 检查 infer.py 是否在运行
    c = get_conf()
    print(f"[CHECK] /status conf={c:.3f} | "
          f"{'✅ infer.py 在线' if c >= 0 else '⚠️  infer.py 无响应/未运行'}")

    rospy.init_node("rotate_detect", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(args.hz)

    twist = Twist()
    twist.angular.z = args.speed

    # 峰值检测状态
    peak_conf = 0.0
    fall_count = 0
    FALL_DELTA = 0.05   # 低于峰值多少算回落
    FALL_FRAMES = 5     # 连续回落几帧确认
    MIN_TRIGGER = 0.4

    mode = "峰值检测" if args.peak else f"阈值 conf>{args.thresh}"
    print(f"[ROTATE] angular={args.speed} | {mode}")
    print("[ROTATE] 开始旋转 (Ctrl+C 紧急停止)")

    t_last_print = 0

    while not rospy.is_shutdown():
        pub.publish(twist)
        conf = get_conf()

        # 每秒打印一次置信度
        now = time.time()
        if now - t_last_print > 1.0:
            print(f"[ROTATE] conf={conf:.3f}  peak={peak_conf:.3f}")
            t_last_print = now

        # ---- 阈值模式 ----
        if not args.peak and conf >= args.thresh:
            print(f"[ROTATE] ✅ conf={conf:.3f} >= {args.thresh}, 停止!")
            break

        # ---- 峰值检测模式 ----
        if args.peak and conf > MIN_TRIGGER:
            if conf > peak_conf:
                peak_conf = conf
                fall_count = 0
            elif conf < peak_conf - FALL_DELTA:
                fall_count += 1
            else:
                fall_count = max(0, fall_count - 1)

            if peak_conf > 0.5 and fall_count >= FALL_FRAMES:
                print(f"[ROTATE] ✅ 峰值 {peak_conf:.3f} 已回落, 停止!")
                break

        rate.sleep()

    # 确保停止
    stop = Twist()
    for _ in range(15):
        pub.publish(stop)
        time.sleep(0.05)

    print("[ROTATE] 已停止, 退出")
    print("[TIP]  查看检测画面: http://192.168.1.109:8080")


if __name__ == "__main__":
    main()

