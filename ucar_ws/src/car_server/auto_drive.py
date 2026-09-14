#!/usr/bin/env python3
"""
自动驾驶全流程 — 单文件集成版
YOLO检测→选道巡线→停车→语音播报，一键完成
"""

import subprocess, sys, time, os, signal, cv2, numpy as np

# ========== 配置 ==========
LEFT_FULL   = "/home/ucar/ucar_ws/src/car_server/ros_line_follow_left_full.py"
RIGHT_FULL  = "/home/ucar/ucar_ws/src/car_server/ros_line_follow_right_full.py"
MIDDLE      = "/home/ucar/ucar_ws/src/car_server/ros_line_follow_v18_mid.py"
TTS_SCRIPT  = "/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py"

MODEL_PATH  = "/home/ucar/ucar_ws/src/yolo_turn/best.pt"
CONF        = 0.5
DETECT_TIME = 3.0   # YOLO检测秒数
NAMES       = {0: "red_light", 1: "straight", 2: "left_turn", 3: "right_turn"}


def run_yolo():
    """集成YOLO检测(同进程, 无子进程开销)"""
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from ultralytics import YOLO

    print("=" * 50)
    print("🔍 YOLO检测中...")
    print("=" * 50)

    bridge = CvBridge()
    latest = [None]

    def cb(msg):
        try: latest[0] = bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    rospy.init_node("auto_drive_yolo", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
    rospy.sleep(1.5)  # 等摄像头

    print("  加载模型...")
    model = YOLO(MODEL_PATH)
    rate = rospy.Rate(10)

    # 阶段A: 等待红灯(确认在停止线)
    print("  ⏳ 等待红灯...")
    while not rospy.is_shutdown():
        frame = latest[0]
        if frame is not None:
            results = model(frame, conf=CONF, verbose=False)
            for r in results:
                if r.boxes is not None and len(r.boxes) > 0:
                    b = max(r.boxes, key=lambda b: float(b.conf[0]))
                    if int(b.cls[0]) == 0:  # red_light
                        print(f"  🚦 检测到红灯, 等待变灯...")
                        rospy.sleep(1)
                        # 阶段B: 等待方向
                        while not rospy.is_shutdown():
                            frame2 = latest[0]
                            if frame2 is not None:
                                r2 = model(frame2, conf=CONF, verbose=False)
                                best_name, best_conf = None, 0
                                for rr in r2:
                                    if rr.boxes is not None and len(rr.boxes) > 0:
                                        bb = max(rr.boxes, key=lambda b: float(b.conf[0]))
                                        cls_id = int(bb.cls[0]); cf = float(bb.conf[0])
                                        name = NAMES.get(cls_id, "unknown")
                                        if name != "red_light" and cf > best_conf:
                                            best_conf = cf; best_name = name
                                if best_name:
                                    print(f"  ✅ 检测到: {best_name} ({best_conf:.2f})")
                                    return best_name
                            rate.sleep()
        rate.sleep()

    return "straight"  # fallback  # 返回model以便后续可能复用


def run_line_follow(command):
    mapping = {
        "left_turn":  ("左道", LEFT_FULL),
        "right_turn": ("右道", RIGHT_FULL),
        "straight":   ("中道", MIDDLE),
    }
    name, script = mapping.get(command, ("中道", MIDDLE))
    print(f"\n{'='*50}\n🚀 阶段2: {name}巡线\n{'='*50}")

    sf = "/tmp/stop_done.txt"
    if os.path.exists(sf): os.remove(sf)

    proc = subprocess.Popen(["python3", script])
    t0 = time.time()
    while time.time() - t0 < 120:
        if os.path.exists(sf):
            print("  🅿 检测到停车标记!"); break
        if proc.poll() is not None:
            print("  ⚠️ 巡线进程已退出"); break
        time.sleep(0.5)

    try: os.kill(proc.pid, signal.SIGTERM)
    except: pass
    return 0


def run_tts():
    print(f"\n{'='*50}\n🔊 语音播报\n{'='*50}")
    try:
        r = subprocess.run(["python3", TTS_SCRIPT, "任务完成"],
                           timeout=15, capture_output=True, text=True)
        print(f"  TTS: {r.stdout.strip()}")
    except Exception as e:
        print(f"  ⚠️ 播报失败: {e}")


if __name__ == "__main__":
    cmd = run_yolo()
    ret = run_line_follow(cmd)
    if ret == 0:
        run_tts()
