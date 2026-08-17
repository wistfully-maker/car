#!/usr/bin/env python3
"""
自动驾驶 V2 — YOLO常驻版(需launch启动yolo_server)
流程: 等红灯→等方向→巡线→停车→语音
"""

import subprocess, sys, time, os, signal

LEFT   = "/home/ucar/ucar_ws/src/car_server/ros_line_follow_v21_left.py"
RIGHT  = "/home/ucar/ucar_ws/src/car_server/ros_line_follow_right_v2.py"
MIDDLE = "/home/ucar/ucar_ws/src/car_server/ros_line_follow_v22_mid.py"
TTS    = "/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py"
R_FILE = "/tmp/yolo_result.txt"


def wait_direction():
    """等待红灯→方向变化"""
    print("=" * 50)
    print("🔍 等待YOLO检测...")
    print("=" * 50)

    # 清旧结果
    if os.path.exists(R_FILE): os.remove(R_FILE)

    # 等红灯
    print("  ⏳ 等待红灯...")
    t0 = time.time()
    red_seen = False
    while time.time()-t0 < 60:
        if os.path.exists(R_FILE):
            with open(R_FILE) as f: r = f.read().strip()
            if r == "red_light":
                if not red_seen: print("  🚦 红灯!"); red_seen=True
                rospy.sleep(1)
                # 等变灯: 清文件, 等新结果
                os.remove(R_FILE)
                rospy.sleep(0.5)
                t1 = time.time()
                while time.time()-t1 < 30:
                    if os.path.exists(R_FILE):
                        with open(R_FILE) as f: r2 = f.read().strip()
                        if r2 in ["left_turn","right_turn","straight"]:
                            print(f"  ✅ 方向: {r2}")
                            return r2
                    time.sleep(0.2)
                print("  ⚠️ 变灯超时30s, 默认直行"); return "straight"
        time.sleep(0.3)
    print("  ⚠️ 未检测到红灯, 默认直行")
    return "straight"


def run_line(command):
    m = {"left_turn":("左道",LEFT),"right_turn":("右道",RIGHT),"straight":("中道",MIDDLE)}
    name, script = m.get(command, ("中道",MIDDLE))
    print(f"\n{'='*50}\n🚀 {name}巡线\n{'='*50}")

    sf = "/tmp/stop_done.txt"
    if os.path.exists(sf): os.remove(sf)

    proc = subprocess.Popen(["python3", script])
    t0 = time.time()
    while time.time()-t0 < 120:
        if os.path.exists(sf): print("  🅿 停车标记!"); break
        if proc.poll() is not None: print("  ⚠️ 巡线退出"); break
        time.sleep(0.5)
    try: os.kill(proc.pid, signal.SIGTERM)
    except: pass
    return 0


def run_tts():
    print(f"\n{'='*50}\n🔊 语音播报\n{'='*50}")
    try:
        r = subprocess.run(["python3", TTS, "任务完成"], timeout=15, capture_output=True, text=True)
        print(f"  TTS: {r.stdout.strip()}")
    except Exception as e:
        print(f"  ⚠️ 播报失败: {e}")


if __name__ == "__main__":
    import rospy
    rospy.init_node("auto_drive_v2", anonymous=True)
    cmd = wait_direction()
    ret = run_line(cmd)
    if ret == 0: run_tts()
