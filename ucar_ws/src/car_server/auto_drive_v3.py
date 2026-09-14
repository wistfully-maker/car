#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动驾驶V3 — YOLO红绿灯 → 选道巡线 → 停车 → 语音播报

用法: rosrun car_server auto_drive_v3.py
依赖: yolo_server.py(常驻), follow_left_v4/follow_right_v4/follow_mid_v4, tts_http.py
"""

import rospy, os, time, subprocess, signal

LEFT   = "/home/ucar/ucar_ws/src/car_server/follow_left_v4.py"
RIGHT  = "/home/ucar/ucar_ws/src/car_server/follow_right_v4.py"
MID    = "/home/ucar/ucar_ws/src/car_server/follow_mid_v4.py"
TTS    = "/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py"
YOLO_FILE = "/tmp/yolo_result.txt"
STOP_FILE = "/tmp/stop_done.txt"


def wait_direction(timeout=30):
    """直接等方向(跳过红灯)"""
    rospy.loginfo("=" * 50)
    rospy.loginfo("🔍 等待方向指令...")
    rospy.loginfo("=" * 50)

    if os.path.exists(STOP_FILE): os.remove(STOP_FILE)

    t0 = time.time()
    while time.time()-t0 < timeout and not rospy.is_shutdown():
        if os.path.exists(YOLO_FILE):
            with open(YOLO_FILE) as f: val = f.read().strip()
            if val in ["left_turn", "right_turn", "straight"]:
                rospy.loginfo(f"  ✅ 方向: {val}")
                return val
        time.sleep(0.3)

    rospy.logwarn(f"  ⚠️ {timeout}秒未检测, 默认直行")
    return "straight"


def run_line_follow(direction):
    """启动对应巡线脚本, 等停车标记"""
    mapping = {
        "left_turn":  ("左道", LEFT),
        "right_turn": ("右道", RIGHT),
        "straight":   ("中道", MID),
    }
    name, script = mapping.get(direction, ("中道", MID))

    rospy.loginfo(f"\n{'='*50}\n🚀 {name}巡线\n{'='*50}")

    if os.path.exists(STOP_FILE): os.remove(STOP_FILE)

    proc = subprocess.Popen(["python3", script])
    t0 = time.time()
    while time.time()-t0 < 120 and not rospy.is_shutdown():
        if os.path.exists(STOP_FILE):
            rospy.loginfo("  🅿 检测到停车标记!")
            break
        if proc.poll() is not None:
            rospy.logwarn("  ⚠️ 巡线进程已退出")
            break
        time.sleep(0.5)

    try:
        os.kill(proc.pid, signal.SIGTERM)
    except:
        pass
    return 0


def run_tts(text="任务完成"):
    """语音播报"""
    rospy.loginfo(f"\n{'='*50}\n🔊 语音播报\n{'='*50}")
    if not os.path.exists(TTS):
        rospy.logwarn("TTS脚本不存在, 跳过")
        return
    try:
        r = subprocess.run(["python3", TTS, text], timeout=15,
                          capture_output=True, text=True)
        rospy.loginfo(f"  TTS: {r.stdout.strip()}")
    except Exception as e:
        rospy.logwarn(f"  ⚠️ 播报失败: {e}")


if __name__ == "__main__":
    rospy.init_node("auto_drive_v3", anonymous=True)
    cmd = wait_direction()
    ret = run_line_follow(cmd)
    if ret == 0:
        run_tts()
