#!/usr/bin/env python3
"""
小车视频录制脚本
================
两种模式:
  1. ROS 模式 (默认): 从 ROS 摄像头话题录制
  2. 直连模式:       直接从 /dev/video0 录制 (不需要 ROS)

用法:
  python3 record_video.py              # ROS模式，录制到手动 Ctrl+C 停止
  python3 record_video.py --direct     # 直连摄像头模式
  python3 record_video.py -t 30        # 录制30秒自动停止
  python3 record_video.py -t 30 -o my_track.mp4  # 指定输出文件名

适合 SSH 无显示器环境使用
"""

import os
import sys
import time
import signal
import argparse
from datetime import datetime

# ==================== 配置 ====================
CAMERA_DEVICE = "/dev/video0"               # 直连模式用的设备
CAMERA_TOPIC  = "/usb_cam/image_raw"         # ROS话题名
OUTPUT_DIR    = os.path.dirname(os.path.abspath(__file__)) + "/videos"
FPS           = 15                            # 录制帧率
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
# ==============================================

# 全局变量，用于信号处理
writer = None
frame_count = 0
start_time = None
output_path = None
ros_mode = True
camera_idx = 0


def format_time(seconds):
    """秒数转为可读时间"""
    m, s = divmod(int(seconds), 60)
    h, m = divmod(m, 60)
    if h > 0:
        return f"{h}h{m:02d}m{s:02d}s"
    elif m > 0:
        return f"{m}m{s:02d}s"
    return f"{s}s"


def print_status(current_frame, elapsed, file_size=0):
    """打印一行录制状态（覆盖式输出）"""
    fps_actual = current_frame / elapsed if elapsed > 0 else 0
    size_mb = file_size / (1024 * 1024) if file_size > 0 else 0

    bar_width = 30
    filled = min(int((elapsed % 60) / 60 * bar_width), bar_width) if elapsed > 0 else 0
    bar = "█" * filled + "░" * (bar_width - filled)

    status = (
        f"\r🎥 录制中... [{bar}] "
        f"帧: {current_frame:6d} | "
        f"时长: {format_time(elapsed)} | "
        f"实际FPS: {fps_actual:4.1f} | "
        f"文件: {size_mb:5.1f}MB"
    )
    sys.stdout.write(status)
    sys.stdout.flush()


def cleanup():
    """释放资源"""
    global writer, output_path
    if writer:
        writer.release()
        writer = None

    elapsed = time.time() - start_time if start_time else 0
    file_size = os.path.getsize(output_path) if output_path and os.path.exists(output_path) else 0

    print()  # 换行
    print("=" * 55)
    print(f"✅ 录制结束")
    print(f"   文件: {output_path}")
    print(f"   帧数: {frame_count}")
    print(f"   时长: {format_time(elapsed)}")
    print(f"   大小: {file_size / (1024*1024):.1f} MB")
    print(f"   平均FPS: {frame_count / elapsed:.1f}" if elapsed > 0 else "   平均FPS: N/A")
    print("=" * 55)


def handle_exit(signum=None, frame=None):
    """处理 Ctrl+C / 终止信号"""
    print("\n⏹  收到停止信号，正在保存...")
    cleanup()
    sys.exit(0)


# ==================== ROS 模式 ====================

def record_ros(duration=0):
    """从 ROS 话题录制视频"""
    global writer, frame_count, start_time, output_path

    import rospy
    import numpy as np
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    bridge = CvBridge()

    def callback(msg):
        global writer, frame_count, start_time, output_path
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        # 第一帧时创建 VideoWriter
        if writer is None:
            h, w = frame.shape[:2]
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            output_path = os.path.join(OUTPUT_DIR,
                datetime.now().strftime("track_%Y%m%d_%H%M%S.mp4"))
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(output_path, fourcc, FPS, (w, h))
            start_time = time.time()
            print(f"\n📹 开始录制 (ROS模式)")
            print(f"   话题: {CAMERA_TOPIC}")
            print(f"   分辨率: {w}×{h}")
            print(f"   目标FPS: {FPS}")
            print(f"   输出: {output_path}")

        writer.write(frame)
        frame_count += 1

        # 状态显示
        elapsed = time.time() - start_time
        file_size = os.path.getsize(output_path) if os.path.exists(output_path) else 0
        if frame_count % 15 == 0:
            print_status(frame_count, elapsed, file_size)

        # 定时停止
        if duration > 0 and elapsed >= duration:
            rospy.signal_shutdown("录制时长已到")

    rospy.init_node("video_recorder", anonymous=True, disable_signals=True)
    rospy.Subscriber(CAMERA_TOPIC, Image, callback, queue_size=2)

    print(f"⏳ 等待摄像头话题 {CAMERA_TOPIC} ...")
    if duration > 0:
        print(f"   将在 {format_time(duration)} 后自动停止")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    cleanup()


# ==================== 直连模式 ====================

def record_direct(duration=0):
    """直接从摄像头设备录制（不需要ROS）"""
    global writer, frame_count, start_time, output_path
    import cv2

    print(f"📷 打开摄像头 {CAMERA_DEVICE} ...")
    cap = cv2.VideoCapture(CAMERA_DEVICE)

    if not cap.isOpened():
        print(f"❌ 无法打开摄像头 {CAMERA_DEVICE}")
        print("   尝试其他设备索引...")
        for idx in [0, 1, 2]:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                print(f"   ✅ 使用 /dev/video{idx}")
                break
        else:
            print("❌ 所有尝试都失败了，请检查摄像头连接")
            return

    # 设置参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output_path = os.path.join(OUTPUT_DIR,
        datetime.now().strftime("track_%Y%m%d_%H%M%S.mp4"))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, FPS, (actual_w, actual_h))
    start_time = time.time()

    print(f"\n📹 开始录制 (直连模式)")
    print(f"   设备: {CAMERA_DEVICE}")
    print(f"   分辨率: {actual_w}×{actual_h}")
    print(f"   目标FPS: {FPS}")
    print(f"   输出: {output_path}")

    frame_interval = 1.0 / FPS
    last_write_time = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("\n⚠️  读取帧失败，跳过...")
            time.sleep(0.01)
            continue

        now = time.time()
        # 按目标帧率写入（避免重复帧）
        if now - last_write_time >= frame_interval:
            writer.write(frame)
            frame_count += 1
            last_write_time = now

        elapsed = now - start_time
        file_size = os.path.getsize(output_path) if os.path.exists(output_path) else 0
        if frame_count % 15 == 0 and frame_count > 0:
            print_status(frame_count, elapsed, file_size)

        # 定时停止
        if duration > 0 and elapsed >= duration:
            print(f"\n⏰ 达到设定时长 {format_time(duration)}，自动停止")
            break

    cap.release()
    cleanup()


# ==================== 主入口 ====================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="小车摄像头录制脚本 (ROS / 直连双模式)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 record_video.py                # ROS模式，Ctrl+C停止
  python3 record_video.py --direct       # 直连摄像头，Ctrl+C停止
  python3 record_video.py -t 60          # 录制60秒自动停止
  python3 record_video.py -t 30 -o test.mp4  # 指定文件名录制30秒
        """
    )
    parser.add_argument("-d", "--direct", action="store_true",
                        help="直连摄像头模式 (不依赖ROS)")
    parser.add_argument("-t", "--time", type=float, default=0,
                        help="录制时长(秒)，0表示手动Ctrl+C停止")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="指定输出文件名 (默认自动生成时间戳文件名)")
    parser.add_argument("--device", type=str, default=CAMERA_DEVICE,
                        help=f"摄像头设备路径 (默认: {CAMERA_DEVICE})")
    parser.add_argument("--topic", type=str, default=CAMERA_TOPIC,
                        help=f"ROS摄像头话题 (默认: {CAMERA_TOPIC})")
    args = parser.parse_args()

    # 注册信号处理
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    # 更新配置
    CAMERA_DEVICE = args.device
    CAMERA_TOPIC = args.topic

    print("=" * 55)
    print("  🏎️  小车视频录制工具")
    print("=" * 55)

    try:
        if args.direct:
            # 直连模式
            import cv2
            record_direct(args.time)
        else:
            # ROS 模式
            import cv2
            try:
                record_ros(args.time)
            except rospy.ROSException as e:
                print(f"\n❌ ROS 连接失败: {e}")
                print("   ROS Master 是否在运行?")
                print("   试试: python3 record_video.py --direct  (直连摄像头)")
                sys.exit(1)
    except ImportError as e:
        print(f"\n❌ 缺少依赖: {e}")
        print("   请安装: pip install opencv-python")
        if not args.direct:
            print("   或使用直连模式: python3 record_video.py --direct")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        cleanup()
        sys.exit(1)
