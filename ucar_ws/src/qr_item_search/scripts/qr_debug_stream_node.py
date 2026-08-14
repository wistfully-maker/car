#!/usr/bin/python3
import json
import threading
import time

from qr_item_search.debug_stream import MjpegStreamServer, draw_overlay


def main():
    import rospy
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rospy.init_node("qr_debug_stream")
    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", 8080))
    max_fps = float(rospy.get_param("~max_fps", 10.0))
    jpeg_quality = int(rospy.get_param("~jpeg_quality", 80))
    bridge = CvBridge()
    server = MjpegStreamServer(host, port)
    state_lock = threading.Lock()
    current = {}
    last_encode = [0.0]
    min_interval = 1.0 / max(max_fps, 0.1)

    def snapshot_callback(message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("debug snapshot must be an object")
            with state_lock:
                current.clear()
                current.update(value)
        except Exception as error:
            rospy.logwarn("debug snapshot parse failed: %s", error)

    def image_callback(message):
        now = time.time()
        if now - last_encode[0] < min_interval:
            return
        last_encode[0] = now
        try:
            image = bridge.imgmsg_to_cv2(message, "bgr8")
            with state_lock:
                state = dict(current)
            overlay = draw_overlay(image, state)
            ok, buffer = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if ok:
                server.set_latest(buffer.tobytes())
        except Exception as error:
            rospy.logwarn("debug stream encode failed: %s", error)

    rospy.Subscriber(rospy.get_param("~image_topic", "/usb_cam/image_raw"), Image,
                     image_callback, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/qr_item_search/debug_snapshot", String, snapshot_callback)
    server.start()
    rospy.loginfo("qr debug stream listening on http://%s:%d/", host, server.address[1])

    def shutdown():
        try:
            server.stop()
        except Exception as error:
            rospy.logwarn("debug stream shutdown failed: %s", error)

    rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
