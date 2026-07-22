#!/usr/bin/python3
import json
import math
import threading

from qr_item_search.image_quality import decode_variants, measure_quality
from qr_item_search.qr_decode import UniqueQrDecoder
from qr_item_search.qr_payload import ItemResolver
from qr_item_search.scanner_logic import ScannerLogic


def main():
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rospy.init_node("qr_scanner")
    bridge = CvBridge()
    decoder_event = threading.Event()
    stop_event = threading.Event()
    current_identity = [None, None]
    publisher = rospy.Publisher("/qr_item_search/scanner_event", String, queue_size=10)

    def publish(payload):
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    worker_count = rospy.get_param("~http_worker_count", 3)
    logic = ScannerLogic(
        decoder=UniqueQrDecoder(),
        resolver=ItemResolver(
            connect_timeout=rospy.get_param("~connect_timeout", 1.0),
            read_timeout=rospy.get_param("~read_timeout", 2.0),
            retries=rospy.get_param("~http_retries", 1),
        ),
        event_publisher=publish,
        quality_function=measure_quality,
        variant_function=decode_variants,
        worker_count=worker_count,
        expected_count=3,
        warning=rospy.logwarn,
        error_handler=rospy.logerr,
    )

    def control_callback(message):
        try:
            value = json.loads(message.data)
            if (not isinstance(value, dict) or type(value.get("protocol_version")) is not int
                    or value.get("protocol_version") != 1):
                raise ValueError("invalid scanner control protocol")
            task_id, search_id = value.get("task_id"), value.get("search_id")
            enabled, enhanced, retry = value.get("enabled"), value.get("enhanced"), value.get("retry_failed")
            yaw = value.get("detected_yaw")
            if not isinstance(task_id, str) or not isinstance(search_id, str):
                raise ValueError("scanner control identity must be strings")
            if type(enabled) is not bool or type(enhanced) is not bool or type(retry) is not bool:
                raise ValueError("scanner control flags must be bool")
            if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
                raise ValueError("detected_yaw must be finite")
            if (not task_id) != (not search_id):
                raise ValueError("scanner control identity must be both empty or both non-empty")
            if task_id == "" and search_id == "":
                logic.set_control(False, False, 0.0, False)
                current_identity[:] = [None, None]
                return
            if [task_id, search_id] != current_identity:
                logic.reset_search(task_id, search_id)
                current_identity[:] = [task_id, search_id]
            logic.set_control(enabled, enhanced, yaw, retry_failed=retry)
        except Exception as error:
            rospy.logwarn("invalid scanner control: %s", error)

    def image_callback(message):
        try:
            image = bridge.imgmsg_to_cv2(message, "bgr8")
            if logic.submit_frame(image):
                decoder_event.set()
        except Exception as error:
            rospy.logwarn("scanner image callback failed: %s", error)

    def decoder_loop():
        while not rospy.is_shutdown() and not stop_event.is_set():
            if decoder_event.wait(0.2):
                decoder_event.clear()
                if stop_event.is_set() or rospy.is_shutdown():
                    break
                try:
                    logic.process_latest_frame()
                except Exception as error:
                    rospy.logerr("scanner decoder failed: %s", error)

    rospy.Subscriber("/qr_item_search/scanner_control", String, control_callback)
    rospy.Subscriber(rospy.get_param("~image_topic", "/usb_cam/image_raw"), Image,
                     image_callback, queue_size=1, buff_size=2 ** 24)
    worker_threads = logic.run_workers(rospy.is_shutdown)
    thread = threading.Thread(target=decoder_loop)
    thread.daemon = True
    thread.start()

    def shutdown():
        stop_event.set()
        try:
            logic.set_control(False, False, 0.0, False)
        except Exception as error:
            rospy.logerr("scanner shutdown failed: %s", error)
        decoder_event.set()

    rospy.on_shutdown(shutdown)
    rospy.spin()
    _ = worker_threads


if __name__ == "__main__":
    main()
