#!/usr/bin/python3
import json
import threading

from qr_item_search.qr_decode import StableQrDecoder
from qr_item_search.qr_payload import ItemResolver
from qr_item_search.scanner_logic import ScannerLogic


def main():
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Empty, Int32, String

    rospy.init_node("qr_scanner")
    bridge = CvBridge()
    observation_publisher = rospy.Publisher(
        "/qr_item_search/observation", String, queue_size=10
    )

    def publish(payload):
        observation_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    logic = ScannerLogic(
        decoder=StableQrDecoder(
            required_frames=rospy.get_param("~required_frames", 2)
        ),
        resolver=ItemResolver(
            connect_timeout=rospy.get_param("~connect_timeout", 1.0),
            read_timeout=rospy.get_param("~read_timeout", 2.0),
            retries=rospy.get_param("~http_retries", 2),
        ),
        publisher=publish,
        warning=rospy.logwarn,
    )

    def image_callback(message):
        if not logic.accepting_images:
            return
        try:
            image = bridge.imgmsg_to_cv2(message, "bgr8")
            logic.handle_image(image)
        except (CvBridgeError, UnicodeDecodeError) as error:
            logic.publish_decode_error(str(error))

    rospy.Subscriber(
        "/qr_item_search/scan_enabled",
        Bool,
        lambda message: logic.set_enabled(message.data),
    )
    rospy.Subscriber(
        "/qr_item_search/wall_index",
        Int32,
        lambda message: logic.set_wall_index(message.data),
    )
    rospy.Subscriber(
        "/qr_item_search/reset",
        Empty,
        lambda message: logic.reset_search(),
    )
    rospy.Subscriber(
        rospy.get_param("~image_topic", "/usb_cam/image_raw"),
        Image,
        image_callback,
        queue_size=1,
        buff_size=2 ** 24,
    )

    worker = threading.Thread(
        target=logic.run_worker,
        args=(rospy.is_shutdown,),
    )
    worker.daemon = True
    worker.start()
    rospy.spin()


if __name__ == "__main__":
    main()
