#!/usr/bin/python3
import json
import threading

from qr_item_search.controller_logic import SearchController


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf.transformations import euler_from_quaternion

    rospy.init_node("item_search_controller")
    speed = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    control = rospy.Publisher("/qr_item_search/scanner_control", String, queue_size=1, latch=True)
    state = rospy.Publisher("/qr_item_search/state", String, queue_size=10, latch=True)
    result = rospy.Publisher("/qr_item_search/result", String, queue_size=10, latch=True)

    class RosOutputs:
        def publish_speed(self, value):
            message = Twist()
            message.angular.z = value
            speed.publish(message)

        def publish_state(self, value):
            state.publish(String(data=value))

        def publish_scanner_control(self, value):
            control.publish(String(data=json.dumps(value, ensure_ascii=False)))

        def publish_result(self, value):
            result.publish(String(data=json.dumps(value, ensure_ascii=False)))

    controller = SearchController(
        outputs=RosOutputs(),
        fast_angular_speed=rospy.get_param("~fast_angular_speed", .40),
        targeted_angular_speed=rospy.get_param("~targeted_angular_speed", .20),
        minimum_effective_speed=rospy.get_param("~minimum_effective_speed", .11),
        fast_sweep_angle=rospy.get_param("~fast_sweep_angle", 6.632251),
        yaw_tolerance=rospy.get_param("~yaw_tolerance", .035),
        heading_timeout=rospy.get_param("~heading_timeout", 1.0),
        camera_timeout=rospy.get_param("~camera_timeout", 1.0),
        search_total_timeout=rospy.get_param("~search_total_timeout", 40.0),
        error_handler=rospy.logerr,
    )
    callback_fault = threading.Event()

    def safe_callback(label, callback):
        if callback_fault.is_set():
            return False
        try:
            return callback()
        except Exception as error:
            callback_fault.set()
            try:
                controller.shutdown()
            except Exception as shutdown_error:
                rospy.logerr("controller emergency shutdown failed: %s", shutdown_error)
            rospy.logerr("%s callback failed: %s", label, error)
            return False

    def odom_callback(message):
        def invoke():
            orientation = message.pose.pose.orientation
            yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
            return controller.update_yaw(yaw, rospy.get_time())
        return safe_callback("odometry", invoke)

    def start_callback(message):
        return safe_callback("start", lambda: controller.start(message.data, rospy.get_time()))

    def stop_callback(message):
        return safe_callback("stop", lambda: controller.stop(message.data, rospy.get_time()))

    def scanner_event_callback(message):
        return safe_callback("scanner event", lambda: controller.handle_scanner_event(
            message.data, rospy.get_time()))

    def timer_callback(event):
        return safe_callback("timer", lambda: controller.tick(rospy.get_time()))

    def shutdown():
        callback_fault.set()
        try:
            controller.shutdown()
        except Exception as error:
            rospy.logerr("controller shutdown failed: %s", error)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/qr_item_search/start", String, start_callback)
    rospy.Subscriber("/qr_item_search/stop", String, stop_callback)
    rospy.Subscriber("/qr_item_search/scanner_event", String, scanner_event_callback)
    rospy.Timer(rospy.Duration(0.05), timer_callback)
    rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
