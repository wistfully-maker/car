#!/usr/bin/python3
import json
import threading

from qr_item_search.controller_logic import SearchController
from qr_item_search.run_metrics import append_run_record


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf.transformations import euler_from_quaternion

    rospy.init_node("item_search_controller")
    speed = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    control = rospy.Publisher("/qr_item_search/scanner_control", String, queue_size=10, latch=True)
    state = rospy.Publisher("/qr_item_search/state", String, queue_size=10, latch=True)
    result = rospy.Publisher("/qr_item_search/result", String, queue_size=10, latch=True)
    snapshot = rospy.Publisher("/qr_item_search/debug_snapshot", String, queue_size=5)

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

    metrics_dir = rospy.get_param("~metrics_dir", "")
    metrics = None
    if metrics_dir:
        def metrics_callback(record):
            append_run_record(metrics_dir, record, warning=rospy.logwarn)
        metrics = metrics_callback

    controller = SearchController(
        outputs=RosOutputs(),
        step_angle_deg=rospy.get_param("~step_angle_deg", 45.0),
        cruise_angular_speed=rospy.get_param("~cruise_angular_speed", 0.50),
        approach_angular_speed=rospy.get_param("~approach_angular_speed", 0.20),
        approach_zone_deg=rospy.get_param("~approach_zone_deg", 10.0),
        yaw_tolerance_deg=rospy.get_param("~yaw_tolerance_deg", 2.0),
        settled_angular_speed=rospy.get_param("~settled_angular_speed", 0.03),
        settled_duration=rospy.get_param("~settled_duration", 0.20),
        scan_window=rospy.get_param("~scan_window", 0.60),
        offset_angle_deg=rospy.get_param("~offset_angle_deg", 22.5),
        max_passes=rospy.get_param("~max_passes", 2),
        search_total_timeout=rospy.get_param("~search_total_timeout", 60.0),
        settling_timeout=rospy.get_param("~settling_timeout", 3.0),
        heading_timeout=rospy.get_param("~heading_timeout", 1.0),
        camera_timeout=rospy.get_param("~camera_timeout", 1.0),
        error_handler=rospy.logerr,
        metrics=metrics,
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
            angular_speed = message.twist.twist.angular.z
            return controller.update_yaw(yaw, angular_speed=angular_speed, now=rospy.get_time())
        return safe_callback("odometry", invoke)

    def start_callback(message):
        return safe_callback("start", lambda: controller.start(message.data, rospy.get_time()))

    def stop_callback(message):
        return safe_callback("stop", lambda: controller.stop(message.data, rospy.get_time()))

    def scanner_event_callback(message):
        return safe_callback("scanner event", lambda: controller.handle_scanner_event(
            message.data, rospy.get_time()))

    def timer_callback(event):
        def invoke():
            controller.tick(rospy.get_time())
            if snapshot.get_num_connections() > 0:
                snapshot.publish(String(data=json.dumps(controller.debug_snapshot(), ensure_ascii=False)))
            return True
        return safe_callback("timer", invoke)

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
