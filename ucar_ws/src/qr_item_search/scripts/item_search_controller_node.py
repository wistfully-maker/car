#!/usr/bin/python3
from qr_item_search.controller_logic import SearchController


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Empty, Int32, String
    from tf.transformations import euler_from_quaternion

    rospy.init_node("item_search_controller")
    speed_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    enabled_publisher = rospy.Publisher(
        "/qr_item_search/scan_enabled",
        Bool,
        queue_size=1,
        latch=True,
    )
    wall_publisher = rospy.Publisher(
        "/qr_item_search/wall_index",
        Int32,
        queue_size=1,
        latch=True,
    )
    state_publisher = rospy.Publisher(
        "/qr_item_search/state",
        String,
        queue_size=10,
        latch=True,
    )
    reset_publisher = rospy.Publisher(
        "/qr_item_search/reset",
        Int32,
        queue_size=1,
        latch=True,
    )

    class RosOutputs:
        def publish_speed(self, value):
            command = Twist()
            command.angular.z = value
            speed_publisher.publish(command)

        def publish_scan_enabled(self, value):
            enabled_publisher.publish(Bool(data=value))

        def publish_wall(self, value):
            wall_publisher.publish(Int32(data=value))

        def publish_state(self, value):
            state_publisher.publish(String(data=value))

        def publish_reset(self, search_id):
            reset_publisher.publish(Int32(data=search_id))

    controller = SearchController(
        wall_yaw_offsets=rospy.get_param(
            "~wall_yaw_offsets",
            [0.0, 1.5708, 3.1416],
        ),
        outputs=RosOutputs(),
        kp=rospy.get_param("~yaw_kp", 1.2),
        max_speed=rospy.get_param("~max_angular_speed", 0.30),
        min_speed=rospy.get_param("~min_angular_speed", 0.11),
        tolerance=rospy.get_param("~yaw_tolerance", 0.035),
        settle_seconds=rospy.get_param("~settle_seconds", 0.8),
        scan_timeout=rospy.get_param("~scan_timeout", 4.0),
        turn_timeout=rospy.get_param("~turn_timeout", 8.0),
    )

    def odom_callback(message):
        orientation = message.pose.pose.orientation
        yaw = euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )[2]
        controller.update_yaw(yaw)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber(
        "/qr_item_search/start",
        Empty,
        lambda message: controller.start(),
    )
    rospy.Subscriber(
        "/qr_item_search/observation",
        String,
        lambda message: controller.handle_observation(message.data),
    )
    rospy.Subscriber(
        "/qr_item_search/match_decision",
        Bool,
        lambda message: controller.match_decision(message.data),
    )
    rospy.Timer(
        rospy.Duration(0.05),
        lambda event: controller.tick(),
    )
    rospy.on_shutdown(controller.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
