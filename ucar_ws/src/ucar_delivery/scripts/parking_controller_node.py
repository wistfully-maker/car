#!/usr/bin/env python3
"""Parking controller node: runs the closed-loop white-frame controller,
feeds it the lidar front distance and odometry velocity, publishes motion
commands only on /cmd_vel/delivery_parking, and reports stage progress and
results.
"""

import json

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ucar_delivery import protocol
from ucar_delivery.frame_detector import FrameObservation
from ucar_delivery.lidar_safety import LidarSafety
from ucar_delivery.parking_controller import ParkingController

STAGE_BY_STATE = {
    ParkingController.CENTER_FRAME: "aligned",
    ParkingController.APPROACH_FRAME: "centered",
    ParkingController.FINAL_STOP: "approach_started",
    ParkingController.VERIFY_STOP: "final_stop_done",
    ParkingController.COMPLETE: "verified",
}


class ParkingControllerNode:
    def __init__(self):
        config = {
            "k_yaw": rospy.get_param("~parking_controller/k_yaw", 0.8),
            "k_lateral": rospy.get_param("~parking_controller/k_lateral", 0.5),
            "max_linear": rospy.get_param("~parking_controller/max_linear", 0.15),
            "max_angular": rospy.get_param("~parking_controller/max_angular", 0.4),
            "max_linear_y": rospy.get_param("~parking_controller/max_linear_y", 0.0),
            "angular_deadband": rospy.get_param(
                "~parking_controller/angular_deadband", 2.0
            ),
            "center_deadband_px": rospy.get_param(
                "~parking_controller/center_deadband_px", 8.0
            ),
            "yaw_deadband_px": rospy.get_param(
                "~parking_controller/yaw_deadband_px", 6.0
            ),
            "min_linear": rospy.get_param("~parking_controller/min_linear", 0.01),
            "min_angular": rospy.get_param("~parking_controller/min_angular", 0.01),
            "accel_linear": rospy.get_param("~parking_controller/accel_linear", 0.3),
            "accel_angular": rospy.get_param(
                "~parking_controller/accel_angular", 1.2
            ),
            "approach_speed": rospy.get_param(
                "~parking_controller/approach_speed", 0.08
            ),
            "consecutive_frames": rospy.get_param(
                "~parking_controller/consecutive_frames", 3
            ),
            "max_frame_age": rospy.get_param(
                "~parking_controller/max_frame_age", 0.5
            ),
            "chassis_capability": rospy.get_param(
                "~parking_controller/chassis_capability", "differential"
            ),
            "visual_stop_front_y": rospy.get_param(
                "~parking_controller/visual_stop_front_y", 420.0
            ),
            "verify_duration": rospy.get_param(
                "~parking_controller/verify_duration", 1.0
            ),
            "velocity_threshold": rospy.get_param(
                "~parking_controller/velocity_threshold", 0.02
            ),
            "frame_reacquire_timeout": rospy.get_param(
                "~parking_controller/frame_reacquire_timeout", 1.0
            ),
            "near_zone_front_y": rospy.get_param(
                "~parking_controller/near_zone_front_y", 380.0
            ),
            "near_zone_loss_is_fatal": rospy.get_param(
                "~parking_controller/near_zone_loss_is_fatal", True
            ),
            "enabled": rospy.get_param("~lidar_safety/enabled", False),
            "front_sector_deg": rospy.get_param(
                "~lidar_safety/front_sector_deg", 60.0
            ),
            "min_range": rospy.get_param("~lidar_safety/min_range", 0.05),
            "max_range": rospy.get_param("~lidar_safety/max_range", 8.0),
            "safety_stop_distance": rospy.get_param(
                "~lidar_safety/safety_stop_distance", 0.25
            ),
            "target_stop_distance": rospy.get_param(
                "~lidar_safety/target_stop_distance", 0.30
            ),
        }
        timeouts = {
            "align": rospy.get_param("~timeouts/frame_align", 30.0),
            "center": rospy.get_param("~timeouts/frame_center", 30.0),
            "approach": rospy.get_param("~timeouts/approach", 60.0),
            "final_stop": rospy.get_param("~timeouts/final_stop", 30.0),
            "verify": rospy.get_param("~timeouts/verify", 15.0),
        }
        self._outputs = []
        self._controller = ParkingController(
            self._outputs, rospy.get_time, timeouts, config
        )
        self._lidar = LidarSafety(config)
        self._front_range = None
        self._context = None
        self._last_state = None
        self._cmd_pub = rospy.Publisher(
            rospy.get_param(
                "~topics/cmd_vel_parking", "/cmd_vel/delivery_parking"
            ),
            Twist, queue_size=1,
        )
        self._progress_pub = rospy.Publisher(
            rospy.get_param(
                "~topics/parking_progress", "/task/delivery_parking_progress"
            ),
            String, queue_size=10,
        )
        self._result_pub = rospy.Publisher(
            rospy.get_param(
                "~topics/parking_result", "/task/delivery_parking_result"
            ),
            String, queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~topics/parking_start", "/task/delivery_parking_start"
            ),
            String, self._on_start, queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~topics/frame_observation", "/task/delivery_frame_observation"
            ),
            String, self._on_observation, queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/scan", "/scan"),
            LaserScan, self._on_scan, queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/odom", "/odom"),
            Odometry, self._on_odometry, queue_size=1,
        )
        rospy.Timer(rospy.Duration(0.2), self._on_tick)
        rospy.on_shutdown(self._on_shutdown)
        self._publish_zero()

    def _context_payload(self):
        return {
            "protocol_version": 1,
            "phase": self._context["phase"],
            "task_id": self._context["task_id"],
            "goal_id": self._context["goal_id"],
        }

    def _on_start(self, message):
        try:
            payload = protocol.load_object(message.data)
        except protocol.ProtocolError as exc:
            rospy.logwarn("ignored invalid parking start: %s", exc)
            return
        self._context = {
            "phase": payload.get("phase"),
            "task_id": payload.get("task_id"),
            "goal_id": payload.get("goal_id"),
        }
        self._controller.start()

    def _matches_context(self, payload):
        if self._context is None:
            return False
        return (
            payload.get("phase") == self._context["phase"]
            and payload.get("task_id") == self._context["task_id"]
            and payload.get("goal_id") == self._context["goal_id"]
        )

    def _on_observation(self, message):
        try:
            payload = protocol.load_object(message.data)
        except protocol.ProtocolError:
            return
        if not self._matches_context(payload):
            return
        raw = payload.get("observation") or {}
        observation = FrameObservation(
            timestamp=float(raw.get("timestamp", 0.0)),
            frame_detected=bool(raw.get("frame_detected", False)),
            confidence=float(raw.get("confidence", 0.0)),
            near_center_x=raw.get("near_center_x"),
            far_center_x=raw.get("far_center_x"),
            left_boundary=raw.get("left_boundary"),
            right_boundary=raw.get("right_boundary"),
            front_boundary_y=raw.get("front_boundary_y"),
            visible_boundary_count=int(raw.get("visible_boundary_count", 0)),
            yaw_error=float(raw.get("yaw_error", 0.0)),
            lateral_error=float(raw.get("lateral_error", 0.0)),
            near_width=raw.get("near_width"),
            far_width=raw.get("far_width"),
            near_thickness=int(raw.get("near_thickness", 0)),
            far_thickness=int(raw.get("far_thickness", 0)),
            confirmed=bool(raw.get("confirmed", False)),
            near_zone=bool(raw.get("near_zone", False)),
            near_zone_loss=bool(raw.get("near_zone_loss", False)),
        )
        self._controller.update(observation, self._front_range, rospy.get_time())
        self._flush_outputs()
        self._publish_progress_if_changed()

    def _on_scan(self, message):
        self._front_range = self._lidar.front_distance(
            message.ranges, message.angle_min, message.angle_increment
        )

    def _on_odometry(self, message):
        velocity = message.twist.twist
        self._controller.on_odometry_velocity(
            velocity.linear.x, velocity.linear.y, velocity.angular.z,
            rospy.get_time(),
        )

    def _on_tick(self, _event):
        self._controller.tick(rospy.get_time())
        self._flush_outputs()
        self._publish_progress_if_changed()

    def _publish_progress_if_changed(self):
        state = self._controller.state
        if state == self._last_state:
            return
        self._last_state = state
        stage = STAGE_BY_STATE.get(state)
        if stage is None or self._context is None:
            return
        self._progress_pub.publish(
            String(data=json.dumps(
                dict(self._context_payload(), stage=stage),
                ensure_ascii=False,
            ))
        )

    def _flush_outputs(self):
        while self._outputs:
            action, payload = self._outputs.pop(0)
            self._dispatch(action, payload)

    def _dispatch(self, action, payload):
        if action == "publish_twist":
            twist = Twist()
            twist.linear.x = float(payload["linear_x"])
            twist.linear.y = float(payload["linear_y"])
            twist.angular.z = float(payload["angular_z"])
            self._cmd_pub.publish(twist)
            return
        if action == "publish_zero":
            self._cmd_pub.publish(Twist())
            return
        if action == "controller_result":
            if self._context is not None:
                self._result_pub.publish(
                    String(data=json.dumps(
                        dict(self._context_payload(), **payload),
                        ensure_ascii=False,
                    ))
                )
            return
        rospy.logerr("unknown parking action: %s", action)

    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    def _on_shutdown(self):
        self._controller.shutdown()
        self._flush_outputs()
        self._publish_zero()


def main():
    rospy.init_node("delivery_parking_controller")
    ParkingControllerNode()
    rospy.loginfo("delivery_parking_controller node started")
    rospy.spin()


if __name__ == "__main__":
    main()
