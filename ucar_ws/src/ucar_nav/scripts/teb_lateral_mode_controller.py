#!/usr/bin/env python3
"""Switch TEB lateral limits according to upcoming global-path curvature."""

import json
import math
import os
import sys

import dynamic_reconfigure.client
from nav_msgs.msg import Path
import rospy
from std_msgs.msg import String
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lateral_mode_logic import (  # noqa: E402
    LateralModeStateMachine,
    PathMetrics,
    analyze_path,
    mode_parameters,
)


def quaternion_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class TebLateralModeController:
    def __init__(self):
        self.controller_rate = float(rospy.get_param("~controller_rate", 5.0))
        self.plan_topic = rospy.get_param(
            "~plan_topic", "/move_base/NavfnROS/plan"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lookahead_distance = float(
            rospy.get_param("~lookahead_distance", 0.8)
        )
        self.resample_spacing = float(
            rospy.get_param("~resample_spacing", 0.10)
        )
        self.plan_timeout = float(rospy.get_param("~plan_timeout", 1.0))
        self.straight_max_vel_x = float(
            rospy.get_param("~straight_max_vel_x", 0.45)
        )
        self.straight_max_vel_x_backwards = float(
            rospy.get_param("~straight_max_vel_x_backwards", 0.10)
        )
        self.straight_max_vel_y = float(
            rospy.get_param("~straight_max_vel_y", 0.02)
        )
        self.straight_acc_lim_y = float(
            rospy.get_param("~straight_acc_lim_y", 0.20)
        )
        self.corner_max_vel_x = float(
            rospy.get_param("~corner_max_vel_x", 0.20)
        )
        self.corner_max_vel_x_backwards = float(
            rospy.get_param("~corner_max_vel_x_backwards", 0.02)
        )
        self.corner_max_vel_y = float(
            rospy.get_param("~corner_max_vel_y", 0.18)
        )
        self.corner_acc_lim_y = float(
            rospy.get_param("~corner_acc_lim_y", 0.60)
        )
        self.reconfigure_namespace = rospy.get_param(
            "~reconfigure_namespace", "/move_base/TebLocalPlannerROS"
        )

        self.machine = LateralModeStateMachine(
            rospy.get_param("~corner_enter_angle_deg", 45.0),
            rospy.get_param("~corner_exit_angle_deg", 10.0),
            rospy.get_param("~heading_exit_tolerance_deg", 10.0),
            rospy.get_param("~exit_hold_time", 0.5),
        )
        self.plan_points = []
        self.plan_received_at = None
        self.client = None
        self.applied_mode = None
        self.last_error = ""

        self.mode_publisher = rospy.Publisher(
            "/navigation/lateral_mode", String, queue_size=1, latch=True
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/navigation/lateral_mode_diagnostics",
            String,
            queue_size=1,
            latch=True,
        )
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.plan_subscriber = rospy.Subscriber(
            self.plan_topic, Path, self._plan_callback, queue_size=1
        )
        rospy.on_shutdown(self._on_shutdown)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.controller_rate), self._tick
        )

    def _plan_callback(self, message):
        self.plan_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        self.plan_received_at = rospy.Time.now()

    def _connect_client(self):
        if self.client is not None:
            return True
        try:
            self.client = dynamic_reconfigure.client.Client(
                self.reconfigure_namespace, timeout=1.0
            )
            self.last_error = ""
            return True
        except Exception as error:  # ROS service failures vary by distro.
            self.last_error = "dynamic reconfigure unavailable: {}".format(error)
            rospy.logwarn_throttle(5.0, self.last_error)
            return False

    def _apply_mode(self, mode):
        if mode == self.applied_mode:
            return
        if not self._connect_client():
            return
        target = mode_parameters(
            mode,
            self.straight_max_vel_x,
            self.straight_max_vel_x_backwards,
            self.straight_max_vel_y,
            self.straight_acc_lim_y,
            self.corner_max_vel_x,
            self.corner_max_vel_x_backwards,
            self.corner_max_vel_y,
            self.corner_acc_lim_y,
        )
        try:
            self.client.update_configuration(target)
            self.applied_mode = mode
            self.last_error = ""
            rospy.loginfo(
                "TEB lateral mode=%s max_vel_x=%.3f max_vel_y=%.3f acc_lim_y=%.3f",
                mode,
                target["max_vel_x"],
                target["max_vel_y"],
                target["acc_lim_y"],
            )
        except Exception as error:
            self.last_error = "dynamic reconfigure failed: {}".format(error)
            self.client = None
            rospy.logerr_throttle(5.0, self.last_error)

    def _invalid_plan(self, reason):
        mode = self.machine.update(
            rospy.Time.now().to_sec(),
            PathMetrics(0.0, 0.0, 0.0),
            0.0,
            plan_valid=False,
        )
        self.last_error = reason
        return mode, PathMetrics(0.0, 0.0, 0.0), None

    def _evaluate(self):
        now = rospy.Time.now()
        if (
            self.plan_received_at is None
            or (now - self.plan_received_at).to_sec() > self.plan_timeout
        ):
            return self._invalid_plan("global plan missing or stale")
        if len(self.plan_points) < 2:
            return self._invalid_plan("global plan has fewer than two points")

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception as error:
            return self._invalid_plan("TF unavailable: {}".format(error))

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        heading = quaternion_yaw(transform.transform.rotation)
        nearest_index = min(
            range(len(self.plan_points)),
            key=lambda index: (
                (self.plan_points[index][0] - x) ** 2
                + (self.plan_points[index][1] - y) ** 2
            ),
        )
        metrics = analyze_path(
            self.plan_points[nearest_index:],
            self.lookahead_distance,
            self.resample_spacing,
        )
        mode = self.machine.update(
            now.to_sec(), metrics, heading, plan_valid=True
        )
        self.last_error = ""
        return mode, metrics, nearest_index

    def _publish(self, mode, metrics, nearest_index):
        self.mode_publisher.publish(String(data=mode))
        diagnostics = {
            "mode": mode,
            "applied_mode": self.applied_mode,
            "turn_angle_deg": round(metrics.turn_angle_deg, 3),
            "exit_heading_rad": round(metrics.exit_heading_rad, 4),
            "analyzed_length": round(metrics.analyzed_length, 3),
            "nearest_path_index": nearest_index,
            "plan_points": len(self.plan_points),
            "error": self.last_error,
        }
        self.diagnostics_publisher.publish(
            String(data=json.dumps(diagnostics, ensure_ascii=False))
        )

    def _tick(self, _event):
        mode, metrics, nearest_index = self._evaluate()
        self._apply_mode(mode)
        self._publish(mode, metrics, nearest_index)

    def _on_shutdown(self):
        try:
            self._apply_mode("STRAIGHT")
        except Exception:
            pass


def main():
    rospy.init_node("teb_lateral_mode_controller")
    TebLateralModeController()
    rospy.spin()


if __name__ == "__main__":
    main()
