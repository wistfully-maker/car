#!/usr/bin/env python3
"""Apply the omni TEB parameter set to the live fast-nav move_base, then run
a persistent corner speed limiter.

由 legacy_navigation_include.launch 在 navigation_mode:=omni 时启动，
与 fast-nav 栈同生命周期：

1. 启动时通过 dynamic_reconfigure 一次性注入全向基准参数；
2. 随后常驻运行弯道降速控制：订阅全局路径，前方出现急弯（默认 45 度
   lookahead）时把速度上限切到 CORNER 档（低速），出弯后恢复 STRAIGHT 档；
3. 保持 latched 状态消息可被任意时刻 echo；交接时随本 roslaunch
   进程组被 supervisor 一起回收，不会影响 stop 栈（该栈由
   mission_integration 直接加载自己的全向配置）。
"""

import math
import os
import sys

import dynamic_reconfigure.client
import rospy
import tf2_ros
from nav_msgs.msg import Path
from std_msgs.msg import String

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "src"
    ),
)
from task_orchestrator.omni_corner_logic import (  # noqa: E402
    OmniCornerStateMachine,
    PathMetrics,
    analyze_path,
    corner_mode_parameters,
)
from task_orchestrator.omni_teb_params import (  # noqa: E402
    ensure_corner_is_slower,
    teb_parameters,
    validate_corner_set,
)

_SPEED_FIELDS = (
    "max_vel_x",
    "max_vel_x_backwards",
    "max_vel_y",
    "acc_lim_y",
    "max_vel_theta",
    "acc_lim_theta",
)


def quaternion_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class OmniTebApplier:
    def __init__(self):
        self.config_path = rospy.get_param(
            "~config_path", "/omni_fast_nav/pickup_teb"
        )
        self.reconfigure_namespace = rospy.get_param(
            "~reconfigure_namespace", "/move_base/TebLocalPlannerROS"
        )
        self.timeout = float(rospy.get_param("~timeout", 60.0))
        self.retry_period = float(rospy.get_param("~retry_period", 2.0))
        self.status_pub = rospy.Publisher(
            "/navigation/omni_teb_status", String, queue_size=1, latch=True
        )
        # 弯道降速控制
        self.corner_enabled = bool(
            rospy.get_param("~corner_enabled", True)
        )
        self.plan_topic = rospy.get_param(
            "~plan_topic", "/move_base/GlobalPlanner/plan"
        )
        self.lookahead_distance = float(
            rospy.get_param("~lookahead_distance", 0.8)
        )
        self.resample_spacing = float(
            rospy.get_param("~resample_spacing", 0.1)
        )
        self.controller_rate = float(
            rospy.get_param("~controller_rate", 5.0)
        )
        self.plan_timeout = float(rospy.get_param("~plan_timeout", 2.0))
        self.corner_config_path = rospy.get_param(
            "~corner_config_path", "/omni_fast_nav/pickup_corner_teb"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.mode_pub = rospy.Publisher(
            "/navigation/omni_teb_mode", String, queue_size=1, latch=True
        )
        self.plan_points = []
        self.plan_received_at = None
        self._plan_sub = rospy.Subscriber(
            self.plan_topic, Path, self._plan_callback, queue_size=1
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._machine = OmniCornerStateMachine(
            rospy.get_param("~corner_enter_angle_deg", 45.0),
            rospy.get_param("~corner_exit_angle_deg", 10.0),
            rospy.get_param("~heading_exit_tolerance_deg", 10.0),
            rospy.get_param("~exit_hold_time", 0.5),
        )
        self._client = None
        self._applied_mode = None
        self._base_set = None
        self._corner_set = None
        self.last_error = ""

    # ==================== 状态发布与注入 ====================

    def _publish_status(self, status):
        try:
            self.status_pub.publish(String(data=status))
        except Exception as exc:
            rospy.logwarn("omni TEB status publish failed: %s", exc)

    def _publish_mode(self, mode):
        try:
            self.mode_pub.publish(String(data=mode))
        except Exception as exc:
            rospy.logwarn("omni TEB mode publish failed: %s", exc)

    def _connect_client(self):
        if self._client is not None:
            return self._client
        try:
            self._client = dynamic_reconfigure.client.Client(
                self.reconfigure_namespace, timeout=2.0
            )
            self.last_error = ""
            return self._client
        except Exception as exc:
            self.last_error = "dynamic reconfigure unavailable: %s" % exc
            rospy.logwarn_throttle(5.0, self.last_error)
            return None

    def _update_configuration(self, target):
        client = self._connect_client()
        if client is None:
            return False
        try:
            client.update_configuration(target)
            self.last_error = ""
            return True
        except Exception as exc:
            self.last_error = "dynamic reconfigure failed: %s" % exc
            self._client = None
            rospy.logerr_throttle(5.0, self.last_error)
            return False

    # ==================== 基准注入 ====================

    def _apply_once(self):
        raw = rospy.get_param(self.config_path, None)
        if raw is None:
            rospy.logerr("omni TEB config missing at %s", self.config_path)
            self._publish_status("failed:config_missing")
            return
        try:
            self._base_set = teb_parameters(raw)
        except ValueError as exc:
            rospy.logerr("invalid omni TEB config: %s", exc)
            self._publish_status("failed:invalid_config")
            return
        if self.corner_enabled:
            corner_raw = rospy.get_param(self.corner_config_path, None)
            if corner_raw is None:
                rospy.logerr(
                    "corner TEB config missing at %s", self.corner_config_path
                )
                self._publish_status("failed:corner_config_missing")
                return
            try:
                self._corner_set = ensure_corner_is_slower(
                    self._base_set, validate_corner_set(corner_raw)
                )
            except ValueError as exc:
                rospy.logerr("invalid corner TEB config: %s", exc)
                self._publish_status("failed:invalid_corner_config")
                return
        if not self._update_configuration(self._base_set):
            self._publish_status("failed:update_rejected")
            return
        rospy.loginfo(
            "omni TEB applied: max_vel_x=%.2f max_vel_y=%.2f "
            "weight_kinematics_nh=%.2f",
            self._base_set["max_vel_x"],
            self._base_set["max_vel_y"],
            self._base_set["weight_kinematics_nh"],
        )
        self._publish_status("applied")
        if self.corner_enabled:
            self._applied_mode = "STRAIGHT"

    # ==================== 弯道降速控制 ====================

    def _plan_callback(self, message):
        self.plan_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        self.plan_received_at = rospy.get_time()

    def _invalid_plan(self, reason):
        mode = self._machine.update(
            rospy.get_time(),
            PathMetrics(0.0, 0.0, 0.0),
            0.0,
            plan_valid=False,
        )
        self.last_error = reason
        return mode

    def _evaluate_corner_mode(self):
        now = rospy.get_time()
        if (
            self.plan_received_at is None
            or now - self.plan_received_at > self.plan_timeout
        ):
            return self._invalid_plan("global plan missing or stale")
        if len(self.plan_points) < 2:
            return self._invalid_plan("global plan has fewer than two points")
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception as exc:
            return self._invalid_plan("TF unavailable: %s" % exc)
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
        mode = self._machine.update(now, metrics, heading, plan_valid=True)
        self.last_error = ""
        return mode

    def _apply_speed_mode(self, mode):
        if self._base_set is None:
            return
        if mode == self._applied_mode:
            return
        straight_speeds = {
            field: self._base_set[field] for field in _SPEED_FIELDS
        }
        target = corner_mode_parameters(
            mode, straight_speeds, self._corner_set
        )
        if not self._update_configuration(target):
            return
        self._applied_mode = mode
        self._publish_mode(mode)
        rospy.loginfo(
            "omni TEB speed mode=%s max_vel_x=%.2f max_vel_theta=%.2f",
            mode,
            target["max_vel_x"],
            target["max_vel_theta"],
        )

    def _run_corner_control(self):
        rate = rospy.Rate(self.controller_rate)
        while not rospy.is_shutdown():
            mode = self._evaluate_corner_mode()
            self._apply_speed_mode(mode)
            rate.sleep()

    def run(self):
        self._apply_once()
        if self.corner_enabled:
            self._run_corner_control()
        else:
            rospy.spin()


def main():
    rospy.init_node("omni_teb_applier")
    OmniTebApplier().run()
    rospy.spin()


if __name__ == "__main__":
    main()
