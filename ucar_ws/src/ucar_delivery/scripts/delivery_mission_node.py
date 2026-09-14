#!/usr/bin/env python3
"""Delivery mission supervisor: protocol, mission engine, move_base session
management and stage orchestration for physical and simulation delivery.

Owns /ucar_delivery/motion_mode publication during the delivery phases, the
move_base action client (through NavSupervisor), sign/frame/parking stage
triggers and the final arrival results. Its manual sign-search/alignment
commands are published only on /cmd_vel/delivery_manual and selected by the
standalone delivery velocity mux.
"""

import json
import math
import threading
import uuid

import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ucar_delivery import protocol
from ucar_delivery.mission import DeliveryMission
from ucar_delivery.nav_supervisor import NavSupervisor
from ucar_delivery.staging_pose_estimator import (
    StagingPoseConsistency,
    StagingPoseError,
    StagingPoseEstimator,
)

# 包内私有运动模式词汇（不依赖 task_orchestrator）。
# 未来接入竞争编排器时，外部仲裁器按同一词汇消费 /ucar_delivery/motion_mode。
IDLE = "IDLE"
NAVIGATION = "NAVIGATION"
VISUAL_SEARCH = "VISUAL_SEARCH"
PARKING = "PARKING"
EMERGENCY_STOP = "EMERGENCY_STOP"
MOTION_MODES = frozenset(
    (IDLE, NAVIGATION, VISUAL_SEARCH, PARKING, EMERGENCY_STOP)
)


def wrap_angle(angle):
    """Wrap an angle into [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def build_move_base_goal(viewpoint, frame_id="map"):
    """Build a MoveBaseGoal from {x, y, yaw} (pure, testable)."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x=float(viewpoint["x"]), y=float(viewpoint["y"]))
    q = quaternion_from_euler(0.0, 0.0, float(viewpoint.get("yaw", 0.0)))
    goal.target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return goal


def odometry_yaw(message):
    """Extract the yaw from an Odometry message (pure, testable)."""
    orientation = message.pose.pose.orientation
    _, _, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )
    return yaw


class DeliveryMissionNode:
    PARKING_PROGRESS_STAGES = {
        "aligned": "on_frame_aligned",
        "centered": "on_frame_centered",
        "approach_started": "on_approach_complete",
        "final_stop_done": "on_final_stop_done",
        "verified": "on_stop_verified",
    }

    def __init__(self):
        self._lock = threading.RLock()
        self._outputs = []
        self._params = self._load_params()
        mission_config = dict(self._params["mission"])
        staging_config = self._params["staging_pose"]
        mission_config.update(
            {
                "staging_estimation_max_retries": int(
                    staging_config.get("staging_estimation_max_retries", 3)
                ),
                "staging_navigation_max_retries": int(
                    staging_config.get("staging_navigation_max_retries", 2)
                ),
            }
        )
        self._mission = DeliveryMission(
            self._outputs, rospy.get_time, self._params["timeouts"],
            mission_config,
        )
        self._supervisor = NavSupervisor(
            self._outputs,
            rospy.get_time,
            self._params["nav_supervisor"],
            self._params["nav_supervisor"],
        )
        self._publishers = self._make_publishers()
        self._viewpoints = self._params["viewpoints"]
        self._frame_id = self._params["frames"]["map"]
        self._laser_frame = self._params["frames"]["laser"]
        # 动态观察点：视觉限定雷达估计 + 连续一致确认
        self._staging_enabled = bool(staging_config.get("enabled", True))
        self._staging_estimator = StagingPoseEstimator(staging_config)
        self._staging_consistency = StagingPoseConsistency(staging_config)
        self._max_scan_age = float(staging_config.get("max_scan_age", 0.5))
        # ROS1 tf.TransformListener.lookupTransform 没有 timeout 参数；
        # tf_timeout 配置保留（见 delivery.yaml 注释），本节点不使用。
        self._tf_listener = TransformListener()
        self._scan_ranges = None
        self._scan_angle_min = None
        self._scan_angle_increment = None
        self._scan_frame = None
        self._scan_stamp = None
        self._scan_stamp_sec = None
        # 已消费的 (frame_id, stamp)；成功或失败都不得重复使用同一帧。
        self._last_estimated_scan_id = None
        self._search_angular_speed = float(
            self._params.get("sign_search", {}).get("angular_speed", 0.35)
        )
        self._align = self._params.get("sign_align", {})
        self._align_gain = float(self._align.get("angular_gain", 0.6))
        self._align_max_angular = float(self._align.get("max_angular", 0.3))
        self._align_tolerance = float(self._align.get("tolerance_rad", 0.06))
        self._align_confirm_frames = int(self._align.get("confirm_frames", 3))
        self._align_target_yaw = None
        self._align_streak = 0
        self._current_yaw = 0.0
        self._rotating_for_search = False
        self._parking_started = False
        self._last_action_state = None
        self._last_mission_state = None
        # 手动命令域（SEARCH_SIGN/ALIGN_SIGN）是否正处于活动状态；
        # 离开该域时发布一次性零命令。
        self._manual_active = False

        self._action_client = SimpleActionClient(
            self._params["move_base_action"], MoveBaseAction
        )
        self._subscribe()
        self._flush_outputs()
        self._publish_zero()
        rospy.Timer(rospy.Duration(0.2), self._on_tick)
        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------- config

    def _load_params(self):
        viewpoints = rospy.get_param("~viewpoints", {})
        for phase in protocol.PHASES:
            if phase not in viewpoints:
                raise ValueError("missing viewpoints for phase %s" % phase)
        params = {
            "viewpoints": viewpoints,
            "timeouts": rospy.get_param("~timeouts", {}),
            "mission": rospy.get_param("~mission", {}),
            "nav_supervisor": rospy.get_param("~nav_supervisor", {}),
            "staging_pose": rospy.get_param("~staging_pose", {}),
            "sign_search": rospy.get_param("~sign_search", {}),
            "sign_align": rospy.get_param("~sign_align", {}),
            "topics": rospy.get_param("~topics", {}),
            "frames": rospy.get_param("~frames", {}),
            "move_base_action": rospy.get_param("~move_base_action", "move_base"),
        }
        return params

    def _topic(self, key, default):
        return self._params["topics"].get(key, default)

    def _make_publishers(self):
        return {
            "publish_motion_mode": rospy.Publisher(
                self._topic("motion_mode", "/ucar_delivery/motion_mode"),
                String, queue_size=1, latch=True,
            ),
            "publish_status": rospy.Publisher(
                self._topic("delivery_status", "/task/delivery_status"),
                String, queue_size=10,
            ),
            "publish_arrival": None,  # resolved per phase at dispatch time
            "publish_delivery_arrival": rospy.Publisher(
                self._topic("delivery_arrived", "/task/delivery_arrived"),
                String, queue_size=10,
            ),
            "publish_simulation_arrival": rospy.Publisher(
                self._topic("simulation_arrived", "/task/simulation_arrived"),
                String, queue_size=10,
            ),
            "publish_sign_start": rospy.Publisher(
                self._topic("sign_start", "/task/delivery_sign_start"),
                String, queue_size=10,
            ),
            "publish_frame_start": rospy.Publisher(
                self._topic("frame_start", "/task/delivery_frame_start"),
                String, queue_size=10,
            ),
            "publish_parking_start": rospy.Publisher(
                self._topic("parking_start", "/task/delivery_parking_start"),
                String, queue_size=10,
            ),
        }

    # ------------------------------------------------------------ dispatch

    def _dispatch(self, action, payload):
        if action == "send_goal":
            self._send_goal(payload)
            return
        if action == "cancel_goal":
            self._action_client.cancel_goal()
            return
        if action == "session_result":
            if payload.get("status") == "interrupted":
                return
            phase = payload.get("phase")
            ok = bool(payload.get("ok"))
            message = payload.get("message") or ""
            if payload.get("goal_kind") == "staging":
                self._mission.on_staging_navigation_result(phase, ok, message)
            else:
                self._mission.on_viewpoint_result(phase, ok, message)
            return
        if action == "publish_motion_mode":
            if payload not in MOTION_MODES:
                rospy.logerr("invalid mission motion mode %r", payload)
                payload = IDLE
            self._publishers["publish_motion_mode"].publish(String(data=payload))
            return
        if action == "publish_status":
            self._publishers["publish_status"].publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
            return
        if action == "publish_arrival":
            publisher = (
                self._publishers["publish_delivery_arrival"]
                if payload.get("goal_id") and self._mission.phase == protocol.PHASE_PHYSICAL
                else self._publishers["publish_simulation_arrival"]
            )
            publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            return
        if action == "navigate_to_viewpoint":
            self._start_viewpoint_navigation(payload)
            return
        if action == "cancel_navigation":
            if payload.get("reason") == "sign detected":
                self._supervisor.on_detection(rospy.get_time())
            else:
                self._supervisor.cancel(payload.get("reason") or "cancelled")
            return
        if action == "start_staging_estimate":
            # 新的估计周期：清空一致性确认状态，避免旧任务/旧点的计数残留。
            self._staging_consistency.reset()
            return
        if action == "navigate_to_staging":
            self._supervisor.begin(dict(payload, goal_kind="staging"))
            return
        if action == "start_sign_search":
            self._publishers["publish_sign_start"].publish(
                String(data=json.dumps(
                    {
                        "protocol_version": 1,
                        "phase": payload["phase"],
                        "task_id": payload["task_id"],
                        "goal_id": payload["goal_id"],
                        "target_workshop": self._mission.goal["target_workshop"],
                    },
                    ensure_ascii=False,
                ))
            )
            self._rotating_for_search = True
            return
        if action == "stop_sign_search":
            self._rotating_for_search = False
            return
        if action == "align_sign":
            self._begin_yaw_align(payload.get("detection"))
            return
        if action == "start_frame_search":
            self._parking_started = False
            self._publishers["publish_frame_start"].publish(
                String(data=json.dumps(
                    dict(payload), ensure_ascii=False
                ))
            )
            return
        if action in (
            "align_frame", "center_frame", "start_approach",
            "final_stop", "verify_stop",
        ):
            if action == "align_frame" and not self._parking_started:
                self._parking_started = True
                self._publishers["publish_parking_start"].publish(
                    String(data=json.dumps(
                        {
                            "protocol_version": 1,
                            "phase": self._mission.phase,
                            "task_id": self._mission.goal["task_id"],
                            "goal_id": self._mission.goal["goal_id"],
                        },
                        ensure_ascii=False,
                    ))
                )
            return
        if action == "publish_zero_velocity":
            self._publish_zero()
            return
        if action == "publish_safe_stop":
            rospy.logwarn("delivery safe stop: %s", payload.get("reason"))
            return
        rospy.logerr("unknown mission action: %s", action)

    def _flush_outputs(self):
        while True:
            with self._lock:
                if not self._outputs:
                    break
                action, payload = self._outputs.pop(0)
            try:
                self._dispatch(action, payload)
            except Exception as exc:
                rospy.logerr("mission dispatch %s failed: %s", action, exc)

    def _start_viewpoint_navigation(self, payload):
        phase = payload["phase"]
        index = int(payload["viewpoint_index"])
        viewpoints = self._viewpoints[phase]
        if index >= len(viewpoints):
            self._supervisor.begin(
                dict(payload, viewpoint=viewpoints[0], goal_kind="viewpoint")
            )
            return
        viewpoint = viewpoints[index]
        self._supervisor.begin(
            dict(payload, viewpoint=viewpoint, goal_kind="viewpoint")
        )

    def _send_goal(self, payload):
        # 动态观察点目标直接携带 map 系 pose；搜索点使用配置的 viewpoint。
        pose = payload.get("pose") or payload.get("viewpoint")
        if pose is None:
            rospy.logerr("send_goal without pose configuration")
            self._supervisor.on_status("ABORTED", rospy.get_time(),
                                       "missing pose configuration")
            return
        goal = build_move_base_goal(pose, self._frame_id)
        self._action_client.send_goal(goal)

    # -------------------------------------------------------------- action

    def _on_tick(self, _event):
        with self._lock:
            state = self._action_client.get_state()
            if state != self._last_action_state:
                self._last_action_state = state
                if state in (3, 4):  # SUCCEEDED, ABORTED
                    self._supervisor.on_status(
                        "SUCCEEDED" if state == 3 else "ABORTED",
                        rospy.get_time(),
                    )
                elif state == 2:  # PREEMPTED
                    self._supervisor.on_status("PREEMPTED", rospy.get_time())
                elif state in (0, 1):  # PENDING, ACTIVE
                    self._supervisor.on_status("ACTIVE", rospy.get_time())
            self._supervisor.tick(rospy.get_time())
            self._mission.tick()
            self._update_staging_estimate()
            self._update_manual_commands()
        self._flush_outputs()

    # --------------------------------------------------------- staging pose

    def _lookup_laser_to_map_transform(self):
        """Query the laser->map transform at the scan frame timestamp.

        Uses the ROS1 tf.TransformListener.lookupTransform(target, source,
        time) signature: the transform must exist at the scan frame time
        (not "latest"). Returns {x, y, yaw} or None; TF failures must never
        be treated as a valid transform.
        """
        if self._scan_stamp is None or not self._scan_frame:
            return None
        try:
            translation, rotation = self._tf_listener.lookupTransform(
                self._frame_id, self._scan_frame, self._scan_stamp,
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "laser->map TF lookup failed: %s", exc)
            return None
        _, _, yaw = euler_from_quaternion(rotation)
        return {"x": float(translation[0]), "y": float(translation[1]),
                "yaw": float(yaw)}

    def _update_staging_estimate(self):
        mission = self._mission
        if mission.state != DeliveryMission.ESTIMATE_STAGING_POSE:
            return
        if not self._staging_enabled:
            mission.on_staging_estimate_failed(
                mission.phase, "staging estimation disabled"
            )
            return
        if self._scan_stamp_sec is None or (
            rospy.get_time() - self._scan_stamp_sec
        ) > self._max_scan_age:
            # 没有新的可信 scan 时等待状态机的有界超时；不得把同一份
            # 缺失/陈旧数据快速重复计作多次估计失败。
            return
        scan_id = (self._scan_frame, self._scan_stamp_sec)
        if scan_id == self._last_estimated_scan_id:
            return
        # 在任何 TF/拟合工作前消费该帧：失败帧同样不可在重新对正后复用。
        self._last_estimated_scan_id = scan_id
        detection = mission.detection or {}
        bearing = detection.get("bearing_rad")
        half_width = detection.get("half_width_rad")
        if bearing is None or half_width is None:
            # 标牌检测必须提供 bearing/half-width，缺失视为不可信观测。
            mission.on_staging_estimate_failed(
                mission.phase, "detection lacks bearing or half width"
            )
            return
        transform = self._lookup_laser_to_map_transform()
        if transform is None:
            mission.on_staging_estimate_failed(mission.phase, "tf unavailable")
            return
        try:
            pose = self._staging_estimator.estimate(
                self._scan_ranges,
                self._scan_angle_min,
                self._scan_angle_increment,
                float(bearing),
                float(half_width),
                transform,
            )
        except StagingPoseError as exc:
            mission.on_staging_estimate_failed(mission.phase, str(exc))
            return
        except Exception as exc:
            mission.on_staging_estimate_failed(
                mission.phase, "staging estimate error: %s" % exc
            )
            return
        if self._staging_consistency.update(pose):
            # 连续一致达到阈值：只发送一次 staging goal。
            self._staging_consistency.reset()
            mission.on_staging_pose_estimated(mission.phase, pose)

    def _update_manual_commands(self):
        state = self._mission.state
        manual = state in (
            DeliveryMission.SEARCH_SIGN, DeliveryMission.ALIGN_SIGN,
        )
        if not manual:
            # 白色停车框阶段由 parking_controller 独占 /cmd_vel/delivery_parking，
            # 本节点不发布任何周期命令；仅在手动静止/离开时发一次性零。
            if self._manual_active:
                self._manual_active = False
                self._publish_zero()
            return
        self._manual_active = True
        if state == DeliveryMission.SEARCH_SIGN and self._rotating_for_search:
            self._publish_twist(0.0, 0.0, self._search_angular_speed)
            return
        if state == DeliveryMission.ALIGN_SIGN and self._align_target_yaw is not None:
            self._drive_yaw_align()
            return

    def _publish_twist(self, linear_x, linear_y, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self._publish_manual_twist(twist)

    def _publish_manual_twist(self, twist):
        publisher = getattr(self, "_manual_publisher", None)
        if publisher is None:
            publisher = rospy.Publisher(
                self._topic("cmd_vel_manual", "/cmd_vel/delivery_manual"),
                Twist, queue_size=1,
            )
            self._manual_publisher = publisher
        publisher.publish(twist)

    def _publish_zero(self):
        self._publish_twist(0.0, 0.0, 0.0)

    def _begin_yaw_align(self, detection):
        pose = (detection or {}).get("observed_pose") or {}
        base_yaw = float(pose.get("yaw", self._current_yaw))
        detection_yaw = float((detection or {}).get("detection_yaw") or 0.0)
        self._align_target_yaw = wrap_angle(base_yaw + detection_yaw)
        self._align_streak = 0

    def _drive_yaw_align(self):
        error = wrap_angle(self._align_target_yaw - self._current_yaw)
        if abs(error) <= self._align_tolerance:
            self._align_streak += 1
            if self._align_streak >= self._align_confirm_frames:
                self._align_target_yaw = None
                self._mission.on_sign_aligned(self._mission.phase)
            else:
                self._publish_zero()
            return
        self._align_streak = 0
        angular = max(-self._align_max_angular,
                      min(self._align_max_angular, self._align_gain * error))
        self._publish_twist(0.0, 0.0, angular)

    # ---------------------------------------------------------- subscribers

    def _on_navigation_goal(self, phase, message):
        try:
            parser = (
                protocol.parse_delivery_goal
                if phase == protocol.PHASE_PHYSICAL
                else protocol.parse_simulation_goal
            )
            goal = parser(message.data)
        except protocol.ProtocolError as exc:
            rospy.logwarn("ignored invalid %s goal: %s", phase, exc)
            return
        with self._lock:
            self._mission.accept_goal(goal)
        self._flush_outputs()

    def _on_sign_found(self, message):
        try:
            parsed = protocol.load_object(message.data)
        except protocol.ProtocolError:
            return
        with self._lock:
            mission = self._mission
            if mission.goal is None:
                return
            if parsed.get("phase") != mission.phase:
                return
            if parsed.get("task_id") != mission.goal["task_id"]:
                return
            if parsed.get("goal_id") != mission.goal["goal_id"]:
                return
            mission.on_sign_found(mission.phase, parsed.get("detection"))
        self._flush_outputs()

    def _on_frame_observation(self, message):
        try:
            parsed = protocol.load_object(message.data)
        except protocol.ProtocolError:
            return
        with self._lock:
            mission = self._mission
            if mission.goal is None:
                return
            if parsed.get("phase") != mission.phase:
                return
            if parsed.get("task_id") != mission.goal["task_id"]:
                return
            if parsed.get("goal_id") != mission.goal["goal_id"]:
                return
            if mission.state == DeliveryMission.ACQUIRE_FRAME:
                mission.on_frame_acquired(
                    mission.phase, parsed.get("observation") or {}
                )
        self._flush_outputs()

    def _on_parking_progress(self, message):
        try:
            parsed = protocol.load_object(message.data)
        except protocol.ProtocolError:
            return
        handler_name = self.PARKING_PROGRESS_STAGES.get(parsed.get("stage"))
        if handler_name is None:
            return
        with self._lock:
            mission = self._mission
            if mission.goal is None:
                return
            if parsed.get("phase") != mission.phase:
                return
            if parsed.get("task_id") != mission.goal["task_id"]:
                return
            if parsed.get("goal_id") != mission.goal["goal_id"]:
                return
            handler = getattr(mission, handler_name)
            handler(mission.phase, parsed.get("observation") or {})
        self._flush_outputs()

    def _on_parking_result(self, message):
        try:
            parsed = protocol.load_object(message.data)
        except protocol.ProtocolError:
            return
        with self._lock:
            mission = self._mission
            if mission.goal is None:
                return
            if parsed.get("phase") != mission.phase:
                return
            if parsed.get("task_id") != mission.goal["task_id"]:
                return
            if parsed.get("goal_id") != mission.goal["goal_id"]:
                return
            if parsed.get("status") != "ok":
                mission.fail(parsed.get("message") or "parking failed")
        self._flush_outputs()

    def _on_cancel(self, message):
        try:
            parsed = protocol.load_object(message.data)
            reason = parsed.get("reason") or "cancelled"
        except protocol.ProtocolError:
            reason = "cancelled"
        with self._lock:
            self._mission.cancel(reason)
            self._supervisor.cancel(reason)
        self._flush_outputs()

    def _on_odometry(self, message):
        self._current_yaw = odometry_yaw(message)
        velocity = message.twist.twist
        try:
            stamp = float(message.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            stamp = rospy.get_time()
        if not math.isfinite(stamp) or stamp <= 0.0:
            stamp = rospy.get_time()
        self._supervisor.on_odometry_velocity(
            velocity.linear.x,
            velocity.linear.y,
            velocity.angular.z,
            stamp,
        )

    def _on_scan(self, message):
        try:
            frame_id = str(message.header.frame_id).strip()
            stamp = message.header.stamp
            stamp_sec = float(stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            frame_id, stamp, stamp_sec = "", None, None
        if (
            not frame_id or stamp is None or stamp_sec is None
            or not math.isfinite(stamp_sec) or stamp_sec <= 0.0
        ):
            self._scan_ranges = None
            self._scan_angle_min = None
            self._scan_angle_increment = None
            self._scan_frame = None
            self._scan_stamp = None
            self._scan_stamp_sec = None
            rospy.logwarn_throttle(
                5.0, "rejecting LaserScan with empty frame or invalid stamp"
            )
            return
        self._scan_ranges = message.ranges
        self._scan_angle_min = message.angle_min
        self._scan_angle_increment = message.angle_increment
        self._scan_frame = frame_id
        self._scan_stamp = stamp
        self._scan_stamp_sec = stamp_sec

    def _subscribe(self):
        topics = self._params["topics"]
        rospy.Subscriber(
            topics.get("scan", "/scan"),
            LaserScan, self._on_scan, queue_size=1,
        )
        rospy.Subscriber(
            topics.get("delivery_goal", "/task/delivery_navigation_goal"),
            String,
            lambda m: self._on_navigation_goal(protocol.PHASE_PHYSICAL, m),
            queue_size=1,
        )
        rospy.Subscriber(
            topics.get("simulation_goal", "/task/simulation_navigation_goal"),
            String,
            lambda m: self._on_navigation_goal(protocol.PHASE_SIMULATION, m),
            queue_size=1,
        )
        rospy.Subscriber(
            topics.get("sign_found", "/task/delivery_sign_found"),
            String, self._on_sign_found, queue_size=1,
        )
        rospy.Subscriber(
            topics.get("frame_observation", "/task/delivery_frame_observation"),
            String, self._on_frame_observation, queue_size=1,
        )
        rospy.Subscriber(
            topics.get("parking_progress", "/task/delivery_parking_progress"),
            String, self._on_parking_progress, queue_size=1,
        )
        rospy.Subscriber(
            topics.get("parking_result", "/task/delivery_parking_result"),
            String, self._on_parking_result, queue_size=1,
        )
        rospy.Subscriber(
            topics.get("cancel", "/task/cancel"),
            String, self._on_cancel, queue_size=1,
        )
        rospy.Subscriber(
            topics.get("odom", "/odom"),
            Odometry, self._on_odometry, queue_size=1,
        )

    def _on_shutdown(self):
        with self._lock:
            self._supervisor.shutdown()
            self._mission.cancel("node shutdown")
        self._publish_zero()
        self._flush_outputs()


def main():
    rospy.init_node("delivery_mission")
    DeliveryMissionNode()
    rospy.loginfo("delivery_mission node started")
    rospy.spin()


if __name__ == "__main__":
    main()
