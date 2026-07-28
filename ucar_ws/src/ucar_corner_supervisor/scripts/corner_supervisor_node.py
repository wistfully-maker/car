#!/usr/bin/env python3

"""ROS velocity supervisor that executes large path corners in place."""

import json
import math
import threading

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Path
import rosgraph
import rospy
from std_msgs.msg import String
import tf2_ros

from ucar_corner_supervisor.corner_geometry import (
    CornerObservation,
    Supervisor,
    SupervisorConfig,
)
from ucar_corner_supervisor.path_corners import (
    CornerQueue,
    extract_corner_plan,
)
from ucar_corner_supervisor.swept_collision import (
    GridMap,
    apply_grid_update,
    check_rotation_sweep,
)


RAW_COMMAND_TOPIC = "/move_base/cmd_vel_raw"
GLOBAL_PLAN_TOPIC = "/move_base/NavfnROS/plan"
MOVE_BASE_STATUS_TOPIC = "/move_base/status"
COSTMAP_TOPIC = "/move_base/local_costmap/costmap"
COSTMAP_UPDATE_TOPIC = "/move_base/local_costmap/costmap_updates"
OUTPUT_COMMAND_TOPIC = "/cmd_vel"
STATE_TOPIC = "~state"
DIAGNOSTIC_TOPIC = "~diagnostic"


def quaternion_yaw(rotation):
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class CornerSupervisorNode:
    def __init__(self):
        self._lock = threading.RLock()
        self._raw_command = (0.0, 0.0, 0.0)
        self._raw_stamp = None
        self._path = []
        self._path_stamp = None
        self._path_revision = 0
        self._planned_revision = -1
        self._planned_path = []
        self._planned_distances = []
        self._goal_active = False
        self._costmap = None
        self._costmap_stamp = None
        self._costmap_frame = None
        self._last_state = None
        self._last_ownership_check = None

        self._rate = float(rospy.get_param("~controller_rate", 20.0))
        self._raw_timeout = float(
            rospy.get_param("~raw_command_timeout", 0.5)
        )
        self._path_timeout = float(rospy.get_param("~path_timeout", 0.0))
        self._ownership_check_interval = float(
            rospy.get_param("~ownership_check_interval", 0.5)
        )
        self._min_corner_angle = math.radians(
            float(rospy.get_param("~min_corner_angle_deg", 45.0))
        )
        self._simplify_tolerance = float(
            rospy.get_param("~path_simplify_tolerance", 0.08)
        )
        self._min_segment_length = float(
            rospy.get_param("~min_stable_segment_length", 0.08)
        )
        self._max_fit_residual = float(
            rospy.get_param("~max_fit_residual", 0.08)
        )
        self._same_turn_merge_distance = float(
            rospy.get_param("~same_turn_merge_distance", 0.05)
        )
        self._path_resample_spacing = float(
            rospy.get_param("~path_resample_spacing", 0.05)
        )
        self._corner_release_margin = float(
            rospy.get_param("~corner_release_margin", 0.10)
        )
        self._completed_match_distance = float(
            rospy.get_param("~completed_corner_match_distance", 0.25)
        )
        self._completed_match_heading = math.radians(
            float(
                rospy.get_param(
                    "~completed_corner_match_heading_deg", 20.0
                )
            )
        )
        self._corner_queue = CornerQueue(
            match_distance=self._completed_match_distance,
            match_heading=self._completed_match_heading,
        )
        self._costmap_timeout = float(
            rospy.get_param("~costmap_timeout", 0.5)
        )
        self._lethal_cost_threshold = int(
            rospy.get_param("~lethal_cost_threshold", 253)
        )
        self._sweep_angle_step = math.radians(
            float(rospy.get_param("~sweep_angle_step_deg", 3.0))
        )
        self._footprint = [
            (float(point[0]), float(point[1]))
            for point in rospy.get_param(
                "~footprint",
                [
                    [0.171, -0.128],
                    [0.171, 0.128],
                    [-0.171, 0.128],
                    [-0.171, -0.128],
                ],
            )
        ]
        self._costmap_topic = rospy.get_param(
            "~costmap_topic", COSTMAP_TOPIC
        )
        self._costmap_update_topic = rospy.get_param(
            "~costmap_update_topic", COSTMAP_UPDATE_TOPIC
        )
        config = SupervisorConfig(
            trigger_distance=float(
                rospy.get_param("~corner_trigger_distance", 0.25)
            ),
            release_distance=float(
                rospy.get_param("~corner_release_distance", 0.45)
            ),
            following_max_lateral=float(
                rospy.get_param("~following_max_lateral", 0.02)
            ),
            turn_max_angular=float(
                rospy.get_param("~turn_max_angular", 0.35)
            ),
            turn_min_angular=float(
                rospy.get_param("~turn_min_angular", 0.18)
            ),
            turn_kp=float(rospy.get_param("~turn_kp", 0.9)),
            heading_tolerance=math.radians(
                float(rospy.get_param("~heading_tolerance_deg", 8.0))
            ),
            heading_hold_time=float(
                rospy.get_param("~heading_hold_time", 0.30)
            ),
            turn_timeout=float(rospy.get_param("~turn_timeout", 8.0)),
        )
        self._supervisor = Supervisor(config)

        self._assert_cmd_vel_is_unowned()
        self._command_publisher = rospy.Publisher(
            OUTPUT_COMMAND_TOPIC, Twist, queue_size=1
        )
        self._state_publisher = rospy.Publisher(
            STATE_TOPIC, String, queue_size=1, latch=True
        )
        self._diagnostic_publisher = rospy.Publisher(
            DIAGNOSTIC_TOPIC, String, queue_size=10
        )
        rospy.Subscriber(
            RAW_COMMAND_TOPIC, Twist, self._raw_command_callback, queue_size=1
        )
        rospy.Subscriber(
            GLOBAL_PLAN_TOPIC, Path, self._path_callback, queue_size=1
        )
        rospy.Subscriber(
            MOVE_BASE_STATUS_TOPIC,
            GoalStatusArray,
            self._status_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self._costmap_topic,
            OccupancyGrid,
            self._costmap_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self._costmap_update_topic,
            OccupancyGridUpdate,
            self._costmap_update_callback,
            queue_size=10,
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.on_shutdown(self.stop)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._rate), self._control_callback
        )

    @staticmethod
    def _cmd_vel_publishers():
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        return next(
            (nodes for topic, nodes in publishers if topic == OUTPUT_COMMAND_TOPIC),
            [],
        )

    @classmethod
    def _assert_cmd_vel_is_unowned(cls):
        owners = cls._cmd_vel_publishers()
        if owners:
            raise RuntimeError(
                "{} already has publishers: {}. Remap move_base output to {} "
                "and stop legacy velocity controllers first.".format(
                    OUTPUT_COMMAND_TOPIC, ", ".join(owners), RAW_COMMAND_TOPIC
                )
            )

    def _raw_command_callback(self, message):
        with self._lock:
            self._raw_command = (
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
            self._raw_stamp = rospy.Time.now()

    def _path_callback(self, message):
        with self._lock:
            self._path = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses
            ]
            self._path_stamp = rospy.Time.now()
            self._path_revision += 1

    def _status_callback(self, message):
        active_codes = (GoalStatus.PENDING, GoalStatus.ACTIVE)
        goal_active = any(
            status.status in active_codes for status in message.status_list
        )
        with self._lock:
            if goal_active and not self._goal_active:
                self._path = []
                self._path_stamp = None
                self._planned_path = []
                self._planned_distances = []
                self._corner_queue.reset()
            self._goal_active = goal_active

    def _costmap_callback(self, message):
        grid = GridMap(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            data=list(message.data),
            origin_yaw=quaternion_yaw(message.info.origin.orientation),
        )
        with self._lock:
            self._costmap = grid
            self._costmap_stamp = rospy.Time.now()
            self._costmap_frame = message.header.frame_id

    def _costmap_update_callback(self, message):
        with self._lock:
            current = self._costmap
            if current is None:
                rospy.logwarn_throttle(
                    2.0, "ignoring costmap update before the full map"
                )
                return
            updated = GridMap(
                width=current.width,
                height=current.height,
                resolution=current.resolution,
                origin_x=current.origin_x,
                origin_y=current.origin_y,
                data=list(current.data),
                origin_yaw=current.origin_yaw,
            )
            try:
                apply_grid_update(
                    updated,
                    message.x,
                    message.y,
                    message.width,
                    message.height,
                    list(message.data),
                )
            except ValueError as error:
                rospy.logwarn_throttle(
                    2.0, "invalid local costmap update: %s", error
                )
                return
            self._costmap = updated
            self._costmap_stamp = rospy.Time.now()

    @staticmethod
    def _remaining_path(path, x, y):
        if not path:
            return []
        nearest = min(
            range(len(path)),
            key=lambda index: (path[index][0] - x) ** 2
            + (path[index][1] - y) ** 2,
        )
        return path[nearest:]

    @staticmethod
    def _nearest_path_index(path, x, y):
        if not path:
            return None
        return min(
            range(len(path)),
            key=lambda index: (path[index][0] - x) ** 2
            + (path[index][1] - y) ** 2,
        )

    @staticmethod
    def _path_distances(path):
        distances = [0.0]
        for first, second in zip(path, path[1:]):
            distances.append(
                distances[-1]
                + math.hypot(
                    second[0] - first[0], second[1] - first[1]
                )
            )
        return distances

    def _control_callback(self, _event):
        now = rospy.Time.now()
        if (
            self._last_ownership_check is None
            or (now - self._last_ownership_check).to_sec()
            >= self._ownership_check_interval
        ):
            self._last_ownership_check = now
            try:
                other_publishers = [
                    owner
                    for owner in self._cmd_vel_publishers()
                    if owner != rospy.get_name()
                ]
                if other_publishers:
                    message = "conflicting /cmd_vel publishers: {}".format(
                        ", ".join(other_publishers)
                    )
                    rospy.logerr_throttle(2.0, message)
                    self._supervisor.fail(message)
            except rosgraph.masterapi.Error as error:
                rospy.logwarn_throttle(
                    2.0, "cannot verify /cmd_vel ownership: %s", error
                )
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0), rospy.Duration(0.05)
            )
            translation = transform.transform.translation
            yaw = quaternion_yaw(transform.transform.rotation)
            tf_valid = True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            translation = None
            yaw = 0.0
            tf_valid = False
            rospy.logwarn_throttle(2.0, "corner supervisor TF: %s", error)

        with self._lock:
            raw_command = self._raw_command
            raw_stamp = self._raw_stamp
            path = list(self._path)
            path_stamp = self._path_stamp
            path_revision = self._path_revision
            goal_active = self._goal_active
            costmap = self._costmap
            costmap_stamp = self._costmap_stamp
            costmap_frame = self._costmap_frame

        raw_fresh = (
            raw_stamp is not None
            and (now - raw_stamp).to_sec() <= self._raw_timeout
        )
        path_fresh = path_stamp is not None and (
            self._path_timeout <= 0.0
            or (now - path_stamp).to_sec() <= self._path_timeout
        )
        observation = None
        planned_corner = None
        corner_count = self._corner_queue.remaining_count()
        if tf_valid and path_fresh:
            if (
                path_revision != self._planned_revision
                and self._supervisor.state in ("IDLE", "FOLLOWING")
            ):
                self._planned_path = path
                self._planned_distances = self._path_distances(path)
                new_corners = extract_corner_plan(
                    path,
                    simplify_tolerance=self._simplify_tolerance,
                    min_corner_angle=self._min_corner_angle,
                    min_segment_length=self._min_segment_length,
                    max_fit_residual=self._max_fit_residual,
                    same_turn_merge_distance=self._same_turn_merge_distance,
                    resample_spacing=self._path_resample_spacing,
                )
                self._corner_queue.replace(new_corners)
                self._planned_revision = path_revision
            nearest_index = self._nearest_path_index(
                self._planned_path, translation.x, translation.y
            )
            progress = (
                self._planned_distances[nearest_index]
                if nearest_index is not None
                else 0.0
            )
            while (
                self._corner_queue.current() is not None
                and self._corner_queue.current().path_distance
                < progress - self._corner_release_margin
            ):
                self._corner_queue.skip_current()
            corner_count = self._corner_queue.remaining_count()
            planned_corner = self._corner_queue.current()
            if planned_corner is not None:
                observation = CornerObservation(
                    distance=max(
                        0.0, planned_corner.path_distance - progress
                    ),
                    turn_angle=planned_corner.turn_angle,
                    exit_heading=planned_corner.exit_heading,
                    point=planned_corner.point,
                )

        costmap_fresh = (
            costmap is not None
            and costmap_stamp is not None
            and (now - costmap_stamp).to_sec() <= self._costmap_timeout
        )
        sweep_safe = True
        sweep_result = None
        sweep_target = self._supervisor.target_heading
        if sweep_target is None and observation is not None:
            sweep_target = observation.exit_heading
        if sweep_target is not None:
            sweep_safe = False
            if costmap_fresh and costmap_frame:
                try:
                    costmap_transform = self._tf_buffer.lookup_transform(
                        costmap_frame,
                        "base_link",
                        rospy.Time(0),
                        rospy.Duration(0.05),
                    )
                    local_translation = (
                        costmap_transform.transform.translation
                    )
                    local_yaw = quaternion_yaw(
                        costmap_transform.transform.rotation
                    )
                    local_target = local_yaw + (
                        (sweep_target - yaw + math.pi)
                        % (2.0 * math.pi)
                        - math.pi
                    )
                    sweep_result = check_rotation_sweep(
                        costmap,
                        pose=(
                            local_translation.x,
                            local_translation.y,
                            local_yaw,
                        ),
                        target_yaw=local_target,
                        footprint=self._footprint,
                        angle_step=self._sweep_angle_step,
                        lethal_threshold=self._lethal_cost_threshold,
                    )
                    sweep_safe = sweep_result.safe
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ) as error:
                    rospy.logwarn_throttle(
                        2.0, "corner supervisor costmap TF: %s", error
                    )

        previous_state = self._supervisor.state
        result = self._supervisor.update(
            now=now.to_sec(),
            goal_active=goal_active,
            raw_fresh=raw_fresh,
            tf_valid=tf_valid,
            yaw=yaw,
            corner=observation,
            raw_command=raw_command,
            corner_confident=(
                planned_corner.confident if planned_corner else True
            ),
            sweep_safe=sweep_safe,
            corner_id=(
                self._corner_queue.current_identity()
            ),
        )
        if (
            previous_state in ("TURNING", "EXIT_ALIGN")
            and result.state == "FOLLOWING"
            and self._corner_queue.current() is not None
        ):
            self._corner_queue.complete_current()
        self._publish_command(result.command)
        if result.state != self._last_state:
            rospy.loginfo("corner supervisor state: %s", result.state)
            self._state_publisher.publish(String(data=result.state))
            self._last_state = result.state
        self._diagnostic_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "state": result.state,
                        "corner_distance": (
                            observation.distance if observation else None
                        ),
                        "corner_angle_deg": (
                            math.degrees(observation.turn_angle)
                            if observation
                            else None
                        ),
                        "corner_count": corner_count,
                        "corner_confidence": (
                            planned_corner.confident
                            if planned_corner
                            else None
                        ),
                        "entry_heading_deg": (
                            math.degrees(planned_corner.entry_heading)
                            if planned_corner
                            else None
                        ),
                        "exit_heading_deg": (
                            math.degrees(planned_corner.exit_heading)
                            if planned_corner
                            else None
                        ),
                        "entry_fit_length": (
                            planned_corner.entry_fit.length
                            if planned_corner
                            else None
                        ),
                        "exit_fit_length": (
                            planned_corner.exit_fit.length
                            if planned_corner
                            else None
                        ),
                        "entry_fit_residual": (
                            planned_corner.entry_fit.max_residual
                            if planned_corner
                            else None
                        ),
                        "exit_fit_residual": (
                            planned_corner.exit_fit.max_residual
                            if planned_corner
                            else None
                        ),
                        "costmap_fresh": costmap_fresh,
                        "sweep_safe": sweep_safe,
                        "blocking_cell": (
                            sweep_result.blocking_cell
                            if sweep_result
                            else None
                        ),
                        "raw_fresh": raw_fresh,
                        "path_fresh": path_fresh,
                        "tf_valid": tf_valid,
                        "message": result.message,
                    },
                    ensure_ascii=False,
                )
            )
        )

    def _publish_command(self, command):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        self._command_publisher.publish(message)

    def stop(self):
        if hasattr(self, "_command_publisher"):
            self._publish_command((0.0, 0.0, 0.0))


def main():
    rospy.init_node("ucar_corner_supervisor")
    try:
        CornerSupervisorNode()
    except Exception as error:
        rospy.logfatal("corner supervisor failed to start: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
