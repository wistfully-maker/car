#!/usr/bin/env python3
"""Navigation-stack handoff supervisor: the only owner of the two navigation
process groups (first-phase fast-nav stack and stop AMCL/TEB/move_base stack).

The ROS-independent state machine lives in task_orchestrator.handoff; this node
translates ROS events into machine observations and machine effects into real
actions. It never discovers kill targets by name pattern: the only process
groups it terminates are the two explicit roslaunch parents it started and
holds. Public hardware (base, lidar, camera, speech) is never owned here.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time

import actionlib
import rosnode
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from task_orchestrator.handoff import (
    ActionGoalCancelled,
    ActionGoalCancelFailed,
    LegacyStackExited,
    LegacyStackStillRunning,
    NavigationHandoff,
    OdomNotStopped,
    OdomStale,
    OdomStopped,
    StopStackNotReady,
    StopStackReady,
    StopStackStarted,
    StopStackStartFailed,
)
from task_orchestrator.protocol import ProtocolError, load_object, require_text


_DONE_GOAL_STATES = frozenset(
    (
        actionlib.GoalStatus.SUCCEEDED,
        actionlib.GoalStatus.PREEMPTED,
        actionlib.GoalStatus.ABORTED,
        actionlib.GoalStatus.RECALLED,
        actionlib.GoalStatus.LOST,
    )
)


def _zero_twist():
    return Twist()


class ManagedProcessGroup:
    """One roslaunch parent process started by this supervisor, killed as a group."""

    def __init__(self, command, log=None):
        self._command = list(command)
        self._log = log
        self._proc = subprocess.Popen(
            self._command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def is_alive(self):
        return self._proc.poll() is None

    def terminate(self, timeout):
        if not self.is_alive():
            return True
        deadline = time.monotonic() + timeout
        self._signal_group(signal.SIGTERM)
        while time.monotonic() < deadline and self.is_alive():
            time.sleep(0.05)
        if self.is_alive():
            self._signal_group(signal.SIGKILL)
            try:
                self._proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                return False
        return not self.is_alive()

    def _signal_group(self, sig):
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except (OSError, ProcessLookupError):
            try:
                self._proc.send_signal(sig)
            except Exception:
                pass


class RosLaunchProcessRunner:
    """Starts one roslaunch parent per launch file; the supervisor holds the handle."""

    def __init__(self, command_prefix=None):
        self._prefix = list(command_prefix) if command_prefix else ["roslaunch"]

    def start(self, launch_path):
        return ManagedProcessGroup([*self._prefix, launch_path])


class StopStackReadinessProbe:
    """Readiness for the stop navigation stack: map, scan, camera, TF, AMCL,
    OCR and the move_base action server."""

    def __init__(
        self,
        max_age,
        action_wait_timeout,
        tf_timeout,
        frames,
        amcl_node,
        ocr_node,
        image_topic,
    ):
        self._max_age = max_age
        self._action_wait_timeout = action_wait_timeout
        self._tf_timeout = tf_timeout
        self._frames = dict(frames)
        self._amcl_node = amcl_node
        self._ocr_node = ocr_node
        self._stamps = {"scan": None, "camera": None}
        self._map_received = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber(image_topic, Image, self._on_camera, queue_size=1)

    def _on_map(self, _message):
        self._map_received = True

    def _on_scan(self, _message):
        self._stamps["scan"] = rospy.get_time()

    def _on_camera(self, _message):
        self._stamps["camera"] = rospy.get_time()

    def _can_transform(self, parent, child):
        try:
            return bool(
                self._tf_buffer.can_transform(
                    parent, child, rospy.Time(0), rospy.Duration(self._tf_timeout)
                )
            )
        except Exception:
            return False

    @staticmethod
    def _node_live(name):
        try:
            return bool(rosnode.rosnode_ping(name, max_count=1, verbose=False))
        except Exception:
            return False

    def _move_base_available(self):
        try:
            return bool(
                self._move_base.wait_for_server(
                    rospy.Duration(self._action_wait_timeout)
                )
            )
        except Exception:
            return False

    def ready(self):
        missing = []
        now = rospy.get_time()
        for name, stamp in (
            ("scan", self._stamps["scan"]),
            ("camera", self._stamps["camera"]),
        ):
            if stamp is None:
                missing.append("%s not received" % name)
            elif now < stamp or now - stamp > self._max_age:
                missing.append("%s stale" % name)
        if not self._map_received:
            missing.append("map not received")
        for parent, child, description in (
            (self._frames["map"], self._frames["odom"], "TF map->odom unavailable"),
            (self._frames["odom"], self._frames["base"], "TF odom->base_link unavailable"),
            (self._frames["base"], self._frames["laser"], "TF base_link->laser unavailable"),
        ):
            if not self._can_transform(parent, child):
                missing.append(description)
        if not self._node_live(self._amcl_node):
            missing.append("%s is not live" % self._amcl_node)
        if not self._node_live(self._ocr_node):
            missing.append("%s is not live" % self._ocr_node)
        if not self._move_base_available():
            missing.append("move_base action unavailable")
        return not missing


class NavigationHandoffDriver:
    """Drives the pure handoff machine and executes its effects through an
    injectable actions object. Bounded operations that need real time gaps
    (action cancel, process exit verification, readiness) are paced by poll()
    and result callbacks, so retries never burn synchronously."""

    def __init__(self, machine, actions):
        self._machine = machine
        self._actions = actions
        self._goal = None
        self._active_probe = None

    @property
    def machine(self):
        return self._machine

    def start(self, goal):
        self._goal = goal
        effects = self._machine.start(goal["task_id"], goal["goal_id"])
        self._drive(effects)
        return effects

    def on_cancel_result(self, ok):
        event = ActionGoalCancelled() if ok else ActionGoalCancelFailed()
        self._drive(self._machine.observe(event))

    def on_odom(self, stamp, near_zero, valid=True):
        if not valid:
            event = OdomStale()
        elif near_zero:
            event = OdomStopped(stamp=stamp)
        else:
            event = OdomNotStopped()
        self._drive(self._machine.observe(event))

    def poll(self):
        if self._machine.state in (
            self._machine.IDLE,
            self._machine.READY,
            self._machine.FAILED,
        ):
            return
        effects = list(self._machine.tick())
        if self._active_probe == "legacy":
            absent = self._actions.verify_legacy_absent()
            if absent:
                self._active_probe = None
                effects += self._machine.observe(LegacyStackExited())
            else:
                effects += self._machine.observe(LegacyStackStillRunning())
        elif self._active_probe == "readiness":
            ready = self._actions.wait_stop_ready()
            if ready:
                self._active_probe = None
                effects += self._machine.observe(StopStackReady())
            else:
                effects += self._machine.observe(StopStackNotReady())
        self._drive(effects)

    def _drive(self, effects):
        for action, payload in effects:
            handler = getattr(self, "_effect_" + action, None)
            if handler is None:
                continue
            handler(payload)

    def _effect_publish_zero(self, _payload):
        self._actions.publish_zero()

    def _effect_cancel_goals(self, _payload):
        self._actions.cancel_goals()

    def _effect_wait_stopped(self, _payload):
        self._actions.wait_stopped()

    def _effect_stop_owned_legacy(self, _payload):
        if self._actions.stop_owned_legacy():
            self._active_probe = "legacy"
        else:
            self._drive(self._machine.observe(LegacyStackStillRunning()))

    def _effect_verify_legacy_absent(self, _payload):
        self._active_probe = "legacy"

    def _effect_start_owned_stop(self, _payload):
        ok = self._actions.start_owned_stop()
        event = StopStackStarted() if ok else StopStackStartFailed()
        self._drive(self._machine.observe(event))

    def _effect_wait_stop_ready(self, _payload):
        self._active_probe = "readiness"

    def _effect_release_task(self, payload):
        # 先授权 stop 来源运动，再放行任务；模式切换本身先发零速度。
        self._actions.publish_motion_mode("STOP_NAVIGATION")
        self._actions.release_task(payload, self._goal)

    def _effect_publish_diagnostic(self, payload):
        self._actions.publish_diagnostic(payload)

    def _effect_handoff_failed(self, payload):
        self._actions.handoff_failed(payload)


class _RosHandoffActions:
    """Real ROS implementation of the driver actions surface."""

    def __init__(
        self,
        config,
        supervisor_config,
        status_pub,
        release_pub,
        zero_pubs,
        initial_pose_pub,
        mode_pub,
    ):
        self._config = config
        self._supervisor = supervisor_config
        self._status_pub = status_pub
        self._release_pub = release_pub
        self._zero_pubs = zero_pubs
        self._initial_pose_pub = initial_pose_pub
        self._mode_pub = mode_pub
        self._driver = None
        self._cancel_client = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction
        )
        self._master_nodes = rosnode.get_node_names
        self._readiness = StopStackReadinessProbe(
            max_age=supervisor_config["readiness_max_age"],
            action_wait_timeout=supervisor_config["readiness_action_wait_timeout"],
            tf_timeout=supervisor_config["readiness_tf_timeout"],
            frames={
                "map": supervisor_config["initial_pose_frame"],
                "odom": supervisor_config["odom_frame"],
                "base": supervisor_config["base_frame"],
                "laser": supervisor_config["laser_frame"],
            },
            amcl_node=supervisor_config["amcl_node"],
            ocr_node=supervisor_config["ocr_node"],
            image_topic=supervisor_config["image_topic"],
        )
        self._runner = RosLaunchProcessRunner()
        self._legacy_group = None
        self._stop_group = None
        if supervisor_config["legacy_nav_launch"]:
            try:
                self._legacy_group = self._runner.start(
                    supervisor_config["legacy_nav_launch"]
                )
            except Exception as exc:
                rospy.logerr(
                    "failed to start legacy navigation stack %s: %s",
                    supervisor_config["legacy_nav_launch"],
                    exc,
                )

    def bind(self, driver):
        self._driver = driver

    def publish_zero(self):
        zero = _zero_twist()
        for publisher in self._zero_pubs:
            try:
                publisher.publish(zero)
            except Exception as exc:
                rospy.logerr("zero velocity publish failed: %s", exc)

    def wait_stopped(self):
        # odom classification continues in the node; nothing else to do here.
        pass

    def cancel_goals(self):
        threading.Thread(target=self._cancel_worker, daemon=True).start()

    def _cancel_worker(self):
        try:
            client = self._cancel_client
            if not client.wait_for_server(rospy.Duration(1.0)):
                ok = False
            else:
                client.cancel_all_goals()
                deadline = rospy.get_time() + self._config["cancel_timeout"]
                while rospy.get_time() < deadline:
                    if client.get_state() in _DONE_GOAL_STATES:
                        break
                    rospy.sleep(0.05)
                ok = client.get_state() in _DONE_GOAL_STATES
        except Exception as exc:
            rospy.logerr("legacy action cancel failed: %s", exc)
            ok = False
        if self._driver is not None:
            self._driver.on_cancel_result(ok)

    def stop_owned_legacy(self):
        group = self._legacy_group
        if group is None:
            return True
        return group.terminate(self._config["legacy_exit_timeout"])

    def verify_legacy_absent(self):
        if self._legacy_group is not None and self._legacy_group.is_alive():
            return False
        try:
            live = set(self._master_nodes())
        except Exception:
            return False
        return not (set(self._supervisor["legacy_nodes"]) & live)

    def start_owned_stop(self):
        try:
            self._stop_group = self._runner.start(
                self._supervisor["stop_integration_launch"]
            )
        except Exception as exc:
            rospy.logerr("failed to start stop navigation stack: %s", exc)
            return False
        self._publish_initial_pose()
        return True

    def wait_stop_ready(self):
        try:
            return bool(self._readiness.ready())
        except Exception as exc:
            rospy.logerr("stop readiness probe failed: %s", exc)
            return False

    def release_task(self, payload, goal):
        message = {
            "protocol_version": 1,
            "task_id": payload["task_id"],
            "goal_id": payload["goal_id"],
            "target_workshop": goal["target_workshop"],
            "selected_item": goal["selected_item"],
        }
        try:
            self._release_pub.publish(
                String(data=json.dumps(message, ensure_ascii=False))
            )
        except Exception as exc:
            rospy.logerr("stop mission release failed: %s", exc)

    def publish_diagnostic(self, payload):
        message = {"protocol_version": 1}
        message.update(payload)
        try:
            self._status_pub.publish(
                String(data=json.dumps(message, ensure_ascii=False))
            )
        except Exception as exc:
            rospy.logerr("handoff diagnostic publish failed: %s", exc)

    def handoff_failed(self, payload):
        self.publish_diagnostic(payload)
        rospy.logerr(
            "navigation handoff failed: %s",
            json.dumps(payload, ensure_ascii=False),
        )

    def publish_motion_mode(self, mode):
        try:
            self._mode_pub.publish(String(data=mode))
        except Exception as exc:
            rospy.logerr("motion mode publish failed: %s", exc)

    def shutdown(self):
        for group in (self._legacy_group, self._stop_group):
            if group is not None:
                group.terminate(1.0)
        self.publish_zero()

    def _publish_initial_pose(self):
        s = self._supervisor
        message = PoseWithCovarianceStamped()
        message.header.frame_id = s["initial_pose_frame"]
        message.pose.pose.position.x = s["initial_pose_x"]
        message.pose.pose.position.y = s["initial_pose_y"]
        yaw = s["initial_pose_yaw"]
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = s["initial_pose_covariance"]
        message.pose.covariance[7] = s["initial_pose_covariance"]
        message.pose.covariance[35] = s["initial_pose_covariance"]
        try:
            self._initial_pose_pub.publish(message)
        except Exception as exc:
            rospy.logerr("initial pose publish failed: %s", exc)


class NavigationHandoffSupervisorNode:
    """ROS wiring for the handoff driver; injectable actions for tests."""

    def __init__(self, actions=None):
        try:
            self._config = self._read_machine_config()
            self._supervisor_config = self._read_supervisor_config()
            self._machine = NavigationHandoff(self._config, rospy.get_time)
        except ValueError as exc:
            rospy.logerr("invalid navigation_handoff configuration: %s", exc)
            raise
        self._lock = threading.RLock()
        self._status_pub = rospy.Publisher(
            "/task/navigation_handoff_status", String, queue_size=10
        )
        self._release_pub = rospy.Publisher(
            "/task/stop_mission_goal", String, queue_size=10
        )
        self._zero_pubs = [
            rospy.Publisher("/cmd_vel/navigation", Twist, queue_size=1),
            rospy.Publisher("/cmd_vel/stop_navigation", Twist, queue_size=1),
        ]
        self._initial_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        self._mode_pub = rospy.Publisher(
            "/task/motion_mode", String, queue_size=1, latch=True
        )
        if actions is None:
            actions = _RosHandoffActions(
                self._config,
                self._supervisor_config,
                self._status_pub,
                self._release_pub,
                self._zero_pubs,
                self._initial_pose_pub,
                self._mode_pub,
            )
        self._driver = NavigationHandoffDriver(self._machine, actions)
        self._actions = actions
        self._actions.bind(self._driver)
        rospy.Subscriber(
            "/task/delivery_navigation_goal",
            String,
            self._on_goal,
            queue_size=1,
        )
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration(self._config["readiness_poll_period"]),
            self._on_timer,
        )
        rospy.on_shutdown(self._on_shutdown)

    def _read_machine_config(self):
        param = lambda key, default: rospy.get_param(
            "~navigation_handoff/" + key, default
        )
        return {
            "cancel_retries": int(param("cancel_retries", 3)),
            "cancel_timeout": float(param("cancel_timeout", 3.0)),
            "stop_stable_duration": float(param("stop_stable_duration", 0.75)),
            "linear_stop_threshold": float(param("linear_stop_threshold", 0.02)),
            "angular_stop_threshold": float(param("angular_stop_threshold", 0.05)),
            "odom_max_age": float(param("odom_max_age", 0.5)),
            "legacy_exit_retries": int(param("legacy_exit_retries", 5)),
            "legacy_exit_timeout": float(param("legacy_exit_timeout", 10.0)),
            "readiness_retries": int(param("readiness_retries", 30)),
            "readiness_poll_period": float(param("readiness_poll_period", 1.0)),
            "total_timeout": float(param("total_timeout", 90.0)),
        }

    def _read_supervisor_config(self):
        param = lambda key, default: rospy.get_param(
            "~navigation_handoff_supervisor/" + key, default
        )
        return {
            "legacy_nav_launch": param("legacy_nav_launch", ""),
            "stop_integration_launch": param("stop_integration_launch", ""),
            "legacy_nodes": list(
                param("legacy_nodes", ["/lidar_loc", "/move_base", "/map_server"])
            ),
            "initial_pose_frame": param("initial_pose_frame", "map"),
            "initial_pose_x": float(param("initial_pose_x", 0.0)),
            "initial_pose_y": float(param("initial_pose_y", 0.0)),
            "initial_pose_yaw": float(param("initial_pose_yaw", 0.0)),
            "initial_pose_covariance": float(param("initial_pose_covariance", 0.1)),
            "readiness_max_age": float(param("readiness_max_age", 3.0)),
            "readiness_action_wait_timeout": float(
                param("readiness_action_wait_timeout", 0.05)
            ),
            "readiness_tf_timeout": float(param("readiness_tf_timeout", 0.05)),
            "amcl_node": param("amcl_node", "/amcl"),
            "ocr_node": param("ocr_node", "/ocr_native_node"),
            "image_topic": param("image_topic", "/usb_cam/image_raw"),
            "odom_frame": param("odom_frame", "odom"),
            "base_frame": param("base_frame", "base_link"),
            "laser_frame": param("laser_frame", "laser_frame"),
        }

    def _on_goal(self, message):
        try:
            goal = load_object(message.data)
            goal["task_id"] = require_text(goal.get("task_id"), "task_id")
            goal["goal_id"] = require_text(goal.get("goal_id"), "goal_id")
            goal["target_workshop"] = require_text(
                goal.get("target_workshop"), "target_workshop"
            )
            goal["selected_item"] = require_text(
                goal.get("selected_item"), "selected_item"
            )
        except (ProtocolError, TypeError, ValueError) as exc:
            rospy.logwarn("ignored invalid delivery navigation goal: %s", exc)
            return
        with self._lock:
            if self._driver.start(goal):
                rospy.loginfo(
                    "navigation handoff started for %s/%s",
                    goal["task_id"],
                    goal["goal_id"],
                )

    def _on_odom(self, message):
        raw_stamp = message.header.stamp
        try:
            stamp = (
                float(raw_stamp.to_sec())
                if hasattr(raw_stamp, "to_sec")
                else float(raw_stamp)
            )
        except (TypeError, ValueError, AttributeError):
            stamp = float("nan")
        now = rospy.get_time()
        age = now - stamp
        valid = math.isfinite(stamp) and 0.0 <= age <= self._config["odom_max_age"]
        twist = message.twist.twist
        near_zero = (
            abs(twist.linear.x) <= self._config["linear_stop_threshold"]
            and abs(twist.linear.y) <= self._config["linear_stop_threshold"]
            and abs(twist.angular.z) <= self._config["angular_stop_threshold"]
        )
        with self._lock:
            self._driver.on_odom(stamp, near_zero=near_zero, valid=valid)

    def _on_timer(self, _event):
        with self._lock:
            self._driver.poll()

    def _on_shutdown(self):
        with self._lock:
            self._actions.shutdown()
        rospy.loginfo(
            "navigation handoff supervisor stopped its owned navigation stacks"
        )


def main():
    rospy.init_node("navigation_handoff_supervisor")
    NavigationHandoffSupervisorNode()
    rospy.loginfo("navigation_handoff_supervisor node started")
    rospy.spin()


if __name__ == "__main__":
    main()
