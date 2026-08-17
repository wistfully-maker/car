#!/usr/bin/env python3
"""Adapt line-start navigation goals to move_base and report settled arrival.

按 fast_nav_adapter_node.py 的模式实现：generation 保护、settled-odom 检测、
300 秒超时、/task/cancel、过期回调抑制与每个身份一次终态到达。
MoveBaseGoal 直接使用传入 pose 的 frame_id/x/y/qz/qw。

精准停车：当 ~precision_parking 配置存在时启用两段式导航——先到最终点
正后方 approach_offset 的中间点，再用 dynamic_reconfigure 把 TEB 限速
并收紧容差，低速爬完最后一小段；结束后恢复原 TEB 参数。
"""

import json
import threading
import time

import actionlib
import dynamic_reconfigure.client
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from line_follow_integration.precision_parking import (
    approach_parameters,
    approach_pose,
    creep_parameters,
)
from line_follow_integration.protocol import (
    ProtocolError,
    build_arrival,
    parse_cancel,
    parse_navigation_goal,
)


class GenerationCoordinator:
    """Serialize external action-client calls while suppressing stale work."""

    def __init__(self):
        self._lock = threading.Lock()
        self._operation_lock = threading.Lock()
        self._generation = 0

    def issue(self):
        with self._lock:
            self._generation += 1
            return self._generation

    def is_current(self, generation):
        with self._lock:
            return generation == self._generation

    def run_if_current(self, generation, operation):
        with self._operation_lock:
            if not self.is_current(generation):
                return False
            operation(lambda: self.is_current(generation))
            return self.is_current(generation)


class LineNavigationSession:
    """Track one active line-navigation identity; emit one terminal arrival."""

    def __init__(self):
        self._lock = threading.RLock()
        self._active_identity = None

    @property
    def active_identity(self):
        with self._lock:
            if self._active_identity is None:
                return None
            return dict(self._active_identity)

    def accept_goal(self, task_id, goal_id):
        identity = {"task_id": task_id, "goal_id": goal_id}
        with self._lock:
            if identity == self._active_identity:
                return False
            self._active_identity = identity
            return True

    def complete(self, task_id, goal_id, succeeded, message=""):
        with self._lock:
            if self._active_identity != {
                "task_id": task_id,
                "goal_id": goal_id,
            }:
                return None
            self._active_identity = None
            return build_arrival(task_id, goal_id, succeeded, message)

    def cancel(self, task_id, reason):
        with self._lock:
            identity = self._active_identity
            if identity is None or identity["task_id"] != task_id:
                return None
            return self.complete(
                identity["task_id"], identity["goal_id"], False, reason
            )


class SettledOdomDetector:
    """Detect continuous near-zero speed over a settle window."""

    def __init__(self, linear_threshold, angular_threshold, settle_time):
        self.linear_threshold = float(linear_threshold)
        self.angular_threshold = float(angular_threshold)
        self.settle_time = float(settle_time)
        self._stopped_since = None
        self._last_timestamp = None

    def reset(self):
        self._stopped_since = None
        self._last_timestamp = None

    def update(self, linear_speed, angular_speed, timestamp):
        if self._last_timestamp is not None and timestamp < self._last_timestamp:
            self._stopped_since = None
        self._last_timestamp = timestamp
        stopped = (
            abs(linear_speed) <= self.linear_threshold
            and abs(angular_speed) <= self.angular_threshold
        )
        if not stopped:
            self._stopped_since = None
            return False
        if self._stopped_since is None:
            self._stopped_since = timestamp
            return False
        return timestamp - self._stopped_since >= self.settle_time


def _parse_cancel(raw, expected_task_id):
    message = parse_cancel(raw)
    if message["task_id"] != expected_task_id:
        raise ProtocolError("task_id mismatch")
    return message["reason"]


class LineNavigationAdapter:
    def __init__(self):
        timeouts = rospy.get_param("~timeouts", {})
        self._action_timeout = float(timeouts.get("navigation", 300.0))
        if self._action_timeout <= 0:
            raise ValueError("navigation timeout must be positive")

        self._lock = threading.RLock()
        self._coordinator = GenerationCoordinator()
        self._session = LineNavigationSession()
        self._detector = SettledOdomDetector(
            rospy.get_param(
                "~line_navigation_adapter/linear_stop_threshold", 0.03
            ),
            rospy.get_param(
                "~line_navigation_adapter/angular_stop_threshold", 0.05
            ),
            rospy.get_param(
                "~line_navigation_adapter/settle_time", 0.5
            ),
        )
        self._phase = "idle"
        self._action_identity = None
        self._action_started = None
        self._current_pose = None
        self._settling_since = None
        # 精准停车配置：缺失即单段导航（原行为）。
        precision = rospy.get_param("~precision_parking", None)
        self._approach_offset = None
        self._approach_params = None
        self._creep_params = None
        if precision is not None:
            if not isinstance(precision, dict):
                raise ValueError("precision_parking must be a mapping")
            offset = precision.get("approach_offset")
            if (
                isinstance(offset, bool)
                or not isinstance(offset, (int, float))
                or not 0.0 < offset <= 0.5
            ):
                raise ValueError(
                    "precision_parking approach_offset must be in (0, 0.5]"
                )
            self._approach_offset = float(offset)
            approach_raw = precision.get("approach_teb")
            if approach_raw is not None:
                self._approach_params = approach_parameters(approach_raw)
            self._creep_params = creep_parameters(
                precision.get("creep_teb")
            )
        self._reconfigure_namespace = rospy.get_param(
            "~line_navigation_adapter/reconfigure_namespace",
            "/move_base/TebLocalPlannerROS",
        )
        self._creep_send_delay = float(
            rospy.get_param("~line_navigation_adapter/creep_send_delay", 0.3)
        )
        self._teb_before = None
        self._publisher = rospy.Publisher(
            "/task/line_navigation_arrived", String, queue_size=10
        )
        self._stop_mode_publisher = rospy.Publisher(
            "/stop/motion_mode", String, queue_size=1, latch=False
        )
        self._client = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction
        )
        rospy.Subscriber(
            "/task/line_navigation_goal", String, self._on_goal
        )
        rospy.Subscriber("/task/cancel", String, self._on_cancel)
        rospy.Subscriber("/odom", Odometry, self._on_odom)
        self._timer = rospy.Timer(rospy.Duration(0.5), self._on_timer)
        rospy.on_shutdown(self._on_shutdown)

    def _publish(self, payload):
        if payload is not None:
            self._publisher.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )

    def _set_stop_mode(self, mode):
        try:
            self._stop_mode_publisher.publish(String(data=mode))
            return True
        except Exception as exc:
            rospy.logerr("failed to publish stop motion mode %s: %s", mode, exc)
            return False

    def _move_base_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.orientation.z = pose["qz"]
        goal.target_pose.pose.orientation.w = pose["qw"]
        return goal

    def _capture_teb(self):
        """记录恢复基准：只在第一次重配前捕获原始 TEB 参数。"""
        if self._teb_before is not None:
            return
        client = dynamic_reconfigure.client.Client(
            self._reconfigure_namespace, timeout=2.0
        )
        self._teb_before = client.get_configuration()

    def _apply_approach(self):
        """中间点段：放宽松朝向容差，防止 TEB 在中间点死磕 yaw。"""
        client = dynamic_reconfigure.client.Client(
            self._reconfigure_namespace, timeout=2.0
        )
        self._capture_teb()
        client.update_configuration(self._approach_params)

    def _apply_creep(self):
        """限速紧容差：记录当前 TEB 参数后注入 creep 档。"""
        client = dynamic_reconfigure.client.Client(
            self._reconfigure_namespace, timeout=2.0
        )
        self._capture_teb()
        client.update_configuration(self._creep_params)

    def _restore_teb(self):
        if self._teb_before is None:
            return
        try:
            client = dynamic_reconfigure.client.Client(
                self._reconfigure_namespace, timeout=2.0
            )
            client.update_configuration(self._teb_before)
        except Exception as exc:
            rospy.logwarn("failed to restore TEB parameters: %s", exc)
        self._teb_before = None

    def _on_goal(self, message):
        try:
            parsed = parse_navigation_goal(message.data)
            identity = {
                "task_id": parsed["task_id"],
                "goal_id": parsed["goal_id"],
            }
            with self._lock:
                if self._session.active_identity == identity:
                    return
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line navigation goal: %s", exc)
            return

        generation = self._coordinator.issue()

        def replace_goal(still_current):
            with self._lock:
                old_identity = self._session.active_identity
                if not self._session.accept_goal(
                    identity["task_id"], identity["goal_id"]
                ):
                    return
                self._detector.reset()
                self._current_pose = dict(parsed["pose"])
                if self._approach_offset is not None:
                    self._phase = "navigating_approach"
                    target_pose = approach_pose(
                        parsed["pose"], self._approach_offset
                    )
                    rospy.loginfo(
                        "line nav %s: two-stage start, approach=(%.3f, %.3f)",
                        identity["goal_id"], target_pose["x"], target_pose["y"],
                    )
                else:
                    self._phase = "navigating"
                    target_pose = dict(parsed["pose"])
                self._action_identity = dict(identity, generation=generation)
                self._action_started = rospy.get_time()
                self._settling_since = None
                goal = self._move_base_goal(target_pose)
            if old_identity is not None:
                self._client.cancel_goal()
            if not still_current():
                return
            if not self._set_stop_mode("NAVIGATION"):
                raise RuntimeError("failed to enable stop navigation mode")
            if self._approach_params is not None:
                try:
                    self._apply_approach()
                    rospy.loginfo(
                        "line nav %s: approach yaw relaxed", identity["goal_id"]
                    )
                except Exception as exc:
                    payload = self._fail_generation(
                        generation,
                        "failed to apply approach parameters: %s" % exc,
                    )
                    self._restore_teb()
                    self._set_stop_mode("IDLE")
                    self._publish(payload)
                    return
            self._client.send_goal(
                goal,
                done_cb=lambda status, result: self._on_done(
                    generation, identity["task_id"], identity["goal_id"],
                    status, result,
                ),
            )
            if not still_current():
                self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, replace_goal)
        except Exception as exc:
            rospy.logerr("move_base line navigation failed: %s", exc)
            self._publish(
                self._fail_generation(
                    generation, "move_base line navigation failed: %s" % exc
                )
            )

    def _fail_generation(self, generation, message):
        with self._lock:
            identity = self._action_identity
            if identity is None or identity.get("generation") != generation:
                return None
            payload = self._session.complete(
                identity["task_id"], identity["goal_id"], False, message
            )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
            self._current_pose = None
        self._set_stop_mode("IDLE")
        return payload

    def _on_done(self, generation, task_id, goal_id, status, _result):
        payload = None
        next_goal = None
        with self._lock:
            if self._action_identity != {
                "task_id": task_id,
                "goal_id": goal_id,
                "generation": generation,
            }:
                return
            if status == GoalStatus.SUCCEEDED:
                if self._phase == "navigating_approach":
                    # 中间点到达：限速紧容差后发最终点。
                    self._phase = "creeping"
                    next_goal = self._move_base_goal(self._current_pose)
                    rospy.loginfo(
                        "line nav %s: approach done, entering creep", goal_id
                    )
                else:
                    self._phase = "settling"
                    self._settling_since = rospy.get_time()
                    self._detector.reset()
                    rospy.loginfo(
                        "line nav %s: final goal done, settling", goal_id
                    )
            else:
                payload = self._session.complete(
                    task_id, goal_id, False,
                    "move_base finished with status %s" % status,
                )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
        if payload is not None:
            self._restore_teb()
            self._set_stop_mode("IDLE")
            self._publish(payload)
            return
        if next_goal is None:
            # 最终点到达（creeping）或普通单段到达：进入 settling，无后续目标。
            return
        # 不能在旧目标的 done 回调里直接 send_goal：actionlib 客户端此时还在
        # 处理 ACTIVE->DONE 转换，会触发 comm state/DONE twice 竞态并把新目标的
        # 回调搞乱。延迟到客户端完全进入 DONE 后再发（独立线程）。
        threading.Thread(
            target=self._send_creep_goal,
            args=(generation, task_id, goal_id, next_goal),
            daemon=True,
        ).start()

    def _send_creep_goal(self, generation, task_id, goal_id, goal):
        if self._creep_send_delay > 0:
            time.sleep(self._creep_send_delay)
        with self._lock:
            if self._action_identity != {
                "task_id": task_id,
                "goal_id": goal_id,
                "generation": generation,
            }:
                return
        try:
            self._apply_creep()
            rospy.loginfo(
                "line nav %s: creep applied, sending final goal", goal_id
            )
            self._client.send_goal(
                goal,
                done_cb=lambda status, result: self._on_done(
                    generation, task_id, goal_id, status, result
                ),
            )
        except Exception as exc:
            payload = self._fail_generation(
                generation, "failed to apply creep parameters: %s" % exc
            )
            self._restore_teb()
            self._set_stop_mode("IDLE")
            self._publish(payload)

    def _on_odom(self, message):
        payload = None
        with self._lock:
            if self._phase != "settling":
                return
            twist = message.twist.twist
            linear_speed = (twist.linear.x ** 2 + twist.linear.y ** 2) ** 0.5
            angular_speed = abs(twist.angular.z)
            if not self._detector.update(
                linear_speed, angular_speed, rospy.get_time()
            ):
                return
            identity = self._session.active_identity
            if identity is not None:
                payload = self._session.complete(
                    identity["task_id"], identity["goal_id"], True
                )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
            self._current_pose = None
            self._settling_since = None
        if payload is not None:
            rospy.loginfo(
                "line nav: settled, publishing arrived"
            )
        self._restore_teb()
        self._set_stop_mode("IDLE")
        self._publish(payload)

    def _settle_fallback(self):
        """settling 兜底：move_base 已 SUCCEEDED 后 5 秒仍未完成停稳检测，
        直接按到达处理，防止检测器异常卡死整条流程。"""
        payload = None
        with self._lock:
            if (
                self._phase != "settling"
                or self._settling_since is None
                or rospy.get_time() - self._settling_since < 5.0
            ):
                return
            identity = self._session.active_identity
            if identity is not None:
                payload = self._session.complete(
                    identity["task_id"], identity["goal_id"], True
                )
            self._detector.reset()
            self._phase = "idle"
            self._action_identity = None
            self._action_started = None
            self._current_pose = None
            self._settling_since = None
        if payload is not None:
            rospy.logwarn(
                "line nav: settle detector did not complete, "
                "falling back to arrived"
            )
        self._restore_teb()
        self._set_stop_mode("IDLE")
        self._publish(payload)

    def _on_cancel(self, message):
        with self._lock:
            identity = self._session.active_identity
            if identity is None:
                return
            expected_task_id = identity["task_id"]
        try:
            reason = _parse_cancel(message.data, expected_task_id)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line navigation cancel: %s", exc)
            return
        generation = self._coordinator.issue()
        result = [None]

        def cancel_current(_still_current):
            with self._lock:
                identity = self._session.active_identity
                if identity is None or identity["task_id"] != expected_task_id:
                    return
                result[0] = self._session.cancel(expected_task_id, reason)
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
            self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, cancel_current)
        except Exception as exc:
            rospy.logerr("move_base line navigation cancel failed: %s", exc)
        if result[0] is not None:
            self._restore_teb()
            self._set_stop_mode("IDLE")
        self._publish(result[0])

    def _on_timer(self, _event):
        with self._lock:
            settling = self._phase == "settling"
        if settling:
            self._settle_fallback()
        with self._lock:
            if (
                self._phase
                not in ("navigating", "navigating_approach", "creeping", "settling")
                or self._action_started is None
            ):
                return
            if rospy.get_time() - self._action_started < self._action_timeout:
                return
            timed_out_identity = self._session.active_identity
        generation = self._coordinator.issue()
        result = [None]

        def cancel_timed_out(_still_current):
            with self._lock:
                if self._session.active_identity != timed_out_identity:
                    return
                result[0] = self._session.complete(
                    timed_out_identity["task_id"],
                    timed_out_identity["goal_id"],
                    False,
                    "move_base line navigation timed out",
                )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
            self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, cancel_timed_out)
        except Exception as exc:
            rospy.logerr("move_base line navigation timeout cancel failed: %s",
                         exc)
        if result[0] is not None:
            self._restore_teb()
            self._set_stop_mode("IDLE")
        self._publish(result[0])

    def _on_shutdown(self):
        generation = self._coordinator.issue()

        def cancel_for_shutdown(_still_current):
            with self._lock:
                identity = self._session.active_identity
                should_cancel = identity is not None
                if identity is not None:
                    self._session.complete(
                        identity["task_id"], identity["goal_id"], False,
                        "ROS shutdown",
                    )
                self._detector.reset()
                self._phase = "idle"
                self._action_identity = None
                self._action_started = None
                self._current_pose = None
            if should_cancel:
                self._client.cancel_goal()

        try:
            self._coordinator.run_if_current(generation, cancel_for_shutdown)
        except Exception as exc:
            rospy.logerr("move_base line navigation shutdown cancel failed: %s",
                         exc)
        self._restore_teb()
        self._set_stop_mode("IDLE")


def main():
    rospy.init_node("line_navigation_adapter")
    LineNavigationAdapter()
    rospy.loginfo("line_navigation_adapter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
