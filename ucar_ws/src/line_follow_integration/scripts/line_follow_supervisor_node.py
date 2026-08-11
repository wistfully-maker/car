#!/usr/bin/env python3
"""Supervise Phase 3 direction selection, line following and process ownership.

流程：
  /task/line_follow/start -> 校验 YOLO 模型 -> 等待原始图像（5s）
  -> 启动本次 YOLO 子进程 -> 等待/锁定方向 -> 结束 YOLO 子进程
  -> 等待派生图像 -> 启动路线子进程 -> 门控 /cmd_vel/line_follow
  -> 新鲜停车标记 = 成功；子进程提前退出或 120s 超时 = 失败。

只终止自己通过 subprocess.Popen 创建并记录的 PID。失败/取消/关闭时先发布
零速度，再结束所拥有的子进程，最后发布一次关联失败。
"""

import hashlib
import json
import subprocess
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

from line_follow_integration.protocol import (
    ProtocolError,
    build_line_status,
    parse_identity_json,
)
from line_follow_integration.runtime import (
    ImageHealthGate,
    LineFollowSession,
    ROUTE_SCRIPTS,
)


def yolo_command():
    return [
        "rosrun", "line_follow_integration", "yolo_server.py",
        "__name:=phase3_yolo_server",
    ]


def route_command(direction):
    return [
        "rosrun", "line_follow_integration", ROUTE_SCRIPTS[direction],
        "__name:=phase3_line_follower",
        "/usb_cam/image_raw:=/line_follow/image_raw",
        "/cmd_vel:=/line_follow/cmd_vel_candidate",
    ]


class LineFollowSupervisor:
    def __init__(self):
        timeouts = rospy.get_param("~timeouts", {})
        runtime = rospy.get_param("~runtime", {})
        camera = rospy.get_param("~camera", {})
        self._image_ready = float(timeouts.get("image_ready", 5.0))
        self._image_max_age = float(timeouts.get("image_max_age", 0.5))
        self._image_recovery_grace = float(
            timeouts.get("image_recovery_grace", 3.0)
        )
        self._direction_timeout = float(timeouts.get("direction", 30.0))
        self._follow_timeout = float(timeouts.get("line_follow", 120.0))
        self._line_topic = camera.get("line_topic", "/line_follow/image_raw")
        self._yolo_result_file = runtime.get(
            "yolo_result_file", "/tmp/yolo_result.txt"
        )
        self._stop_done_file = runtime.get(
            "stop_done_file", "/tmp/stop_done.txt"
        )
        self._yolo_model = runtime.get(
            "yolo_model", "/home/ucar/ucar_ws/src/yolo_turn/best.pt"
        )
        self._yolo_model_sha256 = runtime.get(
            "yolo_model_sha256",
            "cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd",
        )
        if min(
            self._image_ready, self._image_max_age,
            self._image_recovery_grace, self._direction_timeout,
            self._follow_timeout,
        ) <= 0:
            raise ValueError("all line-follow timeouts must be positive")

        self._lock = threading.RLock()
        self._session = LineFollowSession(
            self._yolo_result_file, self._stop_done_file, rospy.get_time,
            self._direction_timeout, self._follow_timeout,
        )
        self._gate = ImageHealthGate(
            rospy.get_time, self._image_max_age, self._image_recovery_grace
        )
        self._children = {}
        self._phase = "idle"
        self._raw_seen_at = None

        self._velocity_publisher = rospy.Publisher(
            "/cmd_vel/line_follow", Twist, queue_size=1
        )
        self._status_publisher = rospy.Publisher(
            "/task/line_follow/status", String, queue_size=10
        )
        rospy.Subscriber(
            "/task/line_follow/start", String, self._on_start, queue_size=1
        )
        rospy.Subscriber(
            "/task/cancel", String, self._on_cancel, queue_size=1
        )
        rospy.Subscriber(
            "/usb_cam/image_raw", Image, self._on_raw_image, queue_size=1
        )
        rospy.Subscriber(
            self._line_topic, Image, self._on_derived_image, queue_size=1
        )
        rospy.Subscriber(
            "/line_follow/cmd_vel_candidate", Twist, self._on_candidate,
            queue_size=1,
        )
        self._timer = rospy.Timer(rospy.Duration(0.1), self._poll)
        rospy.on_shutdown(self._on_shutdown)

    # ---------- 发布与子进程 ----------

    def _publish_status(self, status, direction=None, reason=""):
        with self._lock:
            if self._session.task_id is None:
                return
            payload = build_line_status(
                self._session.task_id, self._session.goal_id,
                status, direction=direction, reason=reason,
            )
        self._status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        rospy.loginfo(
            "line status %s task=%s goal=%s direction=%s reason=%s",
            status, self._session.task_id, self._session.goal_id,
            direction or "-", reason or "-",
        )

    def _publish_zero(self):
        self._velocity_publisher.publish(Twist())

    def _start_child(self, command):
        with self._lock:
            process = subprocess.Popen(command)
            self._children[process.pid] = process
            rospy.loginfo("started owned child pid=%s command=%s",
                          process.pid, command)
            return process

    def _terminate_children(self):
        with self._lock:
            processes = list(self._children.values())
            self._children.clear()
        for process in processes:
            try:
                process.terminate()
            except OSError:
                pass
        deadline = rospy.get_time() + 1.0
        for process in processes:
            remaining = deadline - rospy.get_time()
            if remaining <= 0:
                break
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                pass
            except OSError:
                pass
        for process in processes:
            if process.poll() is None:
                try:
                    process.kill()
                except OSError:
                    pass
                rospy.logwarn("killed still-owned child pid=%s", process.pid)

    def _fail(self, reason):
        self._publish_zero()
        self._terminate_children()
        self._publish_status("failure", reason=reason)
        with self._lock:
            self._session.fail(reason)
            self._phase = "idle"

    # ---------- 消息回调 ----------

    def _on_start(self, message):
        try:
            parsed = parse_identity_json(message.data)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line follow start: %s", exc)
            return
        with self._lock:
            if self._phase != "idle":
                rospy.logwarn(
                    "ignored line follow start while phase is %s",
                    self._phase,
                )
                return
            self._session.start(parsed["task_id"], parsed["goal_id"])
            self._raw_seen_at = None
        if not self._verify_model():
            self._fail("yolo model missing or sha256 mismatch")
            return
        self._publish_status("waiting_signal")
        with self._lock:
            self._phase = "starting_yolo"

    def _verify_model(self):
        try:
            with open(self._yolo_model, "rb") as handle:
                digest = hashlib.sha256(handle.read()).hexdigest()
        except OSError as exc:
            rospy.logerr("yolo model unreadable: %s", exc)
            return False
        if digest != self._yolo_model_sha256:
            rospy.logerr("yolo model sha256 mismatch: %s", digest)
            return False
        return True

    def _on_cancel(self, message):
        try:
            parsed = parse_identity_json(message.data)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line follow cancel: %s", exc)
            return
        with self._lock:
            if self._session.task_id is None:
                return
            if parsed["task_id"] != self._session.task_id:
                return
            if self._phase == "idle":
                return
        self._fail("cancelled")

    def _on_raw_image(self, _message):
        with self._lock:
            if self._phase == "idle":
                return
            if self._raw_seen_at is None:
                self._raw_seen_at = rospy.get_time()

    def _on_derived_image(self, _message):
        self._gate.observe_frame(rospy.get_time())

    def _on_candidate(self, message):
        with self._lock:
            if self._phase != "following":
                return
            if not self._gate.allows_motion():
                return
        try:
            self._velocity_publisher.publish(message)
        except Exception as exc:
            rospy.logerr("line follow candidate publish failed: %s", exc)

    # ---------- 轮询状态机 ----------

    def _poll(self, _event):
        with self._lock:
            phase = self._phase
            if phase == "starting_yolo":
                self._poll_starting_yolo()
            elif phase == "waiting_direction":
                self._poll_waiting_direction()
            elif phase == "waiting_derived":
                self._poll_waiting_derived()
            elif phase == "following":
                self._poll_following()
            elif phase == "terminal":
                self._phase = "idle"

    def _poll_starting_yolo(self):
        if self._raw_seen_at is not None:
            self._phase = "waiting_direction"
            self._start_child(yolo_command())
            return
        if rospy.get_time() - self._session.activation_time >= self._image_ready:
            self._publish_zero()
            self._terminate_children()
            self._publish_status(
                "failure",
                reason="raw image not ready within %s seconds"
                       % self._image_ready,
            )
            self._session.fail("raw image not ready")
            self._phase = "idle"

    def _poll_waiting_direction(self):
        status, direction = self._session.poll_direction()
        if status == "waiting_signal":
            return
        self._terminate_children()
        self._publish_status("direction_selected", direction=direction)
        if self._gate.last_frame is None:
            self._phase = "waiting_derived"
            return
        self._start_route(direction)

    def _start_route(self, direction):
        self._start_child(route_command(direction))
        self._publish_status("following", direction=direction)
        self._phase = "following"

    def _poll_waiting_derived(self):
        gate_status = self._gate.poll()
        if gate_status == "failure":
            self._fail("derived line image never became ready")
            return
        if self._gate.last_frame is not None:
            self._start_route(self._session.direction)

    def _poll_following(self):
        gate_status = self._gate.poll()
        if gate_status == "failure":
            self._fail("derived line image stale beyond recovery grace")
            return
        if gate_status == "stop":
            self._publish_zero()
        direction = self._session.direction
        child_running = bool(self._children)
        status, terminal = self._session.poll_follow(child_running)
        if status == "following":
            return
        self._publish_zero()
        self._terminate_children()
        if status == "success":
            self._publish_status("success", direction=direction)
        else:
            self._publish_status("failure", reason=terminal)
        self._phase = "idle"

    # ---------- 关闭 ----------

    def _on_shutdown(self):
        self._publish_zero()
        with self._lock:
            active = self._session.task_id is not None and self._phase != "idle"
        self._terminate_children()
        if active:
            self._publish_status("failure", reason="ROS shutdown")


def main():
    rospy.init_node("line_follow_supervisor")
    LineFollowSupervisor()
    rospy.loginfo("line_follow_supervisor node started")
    rospy.spin()


if __name__ == "__main__":
    main()
