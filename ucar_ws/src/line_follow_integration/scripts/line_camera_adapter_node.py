#!/usr/bin/env python3
"""Gate the shared camera into a 640x480 15-FPS line-follow image stream.

订阅原始 1020x720 图像与 /task/line_follow/start 启动消息，只在一个关联的
巡线 goal 激活期间发布派生图像；保留输入 header；用消息时间戳限流到
配置 FPS；收到关联 success/failure 状态后停发。不拥有、不重配物理相机。
"""

import threading

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from line_follow_integration.camera_transform import transform_line_frame
from line_follow_integration.protocol import (
    ProtocolError,
    parse_identity_json,
)


class LineCameraAdapter:
    def __init__(self):
        camera = rospy.get_param("~camera", {})
        self.input_topic = camera.get("input_topic", "/usb_cam/image_raw")
        self.line_topic = camera.get("line_topic", "/line_follow/image_raw")
        self.output_width = int(camera.get("output_width", 640))
        self.output_height = int(camera.get("output_height", 480))
        self.output_fps = float(camera.get("output_fps", 15.0))
        self.crop_mode = camera.get("crop_mode", "center_4_3")
        if (
            self.output_width <= 0
            or self.output_height <= 0
            or self.output_fps <= 0
        ):
            raise ValueError("camera output dimensions and fps must be positive")

        self._lock = threading.RLock()
        self._active_identity = None
        self._last_publish_stamp = None
        self._bridge = CvBridge()
        self._publisher = rospy.Publisher(
            self.line_topic, Image, queue_size=1
        )
        rospy.Subscriber(
            self.input_topic, Image, self._on_image, queue_size=1
        )
        rospy.Subscriber(
            "/task/line_follow/start", String, self._on_start, queue_size=1
        )
        rospy.Subscriber(
            "/task/line_follow/status", String, self._on_status, queue_size=1
        )
        rospy.loginfo("line_camera_adapter started on %s -> %s",
                      self.input_topic, self.line_topic)

    def _on_start(self, message):
        try:
            parsed = parse_identity_json(message.data)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line follow start: %s", exc)
            return
        identity = {
            "task_id": parsed["task_id"],
            "goal_id": parsed["goal_id"],
        }
        with self._lock:
            if (
                self._active_identity is not None
                and self._active_identity != identity
            ):
                rospy.logwarn(
                    "ignored line follow start for a different active goal"
                )
                return
            self._active_identity = identity
            self._last_publish_stamp = None

    def _on_status(self, message):
        try:
            parsed = parse_identity_json(message.data)
        except ProtocolError as exc:
            rospy.logwarn("ignored invalid line follow status: %s", exc)
            return
        if parsed.get("status") not in ("success", "failure"):
            return
        identity = {
            "task_id": parsed["task_id"],
            "goal_id": parsed["goal_id"],
        }
        with self._lock:
            if self._active_identity != identity:
                return
            rospy.loginfo("line camera adapter deactivated after %s",
                          parsed["status"])
            self._active_identity = None
            self._last_publish_stamp = None

    def _on_image(self, message):
        with self._lock:
            if self._active_identity is None:
                return
            stamp = message.header.stamp
            if stamp.to_sec() <= 0:
                stamp = rospy.Time.from_sec(rospy.get_time())
            if (
                self._last_publish_stamp is not None
                and stamp - self._last_publish_stamp
                < rospy.Duration(1.0 / self.output_fps)
            ):
                return
            try:
                frame = self._bridge.imgmsg_to_cv2(message, "bgr8")
                output = transform_line_frame(
                    frame, self.output_width, self.output_height,
                    self.crop_mode,
                )
                out_message = self._bridge.cv2_to_imgmsg(output, "bgr8")
                out_message.header = message.header
                self._publisher.publish(out_message)
                self._last_publish_stamp = stamp
            except Exception as exc:
                rospy.logwarn("line camera transform failed: %s", exc)


def main():
    rospy.init_node("line_camera_adapter")
    LineCameraAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
