#!/usr/bin/env python3
"""Gate the shared camera into a 640x480 15-FPS line-follow image stream.

The vehicle camera already publishes the required 640x480 image. While one
line-follow goal is active, forward that ROS Image unchanged at the configured
rate. Do not own, reconfigure, crop, resize, or re-encode the shared camera.
"""

import threading

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

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
        if (
            self.output_width <= 0
            or self.output_height <= 0
            or self.output_fps <= 0
        ):
            raise ValueError("camera output dimensions and fps must be positive")

        self._lock = threading.RLock()
        self._active_identity = None
        self._last_publish_stamp = None
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
            if (
                message.width != self.output_width
                or message.height != self.output_height
            ):
                rospy.logwarn_throttle(
                    5.0,
                    "line camera input is %sx%s, expected %sx%s; "
                    "refusing to alter shared-camera geometry",
                    message.width, message.height,
                    self.output_width, self.output_height,
                )
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
            self._publisher.publish(message)
            self._last_publish_stamp = stamp


def main():
    rospy.init_node("line_camera_adapter")
    LineCameraAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
