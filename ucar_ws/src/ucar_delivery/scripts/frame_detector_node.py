#!/usr/bin/env python3
"""White-frame detector node: runs the pure-numpy detector on the shared
camera stream and publishes confirmed frame observations for the parking
controller. Multi-frame acquisition confirmation and bounded-loss
hysteresis come from FrameConfirmGate; a loss inside the near zone is
flagged immediately. Subscribes to the sole usb_cam image topic; never
starts a second camera and never changes any camera setting.
"""

import json

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ucar_delivery import protocol
from ucar_delivery.frame_detector import (
    FrameConfirmGate,
    FrameObservation,
    WhiteFrameDetector,
)


def observation_to_dict(observation):
    return {
        "timestamp": observation.timestamp,
        "frame_detected": observation.frame_detected,
        "confidence": observation.confidence,
        "near_center_x": observation.near_center_x,
        "far_center_x": observation.far_center_x,
        "left_boundary": observation.left_boundary,
        "right_boundary": observation.right_boundary,
        "front_boundary_y": observation.front_boundary_y,
        "visible_boundary_count": observation.visible_boundary_count,
        "yaw_error": observation.yaw_error,
        "lateral_error": observation.lateral_error,
        "near_width": observation.near_width,
        "far_width": observation.far_width,
        "near_thickness": observation.near_thickness,
        "far_thickness": observation.far_thickness,
        "confirmed": observation.confirmed,
        "near_zone": observation.near_zone,
        "near_zone_loss": observation.near_zone_loss,
    }


def _config_value(key, default):
    return rospy.get_param("~frame_detector/%s" % key, default)


class FrameDetectorNode:
    def __init__(self):
        self._config = {
            "gray_threshold": _config_value("gray_threshold", 180),
            "min_line_pixels": _config_value("min_line_pixels", 60),
            "min_col_pixels": _config_value("min_col_pixels", 60),
            "min_boundaries": _config_value("min_boundaries", 2),
            "roi_x_min": _config_value("roi_x_min", 0),
            "roi_x_max": _config_value("roi_x_max", 640),
            "roi_y_min": _config_value("roi_y_min", 0),
            "roi_y_max": _config_value("roi_y_max", 480),
            "max_line_thickness": _config_value("max_line_thickness", 20),
            "min_geometry_confidence": _config_value(
                "min_geometry_confidence", 0.65
            ),
            "morph_iterations": _config_value("morph_iterations", 0),
            "use_hsv": _config_value("use_hsv", False),
            "hsv_sat_max": _config_value("hsv_sat_max", 60.0),
            "hsv_value_min": _config_value("hsv_value_min", 180.0),
            "perspective_tolerance": _config_value(
                "perspective_tolerance", 0.2
            ),
            "perspective_tolerance_px": _config_value(
                "perspective_tolerance_px", 15.0
            ),
        }
        self._detector = WhiteFrameDetector(self._config)
        self._gate = FrameConfirmGate(self._config)
        self._camera_topic = rospy.get_param("~topics/camera", "/usb_cam/image_raw")
        self._start_topic = rospy.get_param(
            "~topics/frame_start", "/task/delivery_frame_start"
        )
        self._observation_topic = rospy.get_param(
            "~topics/frame_observation", "/task/delivery_frame_observation"
        )
        self._publisher = rospy.Publisher(
            self._observation_topic, String, queue_size=10
        )
        self._context = None
        rospy.Subscriber(
            self._start_topic, String, self._on_start, queue_size=1
        )
        rospy.Subscriber(
            self._camera_topic, Image, self._on_image, queue_size=1
        )

    def _on_start(self, message):
        try:
            payload = protocol.load_object(message.data)
        except protocol.ProtocolError as exc:
            rospy.logwarn("ignored invalid frame start: %s", exc)
            return
        self._context = {
            "phase": payload.get("phase"),
            "task_id": payload.get("task_id"),
            "goal_id": payload.get("goal_id"),
        }
        # 新任务/新阶段：确认与宽限状态不继承旧任务
        self._gate.reset()
        rospy.loginfo("frame search started for %s", payload.get("phase"))

    def _on_image(self, message):
        if self._context is None:
            return
        try:
            image = self._decode_gray(message)
        except Exception as exc:
            rospy.logwarn("frame image decode failed: %s", exc)
            return
        stamp = message.header.stamp.to_sec() if message.header.stamp else rospy.get_time()
        observation = self._detector.detect(image, stamp)
        observation = self._gate.update(observation)
        payload = {
            "protocol_version": 1,
            "phase": self._context["phase"],
            "task_id": self._context["task_id"],
            "goal_id": self._context["goal_id"],
            "observation": observation_to_dict(observation),
        }
        self._publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _decode_gray(self, message):
        import numpy as np
        width = message.width
        height = message.height
        data = np.frombuffer(message.data, dtype=np.uint8)
        if message.encoding in ("mono8", "8UC1"):
            return data.reshape(height, width)
        if message.encoding in ("rgb8", "bgr8"):
            image = data.reshape(height, width, 3)
            if message.encoding == "rgb8":
                image = image[:, :, ::-1]
            gray = (0.299 * image[:, :, 2] + 0.587 * image[:, :, 1]
                    + 0.114 * image[:, :, 0])
            return np.ascontiguousarray(gray.astype(np.uint8))
        raise ValueError("unsupported image encoding: %s" % message.encoding)


def main():
    rospy.init_node("delivery_frame_detector")
    FrameDetectorNode()
    rospy.loginfo("delivery_frame_detector node started")
    rospy.spin()


if __name__ == "__main__":
    main()
