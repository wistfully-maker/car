#!/usr/bin/env python3
"""Workshop sign adapter node: narrow YOLO+OCR backend wrapper.

fake backend (default): deterministic, fed by /task/delivery_sign_fake for
local tests. vehicle backend: imports the vehicle-only inference modules and
fails fast at startup when the contract is not available. This wrapper has
never been validated against the vehicle files and must be checked on the
vehicle before use.
"""

import json
import logging
import math
import sys
from pathlib import Path

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ucar_delivery import protocol
from ucar_delivery.sign_detector import SignDetector
from ucar_delivery.vehicle_sign_backend import (
    VehicleBackendError,
    VehicleSignBackend,
)


class FakeSignBackend:
    """Deterministic backend: consumes /task/delivery_sign_fake messages."""

    def __init__(self, node):
        self._node = node
        rospy.Subscriber(
            node.topic("sign_fake", "/task/delivery_sign_fake"),
            String, self._on_fake, queue_size=1,
        )

    def _on_fake(self, message):
        try:
            payload = protocol.load_object(message.data)
        except protocol.ProtocolError as exc:
            rospy.logwarn("ignored invalid fake sign: %s", exc)
            return
        candidate = {
            "timestamp": rospy.get_time(),
            "target": payload.get("target"),
            "confidence": float(payload.get("confidence", 1.0)),
            "bbox": payload.get("bbox"),
            "detection_yaw": payload.get("detection_yaw"),
            "ocr_text": payload.get("ocr_text"),
            "ocr_confidence": payload.get("ocr_confidence"),
        }
        self._node.on_fake_candidate(candidate, payload.get("pose"))

    def detect(self, image, timestamp):
        return None


def _restore_standard_logging_level_names():
    """Undo global level-name aliases installed by some RKNN runtimes."""
    for level, name in (
        (logging.CRITICAL, "CRITICAL"),
        (logging.ERROR, "ERROR"),
        (logging.WARNING, "WARNING"),
        (logging.INFO, "INFO"),
        (logging.DEBUG, "DEBUG"),
        (logging.NOTSET, "NOTSET"),
    ):
        logging.addLevelName(level, name)


def _load_vehicle_backend_unchecked(module_name, ocr_module_name):
    """Load the vehicle YOLO backend and fail fast on a broken contract.

    Both the YOLO module and the OCR module must be importable; a missing
    OCR module is a startup error, never a silently degraded mission.
    """
    import importlib
    workspace_src = str(Path(__file__).resolve().parents[2])
    if workspace_src not in sys.path:
        sys.path.insert(0, workspace_src)
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        raise RuntimeError(
            "vehicle sign backend %r unavailable: %s; "
            "the wrapper contract was never validated on this machine"
            % (module_name, exc)
        )
    detector = getattr(module, "YoloDetector", None)
    if detector is None:
        raise RuntimeError(
            "vehicle sign backend %r must expose YoloDetector" % module_name
        )
    try:
        ocr_module = importlib.import_module(ocr_module_name)
    except Exception as exc:
        raise RuntimeError(
            "vehicle OCR backend %r unavailable: %s; "
            "missing OCR module must fail fast before any motion"
            % (ocr_module_name, exc)
        )
    ocr_detector = getattr(ocr_module, "OcrDetector", None)
    if ocr_detector is None:
        ocr_detector = getattr(ocr_module, "RapidOcrInfer", None)
    if ocr_detector is None:
        raise RuntimeError(
            "vehicle OCR backend %r must expose OcrDetector or RapidOcrInfer"
            % ocr_module_name
        )
    try:
        return VehicleSignBackend(detector(), ocr_detector())
    except (TypeError, VehicleBackendError) as exc:
        raise RuntimeError("invalid vehicle sign backend contract: %s" % exc)


def _load_vehicle_backend(module_name, ocr_module_name):
    """Load vehicle inference while containing RKNN logging side effects."""
    try:
        return _load_vehicle_backend_unchecked(module_name, ocr_module_name)
    finally:
        # rknnlite 1.6 on the vehicle renames INFO/WARNING to I/W globally.
        # rospy's handler only accepts standard names and otherwise crashes
        # on the first log call with KeyError('I').
        _restore_standard_logging_level_names()


class SignAdapterNode:
    def __init__(self):
        self._backend_mode = rospy.get_param("~sign_detector/backend", "fake")
        self._config = {
            "min_confidence": rospy.get_param("~sign_detector/min_confidence", 0.6),
            "confirm_frames": rospy.get_param("~sign_detector/confirm_frames", 3),
            "ocr_confirm_frames": rospy.get_param(
                "~sign_detector/ocr_confirm_frames", 1
            ),
            "max_image_age": rospy.get_param("~sign_detector/max_image_age", 1.0),
            "target_aliases": rospy.get_param("~sign_detector/target_aliases", {}),
        }
        # 注意：_fake_backend 在 _setup_backend 中才赋值，此处不能提前引用。
        self._backend = None
        self._fake_backend = None
        # 像素 bbox -> 激光系 bearing/half-width 的标定参数（实车待标定）。
        self._camera_center_x = float(
            rospy.get_param("~sign_detector/camera_center_x", 320.0)
        )
        self._camera_focal_px = float(
            rospy.get_param("~sign_detector/camera_focal_px", 500.0)
        )
        self._camera_topic = rospy.get_param("~topics/camera", "/usb_cam/image_raw")
        self._sign_start_topic = rospy.get_param(
            "~topics/sign_start", "/task/delivery_sign_start"
        )
        self._sign_found_topic = rospy.get_param(
            "~topics/sign_found", "/task/delivery_sign_found"
        )
        self._publisher = rospy.Publisher(
            self._sign_found_topic, String, queue_size=10
        )
        self._context = None
        self._setup_backend()
        rospy.Subscriber(
            self._sign_start_topic, String, self._on_start, queue_size=1
        )
        rospy.Subscriber(
            self._camera_topic, Image, self._on_image, queue_size=1
        )

    def topic(self, key, default):
        return {
            "sign_fake": rospy.get_param(
                "~topics/sign_fake", "/task/delivery_sign_fake"
            ),
        }.get(key, default)

    def _setup_backend(self):
        if self._backend_mode == "fake":
            self._fake_backend = FakeSignBackend(self)
            self._detector = SignDetector(
                rospy.get_time, self._config, self._fake_backend
            )
            return
        if self._backend_mode == "vehicle":
            module = rospy.get_param(
                "~sign_detector/vehicle_backend_module", "yolo_biao.infer"
            )
            ocr_module = rospy.get_param(
                "~sign_detector/vehicle_ocr_module", "ocr.ocr_infer"
            )
            try:
                self._backend = _load_vehicle_backend(module, ocr_module)
            except RuntimeError as exc:
                rospy.logfatal(str(exc))
                sys.exit(1)
            self._detector = SignDetector(
                rospy.get_time, self._config, self._backend
            )
            return
        rospy.logfatal(
            "unsupported sign_detector/backend %r; use fake or vehicle",
            self._backend_mode,
        )
        sys.exit(1)

    def _on_start(self, message):
        try:
            payload = protocol.load_object(message.data)
            target = payload.get("target_workshop")
        except protocol.ProtocolError as exc:
            rospy.logwarn("ignored invalid sign start: %s", exc)
            return
        if target not in protocol.ALLOWED_WORKSHOPS:
            rospy.logwarn("ignored unsupported sign target: %s", target)
            return
        self._context = {
            "phase": payload.get("phase"),
            "task_id": payload.get("task_id"),
            "goal_id": payload.get("goal_id"),
        }
        self._detector.set_target(target)
        rospy.loginfo(
            "sign search started for %s (%s)", target, payload.get("phase")
        )

    def _on_image(self, message):
        if self._context is None or self._detector._target_normalized is None:
            return
        try:
            image = self._decode_image(message)
        except Exception as exc:
            rospy.logwarn("sign image decode failed: %s", exc)
            return
        stamp = message.header.stamp.to_sec() if message.header.stamp else rospy.get_time()
        confirmed = self._detector.process_frame(image, stamp, None)
        if confirmed is not None:
            self._publish_found(confirmed)

    def _decode_image(self, message):
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
            return np.ascontiguousarray(image)
        raise ValueError("unsupported image encoding: %s" % message.encoding)

    def on_fake_candidate(self, candidate, pose):
        if self._context is None:
            return
        confirmed = self._detector.on_candidate(
            candidate, candidate["timestamp"], pose
        )
        if confirmed is not None:
            self._publish_found(confirmed)

    def _attach_bearing(self, detection):
        """像素 bbox -> 激光系中心 bearing 与半宽（相机标定参数）。

        bearing = atan2((bbox_center_x - camera_center_x), focal)，
        half_width = atan2(bbox_width/2, focal)。参数为安全初始值，
        实车部署前必须用相机内参标定（见 HANDOFF 待标定清单）。
        """
        bbox = detection.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return detection
        try:
            x1, _y1, x2, _y2 = (float(value) for value in bbox)
        except (TypeError, ValueError):
            return detection
        if not all(math.isfinite(value) for value in (x1, x2)):
            return detection
        center_x = (x1 + x2) / 2.0
        half_width = (x2 - x1) / 2.0
        if half_width <= 0.0:
            return detection
        enriched = dict(detection)
        enriched["bearing_rad"] = math.atan2(
            center_x - self._camera_center_x, self._camera_focal_px
        )
        enriched["half_width_rad"] = math.atan2(
            half_width, self._camera_focal_px
        )
        return enriched

    def _publish_found(self, confirmed):
        if self._context is None:
            return
        payload = {
            "protocol_version": 1,
            "phase": self._context["phase"],
            "task_id": self._context["task_id"],
            "goal_id": self._context["goal_id"],
            "detection": self._attach_bearing(confirmed),
        }
        self._publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self._context = None
        self._detector.reset()


def main():
    rospy.init_node("delivery_sign_adapter")
    SignAdapterNode()
    rospy.loginfo("delivery_sign_adapter node started")
    rospy.spin()


if __name__ == "__main__":
    main()
