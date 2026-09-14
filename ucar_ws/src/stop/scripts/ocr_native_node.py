#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ocr_native_node.py —— OCR 文字识别节点（RKNN PP-OCR）

运行方式:
    rosrun yolo8 ocr_native_node.py

功能：
    订阅摄像头图像，使用 RKNN 加速的 PaddleOCR 模型（检测 + 识别）进行文字识别，
    筛选包含目标关键字（食品/电子/日用）的识别结果，发布为 BoundingBoxes 消息。

模型：
    - 检测模型: models/ppocrv4_det.rknn
    - 识别模型: models/ppocrv4_rec.rknn
    - 字典文件: models/ppocr_keys_v1.txt

ROS 接口：
    发布: /perception/bounding_boxes (BoundingBoxes) —— OCR 识别到的目标检测框
    订阅: /perception/detect_image (Image)           —— 待识别的图像（来自 yolo8_scan）
"""

import os
import logging
import threading
import traceback

import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image

from stop.msg import BoundingBox
from stop.msg import BoundingBoxes


# ==============================================================================
# 全局变量
# ==============================================================================
bridge = None                       # CvBridge 实例
pub = None                          # BoundingBoxes 发布者
ocr_model = None                    # OCR TextSystem 模型实例
infer_lock = threading.Lock()       # 推理互斥锁（防止推理堆积）

# ==============================================================================
# 配置常量
# ==============================================================================
TARGET_KEYWORDS = ["食品", "电子", "日用"]   # 目标关键字列表
TARGET_IMAGE_WIDTH = 640                      # 统一输出坐标系宽度
TARGET_IMAGE_HEIGHT = 480                     # 统一输出坐标系高度


# ==============================================================================
# 工具函数
# ==============================================================================

def script_path(relative_path):
    """
    将相对路径解析为相对于本脚本所在目录的绝对路径。
    """
    if os.path.isabs(relative_path):
        return relative_path
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), relative_path)


def restore_ros_logging_level_names():
    """
    修复 ROS 日志级别名称映射。
    某些库可能会覆盖 logging 的级别名称，此函数恢复标准映射。
    """
    logging.addLevelName(logging.DEBUG, "DEBUG")
    logging.addLevelName(logging.INFO, "INFO")
    logging.addLevelName(logging.WARNING, "WARNING")
    logging.addLevelName(logging.ERROR, "ERROR")
    logging.addLevelName(logging.CRITICAL, "CRITICAL")


def make_empty_boxes(image_msg=None):
    """
    构造一个空的 BoundingBoxes 消息（无检测结果时发布）。
    """
    bounding_boxes = BoundingBoxes()
    bounding_boxes.header.stamp = rospy.Time.now()

    if image_msg is not None:
        bounding_boxes.image_header = image_msg.header
    else:
        bounding_boxes.image_header.stamp = rospy.Time.now()

    return bounding_boxes


def polygon_to_bbox(points):
    """
    将多边形顶点列表转为轴对齐边界框 (xmin, ymin, xmax, ymax)。
    """
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))


def contains_target_keyword(text):
    """
    检查文本是否包含任意目标关键字。
    """
    return any(keyword in text for keyword in TARGET_KEYWORDS)


def flip_box_to_original_image(xmin, ymin, xmax, ymax, image_width, image_height):
    """
    将水平翻转后图像上的坐标映射回原始图像坐标。
    """
    new_xmin = image_width - xmax
    new_xmax = image_width - xmin

    new_xmin = max(0, min(image_width, new_xmin))
    new_xmax = max(0, min(image_width, new_xmax))
    new_ymin = max(0, min(image_height, ymin))
    new_ymax = max(0, min(image_height, ymax))

    return new_xmin, new_ymin, new_xmax, new_ymax


def scale_box(xmin, ymin, xmax, ymax, src_width, src_height, target_width, target_height):
    """
    将边界框从源图像尺寸缩放到目标统一尺寸 (640×480)。
    """
    scale_x = float(target_width) / float(src_width)
    scale_y = float(target_height) / float(src_height)

    scaled_xmin = int(round(xmin * scale_x))
    scaled_ymin = int(round(ymin * scale_y))
    scaled_xmax = int(round(xmax * scale_x))
    scaled_ymax = int(round(ymax * scale_y))

    scaled_xmin = max(0, min(target_width, scaled_xmin))
    scaled_xmax = max(0, min(target_width, scaled_xmax))
    scaled_ymin = max(0, min(target_height, scaled_ymin))
    scaled_ymax = max(0, min(target_height, scaled_ymax))

    return scaled_xmin, scaled_ymin, scaled_xmax, scaled_ymax


# ==============================================================================
# OCR 结果转换
# ==============================================================================

def trans_ocr_result(filter_boxes, filter_rec_res, image_msg):
    """
    将 OCR 模型输出的检测框和识别结果转换为 ROS BoundingBoxes 消息。
    """
    bounding_boxes = make_empty_boxes(image_msg)
    image_width = image_msg.width
    image_height = image_msg.height

    if filter_boxes is None or filter_rec_res is None:
        return bounding_boxes

    for idx, (dt_box, rec_result) in enumerate(zip(filter_boxes, filter_rec_res)):
        try:
            text, score = rec_result[0]

            # 只保留包含目标关键字的识别结果
            if not contains_target_keyword(text):
                if text.strip():
                    rospy.loginfo("OCR text has no target keyword, skip: %s", text)
                continue

            # 多边形 → 轴对齐边界框
            xmin, ymin, xmax, ymax = polygon_to_bbox(dt_box)

            # 水平翻转还原
            xmin, ymin, xmax, ymax = flip_box_to_original_image(
                xmin, ymin, xmax, ymax, image_width, image_height
            )

            # 缩放到统一坐标系
            xmin, ymin, xmax, ymax = scale_box(
                xmin, ymin, xmax, ymax,
                image_width, image_height,
                TARGET_IMAGE_WIDTH, TARGET_IMAGE_HEIGHT,
            )

            # 构造 ROS 消息
            bounding_box = BoundingBox()
            bounding_box.probability = float(score)
            bounding_box.xmin = xmin
            bounding_box.ymin = ymin
            bounding_box.xmax = xmax
            bounding_box.ymax = ymax
            bounding_box.id = idx
            bounding_box.Class = text

            bounding_boxes.bounding_boxes.append(bounding_box)
        except Exception as exc:
            rospy.logwarn("Failed to convert OCR result %d: %s", idx, exc)

    return bounding_boxes


# ==============================================================================
# ROS 回调：图像 → OCR → 发布
# ==============================================================================

def img_callback(msg):
    """
    /perception/detect_image 回调。
    对输入图像执行 OCR 识别，筛选关键字后发布 BoundingBoxes。
    """
    if ocr_model is None:
        rospy.logwarn_throttle(2.0, "OCR model is not ready, drop this frame")
        return

    if not infer_lock.acquire(False):
        rospy.logwarn_throttle(2.0, "OCR is busy, drop this frame")
        return

    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = cv2.flip(img, 1)                                    # 水平翻转
        filter_boxes, filter_rec_res = ocr_model.run(img)         # OCR 推理
        bounding_boxes = trans_ocr_result(filter_boxes, filter_rec_res, msg)
        pub.publish(bounding_boxes)

        if bounding_boxes.bounding_boxes:
            texts = [box.Class for box in bounding_boxes.bounding_boxes]
            rospy.loginfo("OCR published %d boxes: %s", len(texts), ", ".join(texts))
        else:
            rospy.loginfo_throttle(2.0, "OCR published empty BoundingBoxes")
    except Exception as exc:
        rospy.logerr("OCR inference failed: %s", exc)
        rospy.logerr(traceback.format_exc())
        pub.publish(make_empty_boxes(msg))
    finally:
        infer_lock.release()


# ==============================================================================
# 模型加载与释放
# ==============================================================================

def load_ocr_model():
    """
    加载 OCR TextSystem 模型（检测 + 识别）。
    使用 RK3588 NPU Core 0 进行推理加速。
    """
    from ocr import TextSystem
    from rknnlite.api import RKNNLite
    restore_ros_logging_level_names()

    det_model_path = script_path("models/ppocrv4_det.rknn")
    rec_model_path = script_path("models/ppocrv4_rec.rknn")
    character_dict_path = script_path("models/ppocr_keys_v1.txt")
    target = "rk3588"
    drop_score = 0.5

    print("Loading OCR detection model: {}".format(det_model_path))
    print("Loading OCR recognition model: {}".format(rec_model_path))

    model = TextSystem(
        det_model_path=det_model_path,
        rec_model_path=rec_model_path,
        character_dict_path=character_dict_path,
        target=target,
        drop_score=drop_score,
        core_mask=RKNNLite.NPU_CORE_0,
    )
    restore_ros_logging_level_names()
    return model


def release_ocr_model():
    """
    释放 OCR 模型的 RKNN 资源（节点关闭时调用）。
    """
    global ocr_model

    if ocr_model is None:
        return

    try:
        ocr_model.text_detector.model.release()
        ocr_model.text_recognizer.model.release()
        rospy.loginfo("OCR RKNN models released")
    except Exception as exc:
        rospy.logwarn("Failed to release OCR RKNN models: %s", exc)
    finally:
        ocr_model = None


# ==============================================================================
# 主入口
# ==============================================================================
if __name__ == "__main__":
    rospy.init_node("ocr_native_node")

    bridge = cv_bridge.CvBridge()

    # ---- ROS 发布者：OCR 识别结果 ----
    pub = rospy.Publisher(
        "/perception/bounding_boxes",
        BoundingBoxes,
        queue_size=30
    )

    print("Starting native OCR node")
    ocr_model = load_ocr_model()
    rospy.on_shutdown(release_ocr_model)
    print("OCR model loaded")

    # ---- ROS 订阅者：待识别图像 ----
    rospy.Subscriber("/perception/detect_image", Image, img_callback)

    rospy.spin()
