#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
traffic_light_detector.py —— 交通灯检测节点（RKNN YOLO）

运行方式:
    rosrun ucar_nav traffic_light_detector.py    # 一条命令，直接检测出结果

功能：
    默认 OpenCV 直读摄像头（无需先起 usb_cam 节点），利用 RKNN YOLO 模型实时识别
    交通灯类别（green_straight / green_left / green_right / red_stop），
    启动即开始检测（默认 auto_start=True，无需手动发 /traffic/start_detect）。
    采用置信度加权投票机制，达到阈值后决定最终行驶方向，发布到 /start_follow。

ROS 接口：
    发布: /start_follow (String)              —— 最终行驶方向（straight / left / right）
    发布: /traffic_light/debug_image (Image)   —— 带检测框标注的调试图像
    订阅: /traffic/start_detect (Bool)         —— 启动检测信号（auto_start=false 时用）
    订阅: /usb_cam/image_raw (Image)           —— 摄像头图像（仅 use_ros_image:=true 时订阅）

模型：
    默认路径: 脚本同目录下的 yolov8_custom.rknn
    输入尺寸: 640×640
    类别: green_straight, green_left, green_right, red_stop

投票参数：
    - 加权模式 (WEIGHTED_VOTE=True)：按置信度累加，总分达 REQUIRED_WEIGHT_SUM 后决策
    - 计数模式 (WEIGHTED_VOTE=False)：采集满 REQUIRED_FRAMES 帧后计数决策
"""

import os
# 屏蔽 RKNN C 库的 INFO/WARN 级别日志
os.environ["RKNN_LOG_LEVEL"] = "3"

import threading

# ---- RKNN stderr 过滤器（抑制 C 库 D/W 级调试日志） ----
_rknn_stderr_lock = threading.Lock()
_rknn_original_stderr = None
_rknn_devnull_fd = None


def _install_rknn_stderr_filter():
    """将 RKNN C 库的 stderr 重定向到 /dev/null，仅保留 ERROR 级别输出。"""
    global _rknn_original_stderr, _rknn_devnull_fd
    with _rknn_stderr_lock:
        if _rknn_original_stderr is not None:
            return
        _rknn_original_stderr = os.dup(2)
        _rknn_devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(_rknn_devnull_fd, 2)


def _restore_stderr():
    """恢复原始 stderr。"""
    global _rknn_original_stderr, _rknn_devnull_fd
    with _rknn_stderr_lock:
        if _rknn_original_stderr is not None:
            os.dup2(_rknn_original_stderr, 2)
            os.close(_rknn_original_stderr)
            _rknn_original_stderr = None
        if _rknn_devnull_fd is not None:
            os.close(_rknn_devnull_fd)
            _rknn_devnull_fd = None

import time
from collections import deque

import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class LightDetector:
    """
    交通灯检测器。

    封装了 RKNN 模型加载、图像预处理、NPU 推理、NMS 后处理、
    置信度加权投票决策和 FPS 统计的完整流程。
    通过主线程执行推理以避免 RKNN 跨线程崩溃。
    """

    # ==========================================================================
    # 模型与推理参数
    # ==========================================================================
    MODEL_PATH = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "yolov8_custom.rknn"
    )
    INPUT_SIZE = 640                # 模型输入尺寸（正方形）
    CONFIDENCE_THRESHOLD = 0.4      # 置信度过滤阈值
    OVERLAP_THRESHOLD = 0.45        # NMS 交并比阈值

    # ==========================================================================
    # 类别定义
    # ==========================================================================
    CATEGORIES = ["green_straight", "green_left", "green_right", "red_stop"]

    # ==========================================================================
    # 图像预处理参数
    # ==========================================================================
    MIRROR_MODE = None              # 镜像模式：0=上下, 1=左右, -1=上下左右, None=不翻转

    # ==========================================================================
    # 投票统计参数
    # ==========================================================================
    REQUIRED_FRAMES = 10            # 计数模式所需帧数
    WEIGHTED_VOTE = True            # 是否启用置信度加权投票
    REQUIRED_WEIGHT_SUM = 4         # 加权模式下目标总分

    # ==========================================================================
    # FPS 滑动窗口大小
    # ==========================================================================
    FPS_WINDOW = 15

    def __init__(self):
        # ---- 运行状态 ----
        self.is_active = False          # 是否处于检测状态
        self.collected_frames = 0       # 已采集帧数（计数模式）
        self.vote_pool = []             # 投票池（计数模式）
        self.weight_sum = 0.0           # 置信度加权总分
        self.weighted_votes = {}        # 加权投票字典 {方向: 累计权重}

        # ---- ROS 组件 ----
        self.cv_converter = cv_bridge.CvBridge()
        self.pub_direction = None       # 方向发布者 → /start_follow
        self.pub_debug_image = None     # 调试图像发布者

        # ---- NPU 推理引擎 ----
        self.neural_net = None

        # ---- FPS 统计 ----
        self.tick_history = deque(maxlen=self.FPS_WINDOW)
        self.display_fps = 0.0

        # ---- 诊断计数器 ----
        self.total_frame_cnt = 0

        # ---- 日志节流时间戳 ----
        self.last_no_detect_log = 0.0
        self.last_active_heartbeat = 0.0

        # ---- 图像缓冲（回调线程写，主线程读，避免 NPU 跨线程问题） ----
        self.latest_frame = None
        self.cap = None                 # OpenCV 直读模式的摄像头句柄
        self.use_ros_image = False      # 图像来源：False=OpenCV 直读, True=ROS 订阅

    # ========================================================================
    #  图像预处理（静态方法）
    # ========================================================================

    @staticmethod
    def resize_and_pad(bgr_image, target_dim, fill_value=(114, 114, 114)):
        """
        等比例缩放图像并居中填充至目标尺寸。
        返回: (填充后图像, 缩放比例, 左填充像素数, 上填充像素数)
        """
        h, w = bgr_image.shape[:2]
        scale = min(target_dim[0] / h, target_dim[1] / w)

        resized_w = int(round(w * scale))
        resized_h = int(round(h * scale))

        resized = cv2.resize(bgr_image, (resized_w, resized_h),
                             interpolation=cv2.INTER_LINEAR)

        gap_w = target_dim[1] - resized_w
        gap_h = target_dim[0] - resized_h
        half_gap_w = gap_w / 2.0
        half_gap_h = gap_h / 2.0

        pad_left = int(round(half_gap_w - 0.1))
        pad_right = int(round(half_gap_w + 0.1))
        pad_top = int(round(half_gap_h - 0.1))
        pad_bottom = int(round(half_gap_h + 0.1))

        padded_img = np.pad(
            resized,
            ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
            mode='constant',
            constant_values=fill_value[0]
        )

        return padded_img, scale, pad_left, pad_top

    @staticmethod
    def cxcywh_to_corners(bboxes):
        """
        将边界框格式从 [center_x, center_y, width, height] 转为 [x1, y1, x2, y2]。
        """
        corners = np.zeros_like(bboxes)
        half_w = bboxes[:, 2] / 2.0
        half_h = bboxes[:, 3] / 2.0
        corners[:, 0] = bboxes[:, 0] - half_w    # x1
        corners[:, 1] = bboxes[:, 1] - half_h    # y1
        corners[:, 2] = bboxes[:, 0] + half_w    # x2
        corners[:, 3] = bboxes[:, 1] + half_h    # y2
        return corners

    @staticmethod
    def suppress_overlaps(bbox_arr, confidences, overlap_thresh):
        """
        非极大值抑制（NMS）：按置信度排序，移除重叠度超过阈值的冗余检测框。
        返回保留的检测框索引列表。
        """
        num_boxes = len(bbox_arr)
        if num_boxes == 0:
            return []

        x1 = bbox_arr[:, 0]
        y1 = bbox_arr[:, 1]
        x2 = bbox_arr[:, 2]
        y2 = bbox_arr[:, 3]

        zone = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        ranking = confidences.argsort()[::-1]

        retained = []
        while len(ranking) > 0:
            current = ranking[0]
            retained.append(current)

            if len(ranking) == 1:
                break

            rest = ranking[1:]

            inter_left = np.maximum(x1[current], x1[rest])
            inter_top = np.maximum(y1[current], y1[rest])
            inter_right = np.minimum(x2[current], x2[rest])
            inter_bottom = np.minimum(y2[current], y2[rest])

            inter_w = np.maximum(0, inter_right - inter_left)
            inter_h = np.maximum(0, inter_bottom - inter_top)
            inter_area = inter_w * inter_h

            iou_vals = inter_area / (zone[current] + zone[rest] - inter_area + 1e-6)

            live_mask = np.argwhere(iou_vals <= overlap_thresh).flatten()
            ranking = rest[live_mask]

        return retained

    # ========================================================================
    #  后处理流水线
    # ========================================================================

    def _filter_by_confidence(self, raw_pred):
        """
        根据置信度阈值 (CONFIDENCE_THRESHOLD) 过滤原始预测结果。
        置信度 = objectness × class_max_score。
        返回: (角点格式的边界框, 综合置信度, 类别ID)
        """
        bbox_raw = raw_pred[:, :4]
        # sigmoid 归一化：模型输出的是未激活的 logits，必须恢复到 [0,1] 概率。
        # 否则置信度会大到几百~几千（如 2173），加权投票一帧就爆表、失去多帧抗噪意义。
        objectness = 1.0 / (1.0 + np.exp(-raw_pred[:, 4]))
        class_probs = 1.0 / (1.0 + np.exp(-raw_pred[:, 5:5 + len(self.CATEGORIES)]))

        class_ids = np.argmax(class_probs, axis=1)
        class_max_scores = class_probs[np.arange(len(class_probs)), class_ids]
        combined_scores = objectness * class_max_scores

        keep_mask = combined_scores >= self.CONFIDENCE_THRESHOLD

        bbox_arr = self.cxcywh_to_corners(bbox_raw[keep_mask])
        cls_conf = combined_scores[keep_mask]
        cls_ids = class_ids[keep_mask]

        return bbox_arr, cls_conf, cls_ids

    def _map_to_original_coords(self, bbox_arr, scale, pad_left, pad_top, orig_w, orig_h):
        """
        将模型输出坐标（填充后的 640×640 坐标系）逆映射回原始图像尺寸。
        """
        bbox_arr[:, [0, 2]] -= pad_left
        bbox_arr[:, [1, 3]] -= pad_top
        bbox_arr /= scale

        bbox_arr[:, [0, 2]] = np.clip(bbox_arr[:, [0, 2]], 0, orig_w - 1)
        bbox_arr[:, [1, 3]] = np.clip(bbox_arr[:, [1, 3]], 0, orig_h - 1)

        return bbox_arr

    def _apply_nms(self, bbox_arr, cls_conf, cls_ids):
        """
        对检测结果执行非极大值抑制（NMS），移除重叠检测框。
        """
        keep_indices = self.suppress_overlaps(bbox_arr, cls_conf, self.OVERLAP_THRESHOLD)
        return bbox_arr[keep_indices], cls_conf[keep_indices], cls_ids[keep_indices]

    def detect_objects(self, neural_output, scale, pad_left, pad_top, orig_w, orig_h):
        """
        完整的后处理流程：
        维度自适应 → 置信度过滤 → 坐标逆映射 → NMS 去重。

        参数:
            neural_output: NPU 推理原始输出
            scale, pad_left, pad_top: resize_and_pad 的返回值
            orig_w, orig_h: 原始图像尺寸

        返回: (边界框列表, 置信度列表, 类别ID列表)
        """
        pred_array = neural_output[0]
        num_features = 4 + 1 + len(self.CATEGORIES)

        # ---- 维度自适应修复 ----
        # 1. 挤压掉 Padding 造成的冗余维度
        pred_array = np.squeeze(pred_array)

        # 2. 若挤压后仍不是 2D，强制重塑为 (N, num_features)
        if pred_array.ndim > 2:
            if pred_array.shape[-1] == num_features:
                pred_array = pred_array.reshape(-1, num_features)
            else:
                pred_array = pred_array.reshape(num_features, -1).T

        # 3. 确保最终形状为 (N, num_features)
        if pred_array.ndim == 2:
            if pred_array.shape[0] == num_features and pred_array.shape[1] != num_features:
                pred_array = pred_array.T
        elif pred_array.ndim == 1 or pred_array.size == 0:
            return [], [], []

        # 1. 置信度初筛
        bbox_arr, cls_conf, cls_ids = self._filter_by_confidence(pred_array)

        if len(bbox_arr) == 0:
            return [], [], []

        # 2. 坐标逆映射
        bbox_arr = self._map_to_original_coords(bbox_arr, scale, pad_left, pad_top, orig_w, orig_h)

        # 3. NMS 去重
        return self._apply_nms(bbox_arr, cls_conf, cls_ids)

    # ========================================================================
    #  投票决策
    # ========================================================================

    def _collect_and_vote(self, detected_objects):
        """
        收集检测结果（含红灯），达到阈值后触发投票决策。
        - 加权模式：按置信度累加，总分达标后调用 _finalize_vote
        - 计数模式：收集满 REQUIRED_FRAMES 帧后调用 _finalize_vote
        """
        _, confs, class_ids = detected_objects

        frame_all_detections = []

        for i, cid in enumerate(class_ids):
            cat_name = self.CATEGORIES[int(cid)]
            conf = confs[i] if i < len(confs) else 0.0
            frame_all_detections.append(f"{cat_name}({conf:.2f})")

        if frame_all_detections:
            rospy.loginfo("[检测] 本帧结果: {}".format(frame_all_detections))

        if len(class_ids) == 0:
            return

        if self.collected_frames >= self.REQUIRED_FRAMES:
            return

        if self.WEIGHTED_VOTE:
            # ---- 置信度加权模式（红灯也参与，最终输出 red/straight/left/right） ----
            for i, cid in enumerate(class_ids):
                cat_name = self.CATEGORIES[int(cid)]
                conf = confs[i] if i < len(confs) else 0.5
                self.weighted_votes[cat_name] = self.weighted_votes.get(cat_name, 0.0) + conf
                self.weight_sum += conf

            rospy.loginfo("收集进度: {:.1f}/{:.1f}  加权: {}".format(
                self.weight_sum, self.REQUIRED_WEIGHT_SUM,
                {k: f"{v:.2f}" for k, v in self.weighted_votes.items()}))

            if self.weight_sum >= self.REQUIRED_WEIGHT_SUM:
                self._finalize_vote()
        else:
            # ---- 传统计数模式 ----
            for i, cid in enumerate(class_ids):
                self.vote_pool.append(self.CATEGORIES[int(cid)])
                self.collected_frames += 1

            rospy.loginfo("收集进度: {}/{}  本帧录入: {}".format(
                self.collected_frames, self.REQUIRED_FRAMES, frame_all_detections))

            if self.collected_frames >= self.REQUIRED_FRAMES:
                self._finalize_vote()

    def _finalize_vote(self):
        """
        完成投票，选出得票最高的方向，发布到 /start_follow 并关闭节点。

        方向映射:
            green_straight → straight
            green_left     → left
            green_right    → right
        """
        if self.WEIGHTED_VOTE:
            winner = max(self.weighted_votes, key=self.weighted_votes.get)
            rospy.loginfo("加权投票完成  加权统计: {}  →  winner: {}".format(
                {k: "{:.2f}".format(v) for k, v in self.weighted_votes.items()}, winner))
        else:
            tally = {}
            for v in self.vote_pool:
                tally[v] = tally.get(v, 0) + 1
            winner = max(tally, key=tally.get)
            rospy.loginfo("计数投票完成  计票: {}  →  winner: {}".format(tally, winner))

        # 类别名 → 行驶方向映射
        if winner == "green_straight":
            direction = "straight"
        elif winner == "green_left":
            direction = "left"
        elif winner == "green_right":
            direction = "right"
        elif winner == "red_stop":
            direction = "red"
        else:
            direction = "straight"

        self.pub_direction.publish(String(data=direction))

        rospy.loginfo("→ 发布 /start_follow: {}".format(direction))

        rospy.signal_shutdown("投票决策完成")

    # ========================================================================
    #  FPS 统计
    # ========================================================================

    def _update_fps(self, timestamp):
        """
        基于滑动时间窗口计算实时帧率。
        """
        self.tick_history.append(timestamp)
        if len(self.tick_history) >= 2:
            span = self.tick_history[-1] - self.tick_history[0]
            count = len(self.tick_history) - 1
            self.display_fps = count / span if span > 0 else 0.0

    # ========================================================================
    #  ROS 回调
    # ========================================================================

    def _on_start_detect(self, msg):
        """
        /traffic/start_detect 回调。
        收到 True 后激活检测并重置所有投票状态。
        """
        if msg.data:
            self.is_active = True
            self.collected_frames = 0
            self.vote_pool = []
            self.weight_sum = 0.0
            self.weighted_votes = {}
            if self.WEIGHTED_VOTE:
                rospy.loginfo(">>> 交通灯检测已激活，加权模式，目标总分: {} <<<".format(
                    self.REQUIRED_WEIGHT_SUM))
            else:
                rospy.loginfo(">>> 交通灯检测已激活，计数模式，目标帧数: {} <<<".format(
                    self.REQUIRED_FRAMES))

    def _on_image(self, msg):
        """
        /usb_cam/image_raw 回调。
        仅解码图像并写入缓冲区，绝不在子线程执行 NPU 推理（避免死锁）。
        """
        self.total_frame_cnt += 1
        try:
            color_img = self.cv_converter.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = color_img
        except Exception as exc:
            rospy.logerr("图像解码失败: {}".format(exc))

    # ========================================================================
    #  启动与主循环
    # ========================================================================

    def run(self):
        """
        节点入口。
        初始化 ROS 发布/订阅、加载 RKNN 模型、进入主线程推理循环。
        推理在主线程执行以避免 RKNN C 库的跨线程调用崩溃。
        """
        rospy.init_node("traffic_light_detector")

        # 默认直接开始检测（便于单独测试）；传 _auto_start:=false 则改为等待 /traffic/start_detect 触发
        self.is_active = rospy.get_param("~auto_start", True)

        # 修复 rospy 日志级别映射（兼容部分 ROS 发行版）
        from rosgraph.roslogging import _logging_to_rospy_names
        for _abbr, _full in [('I', 'INFO'), ('W', 'WARN'), ('E', 'ERROR'), ('D', 'DEBUG')]:
            if _abbr not in _logging_to_rospy_names and _full in _logging_to_rospy_names:
                _logging_to_rospy_names[_abbr] = _logging_to_rospy_names[_full]

        # 加载 RKNN 库前先安装 stderr 过滤器
        _install_rknn_stderr_filter()

        from rknnlite.api import RKNNLite

        model_path = rospy.get_param("~model_path", self.MODEL_PATH)
        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")

        # ---- ROS 发布者 ----
        self.pub_direction = rospy.Publisher('/start_follow', String, queue_size=1)
        self.pub_debug_image = rospy.Publisher('/traffic_light/debug_image', Image, queue_size=1)

        # ---- 加载 RKNN 模型 ----
        self.neural_net = RKNNLite()

        load_ret = self.neural_net.load_rknn(model_path)
        rospy.loginfo("load_rknn 返回值: {}".format(load_ret))
        if load_ret != 0:
            rospy.logerr("模型加载失败")
            return

        init_ret = self.neural_net.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        rospy.loginfo("init_runtime 返回值: {}".format(init_ret))
        if init_ret != 0:
            rospy.logerr("NPU 初始化失败")
            return

        # ---- 图像来源 ----
        # use_ros_image=False（默认）：OpenCV 直读摄像头，运行脚本即可出结果（无需先起 usb_cam）
        # use_ros_image=True：订阅 ROS 图像话题（比赛集成，配合 usb_cam 节点）
        self.use_ros_image = rospy.get_param("~use_ros_image", False)

        if self.use_ros_image:
            rospy.Subscriber(image_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)
            rospy.loginfo("图像来源: ROS 话题 {}".format(image_topic))
        else:
            video_device = rospy.get_param("~video_device", "/dev/video0")
            self.cap = cv2.VideoCapture(video_device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.cap.isOpened():
                rospy.logerr("无法打开摄像头 {}（可传 _use_ros_image:=true 改用 ROS 话题）".format(video_device))
                self.neural_net.release()
                return
            rospy.loginfo("图像来源: OpenCV 直读摄像头 {}".format(video_device))

        rospy.Subscriber('/traffic/start_detect', Bool, self._on_start_detect)

        cv2.namedWindow("RKNN Traffic Light", cv2.WINDOW_NORMAL)

        rospy.loginfo("=== 交通灯检测节点已就绪，等待 /traffic/start_detect: True ===")

        # ---- 主循环：图像显示 + NPU 推理 + 投票 ----
        try:
            while not rospy.is_shutdown():
                if not self.use_ros_image:
                    ret, frame = self.cap.read()
                    self.latest_frame = frame if ret else None

                if self.latest_frame is not None:
                    color_img = self.latest_frame.copy()

                    if self.MIRROR_MODE is not None:
                        color_img = cv2.flip(color_img, self.MIRROR_MODE)

                    if not self.is_active:
                        # 未激活状态：显示等待提示
                        cv2.putText(color_img, "Waiting for /traffic/start_detect...", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    else:
                        # 检测中心跳日志（每 5 秒一行，避免刷屏）
                        now_hb = time.time()
                        if now_hb - self.last_active_heartbeat >= 5.0:
                            rospy.loginfo("[心跳] 检测中 ... 累计采集 {}/{} 帧, 推理 FPS: {:.1f}".format(
                                self.collected_frames, self.REQUIRED_FRAMES, self.display_fps))
                            self.last_active_heartbeat = now_hb

                        img_h, img_w = color_img.shape[:2]

                        # 图像预处理 → NPU 推理
                        model_input, scale, pad_left, pad_top = self.resize_and_pad(
                            color_img, (self.INPUT_SIZE, self.INPUT_SIZE))

                        tensor = cv2.cvtColor(model_input, cv2.COLOR_BGR2RGB)
                        tensor = np.expand_dims(tensor, axis=0)

                        # 主线程调用 inference，避免死锁
                        outputs = self.neural_net.inference(inputs=[tensor])
                        bboxes, confs, cls_ids = self.detect_objects(
                            outputs, scale, pad_left, pad_top, img_w, img_h)

                        # 绘制检测框和标签
                        for bx, cf, ci in zip(bboxes, confs, cls_ids):
                            left, top, right, bottom = bx.astype(int)
                            cat_name = self.CATEGORIES[int(ci)]
                            text = f"{cat_name} {cf:.2f}"

                            cv2.rectangle(color_img, (left, top), (right, bottom), (0, 255, 0), 2)
                            cv2.putText(color_img, text, (left, max(20, top - 5)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        # 收集投票
                        self._collect_and_vote((bboxes, confs, cls_ids))

                        # 更新并显示 FPS
                        now = time.time()
                        self._update_fps(now)
                        cv2.putText(color_img, f"FPS: {self.display_fps:.1f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                    cv2.imshow("RKNN Traffic Light", color_img)

                    # 发布调试图像（远程查看用）
                    try:
                        debug_msg = self.cv_converter.cv2_to_imgmsg(color_img, encoding="bgr8")
                        debug_msg.header.stamp = rospy.Time.now()
                        self.pub_debug_image.publish(debug_msg)
                    except Exception as exc:
                        rospy.logerr_throttle(5.0, "发布调试图像失败: {}".format(exc))

                cv2.waitKey(1)
                rospy.sleep(0.01)
        finally:
            self.neural_net.release()
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = LightDetector()
    detector.run()

