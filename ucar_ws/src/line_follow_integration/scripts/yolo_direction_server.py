#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红绿灯方向检测服务 (NPU 版) —— 替换旧 yolo_server.py

以 turn_yolo/yolo_live.py 的 RKNN(NPU) 推理为核心, 套符合 line_follow_integration
契约的壳:

  1. 输出词与 ROUTE_SCRIPTS 对齐: straight / left_turn / right_turn / red_light
  2. N 帧一致去抖 (默认 vote_window=3): 方向必须连续 N 帧一致且置信度达标才输出
  3. 每帧都写 /tmp/yolo_result.txt (低置信度/未检出写 red_light = 继续等, 不残留旧值)
  4. 模型路径/阈值/窗口/文件路径均可用 ROS 参数覆盖
  5. 额外发布 /yolo/direction topic (String), 供后续动态识别使用

被 line_follow_supervisor 以 python3 绝对路径拉起, __name:=phase3_yolo_server。
"""

import logging

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite

def _restore_logging_levels():
    """rknnlite 会把 logging 级别名覆盖成 I/W/E/D (INFO->I 等),
    导致 rospy 日志 emit 时 _logging_to_rospy_names[record.levelname] KeyError。
    双向恢复: 反向 _levelToName (决定 record.levelname) 是修复关键。"""
    for _lvl, _name in ((50, "CRITICAL"), (40, "ERROR"), (30, "WARNING"),
                        (20, "INFO"), (10, "DEBUG"), (0, "NOTSET")):
        logging.addLevelName(_lvl, _name)
    logging._nameToLevel.update({
        "CRITICAL": 50, "FATAL": 50, "ERROR": 40, "WARN": 30,
        "WARNING": 30, "INFO": 20, "DEBUG": 10, "NOTSET": 0,
    })


_restore_logging_levels()


class YoloDirectionServer:
    # 类别顺序与新模型训练一致 (0=straight 1=left 2=right 3=red), 不能改顺序
    NAMES = {0: "straight", 1: "left_turn", 2: "right_turn", 3: "red_light"}

    def __init__(self):
        rospy.init_node("yolo_direction_server", anonymous=True)
        self.model_path = rospy.get_param(
            "~model_path",
            "/home/ucar/ucar_ws/src/turn_yolo/turn_yolo_480_cls.rknn",
        )
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.6))
        self.vote_window = int(rospy.get_param("~vote_window", 3))
        if self.vote_window < 1:
            raise ValueError("vote_window must be >= 1")
        self.result_file = rospy.get_param("~result_file", "/tmp/yolo_result.txt")

        self.rknn = RKNNLite()
        self.rknn.load_rknn(self.model_path)
        self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        _restore_logging_levels()  # load/init 可能再次覆盖, 再恢复一次

        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/yolo/direction", String, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self._on_image, queue_size=1)

        self._votes = []  # 最近 vote_window 个 (label, conf)
        rospy.loginfo(
            "yolo_direction_server 就绪 model=%s conf=%.2f vote=%d",
            self.model_path, self.conf_threshold, self.vote_window,
        )

    @staticmethod
    def _letterbox(im, new=(480, 480), color=(114, 114, 114)):
        sh = im.shape[:2]
        r = min(new[0] / sh[0], new[1] / sh[1])
        nu = (int(round(sh[1] * r)), int(round(sh[0] * r)))
        dw, dh = (new[1] - nu[0]) / 2, (new[0] - nu[1]) / 2
        if sh[::-1] != nu:
            im = cv2.resize(im, nu, interpolation=cv2.INTER_LINEAR)
        return cv2.copyMakeBorder(
            im, int(round(dh - 0.1)), int(round(dh + 0.1)),
            int(round(dw - 0.1)), int(round(dw + 0.1)),
            cv2.BORDER_CONSTANT, value=color,
        )

    def _detect(self, img):
        """单帧推理, 返回 (类别词, 置信度), 不做阈值过滤."""
        x = cv2.cvtColor(self._letterbox(img), cv2.COLOR_BGR2RGB)[None]
        out = self.rknn.inference(
            inputs=[x], data_type="uint8", data_format="nhwc"
        )[0]
        preds = out[0].T
        scores = 1.0 / (1.0 + np.exp(-preds))
        conf = scores.max(1)
        cls = scores.argmax(1)
        i = int(conf.argmax())
        return self.NAMES[int(cls[i])], float(conf[i])

    def _decide(self):
        """N 帧一致 + 达标才输出方向, 否则 red_light(继续等)."""
        if len(self._votes) < self.vote_window:
            return "red_light"
        labels = [v[0] for v in self._votes]
        if len(set(labels)) == 1:
            label = labels[0]
            if label in ("straight", "left_turn", "right_turn") and all(
                v[1] >= self.conf_threshold for v in self._votes
            ):
                return label
        return "red_light"

    def _on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            label, conf = self._detect(img)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "yolo 单帧处理失败: %s", exc)
            return
        if conf < self.conf_threshold:
            label = "red_light"  # 置信度不足 → 按等待处理

        self._votes.append((label, conf))
        if len(self._votes) > self.vote_window:
            self._votes.pop(0)

        out = self._decide()
        try:
            with open(self.result_file, "w") as f:
                f.write(out)
        except OSError:
            pass
        self.pub.publish(String(data=out))
        rospy.loginfo_throttle(
            1.0, ">>> %-10s conf=%.2f raw=%s", out, conf, label
        )


def main():
    YoloDirectionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
