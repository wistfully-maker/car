#!/usr/bin/env python3
"""
RapidOCR 门牌车间识别（小车端）
自带检测框 + 识别，无需训练。

用法:
    python3 infer.py               # 摄像头实时
    python3 infer.py --image a.jpg  # 单张图片
"""

import os, sys, argparse, time

class RapidOcrInfer:
    def __init__(self):
        from rapidocr_onnxruntime import RapidOCR
        self.ocr = RapidOCR()
        print("[RapidOCR] 就绪")

    def predict(self, frame):
        import cv2
        frame = cv2.flip(frame, 1)  # 镜像→正向
        result = self.ocr(frame)
        boxes, _ = result

        if not boxes:
            return "", 0.0

        # 取置信度最高的框
        best = max(boxes, key=lambda b: b[2])
        _, text, conf = best

        if not text:
            return "", 0.0

        # 严格匹配 → 仅输出三种合法车间名
        VALID = {
            "电子产品加工车间": ["电", "子"],
            "食品加工车间":     ["食"],
            "日用品加工车间":   ["日", "用"],
        }
        REQUIRED = ["品加工", "加工车", "工车间"]

        # 必须包含特征后缀
        if not any(s in text for s in REQUIRED):
            return "", 0.0

        # 按关键区分字找匹配
        candidates = set()
        for full, keywords in VALID.items():
            for kw in keywords:
                if kw in text:
                    candidates.add(full)

        # 唯一命中才输出
        if len(candidates) == 1:
            return candidates.pop(), float(conf)

        # 多个候选或无候选 → 拒绝
        return "", 0.0


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--image"); p.add_argument("--camera", action="store_true")
    args = p.parse_args()

    infer = RapidOcrInfer()

    if args.camera or not args.image:
        import cv2
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        print("\n[RapidOCR] 摄像头已开启，Ctrl+C 退出\n")
        last = ""
        try:
            while True:
                ret, frame = cap.read()
                if not ret: continue
                text, conf = infer.predict(frame)
                if text and text != last:
                    last = text
                    print(f"  ✅ {text} ({conf:.3f})")
        except KeyboardInterrupt:
            print("\n退出")
        cap.release()

    elif args.image:
        import cv2
        frame = cv2.imread(args.image)
        text, conf = infer.predict(frame)
        print(f"\n  {text} ({conf:.4f})")
