#!/usr/bin/env python3
"""
YOLO 仓库标牌检测（小车端）
基于 onnxruntime，单类检测。

用法:
    python3 yolo_infer.py              # 摄像头实时
    python3 yolo_infer.py --image a.jpg  # 单张
"""

import os, sys, argparse, time, numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "warehouse.onnx")
CONF_THRESH = 0.5
IOU_THRESH = 0.45

class YoloDetector:
    def __init__(self, model_path=None):
        import onnxruntime as ort
        model_path = model_path or MODEL_PATH
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, opts, providers=["CPUExecutionProvider"])
        self.iname = self.session.get_inputs()[0].name
        self.oshape = self.session.get_outputs()[0].shape
        print(f"[YOLO] 模型就绪 | 输出: {self.oshape}")

    def preprocess(self, frame, imgsz=320):
        import cv2
        img = cv2.resize(frame, (imgsz, imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)
        return np.expand_dims(img, axis=0)

    def postprocess(self, output, frame_w, frame_h, imgsz=320):
        """YOLOv8 ONNX 输出格式: (1, 5, 8400) — cx, cy, w, h, conf"""
        preds = output[0][0]  # (5, 8400)
        preds = preds.T       # (8400, 5)

        # 按置信度过滤
        mask = preds[:, 4] > CONF_THRESH
        dets = preds[mask]
        if len(dets) == 0:
            return []

        # NMS
        boxes = []
        for d in dets:
            cx, cy, w, h = d[:4]
            x1 = (cx - w/2) / imgsz * frame_w
            y1 = (cy - h/2) / imgsz * frame_h
            x2 = (cx + w/2) / imgsz * frame_w
            y2 = (cy + h/2) / imgsz * frame_h
            boxes.append([x1, y1, x2, y2, float(d[4])])

        boxes = np.array(boxes)
        # 简单 NMS
        keep = []
        idxs = np.argsort(boxes[:, 4])[::-1]
        while len(idxs) > 0:
            keep.append(idxs[0])
            if len(idxs) == 1: break
            iou = self._iou(boxes[idxs[0]], boxes[idxs[1:]])
            idxs = idxs[1:][iou < IOU_THRESH]
        return boxes[keep].tolist()

    def _iou(self, box, boxes):
        x1 = np.maximum(box[0], boxes[:, 0])
        y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2])
        y2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(0, x2-x1) * np.maximum(0, y2-y1)
        area1 = (box[2]-box[0]) * (box[3]-box[1])
        area2 = (boxes[:, 2]-boxes[:, 0]) * (boxes[:, 3]-boxes[:, 1])
        return inter / (area1 + area2 - inter + 1e-6)

    def predict(self, frame):
        h, w = frame.shape[:2]
        tensor = self.preprocess(frame, imgsz=320)
        outputs = self.session.run(None, {self.iname: tensor})
        detections = self.postprocess(outputs, w, h, imgsz=320)
        return detections  # [[x1,y1,x2,y2,conf], ...]


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--image"); p.add_argument("--camera", action="store_true")
    args = p.parse_args()

    detector = YoloDetector()

    if args.camera or not args.image:
        import cv2
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        print("\n[YOLO] Ctrl+C 退出\n")
        try:
            while True:
                ret, frame = cap.read()
                if not ret: continue
                dets = detector.predict(frame)
                if dets:
                    x1, y1, x2, y2, conf = dets[0]
                    cx = (x1 + x2) / 2
                    print(f"  仓库标牌 conf={conf:.3f} center_x={cx:.0f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
        except KeyboardInterrupt:
            print("\n退出")
        cap.release()
        cv2.destroyAllWindows()

    elif args.image:
        import cv2
        frame = cv2.imread(args.image)
        dets = detector.predict(frame)
        for x1,y1,x2,y2,conf in dets:
            print(f"  仓库标牌: box=({int(x1)},{int(y1)},{int(x2)},{int(y2)}) conf={conf:.3f}")
        if not dets:
            print("  不检测到标牌")

