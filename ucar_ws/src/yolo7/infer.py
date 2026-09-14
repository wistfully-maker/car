#!/usr/bin/env python3
"""
YOLO 门牌检测 + OCR 文字识别 — 完整集成方案
============================================
yolo7 — 单进程，单摄像头，双 NPU 模型协作，完全自包含

=== 整体思路 ===

问题: yolo_ocr (检测) 和 rapidocr_rknn (识别) 是两个独立进程，各自打开摄像头，
      车一转起来 OCR 进程拿到运动模糊帧，无法识别。
      NPU 上同时跑两个进程的模型也可能冲突。

方案: 一个进程打开一次摄像头，YOLO 每帧检测门牌位置。
      当置信度 > 0.75 时，立刻裁剪门牌区域，用 OCR 识别车间名。
      结果只可能是三个加工车间之一，同一个结果不重复输出。

=== 架构 ===

  camera ──► YoloDetector(rknn) ──► dets ──► conf>0.75? ──► crop → OCR(rknn) ──► 车间名
                 │                               │
              每帧都跑                    阈值触发，立刻识别

=== 模型文件（全部在 yolo7 目录内） ===

  ./best.rknn              YOLO26n 门牌检测 (8.4 MB, 输入 320×320, 输出 2100×5)
  ./models/det.rknn        OCR 文字检测 (5.1 MB)
  ./models/cls.rknn        OCR 方向分类 (1.7 MB)
  ./models/rec.rknn        OCR 文字识别 (9.6 MB)
  ./models/characters.txt  字符集

=== 执行逻辑 ===

  YOLO 每帧检测 → 置信度 > 0.75 → 立刻裁剪门牌区域 → OCR 识别 → 输出车间名
  结果只会是: 电子产品加工车间 / 食品加工车间 / 日用品加工车间
  同一个车间名不重复输出

=== 用法 ===

  python3 infer.py                      # 完整管线（检测 + OCR，推荐）
  python3 infer.py --no-ocr             # 纯检测
  python3 infer.py --image test.jpg     # 单张图片

=== 输出（仅结果，零日志） ===

  signboard conf=0.836 box=[785,368,1920,1080]
  [OCR] 食品加工车间 (0.912)

=== 小车部署 ===

  目录: ~/ucar_ws/src/yolo7/
  依赖: pip install rknnlite rapidocr_onnxruntime opencv-python numpy
        # librknnrt.so 已预装于 /usr/lib/
"""

import os
import sys
import argparse
import numpy as np

# ============================================================
# 路径配置（全部本地，不引用外部目录）
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
YOLO_RKNN = os.path.join(SCRIPT_DIR, "best.rknn")
# OCR 模型路径在 ocr.py 中定义为 ./models/，此处不重复定义

# 检测参数
DEFAULT_IMGSZ = 320
DEFAULT_CONF = 0.5
IOU_THRESH = 0.45

# OCR 触发参数
OCR_MIN_CONF = 0.65       # YOLO 置信度超过此值立刻触发 OCR


# ============================================================
# fd 保存/恢复 — 抑制 librknnrt C 库日志
# ============================================================
class _FDSilence:
    """上下文管理器：临时重定向 stdout+stderr 到 /dev/null。

    librknnrt.so 的初始化日志直接 write(fd=1, ...)，绕过 Python 的 sys.stdout。
    YOLO 模型 init_runtime 期间使用。
    """
    def __enter__(self):
        self._devnull = os.open(os.devnull, os.O_WRONLY)
        self._saved_1 = os.dup(1)
        self._saved_2 = os.dup(2)
        os.dup2(self._devnull, 1)
        os.dup2(self._devnull, 2)
        os.close(self._devnull)
        return self

    def __exit__(self, *args):
        try:
            import ctypes
            ctypes.CDLL(None).fflush(None)  # 先冲掉 C 库 buffer → /dev/null
        except Exception:
            pass
        os.dup2(self._saved_1, 1)
        os.dup2(self._saved_2, 2)
        os.close(self._saved_1)
        os.close(self._saved_2)
        return False


# ============================================================
# YOLO 门牌检测器 — RKNN NPU
# ============================================================
class YoloDetector:
    """YOLO26n 门牌检测。

    模型: best.rknn (ONNX graph-surgery 后导出，输出 (1,2100,5) raw predictions)
    输入: BGR uint8 (H, W, 3)
    输出: [[x1, y1, x2, y2, conf], ...]
    """

    def __init__(self, model_path=YOLO_RKNN, imgsz=DEFAULT_IMGSZ,
                 conf_thresh=DEFAULT_CONF):
        self.model_path = model_path
        self.imgsz = imgsz
        self.conf_thresh = conf_thresh
        self._rknn = None
        self._init()

    def _init(self):
        """加载 RKNN 模型，初始化 NPU 运行时（抑制 C 库日志）"""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"YOLO 模型不存在: {self.model_path}")

        from rknnlite.api import RKNNLite
        rknn = RKNNLite()

        with _FDSilence():
            ret = rknn.load_rknn(self.model_path)
            if ret != 0:
                raise RuntimeError(f"YOLO RKNN load failed, ret={ret}")

            ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if ret != 0:
                ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                if ret != 0:
                    raise RuntimeError(f"YOLO RKNN init_runtime failed, ret={ret}")

        self._rknn = rknn

    def predict(self, frame):
        """检测门牌。返回 [[x1,y1,x2,y2,conf], ...]"""
        h, w = frame.shape[:2]
        tensor = self._preprocess(frame)
        outputs = self._rknn.inference(inputs=[tensor])
        return self._postprocess(outputs, w, h)

    def _preprocess(self, frame):
        """BGR uint8 → (1,3,320,320) float32 [0,1] NCHW"""
        import cv2
        img = cv2.resize(frame, (self.imgsz, self.imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)        # HWC → CHW
        img = np.expand_dims(img, axis=0)   # → (1,3,320,320) NCHW
        return np.ascontiguousarray(img)

    def _postprocess(self, outputs, frame_w, frame_h):
        """YOLOv8 raw output → NMS → [[x1,y1,x2,y2,conf], ...]

        自适应 (1,5,N)、(1,6,N)、(1,N,5)、(1,N,6)、(N,5) 等格式。
        """
        if isinstance(outputs, list):
            output = outputs[0]
        else:
            output = outputs

        # ---- 格式检测 → (N, 5) ----
        if output.ndim == 3:
            if output.shape[1] in (5, 6):
                preds = output[0].T
            elif output.shape[2] in (5, 6):
                preds = output[0]
            else:
                return []
        elif output.ndim == 2:
            preds = output
        else:
            return []

        if preds.shape[1] >= 6:
            preds = preds[:, :5]

        # ---- 置信度过滤 ----
        mask = preds[:, 4] >= self.conf_thresh
        preds = preds[mask]
        if len(preds) == 0:
            return []

        # ---- cxcywh → xyxy (归一化坐标 → 像素坐标) ----
        boxes = []
        s = self.imgsz
        for p in preds:
            cx, cy, bw, bh = p[:4]
            conf = float(p[4])
            x1 = (cx - bw / 2) / s * frame_w
            y1 = (cy - bh / 2) / s * frame_h
            x2 = (cx + bw / 2) / s * frame_w
            y2 = (cy + bh / 2) / s * frame_h
            boxes.append([max(0, x1), max(0, y1),
                          min(frame_w, x2), min(frame_h, y2), conf])

        if len(boxes) <= 1:
            return boxes

        # ---- NMS ----
        boxes_np = np.array(boxes)
        idxs = np.argsort(boxes_np[:, 4])[::-1]
        keep = []
        while len(idxs) > 0:
            keep.append(idxs[0])
            if len(idxs) == 1:
                break
            iou = self._box_iou(boxes_np[idxs[0]], boxes_np[idxs[1:]])
            idxs = idxs[1:][iou < IOU_THRESH]
        return boxes_np[keep].tolist()

    @staticmethod
    def _box_iou(box, boxes):
        x1 = np.maximum(box[0], boxes[:, 0])
        y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2])
        y2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        area1 = (box[2] - box[0]) * (box[3] - box[1])
        area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        return inter / (area1 + area2 - inter + 1e-6)

    def release(self):
        if self._rknn:
            self._rknn.release()
            self._rknn = None


# ============================================================
# OCR 文字识别器 — 直接引用 rapidocr_rknn（已验证可用）
# ============================================================
class OCRRecognizer:
    """OCR 车间名识别。

    直接使用 rapidocr_rknn/ocr_infer.py 的 RapidOcrRKNN。
    该模块在 import 时会永久重定向 fd 1/fd 2 到 /dev/null，
    因此需要在 import 前保存、import 后恢复。
    """

    def __init__(self, no_flip=False):
        self._wrapper = None   # RapidOcrRKNN 实例（来自 rapidocr_rknn）
        self._no_flip = no_flip
        self._init()

    def _init(self):
        """导入 rapidocr_rknn 模块（保存/恢复 stdout/stderr）"""
        RAPIDOCR_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "rapidocr_rknn")

        # 保存原始 fd，rapidocr_rknn 模块会永久重定向它们
        _saved_1 = os.dup(1)
        _saved_2 = os.dup(2)

        try:
            sys.path.insert(0, RAPIDOCR_DIR)
            from ocr_infer import RapidOcrRKNN
            self._wrapper = RapidOcrRKNN()
        finally:
            # 冲刷 C 库 buffer → /dev/null，然后恢复 fd
            try:
                import ctypes
                ctypes.CDLL(None).fflush(None)
            except Exception:
                pass
            os.dup2(_saved_1, 1)
            os.dup2(_saved_2, 2)
            os.close(_saved_1)
            os.close(_saved_2)

    def predict_frame(self, frame):
        """对整帧做 OCR，返回 (text, confidence)"""
        import cv2
        if self._wrapper is None:
            return "", 0.0
        # DEBUG: 保存 OCR 输入帧
        cv2.imwrite("ocr_input.png", frame)
        # flip 可选
        ocr_input = frame if self._no_flip else cv2.flip(frame, 1)
        result = self._wrapper.ocr(ocr_input)
        boxes, _ = result
        if not boxes:
            print("[OCR-DEBUG] no boxes at all")
            return "", 0.0
        texts = [(t, float(c)) for _, t, c in boxes if t]
        print(f"[OCR-DEBUG] detected: {texts}")
        # 关键词唯一匹配
        VALID = {
            "电子产品加工车间": ["电", "子"],
            "食品加工车间":     ["食"],
            "日用品加工车间":   ["日", "用"],
        }
        for _, text, conf in sorted(boxes, key=lambda b: b[2], reverse=True):
            if not text or float(conf) < 0.5:
                continue
            candidates = set()
            for full, keywords in VALID.items():
                for kw in keywords:
                    if kw in text:
                        candidates.add(full)
            if len(candidates) == 1:
                return candidates.pop(), float(conf)
        return "", 0.0

    def predict_crop(self, crop):
        """对裁剪后的门牌区域做 OCR（不镜像），返回 (text, confidence)"""
        if self._wrapper is None or crop is None or crop.size == 0:
            return "", 0.0
        # 直接调底层 RapidOCR，不经 cv2.flip
        result = self._wrapper.ocr(crop)
        boxes, _ = result
        if not boxes:
            return "", 0.0
        # 关键词唯一匹配
        VALID = {
            "电子产品加工车间": ["电", "子"],
            "食品加工车间":     ["食"],
            "日用品加工车间":   ["日", "用"],
        }
        for _, text, conf in sorted(boxes, key=lambda b: b[2], reverse=True):
            if not text or float(conf) < 0.5:
                continue
            candidates = set()
            for full, keywords in VALID.items():
                for kw in keywords:
                    if kw in text:
                        candidates.add(full)
            if len(candidates) == 1:
                return candidates.pop(), float(conf)
        return "", 0.0


# ============================================================
# 主管线
# ============================================================
def main():
    p = argparse.ArgumentParser(description="yolo7 — YOLO 门牌检测 + OCR 识别")
    p.add_argument("--model", default=YOLO_RKNN, help="YOLO RKNN 模型路径")
    p.add_argument("--image", default=None, help="单张图片路径")
    p.add_argument("--no-ocr", action="store_true", help="禁用 OCR，纯检测")
    p.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    p.add_argument("--conf", type=float, default=DEFAULT_CONF,
                   help="YOLO 检测置信度阈值")
    p.add_argument("--ocr-conf", type=float, default=OCR_MIN_CONF,
                   help="触发 OCR 的最低 YOLO 置信度")
    p.add_argument("--no-flip", action="store_true",
                   help="OCR 不做镜像 flip")
    args = p.parse_args()

    # ================================================================
    # Phase 1: 初始化 YOLO 检测器
    # ================================================================
    detector = YoloDetector(
        model_path=args.model,
        imgsz=args.imgsz,
        conf_thresh=args.conf,
    )

    # ================================================================
    # Phase 2: 初始化 OCR（可选）
    # ================================================================
    ocr = None
    ocr_min_conf = args.ocr_conf
    if not args.no_ocr:
        try:
            ocr = OCRRecognizer(no_flip=args.no_flip)
        except Exception as e:
            print(f"[WARN] OCR init failed: {e}")
            print("[INFO] Falling back to detection-only mode")
            ocr = None

    # ================================================================
    # Phase 3: 推理循环
    # ================================================================
    import cv2

    # ---- 单张图片 ----
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[ERROR] Cannot read: {args.image}")
            sys.exit(1)

        dets = detector.predict(frame)
        if dets:
            best = max(dets, key=lambda d: d[4])
            x1, y1, x2, y2, conf = best
            print(f"signboard conf={conf:.3f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")

            if ocr is not None:
                crop = frame[int(y1):int(y2), int(x1):int(x2)]
                text, oconf = ocr.predict_crop(crop)
                if text:
                    print(f"[OCR] {text} ({oconf:.3f})")
        else:
            print("(no signboard detected)")

        detector.release()
        return

    # ---- 摄像头实时 ----
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        detector.release()
        sys.exit(1)

    last_det_str = ""
    last_ocr_text = ""
    best_frame = None       # 本轮置信度最高的整帧（原始，供 OCR）
    best_box = None         # 对应检测框 [x1,y1,x2,y2]
    best_conf = 0.0
    ocr_done = False        # 本轮是否已识别成功
    print("=" * 50)
    print("  yolo7 — YOLO + OCR Pipeline")
    print("  Ctrl+C to stop")
    print("=" * 50)

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # ---- YOLO 检测 ----
            dets = detector.predict(frame)

            if dets:
                best = max(dets, key=lambda d: d[4])
                x1, y1, x2, y2, conf = best
                s = f"signboard conf={conf:.3f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]"
                if s != last_det_str:
                    print(s)
                    last_det_str = s

                # ---- 追踪最优帧 ----
                if conf > best_conf:
                    best_conf = conf
                    best_frame = frame.copy()
                    best_box = [int(x1), int(y1), int(x2), int(y2)]

                # ---- 置信度达标 + 本轮未成功 → 立刻 OCR ----
                if ocr is not None and conf >= ocr_min_conf and not ocr_done:
                    text, oconf = ocr.predict_frame(frame)
                    if text:
                        if text != last_ocr_text:
                            print(f"[OCR] {text} ({oconf:.3f})")
                            last_ocr_text = text
                        ocr_done = True
                        # ---- OCR 成功 → 保存该清晰帧 ----
                        drawn = frame.copy()
                        if best_box is not None:
                            bx1, by1, bx2, by2 = best_box
                            bh = by2 - by1
                            by2_adj = int(by1 + bh * 0.45)
                            cv2.rectangle(drawn, (bx1, by1),
                                          (bx2, by2_adj), (0, 255, 0), 2)
                            cv2.putText(drawn, f"{best_conf:.3f}",
                                        (bx1, by1 - 8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.imwrite("ocr_result.png", drawn)

            else:
                # ---- 标牌丢失 → 如果本轮没识别成功，用最优帧 OCR ----
                if best_frame is not None and ocr is not None and not ocr_done:
                    text, oconf = ocr.predict_frame(best_frame)
                    if text:
                        if text != last_ocr_text:
                            print(f"[OCR] {text} ({oconf:.3f})")
                            last_ocr_text = text
                    else:
                        print(f"[OCR] no match (best_conf={best_conf:.3f})")

                # ---- 保存最优帧（画框），调试用 ----
                if best_frame is not None:
                    drawn = best_frame.copy()
                    h, w = drawn.shape[:2]
                    if best_box is not None:
                        bx1, by1, bx2, by2 = best_box
                        bh = by2 - by1
                        by2_adj = int(by1 + bh * 0.45)
                        cv2.rectangle(drawn, (bx1, by1),
                                      (bx2, by2_adj), (0, 255, 0), 2)
                        cv2.putText(drawn, f"{best_conf:.3f}",
                                    (bx1, by1 - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    # PNG 无损保存，避免 JPEG 压缩模糊
                    cv2.imwrite("best_frame.png", drawn)
                    print(f"[SAVE] best_frame.png ({w}x{h})")

                # ---- 重置，准备下一轮 ----
                if last_det_str:
                    last_det_str = ""
                best_frame = None
                best_box = None
                best_conf = 0.0
                ocr_done = False

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        with _FDSilence():
            detector.release()


if __name__ == "__main__":
    main()

