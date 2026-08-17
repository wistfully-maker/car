#!/usr/bin/env python3
"""
YOLO26 门牌检测 + OCR 文字识别 — 小车端 RKNN NPU 推理
======================================================
单进程方案：共享摄像头，YOLO 持续检测作为"有标牌"信号，
全帧 OCR 自己扫描文字，不依赖 YOLO 的粗框定位。

用法:
    python3 infer.py                        # YOLO 检测 + OCR
    python3 infer.py --no-ocr               # 纯 YOLO 检测
    python3 infer.py --image test.jpg       # 单张图片

输出:
    signboard conf=0.836 box=[785,368,1920,1080]
    [OCR] 食品加工车间 (0.912)
"""

import os
import sys
import argparse
import time
import threading
import numpy as np

# ============================================================
# MJPEG HTTP 流 — 浏览器实时看 YOLO 检测画面
# ============================================================
_latest_jpg = None
_lock = threading.Lock()

def _mjpeg_handler():
    """返回一个 HTTP 请求处理器类"""
    from http.server import BaseHTTPRequestHandler
    from socketserver import ThreadingMixIn, TCPServer

    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "close")
                self.end_headers()
                while True:
                    with _lock:
                        jpg = _latest_jpg
                    if jpg is None:
                        time.sleep(0.1)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.05)
                    except (BrokenPipeError, ConnectionResetError):
                        break
            elif self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                html = ("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                        "<title>YOLO Detection</title></head><body>"
                        "<h2>YOLO Detection</h2>"
                        "<img src='/stream' style='max-width:100%;'>"
                        "</body></html>").encode("utf-8")
                self.wfile.write(html)
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, *args):
            pass  # 静默 HTTP 日志

    class _Server(ThreadingMixIn, TCPServer):
        allow_reuse_address = True
        daemon_threads = True

    return _Server(("0.0.0.0", 8080), _Handler)


def _start_stream():
    """后台线程启动 MJPEG 流服务器"""
    server = _mjpeg_handler()
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


# ============================================================
# 配置
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RKNN_MODEL_PATH = os.path.join(SCRIPT_DIR, "best.rknn")
ONNX_MODEL_PATH = os.path.join(SCRIPT_DIR, "best.onnx")
RAPIDOCR_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "rapidocr_rknn")
DEFAULT_IMGSZ = 320
DEFAULT_CONF = 0.5
IOU_THRESH = 0.45


# ============================================================
# YOLO26 RKNN 门牌检测器
# ============================================================
class YoloDetector:
    """YOLO26 门牌检测 + RKNN NPU"""

    def __init__(self, model_path=None, imgsz=DEFAULT_IMGSZ,
                 conf_thresh=DEFAULT_CONF):
        self.model_path = model_path or RKNN_MODEL_PATH
        self.imgsz = imgsz
        self.conf_thresh = conf_thresh
        self._rknn = None
        self._use_rknn = False
        self._init()

    def _init(self):
        """初始化 RKNN NPU，失败则回退 ONNX Runtime"""
        if not os.path.exists(self.model_path):
            onnx_path = ONNX_MODEL_PATH
            if os.path.exists(onnx_path):
                self._init_onnx(onnx_path)
            else:
                raise FileNotFoundError(
                    f"模型不存在: RKNN={self.model_path}, ONNX={onnx_path}"
                )
            return

        try:
            from rknnlite.api import RKNNLite
            rknn = RKNNLite()

            _devnull = os.open(os.devnull, os.O_WRONLY)
            _saved_1 = os.dup(1)
            _saved_2 = os.dup(2)
            os.dup2(_devnull, 1)
            os.dup2(_devnull, 2)
            os.close(_devnull)
            try:
                ret = rknn.load_rknn(self.model_path)
                if ret != 0:
                    raise RuntimeError(f"RKNN load failed, ret={ret}")
                ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
                if ret != 0:
                    ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                    if ret != 0:
                        raise RuntimeError(f"RKNN runtime init failed, ret={ret}")
            finally:
                os.dup2(_saved_1, 1)
                os.dup2(_saved_2, 2)
                os.close(_saved_1)
                os.close(_saved_2)

            self._rknn = rknn
            self._use_rknn = True
            print("[YOLO] RKNN NPU ready")

        except ImportError:
            onnx_path = ONNX_MODEL_PATH
            if os.path.exists(onnx_path):
                self._init_onnx(onnx_path)
            else:
                raise ImportError("rknnlite 和 ONNX 模型均不可用")

    def _init_onnx(self, onnx_path):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._ort_session = ort.InferenceSession(
            onnx_path, opts, providers=["CPUExecutionProvider"])
        self._ort_input = self._ort_session.get_inputs()[0].name
        self._ort_outputs = [o.name for o in self._ort_session.get_outputs()]
        self._use_rknn = False

    def _preprocess(self, frame):
        """BGR → (1, 3, H, W) float32 [0, 1] NCHW"""
        import cv2
        img = cv2.resize(frame, (self.imgsz, self.imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)
        img = np.expand_dims(img, axis=0)
        return np.ascontiguousarray(img)

    def _postprocess(self, outputs, frame_w, frame_h):
        """YOLOv8 ONNX 输出: (1, 5, N) → NMS → [[x1,y1,x2,y2,conf], ...]"""
        if isinstance(outputs, list):
            output = outputs[0]
        else:
            output = outputs

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

        mask = preds[:, 4] >= self.conf_thresh
        preds = preds[mask]
        if len(preds) == 0:
            return []

        boxes = []
        for p in preds:
            cx, cy, w, h = p[:4]
            conf = float(p[4])
            x1 = (cx - w / 2) / self.imgsz * frame_w
            y1 = (cy - h / 2) / self.imgsz * frame_h
            x2 = (cx + w / 2) / self.imgsz * frame_w
            y2 = (cy + h / 2) / self.imgsz * frame_h
            boxes.append([max(0, x1), max(0, y1),
                          min(frame_w, x2), min(frame_h, y2), conf])

        if len(boxes) <= 1:
            return boxes

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

    def _box_iou(self, box, boxes):
        x1 = np.maximum(box[0], boxes[:, 0])
        y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2])
        y2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        area1 = (box[2] - box[0]) * (box[3] - box[1])
        area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        return inter / (area1 + area2 - inter + 1e-6)

    def predict(self, frame):
        """检测门牌。返回 [[x1,y1,x2,y2,conf], ...]"""
        h, w = frame.shape[:2]
        tensor = self._preprocess(frame)
        if self._use_rknn:
            outputs = self._rknn.inference(inputs=[tensor])
        else:
            outputs = self._ort_session.run(
                self._ort_outputs, {self._ort_input: tensor})
        return self._postprocess(outputs, w, h)

    def release(self):
        if self._use_rknn and self._rknn:
            self._rknn.release()


# ============================================================
# 门牌 OCR — 加载 rapidocr_rknn，全帧识别
# ============================================================
class SignboardOCR:
    """全帧 OCR，不依赖 YOLO 裁剪框。"""

    VALID = {
        "电子产品加工车间": ["电", "子"],
        "食品加工车间":     ["食"],
        "日用品加工车间":   ["日", "用"],
    }

    def __init__(self):
        self._ocr_wrapper = None
        self._raw_ocr = None
        self._ready = False
        self._init()

    def _init(self):
        if not os.path.isdir(RAPIDOCR_DIR):
            return
        _devnull = os.open(os.devnull, os.O_WRONLY)
        _saved_1 = os.dup(1)
        _saved_2 = os.dup(2)
        os.dup2(_devnull, 1)
        os.dup2(_devnull, 2)
        os.close(_devnull)
        try:
            sys.path.insert(0, RAPIDOCR_DIR)
            from ocr_infer import RapidOcrRKNN
            self._ocr_wrapper = RapidOcrRKNN()
            self._raw_ocr = self._ocr_wrapper.ocr
            self._ready = True
        except Exception:
            self._ready = False
        finally:
            os.dup2(_saved_1, 1)
            os.dup2(_saved_2, 2)
            os.close(_saved_1)
            os.close(_saved_2)

    def predict(self, frame):
        """全帧 OCR（含镜像翻转），返回 (text, confidence)"""
        if not self._ready:
            return "", 0.0
        return self._ocr_wrapper.predict(frame)

    def predict_crop(self, crop):
        """裁剪区域 OCR（不镜像），返回 (text, confidence)"""
        if not self._ready or crop is None or crop.size == 0:
            return "", 0.0
        result = self._raw_ocr(crop)
        boxes, _ = result
        if not boxes:
            return "", 0.0
        for _, text, conf in sorted(boxes, key=lambda b: b[2], reverse=True):
            if not text or float(conf) < 0.5:
                continue
            candidates = set()
            for full, keywords in self.VALID.items():
                for kw in keywords:
                    if kw in text:
                        candidates.add(full)
            if len(candidates) == 1:
                return candidates.pop(), float(conf)
        return "", 0.0


# ============================================================
# 检测跟踪器 — YOLO 信号 → 触发全帧 OCR
# ============================================================
class DetectionTracker:
    """跟踪 YOLO 检测，管理 OCR 触发时机。

    高置信度时立刻触发全帧 OCR；标牌丢失时用最佳帧 fallback。
    """

    def __init__(self, ocr_trigger_conf=0.60, ocr_cooldown=3.0):
        self.ocr_trigger_conf = ocr_trigger_conf
        self.ocr_cooldown = ocr_cooldown

        self.best_frame = None
        self.best_conf = 0.0
        self.best_box = None
        self.ocr_done = False
        self.last_ocr_time = 0.0
        self.miss_count = 0

    def update(self, dets, frame):
        now = time.time()

        if dets:
            best = max(dets, key=lambda d: d[4])
            conf = best[4]
            self.miss_count = 0

            if conf > self.best_conf:
                self.best_conf = conf
                self.best_frame = frame.copy()
                self.best_box = [int(x) for x in best[:4]]
                self.ocr_done = False

            cooldown_ok = (now - self.last_ocr_time) >= self.ocr_cooldown
            if (not self.ocr_done
                    and conf >= self.ocr_trigger_conf
                    and cooldown_ok
                    and self.best_frame is not None):
                return True, frame

        else:
            self.miss_count += 1
            cooldown_ok = (now - self.last_ocr_time) >= self.ocr_cooldown
            if (not self.ocr_done
                    and self.best_frame is not None
                    and self.best_conf >= self.ocr_trigger_conf
                    and self.miss_count >= 3
                    and cooldown_ok):
                return True, self.best_frame

        return False, None

    def mark_ocr_done(self, text, conf):
        self.ocr_done = True
        self.last_ocr_time = time.time()

    def reset(self):
        self.best_frame = None
        self.best_conf = 0.0
        self.best_box = None
        self.ocr_done = False
        self.miss_count = 0


# ============================================================
# 主管线
# ============================================================
def main():
    p = argparse.ArgumentParser(description="YOLO 门牌检测 + OCR")
    p.add_argument("--model", default=RKNN_MODEL_PATH, help="模型路径")
    p.add_argument("--image", default=None, help="单张图片")
    p.add_argument("--no-ocr", action="store_true", help="禁用 OCR")
    p.add_argument("--no-stream", action="store_true", help="禁用 MJPEG 流")
    p.add_argument("--stream-port", type=int, default=8080, help="MJPEG 流端口")
    p.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    p.add_argument("--conf", type=float, default=DEFAULT_CONF)
    p.add_argument("--ocr-conf", type=float, default=0.55,
                   help="触发 OCR 的最低检测置信度")
    args = p.parse_args()

    # ---- 初始化 YOLO ----
    detector = YoloDetector(
        model_path=args.model,
        imgsz=args.imgsz,
        conf_thresh=args.conf,
    )

    # ---- 初始化 OCR ----
    ocr = None
    tracker = None
    if not args.no_ocr:
        ocr = SignboardOCR()
        tracker = DetectionTracker(ocr_trigger_conf=args.ocr_conf)

    # ---- 单张图片 ----
    if args.image:
        import cv2
        frame = cv2.imread(args.image)
        if frame is None:
            sys.exit(1)
        dets = detector.predict(frame)
        if dets:
            best = max(dets, key=lambda d: d[4])
            x1, y1, x2, y2, conf = best
            print(f"signboard conf={conf:.3f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
            if ocr is not None and ocr._ready:
                text, oconf = ocr.predict(frame)
                if text:
                    print(f"[OCR] {text} ({oconf:.3f})")
        detector.release()
        return

    # ---- 摄像头实时 ----
    import cv2
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        detector.release()
        sys.exit(1)

    # ---- 启动 MJPEG 流 ----
    if not args.no_stream:
        _start_stream()
        import socket
        hostname = socket.gethostname()
        print(f"[STREAM] http://{hostname}:{args.stream_port}/stream")
    print("=" * 50)
    print("  YOLO + OCR Pipeline  |  Ctrl+C to stop")
    print("=" * 50)

    global _latest_jpg
    last_det = ""
    last_ocr_out = ""
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            dets = detector.predict(frame)

            # ---- YOLO 检测 + MJPEG 推流 ----
            if dets:
                best_det = max(dets, key=lambda d: d[4])
                x1, y1, x2, y2, conf = best_det
                s = f"signboard conf={conf:.3f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]"
                if s != last_det:
                    print(s)
                    last_det = s

                if not args.no_stream:
                    preview = frame.copy()
                    bx1, by1, bx2, by2 = int(x1), int(y1), int(x2), int(y2)
                    bh = by2 - by1
                    by2_adj = int(by1 + bh * 0.45)
                    cv2.rectangle(preview, (bx1, by1),
                                  (bx2, by2_adj), (0, 255, 0), 2)
                    cv2.putText(preview, f"{conf:.3f}",
                                (bx1, by1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    _, jpg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    with _lock:
                        _latest_jpg = jpg.tobytes()

                if conf > 0.9:
                    cv2.imwrite("yolo_detect.png", preview)
            else:
                if last_det:
                    last_det = ""
                if not args.no_stream:
                    _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    with _lock:
                        _latest_jpg = jpg.tobytes()

            # ---- OCR 跟踪 & 触发（全帧） ----
            if tracker is not None and ocr is not None and ocr._ready:
                should_ocr, full_frame = tracker.update(dets, frame)
                if should_ocr and full_frame is not None:
                    text, oconf = ocr.predict(full_frame)
                    if text:
                        out = f"[OCR] {text} ({oconf:.3f})"
                        if out != last_ocr_out:
                            print(out)
                            last_ocr_out = out
                    tracker.mark_ocr_done(text, oconf)
                    # 保存 OCR 结果帧
                    drawn = full_frame.copy()
                    if tracker.best_box is not None:
                        bx1, by1, bx2, by2 = tracker.best_box
                        bh = by2 - by1
                        by2_adj = int(by1 + bh * 0.45)
                        cv2.rectangle(drawn, (bx1, by1),
                                      (bx2, by2_adj), (0, 255, 0), 2)
                        cv2.putText(drawn, f"{tracker.best_conf:.3f} {text}",
                                    (bx1, by1 - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.imwrite("ocr_result.png", drawn)
                    tracker.reset()

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        detector.release()


if __name__ == "__main__":
    main()

