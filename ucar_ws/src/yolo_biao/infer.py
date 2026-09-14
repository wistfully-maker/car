#!/usr/bin/env python3
"""
YOLO26 门牌检测 — 小车端 RKNN NPU (yolo_biao)
==============================================
1024×1024 高精度模型，区分墙上标牌 vs 地面白线。
推理+推流并行：NPU 后台检测，摄像头帧率实时出流。

用法:
    python3 infer.py                    # 摄像头实时检测 + MJPEG 流
    python3 infer.py --image test.jpg   # 单张图片

浏览器:
    http://<小车IP>:8080/               # HTML 页面（视频流 + 状态）
    http://<小车IP>:8080/stream         # 纯 MJPEG 流
"""

import os, sys, argparse, time, threading, json, signal
import numpy as np

# ============================================================
# 全局变量 — MJPEG 推流
# ============================================================
_latest_jpg = None
_lock = threading.Lock()
_stream_status = {"det": "-", "fps": 0, "conf": 0.0}
_running = True                          # Ctrl+C 信号标志


def _stream_server(port=8080):
    """HTTP + MJPEG 服务器（纯标准库）"""
    from http.server import BaseHTTPRequestHandler
    from socketserver import ThreadingMixIn, TCPServer

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                while True:
                    with _lock:
                        jpg = _latest_jpg
                    if jpg is None:
                        time.sleep(0.01)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        break
            elif self.path == "/status":
                payload = json.dumps(_stream_status, ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            elif self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(_HTML_PAGE.encode())
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, *args):
            pass

    class Server(ThreadingMixIn, TCPServer):
        allow_reuse_address = True
        daemon_threads = True

    srv = Server(("0.0.0.0", port), Handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv


_HTML_PAGE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>YOLO 标牌检测</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0a;color:#ccc;font-family:'Segoe UI',system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;min-height:100vh}
h1{margin:16px 0 4px;font-size:1.4em;font-weight:400;color:#0f0;letter-spacing:2px}
#status{display:flex;gap:20px;margin:8px 0 12px;font-size:15px;flex-wrap:wrap;justify-content:center}
#status .card{padding:6px 16px;border-radius:8px;background:#111;border:1px solid #333;min-width:100px;text-align:center}
#status .label{font-size:11px;color:#888;text-transform:uppercase}
#status .value{font-size:18px;font-weight:600;margin-top:2px}
#status .on{color:#0f0}
#status .off{color:#f44}
img{max-width:98vw;max-height:70vh;border:2px solid #1a1a1a;border-radius:4px}
</style>
</head>
<body>
<h1>🔍 YOLO 标牌检测 — 1024×1024</h1>
<div id="status">
  <div class="card"><div class="label">检测</div><div class="value" id="det">-</div></div>
  <div class="card"><div class="label">FPS</div><div class="value" id="fps" style="color:#09f">0</div></div>
  <div class="card"><div class="label">置信度</div><div class="value" id="conf" style="color:#ff0">0</div></div>
</div>
<img id="live" src="/stream" alt="live">
<script>
(async()=>{
  async function poll(){try{
    const r=await fetch('/status');const j=await r.json();
    const det=document.getElementById('det');det.textContent=j.det;
    det.className='value '+(j.det==='-'?'off':'on');
    document.getElementById('fps').textContent=j.fps;
    document.getElementById('conf').textContent=j.conf.toFixed(2);
  }catch(e){}setTimeout(poll,400);}
  poll();
})();
</script>
</body>
</html>"""


# ============================================================
# 路径配置
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RKNN_MODEL = os.path.join(SCRIPT_DIR, "best.rknn")
ONNX_MODEL = os.path.join(SCRIPT_DIR, "best.onnx")
DEFAULT_IMGSZ = 1024
DEFAULT_CONF = 0.35
IOU_THRESH = 0.45


# ============================================================
# fd 静默 — 抑制 librknnrt C 库日志
# ============================================================
class _FDSilence:
    def __enter__(self):
        self._devnull = os.open(os.devnull, os.O_WRONLY)
        self._s1, self._s2 = os.dup(1), os.dup(2)
        os.dup2(self._devnull, 1)
        os.dup2(self._devnull, 2)
        os.close(self._devnull)
        return self
    def __exit__(self, *a):
        os.dup2(self._s1, 1); os.dup2(self._s2, 2)
        os.close(self._s1); os.close(self._s2)
        return False


# ============================================================
# YOLO26 门牌检测器 — 1024×1024 RKNN
# ============================================================
class YoloDetector:
    def __init__(self, model_path=None, imgsz=DEFAULT_IMGSZ, conf_thresh=DEFAULT_CONF):
        self.model_path = model_path or RKNN_MODEL
        self.imgsz = imgsz
        self.conf_thresh = conf_thresh
        self._rknn = None
        self._use_rknn = False
        self._init()

    def _init(self):
        if os.path.exists(self.model_path):
            try:
                from rknnlite.api import RKNNLite
                rknn = RKNNLite()
                with _FDSilence():
                    if rknn.load_rknn(self.model_path) != 0:
                        raise RuntimeError("load_rknn failed")
                    ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
                    if ret != 0:
                        ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                        if ret != 0:
                            raise RuntimeError("init_runtime failed")
                self._rknn = rknn
                self._use_rknn = True
                print(f"[YOLO] RKNN NPU ready | imgsz={self.imgsz}")
                return
            except ImportError:
                pass
            except Exception as e:
                print(f"[YOLO] RKNN init failed: {e}")

        # ONNX fallback
        if os.path.exists(ONNX_MODEL):
            import onnxruntime as ort
            opts = ort.SessionOptions()
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self._ort = ort.InferenceSession(ONNX_MODEL, opts,
                                              providers=["CPUExecutionProvider"])
            self._ort_in = self._ort.get_inputs()[0].name
            self._ort_out = [o.name for o in self._ort.get_outputs()]
            self._use_rknn = False
            print("[YOLO] ONNX Runtime fallback")
        else:
            raise FileNotFoundError(f"无模型: {self.model_path} / {ONNX_MODEL}")

    def _preprocess(self, frame):
        import cv2
        img = cv2.resize(frame, (self.imgsz, self.imgsz))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)
        img = np.expand_dims(img, axis=0)
        return np.ascontiguousarray(img)

    def _postprocess(self, outputs, fw, fh):
        if outputs is None:
            return []
        output = outputs[0] if isinstance(outputs, list) else outputs
        if output is None:
            return []
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
            c = float(p[4])
            x1 = (cx - w / 2) / self.imgsz * fw
            y1 = (cy - h / 2) / self.imgsz * fh
            x2 = (cx + w / 2) / self.imgsz * fw
            y2 = (cy + h / 2) / self.imgsz * fh
            boxes.append([max(0, x1), max(0, y1),
                          min(fw, x2), min(fh, y2), c])
        if len(boxes) <= 1:
            return boxes
        boxes_np = np.array(boxes)
        idxs = np.argsort(boxes_np[:, 4])[::-1]
        keep = []
        while len(idxs) > 0:
            keep.append(idxs[0])
            if len(idxs) == 1:
                break
            iou = self._iou(boxes_np[idxs[0]], boxes_np[idxs[1:]])
            idxs = idxs[1:][iou < IOU_THRESH]
        return boxes_np[keep].tolist()

    @staticmethod
    def _iou(box, boxes):
        x1 = np.maximum(box[0], boxes[:, 0]); y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2]); y2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        a1 = (box[2] - box[0]) * (box[3] - box[1])
        a2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        return inter / (a1 + a2 - inter + 1e-6)

    def predict(self, frame):
        h, w = frame.shape[:2]
        tensor = self._preprocess(frame)
        if self._use_rknn:
            outputs = self._rknn.inference(inputs=[tensor])
        else:
            outputs = self._ort.run(self._ort_out, {self._ort_in: tensor})
        return self._postprocess(outputs, w, h)

    def release(self):
        if self._use_rknn and self._rknn:
            self._rknn.release()


# ============================================================
# 主管线
# ============================================================
def main():
    p = argparse.ArgumentParser(description="yolo_biao — 1024×1024 标牌检测")
    p.add_argument("--model", default=RKNN_MODEL, help="RKNN 模型路径")
    p.add_argument("--image", default=None, help="单张图片")
    p.add_argument("--no-stream", action="store_true", help="禁用 MJPEG 流")
    p.add_argument("--port", type=int, default=8080, help="MJPEG 流端口")
    p.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    p.add_argument("--conf", type=float, default=DEFAULT_CONF)
    args = p.parse_args()

    global _stream_status

    # ---- 初始化 YOLO ----
    detector = YoloDetector(model_path=args.model,
                            imgsz=args.imgsz, conf_thresh=args.conf)

    # ---- 单张图片 ----
    if args.image:
        import cv2
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[ERROR] Cannot read: {args.image}")
            sys.exit(1)
        dets = detector.predict(frame)
        if dets:
            best = max(dets, key=lambda d: d[4])
            x1, y1, x2, y2, conf = best
            print(f"signboard conf={conf:.3f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
        else:
            print("(no signboard)")
        detector.release()
        return

    # ---- 摄像头实时 ----
    import cv2
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # 只保留最新帧，消除摄像头缓存延迟
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 手动曝光 (V4L2)
    cap.set(cv2.CAP_PROP_EXPOSURE, 20)        # 2ms, 太暗调大/太亮调小

    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        detector.release()
        sys.exit(1)

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[CAM] {actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.0f}fps")

    # ---- NPU 预热：第一帧推理有冷启动开销，提前跑掉 ----
    ret, wf = cap.read()
    if ret and wf is not None:
        detector.predict(wf)
        print("[WARMUP] NPU 预热完成")

    # ---- 启动 MJPEG 流 ----
    server = None
    if not args.no_stream:
        server = _stream_server(args.port)
        import socket
        ip = "???"
        try:
            # 获取非 loopback IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
        except:
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except:
                pass
        print(f"\n🌐 浏览器: http://{ip}:{args.port}")
        print(f"   推流:   http://{ip}:{args.port}/stream")
    print("=" * 55)
    print("  yolo_biao — 1024×1024 标牌检测")
    print("  Ctrl+C 停止")
    print("=" * 55)

    global _latest_jpg, _running, _stream_status

    # Ctrl+C 信号 → 优雅退出
    def _on_sigint(sig, frame):
        global _running
        _running = False
        print("\n[STOP] 正在退出...")
    signal.signal(signal.SIGINT, _on_sigint)

    last_det_str = ""
    try:
        while _running:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            t0 = time.time()
            dets = detector.predict(frame)
            dt = time.time() - t0
            _stream_status["fps"] = round(1.0 / (dt + 1e-6), 1)

            if dets:
                best_det = max(dets, key=lambda d: d[4])
                x1, y1, x2, y2, conf = best_det
                s = f"signboard conf={conf:.3f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]"
                if s != last_det_str:
                    print(s)
                    last_det_str = s
                _stream_status["det"] = f"conf={conf:.3f}"
                _stream_status["conf"] = round(conf, 3)

                bx1, by1, bx2, by2 = int(x1), int(y1), int(x2), int(y2)
                bh = by2 - by1
                by2_adj = int(by1 + bh * 0.45)
                cv2.rectangle(frame, (bx1, by1), (bx2, by2_adj), (0, 255, 0), 2)
                cv2.putText(frame, f"{conf:.3f}", (bx1, by1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                if conf > 0.9:
                    cv2.imwrite("yolo_detect.png", frame)
            else:
                if last_det_str:
                    last_det_str = ""
                _stream_status["det"] = "-"
                _stream_status["conf"] = 0.0

            if not args.no_stream:
                _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                with _lock:
                    _latest_jpg = jpg.tobytes()

            # 清空摄像头缓冲：推理+编码期间积累的旧帧全部丢弃
            cap.grab()

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        _running = False
        cap.release()
        detector.release()
        if server is not None:
            server.shutdown()


if __name__ == "__main__":
    main()

