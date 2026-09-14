#!/usr/bin/env python3
"""
RapidOCR + RKNN NPU — 文字识别（小车端）
=========================================
完全不改 RapidOCR 官方管线，只把 ONNX Runtime 推理替换为 RKNN NPU。
接口兼容 home/rapidocr/infer.py 的 RapidOcrInfer。

用法:
    from ocr_infer import RapidOcrRKNN
    ocr = RapidOcrRKNN()
    text, conf = ocr.predict(frame)
"""

import os, sys, numpy as np

# 在任何 rknnlite 导入之前，禁掉 librknnrt C 库日志
for _env in ("RKNN_LOG_LEVEL", "RKNN_VERBOSE", "RKNN_LOG"):
    os.environ.setdefault(_env, "0")
# 永久重定向 stdout+stderr（C 库日志写 fd 1，不是 fd 2）
_devnull = os.open(os.devnull, os.O_WRONLY)
os.dup2(_devnull, 1)
os.dup2(_devnull, 2)
os.close(_devnull)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(SCRIPT_DIR, "models")


# ============================================================
# RKNN 推理会话 — 替换 rapidocr_onnxruntime.utils.OrtInferSession
# ============================================================
class _RKNNInferSession:
    """RKNN NPU 推理（失败回退 ONNX Runtime）— 替换 OrtInferSession"""

    NAME_MAP = {
        "ch_PP-OCRv4_det_infer.onnx":    "det",
        "ch_ppocr_mobile_v2.0_cls_infer.onnx":   "cls",
        "ch_PP-OCRv4_rec_infer.onnx":    "rec",
    }

    # 各模型 RKNN 固定输入 shape (NCHW)
    FIXED_SHAPES = {
        "det": (1, 3, 1088, 1920),
        "cls": (1, 3, 48, 192),
        "rec": (1, 3, 48, 512),
    }

    def __init__(self, config):
        self.model_path = config.get("model_path", "")
        model_name = os.path.basename(self.model_path)
        self.model_key = self.NAME_MAP.get(model_name, model_name.replace(".onnx", ""))
        self.rknn_path = os.path.join(MODEL_DIR, f"{self.model_key}.rknn")
        self._expected_shape = self.FIXED_SHAPES.get(self.model_key)
        self._onnx_path = None  # resolved ONNX path (for metadata reading)
        self._meta_dict = None

        self._rknn = None
        self._ort_session = None
        self._use_rknn = False
        self._init()

    def _find_onnx(self):
        """解析 ONNX 模型路径"""
        # 1) config 中的路径
        if os.path.exists(self.model_path):
            return self.model_path
        # 2) rapidocr 包内
        try:
            import rapidocr_onnxruntime as r
            models_dir = os.path.join(os.path.dirname(r.__file__), "models")
            for onnx_name, key in self.NAME_MAP.items():
                if key == self.model_key:
                    candidate = os.path.join(models_dir, onnx_name)
                    if os.path.exists(candidate):
                        return candidate
        except ImportError:
            pass
        # 3) models 目录
        for onnx_name, key in self.NAME_MAP.items():
            if key == self.model_key:
                candidate = os.path.join(MODEL_DIR, onnx_name)
                if os.path.exists(candidate):
                    return candidate
        return None

    def _load_meta(self):
        """从 ONNX 模型读取元数据（字符集等）"""
        if self._meta_dict is not None:
            return
        onnx_path = self._onnx_path or self._find_onnx()
        if onnx_path and os.path.exists(onnx_path):
            try:
                import onnx
                m = onnx.load(onnx_path)
                self._meta_dict = {}
                for prop in m.metadata_props:
                    self._meta_dict[prop.key] = prop.value
            except Exception:
                self._meta_dict = {}

    def _init(self):
        self._onnx_path = self._find_onnx()

        # 优先 RKNN
        if os.path.exists(self.rknn_path):
            try:
                from rknnlite.api import RKNNLite
                rknn = RKNNLite()

                if rknn.load_rknn(self.rknn_path) != 0:
                    raise RuntimeError("load_rknn failed")
                ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
                if ret != 0:
                    ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                    if ret != 0:
                        raise RuntimeError("init_runtime failed")

                self._rknn = rknn
                self._use_rknn = True
                # 预加载 ONNX 元数据
                self._load_meta()
                print(f"[OCR:{self.model_key}] RKNN NPU ready")
                return
            except ImportError:
                pass
            except Exception as e:
                print(f"[OCR:{self.model_key}] RKNN failed: {e}")

        # 回退 ONNX Runtime
        if self._onnx_path and os.path.exists(self._onnx_path):
            import onnxruntime as ort
            opts = ort.SessionOptions()
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self._ort_session = ort.InferenceSession(
                self._onnx_path, opts, providers=["CPUExecutionProvider"])
            self._use_rknn = False
            print(f"[OCR:{self.model_key}] ONNX fallback")
        else:
            raise FileNotFoundError(
                f"[OCR:{self.model_key}] No model found: rknn={self.rknn_path}")

    def __call__(self, input_array):
        if not self._use_rknn:
            input_dict = {self._ort_session.get_inputs()[0].name: input_array}
            return self._ort_session.run(None, input_dict)

        # RKNN 路径 — 自适应输入 shape
        if self.model_key == "det":
            return self._infer_det(input_array)
        elif self.model_key == "cls":
            return self._infer_cls(input_array)
        elif self.model_key == "rec":
            return self._infer_rec(input_array)
        return self._rknn.inference(inputs=[input_array], data_format='nchw')

    def _infer_det(self, arr):
        """det: [1,3,H,W] → 推理 → 返回 NCHW 输出"""
        import cv2
        _, _, h, w = arr.shape
        _, _, fh, fw = self._expected_shape
        if h == fh and w == fw:
            return self._rknn.inference(inputs=[arr], data_format='nchw')
        nhwc = arr[0].transpose(1, 2, 0).astype(np.float32)       # [H, W, 3]
        # letterbox: 保持宽高比，pad 到 640×640
        scale = min(fw / w, fh / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(nhwc, (nw, nh), interpolation=cv2.INTER_LINEAR)
        dw, dh = fw - nw, fh - nh
        top, left = dh // 2, dw // 2
        padded = cv2.copyMakeBorder(resized, top, dh - top, left, dw - left,
                                    cv2.BORDER_CONSTANT, value=0)   # [640, 640, 3]
        nchw = padded.transpose(2, 0, 1)[np.newaxis, ...]          # [1, 3, 640, 640]
        outputs = self._rknn.inference(inputs=[nchw], data_format='nchw')
        # 裁掉输出中的 pad 区域，再 resize 回原始尺寸
        result = []
        for out in outputs:
            if out.ndim == 4:  # [1, C, Ho, Wo]
                _, _, oh, ow = out.shape
                ot = int(oh * top / fh)
                ob = int(oh * (top + nh) / fh)
                ol = int(ow * left / fw)
                or_ = int(ow * (left + nw) / fw)
                cropped = out[:, :, ot:ob, ol:or_]                  # [1, C, nh_out, nw_out]
                onhwc = cropped[0].transpose(1, 2, 0)               # [nh_out, nw_out, C]
                h_out = int(onhwc.shape[0] * h / nh)
                w_out = int(onhwc.shape[1] * w / nw)
                single_chan = (onhwc.shape[2] == 1)
                inp = onhwc[:, :, 0] if single_chan else onhwc
                out_r = cv2.resize(inp, (w_out, h_out), interpolation=cv2.INTER_LINEAR)
                if out_r.ndim == 2:
                    out_r = out_r[:, :, np.newaxis]
                result.append(out_r.transpose(2, 0, 1)[np.newaxis, ...].astype(out.dtype))
            else:
                result.append(out)
        return result

    def _infer_cls(self, arr):
        """cls: [B,3,48,192] → 拆 batch 逐个推理 → stack"""
        if arr.shape[0] == 1:
            return self._rknn.inference(inputs=[arr], data_format='nchw')
        outputs = [self._rknn.inference(inputs=[arr[i:i+1].astype(np.float32)], data_format='nchw')[0]
                   for i in range(arr.shape[0])]
        return [np.concatenate(outputs, axis=0)]

    def _infer_rec(self, arr):
        """rec: [B,3,48,W] → 拆 batch + pad/resize W→512 → 推理 → stack"""
        import cv2
        _, _, fh, fw = self._expected_shape  # (1,3,48,512)
        outputs = []
        for i in range(arr.shape[0]):
            inp = arr[i:i+1]  # [1, 3, 48, W]
            _, _, _, wi = inp.shape
            if wi != fw:
                nhwc = inp[0].transpose(1, 2, 0)                  # [48, W, 3]
                if wi < fw:
                    pad = np.zeros((fh, fw - wi, 3), dtype=np.float32)
                    nhwc = np.concatenate([nhwc, pad], axis=1)     # [48, 512, 3]
                else:
                    nhwc = cv2.resize(nhwc, (fw, fh), interpolation=cv2.INTER_LINEAR)
                inp = nhwc.transpose(2, 0, 1)[np.newaxis, ...]     # [1, 3, 48, 512]
            out = self._rknn.inference(inputs=[inp.astype(np.float32)], data_format='nchw')
            outputs.append(out[0])  # [1, T, num_classes]
        return [np.concatenate(outputs, axis=0)]

    def have_key(self, key: str = "character") -> bool:
        """检查 ONNX 元数据中是否有指定 key"""
        if not self._use_rknn:
            meta = self._ort_session.get_modelmeta().custom_metadata_map
            return key in meta
        # RKNN 模式
        self._load_meta()
        if self._meta_dict and key in self._meta_dict:
            return True
        # 回退：characters.txt
        if key == "character":
            return os.path.exists(os.path.join(MODEL_DIR, "characters.txt"))
        return False

    def get_character_list(self, key: str = "character") -> list:
        """获取字符集列表"""
        if not self._use_rknn:
            meta = self._ort_session.get_modelmeta().custom_metadata_map
            if key in meta:
                return meta[key].splitlines()
            return None
        # RKNN 模式：优先从 ONNX 元数据
        self._load_meta()
        if self._meta_dict and key in self._meta_dict:
            return self._meta_dict[key].splitlines()
        # 回退：从 characters.txt（export_ocr_rknn.py 导出时生成）
        chars_path = os.path.join(MODEL_DIR, "characters.txt")
        if os.path.exists(chars_path):
            with open(chars_path, "r", encoding="utf-8") as f:
                return f.read().splitlines()
        return None

    def release(self):
        if self._use_rknn and self._rknn:
            self._rknn.release()


# ============================================================
# 注入 RKNN Session → 替换官方 OrtInferSession
# ============================================================
def _inject_rknn():
    """用 _RKNNInferSession 替换 rapidocr_onnxruntime 的 OrtInferSession"""
    import rapidocr_onnxruntime.utils as utils
    import rapidocr_onnxruntime.ch_ppocr_det.text_detect as det_mod
    import rapidocr_onnxruntime.ch_ppocr_rec.text_recognize as rec_mod
    import rapidocr_onnxruntime.ch_ppocr_cls.text_cls as cls_mod

    # 替换 utils 模块中的引用
    utils.OrtInferSession = _RKNNInferSession
    # 替换各个子模块中的导入引用（它们 import 时用的是 from ... import OrtInferSession）
    det_mod.OrtInferSession = _RKNNInferSession
    rec_mod.OrtInferSession = _RKNNInferSession
    cls_mod.OrtInferSession = _RKNNInferSession

    print("[RKNN] Injected _RKNNInferSession into rapidocr_onnxruntime")


# ============================================================
# 对外接口 — 和 home/rapidocr/infer.py 完全一致
# ============================================================
class RapidOcrRKNN:
    """RKNN NPU OCR，接口兼容 RapidOcrInfer"""

    def __init__(self):
        # 注入 RKNN
        _inject_rknn()

        # 现在导入 RapidOCR，它会使用我们的 _RKNNInferSession
        from rapidocr_onnxruntime import RapidOCR
        self.ocr = RapidOCR()
        self.text_score = 0.5
        print("[RapidOCR RKNN] Ready")

    def predict(self, frame):
        """识别车间名。返回 (text, confidence)"""
        import cv2

        # 镜像 → 正向（与原版保持一致）
        frame = cv2.flip(frame, 1)

        result = self.ocr(frame)
        boxes, _ = result

        if not boxes:
            return "", 0.0

        # 关键词唯一匹配 → 遍历所有框，找第一个唯一命中
        VALID = {
            "电子产品加工车间": ["电", "子"],
            "食品加工车间":     ["食"],
            "日用品加工车间":   ["日", "用"],
        }

        for _, text, conf in sorted(boxes, key=lambda b: b[2], reverse=True):
            if not text or float(conf) < self.text_score:
                continue
            candidates = set()
            for full, keywords in VALID.items():
                for kw in keywords:
                    if kw in text:
                        candidates.add(full)
            if len(candidates) == 1:
                return candidates.pop(), float(conf)

        return "", 0.0

    def release(self):
        pass


# ============================================================
# 测试入口
# ============================================================
def main():
    import argparse, cv2
    parser = argparse.ArgumentParser(description="RapidOCR RKNN")
    parser.add_argument("--image", default=None)
    parser.add_argument("--camera", action="store_true")
    args = parser.parse_args()

    print("=" * 50)
    print("  RapidOCR + RKNN NPU")
    print("=" * 50)

    ocr = RapidOcrRKNN()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            with open(args.image, "rb") as f:
                data = f.read()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[ERROR] Cannot read: {args.image}")
            return
        text, conf = ocr.predict(img)
        print(f"Result: \"{text}\" conf={conf:.4f}")
    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            print("[ERROR] No camera")
            return
        print("\n[OCR] Running... Ctrl+C to stop\n")
        last = ""
        try:
            while True:
                ret, frame = cap.read()
                if not ret: continue
                text, conf = ocr.predict(frame)
                if text and text != last:
                    last = text
                    print(f"  [OK] {text} ({conf:.3f})")
        except KeyboardInterrupt:
            pass
        cap.release()


if __name__ == "__main__":
    main()

