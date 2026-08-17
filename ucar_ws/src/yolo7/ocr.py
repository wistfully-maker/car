#!/usr/bin/env python3
"""
OCR 文字识别 — RKNN NPU（yolo7 自包含模块）
============================================
不改 RapidOCR 官方管线，用 RKNN NPU 替换 ONNX Runtime 推理。
与 rapidocr_rknn/ocr_infer.py 功能相同，但：
  - 无模块级永久 fd 重定向（不会破坏 stdout/stderr）
  - 每次 RKNN 初始化临时抑制 C 库日志，完成后恢复
  - 模型在 ./models/ 目录下，完全自包含

用法:
    from ocr import RapidOcrRKNN
    ocr = RapidOcrRKNN()
    text, conf = ocr.predict(frame)       # 整帧（含镜像 flip）
    text, conf = ocr.predict_crop(crop)   # 裁剪图（不镜像）
"""

import os
import sys
import numpy as np

# ============================================================
# 路径 & 环境配置
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(SCRIPT_DIR, "models")

# 环境变量防御（C 库可能检查这些）
for _env in ("RKNN_LOG_LEVEL", "RKNN_VERBOSE", "RKNN_LOG"):
    os.environ.setdefault(_env, "0")

def _dbg(msg):
    """绕过 fd 重定向的 debug 输出 — 直接写文件"""
    try:
        with open(os.path.join(SCRIPT_DIR, "ocr_debug.log"), "a") as f:
            f.write(msg + "\n")
    except Exception:
        pass


# ============================================================
# fd 保存/恢复 — 抑制 librknnrt C 库日志
# ============================================================
class _FDSilence:
    """上下文管理器：临时重定向 stdout+stderr 到 /dev/null。

    librknnrt.so 的初始化日志直接 write(fd=1, ...)，绕过 Python。
    在 load_rknn / init_runtime 期间使用此类抑制日志。
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
        os.dup2(self._saved_1, 1)
        os.dup2(self._saved_2, 2)
        os.close(self._saved_1)
        os.close(self._saved_2)
        return False


# ============================================================
# RKNN 推理会话 — 替换 rapidocr_onnxruntime.utils.OrtInferSession
# ============================================================
class _RKNNInferSession:
    """RKNN NPU 推理（失败回退 ONNX Runtime）"""

    NAME_MAP = {
        "ch_PP-OCRv4_det_infer.onnx":          "det",
        "ch_ppocr_mobile_v2.0_cls_infer.onnx": "cls",
        "ch_PP-OCRv4_rec_infer.onnx":          "rec",
    }

    FIXED_SHAPES = {
        "det": (1, 3, 1088, 1920),
        "cls": (1, 3, 48, 192),
        "rec": (1, 3, 48, 512),
    }

    def __init__(self, config):
        self.model_path = config.get("model_path", "")
        model_name = os.path.basename(self.model_path)
        self.model_key = self.NAME_MAP.get(model_name,
                                           model_name.replace(".onnx", ""))
        self.rknn_path = os.path.join(MODEL_DIR, f"{self.model_key}.rknn")
        self._expected_shape = self.FIXED_SHAPES.get(self.model_key)
        self._onnx_path = None
        self._meta_dict = None

        self._rknn = None
        self._ort_session = None
        self._use_rknn = False
        self._init()

    # ----------------------------------------------------------
    # ONNX 路径解析 & 元数据（仅用于读取字符集）
    # ----------------------------------------------------------
    def _find_onnx(self):
        if os.path.exists(self.model_path):
            return self.model_path
        try:
            import rapidocr_onnxruntime as r
            d = os.path.join(os.path.dirname(r.__file__), "models")
            for onnx_name, key in self.NAME_MAP.items():
                if key == self.model_key:
                    p = os.path.join(d, onnx_name)
                    if os.path.exists(p):
                        return p
        except ImportError:
            pass
        for onnx_name, key in self.NAME_MAP.items():
            if key == self.model_key:
                p = os.path.join(MODEL_DIR, onnx_name)
                if os.path.exists(p):
                    return p
        return None

    def _load_meta(self):
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

    # ----------------------------------------------------------
    # 初始化
    # ----------------------------------------------------------
    def _init(self):
        self._onnx_path = self._find_onnx()

        if os.path.exists(self.rknn_path):
            try:
                from rknnlite.api import RKNNLite
                rknn = RKNNLite()

                with _FDSilence():
                    if rknn.load_rknn(self.rknn_path) != 0:
                        raise RuntimeError("load_rknn failed")
                    ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
                    if ret != 0:
                        ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                        if ret != 0:
                            raise RuntimeError("init_runtime failed")

                self._rknn = rknn
                self._use_rknn = True
                self._load_meta()
                _dbg(f"[OCR:{self.model_key}] RKNN OK")
                return
            except ImportError:
                _dbg(f"[OCR:{self.model_key}] rknnlite import FAILED")
            except Exception as e:
                _dbg(f"[OCR:{self.model_key}] RKNN FAIL: {e}")

        # 回退 ONNX Runtime
        if self._onnx_path and os.path.exists(self._onnx_path):
            _dbg(f"[OCR:{self.model_key}] falling back to ONNX")
            import onnxruntime as ort
            opts = ort.SessionOptions()
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self._ort_session = ort.InferenceSession(
                self._onnx_path, opts, providers=["CPUExecutionProvider"])
            self._use_rknn = False
        else:
            _dbg(f"[OCR:{self.model_key}] NO MODEL FOUND: rknn={self.rknn_path}")
            raise FileNotFoundError(
                f"[OCR:{self.model_key}] No model: rknn={self.rknn_path}")

    # ----------------------------------------------------------
    # 推理入口
    # ----------------------------------------------------------
    def __call__(self, input_array):
        if not self._use_rknn:
            input_dict = {self._ort_session.get_inputs()[0].name: input_array}
            return self._ort_session.run(None, input_dict)

        if self.model_key == "det":
            _dbg(f"[OCR:{self.model_key}] infer det input={input_array.shape}")
            return self._infer_det(input_array)
        elif self.model_key == "cls":
            return self._infer_cls(input_array)
        elif self.model_key == "rec":
            return self._infer_rec(input_array)
        return self._rknn.inference(inputs=[input_array], data_format='nchw')

    # ----------------------------------------------------------
    # det: 自适应尺寸 + letterbox → 推理 → crop → resize 回原尺寸
    # ----------------------------------------------------------
    def _infer_det(self, arr):
        import cv2
        _, _, h, w = arr.shape
        _, _, fh, fw = self._expected_shape
        if h == fh and w == fw:
            return self._rknn.inference(inputs=[arr], data_format='nchw')

        nhwc = arr[0].transpose(1, 2, 0).astype(np.float32)
        scale = min(fw / w, fh / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(nhwc, (nw, nh), interpolation=cv2.INTER_LINEAR)
        dw, dh = fw - nw, fh - nh
        top, left = dh // 2, dw // 2
        padded = cv2.copyMakeBorder(resized, top, dh - top, left, dw - left,
                                    cv2.BORDER_CONSTANT, value=0)
        nchw = padded.transpose(2, 0, 1)[np.newaxis, ...]
        outputs = self._rknn.inference(inputs=[nchw], data_format='nchw')

        result = []
        for out in outputs:
            if out.ndim == 4:
                _, _, oh, ow = out.shape
                ot = int(oh * top / fh)
                ob = int(oh * (top + nh) / fh)
                ol = int(ow * left / fw)
                or_ = int(ow * (left + nw) / fw)
                cropped = out[:, :, ot:ob, ol:or_]
                onhwc = cropped[0].transpose(1, 2, 0)
                h_out = int(onhwc.shape[0] * h / nh)
                w_out = int(onhwc.shape[1] * w / nw)
                inp = onhwc[:, :, 0] if onhwc.shape[2] == 1 else onhwc
                out_r = cv2.resize(inp, (w_out, h_out),
                                   interpolation=cv2.INTER_LINEAR)
                if out_r.ndim == 2:
                    out_r = out_r[:, :, np.newaxis]
                result.append(
                    out_r.transpose(2, 0, 1)[np.newaxis, ...].astype(out.dtype))
            else:
                result.append(out)
        return result

    # ----------------------------------------------------------
    # cls: 拆 batch 逐个推理 → stack
    # ----------------------------------------------------------
    def _infer_cls(self, arr):
        if arr.shape[0] == 1:
            return self._rknn.inference(inputs=[arr], data_format='nchw')
        outputs = [
            self._rknn.inference(
                inputs=[arr[i:i+1].astype(np.float32)], data_format='nchw'
            )[0]
            for i in range(arr.shape[0])
        ]
        return [np.concatenate(outputs, axis=0)]

    # ----------------------------------------------------------
    # rec: 拆 batch + pad/resize W→512 → 推理 → stack
    # ----------------------------------------------------------
    def _infer_rec(self, arr):
        import cv2
        _, _, fh, fw = self._expected_shape
        outputs = []
        for i in range(arr.shape[0]):
            inp = arr[i:i+1]
            _, _, _, wi = inp.shape
            if wi != fw:
                nhwc = inp[0].transpose(1, 2, 0)
                if wi < fw:
                    pad = np.zeros((fh, fw - wi, 3), dtype=np.float32)
                    nhwc = np.concatenate([nhwc, pad], axis=1)
                else:
                    nhwc = cv2.resize(nhwc, (fw, fh),
                                      interpolation=cv2.INTER_LINEAR)
                inp = nhwc.transpose(2, 0, 1)[np.newaxis, ...]
            out = self._rknn.inference(
                inputs=[inp.astype(np.float32)], data_format='nchw')
            outputs.append(out[0])
        return [np.concatenate(outputs, axis=0)]

    # ----------------------------------------------------------
    # 字符集
    # ----------------------------------------------------------
    def have_key(self, key="character"):
        if not self._use_rknn:
            meta = self._ort_session.get_modelmeta().custom_metadata_map
            return key in meta
        self._load_meta()
        if self._meta_dict and key in self._meta_dict:
            return True
        if key == "character":
            return os.path.exists(os.path.join(MODEL_DIR, "characters.txt"))
        return False

    def get_character_list(self, key="character"):
        if not self._use_rknn:
            meta = self._ort_session.get_modelmeta().custom_metadata_map
            if key in meta:
                return meta[key].splitlines()
            return None
        self._load_meta()
        if self._meta_dict and key in self._meta_dict:
            return self._meta_dict[key].splitlines()
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

    utils.OrtInferSession = _RKNNInferSession
    det_mod.OrtInferSession = _RKNNInferSession
    rec_mod.OrtInferSession = _RKNNInferSession
    cls_mod.OrtInferSession = _RKNNInferSession


# ============================================================
# 对外接口
# ============================================================
class RapidOcrRKNN:
    """RKNN NPU OCR — 车间名识别。

    用法:
        ocr = RapidOcrRKNN()
        text, conf = ocr.predict(frame)       # 整帧（含镜像）
        text, conf = ocr.predict_crop(crop)   # 裁剪图（不镜像）
    """

    VALID = {
        "电子产品加工车间": ["电", "子"],
        "食品加工车间":     ["食"],
        "日用品加工车间":   ["日", "用"],
    }

    def __init__(self):
        _inject_rknn()
        from rapidocr_onnxruntime import RapidOCR
        self.ocr = RapidOCR()
        self.text_score = 0.5

    def predict(self, frame):
        """整帧识别（含 cv2.flip 镜像）。返回 (text, confidence)"""
        import cv2
        frame = cv2.flip(frame, 1)
        return self._recognize(frame)

    def predict_crop(self, crop):
        """裁剪图识别（不镜像，避免文字反转）。返回 (text, confidence)"""
        return self._recognize(crop)

    def _recognize(self, img):
        """核心识别逻辑：RapidOCR → 关键词唯一匹配"""
        try:
            result = self.ocr(img)
        except Exception as e:
            import traceback
            print(f"[OCR-ERROR] RapidOCR crashed: {e}")
            traceback.print_exc()
            return "", 0.0

        if result is None:
            print("[OCR-DEBUG] RapidOCR returned None")
            return "", 0.0

        boxes, elapse = result

        if not boxes:
            print(f"[OCR-DEBUG] no boxes found, elapse={elapse}")
            return "", 0.0

        # DEBUG: 打印 OCR 检测到的所有文本
        texts = [(t, float(c)) for _, t, c in boxes if t]
        print(f"[OCR-DEBUG] detected texts: {texts}")

        for _, text, conf in sorted(boxes, key=lambda b: b[2], reverse=True):
            if not text or float(conf) < self.text_score:
                continue
            candidates = set()
            for full_name, keywords in self.VALID.items():
                for kw in keywords:
                    if kw in text:
                        candidates.add(full_name)
            if len(candidates) == 1:
                return candidates.pop(), float(conf)

        return "", 0.0

    def release(self):
        pass

