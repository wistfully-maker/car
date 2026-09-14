#!/usr/bin/env python3
"""
PP-OCR RKNN — 车间名文字识别（单文件自包含，零外部 OCR 依赖）
==============================================================
使用 PP-OCRv4 模型（DBNet 检测 + CRNN 识别），RKNN NPU 推理。
约束 CTC 解码仅限 11 个车间名字符，结果只输出 3 个车间名。

依赖: numpy, opencv-python, rknnlite (小车端)
      零依赖 rapidocr_onnxruntime / paddleocr / onnxruntime

用法:
    from infer import WorkshopOCR
    ocr = WorkshopOCR()
    text, conf = ocr.predict(frame)        # 整帧（含镜像）
    text, conf = ocr.predict_crop(roi)     # ROI（不镜像）

部署:
    infer.py 同级目录放:
        det.rknn    # DBNet 文字检测 (1,3,1088,1920)
        rec.rknn    # CRNN 文字识别 (1,3,48,512)
"""

import os, sys, numpy as np

# ── 环境变量防御（RkNN C 库可能检查） ──
for _env in ("RKNN_LOG_LEVEL", "RKNN_VERBOSE", "RKNN_LOG"):
    os.environ.setdefault(_env, "0")

# ── 路径 ──
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── 模型固定输入 ──
DET_SHAPE = (1, 3, 1088, 1920)   # NCHW, H=1088 W=1920
REC_SHAPE = (1, 3, 48, 512)      # NCHW, H=48 W=512

# ── 预处理参数（与 PP-OCRv4 一致） ──
DET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
DET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
REC_MEAN = np.array([0.5, 0.5, 0.5], dtype=np.float32)
REC_STD  = np.array([0.5, 0.5, 0.5], dtype=np.float32)

# ── 11 个车间名有效字符 ──
TARGET_CHARS = ["产", "加", "品", "子", "工", "日", "用", "电", "车", "间", "食"]

# 索引在 _init_charmap() 中从 characters.txt 加载
_VALID_INDICES = None
_ID_TO_CHAR = None


def _init_charmap():
    """从 characters.txt 加载 11 个目标字符在 6622 字符集中的索引。

    characters.txt 与 rec.rknn 配对（同一次 ONNX→RKNN 转换时导出），
    字符顺序严格一致。如果文件缺失，回退到硬编码索引。
    """
    global _VALID_INDICES, _ID_TO_CHAR

    chars_path = os.path.join(SCRIPT_DIR, "characters.txt")
    if os.path.exists(chars_path):
        with open(chars_path, "r", encoding="utf-8") as f:
            full_chars = f.read().splitlines()
        # full_chars[0] 是 blank 占位符 (')
        indices = [0]  # blank
        id_to_char = ["[blank]"]
        for c in TARGET_CHARS:
            try:
                idx = full_chars.index(c)
                indices.append(idx)
                id_to_char.append(c)
            except ValueError:
                raise ValueError(
                    f"字符 '{c}' 不在 characters.txt 中，"
                    "characters.txt 可能与 rec.rknn 不匹配")
        _VALID_INDICES = indices
        _ID_TO_CHAR = id_to_char
    else:
        # ── 硬编码回退（与 rec.rknn 配对的标准 PP-OCRv4 字符集） ──
        _fallback = [
            ("产",   96), ("加",   12), ("品", 1134), ("子",  737),
            ("工",  350), ("日",  131), ("用",  155), ("电",  244),
            ("车", 2027), ("间",  200), ("食",  336),
        ]
        _VALID_INDICES = [0] + [idx for _, idx in _fallback]
        _ID_TO_CHAR = ["[blank]"] + [c for c, _ in _fallback]


# 模块加载时初始化
_init_charmap()

# ── 车间名匹配规则 ──
VALID_WORKSHOPS = {
    "电子产品加工车间": ["电", "子"],
    "食品加工车间":     ["食"],
    "日用品加工车间":   ["日", "用"],
}


# ╔══════════════════════════════════════════════════════════╗
# ║  fd 抑制 — 抑制 librknnrt C 库直接写 fd 的日志          ║
# ╚══════════════════════════════════════════════════════════╝
class _FDSilence:
    """上下文管理器：临时重定向 fd 1,2 → /dev/null（初始化用）。"""
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


def _rknn_infer(rknn, tensor):
    """rknn.inference() 包装：抑制 NHWC 警告 + 防 Ctrl+C 崩溃"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved1 = os.dup(1)
    saved2 = os.dup(2)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        return rknn.inference(inputs=[tensor], data_format="nchw")
    finally:
        os.dup2(saved1, 1)
        os.dup2(saved2, 2)
        os.close(saved1)
        os.close(saved2)


# ╔══════════════════════════════════════════════════════════╗
# ║  DetInfer — DBNet 文字检测                              ║
# ╚══════════════════════════════════════════════════════════╝
class DetInfer:
    """PP-OCRv4 DBNet 文字检测 → RKNN NPU"""

    def __init__(self, model_path=None):
        self.model_path = model_path or os.path.join(SCRIPT_DIR, "det.rknn")
        self.det_thresh = 0.15      # 概率图二值化阈值（FP16 下调低）
        self.box_thresh = 0.05      # box 平均置信度下限（FP16 下调低）
        self.min_area = 40          # 最小轮廓面积
        self._rknn = None
        self._init()

    def _init(self):
        from rknnlite.api import RKNNLite
        rknn = RKNNLite()
        with _FDSilence():
            if rknn.load_rknn(self.model_path) != 0:
                raise RuntimeError(f"det RKNN 加载失败: {self.model_path}")
            ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if ret != 0:
                ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                if ret != 0:
                    raise RuntimeError("det init_runtime 失败")
        self._rknn = rknn

    def preprocess(self, img):
        """BGR (H,W,3) → (1,3,1088,1920) NCHW, letterbox + normalize"""
        import cv2
        h, w = img.shape[:2]
        _, _, fh, fw = DET_SHAPE

        # letterbox: 保持宽高比, pad 到 (fh, fw)
        scale = min(fw / w, fh / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        padded = np.zeros((fh, fw, 3), dtype=np.float32)
        top = (fh - nh) // 2
        left = (fw - nw) // 2
        padded[top:top+nh, left:left+nw] = resized

        # normalize: (BGR/255 - mean) / std
        padded = padded / 255.0
        padded = (padded - DET_MEAN) / DET_STD

        # HWC → NCHW
        tensor = padded.transpose(2, 0, 1)[np.newaxis, ...]
        return np.ascontiguousarray(tensor, dtype=np.float32), \
               (scale, top, left, nw, nh, h, w)

    def postprocess(self, pred, pre_meta, debug=False):
        """概率图 → 文字框列表 (原图坐标)"""
        import cv2
        scale, top, left, nw, nh, orig_h, orig_w = pre_meta
        _, _, fh, fw = DET_SHAPE

        prob = pred[0, 0]  # (fh, fw)

        if debug:
            print(f"[DET-DBG] prob map: shape={prob.shape} "
                  f"min={prob.min():.4f} max={prob.max():.4f} "
                  f"mean={prob.mean():.4f}")

        # 裁掉 pad 区域，resize 回原图尺寸
        crop = prob[top:top+nh, left:left+nw]
        prob_rsz = cv2.resize(crop, (orig_w, orig_h),
                              interpolation=cv2.INTER_LINEAR)

        if debug:
            above = (prob_rsz > self.det_thresh).sum()
            total = prob_rsz.size
            print(f"[DET-DBG] after crop+resize: shape={prob_rsz.shape} "
                  f"min={prob_rsz.min():.4f} max={prob_rsz.max():.4f} "
                  f"pixels>{self.det_thresh}={above}/{total} ({100*above/total:.1f}%)")

        # 二值化
        bitmap = (prob_rsz > self.det_thresh).astype(np.uint8)

        # 找轮廓
        contours, _ = cv2.findContours(
            bitmap, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if debug:
            print(f"[DET-DBG] contours raw: {len(contours)}")

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            # 小量 padding
            pad = max(1, int(min(w, h) * 0.05))
            x, y = max(0, x - pad), max(0, y - pad)
            w = min(prob_rsz.shape[1] - x, w + 2 * pad)
            h = min(prob_rsz.shape[0] - y, h + 2 * pad)

            # box 置信度
            box_conf = float(prob_rsz[y:y+h, x:x+w].mean())
            if box_conf < self.box_thresh:
                continue

            boxes.append([x, y, x + w, y + h, box_conf])

        # 阅读顺序排序: 从上到下, 从左到右
        boxes = self._sort_boxes(boxes)
        return boxes

    @staticmethod
    def _sort_boxes(boxes):
        """按阅读顺序排序 (top-to-bottom, left-to-right)"""
        if len(boxes) <= 1:
            return boxes
        boxes_arr = np.array(boxes)
        y1 = boxes_arr[:, 1]
        heights = boxes_arr[:, 3] - boxes_arr[:, 1]
        median_h = np.median(heights)
        idxs = np.argsort(y1)
        rows = []
        cur_row = [idxs[0]]
        for i in range(1, len(idxs)):
            if y1[idxs[i]] - y1[cur_row[-1]] < median_h * 0.5:
                cur_row.append(idxs[i])
            else:
                rows.append(sorted(cur_row, key=lambda j: boxes_arr[j, 0]))
                cur_row = [idxs[i]]
        rows.append(sorted(cur_row, key=lambda j: boxes_arr[j, 0]))
        result = []
        for row in rows:
            result.extend(boxes[j] for j in row)
        return result

    def __call__(self, img, debug=False):
        tensor, meta = self.preprocess(img)
        outputs = _rknn_infer(self._rknn, tensor)
        if outputs is None:
            return []
        return self.postprocess(outputs[0], meta, debug=debug)

    def release(self):
        if self._rknn:
            self._rknn.release()


# ╔══════════════════════════════════════════════════════════╗
# ║  RecInfer — CRNN 约束 CTC 识别                          ║
# ╚══════════════════════════════════════════════════════════╝
class RecInfer:
    """PP-OCRv4 CRNN 文字识别 → RKNN NPU, 约束 CTC 解码

    字符集已内嵌（11 个车间名字符在 6622 维输出中的索引），
    无需外部 characters.txt 文件。
    """

    def __init__(self, model_path=None):
        self.model_path = model_path or os.path.join(SCRIPT_DIR, "rec.rknn")
        self._rknn = None
        self._init()

    def _init(self):
        from rknnlite.api import RKNNLite
        rknn = RKNNLite()
        with _FDSilence():
            if rknn.load_rknn(self.model_path) != 0:
                raise RuntimeError(f"rec RKNN 加载失败: {self.model_path}")
            ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if ret != 0:
                ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
                if ret != 0:
                    raise RuntimeError("rec init_runtime 失败")
        self._rknn = rknn

    def preprocess(self, crop):
        """BGR crop (H,W,3) → (1,3,48,W') NCHW, resize H=48, pad W=512"""
        import cv2
        h, w = crop.shape[:2]
        _, _, fh, fw = REC_SHAPE  # 48, 512

        # 等比例缩放高度到 48
        ratio = fh / h
        new_w = int(w * ratio)
        resized = cv2.resize(crop, (new_w, fh), interpolation=cv2.INTER_LINEAR)

        # 宽度不足 512 → 右侧 zero pad
        if new_w < fw:
            padded = np.zeros((fh, fw, 3), dtype=np.float32)
            padded[:, :new_w] = resized
        elif new_w > fw:
            padded = cv2.resize(resized, (fw, fh), interpolation=cv2.INTER_LINEAR)
        else:
            padded = resized.astype(np.float32)

        # normalize: (BGR/255 - 0.5) / 0.5
        padded = padded / 255.0
        padded = (padded - REC_MEAN) / REC_STD

        # HWC → NCHW
        tensor = padded.transpose(2, 0, 1)[np.newaxis, ...]
        return np.ascontiguousarray(tensor, dtype=np.float32)

    @staticmethod
    def constrained_ctc_decode(pred):
        """约束 CTC 贪心解码 — 仅限 11 个有效字符 + blank

        Args:
            pred: (1, T, 6622) — rec 模型输出（含 softmax 概率）
        Returns:
            (text, confidence)
        """
        if pred.ndim == 3:
            pred = pred[0]  # (T, 6622)

        # 提取 12 个有效类别的概率（blank + 11 chars）
        probs_12 = pred[:, _VALID_INDICES]                    # (T, 12)
        # 重新归一化
        probs_12 = probs_12 / (probs_12.sum(axis=1, keepdims=True) + 1e-9)

        # 每个时间步取最大概率的类别
        best_idx = probs_12.argmax(axis=1)                    # (T,) 0..11
        best_prob = probs_12.max(axis=1)                      # (T,)

        # CTC 贪心: 去 blank, 去连续重复
        chars = []
        confs = []
        prev = -1
        for idx, prob in zip(best_idx, best_prob):
            if idx != prev and idx != 0:  # 不是 blank 且不与前一个重复
                chars.append(_ID_TO_CHAR[idx])
                confs.append(prob)
            prev = idx

        text = "".join(chars)
        conf = float(np.mean(confs)) if confs else 0.0
        return text, conf

    def __call__(self, crop, debug=False):
        tensor = self.preprocess(crop)
        outputs = _rknn_infer(self._rknn, tensor)
        if outputs is None:
            return "", 0.0
        if debug:
            pred = outputs[0]
            print(f"        [REC-DBG] output shape={pred.shape} "
                  f"dim={pred.shape[-1] if pred.ndim == 3 else '?'}")
            if pred.ndim == 3:
                # 统计有效类别概率质量
                raw_probs = pred[0]  # (T, 6622)
                valid_mass = raw_probs[:, _VALID_INDICES].sum(axis=1)  # (T,)
                print(f"        [REC-DBG] valid(12)-mass: min={valid_mass.min():.4f} "
                      f"max={valid_mass.max():.4f} mean={valid_mass.mean():.4f}")
                # 全 6622 类的 top-3 字符（看模型真正在识别什么）
                top3 = raw_probs.max(axis=0).argsort()[::-1][:3]
                print(f"        [REC-DBG] global top-3 indices: {top3.tolist()}")
        return self.constrained_ctc_decode(outputs[0])

    def release(self):
        if self._rknn:
            self._rknn.release()


# ╔══════════════════════════════════════════════════════════╗
# ║  WorkshopOCR — 对外统一接口                             ║
# ╚══════════════════════════════════════════════════════════╝
class WorkshopOCR:
    """PP-OCR RKNN 车间名识别 — 全流程封装

    用法:
        ocr = WorkshopOCR()
        name, conf = ocr.predict(frame)         # 整帧（含镜像）
        name, conf = ocr.predict_crop(roi)       # ROI（不镜像）
    """

    def __init__(self, det_model=None, rec_model=None, conf_thresh=0.5):
        self.conf_thresh = conf_thresh
        self._det = DetInfer(det_model)
        self._rec = RecInfer(rec_model)

    def predict(self, frame, debug=False):
        """整帧识别 → (车间名, 置信度)"""
        import cv2
        img = cv2.flip(frame, 1)  # 镜像纠正
        return self._recognize(img, debug=debug)

    def predict_crop(self, roi, debug=False):
        """ROI 识别 → (车间名, 置信度)，不镜像"""
        return self._recognize(roi, debug=debug)

    def _recognize(self, img, debug=False):
        """完整管线: det → rec → 车间名匹配"""
        # 1. 检测文字框
        boxes = self._det(img, debug=debug)  # [[x1,y1,x2,y2,conf], ...]
        if debug:
            print(f"[DET] found {len(boxes)} boxes")
        if not boxes:
            return "", 0.0

        # 2. 对每个框做约束 CTC 识别
        results = []
        for i, box in enumerate(boxes):
            x1, y1, x2, y2, box_conf = box
            x1, y1 = max(0, int(x1)), max(0, int(y1))
            x2, y2 = min(img.shape[1], int(x2)), min(img.shape[0], int(y2))
            if x2 <= x1 or y2 <= y1:
                continue
            crop = img[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            rec_debug = debug and i == 0  # 仅第一个 box 做 rec 详细诊断
            text, rec_conf = self._rec(crop, debug=rec_debug)
            if debug:
                print(f"  box [{x1},{y1},{x2},{y2}] box_conf={box_conf:.3f} "
                      f"→ text=\"{text}\" rec_conf={rec_conf:.4f}")
            if text and rec_conf >= self.conf_thresh:
                results.append((text, rec_conf))

        if not results:
            return "", 0.0

        # 按置信度降序
        results.sort(key=lambda r: r[1], reverse=True)

        # 3. 关键词匹配 → 唯一车间名
        for text, rec_conf in results:
            candidates = set()
            for full_name, keywords in VALID_WORKSHOPS.items():
                for kw in keywords:
                    if kw in text:
                        candidates.add(full_name)
            if len(candidates) == 1:
                return candidates.pop(), rec_conf

        if debug:
            print(f"[MATCH] no unique workshop match among results: {results}")
        return "", 0.0

    def predict_print(self, frame):
        """识别并打印所有检测到的文字（调试用）"""
        import cv2
        img = cv2.flip(frame, 1)
        boxes = self._det(img)
        for box in boxes:
            x1, y1, x2, y2, _ = box
            crop = img[max(0, int(y1)):min(img.shape[0], int(y2)),
                       max(0, int(x1)):min(img.shape[1], int(x2))]
            if crop.size == 0:
                continue
            text, conf = self._rec(crop)
            print(f"  [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}] "
                  f"\"{text}\" conf={conf:.4f}")
        return self._recognize(img)

    def release(self):
        self._det.release()
        self._rec.release()


# ╔══════════════════════════════════════════════════════════╗
# ║  测试入口                                               ║
# ╚══════════════════════════════════════════════════════════╝
def main():
    import argparse, cv2

    parser = argparse.ArgumentParser(
        description="PP-OCR RKNN 车间名文字识别")
    parser.add_argument("--image", default=None, help="单张图片路径")
    parser.add_argument("--camera", action="store_true", help="摄像头实时")
    parser.add_argument("--debug", action="store_true",
                        help="打印所有检测到的文字")
    args = parser.parse_args()

    ocr = WorkshopOCR()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[ERROR] 无法读取: {args.image}")
            return
        if args.debug:
            print("=== DEBUG: 完整管线诊断 ===")
            print(f"[IMG] shape={img.shape}")
        name, conf = ocr.predict(img, debug=args.debug)
        print(f'Result: "{name}" conf={conf:.4f}')

    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            print("[ERROR] 无法打开摄像头")
            return

        print("\n[OCR] 运行中... Ctrl+C 停止\n")
        last = ""
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue
                if args.debug:
                    ocr.predict_print(frame)
                name, conf = ocr.predict(frame)
                if name and name != last:
                    last = name
                    print(f'Result: "{name}" conf={conf:.4f}')
        except KeyboardInterrupt:
            print("\n退出")
        cap.release()

    ocr.release()


if __name__ == "__main__":
    main()

