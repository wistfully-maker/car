"""定位 det 1088x1920 模型输出"""
import sys, numpy as np, io
sys.path.insert(0, '.')
from ocr_infer import _RKNNInferSession, _inject_rknn

_orig = _RKNNInferSession.__call__
def dbg(self, arr):
    print(f'[{self.model_key}] input shape={arr.shape} min={arr.min():.4f} max={arr.max():.4f}')
    r = _orig(self, arr)
    for i, o in enumerate(r):
        print(f'[{self.model_key}] out[{i}]: shape={o.shape} min={o.min():.6f} max={o.max():.6f} nonzero={np.count_nonzero(o)}')
    return r
_RKNNInferSession.__call__ = dbg

_inject_rknn()
from rapidocr_onnxruntime import RapidOCR
import cv2
img = cv2.imread('/home/ucar/ucar_ws/src/images/sign/daily/angle_1/shot_1.jpg')
ocr = RapidOCR()
r = ocr(cv2.flip(img, 1))
boxes, _ = r
print(f'\nBoxes: {len(boxes) if boxes else 0}')
for b in boxes or []:
    print(f'  "{b[1]}" conf={b[2]:.4f}')

