#!/usr/bin/env python3
"""单核 vs 三核推理速度对比：加载一次，跑 N 帧纯 NPU 推理。"""
import os, time
import numpy as np, cv2
from rknnlite.api import RKNNLite

BASE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(BASE, "turn_yolo_480_cls.rknn")
IMGSZ = 480
N = 200


def lb(im, new=(IMGSZ, IMGSZ), color=(114, 114, 114)):
    sh = im.shape[:2]
    r = min(new[0] / sh[0], new[1] / sh[1])
    nu = int(round(sh[1] * r)), int(round(sh[0] * r))
    dw, dh = (new[1] - nu[0]) / 2, (new[0] - nu[1]) / 2
    if sh[::-1] != nu:
        im = cv2.resize(im, nu, interpolation=cv2.INTER_LINEAR)
    return cv2.copyMakeBorder(im, int(round(dh - 0.1)), int(round(dh + 0.1)),
                              int(round(dw - 0.1)), int(round(dw + 0.1)),
                              cv2.BORDER_CONSTANT, value=color)


img_dir = os.path.join(BASE, "../images/light/red")
img = cv2.imread(os.path.join(img_dir, sorted(os.listdir(img_dir))[0]))
x = cv2.cvtColor(lb(img), cv2.COLOR_BGR2RGB)[None]

for mask, label in [(RKNNLite.NPU_CORE_0, "单核"), (RKNNLite.NPU_CORE_0_1_2, "三核")]:
    rk = RKNNLite()
    rk.load_rknn(MODEL)
    rk.init_runtime(core_mask=mask)
    for _ in range(10):                              # 预热
        rk.inference(inputs=[x], data_type="uint8", data_format="nhwc")
    t0 = time.time()
    for _ in range(N):
        rk.inference(inputs=[x], data_type="uint8", data_format="nhwc")
    dt = (time.time() - t0) / N * 1000
    print("%s: %.1f ms/帧 = %.1f FPS  (N=%d)" % (label, dt, 1000.0 / dt, N))
    rk.release()
