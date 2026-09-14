#!/usr/bin/env python3
"""加载一次 cls 模型，批量跑 4 类测试图，打印置信度分布，验证是否连续。"""
import os, glob
import numpy as np, cv2
from rknnlite.api import RKNNLite

BASE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(BASE, "turn_yolo_480_cls.rknn")
NAMES = {0: "straight", 1: "left", 2: "right", 3: "red"}
IMGSZ = 480

rknn = RKNNLite()
rknn.load_rknn(MODEL)
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)


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


def detect(img):
    x = cv2.cvtColor(lb(img), cv2.COLOR_BGR2RGB)[None]
    out = rknn.inference(inputs=[x], data_type="uint8", data_format="nhwc")[0]
    preds = out[0].T
    scores = 1.0 / (1.0 + np.exp(-preds))
    conf = scores.max(1)
    cls = scores.argmax(1)
    i = int(conf.argmax())
    return NAMES[int(cls[i])], float(conf[i])


for cname in ["red", "right", "left", "straight"]:
    imgs = sorted(glob.glob(os.path.join(BASE, "../images/light", cname, "*.jpg")))
    confs = []
    ok = 0
    for p in imgs:
        img = cv2.imread(p)
        if img is None:
            continue
        n, cf = detect(img)
        confs.append(cf)
        if n == cname:
            ok += 1
    if confs:
        a = np.array(confs)
        uniq = sorted(set(np.round(a, 2)))
        print("%-8s n=%3d 正确=%3d conf min=%.2f max=%.2f 唯一值=%d" %
              (cname, len(a), ok, a.min(), a.max(), len(uniq)))
        print("          分布:", " ".join("%.2f" % v for v in uniq))
rknn.release()
