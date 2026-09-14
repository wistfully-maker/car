#!/usr/bin/env python3
"""
RapidOCR 3 个 ONNX 模型 → RKNN 批量转换
==========================================
在 PC（x86_64 Linux, 已安装 RKNN-Toolkit2 1.6.0）上运行。

转换的模型:
  1. ch_PP-OCRv3_det_infer.onnx  → det.rknn  (文字检测 DBNet)
  2. ch_ppocr_mobile_v2.0_cls_infer.onnx → cls.rknn (方向分类)
  3. ch_PP-OCRv3_rec_infer.onnx  → rec.rknn  (文字识别 CRNN+CTC)

用法:
    python3 export_ocr_rknn.py                          # 默认 RK3588, FP16
    python3 export_ocr_rknn.py --platform rk3566        # 指定平台
    python3 export_ocr_rknn.py --det_imgsz 480          # 检测模型输入尺寸

前置条件:
    pip install rapidocr-onnxruntime  # 获取 ONNX 模型
    pip install rknn-toolkit2>=1.6.0  # RKNN 转换工具

输出:
    models/det.rknn, cls.rknn, rec.rknn  → 复制到小车 ~/rapidocr_rknn/models/
"""

import os, sys, argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "models")

# RapidOCR ONNX 模型默认路径（pip install rapidocr-onnxruntime 后自动下载）
try:
    import rapidocr_onnxruntime as _r
    _RAPIDOCR_DIR = os.path.dirname(_r.__file__)
except ImportError:
    _RAPIDOCR_DIR = None

DEFAULT_MODELS = {
    "det": {
        "name": "文字检测 DBNet",
        "src": None,  # auto-find
        "dst": os.path.join(OUTPUT_DIR, "det.rknn"),
        "imgsz": 640,
        "mean": [0.485, 0.456, 0.406],
        "std":  [0.229, 0.224, 0.225],
        "scale": 1.0 / 255,
    },
    "cls": {
        "name": "方向分类",
        "src": None,
        "dst": os.path.join(OUTPUT_DIR, "cls.rknn"),
        "imgsz": [3, 48, 192],  # CHW fixed
        "mean": [0.5, 0.5, 0.5],
        "std":  [0.5, 0.5, 0.5],
        "scale": 1.0 / 255,
    },
    "rec": {
        "name": "文字识别 CRNN+CTC",
        "src": None,
        "dst": os.path.join(OUTPUT_DIR, "rec.rknn"),
        "imgsz": [3, 48, 320],  # CHW, W=320 for RKNN fixed
        "mean": [0.5, 0.5, 0.5],
        "std":  [0.5, 0.5, 0.5],
        "scale": 1.0 / 255,
    },
}

# 自动查找 ONNX 模型
if _RAPIDOCR_DIR:
    _models_dir = os.path.join(_RAPIDOCR_DIR, "models")
    if os.path.isdir(_models_dir):
        for f in os.listdir(_models_dir):
            fpath = os.path.join(_models_dir, f)
            if "_fixed" in f:
                continue
            if "det_infer" in f:
                DEFAULT_MODELS["det"]["src"] = fpath
            elif "cls_infer" in f:
                DEFAULT_MODELS["cls"]["src"] = fpath
            elif "rec_infer" in f:
                DEFAULT_MODELS["rec"]["src"] = fpath

PLATFORMS = {
    "rk3588": "RK3588 (6 TOPS)",
    "rk3576": "RK3576 (6 TOPS)",
    "rk3566": "RK3566 (1 TOPS)",
    "rk3568": "RK3568 (1 TOPS)",
    "rk3399pro": "RK3399Pro (3 TOPS)",
}


def convert_one(rknn_api, cfg, args, model_key=""):
    """转换单个模型"""
    name = cfg["name"]
    src = cfg["src"]
    dst = cfg["dst"]

    if not src or not os.path.exists(src):
        print(f"[SKIP] {name}: ONNX 模型未找到 ({src})")
        print(f"       请先 pip install rapidocr-onnxruntime")
        return False

    print(f"\n{'='*50}")
    print(f"  转换: {name}")
    print(f"  输入: {os.path.basename(src)}")
    print(f"  输出: {os.path.basename(dst)}")
    print(f"{'='*50}")

    rknn = rknn_api(verbose=args.verbose)

    # rknn-toolkit2 1.6.0: config 核心参数
    rknn.config(target_platform=args.platform, output_optimize=True)

    # --- 先用 ONNX 固定动态 shape，再加载到 RKNN ---
    import onnx
    imgsz = cfg.get("imgsz", 640)
    if isinstance(imgsz, list):
        fixed_shape = [1] + imgsz  # [3,48,192] → [1,3,48,192]
    else:
        fixed_shape = [1, 3, imgsz, imgsz]  # 640 → [1,3,640,640]

    # 加载 ONNX，强制设置所有输入维度为固定值
    model = onnx.load(src)
    for inp in model.graph.input:
        for i, val in enumerate(fixed_shape):
            dim = inp.type.tensor_type.shape.dim[i]
            dim.ClearField("dim_param")   # 必须先清除动态参数名
            dim.dim_value = val           # 再设置固定值

    # 保存为临时固定 shape ONNX（放 output 目录，避免被 auto-find 误匹配）
    fixed_onnx = os.path.join(os.path.dirname(dst), f"{model_key}_fixed.onnx")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    onnx.save(model, fixed_onnx)
    print(f"  fixed shape: {fixed_shape} -> {os.path.basename(fixed_onnx)}")

    ret = rknn.load_onnx(model=fixed_onnx)
    if ret != 0:
        print(f"  [FAIL] ONNX 加载失败, ret={ret}")
        rknn.release()
        return False
    print(f"  [OK] ONNX loaded")

    # --- 构建（暂时用 simulator 测试，上板前改 target）---
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        print(f"  [FAIL] 构建失败, ret={ret}")
        rknn.release()
        return False
    print(f"  [OK] build done")

    # --- 导出 ---
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    ret = rknn.export_rknn(dst)
    if ret != 0:
        print(f"  [FAIL] 导出失败, ret={ret}")
        rknn.release()
        return False

    size_kb = os.path.getsize(dst) / 1024
    print(f"  [OK] -> {os.path.basename(dst)} ({size_kb:.0f} KB)")

    # --- 提取字符集（rec 模型）---
    if model_key == "rec":
        try:
            import onnx
            m = onnx.load(src)
            for prop in m.metadata_props:
                if prop.key == "character":
                    chars_path = os.path.join(os.path.dirname(dst), "characters.txt")
                    with open(chars_path, "w", encoding="utf-8") as f:
                        f.write(prop.value)
                    print(f"  [OK] -> characters.txt ({len(prop.value.splitlines())} chars)")
                    break
        except Exception as e:
            print(f"  [WARN] 字符集提取失败: {e}")

    rknn.release()
    return True


def main():
    parser = argparse.ArgumentParser(description="RapidOCR ONNX → RKNN 批量转换")
    parser.add_argument("--platform", default="rk3588",
                        choices=list(PLATFORMS.keys()))
    parser.add_argument("--quant", default="fp16", choices=["fp16", "i8"])
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--skip", nargs="*", choices=["det", "cls", "rec"],
                        default=[], help="跳过的模型")
    parser.add_argument("--output_dir", default=OUTPUT_DIR,
                        help="RKNN 输出目录")
    args = parser.parse_args()

    # 更新输出路径
    for k in DEFAULT_MODELS:
        DEFAULT_MODELS[k]["dst"] = os.path.join(
            args.output_dir, f"{k}.rknn"
        )

    print("=" * 60)
    print("  RapidOCR ONNX → RKNN 批量转换")
    print("=" * 60)
    print(f"  平台: {PLATFORMS[args.platform]}")
    print(f"  量化: {args.quant}")
    print(f"  输出: {args.output_dir}")

    # 检查 ONNX 模型
    print(f"\n[INFO] ONNX 模型状态:")
    for k, cfg in DEFAULT_MODELS.items():
        status = "✓" if cfg["src"] and os.path.exists(cfg["src"]) else "✗ 未找到"
        print(f"  {k}: {status}")
    print()

    # 导入 RKNN
    try:
        from rknn.api import RKNN as RKNNClass
    except ImportError:
        print("[ERROR] 未安装 rknn-toolkit2\n"
              "  pip install rknn-toolkit2>=1.6.0")
        sys.exit(1)

    # 逐个转换
    results = {}
    for k, cfg in DEFAULT_MODELS.items():
        if k in args.skip:
            print(f"[SKIP] {cfg['name']} (用户跳过)")
            continue
        results[k] = convert_one(RKNNClass, cfg, args, model_key=k)

    # 汇总
    print(f"\n{'='*60}")
    success = sum(1 for v in results.values() if v)
    total = len(results)
    print(f"  完成: {success}/{total} 个模型转换成功")
    if success > 0:
        print(f"\n  输出文件:")
        for k, cfg in DEFAULT_MODELS.items():
            if results.get(k):
                print(f"    {cfg['dst']}")
        print(f"\n  部署: scp models/*.rknn ucar@car:~/rapidocr_rknn/models/")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()

