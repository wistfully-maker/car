#!/usr/bin/env python3
"""
YOLO26 门牌检测 ONNX → RKNN 模型转换 (yolo_biao)
==================================================
在 PC（x86_64 Linux，RKNN-Toolkit2 1.6.0）上运行。

用法:
    python3 export_rknn.py                    # 默认: FP16, rk3588
    python3 export_rknn.py --quant i8          # INT8 量化
    python3 export_rknn.py --platform rk3566   # 其他平台

输出:
    best.rknn  — 复制到小车 ~/yolo_biao/ 部署
"""

import os, sys, argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ONNX = os.path.join(SCRIPT_DIR, "best.onnx")
DEFAULT_RKNN = os.path.join(SCRIPT_DIR, "best.rknn")


def main():
    parser = argparse.ArgumentParser(description="YOLO26 ONNX → RKNN")
    parser.add_argument("--onnx", default=DEFAULT_ONNX, help="ONNX 路径")
    parser.add_argument("--output", default=DEFAULT_RKNN, help="RKNN 输出路径")
    parser.add_argument("--platform", default="rk3588",
                        choices=["rk3588", "rk3566", "rk3568", "rk3562", "rk3576", "rk3399pro"])
    parser.add_argument("--quant", default="fp16", choices=["fp16", "i8"])
    parser.add_argument("--imgsz", type=int, default=1024, help="输入尺寸 (需与训练一致)")
    parser.add_argument("--calib_dir", default=None, help="INT8 校准图片目录")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-precompile", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.onnx):
        print(f"[ERROR] ONNX 不存在: {args.onnx}")
        sys.exit(1)

    onnx_size = os.path.getsize(args.onnx) / (1024 * 1024)
    print("=" * 60)
    print("  YOLO26 → RKNN (yolo_biao)")
    print("=" * 60)
    print(f"  ONNX:  {args.onnx} ({onnx_size:.1f} MB)")
    print(f"  RKNN:  {args.output}")
    print(f"  平台:  {args.platform}")
    print(f"  量化:  {args.quant}")
    print(f"  输入:  {args.imgsz}×{args.imgsz}")
    print()

    # 检查 ONNX info
    try:
        import onnx
        m = onnx.load(args.onnx)
        opset = m.opset_import[0].version
        inputs = [(n.name, [d.dim_value for d in n.type.tensor_type.shape.dim])
                   for n in m.graph.input]
        outputs = [(n.name, [d.dim_value for d in n.type.tensor_type.shape.dim])
                    for n in m.graph.output]
        print(f"[INFO] ONNX opset={opset}")
        print(f"[INFO] 输入: {inputs}")
        print(f"[INFO] 输出: {outputs}")
        print()
    except ImportError:
        pass

    # 导入 RKNN Toolkit2
    try:
        from rknn.api import RKNN
    except ImportError:
        print("[ERROR] 未安装 rknn-toolkit2")
        print("  pip install rknn-toolkit2>=1.6.0")
        sys.exit(1)

    rknn = RKNN(verbose=args.verbose)
    rknn.config(target_platform=args.platform, output_optimize=True)
    print("[INFO] ✅ RKNN 配置完成")

    print("[INFO] 加载 ONNX...")
    ret = rknn.load_onnx(model=args.onnx)
    if ret != 0:
        print(f"[ERROR] ONNX 加载失败, ret={ret}")
        sys.exit(1)
    print("[INFO] ✅ ONNX 加载成功")

    print("[INFO] 构建 RKNN（约 3-10 分钟）...")
    ret = rknn.build(
        do_quantization=(args.quant == "i8"),
        dataset=args.calib_dir if args.quant == "i8" else None,
    )
    if ret != 0:
        print(f"[ERROR] RKNN 构建失败, ret={ret}")
        sys.exit(1)
    print("[INFO] ✅ RKNN 构建成功")

    ret = rknn.export_rknn(args.output)
    if ret != 0:
        print(f"[ERROR] RKNN 导出失败, ret={ret}")
        sys.exit(1)

    file_size = os.path.getsize(args.output) / (1024 * 1024)
    print(f"\n{'=' * 60}")
    print(f"  ✅ RKNN 导出成功!")
    print(f"{'=' * 60}")
    print(f"  文件: {args.output}")
    print(f"  大小: {file_size:.1f} MB")

    if args.verbose:
        try:
            rknn.accuracy_analysis(inputs=[args.onnx])
        except Exception as e:
            print(f"[INFO] 精度分析跳过: {e}")

    rknn.release()
    print(f"\n  下一步: scp best.rknn ucar@car:~/yolo_biao/")
    print(f"         小车端: python3 infer.py")


if __name__ == "__main__":
    main()
