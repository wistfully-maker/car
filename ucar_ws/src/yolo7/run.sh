#!/bin/bash
# yolo7 — YOLO 门牌检测 + OCR 识别
# 单进程，单摄像头，双 NPU 模型协作
#
# 用法:
#   ./run.sh                    # 完整管线（检测 + OCR）
#   ./run.sh --no-ocr           # 纯检测
#   ./run.sh --image test.jpg   # 单张图片
#   ./run.sh --conf 0.6         # 调高检测阈值
#   ./run.sh --ocr-conf 0.6     # 调高 OCR 触发阈值

cd "$(dirname "$0")"

echo "[yolo7] starting..."
python3 infer.py "$@"
