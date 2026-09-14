#!/bin/bash
# RapidOCR RKNN — 小车端依赖安装
# 用法: bash setup_car.sh

set -e
echo "=== RapidOCR RKNN 小车端安装 ==="

# 1. 安装 rapidocr-onnxruntime（预处理/后处理工具）
echo "[1/2] pip install rapidocr-onnxruntime..."
pip install rapidocr-onnxruntime -q

# 2. 检查 rknnlite（小车应已预装）
echo "[2/2] 检查 rknnlite..."
python3 -c "from rknnlite.api import RKNNLite; print('  RKNNLite OK')" 2>/dev/null || \
    echo "  [WARN] rknnlite 未安装，将使用 ONNX 回退"

echo ""
echo "=== 完成 ==="
echo "测试: python3 ocr_infer.py --image test.jpg"
echo "摄像头: python3 ocr_infer.py --camera"
