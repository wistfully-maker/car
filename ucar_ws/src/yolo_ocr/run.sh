#!/bin/bash
# YOLO_OCR 门牌检测 — 小车端启动脚本
# 用法:
#   ./run.sh                          # 摄像头实时检测
#   ./run.sh --image test.jpg         # 单张图片测试
#   ./run.sh --camera                 # 摄像头实时

cd "$(dirname "$0")"

python3 infer.py "$@"
