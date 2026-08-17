# RapidOCR RKNN — 门牌车间文字识别

RapidOCR 官方管线 + RKNN NPU 加速。预处理/后处理 100% 复用官方，仅推理引擎替换。

## 文件结构

```
rapidocr_rknn/
├── ocr_infer.py          # ★ 小车端主文件
├── export_ocr_rknn.py    # PC端: ONNX → RKNN 转换
├── setup_car.sh          # 小车依赖安装
├── models/               # 模型目录
│   ├── det.rknn          # 文字检测 (DBNet)
│   ├── cls.rknn          # 方向分类 (0°/180°)
│   ├── rec.rknn          # 文字识别 (CRNN+CTC)
│   └── characters.txt    # 字符集 (导出时自动生成)
└── README.md
```

## 快速开始

### Step 1: PC 转换模型（Linux x86_64）

```bash
pip install rapidocr-onnxruntime
pip install rknn_toolkit2-1.6.0+81f21f4d-cp38-cp38-linux_x86_64.whl
python3 export_ocr_rknn.py --platform rk3588
# → models/det.rknn, cls.rknn, rec.rknn, characters.txt
```

### Step 2: 部署到小车

```bash
# 复制整个文件夹
scp -r rapidocr_rknn/ ucar@car:~/rapidocr_rknn/

# 小车上安装依赖
cd ~/rapidocr_rknn
bash setup_car.sh
```

### Step 3: 测试

```bash
python3 ocr_infer.py --image test.jpg   # 单张
python3 ocr_infer.py --camera           # 摄像头
```

### 集成到 vision_node

```python
from ocr_infer import RapidOcrRKNN

ocr = RapidOcrRKNN()
text, conf = ocr.predict(frame)
# → ("电子产品加工车间", 0.885)
```

接口与 `home/rapidocr/infer.py` 的 `RapidOcrInfer` 完全兼容。

## 性能

| 模型 | ONNX CPU | RKNN NPU (RK3588) |
|------|----------|-------------------|
| det 检测 | ~100ms | ~10ms |
| cls 分类 | ~5ms | ~1ms |
| rec 识别 | ~50ms | ~5ms |
| **合计** | **~155ms** | **~16ms (~60 FPS)** |

## 回退机制

无 RKNN 模型时自动回退 ONNX Runtime (CPU)，无需任何配置修改。
