# turn_yolo —— 红绿灯方向检测（小车部署）

用 `yolov8n` 在真实车拍数据上训练的红绿灯方向检测模块，输出 `red / left / right / straight`。

## 目录内容

```
turn_yolo/
├── best.pt          # 训练好的 yolov8n 模型（640×640，4 类）
├── yolo_infer.py    # 主检测：持续运行，投票窗口输出
├── yolo_quick.py    # 快速检测：采集几秒，一次性输出
└── README.md
```

## 类别映射（与训练一致，顺序不能改）

| id | 类别 | 输出 |
|----|------|------|
| 0  | green_straight | `straight` |
| 1  | green_left     | `left` |
| 2  | green_right    | `right` |
| 3  | red_stop       | `red` |

## 快速开始

```bash
# 小车上：订阅 ROS 摄像头 /usb_cam/image_raw
python3 yolo_infer.py

# 无 ROS 测试：直读摄像头
python3 yolo_infer.py --camera 0

# 单张图测试
python3 yolo_infer.py --image test.jpg

# 快速单帧（采集 3 秒出结果）
python3 yolo_quick.py
```

## 输出

- 终端打印：`>>> red (0.86)`
- 结果写入 `/tmp/yolo_result.txt`（供 auto_drive 读取）

## 关键约束

- **输入 640×640**，与训练、与检测节点 `INPUT_SIZE=640` 一致，不要改 `IMGSZ`。
- **类别顺序固定**：`0=straight 1=left 2=right 3=red`（不是旧版 `red/straight/left/right`）。
- 依赖 `ultralytics`（小车上需有 torch + ultralytics 环境）。

## 模型指标

| 类别 | mAP50 |
|------|-------|
| red（红灯） | 0.995 |
| straight | 0.995 |
| left | 0.995 |
| right | 0.912 |
| **all** | **0.974** |

训练数据为真实车拍图，红灯（实心方块）与绿箭头（镂空箭头）已能稳定区分。
