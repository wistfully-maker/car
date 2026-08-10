# stop — 集成 OCR 识别 + LiDAR PCA 精停 + 全流程任务编排
启动launch文件home\ucar\ucar_ws\src\stop\launch\mission.launch
## 概述

`stop` 包整合了基于 RKNN NPU 的中文 OCR（PaddleOCR/PP-OCR）、LiDAR PCA 外法向量精停、以及两阶段（实物+仿真）车间任务编排。适配 RK3588 + ROS 1 Noetic + 麦克纳姆轮全向底盘。

## 文件结构

```
stop/
├── CMakeLists.txt                      # 构建配置
├── package.xml                         # 包依赖声明
├── README.md
├── msg/
│   ├── BoundingBox.msg                 # 单个 OCR 检测框
│   └── BoundingBoxes.msg               # OCR 检测框列表
├── launch/
│   ├── mission.launch                  # 全流程一键启动（含导航栈）
│   ├── mission_light.launch            # 轻量版（依赖已有导航栈）
│   └── scan_and_park.launch            # 单独的旋转扫描+停车
├── scripts/
│   ├── mission_orchestrator.py         # 全流程任务编排（核心）
│   ├── scan_and_park.py                # 旋转扫描 + PCA 精停（独立模式）
│   ├── precision_park.py               # LiDAR PCA 精停工具
│   ├── ocr_native_node.py              # RKNN OCR 推理节点
│   ├── ocr/                            # PP-OCR 模块
│   │   ├── ppocr_system.py             # OCR 系统主入口
│   │   ├── ppocr_det.py                # 文字检测
│   │   ├── ppocr_rec.py                # 文字识别
│   │   ├── rknn_executor.py            # RKNN NPU 推理执行器
│   │   └── utils/                      # 后处理工具
│   └── models/                         # RKNN 模型文件
│       ├── ppocrv4_det.rknn            # 检测模型
│       ├── ppocrv4_rec.rknn            # 识别模型
│       └── ppocr_keys_v1.txt           # 字符集字典
└── wav/                                # 播报音频输出目录
```

## 依赖

| 依赖 | 用途 |
|---|---|
| `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs` | ROS 基础通信 |
| `move_base_msgs`, `actionlib_msgs` | move_base 导航 |
| `tf2_ros`, `tf` | 坐标变换 |
| `cv_bridge`, `numpy`, `opencv` (cv2) | 图像处理 |
| RKNN Toolkit (rknn-toolkit2) | NPU 模型推理 |
| `speech_command/scripts/tts_http.py` | 讯飞 TTS 语音播报 |
| `ucar_controller`, `ydlidar`, `usb_cam` | 硬件驱动 |
| `map_server`, `amcl`, `move_base` | 定位与导航 |

## 构建

```bash
cd ~/ucar_ws
catkin_make --pkg stop
source devel/setup.bash
```

## 运行

### 全流程任务（推荐）

```bash
# 清理上次运行残留
sudo rm -f /var/lock/LCK..ttyS4
sudo pkill -f ros
sleep 2

# 启动
roslaunch stop mission.launch task_input:="食品，苹果；电子，手机"
```

`mission.launch` 包含：硬件驱动 → 地图服务 → AMCL 定位 → move_base + TEB 动态避障 → RKNN OCR → 任务编排。

### 自定义初始位姿

```bash
roslaunch stop mission.launch \
    task_input:="日用，牙刷；电子，手机" \
    initial_pose_x:=-0.813 \
    initial_pose_y:=-2.442 \
    initial_pose_yaw:=0.0
```

初始位姿设为 `(0, 0, 0)` 则跳过自动发布，需在 RViz 手动 2D Pose Estimate。

## 任务输入格式

```
"真实车间关键词，货物名；仿真车间关键词，货物名"
```

| 示例输入 | Phase 1 目标 | Phase 1 播报 | Phase 2 目标 | Phase 2 播报 |
|---|---|---|---|---|
| `食品，苹果；电子，手机` | 食品加工车间 | 已将苹果放入食品加工车间 | 电子产品生产车间 | 仿真任务已完成，已将手机放入电子产品生产车间 |
| `日用，牙刷；电子，手机` | 日用品加工车间 | 已将牙刷放入日用品加工车间 | 电子产品生产车间 | 仿真任务已完成，已将手机放入电子产品生产车间 |

支持的关键字：`食品`、`电子`、`日用`、`电子产品`。

## 配置参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `task_input` | `食品，苹果；电子，手机` | 任务输入字符串 |
| `initial_pose_x` | `-0.813` | 初始位姿 X (map) |
| `initial_pose_y` | `-2.442` | 初始位姿 Y (map) |
| `initial_pose_yaw` | `0.0` | 初始位姿 Yaw (rad) |
| `max_rotations` | `6` | 每航点最大 OCR 旋转扫描次数 |
| `rotate_angle` | `72` | 每次旋转角度 (°) |
| `rotate_speed` | `1.5` | 旋转速度 (rad/s) |
| `target_distance` | `0.20` | 停车距标牌距离 (m) |
| `x_align_tolerance` | `0.50` | X 轴精停容差 (m) |
| `y_align_tolerance` | `12` | Y 轴相机偏移容差 (°) |

## 流程架构

### 双阶段

```
Phase 1 (real): 遍历航点 → OCR 找真实车间 → PCA 精停 → TTS 播报 → 切 Phase 2
Phase 2 (sim):  双指针跳转 → 遍历剩余航点 → OCR 找仿真车间 → PCA 精停 → TTS 播报 → 完成
```

### 回调驱动状态机（Stage 0-6）

| Stage | 说明 | 触发源 |
|---|---|---|
| 0 | 首次发现标牌 → 旋转使标牌居中 | `boxes_callback` |
| 1 | 二次确认 → 启用 LiDAR PCA | `boxes_callback` |
| 2 | PCA 计算法向量 → cmd_vel 慢速逼近 | `LidarCallback` |
| 3 | PCA 旋转对正（Yaw 微调） | `LidarCallback` / `boxes_callback` |
| 4 | 摄像头 Y 轴横移微调 | `boxes_callback` |
| 5 | LiDAR X 轴前进微调 | `LidarCallback` |
| 6 | 任务完成 → 播报 → 切换阶段 | `mission_done()` |

### 导航

- **航点间导航**：`move_base` + TEB 局部规划器（动态避障）
- **Stage 2 逼近**：`cmd_vel` 直接驱动 + LiDAR 实时测距（避免取物点落入代价地图膨胀区导致全局规划失败）
- **Phase 2 切换**：倒车 0.5m + 掉头 180° + 清除代价地图（防止贴墙导致规划失败）

### 双指针优化

Phase 1 遍历时若 OCR 同时识别到仿真车间标牌，记录所在航点索引。Phase 2 直接跳转到该索引，无需重新遍历。

## ROS 接口

### 订阅

| Topic | 类型 | 用途 |
|---|---|---|
| `/usb_cam/image_raw` | `Image` | 摄像头图像 |
| `/perception/bounding_boxes` | `BoundingBoxes` | OCR 检测结果 |
| `/move_base/result` | `MoveBaseActionResult` | 导航结果 |
| `/scan` | `LaserScan` | LiDAR 扫描数据 |

### 发布

| Topic | 类型 | 用途 |
|---|---|---|
| `/move_base_simple/goal` | `PoseStamped` | 导航目标点 |
| `/perception/detect_image` | `Image` | 发布图像给 OCR 节点 |
| `/cmd_vel` | `Twist` | 底盘直接控制 |
| `/mission/result` | `String` | 任务结果 (`phase1_done` / `done` / `failed:not_found`) |
| `/initialpose` | `PoseWithCovarianceStamped` | 自动初始位姿 |

## 常见问题

### 串口锁文件

```bash
sudo rm -f /var/lock/LCK..ttyS4
```

### `Failed to get a plan`

航点导航失败通常是初始位姿不准或机器人贴墙太近。确认 `initial_pose` 与实际位置一致，Phase 2 会自动倒车+掉头处理。

### OCR 无结果

检查摄像头是否正常：`rostopic hz /usb_cam/image_raw`。确认 `ocr_native_node` 在运行。

### TTS 不播报

确认 `speech_command/scripts/tts_http.py` 存在，讯飞 AppID/APIKey 已配置。
