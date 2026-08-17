# light — 红绿灯检测节点（RKNN YOLO）

以 `src/ucar_nav/scripts/traffic_light_detector.py` 为蓝本的 ROS 节点，逻辑与其一致，模型使用本目录下的 `yolov8_custom.rknn`。

## 类别

| 类别 | 含义 | 投票后方向 |
|---|---|---|
| `green_straight` | 绿灯直行 | `straight` |
| `green_left` | 绿灯左转 | `left` |
| `green_right` | 绿灯右转 | `right` |
| `red_stop` | 红灯停止 | （不参与投票） |

## 模型

模型已随本目录打包：`light/yolov8_custom.rknn`（约 12 MB，来自 `src/ucar_nav/scripts/yolov8_custom.rknn`）。

脚本默认加载同目录下的模型，也可用 ROS 参数覆盖：

```xml
<param name="model_path" value="/path/to/yolov8_custom.rknn" />
```

## ROS 接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/traffic/start_detect` | `Bool` | 收到 `True` 后开始检测并重置投票 |
| 订阅 | `/usb_cam/image_raw` | `Image` | 摄像头图像（话题名可用 `~image_topic` 参数改） |
| 发布 | `/start_follow` | `String` | 最终行驶方向：`straight` / `left` / `right` |
| 发布 | `/traffic_light/debug_image` | `Image` | 带检测框标注的调试图像 |

## 运行

⚠️ **必须在 RK3588 小车上运行**（`rknnlite` 是板端库，模型是 `.rknn` 格式，依赖 NPU）。

```bash
# 方式一：直接跑（需先 source 工作空间）
rosrun ucar_nav traffic_light_detector.py

# 方式二：放进 launch（参考 src/ucar_nav/launch/test_traffic_light_follow.launch）
<node pkg="ucar_nav" name="traffic_light_detector" type="traffic_light_detector.py" output="screen" />
```

触发检测（模拟 `item_finder` 到达巡线路口后发的信号）：

```bash
rostopic pub -1 /traffic/start_detect std_msgs/Bool "data: true"
```

识别完成后自动发布 `/start_follow` → `line_follower` 开始巡线。

## 与 src 里 traffic_light_detector.py 的差异

- 逻辑、ROS 接口、预处理（letterbox 640×640）、后处理（多类 NMS）、加权投票完全一致。
- 唯一改动：`MODEL_PATH` 从硬编码的 `/home/iflytek/ucar_ws/...` 改为**脚本同目录下的 `yolov8_custom.rknn`**，方便本目录独立打包部署。
- 另加了一处兜底：`_finalize_vote` 里 winner 若不在三个绿灯类别中（正常不会发生，红灯已过滤），默认按 `straight` 处理，避免 `direction` 未定义报错。
