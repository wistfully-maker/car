# QR 物品连续搜索

`qr_item_search` 是 ROS1 Noetic 包：小车在物品区原地连续旋转，通过前置相机寻找三个二维码，解析二维码 URL 对应的物品名称，并发布协议 v1 搜索结果。

## 规则与搜索策略

赛事场地的四条边随机摆放二维码，每边最多一个，共三个；入口和出口所在区域禁止放置。搜索必须找到全部三个二维码才算 `complete`，不能因找到一个或两个提前成功。

搜索状态与动作如下：

1. `FAST_SWEEP`：以 `0.40 rad/s` 连续旋转约 380°，边转边扫，覆盖跨越起始方向的盲区。
2. `WAITING_HTTP`：三个 URL 都已发现时停车，等待并发 HTTP 解析完成。
3. `TARGETED_RESCAN`：根据未覆盖、过曝、模糊和已识别区域邻域生成区间，以增强图像变体进行质量补扫；解析失败可触发一次补扫重试。
4. 三个物品都解析成功后进入 `COMPLETE`；搜索耗尽或总超时进入 `NOT_FOUND`；内部、相机或航向异常进入 `ERROR`。

## 默认参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `fast_angular_speed` | `0.40 rad/s` | 快速环扫速度 |
| `targeted_angular_speed` | `0.20 rad/s` | 定向补扫速度 |
| `minimum_effective_speed` | `0.11 rad/s` | 底盘最小有效角速度 |
| `fast_sweep_angle` | `6.632251 rad` | 快速环扫角度，约 380° |
| `yaw_tolerance` | `0.035 rad` | 定向到位容差 |
| `heading_timeout` | `1.0 s` | 航向数据超时 |
| `camera_timeout` | `1.0 s` | 扫描运动状态图像超时 |
| `search_total_timeout` | `40.0 s` | 单次搜索总超时 |
| `connect_timeout` | `1.0 s` | HTTP 连接超时 |
| `read_timeout` | `2.0 s` | HTTP 读取超时 |
| `http_retries` | `1` | 首次请求失败后的重试次数 |
| `http_worker_count` | `3` | 并发 HTTP worker 数量 |
| `image_topic` | `/usb_cam/image_raw` | Scanner 订阅的相机图像话题 |

## Protocol v1

所有协议消息通过 `std_msgs/String` 携带 JSON。

开始请求：

```json
{
  "protocol_version": 1,
  "task_id": "mission-42",
  "search_id": "search-001",
  "expected_count": 3
}
```

停止请求：

```json
{
  "protocol_version": 1,
  "task_id": "mission-42",
  "search_id": "search-001",
  "reason": "operator stop"
}
```

完整结果示例：

```json
{
  "protocol_version": 1,
  "task_id": "mission-42",
  "search_id": "search-001",
  "stamp": 1720000000.25,
  "status": "complete",
  "items": [
    {"order": 1, "item_name": "香蕉", "url": "http://server/item/banana", "detected_yaw": 0.62},
    {"order": 2, "item_name": "毛巾", "url": "http://server/item/towel", "detected_yaw": 2.71},
    {"order": 3, "item_name": "手机", "url": "http://server/item/phone", "detected_yaw": 5.18}
  ],
  "message": ""
}
```

`searching` 表示搜索已接受；`complete` 必须恰有三个不同 URL；`not_found` 可携带已成功解析的部分物品；`error` 表示系统无法安全继续；`stopped` 表示匹配当前 identity 的停止请求已执行。`not_found`、`error`、`stopped` 的 `message` 非空。

协议与设计文档：

- [Protocol v1](../../../docs/superpowers/specs/2026-07-19-task1-minimal-integration-protocol.md)
- [连续环扫设计](../../../docs/superpowers/specs/2026-07-19-continuous-qr-sweep-design.md)

## ROS 节点与话题

### `qr_scanner`

订阅：

- `/usb_cam/image_raw` (`sensor_msgs/Image`)
- `/qr_item_search/scanner_control` (`std_msgs/String`)

发布：

- `/qr_item_search/scanner_event` (`std_msgs/String`)：`quality`、`detected`、`resolved`、`resolve_error` 等事件。

### `item_search_controller`

订阅：

- `/odom` (`nav_msgs/Odometry`)
- `/qr_item_search/start` (`std_msgs/String`)
- `/qr_item_search/stop` (`std_msgs/String`)
- `/qr_item_search/scanner_event` (`std_msgs/String`)

发布：

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/qr_item_search/scanner_control` (`std_msgs/String`)
- `/qr_item_search/state` (`std_msgs/String`)
- `/qr_item_search/result` (`std_msgs/String`)

## Windows 测试

本机 Windows Python 3.13 在中文路径下使用了 isolated 路径行为，环境变量方式可能不会按预期加载包。先在 PowerShell 中进入你实际使用的仓库根或工作树根，然后动态映射包目录。若 `Q:` 已被 `subst` 占用，可先执行 `subst Q: /d`；该命令只解除虚拟盘映射，不删除任何文件。

```powershell
Set-Location '你选择的仓库或工作树根目录'
$package = Join-Path $PWD 'ucar_ws\src\qr_item_search'
subst Q: /d  # 仅当 Q: 已存在 subst 映射时执行；报“找不到”可忽略
subst Q: $package
Set-Location Q:\
python -c "import sys,unittest; sys.path.insert(0,r'Q:\src'); suite=unittest.defaultTestLoader.discover(r'Q:\test',pattern='test_*.py'); result=unittest.TextTestRunner(verbosity=1).run(suite); raise SystemExit(not result.wasSuccessful())"
Set-Location $package
subst Q: /d
```

图像质量测试需要 NumPy 和 OpenCV。正式环境请安装与 Python 匹配的发行版；临时验证可将依赖放在独立目录并在上述命令中先执行：

```python
sys.path.insert(0, r'C:\tmp\qr-item-search-test-deps')
```

临时目录必须包含可正常提供 `cv2.cvtColor`、`cv2.Laplacian` 等 API 的完整 OpenCV 二进制包。

## 小车连接、构建与启动

连接小车：

```bash
ssh ucar@172.20.10.4
```

构建并加载工作空间：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
catkin_make
source /home/ucar/ucar_ws/devel/setup.bash
```

在启动搜索包前，分别启动底盘驱动和相机。下面的阻塞命令必须放在不同终端运行，不要在同一个终端中整段顺序粘贴。具体驱动 launch 名称以小车已安装包为准。

终端 A——底盘：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_controller base_driver.launch
```

终端 B——相机：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _image_width:=640 _image_height:=480 _pixel_format:=yuyv
```

终端 C——确认底盘输入：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic hz /odom
```

终端 D——确认相机输入：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic hz /usb_cam/image_raw
```

终端 E——连续搜索：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch qr_item_search qr_item_search.launch image_topic:=/usb_cam/image_raw
```

## 操作与观察

控制终端——开始搜索（一次性 publish，可以随后在同一控制终端发送 stop）：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic pub -1 /qr_item_search/start std_msgs/String "data: '{\"protocol_version\":1,\"task_id\":\"mission-42\",\"search_id\":\"search-001\",\"expected_count\":3}'"
```

停止当前搜索：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic pub -1 /qr_item_search/stop std_msgs/String "data: '{\"protocol_version\":1,\"task_id\":\"mission-42\",\"search_id\":\"search-001\",\"reason\":\"operator stop\"}'"
```

以下观察命令都是阻塞的，分别使用独立终端。

状态终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic echo /qr_item_search/state
```

结果终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic echo /qr_item_search/result
```

速度终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic echo /cmd_vel
```

图像终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rqt_image_view /usb_cam/image_raw
```

## Scanner-only 无运动验证

安全联调应先只启动 scanner，不启动 controller，也不要启动整个包的 launch。相机和 scanner 分别使用独立终端；如果只验证视觉链路，可以不启动底盘驱动。

终端 A——相机：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _image_width:=640 _image_height:=480 _pixel_format:=yuyv
```

终端 B——仅 scanner：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosrun qr_item_search qr_scanner_node.py _image_topic:=/usb_cam/image_raw
```

此模式没有 `/cmd_vel` publisher，不会主动控制底盘。检查：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic info /cmd_vel
```

终端 C——scanner 事件：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic echo /qr_item_search/scanner_event
```

手动启用 scanner：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic pub -1 /qr_item_search/scanner_control std_msgs/String "data: '{\"protocol_version\":1,\"task_id\":\"manual\",\"search_id\":\"scanner-only-1\",\"enabled\":true,\"enhanced\":true,\"detected_yaw\":0.0,\"retry_failed\":false}'"
```

停止 scanner 接帧：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic pub -1 /qr_item_search/scanner_control std_msgs/String "data: '{\"protocol_version\":1,\"task_id\":\"manual\",\"search_id\":\"scanner-only-1\",\"enabled\":false,\"enhanced\":false,\"detected_yaw\":0.0,\"retry_failed\":false}'"
```

## 实车三二维码流程

1. 先在开阔区域架起驱动轮或保持急停可立即触发，完成 scanner-only 无运动验证。
2. 准备三个不同 HTTP/HTTPS URL 的打印二维码；对应服务返回 `{"code": 200, "result": "物品名"}`。
3. 按赛事约束放在四边中的三边，每边不超过一个，避开入口和出口区域。
4. 检查小车能访问三个 URL，确认 `/odom` 和相机图像稳定。
5. 放下小车，清空旋转半径内人员和物品，启动底盘、相机与搜索包。
6. 发送唯一 `task_id/search_id` 的 start JSON，同时观察 state、result、速度与图像。
7. 预期先 FAST 环扫；发现三个 URL 后停车等待 HTTP；若覆盖或质量不足则进入 TARGET 补扫。
8. 收到 `complete` 后核对三个 item 的顺序、URL、名称和检测角度。若得到 `NOT_FOUND` 或 `ERROR`，先保持停车，再查相机、航向、HTTP 和日志。

## 安全与成像建议

- 第一次带运动测试必须在开阔区进行，急停按钮始终可触达；先验证零运动，再允许底盘旋转。
- 测试前后观察 `/cmd_vel`，任何异常立即急停并停止节点。
- 强反光会增加过曝，逆光和阴影会降低对比度；保持均匀照明，必要时给相机加消光遮光罩。
- 使用哑光纸打印高对比度二维码，保留足够白边，避免覆膜反光、折痕和过小码面。
- `NOT_FOUND` 表示在限制时间和补扫区间内未完成三个结果，不等同于系统故障；`ERROR` 表示安全相关输入、发布或内部处理异常。

## 当前范围

当前包只负责物品区连续二维码搜索与结果发布，不包含 LLM、语音交互、机械抓取或后续导航。
