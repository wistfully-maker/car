# QR 物品搜索阶段性说明

本文记录 `qr_item_search` 包截至 2026-07-18 的实现、参数、命令和实车验证结果，供后续模拟赛场标定与联调使用。

## 1. 当前完成范围

当前已经实现：

- 从 `/usb_cam/image_raw` 接收前置摄像头图像。
- 使用 `pyzbar` 和系统 `libzbar` 解码普通二维码。
- 对连续 2 帧中的相同内容进行稳定确认。
- 校验二维码内容为 HTTP/HTTPS URL。
- 请求 URL、解析 JSON，并从 `result` 字段取得物品名称。
- 使用 `/odom` 航向闭环控制 `/cmd_vel`，依次观察三个配置朝向。
- 旋转期间关闭扫码，到位停车并等待画面稳定后才开启扫码。
- 识别成功后等待上层给出“匹配/不匹配”决定。
- 匹配时立即成功停车；不匹配时转向下一面墙。
- 当前墙识别到无效结果后立即进入下一墙，不再等待本墙超时。
- 扫描超时、旋转超时、人工停止或节点关闭时发布零速度。

当前尚未实现：

- 语音指令解析。
- 云端或本地 LLM 物品分类。
- 机械取物。
- 找到物品后的下一目标点导航。
- 模拟比赛场地三面墙的最终角度标定。

## 2. 二维码数据流程

二维码中保存的是 URL，不直接保存物品名称。例如：

```text
http://172.20.10.7:8000/item/food
```

扫码节点的处理流程：

```text
ROS Image
  -> cv_bridge 转为 OpenCV BGR 图像
  -> pyzbar/libzbar 解出 URL
  -> 连续帧确认及去重
  -> HTTP GET
  -> 校验 JSON
  -> 取得 result
  -> 发布 /qr_item_search/observation
```

测试服务返回格式：

```json
{
  "code": 200,
  "result": "香蕉"
}
```

成功 observation 示例：

```json
{
  "status": "success",
  "search_id": 1,
  "wall_index": 0,
  "url": "http://172.20.10.7:8000/item/food",
  "item_name": "香蕉",
  "message": ""
}
```

## 3. ROS 节点与话题

### `qr_scanner`

订阅：

- `/usb_cam/image_raw`：相机图像。
- `/qr_item_search/scan_enabled`：是否允许当前帧参与识别。
- `/qr_item_search/wall_index`：当前墙面编号。
- `/qr_item_search/reset`：当前搜索编号 `search_id`。

发布：

- `/qr_item_search/observation`：JSON 字符串形式的扫码结果。

### `item_search_controller`

订阅：

- `/odom`：底盘航向反馈。
- `/qr_item_search/start`：开始一次搜索。
- `/qr_item_search/observation`：扫码结果。
- `/qr_item_search/match_decision`：上层匹配判断。

发布：

- `/cmd_vel`：只使用 `angular.z` 控制原地旋转。
- `/qr_item_search/scan_enabled`：控制扫码窗口。
- `/qr_item_search/wall_index`：当前墙面编号。
- `/qr_item_search/reset`：新搜索的 `search_id`。
- `/qr_item_search/state`：控制器状态。

状态包括：

```text
IDLE
TURNING
SETTLING
SCANNING
WAITING_MATCH
SUCCESS
NOT_FOUND
ERROR
```

## 4. 当前默认参数

以下是 `launch/qr_item_search.launch` 中的包默认值：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `wall_yaw_offsets` | `[0.0, 1.5708, 3.1416]` | 相对搜索入口航向的三个观察角，约为 0°、90°、180° |
| `yaw_kp` | `1.2` | 航向比例控制系数 |
| `max_angular_speed` | `0.30 rad/s` | 最大旋转角速度，约 17.2°/s |
| `min_angular_speed` | `0.11 rad/s` | 未进入容差时的最小有效角速度，约 6.3°/s |
| `yaw_tolerance` | `0.035 rad` | 到位角度容差，约 2.0° |
| `settle_seconds` | `0.8 s` | 到位停车后的画面稳定等待时间 |
| `scan_timeout` | `4.0 s` | 每面墙默认扫码窗口 |
| `turn_timeout` | `8.0 s` | 单次转向默认最大持续时间 |
| `required_frames` | `2` | 相同二维码连续确认帧数 |
| `connect_timeout` | `1.0 s` | HTTP 连接超时 |
| `read_timeout` | `2.0 s` | HTTP 读取超时 |
| `http_retries` | `2` | HTTP 重试次数 |
| `image_topic` | `/usb_cam/image_raw` | 相机图像话题 |

### 最小有效角速度的原因

U-CAR 底盘驱动会把速度换算成整数编码器脉冲。比例控制接近目标时，若角速度降到约 `0.1 rad/s` 以下，单周期结果可能不足一个脉冲并被取整为 0，导致小车停在容差外，最终触发旋转超时。

当前控制策略为：

```text
误差在容差内：
  angular.z = 0

误差在容差外：
  angular.z = clamp(kp * error, -max_speed, max_speed)
  若绝对值小于 min_speed，则保持同方向 min_speed
```

### 2026-07-18 空旷地测试临时参数

三朝向实车测试时，为方便人工移动手机，临时覆盖为：

```text
scan_timeout = 8.0 s
turn_timeout = 10.0 s
```

这些不是包默认值。正式场地二维码固定后，应先使用默认值测试，再依据实测数据优化。

相机测试参数：

```text
设备：/dev/video0
分辨率：640 x 480
格式：yuyv
IO：mmap
请求帧率：30 FPS
实测 ROS 图像频率：约 20 FPS
```

## 5. 三面墙角度约束

当前默认配置不是旋转 360°或270°，而是依次到达三个相对于搜索入口航向的绝对角：

```text
墙面 0：0°
墙面 1：90°
墙面 2：180°
```

因此默认情况下，从第一面到第三面累计转动约 180°。三个角度只是开发默认值，并非最终比赛场地标定值。

进入模拟比赛场地后必须确定：

1. 固定观察点。
2. 到达观察点时的车头基准朝向。
3. 上、左、下三面二维码墙的实际相对航向。
4. 三面墙的合理扫描顺序。

再把实测角度写入 `wall_yaw_offsets`。三面墙不要求严格相差 90°。

## 6. 测试网址与二维码

Windows 电脑当前 Wi-Fi 地址：

```text
172.20.10.7
```

测试端点：

```text
http://172.20.10.7:8000/item/food         -> 香蕉
http://172.20.10.7:8000/item/daily        -> 毛巾
http://172.20.10.7:8000/item/electronics  -> 手机
```

测试二维码位于仓库：

```text
test_qr_codes/food_banana.png
test_qr_codes/daily_towel.png
test_qr_codes/electronics_phone.png
```

启动 Windows 测试服务：

```powershell
cd D:\program_sec\智能车\.worktrees\qr-item-search
python tools\qr_test_server.py
```

运行服务的窗口必须保持打开。手机、小车和电脑必须处于同一热点或局域网。

电脑验证：

```powershell
Invoke-WebRequest -UseBasicParsing `
  -Uri http://172.20.10.7:8000/item/food
```

小车验证：

```bash
curl -fsS --max-time 3 \
  http://172.20.10.7:8000/item/food
```

## 7. 构建与测试命令

### Windows 本地测试

从包的源码目录运行：

```powershell
cd D:\program_sec\智能车\.worktrees\qr-item-search\ucar_ws\src\qr_item_search\src
python -c "import os,sys,unittest; sys.path.insert(0,os.getcwd()); suite=unittest.defaultTestLoader.discover('../test', pattern='test_*.py'); result=unittest.TextTestRunner(verbosity=1).run(suite); raise SystemExit(not result.wasSuccessful())"
```

截至 2026-07-18，共有 90 项测试。

### 小车 Catkin 构建

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
catkin_make
source /home/ucar/ucar_ws/devel/setup.bash
```

### 小车单元测试

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
python3 -m unittest discover \
  -s /home/ucar/ucar_ws/src/qr_item_search/test \
  -p 'test_*.py'
```

## 8. 启动命令

### 8.1 启动底盘驱动

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch ucar_controller base_driver.launch
```

启动后检查：

```bash
rostopic hz /odom
rostopic info /cmd_vel
```

`/odom` 实测约 20 Hz。

### 8.2 启动相机

```bash
source /opt/ros/noetic/setup.bash
rosrun usb_cam usb_cam_node \
  _video_device:=/dev/video0 \
  _image_width:=640 \
  _image_height:=480 \
  _pixel_format:=yuyv \
  _camera_frame_id:=usb_cam \
  _io_method:=mmap
```

检查：

```bash
rostopic hz /usb_cam/image_raw
```

### 8.3 一次启动扫码和三墙控制器

确保底盘与相机已经启动，然后：

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch qr_item_search qr_item_search.launch
```

覆盖墙面角度示例：

```bash
roslaunch qr_item_search qr_item_search.launch \
  wall_yaw_offsets:='[0.0,1.5708,3.1416]'
```

### 8.4 分开启动节点

只启动扫码节点，不控制底盘：

```bash
rosrun qr_item_search qr_scanner_node.py
```

只启动控制器：

```bash
rosrun qr_item_search item_search_controller_node.py \
  _wall_yaw_offsets:='[0.0,1.5708,3.1416]' \
  _max_angular_speed:=0.30 \
  _min_angular_speed:=0.11 \
  _yaw_kp:=1.2 \
  _yaw_tolerance:=0.035 \
  _settle_seconds:=0.8 \
  _scan_timeout:=4.0 \
  _turn_timeout:=8.0
```

## 9. 运行与调试命令

开始一次三墙搜索：

```bash
rostopic pub -1 \
  /qr_item_search/start \
  std_msgs/Empty '{}'
```

监听状态：

```bash
rostopic echo /qr_item_search/state
```

监听识别结果：

```bash
rostopic echo /qr_item_search/observation
```

上层判断当前物品匹配：

```bash
rostopic pub -1 \
  /qr_item_search/match_decision \
  std_msgs/Bool 'data: true'
```

上层判断当前物品不匹配，继续下一面墙：

```bash
rostopic pub -1 \
  /qr_item_search/match_decision \
  std_msgs/Bool 'data: false'
```

查看当前墙面：

```bash
rostopic echo /qr_item_search/wall_index
```

查看实时速度：

```bash
rostopic echo /cmd_vel
```

查看底盘实际反馈：

```bash
rostopic echo /odom/twist/twist
```

## 10. 停止与安全命令

测试前必须保证小车位于空旷区域。

手动发布零速度：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

关闭二维码控制器与扫码节点：

```bash
rosnode kill /item_search_controller /qr_scanner
```

关闭相机：

```bash
rosnode kill /usb_cam
```

关闭底盘驱动：

```bash
rosnode kill /base_driver
```

最终检查：

```bash
rosnode list
rostopic info /cmd_vel
rostopic info /usb_cam/image_raw
rostopic info /odom
```

安全状态应满足：

- `/item_search_controller` 不存在。
- `/cmd_vel` 没有发布者。
- `/odom/twist/twist` 在关闭底盘前为零。
- 相机和扫码测试结束后不再保留截图诊断节点。

## 11. 阶段性实车结果

截至 2026-07-18：

- Windows 与小车均能用 `pyzbar/libzbar` 解码三张测试二维码。
- 小车能够通过二维码 URL 从电脑取得香蕉、毛巾和手机 JSON。
- 修复最小角速度前，小车会因编码器脉冲死区停在目标容差外。
- 加入 `min_angular_speed=0.11` 后，11.5°目标实测误差约 1.79°，进入配置的约 2°容差。
- 空旷地完成两个约 90°的连续闭环转向，没有进入 `ERROR`。
- 墙面 1 转向后成功识别毛巾，并正确等待匹配决定。
- 发送“不匹配”后正确转向墙面 2。
- 墙面 2 静态复测成功识别香蕉，发送“匹配”后进入 `SUCCESS`。
- 使用手机显示二维码时，人工移动和摩尔纹会影响实时扫描；固定墙面二维码应更稳定。

## 12. 下一阶段

进入模拟比赛场地后：

1. 选择固定物品区观察点。
2. 固定小车到达观察点时的车头方向。
3. 实测三面墙的相对航向。
4. 更新 `wall_yaw_offsets`。
5. 测量每面墙二维码在 640×480 画面中的大小和清晰度。
6. 重新评估 `settle_seconds` 与 `scan_timeout`。
7. 完成“导航到物品区 → 三墙搜索 → 匹配后结束搜索”的联调。
8. 稳定后再进行竞速优化。
