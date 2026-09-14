# QR 持续扫码搜索：运行、调试与联调手册

`qr_item_search` 是 U-CAR-02 上的 ROS 1 Noetic 包。小车到达物品领取区观察点后，以航向闭环快速转到固定观察角（默认每 45° 一站），每站确认停稳后至少驻留 `scan_window` 秒；**从任务开始到进入终态前，二维码解码持续运行**——转向、停稳、驻留期间都接收并解码最新相机帧，识别到新二维码立即异步发起 HTTP 请求，底盘继续转动，直到取得三个不同二维码并全部解析出物品名称。

> 当前包只实现“定点停靠 + 持续扫码 + HTTP 解析 + 结果发布”。语音理解、任务编排器、LLM 分类、语音播报、抓取和下一目标点导航尚未包含在本包中。

## 1. 架构与状态机

```text
IDLE
  -> INITIAL_SCAN
  -> TURNING
  -> SETTLING
  -> SCANNING       此状态表示停稳驻留，不再表示唯一允许扫码的窗口
  -> TURNING
  -> OFFSET_PASS
  -> WAITING_HTTP
  -> COMPLETE / NOT_FOUND / ERROR
```

- 第一站是启动朝向 `0°`，无需先转动；第一圈观察角 `0°、45°、…、315°`。
- 第二圈（偏移圈）观察角 `22.5°、67.5°、…、337.5°`，只运行到补齐三个 URL 或第二圈结束。
- 到达目标角度容差后发布零速度进入 `SETTLING`；里程计角速度连续低于阈值满 `settled_duration` 后才进入 `SCANNING` 驻留。
- **扫码使能规则**：`IDLE/COMPLETE/NOT_FOUND/ERROR/STOPPED` 时关闭；从任务进入 `INITIAL_SCAN` 起至任务进入终态前一直开启，不随 `TURNING/SETTLING/SCANNING` 开关。
- **持续扫码**：解码线程忙时只保留最新帧（无帧积压），URL 在转向途中识别同样有效；慢解码跨过站点边界只要仍属同一搜索任务（同一 `task_id/search_id`）结果照常接受；上一搜索任务的慢结果仍会被 generation/identity 丢弃。
- **驻留**：`scan_window` 是每站最短稳定驻留时间，不再控制扫码开关。驻留期间识别到新 URL 不提前结束本站，默认驻留满后前往下一站；第三个不同 URL 例外——立即发布零速度并进入 `WAITING_HTTP`。
- **HTTP 并行**：识别到新 URL 立即发布 `detected` 并交给 HTTP worker，底盘继续转动；绝不原地等待网络返回。
- `WAITING_HTTP`：三个 URL 已取得后停车等待，全部解析成功才发布 `complete`；最终仍失败发布 `not_found` 并给出原因，不把部分结果伪装成完成。
- 任意停止、超时、异常、ROS shutdown 都会先发布零速度，再关闭扫码。

## 2. 目录结构

```text
launch/qr_item_search.launch          一键启动（scanner + controller，可选调试流）
scripts/qr_scanner_node.py            USB 相机订阅、解码、HTTP、关键帧、frame_seen 心跳
scripts/item_search_controller_node.py 航向控制状态机、协议、指标
scripts/qr_debug_stream_node.py       可选浏览器实时画面（默认关闭）
src/qr_item_search/
  search_state.py                     状态机
  scan_schedule.py                    角度序列
  yaw_control.py                      分档转向命令
  settling.py                         停稳判定
  controller_logic.py                 控制器核心（纯逻辑，可本地测试）
  scanner_logic.py                    扫码核心（纯逻辑，可本地测试）
  qr_decode.py                        pyzbar/ZBar 解码
  qr_payload.py                       URL 校验与 HTTP JSON 解析
  protocol.py                         protocol v1 消息校验
  image_quality.py                    画面质量与解码变体（含 decode_scale）
  keyframes.py                        关键帧异步保存
  run_metrics.py                      结构化指标 JSONL 写入
  debug_stream.py                     MJPEG HTTP 服务器与叠加层
```

## 3. 相机所有权与分辨率

- **相机由外部公共节点启动**（例如 `rosrun usb_cam usb_cam_node ...`），QR launch 不启动、不配置、不重配摄像头，只订阅已有 `/usb_cam/image_raw`。
- 相机输入分辨率与格式由外部相机节点决定；小车现有 `car_server/ucar_camera.launch` 默认为 **640×480 MJPEG**。QR 节点内部的 `decode_scale` 只是解码预处理（软件放大），**不是物理分辨率**；软件放大不增加真实采样细节，只可能改善部分解码器的采样行为。
- 调试流始终显示相机原始画面（可叠加状态文字），不伪装成分辨率提升。

## 4. 安全前提

启动完整搜索会向 `/cmd_vel` 发布角速度，小车会原地旋转。运行前必须：

- 将小车放在物品区观察点附近，清空旋转范围内人员、线缆和障碍物。
- 保证急停或电源开关可立即触达。
- 确认没有其他节点同时控制 `/cmd_vel`。
- 先检查 `/odom` 和 `/usb_cam/image_raw` 持续有数据。
- 第一次调试先执行“Scanner-only 无运动测试”，再执行带底盘运动的完整搜索。

## 5. 连接、编译和环境加载

从电脑连接小车：

```bash
ssh ucar@192.168.1.109
```

首次部署、源码更新后或编译产物不存在时执行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make
source ~/ucar_ws/devel/setup.bash
```

每个新终端至少执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
```

## 6. 一键运行

这里的“一键”是指底盘驱动、相机和 ROS Master 已经启动后，用一个 launch 同时启动本包的 scanner 与 controller：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch qr_item_search qr_item_search.launch image_topic:=/usb_cam/image_raw
```

该 launch 启动：

- `/qr_scanner`
- `/item_search_controller`

它不会自动启动底盘驱动和 USB 相机。执行 launch 后节点处于 `IDLE`，收到 `/qr_item_search/start` 后才开始搜索。正式比赛默认不启动调试流（`start_debug_stream:=false`）。

## 7. 从零开始分步运行

以下阻塞命令必须分别放在独立 SSH 终端。

### 终端 A：底盘驱动

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_controller base_driver.launch
```

### 终端 B：USB 相机

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosrun usb_cam usb_cam_node \
  _video_device:=/dev/video0 \
  _image_width:=640 \
  _image_height:=480 \
  _pixel_format:=mjpeg
```

以上参数与小车当前 `car_server/launch/ucar_camera.launch` 的默认图像规格一致；若公共相机 launch 已经启动，不要重复运行此命令。

### 终端 C：QR scanner

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosrun qr_item_search qr_scanner_node.py _image_topic:=/usb_cam/image_raw
```

### 终端 D：搜索 controller

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosrun qr_item_search item_search_controller_node.py
```

分步运行适合定位某个节点的问题；正常使用优先采用上一节的 launch。

## 8. Windows 浏览器实时画面（调试用）

调试流是独立的旁路节点，默认关闭，不参与正式搜索，也绝不重新打开摄像头（只订阅相机已有的话题）。

一键启动时开启：

```bash
roslaunch qr_item_search qr_item_search.launch \
  start_debug_stream:=true \
  debug_host:=0.0.0.0 \
  debug_port:=8080
```

在 Windows 浏览器打开：

- `http://<小车IP>:8080/`：实时画面，叠加状态、相对/目标航向、停稳标志、已识别数量（`x/3`）与图像质量。
- `http://<小车IP>:8080/snapshot.jpg`：当前帧单张 JPEG。
- `http://<小车IP>:8080/stream.mjpg`：MJPEG 视频流。

分步启动调试流：

```bash
rosrun qr_item_search qr_debug_stream_node.py \
  _image_topic:=/usb_cam/image_raw _host:=0.0.0.0 _port:=8080
```

调试流基于 Python 标准库 HTTP server，不依赖 Flask；帧率默认限制 10 fps（`~max_fps`），编码质量默认 80（`~jpeg_quality`）。

## 9. 启动一次搜索

### 9.1 运行前检查

```bash
rostopic hz /odom
rostopic hz /usb_cam/image_raw
rosnode list
rostopic info /cmd_vel
```

`/odom` 和图像话题应持续输出。检查 `/cmd_vel` 时确认不存在非预期控制节点。

### 9.2 手动发送开始信号

```bash
rostopic pub -1 /qr_item_search/start std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"search_id\":\"search-test-001\",\"expected_count\":3}'"
```

规则：

- 一次完整任务使用一个 `task_id`。
- 每次新的搜索使用新的 `search_id`。
- `expected_count` 当前必须是 `3`。
- 重复发送同一个 `search_id` 不会启动第二次旋转。
- 搜索失败或 scanner 重启后再次尝试，必须换新的 `search_id`。

### 9.3 观察状态和结果

```bash
rostopic echo /qr_item_search/state
rostopic echo /qr_item_search/result
rostopic echo /cmd_vel
```

结果状态含义：

| `status` | 含义 | 上层应做什么 |
|---|---|---|
| `searching` | 已接受任务，正在搜索 | 等待最终结果 |
| `complete` | 三个物品都已解析 | 才能请求 LLM 分类 |
| `not_found` | 超时或两圈后仍不足三个 | 不调用 LLM，报告失败或生成新搜索 |
| `error` | 相机、航向、停稳、协议或内部安全错误 | 停止流程，排查后用新 `search_id` 重试 |
| `stopped` | 收到人工或上层停止指令 | 等待新任务 |

## 10. 怎么停止，以及是否需要手动关闭节点

### 10.1 只停止当前搜索

发送与当前任务匹配的 `task_id` 和 `search_id`：

```bash
rostopic pub -1 /qr_item_search/stop std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"search_id\":\"search-test-001\",\"reason\":\"operator_stop\"}'"
```

这会停车并结束当前搜索，但不会关闭 ROS 节点。

### 10.2 搜索正常结束后要不要关闭节点

- 连续进行多次任务：不需要关闭；收到终态后 controller 已发布零速度，可保留节点等待下一次任务。
- 本轮调试结束、准备关机或移动小车：需要关闭本包、相机和底盘节点。

### 10.3 正常关闭

1. 若搜索仍在进行，先发送 stop。
2. 在运行 `roslaunch qr_item_search ...` 的终端按 `Ctrl+C`。
3. 在相机和底盘终端分别按 `Ctrl+C`。
4. 确认 QR 节点已消失：

```bash
rosnode list | grep -E 'qr_scanner|item_search_controller|qr_debug_stream'
```

无输出表示 QR 节点已关闭。

### 10.4 找不到原终端时关闭

先停止当前搜索，再执行：

```bash
rosnode kill /qr_scanner /item_search_controller
```

调试流节点按实际节点名一并关闭。`rosnode kill` 是异常收尾手段，正常情况优先在 launch 终端按 `Ctrl+C`，这样 shutdown 回调能够发布零速度并清理线程。

### 10.5 紧急情况

人员或物体进入旋转范围时，优先使用实体急停/电源开关，不要只依赖网络命令。

## 11. Scanner-only 无运动测试

此模式只测试相机、二维码解码和 HTTP，不启动 controller，因此本包不会发布 `/cmd_vel`。

启动相机后，仅运行：

```bash
rosrun qr_item_search qr_scanner_node.py _image_topic:=/usb_cam/image_raw
```

观察 scanner 事件：

```bash
rostopic echo /qr_item_search/scanner_event
```

手动启用：

```bash
rostopic pub -1 /qr_item_search/scanner_control std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"manual\",\"search_id\":\"scanner-only-001\",\"enabled\":true,\"enhanced\":true,\"detected_yaw\":0.0,\"retry_failed\":false}'"
```

## 12. scanner 事件：`frame_seen` / `quality` / `detected` / `resolved` 的区别

| 事件 | 谁发布 | 含义 |
|---|---|---|
| `frame_seen` | scanner 图像回调 | 原始图像成功转换并提交给扫码逻辑（按单调时间节流到 5 Hz）。**相机存活的唯一证明** |
| `quality` | scanner 解码线程 | 每处理一帧输出亮度/过曝/清晰度，用于指标与画面质量，**不承担相机存活证明** |
| `detected` | scanner 解码线程 | 某帧解码出一个本搜索尚未记录的新 URL，并已交给 HTTP worker |
| `resolved` | scanner HTTP worker | 该 URL 的 HTTP 请求成功返回非空物品名 |
| `resolve_error` | scanner HTTP worker | 该 URL 解析失败（网络、JSON、业务码等），可重试 |

**相机心跳与超时**：controller 的 `camera_timeout` 只检查属于当前任务的 `frame_seen`。新任务开始时心跳重置并从任务开始获得完整的 `camera_timeout` 宽限；只要原始图像仍到达（哪怕解码线程阻塞、哪怕转向超过 1 秒），就不会报相机超时；图像真正断流超过 `camera_timeout` 才停车进入 `ERROR`。

## 13. 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `step_angle_deg` | 45.0 | 第一圈站点间隔（度），必须整除 360 |
| `cruise_angular_speed` | 0.50 | 主旋转角速度 rad/s |
| `approach_angular_speed` | 0.20 | 近目标角速度 rad/s |
| `approach_zone_deg` | 10.0 | 减速区（度） |
| `yaw_tolerance_deg` | 2.0 | 到达容差（度） |
| `settled_angular_speed` | 0.03 | 停稳角速度阈值 rad/s |
| `settled_duration` | 0.20 | 连续停稳时间 s |
| `settling_timeout` | 3.0 | 停稳判定超时 s，超时进入 `ERROR` |
| `scan_window` | 0.60 | 每站最短稳定驻留时间 s（不再控制扫码开关） |
| `offset_angle_deg` | 22.5 | 第二圈偏移（度） |
| `max_passes` | 2 | 最大扫描圈数 |
| `search_total_timeout` | 60.0 | 单次搜索总超时 s |
| `heading_timeout` | 1.0 | 航向数据超时 s |
| `camera_timeout` | 1.0 | 原始图像心跳超时 s（只查 `frame_seen`） |
| `decode_scale` | 1.5 | scanner 内部解码放大倍率（1.0 / 1.5 / 2.0，最大 2.0） |
| `image_topic` | `/usb_cam/image_raw` | 相机图像话题 |
| `metrics_dir` | `~/qr_metrics` | 结构化指标输出目录 |
| `keyframe_dir` | `~/qr_keyframes` | 关键帧输出目录 |
| `connect_timeout` / `read_timeout` / `http_retries` / `http_worker_count` | 1.0 / 2.0 / 1 / 3 | HTTP 参数（scanner 节点） |
| `start_debug_stream` | false | 是否启动调试流节点 |
| `debug_host` / `debug_port` | 0.0.0.0 / 8080 | 调试流监听地址 |

`decode_scale` 说明：

- 原始帧总是第一个解码变体；`decode_scale > 1.0` 时在其后追加一个有界放大变体（不修改收到的 ROS 图像），再依次是灰度/CLAHE/阈值变体；放大变体失败会跳过并继续其余路径。同一帧会合并所有变体解出的不同 URL，不会因某个变体先识别出一个二维码而跳过后续变体；收齐三个不同 URL 后才提前结束本帧处理。
- 默认 1.5 依据本机性能测试：640×480 下放大到 1.5× 的 resize 开销约 0.5 ms，解码开销约 2.7×（2.0× 时约 4.6×）；持续扫码只保留最新帧，不会产生积压。
- 软件放大**不增加真实采样细节**，只可能改善部分解码器的采样行为；调 `2.0` 前先确认 CPU 与帧率余量。

launch 覆盖示例（所有控制参数均为 launch arg，可从命令行覆盖）：

```bash
roslaunch qr_item_search qr_item_search.launch \
  image_topic:=/usb_cam/image_raw \
  scan_window:=0.60 \
  cruise_angular_speed:=0.50 \
  decode_scale:=1.5 \
  metrics_dir:=/home/ucar/qr_metrics \
  keyframe_dir:=/home/ucar/qr_keyframes
```

分步运行 controller 时临时覆盖参数：

```bash
rosrun qr_item_search item_search_controller_node.py \
  _step_angle_deg:=45.0 _cruise_angular_speed:=0.50 _scan_window:=0.60
```

## 14. 日志与关键帧

- **结构化指标**：每轮搜索结束后在 `metrics_dir/qr_search_runs.jsonl` 追加一行 JSON（UTF-8，中文不转义）。字段包括 `schema`、`task_id`、`search_id`、`config`（全部参数）、`started_at/finished_at/total_seconds`、`terminal_status`、`message`、`state_seconds`、`stations`、`items`。`stations` 中每站至少包含 `pass`、`index`、`target_yaw_deg`、`actual_yaw_deg`、`yaw_error_deg`、`settling_seconds`、`dwell_seconds`、`frame_count`、`quality`、`decoded_url`。
- **关键帧**：`keyframe_dir` 下只保存两类图片——成功站关键帧 `success_<search_id>_p<圈>_s<站>_<时间戳>.jpg`（二维码位置用绿框标注）和失败站最后一帧 `failed_..._p..._s..._<时间戳>.jpg`。成功关键帧必须是真正产生该 URL 的原始帧，帧所属角度使用采集时的航向（不是解码完成时车身航向）。文件名中的 `search_id` 已做安全化处理，目录与文件名均来自参数，不写死。
- 指标或关键帧写入失败只记录 warning，不会阻塞速度控制或解码；“本站未识别”不会阻断持续扫码线程。
- 实车调参时建议同时记录 result、scanner_event 与关键帧，离线核对每站实际航向与图像。

## 15. 单变量调参纪律与实车实验顺序

不要同时调整多个变量。默认 45° 只有在有效解码视场不小于 55° 时才进入正式测试；不足时必须先报告并改用 30°（`step_angle_deg:=30.0`），不能用延长驻留时间掩盖覆盖不足。

实车顺序（由 Codex 在部署验收后执行）：

1. **静态预览**：Scanner-only 模式，确认三张码静止可解码、画面不糊不过曝。
2. **有效解码视场测量**：二维码居中后记录左右两侧仍能连续解码的极限航向，视场须 ≥ 55°（45° 步长）。
3. **小角度安全旋转**：`step_angle_deg` 保持 45°，先小角度验证转向/停稳/驻留/持续扫码闭环。
4. **一圈空载**：场地不放码，跑完一圈验证各站转向、停稳与驻留结束都正常，且全程不出现相机超时误报。
5. **三张码**：放置三张码，跑完整识别流程，检查 `complete` 与关键帧。
6. **参数矩阵**：固定 45° 与 `scan_window:=0.60`，测试 `cruise_angular_speed` 0.40 / 0.50 / 0.60；固定最优速度，测试 `scan_window` 0.40 / 0.60 / 0.80。每组连续 10 轮，至少 9 轮 3/3 且 10 轮全部安全停稳才合格；合格组合再比较平均、P95 与最慢总耗时。

实验记录表见 `docs/parameter-experiment-template.csv`，表头：

```csv
date,step_angle_deg,cruise_speed,scan_window,run_index,status,items_found,total_seconds,p95_station_seconds,max_yaw_error_deg,safe_stop,notes
```

## 16. 故障判断

### 相机断流

- 症状：`result` 为 `error`、`message` 为 `camera timed out`，且是搜索进行中（非刚开始）出现。
- 确认：`rostopic hz /usb_cam/image_raw` 是否停止；`rostopic echo /qr_item_search/scanner_event` 是否还有 `frame_seen`。
- 处理：恢复相机节点后，用新的 `search_id` 重新开始。注意：只要 `frame_seen` 仍在到达（例如解码线程卡住、转向持续超过 1 秒），不会触发该错误。

### 二维码可定位但不可解码

- 症状：调试流中能看到二维码、OpenCV 能框出四角，但多轮驻留后该方向始终没有 `detected`；关键帧中二维码区域像素不足或模糊。
- 排查：检查关键帧里二维码区域尺寸（当前实测约 120×130 像素时 OpenCV/pyzbar 均无法恢复内容）；先检查固定焦距、曝光、码面尺寸与距离，再尝试 `decode_scale:=2.0`。软件放大不增加真实采样细节，不能把“放大后仍不可解码”当作已解决。

### 网络解析失败

- 症状：`detected` 已发布但 `resolved` 迟迟不来，或出现 `resolve_error`；三个 URL 齐了后停在 `WAITING_HTTP`，最终 `not_found` 且 `message` 含 `items unresolved`。
- 确认：在小车上用 `curl '<二维码URL>'` 验证网络和 JSON（`code: 200` 且 `result` 非空）。
- 处理：检查小车网络、服务器可达性与 JSON 格式；单次失败会有限重试，最终失败不会被伪装成完成。

### 其他常见问题

**没有旋转**：检查是否已发送 start；检查 `/odom` 频率与 `/cmd_vel` 订阅；查看 state/result 的错误信息。

**停在某站一直不进入驻留**：状态停在 `SETTLING` 时，`settled_angular_speed` 阈值过严或里程计角速度噪声过大会触发 `settling timeout` 进入 `ERROR`；检查 `settled_duration` 与 `settling_timeout` 的比值。

**能看到二维码但没有结果**：确认二维码内容是完整 HTTP/HTTPS URL；确认返回 `code: 200` 且 `result` 非空；手机屏幕展示二维码时提高亮度但避免反光。

**画面白花或过曝**：避免太阳直射镜头和二维码表面；使用哑光打印纸，保留二维码白边；调整观察点或增加遮光罩。

**收到 `scanner restarted`**：scanner 在活动搜索中重启后，controller 会安全停车并进入 `ERROR`。确认 scanner 已恢复，然后使用新的 `search_id` 重新开始。

**已结束但节点仍在**：这是正常行为。终态只结束一次搜索，不退出常驻节点。继续测试可直接发送新的 start；准备关机则按第 10 节关闭节点。

## 17. Windows 本地测试

在仓库或 worktree 根目录运行：

```powershell
$package = Join-Path $PWD 'ucar_ws\src\qr_item_search'
subst Q: $package
Set-Location Q:\
python -c "import sys,unittest; sys.path.insert(0,r'Q:\src'); suite=unittest.defaultTestLoader.discover(r'Q:\test',pattern='test_*.py'); result=unittest.TextTestRunner(verbosity=1).run(suite); raise SystemExit(not result.wasSuccessful())"
Set-Location $package
subst Q: /d
```

图像与关键帧测试需要与当前 Python 匹配的 NumPy 和 OpenCV。

小车端测试：

```bash
cd ~/ucar_ws/src/qr_item_search
PYTHONPATH=~/ucar_ws/src/qr_item_search/src \
python3 -m unittest discover -s test -p 'test_*.py' -q
```

## 18. 与编排器联调的 topic 协议（protocol v1，保持不变）

- 订阅：`/qr_item_search/start`、`/qr_item_search/stop`（String JSON）
- 发布：`/qr_item_search/state`、`/qr_item_search/result`、`/qr_item_search/scanner_control`、`/qr_item_search/scanner_event`（String JSON）
- 调试旁路（新增，不影响协议）：`/qr_item_search/debug_snapshot`（String JSON，仅调试流节点订阅）
- `protocol_version` 恒为 `1`；`task_id`、`search_id` 原样贯穿；完整成功仍要求三个不同 URL 均解析出非空 `item_name`。
- 旧 `capture_id/capture_after/pass_index/station_index` 字段：scanner 仍会解析（向后兼容），但不再用于拒绝同一搜索任务中的解码结果；`pass_index/station_index` 用于关键帧标注。
- 只有收到 `status: "complete"` 后，编排器才能构造 LLM 分类请求（`selected_order` 必须是三个候选之一）。

## 19. 比赛现场检查清单

- [ ] `ssh ucar@192.168.1.109` 可连接。
- [ ] 二维码三个 URL 均可由小车访问并返回合法 JSON。
- [ ] `/odom` 和 `/usb_cam/image_raw` 持续有数据。
- [ ] 相机无明显过曝、失焦或强反光。
- [ ] QR 节点已启动，状态为 `IDLE`。
- [ ] 小车旋转范围已清空，急停可触达。
- [ ] 本次使用新的 `task_id/search_id`。
- [ ] 同时观察 state、result，必要时开调试流。
- [ ] 结束后先停止搜索，再关闭 QR、相机和底盘节点。
- [ ] 指标与关键帧目录有本轮记录。

更详细的协议与设计：

- [定点停扫重构设计（旧，部分废止）](../../../docs/superpowers/specs/2026-08-08-qr-stop-and-scan-design.md)
- [定点停扫实施计划（旧，部分废止）](../../../docs/superpowers/plans/2026-08-08-qr-stop-and-scan.md)
- [子任务 1 最小集成协议](../../../docs/superpowers/specs/2026-07-19-task1-minimal-integration-protocol.md)
