# QR 物品连续搜索：运行、调试与联调手册

`qr_item_search` 是 U-CAR-02 上的 ROS 1 Noetic 包。小车到达物品领取区观察点后，包控制底盘原地连续旋转，通过前置相机识别场地四边中随机放置的三个二维码，访问二维码 URL，从返回 JSON 中取得三个物品名称，并发布统一结果。

> 当前包只实现“旋转搜索 + 二维码解码 + HTTP 解析 + 结果发布”。语音理解、任务编排器、LLM 分类、语音播报、抓取和下一目标点导航尚未包含在本包中。

## 1. 当前版本的完成条件

比赛要求依次读取三个二维码的全部内容，因此当前搜索只有取得三个不同 URL、且三个 URL 都成功返回有效物品名称时，才发布 `status: "complete"`。识别一个或两个二维码不会提前结束。

二维码 URL 的 HTTP 响应必须是 JSON：

```json
{
  "code": 200,
  "result": "香蕉"
}
```

- `code` 必须是整数 `200`。
- `result` 必须是非空字符串。
- 手机扫码打不开不必然说明二维码无法解码；但小车必须能够联网访问该 URL，才能得到物品名称。

搜索流程：

1. `FAST_SWEEP`：默认以 `0.40 rad/s` 连续旋转约 380°，边转边识别。
2. `WAITING_HTTP`：三个 URL 都发现后停车，等待并发 HTTP 请求完成。
3. `TARGETED_RESCAN`：对未覆盖、过曝、模糊或解析失败的方向低速补扫。
4. 最终进入 `COMPLETE`、`NOT_FOUND`、`ERROR` 或 `STOPPED`，并保持零角速度。

## 2. 安全前提

启动完整搜索会向 `/cmd_vel` 发布角速度，小车会原地旋转。运行前必须：

- 将小车放在物品区观察点附近，清空旋转范围内人员、线缆和障碍物。
- 保证急停或电源开关可立即触达。
- 确认没有其他节点同时控制 `/cmd_vel`。
- 先检查 `/odom` 和 `/usb_cam/image_raw` 持续有数据。
- 第一次调试先执行“Scanner-only 无运动测试”，再执行带底盘运动的完整搜索。

## 3. 连接、编译和环境加载

从电脑连接小车：

```bash
ssh ucar@172.20.10.4
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

## 4. 一键运行 QR 包

这里的“一键”是指底盘驱动、相机和 ROS Master 已经启动后，用一个 launch 同时启动本包的 scanner 与 controller：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
  roslaunch qr_item_search qr_item_search.launch image_topic:=/usb_cam/image_raw
```

该 launch 启动：

- `/qr_scanner`
- `/item_search_controller`

它不会自动启动底盘驱动和 USB 相机。执行 launch 后节点处于 `IDLE`，还不会旋转；收到 `/qr_item_search/start` 后才开始搜索。

## 5. 从零开始分步运行

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
  _pixel_format:=yuyv
```

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

## 6. 启动一次搜索

### 6.1 运行前检查

```bash
rostopic hz /odom
rostopic hz /usb_cam/image_raw
rosnode list
rostopic info /cmd_vel
```

`/odom` 和图像话题应持续输出。`rosnode list` 应包含 QR 两个节点。检查 `/cmd_vel` 时要确认不存在非预期控制节点。

### 6.2 手动发送开始信号

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

### 6.3 观察状态和结果

在其他终端运行：

```bash
rostopic echo /qr_item_search/state
```

```bash
rostopic echo /qr_item_search/result
```

```bash
rostopic echo /cmd_vel
```

```bash
rqt_image_view /usb_cam/image_raw
```

结果状态含义：

| `status` | 含义 | 上层应做什么 |
|---|---|---|
| `searching` | 已接受任务，正在搜索 | 等待最终结果 |
| `complete` | 三个物品都已解析 | 才能请求 LLM 分类 |
| `not_found` | 超时或补扫后仍不足三个 | 不调用 LLM，报告失败或生成新搜索 |
| `error` | 相机、航向、协议或内部安全错误 | 停止流程，排查后用新 `search_id` 重试 |
| `stopped` | 收到人工或上层停止指令 | 等待新任务 |

## 7. 怎么停止，以及是否需要手动关闭节点

### 7.1 只停止当前搜索

发送与当前任务匹配的 `task_id` 和 `search_id`：

```bash
rostopic pub -1 /qr_item_search/stop std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"search_id\":\"search-test-001\",\"reason\":\"operator_stop\"}'"
```

这会停车并结束当前搜索，但不会关闭 ROS 节点。节点会继续运行并等待下一次 start。

### 7.2 搜索正常结束后要不要关闭节点

- 连续进行多次任务：不需要关闭；收到 `complete/not_found/error/stopped` 后 controller 已发布零速度，可保留节点等待下一次任务。
- 本轮调试结束、准备关机或移动小车：需要关闭本包、相机和底盘节点。

### 7.3 正常关闭

1. 若搜索仍在进行，先发送 stop。
2. 在运行 `roslaunch qr_item_search ...` 的终端按 `Ctrl+C`。
3. 在相机和底盘终端分别按 `Ctrl+C`。
4. 确认 QR 节点已消失：

```bash
rosnode list | grep -E 'qr_scanner|item_search_controller'
```

无输出表示 QR 节点已关闭。

### 7.4 找不到原终端时关闭

先停止当前搜索，再执行：

```bash
rosnode kill /qr_scanner /item_search_controller
```

然后按实际节点名关闭相机和底盘。关机前再次检查：

```bash
rosnode list
```

`rosnode kill` 是异常收尾手段，正常情况优先在 launch 终端按 `Ctrl+C`，这样 shutdown 回调能够发布零速度并清理线程。

### 7.5 紧急情况

人员或物体进入旋转范围时，优先使用实体急停/电源开关，不要只依赖网络命令。安全后再关闭 ROS 节点并排查原因。

## 8. Scanner-only 无运动测试

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

停止接帧：

```bash
rostopic pub -1 /qr_item_search/scanner_control std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"manual\",\"search_id\":\"scanner-only-001\",\"enabled\":false,\"enhanced\":false,\"detected_yaw\":0.0,\"retry_failed\":false}'"
```

## 9. 上一步导航如何触发二维码搜索

### 9.1 当前已经实现的接口

QR 包当前只订阅 `/qr_item_search/start`，不直接订阅 `/task1/pickup_arrived`。因此现在人工联调时，到达观察点后直接发送第 6.2 节的 start 消息。

### 9.2 完整系统推荐流程

后续新增独立节点 `task1_orchestrator`，由它拥有整条任务状态：

```text
语音目标
  -> 导航到物品区观察点
  -> pickup_arrived
  -> orchestrator 发布 QR start
  -> QR complete（三个物品）
  -> orchestrator 请求 LLM 分类
  -> LLM 选中一个物品及目标车间
  -> orchestrator 发布语音播报
  -> orchestrator 发布下一导航目标
```

导航模块到达观察点后应发布：

```bash
rostopic pub -1 /task1/pickup_arrived std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"status\":\"arrived\"}'"
```

注意：当前仓库还没有 `task1_orchestrator`，所以单独发送该消息不会启动 QR 搜索。现阶段需要随后人工发送 `/qr_item_search/start`；编排器实现后，这个转换将自动完成。

## 10. 与语音系统联调

语音理解模块不应直接控制底盘或 QR 包。它只需发布解析后的目标大类：

```bash
rostopic pub -1 /voice/target_category std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"target_category\":\"食品加工类\",\"raw_text\":\"前往物品领取区，取得食品加工类物品\"}'"
```

编排器缓存 `target_category`，并保证同一个 `task_id` 贯穿导航、QR、LLM 和播报。二维码包不需要知道目标类别，因为比赛要求先读取全部三个二维码，再由 LLM 从三个候选中选择。

## 11. 与 LLM 联调

只有收到 QR 的 `status: "complete"` 后，编排器才能构造一次 LLM 请求：

```bash
rostopic pub -1 /llm/classify/request std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"target_category\":\"食品加工类\",\"candidates\":[{\"order\":1,\"item_name\":\"香蕉\"},{\"order\":2,\"item_name\":\"毛巾\"},{\"order\":3,\"item_name\":\"手机\"}]}'"
```

LLM 适配器返回：

```bash
rostopic pub -1 /llm/classify/result std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"status\":\"success\",\"selected_order\":1,\"selected_item\":\"香蕉\",\"target_category\":\"食品加工类\",\"target_workshop\":\"食品加工车间\",\"message\":\"\"}'"
```

LLM 适配器必须验证：

- `selected_order` 是三个候选之一。
- `selected_item` 与该序号对应的名称完全一致。
- 分类失败时返回 `status: "error"` 和非空 `message`，编排器不得继续导航。
- 同一 `task_id` 的重复请求不得触发第二次推理。

当前 QR 包不会自动发布 `/llm/classify/request`；这是待开发编排器的职责。

## 12. 与语音播报和下一导航联调

LLM 成功后，编排器发布播报文本：

```bash
rostopic pub -1 /voice/speak std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"text\":\"香蕉属于食品大类，应放置在食品加工车间\"}'"
```

TTS 节点只负责朗读 `text`，不应自行改变物品或车间。

随后编排器发布下一导航目标：

```bash
rostopic pub -1 /task1/navigation_goal std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"target_workshop\":\"食品加工车间\",\"selected_item\":\"香蕉\"}'"
```

当前仓库没有这两个话题的正式消费者。早期联调可分别使用：

```bash
rostopic echo /voice/speak
rostopic echo /task1/navigation_goal
```

确认消息内容一致，但不要让模拟导航消息实际驱动车辆。

## 13. 后续需要开发的 `task1_orchestrator`

编排器至少需要实现以下状态和规则：

1. 缓存 `/voice/target_category`，按 `task_id` 去重。
2. 收到同一 `task_id` 的 `pickup_arrived: arrived` 后生成唯一 `search_id`。
3. 发布一次 `/qr_item_search/start`，等待最多 45 秒。
4. 仅在 QR `complete` 且恰有三个候选时发布一次 LLM 请求。
5. 等待 LLM 最多 15 秒；校验选择结果属于候选集合。
6. 发布一次 `/voice/speak` 和一次 `/task1/navigation_goal`。
7. 任一阶段失败时停止流程；活动搜索中应发布 `/qr_item_search/stop`。
8. 重复消息不得造成重复旋转、重复 LLM 请求或重复导航。

建议先使用规则映射模拟 LLM 跑通编排器，再接 Spark X2 API。这样 QR、状态机和导航协议的错误不会与云端调用问题混在一起。

## 14. 最小端到端模拟联调顺序

在真实语音、LLM 和导航尚未完成时：

1. 启动底盘、相机和 QR 包。
2. 命令行发布 `/voice/target_category`。
3. 命令行发布 `/task1/pickup_arrived`，记录这是未来编排器的输入。
4. 现阶段人工发布 `/qr_item_search/start`。
5. 等待 `/qr_item_search/result` 的 `complete`。
6. 从结果复制三个 `item_name`，人工构造 `/llm/classify/request`。
7. 用命令行模拟 `/llm/classify/result`。
8. 模拟发布 `/voice/speak` 和 `/task1/navigation_goal`，用 `rostopic echo` 核对。
9. 确认所有消息使用同一个 `task_id`，QR 使用唯一 `search_id`。

通过标准：三个二维码不足时不调用 LLM；三个结果到齐后只调用一次；播报和导航中的物品、类别、车间完全一致。

## 15. 默认参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `fast_angular_speed` | `0.40 rad/s` | 快速环扫角速度 |
| `targeted_angular_speed` | `0.20 rad/s` | 定向补扫角速度 |
| `minimum_effective_speed` | `0.11 rad/s` | 底盘最小有效角速度 |
| `fast_sweep_angle` | `6.632251 rad` | 快速环扫角度，约 380° |
| `yaw_tolerance` | `0.035 rad` | 定向到位容差 |
| `heading_timeout` | `1.0 s` | 航向数据超时 |
| `camera_timeout` | `1.0 s` | 运动搜索时图像超时 |
| `search_total_timeout` | `40.0 s` | 单次搜索总超时 |
| `connect_timeout` | `1.0 s` | HTTP 连接超时 |
| `read_timeout` | `2.0 s` | HTTP 读取超时 |
| `http_retries` | `1` | 首次请求失败后的重试次数 |
| `http_worker_count` | `3` | 并发 HTTP worker 数量 |
| `image_topic` | `/usb_cam/image_raw` | 相机图像话题 |

覆盖参数示例：

```bash
roslaunch qr_item_search qr_item_search.launch \
  image_topic:=/usb_cam/image_raw
```

注意：当前 launch 只声明了 `image_topic` 为 launch argument；其余参数写在 launch 文件内。若要调整其他参数，请修改 launch 文件并重新启动节点。不要在比赛现场未经空旷地测试直接提高角速度。

## 16. 调参位置与实车调参方法

### 16.1 先分清三类参数

| 参数类别 | 修改位置 | 是否需要重启节点 | 典型参数 |
|---|---|---|---|
| QR 搜索控制 | `~/ucar_ws/src/qr_item_search/launch/qr_item_search.launch` | 需要重启 QR launch | 转速、扫描角度、总超时 |
| QR 网络解析 | 同一个 QR launch 的 `qr_scanner` 节点参数 | 需要重启 QR launch | HTTP 超时、重试、worker 数 |
| USB 相机硬件 | `/dev/video0` 的 V4L2 controls | 通常立即生效；相机或小车重启后可能恢复 | 对焦、曝光、锐度、白平衡 |
| 相机分辨率/格式 | `/opt/ros/noetic/share/usb_cam/launch/usb_cam-test.launch` 或相机启动命令 | 需要重启相机 | 宽高、YUYV/MJPG、帧率 |

不要直接修改 `/opt/ros/noetic/share` 下的系统文件作为长期方案，系统包更新时会被覆盖。调试阶段优先使用命令行；参数确定后，应在团队自己的相机 launch 或启动脚本中固化。

### 16.2 QR 搜索参数在哪里改

本包当前参数文件：

```bash
nano ~/ucar_ws/src/qr_item_search/launch/qr_item_search.launch
```

当前相关片段：

```xml
<param name="fast_angular_speed" value="0.40"/>
<param name="targeted_angular_speed" value="0.20"/>
<param name="minimum_effective_speed" value="0.11"/>
<param name="fast_sweep_angle" value="6.632251"/>
<param name="yaw_tolerance" value="0.035"/>
<param name="heading_timeout" value="1.0"/>
<param name="camera_timeout" value="1.0"/>
<param name="search_total_timeout" value="40.0"/>
```

保存后不需要重新 `catkin_make`，但必须在原 QR launch 终端按 `Ctrl+C`，再重新执行 `roslaunch`。节点只在启动时读取这些参数，运行中执行 `rosparam set` 不会改变已经构造好的控制器。

分步运行 controller 时，可以临时覆盖参数而不改文件：

```bash
rosrun qr_item_search item_search_controller_node.py \
  _fast_angular_speed:=0.40 \
  _targeted_angular_speed:=0.20 \
  _search_total_timeout:=60.0
```

### 16.3 本轮建议的 QR 参数

从 `search-test-004` 到 `search-test-007` 的结果看，四轮分别解析到 1、2、1、1 个物品。`search-test-005` 在累计 `8.89 rad`（约 509°）识别到第二个二维码，并在约 40 秒结束，证明补扫仍在工作，但调试总超时偏紧。

第一轮建议只改一个值：

```xml
<param name="search_total_timeout" value="60.0"/>
```

其余暂时保持：

```text
fast_angular_speed     = 0.40 rad/s
targeted_angular_speed = 0.20 rad/s
fast_sweep_angle       = 6.632251 rad（约 380°）
```

原因：相机实测稳定约 30 fps。`0.40 rad/s` 时每帧之间只转约 `0.0133 rad`，即 `0.76°`，不是明显过快。现在降速会增加比赛耗时，却不能验证固定焦距、曝光或二维码尺寸问题。

`60 s` 只用于跑通流程和收集数据。三二维码稳定后，应根据实测逐步降回 `45–50 s`，最终再评估是否恢复 `40 s`。

### 16.4 当前相机实际状态

小车当前 `/dev/video0` 实测为：

```text
640 × 480
YUYV
约 30 fps
exposure_auto = 3              自动曝光（光圈优先）
exposure_auto_priority = 1     自动曝光可优先延长曝光
focus_auto = 0                 自动对焦关闭
focus_absolute = 68            固定焦距
sharpness = 50
```

其中最值得先检查的是固定焦距 `68`。二维码位于不同墙边、距离和斜视角不同时，固定焦距可能让一两个码清晰，而另一个码始终不够清晰。

查看当前值：

```bash
v4l2-ctl -d /dev/video0 --get-ctrl=focus_auto
v4l2-ctl -d /dev/video0 --get-ctrl=focus_absolute
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_auto
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_auto_priority
```

查看设备支持的全部控制范围：

```bash
v4l2-ctl -d /dev/video0 --list-ctrls-menus
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### 16.5 正确标定并锁定焦距

不要在小车旋转时让自动对焦不断搜索。推荐在实际物品区、实际观察距离上先自动找焦，再锁定结果。

1. 停止当前 QR 搜索，确保底盘不动。
2. 让相机正对一个比赛尺寸的二维码，距离采用比赛观察点到墙面的典型距离。
3. 打开图像：

```bash
rqt_image_view /usb_cam/image_raw
```

4. 临时开启自动对焦：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=1
```

5. 等待 2–3 秒，画面稳定后读取焦距：

```bash
v4l2-ctl -d /dev/video0 --get-ctrl=focus_absolute
```

6. 记录得到的数值，例如 `N`；关闭自动对焦并锁定：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0
v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=N
```

7. 不移动小车，分别让三个二维码处于画面中央、左右边缘和一定斜角，做 Scanner-only 测试。三个码都能在静止状态快速解码后，才开始旋转测试。

若启用自动对焦后 `focus_absolute` 不更新，说明该摄像头驱动没有可靠报告自动焦点。此时以 `68` 为中心，手动小步调整，例如每次只改变 10–20，并用同一距离、同一二维码比较清晰度和解码时间。不要同时改曝光。

### 16.6 曝光调整顺序

当前帧率稳定，因此第一步不建议直接切换全手动曝光。先禁止自动曝光为了亮度而延长曝光：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto_priority=0
```

保留：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3
```

然后在相同场地重复测试。若仍有明显运动拖影，再测试手动曝光：

1. 静止面对典型二维码，让自动曝光稳定。
2. 读取当前曝光：

```bash
v4l2-ctl -d /dev/video0 --get-ctrl=exposure_absolute
```

3. 记录值 `E`，切换手动并锁定：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=E
```

4. 一次只小幅降低曝光，检查二维码黑白块是否仍有足够对比度。曝光太长会拖影，太短会让暗处二维码丢失。

回到自动曝光：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3
```

V4L2 修改可能在相机节点或小车重启后恢复默认值。每次测试前都用 `--get-ctrl` 重新确认，最终把确定值放进团队自己的启动流程。

### 16.7 分辨率与格式

当前设备支持：

- `YUYV 640×480 @ 30 fps`：当前配置，解码开销较低。
- `YUYV 800×600 @ 15 fps`：像素增加但帧率减半，不优先。
- `MJPG 800×600/1280×720/1920×1080 @ 30 fps`：细节增加，但需要解压且可能增加 CPU 负载和压缩伪影。

第一轮不要同时换分辨率。只有在焦距和曝光调整后，静止画面中的二维码仍然像素过少，才单独测试 `MJPG 1280×720 @ 30 fps`，并同时观察：

```bash
rostopic hz /usb_cam/image_raw
top
```

如果 scanner 处理跟不上，高分辨率反而会减少有效解码次数。

### 16.8 每轮必须保存的诊断数据

当前实现不会保存成功解码的关键帧。相机回调只在内存中保留最新帧，解码后就会被覆盖；最终 result 只包含成功解析的 URL、物品名和航向。因此仅看 `/qr_item_search/result`，无法判断漏码是模糊、过曝、没有进入视野、解码失败还是 HTTP 失败。

下一轮测试至少开三个记录终端：

```bash
rostopic echo /qr_item_search/result \
  > ~/qr_result_$(date +%Y%m%d_%H%M%S).log
```

```bash
rostopic echo /qr_item_search/scanner_event \
  > ~/qr_scanner_event_$(date +%Y%m%d_%H%M%S).log
```

```bash
rosbag record -O ~/qr_debug_$(date +%Y%m%d_%H%M%S).bag \
  /usb_cam/image_raw /odom /cmd_vel \
  /qr_item_search/state /qr_item_search/scanner_event /qr_item_search/result
```

`rosbag` 图像数据较大，测试结束后立即按 `Ctrl+C`，并检查磁盘：

```bash
df -h ~
ls -lh ~/qr_debug_*.bag
```

有了 bag，才能离线定位每个 `detected_yaw` 对应的相机帧，并比较漏掉二维码的方向。

### 16.9 单变量调参顺序

每次使用相同的三个二维码、相同位置和新的 `search_id`，按顺序测试：

1. 现有 `search-test-004` 至 `007` 作为 40 秒基线；下一轮开始保存 result、scanner_event 和 bag。
2. 只把搜索总超时改为 `60 s`，建立不容易被提前截断的调试基线。
3. 保持 60 秒，只标定并锁定焦距；先做静止三二维码测试，再做旋转测试。
4. 保持其他参数不变，只把 `exposure_auto_priority` 改为 `0`。
5. 三码稳定后，再分别评估总超时和角速度。
6. 最后才测试更高分辨率或候选二维码停车方案。

每个配置至少连续测试 5 次，记录成功数量、总耗时、每个二维码航向和失败原因。不要用某一次偶然成功作为最终参数。

### 16.10 当前连续扫描设计的限制与下一版方向

当前图像链路是“latest frame”模型：

```text
相机 30 fps
  -> 回调只保留内存中的最新帧
  -> 单独解码线程取走最新帧
  -> pyzbar 尝试完整解码 URL
  -> 成功后只发布 URL、航向和质量
  -> 图像被后续帧覆盖
```

它不会自动截图，也没有保存“第一次成功解码帧”。单元测试和静态图片测试证明了解码器能工作，实车测试证明至少有部分二维码被成功解码，但没有保留对应实车关键帧。调参阶段应先使用 rosbag 补齐这项证据；后续可增加可配置的关键帧保存功能，默认关闭，避免比赛时持续写盘。

单纯在 pyzbar 已经完整解码 URL 后停车，对“漏掉的二维码”帮助有限，因为最困难的一步已经完成。更有效的下一版是两阶段视觉：

1. 连续旋转时使用较轻量的候选检测，只要求发现二维码外框或三个定位点，不要求已经解出 URL。
2. 候选连续出现 2 帧后，controller 立即减速并停车。
3. 等画面稳定约 `0.2–0.5 s`，保存候选关键帧。
4. 对静止图像执行原图、灰度、CLAHE、自适应阈值和必要的透视矫正解码。
5. 成功则记录 URL 并恢复旋转；失败则在有限等待后恢复旋转，避免被假候选永久卡住。

推荐状态机扩展：

```text
FAST_SWEEP
  -> CANDIDATE_BRAKE
  -> STATIC_DECODE
      -> FAST_SWEEP          成功或候选超时，继续找剩余二维码
      -> WAITING_HTTP        三个 URL 都已取得
```

候选检测可以使用 OpenCV `QRCodeDetector.detect()` 或轮廓/定位点检测；完整内容仍可由当前 pyzbar/ZBar 解码。这样 OpenCV 只负责“看见疑似二维码”，pyzbar 负责“读取内容”，不会要求更换当前已经验证过的解码后端。

在实现该状态前，应先完成本节的焦距、曝光和数据记录实验。如果三个二维码在静止状态都不能稳定解码，停车状态机不会解决根本问题；必须先修正相机、码面尺寸、距离或光照。

## 17. 常见问题排查

### 没有旋转

- 检查是否已经发送 start，而不只是启动 launch。
- 检查 `/odom` 是否有数据且频率稳定。
- 检查 `/qr_item_search/state` 和 `/qr_item_search/result` 的错误信息。
- 检查底盘驱动是否订阅 `/cmd_vel`。

### 能看到二维码但没有结果

- 确认二维码内容是完整的 HTTP/HTTPS URL。
- 在小车上用 `curl '<二维码URL>'` 验证网络和 JSON。
- 确认返回 `code: 200` 且 `result` 非空。
- 手机屏幕展示二维码时提高亮度但避免反光，保持屏幕与镜头尽量正对。

### 画面白花或过曝

- 避免太阳直射镜头和二维码表面。
- 使用哑光打印纸，保留二维码白边。
- 调整观察点或增加遮光罩，让曝光稳定后再测试。

### 收到 `scanner restarted`

scanner 在活动搜索中重启后，controller 会安全停车并进入 `ERROR`。确认 scanner 已恢复，然后使用新的 `search_id` 重新开始，不要重发旧 start。

### 已结束但节点仍在

这是正常行为。`complete/not_found/error/stopped` 只结束一次搜索，不退出常驻节点。继续测试可直接发送新的 start；准备关机则按第 7 节关闭节点。

## 18. Windows 本地测试

在仓库或 worktree 根目录运行：

```powershell
$package = Join-Path $PWD 'ucar_ws\src\qr_item_search'
subst Q: $package
Set-Location Q:\
python -c "import sys,unittest; sys.path.insert(0,r'Q:\src'); suite=unittest.defaultTestLoader.discover(r'Q:\test',pattern='test_*.py'); result=unittest.TextTestRunner(verbosity=1).run(suite); raise SystemExit(not result.wasSuccessful())"
Set-Location $package
subst Q: /d
```

图像测试还需要与当前 Python 匹配的 NumPy 和 OpenCV。

小车端测试：

```bash
cd ~/ucar_ws/src/qr_item_search
PYTHONPATH=~/ucar_ws/src/qr_item_search/src \
python3 -m unittest discover -s test -p 'test_*.py' -q
```

## 19. 比赛现场检查清单

- [ ] `ssh ucar@172.20.10.4` 可连接。
- [ ] 二维码三个 URL 均可由小车访问并返回合法 JSON。
- [ ] `/odom` 和 `/usb_cam/image_raw` 持续有数据。
- [ ] 相机无明显过曝、失焦或强反光。
- [ ] QR 两个节点已启动，状态为 `IDLE`。
- [ ] 小车旋转范围已清空，急停可触达。
- [ ] 本次使用新的 `task_id/search_id`。
- [ ] 同时观察 state、result 和必要的图像画面。
- [ ] 结束后先停止搜索，再关闭 QR、相机和底盘节点。

更详细的协议与设计：

- [子任务 1 最小集成协议](../../../docs/superpowers/specs/2026-07-19-task1-minimal-integration-protocol.md)
- [连续环扫设计](../../../docs/superpowers/specs/2026-07-19-continuous-qr-sweep-design.md)
