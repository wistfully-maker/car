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

## 16. 常见问题排查

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

图像测试还需要与当前 Python 匹配的 NumPy 和 OpenCV。

小车端测试：

```bash
cd ~/ucar_ws/src/qr_item_search
PYTHONPATH=~/ucar_ws/src/qr_item_search/src \
python3 -m unittest discover -s test -p 'test_*.py' -q
```

## 18. 比赛现场检查清单

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
