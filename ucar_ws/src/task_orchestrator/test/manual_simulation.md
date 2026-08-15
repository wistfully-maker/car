# 不动车分层模拟与实车看护清单

本文用于验证 readiness、导航协议、QR、LLM、TTS 和 motion 仲裁的层间契约，不用于证明
真实导航通过。默认不启动硬件和真实 TTS；每条 `rostopic pub` 都用 `-1` 只发一次。

## 1. 安全边界与启动

启动任何调试节点前，先让底盘机械断能或可靠架空驱动轮。优先使用独立 `roscore` 与隔离的
`ROS_MASTER_URI`；若必须共用现场 ROS Master，先确认连接目标并检查冲突：

```bash
echo "$ROS_MASTER_URI"
rosnode list | grep -E 'task_orchestrator|voice_task_adapter|tts_bridge|fast_nav_adapter|readiness_gate|velocity_arbiter'
rostopic info /cmd_vel
```

还要确认没有正式 competition root、没有外部仲裁器正在运行。发现同名节点、未知
`/cmd_vel` publisher 或现场正式流程时立即停止本次模拟，回到其 owner/根终端处理；不要 kill
他人的节点。启动后只模拟零运动状态；即使测试 motion mode 为 `NAVIGATION`，仍不得发送
非零 Twist。

不要在正式 `start_competition.sh` 或外部仲裁器 active 时随便注入速度。尤其当
`/task/motion_mode` 是 `NAVIGATION` 或 `QR_SEARCH` 时，向活动输入 topic 发非零 Twist 会真实
传到车轮。**未架空车轮、未确认唯一 owner、无人持急停时不得发送非零速度。**
**真实导航必须有人看护**；本文默认只看业务输出并用“到达”JSON替代运动。

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_tts_bridge:=false \
  enable_fast_nav_adapter:=false \
  enable_readiness_gate:=false \
  enable_velocity_arbiter:=false
```

另开终端：

```bash
rostopic echo /task/status
rostopic echo /task/motion_mode
```

同时按需观察 `/voice/task_request`、`/task/pickup_navigation_goal`、
`/qr_item_search/start`、`/llm/classify/request`、`/voice/speak` 和
`/task/delivery_navigation_goal`。以下 `<...>` 必须替换为上一阶段实际输出，identity 不可猜。

## 2. readiness 层：不启动真实健康门控

```bash
rostopic pub -1 /question std_msgs/String \
  "data: '小飞小飞，前往物品领取区，取得食品放在对应仓库，并领取仿真环境需要的日用品放在对应仓库'"
```

记录 `/voice/task_request` 的 `<TASK_ID>`；字段应包括：

```json
{"protocol_version":1,"task_id":"<TASK_ID>","physical_target_category":"食品","simulation_target_category":"日用品","raw_text":"..."}
```

此时状态 `CHECKING_DEPENDENCIES`、motion `IDLE`。模拟 readiness：

```bash
rostopic pub -1 /task/dependencies_ready std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"status\":\"ready\"}'"
```

记录取货目标 `<PICKUP_GOAL_ID>`。状态变为 `NAVIGATING_TO_PICKUP`，motion 是
`NAVIGATION`；这只是授权模式，当前没有 adapter/arbiter，所以车不应移动。

## 3. 导航层：只模拟 action 结果

不要启动 `/move_base`，直接模拟 adapter 成功回执：

```bash
rostopic pub -1 /task/pickup_arrived std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"goal_id\":\"<PICKUP_GOAL_ID>\",\"status\":\"arrived\",\"message\":\"\"}'"
```

记录 `/qr_item_search/start` 的 `<SEARCH_ID>`。状态 `WAITING_QR`，motion 为 `QR_SEARCH`。
错误 `goal_id` 应被忽略且 deadline 不刷新。

## 4. QR 层：三物品与 identity

```bash
rostopic pub -1 /qr_item_search/result std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"search_id\":\"<SEARCH_ID>\",\"stamp\":1000.0,\"status\":\"complete\",\"items\":[{\"order\":1,\"item_name\":\"手机\",\"url\":\"https://example/1\",\"detected_yaw\":0.0},{\"order\":2,\"item_name\":\"毛巾\",\"url\":\"https://example/2\",\"detected_yaw\":1.2},{\"order\":3,\"item_name\":\"苹果\",\"url\":\"https://example/3\",\"detected_yaw\":3.4}],\"message\":\"\"}'"
```

记录 LLM `<REQUEST_ID>`；request 必须含 `candidates`、两个母类和相同 `task_id`。状态
`WAITING_LLM` 后 motion 已回 `IDLE`，因此二维码后的任何速度输入都不得获准。

## 5. LLM 与 TTS 层

```bash
rostopic pub -1 /llm/classify/result std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"request_id\":\"<REQUEST_ID>\",\"status\":\"success\",\"physical\":{\"selected_order\":3,\"selected_item\":\"苹果\",\"category\":\"食品\",\"workshop\":\"食品加工车间\"},\"simulation\":{\"selected_order\":2,\"selected_item\":\"毛巾\",\"category\":\"日用品\",\"workshop\":\"日用品加工车间\"},\"message\":\"\"}'"
```

记录 `/voice/speak` 的 `<SPEECH_ID>`。因为已关闭 TTS bridge，人工回执：

```bash
rostopic pub -1 /voice/speak_done std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"speech_id\":\"<SPEECH_ID>\",\"status\":\"success\",\"message\":\"\"}'"
```

应观察到 delivery 消息含：

```json
{"protocol_version":1,"task_id":"<TASK_ID>","goal_id":"<DELIVERY_GOAL_ID>","target_workshop":"食品加工车间","selected_item":"苹果"}
```

但 motion 必须仍是 `IDLE`。不要发送 `/task/delivery_arrived` 来宣称真实配送完成；当前没有
二维码后车间导航适配器，delivery goal 不授运动权。

## 6. motion/速度仲裁的零速分层验证

仅在车轮架空或底盘完全断电时单独启动仲裁器：

```bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_task_orchestrator:=false enable_voice_adapter:=false \
  enable_tts_bridge:=false enable_velocity_arbiter:=true
rostopic echo /cmd_vel
```

安全的零速度消息：

```bash
rostopic pub -1 /task/motion_mode std_msgs/String "data: 'NAVIGATION'"
rostopic pub -1 /cmd_vel/navigation geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
rostopic pub -1 /task/motion_mode std_msgs/String "data: 'QR_SEARCH'"
rostopic pub -1 /cmd_vel/qr geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
rostopic pub -1 /task/motion_mode std_msgs/String "data: 'IDLE'"
```

逐层判断：`NAVIGATION` 只接受 `/cmd_vel/navigation`，`QR_SEARCH` 只接受 `/cmd_vel/qr`，
`IDLE` 两路均不转发；模式切换、输入超过 0.3 秒、非法模式和 shutdown 都应输出零速度。
在 arbiter active 时不得发送非零速度做“试一下”；非零验证属于受控实车任务 8。

## 7. 失败、过期与取消

- readiness 只接受 `status: ready`；`status: error` 不是合法失败回执，而是 malformed，
  `task_orchestrator_node` 对应回调记录并忽略协议错误，状态保持 `CHECKING_DEPENDENCIES`、
  deadline 不刷新，最终在 120 秒超时
  后进入 `ERROR`。该层没有立即失败 JSON；测试失败路径应不发 ready、等待 timeout，并观察
  readiness 日志中的 missing requirements。可用下面消息验证 malformed 被忽略：

```bash
rostopic pub -1 /task/dependencies_ready std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"status\":\"error\"}'"
```

- 导航：匹配 identity 的 `status:failed` 或 300 秒超时，应进入 `ERROR`、motion `IDLE`；
- QR 合法失败是 `not_found` 或 `error` 且 `message` 非空，会立即进入 `ERROR`：

```bash
rostopic pub -1 /qr_item_search/result std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"search_id\":\"<SEARCH_ID>\",\"stamp\":1001.0,\"status\":\"not_found\",\"items\":[],\"message\":\"未找到三个二维码\"}'"
```

- malformed QR（例如 `complete` 只有两个物品、order 不从 1 连续、名称/URL 不唯一）触发
  `ProtocolError`，消息被忽略、仍处于 `WAITING_QR`、deadline 不刷新，最终 120 秒 timeout。
  具体的“两物品 complete”模拟：

```bash
rostopic pub -1 /qr_item_search/result std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"search_id\":\"<SEARCH_ID>\",\"stamp\":1002.0,\"status\":\"complete\",\"items\":[{\"order\":1,\"item_name\":\"手机\",\"url\":\"https://example/1\",\"detected_yaw\":0.0},{\"order\":2,\"item_name\":\"毛巾\",\"url\":\"https://example/2\",\"detected_yaw\":1.0}],\"message\":\"\"}'"
```

- LLM 合法失败是 `status: error` 且 `message` 非空，会立即进入 `ERROR`：

```bash
rostopic pub -1 /llm/classify/result std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"request_id\":\"<REQUEST_ID>\",\"status\":\"error\",\"message\":\"Spark 服务不可用\"}'"
```

- malformed LLM（如 `selected_item` 与候选 order 不匹配/不在 candidates）以及 stale
  `task_id/request_id` 都触发 `ProtocolError` 并被忽略，deadline 不刷新，最终在 120 秒 timeout。
  具体模拟：

```bash
rostopic pub -1 /llm/classify/result std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"request_id\":\"<REQUEST_ID>\",\"status\":\"success\",\"physical\":{\"selected_order\":3,\"selected_item\":\"不存在的物品\",\"category\":\"食品\",\"workshop\":\"食品加工车间\"},\"simulation\":{\"selected_order\":2,\"selected_item\":\"毛巾\",\"category\":\"日用品\",\"workshop\":\"日用品加工车间\"},\"message\":\"\"}'"
```

- TTS：`status:error` 或 60 秒超时，应进入 `ERROR`，不得发布 delivery goal；
- 任一阶段把 ID 换成 `stale-*`，消息应被忽略而不是推进状态，且不刷新 deadline；
- 取消示例：

```bash
rostopic pub -1 /task/cancel std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"reason\":\"operator_cancel\"}'"
```

应进入 `CANCELLED`、motion `IDLE`；在 QR 阶段还应发布匹配 `search_id` 的 stop。

## 8. 结束

在启动本次模拟的根 launch 终端按 `Ctrl+C`。用 `rosnode list`、`rosnode ping` 和
`rostopic info` 确认残留；不要自动 kill 现场节点。真实 TTS 测试时启用 bridge 后不得再人工
发布同一 `speech_id` 的 done。真实导航、相机旋转和底盘非零速度测试必须转入有人看护的
任务 8，不属于本不动车模拟。

## 9. 协议字段速查（带空格格式）

下面仅用于核对字段名；实际 identity 仍取自本轮输出：

```json
[
  {"physical_target_category": "食品", "simulation_target_category": "日用品"},
  {"status": "ready"},
  {"status": "arrived"},
  {"order": 1, "item_name": "手机"},
  {"physical": {}, "simulation": {}},
  {"target_workshop": "食品加工车间", "selected_item": "苹果"},
  {"reason": "operator_cancel"}
]
```

## 10. 第三阶段：巡线联调分层模拟（不动车）

本节只验证第三阶段协议层与状态机，不验证真实导航/巡线/红绿灯。先机械断能或架空
驱动轮；所有 `rostopic pub` 用 `-1` 只发一次；不启动 `line_follow_integration`
的 `phase3.launch` 真实节点（相机适配/导航适配/监管器）、不启动相机、move_base、
YOLO 模型与巡线子进程时，用以下 JSON 依次注入（identity 取自上一轮输出）。

```bash
# 1) 第二阶段仿真车间播报成功后，状态机应发布一次 /task/line_navigation_goal
rostopic echo -n 1 /task/line_navigation_goal

# 2) 注入导航到达（goal_id 取自上面的输出）
rostopic pub -1 /task/line_navigation_arrived std_msgs/String '{
  "protocol_version": 1, "task_id": "task-1", "goal_id": "<goal_id>",
  "status": "arrived", "message": ""}'

# 3) 状态机应发布 /task/line_follow/start；注入红灯（保持等待，不推进）
rostopic pub -1 /task/line_follow/status std_msgs/String '{
  "protocol_version": 1, "task_id": "task-1", "goal_id": "<goal_id>",
  "status": "waiting_signal"}'

# 4) 注入方向锁定（left_turn/right_turn/straight 三选一）
rostopic pub -1 /task/line_follow/status std_msgs/String '{
  "protocol_version": 1, "task_id": "task-1", "goal_id": "<goal_id>",
  "status": "direction_selected", "direction": "left_turn"}'

# 5) 注入巡线成功（只有此时才进入 WAITING_FINAL_SPEECH 并播报“任务完成”）
rostopic pub -1 /task/line_follow/status std_msgs/String '{
  "protocol_version": 1, "task_id": "task-1", "goal_id": "<goal_id>",
  "status": "success", "direction": "left_turn"}'

# 6) 完成播报回执后状态机进入 COMPLETE
rostopic pub -1 /voice/speak_done std_msgs/String '{
  "protocol_version": 1, "task_id": "task-1",
  "speech_id": "<speech_id>", "status": "success", "message": ""}'
```

失败场景：`/task/line_navigation_arrived` 返回 `failed`、`/task/line_follow/status`
返回 `failure`（带 `reason`）、`/voice/speak_done` 返回 `error`，都应进入 `ERROR` 且
最后运动模式为 `IDLE`。`direction_selected` 之前注入 `success` 不推进；错误
`task_id`/`goal_id` 与重复消息不推进、不重复发布。

## 11. Gazebo 软门控无运动模拟

本节只验证状态机和 topic，不启动 TCP、Gazebo、导航或速度节点。启动子 launch 时增加：

```bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_tts_bridge:=false gazebo_phase_enabled:=true timeout_gazebo:=330
```

按前文推进到 `/task/simulation_arrived status=arrived` 后，应进入 `WAITING_GAZEBO`，
`/task/motion_mode` 为 `IDLE`，并看到一次 `/task/gazebo/start`。记录其中真实 task/goal，
再任选一种结果：

```bash
# success
rostopic pub -1 /task/gazebo/complete std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"goal_id\":\"<GAZEBO_GOAL_ID>\",\"status\":\"success\"}'"

# Gazebo 失败：仍应播报并接第三部分
rostopic pub -1 /task/gazebo/complete std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<TASK_ID>\",\"goal_id\":\"<GAZEBO_GOAL_ID>\",\"status\":\"failure\",\"reason\":\"manual_test\"}'"
```

两种结果都进入 `WAITING_SIMULATION_SPEECH`；错误 identity 和重复 complete 不推进。
完全不发 complete 时，330 秒后也应软超时进入同一播报。真实电脑端
`gazebo_task_bridge` 与小车使用不同 ROS Master，通过 TCP 1525 通信；其内部只发布
`/task_controller/start` 并等待 `/task_controller/done` 的本任务 `False -> True`，不接触
任何小车运动 topic。
