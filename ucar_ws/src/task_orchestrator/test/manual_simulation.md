# 手动模拟端到端测试

本流程在小车 ROS 环境执行。它使用真实语音文本适配器，但关闭真实 TTS
桥接，避免测试过程中播放音频；QR、LLM 和导航结果由 `rostopic pub`
模拟。

## 1. 构建并启动

```bash
cd ~/ucar_ws
catkin_make
source devel/setup.bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_tts_bridge:=false
```

另开终端监听关键输出：

```bash
source ~/ucar_ws/devel/setup.bash
rostopic echo /task/status
```

还可以分别监听：

```bash
rostopic echo /voice/task_request
rostopic echo /task/pickup_navigation_goal
rostopic echo /qr_item_search/start
rostopic echo /llm/classify/request
rostopic echo /voice/speak
rostopic echo /task/delivery_navigation_goal
```

以下命令中的 `<...>` 必须替换成上一阶段实际输出的 ID。所有 `rostopic pub`
都使用 `-1`，确保只发送一次。

## 2. 成功路径

### 2.1 输入语音识别文本

```bash
rostopic pub -1 /question std_msgs/String \
  "data: '小飞小飞，前往物品领取区，取得食品，放置在对应仓库，并领取仿真环境中需要的日用品放置在对应仓库'"
```

`/voice/task_request` 应包含：

```json
{
  "protocol_version": 1,
  "task_id": "<TASK_ID>",
  "physical_target_category": "食品",
  "simulation_target_category": "日用品",
  "raw_text": "..."
}
```

记录实际 `<TASK_ID>`。

### 2.2 发送临时依赖就绪门控

```bash
rostopic pub -1 /task/dependencies_ready std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"status\": \"ready\"}'"
```

从 `/task/pickup_navigation_goal` 记录 `<PICKUP_GOAL_ID>`。

### 2.3 模拟到达取货观察点

```bash
rostopic pub -1 /task/pickup_arrived std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"goal_id\": \"<PICKUP_GOAL_ID>\", \"status\": \"arrived\", \"message\": \"\"}'"
```

从 `/qr_item_search/start` 记录 `<SEARCH_ID>`。

### 2.4 模拟三个 QR 候选

```bash
rostopic pub -1 /qr_item_search/result std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"search_id\": \"<SEARCH_ID>\", \"stamp\": 1000.0, \"status\": \"complete\", \"items\": [{\"order\": 1, \"item_name\": \"手机\", \"url\": \"https://example/3\", \"detected_yaw\": 0.0}, {\"order\": 2, \"item_name\": \"毛巾\", \"url\": \"https://example/2\", \"detected_yaw\": 1.2}, {\"order\": 3, \"item_name\": \"苹果\", \"url\": \"https://example/1\", \"detected_yaw\": 3.4}], \"message\": \"\"}'"
```

确认 `/llm/classify/request` 同时包含两个目标母类和三个候选，并记录
`<REQUEST_ID>`。

### 2.5 模拟双目标 LLM 成功结果

```bash
rostopic pub -1 /llm/classify/result std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"request_id\": \"<REQUEST_ID>\", \"status\": \"success\", \"physical\": {\"selected_order\": 3, \"selected_item\": \"苹果\", \"category\": \"食品\", \"workshop\": \"食品加工车间\"}, \"simulation\": {\"selected_order\": 2, \"selected_item\": \"毛巾\", \"category\": \"日用品\", \"workshop\": \"日用品加工车间\"}, \"message\": \"\"}'"
```

`/voice/speak` 的文本必须完全等于：

```text
取得苹果属于食品大类应放置在食品加工车间，仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间
```

记录 `<SPEECH_ID>`。

### 2.6 模拟播报完成

```bash
rostopic pub -1 /voice/speak_done std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"speech_id\": \"<SPEECH_ID>\", \"status\": \"success\", \"message\": \"\"}'"
```

`/task/delivery_navigation_goal` 应包含：

```json
{
  "target_workshop": "食品加工车间",
  "selected_item": "苹果"
}
```

不得包含仿真目标车间。记录 `<DELIVERY_GOAL_ID>`。

### 2.7 模拟到达实物目标车间

```bash
rostopic pub -1 /task/delivery_arrived std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"goal_id\": \"<DELIVERY_GOAL_ID>\", \"status\": \"arrived\", \"message\": \"\"}'"
```

`/task/status` 最终应为 `state: COMPLETE`、`status: complete`。

## 3. 失败与安全路径

每种情况使用新的 `task_id` 重新开始，避免终态或旧 identity 干扰。

### 3.1 QR 未找到

```bash
rostopic pub -1 /qr_item_search/result std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"search_id\": \"<SEARCH_ID>\", \"stamp\": 1001.0, \"status\": \"not_found\", \"items\": [], \"message\": \"items not found\"}'"
```

应进入 `ERROR`，且不发布 LLM 请求或配送导航目标。

### 3.2 LLM 错误

```bash
rostopic pub -1 /llm/classify/result std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"request_id\": \"<REQUEST_ID>\", \"status\": \"error\", \"message\": \"LLM service unavailable\"}'"
```

应进入 `ERROR`，且不发布播报或配送导航目标。

### 3.3 TTS 错误

```bash
rostopic pub -1 /voice/speak_done std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"speech_id\": \"<SPEECH_ID>\", \"status\": \"error\", \"message\": \"TTS failed\"}'"
```

应进入 `ERROR`，且不发布配送导航目标。

### 3.4 过期 identity

将当前阶段的 `search_id`、`request_id` 或 `speech_id` 替换成错误值。消息应
被忽略，状态和 deadline 不变，不得进入 `ERROR`。

### 3.5 取消

```bash
rostopic pub -1 /task/cancel std_msgs/String \
  "data: '{\"protocol_version\": 1, \"task_id\": \"<TASK_ID>\", \"reason\": \"operator_cancel\"}'"
```

应进入 `CANCELLED`。若当时处于 `WAITING_QR`，还应发布一次匹配当前
`search_id` 的 `/qr_item_search/stop`。

## 4. 使用真实 TTS

确认小车联网、音量安全并且 `tts_http.py` 可单独运行后，使用默认启动：

```bash
roslaunch task_orchestrator task_orchestrator.launch
```

执行到 LLM 成功结果后，`tts_bridge` 会真实播报并自动发布
`/voice/speak_done`，不要再人工发布该消息。

## 5. 停止

在 `roslaunch` 终端按 `Ctrl+C`，确认三个节点退出。不要另开第二个
`roslaunch` 重复启动相同节点。
