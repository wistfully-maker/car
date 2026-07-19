# 子任务 1 最小集成协议

## 1. 目的与范围

本文定义语音、导航、二维码搜索、大模型分类和语音播报之间的最小 ROS 集成协议，使各模块尚未全部完成时也能使用命令行或模拟节点独立联调。

本协议只规定模块边界和消息语义，不规定各模块内部实现。第一版使用 ROS 1 标准消息，复杂数据统一放入 `std_msgs/String` 的 UTF-8 JSON 中，避免现在新增自定义消息包。接口稳定后可迁移到自定义消息或 action。

## 2. 统一约定

- 协议版本固定为整数 `1`，字段名为 `protocol_version`。
- 一次完整语音任务使用唯一 `task_id`；一次二维码搜索使用唯一 `search_id`。
- 发起方生成 ID，后续消息原样传递，不得自行替换。
- 时间使用 ROS 时间，JSON 中以秒表示，字段名为 `stamp`。
- JSON 字符串使用 UTF-8，物品名称和类别保留原文，不做隐式翻译或类别改写。
- 未识别的附加字段必须忽略，以便后续兼容扩展。
- 必填字段缺失、类型错误或协议版本不支持时，接收方发布明确错误，不得猜测。
- 每个模块按 `task_id` 或 `search_id` 去重；重复请求不得启动第二次底盘动作或第二次推理。

## 3. 模块与话题所有权

| 话题 | 消息类型 | 发布方 | 订阅方 | 用途 |
| --- | --- | --- | --- | --- |
| `/voice/target_category` | `std_msgs/String` | 语音理解 | `task1_orchestrator` | 提供目标大类和原始指令 |
| `/task1/pickup_arrived` | `std_msgs/String` | 导航模块或模拟节点 | `task1_orchestrator` | 通知已到物品领取区观察点 |
| `/qr_item_search/start` | `std_msgs/String` | `task1_orchestrator` | `qr_item_search` | 启动一次三二维码搜索 |
| `/qr_item_search/stop` | `std_msgs/String` | `task1_orchestrator` 或安全模块 | `qr_item_search` | 停止当前搜索并停车 |
| `/qr_item_search/result` | `std_msgs/String` | `qr_item_search` | `task1_orchestrator` | 返回搜索进度或最终结果 |
| `/llm/classify/request` | `std_msgs/String` | `task1_orchestrator` | LLM 适配器 | 请求从三个物品中选择目标 |
| `/llm/classify/result` | `std_msgs/String` | LLM 适配器 | `task1_orchestrator` | 返回选择结果 |
| `/voice/speak` | `std_msgs/String` | `task1_orchestrator` | 语音合成 | 播报规定文本 |
| `/task1/navigation_goal` | `std_msgs/String` | `task1_orchestrator` | 导航模块 | 请求前往目标车间 |

`task1_orchestrator` 是流程状态的唯一拥有者。二维码包只负责旋转、识别三个二维码和解析网址，不接收目标类别，也不调用 LLM。

## 4. 消息定义

### 4.1 语音目标 `/voice/target_category`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "stamp": 1784426400.0,
  "target_category": "食品加工类",
  "raw_text": "小飞小飞，前往物品领取区，取得食品加工类物品，放置在对应仓库"
}
```

必填字段：`protocol_version`、`task_id`、`target_category`。`raw_text` 和 `stamp` 可选。

### 4.2 到达观察点 `/task1/pickup_arrived`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "stamp": 1784426420.0,
  "status": "arrived"
}
```

`status` 只能是 `arrived` 或 `failed`。失败时应附带 `message`。

### 4.3 启动搜索 `/qr_item_search/start`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "search_id": "search-20260719-001",
  "stamp": 1784426421.0,
  "expected_count": 3
}
```

`expected_count` 第一版必须为 `3`。二维码包收到同一个 `search_id` 的重复启动消息时，只返回当前状态，不重新旋转。

### 4.4 停止搜索 `/qr_item_search/stop`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "search_id": "search-20260719-001",
  "reason": "operator_stop"
}
```

停止请求在任何状态下都必须使二维码控制器立即持续发布零速度并结束当前搜索。

### 4.5 搜索结果 `/qr_item_search/result`

最终成功示例：

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "search_id": "search-20260719-001",
  "stamp": 1784426432.0,
  "status": "complete",
  "items": [
    {
      "order": 1,
      "item_name": "香蕉",
      "url": "http://example.test/food",
      "detected_yaw": 0.82
    },
    {
      "order": 2,
      "item_name": "毛巾",
      "url": "http://example.test/daily",
      "detected_yaw": 2.47
    },
    {
      "order": 3,
      "item_name": "手机",
      "url": "http://example.test/electronic",
      "detected_yaw": 4.91
    }
  ],
  "message": ""
}
```

`status` 取值：

- `searching`：搜索仍在进行，可携带当前已解析的 `items`。
- `complete`：恰好取得三个不同 URL 的有效物品结果。
- `not_found`：补扫结束后仍不足三个有效结果。
- `error`：相机、航向、底盘或内部错误。
- `stopped`：收到停止请求。

`items` 按二维码首次被识别的顺序排列，HTTP 返回先后不得改变顺序。`order` 从 1 开始连续编号。`detected_yaw` 是相对本次搜索起始航向的累计弧度，可大于 `2π`，便于描述越过起点后的补扫位置。

当 `status` 为 `not_found`、`error` 或 `stopped` 时，允许返回已经成功解析的部分结果，并必须提供非空 `message`。`complete` 必须包含三个不同 URL 且 `item_name` 非空。

### 4.6 LLM 请求 `/llm/classify/request`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "stamp": 1784426433.0,
  "target_category": "食品加工类",
  "candidates": [
    {"order": 1, "item_name": "香蕉"},
    {"order": 2, "item_name": "毛巾"},
    {"order": 3, "item_name": "手机"}
  ]
}
```

只有二维码搜索 `complete` 后才能发送该请求。候选顺序必须与二维码搜索结果一致。

### 4.7 LLM 结果 `/llm/classify/result`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "stamp": 1784426434.0,
  "status": "success",
  "selected_order": 1,
  "selected_item": "香蕉",
  "target_category": "食品加工类",
  "target_workshop": "食品加工车间",
  "message": ""
}
```

`status` 只能是 `success` 或 `error`。成功时 `selected_order` 必须对应请求中的一个候选，`selected_item` 必须与该候选完全一致。错误时必须提供 `message`，流程不得继续导航。

### 4.8 播报 `/voice/speak`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "text": "香蕉属于食品大类应放置在食品加工车间"
}
```

播报文案由 `task1_orchestrator` 根据赛事要求生成，TTS 模块只负责朗读，不修改文本。

### 4.9 后续导航 `/task1/navigation_goal`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260719-001",
  "target_workshop": "食品加工车间",
  "selected_item": "香蕉"
}
```

第一版二维码升级不实现该导航消费者，只保留协议。

## 5. 流程和超时

1. `task1_orchestrator` 缓存语音目标。
2. 收到同一 `task_id` 的 `pickup_arrived` 后生成 `search_id` 并启动二维码搜索。
3. 二维码包返回 `complete` 后，编排器发送一次 LLM 请求。
4. LLM 返回 `success` 后，编排器先发送播报文本，再发布导航目标。
5. 任一阶段失败时，编排器停止流程并报告错误；不得用不完整的二维码结果请求 LLM。

建议的编排器默认超时：

- 等待到达观察点：由导航模块自身管理，协议不额外限定。
- 等待二维码最终结果：45 秒。
- 等待 LLM 结果：15 秒。
- 等待 TTS 完成：第一版不阻塞后续导航；后续若 TTS 提供完成回执再调整。

二维码搜索内部的旋转和补扫超时由二维码包配置，但必须在编排器 45 秒总超时内结束。

## 6. 模拟联调

在真实语音、LLM 和导航模块接入前，使用模拟发布者完成端到端验证：

- 命令行发布固定 `target_category`。
- 命令行发布 `pickup_arrived`。
- 模拟 LLM 按确定性映射从三个候选中返回一个结果。
- `/voice/speak` 和 `/task1/navigation_goal` 仅记录消息，不驱动车辆。

最小联调通过标准：

- 同一 `task_id` 能贯穿全部消息。
- 二维码不足三个时不会调用 LLM。
- 三个物品到齐后只调用一次 LLM。
- LLM 成功后播报和导航目标中的物品、类别、车间彼此一致。
- 重复请求不会引发重复旋转、重复推理或重复导航。
