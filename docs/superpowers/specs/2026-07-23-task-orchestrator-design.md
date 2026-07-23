# 全流程任务编排器设计

## 1. 目标与当前进度

新增 ROS 1 Noetic 节点 `task_orchestrator`，作为整车全流程的唯一任务状态拥有者，串联：

1. 离线语音识别；
2. 前往物品领取区的导航；
3. 三二维码连续搜索；
4. LLM 对实物和仿真物品的双目标分类；
5. 严格格式的语音播报；
6. 将实物目标车间发送给后续避障导航。

当前已经验证：

- QR 搜索能稳定返回三个候选物品；
- LLM 能对一个实物目标母类完成分类；
- QR 与 LLM 可以人工通过 topic 联调。

本阶段需要补齐：

- 语音同时提取实物目标母类和仿真目标母类；
- LLM 从同一组三个候选中分别选择实物和仿真物品；
- 统一任务编排、去重、超时、取消和错误处理；
- 等待语音播报完成后，再向避障导航发送实物目标车间。

## 2. 核心架构原则

### 2.1 集中式编排

`task_orchestrator` 保存任务上下文并决定阶段转换。语音、导航、QR、LLM 和 TTS 模块不直接启动彼此。

二维码包不接收目标母类。它只负责识别三个二维码并返回三个物品。编排器将两个目标母类与三个候选组合后发送给 LLM。

### 2.2 节点常驻

基础节点由统一 bringup 在整车启动时启动一次。编排器只发送任务 topic，不在阶段切换时执行 `roslaunch`、`rosrun` 或 `rosnode kill`。

```text
system_bringup
├── base_driver
├── usb_cam
├── pickup_navigation
├── obstacle_navigation
├── qr_scanner
├── item_search_controller
├── llm_adapter
├── offline_voice
├── tts
└── task_orchestrator
```

同一硬件只有一个拥有者：

| 硬件资源 | 唯一拥有节点 |
|---|---|
| `/dev/video0` | `usb_cam` |
| 底盘串口 | `base_driver` |
| 雷达串口 | `ydlidar_node` |
| 麦克风 | `offline_voice` |
| 扬声器 | `tts` |

其他模块订阅公共 topic，不重复打开硬件。第一版不处理运行中关闭并用不同参数重启节点的情形。

### 2.3 统一参数

全流程使用一套统一启动参数。后续需要为不同阶段提速或改变视觉策略时，优先采用：

1. 多个算法节点订阅同一个原始传感器 topic，各自使用独立算法参数；
2. 由硬件管理节点提供动态模式切换接口，例如 `/camera/set_mode`。

## 3. 总体消息流

```text
offline_voice
  │ /voice/task_request
  ▼
task_orchestrator
  │ /task/pickup_navigation_goal
  ▼
pickup_navigation
  │ /task/pickup_arrived
  ▼
task_orchestrator
  │ /qr_item_search/start
  ▼
qr_item_search
  │ /qr_item_search/result
  ▼
task_orchestrator
  │ /llm/classify/request
  ▼
llm_adapter
  │ /llm/classify/result
  ▼
task_orchestrator
  │ /voice/speak
  ▼
tts
  │ /voice/speak_done
  ▼
task_orchestrator
  │ /task/delivery_navigation_goal
  ▼
obstacle_navigation
  │ /task/delivery_arrived
  ▼
task_orchestrator
  │ /task/status
  ▼
COMPLETE
```

## 4. 统一协议约定

- 所有业务 topic 第一版使用 `std_msgs/String`，内容为 UTF-8 JSON。
- `protocol_version` 固定为整数 `1`。
- 一条完整语音任务使用唯一非空 `task_id`。
- 每个异步阶段还使用阶段专用 ID：
  - QR：`search_id`
  - LLM：`request_id`
  - TTS：`speech_id`
  - 导航：`goal_id`
- 所有返回消息必须同时匹配 `task_id` 和对应阶段 ID。
- 未知附加字段允许忽略；必填字段缺失、类型错误或不支持的协议版本必须拒绝。
- 产品母类内部只使用三个规范值：
  - `食品`
  - `日用品`
  - `电子产品`
- 同一组三个二维码候选同时用于实物和仿真目标选择。

## 5. Topic 所有权

| Topic | 发布方 | 订阅方 | 用途 |
|---|---|---|---|
| `/voice/task_request` | 离线语音 | `task_orchestrator` | 两个目标母类和原始指令 |
| `/task/pickup_navigation_goal` | `task_orchestrator` | 前段导航 | 前往物品区观察点 |
| `/task/pickup_arrived` | 前段导航 | `task_orchestrator` | 观察点到达或失败 |
| `/qr_item_search/start` | `task_orchestrator` | QR 包 | 启动三二维码搜索 |
| `/qr_item_search/stop` | `task_orchestrator` | QR 包 | 停止当前搜索 |
| `/qr_item_search/result` | QR 包 | `task_orchestrator` | 搜索状态和三个候选 |
| `/llm/classify/request` | `task_orchestrator` | LLM 适配器 | 两个目标母类和三个候选 |
| `/llm/classify/result` | LLM 适配器 | `task_orchestrator` | 实物和仿真双分类结果 |
| `/voice/speak` | `task_orchestrator` | TTS | 固定格式播报文本 |
| `/voice/speak_done` | TTS | `task_orchestrator` | 播报成功或失败 |
| `/task/delivery_navigation_goal` | `task_orchestrator` | 避障导航 | 前往实物目标车间 |
| `/task/delivery_arrived` | 避障导航 | `task_orchestrator` | 目标车间到达或失败 |
| `/task/cancel` | 人工或安全模块 | `task_orchestrator` | 取消当前任务 |
| `/task/status` | `task_orchestrator` | 调试与上层模块 | 全流程状态、错误和完成信息 |
| `/system/module_status` | 各常驻模块 | `task_orchestrator` | 模块 ready/error 与配置版本 |

## 6. 消息定义

### 6.1 语音任务 `/voice/task_request`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "stamp": 1784775000.0,
  "physical_target_category": "食品",
  "simulation_target_category": "日用品",
  "raw_text": "前往物品领取区，取得食品类物品，并领取仿真环境中需要的日用品类物品"
}
```

必填：

- `protocol_version`
- `task_id`
- `physical_target_category`
- `simulation_target_category`
- `raw_text`

两个母类都必须是规范值。它们可以相同；即使相同，也必须分别保留实物和仿真两个结果字段。

### 6.2 前往物品区 `/task/pickup_navigation_goal`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "goal_id": "pickup-task-20260723-001",
  "target": "物品领取区观察点"
}
```

### 6.3 到达物品区 `/task/pickup_arrived`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "goal_id": "pickup-task-20260723-001",
  "status": "arrived",
  "message": ""
}
```

`status` 只能为 `arrived` 或 `failed`。失败时 `message` 必须非空。

### 6.4 QR 启动 `/qr_item_search/start`

沿用现有 QR protocol v1：

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "search_id": "search-task-20260723-001",
  "expected_count": 3
}
```

编排器只在观察点导航成功后发送该消息。

### 6.5 QR 结果 `/qr_item_search/result`

沿用现有结果格式。只有 `status: "complete"` 且恰好包含三个不同候选时，才进入 LLM 阶段。

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "search_id": "search-task-20260723-001",
  "status": "complete",
  "items": [
    {"order": 1, "item_name": "手机", "url": "https://example/3", "detected_yaw": 0.0},
    {"order": 2, "item_name": "毛巾", "url": "https://example/2", "detected_yaw": 1.27},
    {"order": 3, "item_name": "苹果", "url": "https://example/1", "detected_yaw": 3.39}
  ],
  "message": ""
}
```

`not_found`、`error` 或 `stopped` 不得调用 LLM。

### 6.6 LLM 请求 `/llm/classify/request`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "request_id": "llm-task-20260723-001",
  "physical_target_category": "食品",
  "simulation_target_category": "日用品",
  "candidates": [
    {"order": 1, "item_name": "手机"},
    {"order": 2, "item_name": "毛巾"},
    {"order": 3, "item_name": "苹果"}
  ]
}
```

候选顺序必须与 QR 结果一致。一个请求同时完成实物和仿真两次选择，不拆成两次 LLM 请求。

### 6.7 LLM 结果 `/llm/classify/result`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "request_id": "llm-task-20260723-001",
  "status": "success",
  "physical": {
    "selected_order": 3,
    "selected_item": "苹果",
    "category": "食品",
    "workshop": "食品加工车间"
  },
  "simulation": {
    "selected_order": 2,
    "selected_item": "毛巾",
    "category": "日用品",
    "workshop": "日用品加工车间"
  },
  "message": ""
}
```

校验规则：

- `status` 只能为 `success` 或 `error`。
- 两个 `selected_order` 都必须属于原候选集合。
- `selected_item` 必须与对应 order 完全一致。
- `physical.category` 必须等于语音中的实物目标母类。
- `simulation.category` 必须等于语音中的仿真目标母类。
- 车间必须与本地固定映射一致。
- `error` 时必须包含非空 `message`，且不得继续 TTS 或导航。

当前仅返回 `selected_item/category/workshop` 的单目标 LLM 输出需要升级为上述双目标、带 identity 的协议。

### 6.8 固定分类映射

编排器持有本地可信映射：

```python
CATEGORY_CONFIG = {
    "食品": {
        "label": "食品大类",
        "workshop": "食品加工车间",
    },
    "日用品": {
        "label": "日用品大类",
        "workshop": "日用品加工车间",
    },
    "电子产品": {
        "label": "电子产品大类",
        "workshop": "电子产品生产车间",
    },
}
```

目标仓库由本地映射验证和确定，不完全信任 LLM 返回的自由文本。

### 6.9 播报请求 `/voice/speak`

播报文本由编排器按赛事模板确定性拼接，不能让 LLM 或 TTS 自由改写。

固定模板：

```text
取得[实物名称]属于[实物目标大类]应放置在[实物目标仓库]，仿真环境中取得[仿真物品名称]属于[仿真目标大类]应放置在[仿真目标仓库]
```

示例：

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "speech_id": "speech-task-20260723-001",
  "text": "取得苹果属于食品大类应放置在食品加工车间，仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间"
}
```

### 6.10 播报完成 `/voice/speak_done`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "speech_id": "speech-task-20260723-001",
  "status": "success",
  "message": ""
}
```

只有匹配当前任务和 `speech_id` 的 `success` 才能触发后续导航。

### 6.11 避障导航目标 `/task/delivery_navigation_goal`

只发送实物目标车间，不发送仿真车间：

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "goal_id": "delivery-task-20260723-001",
  "target_workshop": "食品加工车间",
  "selected_item": "苹果"
}
```

### 6.12 到达目标车间 `/task/delivery_arrived`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "goal_id": "delivery-task-20260723-001",
  "status": "arrived",
  "message": ""
}
```

避障导航尚未实现时，可以只观察 `/task/delivery_navigation_goal`，暂不伪造真实车辆已经到达。

### 6.13 取消 `/task/cancel`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "reason": "operator_cancel"
}
```

只有匹配当前活动 `task_id` 的取消请求有效。

### 6.14 全流程状态 `/task/status`

```json
{
  "protocol_version": 1,
  "task_id": "task-20260723-001",
  "state": "WAITING_LLM",
  "status": "running",
  "message": ""
}
```

终态 `status` 为 `complete`、`error` 或 `cancelled`。错误和取消时 `message` 必须非空。

## 7. 状态机

```text
IDLE
  -> CHECKING_DEPENDENCIES
  -> NAVIGATING_TO_PICKUP
  -> WAITING_QR
  -> WAITING_LLM
  -> WAITING_SPEECH
  -> NAVIGATING_TO_WORKSHOP
  -> COMPLETE
```

任一活动状态均可进入：

- `ERROR`
- `CANCELLED`

转换条件：

| 当前状态 | 输入 | 下一状态 |
|---|---|---|
| `IDLE` | 合法语音任务 | `CHECKING_DEPENDENCIES` |
| `CHECKING_DEPENDENCIES` | 所有必要模块 ready | `NAVIGATING_TO_PICKUP` |
| `NAVIGATING_TO_PICKUP` | 匹配的 `arrived` | `WAITING_QR` |
| `WAITING_QR` | QR `complete` 且三个候选有效 | `WAITING_LLM` |
| `WAITING_LLM` | 双目标 LLM 结果校验通过 | `WAITING_SPEECH` |
| `WAITING_SPEECH` | 匹配的播报 `success` | `NAVIGATING_TO_WORKSHOP` |
| `NAVIGATING_TO_WORKSHOP` | 匹配的 `arrived` | `COMPLETE` |

## 8. 超时

所有超时从 YAML 加载，不写死在状态机中：

```yaml
timeouts:
  dependency_ready: 30.0
  pickup_navigation: 300.0
  qr_search: 90.0
  llm_classification: 60.0
  speech: 60.0
  delivery_navigation: 300.0
  cancel_ack: 15.0
```

QR 包内部超时建议为 75 秒，编排器等待 QR 为 90 秒，保留 15 秒用于 ROS 调度、HTTP 和终态传递。

不设置紧张的全任务统一总超时；每个阶段独立计时。以后根据真实多轮数据调整 YAML。

## 9. 去重、新任务和迟到消息

- 同一时间只允许一个活动任务。
- `IDLE`、`COMPLETE`、`ERROR` 或 `CANCELLED` 可接受新的 `task_id`。
- 活动期间收到相同 `task_id` 的语音请求：
  - 不重复执行；
  - 重新发布当前 `/task/status`。
- 活动期间收到不同 `task_id`：
  - 返回 `busy`；
  - 不打断当前任务。
- 每个阶段请求只发布一次。
- 已结束阶段的重复成功消息只重新发布当前状态，不重复触发下游。
- 不匹配当前 `task_id` 或阶段 ID 的迟到结果直接忽略并记录日志。
- LLM、TTS 和导航回调不得仅凭消息内容推进状态，必须同时检查当前状态和 identity。

## 10. 取消与错误处理

取消时：

- QR 阶段发布匹配 identity 的 `/qr_item_search/stop`。
- 导航阶段发布对应导航取消接口；具体 topic 在导航模块设计时确定。
- LLM/TTS 阶段无法真正中断时，标记任务已取消并忽略之后的旧结果。
- 编排器发布 `CANCELLED`，不继续任何下游动作。

阶段失败或超时时：

1. 发送当前模块可用的停止/取消请求；
2. 保持或请求底盘停车；
3. 发布 `ERROR` 和明确原因；
4. 不自动无限重试；
5. 等待新的 `task_id`。

第一版不实现编排器在运行中重启其他 ROS 节点。

## 11. 模块健康与重复启动

仅检查 `rosnode list` 不足以证明模块可用。常驻模块应周期性或 latched 发布：

```json
{
  "module": "qr_item_search",
  "status": "ready",
  "config_version": "qr-v1",
  "message": ""
}
```

编排器在启动任务前检查必要模块 ready 和配置版本。

避免冲突的规则：

- 同一 ROS node name 只由统一 bringup 启动一次。
- 同一串口、相机或音频设备只有一个拥有节点。
- 阶段切换只发送 topic，不重新启动节点。
- 公共参数在 bringup 前统一加载。
- 算法参数不同则使用不同处理节点订阅同一原始数据。
- 后续需要改变硬件参数时使用明确的模式切换接口，不重复启动硬件节点。

## 12. 第一版开发范围

第一版 `task_orchestrator` 实现：

- 纯 Python 协议解析和状态机；
- ROS topic 订阅、发布和定时器；
- 两个目标母类缓存；
- QR 三候选校验；
- 双目标 LLM 请求与结果校验；
- 固定播报文本生成；
- 播报完成后发送实物目标车间；
- 可调阶段超时；
- 去重、迟到消息忽略、取消和错误状态；
- 命令行模拟测试。

暂不实现：

- 运行中重启其他节点；
- 自动切换相机硬件参数；
- 自定义 ROS msg/action；
- 尚未完成的导航算法内部逻辑；
- LLM、TTS、离线语音算法内部实现。

## 13. 推荐开发顺序

由使用者编写代码，Codex 负责逐步讲解、review、测试和本地提交：

1. 建立 `task_orchestrator` ROS 包骨架。
2. 编写纯 Python 协议解析与固定分类映射。
3. 编写纯 Python 状态机和输出 action。
4. 编写单元测试，覆盖正常流程、重复消息、迟到消息、超时和取消。
5. 编写 ROS node 适配层。
6. 使用命令行模拟语音、导航、QR、LLM 和 TTS。
7. 接入现有 QR 与 LLM。
8. 接入离线语音和前段导航。
9. 接入 TTS 完成回执和避障导航目标。

每一步通过 review 和测试后单独本地提交。
