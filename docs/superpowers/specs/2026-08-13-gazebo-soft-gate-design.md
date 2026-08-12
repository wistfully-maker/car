# Gazebo 仿真任务软门控设计

## 目标

在第二部分仿真车间停车成功与原有“仿真任务已完成”播报之间插入一次电脑端 Gazebo 任务，且不牺牲后续播报和第三部分的得分机会。

新流程为：

```text
/task/simulation_arrived status=arrived
  -> 发布 /task/gazebo/start
  -> 等待 /task/gazebo/complete
  -> success / failure / timeout 三条路径汇合
  -> 按原文播报“仿真任务已完成，已将…放入…”
  -> 匹配的 /voice/speak_done status=success
  -> 发布 /task/line_navigation_goal
  -> 红绿灯识别与巡线
```

Gazebo 是软门控。Gazebo 成功、显式失败或超时都必须继续原有播报和第三部分。物理小车未成功停入仿真车间、用户取消任务、TTS 失败等既有硬失败语义不在本设计中放宽。

## 方案比较

### 方案 A：在 `task_orchestrator` 中加入原生等待态（采用）

新增 `WAITING_GAZEBO`，由状态机发布 start、校验 complete、处理软超时，再进入现有 `WAITING_SIMULATION_SPEECH`。

优点：任务顺序、去重、超时和取消都由唯一状态机管理；不会伪造停车到达；容易做纯 Python 回归测试。改动集中在 `task_orchestrator`，工程量可控。

### 方案 B：外置 relay 拦截 `/task/simulation_arrived`

relay 暂存停车成功消息，等 Gazebo 完成后再转发。虽然表面改动少，但会改变既有 topic 的真实性，引入两个节点共同掌握业务顺序，也更容易在重启时重复推进，因此不采用。

### 方案 C：用文件、rosparam 或 shell 轮询完成标志

实现最快，但没有任务身份关联，旧文件和旧参数可能让新任务误完成；电脑与车之间也缺少明确协议，因此不采用。

## ROS 架构和资源所有权

- 小车继续运行 ROS Master 和现有总 launch。
- 电脑端 Gazebo 与小车加入同一个 ROS1 Master；电脑端单独启动 Gazebo 和桥接节点。
- 小车的 `competition_full.launch` 不启动、不停止、不监控 Gazebo 进程，也不新增 `start_gazebo`。
- 电脑端节点不得发布任何小车速度、运动模式、导航目标或语音 topic。
- Gazebo 等待期间 `/task/motion_mode` 必须为 `IDLE`；最终 `/cmd_vel` 仍只由既有速度仲裁链拥有。
- 两个 Gazebo topic 都使用 `std_msgs/String` JSON，均不 latch。电脑端桥接节点必须在任务开始前上线。

## Topic 合同

### `/task/gazebo/start`

车端发布，电脑端订阅。每次仿真车间停车成功只发布一次：

```json
{
  "protocol_version": 1,
  "task_id": "task-...",
  "goal_id": "gazebo-...",
  "selected_item": "手机",
  "target_category": "电子产品",
  "target_workshop": "电子产品生产车间"
}
```

Gazebo 业务只依赖 `selected_item`、`target_category`、`target_workshop`。`task_id` 和 `goal_id` 只用于联调关联、去重和拒绝旧消息，电脑端必须原样回传。

### `/task/gazebo/complete`

电脑端发布，车端订阅。成功消息：

```json
{
  "protocol_version": 1,
  "task_id": "task-...",
  "goal_id": "gazebo-...",
  "status": "success"
}
```

失败消息：

```json
{
  "protocol_version": 1,
  "task_id": "task-...",
  "goal_id": "gazebo-...",
  "status": "failure",
  "reason": "仿真内部错误摘要"
}
```

只接受 `success` 和 `failure`。`failure` 必须带非空 `reason`。协议错误、错误 `task_id`、错误 `goal_id`、等待态之外的消息和重复消息一律不推进状态；协议错误只记录诊断并继续等待，最终由软超时兜底。

## 状态机语义

新增配置：

```text
gazebo_phase_enabled=true       # 总 launch 默认启用；子 launch 默认可关闭
timeout_gazebo=60.0             # 秒，可从总 launch 临时覆盖
```

状态转换：

| 当前状态 | 事件 | 下一状态 | 处理 |
|---|---|---|---|
| `NAVIGATING_TO_SIM_WORKSHOP` | 匹配的 `arrived`，Gazebo 启用 | `WAITING_GAZEBO` | 生成 `gazebo_goal_id`，发布一次 start |
| `NAVIGATING_TO_SIM_WORKSHOP` | 匹配的 `arrived`，Gazebo 关闭 | `WAITING_SIMULATION_SPEECH` | 保留当前兼容流程 |
| `WAITING_GAZEBO` | 匹配的 `success` | `WAITING_SIMULATION_SPEECH` | 记录 success，发起原播报 |
| `WAITING_GAZEBO` | 匹配的 `failure` | `WAITING_SIMULATION_SPEECH` | 记录 reason，仍发起原播报 |
| `WAITING_GAZEBO` | 60 秒超时 | `WAITING_SIMULATION_SPEECH` | 记录 timeout，仍发起原播报 |
| `WAITING_GAZEBO` | 用户取消 | `CANCELLED` | 正常硬停止，不继续播报 |
| `WAITING_SIMULATION_SPEECH` | 匹配的 TTS success | `NAVIGATING_LINE_START` | 发布第三部分导航点 |

三条 Gazebo 结束路径调用同一个内部汇合函数，避免 failure 或 timeout 漏掉播报。失败和超时仅写 ROS warning，并在进入 `WAITING_SIMULATION_SPEECH` 时通过 `/task/status.message` 留下诊断；播报文本仍严格使用现有正向文案，不播报失败原因。为此只给现有 `_start_speech()` 增加一个默认空值的可选 `status_message` 参数，其他播报调用保持原行为。

## 电脑端适配器约束

Gazebo 现有代码尚未提供，因此不猜测其包名、主文件或内部完成条件。实施电脑端适配器前必须先只读确认 Gazebo 仓库的绝对路径、任务入口和真实完成/失败回调。

适配器只负责：

1. 订阅 `/task/gazebo/start` 并按 `task_id + goal_id` 去重；
2. 将三个业务字段交给现有 Gazebo 任务入口；
3. 在真实结束或捕获异常时恰好发布一次 `/task/gazebo/complete`；
4. 原样回传 `task_id` 和 `goal_id`；
5. 不直接修改小车状态机，不发布小车运动命令。

电脑端进程崩溃或网络断开时不要求它补发 failure；车端 60 秒软超时负责继续比赛流程。

## 范围外

- 不修改 Gazebo 场景、模型、评分算法或仿真任务本体。
- 不让车端 SSH 启停电脑端程序。
- 不新增 ROS 自定义消息、service 或 action。
- 不改变第二部分停车、第三部分导航、红绿灯识别或 V5 巡线算法。
- 不把 Gazebo failure/timeout 变成整车 `ERROR`。

## 验收标准

1. 仿真车间停车成功后先出现一条 start，播报不得提前。
2. start 精确携带物品、类别、车间和关联身份。
3. success、failure、timeout 都各自只触发一次原播报，并在 TTS success 后各自只发布一次第三部分导航目标。
4. stale、重复或畸形 complete 不触发播报或导航。
5. `WAITING_GAZEBO` 全程 motion mode 为 `IDLE`，无非零底盘速度。
6. Gazebo 关闭开关后，现有流程行为不变。
7. Gazebo 失败或超时不会使后续可得分环节丢失。
