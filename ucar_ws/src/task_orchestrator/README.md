# task_orchestrator

U-CAR 子任务 1 的全流程 ROS 编排包。它连接语音识别、取货点导航、二维码
扫描、双目标 LLM 分类、语音播报和实物目标车间导航。

## 当前范围

本包已经实现：

- `/question` 中两个母类的解析与去重；
- 单活动任务状态机和分阶段超时；
- QR 三候选校验；
- 实物与仿真物品的双目标 LLM 协议；
- 赛事规定格式的确定性播报文本；
- 对现有 `speech_command/scripts/tts_http.py` 的安全桥接；
- 播报完成后只发送实物目标车间和实物名称；
- 过期消息忽略、重复消息幂等、取消和错误终止。

本包不负责实现导航算法、二维码识别算法、语音识别硬件驱动或 LLM 服务。
这些节点应由统一 bringup 常驻启动，本编排器仅通过 topic 协调。

## 一键启动

在小车 ROS 1 Noetic 环境中：

```bash
cd ~/ucar_ws
catkin_make
source devel/setup.bash
roslaunch task_orchestrator task_orchestrator.launch
```

默认同时启动：

- `task_orchestrator`
- `voice_task_adapter`
- `tts_bridge`

手动模拟 TTS 时应关闭真实桥接，避免实际播报和人工 `speak_done` 冲突：

```bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_tts_bridge:=false
```

也可以使用 `enable_voice_adapter:=false` 关闭 `/question` 适配器，直接发布
`/voice/task_request`。

停止时在 `roslaunch` 终端按 `Ctrl+C`。正常退出不需要逐个执行
`rosnode kill`，也不要重复启动同名节点。

## 状态流程

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

任一活动状态都可以进入 `ERROR` 或 `CANCELLED`。`COMPLETE`、`ERROR` 和
`CANCELLED` 可以接收新的 `task_id`。

## Topic

所有业务 topic 均使用 `std_msgs/String`，`data` 是 protocol v1 JSON。

| 方向 | Topic | 用途 |
|---|---|---|
| 输入 | `/question` | 现有语音模块识别文本 |
| 输入 | `/task/dependencies_ready` | 临时依赖就绪门控 |
| 输入 | `/task/pickup_arrived` | 到达取货观察点 |
| 输入 | `/qr_item_search/result` | QR 搜索中间态或终态 |
| 输入 | `/llm/classify/result` | 实物和仿真双目标结果 |
| 输入 | `/voice/speak_done` | 播报完成或失败 |
| 输入 | `/task/delivery_arrived` | 到达实物目标车间 |
| 输入 | `/task/cancel` | 取消当前任务 |
| 输出 | `/voice/task_request` | 两个目标母类 |
| 输出 | `/task/pickup_navigation_goal` | 取货观察点导航目标 |
| 输出 | `/qr_item_search/start` | 启动 QR 搜索 |
| 输出 | `/qr_item_search/stop` | 停止 QR 搜索 |
| 输出 | `/llm/classify/request` | 三候选和两个目标母类 |
| 输出 | `/voice/speak` | 固定格式播报请求 |
| 输出 | `/task/delivery_navigation_goal` | 实物车间与实物名称 |
| 输出 | `/task/status` | 当前状态和终态 |

`/task/dependencies_ready` 是模块健康状态尚未统一前的临时门控。后续应由
`/system/module_status` 聚合器自动产生，不能在正式比赛中长期依赖人工发布。

## 配置

配置文件为 `config/orchestrator.yaml`：

- `timeouts.*`：每个状态的独立超时；
- `voice_adapter.debounce_seconds`：相同识别文本去重窗口；
- `tts_bridge.command`：现有 TTS 程序的参数列表；
- `tts_bridge.timeout`：单次 TTS 最大等待时间。

两个导航阶段默认 300 秒，均可配置。运行中不通过重复启动节点切换参数。

## 本地测试

Windows 或小车均可运行纯 Python 测试：

```bash
python -m unittest discover \
  -s ucar_ws/src/task_orchestrator/test \
  -p "test_*.py" -v
```

这些测试不需要 `rospy`，不会访问网络、扬声器或真实 TTS 进程。

小车端的完整模拟步骤见
[`test/manual_simulation.md`](test/manual_simulation.md)。

## 尚待真实模块接入

- 让导航节点消费两个导航目标并返回带 identity 的到达结果；
- 将已部署 `llm_spark` 升级为双目标协议；
- 用模块健康聚合器替换临时依赖门控；
- 在小车上执行 Catkin 构建、ROS topic 模拟和真实 TTS 验证。
