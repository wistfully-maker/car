# task_orchestrator

U-CAR 子任务 1 的 ROS 1 Noetic 全流程编排包。它不替代各业务模块，而是使用
`std_msgs/String` 和 protocol v1 JSON 连接：

```text
语音识别
  -> 取货观察点导航
  -> QR 扫描三个候选物品
  -> LLM 同时选择实物和仿真物品
  -> 按赛事固定格式播报
  -> 导航到实物目标车间
```

## 1. 当前完成情况

已经实现并在 `ucar@172.20.10.4` 上验证：

- 从 `/question` 提取两个母类：`食品`、`日用品`、`电子产品`；
- 单活动任务状态机和分阶段超时；
- QR 三候选、顺序、唯一性和 identity 校验；
- 实物与仿真物品的双目标 LLM 协议；
- 赛事规定格式的确定性播报文本；
- 对 `speech_command/scripts/tts_http.py` 的真实 TTS 桥接；
- 播报完成后只向导航发送实物名称和实物目标车间；
- 过期消息忽略、重复消息幂等、取消和错误终止。

2026-07-23 小车验证结果：

- Python 3 单元测试：47/47 通过；
- `catkin_make`：通过；
- 手动端到端成功路径：通过；
- QR、LLM、TTS 失败路径：通过；
- 过期 identity 和取消路径：通过；
- 真实 TTS 自动完成回报：通过；
- `Ctrl+C` 等效停止：三个节点均无残留。

本包不实现导航算法、二维码识别算法、语音识别硬件驱动或 LLM 服务。正式比赛
时这些节点应由统一 bringup 常驻启动，本编排器只通过 topic 协调，不能重复启动
摄像头、串口、底盘或同名业务节点。

## 2. 环境与部署

### 2.1 小车环境

- ROS 1 Noetic；
- Python 3；
- 工作空间：`/home/ucar/ucar_ws`；
- 包路径：`/home/ucar/ucar_ws/src/task_orchestrator`；
- TTS 脚本：
  `/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py`。

通过非交互 SSH 执行 ROS 命令时，必须显式加载环境：

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
```

不要直接使用系统默认的 `python`。该命令在部分小车环境中仍指向 Python 2，
测试和节点统一使用 `python3`。

### 2.2 从 Windows 部署

在 Windows PowerShell 中：

```powershell
cd D:\program_sec\智能车\.worktrees\task_orchestrator

scp -r .\ucar_ws\src\task_orchestrator `
  ucar@172.20.10.4:/home/ucar/ucar_ws/src/

ssh ucar@172.20.10.4
```

首次部署前若小车已经有同名目录，应先确认该目录是否包含现场修改，不能直接覆盖。
部署后在小车执行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make
source devel/setup.bash
rospack find task_orchestrator
```

最后一条命令应输出：

```text
/home/ucar/ucar_ws/src/task_orchestrator
```

## 3. 一键启动

### 3.1 正式运行

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
source devel/setup.bash
roslaunch task_orchestrator task_orchestrator.launch
```

默认同时启动：

- `/task_orchestrator`
- `/voice_task_adapter`
- `/tts_bridge`

这只是一键启动编排包，不会自动启动导航、QR、LLM 和语音识别的全部依赖。正式
全车一键启动应由后续统一 bringup 文件负责，并确保每个硬件所有者节点只启动一次。

### 3.2 人工模拟联调

保留 `/question` 适配器，但关闭真实 TTS：

```bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_tts_bridge:=false
```

此时必须人工发布 `/voice/speak_done`。

完全绕开真实语音和 TTS：

```bash
roslaunch task_orchestrator task_orchestrator.launch \
  enable_voice_adapter:=false \
  enable_tts_bridge:=false
```

此时直接向 `/voice/task_request` 发布 protocol v1 JSON。

## 4. 分步启动

分步启动用于定位具体节点问题。每个终端都应先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
```

终端 1，启动核心状态机：

```bash
rosrun task_orchestrator task_orchestrator_node.py \
  _timeouts/dependency_ready:=30.0 \
  _timeouts/pickup_navigation:=300.0 \
  _timeouts/qr_search:=90.0 \
  _timeouts/llm_classification:=60.0 \
  _timeouts/speech:=60.0 \
  _timeouts/delivery_navigation:=300.0 \
  _timeouts/cancel_ack:=15.0
```

终端 2，需要从 `/question` 生成任务时启动：

```bash
rosrun task_orchestrator voice_task_adapter_node.py \
  _voice_adapter/debounce_seconds:=2.0
```

终端 3，需要真实播报时启动：

```bash
rosrun task_orchestrator tts_bridge_node.py \
  _tts_bridge/command:="['python3', '/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py']" \
  _tts_bridge/timeout:=30.0
```

分步启动前运行：

```bash
rosnode list
```

若同名节点已经存在，不要再次启动。先找到原节点所属的 launch 终端，再决定复用
或停止整个 launch。不要为了切换参数同时运行两个同名节点。

## 5. 状态机

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

状态含义：

| 状态 | 正在等待的输入 |
|---|---|
| `CHECKING_DEPENDENCIES` | `/task/dependencies_ready` |
| `NAVIGATING_TO_PICKUP` | `/task/pickup_arrived` |
| `WAITING_QR` | `/qr_item_search/result` |
| `WAITING_LLM` | `/llm/classify/result` |
| `WAITING_SPEECH` | `/voice/speak_done` |
| `NAVIGATING_TO_WORKSHOP` | `/task/delivery_arrived` |

## 6. Topic 接口

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

完整 JSON 示例和逐步模拟命令见
[`test/manual_simulation.md`](test/manual_simulation.md)。

重要约束：

- 后续模块必须原样返回当前阶段输出中的 `task_id` 和阶段 ID；
- 阶段 ID 包括 `goal_id`、`search_id`、`request_id`、`speech_id`；
- 旧任务或错误阶段 ID 会被忽略；
- LLM 请求中的候选字段名是 `candidates`；
- 配送导航只接收实物结果，不接收仿真目标车间；
- `/task/dependencies_ready` 是临时接口，后续由模块健康聚合器替代。

## 7. 参数位置和修改方法

统一配置文件：

```text
~/ucar_ws/src/task_orchestrator/config/orchestrator.yaml
```

默认值：

```yaml
timeouts:
  dependency_ready: 30.0
  pickup_navigation: 300.0
  qr_search: 90.0
  llm_classification: 60.0
  speech: 60.0
  delivery_navigation: 300.0
  cancel_ack: 15.0

voice_adapter:
  debounce_seconds: 2.0

tts_bridge:
  command:
    - python3
    - /home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py
  timeout: 30.0
```

参数说明：

| 参数 | 作用 | 调整建议 |
|---|---|---|
| `dependency_ready` | 等待依赖就绪 | 调试时可增大；正式运行应由健康检查尽快返回 |
| `pickup_navigation` | 导航到取货点 | 当前 300 秒，场地路线稳定后再缩短 |
| `qr_search` | 等待三个 QR 候选 | 当前 90 秒，应大于 QR 节点一轮搜索上限 |
| `llm_classification` | 等待双目标推理 | 网络不稳定时增大，不建议低于实际最慢响应 |
| `speech` | 等待播报完成 | 必须大于 TTS 下载和播放总时长 |
| `delivery_navigation` | 导航到实物车间 | 当前 300 秒，场地路线稳定后再缩短 |
| `cancel_ack` | 取消阶段预留 | 用于取消和下游停止确认 |
| `debounce_seconds` | 相同语音文本去重 | 误重复触发时增大，连续测试时可减小 |
| `tts_bridge.timeout` | TTS 子进程超时 | 应小于或等于状态机的 `speech` 超时 |

修改 YAML 后需要停止并重新启动本包才能加载新值：

```bash
# 在 roslaunch 终端按 Ctrl+C
roslaunch task_orchestrator task_orchestrator.launch
```

当前版本没有实现运行时动态重配置。不要通过启动第二套同名节点来应用新参数。

查看实际加载值：

```bash
rosparam get /task_orchestrator/timeouts
rosparam get /voice_task_adapter/voice_adapter
rosparam get /tts_bridge/tts_bridge
```

临时测试单个参数可通过分步 `rosrun` 的私有参数覆盖；正式比赛应把最终值写回
`orchestrator.yaml`，保证所有启动方式使用同一组参数。

## 8. 停止和重启

### 8.1 正常停止

在启动它的 `roslaunch` 终端按：

```text
Ctrl+C
```

然后检查：

```bash
rosnode list
```

不应再出现：

```text
/task_orchestrator
/voice_task_adapter
/tts_bridge
```

正常情况下不需要逐个执行 `rosnode kill`。

### 8.2 异常残留

先确认节点：

```bash
rosnode list
rosnode info /task_orchestrator
```

优先回到原 launch 终端按 `Ctrl+C`。只有原终端已丢失时，才执行：

```bash
rosnode kill /task_orchestrator
rosnode kill /voice_task_adapter
rosnode kill /tts_bridge
```

随后确认没有残留 `roslaunch task_orchestrator` 进程。不要杀死 `/rosout`、
底盘、雷达、相机或其他不属于本包的节点。

## 9. 调试方法

### 9.1 检查节点和 topic

```bash
rosnode list
rostopic list | sort
rosnode info /task_orchestrator
```

### 9.2 观察状态和关键输出

每个命令放在单独终端：

```bash
rostopic echo /task/status
rostopic echo /voice/task_request
rostopic echo /task/pickup_navigation_goal
rostopic echo /qr_item_search/start
rostopic echo /llm/classify/request
rostopic echo /voice/speak
rostopic echo /task/delivery_navigation_goal
```

只查看一条消息：

```bash
rostopic echo -n 1 /task/status
```

查看发布者和订阅者是否存在：

```bash
rostopic info /qr_item_search/result
rostopic info /llm/classify/result
rostopic info /voice/speak_done
```

### 9.3 常见故障

**语音后没有任务**

```bash
rostopic echo /question
rostopic echo /voice/task_request
```

确认语句中恰好有两个目标母类。只有一个母类或出现三个母类时会拒绝生成任务。

**停在 `CHECKING_DEPENDENCIES`**

当前仍需 `/task/dependencies_ready`。人工调试可按手册发送；正式比赛必须由健康
检查模块产生。

**停在 `WAITING_QR`**

检查 `/qr_item_search/start` 的 `search_id` 与结果一致，且完成结果包含三个顺序
连续、名称不重复的候选。

**停在 `WAITING_LLM`**

检查 LLM 是否订阅 `/llm/classify/request`，返回的 `request_id` 是否一致，以及
实物和仿真选择是否都来自三个 `candidates`。

**停在 `WAITING_SPEECH`**

检查：

```bash
rosnode list | grep tts_bridge
rostopic echo /voice/speak_done
python3 /home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py "测试播报"
```

人工测试最后一条命令会真实播放声音。

**出现同名节点或资源占用**

先运行 `rosnode list` 和 `rosnode info`。摄像头、底盘、串口和 TTS 等资源只能
由一个常驻节点持有。参数不一致时，第一版采用统一 YAML 并重启原 launch；后期
再考虑动态参数或由单一 bringup 注入参数。

**消息发出但状态不变化**

最常见原因是 `task_id` 或阶段 ID 过期。不要手写猜测 ID，应从上一阶段实际输出
复制。

## 10. 测试

Windows 本地纯 Python 测试：

```powershell
cd D:\program_sec\智能车\.worktrees\task_orchestrator
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v
```

小车端测试：

```bash
cd ~/ucar_ws
python3 -m unittest discover \
  -s src/task_orchestrator/test \
  -p "test_*.py" -v
```

这些单元测试不会访问网络、扬声器或真实 TTS。完整 ROS topic 手动模拟见：

```text
~/ucar_ws/src/task_orchestrator/test/manual_simulation.md
```

人工模拟时使用 `rostopic pub -1`，确保每条事件只发送一次。启用真实
`tts_bridge` 后，不得再人工发布同一阶段的 `/voice/speak_done`。

## 11. 后续模块接入

仍需完成：

1. 导航节点消费 `/task/pickup_navigation_goal` 并发布
   `/task/pickup_arrived`；
2. 已部署 QR 节点消费 `/qr_item_search/start`，返回三个候选；
3. `llm_spark` 消费双目标请求并返回实物和仿真两个选择；
4. 导航节点消费 `/task/delivery_navigation_goal`，仅将实物送往目标车间；
5. 用 `/system/module_status` 健康聚合器替换人工
   `/task/dependencies_ready`；
6. 最终由全车 bringup 一键启动常驻节点，并统一管理所有参数和退出顺序。
