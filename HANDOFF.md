# U-CAR 子任务 1 开发交接

更新时间：2026-07-24

## 1. 工作区与小车

- 本地 worktree：
  `D:\program_sec\智能车\.worktrees\task_orchestrator`
- 分支：`feature/task-orchestrator`
- 小车 SSH：`ssh ucar@172.20.10.4`
- 小车工作空间：`/home/ucar/ucar_ws`
- ROS：Noetic
- 小车 Python：3.7.3

非交互 SSH 中执行 ROS 命令前必须显式加载：

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
```

## 2. 当前模块流转

```text
/question
 -> voice_task_adapter
 -> /voice/task_request
 -> task_orchestrator
 -> /task/pickup_navigation_goal
 -> /task/pickup_arrived
 -> /qr_item_search/start
 -> /qr_item_search/result
 -> /llm/classify/request
 -> /llm/classify/result
 -> /voice/speak
 -> /voice/speak_done
 -> /task/delivery_navigation_goal
 -> /task/delivery_arrived
```

导航尚未正式接入，两次到达消息目前可人工模拟。

## 3. 已完成验证

### 编排器

- 本地和小车单元测试：47/47 通过；
- 手动成功路径、失败路径、过期 identity、取消、真实 TTS、正常停止均通过；
- QR 阶段超时：120 秒；
- LLM 阶段超时：120 秒；
- 两个导航阶段超时：各 300 秒。

详细命令见：

```text
ucar_ws/src/task_orchestrator/README.md
ucar_ws/src/task_orchestrator/test/manual_simulation.md
```

### QR

真实场地联调通过。一次完整结果：

- 手机：order 1；
- 毛巾：order 2；
- 苹果：order 3；
- 约 17 秒完成三个二维码；
- 当前 launch 快速角速度：0.26 rad/s；
- QR 自身总超时：100 秒。

### LLM

原节点的单次双目标长 prompt 会出现：

- 30 秒读取超时；
- Spark 网关 504；
- 模型 `selected_order` 与 `selected_item` 不一致。

现已改为两个并行短请求，每个请求只选择一个 `selected_item`，本地根据可信候选
和母类映射补齐 order、category、workshop，再发布一个原子的 protocol v1 双目标
结果。真实 Spark 测试约 9 秒通过。

Spark 凭据不进入 Git。小车私密文件：

```text
/home/ucar/.config/ucar/spark_api_password
```

权限必须为 600。启动前：

```bash
export SPARK_API_PASSWORD="$(cat ~/.config/ucar/spark_api_password)"
roslaunch llm_spark llm_spark.launch
```

### 真实主链路

人工向 `/question` 发布完整文字后，以下真实链路已经通过：

```text
voice adapter -> QR -> Spark LLM -> TTS -> delivery goal -> COMPLETE
```

测试目标：

- 实物母类：食品；
- 仿真母类：日用品；
- LLM 选择：苹果和毛巾；
- 配送目标只包含：苹果、食品加工车间。

## 4. speech_command 重复唤醒与完整指令（已修复）

现象：

1. 第一次唤醒后可以识别并发布 `/question`；
2. 匹配命令后调用 `gStop()`；
3. 后续硬件唤醒能输出角度，但不再产生 ASR 文本。

根因：

- `gStop()` 发送 `AIUIConstant::CMD_STOP`；
- SDK 注释明确要求 stop 后必须发送 `CMD_START` 才能继续；
- 原 `gWakeup()` 只发送 `CMD_WAKEUP`。

修复源文件保存在：

```text
patches/speech_command/AIUITester.cpp
patches/speech_command/competition_command_gate.h
```

部署目标：

```text
/home/ucar/ucar_ws/src/speech_command/src/AIUITester.cpp
/home/ucar/ucar_ws/src/speech_command/src/competition_command_gate.h
```

修复策略：

- `std::atomic<bool> aiui_stopped` 记录 stop 状态；
- `gStop()` 成功发送后标记 stopped；
- 下一次 `gWakeup()` 先且仅先发送一次 `CMD_START`，再发送 `CMD_WAKEUP`；
- 将内部接收缓冲区改为静态存储，使已进入解析器的串口分片可以跨回调保留；
- 串口数据块开头不是 `A5 01` 时，在块内重新寻找帧头，而不是丢弃整个数据块；
- 本地源码集成回归测试：9/9 通过；
- 独立 C++ 比赛指令门控测试通过；
- 小车 `catkin_make --pkg speech_command` 编译通过；
- 2026-07-24 完成三轮真实连续测试，三轮均完整识别、只发布一次并能再次唤醒；
- 三轮中未再出现 CRC 或 SYNC 失步。

### 原厂 QA 提前打断（已修复）

原 IAT 回调会对每个非空识别结果立即调用 `FindDocument` 和 `QA_list_`。即使只识别
到“请前往领取区”或第一个母类，也会发送原厂串口回复并调用 `gStop()`，导致小车
复述中间文本并覆盖用户后半句。

现通过 `CompetitionCommandGate` 门控：

- 云端 IAT 识别到“领取区”或“仿真环境”后进入比赛任务候选；单独出现母类不会
  启动候选，避免环境对话污染任务；
- 未同时获得领取区、仿真环境、两个不同母类和两次“对应仓库”时继续监听；
- 完整后只发布一次 `/question` 并停止；
- 比赛版本的整个云端 IAT 路径不再调用原厂 QA、不发送串口复述，也不自动调用
  原厂 TTS；普通非比赛文本只记录并忽略；
- 下游正式播报仍由 `task_orchestrator -> /voice/speak -> tts_bridge ->
  tts_http.py` 完成，不受删除 QA 分支影响；
- “小飞小飞”属于硬件唤醒链路，保持不变；
- 只有从 stopped 开始新会话时才清空候选，重复唤醒不会清除已积累文本。
- 自动剥离识别文本前部可能附带的“飞”“小飞”“小飞小飞”；
- 如果 ASR 首次识别错误，用户重新说出包含“领取区”和母类的新任务时，新任务
  会替换旧候选，不会与旧错误片段累计成三个母类。

三轮实机结果：

1. 实物食品，仿真日用品；
2. 实物日用品，仿真电子产品；
3. 实物电子产品，仿真食品。

三条真实识别文本重放给 `voice_task_adapter` 后，三组
`physical_target_category/simulation_target_category` 均与上述顺序一致。

最终部署版本在节点意外退出并重新启动后，再次实测“实物日用品、仿真电子产品”，
完整文本只发布一次 `/question`，随后 AIUI 正常停止录音。测试期间曾出现 ROS
Master 仍登记 `/speech_command_node`、但实际进程已退出的僵尸注册；判断方式是
`rosnode list` 能看到节点，但 `rosnode ping /speech_command_node` 或
`rosnode info /speech_command_node` 无法通信。遇到此情况应重新启动语音 launch，
而不是修改门控规则。

仍保留为后续鲁棒性事项：

- 原厂读取循环仍可能忽略长度不超过 12 字节的串口读取；
- `_serial.available()` 尚未对 1024 字节缓冲区做限幅；
- STOP/WAKEUP 极端并发时仍可进一步串行化；
- 唤醒提示音偶发缺失。

## 5. 导航现状

导航最后处理。最近一次检查发现外部启动：

```text
roslaunch ucar_nav ucar_navigation.launch
```

相关节点包括 AMCL、move_base、雷达、相机、底盘和 RViz。该 launch 不是编排器
联调启动的，不得在未确认所有者的情况下停止。

后续需要实现或适配：

- 消费 `/task/pickup_navigation_goal`；
- 到达观察点后发布带原 `task_id/goal_id` 的 `/task/pickup_arrived`；
- 消费 `/task/delivery_navigation_goal`；
- 到达实物车间后发布带原 identity 的 `/task/delivery_arrived`；
- 失败时返回 `status: failed` 和非空 `message`；
- 导航节点必须常驻，不得因每个任务重复启动底盘、雷达、相机或 move_base。

## 6. 常用验证

```powershell
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v

python -m unittest discover `
  -s ucar_ws/src/llm_spark/test `
  -p "test_*.py" -v

python -m unittest discover `
  -s patches/speech_command `
  -p "test_*.py" -v
```

小车构建：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make
source devel/setup.bash
```

## 7. 最近提交

- `2add375`：双目标 Spark protocol v1；
- `480d5a7`：并行短请求和宽松超时；
- speech_command 重复唤醒和串口重同步修复已完成实机验证。
