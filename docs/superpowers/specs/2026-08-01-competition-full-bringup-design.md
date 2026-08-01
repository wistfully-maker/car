# U-CAR 全流程分层启动与资源所有权设计

## 1. 目标

为子任务 1 提供可复现的一键启动入口，连接语音、领取区导航、QR、LLM、TTS 和
`task_orchestrator`，同时避免底盘串口、雷达串口、摄像头、声卡、ROS 同名节点及
`/cmd_vel` 被多个模块重复占用。

设计必须允许导航、避障和巡线继续保留各自的独立调试 launch，也要允许今后纳入比赛
总流程，不得要求每个功能包重复启动公共硬件。

## 2. 当前能力边界

- `speech_command` 可通过“小飞小飞”硬件唤醒，完整指令只发布一次 `/question`。
- `ucar_waypoint_nav` 已能消费 `/task/pickup_navigation_goal` 并返回
  `/task/pickup_arrived`。
- 导航仍存在偶发 AMCL 偏移，只能标记为“可联调，需人工确认定位对正”。
- 导航尚未消费 `/task/delivery_navigation_goal`，也尚未发布
  `/task/delivery_arrived`；三个车间坐标尚未标定。
- QR 只订阅共享相机图像，不应自行启动第二个相机节点。
- 编排器的配送阶段接口已存在，但在配送导航完成前，真实流程会在
  `NAVIGATING_TO_WORKSHOP` 等待或超时，不得宣称全任务已闭环。

## 3. 方案选择

采用“公共基础层 + 常驻业务层 + 任务模式层 + 安全启动入口”。

不直接并列 include 导航、避障和巡线的完整独立 launch。这种做法会重复启动
`base_driver`、雷达、相机、TF 或 `move_base`，并且可能让多个节点同时发布
`/cmd_vel`。

## 4. 分层架构

```text
start_competition.sh                 安全一键入口：先检查，后启动
  -> competition_full.launch         只负责组合和统一参数
      -> robot_base_bringup.launch   底盘、雷达、相机、公共 TF
      -> navigation_stack.launch     地图、AMCL、move_base
      -> waypoint manager            编排器导航 topic 适配
      -> speech_command              唤醒与 ASR
      -> qr_item_search              QR 识别与旋转搜索
      -> llm_spark                   双目标分类
      -> task_orchestrator           状态编排、语音适配、TTS 桥
      -> future modules              避障、巡线等可选常驻逻辑
```

### 4.1 公共硬件层

每种物理资源必须只有一个所有者：

| 资源 | 唯一所有者 | 其他模块的用法 |
|---|---|---|
| 底盘串口 | `base_driver` | 发布经仲裁的速度 topic |
| 雷达串口 | 雷达驱动 | 订阅 `/scan` |
| USB 相机 | 单一 `usb_cam` | QR、巡线共享订阅图像 topic |
| 语音串口/麦克风 | `speech_command` | 通过 `/question` 取得文本 |
| 地图与定位 | 单一 `map_server` + `amcl` | 共享 TF 和定位结果 |
| 路径规划 | 单一 `move_base` | 通过 action 发送目标 |

公共基础层默认由总 launch 启动。如果现场已有外部 bringup，必须显式传入
`start_robot_base:=false` 和/或 `start_navigation_stack:=false`，不允许隐式自动重用。

### 4.2 常驻业务层

语音、QR、LLM、TTS、导航适配器和编排器都只启动一次并保持运行。任务阶段通过
topic/action 激活，不在状态切换时启动或关闭节点。

每个子 launch 都应支持显式开关，例如：

```text
start_speech
start_qr
start_llm
start_orchestrator
start_waypoint_manager
start_robot_base
start_navigation_stack
start_camera
```

单包调试 launch 可以保留“自己启动依赖”的模式；被总 launch include 时，所有公共
资源开关必须关闭。

### 4.3 任务模式层

导航、避障、巡线可以常驻，但不能同时控制底盘。正式接入避障和巡线前，必须引入
速度仲裁层（例如 `twist_mux` 或等价的自有仲裁节点）：

```text
/cmd_vel/navigation --\
/cmd_vel/avoidance  ----> velocity arbiter ---> /cmd_vel ---> base_driver
/cmd_vel/line       --/
```

编排器只发布“当前任务模式”或阶段命令，不直接实现速度混合。仲裁层必须在节点
异常、模式超时或无所有者时输出零速度。

## 5. 启动入口

### 5.1 组合 launch

`competition_full.launch` 是声明式组合文件，提供每层的启停参数，并把相机 topic、
导航 profile、地图、航点配置等参数统一向下传递。

它不尝试在 XML 中“自动猜测”已有节点。ROS launch 的 include 会并发启动，无法保证
先检查后创建其他节点，因此资源检查不应伪装成普通 launch 节点。

### 5.2 安全一键脚本

`start_competition.sh` 是比赛推荐入口，执行顺序固定为：

1. 加载 ROS 和工作空间环境；
2. 检查 ROS Master 可达；
3. 检查要启动的同名节点是否已存在；
4. 对每个已存节点执行 `rosnode ping`，区分真正运行和 ROS Master 僵尸登记；
5. 检查底盘、雷达、相机、语音串口等已知设备的占用情况；
6. 检查 `/cmd_vel` 已有发布者；
7. 如果发现未声明的外部所有者，打印冲突资源、节点和处理方法后退出；
8. 检查通过后 `exec roslaunch task_orchestrator competition_full.launch ...`。

脚本不会自动 `rosnode kill`、结束未知进程或释放其他团队的硬件；冲突必须由操作者
明确处理。

## 6. 参数与组合模式

总 launch 至少提供：

| 参数 | 默认 | 含义 |
|---|---:|---|
| `start_robot_base` | `true` | 启动底盘与雷达公共层 |
| `start_camera` | `true` | 启动唯一 USB 相机 |
| `start_navigation_stack` | `true` | 启动地图、AMCL、`move_base` |
| `start_waypoint_manager` | `true` | 启动领取区导航适配 |
| `start_speech` | `true` | 启动唤醒与 ASR |
| `start_qr` | `true` | 启动 QR 扫描和旋转控制 |
| `start_llm` | `true` | 启动 Spark 分类节点 |
| `start_orchestrator` | `true` | 启动编排器、语音适配和 TTS 桥 |
| `navigation_profile` | 当前实测配置 | 统一选择规划器配置 |

实际参数名在实施时与已有子 launch 对齐，但不改变“必须显式声明资源所有权”
的规则。

## 7. 运行时流程

```text
全部常驻节点启动
 -> 人工确认 AMCL 激光与地图对正
 -> 健康聚合器或临时人工门控发布 dependencies_ready
 -> “小飞小飞”唤醒并发布完整 /question
 -> 编排器发布领取区目标
 -> waypoint manager 调用 move_base
 -> 到达后启动 QR
 -> QR 返回三个物品
 -> LLM 分类实物和仿真目标
 -> TTS 按比赛格式播报
 -> 编排器发布配送目标
 -> 当前版本等待配送导航接入
```

AMCL 定位确认在当前阶段是必须的人工安全门，不因“一键启动”而取消。

## 8. 异常处理

- 同名节点正常存活：默认拒绝重复启动，提示使用对应 `start_*:=false`。
- ROS 僵尸登记：报告节点名和不可达 URI，不自动删除或杀进程。
- 串口/相机占用：报告设备路径和持有进程，终止启动。
- `/cmd_vel` 多发布者：在尚未引入仲裁层时视为不安全，拒绝全流程启动。
- LLM 密钥缺失：启动前检查私密文件/环境变量，日志不打印密钥内容。
- AMCL 未对正：不发任务；人工重新初始化并确认后再继续。
- 子节点运行期退出：编排器依赖检查或超时进入 `ERROR`，不自动重启硬件节点。

## 9. README 交付要求

`task_orchestrator/README.md` 必须从操作者角度详细说明：

1. 编排器本身做什么，不做什么；
2. 每个 launch 启动的具体节点；
3. 单独 `task_orchestrator.launch` 为什么不能直接唤醒并跑全流程；
4. 安全一键启动、外部导航已启动时的命令、完全分步启动命令；
5. 小飞小飞唤醒后的逐阶段 topic 流转；
6. 当前能自动运行到哪里，哪一步仍需手动或尚未实现；
7. 正常 `Ctrl+C` 停止与异常残留排查；
8. 参数文件位置、临时覆盖方式、修改后的重启边界；
9. 同名节点、串口、相机、`/cmd_vel`、密钥和 ROS 僵尸登记的排障方法；
10. 后续避障、巡线和配送导航的标准接入步骤。

## 10. 测试与验收

### 10.1 离线

- XML 可被 ROS launch 解析；
- 所有 `start_*` 开关能独立启用/禁用对应层；
- 同一公共节点在总 launch 中只出现一次；
- 安全脚本在发现模拟同名节点或资源占用时以非零码退出；
- 安全脚本不包含自动杀节点或释放未知资源的命令；
- 现有语音 9 项、编排器 47 项和 LLM 11 项回归保持通过。

### 10.2 小车分层验证

1. 只启动公共硬件层，确认底盘、雷达、相机各只有一个所有者；
2. 只启动导航层，人工确认 AMCL 对正；
3. 启动业务层，不发任务，检查节点和 topic 唯一性；
4. 单独发布领取区导航目标，验证 identity 原样返回且只返回一次；
5. 人工模拟导航到达，完成语音到 TTS 和配送目标发布的全链路；
6. 实车执行到领取区、QR、LLM、TTS，保持可立即急停；
7. 完成配送导航后，再验证到车间的最终 `COMPLETE`。

## 11. 本次实施范围

本次实现：

- 分层总 launch；
- 启动前安全检查脚本；
- 对已有导航、语音、QR、LLM和编排器的参数传递；
- 极其详细的 README；
- 对避障、巡线和配送导航的扩展接口与文档。

本次不实现：

- AMCL 偏移根因修复；
- 三个车间坐标标定与配送导航逻辑；
- 避障和巡线算法；
- 未有符合现有模块接口时强行接入速度仲裁；
- 自动结束其他团队的节点或进程。
