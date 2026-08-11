# task_orchestrator 操作、部署与联调手册

> **第一部分曾于 2026-08-09 部署到 `ucar@192.168.1.109`；本文新增的第二部分修复目前只在
> `codex/stop-phase2-integration` 本地分支完成，尚未重新部署或实车验收。** 首次部署本版本必须
> 有人看护、保留急停，
> 按本文从静态检查、零速度检查再逐步放开运动。

## 1. 先明确边界和真实终点

编排器负责把语音任务、就绪检查、取货点导航、QR 三物品、双目标 LLM 和 TTS 回执按
protocol v1 串起来；它负责校验 `task_id` 与各阶段 identity、超时、取消、状态和底盘模式。
它不实现底盘驱动、定位、规划、二维码识别、语音识别、LLM 或 TTS 算法，也不应重复启动
现场已有的硬件 owner。

第一阶段（已验收）的真实业务终点是 **TTS 完成后的交接**。TTS 成功后，状态机只发布一次
`/task/delivery_navigation_goal` 并停留在 `DELIVERY_HANDED_OFF`；`/task/motion_mode`
此时严格为 `IDLE`。**第二阶段（stop 联调）**由 navigation handoff supervisor 受控切换到
stop 导航栈：实物车间识别、动态避障导航与停车后回传 `/task/delivery_arrived`，再导航到
仿真目标车间并停车后回传 `/task/simulation_arrived`，最终进入 `COMPLETE`。
`/task/delivery_arrived` 与 `/task/simulation_arrived` 只能由 stop 协议适配器在对应
停车确认后发布，任何人不得人工伪造来证明实车完成；重复终态消息只重发缓存结果，不重复运动。

自动链路是：

```text
/question
 -> voice_task_adapter -> /voice/task_request
 -> task_orchestrator: CHECKING_DEPENDENCIES
 -> readiness_gate -> /task/dependencies_ready
 -> /task/pickup_navigation_goal
 -> fast_nav_adapter -> /move_base -> /task/pickup_arrived
 -> /qr_item_search/start -> 三个物品 /qr_item_search/result
 -> /llm/classify/request -> 实物/仿真双目标 /llm/classify/result
 -> /voice/speak -> tts_bridge -> /voice/speak_done
 -> /task/delivery_navigation_goal（唯一一次，交接触发）
 -> navigation_handoff_supervisor：CANCELLING -> VERIFYING_STOP -> STOPPING_LEGACY
      -> STARTING_STOP -> WAITING_STOP_READY -> READY（每步有界重试，重试中零速度）
 -> /task/stop_mission_goal -> stop 栈（OCR 车间识别 + TEB 动态避障 + PCA 停车）
 -> /task/delivery_arrived -> WAITING_DELIVERY_SPEECH
 -> /voice/speak：“已将{实物}放入{实物车间}” -> 匹配 /voice/speak_done success
 -> /task/simulation_navigation_goal -> NAVIGATING_TO_SIM_WORKSHOP -> stop 栈 Phase 2
 -> /task/simulation_arrived -> WAITING_SIMULATION_SPEECH
 -> /voice/speak：“仿真任务已完成，已将{仿真物品}放入{仿真车间}”
 -> 匹配 /voice/speak_done success -> COMPLETE
```

第二阶段细节、参数、无运动人工注入与回滚见第 9 节和
[`stop/README_INTEGRATION.md`](../../stop/README_INTEGRATION.md)。

## 2. 三种入口不能混用

| 入口 | 启动内容与外部包 | 前置条件 | 成功标志 | 风险与用途 |
|---|---|---|---|---|
| `scripts/start_competition.sh` | 先做安全 preflight，再启动 `competition_full.launch`；默认包括 `ucar_fast_nav`、`speech_command`、`usb_cam`、`qr_item_search`、`llm_spark` 和本包六个节点 | ROS/workspace 可读，设备空闲，Spark secret 合规，不存在同名 live/stale 节点、`/amcl` 或错误 `/cmd_vel` owner | preflight 无 `ERROR`，根 `roslaunch` 常驻；节点、TF、action、状态 topic 均可观察 | **推荐且唯一正式入口**；检查失败必须排因，不能绕过 |
| `roslaunch task_orchestrator competition_full.launch` | 与总 launch 相同，但完全跳过 preflight | 操作者已独立核对所有 owner、设备、secret、`/amcl` 和 `/cmd_vel` | 所有 include 成功启动 | 仅诊断 launch 展开/缺包问题；不能作为安全启动方式 |
| `roslaunch task_orchestrator task_orchestrator.launch` | 默认只启动 `/task_orchestrator`、`/voice_task_adapter`、`/tts_bridge`；可按 arg 单启适配/门控/交接节点 | 所需上游 topic 已由人工或外部节点提供 | 目标节点存在并能收发业务 topic | 只用于业务编排调试；没有导航、相机、QR、LLM、语音和安全 preflight，**不能靠“小飞小飞”唤醒跑全流程** |

总 launch 是本轮节点和硬件 owner 的唯一根。节点设计为常驻，是为了避免每阶段重开串口、
相机、定位和 `move_base` 造成抢占及状态丢失；阶段结束只归零并释放模式，不 kill 节点。

## 3. 部署后第一次启动

### 3.1 登录、构建、权限

本次部署前的车端旧包已完整备份到：

```text
/home/ucar/ucar_backups/task_orchestrator.backup_20260809_183954
```

该目录是本轮部署前的恢复基线，不得被后续同步覆盖或删除。本文不给出覆盖式删除命令。

```bash
ssh ucar@192.168.1.109
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
source /home/ucar/ucar_ws/devel/setup.bash
rospack find task_orchestrator
chmod +x /home/ucar/ucar_ws/src/task_orchestrator/scripts/start_competition.sh
```

`rospack find` 应指向 `/home/ucar/ucar_ws/src/task_orchestrator`。每个新 SSH 终端都重新执行两条
`source`；不要依赖交互 shell 的历史环境。

### 3.2 安全创建 Spark secret

不要把真实 secret 写进 README、Git、shell history 或聊天。用不回显的输入创建单行 LF 文件：

```bash
install -d -m 700 ~/.config/ucar
umask 077
read -r -s -p 'Spark API password: ' SPARK_INPUT; printf '\n'
printf '%s\n' "$SPARK_INPUT" > ~/.config/ucar/spark_api_password
unset SPARK_INPUT
chmod 600 ~/.config/ucar/spark_api_password
stat -c '%U %a %n' ~/.config/ucar/spark_api_password
```

文件必须由当前用户拥有、恰好一条非空文本、不得是符号链接、不得含 NUL/CRLF；脚本读取它
为纯数据而不是 shell 代码。也可在受控终端预先 `export SPARK_API_PASSWORD=...`，但更易泄漏。

### 3.3 启动前人工清单

- 车放在与 `ucar_fast_nav/config/pickup_goal.yaml` 所用地图一致的已知起点，四周留出制动空间；
- 操作者手持底盘急停，确认机械急停有效；首轮架空驱动轮或把速度上限降到安全值；
- `/dev/ttyS0`、`/dev/ttyS4`、`/dev/video0`、`/dev/ttyS3` 均存在且权限可读写；
- 小车网络能访问 Spark/TTS 服务，SSH 不丢包；
- RViz/地图人工确认地图方向、激光与障碍位置大致一致，`map->odom->base_link->laser_frame` 连通；
- 当前定位是 `lidar_loc`，不是 AMCL；不得让 `/amcl` 和 `/lidar_loc` 并存；
- `rostopic info /cmd_vel` 不应已有未知 publisher；确认唯一 owner 策略后再启动。

### 3.4 默认安全一键命令

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
cd /home/ucar/ucar_ws
./src/task_orchestrator/scripts/start_competition.sh
```

清洗节点（仅在确认原 root 已退出、只有僵尸登记时使用）：

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
rosnode cleanup
```

`rosnode cleanup` 只清除 ROS Master 上已经死掉的僵尸登记，**不能关闭任何 live 节点**，
也不是停止比赛的方法；停止请见第 6 节（先 `/task/cancel`，再回根终端 `Ctrl+C`，机械断能确认）。

启动参数均为布尔值，格式 `name:=true|false`：

| arg | 默认 | `true` 时归属 |
|---|---:|---|
| `start_fast_nav` | true | include `ucar_fast_nav/pickup_navigation.launch` |
| `start_base` | true | 由 fast-nav include 启底盘 |
| `start_lidar` | true | 由 fast-nav include 启雷达 |
| `start_camera` | true | 唯一 `/usb_cam` |
| `start_fast_nav_adapter` | true | `/fast_nav_adapter` |
| `start_readiness_gate` | true | `/readiness_gate` |
| `start_speech` | true | `speech_command.launch` |
| `start_qr` | true | QR 两节点，速度 remap 到 `/cmd_vel/qr` |
| `start_llm` | true | `llm_spark.launch` |
| `start_orchestrator` | true | 编排器、voice adapter、TTS bridge |
| `start_velocity_arbiter` | true | 唯一 `/velocity_arbiter`，唯一发布 `/cmd_vel` |
| `start_navigation_handoff` | true | 唯一 `/navigation_handoff_supervisor`，两个导航栈的唯一生命周期 owner；`false` 时回退第一阶段直接启动 fast-nav 的行为 |
| `start_stop_stack` | true | 交接时由 supervisor 启动 stop 栈；`false` 表示 stop 栈由外部提供（supervisor 不持有其进程，只验证就绪） |

**外部 fastnav 已启动**（只允许确实由外部 root 管理且节点健康时）：

```bash
./src/task_orchestrator/scripts/start_competition.sh \
  start_fast_nav:=false start_base:=false start_lidar:=false
```

这仍会启动本包 fast-nav adapter/readiness；外部必须提供 `/map`、TF、`/scan`、`/odom`、
`/lidar_loc`、`/move_base` action 和正确 planner 参数。

**外部仲裁器模式**要求 ROS Master 已经运行且 `/cmd_vel` 恰好有一个已确认的外部仲裁器 owner：

```bash
rosnode list
rostopic info /cmd_vel
./src/task_orchestrator/scripts/start_competition.sh start_velocity_arbiter:=false
```

没有 master、零 owner 或多个 owner 都必须失败。外部仲裁器还必须等价消费
`/task/motion_mode`、`/cmd_vel/navigation`、`/cmd_vel/qr`，切换/超时/退出立即发零速度。

## 4. 唤醒后如何观察完整自动链路

所有下列业务 topic 的 ROS 类型都是 `std_msgs/String`，结构化内容放在消息的 `data` 字段内，
编码为 UTF-8 protocol v1 JSON：

| 方向 | topic | ROS 类型 | `data` 内 JSON 用途 |
|---|---|---|---|
| 输入 | `/question` | `std_msgs/String` | 语音识别原文；唯一不是 protocol v1 JSON 的业务输入 |
| 输入 | `/voice/task_request` | `std_msgs/String` | 两个目标母类、原文和 `task_id` |
| 输入 | `/task/dependencies_ready` | `std_msgs/String` | 只接受匹配任务的 `status: ready` |
| 输入 | `/task/pickup_arrived` | `std_msgs/String` | 取货导航 `goal_id` 的 arrived/failed |
| 输入 | `/qr_item_search/result` | `std_msgs/String` | QR `search_id`、`items` 和状态 |
| 输入 | `/llm/classify/result` | `std_msgs/String` | LLM `request_id` 与双目标选择 |
| 输入 | `/voice/speak_done` | `std_msgs/String` | TTS `speech_id` 的 success/error |
| 输入 | `/task/delivery_arrived` | `std_msgs/String` | 实物停车确认后由 stop 协议适配器回传（arrived/failed） |
| 输入 | `/task/simulation_arrived` | `std_msgs/String` | 仿真停车确认后由 stop 协议适配器回传（arrived/failed） |
| 输入 | `/task/cancel` | `std_msgs/String` | 当前任务取消原因 |
| 输出 | `/task/status` | `std_msgs/String` | 状态机状态、业务状态和消息 |
| 输出 | `/task/motion_mode` | `std_msgs/String` | `data` 是纯文本 `IDLE/NAVIGATION/QR_SEARCH/STOP_NAVIGATION`，不是 JSON |
| 输出 | `/task/pickup_navigation_goal` | `std_msgs/String` | 取货点导航 identity |
| 输出 | `/qr_item_search/start`、`/stop` | `std_msgs/String` | QR 搜索 identity 与控制 |
| 输出 | `/llm/classify/request` | `std_msgs/String` | 两个母类和三候选 |
| 输出 | `/voice/speak` | `std_msgs/String` | 待播文本和 speech identity |
| 输出 | `/task/delivery_navigation_goal` | `std_msgs/String` | 实物配送目标（交接触发，TTS 完成后唯一一次） |
| 输出 | `/task/simulation_navigation_goal` | `std_msgs/String` | 仿真车间目标（实物停车确认后发布） |
| 输出 | `/task/navigation_handoff_status` | `std_msgs/String` | 交接 supervisor 严格终态，只允许关联的 `ready/failed`；过程诊断只写 ROS 日志 |
| 输出 | `/task/stop_mission_goal` | `std_msgs/String` | 交接 READY 后放行给 stop 任务 |
| 输出 | `/stop/mission_event`、`/stop/mission_ack` | `std_msgs/String` | stop 栈私有集成事件与 Phase 2 放行（不对外承诺） |

先在多个终端观察：

```bash
rostopic echo /question
rostopic echo /voice/task_request
rostopic echo /task/status
rostopic echo /task/motion_mode
rostopic echo /task/pickup_navigation_goal
rostopic echo /qr_item_search/start
rostopic echo /qr_item_search/result
rostopic echo /llm/classify/request
rostopic echo /llm/classify/result
rostopic echo /voice/speak
rostopic echo /voice/speak_done
rostopic echo /task/delivery_navigation_goal
```

说“小飞小飞”并完整说出包含两个不同母类的赛事指令后，`/question` 是
`std_msgs/String` 原文；voice adapter 输出的 `/voice/task_request` 也是 `std_msgs/String`，
其 `data` 为 JSON，例如：

```json
{"protocol_version":1,"task_id":"task-...","physical_target_category":"食品","simulation_target_category":"日用品","raw_text":"..."}
```

状态依次应为 `CHECKING_DEPENDENCIES`、`NAVIGATING_TO_PICKUP`、`WAITING_QR`、
`WAITING_LLM`、`WAITING_SPEECH`、`DELIVERY_HANDED_OFF`、`NAVIGATING_TO_WORKSHOP`、
`WAITING_DELIVERY_SPEECH`、`NAVIGATING_TO_SIM_WORKSHOP`、
`WAITING_SIMULATION_SPEECH`、`COMPLETE`。readiness 回执示例：

```json
{"protocol_version":1,"task_id":"task-...","status":"ready"}
```

刚进入依赖检查时，`/task/status` 的 `data` 完整内容形如：

```json
{"protocol_version":1,"task_id":"task-...","state":"CHECKING_DEPENDENCIES","status":"accepted","message":""}
```

当前实际构造器**没有 `stamp` 字段**，ROS adapter 也不补时间戳；排障时不要等待一个不存在的
`stamp`。如果以后协议新增它，必须先改协议测试和消费者，再更新本文。

取货目标含 `task_id/goal_id`；fast-nav adapter 将参数中的固定取货点转换为 `/move_base`
`move_base_msgs/MoveBaseAction`，action 成功且 `/odom` 速度稳定后回：

```json
{"protocol_version":1,"task_id":"task-...","goal_id":"pickup-...","status":"arrived","message":""}
```

QR start 含 `task_id/search_id`；结果必须携带相同 identity，并恰好三个连续 order、唯一物品：

```json
{"protocol_version":1,"task_id":"task-...","search_id":"search-...","stamp":1.0,"status":"complete","items":[{"order":1,"item_name":"手机","url":"https://...","detected_yaw":0.0},{"order":2,"item_name":"毛巾","url":"https://...","detected_yaw":1.2},{"order":3,"item_name":"苹果","url":"https://...","detected_yaw":3.4}],"message":""}
```

QR result 的完整示例见上文，其中候选原始字段名是 `items`。编排器随后只抽取 order/name，
构造完整 LLM request：

```json
{"protocol_version":1,"task_id":"task-...","request_id":"llm-...","physical_target_category":"食品","simulation_target_category":"日用品","candidates":[{"order":1,"item_name":"手机"},{"order":2,"item_name":"毛巾"},{"order":3,"item_name":"苹果"}]}
```

该 request 实际字段是 `candidates`，**没有 `items`**。LLM 成功结果必须原样返回 identity，
完整形态为：

```json
{"protocol_version":1,"task_id":"task-...","request_id":"llm-...","status":"success","physical":{"selected_order":3,"selected_item":"苹果","category":"食品","workshop":"食品加工车间"},"simulation":{"selected_order":2,"selected_item":"毛巾","category":"日用品","workshop":"日用品加工车间"},"message":""}
```

`/voice/speak` 的完整 JSON 含 `task_id/speech_id/text`：

```json
{"protocol_version":1,"task_id":"task-...","speech_id":"speech-...","text":"取得苹果属于食品大类应放置在食品加工车间，仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间"}
```

done 示例：

```json
{"protocol_version":1,"task_id":"task-...","speech_id":"speech-...","status":"success","message":""}
```

首次比赛格式播报完成后会看到一次 delivery message
（`protocol_version/task_id/goal_id/target_workshop/selected_item` 五字段齐全），交接期间
`/task/status` 暂时停留在 `DELIVERY_HANDED_OFF`，同时必须确认：

```bash
rostopic echo -n 1 /task/motion_mode    # data: "IDLE"
rostopic echo -n 1 /cmd_vel             # 六个分量均为 0
```

默认第二部分已接入时，supervisor 会在安全交接后启动 stop 栈并继续两次导航与停车。第一次
`/task/delivery_arrived` 只触发实物停车播报；必须收到匹配的 `/voice/speak_done success` 才发布
`/task/simulation_navigation_goal`。第二次 `/task/simulation_arrived` 只触发最终播报；同样必须收到
匹配的成功回执才进入 `COMPLETE`。关闭 `start_navigation_handoff` 回滚到第一部分时，流程才停在
`DELIVERY_HANDED_OFF`。任何到达或语音回执都不得人工伪造来证明实车完成。

identity 必须从上一阶段实际输出复制，不能猜。旧 `task_id`、错 `goal_id/search_id/request_id/speech_id`
会被忽略；这属于防串任务机制，不是节点“没反应”。

## 5. 参数来源与冲突策略

| 文件/命名空间 | 当前关键值 | 修改方式 |
|---|---|---|
| `task_orchestrator/config/orchestrator.yaml` `timeouts` | dependency 120、pickup 300、QR 120、LLM 120、speech 60、delivery 300、cancel 15 秒 | 节点启动时读取；改 YAML 后重启整个 root |
| 同文件 `fast_nav_adapter` | action 300、settle 0.5、线速度停止阈值 0.03、角速度停止阈值 0.05 | 启动读取，重启 |
| 同文件 `readiness_gate` | 消息最大龄 3、检查周期 1、TF/action 探测 0.05、日志 10 秒；frame 与节点/planner 路径见文件 | 启动读取，重启 |
| 同文件 `velocity_arbiter` | source timeout 0.3、周期 0.05、线速度绝对上限 1.0、角速度绝对上限 2.0 | 启动读取，重启 |
| 同文件 `navigation_handoff` | cancel 重试 3/3 秒、近零时长 0.75、线/角停止阈值 0.02/0.05、odom 最大龄 0.5、旧栈退出重试 5/10 秒、就绪重试 30/1 秒、总时限 90 秒 | 启动读取，重启；调参一次只改一个变量 |
| 同文件 `navigation_handoff_supervisor` | 两个导航 launch 路径、旧栈节点名、初始位姿、AMCL/OCR 节点名、就绪新鲜度 3 秒 | 总 launch 显式 arg 覆盖；`start_stop_stack:=false` 时 stop launch 传空 |
| `ucar_fast_nav/config/pickup_goal.yaml` | `/ucar_fast_nav/pickup_goal` 航点、frame/yaw、地图路径/校验契约 | fast-nav launch 统一加载，禁止本包重复加载 |
| `ucar_fast_nav/config` planner/map | `move_base` planner 参数、地图文件及定位配置 | 属于外部任务 5 契约；改后重启 fast-nav root |
| `qr_item_search/launch/qr_item_search.launch` | image/解码/步进/角速度/驻留/超时等全部由总 launch 的 `qr_*` 参数显式转发；HTTP 1/2 秒、1 重试/3 workers 为内部值 | 调 `qr_*` 参数后重启根 launch（见 5.1） |
| `llm_spark/launch/llm_spark.launch` | `url`/`request_timeout` 由总 launch 的 `llm_url`/`llm_request_timeout` 转发 | 单包调试可直接给该 launch 传 arg；正式流程用全局参数 |
| `speech_command/launch/speech_command.launch` | 无本总 launch 可传 arg；输出 `/question` | 改外部包配置后重启 root |
| `tts_bridge` | `python3 .../tts_http.py`，timeout 30 秒 | YAML，重启 |

`rosparam set` 能改变参数服务器上的值，但大多数节点只在构造时读取，不能据此宣称已经动态生效。
需要不同运行参数时，优先使用明确支持的控制 topic 或 launch arg；必须重载时在根终端 `Ctrl+C`，
确认退出后整套重启。参数统一在启动时加载，不允许为套用另一组参数而重复启动同名节点。

### 5.1 全局 launch 可调参数

`competition_full.launch` 统一暴露比赛现场要调的参数（带 `qr_` 前缀的参数原样转发给
`qr_item_search.launch`，`llm_`/`timeout_` 参数分别转发给 LLM 与编排器）：

| 全局参数 | 默认值 | 下游映射 |
|---|---:|---|
| `qr_image_topic` | `/usb_cam/image_raw` | `image_topic` |
| `qr_start_debug_stream` | `false` | `start_debug_stream` |
| `qr_debug_host` | `0.0.0.0` | `debug_host` |
| `qr_debug_port` | `8080` | `debug_port` |
| `qr_metrics_dir` | `$(env HOME)/qr_metrics` | `metrics_dir` |
| `qr_keyframe_dir` | `$(env HOME)/qr_keyframes` | `keyframe_dir` |
| `qr_decode_scale` | `1.5` | `decode_scale` |
| `qr_step_angle_deg` | `45.0` | `step_angle_deg` |
| `qr_cruise_angular_speed` | `0.50` | `cruise_angular_speed` |
| `qr_approach_angular_speed` | `0.20` | `approach_angular_speed` |
| `qr_approach_zone_deg` | `10.0` | `approach_zone_deg` |
| `qr_yaw_tolerance_deg` | `2.0` | `yaw_tolerance_deg` |
| `qr_settled_angular_speed` | `0.03` | `settled_angular_speed` |
| `qr_settled_duration` | `0.20` | `settled_duration` |
| `qr_scan_window` | `1.0` | `scan_window` |
| `qr_offset_angle_deg` | `22.5` | `offset_angle_deg` |
| `qr_max_passes` | `2` | `max_passes` |
| `qr_search_total_timeout` | `90.0` | `search_total_timeout` |
| `qr_settling_timeout` | `3.0` | `settling_timeout` |
| `qr_heading_timeout` | `1.0` | `heading_timeout` |
| `qr_camera_timeout` | `1.0` | `camera_timeout` |
| `llm_url` | 讯飞 Spark URL（见 `llm_spark.launch`） | `url` |
| `llm_request_timeout` | `90.0` | `request_timeout` |
| `timeout_dependency_ready` | `120.0` | 编排 `timeouts/dependency_ready` |
| `timeout_pickup_navigation` | `300.0` | 编排 `timeouts/pickup_navigation` |
| `timeout_qr_search` | `150.0` | 编排 `timeouts/qr_search` |
| `timeout_llm_classification` | `120.0` | 编排 `timeouts/llm_classification` |
| `timeout_speech` | `60.0` | 编排 `timeouts/speech` |
| `legacy_nav_launch` | `$(find task_orchestrator)/launch/legacy_navigation_include.launch` | supervisor 持有的第一部分导航栈（默认不含硬件） |
| `stop_integration_launch` | `$(find stop)/launch/mission_integration.launch` | supervisor 持有的 stop 导航栈（不含公共硬件） |

固定连接不允许通过比赛命令改写：QR 速度 remap 到 `/cmd_vel/qr`、导航 remap 到
`/cmd_vel/navigation`、stop 栈内部经 `/cmd_vel/stop`（stop mux）隔离、最终唯一
`/cmd_vel` 由 velocity_arbiter 发布、QR 图像默认 `/usb_cam/image_raw`。

覆盖优先级：**总 launch 显式值 > `orchestrator.yaml` 默认值 > Python 内建兜底**。
例如把 QR 搜索编排超时调成 180 秒：

```bash
./src/task_orchestrator/scripts/start_competition.sh timeout_qr_search:=180.0
```

调参必须一次只改一个变量并重启根 launch（`Ctrl+C` 后重新执行）。QR 示例：

- 二维码间距更小时改用 30° 步进：`qr_step_angle_deg:=30.0`，同时通常配套
  `qr_offset_angle_deg:=15.0` 并适当增大 `qr_search_total_timeout`（如 120.0）；
- 扫码驻留过短导致漏码：把 `qr_scan_window` 从 1.0 逐步加大到 1.2~1.5；
- 旋转过快：减小 `qr_cruise_angular_speed`（0.50 → 0.40），一次只调一个量；
- 总搜索时间不够：增大 `qr_search_total_timeout`，同时确认
  `timeout_qr_search`（编排超时）仍严格大于它；默认 150 > 90，禁止设成相等或更小。

`start_*:=true|false` 仍是唯一的布尔开关；`qr_*`、`llm_*`、`timeout_*` 等调参参数按原文、
逐参数边界安全地透传给根 roslaunch，不做布尔校验，也不做 shell 展开。

## 6. 停止、急停与常驻原则

机械急停或切断底盘驱动电源是危险情况下唯一首选；软件 topic、终端和网络都可能失效，
人员不得靠发布 ROS 消息接近仍可能运动的车辆。

正常软件停止可向活动任务发布取消，再在根 `start_competition.sh`/`roslaunch` 终端按
`Ctrl+C`；根进程会回收自己启动的节点：

```bash
rostopic pub -1 /task/cancel std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"<当前实际 TASK_ID>\",\"reason\":\"operator_stop\"}'"
# 随后回到根 launch 终端按 Ctrl+C，并机械确认底盘已经断能/不会运动
```

`/task/cancel` 只能请求业务状态机取消，**不能替代机械急停**，也不能证明底盘已停止。
业务阶段结束不 kill 节点，只切换 `IDLE`、发布零速度并释放模式。安全脚本不会自动 kill 任何
现场节点，也不会擅自清除 stale registration。

下面的零 Twist 仅允许在底盘已经机械断能或驱动轮可靠架空后，用于诊断/确认 topic 路径：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

它会成为第二个 `/cmd_vel` publisher；ROS 多 publisher 的到达顺序不可作为安全机制，
**不能保证安全**，不得作为任何停止或安全手段，不得靠它接近车辆。诊断完立即停止该 publisher，
再检查唯一 owner。机械断能前不要执行这条命令。

异常残留先检查，不自动 kill：

```bash
rosnode list
rosnode ping /velocity_arbiter
rosnode info /velocity_arbiter
rostopic info /cmd_vel
```

只有确认原 root 已丢失、节点确属本次启动且硬件 owner 关系清楚时，才人工执行例如：

```bash
rosnode kill /velocity_arbiter
rosnode kill /task_orchestrator
```

不要照抄 kill 整组硬件节点；先找回 owner，优先停止整个原 root。

## 7. 常见故障：命令、判断和动作

| 现象 | 检查命令 | 判断与处理 |
|---|---|---|
| `live node conflict` / `stale registration` | `rosnode list; rosnode ping /节点; rosnode info /节点` | ping 通是 live owner；不通是 stale。找原 root，勿直接重复启动 |
| device absent/busy/probe permission | `ls -l /dev/ttyS0 /dev/ttyS4 /dev/video0 /dev/ttyS3; fuser /dev/video0` | absent 查接线/udev；busy 查 PID owner；probe permission 查用户组和 `fuser/lsof`，不要跳过检查 |
| `/amcl` 冲突 | `rosnode ping /amcl; rosnode ping /lidar_loc` | 本流程只允许 `lidar_loc`；停止 AMCL 所属 root 后重启 |
| `/cmd_vel` 始终 0 | `rostopic echo /task/motion_mode; rostopic echo /cmd_vel/navigation; rostopic echo /cmd_vel/qr` | `IDLE`、错误源或 0.3 秒源超时都会归零；不要为“让车动”绕过仲裁 |
| 多个 `/cmd_vel` owner | `rostopic info /cmd_vel` | 必须恰好一个已知仲裁器；定位并停止多余 root |
| readiness 报 `map missing` | `rostopic echo -n 1 /map; rostopic info /map` | map 只检查是否至少收到过（通常为 latched），不做新鲜度判断；无消息才是 missing，检查 map_server 与 latch |
| scan/odom missing 或 stale | `rostopic hz /scan; rostopic hz /odom; rostopic echo -n 1 /scan; rostopic echo -n 1 /odom` | `message_max_age: 3.0` 只用于 scan/odom；无消息是 missing，时间戳超过窗口或在未来是 stale/异常 |
| TF/action/planner | `rosrun tf tf_echo map base_link; rosrun tf tf_echo map odom; rosrun tf tf_echo odom base_link; rosparam get /move_base/base_global_planner; rosparam get /move_base/base_local_planner; rostopic info /move_base/status` | 任一缺失都不能放行导航；核对 fast-nav 外部契约 |
| secret 权限/CRLF | `stat -c '%u %a' ~/.config/ucar/spark_api_password; file ~/.config/ucar/spark_api_password` | owner 不对、组/其他可读、CRLF、多行均重建；不要打印内容 |
| QR 看不到/阳光干扰 | `rostopic hz /usb_cam/image_raw; rostopic echo /qr_item_search/result` | 检查相机唯一 owner、曝光、焦距、码大小/反光/直射阳光；不要用加大非零转速掩盖视觉问题 |
| LLM 无结果 | `rostopic info /llm/classify/request; rostopic echo /llm/classify/result` | 检查网络、secret、90 秒 HTTP timeout 与 120 秒编排 timeout、identity 和候选一致性 |
| TTS 无完成 | `rostopic info /voice/speak; rostopic echo /voice/speak_done; rosnode ping /tts_bridge` | 30 秒子进程 timeout 必须先于 60 秒阶段 timeout；先看 bridge 日志，不手工补成功回执 |
| 状态 timeout | `rostopic echo /task/status` | 看 `state/message` 定位 dependency/nav/QR/LLM/speech；修依赖后用新任务重跑 |
| launch 报外部包缺失 | `rospack find ucar_fast_nav; rospack find speech_command; rospack find qr_item_search; rospack find llm_spark; rospack find usb_cam` | 这是故意 fail-fast；补齐同一 catkin workspace 并重新 source，不删 include |

## 8. 测试与联调入口

不动车的分层 topic 模拟、identity JSON、motion/arbiter 安全限制见
[`test/manual_simulation.md`](test/manual_simulation.md)。本地测试：

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

车端运行单测不代表实车验收。第一阶段现场证据仍按任务 8 检查点执行；第二阶段（stop 联调）
的车端验收按 `HANDOFF_TO_CODEX.md` 中的检查点逐个单独授权，先无运动启动、旧栈安全退出、
stop 栈只启动不发目标、AMCL/TF 就绪，再单独实物、单独仿真目标，最后才连续两阶段。

## 9. 第二阶段：stop 导航栈交接

### 9.1 单一安全入口与回滚

第二阶段仍只从 `scripts/start_competition.sh` 启动（默认 `start_navigation_handoff:=true`、
`start_stop_stack:=true`）。preflight 新增：交接 supervisor 节点冲突检查、stop 包存在性与
三个 OCR 模型相对 `VEHICLE_SNAPSHOT.sha256` 的字节校验、交接模式下的底盘/雷达设备检查。

回滚到第一阶段基线：

```bash
./src/task_orchestrator/scripts/start_competition.sh start_navigation_handoff:=false
```

此时总 launch 回到"直接启动 fast-nav、不启动任何 stop 栈节点"的第一阶段行为；
`start_stop_stack:=false` 只用于 stop 栈确由外部 root 提供的部署（supervisor 不持有其进程，
就绪验证失败仍 fail closed）。

### 9.2 交接顺序与有界重试

`/navigation_handoff_supervisor` 是导航栈生命周期唯一 owner，只管理它自己启动并持有的两个
roslaunch 进程组（第一部分 fast-nav include 与 `stop/mission_integration.launch`），
绝不按节点名模糊 kill，也不用 `pkill ros` / `rosnode kill -a`。顺序：

```text
IDLE -> CANCELLING（取消遗留 action goal，cancel_retries 次）
 -> VERIFYING_STOP（新鲜 /odom 连续近零 stop_stable_duration 秒）
 -> STOPPING_LEGACY（停止旧进程组，legacy_exit_retries 次缺席确认）
 -> STARTING_STOP（启动 stop 栈 + supervisor 发布一次 latched 二维码领取区初始位姿）
 -> WAITING_STOP_READY（readiness_retries 次探测 map/AMCL/TF/action/OCR/相机/雷达）
 -> READY -> 发布 /task/stop_mission_goal 放行任务
```

每个状态切换先发零速度再发生命周期动作；取消、退出、就绪任一耗尽重试或超过
`total_timeout` 都进入 `FAILED`（关联的最终失败发布在 `/task/navigation_handoff_status`，
运动模式保持 `IDLE`），等待人工处理或重新启动任务，绝不带着不确定定位继续运动。
重试次数、等待条件和底盘仍运动等过程诊断只写 ROS 日志，不混入该业务 topic。

### 9.3 速度所有权

```text
/cmd_vel/navigation -----\
/cmd_vel/qr --------------> competition velocity arbiter -> /cmd_vel
/cmd_vel/stop ------------/

/cmd_vel/stop_navigation --\
/cmd_vel/stop_manual -------> stop velocity mux -> /cmd_vel/stop
```

- 只有 `competition_velocity_arbiter` 发布最终 `/cmd_vel`；
- 交接 READY 放行时 supervisor 把运动模式切到 `STOP_NAVIGATION`（只转发 `/cmd_vel/stop`）；
- stop 栈内部由 stop mux 在 NAVIGATION（move_base 输出）与 MANUAL（PCA/停车/倒车）之间
  选择，模式切换、未知模式、输入陈旧、非法数值、时间回退、异常和 shutdown 一律先发零速度；
- 每次模式切换必须先发零速度；新模式对应来源的新鲜消息到达前不允许运动。

### 9.4 两个独立到达契约

- 第一次停车（实物车间）确认后才发布 `/task/delivery_arrived`（goal_id = delivery goal 的
  goal_id），编排器播报“已将{物品}放入{车间}”；匹配的播报成功回执前不发布第二阶段目标；
- 第二次停车（仿真目标车间）稳定确认后才发布 `/task/simulation_arrived`（goal_id =
  simulation goal 的 goal_id），编排器播报“仿真任务已完成，已将{物品}放入{车间}”；匹配的
  播报成功回执前不进入 `COMPLETE`；
- stop 任务启动后不自动运动，等 `/task/stop_mission_goal`；Phase 1 停车后停在原地等待
  全局编排器的 `/task/simulation_navigation_goal` 放行（`/stop/mission_ack`，超时
  `phase2_ack_timeout` 秒报失败），放行前不自动进入 Phase 2 运动；
- 重复终态消息只重发缓存结果，不重复运动；过期、错误 phase、错误身份一律忽略。

### 9.5 参数与调参（一次只改一个变量）

| 参数 | 默认 | 单位/说明 |
|---|---:|---|
| `navigation_handoff/cancel_retries` | 3 | 次，action 取消重试 |
| `navigation_handoff/cancel_timeout` | 3.0 | 秒 |
| `navigation_handoff/stop_stable_duration` | 0.75 | 秒，近零持续时长 |
| `navigation_handoff/linear_stop_threshold` | 0.02 | m/s |
| `navigation_handoff/angular_stop_threshold` | 0.05 | rad/s |
| `navigation_handoff/odom_max_age` | 0.5 | 秒 |
| `navigation_handoff/legacy_exit_retries` | 5 | 次 |
| `navigation_handoff/legacy_exit_timeout` | 10.0 | 秒 |
| `navigation_handoff/readiness_retries` | 30 | 次 |
| `navigation_handoff/readiness_poll_period` | 1.0 | 秒 |
| `navigation_handoff/total_timeout` | 90.0 | 秒，交接总时限 |
| `stop/mission_integration` `max_rotations` | 6 | 次，OCR 旋转上限 |
| `stop/mission_integration` `target_distance` | 0.20 | m，PCA 停车目标距离 |
| `stop/mission_integration` `x_align_tolerance` | 0.50 | m |
| `stop/mission_integration` `y_align_tolerance` | 12 | 度 |
| `stop/mission_integration` `phase2_ack_timeout` | 60.0 | 秒 |
| `navigation_handoff/initial_pose_x/y/yaw` | -1.40219/-0.627908/0.053792653589793 | 二维码领取区权威观察点，由 supervisor latched 发布一次 |
| `stop/mission_integration` `initial_pose_x/y/yaw` | 0.0/0.0/0.0 | 集成模式关闭 stop 内部第二个 `/initialpose` 发布者 |

调参必须一次只改一个变量并重启根 launch；stop 栈参数默认值与车端已验证
`mission.launch` 完全一致，未经验证不得批量改动。

三个车间航点保持车端实跑代码的固定坐标，不在本次接口修复中调整：

```text
航点1 (-0.812845, -2.44196)
航点2 ( 0.771455, -2.44196)
航点3 ( 1.784300, -2.43094)
```

`current_point_index` 表示当前正在导航/扫描的航点：到达后不递增，只有该点 OCR 旋转扫描耗尽、
准备前往下一点时才递增一次。这样 Phase 1 识别到的仿真车间索引不会偏移到下一航点。

### 9.6 停止、残留诊断与禁止事项

停止仍按第 6 节：先 `/task/cancel`（带当前真实 `task_id`），再回根终端 `Ctrl+C`；
supervisor 的 shutdown 会停止它持有的两个导航进程组并发布零速度。`rosnode cleanup`
只清除僵尸登记，**不是 live 节点停止机制**。残留诊断：

```bash
rosnode list | grep -E 'move_base|amcl|lidar_loc|map_server|navigation_handoff'
rostopic info /cmd_vel          # 唯一 owner 必须是 /velocity_arbiter
rostopic info /cmd_vel/stop     # 唯一 owner 必须是 /stop_velocity_mux
rostopic echo -n 1 /task/navigation_handoff_status
```

禁止：同时启动 lidar_loc 与 AMCL、同时启动两个 move_base/map server/相机/雷达/底盘
driver；让 stop、move_base 或其他节点直接发布最终 `/cmd_vel`；伪造
`/task/delivery_arrived`、`/task/simulation_arrived`；用 `rosnode cleanup` 停 live 节点。

## 10. 第三阶段：红绿灯识别与巡线联调

在第二阶段仿真车间播报成功后，状态机不再直接完成，而是进入第三阶段：

```text
WAITING_SIMULATION_SPEECH
 -> NAVIGATING_LINE_START      发布 /task/line_navigation_goal（YAML 巡线起点）
 -> WAITING_LINE_DIRECTION     等待红绿灯/方向识别（red_light 保持等待）
 -> LINE_FOLLOWING             按 left_turn/right_turn/straight 巡线
 -> WAITING_FINAL_SPEECH       “任务完成”TTS
 -> COMPLETE
```

### 10.1 公共接口

```text
/task/line_navigation_goal    发布：带 pose 的导航目标（frame_id/x/y/qz/qw）
/task/line_navigation_arrived 订阅：arrived|failed（失败带 message）
/task/line_follow/start       发布：开始方向识别
/task/line_follow/status      订阅：waiting_signal|direction_selected|following|success|failure
/task/motion_mode             LINE_FOLLOW 期间只放行 /cmd_vel/line_follow
/cmd_vel/line_follow          巡线速度隔离源，只有 phase3 supervisor 发布
```

巡线状态带方向：`direction_selected`/`following` 携带 `direction`；`failure` 携带
`reason`。30 秒无方向结果时选择 `straight`。只有匹配 `task_id/goal_id` 的新鲜成功
才推进状态机；重复与过期事件只重发缓存状态。

### 10.2 配置与数据流

第三阶段巡线起点、相机参数与超时来自
`line_follow_integration/config/phase3.yaml`（小车部署路径
`/home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml`），由根 launch
通过 `phase3_config` 参数转发；**修改 YAML 必须重启根 launch**，不修改 Python。
物理相机保持 1020x720 唯一配置；YOLO 使用原始图像；巡线使用派生的 640x480 图像
（`/line_follow/image_raw`）。根 launch 通过 `start_line_follow` 开关包含
`line_follow_integration/launch/phase3.launch`，**不得同时运行原始
`start_all_yolo.launch`**。

### 10.3 速度所有权

```text
巡线脚本 /cmd_vel --remap--> /line_follow/cmd_vel_candidate
  --supervisor 图像健康门控--> /cmd_vel/line_follow
  --velocity_arbiter mode=LINE_FOLLOW--> /cmd_vel
```

图像超过 0.5 秒未更新立即零速度并阻断候选速度，3 秒未恢复则失败；导航到巡线起点仍
使用 `STOP_NAVIGATION` 模式与 stop 导航速度链。

### 10.4 故障与子进程

supervisor 只终止自己创建的 YOLO/巡线子进程（按 PID），禁止 `pkill`/`killall`/
`rosnode kill`。模型缺失或哈希不一致、原始图像 5 秒未就绪、巡线子进程提前退出、
120 秒巡线超时都失败；只有最终停车线的新鲜标记才是成功。失败、取消与关闭先零速度
再结束子进程，最后发布一次关联失败。TTS 失败或超时进入 ERROR，不进入 COMPLETE。

### 10.5 分模块调试与验收

```bash
# 先禁动（机械断能/架空驱动轮），再逐模块启动
roslaunch line_follow_integration phase3.launch enable_line_follow_supervisor:=false enable_line_navigation_adapter:=false
rostopic hz /usb_cam/image_raw       # 原始图像
rostopic hz /line_follow/image_raw   # 派生 640x480 图像
rostopic echo /task/line_follow/status
rostopic echo /task/motion_mode
rostopic echo /cmd_vel/line_follow
```

实车看护验收逐层放行：导航起点与停车姿态 → IDLE 下验证两路图像 → 红灯等待与方向锁定
→ 单独放行 `/cmd_vel/line_follow` → 最终停车线与单次“任务完成”播报 → 全流程。
