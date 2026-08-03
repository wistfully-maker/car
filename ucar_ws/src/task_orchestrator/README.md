# task_orchestrator 操作、部署与联调手册

> **当前状态：已于 2026-08-03 部署到 `ucar@172.20.10.3`，完成 Catkin 编译、Bash
> 语法检查和 153 项车端离线测试。** 本文不表示完整实车流程已经通过；本次没有启动业务
> 节点，也没有进行实车运动。首次上车必须有人看护、保留急停，
> 按本文从静态检查、零速度检查再逐步放开运动。

## 1. 先明确边界和真实终点

编排器负责把语音任务、就绪检查、取货点导航、QR 三物品、双目标 LLM 和 TTS 回执按
protocol v1 串起来；它负责校验 `task_id` 与各阶段 identity、超时、取消、状态和底盘模式。
它不实现底盘驱动、定位、规划、二维码识别、语音识别、LLM 或 TTS 算法，也不应重复启动
现场已有的硬件 owner。

当前自动流程的真实业务终点是 **TTS 完成**。TTS 成功后，状态机会保留兼容接口并发布
`/task/delivery_navigation_goal`，业务状态可显示 `NAVIGATING_TO_WORKSHOP`；但是
`/task/motion_mode` 此时严格为 `IDLE`，总 launch 没有配送适配器。因此这个 delivery goal
只是消息，不授予任何运动权限，**不启动二维码后的动态避障、巡线或车间导航**，更不能把
“发布了目标”误判为“车已经配送”。不要人工伪造 `/task/delivery_arrived` 来证明实车完成。

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
 -> /task/delivery_navigation_goal（兼容消息）
 -> /task/motion_mode = IDLE（真实安全终点）
```

## 2. 三种入口不能混用

| 入口 | 启动内容与外部包 | 前置条件 | 成功标志 | 风险与用途 |
|---|---|---|---|---|
| `scripts/start_competition.sh` | 先做安全 preflight，再启动 `competition_full.launch`；默认包括 `ucar_fast_nav`、`speech_command`、`usb_cam`、`qr_item_search`、`llm_spark` 和本包六个节点 | ROS/workspace 可读，设备空闲，Spark secret 合规，不存在同名 live/stale 节点、`/amcl` 或错误 `/cmd_vel` owner | preflight 无 `ERROR`，根 `roslaunch` 常驻；节点、TF、action、状态 topic 均可观察 | **推荐且唯一正式入口**；检查失败必须排因，不能绕过 |
| `roslaunch task_orchestrator competition_full.launch` | 与总 launch 相同，但完全跳过 preflight | 操作者已独立核对所有 owner、设备、secret、`/amcl` 和 `/cmd_vel` | 所有 include 成功启动 | 仅诊断 launch 展开/缺包问题；不能作为安全启动方式 |
| `roslaunch task_orchestrator task_orchestrator.launch` | 默认只启动 `/task_orchestrator`、`/voice_task_adapter`、`/tts_bridge`；可按 arg 单启三个适配/门控节点 | 所需上游 topic 已由人工或外部节点提供 | 目标节点存在并能收发业务 topic | 只用于业务编排调试；没有导航、相机、QR、LLM、语音和安全 preflight，**不能靠“小飞小飞”唤醒跑全流程** |

总 launch 是本轮节点和硬件 owner 的唯一根。节点设计为常驻，是为了避免每阶段重开串口、
相机、定位和 `move_base` 造成抢占及状态丢失；阶段结束只归零并释放模式，不 kill 节点。

## 3. 部署后第一次启动

### 3.1 登录、构建、权限

车端旧包已于 2026-08-03 09:29 完整备份到：

```text
/home/ucar/ucar_backups/task_orchestrator.backup_20260803_0929
```

该目录是本轮部署前的恢复基线，不得被后续同步覆盖或删除。本文不给出覆盖式删除命令。

```bash
ssh ucar@172.20.10.3
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
catkin_make
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
- `/dev/ucar_controller`、`/dev/ttyS4`、`/dev/video0`、`/dev/ttyS3` 均存在且权限可读写；
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
| 输入 | `/task/delivery_arrived` | `std_msgs/String` | 保留的配送回执接口，当前无自动发布者 |
| 输入 | `/task/cancel` | `std_msgs/String` | 当前任务取消原因 |
| 输出 | `/task/status` | `std_msgs/String` | 状态机状态、业务状态和消息 |
| 输出 | `/task/motion_mode` | `std_msgs/String` | `data` 是纯文本 `IDLE/NAVIGATION/QR_SEARCH`，不是 JSON |
| 输出 | `/task/pickup_navigation_goal` | `std_msgs/String` | 取货点导航 identity |
| 输出 | `/qr_item_search/start`、`/stop` | `std_msgs/String` | QR 搜索 identity 与控制 |
| 输出 | `/llm/classify/request` | `std_msgs/String` | 两个母类和三候选 |
| 输出 | `/voice/speak` | `std_msgs/String` | 待播文本和 speech identity |
| 输出 | `/task/delivery_navigation_goal` | `std_msgs/String` | 保留的实物配送消息，不授运动权 |

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
`WAITING_LLM`、`WAITING_SPEECH`、`NAVIGATING_TO_WORKSHOP`。readiness 回执示例：

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

最后会看到 delivery message（含 `task_id/goal_id/target_workshop/selected_item`），同时必须确认：

```bash
rostopic echo -n 1 /task/motion_mode    # data: "IDLE"
rostopic echo -n 1 /cmd_vel             # 六个分量均为 0
```

identity 必须从上一阶段实际输出复制，不能猜。旧 `task_id`、错 `goal_id/search_id/request_id/speech_id`
会被忽略；这属于防串任务机制，不是节点“没反应”。

## 5. 参数来源与冲突策略

| 文件/命名空间 | 当前关键值 | 修改方式 |
|---|---|---|
| `task_orchestrator/config/orchestrator.yaml` `timeouts` | dependency 120、pickup 300、QR 120、LLM 120、speech 60、delivery 300、cancel 15 秒 | 节点启动时读取；改 YAML 后重启整个 root |
| 同文件 `fast_nav_adapter` | action 300、settle 0.5、线速度停止阈值 0.03、角速度停止阈值 0.05 | 启动读取，重启 |
| 同文件 `readiness_gate` | 消息最大龄 3、检查周期 1、TF/action 探测 0.05、日志 10 秒；frame 与节点/planner 路径见文件 | 启动读取，重启 |
| 同文件 `velocity_arbiter` | source timeout 0.3、周期 0.05、线速度绝对上限 1.0、角速度绝对上限 2.0 | 启动读取，重启 |
| `ucar_fast_nav/config/pickup_goal.yaml` | `/ucar_fast_nav/pickup_goal` 航点、frame/yaw、地图路径/校验契约 | fast-nav launch 统一加载，禁止本包重复加载 |
| `ucar_fast_nav/config` planner/map | `move_base` planner 参数、地图文件及定位配置 | 属于外部任务 5 契约；改后重启 fast-nav root |
| `qr_item_search/launch/qr_item_search.launch` | image `/usb_cam/image_raw`；HTTP 1/2 秒、1 重试/3 workers；角速 0.40/0.20、最小 0.11、总搜索 40 秒等 | launch `<param>`，需重启 QR 所属 root |
| `llm_spark/launch/llm_spark.launch` | URL arg；`request_timeout:=90.0` | 仅单包调试可用该 launch arg，节点启动后固定 |
| `speech_command/launch/speech_command.launch` | 无本总 launch 可传 arg；输出 `/question` | 改外部包配置后重启 root |
| `tts_bridge` | `python3 .../tts_http.py`，timeout 30 秒 | YAML，重启 |

`rosparam set` 能改变参数服务器上的值，但大多数节点只在构造时读取，不能据此宣称已经动态生效。
需要不同运行参数时，优先使用明确支持的控制 topic 或 launch arg；必须重载时在根终端 `Ctrl+C`，
确认退出后整套重启。参数统一在启动时加载，不允许为套用另一组参数而重复启动同名节点。

`request_timeout:=90.0` 只在直接运行
`roslaunch llm_spark llm_spark.launch request_timeout:=90.0` 时有效；当前
`competition_full.launch` 未转发这个 arg，所以不可把它直接追加到 `start_competition.sh` 尾部。
正式总流程若要改该值，必须先给 competition launch 增加显式 arg/转发并配套测试，然后
`Ctrl+C` 根 launch 后重启；不能假设未知尾部参数会穿透 include。

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
| device absent/busy/probe permission | `ls -l /dev/ucar_controller /dev/ttyS4 /dev/video0 /dev/ttyS3; fuser /dev/video0` | absent 查接线/udev；busy 查 PID owner；probe permission 查用户组和 `fuser/lsof`，不要跳过检查 |
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

车端运行单测不代表实车验收。任务 8 仍需完成部署、构建、静态 owner 检查、架空轮/低速、
取货导航、QR、LLM、TTS 和 TTS 后 `IDLE` 的现场证据；不验收二维码后的动态避障、巡线或配送。
