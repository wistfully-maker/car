# ucar_avoid — 配送阶段避障停车适配器（联调版）

> 本包原为队员自主调度的避障找车间/停车模块。本次接入 `task_orchestrator` 总流程后，
> `racecar_control`（avoid.cpp）改为**命令式**执行：由编排器通过 `/task/delivery_navigation_goal`
> 驱动，完成 **lidar_loc → AMCL 定位切换 → 航点扫描 → OCR 匹配车间标牌 → 激光停车 →
> 播报"已将X放入Y" → 回报 `/task/delivery_arrived`** 的完整配送闭环。
> 扫描/停车核心逻辑（`scanAtWaypoint` / `laserParkingAndSpeak`）保持队员原实现不变，
> 可独立迭代，不影响已接入的协议层。

---

## 1. 节点与文件

| 组件 | 路径 | 说明 |
|---|---|---|
| `racecar_control`（C++） | `src/avoid.cpp` | 配送执行器：协议订阅/回报、AMCL 切换、航点扫描、激光停车、语音播报 |
| `vision_node`（Python） | `scripts/vision_node.py` | 标牌 OCR 视觉节点（/vision/*），扫描期间由 racecar_control 通过 `/vision/enable` 开关 |
| 配送 launch | `launch/amcl_delivery.launch` | 常驻启动 `vision_node` + `racecar_control`，参数见 §5 |

## 2. 联调后的启动命令

### 2.1 正式全流程（推荐，唯一入口）

在**小车（192.168.1.109，用户 ucar）**上执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
cd /home/ucar/ucar_ws
./src/task_orchestrator/scripts/start_competition.sh
```

`start_competition.sh` 先做安全 preflight（节点冲突、`/cmd_vel` 唯一 owner、设备占用、Spark
secret），通过后 `exec roslaunch task_orchestrator competition_full.launch`，其中配送组
`start_delivery`（默认 true）include 本包的 `amcl_delivery.launch`。

常用开关（均为 `name:=true|false`）：

| arg | 默认 | 说明 |
|---|---:|---|
| `start_fast_nav` | true | 前段导航（map_server + lidar_loc + move_base，`/cmd_vel/navigation`） |
| `start_delivery` | true | **本包配送组**（vision_node + racecar_control）；preflight 检查 `/vision_node`、`/racecar_control` 冲突 |
| `start_qr` / `start_llm` / `start_speech` / `start_orchestrator` | true | QR 搜索、LLM 分类、语音、编排器 |
| `start_velocity_arbiter` | true | 唯一 `/cmd_vel` 发布者（仲裁器） |

完整链路：语音指令 → `/voice/task_request` → `CHECKING_DEPENDENCIES` → 取货导航 →
QR 三物品 → LLM 双目标 → TTS 播报 → `NAVIGATING_TO_WORKSHOP` → **本包配送** →
`delivery_arrived` → `COMPLETE`（当前仿真阶段关闭，直接完成）。

### 2.2 配送阶段单独调试（不跑语音/QR/LLM）

前置条件：**定位必须是 `lidar_loc`（前段导航），禁止预先启动 AMCL**（本流程运行时才切换
AMCL；若 `/amcl` 已存在，`switchToAmcl()` 的 roslaunch 会因节点名冲突失败并回报 failed）。

```bash
# 终端 1：前段导航栈（lidar_loc + map_server + move_base，提供 map/odom/TF）
roslaunch ucar_fast_nav navigation_stack.launch cmd_vel_topic:=/cmd_vel/navigation

# 终端 2：配送执行器（vision_node + racecar_control）
roslaunch ucar_avoid amcl_delivery.launch

# 终端 3：观察回报与播报
rostopic echo /task/delivery_arrived
rostopic echo /voice/speak
```

手工注入配送目标（JSON 在 `data` 字段内，`task_id`/`goal_id` 自拟即可，只要前后一致）：

```bash
rostopic pub -1 /task/delivery_navigation_goal std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"debug-1\",\"goal_id\":\"goal-1\",\"target_workshop\":\"食品加工车间\",\"selected_item\":\"苹果\"}'"
```

执行过程观察：

```bash
# racecar_control 日志（终端 2）：[GOAL] -> [AMCL] -> [NAV] -> [SCAN] -> [SPEAK] -> [ARRIVAL]
# 定位切换验证：
rosnode list | grep -E "lidar_loc|amcl"        # 应只见 /amcl（lidar_loc 已被 kill）
rosrun tf tf_echo map odom                     # AMCL 的 map->odom 应存在且更新

# 播报验证（/voice/speak 由 tts_bridge 消费后播放）：
rostopic echo -n 1 /voice/speak                # data 含 "已将苹果放入食品加工车间"
```

中止/取消配送：

```bash
rostopic pub -1 /task/cancel std_msgs/String \
  "data: '{\"protocol_version\":1,\"task_id\":\"debug-1\",\"reason\":\"operator_stop\"}'"
```

### 2.3 手动模拟整条配送链路（不动车）

编排器与协议层可完全离线验证（含仿真阶段预留）：

```bash
# 本机（无需 ROS）：
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py"
# 车上（Linux 完整环境，164 项全过）：
cd ~/ucar_ws/src/task_orchestrator && python3 -m unittest discover -s test -p "test_*.py"
```

带 ROS 的手工协议模拟见 `task_orchestrator/test/manual_simulation.md`。

## 3. 协议 v1 接口（本包相关）

所有业务 topic 均为 `std_msgs/String`，JSON 在 `data` 字段（UTF-8）。

| 方向 | topic | 关键字段 |
|---|---|---|
| 输入 | `/task/delivery_navigation_goal` | `protocol_version`(1)、`task_id`、`goal_id`、`target_workshop`、`selected_item` |
| 输入 | `/task/cancel` | `protocol_version`、`task_id`、`reason`（防御性停止；编排器自身也会处理取消） |
| 输出 | `/task/delivery_arrived` | `protocol_version`、`task_id`、`goal_id`、`status`("arrived"/"failed")、`message` |
| 输出 | `/voice/speak` | `protocol_version`、`task_id`、`speech_id`（自生成 `avoid-<nsec>`）、`text` |
| 输出 | `/cmd_vel/navigation` | `geometry_msgs/Twist`，经 velocity_arbiter 放行（NAVIGATION 模式） |

示例消息（与编排器实测一致）：

```json
{"protocol_version":1,"task_id":"task-...","goal_id":"goal-...","target_workshop":"食品加工车间","selected_item":"苹果"}
{"protocol_version":1,"task_id":"task-...","goal_id":"goal-...","status":"arrived","message":""}
{"protocol_version":1,"task_id":"task-...","speech_id":"avoid-...","text":"已将苹果放入食品加工车间"}
```

## 4. 配送执行流程（racecar_control 内部）

```
/task/delivery_navigation_goal
  -> deliveryGoalCallback（JSON 解析，goal_received 防重入，后台线程执行）
  -> switchToAmcl():
       1. TF 读取 map->base_link（lidar_loc 存活期最后可靠位姿）
       2. rosnode kill /lidar_loc
       3. roslaunch ucar_nav launch/config/amcl/amcl_omni.launch（后台）
       4. 发布 /initialpose（播种记录位姿 + 协方差）
       5. 轮询 map->odom TF 出现（15s 超时，失败则回报 failed）
  -> 遍历 waypoints（默认 3 点）:
       sendGoal -> waitForGoalReached -> scanAtWaypoint（必须转满一整圈）
  -> 匹配 target_workshop -> laserParkingAndSpeak（激光停车 0.28m）
  -> speak("已将X放入Y") -> /voice/speak
  -> publishArrival("arrived"/"failed")
  -> finishDelivery（清空 task_id/goal_id，等待下一 goal）
```

定位切换细节：前段导航用 `lidar_loc`（jie_ware，发布 map→odom）；避障停车基于 AMCL
调通，故配送开始时切换。AMCL 复用前段 `map_server` 的 `/static_map` 服务；两张地图
`ucar_fast_nav/maps/002.yaml` 与 `ucar_nav/maps/map.yaml` 为同一张图（SHA256 已核验）。

## 5. 参数（`amcl_delivery.launch`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `~cmd_vel_topic` | `/cmd_vel/navigation` | 速度发布通道（velocity_arbiter NAVIGATION 源） |
| `~rotate_speed` | 0.2 | 扫描旋转角速度 rad/s |
| `~approach_distance` | 0.4 | 停车接近距离 m |
| `~goal_tolerance` | 0.08 | 停车容差 m |
| `~waypoints` | 3 个扫描点（见 launch） | 扫描航点列表（x/y/yaw/name），修改无需重编译 |

## 6. 编译

```bash
cd ~/ucar_ws && source /opt/ros/noetic/setup.bash
catkin_make --pkg ucar_avoid
```

依赖：jsoncpp（经 pkg-config 解析，头文件位于 `/usr/include/jsoncpp` 子目录）、
move_base_msgs、actionlib、tf、cv_bridge、image_transport、sensor_msgs。

## 7. 已知限制与注意事项

1. **停车稳定性**（`laserParkingAndSpeak` 的时机/距离）由队员继续迭代；本包协议层与
   扫描/停车逻辑已解耦，队员改动不影响状态机接入。
2. **仿真阶段未接入**：编排器 `simulation_phase_enabled` 默认 false，实物配送完成即
   `COMPLETE`；状态机（`SIM_DELIVERY`/`WAITING_SIM`）与 `/task/sim_trigger`、
   `/task/sim_complete` 已预留，启用后实体机器人播报"仿真任务已完成，已将X放入Y"。
3. **禁止 `/amcl` 与 `/lidar_loc` 并存**：单独调试配送时先起 lidar_loc 导航栈；
   `start_competition.sh` preflight 也会拒绝 `/amcl` 已在运行的情况。
4. `vision_node` 依赖 `usb_cam`（`/usb_cam/image_raw`）；总 launch 中相机由
   `competition_full.launch` 的 `start_camera` 组启动。
5. `switchToAmcl()` 通过 `system()` 调用 `rosnode kill` / `roslaunch`，要求运行环境
   已 `source` ROS setup；失败路径回报 `failed` 并复位，不会让状态机悬挂。
