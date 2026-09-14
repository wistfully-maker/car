# U-CAR `ucar_fast_nav` 分层启动与编排接入设计

## 1. 目标和本轮终点

将已实车跑通的 `ucar_fast_nav` 接入 `task_orchestrator`，实现：

```text
小飞小飞唤醒
 -> 语音完整指令
 -> 自动检查导航链健康
 -> 导航到二维码观察点
 -> QR 旋转识别三个物品
 -> LLM 选择实物和仿真物品
 -> TTS 按比赛格式播报
```

本轮真实自动验收终点是 **TTS 播报完成**。编排器可保留已有后续目标消息，
但底盘控制模式必须保持 `IDLE`，不启动任何未确定的后续运动。

## 2. 明确不在本轮的内容

- `ucar_waypoint_nav` 不再进入正式总 launch，只保留历史调试。
- 不使用 AMCL，不启动 `/amcl`。
- vendor bundle 中的 `dynamic_obstacle` 只是归档依赖，不启动、不适配、不验收。
- 不实现二维码区之后的航点、避障、巡线或目标区域移动。
- 不修改或覆盖队员的 `/home/ucar/ucar_ws/src/ucar_nav`。
- 不把 `ucar_fast_nav` 的节点启停逻辑写进编排状态机。

## 3. 导航链的唯一实现

正式导航链固定为：

```text
base_driver + ydlidar + TF
 -> jie_ware/lidar_loc
 -> map_server (002.yaml)
 -> GlobalPlanner
 -> TebLocalPlannerROS
 -> /cmd_vel/navigation
```

`ucar_fast_nav/pickup_navigation.launch` 是导航层唯一入口。总 launch 向它传入：

```text
start_base
start_lidar
cmd_vel_topic:=/cmd_vel/navigation
```

导航 launch 不启动相机。相机在总 launch 中单独启动一次，QR 只订阅
`/usb_cam/image_raw`。

## 4. 编排器与导航的隔离层

新增常驻 `fast_nav_adapter` ，编排器不直接依赖 `move_base` 或航点 YAML。

### 4.1 输入和输出

适配器订阅：

```text
/task/pickup_navigation_goal     std_msgs/String protocol v1 JSON
/task/cancel                     std_msgs/String protocol v1 JSON
/odom                            nav_msgs/Odometry
```

适配器调用：

```text
/move_base                       move_base_msgs/MoveBaseAction
```

适器发布：

```text
/task/pickup_arrived             std_msgs/String protocol v1 JSON
```

### 4.2 航点来源

适配器从参数 `/ucar_fast_nav/pickup_goal` 读取：

```text
frame_id, map_sha256, x, y, yaw, position_tolerance, yaw_tolerance
```

不在 `task_orchestrator` 中复制坐标。部署时仍以
`/home/ucar/ucar_ws/src/ucar_fast_nav/config/pickup_goal.yaml` 为唯一航点来源。

### 4.3 到达判定和过期防护

- 保留原始 `task_id` 和 `goal_id`。
- 新目标来临时取消旧 action goal，旧结果不得推进新任务。
- 只有 action 返回 `SUCCEEDED` 才进入停稳判定。
- `/odom` 线速度和角速度连续在可配阈值内达到 `settle_time` 后，才发布
  `status: arrived`。
- action `ABORTED/REJECTED/PREEMPTED/LOST`、超时或取消均发布带非空 `message` 的
  `status: failed`。
- 发布终态后清理当前 identity 并回到 `IDLE`，不关闭 `move_base`。

## 5. 自动导航健康门控

新增常驻 `system_readiness_gate`，使“小飞小飞”后不需手工发布
`/task/dependencies_ready`。

### 5.1 触发时机

门控订阅 `/task/status`。只有看到匹配 `task_id` 进入
`CHECKING_DEPENDENCIES` 后才开始检查，避免 ready 比编排器建立任务更早而被忽略。

### 5.2 必须同时成立的条件

- `/scan`、`/odom`、`/map` 在可配时间窗口内有新消息。
- TF `map -> odom -> base_link -> laser_frame` 完整。
- `/move_base` action server 可用。
- `/lidar_loc` 存活且可 ping。
- `/amcl` 不存在；若存在则拒绝 ready，防止两个 `map -> odom` 发布者。
- 实际规划器参数分别为 `global_planner/GlobalPlanner` 和
  `teb_local_planner/TebLocalPlannerROS`。

全部通过后发布：

```json
{
  "protocol_version": 1,
  "task_id": "来自 /task/status 的当前任务",
  "status": "ready"
}
```

未通过时不发车，定期日志列出尚未满足的条件。编排器的
`dependency_ready` 超时保持可配，本轮默认放宽到 120 秒。

## 6. 底盘速度唯一所有权

当前仅实现两个输入：

```text
/cmd_vel/navigation --\
                      -> velocity_arbiter -> /cmd_vel -> base_driver
/cmd_vel/qr         --/
```

不创建 `/cmd_vel/avoidance`、`/cmd_vel/line` 或动态避障接口。

编排器发布 latched `/task/motion_mode`：

| 编排状态 | 运动模式 |
|---|---|
| `NAVIGATING_TO_PICKUP` | `NAVIGATION` |
| `WAITING_QR` | `QR_SEARCH` |
| 其他所有状态 | `IDLE` |

模式切换时仲裁器立即发布零速度；只转发当前模式对应的输入；活动输入超过
`source_timeout` 无新消息时回落到零速度。QR 完成后运动模式回到 `IDLE`，即使
编排器继续发布后续业务消息，底盘也不会运动。

## 7. 分层启动

`competition_full.launch` 默认启动：

```text
ucar_fast_nav/pickup_navigation.launch
usb_cam
fast_nav_adapter
system_readiness_gate
speech_command
qr_item_search
llm_spark
task_orchestrator + voice_task_adapter + tts_bridge
velocity_arbiter
```

总 launch 提供显式开关：

```text
start_fast_nav, start_base, start_lidar, start_camera,
start_fast_nav_adapter, start_readiness_gate, start_speech,
start_qr, start_llm, start_orchestrator, start_velocity_arbiter
```

如外部已启动完整 `ucar_fast_nav` 链，传 `start_fast_nav:=false`；仅底盘或雷达由外部所有
时，保持 `start_fast_nav:=true` 并分别传 `start_base:=false` 或 `start_lidar:=false`。

单独 `task_orchestrator.launch` 仍只是编排层调试入口，不启动语音、导航、相机、QR 或 LLM。

## 8. 安全一键入口

`start_competition.sh` 在 roslaunch 前：

1. 加载 ROS 和工作空间；
2. 若 ROS Master 已运行，检查同名节点及僵尸登记；干净开机时由 roslaunch 启动 Master；
3. 按 `start_*` 所有权检查底盘串口、`/dev/ttyS4` 雷达和相机占用；
4. 禁止 `/amcl` 与将启动的 `/lidar_loc` 并存；
5. 禁止未经仲裁的外部 `/cmd_vel` 发布者；
6. 从私密文件加载 Spark 密钥，不打印内容；
7. 检查通过后 `exec roslaunch ... competition_full.launch`。

脚本不自动 `rosnode kill`、`pkill`、`killall` 或释放未知资源。

## 9. 节点生命周期和停止

- 底盘、雷达、`lidar_loc`、地图、`move_base`、相机和业务节点整轮常驻。
- 导航到达后 `fast_nav_adapter` 清理 action identity 并回到 `IDLE`，不关闭导航栈。
- QR 完成或取消时发零速度、停止识别窗口并释放 `QR_SEARCH`。
- LLM 和 TTS 完成请求后回到等待状态。
- 只有整车退出、人工 `Ctrl+C`、必须重载参数或节点异常时才停止进程。
- `Ctrl+C` 由总 launch 统一回收它启动的节点，不单独杀其他团队的进程。

## 10. 已知风险

- `lidar_loc` 已证明比原 AMCL 稳定，但健康检查不等于几何定位绝对正确。
- 当前 GlobalPlanner + TEB 能到观察点，但仍会多次蹭墙，实车联调必须有人看护并保留急停。
- 健康门控只验证数据、TF、节点和 action 链路，不会判断路径是否会蹭墙。
- `pickup_goal.yaml` 中地图 SHA-256 应与 `002.pgm` 匹配；不匹配时适配器拒绝发目标。

## 11. 验收

### 11.1 离线

- 导航协议、identity、新旧 goal、action 终态和停稳判定有纯 Python 测试。
- 健康快照的每一失败条件都有单元测试。
- 运动模式只包含 `IDLE/NAVIGATION/QR_SEARCH`。
- 仲裁器不转发非活动源，切换和超时都发零速度。
- 总 launch 不包含 `ucar_waypoint_nav`、`amcl` 或 `dynamic_obstacle`。
- 安全脚本不自动结束未知节点。
- 现有语音、QR、LLM 和编排器回归保持通过。

### 11.2 小车

1. 只启动全部常驻节点，不发任务，确认没有同名节点和设备重复占用。
2. 运行导航 runtime check，确认 `/scan`、`/odom`、TF、`lidar_loc` 和规划器。
3. 只模拟任务状态，确认模式切换时底盘保持零速度。
4. 单独发领取区目标，验证 action、停稳和唯一到达回报。
5. 在人工看护下从“小飞小飞”运行到 TTS 播报完成。
6. TTS 后确认 `/task/motion_mode` 为 `IDLE`，没有未知后续运动。
