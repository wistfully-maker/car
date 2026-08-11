# 第三阶段红绿灯识别与巡线联调设计

## 1. 目标与基线

在已经实车跑通的二维码、两阶段动态避障导航、车间识别与两次停车流程之后，增加第三阶段：

1. 第二阶段仿真车间播报成功；
2. 导航至巡线起点；
3. 停车等待红绿灯/方向识别；
4. 根据 `left_turn`、`right_turn` 或 `straight` 执行对应巡线程序；
5. 检测最终停车线并停车；
6. 由 `task_orchestrator` 单独播报“任务完成”；
7. 匹配的播报成功事件使全局状态进入 `COMPLETE`。

开发基线为 `codex/stop-phase2-integration` 的提交 `7b61069`。该提交已将小车实际使用的第三个车间扫描航点同步为：

```python
(2.2843, -2.43094, 0.0, 1.0)
```

第三阶段开发分支与工作树为：

```text
branch: codex/phase3-line-follow-integration
worktree: D:\program_sec\智能车\.worktrees\phase3-line-follow-integration
```

第三个车间扫描航点与第三阶段巡线起点是两个不同目标，不得混用。

## 2. 第三阶段巡线起点

巡线起点使用 `map` 坐标系：

```yaml
x: 0.5167081260031493
y: -3.125171302690142
qz: -0.7044294777312331
qw: 0.7097739857893512
```

用户提供的 covariance 是定位估计的不确定度，不属于 `move_base` 目标，不发送为导航目标。巡线起点必须从 YAML 读取，不得硬编码在 Python 中。

## 3. 包边界与资源所有权

新增独立 ROS 包 `line_follow_integration`。它负责第三阶段图像适配、方向识别进程监管、巡线子程序监管和第三阶段内部状态上报，不负责全局业务编排、TTS、公共相机、底盘驱动、地图、定位或 `move_base`。

```text
task_orchestrator
  -> 第三阶段业务顺序、任务身份、超时、最终 TTS、COMPLETE/ERROR

line_follow_integration
  -> line_camera_adapter
  -> phase3_supervisor
  -> yolo_server 的受控快照/适配
  -> follow_left_v4.py
  -> follow_right_v4.py
  -> follow_mid_v4.py

现有 stop 导航栈
  -> 复用已运行的 map、AMCL、move_base 和动态避障导航
```

不得启动原始 `start_all_yolo.launch`，因为它会重复启动底盘和 `/dev/video0`。不得启动第二个相机、第二套定位或第二个 `move_base`。

本地 `E:\follow_v1\follow_v1` 作为已跑通算法来源。导入时记录源文件校验值；第一版保留 PID、速度、转向角、白线阈值和时序参数，只做集成必需的接口隔离、成功/失败判定和 TTS 移除。

## 4. 相机数据流

二维码已经调好的唯一物理相机配置保持不变：

```text
/dev/video0 -> /usb_cam/image_raw -> 1020x720
```

图像分为两路：

```text
/usb_cam/image_raw 1020x720
  |-- QR：继续使用原始图像
  |-- YOLO：使用原始图像，保留完整视野，由 Ultralytics 内部等比例缩放
  `-- line_camera_adapter
        -> 中心裁剪为 960x720
        -> 缩放为 640x480
        -> 限制为 15 FPS
        -> /line_follow/image_raw
        -> 左/右/直行巡线程序
```

中心裁剪使巡线输入保持 4:3，避免把 1020x720 直接拉伸为 640x480 造成横向形变。YOLO 不裁剪，以免丢失画面边缘的灯或方向标识。

适配器保留输入消息的时间戳和 frame_id。它可以常驻，但只在第三阶段激活后发布派生图像。它不修改、重启或动态重配物理相机。

如果实车发现裁剪影响巡线，可仅通过 YAML 将 `crop_mode` 改为经验证的其他模式；第一版默认 `center_4_3`。

## 5. 速度隔离

巡线脚本不得直接成为最终 `/cmd_vel` 发布者：

```text
巡线脚本 /cmd_vel
  --ROS remap--> /cmd_vel/line_follow
  --velocity_arbiter, mode=LINE_FOLLOW--> /cmd_vel
```

全局速度仲裁器新增 `LINE_FOLLOW` 模式和 `line_follow` 输入源。只有全局状态为 `LINE_FOLLOWING` 时放行该源。模式切换、错误、超时、节点退出和关闭时都先发布零速度。

第三阶段导航期间继续使用现有 stop 导航速度链和 `STOP_NAVIGATION` 模式；导航成功后先切换到 `IDLE` 并确认零速度，再进入 `LINE_FOLLOW`。

## 6. 全局状态机

第二阶段仿真车间 TTS 收到匹配的 `speak_done status=success` 后，不再直接完成任务：

```text
WAITING_SIMULATION_SPEECH
  -> NAVIGATING_LINE_START
  -> WAITING_LINE_DIRECTION
  -> LINE_FOLLOWING
  -> WAITING_FINAL_SPEECH
  -> COMPLETE
```

详细行为：

1. `NAVIGATING_LINE_START`：发布带任务身份和 YAML 航点的导航请求；复用现有 `move_base`。
2. 导航返回 `SUCCEEDED` 后取消残留目标、切换 `IDLE` 并确认停车。
3. `WAITING_LINE_DIRECTION`：启动本次 YOLO 识别，清除旧结果，只接受激活之后生成的结果。
4. `red_light` 保持等待且不发送巡线速度。
5. 收到 `left_turn`、`right_turn` 或 `straight` 后锁定本次路线；后续识别变化不得中途改路线。
6. 30 秒没有方向结果时按用户确认的规则选择 `straight`。
7. `LINE_FOLLOWING`：启动对应 V4 子程序，并放行 `/cmd_vel/line_follow`。
8. 只有最终停车线检测产生本次新鲜的完成标记，且车辆已经发送零速度，才算巡线成功。
9. 成功后 `task_orchestrator` 请求一次“任务完成”TTS；只有匹配的成功回执进入 `COMPLETE`。

不得由 `auto_drive_v3.py` 或巡线脚本直接调用 TTS。

## 7. 公共接口

所有公共第三阶段接口以 `task_orchestrator` 命名为准：

```text
/task/line_navigation_goal     std_msgs/String JSON
/task/line_navigation_arrived  std_msgs/String JSON
/task/line_follow/start        std_msgs/String JSON
/task/line_follow/status       std_msgs/String JSON
/task/motion_mode              std_msgs/String
/cmd_vel/line_follow           geometry_msgs/Twist
```

公共 JSON 都包含：

```json
{
  "protocol_version": 1,
  "task_id": "task-...",
  "goal_id": "line-..."
}
```

导航目标额外包含 `frame_id` 与 `pose`。导航返回包含 `status=success|failure` 和失败 `reason`。巡线状态使用以下枚举：

```text
waiting_signal
direction_selected
following
success
failure
```

`direction_selected` 和 `following` 携带锁定方向；`failure` 携带可诊断原因。所有消费端按 `task_id/goal_id` 关联并去重，忽略过期或重复事件。

YOLO 的 `/tmp/yolo_result.txt` 和巡线的 `/tmp/stop_done.txt` 只作为包内兼容接口，不得成为 `task_orchestrator` 的公共依赖。`phase3_supervisor` 在每次激活前删除旧文件，并校验新文件修改时间晚于本次激活时间。

## 8. 配置文件与绝对路径

配置文件仓库相对路径：

```text
ucar_ws/src/line_follow_integration/config/phase3.yaml
```

本地开发绝对路径：

```text
D:\program_sec\智能车\.worktrees\phase3-line-follow-integration\ucar_ws\src\line_follow_integration\config\phase3.yaml
```

计划部署到小车后的绝对路径：

```text
/home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml
```

默认内容至少包括：

```yaml
line_start_goal:
  frame_id: map
  x: 0.5167081260031493
  y: -3.125171302690142
  qz: -0.7044294777312331
  qw: 0.7097739857893512

camera:
  input_topic: /usb_cam/image_raw
  line_topic: /line_follow/image_raw
  output_width: 640
  output_height: 480
  output_fps: 15.0
  crop_mode: center_4_3

timeouts:
  navigation: 300.0
  image_ready: 5.0
  image_recovery_grace: 3.0
  direction: 30.0
  line_follow: 120.0
```

现场调整第三阶段起点只修改此 YAML 并重启根 launch，不修改 Python。第二阶段三个车间扫描航点在本任务中暂不迁移，避免扩大改动范围。

## 9. 子进程与故障处理

`phase3_supervisor` 只管理它自己启动的 YOLO 和巡线子进程，并记录 PID。不得使用无目标的 `pkill`、`killall` 或 `rosnode kill`。

故障策略：

- 导航拒绝、失败或 300 秒超时：零速度并进入全局 `ERROR`。
- 图像初始 5 秒未就绪：零速度并失败。
- 巡线过程中图像中断：立即阻断巡线速度；允许 3 秒恢复窗口，仍未恢复才进入 `ERROR`。
- 方向识别 30 秒无结果：选择 `straight`，不是错误。
- 巡线子进程提前退出：失败，不得伪造成功。
- 巡线 120 秒未检测最终停车线：零速度、结束所拥有的子进程并失败。
- 最终停车标记必须属于本次激活；旧文件或旧任务事件不能推进状态机。
- TTS 失败或超时：保持停车并进入 `ERROR`，不得进入 `COMPLETE`。

失败日志必须包含状态、`task_id`、`goal_id`、方向、阶段和 `reason`，以避免再次出现“车辆不动但无第三阶段日志”的情况。

## 10. 启动结构

根启动脚本仍为现有一键入口。根 launch 增加第三阶段包的常驻轻量节点和参数，但不启动原始 `start_all_yolo.launch`。导航、方向等待和巡线的业务顺序只由状态机驱动，不写入 XML。

建议新增全局启停参数：

```text
start_line_follow=true
```

关闭时必须使第三阶段交接明确失败或停留在安全 `IDLE`，不得跳过第三阶段并宣称比赛完成。

## 11. 测试与验收

### 11.1 本地自动测试

- YAML 航点、相机和超时默认值精确；
- 第二阶段仿真播报成功后只发布一次第三阶段导航目标；
- 过期、重复和错误任务身份不能推进状态机；
- move_base 成功前不启动 YOLO 或巡线；
- `red_light` 保持停车；
- 三种方向选择对应正确脚本；
- 方向超时选择 `straight`；
- 子进程提前退出和巡线超时不产生假成功；
- 最终停车成功后只请求一次“任务完成”TTS；
- `LINE_FOLLOW` 只放行 `/cmd_vel/line_follow`；
- launch XML 只有一个公共相机、一个底盘和一套定位导航所有者；
- Python 编译、XML 解析、`git diff --check` 和现有第一、二阶段回归通过。

### 11.2 无运动联调

使用模拟 topic/action 依次注入导航成功、红灯、方向、停车成功和 TTS 回执，验证完整日志、状态机和零速度边界。禁止用伪造成功替代实车最终验收。

### 11.3 实车看护验收

按风险逐层放行：

1. 只验证新增导航起点及停车姿态；
2. 底盘保持 `IDLE`，验证原始 1020x720 YOLO 和派生 640x480 巡线图像；
3. 验证红灯等待与三类方向锁定；
4. 单独放行 `/cmd_vel/line_follow` 验证路线；
5. 验证最终停车线、零速度和单次完成播报；
6. 最后运行第一至第三阶段全流程。

巡线图像验收关注实际帧率、处理延迟、线中心偏差、白线检测和 YOLO 置信度。如果中心裁剪表现不如原生 640x480，再基于实测记录调整适配策略，不在第一版预先重写巡线算法。

## 12. 非目标

第一版不做以下工作：

- 重写已经跑通的巡线 PID 或路线算法；
- 动态重配或重启物理相机；
- 合并 stop 与巡线状态机；
- 迁移第二阶段三个车间扫描航点到第三阶段 YAML；
- 中途根据 YOLO 新结果切换已经锁定的路线；
- 在巡线脚本内直接播报或绕过全局速度仲裁器。
