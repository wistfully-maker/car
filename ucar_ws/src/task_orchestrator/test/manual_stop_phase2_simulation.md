# Stop 第二阶段无运动人工联调手册（非实车验收）

> 本手册只验证消息链路与安全顺序，**不启动任何运动**。所有注入结果必须使用
> 上一步真实输出中复制的 `task_id` / `goal_id`，禁止凭空编造身份。
> 本流程不是车辆验收；实车验收由 Codex 按检查点单独授权执行。

## 0. 安全前提

- 机器人**机械断能**或架空，确认无人误触。
- 先确认本机没有残留导航节点：
  `rosnode list | grep -E 'move_base|amcl|lidar_loc|map_server|navigation_handoff' || true`
- 所有 `rostopic echo -n 1` 观察到的 ID 都要**复制粘贴**到下一步，不要手打。

## 1. 无运动启动

```bash
roslaunch task_orchestrator competition_full.launch
# 或带交接开关的脚本（在真实部署上）：
./src/task_orchestrator/scripts/start_competition.sh
```

预期（全部静态观察，无运动）：

```bash
# 全局编排器状态与运动模式
rostopic echo -n 1 /task/status
# {"protocol_version":1, "task_id":"...", "state":"IDLE", ...}
rostopic echo -n 1 /task/motion_mode        # IDLE

# 交接 supervisor 已注册（不运动、不发目标）
rosnode list | grep navigation_handoff_supervisor
rostopic echo -n 1 /task/navigation_handoff_status

# stop 栈节点已就绪但未激活（mission 等待结构化任务）
rosnode list | grep -E 'mission_orchestrator|stop_protocol_adapter|stop_velocity_mux'
rostopic echo -n 1 /stop/motion_mode         # IDLE
```

## 2. 驱动到 TTS 完成（业务链路）

按第一阶段手册注入任务请求、依赖就绪、取件到达、QR 结果、LLM 结果、语音完成，
直到：

```bash
rostopic echo -n 1 /task/status              # state=DELIVERY_HANDED_OFF
rostopic echo -n 1 /task/motion_mode         # IDLE（不授予运动）
rostopic echo -n 1 /task/delivery_navigation_goal
# 记录其中的 task_id 与 goal_id（例如 delivery-1），后续全部复用
```

## 3. 观察导航栈所有权与速度源（仍无运动）

```bash
# 旧栈（第一部分）由 supervisor 持有；stop 栈尚未启动
rosnode list | grep -E 'move_base|lidar_loc|map_server'
# 应看到旧 move_base/lidar_loc/map_server，不应看到 stop 栈的 AMCL

# 速度所有权：最终 /cmd_vel 只有竞争仲裁器发布
rostopic info /cmd_vel        # Publishers 应只有 /velocity_arbiter
rostopic info /cmd_vel/stop   # 只有 /stop_velocity_mux
rostopic info /cmd_vel/navigation   # 只有旧 move_base（经 fast_nav 配置）
rostopic info /cmd_vel/stop_navigation  # 未启动（stop 栈未起）
rostopic info /cmd_vel/stop_manual      # 未启动
```

## 4. 模拟交接（只读注入）

把第 2 步记录的 `delivery_goal` 复制到：

```bash
rostopic pub -1 /task/delivery_navigation_goal std_msgs/String \
  'data: "{...上一步复制的完整 JSON...}"'
```

预期顺序（全部由 supervisor 状态可见，无运动）：

```bash
rostopic echo /task/navigation_handoff_status
# state=CANCELLING -> VERIFYING_STOP -> STOPPING_LEGACY -> STARTING_STOP
#   -> WAITING_STOP_READY -> READY（有界重试时 reason=retrying）
rosnode list | grep -E 'move_base|amcl|lidar_loc|map_server'
# 旧 move_base/lidar_loc/map_server 退出，stop 栈 map_server/amcl/move_base 出现
rostopic echo -n 1 /task/motion_mode         # STOP_NAVIGATION（放行后）
rostopic echo -n 1 /task/stop_mission_goal   # 记录 task_id 与 goal_id（=delivery-1）
rostopic echo -n 1 /stop/motion_mode         # mission 激活后为 NAVIGATION/MANUAL
```

若交接进入 `FAILED`（reason 见状态消息），停在原地即可：模式保持 IDLE、
无任何到达发布——这正是失败场景的预期安全表现。

## 5. 模拟实物停车完成（Phase 1）

注入 mission 内部事件（ID 必须来自上一步复制的真实输出）：

```bash
rostopic pub -1 /stop/mission_event std_msgs/String 'data: "phase1_done"'
```

预期：

```bash
rostopic echo -n 1 /task/delivery_arrived
# {"protocol_version":1, "task_id":"<复制的>", "goal_id":"<复制的delivery-1>",
#  "status":"arrived", "message":""}
rostopic echo -n 1 /task/status              # state=NAVIGATING_TO_SIM_WORKSHOP
rostopic echo -n 1 /task/simulation_navigation_goal
# 记录其中的 task_id 与 goal_id（例如 simulation-1）
```

## 6. 模拟仿真目标放行（Phase 2）

```bash
rostopic pub -1 /task/simulation_navigation_goal std_msgs/String \
  'data: "{...上一步复制的完整 JSON...}"'
```

预期：

```bash
rostopic echo -n 1 /stop/mission_ack         # action=start_phase2
rostopic echo -n 1 /stop/motion_mode         # mission 进入 Phase 2 后为 NAVIGATION
```

## 7. 模拟仿真停车完成

```bash
rostopic pub -1 /stop/mission_event std_msgs/String 'data: "done"'
rostopic echo -n 1 /task/simulation_arrived
# {"protocol_version":1, "task_id":"<复制的>", "goal_id":"<复制的simulation-1>",
#  "status":"arrived", "message":""}
rostopic echo -n 1 /task/status              # state=COMPLETE
rostopic echo -n 1 /task/motion_mode         # IDLE
```

## 8. 失败与取消注入（任选其一验证安全顺序）

- 交接阶段注入失败：`rostopic pub -1 /stop/mission_event ... 'data: "failed:not_found"'`
  在 phase1_done 之前 → 只能看到 `/task/delivery_arrived` 的 `failed`，
  看不到任何 `arrived`；`/task/motion_mode` 保持 IDLE。
- 取消：`rostopic pub -1 /task/cancel std_msgs/String \
  'data: "{\"protocol_version\":1,\"task_id\":\"<复制的>\",\"reason\":\"operator_cancel\"}"'`
  → `/task/status` state=CANCELLED、motion_mode=IDLE、无成功到达。

## 9. 结束

```bash
# 停掉 launch（Ctrl+C）；supervisor shutdown 会停止它持有的导航进程组并发零速度
```

## 10. 明确边界

- 本手册所有步骤都是**消息注入**，机器人必须保持断能/架空；
- 任何"实车移动、实车停车、连续两阶段"都不在本手册范围内，
  需要 Codex 逐个检查点单独授权；
- 所有 ID 必须复制自上一步真实输出；手打或编造的 ID 会被身份校验忽略，
  属于预期行为而非故障。
