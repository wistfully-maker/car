# 第二阶段接口对齐与流程修复设计

## 目标

以 `task_orchestrator` 的 topic 名称和 protocol v1 JSON 为唯一外部合同，用最小工程量修复二维码播报后的定位交接、首航点误跳过、航点索引错位、两次停车播报缺失，以及 handoff 状态 topic 混入无效诊断消息的问题。

## 范围

本次修改只覆盖已经联调的第一部分到车端 `stop` 第二部分的连接链路：

- 二维码/LLM 首次比赛结果播报完成后切换到 stop 导航栈；
- 从二维码领取区实际位置导航至第一个车间航点；
- 实物车间 OCR、停车、播报；
- 仿真车间 OCR、停车、最终播报；
- 全局任务完成与失败、超时、重复消息处理。

不重写车端 TEB、AMCL、OCR、PCA 停车或底盘算法，也不更换已经实车运行过的三个车间航点。

## 唯一外部接口

全局接口以 `task_orchestrator` 为准：

| 方向 | Topic | 类型与约束 |
|---|---|---|
| 编排器 → handoff | `/task/delivery_navigation_goal` | `std_msgs/String`，protocol v1 JSON |
| handoff → stop | `/task/stop_mission_goal` | `std_msgs/String`，包含全局 task/goal identity |
| stop adapter → 编排器 | `/task/delivery_arrived` | 第一次稳定停车结果 |
| 编排器 → stop adapter | `/task/simulation_navigation_goal` | 第二次导航目标 |
| stop adapter → 编排器 | `/task/simulation_arrived` | 第二次稳定停车结果 |
| 编排器 → TTS | `/voice/speak` | protocol v1，包含 task_id/speech_id/text |
| TTS → 编排器 | `/voice/speak_done` | protocol v1，必须匹配 task_id/speech_id |
| handoff → 编排器 | `/task/navigation_handoff_status` | 只允许 `status=ready|failed` |

`stop` 的 `/stop/mission_event` 和 `/stop/mission_ack` 保持私有适配缝，不允许成为全局状态机的直接依赖。stop 不直接调用本地 TTS 脚本。

## 全局状态机

初始二维码结果播报沿用现有 `WAITING_SPEECH`。第二部分新增两个独立等待状态：

1. `NAVIGATING_TO_WORKSHOP`
2. 收到匹配的 `/task/delivery_arrived status=arrived`
3. `WAITING_DELIVERY_SPEECH`：发布且只发布一次“已将{实物}放入{实物车间}”
4. 收到匹配的 `/voice/speak_done status=success`
5. 发布且只发布一次 `/task/simulation_navigation_goal`
6. `NAVIGATING_TO_SIM_WORKSHOP`
7. 收到匹配的 `/task/simulation_arrived status=arrived`
8. `WAITING_SIMULATION_SPEECH`：发布且只发布一次“仿真任务已完成，已将{仿真物品}放入{仿真车间}”
9. 收到匹配的 `/voice/speak_done status=success`
10. `COMPLETE`

两个新增等待状态使用现有 `timeout_speech`。错误 task_id、过期 speech_id、重复到达和重复 speak_done 不产生第二个导航目标、第二次播报或重复完成。TTS 失败或超时进入 `ERROR`，运动模式回到 `IDLE`。

## 定位交接

第一部分与 stop 使用的 PGM 地图 SHA-256 相同，因此使用同一 map 坐标系。二维码领取区权威观察点为：

```text
x   = -1.40219
y   = -0.627908
yaw = 0.053792653589793
```

定位所有权归 `navigation_handoff_supervisor`：

- stop 的集成 launch 将内部自动 `/initialpose` 默认关闭；
- supervisor 的 `/initialpose` 发布器使用 latch，在 stop 栈启动后立即发布上述领取区位姿，确保稍后启动的 AMCL 也能收到；
- stop readiness 成功后才授权 `STOP_NAVIGATION` 并放行 `/task/stop_mission_goal`。

这样车辆的 TF 会反映二维码区实际位置，到第一个车间航点约 1.91 米，不会触发 `dist < 0.3` 的误跳过分支。

## 航点索引语义

`current_point_index` 始终表示“正在导航或正在 OCR 扫描的航点”，不得在到达回调中提前递增。

- 已经接近当前航点：只标记到达并触发 OCR，不递增；
- move_base 成功：只标记到达并触发 OCR，不递增；
- 当前航点完成全部旋转且仍未找到目标：此时递增一次，再导航到下一航点；
- Phase 1 发现仿真车间时，`sim_point_index` 记录当前物理航点；
- Phase 2 可以准确返回该航点，不会偏移到下一个坐标。

PCA 逼近期间的 move_base 结果不改变搜索航点索引。

车端已验证的三个航点保持不变：

```text
(-0.812845, -2.44196)
( 0.771455, -2.44196)
( 1.784300, -2.43094)
```

## Handoff 状态与诊断

`/task/navigation_handoff_status` 是严格业务接口，只发布相关联的 `ready` 或 `failed`。重试、等待、底盘仍在运动等过程诊断保留在 ROS 日志，不再作为缺少 `status` 的 JSON 发布到该 topic。这样消除 `status must be non-empty` 警告，同时不改变最终成功/失败合同。

## 测试策略

所有生产修改先由失败回归测试锁定：

1. 编排器测试两次停车分别进入两个语音等待状态，匹配 speak_done 后才继续；
2. 测试 TTS 失败、超时、重复和过期 identity 均 fail closed 或被忽略；
3. 场景测试完整顺序为初始播报、第一次停车播报、第二次停车播报，共三次 TTS；
4. supervisor 测试 `/initialpose` 只由 supervisor 发布、坐标为领取区观察点，且任务必须在 readiness 成功后才放行；
5. launch/XML 测试 stop 集成 launch 不再发布航点1初始位姿；
6. stop 特征测试到达航点不递增，扫描耗尽才递增，双指针记录当前航点；
7. handoff 测试状态 topic 上的所有消息都能通过 `task_orchestrator.protocol.parse_navigation_handoff_status`；
8. 运行 task_orchestrator、stop、llm_spark 全量单元测试、compileall、XML 解析与 `git diff --check`。

## 验收标准

- 二维码结果播报结束后，车辆从领取区实际位置导航约 1.91 米至航点1，再开始 OCR；
- 第一次停车播报完成前不进入第二阶段；
- 第二次停车播报完成前不进入 `COMPLETE`；
- 全程外部 topic 名称与 protocol v1 以 `task_orchestrator` 为准；
- 无重复 `/initialpose` 所有者，无 `status must be non-empty` 警告；
- 航点与 OCR 记录不存在 off-by-one；
- 任一失败均停止推进，保持 `IDLE` 并保留明确诊断原因。

以上验收标准在本地只由离线测试证明；在 Codex 完成备份、部署、车端编译和人工看护下的真实
语音/导航/OCR/停车运行前，不得声称第二部分已通过实车验收。
