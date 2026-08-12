# Gazebo 仿真任务软门控联调实施计划

> **执行位置：** `D:\program_sec\智能车\.worktrees\phase3-line-follow-integration`，分支 `codex/phase3-line-follow-integration`。继续使用当前分支，不新建包级分支或 worktree。

**目标：** 在仿真车间停车成功后运行电脑端 Gazebo 任务；无论 Gazebo 成功、失败或超时，均执行原仿真播报并继续第三部分。

**架构：** 车端 `task_orchestrator` 新增 `WAITING_GAZEBO` 软门控，电脑端使用薄适配器订阅 start、调用现有 Gazebo 任务并发布 complete。业务顺序仍只由 `task_orchestrator` 管理，电脑端不获得底盘控制权。

**协议：** `/task/gazebo/start` 与 `/task/gazebo/complete`，均为非 latch 的 `std_msgs/String` JSON protocol v1；`task_id + goal_id` 负责关联和去重。

---

## 任务 0：冻结基线并保护车端现状

**检查位置：**

- 本地：`D:\program_sec\智能车\.worktrees\phase3-line-follow-integration`
- 车端：`/home/ucar/ucar_ws/src/task_orchestrator`
- 车端未回同步修改：`/home/ucar/ucar_ws/src/line_follow_integration/scripts/follow_left_v5.py`
- 车端未回同步修改：`/home/ucar/ucar_ws/src/line_follow_integration/scripts/follow_right_v5.py`

- [ ] 记录本地 `git status --short`、分支和 HEAD；起点应包含 `b6ebdca86e977334b70bc17a24e37b3223fbe895`。
- [ ] 记录车端两个 V5 文件哈希。它们已经去掉“前停车线后 0.15 m/s 限速”，尚未回同步到本地。
- [ ] 本次部署白名单只覆盖 `task_orchestrator`，不得覆盖车端 `line_follow_integration`，从而保留实测 V5。
- [ ] 若以后需要部署整个工作空间，必须先把车端两个 V5 文件同步回本地并单独提交，不能让旧本地副本反向覆盖。

## 任务 1：先锁定状态机软门控合同（RED）

**文件：**

- 修改：`ucar_ws/src/task_orchestrator/test/test_orchestrator.py`
- 修改：`ucar_ws/src/task_orchestrator/test/test_motion_mode.py`

- [ ] 在测试 Harness 的 timeout 表增加 `gazebo: 60.0`，ID 序列在 simulation goal 后插入 `gazebo-1`。
- [ ] 测试仿真停车成功后进入 `WAITING_GAZEBO`，先发布一次 `publish_gazebo_start`，且此时没有新增 `publish_speech`。
- [ ] 精确断言 start 字段为 `protocol_version/task_id/goal_id/selected_item/target_category/target_workshop`。
- [ ] 测试 `WAITING_GAZEBO` 的 motion mode 明确为 `IDLE`。
- [ ] 测试匹配 success 进入 `WAITING_SIMULATION_SPEECH` 并使用当前原文播报。
- [ ] 测试匹配 failure 也进入相同播报状态，记录 reason，但不进入 `ERROR`。
- [ ] 测试超时也进入相同播报状态，记录 timeout，但不进入 `ERROR`。
- [ ] success、failure、timeout 三条路径随后收到匹配 TTS success 时，都只发布一次 `/task/line_navigation_goal`。
- [ ] 测试错误 task、错误 goal、畸形 status、重复 complete 均不重复播报或导航。
- [ ] 测试取消仍进入 `CANCELLED`，物理仿真停车失败仍进入 `ERROR`。
- [ ] 测试 `gazebo_phase_enabled=false` 时保持当前“停车后立即播报”的兼容行为。
- [ ] 运行聚焦测试并确认旧代码因没有 `WAITING_GAZEBO` 和 start action 而失败。

## 任务 2：实现纯状态机最小改动（GREEN）

**文件：**

- 修改：`ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- 修改：`ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py`

- [ ] 新增 `WAITING_GAZEBO`，加入 active states 和 timeout key `gazebo`。
- [ ] 构造函数新增 `gazebo_phase_enabled=False`，不改变未启用时的旧行为。
- [ ] `on_simulation_arrived()` 在停车成功且开关启用时生成 `gazebo_goal_id`，进入等待态并发出一次 `publish_gazebo_start`。
- [ ] start 的三个业务字段只从已校验的 `self.task["simulation"]` 读取，不从原始语音重新推断。
- [ ] 新增 `on_gazebo_complete()`，只接收当前 task/goal 的 success 或 failure。
- [ ] 新增一个内部汇合函数，统一处理 success、failure、timeout，然后调用现有 `format_simulation_delivery_speech()` 和 `_start_speech()`。
- [ ] 只为 `WAITING_GAZEBO` 特判软超时；其余状态继续使用现有 `_fail()` 硬失败。
- [ ] 给 `_start_speech()` 增加默认空值的可选 `status_message`；failure/timeout 在进入播报态时通过 `/task/status.message` 留下 ASCII 诊断摘要，其他调用不变，TTS 文本仍保持现有中文正向文案。
- [ ] 运行任务 1 的聚焦测试直至通过。

## 任务 3：增加 protocol v1 严格解析（RED→GREEN）

**文件：**

- 修改：`ucar_ws/src/task_orchestrator/test/test_protocol.py`
- 修改：`ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py`

- [ ] 先测试 `parse_gazebo_complete(raw, expected_task_id, expected_goal_id)`。
- [ ] success 只需 version、task、goal、status；允许缺省 reason。
- [ ] failure 必须包含去除首尾空白后仍非空的 reason。
- [ ] 拒绝错误 protocol version、错误 task、错误 goal、未知 status、非 JSON object 和非文本 reason。
- [ ] 不复用 `parse_arrival()`，避免混用 `arrived/failed` 与 `success/failure` 两套状态词。
- [ ] 运行 `test_protocol.py` 直至通过。

## 任务 4：接线 ROS 节点、配置和 launch（RED→GREEN）

**文件：**

- 修改：`ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- 修改：`ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- 修改：`ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`
- 修改：`ucar_ws/src/task_orchestrator/launch/competition_full.launch`
- 修改：`ucar_ws/src/task_orchestrator/test/test_package_config.py`
- 修改：`ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`

- [ ] 先用结构测试锁定两个 topic、非 latch publisher、subscriber parser、开关和 timeout 透传。
- [ ] `_make_publishers()` 新增非 latch `/task/gazebo/start`。
- [ ] `_subscribe()` 新增 `/task/gazebo/complete`，只在 `WAITING_GAZEBO` 调用严格 parser。
- [ ] `DEFAULT_TIMEOUTS` 和 YAML 增加 `gazebo: 60.0`。
- [ ] `task_orchestrator.launch` 增加 `gazebo_phase_enabled` 与 `timeout_gazebo`，加载 YAML 后由私有 param 覆盖。
- [ ] `competition_full.launch` 默认传 `gazebo_phase_enabled=true`、`timeout_gazebo=60.0`。
- [ ] 不新增 `start_gazebo`，不 include 电脑端 launch，不修改速度 remap 或底盘 owner。
- [ ] XML 解析测试和 launch 参数优先级测试通过。

## 任务 5：只读定位并接入电脑端 Gazebo

**前置条件：** 用户提供或在电脑上定位 Gazebo 工作空间绝对路径。未看到代码前不得猜测包名和完成判据。

**预计新增/修改内容：** 在 Gazebo 自身工作空间内增加一个薄 ROS1 适配器；最终文件路径在只读检查后写回本计划和交接文档。

- [ ] 只读确认 Gazebo 主启动 launch、任务入口函数、三个业务字段当前如何输入、成功判据和异常出口。
- [ ] 确认 Gazebo 节点能使用与小车相同的 ROS Master，且电脑的 `ROS_IP/ROS_HOSTNAME` 可被小车回连。
- [ ] 适配器订阅 `/task/gazebo/start`，使用 `task_id + goal_id` 去重。
- [ ] 将 `selected_item/target_category/target_workshop` 传给现有 Gazebo 入口，不改变仿真评分算法。
- [ ] 正常结束发一次 success；可判定失败或捕获异常发一次 failure + reason。
- [ ] 进程崩溃不做复杂恢复，由车端 60 秒 timeout 兜底。
- [ ] 适配器不得发布 `/cmd_vel`、`/task/motion_mode`、导航或语音 topic。
- [ ] 单独启动 Gazebo 与适配器；总 launch 不拥有它们的生命周期。

## 任务 6：文档和人工模拟命令

**文件：**

- 修改：`ucar_ws/src/task_orchestrator/README.md`
- 修改：`ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md`
- 修改：`ucar_ws/src/task_orchestrator/test/manual_simulation.md`

- [ ] 写清电脑端必须先启动并加入同一 ROS Master，再启动小车总 launch 和任务。
- [ ] 写清两个 JSON 合同、60 秒默认值、临时覆盖 `timeout_gazebo:=...` 和修改后重启要求。
- [ ] 提供 success、failure、完全不回消息三种无运动模拟方法。
- [ ] 提供 `rostopic info /task/gazebo/start` 检查电脑订阅者的方法；没有订阅者时不得开始正式计分任务。
- [ ] 写清 Gazebo failure/timeout 仍播报并继续，物理停车失败和 TTS 失败仍是硬失败。
- [ ] 写清两个 topic 非 latch，不能在 start 发布后才启动电脑端桥接器。

## 任务 7：本地回归与提交拆分

建议提交：

```text
test(orchestrator): lock gazebo soft-gate contract
feat(orchestrator): gate simulation speech on gazebo result
feat(bringup): wire gazebo protocol and timeout parameters
docs(integration): document gazebo desktop handoff
```

每个生产提交前先看到对应 RED，再看到 GREEN。最终至少运行：

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_orchestrator.py -v
python -m unittest ucar_ws/src/task_orchestrator/test/test_protocol.py -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/task_orchestrator
git diff --check
git status --short
```

额外用 `xml.etree.ElementTree` 解析 `competition_full.launch` 和 `task_orchestrator.launch`。本地测试不声称 Gazebo、网络、语音或实车已经联调成功。

## 任务 8：车端备份与白名单部署

- [ ] 在 `/home/ucar/ucar_ws/backups/` 创建带北京时间戳和说明的 `task_orchestrator` 备份。
- [ ] 再次记录车端两个 V5 哈希，确认部署包不包含 `line_follow_integration`。
- [ ] 只覆盖 `/home/ucar/ucar_ws/src/task_orchestrator` 中本任务修改的白名单文件。
- [ ] 编译 `task_orchestrator` 及必要依赖，source `/home/ucar/ucar_ws/devel/setup.bash`。
- [ ] 先用 `gazebo_phase_enabled:=false` 验证旧流程没有回归，再用 true 做 topic 级模拟。
- [ ] 不在无人看护时触发真实底盘流程。

## 任务 9：分级联调

### 9.1 无 Gazebo、无运动的状态机模拟

- [ ] 模拟到 `WAITING_GAZEBO`，确认 motion mode IDLE、start 一次、播报未提前。
- [ ] 分别发匹配 success、匹配 failure；确认两者都进入同一播报。
- [ ] 不发 complete，等待缩短后的测试 timeout；确认也进入同一播报。
- [ ] 发错误 task/goal 和重复 complete；确认不重复动作。

### 9.2 电脑端 Gazebo 单段联调

- [ ] 电脑先连接 ROS Master并启动适配器，检查 `/task/gazebo/start` 已有订阅者。
- [ ] 人工发布一条 start，核对 Gazebo 收到三个业务字段并真正执行。
- [ ] 核对电脑仅回一条匹配的 complete，成功与异常出口都覆盖。
- [ ] 暂停或杀死电脑端适配器，确认车端只能软超时，不进入整车 ERROR。

### 9.3 车端全流程联调

- [ ] 人工看护启动总 launch，完成前两部分。
- [ ] 仿真车间停车后确认小车静止、状态为 `WAITING_GAZEBO`、电脑开始仿真。
- [ ] Gazebo success：原仿真播报完成后，小车前往第三部分导航点。
- [ ] Gazebo failure：仍完成同一播报并前往第三部分导航点。
- [ ] Gazebo timeout：仍完成同一播报并前往第三部分导航点。
- [ ] 最终确认第三部分红绿灯、方向选择、V5 巡线及停车行为未被本次接口改动覆盖。

## 总验收门槛

1. 两个 topic 合同和 protocol v1 严格匹配。
2. Gazebo 成功、失败、超时不丢失原播报和第三部分。
3. stale/duplicate 消息不重复执行。
4. Gazebo 等待期间小车保持 `IDLE` 和零速度。
5. 电脑端断线最多造成 60 秒延迟，不造成流程永久阻塞。
6. 车端 V5 实测文件不被旧本地副本覆盖。
7. 本地全量回归、XML 解析、compileall 和 `git diff --check` 通过后，才允许白名单部署。
