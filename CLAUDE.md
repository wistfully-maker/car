# Stop 第二阶段联调：DeepSeek V4 Flash 执行约束

## 1. 当前唯一任务

在不重写车端 `stop` 已跑通算法的前提下，将它接入已验收的第一部分，实现：

```text
TTS 后安全切换导航栈
 -> 实物车间识别 + 动态避障导航 + 停车
 -> 仿真目标车间识别 + 动态避障导航 + 停车
 -> COMPLETE
```

设计权威文档：

```text
docs/superpowers/specs/2026-08-10-stop-phase2-integration-design.md
```

设计与本文冲突时，以本文的安全限制为准；用户后续明确指令优先。

## 2. 权威工作位置

只允许在：

```text
D:\program_sec\智能车\.worktrees\stop-phase2-integration
branch: codex/stop-phase2-integration
required baseline ancestor: 77e853a767152d7e877b736af6390605c841e6d2
```

开始前必须运行：

```powershell
git branch --show-current
git rev-parse HEAD
git merge-base --is-ancestor 77e853a HEAD
git status --short
git log -8 --oneline
```

路径不存在、分支不符、基线不是祖先或存在无法识别的脏文件时，立即停止并报告 Codex。不得自行创建、切换分支或 worktree。

## 3. 车端 Stop 来源

权威原始包在小车：

```text
ucar@192.168.1.109:/home/ucar/ucar_ws/src/stop
```

DeepSeek **禁止 SSH/SCP**。只能使用 Codex 事先导入本工作树、附带清单和哈希的 stop 快照。如果本地还没有该快照，停止并报告，不得用 `ucar_avoid`、`ucar_delivery` 或自写代码冒充。

## 4. 不可改变的 Stop 行为

保留：

- `FIND_POINTS_LIST` 航点和原有遍历顺序；
- AMCL + TEB + move_base 配置；
- OCR 节点、车间关键字映射和已验证模型；
- Stage 0-6 的 OCR/PCA/白框停车顺序；
- Phase 1 完成后的倒车、掉头、清理代价地图和 Phase 2 连续流程；
- 车端已跑通的参数默认值。

不得“顺手优化”、重写、换算法或引入 `ucar_delivery` 代替上述逻辑。必须先用 characterization tests 冻结已有行为。

## 5. 允许的最小改造

仅允许为集成所必需的改造：

1. 节点启动后等待结构化任务，不自动运动。
2. 注入实物和仿真的车间/货品目标。
3. 将 stop 的导航和手动/PCA 速度输出重映射到隔离 topic。
4. 将 `phase1_done/done/failed:*` 转换为 protocol v1 JSON。
5. 取消 stop 内部的 TTS 副作用，交给全局编排器。
6. 增加取消、超时、异常和 shutdown 零速度。
7. 新增不拥有公共硬件的 stop integration launch。
8. 新增 navigation handoff supervisor、readiness 和 preflight。

如果必须修改原始算法才能继续，停止并报告 Codex，不得自行扩大范围。

## 6. 全局状态和消息

实现目标顺序：

```text
DELIVERY_HANDED_OFF
 -> SWITCHING_TO_STOP_STACK
 -> NAVIGATING_TO_WORKSHOP
 -> PHYSICAL_PARKED
 -> NAVIGATING_TO_SIM_WORKSHOP
 -> SIMULATION_PARKED
 -> COMPLETE
```

协议：

```text
/task/delivery_navigation_goal -> /task/delivery_arrived
/task/simulation_navigation_goal -> /task/simulation_arrived
```

每个输入/输出都必须携带 `protocol_version=1`、`task_id`、`goal_id`。重复终态任务只允许重发缓存结果，不得重复运动。过期、错误 phase 或错误身份的事件必须忽略。

第一次停车只发布 `/task/delivery_arrived`，不得提前 `COMPLETE`。第二次停车稳定确认后才发布 `/task/simulation_arrived`。

## 7. 导航栈交接安全合同

handoff supervisor 是唯一的导航栈生命周期 owner。必须实现：

```text
保持 IDLE
 -> cancel 遗留 action goal
 -> 新鲜 odom 连续近零
 -> 停止旧 move_base/lidar_loc/map 进程组
 -> 确认旧 owner 退出
 -> 启动 stop AMCL/TEB/move_base/OCR
 -> 发布初始位姿
 -> 等待 map/AMCL/TF/action/OCR/camera/scan 就绪
 -> 放行 stop 任务
```

不得使用 `pkill ros`、`rosnode kill -a`、删除串口锁或通过节点名模糊 kill。只能管理由 supervisor 明确启动并持有句柄的导航进程组。

故障政策不是“一次就 ERROR”：

- 取消、退出、ROS 注册更新、TF/AMCL/move_base/OCR readiness 有界重试；
- 每个重试有单次上限和阶段总超时；
- 重试期间保持零速度并发布可诊断状态；
- 旧栈未停止时可留在交接前重试；
- 旧栈已停止而新栈无法就绪时，零速度并进入带诊断原因的 `ERROR`，等待人工处理或重新启动本次任务；
- 不自动带着不确定定位继续比赛，不自动切回后立即运动。

## 8. 速度所有权

目标连接：

```text
/cmd_vel/stop_navigation --\
/cmd_vel/stop_manual -------> stop_velocity_mux -> /cmd_vel/stop

/cmd_vel/navigation --------\
/cmd_vel/qr -----------------> competition_velocity_arbiter -> /cmd_vel
/cmd_vel/stop ---------------/
```

只有 `competition_velocity_arbiter` 可发布最终 `/cmd_vel`。每次模式切换必须先发零速度；新模式对应源的新鲜消息到达前不允许运动。输入超时、非法数值、未知模式、时间回退、异常和 shutdown 都 fail closed。

## 9. 必须 TDD

每项先写失败测试，确认 RED，再做最小 GREEN，然后回归和单独提交。建议拆分：

1. `test(orchestrator): lock dual workshop navigation contract`
2. `test(handoff): lock bounded navigation stack transition`
3. `chore(stop): import verified vehicle package snapshot`
4. `feat(stop): gate mission on protocol task input`
5. `feat(stop): isolate navigation and manual velocity outputs`
6. `feat(stop): report correlated dual parking results`
7. `feat(bringup): compose controlled stop stack handoff`
8. `docs(integration): document checkpointed phase two acceptance`

原始 stop 快照必须作为独立提交，包含来源、导入时间、文件清单和 SHA-256，不得与改造混在同一提交。

## 10. 禁止

- 修改其他 worktree 或根工作区；
- SSH、SCP、远程编译、启停小车节点或控制底盘；
- push、merge、rebase、reset、clean、`git add .`、`git add -A`；
- 重写 stop 已验证算法或用 `ucar_delivery` 替换 stop；
- 同时启动 lidar_loc 和 AMCL；
- 同时启动两个 move_base、map server、相机、雷达或底盘 driver；
- 让 stop、move_base 或其他节点直接发布最终 `/cmd_vel`；
- 伪造 `/task/delivery_arrived`、`/task/simulation_arrived` 或实车验收结果；
- 为通过测试修改 protocol v1 身份语义；
- 提交密钥、日志、录包、原始相机图像、模型缓存或构建目录。

## 11. 本地验证

最终至少运行：

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/task_orchestrator ucar_ws/src/stop
git diff --check
git status --short
```

必须用 `xml.etree.ElementTree` 解析所有新增或修改 launch。必须有 fake ROS/process tests 验证交接顺序、有界重试、不误停公共硬件、不重复 owner 和全路径零速度。

## 12. 文档和交接

必须新建/更新：

```text
ucar_ws/src/stop/README_INTEGRATION.md
HANDOFF_TO_CODEX.md
```

handoff 记录：分支和 HEAD、每个提交、修改文件、测试数量与完整摘要、stop 快照来源与哈希、导航栈交接顺序、全部 topic/remap、参数表、已知假设、未完成的车端验收。

完成本地实现后停止。Codex 后续按以下检查点验收，每关单独授权：

```text
代码审查
 -> 本地全量回归
 -> 无运动启动
 -> 第一导航栈安全退出
 -> stop 导航栈只启动不发目标
 -> AMCL/TF/readiness
 -> 单独实物导航停车
 -> 单独仿真目标导航停车
 -> 连续两阶段
```

不得声称 DeepSeek 的本地测试证明已经部署、已经实车通过或已经完成全比赛。
