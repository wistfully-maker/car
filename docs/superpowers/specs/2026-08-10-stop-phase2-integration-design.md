# Stop 第二阶段联调设计

## 目标

保留车端 `/home/ucar/ucar_ws/src/stop` 已经实车跑通的固定航点、TEB 动态避障、OCR 车间识别、LiDAR PCA 对正、白框停车和连续两阶段顺序，将它以最小改造接入 `task_orchestrator` 第一部分。

自动流程终点为：

```text
第一部分 TTS 完成
  -> 受控切换 ucar_fast_nav/lidar_loc 到 stop AMCL/TEB/move_base
  -> 实物车间识别、动态避障导航和停车
  -> /task/delivery_arrived
  -> 仿真目标车间识别、动态避障导航和停车
  -> /task/simulation_arrived
  -> COMPLETE
```

## 主要决策

1. `stop` 是第二部分的权威运动实现；不用本地 `ucar_delivery` 重写它。
2. 第一部分保持 HEAD `77e853a` 的导航和 QR 行为。
3. 通过一个专用 navigation handoff supervisor 切换导航栈；不在 XML 中编写业务顺序。
4. 公共底盘、雷达、相机和语音硬件不重复启动。
5. 最终 `/cmd_vel` 始终只有全局速度仲裁器发布。
6. `stop` 的两阶段算法顺序不改，仅增加启动门控、protocol v1 身份、速度隔离和结果映射。

## 工作空间

```text
worktree: D:\program_sec\智能车\.worktrees\stop-phase2-integration
branch: codex/stop-phase2-integration
baseline: 77e853a767152d7e877b736af6390605c841e6d2
```

不在 `feature/task-orchestrator`、根工作区或 `codex/delivery-dual-parking` 中开发。

## 状态与协议

全局状态顺序：

```text
WAITING_SPEECH
 -> DELIVERY_HANDED_OFF
 -> SWITCHING_TO_STOP_STACK
 -> NAVIGATING_TO_WORKSHOP
 -> PHYSICAL_PARKED
 -> NAVIGATING_TO_SIM_WORKSHOP
 -> SIMULATION_PARKED
 -> COMPLETE
```

保留 protocol v1：

```text
/task/delivery_navigation_goal -> /task/delivery_arrived
/task/simulation_navigation_goal -> /task/simulation_arrived
```

所有消息必须以 `task_id + goal_id` 去重和关联。第一次停车不得进入 `COMPLETE`；第二次经停稳确认后才能完成任务。

## 导航栈切换

handoff supervisor 顺序：

1. 收到匹配的 delivery goal，使全局运动模式保持 `IDLE`。
2. 取消第一导航栈的遗留 action goal。
3. 用新鲜 `/odom` 连续确认线速度和角速度近零。
4. 只停止第一部分的 `move_base/lidar_loc/map` 导航进程组。
5. 确认旧 `/move_base`、旧 `map->odom` 发布者和旧定位节点已退出。
6. 启动排除公共硬件的 stop 集成 launch。
7. 等待 map、AMCL、TF、move_base action、OCR、相机和雷达就绪。
8. 发布一次可配置初始位姿，等待定位新鲜度和协方差门槛。
9. 就绪门通过后才向 stop adapter 放行任务。

可恢复故障先做有界重试：action 取消、进程退出、节点注册清理、TF/AMCL/move_base/OCR readiness 均有独立次数和总超时。重试期间持续零速度。只有关键门槛耗尽后才进入带诊断原因的 `ERROR`，等待人工处理或重新启动本次任务。旧栈尚未停止时可留在交接前重试；旧栈已停止后不自动带着不确定定位继续运动。

## Stop 最小改造

保留车端原始算法和参数默认值。改造范围限于：

- 启动后等待任务，不自动运动；
- 接收结构化的实物和仿真目标；
- 把导航速度与手动/PCA/停车速度发往隔离 topic；
- 将 `phase1_done/done/failed:*` 映射为 protocol v1 JSON；
- 取消 stop 内部 TTS，语音统一由全局编排处理；
- 增加取消、超时、异常和 shutdown 零速度；
- 新增不启动底盘、雷达、相机和语音的 integration launch。

不重写航点搜索、OCR、PCA、白框停车、阶段切换或 TEB 参数。

## 速度所有权

```text
/cmd_vel/navigation -----\
/cmd_vel/qr --------------> competition velocity arbiter -> /cmd_vel
/cmd_vel/stop ------------/

/cmd_vel/stop_navigation --\
/cmd_vel/stop_manual -------> stop velocity mux -> /cmd_vel/stop
```

第二阶段中 stop mux 只是内部来源选择器；全局 arbiter 仍是最终 `/cmd_vel` 唯一 owner。模式切换、未知模式、输入陈旧、时间回退、异常和 shutdown 都先输出零速度。

## 测试与验收

DeepSeek 只做本地 TDD、README、handoff 和分阶段提交；禁止 SSH、SCP、车端编译、启停节点或控制底盘。

本地顺序：

1. 冻结两次到达 protocol 和状态顺序；
2. 冻结交接的有界重试、超时和零速度顺序；
3. 由 Codex 导入并记录车端 stop 快照；
4. 加入启动门控和任务注入；
5. 隔离速度输出；
6. 映射两阶段结果；
7. 增加 integration launch、preflight 和无运动模拟；
8. 运行第一部分全量回归与第二部分单测。

Codex 验收必须按检查点进行：无运动启动、旧栈安全退出、新栈只启动不发目标、AMCL/TF 就绪、单独实物流程、单独仿真目标流程，最后才运行连续两阶段。

## 非目标

- 不用 `ucar_delivery` 替换 stop；
- 不重写已实车跑通的 stop 运动算法；
- 不在 DeepSeek 阶段部署或声称实车通过；
- 不实现 PC/Gazebo 端仿真；“仿真目标”在本设计中指小车连续导航到语音任务中的第二个目标车间并停车。
