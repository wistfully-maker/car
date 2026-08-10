# stop 包第二阶段集成说明（DeepSeek 本地实现，非实车验收）

> 本文件说明车端已验证的 `stop` 包如何以最小改造接入全局编排器。
> 车辆算法（FIND_POINTS_LIST 航点、AMCL+TEB+move_base、OCR 车间识别、
> Stage 0-6 停车顺序、Phase 1→2 倒车/掉头/清代价地图/双指针）全部保留；
> 集成只增加启动门控、任务注入、速度隔离、阶段事件与结果映射。
> 实车验收由 Codex 按 `HANDOFF_TO_CODEX.md` 检查点逐个单独授权。

## 1. 车端快照与冻结

- 来源：`ucar@192.168.1.109:/home/ucar/ucar_ws/src/stop`（Codex 导入，DeepSeek 禁止 SSH/SCP）；
- 快照清单：`VEHICLE_SNAPSHOT.sha256`（每个文件的 SHA-256，车端独立读取后本地复核）；
- 冻结对象：`scripts/models/*.rknn`、`scripts/models/ppocr_keys_v1.txt`、
  `scripts/ocr/*`、`scripts/ocr_native_node.py`、`scripts/precision_park.py`、
  `scripts/scan_and_park.py`、`msg/*`——字节级校验见
  `test/test_vehicle_characterization.py`，改动即失败；
- 允许改造：`scripts/mission_orchestrator.py`（集成车缝）、新增 `src/stop_integration/`、
  `scripts/stop_velocity_mux_node.py`、`scripts/stop_protocol_adapter_node.py`、
  `launch/mission_integration.launch`、`setup.py`、`CMakeLists.txt`。

## 2. 集成拓扑

```text
/task/stop_mission_goal -------------------------> mission_orchestrator（门控激活）
/task/cancel ------------------------------------> mission_orchestrator（取消）
/stop/mission_ack（Phase 2 放行） --------------> mission_orchestrator

mission_orchestrator --手动/PCA/停车速度--> /cmd_vel/stop_manual
stop move_base ----------------------------> /cmd_vel/stop_navigation
stop_velocity_mux_node ---------------------> /cmd_vel/stop（唯一 stop 输出）

mission_orchestrator --/stop/mission_event--> stop_protocol_adapter_node
stop_protocol_adapter_node --/task/delivery_arrived--> 全局编排器
                          --/task/simulation_arrived--> 全局编排器
```

- 最终 `/cmd_vel` 仍只有全局 `competition_velocity_arbiter` 发布；
- 公共硬件（base/lidar/camera/speech）不在本包启动；
- stop 栈与第一部分导航栈由 `navigation_handoff_supervisor` 独占持有，不同时运行。

## 3. 车缝行为（mission_orchestrator.py）

- **不自动运动**：启动后订阅 `/task/stop_mission_goal`（protocol v1 JSON），
  `MissionGate` 校验身份/去重/终态缓存后经 `activate()` 注入车间与货品目标；
  重复活动任务不重启，重复终态任务只重发缓存结果；
- **取消**：`/task/cancel`（匹配活动 task_id）→ 停止搜索、`/move_base/cancel` 取消目标、
  零速度、只发一次 `failed:cancelled`；
- **Phase 2 放行**：实物停车后停在原地等待全局编排器的仿真目标；
  `/stop/mission_ack`（action=start_phase2）到达才执行原有 `switch_to_phase2()`
  （倒车 0.5m → 180° 掉头 → 清代价地图 → 双指针跳转），超时
  `phase2_ack_timeout` 秒报 `failed:phase2_timeout`；
- **TTS 移除**：`mission_done()` 不再直接调 `speak()`，语音统一由全局编排器处理；
- **速度模式**：`_set_mode()` 把 NAVIGATION/MANUAL/IDLE 发布到 `/stop/motion_mode`
  供 stop mux 使用（旋转、逼近、微调、倒车为 MANUAL；move_base 目标为 NAVIGATION）。

## 4. 速度隔离（stop_integration/velocity_mux.py + stop_velocity_mux_node.py）

- 模式：`IDLE / NAVIGATION / MANUAL`；`/cmd_vel/stop_navigation`（move_base）与
  `/cmd_vel/stop_manual`（mission）中只有当前模式来源的新鲜消息被转发到
  `/cmd_vel/stop`；
- 模式切换、未知模式、输入陈旧、NaN/Inf/越界、时间回退、异常、shutdown 一律先发零速度；
- mux 只发布 `/cmd_vel/stop`，绝不自称最终 `/cmd_vel` owner。

## 5. 结果映射（stop_protocol_adapter_node.py）

- 订阅 `/task/stop_mission_goal`、`/task/simulation_navigation_goal`、
  `/stop/mission_event`；发布 `/task/delivery_arrived`、`/task/simulation_arrived`、
  `/stop/mission_ack`；
- 第一次停车确认（phase1_done）后才发布实物到达；第二次停车确认（done）后才发布
  仿真到达；失败（`failed:*`）带非空 message 且绝不同时发布成功；
- 重复终态事件只重发缓存结果，不重启任务；错误身份一律忽略；
- Phase 2 暂停在 `phase1_done` 之后，直到匹配的 `/task/simulation_navigation_goal`
  到达才放行。

## 6. 集成 launch（mission_integration.launch）

只启动 stop 栈自身节点：map_server、AMCL、move_base（`cmd_vel_topic` 已配置为
`/cmd_vel/stop_navigation`）、ocr_native_node、mission_orchestrator、
stop_velocity_mux、stop_protocol_adapter。**不启动** base/lidar/camera/speech。
参数（默认值与车端已验证 `mission.launch` 一致）：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `initial_pose_x/y/yaw` | -0.813 / -2.442 / 0.0 | map 系起点 |
| `max_rotations` | 6 | OCR 旋转上限 |
| `target_distance` | 0.20 | PCA 停车目标距离 |
| `x_align_tolerance` | 0.50 | X 轴直行阈值 |
| `y_align_tolerance` | 12 | Y 轴横移阈值（度） |
| `auto_start_delay` | 3.0 | 初始位姿等待 |
| `phase2_ack_timeout` | 60.0 | Phase 2 放行等待上限 |

由 `navigation_handoff_supervisor` 在交接 READY 时启动；shutdown 时由它整体停止。

## 7. 本地验证

```powershell
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/stop
```

- `test_vehicle_characterization.py`：冻结算法标记与模型字节；
- `test_mission_gate.py`：身份/去重/终态缓存/仿真目标合并；
- `test_velocity_mux.py` / `test_velocity_mux_node.py`：隔离与 fail-closed；
- `test_protocol_adapter_node.py`：双到达契约与缓存重发；
- `test_integration_launch.py`：launch 组成与参数默认值。

## 8. 未完成与边界

- 本文件描述的只是本地实现与测试；**没有部署、没有实车运行**；
- 车端编译、部署、无运动启动、旧栈退出、AMCL/TF 就绪、单独实物/仿真、连续两阶段
  均需 Codex 逐个检查点单独授权；
- 已知假设：stop 栈 AMCL 节点名为 `/amcl`、OCR 节点名为 `/ocr_native_node`、
  地图为 `$(find ucar_nav)/maps/map.yaml`、初始位姿默认 -0.813/-2.442/0.0，
  均可在 launch/参数中覆盖，但改变即偏离车端已验证配置。
