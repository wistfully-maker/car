# HANDOFF_TO_CODEX —— Stop 第二阶段联调（DeepSeek 本地实现交接）

> 生成时间：2026-08-10。本文件是 DeepSeek 本地实现的完整交接证据；
> 不包含任何实车部署、编译或运动结果。实车验收由 Codex 按第 10 节检查点逐个单独授权。

## 1. 工作位置与 HEAD

- worktree：`D:\program_sec\智能车\.worktrees\stop-phase2-integration`
- branch：`codex/stop-phase2-integration`
- HEAD：`b6d9bcfbd58849d9c2be1a0810887e87b8338295`
- 基线祖先：`77e853a767152d7e877b736af6390605c841e6d2`（`git merge-base --is-ancestor` 通过）
- 工作区状态：干净（无未提交改动）

## 2. 提交清单（基线 77e853a 之后，按时间顺序）

| commit | 消息 | 内容 |
|---|---|---|
| `abd7a0c` | docs: define stop phase two integration contract | 设计契约（Codex 先行） |
| `9e1d5dc` | docs: plan stop phase two integration | 实施计划（Codex 先行） |
| `6d06802` | chore(stop): import verified vehicle package snapshot | 车端 stop 快照独立导入（Codex 先行，含清单与哈希） |
| `8b4fc5b` | feat(orchestrator): coordinate dual workshop arrivals | 编排器双车间契约：`NAVIGATING_TO_SIM_WORKSHOP`、`/task/simulation_navigation_goal`、`/task/simulation_arrived`，移除旧 sim_trigger 桩 |
| `3c2ffa8` | feat(handoff): model bounded navigation stack transition | ROS 无关交接状态机（IDLE→…→READY，有界重试、近零验证、总时限、状态门控） |
| `4c48467` | feat(handoff): supervise controlled navigation stack switch | 交接 supervisor 节点：双 launch 句柄、fake-ROS 测试、YAML 配置 |
| `302c6f1` | feat(stop): gate vehicle mission on correlated task input | stop 车缝：characterization 冻结 + MissionGate + 激活/取消 seam + 移除自动启动与内部 TTS |
| `a468174` | feat(stop): isolate vehicle navigation and manual velocity | stop mux（/cmd_vel/stop 唯一输出）+ mission 手动速度 → /cmd_vel/stop_manual + /stop/motion_mode |
| `15837c9` | feat(stop): report correlated dual parking results | stop 协议适配器：phase1_done/done/failed → protocol v1 双到达；Phase 2 等仿真目标放行 |
| `da0fd68` | feat(arbiter): authorize isolated stop velocity source | STOP_NAVIGATION 模式 → 仅转发 /cmd_vel/stop；交接放行时 supervisor 发布模式 |
| `4766b1f` | feat(bringup): compose controlled stop stack handoff | mission_integration.launch、legacy_navigation_include.launch、根 launch 硬件/交接开关、preflight（stop 模型哈希、节点冲突） |
| `b6d9bcf` | test(integration): cover stop phase two failure scenarios | 16 个离线端到端场景 + 无运动人工手册 + handoff 状态门控 |

## 3. 修改文件清单（本轮 8b4fc5b..b6d9bcf）

task_orchestrator：
- `src/task_orchestrator/orchestrator.py`、`protocol.py`、`motion_mode.py`、`handoff.py`、`velocity_arbiter.py`
- `scripts/task_orchestrator_node.py`、`velocity_arbiter_node.py`、`navigation_handoff_supervisor_node.py`、`start_competition.sh`
- `launch/competition_full.launch`、`task_orchestrator.launch`、`legacy_navigation_include.launch`（新增）
- `config/orchestrator.yaml`、`CMakeLists.txt`
- `test/`：test_orchestrator、test_protocol、test_motion_mode、test_package_config、test_handoff（新增）、
  test_navigation_handoff_supervisor_node（新增）、test_velocity_arbiter、test_velocity_arbiter_node、
  test_competition_bringup、test_stop_phase2_scenarios（新增）、manual_stop_phase2_simulation.md（新增）

stop：
- `scripts/mission_orchestrator.py`（车缝）、`scripts/stop_velocity_mux_node.py`（新增）、
  `scripts/stop_protocol_adapter_node.py`（新增）
- `src/stop_integration/__init__.py`、`mission_gate.py`、`velocity_mux.py`（新增）
- `launch/mission_integration.launch`（新增）、`setup.py`（新增）、`CMakeLists.txt`
- `test/`：test_vehicle_characterization、test_mission_gate、test_velocity_mux、
  test_velocity_mux_node、test_protocol_adapter_node、test_integration_launch（均新增/扩展）

## 4. 测试结果（本地，2026-08-10）

| 包 | 通过 | 跳过 | 错误 | 说明 |
|---|---:|---:|---:|---|
| task_orchestrator | 232 | 3 | 32 | 32 个错误全部是 `CompetitionStartScriptTests` 在 Windows 无管理员权限下 `os.symlink` 失败（WinError 1314），Linux 上可运行；含 8 个新增交接/preflight 运行时测试 |
| stop | 90 | 0 | 0 | 全绿 |
| llm_spark | 11 | 0 | 0 | 全绿（未改动，回归） |

`python -m compileall -q ucar_ws/src/task_orchestrator ucar_ws/src/stop` 通过；
`git diff --check` 通过；全部新增/修改 launch 均用 `xml.etree.ElementTree.parse` 通过。
注意：**Windows 上的 32 个错误是环境限制，不代表测试失败**；Linux（车端）环境应全绿。

## 5. 车端 stop 快照

- 来源：`ucar@192.168.1.109:/home/ucar/ucar_ws/src/stop`（Codex 导入）
- 清单：`ucar_ws/src/stop/VEHICLE_SNAPSHOT.sha256`
- 冻结文件（字节级校验，测试锁定）：`scripts/models/*.rknn`（ppocrv4_det/rec）、
  `scripts/models/ppocr_keys_v1.txt`、`scripts/ocr/*`、`scripts/ocr_native_node.py`、
  `scripts/precision_park.py`、`scripts/scan_and_park.py`、`msg/*`
- 已改造（允许）：`scripts/mission_orchestrator.py`、`CMakeLists.txt`、新增文件
- 与车端不一致的已知差异：mission_orchestrator.py（车缝）、CMakeLists.txt（catkin_python_setup 等）、
  package.xml 未改动（依赖已齐）

## 6. 运行时 owner 映射（交接前/中/后）

| 资源 | 交接前（第一阶段） | 交接中 | 交接后（stop 栈） |
|---|---|---|---|
| move_base | 旧（fast-nav include，supervisor 持有） | 取消目标→停止进程组 | stop 栈 move_base（/cmd_vel/stop_navigation） |
| 定位 | /lidar_loc | 停止 | /amcl（+stop map_server） |
| map | 旧 map_server | 停止 | stop map_server |
| base/lidar/camera/speech | 根 launch 一次性启动（交接模式） | 不动 | 不动（stop 栈不含硬件） |
| 最终 /cmd_vel | competition_velocity_arbiter | 同（IDLE→零） | 同（STOP_NAVIGATION 仅转发 /cmd_vel/stop） |
| stop 内部 | — | — | stop_velocity_mux（/cmd_vel/stop 唯一发布者） |

监督者只终止自己启动并持有的两个 roslaunch 进程组；禁用 `pkill ros`、`rosnode kill -a`、
节点名模糊 kill。shutdown 时停止两个持有组并发零速度，不发到达。

## 7. 全部 topic/remap

| topic | 方向 | 用途 |
|---|---|---|
| /task/delivery_navigation_goal | 编排器→交接 | TTS 后唯一一次交接触发 |
| /task/navigation_handoff_status | 交接→诊断 | protocol v1 状态/重试/失败原因 |
| /task/stop_mission_goal | 交接→stop | READY 后放行（task_id/goal_id/车间/货品） |
| /task/delivery_arrived | stop→编排器 | 实物停车确认（只发一次成功） |
| /task/simulation_navigation_goal | 编排器→stop | 仿真目标（实物停车后发布） |
| /task/simulation_arrived | stop→编排器 | 仿真停车确认（只发一次成功） |
| /stop/mission_event | stop 内部 | phase1_done/done/failed:* |
| /stop/mission_ack | stop 内部 | action=start_phase2 放行 |
| /stop/motion_mode | stop 内部 | IDLE/NAVIGATION/MANUAL（stop mux 模式） |
| /cmd_vel/navigation | fast-nav | 第一部分来源（remap） |
| /cmd_vel/qr | QR | 来源（remap） |
| /cmd_vel/stop | stop mux | stop 栈唯一输出 |
| /cmd_vel/stop_navigation | stop move_base | param cmd_vel_topic |
| /cmd_vel/stop_manual | stop mission | 手动/PCA/停车/倒车 |
| /cmd_vel | 仲裁器 | 全局唯一最终输出 |

## 8. 参数映射（默认值）

- `navigation_handoff`：cancel_retries=3、cancel_timeout=3.0、stop_stable_duration=0.75、
  linear_stop_threshold=0.02、angular_stop_threshold=0.05、odom_max_age=0.5、
  legacy_exit_retries=5、legacy_exit_timeout=10.0、readiness_retries=30、
  readiness_poll_period=1.0、total_timeout=90.0
- `navigation_handoff_supervisor`：legacy_nav_launch/stop_integration_launch（根 launch arg 覆盖）、
  legacy_nodes=[/lidar_loc,/move_base,/map_server]、初始位姿 frame/map + x/y/yaw/covariance、
  amcl_node=/amcl、ocr_node=/ocr_native_node、image_topic=/usb_cam/image_raw
- stop `mission_integration.launch`：initial_pose_x/y/yaw=-0.813/-2.442/0.0、max_rotations=6、
  target_distance=0.20、x_align_tolerance=0.50、y_align_tolerance=12、auto_start_delay=3.0、
  phase2_ack_timeout=60.0（全部与车端已验证 mission.launch 默认一致）
- `velocity_mux`：source_timeout=0.3、check_period=0.05、max_linear_abs=1.0、max_angular_abs=2.0
- 根 launch 新开关：`start_navigation_handoff:=true|false`、`start_stop_stack:=true|false`

## 9. 已知假设与未完成

- 未部署、未车端编译、未实车运行；所有"到达"均为本地测试注入；
- 假设 stop 栈 AMCL 节点名 `/amcl`、OCR 节点名 `/ocr_native_node`、地图
  `$(find ucar_nav)/maps/map.yaml`、起点 -0.813/-2.442/0.0（与车端 mission.launch 一致）；
- stop mux 的模式由 mission 发布（/stop/motion_mode），未验证 move_base 在目标完成后的
  残余输出行为（mux 的"错误来源忽略"依赖模式门控）；
- 交接 supervisor 的就绪探测包含相机（/usb_cam/image_raw）与 OCR 节点活跃性，
  具体阈值待车端确认；
- `start_stop_stack:=false`（外部 stop 栈）路径只做了空 launch 处理与就绪验证，
  未做外部部署实测；
- Windows 本地的 32 个 symlink 环境错误需在 Linux 重新运行确认。

## 10. Codex 车端验收检查点（每关单独授权）

```text
1. 代码审查
2. 本地全量回归（Linux）
3. 无运动启动（旧栈在、stop 栈在、不发任何目标、/cmd_vel 唯一 owner）
4. 第一导航栈安全退出（交接 FAILED/READY 前旧栈保持；仅 supervisor 持有组被停）
5. stop 导航栈只启动不发目标（无 /task/stop_mission_goal 前零运动）
6. AMCL/TF/readiness 就绪门
7. 单独实物导航停车（/task/delivery_arrived 一次）
8. 单独仿真目标导航停车（/task/simulation_arrived 一次）
9. 连续两阶段（不提前 COMPLETE，第二次停车后才完成）
```

DeepSeek 本地测试通过**不等于**已部署、已实车通过或已完赛。
