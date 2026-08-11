# HANDOFF_TO_CODEX：第一阶段历史交接与第二阶段接口修复

> 第 1～12 节保留 2026-08-09 第一阶段交接记录，便于追溯；其中“第二阶段未实现”等结论已被
> 第 13 节取代。当前第二阶段修改尚未部署到小车，不能把本地测试通过表述为实车验收通过。

> 交接人：OpenCode / DeepSeek V4 Flash
> 日期：2026-08-09
> 交接对象：Codex（代码审查 → 本地全量回归 → 与 915aa37 QR 和车端外部包核对 launch
> 参数 → 车端备份 → 部署编译 → 无运动 topic 模拟 → 人工看护下实车运行到 TTS 与
> delivery goal → 再交给后续避障导航开发）

## 1. 分支与最终 HEAD

```text
worktree: D:\program_sec\智能车\.worktrees\task_orchestrator
branch:   feature/task-orchestrator
HEAD:     2fab15b
起点:     86cf663
```

## 2. 每次提交及其目的

| 提交 | 内容 |
|---|---|
| `13dfac3` `test(orchestrator): lock post-speech delivery handoff contract` | TDD RED→GREEN：TTS 成功后只发布一次 `/task/delivery_navigation_goal`（五字段齐全），状态停留在新状态 `DELIVERY_HANDED_OFF`（motion mode 为 IDLE），不进入 COMPLETE、不伪造 `/task/delivery_arrived`；重复/过期/错误 speech_id 不重复发布；node 层 delivery_arrived 订阅与 cancel 状态表同步更新 |
| `13a5f83` `refactor(bringup): stop auto-starting unfinished delivery navigation` | 删除 `competition_full.launch` 的 `start_delivery`/`delivery_launch` 与 `ucar_avoid` include；删除 `start_competition.sh` 中 start_delivery flag、case 分支和 `/vision_node /racecar_control` 冲突检查；保留 `/task/delivery_navigation_goal` 发布器与协议 |
| `ac9142e` `feat(bringup): forward global QR LLM and timeout parameters` | 总 launch 新增 21 个 `qr_*` 参数（含 `qr_scan_window=1.0`、`qr_search_total_timeout=90.0`）、`llm_url`/`llm_request_timeout=90.0`、5 个阶段超时（`timeout_qr_search=150.0 > 90.0`）并显式转发；`task_orchestrator.launch` 增加私有 `<param>` 在 rosparam load 后覆盖 `timeouts/*` |
| `23ab880` `test(bringup): harden parameter and preflight contracts` | 静态锁定：case 白名单只含 `start_*` 布尔开关、全部层安全冲突检查保留；动态（Linux）验证 `qr_*`/`llm_*`/`timeout_*` 调参参数原样、逐参数边界安全转发 |
| `2fab15b` `docs(orchestrator): document full launch tuning and delivery handoff` | README 参数/交接文档更新 + `test_package_config.py` 断言反转 |

## 3. 修改文件清单

```text
ucar_ws/src/task_orchestrator/launch/competition_full.launch
ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch
ucar_ws/src/task_orchestrator/scripts/start_competition.sh
ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py
ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py
ucar_ws/src/task_orchestrator/test/test_orchestrator.py
ucar_ws/src/task_orchestrator/test/test_competition_bringup.py
ucar_ws/src/task_orchestrator/test/test_package_config.py
ucar_ws/src/task_orchestrator/README.md
ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md（本文件）
```

未修改（尊重禁止清单）：`ucar_avoid`、`ucar_fast_nav`、`speech_command`、`llm_spark`、
`qr_item_search`、`ucar_controller`、相机、导航、底盘算法源码均未触碰；未新建/删除分支
或 worktree；未 push/merge/rebase。

## 4. 测试命令、数量与摘要

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

- 共运行 179 项，其中 161 项通过、18 项跳过，耗时约 0.5 s（Windows 无可用的 Unix Bash，`CompetitionStartScriptTests`
  中的 fake-ROS Bash 测试在 Windows 跳过；车端 Linux 可全量执行，命令相同）。
- 17 项跳过为任务前既有环境限制（`no working Bash available`），1 项为本阶段新增的
  `test_tuning_parameters_are_forwarded_verbatim`（同为 Bash 环境）。
- llm_spark 回归：Codex 复核时已运行 11 项，全部通过。
- 未执行（也禁止 DeepSeek 执行）：车端编译、部署、roslaunch 展开实测、实车运动。

## 5. 全局 launch 实际启动的每个模块

```text
competition_full.launch
  -> ucar_fast_nav/pickup_navigation.launch   底盘/雷达/地图/lidar_loc/move_base
  -> usb_cam/usb_cam_node                     唯一相机 owner（1 个）
  -> speech_command/speech_command.launch     /question、语音串口唯一 owner
  -> qr_item_search/qr_item_search.launch     只订阅图像，速度 remap /cmd_vel/qr
  -> llm_spark/llm_spark.launch               url/request_timeout 显式转发
  -> task_orchestrator.launch（4 组 include） 编排器 + voice adapter + TTS bridge
                                               + fast_nav_adapter + readiness_gate
                                               + velocity_arbiter（按 start_* 分组）
```

本阶段不启动：`ucar_avoid`、AMCL、任何避障/巡线/配送导航节点、`/cmd_vel/avoidance`。

## 6. 完整参数映射表（competition_full.launch）

| 全局参数 | 默认值 | 下游参数 |
|---|---:|---|
| `qr_image_topic` | `/usb_cam/image_raw` | `image_topic` |
| `qr_start_debug_stream` | `false` | `start_debug_stream` |
| `qr_debug_host` | `0.0.0.0` | `debug_host` |
| `qr_debug_port` | `8080` | `debug_port` |
| `qr_metrics_dir` | `$(env HOME)/qr_metrics` | `metrics_dir` |
| `qr_keyframe_dir` | `$(env HOME)/qr_keyframes` | `keyframe_dir` |
| `qr_decode_scale` | `1.5` | `decode_scale` |
| `qr_step_angle_deg` | `45.0` | `step_angle_deg` |
| `qr_cruise_angular_speed` | `0.50` | `cruise_angular_speed` |
| `qr_approach_angular_speed` | `0.20` | `approach_angular_speed` |
| `qr_approach_zone_deg` | `10.0` | `approach_zone_deg` |
| `qr_yaw_tolerance_deg` | `2.0` | `yaw_tolerance_deg` |
| `qr_settled_angular_speed` | `0.03` | `settled_angular_speed` |
| `qr_settled_duration` | `0.20` | `settled_duration` |
| `qr_scan_window` | `1.0` | `scan_window` |
| `qr_offset_angle_deg` | `22.5` | `offset_angle_deg` |
| `qr_max_passes` | `2` | `max_passes` |
| `qr_search_total_timeout` | `90.0` | `search_total_timeout` |
| `qr_settling_timeout` | `3.0` | `settling_timeout` |
| `qr_heading_timeout` | `1.0` | `heading_timeout` |
| `qr_camera_timeout` | `1.0` | `camera_timeout` |
| `llm_url` | 讯飞 Spark URL（与 llm_spark.launch 默认一致） | `url` |
| `llm_request_timeout` | `90.0` | `request_timeout` |
| `timeout_dependency_ready` | `120.0` | 私有 `timeouts/dependency_ready` |
| `timeout_pickup_navigation` | `300.0` | 私有 `timeouts/pickup_navigation` |
| `timeout_qr_search` | `150.0` | 私有 `timeouts/qr_search` |
| `timeout_llm_classification` | `120.0` | 私有 `timeouts/llm_classification` |
| `timeout_speech` | `60.0` | 私有 `timeouts/speech` |

覆盖优先级：总 launch 显式值 > `orchestrator.yaml` 默认值 > Python 内建兜底。
`timeout_qr_search=150.0` 严格大于 QR 内部 `qr_search_total_timeout=90.0`。
固定连接（不可经比赛命令改写）：`/cmd_vel/navigation`、`/cmd_vel/qr` → `velocity_arbiter`
→ 唯一 `/cmd_vel`；QR 图像默认 `/usb_cam/image_raw`。

## 7. TTS 后 delivery topic 的精确 JSON

`/task/delivery_navigation_goal`（std_msgs/String，`data` 为 JSON，仅发布一次，不 latch，
仅在匹配的 `/voice/speak_done status=success` 后发布）：

```json
{"protocol_version":1,"task_id":"task-...","goal_id":"delivery-...","target_workshop":"食品加工车间|日用品加工车间|电子产品生产车间","selected_item":"由 LLM 选出的实物名称"}
```

同一时刻 `/task/motion_mode` 为 `IDLE`，`/task/status` 状态为 `DELIVERY_HANDED_OFF`。

## 8. 明确未实现（本阶段终点，不是完成比赛）

- 避障导航（`ucar_avoid` 未启动、未 include、未授权任何底盘运动）
- `/task/delivery_arrived` 当前无发布者；编排器订阅者已保留，
  `expected_state=DELIVERY_HANDED_OFF`，供后续避障导航回传。
- 两次停车、动态避障、巡线、车间导航
- Gazebo、仿真任务、仿真完成播报
- 状态 `NAVIGATING_TO_WORKSHOP`/`COMPLETE` 在本阶段不可达（代码保留供未来阶段使用）

## 9. 当前工作树所有未跟踪/未提交文件及其归属

进入任务前已存在、未触碰、未提交（归属用户/队友）：

```text
M  ucar_ws/src/task_orchestrator/README.md      # 任务前差异见下；本阶段已按要求在其中
                                                 # 更新（进入前内容逐行说明见第 10 节）
?? docs/superpowers/plans/2026-07-24-speech-command-completeness.md
?? ucar_ws/src/ucar_avoid/HANDOFF.md
```

任务前 README 既有差异（git diff 在 86cf663 上）：3.4 节一键命令代码块后追加了
"清洗节点 / source ... / rosnode cleanup" 段落，以及一行编码损坏文本
"启动参数均为布尔值，格式 `name:=true|false`�?"（`�?` 为既有乱码，本阶段未改动）。
本阶段在保留该段落基础上补充了"`rosnode cleanup` 只清除僵尸登记、不能关闭 live 节点"
的定位说明（AGENTS.md 任务 E 第 12 条要求）。

## 10. 任务 E 收尾（已完成）

- README 已更新：终点 DELIVERY_HANDED_OFF、状态序列、五字段 delivery 消息、5.1 全局参数
  表（QR/LLM/超时 + 下游映射 + 覆盖优先级 + 30°/45° 与 1.0 s 驻留调参指引 + 重启要求 +
  rosnode cleanup 定位 + 未来避障节点订阅/去重/回传说明）。
- `test_package_config.py` 原"未转发 request_timeout"断言已反转为"已转发
  llm_request_timeout/timeout_qr_search + README 优先级说明"断言。
- 已提交 `2fab15b docs(orchestrator): document full launch tuning and delivery handoff`。

## 11. 已知外部依赖与风险

- 本工作树 QR 副本较旧；总 launch 按 915aa37 外部契约实现（`qr_item_search.launch`
  的 21 个 arg 名全部以 `$(arg qr_*)` 显式转发）。Codex 集成权威 QR 时如出现 arg 名
  漂移，先核对 915aa37 契约再改总 launch，并同步更新 README 5.1 表。
- Bash 动态测试在 Windows 本地跳过（无 WSL），需在车端 Linux 跑
  `python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v`
  确认 179 项全绿（无跳过）。
- 未验证项：roslaunch XML 展开实机行为、私有 param 对 `rospy.get_param("~timeouts")`
  的实际合并效果、QR/LLM/语音在车端的真实交互。Codex 部署后建议先
  `roslaunch task_orchestrator competition_full.launch --dry-run` 或等效方式核对展开，
  再进入无运动 topic 模拟与实车阶段。

## 12. 后续 Codex 顺序（建议）

代码审查 → 本地全量回归（车端 Linux）→ 与 915aa37 QR 及车端外部包核对 launch 参数 →
车端备份 → 部署编译 → 无运动 topic 模拟 → 人工看护下实车运行到 TTS 与 delivery goal →
再交给后续避障导航开发。

## 13. 2026-08-11 第二阶段接口修复（当前权威交接）

工作位置：

```text
worktree: D:\program_sec\智能车\.worktrees\stop-phase2-integration
branch:   codex/stop-phase2-integration
```

本轮实现提交（文档提交前）：

| 提交 | 目的 |
|---|---|
| `149be7c` | 两次停车各增加独立 TTS 等待状态；匹配成功回执后才继续/完成 |
| `efe1ec6` | supervisor 单一、latched 发布二维码领取区初始位姿；状态 topic 只保留 `ready/failed` |
| `379888f` | 航点索引延迟到当前航点 OCR 扫描耗尽后递增，消除 OCR/航点 off-by-one |

权威流程：

```text
首次比赛播报成功 -> delivery goal -> 安全切换 stop 栈
 -> 实物车间导航/OCR/停车 -> delivery_arrived
 -> 播报“已将{实物}放入{实物车间}”并等待匹配 speak_done success
 -> simulation goal -> 仿真车间导航/OCR/停车 -> simulation_arrived
 -> 播报“仿真任务已完成，已将{仿真物品}放入{仿真车间}”
 -> 等待匹配 speak_done success -> COMPLETE
```

所有外部业务接口以 `task_orchestrator` protocol v1 为准：
`/task/delivery_navigation_goal`、`/task/stop_mission_goal`、`/task/delivery_arrived`、
`/task/simulation_navigation_goal`、`/task/simulation_arrived`、`/voice/speak`、
`/voice/speak_done`、`/task/navigation_handoff_status`。`/stop/mission_event` 和
`/stop/mission_ack` 只是 stop 内部适配缝。

定位与航点：

```text
二维码领取区 /initialpose: x=-1.40219, y=-0.627908, yaw=0.053792653589793
车间航点1: (-0.812845, -2.44196)
车间航点2: ( 0.771455, -2.44196)
车间航点3: ( 1.784300, -2.43094)
```

`/initialpose` 只有 supervisor 一个 owner；其发布器为 latched。stop 集成 launch 的
`initial_pose_x/y/yaw=0/0/0` 仅用于关闭旧的重复发布路径。三个航点和 OCR/TEB/PCA 算法均未改。

本地验证结果：task_orchestrator 276 项通过、24 项因 Windows 无 Bash 跳过；stop 95 项全部
通过；llm_spark 11 项全部通过；task_orchestrator 与 stop 的 `compileall` 成功；
`competition_full.launch`、`task_orchestrator.launch`、`mission_integration.launch` 三个 XML
解析成功；`git diff --check` 通过。

未执行：小车部署、Catkin 车端编译、ROS 无运动注入、真实语音、真实导航、OCR/停车和底盘运动。
下一步必须先按用户指定位置备份并部署，再在人工看护和急停可用条件下分段验收。

## 14. 第三阶段交接（2026-08-11）

> 本节由第三阶段实施写入；最终测试数量、最终 HEAD 与提交清单在任务 9 全量回归后更新。

### 14.1 范围与来源

第三阶段新增 `line_follow_integration` 包（相机适配、导航适配、巡线监管器）并扩展
`task_orchestrator` 状态机与仲裁器。四个冻结源脚本逐一从权威来源导入并核对 SHA-256
（见 `SOURCE_SNAPSHOT.sha256`）：三个 V4 巡线脚本来自本地 `E:\follow_v1\follow_v1`，
`yolo_server.py` 来自小车 `/home/ucar/ucar_ws/src/car_server/yolo_server.py`（只读
SSH 取得）。未导入 `auto_drive_v3.py`（含 TTS 与假成功逻辑），未导入
`start_all_yolo.launch`。YOLO 模型不提交 Git：

```text
/home/ucar/ucar_ws/src/yolo_turn/best.pt
sha256: cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd
```

### 14.2 公共接口与速度所有权

```text
/task/line_navigation_goal    发布（pose 来自 phase3.yaml）
/task/line_navigation_arrived 订阅（arrived|failed）
/task/line_follow/start       发布
/task/line_follow/status      订阅（waiting_signal|direction_selected|following|success|failure）
/task/motion_mode             LINE_FOLLOW 只放行 /cmd_vel/line_follow
/cmd_vel/line_follow          supervisor 唯一发布者
```

速度链：巡线脚本 `/cmd_vel` remap 到 `/line_follow/cmd_vel_candidate`，supervisor
图像健康门控（0.5s 阻断 + 3s 恢复）后发布 `/cmd_vel/line_follow`，velocity arbiter
在 `LINE_FOLLOW` 模式下放行。外层超时 `line_navigation=310/line_direction=35/
line_follow=125` 严格大于内部 300/30/120 秒。

### 14.3 配置文件

```text
仓库相对: ucar_ws/src/line_follow_integration/config/phase3.yaml
本地绝对: D:\program_sec\智能车\.worktrees\phase3-line-follow-integration\ucar_ws\src\line_follow_integration\config\phase3.yaml
小车部署: /home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml
```

### 14.4 子进程与故障纪律

supervisor 只终止自己 `subprocess.Popen` 创建的 PID；禁止 `pkill`/`killall`/
`rosnode kill`。取消/异常/超时/关闭先零速度再结束子进程，最后发布一次关联失败。
只有本次新鲜停车标记是成功；子进程提前退出与 120s 超时失败；“任务完成”只由
task_orchestrator 播报，匹配回执后进入 COMPLETE。

### 14.5 测试与部署状态

本地 TDD 任务 1～9 全部完成（每任务 RED→GREEN→单独提交，最终证据见 14.6）。
**尚未部署、未 catkin 编译、未实车验收**；车端下一步顺序：审查 diff → 备份车端包 →
部署到 `/home/ucar/ucar_ws/src` → catkin build → 无运动 topic 模拟 → 分级看护实车验收。

### 14.6 最终验证证据（任务 9，2026-08-11）

分支：`codex/phase3-line-follow-integration`
最终 HEAD：`37ae6724965c84eca6ef8e9ceb196a4a1758af9e`
起点：`5c7d82c`（交接说明），基线祖先 `7b61069` 已校验。

提交序列（7b61069..HEAD 共 8 个实施提交）：

```text
c3dc1ae feat(phase3): add line-follow package configuration
4bd6c63 feat(phase3): add line-camera transform
23f6faf feat(phase3): define line-follow protocol and runtime
69ec0b4 feat(phase3): supervise navigation and proven line routes
804a48c feat(orchestrator): sequence phase3 navigation and line follow
d2db4bc feat(orchestrator): isolate line-follow velocity ownership
ab80a47 feat(bringup): include safe phase3 line-follow stack
37ae672 docs(phase3): add tuning and staged vehicle acceptance
```

测试命令与结果（本机 Python 3.13.2）：

```text
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py"   -> 50 tests OK
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py"          -> 319 tests OK
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py"                       -> 113 tests（见下方环境说明）
python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py"                  -> 11 tests OK
python -m compileall -q ucar_ws/src/line_follow_integration ucar_ws/src/task_orchestrator ucar_ws/src/stop  -> 0
XML 解析（phase3/task_orchestrator/competition_full launch）                                -> 0
YAML 解析（phase3.yaml）                                                                    -> 0
四个导入脚本 SHA-256 逐一核对                                                               -> 匹配
git diff --check                                                                           -> 0
```

源文件与模型 SHA-256：

```text
follow_left_v4.py   8a9471e5917b93bdddd1f0b191e8ace48fe4d6baa8269f68204d4fba23fbbfb3
follow_right_v4.py  92fd5098be423bab61c3eb5deb5c202c12adbc6c45bc4638a1af7bf5931a5d73
follow_mid_v4.py    736d0666475f25f48f1e11e888a6c40864af95496754ff37f6311fbba3d0b07e
yolo_server.py      9d7010328a636742c1c60012cd6c4f0c78ae3cfc2ab4cf57c815ec88de4e9a51
best.pt              cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd
```

环境说明（必须告知 Codex）：本仓库 `core.autocrlf=true`，Windows 检出会把 LF 冻结
快照文件污染为 CRLF，导致工作树中 stop 的 `test_frozen_snapshot_files_match_vehicle_manifest`
5 项失败（stop 自 `7b61069` 无任何改动，已用 `git diff 7b61069 HEAD -- ucar_ws/src/stop`
验证为空）。已用 `git -c core.autocrlf=false archive` 提取 LF 版本验证：**stop 113 项
全部通过**，且五个快照 blob 哈希与 `VEHICLE_SNAPSHOT.sha256` 逐一一致。部署/CI 在
Linux（LF）上不受影响。

未验证的实车风险：

- 中心裁剪（960x720→640x480）对真实巡线输入的影响未实测；
- YOLO 在实车相机曝光下的 `red_light/straight/left_turn/right_turn` 置信度未实测；
- 相机适配器 15 FPS 限流、图像健康门控与速度链在真实频率下的行为未实测；
- `start_competition.sh` 的 YOLO 模型预检在车端路径（`/home/ucar/...`）未实测；
- 三个 V4 巡线脚本的 PID/旋转/停车参数在实车场地未复测。

部署顺序（下一步，Codex 执行）：代码审查 → 全量回归 → 车端备份 →
部署到 `/home/ucar/ucar_ws/src` → catkin build → 无运动 topic 模拟 →
分级看护实车验收（导航起点 → 图像验证 → 方向锁定 → 单放 `/cmd_vel/line_follow`
→ 最终停车线 → 全流程）。

### 14.7 Codex 验收修正（2026-08-11）

Codex 在 `a2d43379e050a817a42a66262c65815746302d0f` 上复核后发现并修正：

- Phase 3 两个节点把 `/task/cancel` 错当成必须含 `goal_id` 的消息，导致合法取消被忽略；
- supervisor 以子进程字典非空代替 `poll()` 活性判断，子进程提前退出后可能等待到外层超时；
- 新任务未清空上次图像门禁，且派生图像不新鲜时仍可能启动巡线；
- YOLO/巡线 `Popen` 启动异常未转换为关联失败；方向等待期间 YOLO 退出未立即失败；
- `WAITING_LINE_DIRECTION` 未立即处理关联 `failure`；
- 一键启动预检未拒绝遗留的 Phase 3 常驻节点、YOLO 和匿名巡线子进程；
- Windows `core.autocrlf=true` 会改变五个 stop 冻结快照的工作树字节，已用包内
  `.gitattributes` 固定这些文件为 LF，不修改车端快照内容或清单。

修正后全量本地回归：`line_follow_integration 61`、`task_orchestrator 322`
（其中 31 项动态 Bash 测试因本机无可用 Bash 跳过）、`stop 113`、`llm_spark 11`。
仍未执行车端 catkin 编译、ROS 运行时 topic 模拟或实车运动验收。
