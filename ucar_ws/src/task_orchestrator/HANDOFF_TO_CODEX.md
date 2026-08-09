# HANDOFF_TO_CODEX：第一阶段全流程一键启动完成交接

> 交接人：OpenCode / DeepSeek V4 Flash
> 日期：2026-08-09
> 交接对象：Codex（代码审查 → 本地全量回归 → 与 915aa37 QR 和车端外部包核对 launch
> 参数 → 车端备份 → 部署编译 → 无运动 topic 模拟 → 人工看护下实车运行到 TTS 与
> delivery goal → 再交给后续避障导航开发）

## 1. 分支与最终 HEAD

```text
worktree: D:\program_sec\智能车\.worktrees\task_orchestrator
branch:   feature/task-orchestrator
HEAD:     23ab880（任务 E 文档提交后 HEAD 为下方最终提交）
起点:     86cf663
```

## 2. 每次提交及其目的

| 提交 | 内容 |
|---|---|
| `13dfac3` `test(orchestrator): lock post-speech delivery handoff contract` | TDD RED→GREEN：TTS 成功后只发布一次 `/task/delivery_navigation_goal`（五字段齐全），状态停留在新状态 `DELIVERY_HANDED_OFF`（motion mode 为 IDLE），不进入 COMPLETE、不伪造 `/task/delivery_arrived`；重复/过期/错误 speech_id 不重复发布；node 层 delivery_arrived 订阅与 cancel 状态表同步更新 |
| `13a5f83` `refactor(bringup): stop auto-starting unfinished delivery navigation` | 删除 `competition_full.launch` 的 `start_delivery`/`delivery_launch` 与 `ucar_avoid` include；删除 `start_competition.sh` 中 start_delivery flag、case 分支和 `/vision_node /racecar_control` 冲突检查；保留 `/task/delivery_navigation_goal` 发布器与协议 |
| `ac9142e` `feat(bringup): forward global QR LLM and timeout parameters` | 总 launch 新增 21 个 `qr_*` 参数（含 `qr_scan_window=1.0`、`qr_search_total_timeout=90.0`）、`llm_url`/`llm_request_timeout=90.0`、5 个阶段超时（`timeout_qr_search=150.0 > 90.0`）并显式转发；`task_orchestrator.launch` 增加私有 `<param>` 在 rosparam load 后覆盖 `timeouts/*` |
| `23ab880` `test(bringup): harden parameter and preflight contracts` | 静态锁定：case 白名单只含 `start_*` 布尔开关、全部层安全冲突检查保留；动态（Linux）验证 `qr_*`/`llm_*`/`timeout_*` 调参参数原样、逐参数边界安全转发 |
| 任务 E 提交（待做） | README 参数/交接文档更新 + `test_package_config.py` 断言反转（见第 10 节） |

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

- 179 项通过，18 项跳过，耗时约 0.5 s（Windows 无 WSL/Bash，`CompetitionStartScriptTests`
  中的 fake-ROS Bash 测试在 Windows 跳过；车端 Linux 可全量执行，命令相同）。
- 17 项跳过为任务前既有环境限制（`no working Bash available`），1 项为本阶段新增的
  `test_tuning_parameters_are_forwarded_verbatim`（同为 Bash 环境）。
- llm_spark 回归：未改动 llm_spark 包源码，本阶段未单独运行其测试（按协议不修改外部包；
  若需复核可在车端跑 `python -m unittest discover -s ucar_ws/src/llm_spark/test`）。
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
- `/task/delivery_arrived` 无发布者、无消费者（订阅保留，`expected_state=DELIVERY_HANDED_OFF`）
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

## 10. 任务 E 收尾（本会话内即将完成）

- README 已更新：终点 DELIVERY_HANDED_OFF、状态序列、五字段 delivery 消息、5.1 全局参数
  表（QR/LLM/超时 + 下游映射 + 覆盖优先级 + 30°/45° 与 1.0 s 驻留调参指引 + 重启要求 +
  rosnode cleanup 定位 + 未来避障节点订阅/去重/回传说明）。
- `test_package_config.py` 原"未转发 request_timeout"断言已反转为"已转发
  llm_request_timeout/timeout_qr_search + README 优先级说明"断言。
- 随后提交 `docs(orchestrator): document full launch tuning and delivery handoff`。

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
