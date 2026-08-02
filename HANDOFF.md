# U-CAR 编排与 fast-nav 集成交接

更新时间：2026-08-02。工作树：`D:\program_sec\智能车\.worktrees\task_orchestrator`。
当前内容为本地实现，**待部署**；不得写成已在 `ucar@172.20.10.4` 完整实车通过。

## 1. 提交链与当前基线

从旧业务链到当前运动安全集成的关键提交：

```text
74fda2d  speech_command 完整指令与重复唤醒
bd048a5  明确二维码后运动边界
035f515  ucar_fast_nav 集成设计
70c6986  fast-nav adapter core
395ea7d  fast-nav ROS adapter
8922432  readiness gate（lidar_loc）
6361016  NAVIGATION/QR_SEARCH 速度仲裁
199cc4c  competition_full.launch
5b628bb  start_competition.sh 安全 preflight（当前文档基线）
```

本次文档任务修改 README、manual simulation、HANDOFF 和文档契约测试，不提交。

## 2. 当前架构与不可越过的终点

唯一推荐入口是 `task_orchestrator/scripts/start_competition.sh`：先检查 secret、ROS Master、
同名 live/stale、设备占用、`/amcl`、`/cmd_vel` owner，再 exec
`competition_full.launch`。总 launch 常驻启动 fast-nav、相机、语音、QR、LLM、业务编排、
fast-nav adapter、readiness 和 velocity arbiter。

运动所有权只有三态：

```text
NAVIGATING_TO_PICKUP -> NAVIGATION -> /cmd_vel/navigation
WAITING_QR           -> QR_SEARCH -> /cmd_vel/qr
其他全部状态          -> IDLE      -> 零速度
```

新定位是 `jie_ware/lidar_loc`，不是 AMCL；readiness 要求 `/lidar_loc`、地图、scan/odom、TF、
`/move_base` action 和 planner 健康，并拒绝 `/amcl` 并存。

真实自动业务终点是 TTS done。编排器之后仍发布兼容的 delivery goal 并等待 arrival，但运动模式
为 `IDLE`，当前没有 delivery/obstacle adapter。明确 **不接动态避障**，也不接二维码后巡线、
车间导航或未知航点；未来会话不得看到 delivery goal 就补一个伪运动实现。

## 3. 外部契约（任务 5）

仓库内没有 `ucar_fast_nav` 源包；任务 5 的总 launch 按已核验外部契约使用它：

- `ucar_fast_nav/launch/pickup_navigation.launch` 接受 `start_base`、`start_lidar`、
  `cmd_vel_topic`；
- 外部 launch 唯一加载 `ucar_fast_nav/config/pickup_goal.yaml`，参数在
  `/ucar_fast_nav/pickup_goal`；本包不得重复加载；
- 外部提供 `/map`、`/scan`、`/odom`、`map->odom->base_link->laser_frame`、`/lidar_loc`、
  `/move_base` 和 planner 参数；
- `speech_command.launch` 无可传 arg，发布 `/question`；QR launch 接 `image_topic`，不启动
  相机/底盘；`llm_spark.launch` 提供分类节点；`usb_cam` 是共享相机唯一 owner；
- vendor bundle 中即使有 `dynamic_obstacle`，也只是归档依赖，禁止纳入当前验收。

部署端若这些包或接口不匹配，应让 roslaunch fail-fast 并修正部署，不要删除检查或偷偷改成
旧 `ucar_nav`/AMCL。

## 4. 未完成任务与任务 8

任务 8 尚未执行，至少包括：

1. SSH `ucar@172.20.10.4`，备份现场修改并部署；
2. `catkin_make`、重新 source，核对脚本 executable 和所有外部包；
3. 安全创建 Spark 单行 secret，owner 正确且 `chmod 600`，无 CRLF；
4. 静态检查设备、唯一 owner、地图/初始位姿、`lidar_loc`、TF/action/planner；
5. 架空轮或低速有人看护，验证模式切换和归零；
6. 实车验证取货点 fast-nav、QR 旋转、LLM、TTS；
7. TTS 后必须看到 delivery message 但 `/task/motion_mode=IDLE`、`/cmd_vel=0`；
8. Ctrl+C 根 launch，确认其子节点回收且不误杀外部节点。

验收不包括二维码后的动态避障、巡线或配送。不得宣称“完整比赛路线通过”。

## 5. 任务 17 / Bash skip 注意事项

Windows 上的纯 Python 全包测试能覆盖 launch 文本、adapter/core 和文档契约，但
`test_competition_bringup.py` 中依赖 Bash、`stat/od/awk/fuser/lsof` 语义的测试可能按平台条件
skip；任务 17 的 Bash skip 不能当作 preflight 已验证。最终必须在 Linux/车端用 Bash 运行该
测试或等价检查，并记录 skip 数与原因。不要为了让 Windows 全绿而把安全脚本改写成 PowerShell。

## 6. 下一会话的安全操作原则

- 三入口中只把 `start_competition.sh` 当正式入口；直接 full launch 仅诊断，业务 launch 不能跑全车；
- 所有节点由一个 root launch 常驻；外部模块已启动时对应 `start_*:=false`，禁止重复同名节点；
- 外部仲裁器模式必须已有 ROS Master 且 `/cmd_vel` 恰好一个已确认 owner；
- 参数由 root 启动时统一加载；大多数节点不动态重读。要换参数，先 Ctrl+C 整个 root，再重启；
- 停阶段只归零/释放模式，不 kill；异常先 `rosnode list/ping/info`、`rostopic info` 确认 owner；
- `start_competition.sh` 故意不自动 kill 节点、不清 stale、不抢设备；不要削弱这些失败条件；
- 详细操作者命令以 `ucar_ws/src/task_orchestrator/README.md` 和
  `test/manual_simulation.md` 为准。
