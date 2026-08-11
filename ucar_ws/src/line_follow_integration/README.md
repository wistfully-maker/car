# line_follow_integration：第三阶段红绿灯识别与巡线联调

本包实现第三阶段：仿真车间播报成功 → 导航到 YAML 配置的巡线起点 → 停车等待红绿灯/
方向识别 → 按 `left_turn` / `right_turn` / `straight` 执行对应巡线程序 → 检测最终
停车线并停车 → 由 `task_orchestrator` 单独播报“任务完成”。

```text
WAITING_SIMULATION_SPEECH
 -> NAVIGATING_LINE_START
 -> WAITING_LINE_DIRECTION
 -> LINE_FOLLOWING
 -> WAITING_FINAL_SPEECH
 -> COMPLETE
```

**只有最终停车线检测才是成功**（且必须是本次激活后产生的新鲜完成标记）；子进程提前
退出、图像长期不新鲜或 120 秒超时都失败，不得伪造成功。

## 1. 配置文件与绝对路径

仓库相对路径：

```text
ucar_ws/src/line_follow_integration/config/phase3.yaml
```

本地开发绝对路径：

```text
D:\program_sec\智能车\.worktrees\phase3-line-follow-integration\ucar_ws\src\line_follow_integration\config\phase3.yaml
```

部署到小车后的绝对路径：

```text
/home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml
```

现场调整第三阶段巡线起点、相机参数或超时只修改此 YAML 并**重启根 launch**；不得在
Python 中硬编码 pose。每次运行只改一个参数，验证通过后再改下一个。

## 2. 数据流与速度隔离

物理相机保持二维码已调好的唯一配置（1020x720），本包不拥有、不重配、不重启相机：

```text
/usb_cam/image_raw 1020x720
  |-- QR：继续使用原始图像
  |-- YOLO：使用原始图像，保留完整视野（红绿灯/方向标识）
  `-- line_camera_adapter：中心裁剪 960x720 -> 缩放 640x480 -> 15 FPS
        -> /line_follow/image_raw -> 巡线子程序
```

YOLO 使用原始图像（不裁剪，避免丢失画面边缘的灯或方向标识）；巡线程序使用派生的
640x480 图像。

巡线脚本不得直接发布最终 `/cmd_vel`：

```text
巡线脚本 /cmd_vel
  --remap--> /line_follow/cmd_vel_candidate
  --phase3_supervisor 图像健康门控--> /cmd_vel/line_follow
  --velocity_arbiter, mode=LINE_FOLLOW--> /cmd_vel
```

只有 `line_follow_supervisor` 可以发布 `/cmd_vel/line_follow`；只有全局 velocity
arbiter 可以发布最终 `/cmd_vel`。**不得与原始 `start_all_yolo.launch` 同时运行**，
它会重复启动底盘和相机，破坏唯一 owner。

## 3. 公共接口

```text
/task/line_navigation_goal     std_msgs/String JSON  导航目标（含 pose）
/task/line_navigation_arrived  std_msgs/String JSON  arrived|failed
/task/line_follow/start        std_msgs/String JSON  开始方向识别
/task/line_follow/status       std_msgs/String JSON  waiting_signal|direction_selected|following|success|failure
/task/motion_mode              std_msgs/String       IDLE|NAVIGATION|QR_SEARCH|STOP_NAVIGATION|LINE_FOLLOW
/cmd_vel/line_follow           geometry_msgs/Twist   隔离的巡线速度来源
```

所有 JSON 使用 `protocol_version=1`，按 `task_id/goal_id` 关联、去重并拒绝过期事件。
YOLO 输出类名：`red_light`、`straight`、`left_turn`、`right_turn`（`red_light` 红灯
保持等待）。巡线状态枚举：

```text
waiting_signal     红灯等（继续等待，不发送巡线速度）
direction_selected 已锁定方向（带 direction）
following          巡线中（带 direction）
success            最终停车线新鲜标记
failure            失败（带 reason）
```

`/tmp/yolo_result.txt` 与 `/tmp/stop_done.txt` 只作为包内兼容接口；supervisor 每次
激活前删除旧文件，只接受修改时间晚于本次激活的新文件。

## 4. 超时与故障语义

| 参数 | 默认值 | 语义 |
| --- | --- | --- |
| navigation | 300.0s | 导航到巡线起点内部超时 |
| image_ready | 5.0s | 原始图像未在 5 秒内就绪则失败 |
| image_max_age | 0.5s | 派生图像超过 0.5 秒未更新立即阻断速度 |
| image_recovery_grace | 3.0s | 3 秒恢复窗口，仍未恢复才失败 |
| direction | 30.0s | **30 秒无方向结果时选择 straight**（不是错误） |
| line_follow | 120.0s | 巡线超时：零速度、结束子进程并失败 |

外层 `task_orchestrator` 超时 `line_navigation=310.0`、`line_direction=35.0`、
`line_follow=125.0`，严格大于本包内部对应值，避免外层先行超时。

失败原因（`reason`）含义：

```text
yolo model missing or sha256 mismatch      模型缺失或哈希不一致（启动前校验）
raw image not ready within 5 seconds       原始图像未就绪
derived line image never became ready      派生图像从未就绪
derived line image stale beyond recovery grace  图像超过 3 秒未恢复
line follower exited before final stop     巡线子进程提前退出
line follow timed out                      120 秒未检测到最终停车线
cancelled                                  取消
ROS shutdown                               节点关闭
```

## 5. 子进程所有权

supervisor 只终止自己通过 `subprocess.Popen` 创建并记录 PID 的子进程
（`phase3_yolo_server` 与 `phase3_line_follower`）。禁止 `pkill`、`killall`、
`rosnode kill -a` 或按模糊节点名清理。取消、异常、超时和 shutdown 都先发布零速度，
再结束所拥有的子进程，最后发布一次关联失败。

## 6. 模型与源文件校验

YOLO 模型不提交 Git：

```text
/home/ucar/ucar_ws/src/yolo_turn/best.pt
sha256: cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd
```

四个冻结源脚本哈希记录在 `SOURCE_SNAPSHOT.sha256`；本地回归会逐一核对：

```text
follow_left_v4.py   8a9471e5917b93bdddd1f0b191e8ace48fe4d6baa8269f68204d4fba23fbbfb3
follow_right_v4.py  92fd5098be423bab61c3eb5deb5c202c12adbc6c45bc4638a1af7bf5931a5d73
follow_mid_v4.py    736d0666475f25f48f1e11e888a6c40864af95496754ff37f6311fbba3d0b07e
yolo_server.py      9d7010328a636742c1c60012cd6c4f0c78ae3cfc2ab4cf57c815ec88de4e9a51
```

## 7. 分模块启动（先禁动）

所有调试前先让底盘机械断能或可靠架空驱动轮。逐模块启动，运动先关闭：

```bash
# 只启动相机适配器（不发布速度）
roslaunch line_follow_integration phase3.launch enable_line_follow_supervisor:=false enable_line_navigation_adapter:=false

# 只启动导航适配器（目标来自 /task/line_navigation_goal）
roslaunch line_follow_integration phase3.launch enable_line_camera_adapter:=false enable_line_follow_supervisor:=false

# 只启动监管器（等待 /task/line_follow/start）
roslaunch line_follow_integration phase3.launch enable_line_camera_adapter:=false enable_line_navigation_adapter:=false
```

观察命令：

```bash
rostopic hz /usb_cam/image_raw                 # 原始图像（期望约 30 FPS）
rostopic hz /line_follow/image_raw             # 派生 640x480 图像（期望约 15 FPS）
rostopic echo /task/line_follow/status         # 方向/巡线状态
rostopic echo /task/motion_mode                # 运动模式
rostopic echo /cmd_vel/line_follow             # 隔离的巡线速度（图像新鲜时才有）
rostopic echo /task/line_navigation_arrived    # 导航到达结果
```

## 8. 实车看护验收（逐层放行）

1. 只验证新增导航起点及停车姿态（motion mode=STOP_NAVIGATION）。
2. 底盘保持 IDLE：验证原始 1020x720 YOLO 与派生 640x480 巡线图像。
3. 验证红灯等待与三类方向锁定（红绿灯/方向识别）。
4. 单独放行 `/cmd_vel/line_follow` 验证路线。
5. 验证最终停车线、零速度和单次“任务完成”播报。
6. 最后运行第一至第三阶段全流程。

## 9. 当前状态

- 分支：`codex/phase3-line-follow-integration`
- 本地 TDD 任务 1～8 已完成并单独提交；任务 9 全量回归与交接记录见
  `ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md`。
- 外部 YOLO 模型（`best.pt`）由小车提供，未提交 Git。
- **尚未部署、未编译、未实车验收**：本地测试通过不代表车端可用。
