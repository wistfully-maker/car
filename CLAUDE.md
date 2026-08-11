# 第三阶段红绿灯识别与巡线联调：DeepSeek 实施说明

## 1. 当前唯一任务

在已验证的二维码与两阶段动态避障导航、车间识别和两次停车流程之后，实现第三阶段：

```text
仿真车间播报成功
 -> 导航到 YAML 配置的巡线起点
 -> 停车等待红绿灯/方向识别
 -> 选择 left_turn/right_turn/straight
 -> 巡线并检测最终停车线
 -> task_orchestrator 播报“任务完成”
 -> 匹配 speak_done success 后 COMPLETE
```

严格按实施计划逐任务执行：

```text
docs/superpowers/plans/2026-08-11-phase3-line-follow-integration.md
```

设计依据：

```text
docs/superpowers/specs/2026-08-11-phase3-line-follow-integration-design.md
```

计划与本文件冲突时，以本文件的工作位置和安全约束为准；业务设计以已确认的 spec 为准。仓库中描述旧第一阶段或第二阶段任务的说明属于历史资料，不得据此切换到旧工作树。

## 2. 唯一允许的工作位置

```text
D:\program_sec\智能车\.worktrees\phase3-line-follow-integration
branch: codex/phase3-line-follow-integration
required starting HEAD: a58ba48
required baseline ancestor: 7b61069
```

开始前必须执行：

```powershell
git branch --show-current
git rev-parse HEAD
git merge-base --is-ancestor 7b61069 HEAD
git status --short
git log -6 --oneline
```

分支不符、基线不是祖先、存在无法识别的脏文件或 HEAD 已被其他人推进时，立即停止并报告 Codex。不得创建、切换、合并、rebase 或删除分支/worktree。

## 3. 已同步的实车基线

第二阶段第三个车间扫描航点已从小车同步并提交为 `7b61069`：

```python
(2.2843, -2.43094, 0.0, 1.0)
```

对应 stop 包 113 项回归已通过。第三阶段不得再次修改 `stop` 算法、车间扫描航点、OCR/PCA/停车逻辑或其测试契约。

第三阶段巡线起点是另一个目标：

```yaml
frame_id: map
x: 0.5167081260031493
y: -3.125171302690142
qz: -0.7044294777312331
qw: 0.7097739857893512
```

它必须写入并从以下文件读取，不得硬编码进 Python：

```text
ucar_ws/src/line_follow_integration/config/phase3.yaml
```

## 4. 权威巡线来源

本地已跑通来源：

```text
E:\follow_v1\follow_v1\follow_left_v4.py
E:\follow_v1\follow_v1\follow_right_v4.py
E:\follow_v1\follow_v1\follow_mid_v4.py
E:\follow_v1\follow_v1\auto_drive_v3.py
E:\follow_v1\follow_v1\start_all_yolo.launch
E:\follow_v1\follow_v1\README_V4.md
```

权威 SHA-256：

```text
follow_left_v4.py  8a9471e5917b93bdddd1f0b191e8ace48fe4d6baa8269f68204d4fba23fbbfb3
follow_right_v4.py 92fd5098be423bab61c3eb5deb5c202c12adbc6c45bc4638a1af7bf5931a5d73
follow_mid_v4.py   736d0666475f25f48f1e11e888a6c40864af95496754ff37f6311fbba3d0b07e
auto_drive_v3.py   31353702cf6146bcb021bb0586c8a9f2bdc9ccc7ff2ab46f327b919fa15ef07e
```

只导入三个 V4 路线脚本。不要导入 `auto_drive_v3.py`，其方向选择由新 supervisor 取代，内部 TTS 与假成功行为不能进入集成。不要启动或导入 `start_all_yolo.launch`，它会重复启动底盘和相机。

车端 YOLO 源：

```text
ucar@192.168.1.109:/home/ucar/ucar_ws/src/car_server/yolo_server.py
sha256: 9d7010328a636742c1c60012cd6c4f0c78ae3cfc2ab4cf57c815ec88de4e9a51
```

仅允许为取得该单个源文件执行只读 SSH/标准输出读取和 SHA-256 核对。禁止 SCP 写入小车、修改车端文件、远程编译、启动/停止 ROS、发布 topic 或控制底盘。若无法只读取得文件，停止并报告 Codex。

YOLO 模型不提交到 Git：

```text
/home/ucar/ucar_ws/src/yolo_turn/best.pt
sha256: cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd
```

## 5. 相机与速度所有权

物理相机保持唯一且维持二维码已调好的 1020x720 配置：

```text
/usb_cam/image_raw 1020x720
  -> QR 使用原始图像
  -> YOLO 使用原始图像
  -> line_camera_adapter 中心裁剪 960x720，再缩放 640x480@15 FPS
  -> /line_follow/image_raw
```

不得启动第二个 `usb_cam`，不得修改、重启或动态重配物理相机。

速度路径固定为：

```text
V4 script /cmd_vel
 -> remap /line_follow/cmd_vel_candidate
 -> phase3 supervisor 图像健康门控
 -> /cmd_vel/line_follow
 -> global velocity_arbiter mode=LINE_FOLLOW
 -> /cmd_vel
```

V4 脚本不得直接发布最终 `/cmd_vel` 或公共 `/cmd_vel/line_follow`。只有 supervisor 可发布 `/cmd_vel/line_follow`，只有全局 velocity arbiter 可发布最终 `/cmd_vel`。

## 6. 状态与接口

全局状态：

```text
WAITING_SIMULATION_SPEECH
 -> NAVIGATING_LINE_START
 -> WAITING_LINE_DIRECTION
 -> LINE_FOLLOWING
 -> WAITING_FINAL_SPEECH
 -> COMPLETE
```

公共接口以 task_orchestrator 为准：

```text
/task/line_navigation_goal
/task/line_navigation_arrived
/task/line_follow/start
/task/line_follow/status
/task/motion_mode
/cmd_vel/line_follow
```

所有 JSON 使用 `protocol_version=1` 并按 `task_id/goal_id` 关联、去重和拒绝过期事件。导航结果使用现有 arrival 语义 `arrived|failed`；巡线状态固定为：

```text
waiting_signal
direction_selected
following
success
failure
```

红灯只等待。30 秒无方向结果选择 `straight`。锁定方向后关闭本次拥有的 YOLO 子进程。只有本次新鲜的最终停车标记才成功；子进程提前退出和 120 秒超时必须失败。最终“任务完成”只由 task_orchestrator 播报。

## 7. 故障与进程约束

- 图像超过 0.5 秒未更新，立即阻断候选速度并发零速度；允许 3 秒恢复，仍未恢复才失败。
- 导航、方向、巡线和 TTS 外层状态超时必须严格大于内部超时，按计划默认值执行。
- 只能终止 supervisor 自己通过 `subprocess.Popen` 创建并记录的 PID。
- 禁止 `pkill`、`killall`、`rosnode kill -a` 或按模糊节点名清理。
- cancel、异常、超时和 shutdown 必须先零速度，再结束所拥有的子进程。
- 不得伪造 arrival、line success、speak_done 或实车验收结果。

## 8. TDD、提交和允许修改范围

严格执行计划中的 9 个任务。每项必须：

```text
先写失败测试 -> 运行确认 RED -> 最小实现 -> 运行确认 GREEN -> 回归 -> 单独提交
```

只修改计划逐项列出的文件。每次显式 `git add -- <files>`，禁止 `git add .` 和 `git add -A`。

禁止：

- push、merge、rebase、reset、clean、checkout 覆盖文件；
- 修改根工作区、其他 worktree、stop 算法或外部包；
- 提交模型、密钥、日志、图片、缓存、构建目录或临时文件；
- 部署、远程编译、远程 ROS 操作或实车运动；
- 为通过测试改写 protocol v1、放宽速度隔离或伪造成功。

若计划必须发生结构性变化才能继续，停止并写清证据交给 Codex，不得自行扩大范围。

## 9. 最终本地验证

至少执行：

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/line_follow_integration ucar_ws/src/task_orchestrator ucar_ws/src/stop
git diff --check
git status --short
```

用 `xml.etree.ElementTree` 解析所有新增/修改 launch，用 YAML 解析器验证 `phase3.yaml`，校验四个导入脚本 SHA-256。

## 10. 交接给 Codex

完成后停止，不部署。报告：

1. 分支和最终 HEAD；
2. 每个提交及目的；
3. 修改/新增文件清单；
4. 每条测试命令、测试数量和结果；
5. source/model SHA-256；
6. 完整 topic/remap/状态/超时表；
7. YAML 本地绝对路径；
8. 未验证的实车风险；
9. 所有未提交/未跟踪文件。

Codex 后续只负责：代码审查、全量回归、缺陷修复、车端备份部署、编译和分级实车验收。
