# `ucar_fast_nav` 编排接入实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使一条安全启动命令能启动当前所有已实现模块，并在语音任务到来后自动检查 `ucar_fast_nav` 健康、导航到二维码观察点、完成 QR、LLM 和 TTS。

**Architecture:** 保持 `ucar_fast_nav` 本体不变；在 `task_orchestrator` 包中新增 `fast_nav_adapter`、`system_readiness_gate` 和只支持导航/QR 的 `velocity_arbiter`。`competition_full.launch` 统一组合导航、相机和业务模块，`start_competition.sh` 在启动前拒绝同名节点、AMCL/定位冲突、设备占用和未经仲裁的 `/cmd_vel` 发布者。

**Tech Stack:** ROS 1 Noetic、Python 3、`rospy`、`actionlib`、`move_base_msgs`、`nav_msgs`、`geometry_msgs`、`tf2_ros`、XML roslaunch、Bash、Python `unittest`。

---

### 任务 1：导航适配器纯 Python 核心

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/fast_nav_logic.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_fast_nav_logic.py`

- [ ] 先测试 protocol v1 领取区目标、`task_id/goal_id` 保留、重复/过期目标、成功/失败终态和取消。
- [ ] 先测试 `StopDetector`：线速度、角速度都在阈值内连续达到 `settle_time` 才返回停稳。
- [ ] 运行 `python ucar_ws/src/task_orchestrator/test/test_fast_nav_logic.py -v`，确认因实现缺失而失败。
- [ ] 实现 `FastNavSession`、`StopDetector`、航点字段验证和 arrival JSON 构造。
- [ ] 重跑定向测试和编排器全回归。
- [ ] 提交：`feat: add fast navigation adapter core`。

### 任务 2：`fast_nav_adapter` ROS 节点

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/fast_nav_adapter_node.py`
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- Modify: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Modify: `ucar_ws/src/task_orchestrator/package.xml`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] 先增加源码契约测试：节点订阅 `/task/pickup_navigation_goal`、`/task/cancel`、`/odom`，使用 `MoveBaseAction`，发布 `/task/pickup_arrived`。
- [ ] 运行 `python ucar_ws/src/task_orchestrator/test/test_package_config.py -v`，确认新节点缺失。
- [ ] 从 `/ucar_fast_nav/pickup_goal` 读取 frame/x/y/yaw/容差，生成 quaternion 并发送 `/move_base` action goal。
- [ ] action 终态不是 `SUCCEEDED` 时发布带原 identity 的 failed；成功时等待 odom 停稳后只发一次 arrived。
- [ ] 取消、新 goal 和 ROS shutdown 都执行 `cancel_goal()` 并清理当前 session。
- [ ] 参数默认：`action_timeout: 300`、`settle_time: 0.5`、`linear_stop_threshold: 0.03`、`angular_stop_threshold: 0.05`。
- [ ] 添加 `actionlib/move_base_msgs/nav_msgs/geometry_msgs` 依赖、CMake 安装规则，重跑包测试。
- [ ] 提交：`feat: adapt ucar fast nav to task protocol`。

### 任务 3：自动导航健康门控

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/readiness.py`
- Create: `ucar_ws/src/task_orchestrator/scripts/system_readiness_gate_node.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_readiness.py`
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- Modify: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Modify: `ucar_ws/src/task_orchestrator/package.xml`

- [ ] 先测试 `ReadinessSnapshot` 对新鲜 scan/odom/map、三段 TF、move_base action、lidar_loc、AMCL 冲突和规划器参数的逐项报告。
- [ ] 运行 `python ucar_ws/src/task_orchestrator/test/test_readiness.py -v`，确认因实现缺失而失败。
- [ ] 实现纯 Python `missing_requirements(snapshot)`，返回稳定、可读的缺失列表。
- [ ] ROS 节点订阅 `/task/status`、`/scan`、`/odom`、`/map`，仅对当前 `CHECKING_DEPENDENCIES` 的 task 轮询检查。
- [ ] 用 `tf2_ros.Buffer`、`SimpleActionClient.wait_for_server`、`rosnode ping`和 `rosparam get` 收集快照；禁止 `/amcl`。
- [ ] 通过后只对该 `task_id` 发布一次 `/task/dependencies_ready`；未通过时节流日志列出原因。
- [ ] 将 `timeouts.dependency_ready` 默认改为 120 秒，重跑编排器和 readiness 测试。
- [ ] 提交：`feat: gate tasks on fast navigation health`。

### 任务 4：导航/QR 两路速度仲裁

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py`
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py`
- Create: `ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_motion_mode.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`

- [ ] 先测试状态映射严格为 `NAVIGATING_TO_PICKUP -> NAVIGATION`、`WAITING_QR -> QR_SEARCH`、其他全部 `IDLE`。
- [ ] 先测试编排器正常、错误、取消和超时路径都发布正确 `/task/motion_mode`。
- [ ] 先测试仲裁核心只有 navigation/qr 两个源，拒绝非活动源，切换和 0.3 秒超时回零。
- [ ] 实现 latched `/task/motion_mode`，输入 `/cmd_vel/navigation`、`/cmd_vel/qr`，唯一输出 `/cmd_vel`。
- [ ] 确认 TTS 后编排器即使进入已有后续业务状态，运动模式仍为 `IDLE`。
- [ ] 重跑编排器全回归并提交：`feat: arbitrate navigation and qr velocity`。

### 任务 5：基于 `ucar_fast_nav` 的总 launch

**Files:**
- Create: `ucar_ws/src/task_orchestrator/launch/competition_full.launch`
- Modify: `ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`
- Create: `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`

- [ ] 先测试总 launch 包含 `ucar_fast_nav/pickup_navigation.launch`、`cmd_vel_topic:=/cmd_vel/navigation`、独立相机、两个新适配节点和全部业务 include。
- [ ] 测试显式 `start_*` 开关，并断言 launch 文本不含 `ucar_waypoint_nav`、`amcl`、`dynamic_obstacle`。
- [ ] 实现导航层：`start_fast_nav:=true` 时 include pickup launch 并下传 `start_base/start_lidar`；外部完整导航已运行时传 false。
- [ ] 用独立 group 将 QR `/cmd_vel` remap 到 `/cmd_vel/qr`；导航使用子 launch 已有 `cmd_vel_topic` 参数。
- [ ] 原 `task_orchestrator.launch` 增加三个默认 false 的可选节点：fast nav adapter、readiness gate、velocity arbiter；总 launch 显式启用。
- [ ] 运行 XML/包配置测试并提交：`feat: compose ucar fast nav competition launch`。

### 任务 6：安全一键脚本

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/start_competition.sh`
- Modify: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Modify: `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`

- [ ] 先测试脚本检查 ROS Master、同名节点/ping、`/amcl`、`/lidar_loc`、设备占用、`/cmd_vel` publishers 和 Spark 密钥。
- [ ] 测试脚本不包含 `rosnode kill`、`pkill`、`killall`、`kill -9`。
- [ ] 实现干净开机时由 roslaunch 启动 Master；Master 已运行时才执行节点和 topic 冲突检查。
- [ ] 根据 `start_fast_nav/start_base/start_lidar/start_camera` 只检查本次将获得所有权的资源。
- [ ] 检查通过后 `exec roslaunch task_orchestrator competition_full.launch "$@"`；用普通 CMake `install(PROGRAMS ...)` 安装 Bash 脚本。
- [ ] 本地静态测试、小车 `bash -n` 通过后提交：`feat: preflight ucar fast nav competition launch`。

### 任务 7：重写 README 和手动联调文档

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/README.md`
- Modify: `ucar_ws/src/task_orchestrator/test/manual_simulation.md`
- Modify: `HANDOFF.md`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] 先测试 README 包含三种启动入口、小飞小飞自动流程、`ucar_fast_nav`、`lidar_loc`、自动健康门控、两路速度仲裁、停止/重启、参数和排障。
- [ ] README 明确单独 `task_orchestrator.launch` 不能唤醒并跑全流程，安全一键入口是 `start_competition.sh`。
- [ ] README 详列每个 launch 启动的节点、每条命令的前置/成功标志/停止方法、全部 topic/JSON、参数位置和异常残留检查。
- [ ] README 明确当前真实验收终点是 TTS，不启动或讨论 `dynamic_obstacle` 为正式流程。
- [ ] `manual_simulation.md` 加入 readiness、navigation action 适配、motion mode 和 TTS 终点的分步观察。
- [ ] 文档契约和全部回归通过后提交：`docs: explain ucar fast nav full startup`。

### 任务 8：全回归、部署和分层实车验证

**Files:**
- Verify all changed files
- Deploy: `ucar_ws/src/task_orchestrator/`
- Read only: `/home/ucar/ucar_ws/src/ucar_fast_nav/`

- [ ] 运行 task_orchestrator、QR、LLM、speech 全部本地测试和 `git diff --check`。
- [ ] 部署前只读检查小车 `ucar_fast_nav` 实际 launch/参数与交接快照一致，不覆盖它。
- [ ] 备份小车旧 `task_orchestrator`，仅部署本包，执行 `catkin_make --pkg task_orchestrator` 和车端测试。
- [ ] 只启动业务层且不发任务，验证节点唯一且底盘零速度。
- [ ] 分别模拟 `NAVIGATION/QR_SEARCH/IDLE`，确认只转发活动速度源，切换和超时归零。
- [ ] 启动 `ucar_fast_nav` 后运行其 `runtime_check.sh`，再单独测试 fast nav adapter 到观察点。
- [ ] 在人工看护和可急停条件下，从“小飞小飞”运行到 TTS 播报完成。
- [ ] TTS 后确认 motion mode 为 `IDLE`、底盘不运动；不测试后续动态避障或航点。
- [ ] 在总 launch 终端 `Ctrl+C`，确认只回收本次启动的节点，记录实际结果。
