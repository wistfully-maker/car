# 避障停车模块接入总流程 Implementation Plan

> **执行说明：** 按任务逐项实施本计划，并使用复选框（`- [ ]`）记录进度。若由代理执行，应使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`。每完成一个任务向用户汇报进度，不等待确认即可继续。

**目标：** 把队员开发的避障停车模块（`ucar_avoid`：avoid.cpp + vision_node.py）接入 `task_orchestrator` 总流程，成为配送阶段（`NAVIGATING_TO_WORKSHOP`）的执行器；同时为仿真任务阶段预留状态机（当前直接返回 complete，将来收到仿真完成信号后播报"仿真任务已完成，已将[货品名称]放入[仓库类别]"）。

**架构：** 配送适配器改造自队员的 avoid.cpp，保持其扫描/停车核心逻辑不变（队员后续仍可独立迭代避障逻辑），仅在外围接入协议：订阅 `/task/delivery_navigation_goal`（protocol v1）命令式执行，发布 `/task/delivery_arrived` 回报状态机；速度改经 `/cmd_vel/navigation` 走 velocity_arbiter；播报改发 `/voice/speak` 走 tts_bridge（衔接 QR 阶段已用的语音链路）；配送开始时执行 lidar_loc → AMCL 定位切换（前半段导航必须用 lidar_loc，避障停车模块基于 AMCL 调通）。

**技术栈：** ROS 1 Noetic、C++11（avoid.cpp + jsoncpp）、Python 3.7（task_orchestrator）、AMCL、move_base、catkin。

---

## 关键设计决策（已确认）

1. **接入不影响避障核心迭代**：avoid.cpp 的扫描/停车函数（`scanAtWaypoint`、`laserParkingAndSpeak`、`cleanOCRText`、航点遍历）保持原样，只在构造函数入口、回调出口和速度/播报/协议通道上做最小改造；用参数（`~cmd_vel_topic` 等）支持独立调试。
2. **车间/货物参数化**：删除硬编码 `TASK2_*`/`TASK3_*` 常量，`current_target`/`current_cargo` 从 `/task/delivery_navigation_goal` 的 `target_workshop`/`selected_item` 读取。
3. **播报链路**：实物停车后发布 `/voice/speak`（protocol v1：task_id + speech_id + text，文本"已将X放入Y"），由 tts_bridge 播报；不再直接 `system()` 调 tts_http.py。
4. **速度通道**：`/cmd_vel` → 参数化 `~cmd_vel_topic`，接入时配置为 `/cmd_vel/navigation`（velocity_arbiter 在 NAVIGATION 模式放行）。
5. **AMCL 切换**：收到配送目标后、导航前执行：取消 move_base → kill lidar_loc → 从 TF 读当前 `map→base_link` 位姿作为 initial_pose → 启动 AMCL → 等 `map→odom` TF 接管。切换逻辑独立成函数，失败则发布 failed。
6. **仿真阶段状态机预留**：orchestrator 增加仿真阶段状态与"仿真完成"消息解析、播报动作，但受配置 `~simulation_phase_enabled`（默认 false）门控——当前 delivery_arrived 后直接 COMPLETE（保持现有行为），启用后进入等待仿真完成信号 → 播报"仿真任务已完成，已将X放入Y" → COMPLETE。

## 文件结构

```text
ucar_ws/src/ucar_avoid/          # 队员包（首次纳入 git，基线来自车上）
├── src/avoid.cpp                # 配送适配器（本计划改造）
├── scripts/vision_node.py       # 视觉服务节点（复用，不动）
├── launch/delivery.launch       # 新建：vision_node + racecar_control + amcl_delivery
├── launch/amcl_delivery.launch  # 新建：仅 AMCL 节点（配送阶段定位）
└── CMakeLists.txt               # 修改：安装 launch、链接 jsoncpp/tf2

ucar_ws/src/task_orchestrator/
├── src/task_orchestrator/orchestrator.py  # 修改：仿真阶段预留
├── src/task_orchestrator/protocol.py      # 修改：parse_sim_complete
├── scripts/task_orchestrator_node.py      # 修改：订阅 sim_complete、播报动作
├── launch/competition_full.launch         # 修改：start_delivery 组
└── scripts/start_competition.sh           # 修改：冲突检查
```

## Task 1：基线同步与首次提交

**涉及文件：**
- 提交：`ucar_ws/src/ucar_avoid/**`（首次纳入）
- 提交：`ucar_ws/src/ucar_nav/**`（车上最新版，覆盖分支旧版）

- [ ] **步骤 1：确认车上同步内容**
  已从车上 tar 同步 ucar_avoid（avoid.cpp 27.6KB、vision_node.py、show_camera.py、CMakeLists、package.xml）与 ucar_nav（含车上独有：amcl initial_pose、local_planner 参数化、avoid_obstacles.launch）。注意 ucar_nav 中 common_teb.yaml 为符号链接（目标缺失），提交前确认处理。
- [ ] **步骤 2：清理并提交**
  排除 `__pycache__`、`*.pyc`、备份文件后 `git add`，提交：
  `chore: sync vehicle avoid/nav baseline`

## Task 2：状态机仿真阶段预留

**涉及文件：**
- 修改：`ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- 修改：`ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py`
- 修改：`ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- 修改：`ucar_ws/src/task_orchestrator/test/test_orchestrator.py`
- 修改：`ucar_ws/src/task_orchestrator/test/test_protocol.py`

- [x] **步骤 1：协议层增加 `parse_sim_complete(raw_json, expected_task_id)`**
  校验 protocol_version=1、task_id、status（"success"/"failed"）、可选 message。与现有 `parse_arrival` 模式一致。
- [x] **步骤 2：状态机增加仿真阶段**
  - 新状态：`SIM_DELIVERY`（将来：导航去仿真物品车间）与 `WAITING_SIM`（等待仿真完成信号），进 `_ACTIVE_STATES`
  - 新动作：`publish_sim_trigger`（触发仿真）、`publish_speech`（播报"仿真任务已完成，已将X放入Y"）
  - 转换：`delivery_arrived(success)` → 若 `simulation_phase_enabled`（构造参数）为 false → COMPLETE（现有行为不变）；为 true → `SIM_DELIVERY`（发布 sim_trigger）
  - `on_sim_complete(success)` → 播报"仿真任务已完成，已将X放入Y" → COMPLETE
- [x] **步骤 3：ROS 层接线**
  订阅 `/task/sim_complete`（仅仿真启用时）；`publish_sim_trigger` 发布到 `/task/sim_trigger`。
- [x] **步骤 4：单元测试**
  覆盖：禁用仿真时 delivery_arrived 直接 complete（回归）；启用仿真时进入 SIM_DELIVERY → sim_complete → 播报文本正确 → complete；sim_complete 身份不匹配拒绝。
- [x] **步骤 5：提交**
  `feat: reserve simulation delivery phase in orchestrator`

## Task 3：配送适配器（avoid.cpp 命令式化）

**涉及文件：**
- 修改：`ucar_ws/src/ucar_avoid/src/avoid.cpp`
- 修改：`ucar_ws/src/ucar_avoid/CMakeLists.txt`
- 新建：`ucar_ws/src/ucar_avoid/launch/amcl_delivery.launch`

- [x] **步骤 1：接入协议**
  用 jsoncpp 实现最小 JSON 解析/构造（ROS Noetic 自带）：
  - 订阅 `/task/delivery_navigation_goal`：解析 `task_id`、`goal_id`、`target_workshop`、`selected_item`
  - 订阅 `/task/cancel`：解析 `task_id`（当前任务时取消）
  - 发布 `/task/delivery_arrived`：`{protocol_version:1, task_id, goal_id, status:"success"/"failed", message}`
  - 构造函数不再自动执行 `executeMission()`，改为收到 goal 才执行
- [x] **步骤 2：参数化**
  - 删除 `TASK2_*`/`TASK3_*` 硬编码，`current_target`/`current_cargo` 来自 goal
  - `~cmd_vel_topic` 参数（默认 `/cmd_vel/navigation`），`cmd_pub` 用它发布
  - 扫描航点保留（队员后续可调），从代码常量改为参数/配置文件时标记 TODO（本次不动）
- [x] **步骤 3：播报链路**
  `speak()` 改为发布 `/voice/speak`（JSON：protocol_version、task_id、speech_id（自生成）、text），不等待 speak_done；去掉 `system()` TTS 调用。
- [x] **步骤 4：AMCL 切换**
  新增 `switchToAmcl()`：cancel move_base → `rosnode kill /lidar_loc` → tf2 查询 `map→base_link` 得 initial_pose → `roslaunch ucar_avoid amcl_delivery.launch initial_x/y/a`（后台进程）→ 轮询 `map→odom` TF 出现（超时则失败）。配送失败路径发布 failed 并恢复。
- [x] **步骤 5：执行流程**
  收到 goal → switchToAmcl → 现有 Task2 流程（遍历航点扫描 → 匹配 `target_workshop` → 停车）→ 播报"已将X放入Y" → 发布 arrived(success)；超时/未匹配 → arrived(failed)。
- [x] **步骤 6：CMake 链接**
  `target_link_libraries(racecar_control ${catkin_LIBRARIES} jsoncpp)`；catkin_install_python 增加 vision_node.py（已有）。
- [x] **步骤 7：提交**
  `feat: drive avoid parking from orchestrator protocol`

## Task 4：launch 与启动脚本集成

**涉及文件：**
- 新建：`ucar_ws/src/ucar_avoid/launch/delivery.launch`（vision_node + racecar_control，`~cmd_vel_topic` 传 `/cmd_vel/navigation`）
- 修改：`ucar_ws/src/task_orchestrator/launch/competition_full.launch`（`start_delivery` 组）
- 修改：`ucar_ws/src/task_orchestrator/scripts/start_competition.sh`（冲突检查 + 设备检查）

- [x] **步骤 1：delivery.launch**
  常驻启动 vision_node 与 racecar_control，节点名固定（/vision_node、/racecar_control），不与现有冲突。
- [x] **步骤 2：competition_full.launch**
  新增 `start_delivery`（默认 true）：include delivery.launch。注意与 fast_nav 组的 move_base 复用关系。
- [x] **步骤 3：start_competition.sh**
  冲突检查新增 `/vision_node`、`/racecar_control`；`cmd_vel` 所有权检查保持（arbiter 独占）。
- [x] **步骤 4：提交**
  `feat: integrate delivery adapter into competition launch`

## Task 5：编译与静态验证

- [x] **步骤 1：同步到车上**（ucar_avoid、task_orchestrator 改动）
- [x] **步骤 2：车上 `catkin_make` 编译** ucar_avoid（C++ 语法、jsoncpp/tf2 链接）
- [x] **步骤 3：本机 Python 单元测试**（orchestrator/protocol 改动）
- [ ] **步骤 4：协议 dry-run**：启动 competition_full.launch，手工发布 delivery_goal，核对日志流程（不实车运动时仅验证协议解析与状态流转）

## Task 6：实车验证（用户在旁）

- [ ] **步骤 1：实物配送全流程**：模拟任务 → 状态机 → 配送适配器 → AMCL 切换 → 扫描 → 停车 → 播报"已将X放入Y" → complete
- [ ] **步骤 2：失败路径**：无匹配车间、导航超时、AMCL 切换失败 → failed 且状态机 ERROR
- [ ] **步骤 3：避障逻辑回归**：确认扫描转一圈/停车行为与队员独立运行时一致（验证解耦有效）

## 待确认项（不阻塞开发）

1. 标牌实际文本："电子产品加工车间" 还是 "电子产品生产车间"？（影响 OCR 匹配，配送适配器参数化后由 goal 传入，不硬编码）
2. 将来仿真完成信号 topic 命名（当前按 `/task/sim_complete` 预留）
3. 扫描航点坐标与三个车间/仿真物品车间的关系（当前保留队员 3 点）
