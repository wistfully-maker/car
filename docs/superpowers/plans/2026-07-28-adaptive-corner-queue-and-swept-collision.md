# 自适应连续拐角队列与旋转扫掠防碰实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `ucar_corner_supervisor` 稳定识别连续拐角，并在任何原地转向前验证矩形车体旋转扫掠区域不会碰墙。

**Architecture:** 将纯算法拆为路径分段模块和栅格碰撞模块，ROS 节点只负责消息转换和时间有效性；状态机消费已经拟合的当前拐角和扫掠安全结果。所有新行为先由纯 Python 单元测试定义，再接入 ROS。

**Tech Stack:** ROS 1 Noetic、Python 3.7、`nav_msgs/Path`、`nav_msgs/OccupancyGrid`、TF2、`unittest`。

---

### Task 1: 多拐点稳定线段拟合

**Files:**
- Create: `ucar_ws/src/ucar_corner_supervisor/src/ucar_corner_supervisor/path_corners.py`
- Create: `ucar_ws/src/ucar_corner_supervisor/test/test_path_corners.py`

- [ ] **Step 1: 写失败测试**

覆盖直线、单直角、带锯齿直角、短连接的连续同向和反向直角。断言 API：

```python
corners = extract_corner_plan(
    points,
    simplify_tolerance=0.08,
    min_corner_angle=math.radians(45),
    min_segment_length=0.15,
    max_fit_residual=0.08,
)
self.assertEqual(len(corners), 2)
self.assertAlmostEqual(corners[0].exit_heading, math.pi / 2, delta=0.12)
```

- [ ] **Step 2: 运行并确认 RED**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_path_corners.py
```

预期：因 `path_corners` 不存在而失败。

- [ ] **Step 3: 最小实现**

实现弧长、RDP、相邻段转角、TLS 直线拟合、`LineFit`、`PlannedCorner` 和
`extract_corner_plan()`。同向近邻候选合并，反向候选保留。

- [ ] **Step 4: 运行测试并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_path_corners.py
git add ucar_ws/src/ucar_corner_supervisor
git commit -m "feat: extract stable consecutive path corners"
```

### Task 2: footprint 旋转扫掠碰撞

**Files:**
- Create: `ucar_ws/src/ucar_corner_supervisor/src/ucar_corner_supervisor/swept_collision.py`
- Create: `ucar_ws/src/ucar_corner_supervisor/test/test_swept_collision.py`

- [ ] **Step 1: 写失败测试**

构造 `0.05 m` 栅格，验证空地图安全、墙在旋转外角扫掠中不安全、墙在扫掠外安全、
未知栅格和越界不安全，并验证左右转。

```python
result = check_rotation_sweep(
    grid, pose=(0.0, 0.0, 0.0), target_yaw=math.pi / 2,
    footprint=FOOTPRINT, angle_step=math.radians(3),
)
self.assertFalse(result.safe)
self.assertIsNotNone(result.blocking_cell)
```

- [ ] **Step 2: 运行并确认 RED**

预期：因 `swept_collision` 不存在而失败。

- [ ] **Step 3: 最小实现**

实现 `GridMap`、世界/栅格转换、多边形点包含判断、旋转 footprint 和
`check_rotation_sweep()`，`OccupancyGrid` 代价值 `>=100` 或未知/越界即不安全。

- [ ] **Step 4: 运行测试并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_swept_collision.py
git add ucar_ws/src/ucar_corner_supervisor
git commit -m "feat: validate rotation footprint sweep"
```

### Task 3: 状态机接入拐点置信度和扫掠安全

**Files:**
- Modify: `ucar_ws/src/ucar_corner_supervisor/src/ucar_corner_supervisor/corner_geometry.py`
- Modify: `ucar_ws/src/ucar_corner_supervisor/test/test_supervisor_state.py`

- [ ] **Step 1: 写失败测试**

增加低置信度、costmap 过期、进入转向前扫掠碰撞、转动中新碰撞进入 `BLOCKED`，以及
清除目标后从 `BLOCKED` 回到 `IDLE` 的测试。

- [ ] **Step 2: 运行并确认 RED**

预期：`Supervisor.update()` 不接受 `corner_confident` 和 `sweep_safe`。

- [ ] **Step 3: 最小实现**

扩展 `update()` 参数；近拐点不可靠或扫掠不安全时锁存 `BLOCKED`；`TURNING` 和
`EXIT_ALIGN` 每周期检查 `sweep_safe`。

- [ ] **Step 4: 全部状态机测试通过并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_supervisor_state.py
git commit -am "feat: block unsafe corner rotations"
```

### Task 4: ROS 节点集成

**Files:**
- Modify: `ucar_ws/src/ucar_corner_supervisor/scripts/corner_supervisor_node.py`
- Modify: `ucar_ws/src/ucar_corner_supervisor/config/corner_supervisor.yaml`
- Modify: `ucar_ws/src/ucar_corner_supervisor/test/test_ros_assets.py`

- [ ] **Step 1: 写失败静态测试**

要求节点包含 `/move_base/local_costmap/costmap`、`extract_corner_plan`、
`check_rotation_sweep` 和新增诊断字段；配置包含设计列出的所有参数。

- [ ] **Step 2: 运行并确认 RED**

预期：缺少 costmap 订阅和新参数。

- [ ] **Step 3: 接入实现**

缓存 `OccupancyGrid` 和时间；全局路径更新时生成 `PlannedCorner` 列表；用机器人最近路径
索引计算进度；选择首个未通过拐点；进入和执行转向时计算剩余扫掠；发布完整诊断。

- [ ] **Step 4: 全套测试通过并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_package_config.py
python ucar_ws/src/ucar_corner_supervisor/test/test_path_corners.py
python ucar_ws/src/ucar_corner_supervisor/test/test_swept_collision.py
python ucar_ws/src/ucar_corner_supervisor/test/test_supervisor_state.py
python ucar_ws/src/ucar_corner_supervisor/test/test_ros_assets.py
python ucar_ws/src/ucar_corner_supervisor/test/test_readme.py
git commit -am "feat: integrate robust corner queue and collision guard"
```

### Task 5: 文档、评审和部署

**Files:**
- Modify: `ucar_ws/src/ucar_corner_supervisor/README.md`

- [ ] **Step 1: 更新 README**

说明新参数、`BLOCKED`、诊断字段、只观察模式和不得直接重发目标的恢复流程。

- [ ] **Step 2: 独立代码审查**

审查路径拟合、连续弯边界、栅格坐标、footprint 扫掠和 ROS 时间有效性；修复所有
Critical/Important。

- [ ] **Step 3: 部署和编译**

```powershell
scp -r ucar_ws/src/ucar_corner_supervisor ucar@10.234.15.42:/home/ucar/ucar_ws/src/
ssh ucar@10.234.15.42 "source /opt/ros/noetic/setup.bash && cd /home/ucar/ucar_ws && catkin_make --pkg ucar_corner_supervisor"
```

- [ ] **Step 4: 无运动验证**

确认新 Python 包可导入、全部单元测试通过、launch 可解析、`/cmd_vel` 只有监督器一个
发布者。只读取同一航点生成的诊断，不发送目标。
