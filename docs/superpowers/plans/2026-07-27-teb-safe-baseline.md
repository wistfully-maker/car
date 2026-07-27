# U-CAR TEB 安全基线实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为现有 ROS 1 导航包增加可回退的 TEB 局部规划器，并用相同地图、footprint 和速度上限尽早完成实车对比。

**Architecture:** 保留当前 DWA 文件和启动行为，在 `navigation_stack.launch` 增加显式 `local_planner` 参数；条件分支只加载被选中的插件与配置。TEB使用独立安全参数文件，部署后先验证插件、命名空间和静态参数，再进行短目标与 waypoint 1 实测。

**Tech Stack:** ROS 1 Noetic、move_base、teb_local_planner、roslaunch、YAML、Python unittest。

---

### Task 1: 建立启动选择的失败测试

**Files:**
- Modify: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`
- Test: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 写入失败测试**

新增测试，要求 `navigation_stack.launch` 定义 `local_planner` 参数、分别包含DWA和TEB条件分支，并要求TEB配置存在：

```python
def test_navigation_stack_selects_exactly_one_local_planner(self):
    text = launch_text("navigation_stack.launch")
    self.assertIn('<arg name="local_planner" default="dwa"/>', text)
    self.assertIn('if="$(eval arg(\\'local_planner\\') == \\'dwa\\')"', text)
    self.assertIn('if="$(eval arg(\\'local_planner\\') == \\'teb\\')"', text)
    self.assertIn("teb_local_planner/TebLocalPlannerROS", text)
    self.assertTrue(
        (PACKAGE / "config/local_planners/teb_safe.yaml").is_file()
    )
```

- [ ] **Step 2: 运行测试并确认失败**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: `test_navigation_stack_selects_exactly_one_local_planner` 因缺少参数或TEB文件失败。

- [ ] **Step 3: 暂不修改生产文件**

保持红灯状态并进入Task 2。

### Task 2: 增加独立TEB安全配置和启动分支

**Files:**
- Create: `ucar_ws/src/ucar_nav/config/local_planners/teb_safe.yaml`
- Modify: `ucar_ws/src/ucar_nav/launch/navigation_stack.launch`
- Modify: `ucar_ws/src/ucar_nav/launch/ucar_navigation.launch`
- Modify: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 创建TEB安全参数**

创建 `teb_safe.yaml`，使用矩形footprint模型、全向速度、保守障碍距离和单拓扑：

```yaml
TebLocalPlannerROS:
  odom_topic: odom
  map_frame: map

  max_vel_x: 0.25
  max_vel_x_backwards: 0.10
  max_vel_y: 0.15
  max_vel_theta: 0.50
  acc_lim_x: 1.00
  acc_lim_y: 1.00
  acc_lim_theta: 2.00
  min_turning_radius: 0.0

  footprint_model:
    type: polygon
    vertices: [[0.191, -0.148], [0.191, 0.148], [-0.191, 0.148], [-0.191, -0.148]]

  xy_goal_tolerance: 0.15
  yaw_goal_tolerance: 0.15
  free_goal_vel: false

  min_obstacle_dist: 0.05
  inflation_dist: 0.20
  include_costmap_obstacles: true
  costmap_obstacles_behind_robot_dist: 1.0

  dt_ref: 0.30
  dt_hysteresis: 0.10
  max_samples: 500
  global_plan_overwrite_orientation: true
  allow_init_with_backwards_motion: false
  max_global_plan_lookahead_dist: 2.0

  enable_homotopy_class_planning: false
  enable_multithreading: false
  no_inner_iterations: 5
  no_outer_iterations: 4
```

- [ ] **Step 2: 修改启动文件**

在两个启动入口向下传递 `local_planner`。DWA分支加载：

```xml
<group if="$(eval arg('local_planner') == 'dwa')">
  <rosparam command="load" file="$(find ucar_nav)/config/local_planners/dwa_safe.yaml"/>
  <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS"/>
</group>
```

TEB分支加载：

```xml
<group if="$(eval arg('local_planner') == 'teb')">
  <rosparam command="load" file="$(find ucar_nav)/config/local_planners/teb_safe.yaml"/>
  <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/>
</group>
```

- [ ] **Step 3: 增加参数约束测试**

测试TEB速度、footprint、全向能力和单拓扑设置：

```python
def test_teb_safe_profile_matches_current_safety_limits(self):
    teb = load_yaml(
        "config/local_planners/teb_safe.yaml"
    )["TebLocalPlannerROS"]
    self.assertEqual(0.25, teb["max_vel_x"])
    self.assertEqual(0.15, teb["max_vel_y"])
    self.assertEqual("polygon", teb["footprint_model"]["type"])
    self.assertFalse(teb["enable_homotopy_class_planning"])
    self.assertGreaterEqual(teb["min_obstacle_dist"], 0.05)
```

- [ ] **Step 4: 运行全部本地测试**

Run:

```powershell
python -m unittest discover -s ucar_ws/src/ucar_nav/test -p "test_*.py" -v
git diff --check
```

Expected: 全部测试通过，`git diff --check`无错误。

### Task 3: 安装、部署和静态验证

**Files:**
- Deploy: `ucar_ws/src/ucar_nav/config/local_planners/teb_safe.yaml`
- Deploy: `ucar_ws/src/ucar_nav/launch/navigation_stack.launch`
- Deploy: `ucar_ws/src/ucar_nav/launch/ucar_navigation.launch`

- [ ] **Step 1: 停止导航父进程**

Run:

```bash
pkill -INT -f '/opt/ros/noetic/bin/[r]oslaunch ucar_nav ucar_navigation.launch'
```

Expected: `/move_base`、`/amcl`、`/base_driver`和`/ydlidar_node`退出。

- [ ] **Step 2: 安装TEB**

Run:

```bash
sudo apt-get update
sudo apt-get install -y ros-noetic-teb-local-planner
```

Expected:

```bash
rospack find teb_local_planner
```

输出 `/opt/ros/noetic/share/teb_local_planner`。

- [ ] **Step 3: 部署三个文件并启动TEB**

Run:

```bash
roslaunch ucar_nav ucar_navigation.launch \
  start_camera:=false local_planner:=teb
```

Expected: `/move_base/base_local_planner`为
`teb_local_planner/TebLocalPlannerROS`，日志中没有插件加载错误。

- [ ] **Step 4: 验证互斥加载**

检查TEB命名空间存在，DWA命名空间没有本轮私有参数：

```bash
rosparam get /move_base/base_local_planner
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
```

Expected: 分别为TEB插件名和`0.25`。

### Task 4: 实车安全阶梯测试

**Files:**
- Runtime logs: `/tmp/teb-short-*`
- Runtime logs: `/tmp/teb-waypoint1-*`

- [ ] **Step 1: 初始化AMCL**

发布起点近似位姿后，以`0.2 rad/s`正反旋转各2秒，确认最终姿态基本恢复，检查AMCL协方差。

- [ ] **Step 2: 执行0.25 m短目标**

使用：

```bash
rosrun ucar_nav controlled_nav_goal.py \
  --distance 0.25 --timeout 30
```

Expected: `SUCCEEDED`、无扫墙、停止后`/cmd_vel`为零。

- [ ] **Step 3: 执行waypoint 1并记录**

同步记录`/cmd_vel`和`/odom/twist/twist`，然后运行：

```bash
rosrun ucar_nav controlled_nav_goal.py \
  --waypoint-file /home/ucar/waypoints.xml \
  --waypoint-name 1 --timeout 60
```

Expected: 不持续前后振荡；若失败，自动取消并停车，保留日志后停止加速。

- [ ] **Step 4: 验证后提交**

Run:

```powershell
python -m unittest discover -s ucar_ws/src/ucar_nav/test -p "test_*.py" -v
git diff --check
git status --short
```

Expected: 测试通过；只提交TEB实现、相关测试、计划和必要的既有导航修正，不提交临时地图截图。
