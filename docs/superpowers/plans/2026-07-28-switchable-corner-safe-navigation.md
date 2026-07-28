# 可切换过弯安全导航实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 Navfn+TEB、Navfn+DWA、7月17日 legacy TEB、7月21日 legacy DWA 四套可切换导航配置，并使用当前地图直接测试原航点。

**Architecture:** `navigation_stack.launch` 只负责选择一个导航 profile；每个 profile 明确加载一个全局规划器、一个局部规划器及对应参数。硬件层不随 profile 重启。两套 legacy 配置从校验过的压缩包提取，只将旧地图引用替换为当前 `maps/map.yaml`。

**Tech Stack:** ROS 1 Noetic、move_base、NavfnROS、GlobalPlanner、AstarPlannerRos、TEB、DWA、costmap_2d、roslaunch、Python unittest、PyYAML。

---

### Task 1: 定义四套 profile 的配置契约

**Files:**
- Modify: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`
- Test: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 编写失败测试**

增加断言：

```python
def test_corner_profiles_exist_and_use_current_map(self):
    required = (
        "config/profiles/navfn_teb_corner.yaml",
        "config/profiles/navfn_dwa_corner.yaml",
        "config/profiles/legacy_0717_teb.yaml",
        "config/profiles/legacy_0721_dwa.yaml",
        "config/global_planners/navfn.yaml",
        "config/local_planners/teb_corner_safe.yaml",
        "config/local_planners/dwa_corner_safe.yaml",
    )
    for relative_path in required:
        self.assertTrue((PACKAGE / relative_path).is_file(), relative_path)

def test_corner_profiles_share_kinematic_limits(self):
    teb = load_yaml("config/local_planners/teb_corner_safe.yaml")[
        "TebLocalPlannerROS"
    ]
    dwa = load_yaml("config/local_planners/dwa_corner_safe.yaml")[
        "DWAPlannerROS"
    ]
    for planner in (teb, dwa):
        self.assertEqual(0.45, planner["max_vel_x"])
        self.assertEqual(0.20, planner["max_vel_y"])
        self.assertEqual(0.60, planner["max_vel_theta"])
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: FAIL，报告新增 profile 文件不存在。

- [ ] **Step 3: 暂不实现，提交测试**

```powershell
git add ucar_ws/src/ucar_nav/test/test_navigation_config.py
git commit -m "test: define switchable navigation profiles"
```

### Task 2: 实现 Navfn + TEB/DWA 两套新配置

**Files:**
- Create: `ucar_ws/src/ucar_nav/config/global_planners/navfn.yaml`
- Create: `ucar_ws/src/ucar_nav/config/local_planners/teb_corner_safe.yaml`
- Create: `ucar_ws/src/ucar_nav/config/local_planners/dwa_corner_safe.yaml`
- Create: `ucar_ws/src/ucar_nav/config/profiles/navfn_teb_corner.yaml`
- Create: `ucar_ws/src/ucar_nav/config/profiles/navfn_dwa_corner.yaml`
- Modify: `ucar_ws/src/ucar_nav/config/costmap/local_teb.yaml`

- [ ] **Step 1: 创建 Navfn 参数**

```yaml
NavfnROS:
  allow_unknown: true
  default_tolerance: 0.15
  visualize_potential: false
```

- [ ] **Step 2: 创建 TEB 过弯配置**

使用设计文件第6节的完整参数，关键值为：

```yaml
TebLocalPlannerROS:
  max_vel_x: 0.45
  max_vel_x_backwards: 0.20
  max_vel_y: 0.20
  max_vel_theta: 0.60
  acc_lim_x: 0.80
  acc_lim_y: 0.80
  acc_lim_theta: 1.50
  max_global_plan_lookahead_dist: 0.8
  global_plan_viapoint_sep: 0.15
  weight_viapoint: 5.0
  min_obstacle_dist: 0.08
  inflation_dist: 0.20
  weight_obstacle: 100.0
  no_inner_iterations: 2
  no_outer_iterations: 1
  enable_homotopy_class_planning: false
```

同时保留现有 polygon footprint、目标容差和停止行为。

- [ ] **Step 3: 创建 DWA 过弯配置**

```yaml
DWAPlannerROS:
  max_vel_x: 0.45
  min_vel_x: -0.20
  max_vel_y: 0.20
  min_vel_y: -0.20
  max_vel_theta: 0.60
  min_vel_theta: 0.18
  acc_lim_x: 0.80
  acc_lim_y: 0.80
  acc_lim_trans: 0.80
  acc_lim_theta: 1.50
  sim_time: 1.2
  vx_samples: 8
  vy_samples: 5
  vth_samples: 16
  path_distance_bias: 40.0
  goal_distance_bias: 16.0
  occdist_scale: 0.20
  stop_time_buffer: 0.50
```

- [ ] **Step 4: 将 TEB 局部代价地图坐标系改为 odom**

```yaml
local_costmap:
  global_frame: odom
  update_frequency: 5.0
```

- [ ] **Step 5: 运行测试**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 新配置值通过，legacy文件测试仍失败。

- [ ] **Step 6: 提交**

```powershell
git add ucar_ws/src/ucar_nav/config
git commit -m "feat: add navfn corner navigation profiles"
```

### Task 3: 整理两套历史可运行配置

**Files:**
- Create: `ucar_ws/src/ucar_nav/config/legacy/0717/*`
- Create: `ucar_ws/src/ucar_nav/config/legacy/0721/*`
- Create: `ucar_ws/src/ucar_nav/config/profiles/legacy_0717_teb.yaml`
- Create: `ucar_ws/src/ucar_nav/config/profiles/legacy_0721_dwa.yaml`

- [ ] **Step 1: 从只读参考目录复制配置**

来源：

```text
D:\program_sec\智能车\external\original_navigation_archives_20260728\underscore\ucar_ws\src\ucar_nav
D:\program_sec\智能车\external\original_navigation_archives_20260728\plain\home\ucar\ucar_ws\src\ucar_nav
```

只复制 `launch/config/amcl` 与 `launch/config/move_base` 下的文本配置。

- [ ] **Step 2: 规范化7月17日 profile**

保持：

```text
global_planner/GlobalPlanner
teb_local_planner/TebLocalPlannerROS
原始 TEB max_vel_x=2.4
```

删除硬件、相机和旧地图生命周期；profile地图固定由主启动文件的
`map_file:=$(find ucar_nav)/maps/map.yaml` 提供。

- [ ] **Step 3: 规范化7月21日 profile**

保持：

```text
astar_planner/AstarPlannerRos
dwa_local_planner/DWAPlannerROS
原始 DWA max_vel_x=0.45
原始 local inflation_radius=0.25
```

不加载旧 `map_new.yaml`。

- [ ] **Step 4: 增加来源校验测试**

测试断言 profile 的规划器名称、关键速度和当前地图约束，不比较注释文本。

- [ ] **Step 5: 运行测试并提交**

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
git add ucar_ws/src/ucar_nav/config ucar_ws/src/ucar_nav/test
git commit -m "feat: add legacy navigation comparison profiles"
```

### Task 4: 实现启动时 profile 切换

**Files:**
- Modify: `ucar_ws/src/ucar_nav/launch/navigation_stack.launch`
- Modify: `ucar_ws/src/ucar_nav/launch/ucar_navigation.launch`
- Test: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 先写失败测试**

测试 XML 包含：

```xml
<arg name="navigation_profile" default="navfn_teb_corner"/>
```

并断言四个互斥 group：

```text
navfn_teb_corner
navfn_dwa_corner
legacy_0717_teb
legacy_0721_dwa
```

- [ ] **Step 2: 确认测试失败**

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

- [ ] **Step 3: 修改 launch**

每个 group 设置唯一的：

```xml
<param name="/move_base/base_global_planner" value="..."/>
<param name="/move_base/base_local_planner" value="..."/>
```

并只加载该 profile 对应的 YAML。`map_server` 始终使用：

```xml
<arg name="map_file" default="$(find ucar_nav)/maps/map.yaml"/>
```

- [ ] **Step 4: 验证 XML 与测试**

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 全部 PASS。

- [ ] **Step 5: 提交**

```powershell
git add ucar_ws/src/ucar_nav/launch ucar_ws/src/ucar_nav/test
git commit -m "feat: switch navigation profiles at launch"
```

### Task 5: 文档、部署与直接航点测试

**Files:**
- Modify: `ucar_ws/src/ucar_nav/README.md`
- Modify: `ucar_ws/src/ucar_nav/HANDOFF.md`

- [ ] **Step 1: 写明四套启动命令**

```bash
roslaunch ucar_nav ucar_navigation.launch navigation_profile:=navfn_teb_corner
roslaunch ucar_nav ucar_navigation.launch navigation_profile:=navfn_dwa_corner
roslaunch ucar_nav ucar_navigation.launch navigation_profile:=legacy_0717_teb
roslaunch ucar_nav ucar_navigation.launch navigation_profile:=legacy_0721_dwa
```

- [ ] **Step 2: 写明停止与切换规则**

切换前必须取消目标、发布零速度并关闭旧 `move_base`，不得同时运行两个局部规划器。

- [ ] **Step 3: 本地最终验证**

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
git diff --check
```

- [ ] **Step 4: 部署到小车**

通过 `scp` 部署 `ucar_nav` 变更；不覆盖当前地图PGM/YAML和
`/home/ucar/waypoints.xml`。

- [ ] **Step 5: 按顺序直接测试航点**

每套 profile 从同一物理起点初始化 AMCL，然后执行：

```bash
rosrun ucar_nav controlled_nav_goal.py \
  --waypoint-file /home/ucar/waypoints.xml \
  --waypoint-name 1 \
  --timeout 60
```

失败、超时或人工停止后取消目标并发布零速度，再切换下一 profile。

- [ ] **Step 6: 提交文档**

```powershell
git add ucar_ws/src/ucar_nav/README.md ucar_ws/src/ucar_nav/HANDOFF.md
git commit -m "docs: document switchable corner navigation tests"
```
