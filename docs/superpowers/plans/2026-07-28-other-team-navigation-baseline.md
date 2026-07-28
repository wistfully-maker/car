# 对方导航配置安全复现实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立不覆盖现有配置的 `other_team_baseline` 导航 profile，以对方算法参数和本轮安全限速验证当前失败的直角弯。

**Architecture:** 对方 AMCL、GlobalPlanner、TEB、costmap 和 move_base 参数放入独立目录，由独立 launch 加载。雷达驱动 launch 参数化外参，普通模式保持当前外参，对方基线显式传入对方外参；首次运行通过最后加载的安全覆盖 YAML 限速。

**Tech Stack:** ROS 1 Noetic、roslaunch、move_base、GlobalPlanner、TEB、AMCL、YDLidar、Python unittest、YAML。

---

### Task 1: 固化对方基线文件及来源校验

**Files:**
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/amcl.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/global_planner.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/teb.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/costmap_common.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/local_costmap.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/global_costmap.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/move_base.yaml`
- Create: `ucar_ws/src/ucar_nav/config/baselines/other_team/first_run_limits.yaml`
- Modify: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 写失败测试**

在 `NavigationConfigTests` 中增加测试，断言所有基线文件存在，并检查：

```python
def test_other_team_baseline_preserves_source_parameters(self):
    root = "config/baselines/other_team/"
    planner = load_yaml(root + "global_planner.yaml")["GlobalPlanner"]
    teb = load_yaml(root + "teb.yaml")["TebLocalPlannerROS"]
    move_base = load_yaml(root + "move_base.yaml")
    limits = load_yaml(root + "first_run_limits.yaml")["TebLocalPlannerROS"]

    self.assertEqual(0, planner["orientation_mode"])
    self.assertEqual(253, planner["lethal_cost"])
    self.assertEqual("point", teb["footprint_model"]["type"])
    self.assertEqual(1100.0, teb["weight_kinematics_nh"])
    self.assertEqual(1000.0, teb["weight_kinematics_forward_drive"])
    self.assertEqual(15.0, move_base["controller_frequency"])
    self.assertEqual(5.0, move_base["planner_frequency"])
    self.assertEqual(0.20, limits["max_vel_x"])
    self.assertEqual(0.10, limits["max_vel_x_backwards"])
    self.assertEqual(0.40, limits["max_vel_theta"])
```

- [ ] **Step 2: 运行测试并确认失败**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: `FAIL`，提示 `config/baselines/other_team/*` 不存在。

- [ ] **Step 3: 复制对方原始参数并创建安全覆盖**

从
`D:\program_sec\智能车\external\other_team_ucar_ws_20260728\src\ucar_nav\launch\config`
复制对应参数，只去掉乱码注释，不改变数值。安全覆盖文件内容为：

```yaml
TebLocalPlannerROS:
  max_vel_x: 0.20
  max_vel_x_backwards: 0.10
  max_vel_theta: 0.40
```

`move_base.yaml` 保留：

```yaml
controller_frequency: 15.0
controller_patience: 4.0
planner_frequency: 5.0
planner_patience: 5.0
oscillation_timeout: 8.0
oscillation_distance: 0.3
recovery_behavior_enabled: true
clearing_rotation_allowed: true
```

不复制其中会与 launch 冲突的 `base_local_planner` 和 `base_global_planner` 字段。

- [ ] **Step 4: 运行测试并确认通过**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 新增基线测试及原有测试全部 `OK`。

- [ ] **Step 5: 提交**

```powershell
git add ucar_ws/src/ucar_nav/config/baselines/other_team `
  ucar_ws/src/ucar_nav/test/test_navigation_config.py
git commit -m "feat(nav): 添加对方导航参数基线"
```

### Task 2: 参数化雷达外参并保持默认行为不变

**Files:**
- Modify: `ucar_ws/src/ucar_nav/launch/lidar_bringup.launch`
- Modify: `ucar_ws/src/ucar_nav/launch/robot_base_bringup.launch`
- Modify: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 写失败测试**

将现有固定外参测试改为验证 launch 参数：

```python
def test_lidar_bringup_exposes_extrinsic_arguments(self):
    root = ET.parse(LAUNCH / "lidar_bringup.launch").getroot()
    args = {item.attrib["name"]: item.attrib["default"]
            for item in root.findall("arg")}
    self.assertEqual("-0.11", args["laser_x"])
    self.assertEqual("0.165", args["laser_z"])
    self.assertEqual("-0.07", args["laser_yaw"])
    self.assertEqual("true", args["start_lidar"])
    static = next(node for node in root.findall("node")
                  if node.attrib.get("name") == "base_link_to_laser")
    self.assertIn("$(arg laser_x)", static.attrib["args"])
    self.assertIn("$(arg laser_z)", static.attrib["args"])
    self.assertIn("$(arg laser_yaw)", static.attrib["args"])
```

- [ ] **Step 2: 运行测试并确认失败**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: `FAIL`，提示未找到 `laser_x` 等参数。

- [ ] **Step 3: 参数化 launch**

在 `lidar_bringup.launch` 顶部增加：

```xml
<arg name="laser_x" default="-0.11"/>
<arg name="laser_z" default="0.165"/>
<arg name="laser_yaw" default="-0.07"/>
<arg name="start_lidar" default="true"/>
```

YDLidar 节点增加 `if="$(arg start_lidar)"`，静态 TF 保持启动并改为：

```xml
args="$(arg laser_x) 0.0 $(arg laser_z) $(arg laser_yaw) 0.0 0.0 /base_link /laser_frame 40"
```

`robot_base_bringup.launch` 暴露同名外参参数，并在 include
`lidar_bringup.launch` 时逐项转发。这样比赛启动默认行为不变，已有雷达节点时可以用
`lidar_bringup.launch start_lidar:=false` 只替换静态 TF。

- [ ] **Step 4: 运行测试并确认通过**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 全部 `OK`，默认外参仍是当前值。

- [ ] **Step 5: 提交**

```powershell
git add ucar_ws/src/ucar_nav/launch/lidar_bringup.launch `
  ucar_ws/src/ucar_nav/launch/robot_base_bringup.launch `
  ucar_ws/src/ucar_nav/test/test_navigation_config.py
git commit -m "feat(nav): 支持切换雷达外参"
```

### Task 3: 创建独立对方基线启动入口

**Files:**
- Create: `ucar_ws/src/ucar_nav/launch/other_team_baseline.launch`
- Modify: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 写失败测试**

增加测试，断言：

```python
def test_other_team_baseline_launch_is_isolated_and_safety_limited(self):
    text = launch_text("other_team_baseline.launch")
    self.assertIn("teb_local_planner/TebLocalPlannerROS", text)
    self.assertIn("global_planner/GlobalPlanner", text)
    self.assertIn("config/baselines/other_team/teb.yaml", text)
    self.assertIn("config/baselines/other_team/first_run_limits.yaml", text)
    self.assertIn('default="false"', text)
    self.assertNotIn("base_driver.launch", text)
    self.assertNotIn("ydlidar.launch", text)
```

- [ ] **Step 2: 运行测试并确认失败**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: `FAIL`，提示 launch 不存在。

- [ ] **Step 3: 实现独立 launch**

launch 提供：

```xml
<arg name="map_file" default="$(find ucar_nav)/maps/map.yaml"/>
<arg name="start_map_server" default="true"/>
<arg name="start_amcl" default="true"/>
<arg name="start_move_base" default="true"/>
<arg name="apply_first_run_limits" default="true"/>
```

它只启动 map_server、AMCL 和 move_base，不启动底盘或雷达。move_base 内明确设置：

```xml
<param name="base_global_planner" value="global_planner/GlobalPlanner"/>
<param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/>
```

按顺序加载基线文件，最后在条件组中加载
`first_run_limits.yaml`。对方雷达外参由硬件层使用以下参数启动：

```text
laser_x:=0.11 laser_z:=0.13 laser_yaw:=0.0
```

- [ ] **Step 4: 验证 XML、参数选择及全套单元测试**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
git diff --check
```

Expected: 全部 `OK`，`git diff --check` 无错误。

- [ ] **Step 5: 提交**

```powershell
git add ucar_ws/src/ucar_nav/launch/other_team_baseline.launch `
  ucar_ws/src/ucar_nav/test/test_navigation_config.py
git commit -m "feat(nav): 添加对方导航基线启动入口"
```

### Task 4: 本地静态解析和部署

**Files:**
- Modify: `ucar_ws/src/ucar_nav/README.md`

- [ ] **Step 1: 写入启动和回退命令**

README 增加：

```bash
# 硬件层使用对方雷达外参
roslaunch ucar_nav robot_base_bringup.launch \
  laser_x:=0.11 laser_z:=0.13 laser_yaw:=0.0

# 导航层使用对方基线和首次安全限速
roslaunch ucar_nav other_team_baseline.launch \
  apply_first_run_limits:=true
```

停止命令：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
rosnode kill /move_base /amcl /map_server
```

- [ ] **Step 2: 运行全部本地测试**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
python ucar_ws/src/ucar_nav/test/test_controlled_nav_goal.py -v
python ucar_ws/src/ucar_nav/test/test_controlled_twist_test.py -v
git diff --check
```

Expected: 所有测试 `OK`。

- [ ] **Step 3: 部署源码包**

仅同步 `ucar_nav` 本次新增或修改文件到：

```text
/home/ucar/ucar_ws/src/ucar_nav
```

在小车执行：

```bash
cd /home/ucar/ucar_ws
catkin_make
source devel/setup.bash
roslaunch --dump-params \
  /home/ucar/ucar_ws/src/ucar_nav/launch/other_team_baseline.launch
```

Expected:

```text
/move_base/base_global_planner: global_planner/GlobalPlanner
/move_base/base_local_planner: teb_local_planner/TebLocalPlannerROS
/move_base/controller_frequency: 15.0
/move_base/planner_frequency: 5.0
/move_base/TebLocalPlannerROS/max_vel_x: 0.2
```

- [ ] **Step 4: 提交文档**

```powershell
git add ucar_ws/src/ucar_nav/README.md
git commit -m "docs(nav): 记录对方导航基线操作方法"
```

### Task 5: 静态定位验收

**Files:**
- Runtime artifact: `/tmp/other-team-baseline-alignment.txt`

- [ ] **Step 1: 停止旧目标和旧导航节点**

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
rosnode kill /move_base /amcl /map_server /base_link_to_laser
```

Expected: 底盘停车，旧导航与旧静态 TF 不再存在。

- [ ] **Step 2: 使用对方外参启动唯一 TF 和导航基线**

雷达驱动已存在时执行：

```bash
roslaunch ucar_nav lidar_bringup.launch start_lidar:=false \
  laser_x:=0.11 laser_z:=0.13 laser_yaw:=0.0
```

只启动一个静态 TF，不得再次打开 `/dev/ttyS4`。随后启动
`other_team_baseline.launch apply_first_run_limits:=true`。

- [ ] **Step 3: 初始化 AMCL 并采集匹配结果**

```bash
python3 /tmp/set_initial_pose.py --x -1.827 --y -2.897 --yaw 1.741
sleep 6
python3 /tmp/scan_map_alignment.py | tee \
  /tmp/other-team-baseline-alignment.txt
```

Expected: 记录 5 cm、10 cm 匹配率和最佳位姿修正；本步骤不发导航目标。

- [ ] **Step 4: 作出继续/停止判定**

继续条件：

- 匹配率优于当前起点基线 `62% within_10cm`，或最佳修正明显缩小；
- `/scan`、`/odom`、TF 连续；
- 无重复节点。

否则立即恢复当前雷达外参，不测试导航，并将本车雷达 TF 标定列为下一任务。

### Task 6: 受控拐弯实测

**Files:**
- Runtime artifact: `/tmp/other-team-baseline-corner.bag`
- Runtime artifact: `/tmp/other-team-baseline-move-base.log`

- [ ] **Step 1: 启动记录**

```bash
rosbag record -O /tmp/other-team-baseline-corner.bag \
  /scan /odom /amcl_pose /tf /tf_static /cmd_vel \
  /move_base/GlobalPlanner/plan \
  /move_base/TebLocalPlannerROS/local_plan \
  /move_base/status
```

- [ ] **Step 2: 发送只覆盖失败拐弯的受控目标**

使用现有 `controlled_nav_goal.py` 和已验证航点，不发送完整比赛路线。车旁人员发现
接近墙时立即口头要求停止。

- [ ] **Step 3: 结束后强制停车**

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

- [ ] **Step 4: 验收**

通过条件：

- 无碰墙；
- 无人工接管；
- TEB 未持续报告 `trajectory is not feasible`；
- `/cmd_vel` 发布间隔满足 15 Hz 控制基线，没有超过底盘 watchdog；
- AMCL 匹配没有从起点显著下降。

首次通过后重复两次。三次通过才进入完整路线测试；失败则保留 bag，不叠加新参数，
根据定位、全局路径或局部可行性重新提出单一假设。
