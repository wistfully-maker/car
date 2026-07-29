# 稀疏航点引导的平滑 TEB 导航实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建 `ucar_waypoint_nav` ROS包，从当前静态地图自动生成2至3个稀疏中间航点，并通过穿越式目标切换、GlobalPlanner和TEB在线完成P点到二维码观察点的平滑导航。

**Architecture:** 纯Python模块负责地图坐标、航点选择、穿越判定和协议，ROS节点只负责TF、`move_base` action和话题适配。中间航点在制动前提前切换且不主动取消旧目标，正常速度只由TEB发布；GlobalPlanner为默认全局规划器，Navfn通过launch参数切换。

**Tech Stack:** ROS 1 Noetic、Python 3、`rospy`、`actionlib`、`move_base_msgs`、`tf2_ros`、`PyYAML`、GlobalPlanner、TEB、Python `unittest`。

---

## 文件结构

```text
ucar_ws/src/ucar_waypoint_nav/
  CMakeLists.txt
  package.xml
  setup.py
  config/
    pickup_waypoints.yaml
    global_planner.yaml
    navfn.yaml
    teb.yaml
    controller.yaml
  launch/
    waypoint_teb_navigation.launch
  scripts/
    derive_sparse_waypoints.py
    waypoint_route_manager.py
    capture_navigation_run.sh
  src/ucar_waypoint_nav/
    __init__.py
    map_geometry.py
    waypoint_logic.py
    protocol.py
  test/
    test_package_config.py
    test_map_geometry.py
    test_waypoint_logic.py
    test_protocol.py
    test_ros_assets.py
  README.md
```

## Task 1：建立独立ROS包

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/package.xml`
- Create: `ucar_ws/src/ucar_waypoint_nav/CMakeLists.txt`
- Create: `ucar_ws/src/ucar_waypoint_nav/setup.py`
- Create: `ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/__init__.py`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_package_config.py`

- [ ] **Step 1：编写失败的包结构测试**

```python
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_package_name_and_runtime_dependencies(self):
        root = ET.parse(str(ROOT / "package.xml")).getroot()
        self.assertEqual(root.findtext("name"), "ucar_waypoint_nav")
        dependencies = {node.text for node in root.findall("exec_depend")}
        self.assertTrue(
            {"rospy", "actionlib", "move_base_msgs", "tf2_ros", "std_srvs"}
            <= dependencies
        )

    def test_python_scripts_are_installed(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/derive_sparse_waypoints.py", cmake)
        self.assertIn("scripts/waypoint_route_manager.py", cmake)
```

- [ ] **Step 2：运行测试并确认失败**

Run:

```powershell
python -m unittest ucar_ws/src/ucar_waypoint_nav/test/test_package_config.py -v
```

Expected: `package.xml`或`CMakeLists.txt`不存在，测试失败。

- [ ] **Step 3：实现最小catkin包**

`package.xml`使用format 2，声明：

```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>rospy</build_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>move_base_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>std_srvs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
```

`CMakeLists.txt`调用`catkin_python_setup()`，使用`catkin_install_python()`安装两个Python脚本，并安装`config`、`launch`和`README.md`。

`setup.py`：

```python
from setuptools import setup

setup(
    name="ucar_waypoint_nav",
    version="0.1.0",
    packages=["ucar_waypoint_nav"],
    package_dir={"": "src"},
)
```

- [ ] **Step 4：运行包结构测试**

Expected: `Ran 2 tests ... OK`。

- [ ] **Step 5：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav
git commit -m "feat: scaffold waypoint navigation package"
```

## Task 2：实现地图解析、坐标转换和安全栅格

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/map_geometry.py`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_map_geometry.py`

- [ ] **Step 1：编写PGM和坐标转换失败测试**

```python
class MapGeometryTests(unittest.TestCase):
    def test_world_pixel_round_trip_uses_bottom_left_map_origin(self):
        geometry = MapGeometry(
            width=480,
            height=256,
            resolution=0.05,
            origin=(-12.2, -12.2),
        )
        pixel = geometry.world_to_pixel(-1.40219, -0.627908)
        world = geometry.pixel_to_world(*pixel)
        self.assertAlmostEqual(world[0], -1.40219, delta=0.05)
        self.assertAlmostEqual(world[1], -0.627908, delta=0.05)

    def test_inflation_blocks_cells_within_radius(self):
        occupied = [[False] * 7 for _ in range(7)]
        occupied[3][3] = True
        inflated = inflate_obstacles(occupied, radius_cells=2)
        self.assertTrue(inflated[3][5])
        self.assertFalse(inflated[0][0])
```

- [ ] **Step 2：运行测试并确认导入失败**

Expected: `ModuleNotFoundError: ucar_waypoint_nav.map_geometry`。

- [ ] **Step 3：实现纯Python地图几何**

定义：

```python
@dataclass(frozen=True)
class MapGeometry:
    width: int
    height: int
    resolution: float
    origin: tuple

    def world_to_pixel(self, x, y):
        column = int(round((x - self.origin[0]) / self.resolution))
        row = self.height - 1 - int(
            round((y - self.origin[1]) / self.resolution)
        )
        return column, row

    def pixel_to_world(self, column, row):
        return (
            self.origin[0] + column * self.resolution,
            self.origin[1] + (self.height - 1 - row) * self.resolution,
        )
```

同时实现：

- `read_pgm(path)`：支持P5和P2，跳过注释；
- `load_map(yaml_path)`：读取resolution、origin、threshold；
- `inflate_obstacles(grid, radius_cells)`：按欧氏圆盘膨胀；
- `astar(grid, start, goal)`：8邻域，禁止对角穿过两个占用单元夹角；
- `clearance_map(grid)`：多源BFS得到近似障碍距离。

- [ ] **Step 4：补充A*不可穿墙及未知区域测试并运行**

Expected: 地图几何全部通过。

- [ ] **Step 5：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/map_geometry.py \
        ucar_ws/src/ucar_waypoint_nav/test/test_map_geometry.py
git commit -m "feat: add occupancy map geometry"
```

## Task 3：从地图生成稀疏航点和预览

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/scripts/derive_sparse_waypoints.py`
- Create: `ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.yaml`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_map_geometry.py`

- [ ] **Step 1：编写航点稀疏化失败测试**

测试构造包含三个走廊方向变化的栅格，调用：

```python
waypoints = derive_sparse_waypoints(
    path,
    clearance,
    geometry,
    minimum_spacing=0.45,
    maximum_count=3,
)
self.assertGreaterEqual(len(waypoints), 2)
self.assertLessEqual(len(waypoints), 3)
self.assertTrue(all(point.clearance >= 0.25 for point in waypoints))
```

- [ ] **Step 2：确认测试失败**

Expected: `derive_sparse_waypoints`尚未定义。

- [ ] **Step 3：实现确定性候选选择**

算法：

1. 对A*路径以`0.10 m`间距重采样；
2. 使用前后`0.40 m`窗口计算方向变化；
3. 将大于`35°`的变化聚类；
4. 在每个聚类后方`0.20～0.50 m`路径范围内选择clearance最大的单元；
5. 合并间距小于`0.45 m`的点；
6. 若多于3个，保留对路径拓扑方向约束贡献最大的3个；
7. 为中间点设置`pass_through`，最终点使用已确认四元数。

默认膨胀半径使用：

```text
车体外角半径 sqrt(0.171² + 0.128²) + 0.08 m安全余量
```

脚本参数：

```bash
derive_sparse_waypoints.py \
  --map /home/ucar/ucar_ws/src/ucar_nav/maps/map.yaml \
  --start 0 0 0 \
  --goal -1.40219 -0.627908 -3.0878 \
  --output pickup_waypoints.yaml \
  --preview pickup_waypoints.ppm
```

输出YAML必须含地图SHA-256、中间航点、最终点、clearance和切换半径。预览使用标准库写PPM，避免机器人依赖GUI库。

- [ ] **Step 4：在当前正式地图运行脚本**

Run:

```powershell
$env:PYTHONPATH="ucar_ws/src/ucar_waypoint_nav/src"
python ucar_ws/src/ucar_waypoint_nav/scripts/derive_sparse_waypoints.py `
  --map ucar_ws/src/ucar_nav/maps/map.yaml `
  --start 0 0 0 `
  --goal -1.40219 -0.627908 -3.0878 `
  --output ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.yaml `
  --preview ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.ppm
```

Expected: 输出2至3个中间点；起终点和全部线段位于膨胀后的可通行区域。

- [ ] **Step 5：人工查看预览并验证航点在走廊中心**

将PPM转为PNG只用于本地检查；源码保留PPM或YAML，不提交临时PNG。

- [ ] **Step 6：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav/scripts/derive_sparse_waypoints.py \
        ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.yaml \
        ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.ppm \
        ucar_ws/src/ucar_waypoint_nav/test/test_map_geometry.py
git commit -m "feat: derive sparse pickup waypoints"
```

## Task 4：实现必须穿越的航点状态逻辑

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/waypoint_logic.py`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_waypoint_logic.py`

- [ ] **Step 1：编写穿越切换失败测试**

```python
def test_switches_before_intermediate_goal_without_stop():
    manager = RouteProgress(
        waypoints=[pass_through(1.0, 0.0), terminal(2.0, 0.0)],
        minimum_pass_speed=0.08,
    )
    decision = manager.update(
        x=0.72,
        y=0.0,
        yaw=0.0,
        linear_speed=0.20,
        now=1.0,
    )
    assert decision.send_next_goal
    assert not decision.publish_stop
    assert manager.current_index == 1


def test_does_not_accept_a_stopped_intermediate_waypoint():
    manager = RouteProgress(
        waypoints=[pass_through(1.0, 0.0), terminal(2.0, 0.0)],
        minimum_pass_speed=0.08,
    )
    decision = manager.update(
        x=0.75,
        y=0.0,
        yaw=0.0,
        linear_speed=0.0,
        now=1.0,
    )
    assert not decision.send_next_goal
    assert decision.error == "pass-through speed below minimum"
```

- [ ] **Step 2：确认测试失败**

Expected: `RouteProgress`不存在。

- [ ] **Step 3：实现纯状态逻辑**

类型：

```python
@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    kind: str
    switch_radius: float
    exit_radius: float
    heading_tolerance: float


@dataclass(frozen=True)
class Decision:
    send_next_goal: bool = False
    arrived: bool = False
    publish_stop: bool = False
    error: str = ""
```

`RouteProgress.update()`要求：

- 中间点只有在进入switch radius、方向满足且速度不低于下限时切换；
- 不生成停车命令；
- 每点仅切换一次；
- 定位倒退到进入半径外不回退索引；
- 最终点只有位置、航向、速度和稳定时间同时满足才`arrived`。

- [ ] **Step 4：补充迟滞、重复回调和最终点测试**

Expected: 全部通过。

- [ ] **Step 5：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/waypoint_logic.py \
        ucar_ws/src/ucar_waypoint_nav/test/test_waypoint_logic.py
git commit -m "feat: add pass-through waypoint state machine"
```

## Task 5：实现任务协议

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/protocol.py`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_protocol.py`

- [ ] **Step 1：编写任务标识保持测试**

输入：

```json
{"protocol_version":1,"task_id":"task-001","goal_id":"pickup-001"}
```

测试成功输出保留三个字段并增加`status:"arrived"`；无`task_id`、版本错误或重复
`goal_id`必须拒绝。

- [ ] **Step 2：确认失败并实现**

实现`parse_pickup_goal(text)`、`make_pickup_result(identity, status, message)`，JSON使用
UTF-8、稳定键名和非空失败message。

- [ ] **Step 3：运行测试并提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/protocol.py \
        ucar_ws/src/ucar_waypoint_nav/test/test_protocol.py
git commit -m "feat: add waypoint navigation protocol"
```

## Task 6：实现ROS航点管理节点

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/scripts/waypoint_route_manager.py`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py`

- [ ] **Step 1：编写ROS资源静态测试**

测试脚本包含：

- `SimpleActionClient("/move_base", MoveBaseAction)`；
- TF查询`map -> base_link`；
- `/task/pickup_navigation_goal`订阅；
- `/task/pickup_arrived`发布；
- 状态和诊断发布；
- start/cancel服务；
- `rospy.on_shutdown`中取消目标。

- [ ] **Step 2：确认失败并实现节点**

节点行为：

1. 加载YAML并验证地图校验值；
2. 检查`move_base` action可用；
3. 验证起点位置和航向容差；
4. 发送第一个中间目标；
5. 以20 Hz读取TF、odom和action状态；
6. 穿越条件满足时直接`send_goal(next_goal)`，不调用`cancel_goal()`；
7. 中间过程不发布`/cmd_vel`；
8. 最终成功后发布到达JSON；
9. action失败、TF超时、低速卡停或阶段超时发布失败并取消。

- [ ] **Step 3：运行Windows静态测试**

Expected: 所有不依赖ROS运行时的测试通过。

- [ ] **Step 4：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav/scripts/waypoint_route_manager.py \
        ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py
git commit -m "feat: add ROS waypoint route manager"
```

## Task 7：建立GlobalPlanner、Navfn和TEB单一启动链

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/config/global_planner.yaml`
- Create: `ucar_ws/src/ucar_waypoint_nav/config/navfn.yaml`
- Create: `ucar_ws/src/ucar_waypoint_nav/config/teb.yaml`
- Create: `ucar_ws/src/ucar_waypoint_nav/config/controller.yaml`
- Create: `ucar_ws/src/ucar_waypoint_nav/launch/waypoint_teb_navigation.launch`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py`

- [ ] **Step 1：编写launch/YAML失败测试**

验证：

- launch参数`global_planner`只接受`global_planner`或`navfn`；
- `base_global_planner`按参数设置；
- `base_local_planner`固定为`teb_local_planner/TebLocalPlannerROS`；
- 不包含拐点监督器；
- TEB `free_goal_vel: true`；
- `no_inner_iterations: 5`、`no_outer_iterations: 4`；
- controller frequency初始为10 Hz；
- footprint与实车矩形一致。

- [ ] **Step 2：实现配置**

GlobalPlanner使用已确认配置；Navfn只加载Navfn命名空间。TEB初始配置使用：

```yaml
TebLocalPlannerROS:
  max_vel_x: 0.35
  max_vel_x_backwards: 0.05
  max_vel_y: 0.20
  max_vel_theta: 0.60
  acc_lim_x: 0.80
  acc_lim_y: 0.80
  acc_lim_theta: 1.50
  min_turning_radius: 0.0
  free_goal_vel: true
  no_inner_iterations: 5
  no_outer_iterations: 4
  global_plan_viapoint_sep: 0.25
  max_global_plan_lookahead_dist: 1.5
  allow_init_with_backwards_motion: false
  weight_kinematics_forward_drive: 100.0
```

使用以下实车几何和第一轮障碍参数，不从其他组配置复制：

```yaml
footprint:
  - [0.171, -0.128]
  - [0.171, 0.128]
  - [-0.171, 0.128]
  - [-0.171, -0.128]

TebLocalPlannerROS:
  min_obstacle_dist: 0.15
  inflation_dist: 0.25
  include_costmap_obstacles: true
  costmap_obstacles_behind_robot_dist: 1.0
```

- [ ] **Step 3：运行静态测试和`xmllint`**

机器人侧运行：

```bash
roslaunch --files ucar_waypoint_nav waypoint_teb_navigation.launch
rosparam get /move_base/base_global_planner
rosparam get /move_base/base_local_planner
```

- [ ] **Step 4：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav/config \
        ucar_ws/src/ucar_waypoint_nav/launch \
        ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py
git commit -m "feat: add switchable online planner stack"
```

## Task 8：诊断、README和部署验证

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/scripts/capture_navigation_run.sh`
- Create: `ucar_ws/src/ucar_waypoint_nav/README.md`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py`

- [ ] **Step 1：实现一次测试的诊断采集**

脚本建立时间戳目录，保存：

- `rosparam dump`；
- 节点和话题所有权；
- TF检查；
- `rostopic hz`；
- `rosbag record /scan /tf /tf_static /amcl_pose /odom /cmd_vel
  /move_base/GlobalPlanner/plan /move_base/NavfnROS/plan
  /move_base/TebLocalPlannerROS/local_plan
  /move_base/local_costmap/costmap
  /move_base/local_costmap/costmap_updates
  /ucar_waypoint_nav/state /ucar_waypoint_nav/diagnostic`；
- move_base日志路径；
- Git commit和地图SHA-256。

- [ ] **Step 2：编写完整中文README**

必须覆盖：

- 依赖和编译；
- 一键启动；
- 分层启动；
- GlobalPlanner/Navfn切换；
- AMCL初始化；
- 手动启动路线；
- 编排器话题启动；
- 取消、急停和关闭；
- 航点重新生成；
- 所有参数位置与单变量调法；
- 中间航点停车的排查；
- `trajectory is not feasible`排查；
- 碰墙证据链；
- 日志位置和复现实验格式。

- [ ] **Step 3：运行全部本地测试**

```powershell
$env:PYTHONPATH="ucar_ws/src/ucar_waypoint_nav/src"
python -m unittest discover `
  -s ucar_ws/src/ucar_waypoint_nav/test `
  -p "test_*.py" -v
```

Expected: 全部通过。

- [ ] **Step 4：部署到小车并编译**

只同步`ucar_waypoint_nav`到：

```text
/home/ucar/ucar_ws/src/ucar_waypoint_nav
```

运行：

```bash
cd /home/ucar/ucar_ws
catkin_make --pkg ucar_waypoint_nav
source devel/setup.bash
python3 -m unittest discover \
  -s src/ucar_waypoint_nav/test -p 'test_*.py' -v
```

- [ ] **Step 5：静态启动验证**

不发送目标，只验证：

- 无重复节点；
- 只有move_base拥有正常`/cmd_vel`；
- GlobalPlanner持续产生路径的前置依赖正常；
- TEB控制周期达到配置目标；
- manager处于`IDLE`；
- start/cancel服务存在。

- [ ] **Step 6：提交**

```bash
git add ucar_ws/src/ucar_waypoint_nav
git commit -m "docs: add waypoint navigation operations guide"
```

## Task 9：实车阶梯调试

**Files:**
- Modify: `ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.yaml`
- Modify: `ucar_ws/src/ucar_waypoint_nav/config/teb.yaml`
- Modify: `ucar_ws/src/ucar_waypoint_nav/config/controller.yaml`

- [ ] **Step 1：第一段和第一中间航点**

仅启用首个pass-through点，记录切换前后速度。要求实际速度不降至停车阈值。

- [ ] **Step 2：连续弯**

启用全部中间点；若碰墙，先核对同一时刻scan、TF、costmap和footprint，不先改轨迹
权重。

- [ ] **Step 3：最终观察点**

验证最终位置、最终朝向、停车和二维码模块接管。

- [ ] **Step 4：单变量提速**

依次调整：

1. 直线`max_vel_x`；
2. 弯道`max_vel_theta/max_vel_y`；
3. 加速度；
4. 穿越切换半径；
5. TEB时间最优权重。

每次只改一个变量并记录完整运行。

- [ ] **Step 5：三轮验收并提交比赛基线**

连续三轮满足设计验收后，提交最终YAML、航点预览和测试记录：

```bash
git add ucar_ws/src/ucar_waypoint_nav/config \
        ucar_ws/src/ucar_waypoint_nav/README.md
git commit -m "feat: establish waypoint TEB competition baseline"
```
