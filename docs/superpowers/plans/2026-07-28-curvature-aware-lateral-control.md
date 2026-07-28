# 曲率感知横移控制与 AMCL 初始化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 `NavfnROS + TebLocalPlannerROS` 增加直线/直角弯横移模式切换，并提供一键 AMCL 固定起点初始化脚本。

**Architecture:** 将可离线测试的路径几何与状态机放在 `lateral_mode_logic.py`，ROS 节点只负责订阅路径、查询 TF、调用 TEB dynamic reconfigure 和发布诊断。节点在直线模式使用 `vy=0.02 m/s`，识别到前方直角弯后使用 `vy=0.18 m/s`，出弯且车头转正 0.5 秒后恢复直线模式。

**Tech Stack:** ROS 1 Noetic、Python 3、`rospy`、`nav_msgs`、`tf2_ros`、`dynamic_reconfigure`、`unittest`、YAML。

---

## 文件结构

- Create: `ucar_ws/src/ucar_nav/scripts/lateral_mode_logic.py`
  - 纯路径重采样、角度计算和状态机，不依赖 ROS。
- Create: `ucar_ws/src/ucar_nav/scripts/teb_lateral_mode_controller.py`
  - ROS 输入输出、TF 和 TEB dynamic reconfigure 适配。
- Create: `ucar_ws/src/ucar_nav/config/lateral_mode_controller.yaml`
  - 所有曲率、速度、超时和话题参数。
- Create: `ucar_ws/src/ucar_nav/launch/teb_lateral_mode_controller.launch`
  - 独立启动控制节点。
- Create: `ucar_ws/src/ucar_nav/scripts/initialize_amcl.py`
  - 一键发布固定起点。
- Create: `ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py`
  - 路径几何和状态机离线测试。
- Create: `ucar_ws/src/ucar_nav/test/test_initialize_amcl.py`
  - 四元数和协方差构造测试。
- Modify: `ucar_ws/src/ucar_nav/config/local_planners/teb_corner_safe.yaml`
  - 默认采用直线模式横移参数并恢复计算预算内的优化次数。
- Modify: `ucar_ws/src/ucar_nav/launch/navigation_stack.launch`
  - 仅在 `navfn_teb_corner` Profile 可选包含控制器。
- Modify: `ucar_ws/src/ucar_nav/CMakeLists.txt`
  - 安装新增脚本。
- Modify: `ucar_ws/src/ucar_nav/package.xml`
  - 声明运行依赖。
- Modify: `ucar_ws/src/ucar_nav/README.md`
  - 记录启动、诊断、参数和 AMCL 命令。

### Task 1: 路径几何与状态机

**Files:**
- Create: `ucar_ws/src/ucar_nav/scripts/lateral_mode_logic.py`
- Create: `ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py`

- [ ] **Step 1: 写直线、锯齿和直角弯失败测试**

测试通过 `importlib.util.spec_from_file_location` 加载脚本，构造以下路径：

```python
STRAIGHT = [(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)]
JAGGED = [(0.0, 0.0), (0.2, 0.01), (0.4, -0.01), (0.8, 0.0)]
RIGHT_ANGLE = [(0.0, 0.0), (0.4, 0.0), (0.8, 0.0),
               (0.8, 0.4), (0.8, 0.8)]
```

断言：

```python
self.assertLess(metrics(STRAIGHT).turn_angle_deg, 10.0)
self.assertLess(metrics(JAGGED).turn_angle_deg, 10.0)
self.assertGreater(metrics(RIGHT_ANGLE).turn_angle_deg, 45.0)
```

- [ ] **Step 2: 运行测试并确认红灯**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py -v
```

Expected: FAIL，原因是 `lateral_mode_logic.py` 不存在。

- [ ] **Step 3: 实现纯路径几何**

实现以下公开接口：

```python
@dataclass(frozen=True)
class PathMetrics:
    turn_angle_deg: float
    exit_heading_rad: float
    analyzed_length: float

def normalize_angle(angle: float) -> float: ...
def resample_path(points, spacing: float): ...
def analyze_path(points, lookahead_distance: float,
                 resample_spacing: float) -> PathMetrics: ...
```

规则：

- 删除相邻距离小于 `1e-6` 的重复点；
- 沿弧长以 `resample_spacing` 重采样；
- 截取累计长度不超过 `lookahead_distance` 的点；
- 入口方向取前 30% 线段的向量和；
- 出口方向取后 30% 线段的向量和；
- `turn_angle_deg` 使用归一化后的入口/出口方向差绝对值；
- 少于三个有效点时返回零转角。

- [ ] **Step 4: 增加状态机失败测试**

公开接口：

```python
class LateralModeStateMachine:
    def update(self, now, metrics, robot_heading_rad,
               plan_valid=True) -> str: ...
```

测试：

- `45°` 以上立即从 `STRAIGHT` 进入 `CORNER`；
- 弯后路径已直但车头误差超过 `10°` 时保持 `CORNER`；
- 路径转角低于 `10°` 且车头误差低于 `10°`，连续 `0.5 s` 后退出；
- 中间一次不满足会重置退出计时；
- `plan_valid=False` 返回 `STRAIGHT`。

- [ ] **Step 5: 实现状态机并跑绿**

构造参数：

```python
LateralModeStateMachine(
    enter_angle_deg=45.0,
    exit_angle_deg=10.0,
    heading_exit_tolerance_deg=10.0,
    exit_hold_time=0.5,
)
```

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py -v
```

Expected: 所有测试 PASS。

- [ ] **Step 6: 提交**

```powershell
git add ucar_ws/src/ucar_nav/scripts/lateral_mode_logic.py `
        ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py
git commit -m "feat: add lateral mode path analysis"
```

### Task 2: TEB 横移模式 ROS 控制节点

**Files:**
- Create: `ucar_ws/src/ucar_nav/scripts/teb_lateral_mode_controller.py`
- Create: `ucar_ws/src/ucar_nav/config/lateral_mode_controller.yaml`
- Create: `ucar_ws/src/ucar_nav/launch/teb_lateral_mode_controller.launch`
- Test: `ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py`

- [ ] **Step 1: 为模式参数映射写失败测试**

在纯逻辑模块增加：

```python
def mode_parameters(mode, straight_max_vel_y, straight_acc_lim_y,
                    corner_max_vel_y, corner_acc_lim_y): ...
```

断言：

```python
self.assertEqual(
    {"max_vel_y": 0.02, "acc_lim_y": 0.20},
    mode_parameters("STRAIGHT", 0.02, 0.20, 0.18, 0.60),
)
self.assertEqual(
    {"max_vel_y": 0.18, "acc_lim_y": 0.60},
    mode_parameters("CORNER", 0.02, 0.20, 0.18, 0.60),
)
```

- [ ] **Step 2: 跑红并实现最小映射**

Run:

```powershell
python ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py -v
```

Expected: 先 FAIL；实现后 PASS。

- [ ] **Step 3: 创建 ROS 参数文件**

写入：

```yaml
controller_rate: 5.0
plan_topic: /move_base/NavfnROS/plan
map_frame: map
base_frame: base_link
lookahead_distance: 0.8
resample_spacing: 0.10
corner_enter_angle_deg: 45.0
corner_exit_angle_deg: 10.0
heading_exit_tolerance_deg: 10.0
exit_hold_time: 0.5
plan_timeout: 1.0
straight_max_vel_y: 0.02
straight_acc_lim_y: 0.20
corner_max_vel_y: 0.18
corner_acc_lim_y: 0.60
reconfigure_namespace: /move_base/TebLocalPlannerROS
```

- [ ] **Step 4: 实现 ROS 适配节点**

节点流程：

```python
rospy.Subscriber(plan_topic, Path, self._plan_callback, queue_size=1)
self._tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
self._client = dynamic_reconfigure.client.Client(
    reconfigure_namespace, timeout=5.0
)
self._timer = rospy.Timer(
    rospy.Duration(1.0 / controller_rate), self._tick
)
```

`_tick` 必须：

1. 检查路径时间是否超过 `plan_timeout`；
2. 查询 `map -> base_link`；
3. 找最近路径点，只分析其后的路径；
4. 调用纯逻辑模块；
5. 仅在模式改变或上一次更新失败时调用：

```python
self._client.update_configuration({
    "max_vel_y": target["max_vel_y"],
    "acc_lim_y": target["acc_lim_y"],
})
```

6. 发布 `/navigation/lateral_mode`；
7. 发布 JSON 诊断到 `/navigation/lateral_mode_diagnostics`。

关闭回调尽力恢复直线参数。路径/TF 失效时请求 `STRAIGHT`。

- [ ] **Step 5: 创建独立 launch**

```xml
<launch>
  <arg name="config"
       default="$(find ucar_nav)/config/lateral_mode_controller.yaml"/>
  <node name="teb_lateral_mode_controller"
        pkg="ucar_nav"
        type="teb_lateral_mode_controller.py"
        output="screen"
        respawn="false">
    <rosparam command="load" file="$(arg config)"/>
  </node>
</launch>
```

- [ ] **Step 6: 本地静态验证**

Run:

```powershell
python -m py_compile `
  ucar_ws/src/ucar_nav/scripts/lateral_mode_logic.py `
  ucar_ws/src/ucar_nav/scripts/teb_lateral_mode_controller.py
python -c "import xml.etree.ElementTree as E; E.parse('ucar_ws/src/ucar_nav/launch/teb_lateral_mode_controller.launch')"
```

Expected: 无输出且退出码为 0。

- [ ] **Step 7: 提交**

```powershell
git add ucar_ws/src/ucar_nav/scripts/teb_lateral_mode_controller.py `
        ucar_ws/src/ucar_nav/config/lateral_mode_controller.yaml `
        ucar_ws/src/ucar_nav/launch/teb_lateral_mode_controller.launch `
        ucar_ws/src/ucar_nav/scripts/lateral_mode_logic.py `
        ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py
git commit -m "feat: switch TEB lateral limits by path curvature"
```

### Task 3: AMCL 一键初始化脚本

**Files:**
- Create: `ucar_ws/src/ucar_nav/scripts/initialize_amcl.py`
- Create: `ucar_ws/src/ucar_nav/test/test_initialize_amcl.py`

- [ ] **Step 1: 写四元数和协方差失败测试**

脚本公开纯函数：

```python
def yaw_to_quaternion(yaw): ...
def build_covariance(cov_x, cov_y, cov_yaw): ...
```

断言：

- `yaw=0` 得到 `z=0, w=1`；
- `yaw=π` 得到 `abs(z)=1, abs(w)<1e-6`；
- 协方差长度为 36；
- 索引 `0/7/35` 分别为 `cov_x/cov_y/cov_yaw`，其余为 0。

- [ ] **Step 2: 运行测试确认红灯**

```powershell
python ucar_ws/src/ucar_nav/test/test_initialize_amcl.py -v
```

Expected: FAIL，脚本不存在。

- [ ] **Step 3: 实现脚本**

参数：

```python
frame_id = rospy.get_param("~frame_id", "map")
x = rospy.get_param("~x", 0.0)
y = rospy.get_param("~y", 0.0)
yaw = rospy.get_param("~yaw", 0.0)
cov_x = rospy.get_param("~covariance_x", 0.10)
cov_y = rospy.get_param("~covariance_y", 0.10)
cov_yaw = rospy.get_param("~covariance_yaw", 0.0685)
timeout = rospy.get_param("~subscriber_timeout", 10.0)
```

创建 latched publisher，循环等待 `publisher.get_num_connections() > 0`。超时打印错误并
以退出码 2 结束；成功时发布一次并等待 0.5 秒。

- [ ] **Step 4: 运行测试并提交**

```powershell
python ucar_ws/src/ucar_nav/test/test_initialize_amcl.py -v
git add ucar_ws/src/ucar_nav/scripts/initialize_amcl.py `
        ucar_ws/src/ucar_nav/test/test_initialize_amcl.py
git commit -m "feat: add AMCL initialization helper"
```

### Task 4: 包依赖与导航启动接入

**Files:**
- Modify: `ucar_ws/src/ucar_nav/CMakeLists.txt`
- Modify: `ucar_ws/src/ucar_nav/package.xml`
- Modify: `ucar_ws/src/ucar_nav/config/local_planners/teb_corner_safe.yaml`
- Modify: `ucar_ws/src/ucar_nav/launch/navigation_stack.launch`
- Test: `ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **Step 1: 写启动与依赖失败测试**

断言：

- `package.xml` 包含 `dynamic_reconfigure`、`geometry_msgs`、`tf2_ros`；
- `CMakeLists.txt` 安装三个新脚本；
- `navigation_stack.launch` 声明
  `<arg name="enable_lateral_mode_controller" default="true"/>`；
- 仅 `navfn_teb_corner` 组包含控制器 launch；
- TEB YAML 默认值为 `max_vel_y=0.02`、`acc_lim_y=0.20`；
- 优化次数为 `2×1`，避免复现 1 秒控制周期。

- [ ] **Step 2: 运行测试确认红灯**

```powershell
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 新增断言 FAIL。

- [ ] **Step 3: 修改依赖和安装**

在 `find_package(catkin REQUIRED COMPONENTS ...)` 和 `package.xml` 增加：

```text
dynamic_reconfigure
geometry_msgs
tf2_ros
```

在 `install(PROGRAMS ...)` 增加：

```text
scripts/lateral_mode_logic.py
scripts/teb_lateral_mode_controller.py
scripts/initialize_amcl.py
```

- [ ] **Step 4: 恢复计算预算并设直线默认值**

`teb_corner_safe.yaml`：

```yaml
max_vel_y: 0.02
acc_lim_y: 0.20
no_inner_iterations: 2
no_outer_iterations: 1
```

保留 `max_vel_x=0.45`、`max_vel_theta=0.60`，不在本任务修改其他速度。

- [ ] **Step 5: 接入导航 launch**

顶层增加：

```xml
<arg name="enable_lateral_mode_controller" default="true"/>
```

在 `navfn_teb_corner` 组末尾增加：

```xml
<include if="$(arg enable_lateral_mode_controller)"
         file="$(find ucar_nav)/launch/teb_lateral_mode_controller.launch"/>
```

其他三个 Profile 不得启动控制器。

- [ ] **Step 6: 全量本地验证**

```powershell
python ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py -v
python ucar_ws/src/ucar_nav/test/test_initialize_amcl.py -v
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 新增测试全部 PASS；若旧测试仍断言旧的 `local_planner:=dwa` 接口，更新为四
Profile 合约后重新运行，不能通过添加无效兼容参数绕过测试。

- [ ] **Step 7: 提交**

```powershell
git add ucar_ws/src/ucar_nav/CMakeLists.txt `
        ucar_ws/src/ucar_nav/package.xml `
        ucar_ws/src/ucar_nav/config/local_planners/teb_corner_safe.yaml `
        ucar_ws/src/ucar_nav/launch/navigation_stack.launch `
        ucar_ws/src/ucar_nav/test/test_navigation_config.py
git commit -m "feat: integrate curvature-aware TEB control"
```

### Task 5: 文档、本地构建与小车部署

**Files:**
- Modify: `ucar_ws/src/ucar_nav/README.md`

- [ ] **Step 1: 更新 README**

必须包含：

```bash
roslaunch ucar_nav navigation_stack.launch \
  navigation_profile:=navfn_teb_corner \
  enable_lateral_mode_controller:=true

rosrun ucar_nav initialize_amcl.py

rostopic echo /navigation/lateral_mode
rostopic echo /navigation/lateral_mode_diagnostics

rosrun dynamic_reconfigure dynparam get \
  /move_base/TebLocalPlannerROS max_vel_y
```

记录所有控制参数所在文件：

```text
config/lateral_mode_controller.yaml
```

- [ ] **Step 2: 本地验证**

```powershell
python -m py_compile ucar_ws/src/ucar_nav/scripts/*.py
python ucar_ws/src/ucar_nav/test/test_lateral_mode_logic.py -v
python ucar_ws/src/ucar_nav/test/test_initialize_amcl.py -v
python ucar_ws/src/ucar_nav/test/test_navigation_config.py -v
```

Expected: 全部通过。

- [ ] **Step 3: 提交文档**

```powershell
git add ucar_ws/src/ucar_nav/README.md
git commit -m "docs: explain lateral mode and AMCL initialization"
```

- [ ] **Step 4: 部署前备份**

在小车上创建带时间戳的备份目录，只备份本计划修改的文件。不得覆盖地图和
`/home/ucar/waypoints.xml`。

- [ ] **Step 5: 部署并构建**

同步 `ucar_nav` 修改文件到：

```text
/home/ucar/ucar_ws/src/ucar_nav
```

然后运行：

```bash
cd /home/ucar/ucar_ws
catkin_make
source devel/setup.bash
```

Expected: 构建成功。

- [ ] **Step 6: 静态 ROS 联调**

启动导航但不发送目标：

```bash
roslaunch ucar_nav navigation_stack.launch \
  navigation_profile:=navfn_teb_corner \
  enable_lateral_mode_controller:=true
```

验证：

```bash
rosnode list | grep teb_lateral_mode_controller
rostopic echo -n 1 /navigation/lateral_mode
rosrun dynamic_reconfigure dynparam get \
  /move_base/TebLocalPlannerROS max_vel_y
```

Expected: 模式为 `STRAIGHT`，`max_vel_y=0.02`。

- [ ] **Step 7: 实车测试**

1. 将小车放回固定起点；
2. 执行 `rosrun ucar_nav initialize_amcl.py`；
3. 发送二维码航点；
4. 记录直线、入弯、弯中、出弯四阶段模式；
5. 人工看护并在碰撞趋势出现时立即停止；
6. 保存 `move_base`、模式诊断和 AMCL 日志；
7. 只依据诊断结果单项修改阈值，不同时修改多组参数。

验收：

- 直线阶段保持 `STRAIGHT`；
- 弯前切换 `CORNER`；
- 出弯且车头转正后恢复 `STRAIGHT`；
- 不持续出现控制循环低于 5 Hz；
- 不出现明显直线横移或墙体碰撞。
