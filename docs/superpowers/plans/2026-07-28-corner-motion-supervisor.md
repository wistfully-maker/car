# 独立精确过弯监督器实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新建独立 ROS 包 `ucar_corner_supervisor`，通过速度仲裁和 TF 航向闭环完成直角弯精确转向，同时不修改 `ucar_nav`。

**Architecture:** move_base 的速度输出由团队 launch 重映射到 `/move_base/cmd_vel_raw`。监督器分析 Navfn 全局路径，在普通路段转发受限速度，到达拐角触发距离后临时忽略 TEB 速度并原地闭环转向，转正后恢复转发。

**Tech Stack:** ROS 1 Noetic、Python 3、`rospy`、`nav_msgs`、`geometry_msgs`、`actionlib_msgs`、`tf2_ros`、YAML、`unittest`

---

## 文件结构

```text
ucar_ws/src/ucar_corner_supervisor/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── corner_supervisor.yaml
├── launch/
│   └── corner_supervisor.launch
├── scripts/
│   ├── corner_geometry.py
│   └── corner_supervisor_node.py
└── test/
    ├── test_corner_geometry.py
    ├── test_corner_state_machine.py
    └── test_package_config.py
```

### Task 1：创建独立 ROS 包骨架

**Files:**
- Create: `ucar_ws/src/ucar_corner_supervisor/package.xml`
- Create: `ucar_ws/src/ucar_corner_supervisor/CMakeLists.txt`
- Create: `ucar_ws/src/ucar_corner_supervisor/test/test_package_config.py`

- [ ] **Step 1：写包结构失败测试**

测试必须断言包名为 `ucar_corner_supervisor`，依赖包含
`actionlib_msgs/geometry_msgs/nav_msgs/rospy/std_msgs/tf2_ros`，且 CMake 安装两个脚本。

- [ ] **Step 2：运行并确认红灯**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_package_config.py -v
```

Expected: FAIL，原因是包文件不存在。

- [ ] **Step 3：实现最小包配置**

`package.xml` 使用 format 2；`CMakeLists.txt` 的 `install(PROGRAMS ...)` 包含：

```cmake
scripts/corner_geometry.py
scripts/corner_supervisor_node.py
```

- [ ] **Step 4：运行测试并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_package_config.py -v
git add ucar_ws/src/ucar_corner_supervisor
git commit -m "feat: scaffold corner supervisor package"
```

### Task 2：实现拐角几何检测

**Files:**
- Create: `ucar_ws/src/ucar_corner_supervisor/scripts/corner_geometry.py`
- Create: `ucar_ws/src/ucar_corner_supervisor/test/test_corner_geometry.py`

- [ ] **Step 1：写几何失败测试**

公开接口：

```python
@dataclass(frozen=True)
class CornerObservation:
    found: bool
    distance_to_corner: float
    turn_angle_rad: float
    exit_heading_rad: float

def normalize_angle(angle: float) -> float: ...
def resample_path(points, spacing: float): ...
def find_first_corner(points, spacing: float,
                      min_angle_deg: float,
                      search_distance: float) -> CornerObservation: ...
```

测试：

- 直线返回 `found=False`；
- 90° 左弯返回正转角、拐角距离约 `0.8 m`、出口方向约 `pi/2`；
- 90° 右弯返回负转角；
- 轻微锯齿不误判；
- 重复点不报错；
- 搜索距离不足时不返回远处拐角。

- [ ] **Step 2：运行并确认红灯**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_corner_geometry.py -v
```

Expected: FAIL，脚本不存在。

- [ ] **Step 3：实现最小几何算法**

规则：

- 删除相邻距离小于 `1e-6` 的点；
- 按弧长重采样；
- 对每个候选点分别累计候选点前后 `0.25 m` 的方向向量；
- 第一个方向差达到阈值的候选点即拐角；
- `distance_to_corner` 为路径起点到候选点的累计弧长；
- 只接受不超过 `search_distance` 的候选点。

- [ ] **Step 4：运行测试并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_corner_geometry.py -v
git add ucar_ws/src/ucar_corner_supervisor/scripts/corner_geometry.py `
        ucar_ws/src/ucar_corner_supervisor/test/test_corner_geometry.py
git commit -m "feat: detect first corner and its distance"
```

### Task 3：实现纯状态机和速度仲裁

**Files:**
- Modify: `ucar_ws/src/ucar_corner_supervisor/scripts/corner_geometry.py`
- Create: `ucar_ws/src/ucar_corner_supervisor/test/test_corner_state_machine.py`

- [ ] **Step 1：写状态机失败测试**

公开接口：

```python
@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float
    linear_y: float
    angular_z: float

class CornerSupervisorStateMachine:
    def update(self, now, goal_active, raw_fresh, tf_valid,
               observation, robot_heading, raw_command) -> VelocityCommand: ...
```

测试必须覆盖：

- 无活动目标、原始速度超时或 TF 失效输出全零；
- `FOLLOWING` 转发 `x/theta`，并把 `y` 限制为 `±0.02`；
- 拐角距离大于 `0.25 m` 时不提前转向；
- 距离小于等于 `0.25 m` 时进入 `TURNING`；
- `TURNING` 输出 `x=0/y=0`，角速度方向与航向误差一致；
- 角速度受 `0.18–0.35 rad/s` 限制；
- 误差小于 `8°` 连续 `0.3 s` 后恢复；
- 转向超过 `8 s` 输出零并进入 `ERROR`；
- 同一拐角在释放距离内不会重复触发。

- [ ] **Step 2：运行并确认红灯**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_corner_state_machine.py -v
```

Expected: FAIL，状态机不存在。

- [ ] **Step 3：实现最小状态机**

状态固定为：

```text
IDLE -> FOLLOWING -> TURNING -> EXIT_ALIGN -> FOLLOWING
                             \-> ERROR
```

`TURNING` 角速度：

```python
angular = clamp(turn_kp * heading_error,
                turn_min_vel_theta,
                turn_max_vel_theta)
```

符号必须取自归一化航向误差，进入容差后输出零角速度。

- [ ] **Step 4：运行测试并提交**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_corner_state_machine.py -v
git add ucar_ws/src/ucar_corner_supervisor/scripts/corner_geometry.py `
        ucar_ws/src/ucar_corner_supervisor/test/test_corner_state_machine.py
git commit -m "feat: add deterministic corner motion state machine"
```

### Task 4：实现 ROS 速度仲裁节点

**Files:**
- Create: `ucar_ws/src/ucar_corner_supervisor/scripts/corner_supervisor_node.py`
- Create: `ucar_ws/src/ucar_corner_supervisor/config/corner_supervisor.yaml`
- Create: `ucar_ws/src/ucar_corner_supervisor/launch/corner_supervisor.launch`
- Modify: `ucar_ws/src/ucar_corner_supervisor/test/test_package_config.py`

- [ ] **Step 1：扩展静态失败测试**

断言：

- YAML 包含设计中的全部参数；
- launch 只启动 `corner_supervisor_node.py`；
- launch 不 include `ucar_nav`；
- launch 不启动 `move_base/base_driver/ydlidar`；
- README 或 launch 不修改 `/home/ucar/ucar_ws/src/ucar_nav`。

- [ ] **Step 2：运行并确认红灯**

```powershell
python ucar_ws/src/ucar_corner_supervisor/test/test_package_config.py -v
```

Expected: 新断言 FAIL。

- [ ] **Step 3：实现 ROS 节点**

节点必须：

```python
rospy.Subscriber("/move_base/cmd_vel_raw", Twist, raw_callback, queue_size=1)
rospy.Subscriber("/move_base/NavfnROS/plan", Path, plan_callback, queue_size=1)
rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback, queue_size=1)
rospy.Publisher("/cmd_vel", Twist, queue_size=1)
```

每个控制周期查询 `map -> base_link`，截取车辆最近路径点之后的路径并调用纯逻辑。
发布状态和 JSON 诊断。关闭回调连续发布零速度。原始速度超时、TF 失效和 `ERROR`
状态均不得转发 TEB 指令。

- [ ] **Step 4：创建参数和 launch**

参数使用设计默认值；launch 只加载 YAML 并启动监督器。

- [ ] **Step 5：静态验证并提交**

```powershell
python -m py_compile `
  ucar_ws/src/ucar_corner_supervisor/scripts/corner_geometry.py `
  ucar_ws/src/ucar_corner_supervisor/scripts/corner_supervisor_node.py
python -c "import xml.etree.ElementTree as E; E.parse('ucar_ws/src/ucar_corner_supervisor/launch/corner_supervisor.launch')"
python -m unittest discover `
  -s ucar_ws/src/ucar_corner_supervisor/test `
  -p "test_*.py" -v
git add ucar_ws/src/ucar_corner_supervisor
git commit -m "feat: arbitrate move_base velocity at corners"
```

### Task 5：文档、部署和无运动联调

**Files:**
- Create: `ucar_ws/src/ucar_corner_supervisor/README.md`

- [ ] **Step 1：编写 README**

必须写明团队 launch 唯一所需修改：

```xml
<remap from="/cmd_vel" to="/move_base/cmd_vel_raw"/>
```

并包含：

```bash
roslaunch ucar_corner_supervisor corner_supervisor.launch
rostopic info /cmd_vel
rostopic info /move_base/cmd_vel_raw
rostopic echo /navigation/corner_supervisor/state
rostopic echo /navigation/corner_supervisor/diagnostics
```

说明启动顺序、停止顺序、全部参数、日志记录、ERROR 恢复和单发布者检查。

- [ ] **Step 2：全量本地验证**

```powershell
python -m py_compile ucar_ws/src/ucar_corner_supervisor/scripts/*.py
python -m unittest discover `
  -s ucar_ws/src/ucar_corner_supervisor/test `
  -p "test_*.py" -v
git diff --check
```

- [ ] **Step 3：提交文档**

```powershell
git add ucar_ws/src/ucar_corner_supervisor/README.md
git commit -m "docs: explain corner supervisor integration"
```

- [ ] **Step 4：只部署新包**

只同步：

```text
ucar_ws/src/ucar_corner_supervisor/
```

若车端同名目录存在，先备份到：

```text
/home/ucar/ucar_corner_supervisor_backups/<timestamp>
```

严禁同步 `ucar_nav`。

- [ ] **Step 5：车端构建**

```bash
cd /home/ucar/ucar_ws
catkin_make --pkg ucar_corner_supervisor
source devel/setup.bash
```

- [ ] **Step 6：无运动联调**

在尚未修改团队 move_base remap 时不得发送目标。先验证新包可启动、无目标时 `/cmd_vel`
为零、退出时为零。完成 remap 后验证：

```text
/move_base/cmd_vel_raw 只有 move_base 发布
/cmd_vel 只有 corner_supervisor 发布
```

未满足单发布者条件时禁止实车测试。
