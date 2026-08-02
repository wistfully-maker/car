# U-CAR 全任务分层启动实施计划

> **已废止（2026-08-02）：** 本计划依赖旧 `ucar_waypoint_nav + AMCL`，不得执行。
> 请执行 `docs/superpowers/plans/2026-08-02-ucar-fast-nav-integration.md`。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现一个不重复占用硬件、能在导航与 QR 之间安全交接底盘控制权、并为避障、巡线和其他比赛任务预留接口的全车启动入口。

**Architecture:** 用 `competition_full.launch` 组合公共硬件、导航栈和常驻业务节点；用独立速度仲裁节点将导航、QR、避障和巡线的速度 topic 汇聚到唯一 `/cmd_vel`；用 `start_competition.sh` 在 roslaunch 前检查同名节点、僵尸登记、设备占用、密钥和已有速度发布者。业务节点全程常驻，阶段完成时只停止动作并释放控制权。

**Tech Stack:** ROS 1 Noetic、Python 3、`rospy`、`geometry_msgs/Twist`、`std_msgs/String`、XML roslaunch、Bash、Python `unittest`。

---

## 文件结构

**新建：**

- `ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py`：模式名称、状态到模式的映射和模式消息构造。
- `ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py`：不依赖 ROS 的速度仲裁核心。
- `ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py`：ROS topic 适配、超时和零速度回落。
- `ucar_ws/src/task_orchestrator/launch/competition_full.launch`：全车分层组合 launch。
- `ucar_ws/src/task_orchestrator/scripts/start_competition.sh`：启动前检查和安全一键入口。
- `ucar_ws/src/task_orchestrator/test/test_motion_mode.py`：编排状态与底盘模式测试。
- `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py`：速度仲裁核心测试。
- `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`：launch、remap、启动开关和安全脚本静态回归。

**修改：**

- `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`：每次状态切换发出明确的底盘模式。
- `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`：发布 `/task/motion_mode`。
- `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`：仲裁超时和发布频率。
- `ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`：可选启动仲裁器，保留原来的编排器单包调试入口。
- `ucar_ws/src/task_orchestrator/CMakeLists.txt`：安装新增可执行脚本。
- `ucar_ws/src/task_orchestrator/package.xml`：增加 `geometry_msgs` 运行依赖。
- `ucar_ws/src/task_orchestrator/test/test_package_config.py`：覆盖新增节点、安装规则和 launch。
- `ucar_ws/src/task_orchestrator/README.md`：重写为从开机到停机的详细操作手册。
- `ucar_ws/src/task_orchestrator/test/manual_simulation.md`：加入模式 topic 和分层启动联调步骤。
- `HANDOFF.md`：记录全任务启动架构、当前边界和实车验证结果。

---

### 任务 1：定义底盘模式协议

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_motion_mode.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- Test: `ucar_ws/src/task_orchestrator/test/test_orchestrator.py`

- [ ] **Step 1: 先编写失败测试**

`test_motion_mode.py` 必须断言：

```python
from task_orchestrator.motion_mode import mode_for_state


def test_state_to_motion_mode_is_explicit():
    assert mode_for_state("NAVIGATING_TO_PICKUP") == "NAVIGATION"
    assert mode_for_state("WAITING_QR") == "QR_SEARCH"
    assert mode_for_state("NAVIGATING_TO_WORKSHOP") == "AVOIDANCE"
    assert mode_for_state("WAITING_LLM") == "IDLE"
    assert mode_for_state("ERROR") == "IDLE"
```

在 `test_orchestrator.py` 完整成功路径中增加：

```python
self.assertEqual(
    ["IDLE", "NAVIGATION", "QR_SEARCH", "IDLE", "AVOIDANCE", "IDLE"],
    [item["mode"] for item in h.actions("publish_motion_mode")],
)
```

- [ ] **Step 2: 运行定向测试并确认失败**

```powershell
python ucar_ws/src/task_orchestrator/test/test_motion_mode.py -v
python ucar_ws/src/task_orchestrator/test/test_orchestrator.py -v
```

Expected: `ModuleNotFoundError: task_orchestrator.motion_mode` 或缺少 `publish_motion_mode`。

- [ ] **Step 3: 实现最小模式映射**

`motion_mode.py` 内容：

```python
IDLE = "IDLE"
NAVIGATION = "NAVIGATION"
QR_SEARCH = "QR_SEARCH"
AVOIDANCE = "AVOIDANCE"
LINE_FOLLOW = "LINE_FOLLOW"

STATE_MODES = {
    "NAVIGATING_TO_PICKUP": NAVIGATION,
    "WAITING_QR": QR_SEARCH,
    "NAVIGATING_TO_WORKSHOP": AVOIDANCE,
}


def mode_for_state(state):
    return STATE_MODES.get(state, IDLE)
```

在 `TaskOrchestrator.__init__` 中以 `self.motion_mode = None` 初始化，使新任务进入
`CHECKING_DEPENDENCIES` 时也会明确发布一次 `IDLE`。然后在 `_transition()` 内仅当模式变化时追加：

```python
new_mode = mode_for_state(state)
if new_mode != self.motion_mode and self.task is not None:
    self.motion_mode = new_mode
    self._emit(
        "publish_motion_mode",
        {
            "protocol_version": 1,
            "task_id": self.task["task_id"],
            "mode": new_mode,
        },
    )
```

ROS 适配器增加 latched publisher：

```python
"publish_motion_mode": rospy.Publisher(
    "/task/motion_mode", String, queue_size=10, latch=True
),
```

- [ ] **Step 4: 验证正常、超时、取消和错误都回到 `IDLE`**

Run:

```powershell
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v
```

Expected: 原 47 项加新增模式测试全部 `OK`。

- [ ] **Step 5: 提交**

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py \
  ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py \
  ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py \
  ucar_ws/src/task_orchestrator/test/test_motion_mode.py \
  ucar_ws/src/task_orchestrator/test/test_orchestrator.py
git commit -m "feat: publish exclusive motion modes"
```

---

### 任务 2：实现 ROS 无关的速度仲裁核心

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py`

- [ ] **Step 1: 先写失败测试**

```python
from task_orchestrator.velocity_arbiter import VelocityArbiter

ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
NAV = (0.2, 0.0, 0.0, 0.0, 0.0, 0.1)
QR = (0.0, 0.0, 0.0, 0.0, 0.0, 0.2)


def test_only_active_source_can_reach_output():
    arb = VelocityArbiter(source_timeout=0.3)
    assert arb.set_mode("NAVIGATION", 1.0) == ZERO
    assert arb.accept("qr", QR, 1.1) is None
    assert arb.accept("navigation", NAV, 1.1) == NAV


def test_mode_change_and_timeout_force_zero_once():
    arb = VelocityArbiter(source_timeout=0.3)
    arb.set_mode("NAVIGATION", 1.0)
    arb.accept("navigation", NAV, 1.1)
    assert arb.set_mode("QR_SEARCH", 1.2) == ZERO
    assert arb.accept("qr", QR, 1.2) == QR
    assert arb.tick(1.51) == ZERO
    assert arb.tick(1.52) is None
```

- [ ] **Step 2: 运行并确认类不存在**

```powershell
python ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py -v
```

Expected: FAIL，缺少 `velocity_arbiter.py`。

- [ ] **Step 3: 实现最小仲裁核心**

```python
from task_orchestrator.motion_mode import (
    AVOIDANCE,
    IDLE,
    LINE_FOLLOW,
    NAVIGATION,
    QR_SEARCH,
)

ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
MODE_SOURCE = {
    NAVIGATION: "navigation",
    QR_SEARCH: "qr",
    AVOIDANCE: "avoidance",
    LINE_FOLLOW: "line",
}


class VelocityArbiter:
    def __init__(self, source_timeout):
        self.source_timeout = float(source_timeout)
        self.mode = IDLE
        self.last_command_time = None
        self.zero_sent = True

    def set_mode(self, mode, now):
        if mode not in (IDLE,) + tuple(MODE_SOURCE):
            raise ValueError("unknown motion mode: %s" % mode)
        self.mode = mode
        self.last_command_time = float(now)
        self.zero_sent = True
        return ZERO

    def accept(self, source, command, now):
        if MODE_SOURCE.get(self.mode) != source:
            return None
        self.last_command_time = float(now)
        self.zero_sent = command == ZERO
        return command

    def tick(self, now):
        if self.mode == IDLE or self.zero_sent:
            return None
        if float(now) - self.last_command_time < self.source_timeout:
            return None
        self.zero_sent = True
        return ZERO
```

- [ ] **Step 4: 补齐无效模式、非活动源、负超时和模式快速切换测试**

Run:

```powershell
python ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py -v
```

Expected: 至少 6 项仲裁核心测试全部 `OK`。

- [ ] **Step 5: 提交**

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py \
  ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py
git commit -m "feat: add fail-safe velocity arbiter core"
```

---

### 任务 3：实现速度仲裁 ROS 节点

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py`
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- Modify: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Modify: `ucar_ws/src/task_orchestrator/package.xml`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **Step 1: 先增加源码结构失败测试**

```python
def test_velocity_arbiter_declares_exclusive_topics(self):
    source = (PACKAGE / "scripts" / "velocity_arbiter_node.py").read_text(
        encoding="utf-8"
    )
    for topic in (
        '"/cmd_vel/navigation"',
        '"/cmd_vel/qr"',
        '"/cmd_vel/avoidance"',
        '"/cmd_vel/line"',
        '"/task/motion_mode"',
        '"/cmd_vel"',
    ):
        self.assertIn(topic, source)
    self.assertNotIn("shell=True", source)
```

- [ ] **Step 2: 运行并确认新节点缺失**

```powershell
python ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

Expected: FAIL，`velocity_arbiter_node.py` 不存在。

- [ ] **Step 3: 实现 ROS 适配节点**

节点必须：

```python
MODE_TOPICS = {
    "navigation": "/cmd_vel/navigation",
    "qr": "/cmd_vel/qr",
    "avoidance": "/cmd_vel/avoidance",
    "line": "/cmd_vel/line",
}

output = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
arbiter = VelocityArbiter(
    rospy.get_param("~velocity_arbiter/source_timeout", 0.3)
)
```

模式回调用 `json.loads(message.data)` 严格要求 `protocol_version == 1`、非空 `task_id`和合法
`mode`；收到合法模式时立即发布 `arbiter.set_mode(...)` 返回的零速度。四个速度回调将
`Twist` 转为六元组，仅当 `arbiter.accept(...)` 返回非 `None` 时发布。`rospy.Timer`
以 20 Hz 调用 `tick()`，超时后只发一次零速度。

- [ ] **Step 4: 配置安全默认值并安装脚本**

`orchestrator.yaml` 增加：

```yaml
velocity_arbiter:
  source_timeout: 0.3
  watchdog_hz: 20.0
```

`package.xml` 的 build/exec depend 增加 `geometry_msgs`；`CMakeLists.txt` 的
`find_package(catkin REQUIRED COMPONENTS ...)` 也增加 `geometry_msgs`，并将
`scripts/velocity_arbiter_node.py` 加入 `catkin_install_python`。后续的 Bash 启动脚本使用：

```cmake
install(PROGRAMS
  scripts/start_competition.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

- [ ] **Step 5: 运行包配置和全部单元测试**

```powershell
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v
```

Expected: 全部 `OK`。

- [ ] **Step 6: 提交**

```bash
git add ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py \
  ucar_ws/src/task_orchestrator/config/orchestrator.yaml \
  ucar_ws/src/task_orchestrator/CMakeLists.txt \
  ucar_ws/src/task_orchestrator/package.xml \
  ucar_ws/src/task_orchestrator/test/test_package_config.py
git commit -m "feat: arbitrate exclusive chassis velocity"
```

---

### 任务 4：实现分层全车 launch

**Files:**
- Create: `ucar_ws/src/task_orchestrator/launch/competition_full.launch`
- Modify: `ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`
- Create/Modify: `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`

- [ ] **Step 1: 先写 launch 结构失败测试**

测试解析 XML 并断言：

```python
root = ET.parse(LAUNCH).getroot()
args = {node.attrib["name"] for node in root.findall("arg")}
assert {
    "start_robot_base",
    "start_camera",
    "start_navigation_stack",
    "start_waypoint_manager",
    "start_speech",
    "start_qr",
    "start_llm",
    "start_orchestrator",
    "start_velocity_arbiter",
    "start_obstacle_adapter",
}.issubset(args)
```

另断言文本中存在：

```python
assert '<remap from="/cmd_vel" to="/cmd_vel/navigation"/>' in source
assert '<remap from="/cmd_vel" to="/cmd_vel/qr"/>' in source
assert 'start_navigation_stack' in source
assert 'start_obstacle_adapter" default="false"' in source
```

- [ ] **Step 2: 运行并确认 launch 缺失**

```powershell
python ucar_ws/src/task_orchestrator/test/test_competition_bringup.py -v
```

Expected: FAIL，`competition_full.launch` 不存在。

- [ ] **Step 3: 实现组合 launch**

launch 结构必须为：

```xml
<launch>
  <arg name="start_robot_base" default="true"/>
  <arg name="start_camera" default="true"/>
  <arg name="start_navigation_stack" default="true"/>
  <arg name="start_waypoint_manager" default="true"/>
  <arg name="start_speech" default="true"/>
  <arg name="start_qr" default="true"/>
  <arg name="start_llm" default="true"/>
  <arg name="start_orchestrator" default="true"/>
  <arg name="start_velocity_arbiter" default="true"/>
  <arg name="start_obstacle_adapter" default="false"/>
  <arg name="navigation_profile" default="navfn_teb_corner"/>
  <arg name="image_topic" default="/usb_cam/image_raw"/>

  <include if="$(arg start_robot_base)"
           file="$(find ucar_nav)/launch/robot_base_bringup.launch">
    <arg name="start_camera" value="$(arg start_camera)"/>
  </include>

  <group if="$(arg start_waypoint_manager)">
    <remap from="/cmd_vel" to="/cmd_vel/navigation"/>
    <include file="$(find ucar_waypoint_nav)/launch/waypoint_teb_navigation.launch">
      <arg name="start_navigation_stack" value="$(arg start_navigation_stack)"/>
      <arg name="global_planner" value="global_planner"/>
    </include>
  </group>

  <group if="$(eval arg('start_navigation_stack') and not arg('start_waypoint_manager'))">
    <remap from="/cmd_vel" to="/cmd_vel/navigation"/>
    <include file="$(find ucar_nav)/launch/navigation_stack.launch">
      <arg name="navigation_profile" value="$(arg navigation_profile)"/>
    </include>
  </group>

  <include if="$(arg start_speech)"
           file="$(find speech_command)/launch/speech_command.launch"/>

  <group if="$(arg start_qr)">
    <remap from="/cmd_vel" to="/cmd_vel/qr"/>
    <include file="$(find qr_item_search)/launch/qr_item_search.launch">
      <arg name="image_topic" value="$(arg image_topic)"/>
    </include>
  </group>

  <include if="$(arg start_llm)"
           file="$(find llm_spark)/launch/llm_spark.launch"/>
  <include if="$(arg start_orchestrator)"
           file="$(find task_orchestrator)/launch/task_orchestrator.launch">
    <arg name="enable_velocity_arbiter" value="$(arg start_velocity_arbiter)"/>
  </include>
</launch>
```

`start_obstacle_adapter` 只作为预留且默认 `false`；避障实际接口未确定前，不 include 不存在的节点。

- [ ] **Step 4: 在原编排器 launch 中增加仲裁器开关**

```xml
<arg name="enable_velocity_arbiter" default="false"/>
<node pkg="task_orchestrator"
      type="velocity_arbiter_node.py"
      name="velocity_arbiter"
      output="screen"
      if="$(arg enable_velocity_arbiter)">
  <rosparam command="load"
            file="$(find task_orchestrator)/config/orchestrator.yaml"/>
</node>
```

原 launch 默认不启动仲裁器，保证单独编排器模拟测试不抢占 `/cmd_vel`；只有总 launch
显式传 `true`。

- [ ] **Step 5: 运行 XML 和包配置测试**

```powershell
python ucar_ws/src/task_orchestrator/test/test_competition_bringup.py -v
python ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

Expected: 全部 `OK`。

- [ ] **Step 6: 提交**

```bash
git add ucar_ws/src/task_orchestrator/launch \
  ucar_ws/src/task_orchestrator/test/test_competition_bringup.py \
  ucar_ws/src/task_orchestrator/test/test_package_config.py
git commit -m "feat: compose layered competition bringup"
```

---

### 任务 5：实现不自动杀进程的安全启动脚本

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/start_competition.sh`
- Modify: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Modify: `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`

- [ ] **Step 1: 先增加安全规则失败测试**

```python
def test_start_script_checks_and_never_kills_unknown_nodes():
    source = START_SCRIPT.read_text(encoding="utf-8")
    for required in (
        "rosnode list",
        "rosnode ping",
        "rostopic info /cmd_vel",
        "SPARK_API_PASSWORD",
        "roslaunch task_orchestrator competition_full.launch",
    ):
        assert required in source
    for forbidden in ("rosnode kill", "kill -9", "pkill", "killall"):
        assert forbidden not in source
```

- [ ] **Step 2: 运行并确认脚本缺失**

```powershell
python ucar_ws/src/task_orchestrator/test/test_competition_bringup.py -v
```

Expected: FAIL，`start_competition.sh` 不存在。

- [ ] **Step 3: 实现参数感知的启动前检查**

脚本必须使用：

```bash
#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash

launch_args=("$@")
owns_arg() {
  local name="$1"
  local arg
  for arg in "${launch_args[@]}"; do
    [[ "$arg" == "${name}:=false" ]] && return 1
  done
  return 0
}

master_available=true
if ! rosnode list >/tmp/competition_rosnodes.$$ 2>/dev/null; then
  master_available=false
  : > /tmp/competition_rosnodes.$$
  echo "INFO: ROS Master 尚未运行，按干净开机处理，将由 roslaunch 启动"
fi

if [[ -z "${SPARK_API_PASSWORD:-}" ]]; then
  secret_file="${HOME}/.config/ucar/spark_api_password"
  [[ -r "$secret_file" ]] || {
    echo "ERROR: Spark 密钥未配置" >&2
    exit 3
  }
  export SPARK_API_PASSWORD="$(<"$secret_file")"
fi
```

只有 `master_available=true` 时才执行 ROS 节点和 topic 冲突检查。ROS Master 不可达代表
干净开机时，脚本继续，后续 `roslaunch` 自动启动 Master；不要额外启动一个后台 `roscore`。

根据 `start_robot_base`、`start_camera`、`start_navigation_stack`、`start_speech`、`start_qr`、
`start_llm`、`start_orchestrator` 选择需要检查的节点名。如果 `rosnode list` 已有节点，再执行
`rosnode ping -c 1 "$node"`：可达时报“已有活跃节点”，不可达时报“ROS Master 僵尸登记”，
两者都以非零码退出。

当脚本将自己启动硬件层时，对存在的 `/dev/ucar_controller`、`/dev/ydlidar`、
`/dev/video-camera0` 执行 `fuser`，设备已被占用则报告 PID 并退出。不存在的设备路径只记录
warning，因为不同 udev 规则的设备名可能不同。

检查 `rostopic info /cmd_vel` 的 `Publishers:` 段；如启动前已有直接 `/cmd_vel` 发布者，拒绝启动，
要求将外部模块改发专用输入 topic。

- [ ] **Step 4: 启动时不保留额外包装进程**

```bash
echo "Preflight OK; starting competition_full.launch"
exec roslaunch task_orchestrator competition_full.launch "${launch_args[@]}"
```

用 `trap 'rm -f /tmp/competition_rosnodes.$$' EXIT` 只删除本脚本自己创建的临时文件，不结束任何外部节点。

- [ ] **Step 5: 运行静态测试并在小车使用 `bash -n` 验证**

```powershell
python ucar_ws/src/task_orchestrator/test/test_competition_bringup.py -v
```

```bash
bash -n ~/ucar_ws/src/task_orchestrator/scripts/start_competition.sh
```

Expected: Python 测试 `OK`，`bash -n` 退出码 0。

- [ ] **Step 6: 提交**

```bash
git add ucar_ws/src/task_orchestrator/scripts/start_competition.sh \
  ucar_ws/src/task_orchestrator/CMakeLists.txt \
  ucar_ws/src/task_orchestrator/test/test_competition_bringup.py
git commit -m "feat: add safe competition startup preflight"
```

---

### 任务 6：重写极其详细的运行手册

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/README.md`
- Modify: `ucar_ws/src/task_orchestrator/test/manual_simulation.md`
- Modify: `HANDOFF.md`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **Step 1: 先增加 README 覆盖测试**

```python
def test_readme_explains_full_startup_and_current_boundary(self):
    text = (PACKAGE / "README.md").read_text(encoding="utf-8")
    for phrase in (
        "start_competition.sh",
        "competition_full.launch",
        "task_orchestrator.launch",
        "小飞小飞",
        "start_robot_base:=false",
        "/task/motion_mode",
        "/cmd_vel/navigation",
        "/cmd_vel/qr",
        "NAVIGATION -> QR_SEARCH -> AVOIDANCE",
        "AMCL",
        "僵尸登记",
        "Ctrl+C",
        "避障模块的实际接口尚未确定",
    ):
        self.assertIn(phrase, text)
```

- [ ] **Step 2: 运行并确认旧 README 不满足要求**

```powershell
python ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

Expected: FAIL，缺少全车启动和速度仲裁章节。

- [ ] **Step 3: 按操作顺序重写 README**

README 固定包含以下顶级章节：

```markdown
# U-CAR 全任务编排与分层启动手册
## 1. 先看结论：三个启动入口有什么区别
## 2. 编排器做什么，不做什么
## 3. 节点、硬件资源和唯一所有者表
## 4. 从开机到发车的安全一键流程
## 5. 导航或其他基础层已启动时的命令
## 6. 完全分步启动与逐节点验证
## 7. “小飞小飞”之后每一步发生什么
## 8. 导航、QR、避障、巡线如何停止动作和交接控制权
## 9. 当前真实能运行到哪里
## 10. 参数文件、临时覆盖与重启边界
## 11. 正常停止、异常残留与僵尸节点
## 12. 串口、相机、声卡、密钥和 /cmd_vel 冲突排查
## 13. 逐 topic 调试命令和 JSON 示例
## 14. 避障、巡线和其他任务的后续接入规范
## 15. 测试、部署和回滚
```

每条命令都必须写出执行终端、前置条件、期望节点/topic、成功标志和停止方法。

- [ ] **Step 4: 更新手动模拟和交接文档**

`manual_simulation.md` 增加对 `/task/motion_mode` 的监视；`HANDOFF.md` 明确：

```text
ucar_waypoint_nav 只到二维码区
QR 后续航点由避障模块负责
避障实际 topic/service/action 未确定
节点常驻，阶段结束时只停动作、回报、发零并释放控制权
```

- [ ] **Step 5: 运行文档契约与全部回归**

```powershell
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v
```

Expected: 全部 `OK`。

- [ ] **Step 6: 提交**

```bash
git add ucar_ws/src/task_orchestrator/README.md \
  ucar_ws/src/task_orchestrator/test/manual_simulation.md \
  ucar_ws/src/task_orchestrator/test/test_package_config.py HANDOFF.md
git commit -m "docs: explain full competition bringup"
```

---

### 任务 7：本地全回归和启动边界审查

**Files:**
- Verify only; fix only files directly implicated by failures.

- [ ] **Step 1: 运行编排器全部测试**

```powershell
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v
```

Expected: 旧 47 项加新增测试全部 `OK`。

- [ ] **Step 2: 运行相关模块回归**

```powershell
python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/qr_item_search/test -p "test_*.py" -v
python -m unittest discover -s patches/speech_command -p "test_*.py" -v
```

Expected: LLM 11 项、QR 包测试和语音 9 项全部 `OK`。

- [ ] **Step 3: 检查资源所有权和敏感信息**

```powershell
rg -n "(/cmd_vel|usb_cam|base_driver|ydlidar|speech_command_node|SPARK_API_PASSWORD)" `
  ucar_ws/src/task_orchestrator
rg -n "password|api[_-]?key|Authorization" `
  ucar_ws/src/task_orchestrator ucar_ws/src/llm_spark
git diff --check
```

Expected: 每个公共资源只有一个最终所有者；无明文密钥；`git diff --check` 无输出。

- [ ] **Step 4: 审查尚未接入的避障边界**

确认总 launch 默认 `start_obstacle_adapter:=false`，没有伪造避障 topic，README 明确流程在
`NAVIGATING_TO_WORKSHOP` 等待。

- [ ] **Step 5: 如发现失败，返回对应任务按 TDD 修正**

```bash
git status --short
```

验证任务本身不创建空提交。任何修正都在产生该文件的原任务中补充失败测试、最小修正和
定向提交，然后从 Step 1 重新运行全回归。

---

### 任务 8：部署到小车并分层验证

**Files:**
- Deploy: `ucar_ws/src/task_orchestrator/`
- Read only before launch: `ucar_ws/src/ucar_nav/`, `ucar_ws/src/ucar_waypoint_nav/`, `ucar_ws/src/speech_command/`
- Modify documentation only after observing results: `HANDOFF.md`, `ucar_ws/src/task_orchestrator/README.md`

- [ ] **Step 1: 只读盘点小车当前节点和资源**

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosnode list
rostopic info /cmd_vel
ps -eo pid,ppid,args | grep -E '[r]oslaunch|[m]ove_base|[u]sb_cam|[s]peech_command'
```

不停止任何未确认所有者的节点。

- [ ] **Step 2: 备份小车旧编排包并部署**

```bash
cp -a ~/ucar_ws/src/task_orchestrator \
  ~/ucar_ws/src/task_orchestrator.pre-full-bringup-20260801
```

Windows:

```powershell
scp -r .\ucar_ws\src\task_orchestrator `
  ucar@172.20.10.4:/home/ucar/ucar_ws/src/
```

- [ ] **Step 3: 构建并执行车端测试**

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make --pkg task_orchestrator
source devel/setup.bash
python3 -m unittest discover \
  -s src/task_orchestrator/test -p 'test_*.py' -v
bash -n src/task_orchestrator/scripts/start_competition.sh
```

Expected: 构建成功，所有测试 `OK`，Bash 语法退出码 0。

- [ ] **Step 4: 不控制底盘地验证常驻业务层**

```bash
roslaunch task_orchestrator competition_full.launch \
  start_robot_base:=false \
  start_camera:=false \
  start_navigation_stack:=false \
  start_waypoint_manager:=false
```

验证 `/task_orchestrator`、`/voice_task_adapter`、`/tts_bridge`、`/velocity_arbiter`、QR、LLM 和语音节点各只
有一个实例。不发布 `/task/dependencies_ready`，不让小车运动。

- [ ] **Step 5: 模拟速度源验证仲裁**

先发 `NAVIGATION`，同时向 navigation 和 QR 输入发不同速度，确认 `/cmd_vel` 只输出 navigation；
切换 `QR_SEARCH` 时必须立即先输出零，随后只输出 QR；停止输入 0.3 秒后必须回落到零。

- [ ] **Step 6: 分层启动硬件和导航，人工确认 AMCL**

先确认现场可急停，然后使用总 launch 启动公共层。只在激光墙线与地图墙线对正、
`map -> odom` 静止时不持续跳动后才发布任务。AMCL 偏移不属本次启动架构已修复项。

- [ ] **Step 7: 执行真实链路到 TTS**

验证：

```text
小飞小飞 -> /question -> 到二维码区 -> QR -> LLM -> TTS
```

导航到达后必须释放 `NAVIGATION`，QR 必须获得 `QR_SEARCH`。TTS 完成后编排器发布
`/task/delivery_navigation_goal`，但因避障真实接口未知，不让小车自动进入未验证的后续路段。

- [ ] **Step 8: 正常停止并核对无残留**

在启动总 launch 的终端按 `Ctrl+C`，然后确认本次启动的节点退出。不单独杀底盘、雷达或
其他团队管理的进程。

- [ ] **Step 9: 记录实车结果并提交**

```bash
git add HANDOFF.md ucar_ws/src/task_orchestrator/README.md
git commit -m "docs: record layered bringup validation"
```

只记录亲自观察的结果；未验证的避障、巡线和最终 `COMPLETE` 保持为未完成。
