# U-CAR DWA 导航安全基线实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将小车现有 `ucar_nav` 整理为参数来源唯一、启动职责清晰、能够通过直角弯和当前完整路线的安全 DWA 导航包，并提供完整 README 与交接文件。

**Architecture:** 将硬件所有者节点与导航算法节点拆成两个 launch，兼容入口按参数组合两层；只加载 DWA，不加载未安装的 TEB。使用静态配置测试保证 footprint、costmap 命名空间、DWA 安全参数和 launch 结构正确，再按“无运动检查→底盘低速→单墙→直角弯→完整路线”逐级实车验收。

**Tech Stack:** ROS 1 Noetic、`move_base`、`dwa_local_planner`、`global_planner`、AMCL、`costmap_2d`、Python 3 `unittest`、PyYAML、XML ElementTree、Bash、rosbag。

---

## 文件结构

实施后主要文件及职责：

```text
ucar_ws/src/ucar_nav/
  CMakeLists.txt
  package.xml
  README.md
  HANDOFF.md
  launch/
    robot_base_bringup.launch       # 底盘、雷达、雷达 TF、可选相机
    navigation_stack.launch         # 地图、AMCL、move_base、唯一 DWA
    ucar_navigation.launch          # 兼容的一键组合入口
  config/
    amcl/
      amcl_omni.yaml                # AMCL 唯一参数源
    costmap/
      common.yaml                   # footprint、雷达障碍源
      global.yaml                   # 全局 costmap
      local.yaml                    # 局部 rolling costmap
    local_planners/
      dwa_safe.yaml                 # DWA 安全基线
    global_planner.yaml
    move_base.yaml
  scripts/
    capture_nav_diagnostics.sh      # 保存参数、节点、topic、TF、日志信息
  test/
    test_navigation_config.py       # Windows 可运行的静态配置回归
  maps/
    ...                             # 原始地图保留，默认仍使用 map.yaml
```

原目录 `launch/config/move_base` 和 `launch/config/amcl` 在新配置验证通过后删除，避免
两套参数同时存在。原始多份地图在本阶段保留，不擅自删除。

---

### 任务 1：导入小车原始导航包基线

**文件：**

- 新增：`ucar_ws/src/ucar_nav/**`

- [ ] **步骤 1：确认本地副本与小车源目录一致**

在 Windows PowerShell 运行：

```powershell
cd D:\program_sec\智能车\.worktrees\navigation-safety

Get-ChildItem -Recurse -File .\ucar_ws\src\ucar_nav |
  ForEach-Object {
    [PSCustomObject]@{
      RelativePath = $_.FullName.Substring(
        (Resolve-Path .\ucar_ws\src\ucar_nav).Path.Length + 1
      )
      Length = $_.Length
    }
  } |
  Sort-Object RelativePath
```

预期：包含 `package.xml`、`CMakeLists.txt`、`launch/ucar_navigation.launch`、
AMCL/move_base 配置及全部地图文件。

- [ ] **步骤 2：检查敏感信息**

```powershell
rg -n -i "password|passwd|secret|api[_-]?key|token|ssid|wifi" `
  .\ucar_ws\src\ucar_nav
```

预期：无匹配。若发现凭据，停止提交并将凭据移出仓库。

- [ ] **步骤 3：记录原始文件校验值**

```powershell
Get-FileHash `
  .\ucar_ws\src\ucar_nav\launch\ucar_navigation.launch, `
  .\ucar_ws\src\ucar_nav\launch\config\move_base\*.yaml, `
  .\ucar_ws\src\ucar_nav\maps\map.yaml, `
  .\ucar_ws\src\ucar_nav\maps\map.pgm `
  -Algorithm SHA256
```

将输出复制到提交说明，不创建包含机器绝对路径的仓库文件。

- [ ] **步骤 4：提交未经修改的原始包**

```powershell
git add ucar_ws/src/ucar_nav
git diff --cached --check
git commit -m "chore: import current ucar navigation package"
```

预期：提交只包含从小车复制的原始 `ucar_nav`。

---

### 任务 2：建立静态配置回归测试

**文件：**

- 新增：`ucar_ws/src/ucar_nav/test/test_navigation_config.py`

- [ ] **步骤 1：编写先失败的配置测试**

创建完整测试：

```python
#!/usr/bin/env python3
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch"
CONFIG = PACKAGE / "config"


def load_yaml(relative_path):
    with (PACKAGE / relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def launch_text(name):
    return (LAUNCH / name).read_text(encoding="utf-8")


class NavigationConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        required = (
            "launch/robot_base_bringup.launch",
            "launch/navigation_stack.launch",
            "launch/ucar_navigation.launch",
            "config/amcl/amcl_omni.yaml",
            "config/costmap/common.yaml",
            "config/costmap/global.yaml",
            "config/costmap/local.yaml",
            "config/local_planners/dwa_safe.yaml",
            "config/global_planner.yaml",
            "config/move_base.yaml",
            "scripts/capture_nav_diagnostics.sh",
            "README.md",
            "HANDOFF.md",
        )
        for relative_path in required:
            self.assertTrue(
                (PACKAGE / relative_path).is_file(),
                relative_path,
            )

    def test_launch_files_are_valid_xml(self):
        for path in LAUNCH.glob("*.launch"):
            ET.parse(path)

    def test_navigation_stack_loads_only_dwa(self):
        text = launch_text("navigation_stack.launch")
        self.assertIn(
            'value="dwa_local_planner/DWAPlannerROS"',
            text,
        )
        self.assertIn("dwa_safe.yaml", text)
        self.assertNotIn("teb_local_planner", text.lower())
        self.assertNotIn("teb_safe.yaml", text.lower())

    def test_hardware_and_navigation_launches_are_separated(self):
        hardware = launch_text("robot_base_bringup.launch")
        navigation = launch_text("navigation_stack.launch")
        self.assertIn("base_driver.launch", hardware)
        self.assertIn("ydlidar.launch", hardware)
        self.assertNotIn('pkg="move_base"', hardware)
        self.assertNotIn("base_driver.launch", navigation)
        self.assertNotIn("ydlidar.launch", navigation)
        self.assertNotIn("usb_cam", navigation)

    def test_common_costmap_uses_safe_polygon_and_laser(self):
        common = load_yaml("config/costmap/common.yaml")
        footprint = common["footprint"]
        self.assertEqual(4, len(footprint))
        self.assertEqual(
            [[0.191, -0.148], [0.191, 0.148],
             [-0.191, 0.148], [-0.191, -0.148]],
            footprint,
        )
        source = common["obstacle_layer"]["laser_scan_sensor"]
        self.assertEqual("laser_frame", source["sensor_frame"])
        self.assertEqual("scan", source["topic"])
        self.assertTrue(source["marking"])
        self.assertTrue(source["clearing"])

    def test_local_costmap_has_real_safety_gradient(self):
        local = load_yaml("config/costmap/local.yaml")["local_costmap"]
        self.assertEqual("odom", local["global_frame"])
        self.assertLessEqual(local["transform_tolerance"], 0.5)
        inflation = local["inflation_layer"]
        corner_radius = math.hypot(0.191, 0.148)
        self.assertGreater(inflation["inflation_radius"], corner_radius)
        self.assertGreaterEqual(inflation["inflation_radius"], 0.35)

    def test_dwa_safe_profile_is_holonomic_and_conservative(self):
        dwa = load_yaml(
            "config/local_planners/dwa_safe.yaml"
        )["DWAPlannerROS"]
        self.assertGreater(dwa["vy_samples"], 0)
        self.assertGreater(dwa["max_vel_y"], 0)
        self.assertLess(dwa["min_vel_y"], 0)
        self.assertLessEqual(dwa["max_vel_x"], 0.25)
        self.assertLessEqual(dwa["max_vel_y"], 0.15)
        self.assertLessEqual(dwa["max_vel_theta"], 0.5)
        self.assertGreaterEqual(dwa["occdist_scale"], 0.15)
        self.assertGreaterEqual(dwa["stop_time_buffer"], 0.4)

    def test_move_base_disables_uncommanded_recovery_rotation(self):
        move_base = load_yaml("config/move_base.yaml")
        self.assertFalse(move_base["recovery_behavior_enabled"])
        self.assertFalse(move_base["clearing_rotation_allowed"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **步骤 2：运行测试并确认因新文件尚不存在而失败**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v
```

预期：`test_required_files_exist` 以及依赖新配置的测试失败。失败原因必须是计划中的新
结构尚未创建，而不是 Python 导入错误。

- [ ] **步骤 3：提交测试**

```powershell
git add ucar_ws/src/ucar_nav/test/test_navigation_config.py
git commit -m "test: define DWA navigation safety contract"
```

---

### 任务 3：拆分硬件层与导航层 launch

**文件：**

- 新增：`ucar_ws/src/ucar_nav/launch/robot_base_bringup.launch`
- 新增：`ucar_ws/src/ucar_nav/launch/navigation_stack.launch`
- 修改：`ucar_ws/src/ucar_nav/launch/ucar_navigation.launch`

- [ ] **步骤 1：创建硬件启动层**

`robot_base_bringup.launch` 内容：

```xml
<launch>
  <arg name="start_base" default="true"/>
  <arg name="start_lidar" default="true"/>
  <arg name="start_camera" default="false"/>

  <include if="$(arg start_base)"
           file="$(find ucar_controller)/launch/base_driver.launch"/>

  <include if="$(arg start_lidar)"
           file="$(find ydlidar)/launch/ydlidar.launch"/>

  <include if="$(arg start_camera)"
           file="$(find usb_cam)/launch/usb_cam-test.launch"/>
</launch>
```

雷达静态 TF 由 `ydlidar.launch` 提供；部署后必须通过 `roslaunch --nodes` 和
`rosrun tf tf_echo base_link laser_frame` 验证，不额外启动第二个发布者。

- [ ] **步骤 2：创建导航算法层**

`navigation_stack.launch` 内容：

```xml
<launch>
  <arg name="map_file"
       default="$(find ucar_nav)/maps/map.yaml"/>

  <node name="map_server"
        pkg="map_server"
        type="map_server"
        args="$(arg map_file)"
        output="screen">
    <param name="frame_id" value="map"/>
  </node>

  <node name="amcl"
        pkg="amcl"
        type="amcl"
        output="screen">
    <rosparam command="load"
              file="$(find ucar_nav)/config/amcl/amcl_omni.yaml"/>
  </node>

  <node name="move_base"
        pkg="move_base"
        type="move_base"
        output="screen"
        respawn="false">
    <param name="base_global_planner"
           value="global_planner/GlobalPlanner"/>
    <param name="base_local_planner"
           value="dwa_local_planner/DWAPlannerROS"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/costmap/common.yaml"
              ns="global_costmap"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/costmap/common.yaml"
              ns="local_costmap"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/costmap/global.yaml"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/costmap/local.yaml"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/global_planner.yaml"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/local_planners/dwa_safe.yaml"/>
    <rosparam command="load"
              file="$(find ucar_nav)/config/move_base.yaml"/>
  </node>
</launch>
```

- [ ] **步骤 3：重写兼容的一键入口**

`ucar_navigation.launch` 内容：

```xml
<launch>
  <arg name="start_robot_base" default="true"/>
  <arg name="start_camera" default="false"/>
  <arg name="map_file"
       default="$(find ucar_nav)/maps/map.yaml"/>

  <include if="$(arg start_robot_base)"
           file="$(find ucar_nav)/launch/robot_base_bringup.launch">
    <arg name="start_camera" value="$(arg start_camera)"/>
  </include>

  <include file="$(find ucar_nav)/launch/navigation_stack.launch">
    <arg name="map_file" value="$(arg map_file)"/>
  </include>
</launch>
```

- [ ] **步骤 4：运行 launch 静态测试**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v
```

预期：launch 相关测试通过；尚未创建的配置、诊断脚本和文档检查继续按预期失败。

- [ ] **步骤 5：提交 launch 分层**

```powershell
git add ucar_ws/src/ucar_nav/launch
git commit -m "refactor: separate navigation hardware and stack launch"
```

---

### 任务 4：建立唯一 costmap 与 DWA 安全配置

**文件：**

- 新增：`ucar_ws/src/ucar_nav/config/costmap/common.yaml`
- 新增：`ucar_ws/src/ucar_nav/config/costmap/global.yaml`
- 新增：`ucar_ws/src/ucar_nav/config/costmap/local.yaml`
- 新增：`ucar_ws/src/ucar_nav/config/local_planners/dwa_safe.yaml`
- 新增：`ucar_ws/src/ucar_nav/config/global_planner.yaml`
- 新增：`ucar_ws/src/ucar_nav/config/move_base.yaml`

- [ ] **步骤 1：创建通用车体与雷达配置**

`config/costmap/common.yaml`：

```yaml
footprint:
  - [0.191, -0.148]
  - [0.191, 0.148]
  - [-0.191, 0.148]
  - [-0.191, -0.148]

obstacle_layer:
  enabled: true
  combination_method: 1
  track_unknown_space: true
  obstacle_range: 2.0
  raytrace_range: 3.0
  observation_sources: laser_scan_sensor
  laser_scan_sensor:
    sensor_frame: laser_frame
    data_type: LaserScan
    topic: scan
    marking: true
    clearing: true
    inf_is_valid: true

inflation_layer:
  enabled: true
  inflation_radius: 0.35
  cost_scaling_factor: 2.5

static_layer:
  enabled: true
```

扩大的 footprint 是首轮安全值，不声称等于实车尺寸。任务 8 实测后再决定是否缩减。

- [ ] **步骤 2：创建全局 costmap**

`config/costmap/global.yaml`：

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 2.0
  publish_frequency: 1.0
  static_map: true
  rolling_window: false
  transform_tolerance: 0.5
  track_unknown_space: true
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

- [ ] **步骤 3：创建局部 costmap**

`config/costmap/local.yaml`：

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 8.0
  publish_frequency: 4.0
  static_map: false
  rolling_window: true
  width: 3.0
  height: 3.0
  resolution: 0.05
  transform_tolerance: 0.5
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
  inflation_layer:
    inflation_radius: 0.35
    cost_scaling_factor: 2.5
```

- [ ] **步骤 4：创建保守的全向 DWA 配置**

`config/local_planners/dwa_safe.yaml`：

```yaml
DWAPlannerROS:
  acc_lim_x: 0.30
  acc_lim_y: 0.30
  acc_lim_theta: 0.80

  max_vel_trans: 0.25
  min_vel_trans: 0.05
  max_vel_x: 0.25
  min_vel_x: -0.10
  max_vel_y: 0.15
  min_vel_y: -0.15
  max_vel_theta: 0.50
  min_vel_theta: 0.10

  trans_stopped_vel: 0.05
  theta_stopped_vel: 0.10

  yaw_goal_tolerance: 0.15
  xy_goal_tolerance: 0.15
  latch_xy_goal_tolerance: true

  sim_time: 2.0
  sim_granularity: 0.025
  angular_sim_granularity: 0.025
  vx_samples: 12
  vy_samples: 12
  vth_samples: 24
  controller_frequency: 10.0

  path_distance_bias: 32.0
  goal_distance_bias: 20.0
  occdist_scale: 0.15
  forward_point_distance: 0.25
  stop_time_buffer: 0.40
  scaling_speed: 0.15
  max_scaling_factor: 0.30

  oscillation_reset_dist: 0.10
  prune_plan: true
  publish_traj_pc: false
  publish_cost_grid_pc: false
  global_frame_id: odom
```

- [ ] **步骤 5：创建全局规划器与 move_base 配置**

`config/global_planner.yaml`：

```yaml
GlobalPlanner:
  allow_unknown: true
  default_tolerance: 0.20
  visualize_potential: false
  use_dijkstra: false
  use_quadratic: false
  use_grid_path: true
  old_navfn_behavior: false
  lethal_cost: 253
  neutral_cost: 66
  cost_factor: 0.55
  publish_potential: true
  orientation_mode: 0
  orientation_window_size: 1
```

`config/move_base.yaml`：

```yaml
shutdown_costmaps: false
controller_frequency: 10.0
controller_patience: 4.0
planner_frequency: 2.0
planner_patience: 5.0
oscillation_timeout: 8.0
oscillation_distance: 0.30
recovery_behavior_enabled: false
clearing_rotation_allowed: false
```

- [ ] **步骤 6：运行配置测试**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v
```

预期：除尚未创建的 AMCL、诊断脚本、README 和 HANDOFF 文件检查外，其余测试通过。

- [ ] **步骤 7：提交安全配置**

```powershell
git add ucar_ws/src/ucar_nav/config
git commit -m "feat: add conservative holonomic DWA profile"
```

---

### 任务 5：固化 AMCL 参数并删除旧重复配置

**文件：**

- 新增：`ucar_ws/src/ucar_nav/config/amcl/amcl_omni.yaml`
- 删除：`ucar_ws/src/ucar_nav/launch/config/amcl/amcl_omni.launch`
- 删除：`ucar_ws/src/ucar_nav/launch/config/amcl/amcl_diff.launch`
- 删除：`ucar_ws/src/ucar_nav/launch/config/move_base/*.yaml`
- 删除：`ucar_ws/src/ucar_nav/launch/config/rviz/tebrviz.rviz`

- [ ] **步骤 1：将当前 AMCL 全向参数转换为唯一 YAML**

`config/amcl/amcl_omni.yaml`：

```yaml
odom_model_type: omni
odom_alpha1: 0.2
odom_alpha2: 0.2
odom_alpha3: 0.8
odom_alpha4: 0.2
odom_alpha5: 0.1

min_particles: 1000
max_particles: 10000
kld_err: 0.05
kld_z: 0.99

laser_max_beams: 50
laser_model_type: beam
laser_z_hit: 0.5
laser_z_short: 0.05
laser_z_max: 0.05
laser_z_rand: 0.5
laser_sigma_hit: 0.2
laser_lambda_short: 0.1
laser_likelihood_max_dist: 2.0

update_min_d: 0.2
update_min_a: 0.5
resample_interval: 1
transform_tolerance: 0.2
recovery_alpha_slow: 0.0
recovery_alpha_fast: 0.0

odom_frame_id: odom
base_frame_id: base_link
global_frame_id: map
use_map_topic: false
first_map_only: true
gui_publish_rate: 10.0
```

此任务只消除重复和矛盾参数，不调整原有 AMCL 噪声模型。定位参数优化必须基于后续
实测数据单独进行。

- [ ] **步骤 2：删除旧 AMCL 和 move_base 参数目录**

使用 `apply_patch` 删除：

```text
launch/config/amcl/amcl_omni.launch
launch/config/amcl/amcl_diff.launch
launch/config/move_base/costmap_common_params.yaml
launch/config/move_base/global_costmap_params.yaml
launch/config/move_base/global_planner_params.yaml
launch/config/move_base/local_costmap_params.yaml
launch/config/move_base/move_base_params.yaml
launch/config/move_base/my_navigation.yaml
launch/config/move_base/dwa_local_planner_params.yaml
launch/config/move_base/teb_local_planner_params.yaml
launch/config/rviz/tebrviz.rviz
```

- [ ] **步骤 3：运行完整静态测试**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v

rg -n "teb_local_planner|TebLocalPlannerROS" `
  ucar_ws/src/ucar_nav/launch `
  ucar_ws/src/ucar_nav/config
```

预期：测试只因文档和诊断脚本尚未创建而失败；`rg` 无匹配。

- [ ] **步骤 4：提交唯一参数源**

```powershell
git add ucar_ws/src/ucar_nav/config/amcl `
  ucar_ws/src/ucar_nav/launch/config
git commit -m "refactor: consolidate navigation parameter sources"
```

---

### 任务 6：增加诊断快照脚本

**文件：**

- 新增：`ucar_ws/src/ucar_nav/scripts/capture_nav_diagnostics.sh`
- 修改：`ucar_ws/src/ucar_nav/CMakeLists.txt`

- [ ] **步骤 1：创建只读诊断脚本**

```bash
#!/usr/bin/env bash
set -euo pipefail

output_root="${1:-$HOME/ucar_nav_diagnostics}"
stamp="$(date +%Y%m%d-%H%M%S)"
output_dir="${output_root}/${stamp}"
mkdir -p "${output_dir}"

rosnode list >"${output_dir}/nodes.txt"
rostopic list >"${output_dir}/topics.txt"
rosparam get /move_base >"${output_dir}/move_base_params.yaml"
rostopic info /scan >"${output_dir}/scan_info.txt" 2>&1 || true
rostopic info /odom >"${output_dir}/odom_info.txt" 2>&1 || true
rosnode info /move_base >"${output_dir}/move_base_info.txt" 2>&1 || true
rosrun tf tf_echo base_link laser_frame \
  >"${output_dir}/base_link_to_laser_frame.txt" 2>&1 &
tf_pid=$!
sleep 2
kill "${tf_pid}" 2>/dev/null || true
wait "${tf_pid}" 2>/dev/null || true

{
  echo "git_commit=$(git -C "$HOME/ucar_ws/src/ucar_nav" rev-parse HEAD 2>/dev/null || echo untracked)"
  echo "map_sha256=$(sha256sum "$HOME/ucar_ws/src/ucar_nav/maps/map.yaml" | awk '{print $1}')"
  echo "image_sha256=$(sha256sum "$HOME/ucar_ws/src/ucar_nav/maps/map.pgm" | awk '{print $1}')"
} >"${output_dir}/manifest.txt"

printf '%s\n' "${output_dir}"
```

脚本不发布速度、不发送目标、不修改参数。

- [ ] **步骤 2：在 CMake 中安装脚本和资源目录**

用以下内容替换原 `CMakeLists.txt` 的安装部分：

```cmake
install(PROGRAMS
  scripts/capture_nav_diagnostics.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

- [ ] **步骤 3：运行静态测试并检查脚本**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v

rg -n "cmd_vel|move_base_simple/goal|rostopic pub|rosparam set" `
  ucar_ws/src/ucar_nav/scripts/capture_nav_diagnostics.sh
```

预期：`rg` 无匹配，证明诊断脚本不会控制车辆或改参数。

- [ ] **步骤 4：提交诊断能力**

```powershell
git add ucar_ws/src/ucar_nav/scripts `
  ucar_ws/src/ucar_nav/CMakeLists.txt
git commit -m "feat: capture reproducible navigation diagnostics"
```

---

### 任务 7：补齐依赖、README 初稿与交接模板

**文件：**

- 修改：`ucar_ws/src/ucar_nav/package.xml`
- 新增：`ucar_ws/src/ucar_nav/README.md`
- 新增：`ucar_ws/src/ucar_nav/HANDOFF.md`

- [ ] **步骤 1：明确 ROS 运行依赖**

在 `package.xml` 中保留 `catkin`，并将运行依赖整理为：

```xml
<exec_depend>amcl</exec_depend>
<exec_depend>dwa_local_planner</exec_depend>
<exec_depend>global_planner</exec_depend>
<exec_depend>map_server</exec_depend>
<exec_depend>move_base</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>tf</exec_depend>
<exec_depend>ucar_controller</exec_depend>
<exec_depend>usb_cam</exec_depend>
<exec_depend>ydlidar</exec_depend>
<test_depend>python3-yaml</test_depend>
```

- [ ] **步骤 2：编写 README 初稿**

README 至少写出以下可执行命令：

```bash
# 车上编译
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make --pkg ucar_nav
source devel/setup.bash

# 启动前检查
rosnode list
rosnode ping -c 1 /base_driver
rosnode ping -c 1 /ydlidar_node
rosnode ping -c 1 /amcl
rosnode ping -c 1 /move_base

# 全部节点均未运行时的一键启动
roslaunch ucar_nav ucar_navigation.launch \
  start_robot_base:=true \
  start_camera:=false

# 底盘和雷达已由其他 launch 启动时
roslaunch ucar_nav ucar_navigation.launch \
  start_robot_base:=false

# 分层启动
roslaunch ucar_nav robot_base_bringup.launch start_camera:=false
roslaunch ucar_nav navigation_stack.launch

# 保存诊断
rosrun ucar_nav capture_nav_diagnostics.sh

# 停止
# 回到对应 roslaunch 终端按 Ctrl+C
rosnode list
```

README 同时解释每个 DWA、costmap、footprint、TF 参数的位置和安全调参顺序。

- [ ] **步骤 3：创建 HANDOFF 模板**

HANDOFF 必须明确写入：

```text
当前范围：只完成 DWA，不包含 task_orchestrator 导航适配器。
后续输入：/task/pickup_navigation_goal、/task/delivery_navigation_goal。
后续输出：/task/pickup_arrived、/task/delivery_arrived。
正式适配器应使用 move_base_msgs/MoveBaseAction。
```

并为每轮实测保留表格字段：

```markdown
| 日期 | Commit | 地图 | 起点/目标 | 最大速度 | 最小墙距 | 耗时 | 结果 |
|---|---|---|---|---:|---:|---:|---|
```

- [ ] **步骤 4：运行全部本地测试**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v

git diff --check
```

预期：全部测试通过，`git diff --check` 无输出。

- [ ] **步骤 5：提交文档和依赖**

```powershell
git add ucar_ws/src/ucar_nav/package.xml `
  ucar_ws/src/ucar_nav/README.md `
  ucar_ws/src/ucar_nav/HANDOFF.md
git commit -m "docs: add DWA navigation operations guide"
```

---

### 任务 8：部署前备份、车端构建与无运动验证

**文件：**

- 部署：`ucar_ws/src/ucar_nav/**`
- 备份：小车 `/home/ucar/ucar_nav_backups/ucar_nav-<时间>`

- [ ] **步骤 1：确认车上导航节点全部停止**

只读检查：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
rosnode list
rosnode ping -c 1 /base_driver
rosnode ping -c 1 /ydlidar_node
rosnode ping -c 1 /amcl
rosnode ping -c 1 /move_base
```

若任一节点可通信，先找到拥有它的 launch 终端，由该终端 `Ctrl+C`。不要批量停止其他
团队节点。

- [ ] **步骤 2：备份原导航包**

```bash
stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p "$HOME/ucar_nav_backups"
cp -a ~/ucar_ws/src/ucar_nav \
  "$HOME/ucar_nav_backups/ucar_nav-${stamp}"
printf '%s\n' "$HOME/ucar_nav_backups/ucar_nav-${stamp}"
```

记录实际备份路径到 HANDOFF。

- [ ] **步骤 3：从 Windows 部署**

```powershell
tar --exclude="__pycache__" --exclude="*.pyc" `
  -czf "$env:TEMP\ucar_nav-deploy.tar.gz" `
  -C D:\program_sec\智能车\.worktrees\navigation-safety\ucar_ws\src `
  ucar_nav

scp "$env:TEMP\ucar_nav-deploy.tar.gz" `
  ucar@172.20.10.4:/tmp/ucar_nav-deploy.tar.gz
```

在小车端将归档解压到唯一临时目录，再同步到已确认的包目录：

```bash
staging="$(mktemp -d /tmp/ucar-nav-deploy.XXXXXX)"
tar -xzf /tmp/ucar_nav-deploy.tar.gz -C "$staging"
rsync -a --delete "$staging/ucar_nav/" \
  "$HOME/ucar_ws/src/ucar_nav/"
```

- [ ] **步骤 4：车端编译**

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make --pkg ucar_nav
source devel/setup.bash
rospack find ucar_nav
```

预期：退出码 0，包路径为 `/home/ucar/ucar_ws/src/ucar_nav`。

- [ ] **步骤 5：验证 launch 展开结果，不启动节点**

```bash
roslaunch ucar_nav robot_base_bringup.launch --nodes
roslaunch ucar_nav navigation_stack.launch --nodes
roslaunch ucar_nav ucar_navigation.launch --nodes
```

预期：

- 硬件层只有底盘、雷达及雷达 TF；
- 导航层只有 map_server、AMCL、move_base；
- 组合入口没有重复节点；
- 未出现 TEB。

- [ ] **步骤 6：启动后只做无运动检查**

在明确小车轮子不会被意外目标驱动、现场人员可立即断电的条件下：

```bash
roslaunch ucar_nav ucar_navigation.launch \
  start_robot_base:=true \
  start_camera:=false
```

另一个终端执行：

```bash
rosnode list
rostopic hz /scan
rostopic hz /odom
rosparam get /move_base/base_local_planner
rosparam get /move_base/local_costmap/footprint
rosparam get /move_base/local_costmap/inflation_layer
rosparam get /move_base/DWAPlannerROS
rosrun ucar_nav capture_nav_diagnostics.sh
```

预期：局部规划器唯一为 DWA，参数与 YAML 一致，没有速度输出导致车辆运动。

- [ ] **步骤 7：正常停止并检查残留**

在 launch 终端按 `Ctrl+C`，随后：

```bash
rosnode list
```

预期：本 launch 启动的底盘、雷达、地图、AMCL、move_base 均已退出。

---

### 任务 9：实车几何与底盘运动学验证

**文件：**

- 修改：`ucar_ws/src/ucar_nav/HANDOFF.md`
- 可能修改：`config/costmap/common.yaml`
- 可能修改：雷达 TF 的唯一上游配置文件

- [ ] **步骤 1：实测车体边界**

以 `base_link` 投影点为原点，记录：

```text
front_m
rear_m
left_m
right_m
```

若与 `0.171/0.171/0.128/0.128 m` 不同，更新 footprint 为实测最大值各增加
`0.02 m`，并同步修改测试中的期望值。

- [ ] **步骤 2：验证雷达 TF**

```bash
rosrun tf tf_echo base_link laser_frame
rostopic echo -n 1 /scan/header
```

在 RViz 中显示 `LaserScan`、局部 costmap 和 RobotModel。原地缓慢旋转时，静止墙面
不得相对地图产生明显圆周漂移。

若 TF 不正确，只修改雷达 TF 的唯一发布源，禁止增加第二个静态 TF 发布器。

- [ ] **步骤 3：验证空旷地运动方向**

每次仅发送 0.5 秒，操作者手放在急停/电源旁：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.10}}'
```

每条命令由人工按 `Ctrl+C` 停止，并立即发布一次零速度：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'
```

记录实际方向、里程计方向和停止距离。若方向不一致，停止导航测试并先修复底盘/里程计
坐标定义。

- [ ] **步骤 4：提交几何校正**

如果文件发生变化：

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v
git add ucar_ws/src/ucar_nav
git commit -m "fix: align navigation geometry with measured robot"
```

若无需变化，只更新 HANDOFF 的实测记录并提交：

```powershell
git add ucar_ws/src/ucar_nav/HANDOFF.md
git commit -m "docs: record robot geometry verification"
```

---

### 任务 10：单墙与直角弯 DWA 调试

**文件：**

- 可能修改：`config/costmap/local.yaml`
- 可能修改：`config/local_planners/dwa_safe.yaml`
- 修改：`HANDOFF.md`

- [ ] **步骤 1：录制安全基线数据**

```bash
mkdir -p ~/ucar_nav_bags
rosbag record -O ~/ucar_nav_bags/dwa-corner-baseline.bag \
  /tf /tf_static /scan /odom /amcl_pose \
  /move_base/GlobalPlanner/plan \
  /move_base/DWAPlannerROS/local_plan \
  /move_base/local_costmap/costmap \
  /cmd_vel /move_base/status
```

- [ ] **步骤 2：执行单墙测试**

固定起点、目标点和初始定位。最高前进速度保持 `0.25 m/s`、最高旋转速度保持
`0.50 rad/s`。确认沿墙时 footprint 不进入致命代价区。

若墙面在 costmap 中位置错误或跳动，停止调 DWA，返回任务 9 检查 TF/AMCL。

- [ ] **步骤 3：执行第一次直角弯测试**

只执行一次低速路线，由人工全程监控。出现以下任一条件立即取消目标并发布零速度：

- 预测车身外角距墙小于 `0.05 m`；
- 局部路径穿过墙体；
- footprint 在 costmap 中与墙重叠；
- AMCL 位姿明显跳变；
- 小车持续振荡 2 秒。

- [ ] **步骤 4：按单变量决策表调整**

每次只选一行修改并重新启动：

| 现象 | 只修改的参数 | 调整 |
|---|---|---|
| 轨迹仍过度贴墙 | `occdist_scale` | `0.15 -> 0.20 -> 0.25` |
| 安全梯度太窄 | `inflation_radius` | `0.35 -> 0.40` |
| 转弯来不及制动 | `max_vel_x` | `0.25 -> 0.20` |
| 角速度过高扫墙 | `max_vel_theta` | `0.50 -> 0.40` |
| 预测距离不足 | `sim_time` | `2.0 -> 2.5` |
| 轨迹选择过粗 | `vth_samples` | `24 -> 32` |
| 横移轨迹过粗 | `vy_samples` | `12 -> 16` |

每次调整后：

```bash
rosparam get /move_base > ~/ucar_nav_diagnostics/active_params.yaml
```

随后停止并重启原 launch，确认 YAML 是唯一来源。

- [ ] **步骤 5：完成三次直角弯验收**

同一 commit、地图、起点、目标和速度下连续三次：

- 无碰撞；
- 无人工接管；
- 无持续振荡；
- 记录最小墙距和耗时。

- [ ] **步骤 6：提交通过直角弯的最终参数**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v
git add ucar_ws/src/ucar_nav/config `
  ucar_ws/src/ucar_nav/HANDOFF.md
git commit -m "fix: tune DWA for safe cornering"
```

---

### 任务 11：当前完整路线验收

**文件：**

- 修改：`ucar_ws/src/ucar_nav/HANDOFF.md`
- 修改：`ucar_ws/src/ucar_nav/README.md`

- [ ] **步骤 1：固定完整路线条件**

在 HANDOFF 写明：

- 使用的地图及 SHA256；
- 起点 `x/y/yaw`；
- 目标点 `x/y/yaw`；
- AMCL 初始化方法；
- DWA 速度上限；
- 测试 commit。

- [ ] **步骤 2：连续执行三次完整路线**

每次均录制任务 10 所列 topics。任何碰撞趋势、定位跳变或人工接管都算该轮失败，不得
只记录成功轮次。

- [ ] **步骤 3：根据证据决定是否继续调参**

若三轮均通过，不再修改参数。

若失败，按以下顺序归因：

1. `/scan` 和 costmap 中障碍是否正确；
2. `map -> odom -> base_link` 是否连续；
3. footprint 是否正确；
4. 局部路径是否已避障但底盘没有按指令执行；
5. 最后才调整 DWA 权重或速度。

一次只验证一个假设，禁止批量改参数。

- [ ] **步骤 4：更新 README 和 HANDOFF**

记录三轮全部结果、有效参数、已知限制、部署备份路径、诊断目录和 rosbag 路径。

- [ ] **步骤 5：提交完整路线验收记录**

```powershell
git add ucar_ws/src/ucar_nav/README.md `
  ucar_ws/src/ucar_nav/HANDOFF.md
git commit -m "docs: record DWA route acceptance"
```

---

### 任务 12：最终回归与交接

**文件：**

- 检查：`ucar_ws/src/ucar_nav/**`

- [ ] **步骤 1：运行本地静态回归**

```powershell
python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v

git diff --check
git status --short
```

预期：测试全部通过，`git diff --check` 无输出，仅允许计划文档等明确保留文件。

- [ ] **步骤 2：运行车端构建与参数回归**

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make --pkg ucar_nav
source devel/setup.bash

roslaunch ucar_nav navigation_stack.launch --nodes
```

短暂启动无运动检查后：

```bash
test "$(rosparam get /move_base/base_local_planner)" \
  = "dwa_local_planner/DWAPlannerROS"
rosparam get /move_base/local_costmap/footprint
rosparam get /move_base/local_costmap/inflation_layer
rosparam get /move_base/DWAPlannerROS
```

预期：只加载 DWA，参数与最终 YAML 一致。

- [ ] **步骤 3：检查交接文件完整性**

```powershell
rg -n "task_orchestrator|pickup_navigation_goal|delivery_navigation_goal|MoveBaseAction" `
  ucar_ws/src/ucar_nav/HANDOFF.md

rg -n "TBD|TODO|以后补|稍后填写" `
  ucar_ws/src/ucar_nav/README.md `
  ucar_ws/src/ucar_nav/HANDOFF.md
```

预期：第一条包含四类交接接口信息；第二条无匹配。

- [ ] **步骤 4：提交最终收尾**

若仍有文档或测试修订：

```powershell
git add ucar_ws/src/ucar_nav
git commit -m "chore: finalize DWA navigation handoff"
```

若工作树干净，不创建空提交。

- [ ] **步骤 5：交付结果**

向用户提供：

- 分支和 worktree；
- 最终 commit；
- README 链接；
- HANDOFF 链接；
- 小车部署路径和备份路径；
- 三次直角弯和三次完整路线结果；
- 明确说明编排器适配器尚未在本分支实现。
