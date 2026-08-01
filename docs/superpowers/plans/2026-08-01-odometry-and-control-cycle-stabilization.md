# 方向里程计与导航控制周期稳定化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变底盘实际控制运动学的前提下修正逆时针里程计角度，并通过可重复诊断把导航控制输出稳定到至少8 Hz，或确定硬件可长期维持的真实频率。

**Architecture:** 将方向角速度缩放封装成无ROS依赖的小函数，由 `base_driver` 只在里程计积分和发布前调用；命令到四轮目标速度的路径保持不变。另在 `ucar_waypoint_nav` 中增加只读控制周期采集工具，先输出结构化统计，再根据证据只选择一个性能瓶颈分支实施。

**Tech Stack:** ROS 1 Noetic、C++11、catkin/gtest、Python 2.7兼容ROS节点、`rospy`、`rosbag`、PowerShell/SSH。

---

## 文件结构

- 新增本地同步包：`ucar_ws/src/ucar_controller/`
  - 保留小车当前底盘驱动的完整源码和配置，作为后续可追踪部署源。
- 新增：`ucar_ws/src/ucar_controller/include/ucar_controller/odom_angular_scale.h`
  - 无ROS依赖的方向选择函数，便于单元测试。
- 修改：`ucar_ws/src/ucar_controller/src/base_driver.cpp`
  - 加载两个参数，并只在里程计计算路径修正 `Vth`。
- 修改：`ucar_ws/src/ucar_controller/config/driver_params_ucarV2.yaml`
  - 保存方向系数，并统一 `cmd_timeout: 0.5`。
- 修改：`ucar_ws/src/ucar_controller/CMakeLists.txt`
  - 注册方向缩放单元测试。
- 新增：`ucar_ws/src/ucar_controller/test/test_odom_angular_scale.cpp`
  - 验证顺时针、逆时针、零速度和默认系数行为。
- 新增：`ucar_ws/src/ucar_waypoint_nav/scripts/diagnose_control_cycle.py`
  - 只读订阅关键话题，输出频率、最大间隔、零速度比例和时间戳异常。
- 新增：`ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/cycle_statistics.py`
  - 无ROS依赖的周期统计核心。
- 新增：`ucar_ws/src/ucar_waypoint_nav/test/test_cycle_statistics.py`
  - 覆盖周期、间隔、零速度和时间倒退统计。
- 修改：`ucar_ws/src/ucar_waypoint_nav/CMakeLists.txt`
  - 安装诊断脚本。
- 修改：`ucar_ws/src/ucar_waypoint_nav/README.md`
  - 写明标定、诊断、判断分支、部署和回退命令。
- 更新：`docs/navigation/odom-calibration-2026-08-01.md`
  - 记录一次逆时针修正后结果及导航集成观察。

### Task 1: 同步并固化当前底盘驱动基线

**Files:**
- Create: `ucar_ws/src/ucar_controller/**`
- Verify: `ucar_ws/src/ucar_controller/src/base_driver.cpp`
- Verify: `ucar_ws/src/ucar_controller/config/driver_params_ucarV2.yaml`

- [ ] **Step 1: 记录小车源码校验值**

Run:

```powershell
ssh ucar@172.20.10.3 "cd /home/ucar/ucar_ws/src && find ucar_controller -type f -print0 | sort -z | xargs -0 sha256sum"
```

Expected: 输出 `base_driver.cpp`、头文件、CMake和配置文件的SHA-256，无报错。

- [ ] **Step 2: 将小车当前包复制到本地临时目录**

Run:

```powershell
scp -r ucar@172.20.10.3:/home/ucar/ucar_ws/src/ucar_controller C:\tmp\ucar_controller_robot
```

Expected: `C:\tmp\ucar_controller_robot\src\base_driver.cpp` 存在。

- [ ] **Step 3: 比较目标目录，禁止覆盖不明本地修改**

Run:

```powershell
if (Test-Path ucar_ws\src\ucar_controller) {
  git status --short -- ucar_ws/src/ucar_controller
  git diff --no-index -- ucar_ws/src/ucar_controller C:/tmp/ucar_controller_robot
}
```

Expected: 若目标不存在则继续；若存在差异，先逐项确认来源，不使用强制覆盖。

- [ ] **Step 4: 用 `apply_patch` 将确认后的包加入工作区**

复制时必须保持原目录结构；以下文件至少与小车校验值一致：

```text
CMakeLists.txt
package.xml
include/ucar_controller/base_driver.h
src/base_driver.cpp
config/driver_params_ucarV2.yaml
launch/base_driver.launch
```

- [ ] **Step 5: 提交底盘基线**

```bash
git add ucar_ws/src/ucar_controller
git commit -m "chore: import deployed base controller baseline"
```

### Task 2: 用测试定义方向角速度缩放

**Files:**
- Create: `ucar_ws/src/ucar_controller/test/test_odom_angular_scale.cpp`
- Create: `ucar_ws/src/ucar_controller/include/ucar_controller/odom_angular_scale.h`
- Modify: `ucar_ws/src/ucar_controller/CMakeLists.txt`

- [ ] **Step 1: 写失败测试**

测试内容：

```cpp
#include <gtest/gtest.h>
#include "ucar_controller/odom_angular_scale.h"

TEST(OdomAngularScale, AppliesCcwScaleToPositiveVelocity) {
  EXPECT_NEAR(0.986, ucar_controller::scaleOdomAngularVelocity(1.0, 0.986, 1.0), 1e-9);
}

TEST(OdomAngularScale, AppliesCwScaleToNegativeVelocity) {
  EXPECT_NEAR(-0.8, ucar_controller::scaleOdomAngularVelocity(-1.0, 0.986, 0.8), 1e-9);
}

TEST(OdomAngularScale, LeavesZeroUnchanged) {
  EXPECT_DOUBLE_EQ(0.0, ucar_controller::scaleOdomAngularVelocity(0.0, 0.986, 1.0));
}

TEST(OdomAngularScale, IdentityScalesPreserveLegacyBehavior) {
  EXPECT_DOUBLE_EQ(0.7, ucar_controller::scaleOdomAngularVelocity(0.7, 1.0, 1.0));
  EXPECT_DOUBLE_EQ(-0.7, ucar_controller::scaleOdomAngularVelocity(-0.7, 1.0, 1.0));
}
```

- [ ] **Step 2: 注册并运行测试，确认失败**

在 `CMakeLists.txt` 的测试区加入：

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_odom_angular_scale test/test_odom_angular_scale.cpp)
  if(TARGET test_odom_angular_scale)
    target_include_directories(test_odom_angular_scale PRIVATE include)
  endif()
endif()
```

Run on robot or ROS Noetic environment:

```bash
cd /home/ucar/ucar_ws
catkin_make run_tests_ucar_controller_gtest_test_odom_angular_scale
```

Expected: FAIL，因为头文件或函数尚不存在。

- [ ] **Step 3: 实现最小纯函数**

创建头文件：

```cpp
#pragma once

namespace ucar_controller {

inline double scaleOdomAngularVelocity(double raw_vth,
                                       double ccw_scale,
                                       double cw_scale) {
  if (raw_vth > 0.0) return raw_vth * ccw_scale;
  if (raw_vth < 0.0) return raw_vth * cw_scale;
  return 0.0;
}

}  // namespace ucar_controller
```

- [ ] **Step 4: 运行测试确认通过**

Run:

```bash
catkin_make run_tests_ucar_controller_gtest_test_odom_angular_scale
catkin_test_results build/test_results/ucar_controller
```

Expected: 4 tests, 0 failures。

- [ ] **Step 5: 提交纯函数和测试**

```bash
git add ucar_ws/src/ucar_controller/include/ucar_controller/odom_angular_scale.h \
        ucar_ws/src/ucar_controller/test/test_odom_angular_scale.cpp \
        ucar_ws/src/ucar_controller/CMakeLists.txt
git commit -m "test: define directional odometry scaling"
```

### Task 3: 接入 `base_driver` 里程计发布路径

**Files:**
- Modify: `ucar_ws/src/ucar_controller/include/ucar_controller/base_driver.h`
- Modify: `ucar_ws/src/ucar_controller/src/base_driver.cpp`
- Modify: `ucar_ws/src/ucar_controller/config/driver_params_ucarV2.yaml`
- Modify: `ucar_ws/src/ucar_nav/launch/robot_base_bringup.launch`

- [ ] **Step 1: 在类中增加参数成员**

在 `base_driver.h` 与其他运动学参数相邻位置加入：

```cpp
double odom_angular_scale_ccw_;
double odom_angular_scale_cw_;
```

- [ ] **Step 2: 加载并打印参数**

在构造函数加载 `wheel_radius`、`base_shape_a` 的位置加入：

```cpp
pravite_nh.param("odom_angular_scale_ccw", odom_angular_scale_ccw_, 1.0);
pravite_nh.param("odom_angular_scale_cw", odom_angular_scale_cw_, 1.0);
ROS_INFO("odom angular scales: ccw=%.6f cw=%.6f",
         odom_angular_scale_ccw_, odom_angular_scale_cw_);
```

- [ ] **Step 3: 只在里程计路径修正 `Vth`**

保留四轮反解得到的原始角速度，然后立即修正：

```cpp
double Vth_raw = (-vw1 + vw2 + vw3 - vw4) /
                 (4 * (base_shape_a_ + base_shape_b_));
double Vth = ucar_controller::scaleOdomAngularVelocity(
    Vth_raw, odom_angular_scale_ccw_, odom_angular_scale_cw_);
```

后续 `delta_th`、`odom_tmp.twist.twist.angular.z` 和TF积分继续使用 `Vth`。不得修改
`cmdVelCallback` 中由 `linear_x/linear_y/angular_z` 计算四轮目标速度的代码。

- [ ] **Step 4: 固化参数和超时**

`driver_params_ucarV2.yaml` 保存：

```yaml
odom_angular_scale_ccw: 0.986
odom_angular_scale_cw: 1.000
cmd_timeout: 0.5
```

删除 `robot_base_bringup.launch` 中额外覆盖 `/base_driver/cmd_timeout` 的 `<param>`，确保
只有底盘YAML一个来源。

- [ ] **Step 5: 编译和回归测试**

```bash
cd /home/ucar/ucar_ws
catkin_make --pkg ucar_controller
catkin_make run_tests_ucar_controller
catkin_test_results build/test_results/ucar_controller
```

Expected: 编译成功，全部测试通过。

- [ ] **Step 6: 提交接入改动**

```bash
git add ucar_ws/src/ucar_controller ucar_ws/src/ucar_nav/launch/robot_base_bringup.launch
git commit -m "fix: compensate directional angular odometry"
```

### Task 4: 部署并完成一次逆时针验收

**Files:**
- Deploy: `/home/ucar/ucar_ws/src/ucar_controller/`
- Update: `docs/navigation/odom-calibration-2026-08-01.md`

- [ ] **Step 1: 备份小车待覆盖文件**

```bash
mkdir -p /home/ucar/backups/odom-scale-20260801
cp /home/ucar/ucar_ws/src/ucar_controller/src/base_driver.cpp \
   /home/ucar/backups/odom-scale-20260801/
cp /home/ucar/ucar_ws/src/ucar_controller/config/driver_params_ucarV2.yaml \
   /home/ucar/backups/odom-scale-20260801/
```

- [ ] **Step 2: 部署已提交文件并编译**

使用 `scp` 部署 `ucar_controller` 和更新后的 `robot_base_bringup.launch`，然后：

```bash
cd /home/ucar/ucar_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg ucar_controller
source devel/setup.bash
```

Expected: 编译成功。

- [ ] **Step 3: 启动并确认实际参数**

```bash
roslaunch ucar_nav robot_base_bringup.launch start_camera:=false
rosparam get /base_driver/odom_angular_scale_ccw
rosparam get /base_driver/odom_angular_scale_cw
rosparam get /base_driver/cmd_timeout
```

Expected:

```text
0.986
1.0
0.5
```

- [ ] **Step 4: 做一次逆时针原地旋转**

```bash
rosrun ucar_waypoint_nav calibrate_odom_rotation_once.py \
  --angular-speed 0.6 \
  --target-angle-deg 360 \
  --output-dir /home/ucar/ucar_nav_bags/odom_calibration
```

Expected: 安全停止；实车相对基准线的误差比原来的约5°明显减小。

- [ ] **Step 5: 记录结果**

在标定记录中写入：Git提交、参数、bag路径、里程计累计角、人工估计实际角和是否通过。

### Task 5: 为控制周期统计核心编写测试

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/test/test_cycle_statistics.py`
- Create: `ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/cycle_statistics.py`

- [ ] **Step 1: 写失败测试**

```python
import unittest
from ucar_waypoint_nav.cycle_statistics import IntervalStatistics


class IntervalStatisticsTests(unittest.TestCase):
    def test_reports_rate_and_max_gap(self):
        stats = IntervalStatistics()
        for stamp in (0.0, 0.1, 0.2, 0.5):
            stats.observe(stamp)
        result = stats.snapshot()
        self.assertAlmostEqual(result['mean_rate_hz'], 6.0, places=6)
        self.assertAlmostEqual(result['max_gap_sec'], 0.3, places=6)

    def test_counts_time_regression(self):
        stats = IntervalStatistics()
        stats.observe(2.0)
        stats.observe(1.5)
        self.assertEqual(stats.snapshot()['time_regressions'], 1)

    def test_counts_zero_commands(self):
        stats = IntervalStatistics()
        stats.observe(0.0, is_zero=True)
        stats.observe(0.1, is_zero=False)
        self.assertAlmostEqual(stats.snapshot()['zero_ratio'], 0.5)
```

- [ ] **Step 2: 运行并确认失败**

```powershell
python -m unittest ucar_ws/src/ucar_waypoint_nav/test/test_cycle_statistics.py -v
```

Expected: FAIL，因为模块尚不存在。

- [ ] **Step 3: 实现最小统计类**

实现要求：

```python
class IntervalStatistics(object):
    def __init__(self):
        self.first_stamp = None
        self.last_stamp = None
        self.intervals = []
        self.message_count = 0
        self.zero_count = 0
        self.time_regressions = 0

    def observe(self, stamp, is_zero=False):
        stamp = float(stamp)
        if self.first_stamp is None:
            self.first_stamp = stamp
            self.last_stamp = stamp
        elif stamp < self.last_stamp:
            self.time_regressions += 1
        else:
            self.intervals.append(stamp - self.last_stamp)
            self.last_stamp = stamp
        self.message_count += 1
        if is_zero:
            self.zero_count += 1

    def snapshot(self):
        elapsed = 0.0
        if self.first_stamp is not None and self.last_stamp is not None:
            elapsed = self.last_stamp - self.first_stamp
        rate = 0.0
        if elapsed > 0.0:
            rate = len(self.intervals) / elapsed
        zero_ratio = 0.0
        if self.message_count:
            zero_ratio = float(self.zero_count) / self.message_count
        return {
            'message_count': self.message_count,
            'mean_rate_hz': rate,
            'max_gap_sec': max(self.intervals) if self.intervals else 0.0,
            'zero_ratio': zero_ratio,
            'time_regressions': self.time_regressions,
        }
```

无消息和单条消息时频率返回 `0.0`。

- [ ] **Step 4: 运行测试确认通过**

```powershell
python -m unittest ucar_ws/src/ucar_waypoint_nav/test/test_cycle_statistics.py -v
```

Expected: 3 tests, OK。

- [ ] **Step 5: 提交统计核心**

```bash
git add ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/cycle_statistics.py \
        ucar_ws/src/ucar_waypoint_nav/test/test_cycle_statistics.py
git commit -m "test: add navigation cycle statistics"
```

### Task 6: 实现只读控制周期诊断工具

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/scripts/diagnose_control_cycle.py`
- Modify: `ucar_ws/src/ucar_waypoint_nav/CMakeLists.txt`
- Modify: `ucar_ws/src/ucar_waypoint_nav/README.md`

- [ ] **Step 1: 实现ROS订阅器**

脚本必须订阅：

```text
/cmd_vel
/scan
/odom
/move_base/TebLocalPlannerROS/local_plan
/move_base/GlobalPlanner/plan
```

`Twist` 使用全部分量小于 `1e-6` 判断零速度；有header的话同时统计消息时间戳与接收
时间，无header的 `/cmd_vel` 使用接收时间。脚本不得创建任何发布器。

- [ ] **Step 2: 每10秒打印并在结束时保存JSON**

输出字段固定为：

```json
{
  "duration_sec": 60.0,
  "topics": {
    "/cmd_vel": {
      "message_count": 480,
      "mean_rate_hz": 8.0,
      "max_gap_sec": 0.22,
      "zero_ratio": 0.08,
      "time_regressions": 0
    }
  }
}
```

命令行参数：

```text
--duration 60
--output /home/ucar/navigation_runs/20260801-120000/control_cycle.json
```

- [ ] **Step 3: 安装脚本并运行完整本地测试**

在 `catkin_install_python(PROGRAMS ...)` 加入脚本，然后：

```powershell
python -m unittest discover -s ucar_ws/src/ucar_waypoint_nav/test -p "test_*.py" -v
```

Expected: 全部测试通过。

- [ ] **Step 4: 提交诊断工具**

```bash
git add ucar_ws/src/ucar_waypoint_nav/scripts/diagnose_control_cycle.py \
        ucar_ws/src/ucar_waypoint_nav/CMakeLists.txt \
        ucar_ws/src/ucar_waypoint_nav/README.md
git commit -m "feat: add navigation control cycle diagnostics"
```

### Task 7: 实车采集控制周期基线

**Files:**
- Create on robot: `/home/ucar/navigation_runs/YYYYMMDD-HHMMSS/control_cycle.json`
- Create on robot: `/home/ucar/navigation_runs/YYYYMMDD-HHMMSS/process_cpu.log`
- Create on robot: `/home/ucar/navigation_runs/YYYYMMDD-HHMMSS/navigation.bag`

- [ ] **Step 1: 部署诊断工具并启动现有导航配置**

确认没有重复节点：

```bash
rosnode list | grep -E 'move_base|amcl|map_server|waypoint|ydlidar|base_driver'
rostopic info /cmd_vel
```

Expected: `/cmd_vel` 只有 `/move_base` 一个发布者。

- [ ] **Step 2: 同时记录话题和进程负载**

```bash
RUN=/home/ucar/navigation_runs/$(date +%Y%m%d-%H%M%S)
mkdir -p "$RUN"
rosbag record -O "$RUN/navigation.bag" /cmd_vel /scan /odom /tf /tf_static \
  /amcl_pose /move_base/GlobalPlanner/plan \
  /move_base/TebLocalPlannerROS/local_plan &
echo $! > "$RUN/rosbag.pid"
rosrun ucar_waypoint_nav diagnose_control_cycle.py --duration 90 \
  --output "$RUN/control_cycle.json"
kill "$(cat "$RUN/rosbag.pid")"
```

另一个终端：

```bash
pidstat -h -p $(pgrep -d, -f 'move_base|amcl|ydlidar_node|base_driver') 1 90 > "$RUN/process_cpu.log"
```

若系统没有 `pidstat`，使用：

```bash
top -b -d 1 -n 90 > "$RUN/process_cpu.log"
```

- [ ] **Step 3: 跑相同导航路线并保存ROS日志**

```bash
rosservice call /ucar_waypoint_nav/start
```

保存 `move_base` 终端输出，统计 control loop、costmap missed-rate、trajectory infeasible
出现次数和时间段。

- [ ] **Step 4: 形成单一根因判断**

使用以下判据：

- `move_base` 单核持续接近100%，同时局部路径低频：TEB/局部规划计算瓶颈；
- local costmap更新警告主导且TEB CPU不高：costmap瓶颈；
- `/scan` 或 `/odom` 有大间隔、时间倒退：传感器或底盘链；
- 局部路径频率正常但 `/cmd_vel` 零比例高：轨迹可行性问题；
- `/cmd_vel` 连续而实车断续：底盘串口/最小有效速度问题。

不得在同一轮同时得出多个未经验证的“根因”。证据不足时重复采集，不修改参数。

### Task 8: 依据证据实施一个性能修复

**Files:**
- Conditional Modify: `ucar_ws/src/ucar_waypoint_nav/config/teb.yaml`
- Conditional Modify: `ucar_ws/src/ucar_nav/config/costmap/local_teb.yaml`
- Conditional Modify: TF或驱动的已定位源文件
- Update: `ucar_ws/src/ucar_waypoint_nav/README.md`

- [ ] **Step 1: 若TEB计算是瓶颈，只启用多线程**

第一项实验只改：

```yaml
enable_multithreading: true
```

保持优化次数、障碍参数、速度和footprint不变，重新采集90秒对照。

- [ ] **Step 2: 若costmap发布是瓶颈，只降低非必要发布频率**

第一项实验只把局部costmap：

```yaml
publish_frequency: 0.5
```

保持 `update_frequency: 5.0`，避免降低障碍更新能力，重新采集90秒对照。

- [ ] **Step 3: 若TF/时间戳是瓶颈，修复源头**

定位具体发布者后，保证消息时间戳不晚于最新可用TF且单调递增。不得仅提高
`transform_tolerance`。修改前必须有能重现时间倒退或外推错误的日志证据。

- [ ] **Step 4: 若底盘执行链是瓶颈，记录串口发送周期**

只在 `base_driver` 增加节流诊断日志，记录收到 `/cmd_vel` 与实际串口发送的间隔；不得
在没有发送周期证据前调整最小速度或命令保持逻辑。

- [ ] **Step 5: 比较修改前后指标**

通过条件：

```text
/cmd_vel mean_rate_hz >= 8.0
max_gap_sec < 0.5
持续 missed-rate 消失或显著减少
直线与弯道不再因周期断续一走一停
```

若10 Hz确实不可持续，但5 Hz连续稳定，才将 `controller_frequency` 改为 `5.0`，并在
README记录这是硬件实测上限而不是性能修复。

- [ ] **Step 6: 提交被证据支持的单一修复**

```bash
git add ucar_ws/src/ucar_waypoint_nav/config/teb.yaml \
        ucar_ws/src/ucar_nav/config/costmap/local_teb.yaml \
        ucar_ws/src/ucar_controller/src/base_driver.cpp \
        ucar_ws/src/ucar_waypoint_nav/README.md
git commit -m "perf: stabilize navigation control cycle"
```

未修改的文件不会进入提交；若某个条件分支没有采用，先用 `git diff --cached --name-only`
确认暂存区只包含本轮证据支持的文件。

### Task 9: 多轮导航集成验收与交接

**Files:**
- Modify: `docs/navigation/odom-calibration-2026-08-01.md`
- Modify: `ucar_ws/src/ucar_waypoint_nav/README.md`

- [ ] **Step 1: 固定同一初始位姿和路线连续测试**

至少运行三轮，逐轮记录：

```text
Git提交
CCW/CW里程计系数
controller_frequency
平均/最低控制频率
最大命令间隔
雷达框是否转弯后漂移
是否碰墙
是否人工接管
是否到达二维码观察点
```

- [ ] **Step 2: 验证运行参数来自唯一配置**

```bash
rosparam get /base_driver/odom_angular_scale_ccw
rosparam get /base_driver/odom_angular_scale_cw
rosparam get /base_driver/cmd_timeout
rosparam get /move_base/controller_frequency
rosnode info /move_base
rostopic info /cmd_vel
```

- [ ] **Step 3: 更新README的部署、回退和诊断命令**

README必须明确：

- 正常启动命令；
- 参数文件的唯一位置；
- 如何运行一次逆时针标定；
- 如何采集控制周期；
- 如何恢复两个系数为 `1.0`；
- 如何判断是TEB、costmap、TF还是底盘执行问题。

- [ ] **Step 4: 运行最终验证**

```bash
python -m unittest discover -s ucar_ws/src/ucar_waypoint_nav/test -p "test_*.py" -v
cd /home/ucar/ucar_ws
catkin_make --pkg ucar_controller ucar_waypoint_nav
catkin_make run_tests_ucar_controller run_tests_ucar_waypoint_nav
catkin_test_results
```

Expected: 所有测试和编译通过，无新增失败。

- [ ] **Step 5: 提交交接文档**

```bash
git add docs/navigation/odom-calibration-2026-08-01.md \
        ucar_ws/src/ucar_waypoint_nav/README.md
git commit -m "docs: record navigation stabilization results"
```
