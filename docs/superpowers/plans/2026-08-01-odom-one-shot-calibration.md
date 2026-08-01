# U-CAR One-Shot Odometry Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 构建并执行一次性360°里程计旋转标定工具。

**Architecture:** 纯Python模块负责角度解包与完成判定，ROS节点负责消息收发、超时和停车。实车数据写入独立rosbag，参数修改推迟到人工角度反馈之后。

**Tech Stack:** Python 3、ROS 1 Noetic、`rospy`、`nav_msgs/Odometry`、`geometry_msgs/Twist`、`unittest`、rosbag。

---

### Task 1: 角度累计逻辑

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/src/ucar_waypoint_nav/odom_calibration.py`
- Test: `ucar_ws/src/ucar_waypoint_nav/test/test_odom_calibration.py`

- [ ] 先编写跨越 `-π/π` 和目标完成判定的失败测试。
- [ ] 运行测试并确认因模块不存在而失败。
- [ ] 实现最小角度累计逻辑。
- [ ] 运行测试并确认通过。

### Task 2: 一次性ROS执行节点

**Files:**
- Create: `ucar_ws/src/ucar_waypoint_nav/scripts/calibrate_odom_rotation_once.py`

- [ ] 订阅 `/odom` 并等待第一帧。
- [ ] 以20 Hz发布配置角速度。
- [ ] 达到目标、超时、里程计失联或关闭时发布零速度。
- [ ] 使用 `py_compile` 验证语法。

### Task 3: 部署与实车标定

- [ ] 部署模块和脚本到小车对应功能包。
- [ ] 启动独占的 `base_driver` 并确认 `/cmd_vel` 无其他发布者。
- [ ] 录制 `/home/ucar/ucar_nav_bags/odom_calibration/rotation_360_baseline_01.bag`。
- [ ] 执行一次360°旋转并正常关闭bag。
- [ ] 读取bag计算里程计累计角度，等待现场报告实际角度误差。

