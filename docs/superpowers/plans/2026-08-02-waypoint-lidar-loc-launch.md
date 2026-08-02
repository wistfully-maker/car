# 航点导航切换 lidar_loc 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不修改 `ucar_nav/config` 和原 AMCL launch 的前提下，新增使用 `jie_ware/lidar_loc` 的航点导航入口。

**Architecture:** 新 launch 复制现有航点导航栈的参数加载与管理器启动方式，仅将 `amcl` 节点替换为 `jie_ware/lidar_loc`。原 `waypoint_teb_navigation.launch` 保持不变，两种定位方式不允许同时启动。

**Tech Stack:** ROS 1 Noetic launch XML、Python `unittest`、`xml.etree.ElementTree`

---

### Task 1: 锁定新 launch 的边界

**Files:**
- Modify: `ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py`
- Create: `ucar_ws/src/ucar_waypoint_nav/launch/waypoint_teb_lidar_loc.launch`

- [ ] **Step 1: 写失败测试**

测试新 launch 可解析，启动 `jie_ware/lidar_loc`，不启动 `amcl`，保留 `move_base` 和航点管理器，并确认原 launch 仍包含 AMCL。

- [ ] **Step 2: 运行测试并确认因新文件缺失而失败**

```powershell
python -m unittest ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py -v
```

- [ ] **Step 3: 新增最小 lidar_loc launch**

复用当前参数文件，启动 `map_server`、`lidar_loc`、`move_base` 和 `waypoint_route_manager.py`。

- [ ] **Step 4: 运行单测与 XML 静态验证**

```powershell
python -m unittest ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py -v
```

- [ ] **Step 5: 提交仅本次相关文件**

```powershell
git add docs/superpowers/plans/2026-08-02-waypoint-lidar-loc-launch.md ucar_ws/src/ucar_waypoint_nav/test/test_ros_assets.py ucar_ws/src/ucar_waypoint_nav/launch/waypoint_teb_lidar_loc.launch
git commit -m "feat: add lidar localization waypoint launch"
```

### Task 2: 部署与静态验证

**Files:**
- Deploy: `/home/ucar/ucar_ws/src/ucar_waypoint_nav/launch/waypoint_teb_lidar_loc.launch`

- [ ] **Step 1: 备份同名远程文件（若存在）**
- [ ] **Step 2: SCP 新 launch 到小车**
- [ ] **Step 3: 编译 `ucar_waypoint_nav`**
- [ ] **Step 4: 用 `roslaunch --files` 和 `--dump-params` 验证节点与参数，不真正启动小车**
- [ ] **Step 5: 确认原 AMCL launch 未变、`ucar_nav/config` 未变**
