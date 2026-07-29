# U-CAR 稀疏航点引导的平滑 TEB 导航设计

## 1. 决策与范围

固定几何路线执行方案因比赛规则不允许而停止实施。本方案满足以下约束：

- 允许预设普通航点或中间航点；
- 每一段路径必须由算法根据地图在线规划；
- 使用全局规划器和 TEB 局部规划器；
- 第一阶段只完成 P 点到二维码观察点；
- 中间航点尽量穿越，不停车；
- 最终观察点停车并面向距离起点最近的墙；
- 到达后将底盘控制权交给逆时针旋转的二维码模块。

本方案不使用固定轨迹、固定转角或按时间开环运动。

## 2. 已确认位姿

- 起点 P：`x=0.0, y=0.0, yaw=0.0`。
- 二维码观察点：`x=-1.40219, y=-0.627908`。
- 最终四元数：`z=0.999639, w=-0.0268713`，即
  `yaw≈-3.0878 rad`。
- 地图：`ucar_nav/maps/map.yaml`，分辨率 `0.05 m/pixel`。

## 3. 总体架构

```text
task_orchestrator 或手动调试命令
              |
              v
waypoint_route_manager
  起点检查 -> 中间航点序列 -> 最终观察点
              |
              v  move_base action
GlobalPlanner（默认，可切换 Navfn）
              |
              v  在线全局路径
TebLocalPlannerROS
              |
              v
/cmd_vel -> base_driver
```

正常导航期间只有 `move_base/TEB` 发布 `/cmd_vel`。现有拐点监督器、固定路线执行器、
DWA 和其他速度转发节点不得同时运行。

## 4. 稀疏航点设计

航点不按每个墙角设置，而按“拓扑走廊”设置。其目的仅是约束在线规划必须经过正确
走廊，不能替代规划算法。

离线工具从当前地图中：

1. 按矩形车体外轮廓和安全余量膨胀占用栅格；
2. 从固定起点到固定终点搜索参考通路；
3. 找出走廊方向或拓扑区域发生显著变化的位置；
4. 将候选航点投影到该区域具有较大障碍间距的中心；
5. 合并距离过近、方向作用重复的候选点；
6. 输出2至3个稀疏中间航点和地图叠加预览图。

最终航点配置保存每个点的：

- 名称；
- `x/y/yaw`；
- 类型：`pass_through` 或 `terminal`；
- 切换半径；
- 切换方向容差；
- 最大等待时间。

航点不得放在墙角顶点、狭窄门槛或必须原地旋转的位置。连续弯之间若没有足够稳定
直线段，只放一个能够约束下一走廊的航点。

## 5. 穿越式航点管理

### 5.1 状态机

```text
IDLE
  -> VALIDATING_START
  -> NAVIGATING
      -> SWITCHING        中间航点满足穿越条件
      -> ARRIVED          最终航点到达
      -> BLOCKED          move_base失败或超时
  -> CANCELLED
  -> ERROR
```

### 5.2 中间航点切换

中间航点不等待 `move_base` 报告完整到达。满足以下全部条件时提前发送下一个目标：

- 小车进入该航点的切换半径；
- 小车沿路线前进方向已经越过切换门线，或朝向与下一走廊一致；
- 当前定位、TF和 `move_base` 状态有效；
- 下一个目标尚未发送。

使用进入半径与退出半径迟滞，确保一个航点只切换一次。新目标保留当前速度控制链，
管理器自身不向 `/cmd_vel` 发布零速度。

### 5.3 最终航点

最终观察点必须等待 `move_base` 成功，并额外验证：

- 位置误差在配置容差内；
- 航向面向指定墙；
- 底盘速度接近零；
- 条件持续满足稳定时间。

随后发布领取区到达结果并结束导航所有权。

## 6. 全局规划器

默认使用 `global_planner/GlobalPlanner`：

```yaml
use_dijkstra: true
use_quadratic: true
use_grid_path: false
old_navfn_behavior: false
allow_unknown: false
outline_map: true
orientation_mode: 1
```

保留 launch 参数选择 `navfn/NavfnROS`，用于同地图、同航点、同TEB的对比诊断。
正式运行只启动其中一个全局规划器，未选中规划器的参数不得加载。

GlobalPlanner首先必须通过以下静态与在线验证：

- 插件存在且能加载；
- 起终点均位于可通行代价；
- 能持续发布非空全局路径；
- 路径不穿越膨胀墙体；
- 路径方向不在直线走廊中反复翻转。

## 7. TEB 基线

第一版恢复为单一、可追踪的TEB控制链，不使用外部转弯状态机。

调参顺序固定为：

1. footprint、雷达、TF和局部代价地图；
2. 控制频率与TEB实际计算频率；
3. 前进、横移、旋转速度和加速度；
4. 障碍距离与膨胀；
5. 全局路径跟随和途经点权重；
6. 时间最优权重和提速。

优化迭代从 `no_inner_iterations=5`、`no_outer_iterations=4` 起步。控制频率以实测能够
稳定维持的频率为准；配置频率不得高于节点长期实际输出能力。

允许麦克纳姆底盘在弯道使用横移和旋转。通过较高前进运动学权重强烈抑制倒退；
若TEB仍选择持续倒退，先检查目标切换、全局路径方向和局部可行性，不使用外部固定
轨迹掩盖。

## 8. 碰墙根因验证

在评价平滑度前，必须确认墙体在同一时刻存在于：

- `/scan`；
- TF变换后的正确空间位置；
- `/move_base/local_costmap/costmap`；
- TEB障碍集合；
- 正确矩形footprint之外。

每次实车测试记录：

- `/scan`；
- `/tf`、`/tf_static`；
- `/amcl_pose`；
- `/odom`；
- 全局和局部costmap；
- 全局和局部路径；
- `/cmd_vel`；
- `move_base`与TEB日志；
- CPU占用和控制周期。

若TEB已经输出零速度但底盘继续运动，检查底盘命令超时和其他速度发布者；若costmap
中没有实际墙体，先修复雷达、TF或障碍层，不继续调整TEB轨迹权重。

## 9. ROS 包边界

新建独立包：

```text
ucar_waypoint_nav
```

包含：

```text
config/
  pickup_waypoints.yaml
  global_planner.yaml
  teb.yaml
launch/
  waypoint_teb_navigation.launch
scripts/
  derive_sparse_waypoints.py
  waypoint_route_manager.py
  capture_navigation_run.sh
src/ucar_waypoint_nav/
  map_geometry.py
  waypoint_logic.py
  protocol.py
test/
```

该包负责航点派发、规划器选择、参数归档和诊断，不实现自己的路径规划或速度控制。

## 10. 接口

- `/task/pickup_navigation_goal`：任务编排器启动领取区导航；
- `/task/pickup_arrived`：发布成功或失败及原任务标识；
- `/ucar_waypoint_nav/state`：发布状态机状态；
- `/ucar_waypoint_nav/diagnostic`：当前航点、距离、切换条件和规划器状态；
- `/ucar_waypoint_nav/start`：手动调试启动服务；
- `/ucar_waypoint_nav/cancel`：取消并停车。

## 11. 测试阶梯

1. 离线解析地图并生成航点预览；
2. GlobalPlanner对每段产生非空无碰撞路径；
3. 仿真消息测试航点只切换一次；
4. 实车只跑第一段；
5. 实车跑第一个弯；
6. 实车跑连续弯；
7. 实车完成到二维码观察点；
8. 调整穿越半径消除航点停顿；
9. 连续三轮完成后逐步提速。

任何一次碰墙、持续振荡或轨迹不可行都保留完整日志，单变量修改后重测。

## 12. 验收标准

- 每段路径由GlobalPlanner或Navfn在线生成；
- 局部轨迹由TEB在线优化；
- 2至3个中间航点约束正确走廊；
- 中间航点无明显停车；
- 直线无持续蛇形；
- 连续弯不碰墙、不持续振荡；
- 最终停在二维码观察点并面向指定墙；
- 到达后二维码模块能够接管旋转；
- 相同初始条件下连续三轮完成；
- 启动、停止、切换规划器、调参和日志操作有完整README。

## 13. 废止内容

`2026-07-29-fixed-smooth-route-executor-design.md`仅保留为历史决策记录，不进入实施。
本方案不创建 `ucar_fixed_route`，也不部署固定Bezier路线。
