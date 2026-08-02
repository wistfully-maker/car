# 另一组导航链提取交接

## 当前完成范围

本归档用于明天将另一组已跑通的导航链部署到本组小车。它不包含任务编排器联调，也不修改本组 `ucar_nav`。

当前选择的链路是：

```text
ucar_controller + ydlidar + jie_ware/lidar_loc
  + map_server(002.yaml)
  + GlobalPlanner
  + TebLocalPlannerROS
```

## 已确认事实

1. 当前 `ucar_navigation.launch` 使用 `jie_ware/lidar_loc`，不是 AMCL。
2. 带 `bak_lidar_loc_20260801_175454` 的文件实际是 AMCL omni 旧入口。
3. 当前与备份的规划器均为 GlobalPlanner + TEB；差异主要是定位。
4. TEB 配置含 `max_vel_y` 和 `acc_lim_y`，属于全向底盘配置。
5. 原 launch 明确指定 GlobalPlanner 和 TEB，覆盖 `move_base_params.yaml` 内遗留的 Astar/DWA 默认值。
6. 对方曾通过 `roslaunch ucar_nav ucar_navigation.launch` 实际启动此链，ROS 日志确认加载 TEB 和 GlobalPlanner。
7. dynamic_obstacle 链接口没有完全闭合，首轮复现不启用。
8. `config/pickup_goal.yaml` 只保存二维码观察点终点；航向已由 `-3.0878 rad` 反转为 `0.053792653589793 rad`。
9. `pickup_navigation.launch` 只加载终点参数，不依赖、不启动 `ucar_waypoint_nav`，也不会自动发送目标。

## 明天优先比较的硬件差异

- `/dev/ttyS4` 是否仍为本组雷达；
- 本组 `base_link -> laser_frame` 是否应使用 `x=0.11, z=0.13`；
- 本组已标定的 `ucar_controller` 里程计参数与对方参数的差异；
- 本组 `jie_ware/lidar_loc.cpp` 的 SHA256；
- 地图 `002.yaml` 是否与现场摆放一致；
- 实际 `/move_base/*` 参数是否来自本归档，而非参数服务器残留。

## 首轮成功标准

- 所有运行检查通过；
- 激光点云在起点与地图贴合；
- 旋转后 `map -> odom` 不出现明显随机跳变；
- GlobalPlanner 生成非空、不过墙的路径；
- TEB 生成局部轨迹并持续输出控制；
- 完成 P 点到二维码观察点且无碰撞；
- 保存 rosbag、参数快照、启动日志和完成时间。

## 失败时按层判断

1. **不能编译**：检查缺失包或 `jie_ware` 版本，不改规划参数。
2. **没有 TF**：检查底盘 odom、雷达静态 TF 和 lidar_loc，不调 TEB。
3. **激光不贴地图**：检查雷达 TF、初始位姿、地图和 lidar_loc，不调速度。
4. **全局路径错误**：检查 global costmap 与 GlobalPlanner。
5. **全局路径正确但不动**：检查 local costmap、TEB、控制频率和 `/cmd_vel` 所有权。
6. **命令正常但实车异常**：检查底盘驱动、最小有效速度和里程计，不修改地图掩盖问题。

## 后续联调门槛

独立导航链至少完整跑通一次并保存证据后，才接入 `task_orchestrator`。联调时应复用 `move_base` action，不复制或再次启动底盘、雷达、定位和 move_base。
