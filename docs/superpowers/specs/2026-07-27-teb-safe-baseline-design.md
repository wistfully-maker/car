# U-CAR TEB 安全基线切换设计

## 1. 背景与结论

当前 DWA 已能稳定以 10 Hz 发布控制指令，但实车数据表明其在起点至窄门路线中频繁输出
零速和反向速度，最终报告 `Failed to find a valid control`。提高最低速度消除了大部分
底盘低速死区指令，却没有解决局部轨迹不可行问题。

本阶段改用 TEB 建立安全基线。DWA 配置完整保留，作为对照和回退方案。

## 2. 范围

- 在小车安装 ROS Noetic 的 `teb_local_planner`。
- 新增独立的 `teb_safe.yaml`，不覆盖 `dwa_safe.yaml`。
- `navigation_stack.launch` 通过参数在 `dwa` 与 `teb` 间选择，任一时刻只启动一个局部规划器。
- TEB 第一轮沿用现有地图、AMCL、costmap、footprint 和 `0.25 m/s` 最高平移速度。
- 只测试导航安全基线，不接入任务编排器。

## 3. TEB 初始约束

- 明确启用全向底盘横移能力。
- footprint 使用现有矩形安全轮廓，不缩小车体。
- 平移速度上限 `0.25 m/s`，横移上限 `0.15 m/s`，角速度上限 `0.50 rad/s`。
- 平移加速度上限 `1.0 m/s²`，角加速度上限 `2.0 rad/s²`。
- 禁止自动缩短障碍物安全距离来换取可行路径。
- 第一轮关闭多拓扑并行搜索，先降低计算负担并获得可重复结果。

## 4. 启动与回退

正式启动时显式指定：

```bash
roslaunch ucar_nav ucar_navigation.launch \
  start_camera:=false local_planner:=teb
```

回退 DWA：

```bash
roslaunch ucar_nav ucar_navigation.launch \
  start_camera:=false local_planner:=dwa
```

启动文件必须只加载选中规划器的 YAML，不能同时加载或运行两个局部规划器。

## 5. 实测顺序

1. 静态检查：插件可加载、参数命名空间正确、无重复节点。
2. 受控短目标：验证速度方向、停车和轨迹连续性。
3. 起点到 waypoint 1：记录 `/cmd_vel`、`/odom`、TEB轨迹、最终状态和日志。
4. 与DWA对比：零速比例、反向切换、完成时间、最小墙距和是否人工急停。
5. 只有安全通过后，才把最高平移速度从 `0.25` 提高到 `0.30 m/s`。

任何阶段出现扫墙趋势，立即取消目标并保留日志；不得通过缩小 footprint 掩盖问题。

## 6. 验收

- TEB 能连续生成局部轨迹，不出现持续前后振荡。
- waypoint 1 至少成功一次且无碰撞，随后连续三次成功。
- 运行参数可追溯，DWA仍可通过一个启动参数恢复。
- 提速前完成相同路线、相同起点和相同安全轮廓下的对比。
