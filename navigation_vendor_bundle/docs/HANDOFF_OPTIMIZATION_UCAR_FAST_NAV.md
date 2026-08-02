# `ucar_fast_nav` 后续导航优化交接

## 1. 当前状态

当前已完成：

- 对方已跑通导航链的独立复制包 `ucar_fast_nav`；
- `jie_ware/lidar_loc` 在本组小车上定位稳定，雷达不再像原 AMCL 链路那样明显漂移；
- GlobalPlanner + TEB 可以跑完到二维码观察点的路线；
- 控制频率配置为 15 Hz，实测超期多为 `0.072–0.078 s`，相比旧链路出现过的 `0.5765 s` 已大幅改善；
- 主要未解决问题是车头和车身在多个弯道蹭墙。

## 2. 关键证据

第一份完整碰墙 bag：

```text
/home/ucar/ucar_nav_bags/ucar_fast_wall_contact.bag
```

bag 信息：

```text
开始: 1785640724.20
时长: 81 s
大小: 3.9 MB
消息: 10691
压缩: lz4
```

关键话题齐全：`/scan`、`/tf`、`/odom`、`/cmd_vel`、全局/局部路径、全局/局部 costmap 和 footprint。

自动初筛发现多次：

- 前方雷达距离仅 `0.10–0.17 m`；
- 同时仍输出前进、横移和旋转命令；
- 多次角速度达到 `±1.2 rad/s`；
- 后方激光距离经常固定在约 `0.199 m`，可能是雷达看到车体或附件，后续算法不应把该值直接当成墙体碰撞。

分析脚本：

```text
 D:\program_sec\智能车\.worktrees\navigation-vendor-bundle\navigation_vendor_bundle\tools\analyze_fast_wall_bag.py
```

## 3. 当时运行参数

costmap footprint：

```yaml
footprint:
  - [ 0.156, -0.118]
  - [ 0.156,  0.118]
  - [-0.156,  0.118]
  - [-0.156, -0.118]
```

来源：

```text
/home/ucar/ucar_ws/src/ucar_fast_nav/config/move_base/costmap_common_params.yaml
```

TEB 当时运行参数：

```yaml
footprint_model:
  type: point
  vertices:
    - [ 0.164, -0.122]
    - [ 0.164,  0.122]
    - [-0.164,  0.122]
    - [-0.164, -0.122]

min_obstacle_dist: 0.15
inflation_dist: 0.5
weight_obstacle: 50.0
weight_inflation: 0.1
feasibility_check_no_poses: 5

max_vel_x: 2.4
max_vel_y: 0.2
max_vel_theta: 1.2
```

来源：

```text
/home/ucar/ucar_ws/src/ucar_fast_nav/config/move_base/teb_local_planner_params.yaml
```

costmap inflation 当时运行值：

```yaml
inflation_radius: 0.05
cost_scaling_factor: 2.0
```

局部 costmap 与 TEB 均使用 `map` frame，没有旧 waypoint + lidar_loc 混合链路的 `map -> odom` 未来外推问题。

## 4. point/polygon 实验结论

bag 录制时 TEB 的 `footprint_model.type` 为 `point`。从静态配置理论看，`point` 不会使用 `vertices` 表示真实车身，它与 costmap 的 polygon footprint 存在模型不一致。

但已进行一次实车实验：

```yaml
type: polygon
```

该轮效果比 `point` 更差。因此后续不得把“改为 polygon”当成已验证修复，也不得忽略以下联动关系：

- polygon 启用后，`min_obstacle_dist: 0.15` 会变成“车身外轮廓之外再留 15 cm”，可能使狭窄通道过度约束；
- `inflation_dist: 0.5` 与 polygon 同时使用可能进一步压缩可行轨迹；
- TEB footprint 与 costmap footprint 尺寸不同；
- 只改类型而不同步调整净空参数，不是有效的 polygon 对比实验。

开始下一轮优化前，必须先查看小车当前文件和运行参数，确认人工实验后是否已恢复 `point`：

```bash
grep -n -A10 -B2 footprint_model \
  /home/ucar/ucar_ws/src/ucar_fast_nav/config/move_base/teb_local_planner_params.yaml

rosparam get /move_base/TebLocalPlannerROS/footprint_model
```

## 5. 推荐的优化顺序

### 阶段 A：固化可重现基线

1. 备份当前能跑完全程的 point 参数。
2. 每轮记录 YAML、`rosparam dump`、bag 和视频。
3. 固定起点、终点、电池状态和现场墙体。
4. 一次只改一组强相关参数。

### 阶段 B：区分路径贴墙与车身扫墙

使用 bag 分别检查：

1. GlobalPlanner 路径是否已经贴墙；
2. TEB local plan 是否在弯道内侧过度切角；
3. local costmap 是否存在现实墙体；
4. 规划中心轨迹正确时，矩形车身外角是否仍越界；
5. TEB 已输出停车时，底盘是否仍执行旧 `/cmd_vel`。

### 阶段 C：polygon 联动实验

若再测 polygon，必须作为一套相互匹配的实验参数，而不是只改 `type`。可从以下范围起步：

```yaml
footprint_model:
  type: polygon
  vertices:
    - [ 0.156, -0.118]
    - [ 0.156,  0.118]
    - [-0.156,  0.118]
    - [-0.156, -0.118]

min_obstacle_dist: 0.02–0.04
inflation_dist: 0.07–0.10
```

`weight_obstacle`、速度和 costmap inflation 先保持基线，第一轮只验证是否还存在车身扫墙。若无可行轨迹，先检查走廊实际净宽和 footprint，不要立即继续缩小车身。

### 阶段 D：速度和平滑性

只在连续至少三轮无碰撞后调整：

- `max_vel_x/max_vel_y/max_vel_theta`；
- 加速度限制；
- `dt_ref`、优化迭代次数和控制频率；
- `weight_optimaltime`、`weight_viapoint` 和路径跟随。

不要通过单纯降低速度掩盖几何碰撞问题。

## 6. 每轮必录数据

```bash
rosbag record --lz4 \
  -O /home/ucar/ucar_nav_bags/ucar_fast_wall_contact_next \
  /scan /tf /tf_static /odom /cmd_vel /map \
  /move_base/goal /move_base/status /move_base/result \
  /move_base/GlobalPlanner/plan \
  /move_base/TebLocalPlannerROS/global_plan \
  /move_base/TebLocalPlannerROS/local_plan \
  /move_base/local_costmap/costmap \
  /move_base/local_costmap/costmap_updates \
  /move_base/local_costmap/footprint \
  /move_base/global_costmap/costmap \
  /move_base/global_costmap/costmap_updates \
  /move_base/global_costmap/footprint
```

并保存：

```bash
rosparam dump /home/ucar/ucar_nav_bags/ucar_fast_wall_contact_next_params.yaml
```

人工记录碰撞时间点、碰撞部位、是否自行恢复和是否人工急停。

## 7. 验收标准

1. 相同起点到二维码观察点连续三轮无车头、车尾或侧边碰墙。
2. 雷达与地图全程稳定贴合。
3. 无持续振荡、异常倒车或人工接管。
4. 控制周期长期接近配置值，不出现类似 `0.5 s` 的严重超时。
5. 每轮都能追溯到确切 YAML、运行参数、bag 和代码版本。
