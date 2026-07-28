# U-CAR 可切换过弯安全导航设计

## 1. 目标

建立两套可在同一启动文件中切换的过弯安全配置：

1. `NavfnROS + TEB`；
2. `NavfnROS + DWA`。

同时纳入两份曾经能够完成导航的历史压缩包，建立两个独立的 legacy 对照配置：

3. 2026-07-17 `GlobalPlanner + TEB`；
4. 2026-07-21 `AstarPlannerRos + DWA`。

四套方案使用相同地图、航点和车体轮廓，
通过同一起点到 `/home/ucar/waypoints.xml` 航点的实车测试比较效果。

本阶段只解决稳定生成全局路径和通过固定直角弯，不接入任务编排器，也不追求比赛
极限速度。

## 2. 启动接口

`navigation_stack.launch` 增加两个独立参数：

```text
global_planner:=navfn
local_planner:=teb
```

或：

```text
global_planner:=navfn
local_planner:=dwa
```

历史配置通过独立 profile 选择：

```text
navigation_profile:=legacy_0717_teb
navigation_profile:=legacy_0721_dwa
```

保留 `global_planner/GlobalPlanner` 作为诊断回退选项，但首轮实车只比较
两套 corner-safe 配置和两套 legacy 配置。

任意一次启动只能加载一个全局规划器和一个局部规划器的参数。

## 2.1 历史压缩包来源

历史配置来源和校验值：

```text
E:\ucar_ws_backup.tar.gz
SHA256 4962E3BF5B6D5B3D017DD2337B9CA909EE1084ECF4E189B998022D2E0BF56C0B

E:\ucar_wsbackup.tar.gz
SHA256 3D768C9C8E51F2E58C032B3C719B9EC3DDA47B650C578EA3D3B0CFA2C48F3BA4
```

只提取并整理 `ucar_nav` 的导航配置，不复制 `build`、`devel` 或整个工作空间。
原始文件保存在工作区外的只读参考目录：

```text
D:\program_sec\智能车\external\original_navigation_archives_20260728
```

## 3. 不修改的基准

首轮测试不修改：

- `maps/map.yaml` 及其 PGM；
- `/home/ucar/waypoints.xml`；
- 车体 footprint：

```yaml
footprint:
  - [0.171, -0.128]
  - [0.171,  0.128]
  - [-0.171, 0.128]
  - [-0.171, -0.128]
```

- 小车物理起点。

每轮测试从相同物理起点重新初始化 AMCL。只有当 Navfn 在小车静止时仍不能为原航点
生成路径，才检查起点格、目标格及通道代价值；不提前修改地图或航点。

两套 legacy 配置原本引用 `maps/map_new.yaml`。整理后的 profile 必须改为当前
`maps/map.yaml`，不得把压缩包内旧地图部署为比赛地图。

## 4. 代价地图

### 4.1 全局代价地图

全局代价地图只包含：

- `static_layer`；
- `inflation_layer`。

不加载实时雷达障碍层，避免定位误差或重复墙体阻断全局路径。

首轮参数：

```yaml
global_frame: map
rolling_window: false
inflation_radius: 0.25
cost_scaling_factor: 2.5
```

### 4.2 局部代价地图

局部代价地图包含：

- `obstacle_layer`；
- `inflation_layer`。

首轮参数：

```yaml
global_frame: odom
rolling_window: true
update_frequency: 5.0
inflation_radius: 0.25
cost_scaling_factor: 2.5
```

使用 `odom` 滚动窗口，避免 AMCL 小幅修正直接造成局部障碍和轨迹跳动。

## 5. Navfn 配置

```yaml
NavfnROS:
  allow_unknown: true
  default_tolerance: 0.15
  visualize_potential: false
```

Navfn 只负责生成全局路径，不控制底盘。

## 6. TEB 过弯基线

```yaml
max_vel_x: 0.20
max_vel_x_backwards: 0.08
max_vel_y: 0.08
max_vel_theta: 0.40

acc_lim_x: 0.50
acc_lim_y: 0.50
acc_lim_theta: 0.80

max_global_plan_lookahead_dist: 0.8
global_plan_viapoint_sep: 0.15
weight_viapoint: 5.0

min_obstacle_dist: 0.08
inflation_dist: 0.20
weight_obstacle: 100.0

no_inner_iterations: 2
no_outer_iterations: 1
enable_homotopy_class_planning: false
```

短前瞻用于减少直角弯处切弯；via-point 约束用于让局部轨迹跟随全局路径；限制横移
用于减少直线左右摇摆。低迭代次数用于保证控制循环频率。

## 7. DWA 过弯基线

```yaml
max_vel_x: 0.20
min_vel_x: -0.08
max_vel_y: 0.08
min_vel_y: -0.08
max_vel_theta: 0.40
min_vel_theta: 0.18

acc_lim_x: 0.80
acc_lim_y: 0.80
acc_lim_theta: 1.20

sim_time: 1.2
vx_samples: 8
vy_samples: 5
vth_samples: 16

path_distance_bias: 40.0
goal_distance_bias: 16.0
occdist_scale: 0.20
stop_time_buffer: 0.50
```

DWA 与 TEB 使用相同速度上限。提高路径和障碍权重，优先保证沿全局路径且不扫墙。

## 8. 测试流程

按用户要求，实车测试直接发送原航点，不再执行额外直行、旋转或分段运动测试。

测试顺序：

1. `NavfnROS + TEB`；
2. `NavfnROS + DWA`。
3. `legacy_0717_teb`；
4. `legacy_0721_dwa`。

两套 legacy 配置保留压缩包中的原始规划器、评分、代价地图和速度参数，不加载统一
限速覆盖层。因此 `legacy_0717_teb` 使用原文件中的 `2.4 m/s`，
`legacy_0721_dwa` 使用原文件中的 `0.45 m/s`。

四套方案的速度不同，测试结果必须分别记录完成时间和碰撞情况，不能只根据完成时间
判断规划器优劣。实车测试由现场人员全程看护。

每轮：

1. 从同一物理起点初始化 AMCL；
2. 启动指定规划器组合；
3. 向 `/home/ucar/waypoints.xml` 的目标航点发送受控导航目标；
4. 设置超时；
5. 成功、失败或超时后均取消目标并发布零速度；
6. 记录完成时间、是否碰墙、是否卡住、是否左右摇摆及日志。

出现碰撞趋势时允许立即人工停止。

## 9. 验收标准

候选配置必须满足：

- 能为原航点生成全局路径；
- 能通过固定直角弯；
- 不碰墙；
- 不持续左右振荡；
- 不因规划失败长期保持非零速度；
- 相同组合连续三次成功。

首轮只要求找到能完成一次全路线的候选配置；随后再进行三次重复验收。

## 10. 回退

现有配置、Git历史和两份压缩包保持不删除。所有新配置使用独立文件，启动参数决定
加载哪套方案。四套方案均失败时，仍可切回当前配置或从导入提交 `aaa6cb3` 提取
最初配置进行独立对照，不直接覆盖当前工作文件。
