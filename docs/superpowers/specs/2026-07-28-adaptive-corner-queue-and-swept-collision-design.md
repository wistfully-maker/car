# 自适应连续拐角队列与旋转扫掠防碰设计

## 1. 背景

首次实车测试中，第一个弯通过顺畅，但第二个紧邻弯发生过转、扫墙，随后 TEB 持续报告
轨迹不可行并中止。日志证明监督器只执行了一次 `TURNING`。现有算法用拐点前后固定
`0.2 m` 的路径向量估计方向，会把 Navfn 的短斜线、平滑过渡或连续弯中间段误当成
稳定出口；同时 `TURNING` 直接发布角速度，没有检查矩形车体旋转过程是否扫入障碍物。

## 2. 目标

1. 从一条 Navfn 路径中一次提取按路径顺序排列的多个拐点。
2. 用路径简化和分段直线拟合获得稳定入口、出口方向，不依赖固定加长窗口。
3. 两个拐点之间直线很短时仍分别保留两个拐点。
4. 用沿路径的进度管理当前拐点，驶过一个拐点后立即激活下一个。
5. 在允许原地转向前验证矩形 footprint 的完整旋转扫掠区域无碰撞。
6. 转向过程中持续检查代价地图；新障碍出现时立即停车并进入 `BLOCKED`。
7. 无法可靠拟合或无法安全转向时停车，不把控制权直接交还 TEB。

本阶段不实现自动前移、后退或人工移动后的恢复动作。

## 3. 路径处理

### 3.1 去噪和折线简化

先按弧长重采样，再用 Ramer–Douglas–Peucker（RDP）算法去掉栅格锯齿。简化容差默认
`0.08 m`。简化结果仍保留原路径累计弧长，便于把拐点映射回原路径。

### 3.2 候选拐点

对简化折线的相邻线段计算有符号转角。绝对转角达到 `45°` 的顶点成为候选。相距过近
且转向相同的候选合并为一个转弯区；转向相反的候选必须保留，以支持 S 弯和连续直角弯。

### 3.3 稳定方向拟合

入口和出口分别从候选点向外收集同一方向的路径点，使用总最小二乘直线拟合。收集过程
在方向突变或到达相邻拐点时停止，因此窗口长度自适应。

每条拟合线输出：

- 航向；
- 有效长度；
- 最大正交残差；
- 使用点数。

默认最低稳定长度 `0.08 m`、最大残差 `0.08 m`。连续弯之间只有短直线时允许使用
`0.08 m`；若仍不足，则该拐点标为低置信度并禁止自动转向。

### 3.4 拐点队列

每次收到新全局路径后生成 `CornerPlan` 列表：

- `path_distance`；
- `point`；
- `entry_heading`；
- `exit_heading`；
- `turn_angle`；
- `entry_fit`、`exit_fit`；
- `confidence`。

新路径只在 `FOLLOWING` 状态替换队列；`TURNING` 和 `EXIT_ALIGN` 期间锁定当前目标。
小车沿路径的投影进度超过当前拐点和释放余量后弹出该拐点，立即使用下一项。

## 4. 旋转扫掠防碰

节点订阅 `/move_base/local_costmap/costmap`。使用地图分辨率、原点和栅格值查询占用状态。
未知栅格按占用处理。

在进入 `TURNING` 前，以当前 `map -> base_link` 位姿为中心，将矩形 footprint 从当前
航向按不大于 `3°` 的步长旋转到目标航向。每一个角度都栅格化 footprint 多边形：

- 任一覆盖栅格在 `OccupancyGrid` 中达到 `100`，判定扫掠碰撞；
- footprint 超出 costmap，判定不安全；
- costmap 超过 `1.5 s` 未更新，判定不安全。当前发布频率为 `1 Hz`，因此不能使用
  `0.5 s` 超时。

只有完整扫掠无碰撞才进入 `TURNING`。转动过程中每个控制周期重新检查“当前角度到目标
角度”的剩余扫掠；若变为不安全，状态进入 `BLOCKED` 并持续输出零速度。

## 5. 状态机变化

```text
IDLE
  -> FOLLOWING
      -> TURNING       当前拐点到达触发距离、拟合可靠、扫掠安全
      -> BLOCKED       拟合不可靠、costmap 过期或扫掠碰撞
  TURNING
      -> EXIT_ALIGN
      -> BLOCKED       转动中扫掠变为不安全
      -> ERROR         TF/原始速度超时或转向超时
  EXIT_ALIGN
      -> FOLLOWING     稳定保持完成，并推进拐点队列
      -> BLOCKED       扫掠变为不安全
```

`BLOCKED` 在当前导航目标清除前锁存，避免 TEB 再次驱动车辆。

## 6. ROS 参数

- `path_simplify_tolerance: 0.08`
- `min_stable_segment_length: 0.08`
- `max_fit_residual: 0.08`
- `same_turn_merge_distance: 0.05`
- `corner_release_margin: 0.10`
- `costmap_topic: /move_base/local_costmap/costmap`
- `costmap_timeout: 1.5`
- `lethal_cost_threshold: 100`
- `sweep_angle_step_deg: 3.0`
- `footprint: [[0.171,-0.128],[0.171,0.128],[-0.171,0.128],[-0.171,-0.128]]`

## 7. 诊断

诊断 JSON 增加：

- `corner_index`、`corner_count`；
- `corner_point`、`corner_distance`；
- `entry_heading_deg`、`exit_heading_deg`、`turn_angle_deg`；
- `entry_fit_length`、`exit_fit_length`；
- `entry_fit_residual`、`exit_fit_residual`；
- `corner_confidence`；
- `costmap_fresh`、`sweep_safe`、`blocking_cell`。

## 8. 验收

### 离线

- 直线路径无拐点；
- 单个 90° 弯方向正确；
- Navfn 锯齿和斜线不会改变稳定走廊方向；
- 两个紧邻、同向或反向的 90° 弯均输出两个有序拐点；
- 短连接段低于最低拟合长度时标为低置信度；
- 空代价地图允许旋转；
- footprint 扫过墙时拒绝旋转；
- 转动中新障碍能使状态进入 `BLOCKED`。

### 实车

先只观察诊断，不发车；确认两个现场拐点都存在、出口航向与走廊一致、扫掠安全结果合理。
随后再发送同一航点。通过标准为两个弯分别进入一次 `TURNING`，不扫墙，且最终导航不因
轨迹不可行中止。
