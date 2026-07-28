# U-CAR 曲率感知横移控制与 AMCL 初始化设计

## 1. 背景

当前 `NavfnROS + TebLocalPlannerROS` 已能生成并执行通往二维码启动点的路径，但实车测试
中存在以下问题：

- 直线路段会出现不必要的横移；
- 直角弯处仍需要麦克纳姆底盘的横移能力；
- 过弯后车身可能尚未转正，不能在刚越过拐点时立即关闭横移；
- 直接增加 TEB 优化迭代会使控制周期从目标 5 Hz 恶化到约 1 Hz，不能依赖提高计算量
  解决问题；
- 小车被人工搬回起点或 AMCL 重启后，需要重复发布初始位姿。

本设计通过一个轻量级曲率感知控制节点动态调整 TEB 的横移上限，并提供独立的 AMCL
初始化脚本。第一阶段不自动接入完整比赛编排器。

## 2. 目标

### 2.1 横移控制

- 直线路段保留极小横移能力，避免明显左右漂移；
- 在前方出现直角弯时提前开放较高横移能力；
- 过弯后等待车身与出口路径方向对齐，再恢复直线模式；
- 不重启 `move_base`，不切换规划器实例；
- 不过滤或篡改最终 `/cmd_vel`；
- 不明显增加小车 CPU 负载；
- 所有判断阈值和速度均可通过 ROS 参数调整。

### 2.2 AMCL 初始化

- 提供一条简短命令发布固定起点的 `/initialpose`；
- 默认起点为 `x=0`、`y=0`、`yaw=0`；
- 支持通过 ROS 参数覆盖坐标、朝向和协方差；
- 发布前等待 AMCL 订阅者，避免消息在节点尚未就绪时丢失；
- 本阶段不自动包含到导航 launch 中。

## 3. 不在本阶段实现

- 不安装或接入 `astar_planner`；
- 不继续调整 DWA 或旧版 GlobalPlanner 配置；
- 不开启 TEB Homotopy Class Planning；
- 不根据固定地图坐标硬编码某一个弯道；
- 不修改全局规划器生成的路径；
- 不将 AMCL 初始化自动接入比赛任务编排器；
- 不让控制节点直接发布底盘 `/cmd_vel`。

## 4. 方案选择

采用“订阅全局路径、判断局部曲率、通过 dynamic reconfigure 修改 TEB 横移参数”的
方案。

未采用以下方案：

- `/cmd_vel` 横移过滤：TEB 会误以为横移命令已被底盘执行，导致内部预测与真实运动
  不一致；
- 两套 `move_base` 切换：切换成本高，并会重新引入重复节点和参数所有权问题；
- 仅使用固定 TEB 权重：无法明确保证直线与弯道使用不同横移上限。

## 5. ROS 组件

### 5.1 `teb_lateral_mode_controller.py`

职责：

- 订阅 `/move_base/NavfnROS/plan`；
- 使用 TF 获取 `map -> base_link`；
- 在全局路径中找到距离机器人最近的路径点；
- 截取机器人前方固定距离的路径；
- 对截取路径重采样，降低网格锯齿和点密度变化的影响；
- 计算入口方向、出口方向、前方累计转向角和车头方向误差；
- 维护 `STRAIGHT` 与 `CORNER` 状态；
- 仅在状态变化时，通过 dynamic reconfigure 修改 TEB 的
  `max_vel_y` 和 `acc_lim_y`；
- 发布当前模式和诊断数据。

节点不拥有 `move_base`、AMCL、雷达或底盘节点的生命周期。

### 5.2 `initialize_amcl.py`

职责：

- 从私有 ROS 参数读取固定起点与协方差；
- 等待 `/initialpose` 至少有一个订阅者；
- 发布一次 `geometry_msgs/PoseWithCovarianceStamped`；
- 使用 `map` 作为默认坐标系；
- 将角度参数从弧度转换为四元数；
- 输出本次发布的坐标、角度和协方差；
- 超时未发现 AMCL 订阅者时返回非零状态。

## 6. 横移状态机

### 6.1 `STRAIGHT`

参数：

```yaml
max_vel_y: 0.02
acc_lim_y: 0.20
```

进入条件：

- 节点启动；
- 全局路径缺失或过期；
- TF 查询失败；
- dynamic reconfigure 服务暂时不可用；
- `CORNER` 的退出条件连续成立。

直线模式仍保留 `0.02 m/s` 横移，用于抵消小误差，但不能形成肉眼明显的横向行驶。

### 6.2 `CORNER`

参数：

```yaml
max_vel_y: 0.18
acc_lim_y: 0.60
```

进入条件：

- 机器人前方 `0.8 m` 范围内，重采样路径的入口方向与出口方向差达到或超过 `45°`。

进入后保持弯道模式，直到退出条件全部满足，不能因为单帧路径波动提前退出。

### 6.3 出弯条件

必须同时满足：

1. 前方重采样路径的方向变化低于 `10°`；
2. 车头朝向与出口路径方向误差低于 `10°`；
3. 上述条件连续保持至少 `0.5 s`。

采用连续保持时间形成滞回，避免在直角拐点附近频繁切换。

## 7. 路径分析

### 7.1 最近点

在收到的全局路径中，以机器人在 `map` 坐标系的位置寻找最近路径点。分析窗口从该点
开始，禁止使用机器人已经通过的路径段判断未来弯道。

### 7.2 前视窗口

沿路径累计距离，截取默认 `0.8 m`。窗口长度为 ROS 参数：

```yaml
lookahead_distance: 0.8
```

如果剩余路径长度不足，则使用所有剩余点。

### 7.3 重采样与角度

- 以固定弧长间隔重采样，默认间隔 `0.10 m`；
- 忽略长度过短的线段；
- 对角度差统一归一化到 `[-π, π]`；
- 入口方向使用窗口前部若干线段的平均方向；
- 出口方向使用窗口后部若干线段的平均方向；
- 不以单个相邻路径点的角度作为直角弯依据。

这可以抑制 Navfn 栅格路径的小幅锯齿。

## 8. 参数

建议初始参数：

```yaml
controller_rate: 5.0
plan_topic: /move_base/NavfnROS/plan
map_frame: map
base_frame: base_link
lookahead_distance: 0.8
resample_spacing: 0.10
corner_enter_angle_deg: 45.0
corner_exit_angle_deg: 10.0
heading_exit_tolerance_deg: 10.0
exit_hold_time: 0.5
plan_timeout: 1.0
straight_max_vel_y: 0.02
straight_acc_lim_y: 0.20
corner_max_vel_y: 0.18
corner_acc_lim_y: 0.60
reconfigure_namespace: /move_base/TebLocalPlannerROS
```

AMCL 初始化脚本建议参数：

```yaml
frame_id: map
x: 0.0
y: 0.0
yaw: 0.0
covariance_x: 0.10
covariance_y: 0.10
covariance_yaw: 0.0685
subscriber_timeout: 10.0
```

## 9. 数据输出

控制节点发布：

- `/navigation/lateral_mode`，类型 `std_msgs/String`，值为 `STRAIGHT` 或 `CORNER`；
- `/navigation/lateral_mode_diagnostics`，第一版使用 JSON 字符串，至少包含：
  - 当前模式；
  - 前方路径方向变化；
  - 车头与出口方向误差；
  - 当前前视路径长度；
  - 最近路径点索引；
  - 最近一次参数切换时间；
  - 最近错误信息。

日志只在状态变化、服务错误和路径/TF 超时时输出，避免高频刷屏。

## 10. 异常与失效保护

- 没有全局路径：切换或保持 `STRAIGHT`；
- 路径超时：切换或保持 `STRAIGHT`；
- 路径点不足：切换或保持 `STRAIGHT`；
- TF 查询失败：切换或保持 `STRAIGHT`；
- TEB dynamic reconfigure 服务不可用：周期性重试，但不启动底盘运动；
- 参数切换失败：记录错误并保持最近一次已确认的模式；
- 收到新全局路径：重新计算最近点和前视窗口，但不无条件重置模式；
- ROS 关闭：尽力将 TEB 恢复为 `STRAIGHT` 参数。

控制节点故障不能直接停止 `move_base`，但其默认和失效状态均限制横移为直线模式。

## 11. 启动方式

新增独立 launch：

```bash
roslaunch ucar_nav teb_lateral_mode_controller.launch
```

在 `navfn_teb_corner` 完成独立验证后，可由 `navigation_stack.launch` 通过参数选择是否
包含：

```text
enable_lateral_mode_controller:=true
```

第一轮联调允许分步启动，以便单独观察状态话题和 dynamic reconfigure 参数。

AMCL 初始化：

```bash
rosrun ucar_nav initialize_amcl.py
```

覆盖起点示例：

```bash
rosrun ucar_nav initialize_amcl.py \
  _x:=0.10 _y:=-0.05 _yaw:=3.14159
```

## 12. 测试

### 12.1 离线单元测试

- 直线路径判定为 `STRAIGHT`；
- 轻微栅格锯齿不会进入 `CORNER`；
- 90° 路径进入 `CORNER`；
- 接近弯道时能够提前进入 `CORNER`；
- 越过拐点但车头未转正时继续保持 `CORNER`；
- 路径变直且车头转正并保持 0.5 秒后退出；
- 角度跨越 `±π` 时计算正确；
- 无路径、旧路径、短路径和 TF 失败回到 `STRAIGHT`；
- 状态未变化时不重复调用 dynamic reconfigure。

### 12.2 小车静态联调

- 启动控制节点后 TEB 横移参数变为直线值；
- 发布人工直线路径时保持 `STRAIGHT`；
- 发布人工 90° 路径时切换为 `CORNER`；
- 状态切换时 TEB 参数实际变为对应值；
- 停止发布路径后恢复 `STRAIGHT`。

### 12.3 实车航点测试

- 使用固定起点、固定 AMCL 初值和固定二维码航点；
- 直线段不出现明显横向行驶；
- 进入直角弯前切换为 `CORNER`；
- 弯中允许横移但不触墙；
- 出弯且车头转正后恢复 `STRAIGHT`；
- 控制循环不出现持续低于 5 Hz 的情况；
- 每轮保留模式日志、路径、AMCL 位姿和 `move_base` 日志。

## 13. 第一版验收标准

1. 曲率判断的离线测试全部通过；
2. 节点只修改 TEB 的 `max_vel_y` 和 `acc_lim_y`；
3. 无路径或 TF 异常时进入直线模式；
4. 直线路段横移上限为 `0.02 m/s`；
5. 直角弯横移上限为 `0.18 m/s`；
6. 出弯后车头转正才恢复直线模式；
7. AMCL 初始化脚本可用一条命令完成固定起点初始化；
8. 不新增重复的底盘、雷达、AMCL 或 `move_base` 节点；
9. 所有阈值均可通过 ROS 参数调整；
10. 实车结果不满足要求时，仅根据模式诊断数据单项调整阈值。
