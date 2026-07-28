# U-CAR 独立精确过弯监督器设计

## 1. 目标

新建独立 ROS 1 功能包 `ucar_corner_supervisor`，在不修改 `ucar_nav` 源码和配置的
前提下，为现有 `move_base + Navfn + TEB` 增加精确过弯控制。

监督器解决以下已由实车日志确认的问题：

- 基于固定前视窗口直接开放 TEB 横移会在弯前过早横移；
- TEB 在窄直角弯可能选择“前进接近零、持续满速横移”的局部最优；
- 仅动态修改速度上限无法保证“到达拐角后转向、转正后继续”的动作顺序。

## 2. 包边界

新包部署到：

```text
/home/ucar/ucar_ws/src/ucar_corner_supervisor
```

新包只包含：

- 路径几何与状态机；
- 速度仲裁节点；
- 独立参数文件与 launch；
- 离线测试；
- 部署、联调和故障恢复文档。

新包不得修改或覆盖：

```text
/home/ucar/ucar_ws/src/ucar_nav
/home/ucar/waypoints.xml
```

`ucar_nav` 继续由团队成员维护地图、AMCL、move_base、规划器和导航 launch。

## 3. 集成接口

启动 `move_base` 的团队 launch 必须把其速度输出重映射：

```xml
<remap from="/cmd_vel" to="/move_base/cmd_vel_raw"/>
```

数据流为：

```text
move_base
  -> /move_base/cmd_vel_raw
  -> ucar_corner_supervisor
  -> /cmd_vel
  -> base_driver
```

监督器必须是 `/cmd_vel` 的唯一发布者。原
`teb_lateral_mode_controller` 不得与监督器同时运行。

监督器订阅：

- `/move_base/cmd_vel_raw`：TEB 原始速度；
- `/move_base/NavfnROS/plan`：全局路径；
- TF `map -> base_link`：车辆位姿和航向；
- `/move_base/status`：是否存在活动导航目标。

监督器发布：

- `/cmd_vel`：仲裁后的唯一底盘速度；
- `/navigation/corner_supervisor/state`：当前状态；
- `/navigation/corner_supervisor/diagnostics`：JSON 诊断。

## 4. 路径几何

对车辆当前位置之后的全局路径等弧长重采样，寻找第一个满足以下条件的拐角：

- 入口与出口方向差不小于 `45°`；
- 拐角位于配置的最大搜索距离内；
- 拐角前后均有足够路径长度计算稳定方向。

几何分析输出：

- `corner_found`；
- `distance_to_corner`：沿路径到拐角顶点的距离；
- `turn_angle`：带符号转角；
- `exit_heading`：拐角出口在 `map` 坐标系中的方向。

不得用“整个前视窗口首尾方向差”直接决定开始横移。距离只负责选择何时进入精确转向，
从而消除提前横移。

## 5. 状态机

### 5.1 `IDLE`

无活动导航目标或原始速度超时。持续发布零速度。

### 5.2 `FOLLOWING`

正常转发 TEB 速度，但将横移限制在：

```text
abs(linear.y) <= 0.02 m/s
```

发现拐角但距离仍大于触发距离时继续 `FOLLOWING`，不得提前开放横移。

### 5.3 `TURNING`

当 `distance_to_corner <= 0.25 m` 时进入：

- 忽略 `/move_base/cmd_vel_raw`；
- 发布 `linear.x=0`、`linear.y=0`；
- 根据 `exit_heading` 和当前车头方向闭环发布 `angular.z`；
- 初始最大角速度为 `0.35 rad/s`；
- 接近目标方向时按航向误差比例减速；
- 角速度不得低于底盘可执行的最小值，除非已经进入容差。

监督器不取消 move_base 目标。move_base 可以继续计算原始速度，但在 `TURNING` 期间不会
传给底盘。

### 5.4 `EXIT_ALIGN`

航向误差不大于 `8°` 后进入候选退出状态：

- 连续保持 `0.3 s`；
- 期间任一时刻误差超限则回到 `TURNING`；
- 保持成功后进入 `FOLLOWING`；
- 为避免立即重复识别同一拐角，记录已通过拐角位置并设置最小释放距离。

## 6. 安全与故障处理

- `/move_base/cmd_vel_raw` 超过 `0.5 s` 未更新：发布零速度；
- TF 超过配置时间不可用：发布零速度并进入 `ERROR`；
- 全局路径无效时：不执行自主原地转向，仅在原始速度新鲜且存在活动目标时转发受限速度；
- `TURNING` 超过 `8 s`：发布零速度并进入 `ERROR`；
- 节点关闭或异常：连续发布零速度；
- 发现 `/cmd_vel` 存在其他发布者时在日志中明确报错，测试不得继续；
- 人工取消目标或 move_base 不再 ACTIVE：立即进入 `IDLE`。

## 7. 参数

第一版参数：

```yaml
controller_rate: 20.0
raw_cmd_timeout: 0.5
corner_search_distance: 1.2
corner_trigger_distance: 0.25
corner_min_angle_deg: 45.0
path_resample_spacing: 0.05
following_max_vel_y: 0.02
turn_max_vel_theta: 0.35
turn_min_vel_theta: 0.18
turn_kp: 0.9
heading_tolerance_deg: 8.0
heading_hold_time: 0.3
turn_timeout: 8.0
corner_release_distance: 0.45
```

第一次实测只允许单项调整。若第一弯仍提前，优先减小
`corner_trigger_distance`，不得重新增加 TEB 横移速度。

## 8. 启动与停止

团队导航 launch 先启动并保证 move_base 已 remap。随后启动：

```bash
roslaunch ucar_corner_supervisor corner_supervisor.launch
```

停止时先取消导航目标，再在监督器终端按 `Ctrl+C`。监督器退出前必须发布零速度。

## 9. 测试

### 9.1 Windows 离线测试

- 直线路径无拐角；
- 90° 左右弯能够返回拐角距离和出口航向；
- 锯齿噪声不误判；
- `FOLLOWING` 不提前横移；
- 到达触发距离进入 `TURNING`；
- TURNING 只输出角速度；
- 航向稳定 0.3 秒后退出；
- 原始速度、TF、目标或转向超时均输出零速度。

### 9.2 小车静态联调

- `/cmd_vel` 只有监督器一个发布者；
- `/move_base/cmd_vel_raw` 只有 move_base 发布；
- 无目标时监督器输出零；
- 不发真实目标也能通过构造路径验证状态切换；
- 节点停止时底盘保持停止。

### 9.3 实车验收

- 第一弯前不出现明显横移；
- 到达触发距离后原地转向；
- 转向阶段 `linear.x/y` 均为零；
- 转正后恢复前进；
- 第二弯重复相同行为；
- 不出现贴墙持续横移；
- 人工取消后立即停车。

## 10. 部署规则

只允许同步：

```text
ucar_corner_supervisor/
```

禁止再次使用完整 `ucar_nav` 打包或 `rsync --delete`。部署前只备份同名新包；若新包
尚不存在则无需备份。构建使用：

```bash
cd ~/ucar_ws
catkin_make --pkg ucar_corner_supervisor
```
