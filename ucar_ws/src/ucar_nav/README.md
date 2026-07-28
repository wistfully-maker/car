# ucar_nav：DWA 安全导航包

本包为 U-CAR 的 ROS 1 Noetic 导航配置包。当前版本只启用
`dwa_local_planner/DWAPlannerROS`，目标是先建立稳定、可重复、不会在直角弯处扫墙
的安全基线。

本包不实现 `task_orchestrator` 导航适配器，也不包含 TEB、MPPI 或 MPC。后续任务
编排接入说明见 [`HANDOFF.md`](HANDOFF.md)。

## 1. 当前架构

```text
robot_base_bringup.launch
  ├─ base_driver
  ├─ ydlidar_node
  ├─ 雷达 TF（由 ydlidar.launch 提供）
  └─ usb_cam（默认关闭）

navigation_stack.launch
  ├─ map_server
  ├─ amcl
  └─ move_base
       ├─ GlobalPlanner
       └─ DWAPlannerROS

ucar_navigation.launch
  └─ 按参数组合以上两层
```

同一个 `move_base` 实例同一时刻只能运行一个局部规划器。当前只加载 DWA；旧 TEB
参数和旧 TEB RViz 配置已从活动目录删除。

## 2. 文件位置

小车部署路径：

```text
/home/ucar/ucar_ws/src/ucar_nav
```

关键文件：

| 文件 | 作用 |
|---|---|
| `launch/robot_base_bringup.launch` | 底盘、雷达、可选摄像头 |
| `launch/navigation_stack.launch` | 地图、AMCL、move_base |
| `launch/ucar_navigation.launch` | 兼容的一键组合入口 |
| `config/amcl/amcl_omni.yaml` | AMCL 唯一参数源 |
| `config/costmap/common.yaml` | footprint、雷达障碍源、公共 inflation |
| `config/costmap/global.yaml` | 全局代价地图 |
| `config/costmap/local.yaml` | 局部滚动代价地图 |
| `config/local_planners/dwa_safe.yaml` | 当前 DWA 安全基线 |
| `config/global_planner.yaml` | GlobalPlanner 参数 |
| `config/move_base.yaml` | move_base 频率、耐心和恢复行为 |
| `scripts/capture_nav_diagnostics.sh` | 保存只读诊断快照 |
| `test/test_navigation_config.py` | Windows 可运行的静态配置测试 |

## 3. 部署前准备

### 3.1 Windows 打包并上传到暂存区

在 PowerShell：

```powershell
cd D:\program_sec\智能车\.worktrees\navigation-safety

tar --exclude="__pycache__" --exclude="*.pyc" `
  -czf "$env:TEMP\ucar_nav-deploy.tar.gz" `
  -C .\ucar_ws\src ucar_nav

scp "$env:TEMP\ucar_nav-deploy.tar.gz" `
  ucar@172.20.10.4:/tmp/ucar_nav-deploy.tar.gz

ssh ucar@172.20.10.4
```

不要直接用 `scp -r` 覆盖正式目录，因为它不会删除已经从新版移除的旧参数文件。

### 3.2 车端备份与同步

备份必须放在 catkin 工作空间外，避免 catkin 扫描出两个同名 `ucar_nav` 包：

```bash
stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p "$HOME/ucar_nav_backups"
cp -a ~/ucar_ws/src/ucar_nav \
  "$HOME/ucar_nav_backups/ucar_nav-${stamp}"
printf '%s\n' "$HOME/ucar_nav_backups/ucar_nav-${stamp}"

staging="$(mktemp -d /tmp/ucar-nav-deploy.XXXXXX)"
tar -xzf /tmp/ucar_nav-deploy.tar.gz -C "$staging"
rsync -a --delete "$staging/ucar_nav/" \
  "$HOME/ucar_ws/src/ucar_nav/"
```

`rsync --delete` 只允许将已经核对的暂存包同步到明确的
`/home/ucar/ucar_ws/src/ucar_nav/`，不得对 `src/` 或工作空间根目录执行。

### 3.3 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/ucar_ws
catkin_make --pkg ucar_nav
source devel/setup.bash
rospack find ucar_nav
```

最后一条应输出：

```text
/home/ucar/ucar_ws/src/ucar_nav
```

## 4. 启动前冲突检查

导航 launch 涉及底盘串口、雷达串口和可选摄像头。启动前必须执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash

rosnode list
rosnode ping -c 1 /base_driver
rosnode ping -c 1 /ydlidar_node
rosnode ping -c 1 /amcl
rosnode ping -c 1 /move_base
rosnode ping -c 1 /usb_cam
```

判断方法：

- `rosnode list` 中不存在节点：可以由本包启动；
- 节点存在且 `rosnode ping` 成功：节点正在运行，不得再次启动；
- 节点存在但 ping 失败：ROS Master 中可能是僵尸登记，应先确认实际进程；
- 不要仅凭节点名存在就判断进程正常。

检查进程：

```bash
ps -eo pid,ppid,args | grep -E \
  '[b]ase_driver|[y]dlidar|[m]ove_base|[a]mcl|[u]sb_cam|roslaunch'
```

不要为了清理本包而停止其他团队的底盘、雷达、相机或导航 launch。优先回到原
launch 终端按 `Ctrl+C`。

## 5. 启动方式

### 5.1 全部相关节点均未启动

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash

roslaunch ucar_nav ucar_navigation.launch \
  start_robot_base:=true \
  start_camera:=false
```

这会启动：

- 底盘；
- 雷达；
- 地图；
- AMCL；
- move_base。

默认不启动相机，避免与 QR 模块争抢设备。

### 5.2 底盘和雷达已经由其他 launch 启动

```bash
roslaunch ucar_nav ucar_navigation.launch \
  start_robot_base:=false
```

此时只启动地图、AMCL 和 move_base。必须确认外部节点已经提供：

- `/scan`；
- `/odom`；
- `odom -> base_link`；
- `base_link -> laser_frame`。

### 5.3 分层启动

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_nav robot_base_bringup.launch \
  start_base:=true \
  start_lidar:=true \
  start_camera:=false
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_nav navigation_stack.launch
```

分层启动适合定位问题，也方便后续全车 bringup 复用已经运行的硬件节点。

### 5.4 切换地图

```bash
roslaunch ucar_nav navigation_stack.launch \
  map_file:=/home/ucar/ucar_ws/src/ucar_nav/maps/map.yaml
```

当前默认地图是 `maps/map.yaml`。不要在没有确认地图方向、原点和现场对应关系时随意
切换其他历史地图。

## 6. 正常停止

在启动该 launch 的终端按：

```text
Ctrl+C
```

随后检查：

```bash
rosnode list
```

若使用一键入口且 `start_robot_base:=true`，以下节点应退出：

```text
/base_driver
/ydlidar_node
/map_server
/amcl
/move_base
```

若使用 `start_robot_base:=false`，外部底盘和雷达节点不归本 launch 管理，不应被
停止。

只有原 launch 终端丢失并确认节点确实属于本次测试时，才使用：

```bash
rosnode kill /move_base
rosnode kill /amcl
rosnode kill /map_server
```

不要批量执行模糊匹配的 `kill` 命令。

## 7. 无运动检查

启动后不要立即发送目标，先执行：

```bash
rosnode list
rostopic hz /scan
rostopic hz /odom
rosparam get /move_base/base_local_planner
rosparam get /move_base/local_costmap/footprint
rosparam get /move_base/local_costmap/inflation_layer
rosparam get /move_base/DWAPlannerROS
```

局部规划器必须是：

```text
dwa_local_planner/DWAPlannerROS
```

保存诊断快照：

```bash
rosrun ucar_nav capture_nav_diagnostics.sh
```

脚本默认输出到：

```text
~/ucar_nav_diagnostics/<日期-时间>/
```

该脚本只读取节点、topic、参数和 TF，不发布速度、不发送导航目标、不修改参数。

## 8. 当前安全参数

### 8.1 footprint

文件：

```text
config/costmap/common.yaml
```

当前首轮安全值：

```yaml
footprint:
  - [0.191, -0.148]
  - [0.191, 0.148]
  - [-0.191, 0.148]
  - [-0.191, -0.148]
```

这是名义车体尺寸外扩 2 cm 的保守起点，必须通过实测确认 `base_link` 位置和车体
突出部件后再决定是否调整。

### 8.2 障碍膨胀

文件：

```text
config/costmap/local.yaml
config/costmap/common.yaml
```

当前值：

```yaml
inflation_radius: 0.35
cost_scaling_factor: 2.5
```

`inflation_radius` 越大，规划器越早认为墙附近代价升高；它不是车体尺寸，不能代替
footprint。

### 8.3 DWA 速度

文件：

```text
config/local_planners/dwa_safe.yaml
```

当前最高速度：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `max_vel_x` | 0.25 m/s | 前进速度 |
| `max_vel_y` | 0.15 m/s | 横移速度 |
| `max_vel_theta` | 0.50 rad/s | 旋转速度 |
| `acc_lim_x` | 0.30 m/s² | 前后加速度 |
| `acc_lim_y` | 0.30 m/s² | 横移加速度 |
| `acc_lim_theta` | 0.80 rad/s² | 旋转加速度 |

当前值用于建立安全基线，不代表最终比赛速度。

### 8.4 DWA 采样与评分

| 参数 | 当前值 | 调整作用 |
|---|---:|---|
| `sim_time` | 2.0 s | 向前预测时长 |
| `vx_samples` | 12 | 前后速度采样 |
| `vy_samples` | 12 | 横移速度采样 |
| `vth_samples` | 24 | 旋转速度采样 |
| `path_distance_bias` | 32.0 | 贴近全局路径 |
| `goal_distance_bias` | 20.0 | 接近局部目标 |
| `occdist_scale` | 0.15 | 远离障碍物 |
| `stop_time_buffer` | 0.40 s | 碰撞前所需停止余量 |

## 9. 调参顺序

每次只修改一个变量：

| 现象 | 优先修改 | 调整方向 |
|---|---|---|
| 轨迹过度贴墙 | `occdist_scale` | 0.15→0.20→0.25 |
| 安全梯度太窄 | `inflation_radius` | 0.35→0.40 |
| 转弯来不及制动 | `max_vel_x` | 0.25→0.20 |
| 旋转过快扫墙 | `max_vel_theta` | 0.50→0.40 |
| 预测距离不足 | `sim_time` | 2.0→2.5 |
| 旋转轨迹选择过粗 | `vth_samples` | 24→32 |
| 横移轨迹选择过粗 | `vy_samples` | 12→16 |

修改 YAML 后必须停止并重新启动原 launch，再确认实际值：

```bash
rosparam get /move_base/DWAPlannerROS
rosparam get /move_base/local_costmap/inflation_layer
```

不要只使用 `rosparam set` 后离开现场；临时验证成功的参数必须写回 YAML。

## 10. 实车测试阶梯

### 第 0 级：静态与无运动检查

- launch 可解析；
- 只加载 DWA；
- `/scan` 和 `/odom` 有稳定频率；
- TF 连续；
- 参数与 YAML 一致。

### 第 1 级：空旷地运动学

- 低速前进、后退；
- 低速左右横移；
- 低速原地旋转；
- 检查实际方向、里程计方向和停止距离。

使用受限测试脚本一次只测试一个轴：

```bash
rosrun ucar_nav controlled_twist_test.py --x 0.10 --duration 2.0
rosrun ucar_nav controlled_twist_test.py --x -0.10 --duration 2.0
rosrun ucar_nav controlled_twist_test.py --y 0.08 --duration 2.0
rosrun ucar_nav controlled_twist_test.py --y -0.08 --duration 2.0
rosrun ucar_nav controlled_twist_test.py --yaw 0.20 --duration 2.0
rosrun ucar_nav controlled_twist_test.py --yaw -0.20 --duration 2.0
```

脚本限制平移绝对值不超过 `0.10 m/s`、旋转绝对值不超过
`0.25 rad/s`、持续时间不超过 2 秒，并在结束或异常时连续发送零速度。它只用于
有人现场看护的底盘运动学测试，不用于比赛导航。

### 第 2 级：单墙

- 平行墙面低速直行；
- 墙边停止；
- 安全位置原地旋转；
- 检查 LaserScan、footprint 和局部 costmap。

### 第 3 级：直角弯

- 固定起点、目标和初始位姿；
- 人工全程准备急停；
- 连续至少三次无碰撞；
- 记录最小墙距、耗时和振荡。

### 第 4 级：当前完整路线

- 固定地图、起点、目标和参数；
- 连续至少三次无碰撞；
- 所有失败轮次也必须记录；
- 完成后才能逐步提高速度。

## 11. rosbag 记录

```bash
mkdir -p ~/ucar_nav_bags

rosbag record -O ~/ucar_nav_bags/dwa-test.bag \
  /tf /tf_static /scan /odom /amcl_pose \
  /move_base/GlobalPlanner/plan \
  /move_base/DWAPlannerROS/local_plan \
  /move_base/local_costmap/costmap \
  /cmd_vel /move_base/status
```

若 topic 名不存在，先运行：

```bash
rostopic list | sort
```

再按实际发布名称调整记录列表。

## 12. 常见问题

### 小车拐弯时外角扫墙

依次检查：

1. footprint 是否与实车一致；
2. 雷达 TF 是否正确；
3. 局部 costmap 中墙面是否稳定；
4. AMCL 是否跳变；
5. inflation 是否真正加载到局部命名空间；
6. 局部路径是否已经穿墙；
7. 底盘是否正确执行 `/cmd_vel`。

只有前六项正确后才调整 DWA 权重和速度。

### 参数文件修改后没有变化

```bash
rosparam get /move_base/DWAPlannerROS
rosparam get /move_base/local_costmap
```

必须停止并重新启动 launch。不要同时启动第二个同名 `move_base`。

### 节点名存在但不能通信

```bash
rosnode ping -c 1 /move_base
rosnode info /move_base
ps -eo pid,ppid,args | grep '[m]ove_base'
```

这可能是 ROS Master 僵尸登记，不代表进程正在工作。

### 雷达或底盘串口被占用

检查是否已经存在 `base_driver` 或 `ydlidar_node`。返回原 launch 终端停止，不要再
启动第二套硬件层。

### 相机冲突

导航默认 `start_camera:=false`。相机应由 QR 或全车 bringup 的唯一节点持有。

## 13. Windows 静态测试

```powershell
cd D:\program_sec\智能车\.worktrees\navigation-safety

python -m unittest discover `
  -s ucar_ws/src/ucar_nav/test `
  -p "test_*.py" -v
```

这些测试不会连接小车，不会发布速度或目标。

## 14. 当前验收状态

截至本 README 初稿：

- Windows 静态配置与受控运动脚本测试：15 项通过；
- 车端部署与构建：已完成，原包备份见 `HANDOFF.md`；
- 无运动参数验证：已通过，未观察到 `/cmd_vel` 输出；
- 雷达 TF 与导航 TF 链：已在静止实车上验证；
- 空旷地六方向运动学：已通过；
- 直角弯三次验收：尚未执行；
- 完整路线三次验收：尚未执行。

只有上述实车项目完成并写入 `HANDOFF.md` 后，才可将本配置称为实车稳定版本。

## 15. Navfn + TEB 精确横移模式（当前实验方案）

当前用于实测的 Profile 是 `navfn_teb_corner`。它使用 Navfn 生成全局路径，
TEB 负责局部轨迹，并由 `teb_lateral_mode_controller` 根据前方路径曲率切换横移能力：

- `STRAIGHT`：直线路段限制横移，`max_vel_y=0.02 m/s`、`acc_lim_y=0.20 m/s²`；
- `CORNER`：前方出现大角度弯道时允许横移，`max_vel_y=0.18 m/s`、
  `acc_lim_y=0.60 m/s²`；
- 出弯后，只有前方路径重新变直、车头与出口方向误差不超过 10°，并连续保持
  0.5 秒，才恢复 `STRAIGHT`。

控制器只修改上述两个 TEB 参数，不修改前进速度、旋转速度、障碍距离或代价地图。
全局路径或 TF 超时后会回退到 `STRAIGHT`。

### 15.1 启动

硬件层已经运行时：

```bash
source /opt/ros/noetic/setup.bash
source ~/ucar_ws/devel/setup.bash

roslaunch ucar_nav navigation_stack.launch \
  navigation_profile:=navfn_teb_corner \
  enable_lateral_mode_controller:=true
```

需要临时关闭曲率控制、只使用 YAML 固定值时：

```bash
roslaunch ucar_nav navigation_stack.launch \
  navigation_profile:=navfn_teb_corner \
  enable_lateral_mode_controller:=false
```

### 15.2 一键初始化 AMCL

小车放在地图固定起点 `(0, 0, 0)` 后执行：

```bash
rosrun ucar_nav initialize_amcl.py
```

若实际起始位姿不同，可临时传参（yaw 单位为弧度）：

```bash
rosrun ucar_nav initialize_amcl.py \
  _x:=0.0 _y:=0.0 _yaw:=0.0 \
  _covariance_x:=0.10 _covariance_y:=0.10 \
  _covariance_yaw:=0.0685
```

脚本会等待 AMCL 订阅 `/initialpose`，成功后只发布一次；10 秒内没有订阅者则返回
退出码 2。它目前不由导航 launch 自动执行，避免小车摆放位置变化时写入错误位姿。

### 15.3 观察模式与实时参数

```bash
rostopic echo /navigation/lateral_mode
rostopic echo /navigation/lateral_mode_diagnostics

rosparam get /move_base/TebLocalPlannerROS/max_vel_y
rosparam get /move_base/TebLocalPlannerROS/acc_lim_y
```

诊断 JSON 包含当前模式、已应用模式、前方转角、出口方向、分析路径长度、最近路径点
索引和错误信息。若 `mode` 已变化但 `applied_mode` 未变化，应检查
`/move_base/TebLocalPlannerROS` dynamic reconfigure 服务是否存在。

### 15.4 调参位置

所有模式切换参数位于：

```text
config/lateral_mode_controller.yaml
```

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `lookahead_distance` | 0.8 m | 向前分析的全局路径长度 |
| `resample_spacing` | 0.10 m | 路径等弧长重采样间隔 |
| `corner_enter_angle_deg` | 45° | 达到该转角进入弯道模式 |
| `corner_exit_angle_deg` | 10° | 低于该转角才允许退出 |
| `heading_exit_tolerance_deg` | 10° | 出弯时车头方向允许误差 |
| `exit_hold_time` | 0.5 s | 满足退出条件的持续时间 |
| `plan_timeout` | 1.0 s | 全局路径失效判定时间 |
| `straight_max_vel_y` | 0.02 m/s | 直线路段最大横移速度 |
| `straight_acc_lim_y` | 0.20 m/s² | 直线路段横移加速度 |
| `corner_max_vel_y` | 0.18 m/s | 弯道路段最大横移速度 |
| `corner_acc_lim_y` | 0.60 m/s² | 弯道路段横移加速度 |

修改 YAML 后应重启导航 launch。第一轮实测只根据诊断单项调整：

- 太晚进入弯道：增大 `lookahead_distance`，例如 `0.8 -> 1.0`；
- 直线误判为弯道：提高 `corner_enter_angle_deg`，例如 `45 -> 55`；
- 出弯后长期保持横移：适当增大 `corner_exit_angle_deg` 或
  `heading_exit_tolerance_deg`，一次只改一个；
- 弯道仍缺少调整能力：先确认确实进入 `CORNER`，再提高
  `corner_max_vel_y`，不要同时修改多个参数。

TEB 的静态默认值在：

```text
config/local_planners/teb_corner_safe.yaml
```

该文件默认使用直线模式值，并保持优化迭代为 `no_inner_iterations=2`、
`no_outer_iterations=1`，防止再次出现约 1 秒一次的低频控制。

### 15.5 实车记录

```bash
rosbag record -O ~/ucar_nav_bags/navfn-teb-lateral.bag \
  /tf /tf_static /scan /odom /amcl_pose \
  /move_base/NavfnROS/plan \
  /move_base/TebLocalPlannerROS/local_plan \
  /move_base/local_costmap/costmap \
  /navigation/lateral_mode \
  /navigation/lateral_mode_diagnostics \
  /cmd_vel /move_base/status
```

发车前先确认 AMCL 初始位姿，再观察一轮无运动状态。实测时记录直线、入弯、弯中、
出弯四阶段的模式；出现碰撞趋势立即人工停止。
