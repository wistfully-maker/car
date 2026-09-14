# ucar_waypoint_nav 使用与调试手册

## 1. 功能与控制链

本包使用3个稀疏中间航点约束正确走廊。每一段仍由GlobalPlanner或Navfn根据静态
地图在线生成全局路径，TEB在线优化局部轨迹。

中间航点必须连续穿越：管理器在小车进入切换半径且仍保持前进速度时直接发送下一个
action目标，不先取消旧目标，也不发布零速度。只有最终二维码观察点停车。

正常控制链：

```text
waypoint_route_manager
  -> /move_base action
  -> GlobalPlanner或Navfn
  -> TebLocalPlannerROS
  -> /cmd_vel
  -> base_driver
```

`waypoint_route_manager`不发布`/cmd_vel`。运行本包时必须关闭DWA、拐点监督器、
TEB横移模式转发器和其他导航栈，保证只有`/move_base`发布正常速度。

## 2. 当前航点

文件：`config/pickup_waypoints.yaml`

| 名称 | 类型 | x | y | yaw |
|---|---|---:|---:|---:|
| pass_1 | 穿越 | 1.10 | -0.30 | 0.927 |
| pass_2 | 穿越 | 2.55 | -0.25 | 1.326 |
| pass_3 | 穿越 | 0.05 | -0.70 | 3.017 |
| pickup_observation | 终点 | -1.40219 | -0.627908 | -3.0878 |

最终车头面向距离P点最近的物品领取区墙面。到达后二维码模块可逆时针旋转。

## 3. 编译

```bash
cd /home/ucar/ucar_ws
catkin_make --pkg ucar_waypoint_nav
source /home/ucar/ucar_ws/devel/setup.bash
```

每个新终端都需要：

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
```

## 4. 启动前检查

检查是否已有导航或速度控制节点：

```bash
rosnode list | grep -E 'move_base|amcl|map_server|corner|lateral|waypoint'
rostopic info /cmd_vel
```

`/cmd_vel`只能有一个正常导航发布者。`/base_driver`应当是订阅者。

检查基础硬件：

```bash
rostopic hz /scan
rostopic hz /odom
rosrun tf tf_echo odom base_link
```

本包launch不启动底盘和雷达。必须先由全车bringup或团队现有launch启动：

- `base_driver`
- 雷达节点和`laser_frame` TF
- `odom -> base_link`

## 5. 一键启动

默认使用GlobalPlanner：

```bash
roslaunch ucar_waypoint_nav waypoint_teb_navigation.launch \
  global_planner:=global_planner
```

这个launch启动：

- `map_server`
- `amcl`
- `move_base`
- `global_planner/GlobalPlanner`
- `teb_local_planner/TebLocalPlannerROS`
- `ucar_waypoint_route_manager`

它不自动发车。AMCL初始化并检查正确后，才调用启动服务。

使用Navfn进行同条件对比：

```bash
roslaunch ucar_waypoint_nav waypoint_teb_navigation.launch \
  global_planner:=navfn
```

两个全局规划器不能同时运行。

## 6. 分步启动

如果地图、AMCL和move_base已经由团队启动，只启动管理器：

```bash
roslaunch ucar_waypoint_nav waypoint_teb_navigation.launch \
  start_navigation_stack:=false
```

此模式要求外部move_base已经使用TEB，并且地图文件必须与
`pickup_waypoints.yaml`中的SHA-256匹配。

若只想检查launch展开的文件：

```bash
roslaunch --files ucar_waypoint_nav waypoint_teb_navigation.launch
```

## 7. AMCL初始化

比赛P点使用：

```text
x=0.0
y=0.0
yaw=0.0
```

推荐使用已有脚本：

```bash
rosrun ucar_nav initialize_amcl.py \
  --x 0.0 --y 0.0 --yaw 0.0 \
  --cov-x 0.04 --cov-y 0.04 --cov-yaw 0.03
```

确认：

```bash
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map base_link
```

管理器默认只允许距P点不超过`0.20 m`、航向误差不超过`0.25 rad`时启动。

## 8. 手动开始与监控

启动整条路线：

```bash
rosservice call /ucar_waypoint_nav/start
```

监控：

```bash
rostopic echo /ucar_waypoint_nav/state
rostopic echo /ucar_waypoint_nav/diagnostic
rostopic echo /move_base/status
rostopic echo /cmd_vel
```

正常状态：

```text
IDLE -> NAVIGATING -> SWITCHING -> NAVIGATING -> ARRIVED
```

`SWITCHING`应很短。中间航点处实车不能停下。

## 9. 编排器启动

```bash
rostopic pub -1 /task/pickup_navigation_goal std_msgs/String \
  'data: "{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"goal_id\":\"pickup-test-001\"}"'
```

到达或失败结果：

```bash
rostopic echo /task/pickup_arrived
```

成功结果保留原`task_id`和`goal_id`，`status`为`arrived`。

## 10. 取消、急停与关闭

软件取消：

```bash
rosservice call /ucar_waypoint_nav/cancel
```

也可以直接取消move_base：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
```

确认零速度：

```bash
rostopic echo -n 1 /cmd_vel
```

关闭launch：

```text
在运行roslaunch的终端按 Ctrl+C
```

如果终端丢失，按顺序关闭：

```bash
rosservice call /ucar_waypoint_nav/cancel || true
rosnode kill /ucar_waypoint_route_manager
rosnode kill /move_base /amcl /map_server
```

运行结束后应手动关闭本次测试启动的节点；不要关闭团队仍在使用的底盘或雷达节点。

## 11. 重新生成航点

只在地图、起点或终点改变后运行：

```bash
rosrun ucar_waypoint_nav derive_sparse_waypoints.py \
  --map /home/ucar/ucar_ws/src/ucar_nav/maps/map.yaml \
  --start 0 0 0 \
  --goal -1.40219 -0.627908 -3.0878 \
  --output /home/ucar/ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.yaml \
  --preview /home/ucar/ucar_ws/src/ucar_waypoint_nav/config/pickup_waypoints.ppm
```

参数：

- `--inflation-radius`：离线拓扑搜索膨胀，默认`0.15 m`；
- `--maximum-count`：最多中间点数，默认3。

PGM预览中：

- 绿色：P点；
- 红色：二维码终点；
- 蓝色：仅用于选择航点的离线参考通路；
- 橙色：中间航点。

蓝色通路不参与比赛运行；比赛每段路径仍由在线算法生成。

## 12. 参数位置

### 12.1 航点和穿越

文件：`config/pickup_waypoints.yaml`

- `switch_radius`：提前切换半径，当前`0.35 m`；
- `heading_tolerance`：进入下一走廊允许的航向误差；
- `minimum_pass_speed`：穿越最低速度，当前`0.08 m/s`；
- `position_tolerance/yaw_tolerance`：仅最终点使用。

中间点出现减速或停车时，首先把`switch_radius`每次增加`0.05 m`。不能同时修改
TEB速度、航点坐标和切换半径。

### 12.2 TEB

文件：`config/teb.yaml`

- `max_vel_x`：前进上限；
- `max_vel_x_backwards`：倒退上限；
- `max_vel_y`：麦克纳姆横移上限；
- `max_vel_theta`：角速度上限；
- `acc_lim_*`：加速度；
- `no_inner_iterations/no_outer_iterations`：优化量；
- `global_plan_viapoint_sep`：全局路径途经约束间距；
- `max_global_plan_lookahead_dist`：局部前视距离；
- `weight_kinematics_forward_drive`：前进偏好；
- `weight_obstacle`：障碍代价；
- `free_goal_vel`：中间航点必须保持`true`。

### 12.3 move_base频率

文件：`config/controller.yaml`

- `controller_frequency: 10.0`
- `planner_frequency: 2.0`

如果日志持续报告控制周期错过10 Hz，降低到实测CPU能够长期维持的频率，不要只提高
配置数字。

### 12.4 管理器

文件：`config/manager.yaml`

- `route_timeout: 300.0`
- `pass_speed_grace: 0.50`
- `start_xy_tolerance: 0.20`
- `start_yaw_tolerance: 0.25`

## 13. 日志采集

开始测试前：

```bash
rosrun ucar_waypoint_nav capture_navigation_run.sh
```

输出默认位于：

```text
/home/ucar/navigation_runs/YYYYMMDD-HHMMSS/
```

停止bag：

```bash
kill $(cat /home/ucar/navigation_runs/本轮目录/rosbag.pid)
```

## 14. 常见问题

### 14.1 中间航点停车

检查诊断中的：

- `distance`
- `speed`
- `action_state`
- `decision_error`

若move_base先到`SUCCEEDED`，说明切换太晚；增加`switch_radius`。若进入切换半径但
航向不满足，检查航点yaw是否确实指向下一走廊。

### 14.2 `trajectory is not feasible`

按顺序检查：

1. `/scan`是否看到实际墙面；
2. `map -> base_link -> laser_frame`是否正确；
3. 局部costmap中墙体位置是否正确；
4. footprint是否为`0.342 m × 0.256 m`矩形；
5. 全局路径是否穿过膨胀墙体；
6. TEB实际控制频率是否跟得上。

不要先降低障碍权重或缩小footprint。

### 14.3 撞墙

如果TEB已经发布零速度但车仍运动，检查：

```bash
rostopic info /cmd_vel
rosparam get /base_driver/cmd_timeout
```

如果costmap中没有墙，问题在雷达、TF或障碍层，不是TEB平滑参数。

### 14.4 GlobalPlanner无路径

切换Navfn作同条件诊断：

```bash
roslaunch ucar_waypoint_nav waypoint_teb_navigation.launch \
  global_planner:=navfn
```

Navfn有路径而GlobalPlanner没有时，比较全局规划参数；两者都没有时，检查航点是否落
在致命代价或未知区域。
