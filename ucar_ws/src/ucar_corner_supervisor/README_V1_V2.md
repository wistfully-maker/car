# U-CAR 拐弯监督器 V1/V2 完整使用说明

对应小车目录：

```text
/home/ucar/ucar_ws/src/ucar_corner_supervisor
```

当前连接命令：

```bash
ssh ucar@10.234.15.42
```

本包只负责接管 `move_base` 的速度输出和处理大角度拐弯。地图、AMCL、TEB、底盘和
雷达仍由 `ucar_nav`、`ucar_controller`、`ydlidar` 提供。

## 1. 两个版本

| 版本 | 完整启动文件 | 监督器节点 | 目的 |
|---|---|---|---|
| 第一版 V1 | `navigation_with_corner_supervisor_legacy_v1.launch` | `/ucar_corner_supervisor_legacy_v1` | 冻结提交 `19e0ce7`，复现第一次较丝滑的第一弯 |
| 新版 V2 | `navigation_with_corner_supervisor_v2.launch` | `/ucar_corner_supervisor` | 多拐点队列、稳定线段拟合、旋转扫掠保护和紧弯接管 |

两个版本绝对不能同时运行。V1 是对照基线，不要直接修改
`corner_supervisor_legacy_v1.yaml`；实验参数应复制成新文件。

V2 当前策略：

- 小于 `70°` 的普通弯交给 TEB；
- 大于等于 `70°` 的紧弯由监督器在约 `0.20 m` 前接管；
- 原地旋转前检查实时局部代价地图中的矩形车身扫掠空间；
- 地图过期、方向拟合不可信或旋转空间不安全时停车。

## 2. 节点结构和唯一所有者

```text
robot_base_bringup.launch
  ├─ /base_driver             /dev/ttyS0 唯一所有者
  ├─ /ydlidar_node            /dev/ttyS4 唯一所有者
  └─ /base_link_to_laser

navigation_with_corner_supervisor_*.launch
  ├─ /map_server
  ├─ /amcl
  ├─ /move_base               输出重映射到 /move_base/cmd_vel_raw
  └─ 一个监督器               /cmd_vel 唯一发布者
```

正确速度链：

```text
/move_base -> /move_base/cmd_vel_raw -> 监督器 -> /cmd_vel -> /base_driver
```

禁止同时启动原始导航 launch、第二套 `move_base`、`line_follower` 或
`lateral_mode_controller`。ROS 同名节点被再次启动时，旧节点会退出；底盘和雷达还
会发生串口锁冲突。

## 3. 启动前检查

```bash
ssh ucar@10.234.15.42
source /home/ucar/ucar_ws/devel/setup.bash

rosnode list | sort
ps -eo pid,args | grep -E \
  '[r]oslaunch|[m]ove_base|[a]mcl|[m]ap_server|[b]ase_driver|[y]dlidar|[l]ine_follower'
rostopic info /cmd_vel
rostopic info /move_base/cmd_vel_raw
```

已有正确节点时不要重复启动。切换版本前优先回到原启动终端按 `Ctrl-C`。找不到原
终端时按节点名关闭：

```bash
rosnode kill /ucar_corner_supervisor 2>/dev/null
rosnode kill /ucar_corner_supervisor_legacy_v1 2>/dev/null
rosnode kill /move_base 2>/dev/null
rosnode kill /amcl 2>/dev/null
rosnode kill /map_server 2>/dev/null
rosnode kill /ydlidar_node 2>/dev/null
rosnode kill /base_link_to_laser 2>/dev/null
rosnode kill /base_driver 2>/dev/null
sleep 2
```

再次检查 `/cmd_vel` 和进程。仅当 `rosnode list` 中存在无法联系的旧注册时执行：

```bash
rosnode cleanup
```

不要使用 `rosnode kill -a`。

## 4. 推荐启动方式（不需要 RViz）

保留两个 SSH 终端，便于看到错误和使用 `Ctrl-C`。

### 4.1 终端 A：底盘和雷达

```bash
ssh ucar@10.234.15.42
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch ucar_nav robot_base_bringup.launch start_camera:=false
```

该命令不启动相机。二维码阶段的相机只能由一个模块统一启动。

验证基础输入：

```bash
rostopic hz /scan
rostopic hz /odom
rosrun tf tf_echo odom base_link
```

期望 `/scan` 约 `10 Hz`，`/odom` 连续更新。

### 4.2 终端 B：只能选择一个版本

第一版 V1：

```bash
ssh ucar@10.234.15.42
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch ucar_corner_supervisor \
  navigation_with_corner_supervisor_legacy_v1.launch
```

新版 V2：

```bash
ssh ucar@10.234.15.42
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch ucar_corner_supervisor \
  navigation_with_corner_supervisor_v2.launch
```

旧名字 `navigation_with_corner_supervisor.launch` 等价于 V2。人工测试建议始终使用带
版本号的名字。

启动后必须验证：

```bash
rosnode list | sort
rostopic info /cmd_vel
rostopic info /move_base/cmd_vel_raw
```

正确结果：

- `/cmd_vel` 只有当前监督器发布，`/base_driver` 订阅；
- `/move_base/cmd_vel_raw` 只有 `/move_base` 发布，当前监督器订阅；
- 不存在第二个 `/move_base` 或 `/line_follower`。

## 5. AMCL 初始化

以下情况必须重新初始化：

- 开机后的第一次导航；
- 人工抬起、推动或转动小车；
- 换电池后位置变化；
- 撞墙、严重打滑或 AMCL 跳变；
- 地图或起点变化；
- 地图中的位置、朝向与现实不一致。

先把车放在固定起点，车头与地图起始方向一致。当前测试起点：

```text
x=0.0, y=0.0, yaw=0°
```

发布 36 元素协方差的初始位姿：

```bash
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
'{header: {frame_id: map},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [
      0.04,0,0,0,0,0,
      0,0.04,0,0,0,0,
      0,0,0,0,0,0,
      0,0,0,0,0,0,
      0,0,0,0,0,0,
      0,0,0,0,0,0.03
    ]
  }}'
```

等待并检查：

```bash
sleep 3
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map base_link
```

位置/朝向明显错误或协方差持续很大时不要发目标。

## 6. 清理代价地图并发布二维码航点

航点文件：

```text
/home/ucar/waypoints.xml
```

当前二维码启动航点：

```text
x=-1.40219
y=-0.627908
orientation.z=0.999639
orientation.w=-0.0268713
```

清理上一轮动态障碍：

```bash
rosservice call /move_base/clear_costmaps '{}'
```

发布目标：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'{header: {frame_id: map},
  pose: {
    position: {x: -1.40219, y: -0.627908, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.999639, w: -0.0268713}
  }}'
```

这等价于 RViz 的 “2D Nav Goal”，无需 RViz。

## 7. 运行监控

V1：

```bash
rostopic echo /ucar_corner_supervisor_legacy_v1/state
rostopic echo /ucar_corner_supervisor_legacy_v1/diagnostic
```

V2：

```bash
rostopic echo /ucar_corner_supervisor/state
rostopic echo /ucar_corner_supervisor/diagnostic
```

通用：

```bash
rostopic echo /move_base/status
rostopic echo /cmd_vel
rostopic echo /move_base/cmd_vel_raw
rostopic hz /scan
rostopic hz /odom
rostopic hz /move_base/local_costmap/costmap_updates
```

状态：

| 状态 | 含义 |
|---|---|
| `IDLE` | 没有有效导航目标，输出零速度 |
| `FOLLOWING` | 转发 TEB 速度并限制横移 |
| `TURNING` | 监督器接管，x/y 为零并原地旋转 |
| `EXIT_ALIGN` | 进入角度容差，停车确认稳定 |
| `BLOCKED` | V2 判断拟合或旋转空间不安全 |
| `ERROR` | TF、速度输入、超时或话题所有权异常 |

V2 诊断字段：

- `corner_count`：剩余拐点数；
- `corner_distance`：沿全局路径到紧弯的距离；
- `corner_angle_deg`：拟合转角；
- `entry_heading_deg`、`exit_heading_deg`：稳定入弯/出弯方向；
- `corner_confidence`：方向拟合是否可信；
- `costmap_fresh`：局部代价地图是否新鲜；
- `sweep_needed`、`sweep_safe`：是否检查及旋转是否安全；
- `sweep_reason`、`blocking_cell`：阻塞原因和单元；
- `raw_fresh`、`path_fresh`、`tf_valid`：输入健康状态。

## 8. 取消、急停和关闭

软件取消目标：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
```

随后确认：

```bash
rostopic echo -n 1 /cmd_vel
rostopic echo -n 1 /move_base/status
```

速度必须为零。实车测试仍需准备物理急停。

正常关闭：

1. 终端 B 按 `Ctrl-C`，关闭导航和监督器；
2. 确认没有非零速度；
3. 终端 A 按 `Ctrl-C`，关闭底盘和雷达；
4. 再关闭小车电源。

最后检查：

```bash
rosnode list | sort
rostopic info /cmd_vel
ps -eo pid,args | grep -E \
  '[m]ove_base|[a]mcl|[m]ap_server|[b]ase_driver|[y]dlidar'
```

## 9. V1 参数

文件：

```text
/home/ucar/ucar_ws/src/ucar_corner_supervisor/config/corner_supervisor_legacy_v1.yaml
```

| 参数 | 冻结值 | 含义 |
|---|---:|---|
| `controller_rate` | 20 Hz | 监督循环频率 |
| `raw_command_timeout` | 0.5 s | 收不到 TEB 速度多久后停车 |
| `ownership_check_interval` | 0.5 s | 检查 `/cmd_vel` 冲突周期 |
| `path_timeout` | 0 | 0 表示目标有效时保留最后全局路径 |
| `path_search_distance` | 1.2 m | 只在车前该距离寻找第一个拐点 |
| `min_corner_angle_deg` | 45° | 最小接管转角 |
| `path_resample_spacing` | 0.05 m | 路径等距采样间距 |
| `direction_window` | 0.20 m | 估计拐点前后方向的固定窗口 |
| `corner_trigger_distance` | 0.25 m | 开始停车转向的距离 |
| `corner_release_distance` | 0.45 m | 防止同一拐点重复触发 |
| `following_max_lateral` | 0.02 m/s | FOLLOWING 最大横移 |
| `turn_max_angular` | 0.35 rad/s | 转向最大角速度 |
| `turn_min_angular` | 0.18 rad/s | 克服底盘死区的最小角速度 |
| `turn_kp` | 0.9 | 航向误差比例系数 |
| `heading_tolerance_deg` | 8° | 对正角度容差 |
| `heading_hold_time` | 0.30 s | 容差内保持时间 |
| `turn_timeout` | 8 s | 单次转向超时 |

V1 没有 V2 的矩形车身旋转保护，也不能可靠维护多个紧邻拐点。它用于复现第一版行为，
不代表最终比赛版本。

## 10. V2 参数

文件：

```text
/home/ucar/ucar_ws/src/ucar_corner_supervisor/config/corner_supervisor.yaml
```

共用参数含义与 V1 相同。额外参数：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `min_corner_angle_deg` | 70° | 只接管紧弯；降低会接管更多普通弯 |
| `path_simplify_tolerance` | 0.08 m | RDP 路径简化容差 |
| `min_stable_segment_length` | 0.08 m | 方向拟合最短线段 |
| `max_fit_residual` | 0.08 m | 最大拟合残差 |
| `same_turn_merge_distance` | 0.05 m | 同向临近拐点合并距离 |
| `path_resample_spacing` | 0.05 m | 等弧长采样间距 |
| `corner_release_margin` | 0.10 m | 超过拐点多少后跳过 |
| `completed_corner_match_distance` | 0.25 m | 重规划匹配完成拐点的位置容差 |
| `completed_corner_match_heading_deg` | 20° | 重规划匹配出弯方向容差 |
| `costmap_timeout` | 1.5 s | 局部地图最大年龄 |
| `lethal_cost_threshold` | 100 | 视为致命障碍的栅格值 |
| `sweep_angle_step_deg` | 3° | 旋转扫掠采样角度 |
| `sweep_check_rate` | 5 Hz | 紧弯前/转向中检查频率 |
| `footprint` | 0.342×0.256 m | 必须覆盖车壳突出物的矩形轮廓 |
| `corner_trigger_distance` | 0.20 m | 紧弯接管距离 |
| `following_max_lateral` | 0.05 m/s | 转发 TEB 时允许的小幅横移 |

调参原则：

1. 每轮只改一类参数；
2. 记录 Git 提交、版本、起点、目标、完成时间和现场现象；
3. `rosparam set` 只用于临时验证，成功后回写 YAML 并重启；
4. V1 连续三轮建立基线前不要修改冻结文件；
5. V2 的 `BLOCKED` 不能靠提高速度或降低障碍阈值绕过。

## 11. TEB、代价地图和 move_base 参数

当前 profile：

```text
/home/ucar/ucar_ws/src/ucar_nav/config/profiles/navfn_teb_corner.yaml
```

加载文件：

```text
config/global_planners/navfn.yaml
config/local_planners/teb_corner_safe.yaml
config/costmap/common_teb.yaml
config/costmap/global_teb.yaml
config/costmap/local_teb.yaml
config/move_base_teb.yaml
```

TEB 常用参数：

- `max_vel_x`、`max_vel_x_backwards`、`max_vel_y`、`max_vel_theta`：各方向速度；
- `acc_lim_x/y/theta`：加速度；
- `min_obstacle_dist`、`inflation_dist`：轨迹障碍距离；
- `max_global_plan_lookahead_dist`：局部规划看到的全局路径长度；
- `global_plan_viapoint_sep`：全局路径参考点间距；
- `weight_kinematics_nh`：非完整约束；全向底盘过高会限制横移；
- `weight_kinematics_forward_drive`：向前行驶偏好；
- `weight_obstacle`、`weight_viapoint`：避障和贴路径权重；
- `feasibility_check_no_poses`：执行 footprint 复核的轨迹位姿数；
- `no_inner_iterations`、`no_outer_iterations`：优化迭代次数；
- `dt_ref`：轨迹时间分辨率。

costmap 常用参数：

- `footprint`：真实车身轮廓；
- `inflation_radius`、`cost_scaling_factor`：障碍膨胀；
- `update_frequency`、`publish_frequency`：更新/发布频率；
- `obstacle_range`、`raytrace_range`：雷达标记/清除范围；
- `transform_tolerance`：允许 TF 延迟。

`trajectory is not feasible` 表示 TEB 轨迹通过 footprint/代价地图复核失败，可能原因：

- AMCL 起始定位错误；
- 雷达障碍进入车身附近；
- footprint 或雷达 TF 错误；
- 走廊对安全距离确实太窄；
- 横移/运动学权重不允许替代轨迹；
- 全局路径贴墙；
- 多套导航节点令参数或话题来源混乱。

## 12. 日志与故障排查

```bash
ls -lt ~/.ros/log/latest/
grep -R -E \
  'trajectory is not feasible|oscillat|Control loop missed|new node registered|conflicting /cmd_vel|ERROR|FATAL' \
  ~/.ros/log/latest/
```

### `/cmd_vel` 有两个发布者

立即取消目标并关闭两套导航。常见来源是未重映射的原始 `move_base`、
`line_follower` 或另一套 launch。

### `new node registered with same name`

另一套 launch 顶掉了当前节点。该轮数据无效，全部关闭后重新启动。

### 雷达无法锁定 `/dev/ttyS4`

已有雷达进程或串口锁。先确认没有 `ydlidar_node`；不要删除正在使用的锁文件。

### AMCL/TF extrapolation

检查 `/odom`、`/scan` 和 `map -> odom -> base_link`。基础节点重启后重新初始化
AMCL。

### TEB 持续不可行

先不要移动小车，记录失败现场：

```bash
rostopic echo -n 1 /amcl_pose
rostopic echo -n 1 /move_base/NavfnROS/plan
rostopic echo -n 1 /move_base/local_costmap/costmap
rosparam dump /tmp/navigation_params.yaml /move_base
```

结合现场位置判断定位、全局路径、障碍还是运动学问题。

## 13. V1/V2 验收顺序

V1：

1. 固定起点和朝向；
2. 每轮初始化 AMCL；
3. 连续三轮只观察第一弯；
4. 记录流畅程度、横摆、卡住和人工急停；
5. 第一弯三轮稳定后再进入第二弯。

V2：

1. 与 V1 使用同一起点、地图和目标；
2. 普通弯应保持 `FOLLOWING`；
3. 紧弯前才进入 `TURNING`；
4. `sweep_safe:false` 时必须停车；
5. 完整路线连续三轮无碰撞后才作为默认比赛版。

不能使用不同起点比较两版；每轮都要先确认只有一个导航入口。

## 14. 构建与部署

Windows 同步：

```powershell
scp -r `
  D:\program_sec\智能车\.worktrees\corner-supervisor\ucar_ws\src\ucar_corner_supervisor `
  ucar@10.234.15.42:/home/ucar/ucar_ws/src/
```

小车构建：

```bash
cd /home/ucar/ucar_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg ucar_corner_supervisor
source /home/ucar/ucar_ws/devel/setup.bash
```

部署前关闭正在运行的本包节点，部署后重启才能加载新代码和 YAML。
