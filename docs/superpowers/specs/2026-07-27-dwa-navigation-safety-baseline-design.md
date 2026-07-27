# U-CAR DWA 导航安全基线设计

## 1. 本对话范围

本对话只完成一套稳定、可重复、不会在拐弯处扫墙的 DWA 导航方案。

完成 DWA 实车验收后，输出导航 README 和交接文件。交接文件说明当前启动方式、
有效参数、测试结果、已知限制，以及后续接入 `task_orchestrator` 所需接口。

以下工作不在本对话实施：

- 不编写任务导航适配器；
- 不修改 `task_orchestrator`；
- 不联调语音、QR、LLM、TTS 与导航全链路；
- 不安装或调试 TEB、MPPI、MPC；
- 不以追求比赛极限速度为本阶段目标。

## 2. 当前系统

当前 `ucar_nav` 使用：

- ROS 1 Noetic；
- `move_base`；
- `GlobalPlanner`；
- `DWAPlannerROS`；
- AMCL；
- 2D 激光雷达；
- 麦克纳姆全向底盘。

底盘驱动支持：

- `/cmd_vel.linear.x`：前后；
- `/cmd_vel.linear.y`：横移；
- `/cmd_vel.angular.z`：旋转。

车上目前未安装 `teb_local_planner`。仓库虽然存在 TEB 参数文件，但当前没有 TEB
插件可加载，实际局部规划器仍为 DWA。

实车已知问题是：小车刚进入拐弯时，矩形车身外角扫到墙。其余直线、动态障碍、
定位漂移和完整路线问题尚未获得足够测试证据。

## 3. 当前配置风险

已发现但仍需逐项验证的风险：

1. 文件参数与 ROS 参数服务器残留值不同，无法追溯某次测试实际使用的参数；
2. `ucar_navigation.launch` 同时加载 DWA 和未使用的 TEB 参数；
3. 局部 inflation 覆写位置疑似不在正确命名空间，实际值仍为 `0.18 m`；
4. 车体矩形外角到中心约为 `0.214 m`，当前安全梯度可能不足；
5. DWA 障碍物评分权重偏低；
6. 全向底盘的 `vy` 范围和 `vy_samples` 在文件与残留运行参数中不一致；
7. `transform_tolerance: 10` 秒可能掩盖 TF 延迟；
8. 总 launch 同时启动底盘、雷达、相机和导航，容易与其他 launch 冲突；
9. 当前没有自动保存每轮测试的参数快照、日志和运动数据。

这些项目是调查清单，不应在没有实测证据时一次全部修改。

## 4. 局部规划器使用原则

仓库可以同时保存多套局部规划器配置，例如：

```text
config/local_planners/dwa_safe.yaml
config/local_planners/teb_safe.yaml
```

但一个 `move_base` 实例同一时刻只允许通过 `base_local_planner` 激活一个局部规划器。

本阶段固定为：

```yaml
base_local_planner: "dwa_local_planner/DWAPlannerROS"
```

不得同时启动两个能够直接向同一 `/cmd_vel` 发布速度的导航控制器。若以后需要在线
对照多个控制器，必须增加明确的速度仲裁和控制权切换机制；当前阶段不实现。

## 5. 包结构整理

目标结构：

```text
ucar_nav/
  launch/
    robot_base_bringup.launch
    navigation_stack.launch
    ucar_navigation.launch
  config/
    amcl/
    costmap/
    local_planners/
      dwa_safe.yaml
    move_base.yaml
  maps/
  scripts/
    capture_nav_diagnostics.sh
  test/
  README.md
```

### 5.1 硬件启动层

`robot_base_bringup.launch` 只负责：

- `base_driver`；
- `ydlidar_node`；
- 雷达静态 TF；
- 可选相机。

相机使用参数决定是否启动。若其他模块已经持有摄像头，本层不得重复启动。

### 5.2 导航算法层

`navigation_stack.launch` 只负责：

- `map_server`；
- AMCL；
- `move_base`；
- DWA 配置。

### 5.3 兼容入口

`ucar_navigation.launch` 作为兼容入口，通过参数决定是否同时包含硬件层和导航层。
默认行为必须在 README 中明确，不能依赖使用者猜测。

## 6. 调参前的根因检查

### 6.1 雷达 TF

确认：

- `base_link -> laser_frame` 的平移与实车安装位置一致；
- 雷达正方向与车头方向一致；
- 车辆原地旋转时，静止墙面在局部代价地图中不产生明显圆周漂移；
- 系统中不存在两个节点同时发布相同 TF。

TF 错误会使规划器认为墙在错误位置，单纯扩大 inflation 不能从根本上修复。

### 6.2 车体轮廓

当前名义 footprint：

```yaml
footprint:
  - [ 0.171, -0.128 ]
  - [ 0.171,  0.128 ]
  - [-0.171,  0.128 ]
  - [-0.171, -0.128 ]
```

需要实测：

- 车体最前、最后、最左、最右；
- 保险杠、线缆、摄像头支架等突出部分；
- `base_link` 是否位于上述矩形中心。

正式参数使用“实测外轮廓 + 小幅安全余量”，不能用圆形 `robot_radius` 代替矩形
footprint。

### 6.3 定位与里程计

确认：

- 前进、横移、旋转时 `/odom` 方向正确；
- AMCL 位姿在静止时稳定；
- 转弯时 `map -> odom` 不发生明显跳变；
- 地图分辨率、原点和现场墙体一致。

### 6.4 制动距离

分别测量低速前进、横移和旋转后的停止响应。DWA 的预测轨迹、控制频率和
`stop_time_buffer` 必须覆盖实车实际制动距离。

## 7. DWA 安全基线

### 7.1 代价地图

必须做到：

- footprint 只维护一份；
- 障碍物源只维护一份；
- 全局和局部 costmap 加载到各自正确命名空间；
- 局部 inflation 参数实际出现在
  `/move_base/local_costmap/inflation_layer/*`；
- inflation radius 大于车体外角半径，并形成额外安全梯度；
- 激光雷达同时启用 marking 和 clearing；
- 将过大的 TF 容忍时间收紧到基于实测延迟的数值；
- 优先验证局部滚动窗口使用 `odom`，降低 AMCL 修正对短期避障的扰动。

第一轮使用保守参数，后续只能按单变量逐项缩减安全距离。

### 7.2 速度与加速度

明确配置：

- `max_vel_x/min_vel_x`；
- `max_vel_y/min_vel_y`；
- `max_vel_theta/min_vel_theta`；
- `acc_lim_x/acc_lim_y/acc_lim_theta`；
- `vx_samples/vy_samples/vth_samples`；
- `sim_time`；
- `stop_time_buffer`。

第一轮限制速度与加速度。确认横移方向、横移里程计和横移制动均正确后，再启用有效的
`vy` 采样范围。

### 7.3 轨迹评分

主要参数：

- `path_distance_bias`：贴近全局路径；
- `goal_distance_bias`：朝向局部目标；
- `occdist_scale`：远离障碍物；
- `forward_point_distance`：前向评分点；
- `scaling_speed/max_scaling_factor`：高速时放大车体碰撞轮廓。

调试顺序：

1. 先确保所有碰撞轨迹被淘汰；
2. 再确保转弯时主动留出墙距；
3. 再减少抖动和停顿；
4. 最后提高完成速度。

不得同时修改速度、膨胀半径和三个评分权重，否则无法判断哪个变化产生效果。

## 8. 测试阶梯

### 8.1 第 0 级：静态验证

- launch XML 可解析；
- YAML 可解析；
- 未加载 TEB 参数；
- DWA 是唯一有效局部规划器；
- 运行时参数与 YAML 一致；
- 没有同名硬件或导航节点；
- `/scan`、`/odom` 和 TF 可用。

### 8.2 第 1 级：空旷地底盘验证

- 低速前进、后退；
- 低速左右横移；
- 低速原地旋转；
- 检查方向、里程计和停止响应。

### 8.3 第 2 级：单墙验证

- 以固定距离平行墙面直行；
- 在墙边低速停止；
- 在安全位置原地旋转；
- 观察局部 costmap 中墙面和 footprint 的关系。

### 8.4 第 3 级：直角弯验证

- 固定起点、目标点和初始位姿；
- 从最低安全速度开始；
- 记录车身外角最小墙距；
- 连续至少三次无碰撞；
- 不出现持续振荡、贴墙或人工接管。

### 8.5 第 4 级：当前完整路线

- 使用固定地图、起点和目标点；
- 连续至少三次无碰撞到达；
- 到达状态正确；
- 每次记录耗时、最小墙距和异常；
- 完成后才允许逐步提高速度。

任何测试出现碰撞趋势时立即停止，并保留该轮参数、日志和 rosbag。

## 9. 测试记录

每次实车测试保存：

- Git commit；
- 地图文件和校验值；
- 完整 ROS 参数快照；
- 起点、目标点和初始位姿；
- 最大速度与加速度；
- footprint；
- inflation 参数；
- 是否成功；
- 最小墙距；
- 完成时间；
- 是否振荡或人工停止；
- rosbag 和日志路径。

禁止只使用 `rosparam set` 调参而不回写 YAML。临时参数验证有效后，必须写回配置并
重启复测。

## 10. README 与交接文件

### 10.1 README

README 必须包含：

- 部署与编译；
- 分层启动和一键启动；
- 启动前冲突检查；
- 正常停止和异常残留处理；
- 参数文件位置和参数含义；
- 第 0～4 级测试命令；
- rosbag、参数快照和日志保存；
- 常见故障排查；
- 当前验收结果。

### 10.2 交接文件

交接文件必须包含：

- 当前 DWA commit；
- 小车部署路径；
- 当前有效 launch；
- 当前有效参数；
- 地图与目标点信息；
- 实车测试记录；
- 已知限制；
- 后续 `task_orchestrator` 接入接口；
- 明确说明本分支尚未实现编排器导航适配器。

交接文件只描述后续接口，不在本对话实现接入代码。

## 11. 本阶段验收标准

本阶段完成必须同时满足：

1. DWA 是唯一运行的局部规划器；
2. 文件参数与运行参数一致；
3. footprint、雷达 TF 和里程计经过实车检查；
4. 硬件与导航启动职责分离，不会重复占用资源；
5. 直角弯连续至少三次无碰撞；
6. 当前完整路线连续至少三次无碰撞；
7. 正常停止后无本包节点残留；
8. README 能让其他成员独立部署、启动、测试和调参；
9. 交接文件足以支持另一个对话继续接入 `task_orchestrator`。
