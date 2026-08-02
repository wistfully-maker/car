# `ucar_fast_nav` 全流程联调交接

## 1. 交接结论

`ucar_fast_nav` 已在实车和当前场地验证能完整导航到二维码观察点，定位使用 `jie_ware/lidar_loc`，不使用 AMCL。定位稳定、雷达与地图贴合，但当前路线仍存在多次车头或车身蹭墙，因此可用于联调功能链路，暂不应视为比赛最终参数。

小车上的独立包：

```text
/home/ucar/ucar_ws/src/ucar_fast_nav
```

不要覆盖队员使用的：

```text
/home/ucar/ucar_ws/src/ucar_nav
```

## 2. 导航链路

```text
ucar_controller + ydlidar + TF
  -> jie_ware/lidar_loc (map -> odom)
  -> map_server (002.yaml)
  -> GlobalPlanner
  -> TebLocalPlannerROS
  -> /cmd_vel
```

任务编排器不应重复启动底盘、雷达、`lidar_loc`、`map_server` 或 `move_base`。这些节点由导航 bringup 统一拥有，整个任务期间保持运行。

## 3. 启动和停止

底盘、雷达和导航都未启动时：

```bash
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch ucar_fast_nav pickup_navigation.launch
```

该 launch 会启动导航全链路并加载二维码终点参数，但不会自动发送目标，不会自动发车。

若底盘和雷达已由其他人启动：

```bash
roslaunch ucar_fast_nav pickup_navigation.launch \
  start_base:=false \
  start_lidar:=false
```

启动前必须确认没有重名节点、串口占用或多个 `/cmd_vel` 发布者：

```bash
rosnode list
rostopic info /cmd_vel
rostopic hz /scan
rostopic hz /odom
```

正常停止使用 launch 终端中的 `Ctrl+C`。停止后确认：

```bash
rosnode list | grep -E 'move_base|map_server|lidar_loc|ydlidar|base_driver'
```

## 4. 二维码观察点

参数文件：

```text
/home/ucar/ucar_ws/src/ucar_fast_nav/config/pickup_goal.yaml
```

当前终点：

```yaml
frame_id: map
x: -1.40219
y: -0.627908
yaw: 0.053792653589793
```

该航向是原观察点航向反转 180° 后的结果。手动发送命令：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: -1.40219, y: -0.627908, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.026893084056159769, w: 0.9996383156071742}}}"
```

## 5. 接入任务编排器

联调层只需实现一个轻量导航适配器，不要把 `ucar_fast_nav` 的节点启停逻辑写入任务状态机。

预期流程：

```text
语音指令完成
  -> task_orchestrator 发布领取区导航请求
  -> navigation_adapter 向 /move_base action 发送 pickup_goal
  -> move_base 返回 SUCCEEDED
  -> navigation_adapter 发布 /task/pickup_arrived
  -> task_orchestrator 启动 QR 旋转识别
```

适配器必须保留 `task_id` 和 `goal_id`，只有 `move_base` action 返回 `SUCCEEDED` 且小车已停稳时才发布到达。新任务到来时必须取消旧 action goal，禁止过期结果推进新任务。

## 6. 联调前置门槛

1. `/scan` 和 `/odom` 持续发布。
2. `map -> odom -> base_link -> laser_frame` TF 完整。
3. RViz 中雷达墙体与静态地图贴合且不漂移。
4. `/move_base` action server 可用。
5. 只有一个定位节点发布 `map -> odom`，禁止 AMCL 与 `lidar_loc` 同时运行。
6. 现场必须有人看护；在蹭墙问题修复前，不做无人全流程测试。

## 7. 已知风险

- `lidar_loc` 定位已证明比原 AMCL 稳定。
- 当前 GlobalPlanner + TEB 能完成路线，但会多次蹭墙。
- 将 TEB `footprint_model.type` 从 `point` 改为 `polygon` 的单次实测效果更差，不得直接宣称 polygon 已解决问题。
- 联调可使用当前能跑通的 point 基线，但必须有人看护并保留急停。

