# 明日人工部署步骤

## 部署目标

只部署新包：

```text
/home/ucar/ucar_ws/src/ucar_fast_nav
```

不得覆盖：

```text
/home/ucar/ucar_ws/src/ucar_nav
/home/ucar/ucar_ws/src/ucar_controller
```

## 1. 连接后先记录现状

```bash
ssh ucar@小车IP
date
rosnode list 2>/dev/null || true
rospack find ucar_nav
rospack find ucar_controller
rospack find ydlidar
rospack find jie_ware
rospack find teb_local_planner
```

对以下目录生成备份，不删除原目录：

```bash
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p /home/ucar/navigation_backups/$stamp
cp -a /home/ucar/ucar_ws/src/jie_ware /home/ucar/navigation_backups/$stamp/ 2>/dev/null || true
cp -a /home/ucar/ucar_ws/src/ucar_controller /home/ucar/navigation_backups/$stamp/
```

## 2. 上传独立包

从 Windows 上传：

```powershell
scp -r "D:\program_sec\智能车\.worktrees\navigation-vendor-bundle\navigation_vendor_bundle\source_snapshot\ucar_fast_nav" ucar@小车IP:/home/ucar/ucar_ws/src/
```

若目标已经存在，先将旧的 `ucar_fast_nav` 改名为带时间戳的备份；不要覆盖式混合两个版本。

## 3. 核对定位依赖

先比较本组 `jie_ware` 与归档版本，不同不等于立即覆盖：

```bash
sha256sum /home/ucar/ucar_ws/src/jie_ware/src/lidar_loc.cpp
```

只有确认本组缺少或版本不一致导致编译/行为差异时，才把原目录改名备份，然后上传：

```text
navigation_vendor_bundle/vendor_dependencies/jie_ware
```

## 4. 编译

```bash
cd /home/ucar/ucar_ws
catkin_make
source /home/ucar/ucar_ws/devel/setup.bash
rospack find ucar_fast_nav
roslaunch --files ucar_fast_nav navigation_full.launch
```

任何编译错误先停止，不删除本组其他包。

## 5. 启动选择

如果底盘和雷达均未运行：

```bash
roslaunch ucar_fast_nav navigation_full.launch
```

需要加载二维码观察点参数时，使用：

```bash
roslaunch ucar_fast_nav pickup_navigation.launch
```

该 launch 只加载一个最终目标，不启动 `ucar_waypoint_nav`，也不会自动发车。目标参数位于：

```bash
rosparam get /ucar_fast_nav/pickup_goal
```

如果队友已经启动底盘和雷达：

```bash
roslaunch ucar_fast_nav navigation_full.launch start_base:=false start_lidar:=false
```

如果仅雷达已经运行：

```bash
roslaunch ucar_fast_nav navigation_full.launch start_lidar:=false
```

启动后不要立即发目标，先运行：

```bash
rosrun ucar_fast_nav runtime_check.sh
```

确认定位与运行检查正常后，人工发送唯一终点：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: -1.40219, y: -0.627908, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.026893084056159769, w: 0.9996383156071742}}}"
```

目标航向为 `yaw=0.053792653589793 rad`，即原航向 `-3.0878 rad` 反转 180°。

## 6. 定位与实车测试

在 RViz 或 `/initialpose` 设置真实初始位姿，确认激光点云贴合地图并保持稳定。然后先检查规划，不让车动；最后在现场有人看护时发送二维码观察点目标。

首轮记录：

```bash
rosparam dump /home/ucar/navigation_test_params.yaml /move_base
rosbag record -O /home/ucar/ucar_nav_bags/vendor_chain_first_run.bag /scan /odom /tf /tf_static /initialpose /amcl_pose /move_base/GlobalPlanner/plan /move_base/TebLocalPlannerROS/local_plan /cmd_vel
```

`/amcl_pose` 在 lidar_loc 模式下可能没有数据，不影响录包。

## 7. 停止与回滚

正常停止使用启动终端中的 `Ctrl+C`。确认以下节点退出：

```bash
rosnode list | grep -E 'move_base|map_server|lidar_loc|ydlidar|base_driver'
```

回滚只需停止新入口并不再启动 `ucar_fast_nav`。现有 `ucar_nav` 从未覆盖。如替换过 `jie_ware`，将当前目录改名并把 `/home/ucar/navigation_backups/<时间戳>/jie_ware` 恢复原路径，然后重新编译。
