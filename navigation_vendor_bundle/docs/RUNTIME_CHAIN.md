# 导航运行链与参数来源

## 结论

另一组当前实际入口不是差速导航，也不是 AMCL。它使用：

```text
底盘里程计 + 激光扫描匹配定位 + GlobalPlanner + TEB 全向局部规划
```

本归档将地图和规划参数放入独立包 `ucar_fast_nav`，不引用本组现有 `ucar_nav`。

## 节点与所有权

| 节点 | 包 | 输入 | 输出/TF | 所有权 |
|---|---|---|---|---|
| `/base_driver` | `ucar_controller` | `/cmd_vel` | `/odom`、`odom -> base_link` | 唯一底盘驱动 |
| `/ydlidar_node` | `ydlidar` | `/dev/ttyS4` | `/scan` | 唯一雷达驱动 |
| `/base_link_to_laser` | `tf` | 固定参数 | `base_link -> laser_frame` | 雷达 launch |
| `/map_server` | `map_server` | `maps/002.yaml` | `/map` | 新包 |
| `/lidar_loc` | `jie_ware` | `/map`、`/scan`、`/initialpose`、`odom -> base_link` | `map -> odom` | 新包启动、外部实现 |
| `/move_base` | `move_base` | 地图、TF、传感器、目标 | `/cmd_vel`、全局/局部路径 | 新包 |

## 实际规划器

`move_base_params.yaml` 内仍保留 DWA 和自定义 Astar 的旧默认值，但独立 launch 在加载 YAML 前后明确设置：

```xml
<param name="base_global_planner" value="global_planner/GlobalPlanner"/>
<param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/>
```

因此实际运行组合是 GlobalPlanner + TEB。明天必须通过参数服务器再次确认：

```bash
rosparam get /move_base/base_global_planner
rosparam get /move_base/base_local_planner
```

## 定位入口差异

- `vendor_dependencies/original_entrypoints/ucar_navigation.launch`：当前版本，使用 `jie_ware/lidar_loc`。
- `vendor_dependencies/original_entrypoints/ucar_navigation.launch.bak_lidar_loc_20260801_175454`：修改前备份，使用 AMCL omni。

备份文件名表示“切换到 lidar_loc 时生成的备份”，不是“该文件使用 lidar_loc”。本归档首轮使用当前版本。

`lidar_loc` 从 `/initialpose` 接受初始位置，结合 `/scan` 与静态地图计算 `map -> base_link`，再扣除 `odom -> base_link` 后发布 `map -> odom`。

## 原始硬件配置

对方雷达入口固定为：

```text
port=/dev/ttyS4
frame_id=laser_frame
frequency=10 Hz
range_min=0.08 m
range_max=16.0 m
base_link -> laser_frame: x=0.11, y=0, z=0.13, yaw=0
```

底盘入口加载 `ucar_controller/config/driver_params_ucarV2.yaml`。该配置没有复制进新包，因为本组驱动已经修改过里程计标定，明天必须先对比，不能覆盖。

## 代价地图注意事项

原始参数同时存在：

```text
costmap_common_params.yaml: inflation_radius=0.05
local_costmap_params.yaml: inflation_radius=0.25
```

前者被分别加载进 `global_costmap` 和 `local_costmap` 命名空间，后者随后加载。必须以运行时参数为准：

```bash
rosparam get /move_base/global_costmap/inflation_layer
rosparam get /move_base/local_costmap/inflation_layer
```

归档阶段不擅自消除这一差异，以保证与对方运行文件一致。

## 未启用链路

`dynamic_obstacle` 原入口被归档但未启用，原因是：

- tracker 默认订阅 `/scan_filtered`，雷达入口发布 `/scan`；
- 部分代码使用 `base_footprint`，主链使用 `base_link`；
- safety monitor 输出 `/safety/cmd_vel`，原入口没有闭合到底盘 `/cmd_vel` 的 mux。

在主导航复现成功前，不引入该链路。
