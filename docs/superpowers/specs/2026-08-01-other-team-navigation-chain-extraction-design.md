# 另一组成功导航链独立整理设计

## 1. 目标

从 `D:\program_sec\智能车\另一组小车\ucar` 中提取另一组实际使用的导航完整链路，整理为明天可部署到本组 U-CAR 的独立 ROS 包 `ucar_fast_nav`。

本阶段只整理本地源码、配置、依赖清单和部署交接说明，不连接或修改本组小车，不接入任务编排器，也不制作给用户自行执行的一键部署程序。明天由 Codex 在现场连接小车后完成备份、部署、编译和实车验证。

## 2. 不可破坏边界

- 不修改、删除或覆盖本组小车现有 `/home/ucar/ucar_ws/src/ucar_nav`。
- 不修改另一组小车的本地原始副本。
- 不把另一组整个工作空间未经筛选地覆盖到本组小车。
- 不同时启动两个底盘驱动、两个雷达驱动或两个 `move_base`。
- 所有来源文件保留来源路径和 SHA256，保证可追溯。

## 3. 独立包结构

整理结果保存为：

```text
navigation_vendor_bundle/
  source_snapshot/ucar_fast_nav/
    launch/
    config/
    maps/
    scripts/
    package.xml
    CMakeLists.txt
  vendor_dependencies/
  manifest/
  docs/
    HANDOFF.md
    DEPLOYMENT.md
    RUNTIME_CHAIN.md
```

`ucar_fast_nav` 使用新包名，所有地图和规划参数通过 `$(find ucar_fast_nav)` 加载，不引用或覆盖现有 `ucar_nav` 内的文件。

## 4. 运行链路

目标运行链为：

```text
ucar_controller/base_driver
  -> odom -> base_link

ydlidar
  -> /scan

map_server
  -> /map

jie_ware/lidar_loc
  -> map -> odom

move_base
  -> global_planner/GlobalPlanner
  -> teb_local_planner/TebLocalPlannerROS
  -> /cmd_vel
```

定位使用当前 `ucar_navigation.launch` 中的 `jie_ware/lidar_loc`，不使用备份文件中的 AMCL。备份中的 AMCL 入口仅作为对照资料保存。

## 5. 文件处理策略

### 5.1 复制进新包

- 实际静态地图及 YAML；
- GlobalPlanner 参数；
- TEB 参数；
- global/local/common costmap 参数；
- move_base 通用参数；
- 从原入口改写出的独立 launch；
- 运行前只读自检脚本。

### 5.2 作为外部依赖，不覆盖

- `ucar_controller`；
- `ydlidar`；
- ROS navigation 基础包；
- `teb_local_planner`；
- `jie_ware`。

`jie_ware` 的源码和构建信息会保存到 `vendor_dependencies`，但明天只有在本组小车缺少或版本不一致时，才经备份后单独部署。底盘与雷达驱动默认复用本组已有包，通过独立 launch 传入对方运行参数。

### 5.3 不纳入首轮运行

另一组 `dynamic_obstacle` 链路当前存在 `/scan_filtered`、`base_footprint` 和安全速度话题未闭合的问题。为保证复现实验变量明确，首轮不启动该链路，但原始文件和问题说明会归档。

## 6. 明天部署流程

1. 连接本组小车并记录 Git 状态、ROS 包路径和现有节点。
2. 备份可能涉及的现有包与运行参数。
3. 上传 `ucar_fast_nav`，不触碰 `ucar_nav`。
4. 核对 `jie_ware`、TEB、GlobalPlanner、底盘和雷达依赖版本。
5. 编译工作空间并执行 launch 静态解析。
6. 启动时保证底盘、雷达、定位和 `move_base` 均只有一个实例。
7. 验证 `/scan`、`/odom`、TF、地图、全局路径、局部轨迹和 `/cmd_vel`。
8. 初始化定位后发送原导航目标，进行有人看护的实车测试。
9. 成功则记录参数快照和日志；失败则按层定位差异，不修改现有 `ucar_nav`。

## 7. 验收标准

- 整理包不引用另一组机器上的绝对路径；
- 地图和所有规划参数有明确唯一来源；
- `ucar_fast_nav` 的 launch 能在 ROS Noetic 中解析；
- manifest 列出全部源文件、来源路径和 SHA256；
- HANDOFF 明确启动顺序、冲突节点、检查命令和回滚方法；
- 明天可直接将新包部署到 `/home/ucar/ucar_ws/src/` 后编译；
- 现有 `ucar_nav` 和队友代码保持不变。

## 8. 后续边界

只有独立导航链在本组小车实车跑通后，才开始接入 `task_orchestrator`。联调接口、速度选择器和任务话题不进入本次整理范围。
