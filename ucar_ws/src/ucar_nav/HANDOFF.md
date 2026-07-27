# ucar_nav DWA 导航交接

更新时间：2026-07-27

## 1. 范围

本分支只建立 DWA 导航安全基线，不实现 `task_orchestrator` 导航适配器。

分支与工作目录：

```text
branch: codex/navigation-safety
worktree: D:\program_sec\智能车\.worktrees\navigation-safety
car package: /home/ucar/ucar_ws/src/ucar_nav
```

## 2. 当前架构

```text
robot_base_bringup.launch
  -> base_driver + ydlidar + optional camera

navigation_stack.launch
  -> map_server + AMCL + move_base
  -> GlobalPlanner + DWAPlannerROS

ucar_navigation.launch
  -> 可选硬件层 + 导航层
```

当前唯一局部规划器：

```text
dwa_local_planner/DWAPlannerROS
```

## 3. 当前有效配置

```text
config/amcl/amcl_omni.yaml
config/costmap/common.yaml
config/costmap/global.yaml
config/costmap/local.yaml
config/local_planners/dwa_safe.yaml
config/global_planner.yaml
config/move_base.yaml
```

旧 `launch/config/amcl`、`launch/config/move_base` 和 TEB RViz 配置已经从活动目录删除。

当前首轮安全值：

```text
footprint: x = ±0.191 m, y = ±0.148 m
inflation_radius: 0.35 m
max_vel_x: 0.25 m/s
max_vel_y: 0.15 m/s
max_vel_theta: 0.50 rad/s
occdist_scale: 0.15
stop_time_buffer: 0.40 s
```

这些值尚需车体实测和实车路线验证，不能作为最终比赛参数结论。

## 4. 当前地图

默认：

```text
/home/ucar/ucar_ws/src/ucar_nav/maps/map.yaml
/home/ucar/ucar_ws/src/ucar_nav/maps/map.pgm
```

原始校验值：

```text
map.yaml:
36FA7FE7D09C50F0D12E321FAD7DD0FDC90A8297762BB42BA6005F2590663B17

map.pgm:
1B50360750BAA7B86BF0E7CC86AF1E4AE81753D819B16D9610A9719F72523E2E
```

地图方向、原点和正式比赛场地对应关系仍需实车确认。

## 5. 已完成验证

- 从小车复制 34 个原始文件；
- 敏感信息扫描无匹配；
- launch 已拆分硬件层与导航层；
- 活动 launch/config 中不再加载或引用 TEB；
- DWA/costmap 静态契约测试已建立；
- 诊断脚本不包含速度发布、目标发布或参数写入。

## 6. 尚未完成验证

- 车端备份和部署；
- `catkin_make --pkg ucar_nav`；
- `roslaunch --nodes`；
- 无运动参数快照；
- footprint 实测；
- 雷达 TF 实测；
- 前进、横移、旋转方向与制动；
- 单墙测试；
- 直角弯连续三次；
- 当前完整路线连续三次。

这些项目完成前，不得声称 DWA 已经实车稳定。

## 7. 实车测试记录

| 日期 | Commit | 地图 | 起点/目标 | 最大速度 | 最小墙距 | 耗时 | 结果 |
|---|---|---|---|---:|---:|---:|---|
| 2026-07-27 | `efb4418` | `map.yaml` | 无目标运行检查 | 0.25 m/s | 未测量 | 未测量 | 通过；未发送目标、无 `/cmd_vel` 输出 |
| 2026-07-27 | 待提交 | 不使用地图 | 空旷地六方向运动学 | 0.10 m/s、0.20 rad/s | 不适用 | 每轴 2 s | 通过；方向正确、制动正常、往返基本复位 |

## 8. 小车备份与部署记录

安全基线已部署至 `/home/ucar/ucar_ws/src/ucar_nav`。原导航包备份位于：

```text
/home/ucar/ucar_nav_backups/ucar_nav-pre-dwa-20260727-01
```

部署后已确认：

- 备份中存在 `package.xml`；
- 车端包与部署暂存目录逐文件一致；
- 新的 `config/local_planners/dwa_safe.yaml` 存在；
- 旧 TEB 参数和 RViz 文件已移除；
- `catkin_make --pkg ucar_nav` 成功；
- 三个 launch 文件均可由 `roslaunch --nodes` 正确解析；
- 无目标启动时雷达约 10 Hz、里程计约 20 Hz；
- `base_link -> laser_frame` 为 `[0, 0, 0.13]`，旋转为单位四元数；
- `map -> base_link`、局部代价地图均可正常读取；
- 运行日志没有 ERROR，3 秒观察期内 `/cmd_vel` 无输出；
- 诊断快照位于 `/tmp/ucar-nav-diagnostics-no-goal/20260727-120038`；
- 检查结束后已关闭本轮节点，尚未进行运动测试。

后续再次部署时，备份仍必须位于 catkin 工作空间外：

```bash
stamp="$(date +%Y%m%d-%H%M%S)"
mkdir -p "$HOME/ucar_nav_backups"
cp -a ~/ucar_ws/src/ucar_nav \
  "$HOME/ucar_nav_backups/ucar_nav-${stamp}"
```

部署完成后将实际备份路径写回本节。

## 9. 后续 task_orchestrator 接口

另一个对话负责实现导航适配器。本分支没有该适配器代码。

后续输入：

```text
/task/pickup_navigation_goal
/task/delivery_navigation_goal
```

后续输出：

```text
/task/pickup_arrived
/task/delivery_arrived
```

适配器应使用：

```text
move_base_msgs/MoveBaseAction
```

它需要：

- 校验 `protocol_version`；
- 原样保留 `task_id`；
- 原样保留 `goal_id`；
- 将领取区和三个车间名称映射为 YAML 位姿；
- 成功时发布 `status: arrived`；
- 失败时发布 `status: failed` 和非空 `message`；
- 取消旧目标并忽略过期 action 结果；
- 不重复启动底盘、雷达、AMCL 或 move_base。

## 10. 后续接手顺序

1. 阅读 `README.md`；
2. 查看最终实车测试表；
3. 确认 DWA commit 和小车部署版本一致；
4. 确认领取区和车间位姿；
5. 在独立分支实现导航适配器；
6. 先人工发布编排 topic 验证；
7. 再接入完整语音、QR、LLM、TTS 流程。
