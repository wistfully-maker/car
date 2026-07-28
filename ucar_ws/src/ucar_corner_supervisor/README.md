# ucar_corner_supervisor

这是一个独立于 `ucar_nav` 的 ROS 1 功能包。它不启动底盘、雷达、地图、AMCL、
`move_base`、Navfn 或 TEB，只接管导航速度输出：

```text
move_base + TEB
       |
       | /move_base/cmd_vel_raw
       v
ucar_corner_supervisor ----> /cmd_vel ----> 底盘
       ^
       | Navfn 全局路径、move_base 状态、map->base_link TF
```

直线路段继续使用 TEB 的前进和旋转指令，仅把横移限制在极小范围；全局路径中的大角度
拐点进入触发距离后，监督器停止平移并按出口方向原地闭环转向，转正且稳定后再把控制权
交还 TEB。这样保留 TEB 的避障和跟踪能力，同时避免其在窄直角弯同时前进、横移、旋转。

## 1. 重要约束

- `/cmd_vel` 必须只有本节点一个发布者。节点启动时会检查，发现其他发布者便拒绝启动。
- `move_base` 的输出必须重映射到 `/move_base/cmd_vel_raw`。
- 本包与旧的 `teb_lateral_mode_controller`、速度转发器或其他 `/cmd_vel` 发布者
  **不得同时运行**。
- 本包不发送、取消或重发 `move_base` 目标。
- 本包不能单独完成导航；底盘、雷达、TF、地图、AMCL 和 `move_base` 必须先正常运行。

## 2. 团队导航 launch 必须做的一处修改

在团队实际启动 `move_base` 的 `<node>` 内加入：

```xml
<node pkg="move_base" type="move_base" name="move_base" output="screen">
  <remap from="/cmd_vel" to="/move_base/cmd_vel_raw"/>
  <!-- 原有 rosparam、remap 等内容保持不变 -->
</node>
```

不要在本包里复制团队的导航 launch。这样队员仍可独立维护 `ucar_nav`，本包只依赖固定
话题接口。

修改后检查：

```bash
rostopic info /move_base/cmd_vel_raw
rostopic info /cmd_vel
```

发送导航目标前，前者应显示 `/move_base` 为发布者；后者只能显示
`/ucar_corner_supervisor` 为发布者。底盘驱动通常是 `/cmd_vel` 的订阅者，不是发布者。

## 3. 部署与编译

在 Windows 仓库根目录部署时，只复制新包，禁止整目录覆盖 `ucar_nav`：

```powershell
scp -r .worktrees\corner-supervisor\ucar_ws\src\ucar_corner_supervisor ucar@10.234.15.42:/home/ucar/ucar_ws/src/
```

在小车上编译：

```bash
ssh ucar@10.234.15.42
cd /home/ucar/ucar_ws
catkin_make
source /home/ucar/ucar_ws/devel/setup.bash
rospack find ucar_corner_supervisor
```

## 4. 启动顺序

1. 启动团队的底盘、雷达和 TF。
2. 启动地图、AMCL 和已经加入速度重映射的 `move_base`。
3. 确认没有旧速度控制器：

   ```bash
   rosnode list | grep -E 'lateral|supervisor|velocity'
   rostopic info /cmd_vel
   ```

4. 启动本包：

   ```bash
   source /home/ucar/ucar_ws/devel/setup.bash
   roslaunch ucar_corner_supervisor corner_supervisor.launch
   ```

5. 再用 RViz、航点脚本或任务模块发送 `move_base` 目标。

本包启动后不会自行发车；没有活动目标时始终输出零速度。

若希望在不修改 `ucar_nav` 的情况下完成一次独立实测，可在关闭旧导航 launch 后使用：

```bash
roslaunch ucar_corner_supervisor navigation_with_corner_supervisor.launch
```

该入口会复用 `ucar_nav/navigation_stack.launch`，强制关闭旧横移控制器，只在其作用域内
把 `/cmd_vel` 重映射为 `/move_base/cmd_vel_raw`，并启动监督器。底盘和雷达 bringup
仍需提前运行。

## 5. 停止

前台运行时按 `Ctrl+C`。若在其他终端或后台运行：

```bash
rosnode kill /ucar_corner_supervisor
```

节点退出时会发布一次零速度。随后如果还要停止整套导航，再关闭 `move_base` 和底盘相关
launch。不要先关闭 ROS master，否则节点无法完成正常退出流程。

## 6. 状态和调试

查看状态：

```bash
rostopic echo /ucar_corner_supervisor/state
rostopic echo /ucar_corner_supervisor/diagnostic
```

状态含义：

- `IDLE`：没有活动导航目标；
- `FOLLOWING`：转发 TEB 指令，横移受限；
- `TURNING`：已到拐点，停止平移并原地转向；
- `EXIT_ALIGN`：角度已进入容差，正在等待稳定保持；
- `BLOCKED`：拐角拟合不可靠、局部代价地图过期或旋转扫掠会碰撞；
- `ERROR`：TF 丢失或转向超时，输出零速度；清除当前目标后才能复位。

常用检查：

```bash
rostopic hz /move_base/cmd_vel_raw
rostopic hz /move_base/NavfnROS/plan
rostopic echo /cmd_vel
rosrun tf tf_echo map base_link
rostopic info /cmd_vel
```

若节点拒绝启动并提示 `/cmd_vel already has publishers`，先按提示停止旧控制器或修正
`move_base` 重映射，不能绕过检查。

## 7. 参数

全部比赛参数位于：

```text
/home/ucar/ucar_ws/src/ucar_corner_supervisor/config/corner_supervisor.yaml
```

修改后重启本节点生效，不需要重启底盘和导航栈。

| 参数 | 初始值 | 作用与调法 |
|---|---:|---|
| `controller_rate` | 20 Hz | 输出频率；底盘仍顿挫时先测原始话题频率，不盲目提高 |
| `raw_command_timeout` | 0.5 s | TEB 指令超时即停车 |
| `ownership_check_interval` | 0.5 s | 运行中复查 `/cmd_vel` 是否出现冲突发布者 |
| `path_timeout` | 0 | 0 表示活动目标期间保留最后一条路径，避免低频重规划漏弯 |
| `min_corner_angle_deg` | 45° | 超过此角度才按大弯处理；误触发时提高 |
| `path_simplify_tolerance` | 0.08 m | RDP 路径简化容差；太大可能吞掉短弯，太小会保留锯齿 |
| `min_stable_segment_length` | 0.15 m | 入口和出口稳定线段的最低长度 |
| `max_fit_residual` | 0.08 m | 直线拟合允许的最大横向残差 |
| `same_turn_merge_distance` | 0.20 m | 同方向重复候选拐点的合并距离；反方向拐点不会合并 |
| `costmap_timeout` | 1.5 s | 局部代价地图最大允许数据年龄 |
| `lethal_cost_threshold` | 100 | OccupancyGrid 中判为墙或致命障碍的值 |
| `sweep_angle_step_deg` | 3° | 旋转扫掠检查的角度采样步长 |
| `footprint` | 0.342×0.256 m | 小车矩形外轮廓，必须覆盖车壳突出部分 |
| `corner_trigger_distance` | 0.25 m | 距拐点多近才停车转向；提前转弯时减小 |
| `corner_release_distance` | 0.45 m | 防止同一拐点反复触发 |
| `following_max_lateral` | 0.02 m/s | 直线跟踪允许的极小横移；左右摆动时减小 |
| `turn_max_angular` | 0.35 rad/s | 原地转向最高角速度 |
| `turn_min_angular` | 0.18 rad/s | 克服底盘死区的最低角速度 |
| `turn_kp` | 0.9 | 航向误差到角速度的比例 |
| `heading_tolerance_deg` | 8° | 转正角度容差；总差一点时可略增，摆动时勿增 |
| `heading_hold_time` | 0.30 s | 连续处于容差内多久才继续前进 |
| `turn_timeout` | 8 s | 单次原地转向最长时间，超时进入 `ERROR` |

第一轮实测只建议调三个参数：

1. 横移仍提前：将 `following_max_lateral` 从 `0.02` 降到 `0.01` 或 `0`；
2. 停车位置太早/太晚：每次以 `0.03 m` 调 `corner_trigger_distance`；
3. 转向过冲/转不动：分别微调 `turn_max_angular`、`turn_min_angular` 和 `turn_kp`。

每次只改一类参数，并记录该轮日志，否则无法判断因果。

诊断中的 `corner_count` 表示当前路径前方识别出的拐点数量；`entry_heading_deg` 和
`exit_heading_deg` 是稳定线段拟合方向。`corner_confidence` 为 `false` 或
`sweep_safe` 为 `false` 时禁止发车，应先检查路径拟合和 `blocking_cell`，不能通过
增大速度绕过。

## 8. 首次实车验收

首次只发已经使用过的二维码航点，现场人员握住急停：

1. 直线段状态为 `FOLLOWING`，不出现大幅横移；
2. 拐点约 0.25 m 前进入 `TURNING`，此时 `/cmd_vel` 的 x、y 必须为 0；
3. 原地转到出口方向后进入 `EXIT_ALIGN`；
4. 约 0.3 s 后恢复 `FOLLOWING` 并向前走；
5. 同一个弯不会连续触发；
6. 两个弯都通过后再进行连续三轮验收。

## 9. 回滚

若新监督器未通过实测：

```bash
rosnode kill /ucar_corner_supervisor
```

然后撤销团队 `move_base` 节点中的速度重映射，使其重新直接发布 `/cmd_vel`。这就是完整
回滚；不需要删除或改动 `ucar_nav` 的其他文件。回滚后旧控制方式和本节点仍然不得同时
运行。

## 10. 与任务编排器的边界

本包不订阅任务编排器协议。任务编排器或导航适配器仍按原方式向 `move_base` action
发送目标；本包只在底层处理该目标执行期间的速度。后续接入全流程时无需修改本包状态机。
