# U-CAR 国赛最终版使用手册

本手册只说明国赛版相对省赛版新增或改变的操作。完整系统结构和二维码模块说明可参考省赛版本文档。国赛车端统一使用 `/home/ucar/ucar_ws_pro`。

## 1 环境确认

打开全新终端，避免残留的省赛工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws_pro
catkin_make
source devel/setup.bash

rospack find task_orchestrator
rospack find line_follow_integration
rospack find ucar_avoid
```

三个路径都应位于 `/home/ucar/ucar_ws_pro/src`。如果指向 `/home/ucar/ucar_ws`，关闭当前终端后重试，不要混用两个 workspace 的 `devel/setup.bash`。

## 2 正式启动

```bash
cd /home/ucar/ucar_ws_pro
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./src/task_orchestrator/scripts/start_competition.sh
```

首次运行保留机械急停，并先确认：

```bash
rostopic info /cmd_vel
rostopic hz /usb_cam/image_raw
rostopic echo /task/motion_mode
```

最终 `/cmd_vel` 只能有全局速度仲裁器一个发布者。

## 3 坡道功能

坡道逻辑位于：

```text
ucar_ws/src/ucar_avoid/scripts/slope_intervention.py
```

验证顺序：先检查 IMU 俯仰角是否稳定，再在底盘架空条件下观察触发和状态变化，最后才在有人看护的低速场景测试。坡道干预开始后会暂时接管既定运动流程；异常、俯仰角漂移或恢复导航失败时立即机械急停，不要连续重启多个坡道节点。

排查重点：

- IMU 数据是否新鲜、静止基线是否已建立；
- 坡道是否只触发一次；
- 取消导航和恢复导航的目标是否属于当前任务；
- 坡道完成或失败后是否停止继续输出干预速度。

## 4 红绿灯线路预启动

红绿灯线路预启动由李冠桥负责。它的目标是提前完成识别与巡线链所需的资源准备，避免车辆到达红绿灯区域后才加载模型或启动进程。

预启动后的正确状态是“节点或子进程已准备，但车辆仍不运动”。检查：

```bash
rostopic hz /usb_cam/image_raw
rostopic echo /task/line_follow/status
rostopic echo /task/motion_mode
rostopic echo /cmd_vel/line_follow
```

安全约束：

1. 到达巡线阶段前，`/task/motion_mode` 不得因预启动提前切换到 `LINE_FOLLOW`；
2. 红灯识别结果为 `red_light` 时继续等待；
3. 方向锁定后才允许启动对应路线；
4. 不得同时运行旧的 `start_all_yolo.launch`，它可能重复占用相机和底盘；
5. 预启动产生的旧结果必须在正式任务开始前清理或按时间戳拒绝。

如果预启动后长时间没有方向结果，依次检查模型路径、模型哈希、相机 topic、图像尺寸、NPU/RKNN 初始化和子进程日志。不要为缩短等待而绕过状态门控。

## 5 巡线调整

国赛巡线代码主要位于：

```text
ucar_ws/src/car_server
ucar_ws/src/line_follow_integration
```

路线选择关系由集成层维护。调试时先确认 `/line_follow/image_raw` 连续更新，再观察 `/line_follow/cmd_vel_candidate` 和 `/cmd_vel/line_follow`；不要让原始巡线脚本直接发布最终 `/cmd_vel`。

```bash
rostopic hz /line_follow/image_raw
rostopic echo /line_follow/cmd_vel_candidate
rostopic echo /cmd_vel/line_follow
rostopic echo /task/line_follow/status
```

改巡线参数时一次只改一个变量，并分别记录左转、右转、直行路线的入弯、出弯、停车线识别和最终停车表现。出现图像中断、子进程退出或超时，系统应先归零再报告失败，不能用旧的 `/tmp/yolo_result.txt` 或 `/tmp/stop_done.txt` 判定本轮成功。

## 6 停止与回退

正常停止使用启动根终端的 `Ctrl+C`，等待其拥有的子进程退出。若有残留，先用 `rosnode info` 确认归属，再停止对应 roslaunch。禁止使用 `pkill ros`、`killall` 或 `rosnode kill -a`。

需要回退省赛版本时，不要在同一工作空间覆盖运行中的文件。先停止全部相关 launch，备份 `/home/ucar/ucar_ws_pro/src` 中的国赛改动，再从省赛归档分支恢复到独立目录并重新构建、source 和检查。

