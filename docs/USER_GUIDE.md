# U-CAR 省赛最终版使用手册

本手册按部署准备、设备初始化、数据录制、正式启动、流程观察、单模块排查和安全停止的顺序编写。省赛路径统一为 `/home/ucar/ucar_ws`，不能混用国赛的 `/home/ucar/ucar_ws_pro`。

## 1 运行前准备

### 1.1 安全条件

- 操作者全程在车旁，机械急停或电源开关可立即触达；
- 首次运行、改参数或排障时架空驱动轮；
- 清空车辆旋转范围内的人员、线缆和障碍物；
- 不同时启动第二套底盘、雷达、相机、定位、`move_base` 或速度节点；
- 小车与仿真电脑在同一受控网络，比赛需要的服务可达。

### 1.2 加载和构建环境

每个新终端执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
source devel/setup.bash
```

源码或依赖变化后重新构建：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
catkin_make
source devel/setup.bash
```

确认没有混入国赛工作空间：

```bash
echo "$CMAKE_PREFIX_PATH"
rospack find task_orchestrator
rospack find qr_item_search
rospack find line_follow_integration
```

输出应指向 `/home/ucar/ucar_ws/src`。

### 1.3 设备和资源检查

```bash
ls -l /dev/ttyS0 /dev/ttyS4 /dev/ttyS3 /dev/video0
rosnode list
rostopic info /cmd_vel
```

设备占用要找到原进程和原 launch，不要直接重复启动。Spark 密钥应通过环境变量或 `~/.config/ucar/spark_api_password` 提供；密钥文件必须由当前用户拥有、权限为 600、仅含一行，不能提交 Git。

## 2 相机日志问题的现场处理

历史现场经验是首次启动相机可能刷出大量转换日志。可在正式流程前执行一次硬件初始化：

```bash
roslaunch car_server start_all.launch
```

看到底盘和相机节点成功启动后，立即在该终端按 `Ctrl+C`，等待节点完全退出，再执行正式启动。

`start_all.launch` 会启动底盘和 `/usb_cam`，绝对不能与 `start_competition.sh` 并行运行。停止后确认：

```bash
rosnode list | grep -E 'usb_cam|base_driver'
```

仍有输出时先找到所属 roslaunch 并停止，不能直接开第二份。

## 3 比赛 rosbag 录制

每次正式跑车建议先开独立终端录包。以下内容覆盖 TF、导航、分级速度、语音、QR、LLM、停车和巡线状态：

```bash
mkdir -p /home/ucar/bags/one-two
rosbag record -O /home/ucar/bags/one-two/full_$(date +%m%d_%H%M).bag -b 2048 \
  /tf /tf_static /odom /scan \
  /cmd_vel /cmd_vel/navigation /cmd_vel/qr \
  /cmd_vel/stop_navigation /cmd_vel/stop_manual /cmd_vel/line_follow \
  /move_base/goal /move_base/result /move_base/status /move_base/feedback \
  /move_base/current_goal /move_base/GlobalPlanner/plan \
  /move_base/TebLocalPlannerROS/local_plan \
  /move_base_simple/goal /amcl_pose /initialpose \
  /question /voice/speak /voice/speak_done \
  /task/pickup_navigation_goal /task/pickup_arrived \
  /qr_item_search/result /llm/classify/request /llm/classify/result \
  /task/delivery_navigation_goal /task/delivery_arrived /task/stop_mission_goal \
  /stop/motion_mode /stop/mission_event /task/navigation_handoff_status \
  /task/line_navigation_goal /task/line_navigation_arrived \
  /task/line_follow/start /task/line_follow/status /task/motion_mode
```

比赛结束在录包终端按 `Ctrl+C`，等待文件关闭提示后再关机：

```bash
ls -lh /home/ucar/bags/one-two/
rosbag info /home/ucar/bags/one-two/文件名.bag
```

磁盘或写入带宽不足时不要额外录制原始图像 topic。旧 bag 应先复制到电脑再删除。

## 4 正式启动

### 4.1 默认命令

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
source devel/setup.bash
./src/task_orchestrator/scripts/start_competition.sh
```

脚本会 fail-closed 检查工作空间、密钥、设备、同名节点、定位冲突、模型哈希和 `/cmd_vel` owner。出现 `ERROR` 时修复根因，不能绕过 preflight。

### 4.2 开启二维码实时视频

```bash
./src/task_orchestrator/scripts/start_competition.sh \
  qr_start_debug_stream:=true \
  qr_debug_host:=0.0.0.0 \
  qr_debug_port:=8080
```

同一局域网浏览器访问：

- `http://小车IP:8080/`：状态叠加实时画面；
- `http://小车IP:8080/snapshot.jpg`：当前单帧；
- `http://小车IP:8080/stream.mjpg`：MJPEG 流。

访问失败时检查：

```bash
rosnode ping /qr_debug_stream
ss -lntp | grep 8080
rostopic hz /usb_cam/image_raw
```

### 4.3 比赛使用的 30 度二维码参数

```bash
./src/task_orchestrator/scripts/start_competition.sh \
  qr_start_debug_stream:=true \
  qr_debug_host:=0.0.0.0 \
  qr_debug_port:=8080 \
  qr_step_angle_deg:=30.0 \
  qr_offset_angle_deg:=15.0 \
  qr_cruise_angular_speed:=0.70 \
  qr_approach_angular_speed:=0.30 \
  qr_search_total_timeout:=120.0 \
  timeout_qr_search:=150.0
```

`timeout_qr_search` 必须大于 QR 内部 `qr_search_total_timeout`。30° 会增加观察点数量，因此要一起保证总超时足够；不要在同一轮继续修改驻留、曝光或解码倍率。

## 5 仿真电脑

车端根流程启动时同步启动仿真电脑的任务程序，车端桥接端口默认 1525。仿真电脑命令由负责该模块的同学维护；不要猜测命令，也不要让电脑端和车端共用同一个 ROS Master。联调前确认双方 IP、端口和防火墙。

## 6 启动后检查

```bash
rosnode list
rostopic info /cmd_vel
rostopic hz /usb_cam/image_raw
rostopic hz /scan
rostopic hz /odom
rosrun tf tf_echo map base_link
rostopic echo /task/status
rostopic echo /task/motion_mode
```

放行条件：

- `/cmd_vel` 只有 `/velocity_arbiter` 一个发布者；
- 图像、雷达和里程计连续更新；
- `map -> odom -> base_link` 连通；
- 没有同时运行 `/amcl` 与 `/lidar_loc`；
- 车辆静止时最终速度为零。

## 7 完整比赛流程观察

```text
“小飞小飞” -> /question
 -> /task/pickup_navigation_goal -> /task/pickup_arrived
 -> QR 搜索 -> /qr_item_search/result complete
 -> /llm/classify/request -> /llm/classify/result
 -> /voice/speak -> /voice/speak_done
 -> /task/delivery_navigation_goal
 -> 导航栈交接 -> 车间导航与停车
 -> Gazebo 软门控
 -> 巡线起点导航 -> 红绿灯方向 -> 巡线 -> 最终停车
```

推荐打开四个观察终端：

```bash
rostopic echo /task/status
rostopic echo /qr_item_search/result
rostopic echo /task/navigation_handoff_status
rostopic echo /task/line_follow/status
```

任何阶段只接受当前 `task_id/goal_id`。不得人工发布假的 arrived、success 或 speak_done 来证明流程完成。

## 8 二维码模块单独排查

### 8.1 无运动 scanner 测试

先启动公共相机，然后只运行 scanner：

```bash
rosrun qr_item_search qr_scanner_node.py _image_topic:=/usb_cam/image_raw
rostopic echo /qr_item_search/scanner_event
```

不启动 controller 时二维码包不会发布旋转速度。依次观察 `frame_seen`、`quality`、`detected`、`resolved` 和 `resolve_error`。

### 8.2 完整 QR 模块

```bash
roslaunch qr_item_search qr_item_search.launch \
  image_topic:=/usb_cam/image_raw \
  scan_window:=1.0 \
  start_debug_stream:=true
```

QR launch 不启动相机和底盘。手动运动测试前必须已有公共相机、里程计和安全的速度接管关系。

### 8.3 结果状态

| 状态 | 含义 | 处理 |
| --- | --- | --- |
| `searching` | 正在旋转与解析 | 等待终态 |
| `complete` | 三个 URL 均解析成功 | 才能进入 LLM |
| `not_found` | 两圈或超时后不足三个 | 查角度覆盖、图像和网络 |
| `error` | 相机、航向、停稳或协议错误 | 停止并用新任务重试 |
| `stopped` | 人工或上层取消 | 等待下一任务 |

### 8.4 调参顺序

1. 静止验证三张码均可解码；
2. 检查焦距、曝光、反光、码尺寸和白边；
3. 测有效解码视场，再决定 45° 或 30°；
4. 固定角度后只调巡航与逼近速度；
5. 固定运动参数后再调 `scan_window` 或 `decode_scale`；
6. 每组连续多跑，比较成功率、平均和最慢耗时；
7. 保存 `~/qr_metrics`、`~/qr_keyframes` 和 rosbag。

## 9 航点调整

省赛路径：

```text
/home/ucar/ucar_ws/src/ucar_fast_nav/config/pickup_goal.yaml
/home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml
```

前者是二维码领取区观察点；后者包含黄色停止线或巡线起点、图像参数、超时和模型路径。修改前先备份；一次只改一个坐标或容差；修改后停止并重启根 launch。不要把定位 covariance 当作 move_base 目标字段。

## 10 常见故障

| 现象 | 检查 | 处理 |
| --- | --- | --- |
| 相机日志刷屏 | `/rosout`、图像 topic、节点 owner | 必要时按第 2 节预热并彻底停止，再开全流程 |
| 相机设备 busy | `fuser /dev/video0`、`rosnode list` | 停止原相机所属 launch，禁止重复启动 |
| QR 看见但不解码 | 实时画面、关键帧、码尺寸、焦距、曝光 | 先改善物理成像，再试解码倍率 |
| QR 旋转不停 | `/odom`、QR result、超时参数 | 查航向新鲜度、停稳条件和超时层级 |
| LLM 无结果 | request/result、网络、密钥 | 查 HTTP 可达性和 90/120 秒超时 |
| 导航不动 | motion mode、隔离速度、TF、move_base status | 不绕过仲裁，修复 readiness 失败项 |
| 多个 `/cmd_vel` owner | `rostopic info /cmd_vel` | 停止多余 root，只保留仲裁器 |
| 巡线无图 | 原始图像和 `/line_follow/image_raw` | 查相机适配、尺寸和新鲜度 |
| 旧结果触发新任务 | task/goal identity、临时文件时间 | 清理旧文件并确认按时间戳和 ID 拒绝 |

## 11 正常停止

1. 在 rosbag 终端按 `Ctrl+C`，等待文件关闭；
2. 在 `start_competition.sh` 终端按 `Ctrl+C`；
3. 等待它拥有的子进程退出；
4. 检查节点和速度 owner。

```bash
rosnode list
rostopic info /cmd_vel
```

建议单独开终端运行 `roscore`，结束时直接 `Ctrl+C`。找不到原终端时：

```bash
pgrep -af 'roscore'
```

确认 PID 确属本次 roscore 后再执行：

```bash
sudo kill -INT <PID>
```

不要照抄错误 PID，也不要用 `kill -9` 正常关机。`rosnode cleanup` 只清理僵尸登记；`pkill ros`、`killall` 和 `rosnode kill -a` 可能误伤其他节点。

## 12 结束后归档

- 保存 rosbag、QR metrics、关键帧和本次参数命令；
- 记录成功或失败发生在哪个 task state；
- 记录小车 IP、场地光照、二维码位置和修改的 YAML；
- 不归档真实密钥、令牌、`build` 或 `devel`；
- 重要车端版本用 Git commit 和分支名标记，不再靠“最终版”文件夹区分。

