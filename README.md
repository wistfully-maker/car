# U-CAR 省赛最终版技术说明

本分支是 2026 年 8 月 17 日从比赛小车归档的省赛最终联调版本，车端工作空间固定为 `/home/ucar/ucar_ws`。它保存了从语音接题、导航、二维码识别、目标分类、语音播报，到车间停车、红绿灯识别和巡线停车的完整 ROS 工程。

## 版本边界

- 分支：`codex/vehicle-snapshot-20260817`
- 原始车端快照：`daa3605c72124dd157c325a5f87e8b1826ece522`
- 省赛工作空间：`/home/ucar/ucar_ws`
- 国赛工作空间：`/home/ucar/ucar_ws_pro`，不属于本分支的运行路径
- 正式入口：`ucar_ws/src/task_orchestrator/scripts/start_competition.sh`

## 技术栈

| 层级 | 技术 | 在项目中的用途 |
| --- | --- | --- |
| 操作系统与中间件 | Ubuntu、ROS 1 Noetic、ROS Master | 多节点通信、参数管理、日志与生命周期 |
| 构建与包管理 | catkin、CMake、package.xml | 构建 ROS 工作空间和声明包依赖 |
| 语言 | Python 3、C++、Bash、XML、YAML | 视觉与编排、底层节点、安全启动、launch 和参数配置 |
| ROS 通信 | topic、actionlib、std_msgs、geometry_msgs、sensor_msgs、nav_msgs、move_base_msgs | 任务协议、速度、图像、里程计和导航结果传递 |
| 导航 | move_base、GlobalPlanner、TEB Local Planner、map_server、lidar_loc、tf/tf2 | 取货区及车间导航、局部规划、地图定位和坐标变换 |
| 视觉 | OpenCV、cv_bridge、NumPy | 图像转换、质量评估、巡线和视觉诊断 |
| 二维码 | pyzbar、ZBar、OpenCV 多图像变体、HTTP requests | QR 解码、跨帧去重、URL 请求和物品名称解析 |
| 目标识别 | YOLO、RKNN/NPU | 红绿灯、方向标志和车间目标识别 |
| 语音与智能分类 | speech_command、Spark LLM HTTP API、TTS bridge | 唤醒与题目输入、双目标分类、播报与回执 |
| 可靠性设计 | 状态机、protocol v1、task_id/goal_id、超时、幂等、fail-closed | 防止过期消息、重复动作、假成功和资源冲突 |
| 调试与验证 | rosbag、rostopic、rosnode、rosparam、Git、SHA-256、unittest | 现场复现、链路观测、版本追踪和回归验证 |

二维码包的直接运行依赖包括 `rospy`、`std_msgs`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`cv_bridge`、`tf`、`python3-pyzbar`、`python3-opencv`、`python3-numpy` 和 `python3-requests`。

## 软件架构

```text
/question
  -> task_orchestrator 全局状态机
  -> ucar_fast_nav + move_base 到二维码领取区
  -> qr_item_search 旋转并持续识别三个二维码
  -> llm_spark 选择实物目标与仿真目标
  -> TTS 播报并等待 speak_done
  -> stop 导航栈完成车间导航与停车
  -> Gazebo 软门控
  -> line_follow_integration 识别红绿灯方向并巡线
  -> 最终停车和任务完成播报
```

关键设计原则：

- `task_orchestrator` 只管理业务顺序和身份关联，不实现视觉或底盘算法；
- 相机、底盘、雷达、地图、定位和 `move_base` 各只有一个 owner；
- 导航、QR、停车和巡线分别输出隔离速度，最终只有 `velocity_arbiter` 发布 `/cmd_vel`；
- 节点常驻，由消息激活，不在状态切换时反复 `roslaunch` 或模糊 kill；
- 所有任务结果按 `task_id/goal_id` 去重，超时、取消和异常均先归零。

```text
/cmd_vel/navigation ----\
/cmd_vel/qr -------------+--> velocity_arbiter --> /cmd_vel
/cmd_vel/stop -----------+
/cmd_vel/line_follow ----/
```

## 主要模块

| 包 | 职责 |
| --- | --- |
| `task_orchestrator` | 全局状态机、安全 preflight、适配器、TTS bridge、速度仲裁 |
| `qr_item_search` | 持续扫码、旋转控制、HTTP 解析、指标、关键帧和 MJPEG 调试流 |
| `ucar_fast_nav` | 二维码领取区导航及 `pickup_goal.yaml` |
| `llm_spark` | 接收候选物品并调用 Spark LLM 分类 |
| `stop` | 车间识别、导航、PCA 对正和停车 |
| `line_follow_integration` | 巡线起点导航、图像适配、方向识别与巡线进程监管 |
| `ucar_controller`、`ydlidar` | 底盘和雷达硬件驱动 |
| `speech_command` | “小飞小飞”唤醒与 `/question` 发布 |

## 二维码模块详解

`qr_item_search` 由三个节点组成：

- `qr_scanner`：接收图像，生成灰度、CLAHE、阈值和软件缩放等解码变体，合并同一帧中的不同 URL；
- `item_search_controller`：根据里程计闭环转向，在各观察角停稳驻留，并管理搜索终态；
- `qr_debug_stream`：可选的 MJPEG 浏览器预览，不参与控制。

默认搜索为第一圈 `0°、45°……315°`，第二圈偏移 `22.5°`。从任务开始到结束持续扫码，转向、停稳和驻留期间都可接受新二维码；识别到 URL 后异步进行 HTTP 请求，车辆不因网络响应而原地阻塞。三个不同 URL 均解析为非空物品名后才发布 `complete`。

省赛实车确认 `qr_scan_window=1.0 s` 比 0.6 秒稳定。比赛现场使用过的高覆盖参数为：30° 步长、15° 偏移、0.70 rad/s 巡航速度、0.30 rad/s 逼近速度和 120 秒内部总超时。切到 30° 时必须同步考虑观察点增多带来的总耗时。

## 配置入口

- 二维码领取区航点：`ucar_ws/src/ucar_fast_nav/config/pickup_goal.yaml`
- 黄色停止线或巡线起点：`ucar_ws/src/line_follow_integration/config/phase3.yaml`
- 二维码参数：`ucar_ws/src/task_orchestrator/launch/competition_full.launch`
- 编排器高级参数：`ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- 巡线图像、超时和模型路径：`ucar_ws/src/line_follow_integration/config/phase3.yaml`

配置修改后必须 `Ctrl+C` 停止根 launch 并重新启动。运行中的节点不会自动读取新 YAML。

## 快速启动

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
source devel/setup.bash
./src/task_orchestrator/scripts/start_competition.sh
```

开启 QR 实时画面：

```bash
./src/task_orchestrator/scripts/start_competition.sh \
  qr_start_debug_stream:=true \
  qr_debug_host:=0.0.0.0 \
  qr_debug_port:=8080
```

浏览器访问 `http://小车IP:8080/`。完整部署、比赛参数、rosbag、分模块调试和停止方法见 [省赛最终版使用手册](docs/USER_GUIDE.md)。

## 安全与维护

- 首次或改参后的运动测试必须有人看护并可立即机械急停；
- 不要让 `car_server/start_all.launch` 与全流程同时运行，它会重复启动底盘和相机；
- 不允许多个定位、多个 `move_base` 或多个 `/cmd_vel` owner；
- 不在 Git、README、日志或 shell history 中保存真实密钥；
- `rosnode cleanup` 只清理 ROS Master 僵尸登记，不会关闭 live 节点；
- 每次比赛或重大调参建议录制 rosbag，以便离线复盘 TF、导航、速度和任务状态。

