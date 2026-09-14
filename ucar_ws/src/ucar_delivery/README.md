# ucar_delivery：独立配送包（导航、标牌确认与白框停车）

> **状态：本地实现与测试完成，未经过实车/场地/Gazebo 验证，未接入竞争编排器。**
> 本包的全部运动逻辑、视觉算法与协议在本地以纯逻辑、合成图像和 fake-ROS 方式测试；
> 本地测试**不能**证明真实导航精度、感知准确率、停车精度或任何集成。任何上车验证必须由
> Codex 协调，有人看护并保留机械急停。

## 1. 包用途与本阶段停止点

本阶段把 `ucar_delivery` 修复为一个**可独立运行**的配送包：接受一个模拟的物理配送目标
（`/task/delivery_navigation_goal`）或仿真车间目标（`/task/simulation_navigation_goal`），
用 AMCL + move_base/TEB 导航到配置视点，YOLO/OCR 确认目标车间标牌，相机白框检测与闭环控制
完成三面白框停车，最后发布对应到达回执。

**本阶段停止点：不接入 `task_orchestrator`、PC/Gazebo、比赛总 launch。** 物理配送与
仿真车间是**两次独立运行**，不自动链接。未来的编排器集成（发布/订阅这些 goal 与回执、
停用本地 mux）只作为远期契约，见第 15 节。

## 2. 两条独立流程

```text
物理配送（独立运行）：
mock /task/delivery_navigation_goal
  -> ucar_delivery 按配置视点 move_base 导航
  -> YOLO/OCR 适配确认目标车间标牌
  -> 相机检测并对齐三面白框
  -> 闭环停车（可选激光前墙安全、稳定停止验证）
  -> /task/delivery_arrived

仿真车间（独立运行）：
mock /task/simulation_navigation_goal
  -> 同一 ucar_delivery 引擎以 phase=simulation 运行
  -> 导航/搜索/停车
  -> /task/simulation_arrived
```

两条流程分别可测；本阶段**不**自动把它们链接成一条链。

## 3. 传感器与算法分工（不可互换）

| 子系统 | 职责 | 本包用法 |
|---|---|---|
| AMCL + map + move_base/TEB | 全局定位、障碍感知、视点间移动 | `move_base` action，goal 用 `map` 系坐标 |
| YOLO + OCR（窄接口适配） | 车间标牌候选与语义确认 | `sign_adapter_node`（fake 默认；vehicle 已按车端 `predict` 契约适配） |
| 相机白框 | 停车框获取、yaw 修正、横向居中、接近引导 | `frame_detector_node` + `parking_controller_node` |
| 视觉限定雷达扇区 | 目标表面前缘拟合 → map 系动态观察点（找对车间后才进白框停车） | `staging_pose_estimator.py`（mission 节点内） |
| 2D 激光 `/scan` | 代价地图障碍、动态观察点拟合、可选前墙安全/纵向停车辅助 | `lidar_safety`（默认**关闭**） |

"laser" 与 "radar" 在本文指同一个 2D 激光雷达。AMCL 不直接测量墙距；激光不作为几何对齐
判定依据（单面前墙无法判定横向居中），白框视觉始终是停车几何主依据。
`lidar_safety.enabled` 只有完成前向扇形覆盖、位姿、有限距离与有效墙面返回验证后才能打开。

## 4. 协议 v1：topic、JSON 与相关性规则

所有业务消息为 `std_msgs/String`，`data` 内为 UTF-8 protocol v1 JSON。

| 方向 | topic | 说明 |
|---|---|---|
| 输入 | `/task/delivery_navigation_goal` | 物理配送目标 |
| 输入 | `/task/simulation_navigation_goal` | 仿真车间配送目标 |
| 输入 | `/task/cancel` | 取消当前任务 |
| 输入 | `/task/delivery_sign_found` | 标牌确认结果（内部接口） |
| 输入 | `/task/delivery_frame_observation` | 白框观察（frame_detector 发布） |
| 输入 | `/task/delivery_parking_progress` | 停车阶段进度（controller 发布） |
| 输入 | `/task/delivery_parking_result` | 停车结果（controller 发布） |
| 输入 | `/task/delivery_sign_fake` | fake 后端注入（仅测试/本地） |
| 输入 | `/usb_cam/image_raw`、`/scan`、`/odom` | 感知输入（只订阅，不创建） |
| 输出 | `/task/delivery_arrived` | 物理配送回执 |
| 输出 | `/task/simulation_arrived` | 仿真车间回执 |
| 输出 | `/ucar_delivery/motion_mode` | `IDLE` / `NAVIGATION` / `VISUAL_SEARCH` / `PARKING` / `EMERGENCY_STOP` |
| 输出 | `/task/delivery_status` | 配送状态机状态（调试） |
| 输出 | `/task/delivery_sign_start`、`/task/delivery_frame_start`、`/task/delivery_parking_start` | 阶段启动（内部接口） |
| 输出 | `/cmd_vel/delivery_manual` | 标牌搜索/对齐源命令（mission 独占） |
| 输出 | `/cmd_vel/delivery_parking` | 停车源命令（parking controller 独占） |

### 4.1 `/task/delivery_navigation_goal`（输入）

```json
{"protocol_version":1,"task_id":"task-test-001","goal_id":"delivery-test-001","target_workshop":"食品加工车间","selected_item":"苹果"}
```

### 4.2 `/task/delivery_arrived`（输出，成功）

```json
{"protocol_version":1,"task_id":"task-test-001","goal_id":"delivery-test-001","status":"arrived","message":""}
```

失败：`status="failed"` 且 `message` 非空、可执行。

### 4.3 `/task/simulation_navigation_goal`（输入）

```json
{"protocol_version":1,"task_id":"task-test-001","goal_id":"simulation-test-001","target_workshop":"日用品加工车间","selected_item":"毛巾"}
```

### 4.4 `/task/simulation_arrived`（输出，成功）

```json
{"protocol_version":1,"task_id":"task-test-001","goal_id":"simulation-test-001","status":"arrived","message":""}
```

### 4.5 相关性规则

- 校验 `protocol_version`、`task_id`、`goal_id`、车间与物品；`target_workshop` 只允许
  `食品加工车间`、`日用品加工车间`、`电子产品生产车间`；
- 标识符原样保留；所有感知/停车事件按 `phase + task_id + goal_id` 相关性过滤，
  跨阶段/陈旧事件忽略；
- 重复投递**活动**目标：只重发状态，不重复移动；
- 重复投递**终端**目标（已 failed/cancelled/timeout/arrived）：只重发终端状态/结果，
  绝不再次移动；不同的 `(phase, task_id, goal_id)` 可重置失败终态并启动新任务；
- `arrived` 只在 `VERIFY_STOP` 稳定验证通过后发布，单独到达视点不会发布；
- 失败、超时、取消、相机丢失、墙边丢线、激光危险必须先停再发 `failed`。

## 5. 速度源与本地 velocity mux 所有权

```text
/cmd_vel/delivery_navigation ----\
/cmd_vel/delivery_manual ---------> delivery_velocity_mux_node -> /cmd_vel
/cmd_vel/delivery_parking -------/
                                    模式：/ucar_delivery/motion_mode
```

- 独立模式恰好一个节点发布最终 `/cmd_vel`：`delivery_velocity_mux_node`；
- 每个输入 topic 恰好一个拥有者：
  - move_base 拥有 `/cmd_vel/delivery_navigation`；
  - `delivery_mission_node` 只在 `SEARCH_SIGN` / `ALIGN_SIGN` 发布
    `/cmd_vel/delivery_manual`（白框停车状态**不**发布任何周期命令；
    离开手动域/安全终止/取消/关闭时发一次性零）；
  - `parking_controller_node` 只从白框对齐到稳定停止验证发布
    `/cmd_vel/delivery_parking`；
- mux 是 fail-closed 的：模式切换瞬间归零；输入陈旧超过 `velocity_mux.source_timeout`
  （0.3 s）归零；`IDLE` / `EMERGENCY_STOP` / 未知模式 / 关闭归零。
- 未来集成：停用本地 mux，把这三个隔离源重映射进外部竞争仲裁器。**本阶段不实现。**

## 6. 外部必须提供的共享硬件

`delivery_standalone.launch` 不拥有硬件，以下必须由外部启动：

- base 底盘驱动（最终 `/cmd_vel` 的执行者）；
- 2D 激光驱动（`/scan`）；
- 唯一公共相机（默认 `/usb_cam/image_raw`，不修改任何相机设备参数）。

## 7. 构建与源码命令（车端环境）

```bash
cd ~/ucar_ws
catkin_make --pkg ucar_delivery
source devel/setup.bash
rospack find ucar_delivery   # 确认安装
```

`catkin_make --pkg ucar_delivery` 会重新运行工作区 CMake 配置并刷新 devel 包装，
但实际 make 目标限定在 `ucar_delivery`；它不会修改其他包源码。本包已在车端以该命令
编译通过。不要在竞赛节点运行中编译。

## 8. 本地启动方式

本地（Windows/无 ROS）只能做纯逻辑/静态测试，不能启动 ROS 图：

```powershell
python -m unittest discover -s ucar_ws/src/ucar_delivery/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/ucar_delivery
```

测试套件覆盖端到端纯本地场景（`test_delivery_scenarios.py`）：第一/第二搜索
视点、动态观察点估计重试与换点、staging 导航失败、远区白框丢失重获、近区
丢框失败、激光危险、取消/超时/关闭、实物→仿真独立任务、跨阶段/陈旧事件
拒绝，以及每个检查点在 `/task/delivery_status` 上发布带
`phase/task_id/goal_id/state/status/message` 的遥测（`state` 为状态机精确
名称，无业务别名）。

ROS1 环境下的独立运行：

```bash
# 终端 1：独立配送根 launch（一条导航栈 + 4 个配送组件 + 本地 mux）
roslaunch ucar_delivery delivery_standalone.launch sign_backend:=vehicle
```

`sign_backend` 默认保持 `fake`，防止误把测试注入后端当作实车识别。实车流程必须显式传
`sign_backend:=vehicle`；加载 `yolo_biao.infer.YoloDetector` 或
`ocr.ocr_infer.RapidOcrInfer` 失败时节点启动即退出，不允许进入运动流程。

### 8.1 mock 物理配送目标

```bash
rostopic pub -1 /task/delivery_navigation_goal std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"goal_id\":\"delivery-test-001\",\"target_workshop\":\"食品加工车间\",\"selected_item\":\"苹果\"}'"
```

等价于 `roslaunch ucar_delivery delivery_standalone.launch` 之外另起
`mock_goal_publisher.py`（phase=physical）。

### 8.2 mock 仿真车间目标

```bash
rostopic pub -1 /task/simulation_navigation_goal std_msgs/String \
"data: '{\"protocol_version\":1,\"task_id\":\"task-test-001\",\"goal_id\":\"simulation-test-001\",\"target_workshop\":\"日用品加工车间\",\"selected_item\":\"毛巾\"}'"
```

### 8.3 预期状态序列与到达消息

```text
goal -> NAVIGATE_VIEWPOINT -> SEARCH_SIGN -> ALIGN_SIGN
     -> ESTIMATE_STAGING_POSE -> NAVIGATE_STAGING_POSE -> ACQUIRE_FRAME
     -> ALIGN_FRAME -> CENTER_FRAME -> APPROACH_FRAME -> FINAL_STOP
     -> VERIFY_STOP -> ARRIVED
（模式切换：NAVIGATION -> VISUAL_SEARCH -> PARKING -> IDLE）
```

标牌对正后不会直接进入白框搜索：必须先用视觉限定雷达扇区拟合目标表面并
生成 map 系动态观察点（连续一致确认），经 move_base 导航到该点并确认停稳，
才启动白框搜索。动态观察点只是"能看到白框的安全观察点"，最终停车由白框
视觉闭环决定。到达回执见第 4.2/4.4 节；任何失败/超时/取消先 `SAFE_STOP`
发布零速度，再发 `status="failed"` 且带可执行 `message` 的回执。

动态点估计严格使用每条 `LaserScan.header.frame_id` 与原始非零时间戳查询
scan→map TF；空 frame、零/非法 stamp、陈旧 scan 都不参与估计。每个
`(frame_id, stamp)` 无论 TF/拟合成功或失败都最多消费一次，重新对正后必须等待
新 scan，不能用同一失败帧耗尽重试。staging move_base 成功后还必须满足下表的
里程计新鲜度、线/角速度近零与连续稳定时长，才进入 `ACQUIRE_FRAME`。

### 8.4 分阶段启动与无运动检查（Codex 逐检查点实车验收，DeepSeek 不执行）

**任何一关失败，不进入下一关；每关之间保持机械急停与人员看护。**

- **CP0 静态与资源所有权（不允许车轮运动）**：`rosnode list` 确认无重复
  资源（单 map/AMCL/move_base、单相机、单最终 `/cmd_vel` 发布者）；
  `rosparam get /ucar_delivery/motion_mode` 应为 `IDLE`；`rostopic echo -n1
  /cmd_vel` 持续为零。
- **CP1 传感器方向与时间戳（架空轮/断电机）**：人工把目标置左/右时
  `/task/delivery_status` 中检测 bearing 符号正确；前方放板时 `/scan` 前向
  扇区距离正确；`rosrun tf view_frames` 检查 map/odom/base_link/laser_frame；
  `/scan` 与相机图像时间戳不陈旧。
- **CP2 纯感知/静止估计（底盘保持 IDLE）**：在三个距离与至少两个斜角摆放
  目标，观察标牌连续确认、`/task/delivery_status` 的
  `ESTIMATE_STAGING_POSE` 阶段、雷达内点与残差、动态观察点；重复估计波动
  小于 `estimation_consistency_*` 阈值；错误标牌与白色杂物不触发。**不发
  送导航目标。**
- **CP3 单搜索点导航（低速、清空场地）**：只测试一个已标定搜索点，禁用白框
  停车；观察 action 状态、`/ucar_delivery/motion_mode=NAVIGATION`、到点停稳、
  取消延迟与失败恢复。
- **CP4 搜索与动态观察点导航**：允许旋转识别并生成 staging goal，禁用停车
  控制器；确认识别时及时停止搜索、动态点经一致性确认后只发送一次、
  move_base 到点后朝向白框、白框在相机 ROI 内。
- **CP5 白框获取与对正（禁前进，仅低角速度对正）**：完整框与斜框识别稳定；
  杂物不触发；转向方向正确；丢框立即零速；差速模式无横移。
- **CP6 短距离低速停车（人员持急停）**：分别测正入、左右偏置、轻微偏航；
  先禁用 `lidar_safety` 再单独启用；不越过安全边界；近区丢框停车失败；
  到白线后零速；稳定时长满足后才有 `arrived`。
- **CP7 完整单次实物流程**：mock 物理 goal，`/task/delivery_arrived` 只发布
  一次 `arrived`，重复 goal 不再运动。
- **CP8 独立第二次仿真车间流程与恢复**：发送独立 simulation goal；测试一次
  人为失败后发送新 goal；结果发布到 `/task/simulation_arrived`；旧事件不驱动
  车辆。

每关保存：精确提交 HEAD、启动命令、参数快照、关键 topic 摘要、失败原因与
是否通过。只有用户明确同意后才进入下一检查点。观察 topic 清单见第 13 节。

## 9. 全部 launch/YAML 参数（单位、默认值、安全调参方向）

全部环境相关参数在 `config/delivery.yaml`（launch 可覆盖）。

| 参数 | 默认 | 单位 | 安全范围 | 说明/调参方向 |
|---|---|---|---|---|
| `mission.viewpoint_count` | 3 | 个 | 1-6 | 每阶段搜索视点数，须等于对应视点列表长度 |
| `mission.viewpoint_max_retries` | 2 | 次 | 0-4 | 单视点导航失败重试预算 |
| `viewpoints.physical/.simulation` | — | m/rad | 场地内 | map 系 {x,y,yaw}，实车按地图标定 |
| `timeouts.*` | 见文件 | s | 见下 | navigation 120 / sign_search 60 / sign_align 30 / staging_estimate 30 / staging_navigation 120 / frame_acquire 45 / frame_align 30 / frame_center 30 / approach 60 / final_stop 30 / verify 15 / safe_stop 10 |
| `staging_pose.enabled` | true | — | — | 动态观察点总开关 |
| `staging_pose.camera_lidar_yaw_offset_deg` | 0.0 | ° | -30..30 | 相机/雷达安装 yaw 补偿（待标定） |
| `staging_pose.bearing_margin_deg` | 2.0 | ° | 0..10 | 视觉半宽外的扇区余量 |
| `staging_pose.min_valid_points` | 8 | 点 | 4..30 | 扇区内有效激光点数下限 |
| `staging_pose.min_inliers` | 6 | 点 | 3..20 | 拟合内点下限 |
| `staging_pose.max_fit_residual` | 0.04 | m | 0.01..0.10 | 前缘拟合残差上限 |
| `staging_pose.max_range_spread` | 0.50 | m | 0.1..1.0 | 距离离群过滤窗口 |
| `staging_pose.staging_distance` | 0.70 | m | 0.3..1.5 | 观察点退后距离（安全下限由场地定） |
| `staging_pose.min_staging_travel` | 0.15 | m | 0.05..0.5 | 观察点距机器人最小距离 |
| `staging_pose.max_staging_travel` | 2.00 | m | 0.5..5.0 | 观察点距机器人最大距离 |
| `staging_pose.estimation_confirmations` | 3 | 次 | 1..10 | 连续一致估计确认次数 |
| `staging_pose.estimation_consistency_xy` | 0.10 | m | 0.02..0.3 | 估计一致性 xy 阈值 |
| `staging_pose.estimation_consistency_yaw_deg` | 8.0 | ° | 2..20 | 估计一致性 yaw 阈值 |
| `staging_pose.staging_estimation_max_retries` | 3 | 次 | 0..10 | 当前点估计失败重试（重新对正采样） |
| `staging_pose.staging_navigation_max_retries` | 2 | 次 | 0..5 | staging 导航失败重试 |
| `staging_pose.tf_timeout` | 0.5 | s | 0.1..2 | 预留：ROS1 tf API 无 timeout 参数（按 scan 时间戳查询），tf2 迁移时使用 |
| `staging_pose.max_scan_age` | 0.5 | s | 0.1..2 | 最新 scan 最大年龄 |
| `sign_detector.camera_center_x` | 320.0 | px | — | 相机光轴像素横坐标（待标定） |
| `sign_detector.camera_focal_px` | 500.0 | px | — | 相机焦距 fx（待标定） |
| `nav_supervisor.action_timeout` | 120 | s | 30-300 | move_base 单次尝试超时 |
| `nav_supervisor.cancel_timeout` | 10 | s | 3-30 | 取消后等待终止上限 |
| `nav_supervisor.settle_time` | 0.5 | s | 0-3 | 到达后稳定等待 |
| `nav_supervisor.settle_timeout` | 5.0 | s | 1-15 | staging 到达后等待停稳的总上限；超时安全失败 |
| `nav_supervisor.odom_timeout` | 0.5 | s | 0.1-2 | staging 停稳判定允许的最大里程计年龄 |
| `nav_supervisor.settle_velocity_threshold` | 0.02 | m/s | 0.005-0.05 | `|vx|`、`|vy|` 近零阈值 |
| `nav_supervisor.settle_angular_threshold` | 0.05 | rad/s | 0.01-0.15 | `|vth|` 近零阈值 |
| `nav_supervisor.settle_stable_duration` | 0.5 | s | 0.2-2 | 新鲜近零里程计必须连续满足的时长 |
| `sign_detector.min_confidence` | 0.6 | — | 0.5-0.9 | 标牌候选置信度下限 |
| `sign_detector.confirm_frames` | 3 | 帧 | 1-10 | 连续确认帧数 |
| `sign_detector.ocr_confirm_frames` | 1 | 帧 | 0-3 | 其中 OCR 匹配帧数 |
| `sign_detector.max_image_age` | 1.0 | s | 0.3-2 | 陈旧图像拒绝 |
| `sign_detector.backend` / launch `sign_backend` | fake | — | fake/vehicle | 实车显式设为 vehicle；导入或接口不符即 fail-fast |
| `frame_detector.gray_threshold` | 180 | 灰度 | 120-220 | 白线/蓝地阈值，按光照调 |
| `frame_detector.min_line_pixels` | 60 | px | 30-150 | 水平线段最短像素 |
| `frame_detector.min_col_pixels` | 60 | px | 30-150 | 垂直侧线最短像素 |
| `frame_detector.roi_x_min/max, roi_y_min/max` | 0/640/120/480 | px | 场地内 | 检测 ROI（全图坐标），ROI 外目标不触发 |
| `frame_detector.max_line_thickness` | 20 | 行 | 5..50 | 横线行跨度上限（大片白色判杂物） |
| `frame_detector.min_geometry_confidence` | 0.65 | — | 0.5..0.9 | 单帧几何置信度阈值 |
| `frame_detector.morph_iterations` | 0 | 次 | 0..3 | 开闭运算迭代（细线会损失边缘，默认关） |
| `frame_detector.use_hsv` | false | — | — | 启用 HSV 白色约束（滤彩色杂物） |
| `frame_detector.hsv_sat_max` | 60.0 | 饱和度 | 30..120 | HSV 白色饱和度上限 |
| `frame_detector.hsv_value_min` | 180.0 | 明度 | 120..240 | HSV 白色明度下限 |
| `frame_detector.perspective_tolerance(_px)` | 0.2/15.0 | —/px | — | 近宽远窄透视容差 |
| `frame_detector.acquire_confirm_frames` | 3 | 帧 | 1..10 | 获取确认帧数 |
| `frame_detector.lost_grace_frames` | 2 | 帧 | 0..10 | 丢失宽限帧数 |
| `frame_detector.near_zone_front_y` | 380 | px | 300..460 | 近区判定线（640x480 图） |
| `parking_controller.k_yaw` | 0.8 | rad/px | 0.2-2 | yaw 增益（符号已锁定，只调幅度） |
| `parking_controller.k_lateral` | 0.5 | rad/px | 0.1-1.5 | 横向增益 |
| `parking_controller.max_linear` | 0.15 | m/s | 0.05-0.3 | 最大前进速度 |
| `parking_controller.max_angular` | 0.4 | rad/s | 0.1-1.0 | 最大角速度 |
| `parking_controller.accel_linear/angular` | 0.3/1.2 | m/s², rad/s² | 0.05-1.5 | 加速度限幅 |
| `parking_controller.approach_speed` | 0.08 | m/s | 0.03-0.15 | 接近阶段速度 |
| `parking_controller.consecutive_frames` | 3 | 帧 | 1-10 | 阶段转换需连续帧 |
| `parking_controller.chassis_capability` | differential | — | differential/holonomic | holonomic 且 `max_linear_y>0` 才启用横移 |
| `parking_controller.visual_stop_front_y` | 420 | px | 380-470 | 无激光时的视觉停车参考（640x480 图） |
| `parking_controller.verify_duration` | 1.0 | s | 0.5-3 | 最终稳定验证时长 |
| `parking_controller.velocity_threshold` | 0.02 | m/s | 0.005-0.05 | 近零速度阈值 |
| `parking_controller.frame_reacquire_timeout` | 1.0 | s | 0.3..3 | 远区丢框后重获时限，超时失败 |
| `parking_controller.near_zone_front_y` | 380 | px | 300..460 | 近区判定线；近区丢框立即失败 |
| `parking_controller.near_zone_loss_is_fatal` | true | — | — | 近区丢框是否立即失败（false 危险，不推荐） |
| `lidar_safety.enabled` | **false** | — | — | 只有完成验证后才能 true |
| `lidar_safety.front_sector_deg` | 60 | ° | 20-120 | 前向扇形 |
| `lidar_safety.min_range/max_range` | 0.05/8.0 | m | — | 有效距离过滤 |
| `lidar_safety.safety_stop_distance` | 0.25 | m | 0.15-0.5 | 低于即立即零速度 |
| `lidar_safety.target_stop_distance` | 0.30 | m | ≥safety | 接近停车目标距离 |
| `sign_search.angular_speed` | 0.35 | rad/s | 0.1-0.6 | 搜索旋转速度 |
| `sign_align.angular_gain/max_angular/tolerance_rad` | 0.6/0.3/0.06 | — | — | 标牌对齐参数 |
| `velocity_mux.source_timeout` | 0.3 | s | 0.1-1.0 | 输入陈旧归零时限 |
| `velocity_mux.publish_rate` | 20.0 | Hz | 5-50 | mux 最终 `/cmd_vel` 发布频率 |
| `topics.cmd_vel_navigation` | `/cmd_vel/delivery_navigation` | — | — | move_base 输出隔离 topic |
| `topics.cmd_vel_manual` | `/cmd_vel/delivery_manual` | — | — | 标牌搜索/对齐源 |
| `topics.cmd_vel_parking` | `/cmd_vel/delivery_parking` | — | — | 停车源 |
| `topics.motion_mode` | `/ucar_delivery/motion_mode` | — | — | 私有运动模式 topic |

## 10. 一次只调一个变量

- **标牌搜索**：先调 `sign_detector.min_confidence` 与 `confirm_frames`（用离线图集验证
  误报/漏报），再调 `sign_search.angular_speed` 与 `timeouts.sign_search`；
- **白线控制**：先调 `frame_detector.gray_threshold`（观察 `delivery_status` 中
  `visible_boundary_count`），再调 `k_yaw`/`k_lateral`，最后调 `approach_speed`；
- **激光安全**：开启 `lidar_safety.enabled` 前先记录 `/scan` 前向扇形原始值，确认
  `safety_stop_distance` 下不会误触发，再逐级收紧；
- **最终停止**：调 `target_stop_distance`（或 `visual_stop_front_y`）与
  `verify_duration`，每次只改一个参数并记录前后停车距离；
- **mux 超时**：若看到周期性归零，先确认对应源是否以高于 `source_timeout` 的频率发布，
  再微调 `velocity_mux.source_timeout`（默认 0.3 s 不应随意加大，它是 fail-closed 底线）。

## 11. 故障排查

| 现象 | 排查 |
|---|---|
| TF 无效 | `rosrun tf view_frames`；确认 map/odom/base_link/laser_frame 链条 |
| AMCL 丢失 | `rostopic echo /amcl_pose`；必要时在起点重发 `/initialpose` |
| move_base abort | 检查代价地图与 TEB 日志、`viewpoint_max_retries`；视点是否可导航 |
| 相机无图像 | 确认外部相机已启动；`rostopic hz /usb_cam/image_raw` |
| 无 `/scan` | 激光驱动未启动或掉线；确认 `laser_frame` TF |
| 标牌未找到 | 确认 `target_workshop` 允许值、`backend` 模式、`min_confidence`、OCR 文本 |
| OCR 不匹配 | 检查标牌文本与 `target_workshop` 全角/半角、空格归一化 |
| 白线丢失 | 降低 `gray_threshold`、检查 ROI 光照；确认 `min_line_pixels` 合理；远区丢框会自动重获，近区丢框会失败（设计如此） |
| 动态点估计失败 | 检查 `camera_lidar_yaw_offset_deg` 标定、视觉 bbox 的 bearing/half_width、`/scan` 新鲜度与 laser->map TF |
| 观察点太近/太远 | 调 `staging_distance` 与 `min/max_staging_travel`；先确认激光与相机安装方向一致 |
| 墙太近 | 若 `lidar_safety.enabled`，检查 `safety_stop_distance` 与扇形角度 |
| 模式陈旧 | 检查 `/ucar_delivery/motion_mode` 是否停在旧值；mux `source_timeout` 是否过小 |
| 重复目标 | 同一 `(phase, task_id, goal_id)` 只重发状态/结果，不重复移动（设计如此） |
| mux 周期性归零 | 对应源发布频率应高于 `velocity_mux.source_timeout`；检查是否只有 mux 发布 `/cmd_vel` |
| identity 不匹配 | 严格复制上阶段输出的 `task_id/goal_id`，陈旧消息会被静默忽略 |

## 12. 安全停止行为

- 任何失败/超时/取消/丢线/激光危险都先发零速度再进入终态（测试锁定顺序）；
- 手动命令域离开、`SAFE_STOP`、取消、关闭都会发布一次性零命令；
- mux 在模式切换、输入陈旧、`EMERGENCY_STOP`、未知模式与关闭时输出零；
- 白色停车框丢失分两种：**远区**（前白线在 `near_zone_front_y` 之上）丢框 →
  立即零速并在 `frame_reacquire_timeout` 内重获，超时安全失败；**近区**丢框
  （`near_zone_loss_is_fatal=true`）→ 立即零速并失败，禁止继续盲走；
  近区状态由最后一帧确认几何跨越丢失宽限保存，因此完全空白的丢失帧不需要
  自己继续携带 `front_boundary_y`；
  `VERIFY_STOP` 期间丢框直接失败，绝不重获后报告 verified；
- `verified` 要求对齐 + 白线/雷达终停条件 + 近零速度 + 稳定时长，任一条件
  丢失稳定计时清零；
- 动态观察点估计只在视觉扇区内使用通过有限/范围检查的激光点，估计失败在
  当前点重新对正采样（有界重试），耗尽后换搜索点，从不盲目前进；
- 上电/首跑必须架空车轮或有人看护，机械急停常备；
- 本包节点常驻，失败终态可由**不同的** `(phase, task_id, goal_id)` 重置并启动新任务；
  重复投递终端目标只重发结果，不再移动。

## 13. 调试信息

- `/task/delivery_status`：状态机 state/status/message（含阶段）；
- `/ucar_delivery/motion_mode`：`IDLE/NAVIGATION/VISUAL_SEARCH/PARKING` 切换；
- `/task/delivery_parking_progress`：`aligned/centered/approach_started/final_stop_done/verified`；
- 动态观察点阶段观察：`/task/delivery_status` 的
  `ESTIMATE_STAGING_POSE`/`NAVIGATE_STAGING_POSE` 状态、`/scan` 原始数据、
  laser→map TF（`rosrun tf view_frames`）、staging 估计失败原因（日志）；
- 日志：`rosnode` 输出标牌确认、safe stop、结果与失败原因。

## 14. 已知限制与未验证假设

- **PC/Gazebo 仿真未接入**；`/task/sim_trigger` 与 `/task/sim_complete` 在本阶段
  **不存在**（stub 已移除），是未来契约；
- 未接入竞争编排器；`velocity_arbiter` 不参与本阶段；
- 车端契约已核对为 `~/ucar_ws/src/yolo_biao/infer.py` 的
  `YoloDetector.predict(frame)`（输出 `[x1,y1,x2,y2,confidence]` 列表）与
  `~/ucar_ws/src/ocr/ocr_infer.py` 的 `RapidOcrInfer.predict(crop)`（输出
  `(text, confidence)`）。适配器按置信度选择 YOLO 框、裁剪后实际执行 OCR，并生成
  `timestamp/target/confidence/bbox/ocr_text/ocr_confidence`；模块、类或签名不符时
  启动即 fail-fast。OCR 的“电子产品加工车间”通过显式别名映射到协议名称
  “电子产品生产车间”；
- 动态观察点：`camera_lidar_yaw_offset_deg`、`sign_detector.camera_center_x /
  camera_focal_px`（bbox→bearing 标定）、激光与相机安装方向、laser→map TF
  均未实车验证；估计结果只是"能看到白框的安全观察点"，不是最终停车点；
- 视点坐标、白框阈值、停车增益全部需要实车标定；
- 本地测试不证明真实导航、感知、停车精度或任何集成。

## 15. 未来工作（本阶段不实现）

- 接入真实编排器：由 `task_orchestrator` 发布 `/task/delivery_navigation_goal` 与
  `/task/simulation_navigation_goal`，消费对应到达回执；
- 停用本地 mux：把三个隔离源重映射进外部竞争仲裁器（`/ucar_delivery/motion_mode`
  词汇已保留兼容）；
- 接入 PC/Gazebo：届时再引入 `/task/sim_trigger` 与 `/task/sim_complete` 并端到端验证。

详细交接清单见 [HANDOFF_TO_CODEX.md](HANDOFF_TO_CODEX.md)。
