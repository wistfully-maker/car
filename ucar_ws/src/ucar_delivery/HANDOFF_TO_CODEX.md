# ucar_delivery -> Codex 交接文档（动态视觉配送导航与双停车）

> 本文件是给 Codex 的交接记录。DeepSeek V4 Flash 只完成本地实现、测试、文档与
> 本地提交；部署、上车、PC/Gazebo 集成与逐检查点实车验收由 Codex 负责。
> 本地测试**不能**证明实车行为。Codex 后续顺序固定为：代码审查 → 本地全量回归 →
> 车端覆盖部署 → 包级编译 → CP0 静态检查 → CP1/CP2 无运动感知 → 用户逐关授权的
> CP3–CP8 实车验收（见 README 8.4 与 AGENTS.md 第 11 节）。

## 1. 分支与最终提交

- 工作区：`D:\program_sec\智能车\.worktrees\delivery-dual-parking`
- 分支：`codex/delivery-dual-parking`
- 计划要求起点：`959f85d`（祖先）✓
- 最终 HEAD：本文件所在提交；以 `git rev-parse --short HEAD` 的实际值为准

## 2. 本阶段每个提交与用途

| 提交 | 用途 |
|---|---|
| `27f35f0` | `docs(delivery): plan dynamic visual staging and checkpointed acceptance` —— 本计划文件（AGENTS.md）所在提交 |
| `56ae62e` | `test(delivery): lock dynamic staging mission contract` —— 任务 A：新增 `ESTIMATE_STAGING_POSE`/`NAVIGATE_STAGING_POSE` 状态合同（RED→GREEN）；标牌对正后不得直接进入 `ACQUIRE_FRAME`；动态点未确认不发 staging goal；staging 导航成功且停稳后才启动白框搜索；估计失败在当前点重新对正采样、耗尽后换点；staging 导航失败有界重试；所有失败/取消/超时先零速；三速度源与唯一 mux 合同不变 |
| `4b2aece` | `feat(delivery): estimate safe staging pose from vision and lidar` —— 任务 B：新增 `staging_pose_estimator.py`（ROS-free 视觉限定雷达前缘拟合 + `StagingPoseConsistency` 连续一致确认 + `pose_to_quaternion`）；24 个测试覆盖正面/斜板/噪声/部分遮挡/法向朝向/补偿/变换/拒绝路径 |
| `4241a8a` | `feat(delivery): navigate through validated dynamic staging pose` —— 任务 C：mission 节点接入 `/scan` 缓存与 laser→map TF；`NavSupervisor` 会话携带 `goal_kind`（viewpoint/staging）路由结果；标牌 bbox→bearing/half-width（`sign_detector.camera_center_x/focal_px` 标定参数）；连续一致确认后只发一次 staging goal；`delivery.yaml` 增加 `staging_pose` 配置块 |
| `fb4a722` | `feat(delivery): validate parking frame geometry over time` —— 任务 D：白框检测增强（ROI、可选 HSV 白色约束、开闭运算、横线厚度/交点/透视几何一致性置信、`FrameConfirmGate` 多帧获取确认与丢失宽限、近区丢失标志）；新增 `test/fixtures/README.md`（脱敏夹具规则） |
| `d6e923e` | `fix(delivery): close fail-safe visual parking loop` —— 任务 E：远区丢框 → 零速 + `REACQUIRE` 有限重获（`frame_reacquire_timeout` 超时失败）；近区丢框立即失败；`VERIFY_STOP` 期间丢框直接失败；verify 增加白线/雷达终停条件（任一条件丢失清零稳定计时）；lidar 启用时无效 scan 立即失败（不视为安全） |
| `37de76e` | `test(delivery): cover dynamic navigation and parking scenarios` —— 任务 F：`test_delivery_scenarios.py` 16 个端到端纯本地场景（12 项覆盖要求），含每个检查点 `/task/delivery_status` 精确状态名遥测断言 |
| `90512bd` | `docs(delivery): document checkpointed vehicle acceptance` —— 任务 G：本交接与 README 分阶段验收文档 |
| `219feb5` | `fix(delivery): honor scan-stamp TF and fail-fast vehicle OCR contract` —— 首轮验收退回修复：SignAdapterNode 初始化与 ROS1 TF 调用签名；该提交仍遗留以下五项安全缺口 |
| `ef7cef8` | `docs(delivery): plan safety contract fixes` —— Codex 五项阻塞修复的 TDD 实施计划 |
| `56acd0e` | `fix(delivery): consume exact scan frame once` —— 使用 scan 自身 frame/stamp；拒绝空/零 header；成功或失败帧都只消费一次 |
| `ecfca56` | `fix(delivery): compose vehicle yolo and ocr` —— 新增真实 YOLO→OCR 组合适配，不再由假 YOLO 预填 OCR 结果 |
| `0e8e3c1` | `fix(delivery): fail closed on near-zone frame loss` —— 保存最后确认的近区状态，完全丢框也立即失败 |
| `3055a97` | `fix(delivery): gate staging handoff on stopped odometry` —— staging action 成功后增加新鲜近零里程计连续稳定门与有界超时 |
| `4c333f4` | `docs(delivery): record safety-fix acceptance evidence` —— 五项安全修复的测试与限制证据 |
| 本次提交 | `fix(delivery): match deployed vehicle inference contract` —— CP0 核对车端真实 YOLO/OCR `predict` 契约，增加裁框 OCR、电子车间别名和 launch 后端显式选择 |

## 3. 本阶段修改/新增文件清单（相对 `959f85d`）

`ucar_ws/src/ucar_delivery/`：

- 新增：`src/ucar_delivery/staging_pose_estimator.py`、
  `src/ucar_delivery/vehicle_sign_backend.py`、
  `test/test_staging_pose_estimator.py`、`test/test_delivery_scenarios.py`、
  `test/fixtures/README.md`
- 修改：`AGENTS.md`（本计划）、`config/delivery.yaml`、`README.md`、
  `HANDOFF_TO_CODEX.md`（本文件）、`src/ucar_delivery/mission.py`、
  `src/ucar_delivery/nav_supervisor.py`、`src/ucar_delivery/frame_detector.py`、
  `src/ucar_delivery/parking_controller.py`、`scripts/delivery_mission_node.py`、
  `scripts/sign_adapter_node.py`、`scripts/frame_detector_node.py`、
  `scripts/parking_controller_node.py`、`test/test_mission.py`、
  `test/test_nodes.py`、`test/test_nav_supervisor.py`、
  `test/test_frame_detector.py`、`test/test_parking_controller.py`、
  `test/test_package_config.py`

`ucar_ws/src/task_orchestrator/`：**无差异**（验证门必查）。

仓库文档新增：`docs/superpowers/plans/2026-08-10-delivery-safety-contract-fixes.md`。

`ucar_ws/src/ucar_nav/`：**原则上无新差异**（仅此前阶段已存在的
`navigation_stack.launch` cmd_vel_topic arg/remap；本阶段未改动）。

## 4. 测试命令、数量、时长与结果（Windows 本地 Python 3.13）

```powershell
python -m unittest discover -s ucar_ws/src/ucar_delivery/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/ucar_delivery
python -c "import xml.etree.ElementTree as ET; ET.parse(r'ucar_ws/src/ucar_delivery/launch/delivery.launch'); ET.parse(r'ucar_ws/src/ucar_delivery/launch/delivery_standalone.launch'); ET.parse(r'ucar_ws/src/ucar_nav/launch/navigation_stack.launch')"
git diff --check
git diff --name-status 959f85d..HEAD -- ucar_ws/src/task_orchestrator
git diff --name-status 959f85d..HEAD -- ucar_ws/src/ucar_nav
git status --short
```

结果（最终全量，见验证门）：**348 测试，0 失败 0 跳过**（`test_*.py` 全量，
含 scan frame/stamp 单次消费、真实 YOLO+OCR 组合、近区完全丢框、staging
里程计停稳门，以及原任务 A–F 的 16 个场景测试）；`compileall`、XML 解析、
`git diff --check` 通过；
`task_orchestrator` 无差异；`ucar_nav` 无新差异。

## 5. 最终状态机与速度所有权

状态顺序：`IDLE -> NAVIGATE_VIEWPOINT -> SEARCH_SIGN -> ALIGN_SIGN ->
ESTIMATE_STAGING_POSE -> NAVIGATE_STAGING_POSE -> ACQUIRE_FRAME ->
ALIGN_FRAME -> CENTER_FRAME -> APPROACH_FRAME -> FINAL_STOP -> VERIFY_STOP
-> ARRIVED`；失败/取消/超时统一 `任意活动状态 -> SAFE_STOP -> FAILED |
CANCELLED | TIMEOUT`（先零速后终态）。

模式映射：`NAVIGATE_VIEWPOINT/NAVIGATE_STAGING_POSE -> NAVIGATION`；
`SEARCH_SIGN/ALIGN_SIGN/ESTIMATE_STAGING_POSE -> VISUAL_SEARCH`；
`ACQUIRE_FRAME..VERIFY_STOP -> PARKING`；`IDLE/SAFE_STOP/terminal -> IDLE`。

速度所有权不变：move_base 独占 `/cmd_vel/delivery_navigation`；mission 仅
`SEARCH_SIGN/ALIGN_SIGN` 拥有 `/cmd_vel/delivery_manual`（离开手动域/终止/
取消/关闭发一次性零）；parking controller 独占 `/cmd_vel/delivery_parking`；
`delivery_velocity_mux_node` 是唯一最终 `/cmd_vel` 发布者（fail-closed）。

## 6. 动态观察点算法（输入/输出/拒绝条件/默认参数）

- 输入：`estimate(ranges, angle_min, angle_increment, bearing, half_width,
  laser_to_map_transform, config)` —— bearing/half_width 由视觉 bbox +
  `camera_center_x/camera_focal_px` 标定得到（另加
  `camera_lidar_yaw_offset_deg` 安装补偿）；
- 算法：视觉扇区（含 `bearing_margin_deg`）内提取有限/在范围激光点 →
  距离离群过滤 → PCA 拟合前缘 → 法向量固定朝向机器人 → 表面中心沿法向量
  退后 `staging_distance` → laser→map 旋转平移；
- 输出：`{x, y, yaw(面向表面中心, 归一化 [-pi,pi]), surface_center_laser,
  surface_normal_laser, inlier_count, residual, confidence}`；
- 拒绝（`StagingPoseError`，结构化原因，禁止 NaN/原点/半成品）：点数不足、
  离群过滤后不足、残差过大、内点不足、观察点太近/太远、表面比
  `staging_distance` 近、变换非有限；
- 连续一致：`StagingPoseConsistency` 连续 `estimation_confirmations` 次落在
  `estimation_consistency_xy/yaw_deg` 内才发 staging goal（只发一次）；
- 默认参数见 `config/delivery.yaml` 的 `staging_pose:` 块（安全初始值，
  不是实车最优值）。

## 7. 白框检测与停车成功条件

- 检测：灰度阈值（可选 HSV 低饱和/高明度）→ 开闭运算（默认关）→ ROI 内
  行/列 profile 提取横线与左右边线 → 几何置信度（边界数基础分 + 横线厚度/
  交点/透视一致性加减分，`min_geometry_confidence` 阈值）；
- 多帧滞回：`acquire_confirm_frames` 连续帧确认、`lost_grace_frames` 丢失
  宽限、`near_zone_front_y` 近区判定（近区丢框立即标志）；
- 停车成功（`verified`）唯一条件：`VERIFY_STOP` 中对齐（yaw/lateral
  deadband）+ 白线（`visual_stop_front_y`）或雷达（`target_stop_distance`）
  终停条件 + 里程计近零（`velocity_threshold`）+ 稳定时长
  （`verify_duration`），任一条件丢失稳定计时清零；只有 controller `ok`
  才把 mission 推到 `ARRIVED` 并发布 `arrived`。

## 8. 实物/仿真 mock JSON 与预期结果

物理：`/task/delivery_navigation_goal` 输入
`{"protocol_version":1,"task_id":"task-test-001","goal_id":"delivery-test-001","target_workshop":"食品加工车间","selected_item":"苹果"}` →
稳定停车后 `/task/delivery_arrived` 输出
`{"protocol_version":1,"task_id":"task-test-001","goal_id":"delivery-test-001","status":"arrived","message":""}`。

仿真：`/task/simulation_navigation_goal` 输入（`target_workshop` 用
`日用品加工车间`/`电子产品生产车间`）→ `/task/simulation_arrived` 输出。
`arrived` 唯一发布条件为 `VERIFY_STOP` 成功；到达搜索点、识别标牌、生成
动态观察点或到达动态观察点都**不**发布成功。

## 9. CP0–CP8 观察 topic 与待标定参数

观察 topic：`/ucar_delivery/motion_mode`、`/task/delivery_status`、
`/task/delivery_sign_found`、`/task/delivery_frame_observation`、
`/task/delivery_parking_progress`、`/task/delivery_parking_result`、
`/task/delivery_arrived`、`/task/simulation_arrived`、`/scan`、`/odom`、
`/cmd_vel`（唯一最终发布者）、TF（`rosrun tf view_frames`）。

待标定参数（均有单位/默认值/安全方向/回退值，见 README 第 9 节参数表）：

- `viewpoints.physical/.simulation`（map 系坐标）；
- `sign_detector.camera_center_x / camera_focal_px`（bbox→bearing 相机内参）、
  `staging_pose.camera_lidar_yaw_offset_deg`（相机/雷达安装补偿）、
  `staging_pose.staging_distance` 与 `min/max_staging_travel`（观察点距离）；
- `frame_detector.gray_threshold / roi_* / min_geometry_confidence /
  near_zone_front_y / acquire_confirm_frames / lost_grace_frames`（光照与
  场地几何）；`use_hsv` 与 `hsv_*`（彩色杂物场景按需开启）；
- `parking_controller.k_yaw / k_lateral / approach_speed /
  visual_stop_front_y / frame_reacquire_timeout / verify_duration /
  velocity_threshold`；
- `sign_align.*`、`sign_search.angular_speed`、`timeouts.*`；
- 验证后才可启用：`lidar_safety.enabled`。

## 10. 未验证假设与风险

- laser→map TF（map/odom/base_link/laser_frame）从未在实车验证；TF 失败按
  不可信处理（估计失败，不进 staging）；
- 相机/雷达安装方向一致性、`camera_center_x/focal_px`、yaw 安装补偿全部
  为安全初始值，未标定；
- CP0 已核对车端接口：`yolo_biao.infer.YoloDetector.predict(frame)` 返回
  `[x1,y1,x2,y2,confidence]` 列表；`ocr.ocr_infer.RapidOcrInfer.predict(crop)`
  返回 `(text, confidence)`。适配器已按该契约组合，并保留启动期 fail-fast；真实
  相机图像上的识别率仍需 CP1/CP2 无运动验收；
- 动态观察点估计在真实场地/真实标牌（非理想平面）下的内点率与残差未知；
- 远区重获、近区失败、verify 终停条件的真实时间尺度未验证；
- 真实导航精度、AMCL 定位、底盘符号约定（`angular_z = -k_yaw*yaw_error -
  k_lateral*lateral_error`）未验证；
- 本地测试不构成实车/仿真验收；未部署、未上车、未启动任何运动、未 push/
  merge/部署。

## 11. 明确未执行事项

- 未接入 PC/Gazebo；`/task/sim_trigger` 与 `/task/sim_complete` 不存在；
- 未接入竞争编排器 `task_orchestrator`（无差异）；
- 已执行部署、包级编译与 CP0 接口核对；尚未启动配送 ROS 节点，CP1–CP8 未执行；
- 未 push、未 merge、未创建远程分支。

## 12. 文档路径

- 中文 README：`ucar_ws/src/ucar_delivery/README.md`
- 本交接：`ucar_ws/src/ucar_delivery/HANDOFF_TO_CODEX.md`
- 计划：`AGENTS.md`（含 CP0–CP8 详细验收设计，第 11 节）
