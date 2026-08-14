# HANDOFF TO CODEX — QR 持续扫码重构（第二轮）

> 本文件由 OpenCode/DeepSeek 在本地实现与测试完成后编写。**尚未部署、尚未实车验证。** 实车部署、备份、编译与验收由 Codex 独立执行。

## 1. 分支与提交

分支：`feature/qr-stop-scan`（工作树 `.worktrees/qr-stop-scan`）。

基线：`a742bfa fix(qr): harden stop-scan safety and tuning`（wistfully-maker 提交，包含第一轮 12 个提交）。

本轮提交（新 -> 旧）：

| 哈希 | 说明 |
|---|---|
| `53bc2cd` | fix(qr): expose decode scale launch override（Codex 验收修复：允许 launch 命令行覆盖 `decode_scale`，同步当前 SSH 与相机格式文档） |
| `217b4a5` | docs(qr): document continuous scanning, heartbeat and decode scaling |
| `5d8fa72` | feat(qr): add bounded in-node decode scaling（任务 C） |
| `97b8727` | refactor(qr): scan continuously for the whole search instead of per-station windows（任务 B） |
| `3143ae3` | fix(qr): decouple camera liveness from decode via frame_seen heartbeat（任务 A） |

DeepSeek 交付 HEAD：`217b4a5`；Codex 本地验收修复后代码 HEAD：`53bc2cd`。未 push、未 merge、未 rebase。工作树根目录的未跟踪诊断图片 `failed_stop-scan-001_p0_s0_1786185256.055.jpg` 与 `qr_highres_1280x720.jpg` 均未改动、未提交（Codex 实车诊断素材）。

## 2. 本地验收结果（全部通过）

测试命令（Windows 本地，Python 3.11 + cv2，pyzbar 在测试中 mock；本机 Python 3.13 因 `python313._pth` 隔离模式忽略 PYTHONPATH，故用 README 第 17 节的 `sys.path.insert` 方式运行）：

```
Ran 284 tests in 3.779s
OK
```

Codex 在 `53bc2cd` 上重新运行完整测试、`compileall` 与 `git diff --check`，结果均通过。Windows Python 3.11 环境未安装 `pyzbar`，因此两张实拍静态图的真实 ZBar 复测留到小车原生运行环境执行，不以 mock 测试替代该项实车验收。

- `python -m compileall -q ucar_ws/src/qr_item_search`：通过
- `git diff --check`：通过
- `git status --short`：仅显示两张未跟踪诊断图片
- launch XML 可解析（`test_package_config` 覆盖）；QR launch 不启动相机、不含相机 width/height/fps/pixel format 参数；`decode_scale` 只传给 qr_scanner；全部控制参数为 launch arg 可从命令行覆盖；`start_debug_stream` 默认 false

## 3. 本轮行为变化（相对 a742bfa）

### 状态机与 scanner enable

- 状态机不变（`INITIAL_SCAN → TURNING → SETTLING → SCANNING → … → WAITING_HTTP → COMPLETE/NOT_FOUND/ERROR`）。
- **扫码使能**：从任务进入 `INITIAL_SCAN` 起至终态前一直 `enabled=true`（含 `TURNING/SETTLING/WAITING_HTTP`），不再随状态开关；终态/stop/shutdown 才置 false。
- **驻留语义**：`scan_window` 现为“每站最短稳定驻留时间”，不再控制扫码开关；驻留期间识别到 URL 不提前结束本站，驻留满后前往下一站；第三个 URL 例外——任何状态识别到即立即零速进入 `WAITING_HTTP`。
- **检测结果接受范围**：URL 在转向/停稳途中识别同样接受；慢解码跨过站点边界只要同一 `task_id/search_id` 仍接受；上一搜索任务的慢结果继续由 generation/identity 丢弃。
- controller 不再发布 `capture_id/capture_after`；`pass_index/station_index` 保留（关键帧标注 + 失败站末帧判定）。

### 相机心跳（任务 A）

- scanner 图像回调在 CvBridge 成功并成功提交帧后，按单调时间节流 5 Hz 发布 `frame_seen` scanner event（identity 取当前控制身份）。
- controller 只对属于当前任务的 `frame_seen` 更新 `_last_frame_seen`；`camera_timeout` 只查 `_last_frame_seen`，覆盖全部 ACTIVE 状态（含 WAITING_HTTP）；新任务重置心跳并给满 `camera_timeout` 宽限。
- `quality` 只用于指标与画面质量，不再承担相机存活证明。
- 修复的实车缺陷：上一扫描站最后一个 `quality` 事件被当作心跳，转向超过 `camera_timeout` 后新站进入 0.005 秒内误报 `camera timed out`（实测证据 `INITIAL_SCAN 0.647s / TURNING 2.050s / SCANNING 0.005s → ERROR`）。

### 关键帧

- 成功关键帧仍用真正产生 URL 的原始帧，帧角度用采集时 yaw（控制消息在站点边界携带的航向）。
- 失败站末帧改由“站点上下文变化”（控制中 pass/station 变化）或“终态关闭”触发，不再依赖窗口开关边沿；“本站未识别”不阻断持续扫码。

### decode_scale（任务 C）

- `decode_variants(image, enhanced, decode_scale=1.0)`：原始帧始终第一个变体；`>1.0` 时追加一个有界放大变体（`cv2.resize` INTER_LINEAR，不修改输入数组），失败自动跳过并继续灰度/CLAHE/阈值路径。
- 校验：非有限值、`<=0`、`>2.0` 拒绝（`MAX_DECODE_SCALE=2.0`）。
- scanner 节点 `~decode_scale` 默认 **1.5**（launch 中 `<param name="decode_scale" value="1.5"/>`），只作用于 scanner。
- 默认值依据本机性能测试（OpenCV QRCodeDetector 代理解码，640×480 合成二维码、QR 区域约 120×120）：

  | scale | resize | 解码中位 | 相对解码成本 |
  |---|---|---:|---:|
  | 1.0 | 0.02 ms | 7.6 ms | 1.0× |
  | 1.5 | 0.5 ms | 20.6 ms | ~2.7× |
  | 2.0 | 0.9 ms | 35.4 ms | ~4.6× |

  1.5× 在成本与采样收益（约 2.25× 像素采样）间取平衡；持续扫码只保留最新帧，无积压。pyzbar 在小车上按像素数同向缩放，具体数值待实车验证。

## 4. 废弃行为与测试替代关系（任务 B）

| 废弃行为（a742bfa） | 替代（提交 `97b8727`） |
|---|---|
| `test_window_change_during_slow_decode_discards_old_result`：慢解码跨站丢弃结果 | `test_slow_decode_crossing_station_boundary_is_accepted`（同搜索跨站接受）+ `test_previous_search_result_still_dropped`（跨搜索仍丢弃） |
| `test_rejects_frame_submitted_under_old_capture_id` / `test_rejects_frame_with_stamp_before_capture_after` / `test_rejects_frame_without_stamp_when_gating_active`：capture_id/stamp 门控 | `test_capture_fields_do_not_reject_same_search_frames` / `test_frames_without_stamp_are_accepted` / `test_no_gating_when_capture_fields_absent`（字段仍解析校验，但不拒绝同搜索帧） |
| `test_window_close_without_url_saves_last_frame` / `test_window_close_with_url_skips_failed_save`：窗口开关边沿保存失败站末帧 | `test_station_change_without_url_saves_last_frame` / `test_station_change_with_url_skips_failed_save` / `test_disable_without_url_saves_last_frame`（站点上下文变化/终态触发） |
| `test_new_url_ends_station_without_waiting_resolved` / `test_scanner_only_enabled_inside_scan_windows` / `test_three_urls_stop_and_wait_http`（关闭扫码断言） | `test_detected_does_not_end_dwell_and_http_proceeds` / `test_scanner_stays_enabled_across_dwell_and_turning` / 新增 `test_continuous_scanning.py` 全文件（全状态 enabled、驻留不中断、任意状态三 URL 即停、终态关闭扫码） |

## 5. 修改文件清单（本轮）

- `src/qr_item_search/controller_logic.py`：frame_seen 心跳、camera 检查覆盖全部 ACTIVE 状态、驻留语义、持续使能控制、站点上下文控制
- `src/qr_item_search/scanner_logic.py`：移除 capture 门控与跨站丢弃、站点上下文失败站保存
- `src/qr_item_search/image_quality.py`：`decode_scale` 变体与校验（`MAX_DECODE_SCALE=2.0`）
- `scripts/qr_scanner_node.py`：`frame_seen` 心跳发布（5 Hz 节流）、`~decode_scale` 接线
- `launch/qr_item_search.launch`：scanner 增加 `decode_scale=1.5`
- 测试：新增 `test_camera_heartbeat.py`、`test_continuous_scanning.py`；重写 `test_capture_window.py`；修改 `test_image_quality.py`、`test_controller_logic.py`、`test_qr_scanner_logic.py`、`test_package_config.py`
- `README.md`：持续扫码语义、心跳/事件区别、故障判断、decode_scale 说明

未改动：公共 `usb_cam`、`car_server/ucar_camera.launch`、相机原始配置、`task_orchestrator`、导航、避障、语音、LLM、`sweep_coverage.py`（YawTracker 仍被使用）。

## 6. 已知限制与实车需验证事项

- **camera_timeout 覆盖 WAITING_HTTP**：等待 HTTP 期间若相机真正断流也会 `ERROR`（设计意图：相机健康与解析解耦）；若实车出现该状态下的意外断流，需先查相机而非改参数。
- **frame_seen 身份**：scanner 只在收到非空身份控制后才提交帧并发布心跳，启动竞态下 controller 不会误收空身份心跳。
- **转向途中 yaw 标注**：帧角度取最近一次站点边界控制携带的航向；驻留期间精确，转向期间为近似值（解码完成时车身航向绝不使用）。如需更精确需 scanner 订阅 /odom，本轮未做。
- **decode_scale 实车验证**：1.5 是本机测量；小车上 pyzbar 成本与帧率影响需按“单变量”原则实测，必要时降回 1.0 或升 2.0（先确认 CPU 余量）。
- 失败关键帧（`failed_stop-scan-001_p0_s0_*.jpg`）在 120×130 像素区域 OpenCV/pyzbar 均不可解码；软件放大不增加真实采样细节，**不得声称该帧已可解码**。真实修复依赖焦距/曝光/距离/码面尺寸，不是 resize。
- Windows 本地测试需 Python 3.11+ cv2；pyzbar 在测试中 mock。
- 实车先验证“一圈空载无相机超时误报”，再进入三码与参数矩阵。

## 7. 实车验收与部署（Codex 执行，DeepSeek 未执行）

1. 提交审查与本地全量回归（`PYTHONPATH=~/ucar_ws/src/qr_item_search/src python3 -m unittest discover -s test -p 'test_*.py' -q`）。
2. 备份车端旧包；catkin 编译；只开相机的实时画面验证（`start_debug_stream:=true`）。
3. 只读节点检查：确认 QR launch 未启动/重配 usb_cam，`decode_scale` 仅在 scanner。
4. 空旷地安全运行：一圈空载验证持续扫码 + 各站停靠 + 无相机超时误报。
5. 三码实测与参数矩阵（45° + 速度 0.40/0.50/0.60，窗口 0.40/0.60/0.80；每组连续 10 轮 ≥9 轮 3/3 且全部安全停稳），记录到 `docs/parameter-experiment-template.csv`。
