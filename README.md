# U-CAR 省赛最终版

本分支保存 2026 年 8 月 17 日从比赛小车采集的完整联调状态，作为省赛最终版归档。代码覆盖语音任务、取货区导航、二维码识别、LLM 分类与播报、车间导航与停车、红绿灯识别和巡线等比赛环节；车端默认工作空间为 `/home/ucar/ucar_ws`。

## 版本信息

- 分支：`codex/vehicle-snapshot-20260817`
- 基线提交：`daa3605c72124dd157c325a5f87e8b1826ece522`
- 快照时间：2026 年 8 月 17 日
- ROS：ROS 1 Noetic
- 构建：catkin
- 主要语言：Python、C++、Shell、XML、YAML
- 主要组件：OpenCV、pyzbar、move_base、GlobalPlanner、TEB、actionlib、tf、usb_cam、RKNN/YOLO、Spark LLM

## 系统流程

```text
语音唤醒与题目输入
 -> 取货区导航
 -> 旋转扫描三个二维码
 -> LLM 选择实物与仿真目标
 -> TTS 播报分类结果
 -> 实物车间导航与停车
 -> 仿真目标处理
 -> 红绿灯方向识别
 -> 对应路线巡线与最终停车
```

全局业务顺序由 `task_orchestrator` 管理。相机、底盘、雷达、定位和导航等公共资源只允许一个 owner；各运动模块先输出到独立速度 topic，再由速度仲裁器统一发布 `/cmd_vel`。

## 二维码模块

二维码模块位于 `ucar_ws/src/qr_item_search`，负责在领取区旋转搜索并识别三个物品二维码。模块主要能力包括：

- 订阅公共相机 `/usb_cam/image_raw`，不重复启动相机；
- 按离散角度旋转并在每个观察点驻留扫码；
- 融合连续图像中的多次识别结果，跨图像变体去重；
- 通过 `task_id` 和 `goal_id` 关联任务，拒绝过期结果；
- 将候选速度 remap 到 `/cmd_vel/qr`，由全局仲裁器决定是否放行；
- 提供相机心跳、总搜索超时、稳定判定、调试流、关键帧和指标目录等诊断能力。

省赛车端确认的关键经验值是 `scan_window=1.0 s`。驻留过短会降低真实光照、轻微运动模糊和二维码反光条件下的解码稳定性。角度步长、偏移、转速和总超时应一次只改一个变量，具体参数见 `ucar_ws/src/qr_item_search/README.md`。

## 目录导航

| 目录 | 用途 |
| --- | --- |
| `ucar_ws/src/task_orchestrator` | 全局状态机、一键启动、安全检查和速度仲裁 |
| `ucar_ws/src/qr_item_search` | 二维码旋转搜索、解码、结果聚合和诊断 |
| `ucar_ws/src/llm_spark` | 物品分类与目标选择 |
| `ucar_ws/src/ucar_fast_nav` | 取货区导航链 |
| `ucar_ws/src/stop` | 车间识别、导航和停车 |
| `ucar_ws/src/line_follow_integration` | 红绿灯方向识别与巡线联调 |
| `ucar_ws/src/ucar_avoid` | 避障相关代码 |
| `docs/vehicle-baselines` | 车端快照和恢复依据 |

## 快速开始

详细操作见 [省赛版使用手册](docs/USER_GUIDE.md)。正式启动入口为：

```bash
cd /home/ucar/ucar_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./src/task_orchestrator/scripts/start_competition.sh
```

首次运行必须架空驱动轮或保留机械急停，先确认 `/cmd_vel` 只有速度仲裁器一个发布者，再逐阶段放行运动。

## 维护原则

- 不在运行中重复启动相机、底盘、雷达、定位或 `move_base`；
- 不绕过速度仲裁器直接发布最终 `/cmd_vel`；
- 不用 `rosnode cleanup` 关闭仍存活的节点；
- 不在仓库、日志或文档中保存真实密钥；
- 修改 YAML 或 launch 参数后停止根 launch 并重新启动；
- 部署前先备份车端目标目录，出现问题按文件恢复，不整包覆盖外部依赖。

