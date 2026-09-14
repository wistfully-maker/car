# U-CAR 国赛最终版

本分支保存国赛使用版本。它以省赛最终版为基础，车端工作空间改为 `/home/ucar/ucar_ws_pro`，并围绕国赛场地补充坡道处理、红绿灯线路预启动和巡线代码调整。

## 版本信息

- 分支：`codex/national-baseline-20260817`
- 基线提交：`bdf005033d380182cf0f68772df344388e95c792`
- 上游省赛快照：`daa3605c72124dd157c325a5f87e8b1826ece522`
- ROS：ROS 1 Noetic
- 车端工作空间：`/home/ucar/ucar_ws_pro`

## 相对省赛版的主要变化

### 工作空间迁移

启动脚本、模型路径、配置路径和文档示例由 `/home/ucar/ucar_ws` 统一调整为 `/home/ucar/ucar_ws_pro`。运行前必须确认 `rospack find` 指向新工作空间，避免两个工作空间同时 source 后加载到旧包。

### 坡道代码

`ucar_ws/src/ucar_avoid/scripts/slope_intervention.py` 提供独立坡道干预逻辑：检测俯仰角变化后暂停常规导航，执行坡道穿越，并在满足条件后恢复导航。该功能涉及底盘运动，必须在有人看护和机械急停可用的条件下验证。

### 红绿灯线路预启动

国赛流程在进入红绿灯与巡线阶段前提前准备相关识别和线路运行链，减少临场启动模型、节点或子进程带来的等待和超时风险。预启动只负责提前准备资源，正式运动仍必须等待任务状态机放行，不能提前获得最终 `/cmd_vel` 控制权。

红绿灯线路预启动由项目成员李冠桥负责设计、接入和联调。维护时应重点保护以下约束：

- 红灯状态只等待，不发布巡线速度；
- 方向结果只接受 `left_turn`、`right_turn` 或 `straight`；
- 预启动不能重复启动公共相机、底盘或原始 `start_all_yolo.launch`；
- 巡线候选速度先进入 `/line_follow/cmd_vel_candidate`，经监管器和全局仲裁后才能到 `/cmd_vel`；
- 旧结果文件和过期任务结果不得触发新一轮巡线。

### 巡线代码调整

国赛版根据场地和路线调整了巡线实现，相关代码集中在 `ucar_ws/src/car_server` 与 `ucar_ws/src/line_follow_integration`。方向识别完成后，监管器按左转、右转或直行选择对应路线脚本；巡线图像、速度门控、最终停车线和失败超时仍由集成层统一管理。

## 快速开始

详细步骤见 [国赛版使用手册](docs/USER_GUIDE.md)。正式入口：

```bash
cd /home/ucar/ucar_ws_pro
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./src/task_orchestrator/scripts/start_competition.sh
```

严禁同时 source 省赛旧工作空间后直接启动。若 `rospack find task_orchestrator`、`rospack find line_follow_integration` 或 `rospack find ucar_avoid` 未指向 `/home/ucar/ucar_ws_pro/src`，应先清理终端环境并重新打开 shell。

