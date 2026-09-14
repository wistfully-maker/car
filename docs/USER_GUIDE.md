# U-CAR 省赛最终版使用手册

本手册用于在比赛小车上部署、启动、观察和停止省赛最终版。默认车端工作空间为 `/home/ucar/ucar_ws`。所有运动测试必须有人看护并能立即触发机械急停。

## 1 准备环境

```bash
source /opt/ros/noetic/setup.bash
cd /home/ucar/ucar_ws
catkin_make
source devel/setup.bash
```

每个新终端都要重新 source。先确认关键包可见：

```bash
rospack find task_orchestrator
rospack find qr_item_search
rospack find llm_spark
rospack find line_follow_integration
```

检查底盘、雷达、相机和语音设备是否存在，并确认没有其他 roslaunch 占用相同设备。Spark 密钥必须通过环境变量或权限为 600 的安全文件提供，不得写进代码或命令历史。

## 2 正式启动

```bash
cd /home/ucar/ucar_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./src/task_orchestrator/scripts/start_competition.sh
```

脚本会先执行 fail-closed 安全检查。遇到同名节点、设备占用、定位冲突、多个 `/cmd_vel` 发布者或密钥不合规时，应排查原 owner，不能通过重复启动或强制 kill 绕过。

## 3 启动后检查

```bash
rosnode list
rostopic info /cmd_vel
rostopic hz /usb_cam/image_raw
rostopic hz /scan
rostopic hz /odom
rostopic echo /task/status
rostopic echo /task/motion_mode
```

`/cmd_vel` 应只有全局速度仲裁器一个发布者。地图、激光、里程计和 `map -> odom -> base_link` TF 不完整时不要发送导航任务。

## 4 二维码模块单独调试

调试前架空驱动轮。二维码模块只订阅公共相机，速度输出必须保留在 `/cmd_vel/qr`。

```bash
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch qr_item_search qr_item_search.launch \
  image_topic:=/usb_cam/image_raw \
  scan_window:=1.0 \
  start_debug_stream:=false
```

观察：

```bash
rostopic hz /usb_cam/image_raw
rostopic echo /qr_item_search/result
rostopic echo /cmd_vel/qr
```

推荐调参顺序：

1. 先固定车辆与二维码位置，只验证相机清晰度、曝光、焦距和反光；
2. 保持 `scan_window=1.0`，只调整单个运动参数；
3. 调整 `step_angle_deg` 后再单独试验 `offset_angle_deg`；30 度步长通常同时测试 15 度偏移；
4. 增加观察点数量后相应增大总搜索超时；
5. 记录每轮三个二维码的成功数、用时和失败角度，不凭一次成功定参数。

常见问题：

| 现象 | 优先检查 |
| --- | --- |
| 完全没有图像 | 相机 owner、topic 名、设备占用和 USB 连接 |
| 有图但不解码 | 焦距、曝光、码尺寸、反光、阳光直射和运动模糊 |
| 偶发漏码 | 驻留时间是否过短、角度覆盖是否有空隙 |
| 重复结果 | 任务 identity、URL 规范化和跨帧去重 |
| 车辆持续旋转 | 内部总超时、编排超时、里程计新鲜度和停止条件 |

## 5 全流程观察

唤醒词触发完整 `/question` 后，重点观察：

```bash
rostopic echo /question
rostopic echo /task/pickup_navigation_goal
rostopic echo /task/pickup_arrived
rostopic echo /qr_item_search/result
rostopic echo /llm/classify/result
rostopic echo /voice/speak_done
rostopic echo /task/delivery_navigation_goal
rostopic echo /task/line_follow/status
```

每个阶段只接受当前 `task_id` 和 `goal_id` 的结果。不要人工伪造到达或成功消息来证明全流程完成。

## 6 安全停止

正常结束时先在启动根终端按 `Ctrl+C`，等待其拥有的子进程退出，再检查：

```bash
rosnode list
rostopic info /cmd_vel
```

若仍有节点，先用 `rosnode info <node>` 确认 owner，再停止对应的 roslaunch。`rosnode cleanup` 只清理 ROS Master 中的僵尸登记，不能关闭 live 节点。禁止使用无目标的 `pkill ros`、`killall` 或 `rosnode kill -a`。

## 7 版本恢复

省赛归档提交为 `daa3605c72124dd157c325a5f87e8b1826ece522`。恢复前先备份车端文件；只恢复确认损坏的文件或项目自有包，不覆盖密钥、模型、`build`、`devel`、日志和车端外部依赖。恢复后重新 `catkin_make`、source，并从无运动检查开始验收。

