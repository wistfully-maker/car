# speech_command overlay

本目录保存小车 `speech_command` 的受控源码覆盖文件，避免把包含第三方 SDK、
二进制资源和私密配置的整个原厂包纳入当前仓库。

部署目标：

```text
patches/speech_command/AIUITester.cpp
  -> /home/ucar/ucar_ws/src/speech_command/src/AIUITester.cpp
patches/speech_command/competition_command_gate.h
  -> /home/ucar/ucar_ws/src/speech_command/src/competition_command_gate.h
```

部署前必须备份小车原文件；部署后执行 `catkin_make`，并完成至少三轮连续
“唤醒→识别→停止→再次唤醒”测试。

当前覆盖文件修复：

- `CMD_STOP` 后下一次硬件唤醒先发送一次 `CMD_START`；
- 串口分片缓存跨回调保留；
- 串口数据失步时从数据块内部重新寻找 `A5 01` 帧头。
- 比赛长指令必须包含领取区、仿真环境、两个不同母类和两次“对应仓库”才发布；
- 不完整比赛片段继续监听，不触发原厂 QA 复述；
- 完整比赛任务只发布一次 `/question`；
- 比赛版本的云端 IAT 路径完全停用原厂 QA：普通非比赛文本也只记录并忽略，
  不调用 `FindDocument`、不写串口回复、不触发自动 TTS；
- “小飞小飞”仍由硬件唤醒链路处理，不依赖已停用的 QA；
- 当 ASR 误识别后再次说出一条包含“领取区”和母类的完整新任务时，用新任务替换
  旧候选，避免错误片段累积出第三个母类。

2026-07-24 已在小车完成多轮双目标测试，验证完整识别、单次发布、无 QA 串口
复述、停止后再次唤醒及 `voice_task_adapter` 母类顺序。最终部署版本另验证了
“日用品 + 电子产品”完整指令，`/question` 单次发布且节点按设计停止录音。

进一步的串口短分片、读取限幅和极端并发加固暂不在本阶段处理，详见仓库根目录
`HANDOFF.md`。

测试：

```bash
python -m unittest discover \
  -s patches/speech_command \
  -p "test_*.py" -v
```

独立 C++ 门控测试：

```bash
g++ -std=c++11 -Ipatches/speech_command \
  patches/speech_command/test_competition_command_gate.cpp \
  -o /tmp/test_gate
/tmp/test_gate
```
