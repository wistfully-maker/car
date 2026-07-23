# 任务编排器核心实施计划

> **执行说明：** 按任务逐项实施本计划，并使用复选框（`- [ ]`）记录进度。若由代理执行，应使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`。

**目标：** 构建经过测试、与 ROS 解耦的任务编排核心，以及 ROS topic 适配层、语音文本适配器和 TTS 桥接节点；先通过模拟导航、二维码、LLM 和 TTS 模块跑通完整流程。

**架构：** 业务规则放在不依赖 `rospy` 的纯 Python 模块中。轻量 ROS 脚本把 `std_msgs/String` JSON 消息转换成状态机事件，并发布状态机返回的动作。现有语音硬件节点继续独占麦克风和串口；适配器读取 `/question` 并调用现有 TTS 脚本，不重启 `speech_command_node`。

**技术栈：** ROS 1 Noetic、Python 3.7、`rospy`、`std_msgs/String`、JSON 协议 v1、`unittest`、Catkin。

---

## 实施范围

本计划实现编排器包，并使用模拟 topic 验证完整流程。本阶段不修改已经部署的 `llm_spark` 源码，也不修改导航算法；等核心协议稳定后，再分别制定实际模块的接入计划。

## 文件结构

```text
ucar_ws/src/task_orchestrator/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   └── orchestrator.yaml
├── launch/
│   └── task_orchestrator.launch
├── scripts/
│   ├── task_orchestrator_node.py
│   ├── voice_task_adapter_node.py
│   └── tts_bridge_node.py
├── src/task_orchestrator/
│   ├── __init__.py
│   ├── categories.py
│   ├── protocol.py
│   ├── voice_parser.py
│   └── orchestrator.py
└── test/
    ├── test_categories.py
    ├── test_protocol.py
    ├── test_voice_parser.py
    ├── test_orchestrator.py
    └── test_package_config.py
```

## 任务 1：创建 Catkin 包骨架

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/package.xml`
- 新建：`ucar_ws/src/task_orchestrator/CMakeLists.txt`
- 新建：`ucar_ws/src/task_orchestrator/setup.py`
- 新建：`ucar_ws/src/task_orchestrator/src/task_orchestrator/__init__.py`
- 新建：`ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **步骤 1：创建包目录**

在新 worktree 根目录执行：

```powershell
New-Item -ItemType Directory -Force `
  ucar_ws/src/task_orchestrator/src/task_orchestrator, `
  ucar_ws/src/task_orchestrator/scripts, `
  ucar_ws/src/task_orchestrator/launch, `
  ucar_ws/src/task_orchestrator/config, `
  ucar_ws/src/task_orchestrator/test
```

预期结果：`ucar_ws/src/task_orchestrator` 下出现五个目录。

- [ ] **步骤 2：编写第一个包结构测试**

创建 `test/test_package_config.py`：

```python
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml",
            "CMakeLists.txt",
            "setup.py",
            "src/task_orchestrator/__init__.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **步骤 3：运行测试并确认它按预期失败**

```powershell
python ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

预期结果：出现断言失败，因为包配置文件尚未创建。不能是 `ModuleNotFoundError`；后者表示测试命令不适用于当前 Windows 环境。

- [ ] **步骤 4：创建 `package.xml`**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>task_orchestrator</name>
  <version>0.1.0</version>
  <description>Full workflow orchestration for U-CAR task execution.</description>
  <maintainer email="ucar@example.com">U-CAR Team</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export/>
</package>
```

- [ ] **步骤 5：创建 `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(task_orchestrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()
```

三个 ROS 节点脚本目前还不存在，因此这里暂不添加
`catkin_install_python()`。等任务 6～8 创建脚本后再统一添加，避免
Catkin 在配置阶段引用不存在的文件。

- [ ] **步骤 6：创建 `setup.py` 和 `__init__.py`**

`setup.py`:

```python
#!/usr/bin/env python3
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name="task_orchestrator",
    version="0.1.0",
    packages=["task_orchestrator"],
    package_dir={"": "src"},
)

setup(**setup_args)
```

`src/task_orchestrator/__init__.py`:

```python
"""U-CAR full task orchestration package."""
```

- [ ] **步骤 7：重新运行包结构测试**

```powershell
python ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

预期结果：测试通过。

- [ ] **步骤 8：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "build: scaffold task orchestrator package"
```

## 任务 2：实现可信类别映射和播报文本格式化

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/src/task_orchestrator/categories.py`
- 新建：`ucar_ws/src/task_orchestrator/test/test_categories.py`

- [ ] **步骤 1：编写类别测试**

```python
import unittest
import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.categories import (
    category_config,
    format_result_speech,
)


class CategoryTests(unittest.TestCase):
    def test_all_competition_categories_have_labels_and_workshops(self):
        self.assertEqual("食品加工车间", category_config("食品")["workshop"])
        self.assertEqual("日用品大类", category_config("日用品")["label"])
        self.assertEqual("电子产品生产车间", category_config("电子产品")["workshop"])

    def test_unknown_category_is_rejected(self):
        with self.assertRaises(ValueError):
            category_config("玩具")

    def test_speech_uses_exact_competition_template(self):
        text = format_result_speech(
            physical_item="苹果",
            physical_category="食品",
            simulation_item="毛巾",
            simulation_category="日用品",
        )
        self.assertEqual(
            "取得苹果属于食品大类应放置在食品加工车间，"
            "仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间",
            text,
        )


if __name__ == "__main__":
    unittest.main()
```

- [ ] **步骤 2：运行测试并确认导入失败**

```powershell
python ucar_ws/src/task_orchestrator/test/test_categories.py -v
```

预期结果：测试失败，提示缺少 `task_orchestrator.categories`。

测试文件通过自身位置计算 `src` 目录并加入 `sys.path`。这样可以避开
Windows Python 在处理含中文父目录的 `PYTHONPATH` 时出现的编码问题。

- [ ] **步骤 3：实现 `categories.py`**

```python
_CATEGORY_CONFIG = {
    "食品": {"label": "食品大类", "workshop": "食品加工车间"},
    "日用品": {"label": "日用品大类", "workshop": "日用品加工车间"},
    "电子产品": {
        "label": "电子产品大类",
        "workshop": "电子产品生产车间",
    },
}


def category_config(category):
    if category not in _CATEGORY_CONFIG:
        raise ValueError("unsupported category: %s" % category)
    return dict(_CATEGORY_CONFIG[category])


def format_result_speech(
    physical_item,
    physical_category,
    simulation_item,
    simulation_category,
):
    physical = category_config(physical_category)
    simulation = category_config(simulation_category)
    return (
        "取得%s属于%s应放置在%s，仿真环境中取得%s属于%s应放置在%s"
        % (
            physical_item,
            physical["label"],
            physical["workshop"],
            simulation_item,
            simulation["label"],
            simulation["workshop"],
        )
    )
```

- [ ] **步骤 4：运行类别测试**

预期结果：3 项测试全部通过。

- [ ] **步骤 5：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/categories.py \
  ucar_ws/src/task_orchestrator/test/test_categories.py
git commit -m "feat: add trusted task category mapping"
```

## 任务 3：解析固定格式的语音指令

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/src/task_orchestrator/voice_parser.py`
- 新建：`ucar_ws/src/task_orchestrator/test/test_voice_parser.py`

- [ ] **步骤 1：编写解析器测试**

```python
import unittest
import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_ROOT))

from task_orchestrator.voice_parser import parse_categories


class VoiceParserTests(unittest.TestCase):
    def test_extracts_two_categories_in_order(self):
        result = parse_categories(
            "小飞小飞，前往物品领取区，取得食品，放置在对应仓库，"
            "并领取仿真环境中需要的日用品放置在对应仓库"
        )
        self.assertEqual(("食品", "日用品"), result)

    def test_supports_electronic_products(self):
        result = parse_categories(
            "取得电子产品，并领取仿真环境中需要的食品"
        )
        self.assertEqual(("电子产品", "食品"), result)

    def test_rejects_one_category_only(self):
        with self.assertRaises(ValueError):
            parse_categories("前往物品领取区，取得食品")

    def test_rejects_ambiguous_extra_category(self):
        with self.assertRaises(ValueError):
            parse_categories("取得食品和日用品，仿真环境需要电子产品")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **步骤 2：运行测试并确认失败**

```powershell
python ucar_ws/src/task_orchestrator/test/test_voice_parser.py -v
```

预期结果：提示缺少 `voice_parser`。

- [ ] **步骤 3：实现最小化、确定性的解析器**

```python
import re


_CATEGORY_PATTERN = re.compile("电子产品|日用品|食品")


def parse_categories(text):
    if not isinstance(text, str) or not text.strip():
        raise ValueError("voice text must be non-empty")
    matches = _CATEGORY_PATTERN.findall(text)
    if len(matches) != 2:
        raise ValueError("voice instruction must contain exactly two categories")
    return matches[0], matches[1]
```

- [ ] **步骤 4：运行测试**

```powershell
python ucar_ws/src/task_orchestrator/test/test_voice_parser.py -v
```

预期结果：4 项测试全部通过。

- [ ] **步骤 5：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/voice_parser.py \
  ucar_ws/src/task_orchestrator/test/test_voice_parser.py
git commit -m "feat: parse dual-category voice commands"
```

## 任务 4：实现协议解析和消息身份校验

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py`
- 新建：`ucar_ws/src/task_orchestrator/test/test_protocol.py`

需要实现的解析函数：

```python
parse_task_request(raw_json)
parse_arrival(raw_json, expected_task_id, expected_goal_id)
parse_qr_result(raw_json, expected_task_id, expected_search_id)
parse_llm_result(raw_json, context)
parse_speech_done(raw_json, expected_task_id, expected_speech_id)
parse_cancel(raw_json, expected_task_id)
```

- [ ] **步骤 1：为合法任务消息和二维码消息编写测试**

测试必须验证：

- `protocol_version` 必须是整数 `1`，布尔值 `true` 也不能冒充版本号；
- 类别属于可信类别集合；
- 二维码 `complete` 结果恰好包含三个互不重复的顺序号、名称和 URL；
- QR 的 `searching` 只表示中间状态，不会被误判为完成；
- LLM 选择的 order、名称、类别和车间必须能与本地上下文相互校验；
- `failed` 或 `error` 状态必须附带非空错误信息；
- 身份标识不匹配的消息会被拒绝。

- [ ] **步骤 2：运行测试并确认失败**

```powershell
python ucar_ws/src/task_orchestrator/test/test_protocol.py -v
```

预期结果：提示缺少协议解析函数。

- [ ] **步骤 3：实现小型校验辅助函数**

使用以下基础函数：

```python
def load_object(raw_json):
    value = json.loads(raw_json)
    if not isinstance(value, dict):
        raise ProtocolError("message must be a JSON object")
    if value.get("protocol_version") != 1:
        raise ProtocolError("unsupported protocol version")
    return value


def require_text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("%s must be non-empty" % field)
    return value.strip()
```

- [ ] **步骤 4：逐个实现解析函数**

每完成一个解析函数，只运行对应测试。不要在一次编辑中同时实现全部解析器。

- [ ] **步骤 5：运行完整协议测试集**

```powershell
python ucar_ws/src/task_orchestrator/test/test_protocol.py -v
```

预期结果：全部协议测试在 Python 3.7 下通过。

- [ ] **步骤 6：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py \
  ucar_ws/src/task_orchestrator/test/test_protocol.py
git commit -m "feat: validate orchestrator protocol messages"
```

## 任务 5：实现纯 Python 任务编排状态机

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- 新建：`ucar_ws/src/task_orchestrator/test/test_orchestrator.py`

核心状态机返回动作元组，不直接发布 ROS topic：

```python
("publish_status", payload)
("publish_pickup_goal", payload)
("publish_qr_start", payload)
("publish_qr_stop", payload)
("publish_llm_request", payload)
("publish_speech", payload)
("publish_delivery_goal", payload)
```

- [ ] **步骤 1：测试完整成功路径**

依次输入：

1. 合法的双母类任务请求；
2. 依赖模块全部 ready；
3. 到达物品领取区观察点；
4. QR 返回三个完整候选；
5. LLM 成功结果；
6. 播报成功完成；
7. 到达实物目标车间。

测试消息必须使用任务 4 已确认的正式字段。例如任务请求使用
`physical_target_category` 和 `simulation_target_category`，QR 候选使用
`order` 和 `item_name`，LLM 结果同时包含 `physical` 与 `simulation`。

断言状态依次为：

```text
CHECKING_DEPENDENCIES
NAVIGATING_TO_PICKUP
WAITING_QR
WAITING_LLM
WAITING_SPEECH
NAVIGATING_TO_WORKSHOP
COMPLETE
```

- [ ] **步骤 2：运行测试并确认失败**

```powershell
python ucar_ws/src/task_orchestrator/test/test_orchestrator.py -v
```

预期结果：提示缺少 `TaskOrchestrator`。

- [ ] **步骤 3：实现构造函数和任务接收逻辑**

构造函数参数：

```python
TaskOrchestrator(outputs, clock, id_factory, timeouts)
```

保存以下状态：

```python
self.state = "IDLE"
self.task = None
self.deadline = None
self.last_status = None
```

- [ ] **步骤 4：逐个实现状态转换**

每次状态转换都必须：

1. 校验当前状态；
2. 校验当前任务和阶段 identity；
3. 只改变一次状态；
4. 设置下一阶段的独立 deadline；
5. 返回需要发布的动作。

- [ ] **步骤 5：添加失败路径测试**

覆盖以下情况：

- QR `not_found`;
- LLM 返回错误；
- TTS 返回错误；
- 导航失败；
- 每个阶段分别超时。

- [ ] **步骤 6：添加并发安全和幂等行为测试**

覆盖以下情况：

- 重复的相同 `task_id`；
- 忙碌时收到不同 `task_id`；
- 过期 `search_id`；
- 过期 `request_id`；
- 过期 `speech_id`；
- 重复收到成功结果；
- 在每个活动状态中取消任务。

- [ ] **步骤 7：运行完整纯 Python 测试集**

```powershell
python -m unittest discover `
  -s ucar_ws/src/task_orchestrator/test `
  -p "test_*.py" -v
```

预期结果：不导入 `rospy`，全部测试通过。

- [ ] **步骤 8：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py \
  ucar_ws/src/task_orchestrator/test/test_orchestrator.py
git commit -m "feat: add task orchestration state machine"
```

## 任务 6：添加配置和 ROS 节点适配层

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- 新建：`ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- 新建：`ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`
- 修改：`ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **步骤 1：添加配置文件**

```yaml
timeouts:
  dependency_ready: 30.0
  pickup_navigation: 300.0
  qr_search: 90.0
  llm_classification: 60.0
  speech: 60.0
  delivery_navigation: 300.0
  cancel_ack: 15.0
```

- [ ] **步骤 2：扩展包结构测试**

使用 Python `ast` 和 XML 解析来校验准确的 topic 名称，并确认所有发布者和订阅者都使用 `std_msgs/String`。

- [ ] **步骤 3：实现轻量 ROS 输入输出层**

`task_orchestrator_node.py` 必须：

- 发布者只创建一次；
- 订阅者只创建一次；
- ROS 层自身不解析业务字段；
- 把回调事件交给纯 Python 核心；
- call `tick(rospy.get_time())` from a timer;
- 捕获回调异常并发布错误状态；
- never execute `roslaunch`, `rosrun`, or `rosnode kill`.

- [ ] **步骤 4：创建 launch 文件**

当任务 6～8 中的三个节点脚本全部创建后，在 `CMakeLists.txt` 末尾添加：

```cmake
catkin_install_python(PROGRAMS
  scripts/task_orchestrator_node.py
  scripts/voice_task_adapter_node.py
  scripts/tts_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

```xml
<launch>
  <rosparam command="load" file="$(find task_orchestrator)/config/orchestrator.yaml"/>
  <node pkg="task_orchestrator"
        type="task_orchestrator_node.py"
        name="task_orchestrator"
        output="screen"/>
</launch>
```

- [ ] **步骤 5：运行包测试和 `py_compile`**

```bash
python3 -m py_compile scripts/task_orchestrator_node.py
python3 -m unittest discover -s test -p 'test_*.py' -v
```

- [ ] **步骤 6：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "feat: expose orchestrator ROS topics"
```

## 任务 7：添加语音任务适配器

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/scripts/voice_task_adapter_node.py`
- 修改：`ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **步骤 1：添加适配器 topic 测试**

断言：

- subscriber `/question`;
- publisher `/voice/task_request`;
- both use `std_msgs/String`.

- [ ] **步骤 2：实现回调行为**

每次收到非空 `/question` 时：

1. call `parse_categories`;
2. generate one UUID task ID;
3. publish protocol-v1 JSON;
4. reject text that does not contain exactly two categories;
5. suppress an identical text received again within a configurable short debounce window.

- [ ] **步骤 3：运行单元测试和包测试**

预期结果：解析器测试和 AST 测试全部通过。

- [ ] **步骤 4：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator/scripts/voice_task_adapter_node.py \
  ucar_ws/src/task_orchestrator/test
git commit -m "feat: adapt recognized speech into task requests"
```

## 任务 8：添加 TTS 桥接节点

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/scripts/tts_bridge_node.py`
- 新建：`ucar_ws/src/task_orchestrator/src/task_orchestrator/tts_runner.py`
- 新建：`ucar_ws/src/task_orchestrator/test/test_tts_runner.py`

- [ ] **步骤 1：在不调用真实音频设备的情况下测试命令执行**

注入一个可调用的运行器并断言：

- 传入的文本完全一致；
- 退出码为零时转换成 `success`；
- 非零退出码和超时转换成 `error`；
- one `speech_id` is never played twice.

- [ ] **步骤 2：实现带超时限制的子进程运行器**

调用方式：

```text
/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py <text>
```

使用参数列表，不使用 `shell=True`。超时时间从 ROS 参数读取。

- [ ] **步骤 3：发布 `/voice/speak_done`**

消息必须包含：

```json
{
  "protocol_version": 1,
  "task_id": "...",
  "speech_id": "...",
  "status": "success",
  "message": ""
}
```

- [ ] **步骤 4：运行测试**

任何测试都不能访问网络、扬声器或真实 TTS 进程。

- [ ] **步骤 5：让 Codex 审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "feat: bridge orchestrator speech to existing TTS"
```

## 任务 9：完成 ROS 模拟端到端联调

**涉及文件：**
- 新建：`ucar_ws/src/task_orchestrator/test/manual_simulation.md`
- 修改：`ucar_ws/src/task_orchestrator/README.md`

- [ ] **步骤 1：启动任务编排器包**

```bash
roslaunch task_orchestrator task_orchestrator.launch
```

- [ ] **步骤 2：模拟各个外部模块**

使用不同终端监听输出 topic，并发布以下模拟消息：

- `/question`;
- `/task/pickup_arrived`;
- `/qr_item_search/result`;
- `/llm/classify/result`;
- `/voice/speak_done`;
- `/task/delivery_arrived`.

全程使用同一个 `task_id`，其他 ID 使用编排器实际发布出来的值。

- [ ] **步骤 3：核对严格格式的播报文本**

预期结果：

```text
取得苹果属于食品大类应放置在食品加工车间，仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间
```

- [ ] **步骤 4：核对实物配送导航目标**

确认 `/task/delivery_navigation_goal` 只包含实物目标车间和实物名称。

- [ ] **步骤 5：验证失败路径**

分别使用以下情况重复测试：

- QR `not_found`;
- LLM 返回 `error`；
- TTS 返回 `error`；
- 过期 ID；
- 取消任务。

任何失败路径都不得发布配送导航目标。

- [ ] **步骤 6：让 Codex 完成核心版本最终审查并提交**

建议提交命令：

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "test: document orchestrator integration simulation"
```

## 后续实施计划

本计划全部通过后：

1. Upgrade `llm_spark` to dual-target protocol and remove hard-coded credentials from tracked source.
2. Integrate real pickup navigation and define its cancel/arrival contract.
3. Integrate `speech_command` and verify the full competition sentence on the real microphone.
4. Integrate TTS audio completion and obstacle-navigation delivery goal.
