# Task Orchestrator Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a tested ROS-independent task orchestration core, ROS topic adapter, voice text adapter, and TTS bridge that can complete the full workflow with simulated navigation, QR, LLM, and TTS modules.

**Architecture:** Business rules live in pure Python modules with no `rospy` dependency. Thin ROS scripts translate `std_msgs/String` JSON messages into state-machine events and publish returned actions. Existing speech hardware remains the sole microphone/serial owner; adapters consume `/question` and call the existing TTS script without restarting `speech_command_node`.

**Tech Stack:** ROS 1 Noetic, Python 3.7, `rospy`, `std_msgs/String`, JSON protocol v1, `unittest`, Catkin.

---

## Scope

This plan implements the orchestrator package and proves the complete flow with mock topics. It does not yet modify the deployed `llm_spark` source or navigation algorithms. Those integrations get separate plans after the core protocol is stable.

## File map

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

## Task 1: Create the Catkin package skeleton

**Files:**
- Create: `ucar_ws/src/task_orchestrator/package.xml`
- Create: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Create: `ucar_ws/src/task_orchestrator/setup.py`
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/__init__.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **Step 1: Create the package directories**

From the new worktree root:

```powershell
New-Item -ItemType Directory -Force `
  ucar_ws/src/task_orchestrator/src/task_orchestrator, `
  ucar_ws/src/task_orchestrator/scripts, `
  ucar_ws/src/task_orchestrator/launch, `
  ucar_ws/src/task_orchestrator/config, `
  ucar_ws/src/task_orchestrator/test
```

Expected: five directories exist under `ucar_ws/src/task_orchestrator`.

- [ ] **Step 2: Write the first package-structure test**

Create `test/test_package_config.py`:

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

- [ ] **Step 3: Run the test and observe failure**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

Expected: FAIL because package files do not exist.

- [ ] **Step 4: Create `package.xml`**

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

- [ ] **Step 5: Create `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(task_orchestrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/task_orchestrator_node.py
  scripts/voice_task_adapter_node.py
  scripts/tts_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

- [ ] **Step 6: Create `setup.py` and `__init__.py`**

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

- [ ] **Step 7: Re-run the package test**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

Expected: PASS.

- [ ] **Step 8: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "build: scaffold task orchestrator package"
```

## Task 2: Implement trusted category mapping and speech formatting

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/categories.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_categories.py`

- [ ] **Step 1: Write category tests**

```python
import unittest

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

- [ ] **Step 2: Run the test and observe import failure**

```powershell
$env:PYTHONPATH='ucar_ws/src/task_orchestrator/src'
python -m unittest ucar_ws/src/task_orchestrator/test/test_categories.py -v
```

Expected: FAIL with missing `task_orchestrator.categories`.

- [ ] **Step 3: Implement `categories.py`**

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

- [ ] **Step 4: Run the category tests**

Expected: 3 tests PASS.

- [ ] **Step 5: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/categories.py \
  ucar_ws/src/task_orchestrator/test/test_categories.py
git commit -m "feat: add trusted task category mapping"
```

## Task 3: Parse the fixed voice instruction

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/voice_parser.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_voice_parser.py`

- [ ] **Step 1: Write parser tests**

```python
import unittest

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

- [ ] **Step 2: Run and observe failure**

Expected: missing `voice_parser`.

- [ ] **Step 3: Implement the minimal deterministic parser**

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

- [ ] **Step 4: Run the tests**

Expected: 4 tests PASS.

- [ ] **Step 5: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/voice_parser.py \
  ucar_ws/src/task_orchestrator/test/test_voice_parser.py
git commit -m "feat: parse dual-category voice commands"
```

## Task 4: Implement protocol parsing and identity validation

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_protocol.py`

Required parsers:

```python
parse_task_request(raw_json)
parse_arrival(raw_json, expected_task_id, expected_goal_id)
parse_qr_result(raw_json, expected_task_id, expected_search_id)
parse_llm_result(raw_json, context)
parse_speech_done(raw_json, expected_task_id, expected_speech_id)
parse_cancel(raw_json, expected_task_id)
```

- [ ] **Step 1: Write tests for valid task and QR messages**

Tests must assert:

- protocol version is integer `1`;
- categories are trusted values;
- QR `complete` contains exactly three distinct orders, names and URLs;
- mismatched identity is rejected.

- [ ] **Step 2: Run tests and verify failure**

Expected: missing protocol functions.

- [ ] **Step 3: Implement small validation helpers**

Use these exact primitives:

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

- [ ] **Step 4: Implement each parser one at a time**

After each parser, run only its tests. Do not implement all parsers in one edit.

- [ ] **Step 5: Run the full protocol suite**

Expected: all protocol tests PASS on Python 3.7.

- [ ] **Step 6: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py \
  ucar_ws/src/task_orchestrator/test/test_protocol.py
git commit -m "feat: validate orchestrator protocol messages"
```

## Task 5: Implement the pure orchestration state machine

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_orchestrator.py`

The core returns action tuples instead of publishing ROS directly:

```python
("publish_status", payload)
("publish_pickup_goal", payload)
("publish_qr_start", payload)
("publish_qr_stop", payload)
("publish_llm_request", payload)
("publish_speech", payload)
("publish_delivery_goal", payload)
```

- [ ] **Step 1: Test the happy path**

Feed:

1. task request;
2. dependencies ready;
3. pickup arrived;
4. QR complete;
5. LLM success;
6. speech done;
7. delivery arrived.

Assert states:

```text
CHECKING_DEPENDENCIES
NAVIGATING_TO_PICKUP
WAITING_QR
WAITING_LLM
WAITING_SPEECH
NAVIGATING_TO_WORKSHOP
COMPLETE
```

- [ ] **Step 2: Run and verify failure**

Expected: missing `TaskOrchestrator`.

- [ ] **Step 3: Implement constructor and task acceptance**

Constructor inputs:

```python
TaskOrchestrator(outputs, clock, id_factory, timeouts)
```

Store:

```python
self.state = "IDLE"
self.task = None
self.deadline = None
self.last_status = None
```

- [ ] **Step 4: Implement one transition at a time**

For every transition:

1. validate current state;
2. validate identities;
3. mutate state once;
4. set the next deadline;
5. return actions.

- [ ] **Step 5: Add failure tests**

Cover:

- QR `not_found`;
- LLM error;
- TTS error;
- navigation failure;
- each stage timeout.

- [ ] **Step 6: Add concurrency-safety behavior tests**

Cover:

- duplicate same `task_id`;
- different `task_id` while busy;
- stale `search_id`;
- stale `request_id`;
- stale `speech_id`;
- repeated success result;
- cancellation in every active state.

- [ ] **Step 7: Run the complete pure-Python suite**

Expected: all tests PASS without importing `rospy`.

- [ ] **Step 8: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py \
  ucar_ws/src/task_orchestrator/test/test_orchestrator.py
git commit -m "feat: add task orchestration state machine"
```

## Task 6: Add configuration and ROS node adapter

**Files:**
- Create: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- Create: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- Create: `ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **Step 1: Add configuration**

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

- [ ] **Step 2: Extend package tests**

Use Python `ast` and XML parsing to assert exact topic names and that all publishers/subscribers use `std_msgs/String`.

- [ ] **Step 3: Implement thin ROS outputs**

`task_orchestrator_node.py` must:

- create publishers once;
- create subscribers once;
- parse no business fields itself;
- delegate callbacks to the pure core;
- call `tick(rospy.get_time())` from a timer;
- catch callback exceptions and publish an error status;
- never execute `roslaunch`, `rosrun`, or `rosnode kill`.

- [ ] **Step 4: Create launch file**

```xml
<launch>
  <rosparam command="load" file="$(find task_orchestrator)/config/orchestrator.yaml"/>
  <node pkg="task_orchestrator"
        type="task_orchestrator_node.py"
        name="task_orchestrator"
        output="screen"/>
</launch>
```

- [ ] **Step 5: Run package tests and `py_compile`**

```bash
python3 -m py_compile scripts/task_orchestrator_node.py
python3 -m unittest discover -s test -p 'test_*.py' -v
```

- [ ] **Step 6: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "feat: expose orchestrator ROS topics"
```

## Task 7: Add the voice task adapter

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/voice_task_adapter_node.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [ ] **Step 1: Add an adapter topic test**

Assert:

- subscriber `/question`;
- publisher `/voice/task_request`;
- both use `std_msgs/String`.

- [ ] **Step 2: Implement callback behavior**

On each non-empty `/question`:

1. call `parse_categories`;
2. generate one UUID task ID;
3. publish protocol-v1 JSON;
4. reject text that does not contain exactly two categories;
5. suppress an identical text received again within a configurable short debounce window.

- [ ] **Step 3: Run unit and package tests**

Expected: parser and AST tests PASS.

- [ ] **Step 4: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator/scripts/voice_task_adapter_node.py \
  ucar_ws/src/task_orchestrator/test
git commit -m "feat: adapt recognized speech into task requests"
```

## Task 8: Add the TTS bridge

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/tts_bridge_node.py`
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/tts_runner.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_tts_runner.py`

- [ ] **Step 1: Test command execution without invoking real audio**

Inject a runner callable and assert:

- exact text is passed;
- exit code zero becomes `success`;
- nonzero exit code and timeout become `error`;
- one `speech_id` is never played twice.

- [ ] **Step 2: Implement a bounded subprocess runner**

Invoke:

```text
/home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py <text>
```

Use an argument list, not `shell=True`. Timeout comes from ROS parameters.

- [ ] **Step 3: Publish `/voice/speak_done`**

Include:

```json
{
  "protocol_version": 1,
  "task_id": "...",
  "speech_id": "...",
  "status": "success",
  "message": ""
}
```

- [ ] **Step 4: Run tests**

No test may call the network, speaker, or real TTS process.

- [ ] **Step 5: Ask Codex to review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "feat: bridge orchestrator speech to existing TTS"
```

## Task 9: Complete simulated end-to-end ROS integration

**Files:**
- Create: `ucar_ws/src/task_orchestrator/test/manual_simulation.md`
- Modify: `ucar_ws/src/task_orchestrator/README.md`

- [ ] **Step 1: Start the orchestrator package**

```bash
roslaunch task_orchestrator task_orchestrator.launch
```

- [ ] **Step 2: Simulate each external module**

Use separate terminals to echo outgoing topics and publish:

- `/question`;
- `/task/pickup_arrived`;
- `/qr_item_search/result`;
- `/llm/classify/result`;
- `/voice/speak_done`;
- `/task/delivery_arrived`.

Use one consistent `task_id` and the IDs emitted by the orchestrator.

- [ ] **Step 3: Verify the exact speech text**

Expected:

```text
取得苹果属于食品大类应放置在食品加工车间，仿真环境中取得毛巾属于日用品大类应放置在日用品加工车间
```

- [ ] **Step 4: Verify delivery goal scope**

Assert `/task/delivery_navigation_goal` contains only the physical workshop and physical item.

- [ ] **Step 5: Verify failures**

Repeat with:

- QR `not_found`;
- LLM `error`;
- TTS `error`;
- stale IDs;
- cancel.

No failed path may publish a delivery goal.

- [ ] **Step 6: Ask Codex for final core review and commit**

Suggested commit:

```bash
git add ucar_ws/src/task_orchestrator
git commit -m "test: document orchestrator integration simulation"
```

## Follow-up plans

After this plan passes:

1. Upgrade `llm_spark` to dual-target protocol and remove hard-coded credentials from tracked source.
2. Integrate real pickup navigation and define its cancel/arrival contract.
3. Integrate `speech_command` and verify the full competition sentence on the real microphone.
4. Integrate TTS audio completion and obstacle-navigation delivery goal.
