# Phase 3 Line-Follow Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the verified two-phase U-CAR workflow so it navigates to a YAML-configured line start, waits for a traffic-light direction, follows the selected route through the sole velocity arbiter, stops at the final line, and speaks one globally owned completion message.

**Architecture:** Add a focused `line_follow_integration` ROS package containing a camera adapter, a move_base adapter, a line-process supervisor, and frozen copies of the proven YOLO/route scripts. Extend `task_orchestrator` only with Phase 3 states and protocol wiring, and extend the existing arbiter with one isolated `/cmd_vel/line_follow` source. Keep the physical 1020x720 camera, base, localization, move_base, and TTS single-owned.

**Tech Stack:** ROS 1/catkin, Python 3, rospy, actionlib/move_base, sensor_msgs/cv_bridge/OpenCV, std_msgs JSON protocol v1, geometry_msgs/Twist, unittest, XML/YAML structural tests.

---

## File structure

Create the following focused package:

```text
ucar_ws/src/line_follow_integration/
  CMakeLists.txt                         catkin packaging and script installation
  package.xml                           ROS runtime dependencies
  setup.py                              Python package installation
  SOURCE_SNAPSHOT.sha256                exact proven source hashes
  config/phase3.yaml                    line-start pose, image, timeout defaults
  launch/phase3.launch                  three integration nodes; no hardware owner
  scripts/line_camera_adapter_node.py   gated 1020x720 -> 640x480 line image
  scripts/line_navigation_adapter_node.py move_base goal, settle, arrival result
  scripts/line_follow_supervisor_node.py direction/process ownership and status
  scripts/yolo_server.py                exact vehicle snapshot
  scripts/follow_left_v4.py             exact E: snapshot
  scripts/follow_right_v4.py            exact E: snapshot
  scripts/follow_mid_v4.py              exact E: snapshot
  src/line_follow_integration/__init__.py
  src/line_follow_integration/camera_transform.py pure crop/resize logic
  src/line_follow_integration/protocol.py pure Phase 3 JSON validation/builders
  src/line_follow_integration/runtime.py pure fresh-file and route/session logic
  test/test_package_config.py
  test/test_camera_transform.py
  test/test_protocol.py
  test/test_runtime.py
  test/test_ros_contract.py
```

Modify these existing files:

```text
ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py
ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py
ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py
ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py
ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py
ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py
ucar_ws/src/task_orchestrator/config/orchestrator.yaml
ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch
ucar_ws/src/task_orchestrator/launch/competition_full.launch
ucar_ws/src/task_orchestrator/scripts/start_competition.sh
ucar_ws/src/task_orchestrator/CMakeLists.txt
ucar_ws/src/task_orchestrator/package.xml
ucar_ws/src/task_orchestrator/test/test_orchestrator.py
ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py
ucar_ws/src/task_orchestrator/test/test_motion_mode.py
ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py
ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py
ucar_ws/src/task_orchestrator/test/test_package_config.py
ucar_ws/src/task_orchestrator/test/test_competition_bringup.py
ucar_ws/src/task_orchestrator/test/manual_simulation.md
ucar_ws/src/task_orchestrator/README.md
ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md
```

Do not modify `stop` algorithms during Phase 3 implementation. The vehicle-synchronized third workshop waypoint is already frozen by commit `7b61069` and its characterization test.

### Task 1: Create the Phase 3 package and configuration contract

**Files:**
- Create: `ucar_ws/src/line_follow_integration/test/test_package_config.py`
- Create: `ucar_ws/src/line_follow_integration/CMakeLists.txt`
- Create: `ucar_ws/src/line_follow_integration/package.xml`
- Create: `ucar_ws/src/line_follow_integration/setup.py`
- Create: `ucar_ws/src/line_follow_integration/src/line_follow_integration/__init__.py`
- Create: `ucar_ws/src/line_follow_integration/config/phase3.yaml`
- Create: `ucar_ws/src/line_follow_integration/launch/phase3.launch`

- [ ] **Step 1: Write the failing package/configuration test**

Create `test/test_package_config.py` with assertions equivalent to:

```python
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class PackageConfigTests(unittest.TestCase):
    def test_required_files_exist(self):
        for relative in (
            "package.xml", "CMakeLists.txt", "setup.py",
            "config/phase3.yaml", "launch/phase3.launch",
            "src/line_follow_integration/__init__.py",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_phase3_defaults_are_exact(self):
        config = yaml.safe_load(
            (ROOT / "config/phase3.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual({
            "frame_id": "map",
            "x": 0.5167081260031493,
            "y": -3.125171302690142,
            "qz": -0.7044294777312331,
            "qw": 0.7097739857893512,
        }, config["line_start_goal"])
        self.assertEqual("/usb_cam/image_raw", config["camera"]["input_topic"])
        self.assertEqual("/line_follow/image_raw", config["camera"]["line_topic"])
        self.assertEqual([640, 480, 15.0, "center_4_3"], [
            config["camera"]["output_width"],
            config["camera"]["output_height"],
            config["camera"]["output_fps"],
            config["camera"]["crop_mode"],
        ])
        self.assertEqual({
            "navigation": 300.0,
            "image_ready": 5.0,
            "image_max_age": 0.5,
            "image_recovery_grace": 3.0,
            "direction": 30.0,
            "line_follow": 120.0,
        }, config["timeouts"])

    def test_launch_does_not_own_shared_hardware(self):
        root = ET.parse(ROOT / "launch/phase3.launch").getroot()
        text = ET.tostring(root, encoding="unicode")
        for forbidden in ("usb_cam_node", "base_driver", "ydlidar", "map_server", "amcl"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
```

Expected: FAIL because the package files do not exist.

- [ ] **Step 3: Add the minimal catkin package and exact YAML**

Create `config/phase3.yaml` exactly as follows:

```yaml
line_start_goal:
  frame_id: map
  x: 0.5167081260031493
  y: -3.125171302690142
  qz: -0.7044294777312331
  qw: 0.7097739857893512

camera:
  input_topic: /usb_cam/image_raw
  line_topic: /line_follow/image_raw
  output_width: 640
  output_height: 480
  output_fps: 15.0
  crop_mode: center_4_3

timeouts:
  navigation: 300.0
  image_ready: 5.0
  image_max_age: 0.5
  image_recovery_grace: 3.0
  direction: 30.0
  line_follow: 120.0

runtime:
  yolo_result_file: /tmp/yolo_result.txt
  stop_done_file: /tmp/stop_done.txt
  yolo_model: /home/ucar/ucar_ws/src/yolo_turn/best.pt
  yolo_model_sha256: cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd
```

Create a normal `catkin_python_setup()` package depending on `actionlib`, `actionlib_msgs`, `cv_bridge`, `geometry_msgs`, `move_base_msgs`, `nav_msgs`, `rospy`, `sensor_msgs`, and `std_msgs`. Register `catkin_add_nosetests(test)` and install `launch` and `config`. Add a syntactically valid `phase3.launch` containing the three node declarations disabled only by explicit `enable_*` arguments; do not include any hardware launch.

- [ ] **Step 4: Run package tests and XML parsing**

Run:

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -c "import xml.etree.ElementTree as ET; ET.parse(r'ucar_ws/src/line_follow_integration/launch/phase3.launch')"
```

Expected: all current tests PASS and XML parsing exits 0.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/line_follow_integration/CMakeLists.txt ucar_ws/src/line_follow_integration/package.xml ucar_ws/src/line_follow_integration/setup.py ucar_ws/src/line_follow_integration/config/phase3.yaml ucar_ws/src/line_follow_integration/launch/phase3.launch ucar_ws/src/line_follow_integration/src/line_follow_integration/__init__.py ucar_ws/src/line_follow_integration/test/test_package_config.py
git commit -m "feat(phase3): add line-follow package configuration"
```

### Task 2: Implement the pure line-camera transform

**Files:**
- Create: `ucar_ws/src/line_follow_integration/test/test_camera_transform.py`
- Create: `ucar_ws/src/line_follow_integration/src/line_follow_integration/camera_transform.py`

- [ ] **Step 1: Write failing pure image-transform tests**

```python
import unittest
import numpy as np

from line_follow_integration.camera_transform import center_crop_4_3, transform_line_frame


class CameraTransformTests(unittest.TestCase):
    def test_1020x720_center_crop_is_960x720(self):
        frame = np.zeros((720, 1020, 3), dtype=np.uint8)
        frame[:, :, 0] = np.arange(1020, dtype=np.uint16) % 256
        cropped = center_crop_4_3(frame)
        self.assertEqual((720, 960, 3), cropped.shape)
        np.testing.assert_array_equal(frame[:, 30:990], cropped)

    def test_transform_returns_640x480_without_mutating_input(self):
        frame = np.random.default_rng(7).integers(0, 256, (720, 1020, 3), dtype=np.uint8)
        original = frame.copy()
        output = transform_line_frame(frame, 640, 480, "center_4_3")
        self.assertEqual((480, 640, 3), output.shape)
        np.testing.assert_array_equal(original, frame)

    def test_invalid_shape_and_mode_fail_closed(self):
        with self.assertRaises(ValueError):
            center_crop_4_3(np.zeros((10, 10), dtype=np.uint8))
        with self.assertRaises(ValueError):
            transform_line_frame(np.zeros((10, 10, 3), dtype=np.uint8), 640, 480, "stretch")
```

- [ ] **Step 2: Run the focused test and verify RED**

```powershell
$env:PYTHONPATH='ucar_ws/src/line_follow_integration/src'
python -m unittest ucar_ws/src/line_follow_integration/test/test_camera_transform.py -v
```

Expected: import failure for `camera_transform`.

- [ ] **Step 3: Implement the pure transform**

```python
import cv2


def center_crop_4_3(frame):
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be HxWx3")
    height, width = frame.shape[:2]
    target_width = int(round(height * 4.0 / 3.0))
    if width < target_width:
        raise ValueError("frame is narrower than 4:3")
    left = (width - target_width) // 2
    return frame[:, left:left + target_width].copy()


def transform_line_frame(frame, width, height, crop_mode):
    if crop_mode != "center_4_3":
        raise ValueError("unsupported crop_mode: %s" % crop_mode)
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("output dimensions must be positive integers")
    return cv2.resize(center_crop_4_3(frame), (width, height), interpolation=cv2.INTER_AREA)
```

- [ ] **Step 4: Run GREEN**

Run:

```powershell
$env:PYTHONPATH='ucar_ws/src/line_follow_integration/src'
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/line_follow_integration/src/line_follow_integration/camera_transform.py ucar_ws/src/line_follow_integration/test/test_camera_transform.py
git commit -m "feat(phase3): add line-camera transform"
```

### Task 3: Define Phase 3 protocol and fresh-result runtime logic

**Files:**
- Create: `ucar_ws/src/line_follow_integration/test/test_protocol.py`
- Create: `ucar_ws/src/line_follow_integration/test/test_runtime.py`
- Create: `ucar_ws/src/line_follow_integration/src/line_follow_integration/protocol.py`
- Create: `ucar_ws/src/line_follow_integration/src/line_follow_integration/runtime.py`

- [ ] **Step 1: Write failing protocol and freshness tests**

The tests must cover exact identity, finite pose values, status enums, stale files, red-light waiting, three route mappings, the 30-second straight fallback, fresh stop markers, child exit, 120-second timeout, candidate-speed blocking on stale images, 3-second recovery, and terminal image failure. Use a temporary directory and an injected clock.

```python
import os
import tempfile
import unittest
from pathlib import Path

from line_follow_integration.runtime import ImageHealthGate, LineFollowSession


class FakeClock:
    def __init__(self, value):
        self.value = float(value)

    def __call__(self):
        return self.value


def write_fresh(path, text, timestamp):
    path.write_text(text, encoding="utf-8")
    os.utime(path, (timestamp, timestamp))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_direction_session_waits_for_red_then_locks_left(self):
        clock = FakeClock(100.0)
        result = self.root / "yolo.txt"
        session = LineFollowSession(str(result), str(self.root / "done"), clock, 30.0, 120.0)
        session.start("task-1", "line-1")
        write_fresh(result, "red_light", 101.0)
        self.assertEqual(("waiting_signal", None), session.poll_direction())
        write_fresh(result, "left_turn", 102.0)
        self.assertEqual(("direction_selected", "left_turn"), session.poll_direction())
        self.assertEqual(("direction_selected", "left_turn"), session.poll_direction())

    def test_direction_timeout_defaults_to_straight(self):
        clock = FakeClock(10.0)
        session = LineFollowSession(str(self.root / "yolo"), str(self.root / "done"), clock, 30.0, 120.0)
        session.start("task-1", "line-1")
        clock.value = 40.0
        self.assertEqual(("direction_selected", "straight"), session.poll_direction())

    def test_stale_stop_file_never_succeeds(self):
        done = self.root / "done"
        write_fresh(done, "done", 9.0)
        clock = FakeClock(10.0)
        session = LineFollowSession(str(self.root / "yolo"), str(done), clock, 30.0, 120.0)
        session.start("task-1", "line-1")
        session.lock_direction("straight")
        self.assertEqual(("following", None), session.poll_follow(child_running=True))

    def test_image_gate_stops_then_recovers_before_grace(self):
        clock = FakeClock(10.0)
        gate = ImageHealthGate(clock, max_age=0.5, recovery_grace=3.0)
        gate.observe_frame(10.0)
        self.assertTrue(gate.allows_motion())
        clock.value = 10.6
        self.assertEqual("stop", gate.poll())
        self.assertFalse(gate.allows_motion())
        clock.value = 12.0
        gate.observe_frame(12.0)
        self.assertEqual("healthy", gate.poll())
        self.assertTrue(gate.allows_motion())

    def test_image_gate_fails_after_recovery_grace(self):
        clock = FakeClock(20.0)
        gate = ImageHealthGate(clock, max_age=0.5, recovery_grace=3.0)
        gate.observe_frame(20.0)
        clock.value = 20.6
        self.assertEqual("stop", gate.poll())
        clock.value = 23.6
        self.assertEqual("failure", gate.poll())
```

- [ ] **Step 2: Run focused tests and verify RED**

```powershell
$env:PYTHONPATH='ucar_ws/src/line_follow_integration/src'
python -m unittest ucar_ws/src/line_follow_integration/test/test_protocol.py ucar_ws/src/line_follow_integration/test/test_runtime.py -v
```

Expected: imports fail because `protocol.py` and `runtime.py` do not exist.

- [ ] **Step 3: Implement the complete pure interfaces**

Implement `protocol.py` with these complete validation/building rules:

```python
import json
import math


class ProtocolError(ValueError):
    pass


PROTOCOL_VERSION = 1
LINE_STATUSES = frozenset(("waiting_signal", "direction_selected", "following", "success", "failure"))
DIRECTIONS = frozenset(("left_turn", "right_turn", "straight"))


def _load_object(raw):
    try:
        value = json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, ValueError) as exc:
        raise ProtocolError("invalid JSON: %s" % exc)
    if not isinstance(value, dict):
        raise ProtocolError("message must be an object")
    return dict(value)


def _text(value, field):
    if not isinstance(value, str) or not value or value != value.strip():
        raise ProtocolError("%s must be non-blank text without surrounding whitespace" % field)
    return value


def _finite(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("%s must be finite" % field)
    try:
        number = float(value)
    except (OverflowError, TypeError, ValueError):
        raise ProtocolError("%s must be finite" % field)
    if not math.isfinite(number):
        raise ProtocolError("%s must be finite" % field)
    return number


def parse_identity_json(raw):
    value = _load_object(raw)
    if value.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")
    value["task_id"] = _text(value.get("task_id"), "task_id")
    value["goal_id"] = _text(value.get("goal_id"), "goal_id")
    return value


def parse_navigation_goal(raw):
    value = parse_identity_json(raw)
    pose = value.get("pose")
    if not isinstance(pose, dict):
        raise ProtocolError("pose must be an object")
    normalized = {"frame_id": _text(pose.get("frame_id"), "frame_id")}
    for field in ("x", "y", "qz", "qw"):
        normalized[field] = _finite(pose.get(field), field)
    if math.hypot(normalized["qz"], normalized["qw"]) < 1e-9:
        raise ProtocolError("orientation must be non-zero")
    value["pose"] = normalized
    return value


def build_arrival(task_id, goal_id, succeeded, message=""):
    task_id = _text(task_id, "task_id")
    goal_id = _text(goal_id, "goal_id")
    if not isinstance(succeeded, bool):
        raise ProtocolError("succeeded must be boolean")
    if not isinstance(message, str):
        raise ProtocolError("message must be text")
    message = message.strip()
    if not succeeded and not message:
        raise ProtocolError("failed arrival requires message")
    return {
        "protocol_version": 1,
        "task_id": task_id,
        "goal_id": goal_id,
        "status": "arrived" if succeeded else "failed",
        "message": message,
    }


def build_line_status(task_id, goal_id, status, direction=None, reason=""):
    task_id = _text(task_id, "task_id")
    goal_id = _text(goal_id, "goal_id")
    if status not in LINE_STATUSES:
        raise ProtocolError("unknown line status")
    if direction is not None and direction not in DIRECTIONS:
        raise ProtocolError("unknown direction")
    if status in ("direction_selected", "following", "success") and direction is None:
        raise ProtocolError("status requires direction")
    if status == "failure" and not isinstance(reason, str):
        raise ProtocolError("failure reason must be text")
    clean_reason = reason.strip() if isinstance(reason, str) else ""
    if status == "failure" and not clean_reason:
        raise ProtocolError("failure requires reason")
    if status != "failure" and clean_reason:
        raise ProtocolError("non-failure status cannot have reason")
    payload = {
        "protocol_version": 1,
        "task_id": task_id,
        "goal_id": goal_id,
        "status": status,
    }
    if direction is not None:
        payload["direction"] = direction
    if clean_reason:
        payload["reason"] = clean_reason
    return payload
```

Every function must reject non-object JSON, protocol versions other than 1, blank/surrounding-whitespace identities, non-finite pose numbers, unknown statuses/directions, success with a reason, and failure without a reason.

Implement `runtime.py` with these exact route mappings and terminal semantics:

```python
import math
import os


ROUTE_SCRIPTS = {
    "left_turn": "follow_left_v4.py",
    "right_turn": "follow_right_v4.py",
    "straight": "follow_mid_v4.py",
}


class ImageHealthGate:
    def __init__(self, clock, max_age, recovery_grace):
        self.clock = clock
        self.max_age = float(max_age)
        self.recovery_grace = float(recovery_grace)
        if self.max_age <= 0 or self.recovery_grace <= 0:
            raise ValueError("image timing must be positive")
        self.last_frame = None
        self.unhealthy_since = None
        self.stop_emitted = False

    def observe_frame(self, timestamp):
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("image timestamp must be finite")
        self.last_frame = timestamp
        self.unhealthy_since = None
        self.stop_emitted = False

    def allows_motion(self):
        now = float(self.clock())
        return self.last_frame is not None and 0.0 <= now - self.last_frame <= self.max_age

    def poll(self):
        now = float(self.clock())
        if self.allows_motion():
            self.unhealthy_since = None
            self.stop_emitted = False
            return "healthy"
        if self.unhealthy_since is None:
            self.unhealthy_since = now
        if now - self.unhealthy_since >= self.recovery_grace:
            return "failure"
        if not self.stop_emitted:
            self.stop_emitted = True
            return "stop"
        return "blocked"


class LineFollowSession:
    def __init__(self, result_file, stop_file, clock, direction_timeout, follow_timeout):
        self.result_file = result_file
        self.stop_file = stop_file
        self.clock = clock
        self.direction_timeout = float(direction_timeout)
        self.follow_timeout = float(follow_timeout)
        if self.direction_timeout <= 0 or self.follow_timeout <= 0:
            raise ValueError("timeouts must be positive")
        self.clear()

    def start(self, task_id, goal_id):
        if not isinstance(task_id, str) or not task_id.strip():
            raise ValueError("task_id must be text")
        if not isinstance(goal_id, str) or not goal_id.strip():
            raise ValueError("goal_id must be text")
        for path in (self.result_file, self.stop_file):
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
        self.task_id = task_id
        self.goal_id = goal_id
        self.activation_time = float(self.clock())
        self.follow_started = None
        self.direction = None
        self.terminal = None

    def _fresh_text(self, path):
        try:
            if os.path.getmtime(path) < self.activation_time:
                return None
            with open(path, "r", encoding="utf-8") as handle:
                return handle.read().strip()
        except (FileNotFoundError, OSError, UnicodeError):
            return None

    def poll_direction(self):
        if self.direction is not None:
            return ("direction_selected", self.direction)
        value = self._fresh_text(self.result_file)
        if value in ROUTE_SCRIPTS:
            self.lock_direction(value)
            return ("direction_selected", self.direction)
        if float(self.clock()) - self.activation_time >= self.direction_timeout:
            self.lock_direction("straight")
            return ("direction_selected", self.direction)
        return ("waiting_signal", None)

    def lock_direction(self, direction):
        if direction not in ROUTE_SCRIPTS:
            raise ValueError("unknown direction")
        if self.direction is None:
            self.direction = direction
            self.follow_started = float(self.clock())
        return self.direction

    def poll_follow(self, child_running):
        if self.terminal is not None:
            return self.terminal
        if self.direction is None or self.follow_started is None:
            raise RuntimeError("direction is not locked")
        if self._fresh_text(self.stop_file) is not None:
            self.terminal = ("success", self.direction)
        elif not child_running:
            self.terminal = ("failure", "line follower exited before final stop")
        elif float(self.clock()) - self.follow_started >= self.follow_timeout:
            self.terminal = ("failure", "line follow timed out")
        else:
            return ("following", None)
        return self.terminal

    def fail(self, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be non-blank text")
        self.terminal = ("failure", reason.strip())
        return self.terminal

    def clear(self):
        self.task_id = None
        self.goal_id = None
        self.activation_time = None
        self.follow_started = None
        self.direction = None
        self.terminal = None
```

On `start`, remove both compatibility files if they exist, record activation time, and reset the locked direction. Only files with `mtime >= activation_time` are fresh. `poll_follow` returns success only for a fresh stop file, failure for a stopped child, failure after 120 seconds, and otherwise `following`.

- [ ] **Step 4: Run focused and package tests**

```powershell
$env:PYTHONPATH='ucar_ws/src/line_follow_integration/src'
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/line_follow_integration/src/line_follow_integration/protocol.py ucar_ws/src/line_follow_integration/src/line_follow_integration/runtime.py ucar_ws/src/line_follow_integration/test/test_protocol.py ucar_ws/src/line_follow_integration/test/test_runtime.py
git commit -m "feat(phase3): define line-follow protocol and runtime"
```

### Task 4: Add frozen algorithms and ROS navigation/process supervisors

**Files:**
- Create: `ucar_ws/src/line_follow_integration/SOURCE_SNAPSHOT.sha256`
- Create: `ucar_ws/src/line_follow_integration/scripts/yolo_server.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/follow_left_v4.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/follow_right_v4.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/follow_mid_v4.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/line_camera_adapter_node.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/line_navigation_adapter_node.py`
- Create: `ucar_ws/src/line_follow_integration/scripts/line_follow_supervisor_node.py`
- Create: `ucar_ws/src/line_follow_integration/test/test_ros_contract.py`
- Modify: `ucar_ws/src/line_follow_integration/CMakeLists.txt`
- Modify: `ucar_ws/src/line_follow_integration/launch/phase3.launch`

- [ ] **Step 1: Write failing source and ROS-contract tests**

The test must assert:

```python
EXPECTED_SOURCES = {
    "follow_left_v4.py": "8a9471e5917b93bdddd1f0b191e8ace48fe4d6baa8269f68204d4fba23fbbfb3",
    "follow_right_v4.py": "92fd5098be423bab61c3eb5deb5c202c12adbc6c45bc4638a1af7bf5931a5d73",
    "follow_mid_v4.py": "736d0666475f25f48f1e11e888a6c40864af95496754ff37f6311fbba3d0b07e",
    "yolo_server.py": "9d7010328a636742c1c60012cd6c4f0c78ae3cfc2ab4cf57c815ec88de4e9a51",
}
```

Also assert that:

- `line_camera_adapter_node.py` subscribes `/usb_cam/image_raw`, `/task/line_follow/start`, and `/task/line_follow/status`, publishes `/line_follow/image_raw`, preserves headers, rate-limits to the configured FPS, and contains no `/cmd_vel` or camera-device ownership;
- the supervisor child command remaps each route script `/usb_cam/image_raw` to `/line_follow/image_raw` and `/cmd_vel` to the internal `/line_follow/cmd_vel_candidate`;
- no route or YOLO script is launched by `start_all_yolo.launch`;
- `line_navigation_adapter_node.py` subscribes `/task/line_navigation_goal`, `/task/cancel`, `/odom`, uses `/move_base`, and publishes `/task/line_navigation_arrived`;
- `line_follow_supervisor_node.py` subscribes `/task/line_follow/start`, `/task/cancel`, `/usb_cam/image_raw`, `/line_follow/image_raw`, and `/line_follow/cmd_vel_candidate`; it is the only Phase 3 node that publishes `/cmd_vel/line_follow`;
- only the supervisor terminates PIDs returned by its own `subprocess.Popen` calls;
- no `tts_http`, `/voice/speak`, `pkill`, `killall`, or broad `rosnode kill` occurs.

- [ ] **Step 2: Run tests and verify RED**

```powershell
$env:PYTHONPATH='ucar_ws/src/line_follow_integration/src'
python -m unittest ucar_ws/src/line_follow_integration/test/test_ros_contract.py -v
```

Expected: missing scripts and contract failures.

- [ ] **Step 3: Import exact proven source snapshots**

Import byte-for-byte:

```text
E:\follow_v1\follow_v1\follow_left_v4.py  -> scripts/follow_left_v4.py
E:\follow_v1\follow_v1\follow_right_v4.py -> scripts/follow_right_v4.py
E:\follow_v1\follow_v1\follow_mid_v4.py   -> scripts/follow_mid_v4.py
ucar@192.168.1.109:/home/ucar/ucar_ws/src/car_server/yolo_server.py -> scripts/yolo_server.py
```

Write the four lowercase hashes above to `SOURCE_SNAPSHOT.sha256`. Do not import `auto_drive_v3.py`; `line_follow_supervisor_node.py` replaces its direction/process role and prevents its duplicate TTS/false-success behavior. Do not import `start_all_yolo.launch`.

- [ ] **Step 4: Implement the camera adapter, navigation adapter and line supervisor**

Implement `line_camera_adapter_node.py` using `camera_transform.py` and `protocol.py`. It must publish only while one correlated line goal is active, limit output using message timestamps to 15 FPS, preserve the input header, deactivate on correlated success/failure, and never change physical-camera parameters.

Model `line_navigation_adapter_node.py` after the already tested `fast_nav_adapter_node.py`, but build its `MoveBaseGoal` from the incoming `pose` and preserve `qz/qw`. Use `ActionOperationCoordinator`-equivalent generation protection, a settled-odom detector, a 300-second timeout, `/task/cancel`, stale callback suppression, and one terminal arrival per identity.

Implement `line_follow_supervisor_node.py` around `LineFollowSession` with these exact child commands:

```python
yolo_command = [
    "rosrun", "line_follow_integration", "yolo_server.py",
    "__name:=phase3_yolo_server",
]
route_command = [
    "rosrun", "line_follow_integration", ROUTE_SCRIPTS[direction],
    "__name:=phase3_line_follower",
    "/usb_cam/image_raw:=/line_follow/image_raw",
    "/cmd_vel:=/line_follow/cmd_vel_candidate",
]
```

Before starting YOLO, verify `/home/ucar/ucar_ws/src/yolo_turn/best.pt` exists and its SHA-256 equals `cb1c5db5da5db75fe40000410295970f2d7fb6a59d9600f82a22d836829e1cdd`. Require a raw frame within 5 seconds, and require a derived frame before starting the route. Publish `waiting_signal`, then `direction_selected`; once the direction is locked, terminate the owned YOLO child before starting the route and publishing `following`. Publish `success` only for a fresh stop marker. Forward candidate Twist messages only while the derived frame age is at most 0.5 seconds. On a stale derived frame, publish zero immediately and drop candidates; resume if frames recover within 3 seconds, otherwise fail. On failure/cancel/shutdown, publish zero first, terminate only owned live children, wait briefly, kill only those still-owned children, and publish one correlated failure.

- [ ] **Step 5: Run contract tests and compile checks**

```powershell
$env:PYTHONPATH='ucar_ws/src/line_follow_integration/src'
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/line_follow_integration
```

Expected: all tests PASS; compileall exits 0.

- [ ] **Step 6: Commit**

```powershell
git add -- ucar_ws/src/line_follow_integration/SOURCE_SNAPSHOT.sha256 ucar_ws/src/line_follow_integration/scripts/yolo_server.py ucar_ws/src/line_follow_integration/scripts/follow_left_v4.py ucar_ws/src/line_follow_integration/scripts/follow_right_v4.py ucar_ws/src/line_follow_integration/scripts/follow_mid_v4.py ucar_ws/src/line_follow_integration/scripts/line_camera_adapter_node.py ucar_ws/src/line_follow_integration/scripts/line_navigation_adapter_node.py ucar_ws/src/line_follow_integration/scripts/line_follow_supervisor_node.py ucar_ws/src/line_follow_integration/test/test_ros_contract.py ucar_ws/src/line_follow_integration/CMakeLists.txt ucar_ws/src/line_follow_integration/launch/phase3.launch
git commit -m "feat(phase3): supervise navigation and proven line routes"
```

### Task 5: Extend the global task state machine and public protocol

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/test/test_orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- Modify: `ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`

- [ ] **Step 1: Write failing end-to-end state-machine tests**

Add helpers that progress an existing two-phase scenario through simulation speech, line navigation, direction selection, final stop, and final TTS. Assert this exact sequence:

```python
expected = [
    "WAITING_SIMULATION_SPEECH",
    "NAVIGATING_LINE_START",
    "WAITING_LINE_DIRECTION",
    "LINE_FOLLOWING",
    "WAITING_FINAL_SPEECH",
    "COMPLETE",
]
```

The main happy-path assertions must be equivalent to:

```python
h.speech_done()
self.assertEqual("NAVIGATING_LINE_START", h.orch.state)
nav_goal = h.actions("publish_line_navigation_goal")[-1]
self.assertEqual({"frame_id", "x", "y", "qz", "qw"}, set(nav_goal["pose"]))

h.line_navigation_arrived(nav_goal["goal_id"], "arrived")
self.assertEqual("WAITING_LINE_DIRECTION", h.orch.state)
line_start = h.actions("publish_line_follow_start")[-1]

h.line_status(line_start["goal_id"], "direction_selected", direction="left_turn")
self.assertEqual("LINE_FOLLOWING", h.orch.state)
h.line_status(line_start["goal_id"], "success", direction="left_turn")
self.assertEqual("WAITING_FINAL_SPEECH", h.orch.state)
self.assertEqual("任务完成", h.actions("publish_speech")[-1]["text"])
h.speech_done()
self.assertEqual("COMPLETE", h.orch.state)
```

Add failure/identity tests for navigation failure, line failure, line timeout, final TTS failure, duplicate results, wrong `task_id`, wrong `goal_id`, success before direction selection, and no completion before the matching final TTS response. Update old tests that expected completion immediately after simulation speech to expect `NAVIGATING_LINE_START`.

Assert the outer orchestrator defaults are `line_navigation=310.0`, `line_direction=35.0`, and `line_follow=125.0`, each strictly greater than the Phase 3 node's corresponding 300/30/120-second limit.

- [ ] **Step 2: Run focused tests and verify RED**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_orchestrator.py ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py -v
```

Expected: new Phase 3 states/actions are missing and old completion assertions fail.

- [ ] **Step 3: Implement the state machine**

Add states:

```python
NAVIGATING_LINE_START = "NAVIGATING_LINE_START"
WAITING_LINE_DIRECTION = "WAITING_LINE_DIRECTION"
LINE_FOLLOWING = "LINE_FOLLOWING"
WAITING_FINAL_SPEECH = "WAITING_FINAL_SPEECH"
```

Add them to active states and timeout keys:

```python
NAVIGATING_LINE_START: "line_navigation",
WAITING_LINE_DIRECTION: "line_direction",
LINE_FOLLOWING: "line_follow",
WAITING_FINAL_SPEECH: "speech",
```

Add these defaults to `orchestrator.yaml` and the node's Python fallback dictionary:

```yaml
timeouts:
  line_navigation: 310.0
  line_direction: 35.0
  line_follow: 125.0
```

Pass a validated `line_start_goal` into `TaskOrchestrator`. Replace the `WAITING_SIMULATION_SPEECH -> _complete()` branch with `_publish_line_navigation_goal()`. Implement:

```python
def _publish_line_navigation_goal(self):
    goal_id = self._id_factory()
    self.task["line_navigation_goal_id"] = goal_id
    self._transition(self.NAVIGATING_LINE_START)
    self._publish_status("running")
    self._emit("publish_line_navigation_goal", {
        "protocol_version": 1,
        "task_id": self.task["task_id"],
        "goal_id": goal_id,
        "pose": dict(self._line_start_goal),
    })
```

On matched navigation arrival, create `line_follow_goal_id`, transition to `WAITING_LINE_DIRECTION`, and emit `publish_line_follow_start`. On `direction_selected`, transition to `LINE_FOLLOWING`. On matched `success`, call `_start_speech(WAITING_FINAL_SPEECH, "任务完成")`. On matched `failure`, call `_fail(reason)`. Only matched final speech success calls `_complete()`.

Add protocol parsing and ROS publishers/subscribers for:

```text
publish: /task/line_navigation_goal, /task/line_follow/start
subscribe: /task/line_navigation_arrived, /task/line_follow/status
```

Load the Phase 3 pose and timeouts through `task_orchestrator.launch`; do not duplicate pose literals in the node.

- [ ] **Step 4: Run all orchestrator tests**

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py ucar_ws/src/task_orchestrator/config/orchestrator.yaml ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch ucar_ws/src/task_orchestrator/test/test_orchestrator.py ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py ucar_ws/src/task_orchestrator/test/test_package_config.py
git commit -m "feat(orchestrator): sequence phase3 navigation and line follow"
```

### Task 6: Add the isolated line-follow velocity source

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/test/test_motion_mode.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py`

- [ ] **Step 1: Write failing motion-ownership tests**

Add assertions:

```python
self.assertEqual(STOP_NAVIGATION, motion_mode_for_state("NAVIGATING_LINE_START"))
self.assertEqual(IDLE, motion_mode_for_state("WAITING_LINE_DIRECTION"))
self.assertEqual(LINE_FOLLOW, motion_mode_for_state("LINE_FOLLOWING"))
self.assertEqual(IDLE, motion_mode_for_state("WAITING_FINAL_SPEECH"))

self.arbiter.set_mode(LINE_FOLLOW, 10.0)
self.assertIsNone(self.arbiter.accept("navigation", moving_twist(), 10.1))
self.assertIsNone(self.arbiter.accept("qr", moving_twist(), 10.1))
self.assertIsNone(self.arbiter.accept("stop", moving_twist(), 10.1))
self.assertEqual(moving_values, values(self.arbiter.accept("line_follow", moving_twist(), 10.1)))
```

Node tests must assert exactly one new subscription `/cmd_vel/line_follow`, no additional final `/cmd_vel` publisher, stale-source zero, invalid-value zero, and zero before a mode switch begins forwarding line motion.

- [ ] **Step 2: Run focused tests and verify RED**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_motion_mode.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py -v
```

Expected: `LINE_FOLLOW` and `line_follow` source are unknown.

- [ ] **Step 3: Implement the minimal fourth source**

```python
LINE_FOLLOW = "LINE_FOLLOW"

# motion_mode_for_state
if state == "NAVIGATING_LINE_START":
    return STOP_NAVIGATION
if state == "LINE_FOLLOWING":
    return LINE_FOLLOW
```

Extend the arbiter with:

```python
SOURCES = frozenset(("navigation", "qr", "stop", "line_follow"))
ACTIVE_SOURCE = {
    NAVIGATION: "navigation",
    QR_SEARCH: "qr",
    STOP_NAVIGATION: "stop",
    LINE_FOLLOW: "line_follow",
}
```

Subscribe `/cmd_vel/line_follow` in `velocity_arbiter_node.py`. Preserve all existing finite-value, magnitude, epoch, stale timeout, shutdown, and zero-before-switch behavior.

- [ ] **Step 4: Run the focused and full orchestrator suites**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_motion_mode.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py ucar_ws/src/task_orchestrator/test/test_motion_mode.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py
git commit -m "feat(orchestrator): isolate line-follow velocity ownership"
```

### Task 7: Compose Phase 3 into the one-button launch and preflight

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`
- Modify: `ucar_ws/src/task_orchestrator/launch/competition_full.launch`
- Modify: `ucar_ws/src/task_orchestrator/scripts/start_competition.sh`
- Modify: `ucar_ws/src/task_orchestrator/package.xml`

- [ ] **Step 1: Write failing launch and shell tests**

Assert the root launch:

- defines `start_line_follow=true`;
- includes `line_follow_integration/launch/phase3.launch` exactly once;
- forwards the Phase 3 config path and all five timeout/pose/image parameters through the package launch;
- still contains exactly one `usb_cam_node`, one base owner, and one navigation owner at a time;
- never includes `start_all_yolo.launch`;
- keeps QR on raw `/usb_cam/image_raw` and line output on `/line_follow/image_raw`;
- keeps the sole final `/cmd_vel` publisher in `velocity_arbiter_node.py`.

Extend fake-shell tests so `start_line_follow:=true|false` is strictly validated, non-boolean Phase 3 tuning args are forwarded verbatim, and enabling Phase 3 fails before roslaunch if the model is absent or its SHA differs. Disabling Phase 3 must skip only the model check and must not weaken existing hardware/secret/owner checks.

- [ ] **Step 2: Run focused tests and verify RED**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_competition_bringup.py ucar_ws/src/task_orchestrator/test/test_package_config.py -v
```

Expected: no Phase 3 include/parameter/preflight support.

- [ ] **Step 3: Implement the root composition**

Add:

```xml
<arg name="start_line_follow" default="true"/>
<arg name="line_follow_launch" default="$(find line_follow_integration)/launch/phase3.launch"/>
<group if="$(arg start_line_follow)">
  <include file="$(arg line_follow_launch)"/>
</group>
```

Forward the same `line_start_goal`, camera and timeout values into both `task_orchestrator` and Phase 3 launch without duplicating a second source of defaults. Add `line_follow_integration` as an exec dependency. Update preflight using positional argument arrays only; do not use `eval`, glob expansion, command substitution from user input, broad process termination, or automatic device cleanup.

- [ ] **Step 4: Run bringup and all package tests**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_competition_bringup.py ucar_ws/src/task_orchestrator/test/test_package_config.py -v
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/task_orchestrator/launch/competition_full.launch ucar_ws/src/task_orchestrator/scripts/start_competition.sh ucar_ws/src/task_orchestrator/package.xml ucar_ws/src/task_orchestrator/test/test_competition_bringup.py ucar_ws/src/task_orchestrator/test/test_package_config.py
git commit -m "feat(bringup): include safe phase3 line-follow stack"
```

### Task 8: Document operation, tuning and safe test procedure

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/README.md`
- Modify: `ucar_ws/src/task_orchestrator/test/manual_simulation.md`
- Modify: `ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md`
- Create: `ucar_ws/src/line_follow_integration/README.md`
- Modify: `ucar_ws/src/line_follow_integration/CMakeLists.txt`

- [ ] **Step 1: Write failing documentation assertions**

Add tests that require all of these literal operator-visible items:

```text
/task/line_navigation_goal
/task/line_navigation_arrived
/task/line_follow/start
/task/line_follow/status
/cmd_vel/line_follow
LINE_FOLLOW
red_light
left_turn
right_turn
straight
任务完成
/home/ucar/ucar_ws/src/line_follow_integration/config/phase3.yaml
D:\program_sec\智能车\.worktrees\phase3-line-follow-integration\ucar_ws\src\line_follow_integration\config\phase3.yaml
```

Require the README to state that changing YAML requires restarting the root launch, only final stop-line detection is success, 30-second direction timeout selects straight, physical camera remains 1020x720, YOLO uses raw images, the line follower uses derived 640x480 images, and `start_all_yolo.launch` must not be run concurrently.

- [ ] **Step 2: Run documentation tests and verify RED**

```powershell
python -m unittest ucar_ws/src/task_orchestrator/test/test_package_config.py ucar_ws/src/line_follow_integration/test/test_package_config.py -v
```

Expected: missing Phase 3 operator documentation.

- [ ] **Step 3: Write the operator and handoff documentation**

Document:

- the complete Phase 1 -> Phase 2 -> Phase 3 state/topic timeline;
- the exact YAML local and vehicle absolute paths;
- safe per-module launch commands with motion disabled first;
- `rostopic echo`/`hz` commands for raw image, derived image, direction status, motion mode and isolated velocity;
- how to change only one parameter per run and restart;
- model/source SHA validation;
- error reason meanings and owned-process behavior;
- staged real-vehicle acceptance from navigation-only to full motion;
- branch, commits, tests, external model, and deployment not yet performed.

- [ ] **Step 4: Re-run documentation and package tests**

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add -- ucar_ws/src/task_orchestrator/README.md ucar_ws/src/task_orchestrator/test/manual_simulation.md ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md ucar_ws/src/task_orchestrator/test/test_package_config.py ucar_ws/src/line_follow_integration/README.md ucar_ws/src/line_follow_integration/CMakeLists.txt ucar_ws/src/line_follow_integration/test/test_package_config.py
git commit -m "docs(phase3): add tuning and staged vehicle acceptance"
```

### Task 9: Run full regression and prepare implementation handoff

**Files:**
- Modify only if verification exposes a concrete defect in an in-scope file.

- [ ] **Step 1: Run every local suite**

```powershell
python -m unittest discover -s ucar_ws/src/line_follow_integration/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py" -v
```

Expected: every suite reports `OK`, including the 113-test stop baseline.

- [ ] **Step 2: Run syntax, XML, YAML, source-hash and whitespace checks**

```powershell
python -m compileall -q ucar_ws/src/line_follow_integration ucar_ws/src/task_orchestrator ucar_ws/src/stop
python -c "import xml.etree.ElementTree as ET; [ET.parse(p) for p in [r'ucar_ws/src/line_follow_integration/launch/phase3.launch', r'ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch', r'ucar_ws/src/task_orchestrator/launch/competition_full.launch']]"
python -c "import yaml; yaml.safe_load(open(r'ucar_ws/src/line_follow_integration/config/phase3.yaml', encoding='utf-8'))"
python -c "import hashlib, pathlib; expected={'follow_left_v4.py':'8a9471e5917b93bdddd1f0b191e8ace48fe4d6baa8269f68204d4fba23fbbfb3','follow_right_v4.py':'92fd5098be423bab61c3eb5deb5c202c12adbc6c45bc4638a1af7bf5931a5d73','follow_mid_v4.py':'736d0666475f25f48f1e11e888a6c40864af95496754ff37f6311fbba3d0b07e','yolo_server.py':'9d7010328a636742c1c60012cd6c4f0c78ae3cfc2ab4cf57c815ec88de4e9a51'}; root=pathlib.Path(r'ucar_ws/src/line_follow_integration/scripts'); assert all(hashlib.sha256((root/name).read_bytes()).hexdigest()==digest for name,digest in expected.items())"
git diff --check
git status --short
```

Expected: all commands exit 0; status contains no build products, logs, models, secrets or unrelated files.

- [ ] **Step 3: Inspect the commit series and scope**

```powershell
git log --oneline 7b61069..HEAD
git diff --stat 7b61069..HEAD
git diff --name-only 7b61069..HEAD
```

Expected: only the design/plan, `line_follow_integration`, Phase 3-related `task_orchestrator` files, and documentation are present. No stop algorithm change follows `7b61069`.

- [ ] **Step 4: Record final verification evidence**

Update `HANDOFF_TO_CODEX.md` with the final HEAD, each commit purpose, exact test counts/results, model and source hashes, the YAML absolute paths, known unverified real-vehicle behavior, and the required deployment sequence. Do not claim catkin or vehicle success before those commands actually run on the vehicle.

- [ ] **Step 5: Commit the verified handoff if it changed**

```powershell
git add -- ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md
git commit -m "docs(phase3): record verified implementation handoff"
```

After this plan is complete, implementation stops before deployment. The next separate operation is: review diff -> back up vehicle packages -> deploy to `/home/ucar/ucar_ws/src` -> catkin build -> no-motion topic simulation -> staged supervised vehicle test.
