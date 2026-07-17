# U-CAR-02 QR Item Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS Noetic Python 3 package that turns the U-CAR-02 toward three configured wall headings, scans one ordinary QR code per wall, fetches the encoded URL, publishes the returned item name, and stops early when an external matcher reports success.

**Architecture:** Keep QR decoding/HTTP parsing and yaw-control math independent from ROS so they can be tested with `unittest`. A ROS scanner node subscribes to `/usb_cam/image_raw` and publishes JSON observations, while a search controller uses `/odom` feedback and `/cmd_vel` commands to run a non-blocking `TURNING -> SETTLING -> SCANNING -> WAITING_MATCH` state machine. The decoder uses `pyzbar` with the already-installed `libzbar.so.0`, because the car's OpenCV 3.2.0 has no `QRCodeDetector`.

**Tech Stack:** ROS Noetic, Python 3.7, `rospy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, OpenCV 3.2 for image conversion, `pyzbar`/`libzbar` for QR decoding, `requests`, `unittest`, `rostest`.

---

## File Structure

- `ucar_ws/src/qr_item_search/CMakeLists.txt` - catkin package build/install/test registration.
- `ucar_ws/src/qr_item_search/package.xml` - ROS and system dependency declaration.
- `ucar_ws/src/qr_item_search/setup.py` - installs the pure Python package.
- `ucar_ws/src/qr_item_search/src/qr_item_search/qr_payload.py` - URL validation, HTTP request, and JSON response validation.
- `ucar_ws/src/qr_item_search/src/qr_item_search/qr_decode.py` - `pyzbar` adapter and stable-frame confirmation.
- `ucar_ws/src/qr_item_search/src/qr_item_search/yaw_control.py` - angle normalization and bounded angular velocity calculation.
- `ucar_ws/src/qr_item_search/src/qr_item_search/search_state.py` - ROS-independent search state transitions.
- `ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py` - image subscriber and asynchronous URL resolver.
- `ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py` - three-wall rotation/search controller.
- `ucar_ws/src/qr_item_search/launch/qr_item_search.launch` - configurable topics, angles, speeds, and timeouts.
- `ucar_ws/src/qr_item_search/test/test_qr_payload.py` - payload parser tests.
- `ucar_ws/src/qr_item_search/test/test_qr_decode.py` - decoder confirmation/deduplication tests.
- `ucar_ws/src/qr_item_search/test/test_yaw_control.py` - wraparound and velocity tests.
- `ucar_ws/src/qr_item_search/test/test_search_state.py` - state-machine transition tests.

### ROS Interfaces

- Subscribe `/usb_cam/image_raw` (`sensor_msgs/Image`) - calibrated camera stream.
- Subscribe `/odom` (`nav_msgs/Odometry`) - closed-loop yaw feedback.
- Subscribe `/qr_item_search/start` (`std_msgs/Empty`) - start a three-wall search.
- Subscribe `/qr_item_search/match_decision` (`std_msgs/Bool`) - external matcher response for the latest candidate.
- Publish `/cmd_vel` (`geometry_msgs/Twist`) - angular velocity and guaranteed zero-stop commands.
- Publish `/qr_item_search/scan_enabled` (`std_msgs/Bool`) - controller gates QR processing.
- Publish `/qr_item_search/wall_index` (`std_msgs/Int32`) - active wall number.
- Publish `/qr_item_search/observation` (`std_msgs/String`) - JSON object containing `status`, `wall_index`, `url`, `item_name`, and `message`.
- Publish `/qr_item_search/state` (`std_msgs/String`) - `IDLE`, `TURNING`, `SETTLING`, `SCANNING`, `WAITING_MATCH`, `SUCCESS`, `NOT_FOUND`, or `ERROR`.

### Required One-Time Car Dependency

Do not replace or upgrade OpenCV. Before deployment, install only the Python binding for the existing ZBar library:

```bash
sudo apt-get install python3-pyzbar
```

Verify:

```bash
python3 -c "from pyzbar.pyzbar import decode; print('pyzbar OK')"
```

Expected: `pyzbar OK`.

This installation changes the car and therefore requires explicit user approval at execution time.

---

### Task 1: Create the Catkin Package Skeleton

**Files:**
- Create: `ucar_ws/src/qr_item_search/CMakeLists.txt`
- Create: `ucar_ws/src/qr_item_search/package.xml`
- Create: `ucar_ws/src/qr_item_search/setup.py`
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/__init__.py`

- [ ] **Step 1: Create the package manifest**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>qr_item_search</name>
  <version>0.1.0</version>
  <description>Three-wall QR item search for U-CAR-02.</description>
  <maintainer email="ucar@localhost">U-CAR Team</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>cv_bridge</depend>
  <exec_depend>python3-requests</exec_depend>
  <exec_depend>python3-pyzbar</exec_depend>
  <test_depend>rostest</test_depend>
</package>
```

- [ ] **Step 2: Create Python package installation**

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["qr_item_search"],
    package_dir={"": "src"},
)
setup(**setup_args)
```

- [ ] **Step 3: Create catkin build rules**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(qr_item_search)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/qr_scanner_node.py
  scripts/item_search_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
```

- [ ] **Step 4: Create an empty package initializer**

```python
"""QR item search core logic."""
```

- [ ] **Step 5: Validate package metadata**

Run:

```bash
python3 -m compileall ucar_ws/src/qr_item_search
```

Expected: exit code 0.

- [ ] **Step 6: Commit**

```bash
git add ucar_ws/src/qr_item_search
git commit -m "build: scaffold qr item search package"
```

---

### Task 2: Implement URL and Item Payload Resolution

**Files:**
- Create: `ucar_ws/src/qr_item_search/test/test_qr_payload.py`
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/qr_payload.py`

- [ ] **Step 1: Write failing payload tests**

```python
import json
import unittest
from unittest.mock import Mock

from qr_item_search.qr_payload import (
    InvalidPayload,
    InvalidQrUrl,
    ItemResolver,
)


class ItemResolverTest(unittest.TestCase):
    def test_rejects_non_http_qr_content(self):
        with self.assertRaises(InvalidQrUrl):
            ItemResolver().resolve("banana")

    def test_returns_trimmed_item_name(self):
        session = Mock()
        response = Mock()
        response.json.return_value = {"code": 200, "result": " 香蕉 "}
        response.raise_for_status.return_value = None
        session.get.return_value = response

        result = ItemResolver(session=session, retries=0).resolve(
            "https://example.test/item"
        )

        self.assertEqual(result, "香蕉")
        session.get.assert_called_once_with(
            "https://example.test/item",
            timeout=(1.0, 2.0),
        )

    def test_rejects_application_error(self):
        session = Mock()
        response = Mock()
        response.json.return_value = {"code": 400, "result": ""}
        response.raise_for_status.return_value = None
        session.get.return_value = response

        with self.assertRaises(InvalidPayload):
            ItemResolver(session=session, retries=0).resolve(
                "http://example.test/item"
            )

    def test_retries_one_network_failure(self):
        session = Mock()
        good = Mock()
        good.json.return_value = {"code": 200, "result": "电脑"}
        good.raise_for_status.return_value = None
        session.get.side_effect = [RuntimeError("offline"), good]

        result = ItemResolver(session=session, retries=1).resolve(
            "https://example.test/item"
        )

        self.assertEqual(result, "电脑")
        self.assertEqual(session.get.call_count, 2)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -m unittest \
  ucar_ws/src/qr_item_search/test/test_qr_payload.py -v
```

Expected: FAIL with `ModuleNotFoundError: qr_item_search.qr_payload`.

- [ ] **Step 3: Implement the resolver**

```python
from urllib.parse import urlparse

import requests


class InvalidQrUrl(ValueError):
    pass


class InvalidPayload(ValueError):
    pass


class ItemResolver:
    def __init__(
        self,
        session=None,
        connect_timeout=1.0,
        read_timeout=2.0,
        retries=2,
    ):
        self._session = session or requests.Session()
        self._timeout = (connect_timeout, read_timeout)
        self._retries = retries

    def resolve(self, qr_text):
        parsed = urlparse(qr_text.strip())
        if parsed.scheme not in ("http", "https") or not parsed.netloc:
            raise InvalidQrUrl(qr_text)

        last_error = None
        for _ in range(self._retries + 1):
            try:
                response = self._session.get(qr_text, timeout=self._timeout)
                response.raise_for_status()
                payload = response.json()
                if payload.get("code") != 200:
                    raise InvalidPayload("code is not 200")
                item_name = payload.get("result")
                if not isinstance(item_name, str) or not item_name.strip():
                    raise InvalidPayload("result is empty")
                return item_name.strip()
            except InvalidPayload:
                raise
            except Exception as error:
                last_error = error
        raise last_error
```

- [ ] **Step 4: Run tests and verify GREEN**

Run the command from Step 2.

Expected: 4 tests pass.

- [ ] **Step 5: Commit**

```bash
git add ucar_ws/src/qr_item_search/src/qr_item_search/qr_payload.py \
  ucar_ws/src/qr_item_search/test/test_qr_payload.py
git commit -m "feat: resolve item names from QR URLs"
```

---

### Task 3: Implement QR Decoding, Stable Confirmation, and Deduplication

**Files:**
- Create: `ucar_ws/src/qr_item_search/test/test_qr_decode.py`
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/qr_decode.py`

- [ ] **Step 1: Write failing confirmation tests**

```python
import unittest

from qr_item_search.qr_decode import StableQrDecoder


class StableQrDecoderTest(unittest.TestCase):
    def test_confirms_same_value_on_two_frames(self):
        values = iter([["https://a.test"], ["https://a.test"]])
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=2,
        )

        self.assertIsNone(decoder.process(object()))
        self.assertEqual(decoder.process(object()), "https://a.test")

    def test_different_value_resets_confirmation(self):
        values = iter([
            ["https://a.test"],
            ["https://b.test"],
            ["https://b.test"],
        ])
        decoder = StableQrDecoder(
            backend=lambda image: next(values),
            required_frames=2,
        )

        self.assertIsNone(decoder.process(object()))
        self.assertIsNone(decoder.process(object()))
        self.assertEqual(decoder.process(object()), "https://b.test")

    def test_reset_allows_same_qr_in_new_search(self):
        decoder = StableQrDecoder(
            backend=lambda image: ["https://a.test"],
            required_frames=1,
        )

        self.assertEqual(decoder.process(object()), "https://a.test")
        self.assertIsNone(decoder.process(object()))
        decoder.reset_search()
        self.assertEqual(decoder.process(object()), "https://a.test")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -m unittest \
  ucar_ws/src/qr_item_search/test/test_qr_decode.py -v
```

Expected: FAIL with `ModuleNotFoundError: qr_item_search.qr_decode`.

- [ ] **Step 3: Implement the decoder**

```python
from pyzbar.pyzbar import ZBarSymbol, decode


def pyzbar_backend(image):
    results = decode(image, symbols=[ZBarSymbol.QRCODE])
    values = []
    for result in results:
        values.append(result.data.decode("utf-8").strip())
    return values


class StableQrDecoder:
    def __init__(self, backend=pyzbar_backend, required_frames=2):
        self._backend = backend
        self._required_frames = required_frames
        self._candidate = None
        self._count = 0
        self._seen = set()

    def process(self, image):
        values = self._backend(image)
        value = values[0] if values else None
        if not value or value in self._seen:
            self._candidate = None
            self._count = 0
            return None
        if value != self._candidate:
            self._candidate = value
            self._count = 1
        else:
            self._count += 1
        if self._count < self._required_frames:
            return None
        self._seen.add(value)
        self._candidate = None
        self._count = 0
        return value

    def reset_wall(self):
        self._candidate = None
        self._count = 0

    def reset_search(self):
        self.reset_wall()
        self._seen.clear()
```

- [ ] **Step 4: Run tests and verify GREEN**

Run the command from Step 2.

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add ucar_ws/src/qr_item_search/src/qr_item_search/qr_decode.py \
  ucar_ws/src/qr_item_search/test/test_qr_decode.py
git commit -m "feat: add stable QR decoding"
```

---

### Task 4: Implement Closed-Loop Yaw Control

**Files:**
- Create: `ucar_ws/src/qr_item_search/test/test_yaw_control.py`
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/yaw_control.py`

- [ ] **Step 1: Write failing yaw tests**

```python
import math
import unittest

from qr_item_search.yaw_control import angular_command, normalize_angle


class YawControlTest(unittest.TestCase):
    def test_normalizes_across_positive_pi(self):
        self.assertAlmostEqual(normalize_angle(3.5), 3.5 - 2.0 * math.pi)

    def test_uses_shortest_turn_across_wraparound(self):
        speed, reached = angular_command(
            current_yaw=math.radians(179),
            target_yaw=math.radians(-179),
            kp=1.0,
            max_speed=0.35,
            tolerance=0.03,
        )
        self.assertGreater(speed, 0.0)
        self.assertFalse(reached)

    def test_returns_zero_inside_tolerance(self):
        speed, reached = angular_command(
            current_yaw=1.0,
            target_yaw=1.01,
            kp=1.0,
            max_speed=0.35,
            tolerance=0.03,
        )
        self.assertEqual(speed, 0.0)
        self.assertTrue(reached)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -m unittest \
  ucar_ws/src/qr_item_search/test/test_yaw_control.py -v
```

Expected: FAIL with `ModuleNotFoundError: qr_item_search.yaw_control`.

- [ ] **Step 3: Implement yaw math**

```python
import math


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_command(current_yaw, target_yaw, kp, max_speed, tolerance):
    error = normalize_angle(target_yaw - current_yaw)
    if abs(error) <= tolerance:
        return 0.0, True
    speed = max(-max_speed, min(max_speed, kp * error))
    return speed, False
```

- [ ] **Step 4: Run tests and verify GREEN**

Run the command from Step 2.

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```bash
git add ucar_ws/src/qr_item_search/src/qr_item_search/yaw_control.py \
  ucar_ws/src/qr_item_search/test/test_yaw_control.py
git commit -m "feat: add closed-loop yaw control"
```

---

### Task 5: Implement the Search State Machine

**Files:**
- Create: `ucar_ws/src/qr_item_search/test/test_search_state.py`
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/search_state.py`

- [ ] **Step 1: Write failing state transition tests**

```python
import unittest

from qr_item_search.search_state import SearchMachine


class SearchMachineTest(unittest.TestCase):
    def test_matching_first_wall_finishes_immediately(self):
        machine = SearchMachine(wall_count=3)
        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.observation_succeeded()
        machine.match_decision(True)
        self.assertEqual(machine.state, "SUCCESS")
        self.assertEqual(machine.wall_index, 0)

    def test_failed_payload_advances_without_scan_timeout(self):
        machine = SearchMachine(wall_count=3)
        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.observation_failed()
        self.assertEqual(machine.state, "TURNING")
        self.assertEqual(machine.wall_index, 1)

    def test_last_wall_timeout_reports_not_found(self):
        machine = SearchMachine(wall_count=1)
        machine.start()
        machine.turn_reached()
        machine.settled()
        machine.scan_timeout()
        self.assertEqual(machine.state, "NOT_FOUND")

    def test_stop_returns_to_idle(self):
        machine = SearchMachine(wall_count=3)
        machine.start()
        machine.stop()
        self.assertEqual(machine.state, "IDLE")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -m unittest \
  ucar_ws/src/qr_item_search/test/test_search_state.py -v
```

Expected: FAIL with `ModuleNotFoundError: qr_item_search.search_state`.

- [ ] **Step 3: Implement transitions**

```python
class SearchMachine:
    def __init__(self, wall_count):
        self.wall_count = wall_count
        self.wall_index = 0
        self.state = "IDLE"

    def start(self):
        self.wall_index = 0
        self.state = "TURNING"

    def turn_reached(self):
        if self.state == "TURNING":
            self.state = "SETTLING"

    def settled(self):
        if self.state == "SETTLING":
            self.state = "SCANNING"

    def observation_succeeded(self):
        if self.state == "SCANNING":
            self.state = "WAITING_MATCH"

    def observation_failed(self):
        if self.state == "SCANNING":
            self._advance()

    def scan_timeout(self):
        if self.state == "SCANNING":
            self._advance()

    def match_decision(self, matched):
        if self.state != "WAITING_MATCH":
            return
        if matched:
            self.state = "SUCCESS"
        else:
            self._advance()

    def stop(self):
        self.state = "IDLE"

    def fail(self):
        self.state = "ERROR"

    def _advance(self):
        if self.wall_index + 1 >= self.wall_count:
            self.state = "NOT_FOUND"
        else:
            self.wall_index += 1
            self.state = "TURNING"
```

- [ ] **Step 4: Run tests and verify GREEN**

Run the command from Step 2.

Expected: 4 tests pass.

- [ ] **Step 5: Commit**

```bash
git add ucar_ws/src/qr_item_search/src/qr_item_search/search_state.py \
  ucar_ws/src/qr_item_search/test/test_search_state.py
git commit -m "feat: define QR search state machine"
```

---

### Task 6: Implement the ROS QR Scanner Node

**Files:**
- Create: `ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py`

- [ ] **Step 1: Add a failing import check**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -c \
  "import runpy; runpy.run_path('ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py')"
```

Expected: FAIL because `qr_scanner_node.py` does not exist.

- [ ] **Step 2: Implement the scanner node**

```python
#!/usr/bin/python3
import json
import queue
import threading

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

from qr_item_search.qr_decode import StableQrDecoder
from qr_item_search.qr_payload import (
    InvalidPayload,
    InvalidQrUrl,
    ItemResolver,
)


class QrScannerNode:
    def __init__(self):
        self._bridge = CvBridge()
        self._decoder = StableQrDecoder(
            required_frames=rospy.get_param("~required_frames", 2)
        )
        self._resolver = ItemResolver(
            connect_timeout=rospy.get_param("~connect_timeout", 1.0),
            read_timeout=rospy.get_param("~read_timeout", 2.0),
            retries=rospy.get_param("~http_retries", 2),
        )
        self._enabled = False
        self._wall_index = -1
        self._busy = False
        self._jobs = queue.Queue(maxsize=1)
        self._observation_pub = rospy.Publisher(
            "/qr_item_search/observation", String, queue_size=10
        )
        rospy.Subscriber(
            "/qr_item_search/scan_enabled", Bool, self._enabled_callback
        )
        rospy.Subscriber(
            "/qr_item_search/wall_index", Int32, self._wall_callback
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/usb_cam/image_raw"),
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        threading.Thread(target=self._worker, daemon=True).start()

    def _enabled_callback(self, message):
        self._enabled = message.data
        if not self._enabled:
            self._decoder.reset_wall()

    def _wall_callback(self, message):
        self._wall_index = message.data
        self._decoder.reset_wall()

    def _image_callback(self, message):
        if not self._enabled or self._busy:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, "bgr8")
            qr_text = self._decoder.process(image)
        except (CvBridgeError, UnicodeDecodeError) as error:
            self._publish("decode_error", "", "", str(error))
            return
        if qr_text:
            self._busy = True
            self._jobs.put_nowait(qr_text)

    def _worker(self):
        while not rospy.is_shutdown():
            try:
                qr_text = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                item_name = self._resolver.resolve(qr_text)
                self._publish("success", qr_text, item_name, "")
            except InvalidQrUrl as error:
                self._publish("invalid_url", qr_text, "", str(error))
            except InvalidPayload as error:
                self._publish("invalid_payload", qr_text, "", str(error))
            except Exception as error:
                self._publish("http_error", qr_text, "", str(error))
            finally:
                self._busy = False
                self._jobs.task_done()

    def _publish(self, status, url, item_name, message):
        payload = {
            "status": status,
            "wall_index": self._wall_index,
            "url": url,
            "item_name": item_name,
            "message": message,
        }
        self._observation_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


if __name__ == "__main__":
    rospy.init_node("qr_scanner")
    QrScannerNode()
    rospy.spin()
```

- [ ] **Step 3: Make the script executable**

Run:

```bash
chmod +x ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py
```

Expected: exit code 0.

- [ ] **Step 4: Run static compilation**

Run:

```bash
python3 -m py_compile \
  ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py
```

Expected: exit code 0.

- [ ] **Step 5: Commit**

```bash
git add ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py
git commit -m "feat: add ROS QR scanner node"
```

---

### Task 7: Implement the ROS Search Controller

**Files:**
- Create: `ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py`

- [ ] **Step 1: Add a failing import check**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -c \
  "import runpy; runpy.run_path('ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py')"
```

Expected: FAIL because `item_search_controller_node.py` does not exist.

- [ ] **Step 2: Implement the controller**

```python
#!/usr/bin/python3
import json
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Int32, String
from tf.transformations import euler_from_quaternion

from qr_item_search.search_state import SearchMachine
from qr_item_search.yaw_control import angular_command, normalize_angle


class ItemSearchController:
    def __init__(self):
        self._offsets = rospy.get_param(
            "~wall_yaw_offsets", [0.0, 1.5708, 3.1416]
        )
        self._machine = SearchMachine(len(self._offsets))
        self._current_yaw = None
        self._entry_yaw = None
        self._state_started = rospy.Time.now()
        self._kp = rospy.get_param("~yaw_kp", 1.2)
        self._max_speed = rospy.get_param("~max_angular_speed", 0.30)
        self._yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.035)
        self._settle_seconds = rospy.get_param("~settle_seconds", 0.8)
        self._scan_timeout = rospy.get_param("~scan_timeout", 4.0)
        self._turn_timeout = rospy.get_param("~turn_timeout", 8.0)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._enabled_pub = rospy.Publisher(
            "/qr_item_search/scan_enabled", Bool, queue_size=1, latch=True
        )
        self._wall_pub = rospy.Publisher(
            "/qr_item_search/wall_index", Int32, queue_size=1, latch=True
        )
        self._state_pub = rospy.Publisher(
            "/qr_item_search/state", String, queue_size=10, latch=True
        )
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/qr_item_search/start", Empty, self._start_callback)
        rospy.Subscriber(
            "/qr_item_search/observation", String, self._observation_callback
        )
        rospy.Subscriber(
            "/qr_item_search/match_decision", Bool, self._match_callback
        )
        rospy.on_shutdown(self._stop)
        rospy.Timer(rospy.Duration(0.05), self._tick)
        self._publish_state()

    def _odom_callback(self, message):
        q = message.pose.pose.orientation
        self._current_yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )[2]

    def _start_callback(self, _message):
        if self._current_yaw is None:
            self._machine.fail()
            self._publish_state()
            return
        self._entry_yaw = self._current_yaw
        self._machine.start()
        self._enter_state()

    def _observation_callback(self, message):
        if self._machine.state != "SCANNING":
            return
        payload = json.loads(message.data)
        if payload.get("wall_index") != self._machine.wall_index:
            return
        if payload.get("status") == "success":
            self._machine.observation_succeeded()
        else:
            self._machine.observation_failed()
        self._enter_state()

    def _match_callback(self, message):
        if self._machine.state != "WAITING_MATCH":
            return
        self._machine.match_decision(message.data)
        self._enter_state()

    def _tick(self, _event):
        state = self._machine.state
        elapsed = (rospy.Time.now() - self._state_started).to_sec()
        if state == "TURNING":
            if self._current_yaw is None or elapsed > self._turn_timeout:
                self._machine.fail()
                self._enter_state()
                return
            target = normalize_angle(
                self._entry_yaw + self._offsets[self._machine.wall_index]
            )
            speed, reached = angular_command(
                self._current_yaw,
                target,
                self._kp,
                self._max_speed,
                self._yaw_tolerance,
            )
            self._publish_speed(speed)
            if reached:
                self._machine.turn_reached()
                self._enter_state()
        elif state == "SETTLING" and elapsed >= self._settle_seconds:
            self._machine.settled()
            self._enter_state()
        elif state == "SCANNING" and elapsed >= self._scan_timeout:
            self._machine.scan_timeout()
            self._enter_state()

    def _enter_state(self):
        self._state_started = rospy.Time.now()
        self._stop()
        scanning = self._machine.state == "SCANNING"
        self._enabled_pub.publish(Bool(data=scanning))
        self._wall_pub.publish(Int32(data=self._machine.wall_index))
        self._publish_state()

    def _publish_state(self):
        self._state_pub.publish(String(data=self._machine.state))

    def _publish_speed(self, angular_z):
        command = Twist()
        command.angular.z = angular_z
        self._cmd_pub.publish(command)

    def _stop(self):
        self._publish_speed(0.0)


if __name__ == "__main__":
    rospy.init_node("item_search_controller")
    ItemSearchController()
    rospy.spin()
```

- [ ] **Step 3: Make the script executable and compile it**

Run:

```bash
chmod +x \
  ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py
python3 -m py_compile \
  ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py
```

Expected: exit code 0.

- [ ] **Step 4: Commit**

```bash
git add \
  ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py
git commit -m "feat: add three-wall search controller"
```

---

### Task 8: Add Launch Configuration and Full Local Verification

**Files:**
- Create: `ucar_ws/src/qr_item_search/launch/qr_item_search.launch`
- Modify: `ucar_ws/src/qr_item_search/CMakeLists.txt`

- [ ] **Step 1: Create the launch file**

```xml
<launch>
  <arg name="image_topic" default="/usb_cam/image_raw"/>
  <arg name="wall_yaw_offsets" default="[0.0, 1.5708, 3.1416]"/>

  <node pkg="qr_item_search"
        type="qr_scanner_node.py"
        name="qr_scanner"
        output="screen">
    <param name="image_topic" value="$(arg image_topic)"/>
    <param name="required_frames" value="2"/>
    <param name="connect_timeout" value="1.0"/>
    <param name="read_timeout" value="2.0"/>
    <param name="http_retries" value="2"/>
  </node>

  <node pkg="qr_item_search"
        type="item_search_controller_node.py"
        name="item_search_controller"
        output="screen">
    <rosparam param="wall_yaw_offsets" subst_value="true">
      $(arg wall_yaw_offsets)
    </rosparam>
    <param name="yaw_kp" value="1.2"/>
    <param name="max_angular_speed" value="0.30"/>
    <param name="yaw_tolerance" value="0.035"/>
    <param name="settle_seconds" value="0.8"/>
    <param name="scan_timeout" value="4.0"/>
    <param name="turn_timeout" value="8.0"/>
  </node>
</launch>
```

- [ ] **Step 2: Register tests and launch installation**

Add to `CMakeLists.txt`:

```cmake
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
```

Ensure there is only one `if(CATKIN_ENABLE_TESTING)` block after editing.

- [ ] **Step 3: Run all pure Python tests**

Run:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -m unittest discover \
  -s ucar_ws/src/qr_item_search/test -p 'test_*.py' -v
```

Expected: 14 tests pass.

- [ ] **Step 4: Run syntax checks**

Run:

```bash
python3 -m compileall ucar_ws/src/qr_item_search
```

Expected: exit code 0 and no syntax errors.

- [ ] **Step 5: Inspect launch XML**

Run:

```bash
xmllint --noout \
  ucar_ws/src/qr_item_search/launch/qr_item_search.launch \
  ucar_ws/src/qr_item_search/package.xml
```

Expected: exit code 0.

- [ ] **Step 6: Commit**

```bash
git add ucar_ws/src/qr_item_search/CMakeLists.txt \
  ucar_ws/src/qr_item_search/launch/qr_item_search.launch
git commit -m "feat: launch configurable QR item search"
```

---

### Task 9: Deploy to the Car and Perform Stationary Integration Tests

**Files:**
- Copy local `ucar_ws/src/qr_item_search/` to car path `/home/ucar/ucar_ws/src/qr_item_search/`.

- [ ] **Step 1: Obtain approval for car changes**

Before running any command, report that deployment will:

- add `/home/ucar/ucar_ws/src/qr_item_search`;
- install `python3-pyzbar`;
- rebuild `/home/ucar/ucar_ws`;
- not change or upgrade OpenCV.

Proceed only after explicit user approval.

- [ ] **Step 2: Install the decoder binding**

Run on the car:

```bash
sudo apt-get update
sudo apt-get install -y python3-pyzbar
python3 -c "from pyzbar.pyzbar import decode; print('pyzbar OK')"
```

Expected: `pyzbar OK`.

- [ ] **Step 3: Copy the package and build**

Run locally:

```bash
scp -r ucar_ws/src/qr_item_search ucar:/home/ucar/ucar_ws/src/
ssh ucar \
  "source /opt/ros/noetic/setup.bash && cd /home/ucar/ucar_ws && catkin_make"
```

Expected: catkin build exits 0.

- [ ] **Step 4: Run package tests on the car**

Run:

```bash
ssh ucar \
  "source /opt/ros/noetic/setup.bash && \
   source /home/ucar/ucar_ws/devel/setup.bash && \
   PYTHONPATH=/home/ucar/ucar_ws/src/qr_item_search/src \
   python3 -m unittest discover \
   -s /home/ucar/ucar_ws/src/qr_item_search/test \
   -p 'test_*.py' -v"
```

Expected: 14 tests pass.

- [ ] **Step 5: Test scanning without allowing motion**

Start only the camera and scanner node, not the controller:

```bash
roslaunch usb_cam usb_cam-test.launch
rosrun qr_item_search qr_scanner_node.py
rostopic pub -1 /qr_item_search/wall_index std_msgs/Int32 "data: 0"
rostopic pub -1 /qr_item_search/scan_enabled std_msgs/Bool "data: true"
rostopic echo /qr_item_search/observation
```

Expected when a test QR is held in front of the camera: one JSON observation containing the decoded URL and item result, or a precise error status if the URL is not a valid competition endpoint.

- [ ] **Step 6: Commit deployment notes if calibration values changed**

Do not commit machine-specific calibrated angles yet. Record measured values in the next task's launch file update after controlled motion testing.

---

### Task 10: Perform Controlled Rotation Testing and Calibrate Wall Angles

**Files:**
- Modify: `ucar_ws/src/qr_item_search/launch/qr_item_search.launch`
- Create: `docs/qr-item-search-calibration.md`

- [ ] **Step 1: Establish a safe test area**

Raise the wheels or provide an obstacle-free area, keep an emergency terminal ready, and verify the stop command:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Expected: all wheels stop.

- [ ] **Step 2: Start required nodes**

```bash
roscore
roslaunch ucar_controller base_driver.launch
roslaunch usb_cam usb_cam-test.launch
roslaunch qr_item_search qr_item_search.launch
```

Expected topics:

```bash
rostopic info /cmd_vel
rostopic info /odom
rostopic info /usb_cam/image_raw
```

Each command must show an active publisher or subscriber appropriate to the topic.

- [ ] **Step 3: Trigger a search and monitor it**

```bash
rostopic echo /qr_item_search/state
rostopic echo /qr_item_search/observation
rostopic pub -1 /qr_item_search/start std_msgs/Empty "{}"
```

Expected: `TURNING -> SETTLING -> SCANNING`; a failed QR processing result immediately advances to the next `TURNING` state without waiting for scan timeout.

- [ ] **Step 4: Simulate a successful upper-layer match**

After a successful observation:

```bash
rostopic pub -1 /qr_item_search/match_decision std_msgs/Bool "data: true"
```

Expected: state becomes `SUCCESS` and `/cmd_vel.angular.z` becomes 0.

- [ ] **Step 5: Measure and record wall headings**

For each wall, record:

```text
wall_0_offset_rad: measured relative yaw
wall_1_offset_rad: measured relative yaw
wall_2_offset_rad: measured relative yaw
camera_to_qr_distance_m: measured distance
settle_seconds: smallest reliable value
scan_timeout: reliable first-version value
```

Write the measured values and test date to `docs/qr-item-search-calibration.md`, then update `wall_yaw_offsets`, `settle_seconds`, and `scan_timeout` in the launch file.

- [ ] **Step 6: Re-run the complete stationary-first search**

Expected:

- no QR accepted during `TURNING`;
- each wall produces at most one observation;
- invalid URL or HTTP/JSON failure advances immediately;
- `match_decision=true` stops the search immediately;
- a full unsuccessful cycle ends in `NOT_FOUND` with zero angular velocity.

- [ ] **Step 7: Commit calibrated defaults**

```bash
git add ucar_ws/src/qr_item_search/launch/qr_item_search.launch \
  docs/qr-item-search-calibration.md
git commit -m "test: calibrate three-wall QR search"
```

---

## Final Verification

Run locally:

```bash
PYTHONPATH=ucar_ws/src/qr_item_search/src python3 -m unittest discover \
  -s ucar_ws/src/qr_item_search/test -p 'test_*.py' -v
python3 -m compileall ucar_ws/src/qr_item_search
git status --short
```

Expected:

- 14 tests pass;
- compilation exits 0;
- only intentionally untracked competition assets remain.

Run on the car:

```bash
source /opt/ros/noetic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
rostopic hz /usb_cam/image_raw
rostopic hz /odom
rostopic echo -n 1 /qr_item_search/state
```

Expected:

- camera and odometry topics publish continuously;
- state is one of the documented states;
- after every terminal state, `/cmd_vel` is zero.
