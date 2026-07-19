# Continuous QR Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed-wall stop-and-scan behavior with a safe continuous sweep that collects and resolves three unique QR URLs, then performs quality-guided targeted rescans when necessary.

**Architecture:** Keep the existing two-node ROS 1 package boundary: `qr_scanner` owns latest-frame decoding, quality measurement, URL de-duplication, and concurrent HTTP resolution; `item_search_controller` owns yaw accumulation, sweep/rescan motion, timeouts, ordered aggregation, and final protocol output. Pure-Python modules carry all state and policy so Windows unit tests cover behavior without ROS, while ROS scripts only adapt topics, parameters, images, odometry, and `Twist`.

**Tech Stack:** Python 3, ROS 1 (`rospy`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`), `pyzbar`/ZBar, OpenCV, NumPy, `requests`, `unittest`.

---

## File Structure

The implementation intentionally replaces the old fixed-wall behavior instead of maintaining two incompatible modes.

- Create `ucar_ws/src/qr_item_search/src/qr_item_search/protocol.py`: validate protocol-v1 start/stop messages and build result payloads.
- Create `ucar_ws/src/qr_item_search/src/qr_item_search/sweep_coverage.py`: unwrap yaw, collect 24-sector quality statistics, and select targeted rescan intervals.
- Create `ucar_ws/src/qr_item_search/src/qr_item_search/image_quality.py`: measure brightness/overexposure/sharpness and provide the bounded rescan decode variants.
- Modify `ucar_ws/src/qr_item_search/src/qr_item_search/qr_decode.py`: accept all unique QR URLs from one frame with single-frame confirmation.
- Modify `ucar_ws/src/qr_item_search/src/qr_item_search/scanner_logic.py`: replace wall/busy semantics with latest-frame processing and independent concurrent resolver jobs.
- Modify `ucar_ws/src/qr_item_search/src/qr_item_search/search_state.py`: replace fixed-wall states with `FAST_SWEEP`, `WAITING_HTTP`, and `TARGETED_RESCAN`.
- Modify `ucar_ws/src/qr_item_search/src/qr_item_search/controller_logic.py`: implement continuous rotation, aggregation, targeted rescan, total timeout, and terminal output.
- Modify `ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py`: adapt ROS image/yaw/control topics to the new scanner logic and run decoder/resolver workers.
- Modify `ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py`: accept protocol JSON, publish scanner control and final results, and remove match-decision wiring.
- Modify `ucar_ws/src/qr_item_search/launch/qr_item_search.launch`: expose the approved continuous-sweep parameters.
- Modify `ucar_ws/src/qr_item_search/README.md`: document the new workflow, topics, commands, parameters, and test procedure.
- Replace or extend the package tests listed in the tasks below. Tests for removed fixed-wall behavior must be deleted, not left skipped.

### Task 1: Protocol-v1 Value Validation and Result Serialization

**Files:**
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/protocol.py`
- Create: `ucar_ws/src/qr_item_search/test/test_protocol.py`

- [ ] **Step 1: Write failing protocol parsing tests**

```python
import json
import unittest

from qr_item_search.protocol import (
    ProtocolError,
    build_search_result,
    parse_start_request,
    parse_stop_request,
)


class ProtocolTest(unittest.TestCase):
    def test_parses_protocol_v1_start(self):
        request = parse_start_request(json.dumps({
            "protocol_version": 1,
            "task_id": "task-1",
            "search_id": "search-1",
            "expected_count": 3,
        }))
        self.assertEqual("task-1", request.task_id)
        self.assertEqual("search-1", request.search_id)
        self.assertEqual(3, request.expected_count)

    def test_rejects_missing_wrong_or_unsupported_fields(self):
        invalid = (
            {},
            {"protocol_version": 2, "task_id": "t", "search_id": "s",
             "expected_count": 3},
            {"protocol_version": 1, "task_id": "", "search_id": "s",
             "expected_count": 3},
            {"protocol_version": 1, "task_id": "t", "search_id": "s",
             "expected_count": 2},
        )
        for payload in invalid:
            with self.subTest(payload=payload):
                with self.assertRaises(ProtocolError):
                    parse_start_request(json.dumps(payload))

    def test_stop_requires_matching_identity_fields(self):
        request = parse_stop_request(json.dumps({
            "protocol_version": 1,
            "task_id": "task-1",
            "search_id": "search-1",
            "reason": "operator_stop",
        }))
        self.assertEqual("operator_stop", request.reason)

    def test_complete_result_preserves_detection_order(self):
        payload = build_search_result(
            task_id="task-1",
            search_id="search-1",
            stamp=12.5,
            status="complete",
            items=[
                {"order": 1, "item_name": "香蕉",
                 "url": "https://x/1", "detected_yaw": 0.8},
                {"order": 2, "item_name": "毛巾",
                 "url": "https://x/2", "detected_yaw": 2.4},
                {"order": 3, "item_name": "手机",
                 "url": "https://x/3", "detected_yaw": 4.9},
            ],
            message="",
        )
        self.assertEqual([1, 2, 3], [item["order"] for item in payload["items"]])

    def test_complete_requires_exactly_three_valid_items(self):
        with self.assertRaises(ProtocolError):
            build_search_result("t", "s", 1.0, "complete", [], "")
```

- [ ] **Step 2: Run the tests and verify they fail because the module is absent**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_protocol -v
```

Expected: `ModuleNotFoundError: No module named 'qr_item_search.protocol'`.

- [ ] **Step 3: Implement strict protocol parsing and result construction**

```python
import json
import math
from collections import namedtuple


PROTOCOL_VERSION = 1
EXPECTED_QR_COUNT = 3
StartRequest = namedtuple("StartRequest", "task_id search_id expected_count")
StopRequest = namedtuple("StopRequest", "task_id search_id reason")


class ProtocolError(ValueError):
    pass


def _object(raw):
    try:
        value = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise ProtocolError(str(error))
    if not isinstance(value, dict):
        raise ProtocolError("message must be a JSON object")
    if value.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol_version")
    return value


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("{} must be a non-empty string".format(name))
    return value.strip()


def parse_start_request(raw):
    value = _object(raw)
    if value.get("expected_count") != EXPECTED_QR_COUNT:
        raise ProtocolError("expected_count must be 3")
    return StartRequest(
        _text(value.get("task_id"), "task_id"),
        _text(value.get("search_id"), "search_id"),
        EXPECTED_QR_COUNT,
    )


def parse_stop_request(raw):
    value = _object(raw)
    return StopRequest(
        _text(value.get("task_id"), "task_id"),
        _text(value.get("search_id"), "search_id"),
        _text(value.get("reason"), "reason"),
    )


def build_search_result(task_id, search_id, stamp, status, items, message):
    task_id = _text(task_id, "task_id")
    search_id = _text(search_id, "search_id")
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        raise ProtocolError("stamp must be finite")
    if not math.isfinite(stamp):
        raise ProtocolError("stamp must be finite")
    if status not in {"searching", "complete", "not_found", "error", "stopped"}:
        raise ProtocolError("invalid status")
    normalized = list(items)
    if status == "complete" and len(normalized) != EXPECTED_QR_COUNT:
        raise ProtocolError("complete requires exactly three items")
    for index, item in enumerate(normalized, 1):
        if item.get("order") != index:
            raise ProtocolError("items must have consecutive detection order")
        _text(item.get("item_name"), "item_name")
        _text(item.get("url"), "url")
        yaw = item.get("detected_yaw")
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)):
            raise ProtocolError("detected_yaw must be finite")
        if not math.isfinite(yaw):
            raise ProtocolError("detected_yaw must be finite")
    if status in {"not_found", "error", "stopped"}:
        message = _text(message, "message")
    return {
        "protocol_version": PROTOCOL_VERSION,
        "task_id": task_id,
        "search_id": search_id,
        "stamp": float(stamp),
        "status": status,
        "items": normalized,
        "message": message,
    }
```

- [ ] **Step 4: Run the protocol tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_protocol -v
```

Expected: 5 tests pass.

- [ ] **Step 5: Commit the protocol unit**

```powershell
git add ucar_ws/src/qr_item_search/src/qr_item_search/protocol.py ucar_ws/src/qr_item_search/test/test_protocol.py
git commit -m "feat: add QR search protocol validation"
```

### Task 2: Continuous Yaw Tracking and Quality-Guided Coverage

**Files:**
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/sweep_coverage.py`
- Create: `ucar_ws/src/qr_item_search/test/test_sweep_coverage.py`
- Modify: `ucar_ws/src/qr_item_search/src/qr_item_search/yaw_control.py`
- Modify: `ucar_ws/src/qr_item_search/test/test_yaw_control.py`

- [ ] **Step 1: Write failing tests for yaw unwrapping, sector statistics, and rescan intervals**

```python
import math
import unittest

from qr_item_search.sweep_coverage import CoverageMap, YawTracker


class SweepCoverageTest(unittest.TestCase):
    def test_yaw_tracker_crosses_pi_without_reversing(self):
        tracker = YawTracker()
        tracker.reset(math.radians(179))
        self.assertAlmostEqual(
            math.radians(2),
            tracker.update(math.radians(-179)),
            places=5,
        )

    def test_sector_zero_wraps_at_two_pi(self):
        coverage = CoverageMap(sector_count=24)
        coverage.record(2 * math.pi + 0.01, 100.0, 0.01, 80.0, False)
        self.assertEqual(1, coverage.sectors[0].frame_count)

    def test_merges_adjacent_bad_sectors_with_margin(self):
        coverage = CoverageMap(
            sector_count=24,
            minimum_frames=2,
            overexposed_threshold=0.25,
            sharpness_threshold=30.0,
            margin=math.radians(10),
        )
        for sector in range(24):
            yaw = (sector + 0.5) * 2 * math.pi / 24
            coverage.record(yaw, 100.0, 0.01, 80.0, False)
            coverage.record(yaw, 100.0, 0.01, 80.0, False)
        coverage.sectors[5].overexposed_sum = 2.0
        coverage.sectors[6].sharpness_sum = 0.0
        intervals = coverage.rescan_intervals()
        self.assertEqual(1, len(intervals))
        self.assertLess(intervals[0].start, 5 * 2 * math.pi / 24)
        self.assertGreater(intervals[0].end, 7 * 2 * math.pi / 24)

    def test_qr_neighbor_sectors_are_candidates(self):
        coverage = CoverageMap(sector_count=24)
        for sector in range(24):
            yaw = (sector + 0.5) * 2 * math.pi / 24
            coverage.record(yaw, 100.0, 0.0, 100.0, False)
            coverage.record(yaw, 100.0, 0.0, 100.0, False)
        coverage.record(math.pi, 100.0, 0.0, 100.0, True)
        intervals = coverage.rescan_intervals()
        self.assertTrue(intervals)

    def test_bad_sectors_across_zero_form_one_wrapped_interval(self):
        coverage = CoverageMap(sector_count=24, minimum_frames=2)
        for sector in range(1, 23):
            yaw = (sector + 0.5) * 2 * math.pi / 24
            coverage.record(yaw, 100.0, 0.0, 100.0, False)
            coverage.record(yaw, 100.0, 0.0, 100.0, False)
        intervals = coverage.rescan_intervals()
        self.assertEqual(1, len(intervals))
        self.assertGreater(intervals[0].end, 2 * math.pi)
```

- [ ] **Step 2: Run the focused tests and observe the missing module**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_sweep_coverage -v
```

Expected: import failure for `sweep_coverage`.

- [ ] **Step 3: Implement the yaw tracker and deterministic sector selection**

```python
import math
from collections import namedtuple

from qr_item_search.yaw_control import normalize_angle


Interval = namedtuple("Interval", "start end priority")


class Sector:
    def __init__(self):
        self.frame_count = 0
        self.brightness_sum = 0.0
        self.overexposed_sum = 0.0
        self.sharpness_sum = 0.0
        self.decoded = False


class YawTracker:
    def __init__(self):
        self._last = None
        self._accumulated = 0.0

    def reset(self, yaw):
        self._last = yaw
        self._accumulated = 0.0

    def update(self, yaw):
        if self._last is None:
            self.reset(yaw)
            return 0.0
        self._accumulated += normalize_angle(yaw - self._last)
        self._last = yaw
        return self._accumulated


class CoverageMap:
    def __init__(
        self,
        sector_count=24,
        minimum_frames=2,
        overexposed_threshold=0.25,
        sharpness_threshold=30.0,
        margin=math.radians(10),
    ):
        self.sector_count = sector_count
        self.minimum_frames = minimum_frames
        self.overexposed_threshold = overexposed_threshold
        self.sharpness_threshold = sharpness_threshold
        self.margin = margin
        self.sectors = [Sector() for unused in range(sector_count)]

    def record(self, yaw, brightness, overexposed, sharpness, decoded):
        width = 2 * math.pi / self.sector_count
        index = int((yaw % (2 * math.pi)) / width) % self.sector_count
        sector = self.sectors[index]
        sector.frame_count += 1
        sector.brightness_sum += brightness
        sector.overexposed_sum += overexposed
        sector.sharpness_sum += sharpness
        sector.decoded = sector.decoded or bool(decoded)

    def rescan_intervals(self):
        priorities = {}
        for index, sector in enumerate(self.sectors):
            if sector.frame_count < self.minimum_frames:
                priorities[index] = 0
            elif sector.overexposed_sum / sector.frame_count >= (
                self.overexposed_threshold
            ):
                priorities[index] = 1
            elif sector.sharpness_sum / sector.frame_count < (
                self.sharpness_threshold
            ):
                priorities[index] = 1
            if sector.decoded:
                priorities.setdefault((index - 1) % self.sector_count, 2)
                priorities.setdefault((index + 1) % self.sector_count, 2)
        priorities.setdefault(0, 3)
        return self._merge(priorities)

    def _merge(self, priorities):
        width = 2 * math.pi / self.sector_count
        ordered = sorted(priorities)
        groups = []
        for index in ordered:
            if not groups or index != groups[-1][-1] + 1:
                groups.append([index])
            else:
                groups[-1].append(index)
        if (
            len(groups) > 1
            and groups[0][0] == 0
            and groups[-1][-1] == self.sector_count - 1
        ):
            wrapped = groups[-1] + [
                index + self.sector_count for index in groups[0]
            ]
            groups = groups[1:-1] + [wrapped]
        return sorted(
            (
                Interval(
                    group[0] * width - self.margin,
                    (group[-1] + 1) * width + self.margin,
                    min(
                        priorities[index % self.sector_count]
                        for index in group
                    ),
                )
                for group in groups
            ),
            key=lambda interval: (interval.priority, interval.start),
        )
```

- [ ] **Step 4: Add a directed angular command for approaching rescan interval starts**

Add to `yaw_control.py`:

```python
def directed_angular_command(error, speed, tolerance, min_speed=0.0):
    values = (error, speed, tolerance, min_speed)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("control parameters must be finite")
    if speed <= 0 or tolerance < 0 or min_speed < 0 or min_speed > speed:
        raise ValueError("invalid directed control parameters")
    if abs(error) <= tolerance:
        return 0.0, True
    magnitude = max(min_speed, min(speed, abs(error)))
    return math.copysign(magnitude, error), False
```

Add focused positive, negative, tolerance, minimum-speed, and invalid-parameter cases to `test_yaw_control.py`.

- [ ] **Step 5: Run coverage and yaw tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_sweep_coverage ucar_ws.src.qr_item_search.test.test_yaw_control -v
```

Expected: all coverage and yaw-control tests pass.

- [ ] **Step 6: Commit the sweep geometry unit**

```powershell
git add ucar_ws/src/qr_item_search/src/qr_item_search/sweep_coverage.py ucar_ws/src/qr_item_search/src/qr_item_search/yaw_control.py ucar_ws/src/qr_item_search/test/test_sweep_coverage.py ucar_ws/src/qr_item_search/test/test_yaw_control.py
git commit -m "feat: track continuous QR sweep coverage"
```

### Task 3: Image Quality Metrics and Bounded Rescan Variants

**Files:**
- Create: `ucar_ws/src/qr_item_search/src/qr_item_search/image_quality.py`
- Create: `ucar_ws/src/qr_item_search/test/test_image_quality.py`
- Modify: `ucar_ws/src/qr_item_search/src/qr_item_search/qr_decode.py`
- Modify: `ucar_ws/src/qr_item_search/test/test_qr_decode.py`

- [ ] **Step 1: Write failing image-quality and multi-code decoder tests**

```python
import unittest
from unittest.mock import Mock, patch

import numpy as np

from qr_item_search.image_quality import decode_variants, measure_quality
from qr_item_search.qr_decode import UniqueQrDecoder


class ImageQualityTest(unittest.TestCase):
    def test_white_frame_is_reported_as_overexposed(self):
        quality = measure_quality(np.full((20, 20, 3), 255, dtype=np.uint8))
        self.assertEqual(255.0, quality.brightness)
        self.assertEqual(1.0, quality.overexposed)

    def test_rescan_variants_are_bounded_and_ordered(self):
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        variants = list(decode_variants(frame, enhanced=True))
        self.assertEqual(4, len(variants))
        self.assertIs(frame, variants[0])
        self.assertTrue(all(value.ndim in (2, 3) for value in variants))

    def test_fast_scan_only_returns_original(self):
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        self.assertEqual([frame], list(decode_variants(frame, False)))


class UniqueQrDecoderTest(unittest.TestCase):
    def test_accepts_multiple_unique_values_from_one_frame(self):
        backend = Mock(return_value=[
            "https://x/1", "https://x/2", "https://x/1"
        ])
        decoder = UniqueQrDecoder(backend)
        self.assertEqual(
            ["https://x/1", "https://x/2"],
            decoder.process(object()),
        )
        self.assertEqual([], decoder.process(object()))

    def test_reset_search_clears_seen_urls(self):
        backend = Mock(return_value=["https://x/1"])
        decoder = UniqueQrDecoder(backend)
        decoder.process(object())
        decoder.reset_search()
        self.assertEqual(["https://x/1"], decoder.process(object()))
```

- [ ] **Step 2: Run the tests and verify the new APIs are absent**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_image_quality ucar_ws.src.qr_item_search.test.test_qr_decode -v
```

Expected: imports for `image_quality` or `UniqueQrDecoder` fail.

- [ ] **Step 3: Implement quality measurement and the four exact decode variants**

```python
from collections import namedtuple

import cv2
import numpy as np


FrameQuality = namedtuple(
    "FrameQuality", "brightness overexposed sharpness"
)


def _gray(image):
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def measure_quality(image):
    gray = _gray(image)
    return FrameQuality(
        float(np.mean(gray)),
        float(np.count_nonzero(gray >= 250)) / float(gray.size),
        float(cv2.Laplacian(gray, cv2.CV_64F).var()),
    )


def decode_variants(image, enhanced):
    yield image
    if not enhanced:
        return
    gray = _gray(image)
    yield gray
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    yield clahe
    yield cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        5,
    )
```

- [ ] **Step 4: Replace stable per-wall confirmation with unique single-frame decoding**

Replace `StableQrDecoder` with:

```python
class UniqueQrDecoder:
    def __init__(self, backend=pyzbar_backend):
        self._backend = backend
        self._seen = set()

    def process(self, image):
        accepted = []
        for value in self._backend(image):
            value = value.strip()
            if value and value not in self._seen:
                self._seen.add(value)
                accepted.append(value)
        return accepted

    def reset_search(self):
        self._seen.clear()
```

Update old decoder tests to assert lists, single-frame acceptance, per-search de-duplication, multiple codes per frame, backend errors, and reset behavior. Delete tests that require two matching frames or call `reset_wall()`.

- [ ] **Step 5: Run the focused tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_image_quality ucar_ws.src.qr_item_search.test.test_qr_decode -v
```

Expected: all image and decoder tests pass.

- [ ] **Step 6: Commit image processing**

```powershell
git add ucar_ws/src/qr_item_search/src/qr_item_search/image_quality.py ucar_ws/src/qr_item_search/src/qr_item_search/qr_decode.py ucar_ws/src/qr_item_search/test/test_image_quality.py ucar_ws/src/qr_item_search/test/test_qr_decode.py
git commit -m "feat: decode unique QR codes during motion"
```

### Task 4: Latest-Frame Scanner with Concurrent HTTP Resolution

**Files:**
- Modify: `ucar_ws/src/qr_item_search/src/qr_item_search/scanner_logic.py`
- Modify: `ucar_ws/src/qr_item_search/src/qr_item_search/qr_payload.py`
- Replace: `ucar_ws/src/qr_item_search/test/test_qr_scanner_logic.py`
- Modify: `ucar_ws/src/qr_item_search/test/test_qr_payload.py`

- [ ] **Step 1: Extend resolver tests for finite retry behavior**

Add tests proving `ItemResolver`:

```python
def test_retries_request_errors_only_up_to_configured_limit(self):
    session = Mock()
    session.get.side_effect = [
        requests.Timeout("one"),
        requests.Timeout("two"),
        FakeResponse({"code": 200, "result": "香蕉"}),
    ]
    resolver = ItemResolver(session=session, retries=1)
    with self.assertRaises(requests.Timeout):
        resolver.resolve("https://example.test/item")
    self.assertEqual(2, session.get.call_count)
```

Keep existing validation tests for HTTP(S), host name, JSON object, `code == 200`, and non-empty `result`.

- [ ] **Step 2: Write failing scanner tests for latest-frame replacement and independent resolver jobs**

The replacement test module must use this public scanner API:

```python
logic = ScannerLogic(
    decoder=decoder,
    resolver=resolver,
    event_publisher=publisher,
    quality_function=quality_function,
    variant_function=variant_function,
    worker_count=3,
)
logic.reset_search("task-1", "search-1")
logic.set_control(enabled=True, enhanced=False, detected_yaw=0.4)
logic.submit_frame(frame)
logic.process_latest_frame()
logic.work_once(block=False)
```

Include concrete tests:

```python
def test_network_job_does_not_disable_frame_acceptance(self):
    self.decoder.process.side_effect = [
        ["https://x/1"], ["https://x/2"]
    ]
    self.logic.submit_frame("frame-1")
    self.assertTrue(self.logic.process_latest_frame())
    self.logic.submit_frame("frame-2")
    self.assertTrue(self.logic.process_latest_frame())
    self.assertEqual(2, self.logic.jobs.qsize())

def test_submit_frame_replaces_unprocessed_frame(self):
    self.logic.submit_frame("old")
    self.logic.submit_frame("new")
    self.logic.process_latest_frame()
    self.decoder.process.assert_called_once_with("new")

def test_resolution_completion_keeps_detection_order(self):
    self.decoder.process.side_effect = [
        ["https://x/1"], ["https://x/2"], ["https://x/3"]
    ]
    for index in range(3):
        self.logic.set_control(True, False, float(index))
        self.logic.submit_frame(index)
        self.logic.process_latest_frame()
    self.resolver.resolve.side_effect = ["手机", "香蕉", "毛巾"]
    for unused in range(3):
        self.logic.work_once(block=False)
    resolved = [
        call.args[0] for call in self.publisher.call_args_list
        if call.args[0]["event"] == "resolved"
    ]
    self.assertEqual([1, 2, 3], [event["order"] for event in resolved])

def test_reset_discards_old_inflight_result(self):
    # Block resolver.resolve in a thread, reset to search-2, release it,
    # then assert no event for search-1 is published.
```

Also test quality events, invalid URL events, enhanced variant short-circuiting, duplicate URLs, the three threads started by `run_workers`, publish exceptions, decoder exceptions, and shutdown/reset during active work.

Add a finite rescan retry test:

```python
def test_retry_failed_requeues_each_failed_url_only_once(self):
    self.detect_and_fail("https://x/1")
    self.logic.set_control(
        enabled=True,
        enhanced=True,
        detected_yaw=0.8,
        retry_failed=True,
    )
    self.assertEqual(1, self.logic.jobs.qsize())
    self.logic.set_control(
        enabled=True,
        enhanced=True,
        detected_yaw=0.9,
        retry_failed=True,
    )
    self.assertEqual(1, self.logic.jobs.qsize())
```

- [ ] **Step 3: Run scanner tests and verify old wall-based implementation fails**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_qr_scanner_logic ucar_ws.src.qr_item_search.test.test_qr_payload -v
```

Expected: scanner constructor/API failures against the old implementation.

- [ ] **Step 4: Replace scanner state with latest-frame and detection records**

Use these exact records and event fields:

```python
ScannerJob = namedtuple(
    "ScannerJob", "generation task_id search_id order url detected_yaw"
)


class ScannerLogic:
    def __init__(
        self,
        decoder,
        resolver,
        event_publisher,
        quality_function,
        variant_function,
        jobs=None,
        worker_count=3,
        warning=None,
        error_handler=None,
    ):
        if worker_count != 3:
            raise ValueError("worker_count must be 3 for protocol v1")
        self._decoder = decoder
        self._resolver = resolver
        self._publish = event_publisher
        self._measure_quality = quality_function
        self._variants = variant_function
        self._jobs = jobs or queue.Queue(maxsize=3)
        self._lock = threading.RLock()
        self._generation = 0
        self._task_id = ""
        self._search_id = ""
        self._enabled = False
        self._enhanced = False
        self._yaw = 0.0
        self._latest_frame = None
        self._order_by_url = {}
        self._failed_jobs = {}
        self._rescan_retried = set()
        self._worker_count = worker_count
        self._warning = warning or (lambda message: None)
        self._error_handler = error_handler or (lambda error: None)
```

`submit_frame(frame)` replaces `_latest_frame` under the lock. `process_latest_frame()` atomically takes that frame, measures quality once, then decodes variants until at least one URL is found. It publishes a `quality` event:

```python
{
    "event": "quality",
    "task_id": task_id,
    "search_id": search_id,
    "detected_yaw": yaw,
    "brightness": quality.brightness,
    "overexposed": quality.overexposed,
    "sharpness": quality.sharpness,
    "decoded": bool(urls),
}
```

For each new URL it validates with `ItemResolver` URL validation extracted into `validate_url(url)`, assigns `order = len(_order_by_url) + 1`, queues one `ScannerJob`, and immediately publishes:

```python
{
    "event": "detected",
    "task_id": task_id,
    "search_id": search_id,
    "order": order,
    "url": url,
    "detected_yaw": yaw,
}
```

`work_once()` resolves a job without holding the scanner lock and publishes one of:

```python
{"event": "resolved", "task_id": task_id, "search_id": search_id,
 "order": order, "url": url, "detected_yaw": yaw,
 "item_name": item_name, "message": ""}

{"event": "resolve_error", "task_id": task_id, "search_id": search_id,
 "order": order, "url": url, "detected_yaw": yaw,
 "item_name": "", "message": str(error)}
```

Before publishing, compare `job.generation` with the current generation so a reset discards stale work. `run_workers(is_shutdown)` starts exactly three daemon threads that call `work_once(timeout=0.2)`. Never set a global busy flag for HTTP work.

`set_control(enabled, enhanced, detected_yaw, retry_failed=False)` updates scan mode. On the first call with `retry_failed=True` in a search, it requeues each failed job exactly once and records its URL in `_rescan_retried`; later control updates cannot requeue it again. With `ItemResolver(retries=1)`, this produces at most three requests per URL across the search: initial request, normal resolver retry, and one targeted-rescan request.

- [ ] **Step 5: Extract URL validation without changing payload semantics**

In `qr_payload.py`, add:

```python
def validate_url(qr_content):
    normalized = qr_content.strip()
    parsed = urlparse(normalized)
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise InvalidQrUrl(normalized)
    return normalized
```

Call `validate_url()` at the beginning of `ItemResolver.resolve()`. Scanner uses the same function before assigning an order.

- [ ] **Step 6: Run resolver and scanner tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_qr_payload ucar_ws.src.qr_item_search.test.test_qr_scanner_logic -v
```

Expected: all scanner concurrency, replacement, order, reset, and resolver tests pass.

- [ ] **Step 7: Commit the independent scanner pipeline**

```powershell
git add ucar_ws/src/qr_item_search/src/qr_item_search/scanner_logic.py ucar_ws/src/qr_item_search/src/qr_item_search/qr_payload.py ucar_ws/src/qr_item_search/test/test_qr_scanner_logic.py ucar_ws/src/qr_item_search/test/test_qr_payload.py
git commit -m "feat: resolve QR items without blocking scanning"
```

### Task 5: Replace the Fixed-Wall Search State Machine

**Files:**
- Modify: `ucar_ws/src/qr_item_search/src/qr_item_search/search_state.py`
- Replace: `ucar_ws/src/qr_item_search/test/test_search_state.py`

- [ ] **Step 1: Write failing transition tests for the approved state graph**

```python
import unittest

from qr_item_search.search_state import InvalidTransition, SearchMachine


class SearchMachineTest(unittest.TestCase):
    def setUp(self):
        self.machine = SearchMachine()

    def test_start_enters_fast_sweep(self):
        self.machine.start()
        self.assertEqual("FAST_SWEEP", self.machine.state)

    def test_three_detections_wait_for_http(self):
        self.machine.start()
        self.machine.urls_collected()
        self.assertEqual("WAITING_HTTP", self.machine.state)

    def test_incomplete_fast_sweep_enters_targeted_rescan(self):
        self.machine.start()
        self.machine.fast_sweep_finished()
        self.assertEqual("TARGETED_RESCAN", self.machine.state)

    def test_three_resolved_items_complete_from_active_states(self):
        for state_setup in (
            lambda machine: machine.start(),
            lambda machine: (machine.start(), machine.urls_collected()),
            lambda machine: (machine.start(), machine.fast_sweep_finished()),
        ):
            with self.subTest(state_setup=state_setup):
                machine = SearchMachine()
                state_setup(machine)
                machine.items_resolved()
                self.assertEqual("COMPLETE", machine.state)

    def test_rescan_exhaustion_reports_not_found(self):
        self.machine.start()
        self.machine.fast_sweep_finished()
        self.machine.rescan_finished()
        self.assertEqual("NOT_FOUND", self.machine.state)

    def test_stop_is_terminal_stopped_until_new_start(self):
        self.machine.start()
        self.machine.stop()
        self.assertEqual("STOPPED", self.machine.state)
        self.machine.start()
        self.assertEqual("FAST_SWEEP", self.machine.state)

    def test_invalid_transition_raises(self):
        with self.assertRaises(InvalidTransition):
            self.machine.fast_sweep_finished()
```

- [ ] **Step 2: Run state tests and confirm the old constructor/transitions fail**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_search_state -v
```

Expected: failures because `SearchMachine` still requires `wall_count`.

- [ ] **Step 3: Implement only the approved state graph**

```python
class InvalidTransition(RuntimeError):
    pass


class SearchMachine:
    ACTIVE = {"FAST_SWEEP", "WAITING_HTTP", "TARGETED_RESCAN"}
    RESTARTABLE = {"IDLE", "COMPLETE", "NOT_FOUND", "ERROR", "STOPPED"}

    def __init__(self):
        self.state = "IDLE"

    def start(self):
        if self.state not in self.RESTARTABLE:
            self._invalid("start")
        self.state = "FAST_SWEEP"

    def urls_collected(self):
        if self.state not in {"FAST_SWEEP", "TARGETED_RESCAN"}:
            self._invalid("urls_collected")
        self.state = "WAITING_HTTP"

    def fast_sweep_finished(self):
        self._require("FAST_SWEEP", "fast_sweep_finished")
        self.state = "TARGETED_RESCAN"

    def resume_rescan(self):
        self._require("WAITING_HTTP", "resume_rescan")
        self.state = "TARGETED_RESCAN"

    def items_resolved(self):
        if self.state not in self.ACTIVE:
            self._invalid("items_resolved")
        self.state = "COMPLETE"

    def rescan_finished(self):
        self._require("TARGETED_RESCAN", "rescan_finished")
        self.state = "NOT_FOUND"

    def timeout(self):
        if self.state not in self.ACTIVE:
            self._invalid("timeout")
        self.state = "NOT_FOUND"

    def stop(self):
        self.state = "STOPPED"

    def fail(self):
        self.state = "ERROR"

    def _require(self, expected, event):
        if self.state != expected:
            self._invalid(event)

    def _invalid(self, event):
        raise InvalidTransition(
            "{} is invalid while in {}".format(event, self.state)
        )
```

- [ ] **Step 4: Run state tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_search_state -v
```

Expected: all new state tests pass.

- [ ] **Step 5: Commit the new state graph**

```powershell
git add ucar_ws/src/qr_item_search/src/qr_item_search/search_state.py ucar_ws/src/qr_item_search/test/test_search_state.py
git commit -m "refactor: replace fixed-wall QR state machine"
```

### Task 6: Continuous-Sweep Controller and Ordered Result Aggregation

**Files:**
- Modify: `ucar_ws/src/qr_item_search/src/qr_item_search/controller_logic.py`
- Replace: `ucar_ws/src/qr_item_search/test/test_controller_logic.py`

- [ ] **Step 1: Write controller tests around observable outputs**

Use a `RecordingOutputs` with:

```python
class RecordingOutputs:
    def __init__(self):
        self.speeds = []
        self.states = []
        self.controls = []
        self.results = []

    def publish_speed(self, value):
        self.speeds.append(value)

    def publish_state(self, value):
        self.states.append(value)

    def publish_scanner_control(self, payload):
        self.controls.append(payload)

    def publish_result(self, payload):
        self.results.append(payload)
```

Write concrete tests for:

```python
def test_start_enables_fast_scanning_and_rotation(self):
    self.controller.update_yaw(0.0, now=0.0)
    self.assertTrue(self.controller.start(START_JSON, now=0.0))
    self.controller.tick(now=0.1)
    self.assertEqual("FAST_SWEEP", self.controller.state)
    self.assertEqual(0.40, self.outputs.speeds[-1])
    self.assertTrue(self.outputs.controls[-1]["enabled"])
    self.assertFalse(self.outputs.controls[-1]["enhanced"])

def test_third_detected_url_stops_before_http_finishes(self):
    self.start()
    for order in (1, 2, 3):
        self.controller.handle_scanner_event(json.dumps({
            "event": "detected", "task_id": "task-1",
            "search_id": "search-1", "order": order,
            "url": "https://x/{}".format(order),
            "detected_yaw": float(order),
        }), now=float(order))
    self.assertEqual("WAITING_HTTP", self.controller.state)
    self.assertEqual(0.0, self.outputs.speeds[-1])

def test_reverse_http_completion_still_outputs_detection_order(self):
    self.start_and_detect_three()
    for order, name in ((3, "手机"), (1, "香蕉"), (2, "毛巾")):
        self.resolve(order, name)
    result = self.outputs.results[-1]
    self.assertEqual("complete", result["status"])
    self.assertEqual([1, 2, 3], [item["order"] for item in result["items"]])

def test_380_degree_sweep_enters_targeted_rescan_when_incomplete(self):
    self.start()
    self.controller.update_yaw(math.radians(-170), now=1.0)
    self.controller.update_yaw(math.radians(20), now=2.0)
    self.controller.update_yaw(math.radians(40), now=3.0)
    self.controller.tick(now=3.0)
    self.assertEqual("TARGETED_RESCAN", self.controller.state)

def test_rescan_uses_enhanced_processing_and_low_speed(self):
    self.enter_targeted_rescan()
    self.controller.tick(now=3.1)
    self.assertTrue(self.outputs.controls[-1]["enhanced"])
    self.assertLessEqual(abs(self.outputs.speeds[-1]), 0.20)

def test_timeout_stop_heading_timeout_and_shutdown_publish_zero(self):
    # Create separate controllers for each terminal path and assert the final
    # speed is exactly 0.0 and the protocol status is correct.

def test_no_camera_quality_event_enters_safe_error(self):
    self.start()
    self.controller.update_yaw(0.1, now=0.8)
    self.controller.tick(now=1.1)
    self.assertEqual("ERROR", self.controller.state)
    self.assertEqual("error", self.outputs.results[-1]["status"])
    self.assertEqual(0.0, self.outputs.speeds[-1])
```

Also cover malformed/stale scanner events, duplicate order/URL, bad protocol start, repeated identical `search_id`, different start while active, partial `not_found`, quality event routing into `CoverageMap`, output exceptions, time rollback, non-finite time/yaw, and retrying a failed resolver event during targeted rescan.

- [ ] **Step 2: Run controller tests and verify fixed-wall APIs fail**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_controller_logic -v
```

Expected: constructor, start payload, scanner event, and state failures.

- [ ] **Step 3: Implement the continuous controller constructor and search identity**

The constructor must have this stable signature:

```python
def __init__(
    self,
    outputs,
    fast_angular_speed=0.40,
    targeted_angular_speed=0.20,
    minimum_effective_speed=0.11,
    fast_sweep_angle=6.632251,
    yaw_tolerance=0.035,
    heading_timeout=1.0,
    camera_timeout=1.0,
    search_total_timeout=40.0,
    coverage=None,
    error_handler=None,
    clock=None,
):
```

Store:

```python
self._machine = SearchMachine()
self._tracker = YawTracker()
self._coverage = coverage or CoverageMap()
self._request = None
self._current_yaw = None
self._relative_yaw = 0.0
self._last_heading_time = None
self._last_image_time = None
self._started_at = None
self._detected = {}
self._resolved = {}
self._resolve_errors = {}
self._rescan_intervals = []
self._rescan_index = 0
self._last_completed_search_id = None
self._lock = threading.RLock()
```

`start(raw, now)` parses `StartRequest`. If the same active or completed `search_id` is repeated, publish the current result/state and return `False`. A new valid request resets all search data, calls `YawTracker.reset(current_yaw)`, enters `FAST_SWEEP`, enables non-enhanced scanning, and publishes `searching`.

- [ ] **Step 4: Implement yaw updates, event aggregation, and sweep transitions**

`update_yaw(yaw, now)` validates both inputs, updates `_relative_yaw`, stores `_last_heading_time`, and republishes scanner control containing:

```python
{
    "protocol_version": 1,
    "task_id": request.task_id,
    "search_id": request.search_id,
    "enabled": state in {"FAST_SWEEP", "TARGETED_RESCAN"},
    "enhanced": state == "TARGETED_RESCAN",
    "detected_yaw": relative_yaw,
    "retry_failed": entering_targeted_rescan,
}
```

`handle_scanner_event(raw, now)` accepts only matching task/search IDs. It:

- forwards `quality` fields to `CoverageMap.record()` and updates `_last_image_time`;
- inserts `detected` records keyed by `order`, rejecting duplicate order/URL inconsistencies;
- stores `resolved` item names keyed by `order`;
- stores `resolve_error` details keyed by `order`;
- transitions to `WAITING_HTTP` and stops when three different URLs are detected;
- transitions to `COMPLETE` when orders 1, 2, and 3 are resolved;
- never sorts by resolution completion time.

`tick(now)`:

- publishes `+0.40` in `FAST_SWEEP`;
- ends the fast sweep when `_relative_yaw >= fast_sweep_angle`;
- builds `CoverageMap.rescan_intervals()` when fewer than three valid results remain;
- turns toward and passes each interval at no more than `0.20 rad/s`;
- enters `NOT_FOUND` after intervals are exhausted;
- enters `NOT_FOUND` at total search timeout;
- enters `ERROR` if `now - _last_heading_time > heading_timeout`;
- enters `ERROR` if no `quality` event arrives within `camera_timeout`, or if quality events stop for longer than `camera_timeout`;
- emits zero speed in every non-motion state.

Use `directed_angular_command()` while approaching interval boundaries. Represent rescan interval targets on the same accumulated-yaw axis by choosing the nearest equivalent angle `interval_angle + k * 2π`.

- [ ] **Step 5: Implement one terminal-output path**

All paths call:

```python
def _finish_locked(self, status, message, now):
    self._emit("publish_speed", 0.0)
    self._emit("publish_scanner_control", self._scanner_control(False))
    payload = build_search_result(
        self._request.task_id,
        self._request.search_id,
        now,
        status,
        self._ordered_items(),
        message,
    )
    self._emit("publish_result", payload)
    self._emit("publish_state", self._machine.state)
```

`_ordered_items()` only includes successfully resolved entries and constructs consecutive `order` values based on original detection order. For `complete`, all original orders are already 1–3. For partial results, renumber output consecutively while retaining their relative discovery order as required by the integration protocol.

`stop(raw, now)` validates identity, ignores a stop for another search, enters `STOPPED`, and calls `_finish_locked("stopped", reason, now)`. `shutdown()` always attempts zero speed even if another publisher throws.

- [ ] **Step 6: Run controller and supporting tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_controller_logic ucar_ws.src.qr_item_search.test.test_search_state ucar_ws.src.qr_item_search.test.test_sweep_coverage -v
```

Expected: all transition, aggregation, timeout, idempotency, and safety tests pass.

- [ ] **Step 7: Commit the controller replacement**

```powershell
git add ucar_ws/src/qr_item_search/src/qr_item_search/controller_logic.py ucar_ws/src/qr_item_search/test/test_controller_logic.py
git commit -m "feat: control continuous QR sweep and targeted rescan"
```

### Task 7: Wire the New ROS Topics, Workers, and Launch Parameters

**Files:**
- Modify: `ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py`
- Modify: `ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py`
- Modify: `ucar_ws/src/qr_item_search/launch/qr_item_search.launch`
- Modify: `ucar_ws/src/qr_item_search/test/test_package_config.py`

- [ ] **Step 1: Replace package configuration expectations**

Update AST/XML tests to require:

```python
expected_controller_parameters = {
    "fast_angular_speed": "0.40",
    "targeted_angular_speed": "0.20",
    "minimum_effective_speed": "0.11",
    "fast_sweep_angle": "6.632251",
    "yaw_tolerance": "0.035",
    "heading_timeout": "1.0",
    "camera_timeout": "1.0",
    "search_total_timeout": "40.0",
}
expected_scanner_parameters = {
    "image_topic": "$(arg image_topic)",
    "connect_timeout": "1.0",
    "read_timeout": "2.0",
    "http_retries": "1",
    "http_worker_count": "3",
}
```

AST tests must prove:

- `/qr_item_search/start`, `/qr_item_search/stop`, `/qr_item_search/scanner_control`, `/qr_item_search/scanner_event`, and `/qr_item_search/result` use `std_msgs/String`;
- `/qr_item_search/match_decision`, `/qr_item_search/wall_index`, and `/qr_item_search/scan_enabled` no longer appear;
- both node scripts register shutdown handlers;
- controller publishes `/cmd_vel` with queue size 1;
- image subscriber keeps queue size 1.

- [ ] **Step 2: Run package tests and confirm old wiring fails**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_package_config -v
```

Expected: launch/topic assertions fail against fixed-wall wiring.

- [ ] **Step 3: Adapt `qr_scanner_node.py` without policy logic**

Use:

```python
logic = ScannerLogic(
    decoder=UniqueQrDecoder(),
    resolver=ItemResolver(
        connect_timeout=rospy.get_param("~connect_timeout", 1.0),
        read_timeout=rospy.get_param("~read_timeout", 2.0),
        retries=rospy.get_param("~http_retries", 1),
    ),
    event_publisher=publish_event,
    quality_function=measure_quality,
    variant_function=decode_variants,
    worker_count=rospy.get_param("~http_worker_count", 3),
    warning=rospy.logwarn,
    error_handler=rospy.logerr,
)
```

The scanner node:

- subscribes to `/qr_item_search/scanner_control` (`String`);
- parses control JSON and calls `reset_search(task_id, search_id)` when identity changes;
- calls `set_control(enabled, enhanced, detected_yaw, retry_failed)`;
- converts each image and calls `submit_frame(image)`;
- runs one decoder thread that repeatedly calls `process_latest_frame()` at up to the incoming frame rate;
- starts exactly three resolver workers via `run_workers(rospy.is_shutdown)`;
- publishes JSON events on `/qr_item_search/scanner_event`;
- disables scanner logic during shutdown.

- [ ] **Step 4: Adapt `item_search_controller_node.py`**

Publish:

```python
scanner_control_publisher = rospy.Publisher(
    "/qr_item_search/scanner_control", String, queue_size=1, latch=True
)
result_publisher = rospy.Publisher(
    "/qr_item_search/result", String, queue_size=10, latch=True
)
```

`RosOutputs` serializes `publish_scanner_control()` and `publish_result()` with `json.dumps(..., ensure_ascii=False)`. Subscribe to start, stop, and scanner-event `String` topics. The odometry callback supplies both yaw and `rospy.get_time()` to `update_yaw()`. The 20 Hz timer calls `tick(rospy.get_time())`.

Construct `SearchController` with the exact launch parameters from Step 1. Catch protocol errors at the ROS boundary only to log them; controller-generated error results must still be published through the normal output path.

- [ ] **Step 5: Replace launch parameters**

The final launch file must be:

```xml
<launch>
  <arg name="image_topic" default="/usb_cam/image_raw"/>

  <node pkg="qr_item_search" type="qr_scanner_node.py"
        name="qr_scanner" output="screen">
    <param name="image_topic" value="$(arg image_topic)"/>
    <param name="connect_timeout" value="1.0"/>
    <param name="read_timeout" value="2.0"/>
    <param name="http_retries" value="1"/>
    <param name="http_worker_count" value="3"/>
  </node>

  <node pkg="qr_item_search" type="item_search_controller_node.py"
        name="item_search_controller" output="screen">
    <param name="fast_angular_speed" value="0.40"/>
    <param name="targeted_angular_speed" value="0.20"/>
    <param name="minimum_effective_speed" value="0.11"/>
    <param name="fast_sweep_angle" value="6.632251"/>
    <param name="yaw_tolerance" value="0.035"/>
    <param name="heading_timeout" value="1.0"/>
    <param name="camera_timeout" value="1.0"/>
    <param name="search_total_timeout" value="40.0"/>
  </node>
</launch>
```

- [ ] **Step 6: Run configuration and syntax tests**

Run:

```powershell
python -m unittest ucar_ws.src.qr_item_search.test.test_package_config -v
python -m py_compile ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py
```

Expected: package tests pass and `py_compile` exits 0.

- [ ] **Step 7: Commit the ROS adapter migration**

```powershell
git add ucar_ws/src/qr_item_search/scripts/qr_scanner_node.py ucar_ws/src/qr_item_search/scripts/item_search_controller_node.py ucar_ws/src/qr_item_search/launch/qr_item_search.launch ucar_ws/src/qr_item_search/test/test_package_config.py
git commit -m "feat: wire continuous QR sweep ROS nodes"
```

### Task 8: Documentation, Full Regression, and Deployable Check

**Files:**
- Modify: `ucar_ws/src/qr_item_search/README.md`
- Verify: all files under `ucar_ws/src/qr_item_search`

- [ ] **Step 1: Rewrite README workflow and commands**

Document:

- the four-side random placement rule and requirement to read all three QR codes;
- continuous `FAST_SWEEP`, `WAITING_HTTP`, and `TARGETED_RESCAN` behavior;
- protocol-v1 start/stop/result JSON examples copied from the approved integration protocol;
- every launch parameter and its default;
- the exact start command:

```bash
rostopic pub -1 /qr_item_search/start std_msgs/String \
  '{"data":"{\"protocol_version\":1,\"task_id\":\"manual-1\",\"search_id\":\"search-1\",\"expected_count\":3}"}'
```

- the exact stop command:

```bash
rostopic pub -1 /qr_item_search/stop std_msgs/String \
  '{"data":"{\"protocol_version\":1,\"task_id\":\"manual-1\",\"search_id\":\"search-1\",\"reason\":\"operator_stop\"}"}'
```

- observation commands for `/qr_item_search/state`, `/qr_item_search/result`, `/cmd_vel`, and `/usb_cam/image_raw`;
- Windows unit-test command and on-car `catkin_make`/launch sequence;
- safety note that the first live test requires an open area and an operator ready to stop the robot.

- [ ] **Step 2: Run the complete Windows test suite**

Run from `ucar_ws/src/qr_item_search`:

```powershell
$env:PYTHONPATH = "$PWD\src"
python -m unittest discover -s test -p "test_*.py" -v
```

Expected: all tests pass with no skipped tests and no old fixed-wall assertions.

- [ ] **Step 3: Run static repository checks**

Run:

```powershell
python -m compileall -q src scripts
git diff --check
rg -n "wall_index|wall_yaw_offsets|match_decision|WAITING_MATCH|SETTLING|SCANNING" src scripts launch test README.md
```

Expected:

- `compileall` and `git diff --check` exit 0.
- `rg` returns no matches except an explicitly labelled historical migration note in README, if retained.

- [ ] **Step 4: Build in the ROS environment before operating the chassis**

On U-CAR-02:

```bash
cd ~/ucar_ws
catkin_make
source devel/setup.bash
rostest qr_item_search qr_item_search.test
```

Expected: Catkin build succeeds and the package tests pass. If no `.test` launch exists in the current package, run:

```bash
python3 -m unittest discover \
  -s src/qr_item_search/test -p 'test_*.py' -v
```

Expected: the same suite passes under the robot’s Python 3 environment.

- [ ] **Step 5: Perform a no-motion ROS smoke test**

Start only `usb_cam` and `qr_scanner`, publish scanner-control messages manually with `enabled: true`, keep `item_search_controller` stopped, and present the three known test QR images. Verify:

- three `detected` events appear in presentation order;
- HTTP resolution continues while subsequent QR images are presented;
- three `resolved` events contain the expected item names;
- repeated presentation does not create duplicate orders.

No `/cmd_vel` publisher from this package is active during this step.

- [ ] **Step 6: Commit documentation and verification updates**

```powershell
git add ucar_ws/src/qr_item_search/README.md
git commit -m "docs: update continuous QR sweep operations"
```

- [ ] **Step 7: Record final evidence**

Capture in the implementation handoff:

- branch and final commit hash;
- Windows test count and output summary;
- robot build/test result;
- exact parameters used;
- whether no-motion three-QR smoke test passed;
- any remaining prerequisite before the first controlled chassis sweep.
