# Stop Phase Two Interface Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Repair the QR-to-stop localization handoff, waypoint indexing, two post-parking TTS gates, and strict task-orchestrator topic contracts without replacing the proven stop navigation/OCR/parking algorithms.

**Architecture:** `task_orchestrator` remains the only global business state machine and the only owner of `/voice/speak`, `/voice/speak_done`, and `/task/*`. The stop package remains a private navigation/OCR/parking worker behind `stop_protocol_adapter`; the handoff supervisor owns one latched pickup-area `/initialpose`, while stop no longer resets localization to waypoint 1.

**Tech Stack:** ROS1 Noetic, Python 3, `std_msgs/String` protocol-v1 JSON, `unittest`, roslaunch XML, catkin package layout.

---

### Task 1: Gate both parking transitions on global TTS completion

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/categories.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_categories.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_package_config.py`

- [x] **Step 1: Write failing speech-format and state-transition tests**

Add exact formatter assertions:

```python
from task_orchestrator.categories import (
    format_delivery_speech,
    format_simulation_delivery_speech,
)

def test_delivery_speech_templates(self):
    self.assertEqual(
        "已将毛巾放入日用品加工车间",
        format_delivery_speech("毛巾", "日用品加工车间"),
    )
    self.assertEqual(
        "仿真任务已完成，已将手机放入电子产品生产车间",
        format_simulation_delivery_speech("手机", "电子产品生产车间"),
    )
```

Extend the harness ID sequence with `delivery-speech-1` and `simulation-speech-1`. Add tests that assert:

```python
h.delivery_arrived()
self.assertEqual("WAITING_DELIVERY_SPEECH", h.orch.state)
self.assertEqual([], h.actions("publish_simulation_navigation_goal"))
self.assertEqual("已将苹果放入食品加工车间",
                 h.actions("publish_speech")[-1]["text"])
h.speech_done(speech_id="delivery-speech-1")
self.assertEqual("NAVIGATING_TO_SIM_WORKSHOP", h.orch.state)

h.simulation_arrived(goal_id="simulation-1")
self.assertEqual("WAITING_SIMULATION_SPEECH", h.orch.state)
self.assertNotEqual("complete", h.actions("publish_status")[-1]["status"])
self.assertEqual("仿真任务已完成，已将毛巾放入日用品加工车间",
                 h.actions("publish_speech")[-1]["text"])
h.speech_done(speech_id="simulation-speech-1")
self.assertEqual("COMPLETE", h.orch.state)
```

Also cover failed TTS, speech timeout, stale `speech_id`, duplicate arrival, duplicate `speak_done`, cancellation in each new waiting state, and `simulation_phase_enabled=False` completing only after the physical parking speech succeeds.

- [x] **Step 2: Run focused tests and verify RED**

Run:

```powershell
python -m unittest ucar_ws.src.task_orchestrator.test.test_categories ucar_ws.src.task_orchestrator.test.test_orchestrator -v
```

Expected: failures for missing formatter functions and missing `WAITING_DELIVERY_SPEECH` / `WAITING_SIMULATION_SPEECH` behavior.

- [x] **Step 3: Implement exact formatters and the two waiting states**

Add pure formatters:

```python
def format_delivery_speech(selected_item, workshop):
    return "已将%s放入%s" % (selected_item, workshop)


def format_simulation_delivery_speech(selected_item, workshop):
    return "仿真任务已完成，已将%s放入%s" % (selected_item, workshop)
```

Add `WAITING_DELIVERY_SPEECH` and `WAITING_SIMULATION_SPEECH` to active states and map both to the existing `speech` timeout. Refactor `on_speech_done` to dispatch by the current speech waiting state:

```python
if self.state == self.WAITING_SPEECH:
    self._publish_delivery_navigation_goal()
elif self.state == self.WAITING_DELIVERY_SPEECH:
    if self.simulation_phase_enabled:
        self._publish_simulation_navigation_goal()
    else:
        self._complete()
elif self.state == self.WAITING_SIMULATION_SPEECH:
    self._complete()
```

On a successful physical arrival, generate a new `speech_id`, transition to `WAITING_DELIVERY_SPEECH`, and emit one `publish_speech`. On a successful simulation arrival, do the same for `WAITING_SIMULATION_SPEECH`. Reuse the existing task/speech identity checks and fail-closed handling.

Update `/voice/speak_done` and `/task/cancel` subscriptions so the node accepts all three speech waiting states:

```python
(
    TaskOrchestrator.WAITING_SPEECH,
    TaskOrchestrator.WAITING_DELIVERY_SPEECH,
    TaskOrchestrator.WAITING_SIMULATION_SPEECH,
)
```

- [x] **Step 4: Run focused tests and verify GREEN**

Run the Task 1 command again. Expected: all category and orchestrator tests pass.

- [x] **Step 5: Commit the TTS state-machine change**

```powershell
git add -- ucar_ws/src/task_orchestrator/src/task_orchestrator/categories.py ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py ucar_ws/src/task_orchestrator/test/test_categories.py ucar_ws/src/task_orchestrator/test/test_orchestrator.py ucar_ws/src/task_orchestrator/test/test_package_config.py
git commit -m "fix(orchestrator): gate both parking phases on TTS"
```

### Task 2: Make the pickup pose single-owner and keep status messages protocol-valid

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`
- Modify: `ucar_ws/src/task_orchestrator/scripts/navigation_handoff_supervisor_node.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_navigation_handoff_supervisor_node.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_protocol.py`
- Modify: `ucar_ws/src/stop/launch/mission_integration.launch`
- Modify: `ucar_ws/src/stop/test/test_vehicle_characterization.py`

- [x] **Step 1: Write failing handoff-pose and status-contract tests**

Add node wiring assertions that the supervisor `/initialpose` publisher is latched and uses:

```python
self.assertAlmostEqual(-1.40219, pose.pose.pose.position.x)
self.assertAlmostEqual(-0.627908, pose.pose.pose.position.y)
self.assertAlmostEqual(0.053792653589793, yaw_from_quaternion(pose.pose.pose.orientation))
```

Assert the handoff lifecycle order remains:

```text
start_owned_stop -> publish_initial_pose -> wait_stop_ready -> release_task
```

Add a status-topic test that feeds every JSON published to `/task/navigation_handoff_status` through `parse_navigation_handoff_status`; retry diagnostics must not appear on that topic.

Add launch/source tests asserting `mission_integration.launch` uses `0.0/0.0/0.0` for its internal initial-pose defaults, so `mission_orchestrator` does not publish a second pose.

- [x] **Step 2: Run focused tests and verify RED**

Run:

```powershell
python -m unittest ucar_ws.src.task_orchestrator.test.test_navigation_handoff_supervisor_node ucar_ws.src.task_orchestrator.test.test_protocol ucar_ws.src.stop.test.test_vehicle_characterization -v
```

Expected: failures because the supervisor pose is `(0,0,0)`, its publisher is not latched, stop still uses waypoint 1 as initial pose, and diagnostic JSON without `status` reaches the status topic.

- [x] **Step 3: Implement one authoritative pickup pose**

Set the supervisor defaults to the pickup observation goal:

```yaml
initial_pose_x: -1.40219
initial_pose_y: -0.627908
initial_pose_yaw: 0.053792653589793
```

Create the supervisor publisher with `latch=True`. Keep `_publish_initial_pose()` immediately after the owned stop launch starts so late AMCL subscribers receive the latched pose. Change `mission_integration.launch` initial-pose defaults to zero, which activates the existing “do not auto-publish” branch in stop.

Change real `publish_diagnostic()` to ROS logging only:

```python
def publish_diagnostic(self, payload):
    rospy.logwarn(
        "navigation handoff diagnostic: %s",
        json.dumps(payload, ensure_ascii=False),
    )
```

Keep `_publish_handoff_status()` as the only publisher of `ready|failed` protocol messages.

- [x] **Step 4: Run focused tests and verify GREEN**

Run the Task 2 command again. Expected: all focused tests pass and every handoff status message satisfies protocol v1.

- [x] **Step 5: Commit the localization and status changes**

```powershell
git add -- ucar_ws/src/task_orchestrator/config/orchestrator.yaml ucar_ws/src/task_orchestrator/scripts/navigation_handoff_supervisor_node.py ucar_ws/src/task_orchestrator/test/test_navigation_handoff_supervisor_node.py ucar_ws/src/task_orchestrator/test/test_protocol.py ucar_ws/src/stop/launch/mission_integration.launch ucar_ws/src/stop/test/test_vehicle_characterization.py
git commit -m "fix(handoff): preserve pickup pose across navigation stacks"
```

### Task 3: Correct stop waypoint index semantics

**Files:**
- Modify: `ucar_ws/src/stop/scripts/mission_orchestrator.py`
- Modify: `ucar_ws/src/stop/test/test_vehicle_characterization.py`

- [x] **Step 1: Write failing structural regression tests**

Parse `mission_orchestrator.py` and assert:

```python
self.assertNotIn("current_point_index += 1", already_at_branch)
self.assertNotIn("current_point_index += 1", succeeded_goal_branch)
self.assertIn("current_point_index += 1", exhausted_scan_branch)
self.assertLess(
    exhausted_scan_branch.index("current_point_index += 1"),
    exhausted_scan_branch.index("go_to_find_point()"),
)
```

Keep the existing assertion that Phase 1 records `sim_point_index = current_point_index` so it now records the physical scan waypoint.

- [x] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest ucar_ws.src.stop.test.test_vehicle_characterization -v
```

Expected: failures because the already-at and move-base-success branches currently increment before OCR.

- [x] **Step 3: Move the single index increment to scan exhaustion**

Remove the increment from the `dist < 0.3` branch and normal move-base success branch. In the OCR “Exhausted, next waypoint” branch use:

```python
rospy.loginfo("  Exhausted, next waypoint")
current_point_index += 1
go_to_find_point()
rotate_num = 0
```

Do not change the stage-2 PCA approach logic, waypoint coordinates, OCR matching, or parking controllers.

- [x] **Step 4: Run the focused test and verify GREEN**

Run the Task 3 command again. Expected: all stop characterization tests pass.

- [x] **Step 5: Commit the waypoint fix**

```powershell
git add -- ucar_ws/src/stop/scripts/mission_orchestrator.py ucar_ws/src/stop/test/test_vehicle_characterization.py
git commit -m "fix(stop): retain scan waypoint until OCR exhausts"
```

### Task 4: Update end-to-end scenarios and operator documentation

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py`
- Modify: `ucar_ws/src/task_orchestrator/README.md`
- Modify: `ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md`
- Modify: `docs/superpowers/specs/2026-08-11-stop-phase2-interface-alignment-design.md`

- [x] **Step 1: Write the failing full-scenario assertions**

Update `Scenario.run_to_complete()` so it explicitly acknowledges each parking speech:

```python
self.adapter.mission.phase1_arrived()
self.complete_speech("delivery-speech-1")
sim_goal = self.actions_("publish_simulation_navigation_goal")[-1]
self.adapter.on_simulation_goal(sim_goal)
self.adapter.mission.phase2_arrived()
self.complete_speech("simulation-speech-1")
```

Assert exactly three TTS requests in order and assert no simulation goal exists before physical speech completion and no `COMPLETE` exists before final speech completion.

- [x] **Step 2: Run scenario tests and verify RED where helpers are not yet updated**

Run:

```powershell
python -m unittest ucar_ws.src.task_orchestrator.test.test_stop_phase2_scenarios -v
```

Expected: existing scenario helpers fail because they still advance directly from arrivals.

- [x] **Step 3: Update scenario drivers and operator docs**

Update all scenario paths to acknowledge the two new TTS gates. Document the exact runtime sequence, the pickup-area initial pose, the three hard-coded workshop waypoints, the single-owner `/initialpose` rule, and the authoritative task-orchestrator topics. State that the local result is not a vehicle acceptance until Codex deploys and observes the real run.

- [x] **Step 4: Run all local verification**

Run:

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/task_orchestrator ucar_ws/src/stop
python -c "import xml.etree.ElementTree as ET; [ET.parse(p) for p in ['ucar_ws/src/task_orchestrator/launch/competition_full.launch','ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch','ucar_ws/src/stop/launch/mission_integration.launch']]"
git diff --check
git status --short
```

Expected: all unit tests pass, compileall is silent, XML parsing is silent, `git diff --check` is silent, and status contains only intended files before the final commit.

- [x] **Step 5: Commit tests and documentation**

```powershell
git add -- ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py ucar_ws/src/task_orchestrator/README.md ucar_ws/src/task_orchestrator/HANDOFF_TO_CODEX.md docs/superpowers/specs/2026-08-11-stop-phase2-interface-alignment-design.md docs/superpowers/plans/2026-08-11-stop-phase2-interface-alignment.md
git commit -m "test(integration): verify spoken dual-workshop workflow"
```

### Task 5: Final review checkpoint

**Files:**
- Inspect all files changed since `7fb9800`

- [x] **Step 1: Review the complete diff and commit boundaries**

Run:

```powershell
git diff --stat 7fb9800..HEAD
git diff --check 7fb9800..HEAD
git log --oneline 7fb9800..HEAD
git status --short
```

Confirm no secret, ROS log, build output, map binary, keyframe, or unrelated package change is present.

- [x] **Step 2: Record deployment handoff**

Report the final HEAD, exact test counts, exact pickup pose, TTS texts and sequencing, remaining hard-coded waypoints, and that no vehicle deployment or motion test was performed in this implementation session.
