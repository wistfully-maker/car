# Stop Phase Two Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the vehicle-proven `stop` package to the completed first-phase orchestrator through a controlled navigation-stack handoff, then complete two correlated workshop navigation and parking phases.

**Architecture:** Keep `stop`'s vehicle-proven OCR, waypoint, TEB, PCA, parking, and phase-switch algorithms intact. Add a ROS-independent handoff state machine, a thin protocol adapter around `stop`, and one final velocity-ownership path; the global orchestrator remains the business authority and reports physical and simulation arrivals separately.

**Tech Stack:** ROS 1 Noetic, Python 3, `rospy`, `std_msgs/String` JSON protocol v1, `geometry_msgs/Twist`, `nav_msgs/Odometry`, `actionlib`, AMCL, TEB, `roslaunch`, `unittest`, `xml.etree.ElementTree`.

---

## Fixed workspace and prerequisites

Work only in:

```text
D:\program_sec\智能车\.worktrees\stop-phase2-integration
branch: codex/stop-phase2-integration
baseline ancestor: 77e853a767152d7e877b736af6390605c841e6d2
```

Before Task 1, Codex—not DeepSeek—must import the exact vehicle directory
`/home/ucar/ucar_ws/src/stop` into `ucar_ws/src/stop`, generate
`ucar_ws/src/stop/VEHICLE_SNAPSHOT.sha256`, and commit the untouched snapshot as
`chore(stop): import verified vehicle package snapshot`. DeepSeek must stop if that
commit, directory, or manifest is absent.

Run before every task:

```powershell
git branch --show-current
git merge-base --is-ancestor 77e853a HEAD
git status --short
```

Expected: branch is `codex/stop-phase2-integration`, ancestor check exits 0, and
only the files explicitly listed by the active task are modified.

## File responsibility map

| File | Responsibility |
|---|---|
| `task_orchestrator/src/task_orchestrator/orchestrator.py` | Global two-arrival business state machine |
| `task_orchestrator/src/task_orchestrator/protocol.py` | Strict protocol v1 parsing and identity checks |
| `task_orchestrator/scripts/task_orchestrator_node.py` | ROS publishers/subscribers only |
| `task_orchestrator/src/task_orchestrator/handoff.py` | ROS-independent bounded navigation-stack transition |
| `task_orchestrator/scripts/navigation_handoff_supervisor_node.py` | Process/action/odom/readiness ROS adapter |
| `task_orchestrator/src/task_orchestrator/velocity_arbiter.py` | Final first/QR/stop velocity ownership |
| `stop/scripts/mission_orchestrator.py` | Existing vehicle algorithm plus minimal activation/result seam |
| `stop/scripts/stop_protocol_adapter_node.py` | Correlated task input and phase result conversion |
| `stop/src/stop_integration/mission_gate.py` | ROS-independent task identity, dedupe, and terminal-result cache |
| `stop/src/stop_integration/velocity_mux.py` | Isolated stop navigation/manual source selection |
| `stop/scripts/stop_velocity_mux_node.py` | ROS adapter publishing only `/cmd_vel/stop` |
| `stop/launch/mission_integration.launch` | Stop navigation/OCR nodes without shared hardware owners |
| `task_orchestrator/launch/competition_full.launch` | Root composition and explicit opt-in parameters |

### Task 1: Freeze the dual-workshop orchestrator contract

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/test/test_orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_protocol.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py`

- [ ] **Step 1: Add failing state-machine tests**

Add tests that drive the existing harness through TTS success, matching physical
arrival, and matching simulation arrival. Assert this exact sequence:

```python
self.assertEqual("DELIVERY_HANDED_OFF", h.orch.state)
h.delivery_arrived(status="arrived")
self.assertEqual("NAVIGATING_TO_SIM_WORKSHOP", h.orch.state)
goal = h.actions("publish_simulation_navigation_goal")[-1]
self.assertEqual(h.orch.task["task_id"], goal["task_id"])
self.assertEqual("日用品加工车间", goal["target_workshop"])
self.assertEqual("毛巾", goal["selected_item"])
h.simulation_arrived(goal_id=goal["goal_id"], status="arrived")
self.assertEqual("COMPLETE", h.orch.state)
```

Also assert physical failure never publishes a simulation goal; stale, duplicate,
wrong-task, and wrong-goal simulation results never advance; first arrival never
publishes `complete`; each navigation state has its own timeout.

- [ ] **Step 2: Run the focused tests and verify RED**

```powershell
python ucar_ws/src/task_orchestrator/test/test_orchestrator.py -v
python ucar_ws/src/task_orchestrator/test/test_protocol.py -v
```

Expected: FAIL because `publish_simulation_navigation_goal`,
`NAVIGATING_TO_SIM_WORKSHOP`, and the parser do not exist on the baseline.

- [ ] **Step 3: Implement the minimal core transitions**

Use these output contracts without adding unrelated fields:

```python
{
    "protocol_version": 1,
    "task_id": self.task["task_id"],
    "goal_id": simulation_goal_id,
    "target_workshop": self.task["simulation"]["workshop"],
    "selected_item": self.task["simulation"]["selected_item"],
}
```

Add `NAVIGATING_TO_SIM_WORKSHOP` to active states and map it to a dedicated
`simulation_navigation` timeout. Parse `/task/simulation_arrived` with the same
strict arrival schema and expected identity used for physical arrival. Preserve
the physical `DELIVERY_HANDED_OFF` boundary and do not restore the obsolete
`/task/sim_trigger` stub.

- [ ] **Step 4: Wire exact ROS topics**

Add one non-latched publisher for `/task/simulation_navigation_goal` and one
subscriber for `/task/simulation_arrived`. Keep all callbacks serialized through
the existing lock and dispatch queue.

- [ ] **Step 5: Run focused and full orchestrator tests**

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
```

Expected: all runnable tests pass; Windows Bash-only tests may remain skipped.

- [ ] **Step 6: Commit explicit files**

```powershell
git add -- ucar_ws/src/task_orchestrator/test/test_orchestrator.py ucar_ws/src/task_orchestrator/test/test_protocol.py ucar_ws/src/task_orchestrator/src/task_orchestrator/orchestrator.py ucar_ws/src/task_orchestrator/src/task_orchestrator/protocol.py ucar_ws/src/task_orchestrator/scripts/task_orchestrator_node.py
git commit -m "feat(orchestrator): coordinate dual workshop arrivals"
```

### Task 2: Implement the bounded handoff state machine

**Files:**
- Create: `ucar_ws/src/task_orchestrator/src/task_orchestrator/handoff.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_handoff.py`

- [ ] **Step 1: Write failing pure-unit tests**

Define tests against this public surface:

```python
machine = NavigationHandoff(config, clock)
effects = machine.start(task_id="task-1", goal_id="delivery-1")
effects = machine.observe(ActionGoalCancelled())
effects = machine.observe(OdomStopped(stamp=clock()))
effects = machine.observe(LegacyStackExited())
effects = machine.observe(StopStackReady())
```

Assert exact states `IDLE -> CANCELLING -> VERIFYING_STOP -> STOPPING_LEGACY ->
STARTING_STOP -> WAITING_STOP_READY -> READY`. Assert every transition emits
`publish_zero` before lifecycle effects. Add tests for duplicate start, stale
identity, odom freshness, continuous stop duration, per-operation retry limits,
total deadline, clock rollback, and terminal idempotence.

- [ ] **Step 2: Run and verify RED**

```powershell
python ucar_ws/src/task_orchestrator/test/test_handoff.py -v
```

Expected: import failure because `handoff.py` does not exist.

- [ ] **Step 3: Implement immutable events and deterministic effects**

Use small event classes or named tuples and effect dictionaries. The core must
not import `rospy`, `roslaunch`, `subprocess`, or ROS messages. Configuration must
validate positive finite values for:

```text
cancel_retries, cancel_timeout, stop_stable_duration, odom_max_age,
legacy_exit_retries, legacy_exit_timeout, readiness_retries,
readiness_poll_period, total_timeout
```

Exhausted retries produce `publish_zero`, `publish_diagnostic`, then
`handoff_failed`; a transient miss produces `publish_zero`, diagnostic `retrying`,
and the appropriate retry effect.

- [ ] **Step 4: Run tests and commit**

```powershell
python ucar_ws/src/task_orchestrator/test/test_handoff.py -v
git add -- ucar_ws/src/task_orchestrator/src/task_orchestrator/handoff.py ucar_ws/src/task_orchestrator/test/test_handoff.py
git commit -m "feat(handoff): model bounded navigation stack transition"
```

### Task 3: Add the handoff ROS/process adapter

**Files:**
- Create: `ucar_ws/src/task_orchestrator/scripts/navigation_handoff_supervisor_node.py`
- Create: `ucar_ws/src/task_orchestrator/test/test_navigation_handoff_supervisor_node.py`
- Modify: `ucar_ws/src/task_orchestrator/CMakeLists.txt`
- Modify: `ucar_ws/src/task_orchestrator/package.xml`
- Modify: `ucar_ws/src/task_orchestrator/config/orchestrator.yaml`

- [ ] **Step 1: Write fake-ROS and fake-process tests**

Inject fake action cancel, owned process handles, ROS master snapshots, odometry,
and readiness probes. Assert:

```python
self.assertEqual(["cancel_goals", "wait_stopped", "stop_owned_legacy",
                  "verify_legacy_absent", "start_owned_stop",
                  "wait_stop_ready", "release_task"], fake.effects)
self.assertNotIn("pkill", fake.commands)
self.assertNotIn("rosnode kill -a", fake.commands)
```

Prove that only process groups created by the supervisor are terminated; base,
lidar, camera, speech, QR, and LLM owners are never targets. Prove Ctrl+C stops
both owned navigation process groups and publishes zero without reporting arrival.

- [ ] **Step 2: Run and verify RED**

```powershell
python ucar_ws/src/task_orchestrator/test/test_navigation_handoff_supervisor_node.py -v
```

Expected: import failure because the node does not exist.

- [ ] **Step 3: Implement the thin adapter**

The node subscribes to `/task/delivery_navigation_goal`, `/odom`, and process/readiness
events. It owns two explicit `roslaunch` parent handles: the first-phase navigation
include and the stop navigation include. It must never discover kill targets by
substring. Publish diagnostics as protocol v1 JSON on `/task/navigation_handoff_status`
and release the correlated task on `/task/stop_mission_goal` only after `READY`.

- [ ] **Step 4: Add validated configuration**

Add a `navigation_handoff` YAML section with exact defaults:

```yaml
navigation_handoff:
  cancel_retries: 3
  cancel_timeout: 3.0
  stop_stable_duration: 0.75
  linear_stop_threshold: 0.02
  angular_stop_threshold: 0.05
  odom_max_age: 0.5
  legacy_exit_retries: 5
  legacy_exit_timeout: 10.0
  readiness_retries: 30
  readiness_poll_period: 1.0
  total_timeout: 90.0
```

- [ ] **Step 5: Run tests and commit**

```powershell
python ucar_ws/src/task_orchestrator/test/test_navigation_handoff_supervisor_node.py -v
python -m compileall -q ucar_ws/src/task_orchestrator
git add -- ucar_ws/src/task_orchestrator/scripts/navigation_handoff_supervisor_node.py ucar_ws/src/task_orchestrator/test/test_navigation_handoff_supervisor_node.py ucar_ws/src/task_orchestrator/CMakeLists.txt ucar_ws/src/task_orchestrator/package.xml ucar_ws/src/task_orchestrator/config/orchestrator.yaml
git commit -m "feat(handoff): supervise controlled navigation stack switch"
```

### Task 4: Freeze and gate the vehicle stop mission

**Files:**
- Create: `ucar_ws/src/stop/src/stop_integration/__init__.py`
- Create: `ucar_ws/src/stop/src/stop_integration/mission_gate.py`
- Create: `ucar_ws/src/stop/test/test_mission_gate.py`
- Create: `ucar_ws/src/stop/test/test_vehicle_characterization.py`
- Modify: `ucar_ws/src/stop/scripts/mission_orchestrator.py`
- Modify: `ucar_ws/src/stop/setup.py`

- [ ] **Step 1: Add characterization tests before editing vehicle code**

Parse `mission_orchestrator.py` with `ast` and assert the imported snapshot still
contains the exact waypoint tuples, warehouse aliases, Stage 0-6 assignments,
Phase 1 backup duration/speed, 180-degree turn, costmap clear, and Phase 2 jump
logic. Hash the OCR model files against `VEHICLE_SNAPSHOT.sha256`; tests must fail
if model bytes change.

- [ ] **Step 2: Add failing mission-gate tests**

Test this ROS-independent API:

```python
gate = MissionGate()
accepted = gate.accept({
    "protocol_version": 1,
    "task_id": "task-1",
    "physical_goal_id": "delivery-1",
    "simulation_goal_id": "simulation-1",
    "physical": {"target_workshop": "食品加工车间", "selected_item": "苹果"},
    "simulation": {"target_workshop": "日用品加工车间", "selected_item": "毛巾"},
})
self.assertTrue(accepted.start)
```

Assert startup alone never starts motion; duplicate active identity does not
restart; duplicate terminal identity returns cached results; malformed categories,
empty IDs, protocol mismatch, and cross-task updates are rejected.

- [ ] **Step 3: Run and verify RED**

```powershell
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
```

Expected: mission-gate imports fail while characterization tests pass.

- [ ] **Step 4: Implement the gate and one narrow seam**

Keep callbacks and Stage logic intact. Replace only automatic `start_mission()`
with a callable activation seam that receives validated target values. Remove the
direct TTS call from `mission_done()`; emit an internal phase event instead. A
cancel request must set searching false, cancel the active move_base goal through
the adapter, publish zero, and emit failure once.

- [ ] **Step 5: Run tests and compare characterization**

```powershell
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
```

Expected: all tests pass and characterization confirms the vehicle algorithm and
model hashes remain unchanged.

- [ ] **Step 6: Commit**

```powershell
git add -- ucar_ws/src/stop/src/stop_integration/__init__.py ucar_ws/src/stop/src/stop_integration/mission_gate.py ucar_ws/src/stop/test/test_mission_gate.py ucar_ws/src/stop/test/test_vehicle_characterization.py ucar_ws/src/stop/scripts/mission_orchestrator.py ucar_ws/src/stop/setup.py
git commit -m "feat(stop): gate vehicle mission on correlated task input"
```

### Task 5: Isolate stop velocity sources

**Files:**
- Create: `ucar_ws/src/stop/src/stop_integration/velocity_mux.py`
- Create: `ucar_ws/src/stop/scripts/stop_velocity_mux_node.py`
- Create: `ucar_ws/src/stop/test/test_velocity_mux.py`
- Create: `ucar_ws/src/stop/test/test_velocity_mux_node.py`
- Modify: `ucar_ws/src/stop/scripts/mission_orchestrator.py`
- Modify: `ucar_ws/src/stop/CMakeLists.txt`
- Modify: `ucar_ws/src/stop/package.xml`

- [ ] **Step 1: Write failing mux tests**

Use modes `IDLE`, `NAVIGATION`, and `MANUAL`. Assert mode changes publish zero
before accepting the new source; wrong source is ignored; stale source, NaN/Inf,
bounds violations, clock rollback, exception, and shutdown publish zero. Assert
the node publishes only `/cmd_vel/stop`, subscribes only to
`/cmd_vel/stop_navigation` and `/cmd_vel/stop_manual`, and never advertises final
`/cmd_vel`.

- [ ] **Step 2: Run and verify RED**

```powershell
python ucar_ws/src/stop/test/test_velocity_mux.py -v
python ucar_ws/src/stop/test/test_velocity_mux_node.py -v
```

Expected: import failure because mux files do not exist.

- [ ] **Step 3: Implement pure mux and node**

Use defensive Twist copying and positive-finite validation matching the existing
competition arbiter style. Modify the stop mission's direct publisher to
`/cmd_vel/stop_manual`; configure stop move_base's `cmd_vel_topic` as
`/cmd_vel/stop_navigation`. Do not add a parking algorithm or a third source—the
vehicle stop package already uses one manual path for PCA and parking.

- [ ] **Step 4: Run tests and commit**

```powershell
python ucar_ws/src/stop/test/test_velocity_mux.py -v
python ucar_ws/src/stop/test/test_velocity_mux_node.py -v
git add -- ucar_ws/src/stop/src/stop_integration/velocity_mux.py ucar_ws/src/stop/scripts/stop_velocity_mux_node.py ucar_ws/src/stop/test/test_velocity_mux.py ucar_ws/src/stop/test/test_velocity_mux_node.py ucar_ws/src/stop/scripts/mission_orchestrator.py ucar_ws/src/stop/CMakeLists.txt ucar_ws/src/stop/package.xml
git commit -m "feat(stop): isolate vehicle navigation and manual velocity"
```

### Task 6: Convert stop phase events to protocol v1 results

**Files:**
- Create: `ucar_ws/src/stop/scripts/stop_protocol_adapter_node.py`
- Create: `ucar_ws/src/stop/test/test_protocol_adapter_node.py`
- Modify: `ucar_ws/src/stop/src/stop_integration/mission_gate.py`
- Modify: `ucar_ws/src/stop/scripts/mission_orchestrator.py`
- Modify: `ucar_ws/src/stop/CMakeLists.txt`

- [ ] **Step 1: Write failing adapter tests**

Feed one `/task/stop_mission_goal`, then internal `phase1_done` and `done` events.
Assert exact outputs:

```python
{"protocol_version": 1, "task_id": "task-1", "goal_id": "delivery-1",
 "status": "arrived", "message": ""}
{"protocol_version": 1, "task_id": "task-1", "goal_id": "simulation-1",
 "status": "arrived", "message": ""}
```

Assert physical success is published only after the first stop command and stop
verification; simulation success only after the second. Failure includes a
non-empty message, stops motion first, and never publishes success. Duplicate
events republish cached terminal output without restarting.

- [ ] **Step 2: Run and verify RED**

```powershell
python ucar_ws/src/stop/test/test_protocol_adapter_node.py -v
```

Expected: import failure because adapter does not exist.

- [ ] **Step 3: Implement adapter and internal event topic**

Use `/stop/mission_event` only as a private integration seam. The adapter owns
public protocol topics and validates every identity. It must pause Phase 2 until
the global orchestrator publishes the matching `/task/simulation_navigation_goal`;
on that acknowledgment it allows the existing `switch_to_phase2()` path to run.

- [ ] **Step 4: Run tests and commit**

```powershell
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
git add -- ucar_ws/src/stop/scripts/stop_protocol_adapter_node.py ucar_ws/src/stop/test/test_protocol_adapter_node.py ucar_ws/src/stop/src/stop_integration/mission_gate.py ucar_ws/src/stop/scripts/mission_orchestrator.py ucar_ws/src/stop/CMakeLists.txt
git commit -m "feat(stop): report correlated dual parking results"
```

### Task 7: Extend final competition velocity ownership

**Files:**
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py`
- Modify: `ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py`
- Modify: `ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_motion_mode.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py`
- Modify: `ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py`

- [ ] **Step 1: Add failing stop-source tests**

Add mode `STOP_NAVIGATION` mapped only to source `stop`. Assert navigation and QR
inputs are ignored in this mode, `/cmd_vel/stop` alone is forwarded, and every
transition into or out of it first emits zero. Preserve all existing first-phase
tests unchanged.

- [ ] **Step 2: Run and verify RED**

```powershell
python ucar_ws/src/task_orchestrator/test/test_motion_mode.py -v
python ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py -v
python ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py -v
```

Expected: FAIL because `STOP_NAVIGATION` and source `stop` are unknown.

- [ ] **Step 3: Implement the minimal third source**

Subscribe to `/cmd_vel/stop`; add it to `SOURCES` and map only
`STOP_NAVIGATION -> stop`. Keep one final `/cmd_vel` publisher, existing numeric
bounds, source timeout, clock rollback, and shutdown behavior.

- [ ] **Step 4: Run tests and commit**

```powershell
python ucar_ws/src/task_orchestrator/test/test_motion_mode.py -v
python ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py -v
python ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py -v
git add -- ucar_ws/src/task_orchestrator/src/task_orchestrator/motion_mode.py ucar_ws/src/task_orchestrator/src/task_orchestrator/velocity_arbiter.py ucar_ws/src/task_orchestrator/scripts/velocity_arbiter_node.py ucar_ws/src/task_orchestrator/test/test_motion_mode.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter.py ucar_ws/src/task_orchestrator/test/test_velocity_arbiter_node.py
git commit -m "feat(arbiter): authorize isolated stop velocity source"
```

### Task 8: Compose the integration launch and preflight

**Files:**
- Create: `ucar_ws/src/stop/launch/mission_integration.launch`
- Modify: `ucar_ws/src/task_orchestrator/launch/competition_full.launch`
- Modify: `ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch`
- Modify: `ucar_ws/src/task_orchestrator/scripts/start_competition.sh`
- Modify: `ucar_ws/src/task_orchestrator/test/test_competition_bringup.py`
- Create: `ucar_ws/src/stop/test/test_integration_launch.py`

- [ ] **Step 1: Write failing XML/preflight tests**

Parse launch XML with `ElementTree`. Assert one owner each for base, lidar, camera,
map, localization, and move_base at any phase; `mission_integration.launch` never
includes base/lidar/camera/speech; stop move_base remaps to
`/cmd_vel/stop_navigation`; stop mission publishes manual commands only to
`/cmd_vel/stop_manual`; no node except competition arbiter publishes `/cmd_vel`.

Add fake-shell tests proving explicit booleans for
`start_navigation_handoff` and `start_stop_stack`, argument-boundary preservation,
no `eval`, no broad kill, and fail-closed detection of simultaneously live AMCL
and lidar_loc or multiple move_base/map-to-odom owners.

- [ ] **Step 2: Run and verify RED**

```powershell
python ucar_ws/src/task_orchestrator/test/test_competition_bringup.py -v
python ucar_ws/src/stop/test/test_integration_launch.py -v
```

Expected: FAIL because integration launch and switches are absent.

- [ ] **Step 3: Implement launch composition**

Root launch starts shared hardware and all non-navigation persistent nodes. The
handoff supervisor owns the two navigation child launches. `mission_integration`
starts only stop map/AMCL/move_base/OCR/mission/adapter/mux and exposes initial
pose, waypoint-search, rotation, PCA, parking, timeout, and retry parameters
without changing vehicle defaults.

- [ ] **Step 4: Implement preflight**

Preflight must reject unknown boolean values, duplicate public hardware owners,
pre-existing conflicting localization/navigation nodes, multiple `/cmd_vel`
publishers, missing stop package/model files, manifest mismatch, missing secrets,
and absent devices owned by this launch. External reuse is allowed only through
an explicit `start_*:=false` switch.

- [ ] **Step 5: Run tests and commit**

```powershell
python ucar_ws/src/task_orchestrator/test/test_competition_bringup.py -v
python ucar_ws/src/stop/test/test_integration_launch.py -v
git add -- ucar_ws/src/stop/launch/mission_integration.launch ucar_ws/src/task_orchestrator/launch/competition_full.launch ucar_ws/src/task_orchestrator/launch/task_orchestrator.launch ucar_ws/src/task_orchestrator/scripts/start_competition.sh ucar_ws/src/task_orchestrator/test/test_competition_bringup.py ucar_ws/src/stop/test/test_integration_launch.py
git commit -m "feat(bringup): compose controlled stop stack handoff"
```

### Task 9: Add offline end-to-end scenarios

**Files:**
- Create: `ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py`
- Create: `ucar_ws/src/task_orchestrator/test/manual_stop_phase2_simulation.md`

- [ ] **Step 1: Write scenario tests with fake time and processes**

Cover: happy path through two arrivals; transient cancel/readiness retry then
success; old stack never exits; stale odom; AMCL never becomes ready; OCR absent;
physical navigation failure; first parking failure; simulation acknowledgment
missing; second navigation failure; second parking failure; cancel during every
active state; repeated and stale protocol messages. For every failure assert the
last commanded velocity is zero and no downstream success topic was emitted.

- [ ] **Step 2: Run and verify RED, then add only needed fake adapters**

```powershell
python ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py -v
```

Expected before fixtures: FAIL on missing fake composition. Implement test-local
fakes only; do not add production simulation stubs or fake arrival publishers.

- [ ] **Step 3: Write manual no-motion procedure**

Document exact commands to observe status, identity, navigation owners, TF,
localization, all velocity sources, final `/cmd_vel`, physical arrival, simulation
goal, and simulation arrival. Every injected result must use IDs copied from real
preceding outputs. State clearly that this is not vehicle acceptance.

- [ ] **Step 4: Run and commit**

```powershell
python ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py -v
git add -- ucar_ws/src/task_orchestrator/test/test_stop_phase2_scenarios.py ucar_ws/src/task_orchestrator/test/manual_stop_phase2_simulation.md
git commit -m "test(integration): cover stop phase two failure scenarios"
```

### Task 10: Documentation, handoff, and full regression

**Files:**
- Create: `ucar_ws/src/stop/README_INTEGRATION.md`
- Create: `HANDOFF_TO_CODEX.md`
- Modify: `ucar_ws/src/task_orchestrator/README.md`

- [ ] **Step 1: Document operator workflow**

Document the single safe entry point, module-by-module debug commands, full topic
timeline, controlled stack transition, bounded-retry policy, two separate arrival
contracts, all launch/YAML parameters with defaults and units, one-variable-at-a-time
tuning, safe shutdown, residual-node diagnosis, and rollback to first-phase baseline.
Explicitly prohibit `rosnode cleanup` as a live-node stop mechanism.

- [ ] **Step 2: Write handoff evidence**

Record branch and final HEAD, every commit, modified files, exact test counts and
summaries, vehicle snapshot source/manifest/hash, every runtime owner before/during/
after handoff, complete parameter mapping, unresolved vehicle assumptions, all dirty
files, and the checkpointed Codex vehicle acceptance sequence.

- [ ] **Step 3: Run full local verification**

```powershell
python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py" -v
python -m compileall -q ucar_ws/src/task_orchestrator ucar_ws/src/stop
git diff --check
git status --short
```

Expected: all runnable tests pass; every skip is named and justified; compileall
and diff check exit 0; status contains only files intentionally staged for docs.

- [ ] **Step 4: Parse every launch XML**

Run a short `python -c` using `xml.etree.ElementTree.parse` over:

```text
task_orchestrator/launch/competition_full.launch
task_orchestrator/launch/task_orchestrator.launch
stop/launch/mission.launch
stop/launch/mission_light.launch
stop/launch/mission_integration.launch
```

Expected: all parse without exceptions.

- [ ] **Step 5: Commit documentation explicitly**

```powershell
git add -- ucar_ws/src/stop/README_INTEGRATION.md ucar_ws/src/task_orchestrator/README.md HANDOFF_TO_CODEX.md
git commit -m "docs(integration): hand off checkpointed stop phase two acceptance"
```

## Completion boundary

DeepSeek stops after Task 10 and reports to Codex. It must not SSH, deploy, compile
on the vehicle, start or stop vehicle nodes, publish movement, merge, push, or claim
vehicle success. Codex performs review and then separately authorizes each vehicle
checkpoint: backup, deploy/build, no-motion startup, legacy-stack stop, stop-stack
readiness, physical-only run, simulation-target-only run, and finally the continuous
two-phase run.
