# Stop Waypoint Memory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remember canonical workshop labels observed at all three waypoints during physical-item delivery, then prefer the remembered simulation-item waypoint while still rerunning the full OCR and parking sequence there.

**Architecture:** Add one ROS-independent `WaypointMemory` helper under the existing installable `stop_integration` package. The vehicle-proven mission script records canonical OCR matches into that helper and uses a single phase-aware waypoint-advance function for scan exhaustion and navigation retry exhaustion. Phase 2 builds a deterministic route from the remembered workshop candidates; it never reuses camera, LiDAR, PCA, or parking measurements.

**Tech Stack:** Python 3.7-compatible standard library, ROS1 `rospy`, existing `unittest`/AST regression tests, catkin Python packaging.

---

## Scope and invariants

- Keep all external task-orchestrator topics, JSON protocols, phase acknowledgements, waypoint coordinates, Stage 0-6 parking logic, retry limits, and motion ownership unchanged.
- Preserve initial physical-item search order `1 -> 2 -> 3`.
- Record every OCR label that maps through `WAREHOUSE_MAP`, not only the currently requested simulation category.
- Directly prefer a remembered simulation waypoint only when exactly one waypoint candidate exists. Zero or multiple candidates use the deterministic fallback route.
- On Phase 2 arrival, always reset to Stage 0 and perform fresh OCR plus the existing parking procedure.
- Never cache or replay camera angles, LiDAR ranges, PCA angles, alignment offsets, velocity commands, or a prior parking success.
- Known simulation route: remembered simulation waypoint first, then every other waypoint except the physical parking waypoint, without duplicates.
- Unknown/conflicting simulation route: waypoints after the physical parking waypoint, then waypoints before it, excluding that physical waypoint.
- Both scan exhaustion and exhausted navigation retries call the same route-advance helper.

### Task 1: Add RED tests for the pure waypoint-memory policy

**Files:**
- Create: `ucar_ws/src/stop/test/test_waypoint_memory.py`
- Test target: `ucar_ws/src/stop/src/stop_integration/waypoint_memory.py`

- [ ] **Step 1: Write the failing import and behavior tests**

  Cover:

  ```python
  memory.record("电子产品生产车间", 0)
  memory.record("日用品加工车间", 1)
  self.assertEqual({0}, memory.candidates("电子产品生产车间"))
  self.assertEqual(0, memory.unique_waypoint("电子产品生产车间"))
  ```

  Also assert duplicate observations are idempotent, two different waypoint observations make `unique_waypoint()` return `None`, invalid indexes are rejected, and scan-complete waypoint tracking is independent from label storage.

- [ ] **Step 2: Add exact route examples**

  Assert zero-based routes for the approved scenarios:

  ```python
  # a at waypoint 2, remembered b at waypoint 1
  self.assertEqual([0, 2], build_phase2_route(physical_waypoint=1, preferred_waypoint=0, waypoint_count=3))
  # a at waypoint 1, remembered b at waypoint 3
  self.assertEqual([2, 1], build_phase2_route(physical_waypoint=0, preferred_waypoint=2, waypoint_count=3))
  # a at waypoint 3, remembered b at waypoint 2
  self.assertEqual([1, 0], build_phase2_route(physical_waypoint=2, preferred_waypoint=1, waypoint_count=3))
  ```

  Assert unknown/conflicting fallback routes are `[2, 0]`, `[1, 2]`, and `[0, 1]` for physical waypoints 2, 1, and 3 respectively (human numbering), and preferred physical waypoint is ignored because the two targets are guaranteed to belong to different workshops.

- [ ] **Step 3: Run the focused test and confirm RED**

  Run:

  ```powershell
  python -m unittest ucar_ws.src.stop.test.test_waypoint_memory -v
  ```

  Expected: import failure because `stop_integration.waypoint_memory` does not exist.

- [ ] **Step 4: Commit the RED test**

  ```powershell
  git add ucar_ws/src/stop/test/test_waypoint_memory.py
  git commit -m "test(stop): specify waypoint memory routing"
  ```

### Task 2: Implement the pure memory and routing helper

**Files:**
- Create: `ucar_ws/src/stop/src/stop_integration/waypoint_memory.py`
- Modify: `ucar_ws/src/stop/test/test_python_packaging.py`

- [ ] **Step 1: Implement the minimum Python 3.7-compatible API**

  Provide:

  ```python
  class WaypointMemory(object):
      def __init__(self, waypoint_count): ...
      def reset(self): ...
      def record(self, workshop, waypoint_index): ...
      def mark_scanned(self, waypoint_index): ...
      def candidates(self, workshop): ...
      def unique_waypoint(self, workshop): ...

  def build_phase2_route(physical_waypoint, preferred_waypoint, waypoint_count): ...
  ```

  Return copies of candidate sets, validate indexes, and keep route construction free of ROS dependencies.

- [ ] **Step 2: Extend packaging regression coverage**

  Assert `stop_integration` remains present in `setup.py`; no new package entry is needed because the module lives inside the existing package.

- [ ] **Step 3: Run focused tests and confirm GREEN**

  Run:

  ```powershell
  python -m unittest ucar_ws.src.stop.test.test_waypoint_memory ucar_ws.src.stop.test.test_python_packaging -v
  ```

  Expected: all tests pass.

- [ ] **Step 4: Commit the helper**

  ```powershell
  git add ucar_ws/src/stop/src/stop_integration/waypoint_memory.py ucar_ws/src/stop/test/test_python_packaging.py
  git commit -m "feat(stop): add workshop waypoint memory policy"
  ```

### Task 3: Add RED mission-wiring contract tests

**Files:**
- Modify: `ucar_ws/src/stop/test/test_vehicle_characterization.py`
- Test target: `ucar_ws/src/stop/scripts/mission_orchestrator.py`

- [ ] **Step 1: Replace obsolete double-pointer assertions**

  Remove assertions requiring `sim_point_index` and raw `current_point_index += 1`. Add AST/source contracts requiring:

  - `WaypointMemory` and `build_phase2_route` imports;
  - a reset at mission start;
  - a canonical-workshop recording helper called for OCR observations in `boxes_callback`;
  - a `mark_scanned` call when a waypoint scan is exhausted and when the current target is confirmed;
  - one `advance_to_next_waypoint()` helper used by both scan exhaustion and exhausted navigation retries;
  - Phase 2 route construction after the acknowledged simulation target is known;
  - Stage 0 and rotation counters reset before Phase 2 navigation;
  - no cached perception/parking fields in the memory helper or mission globals.

- [ ] **Step 2: Preserve frozen vehicle assertions**

  Keep exact waypoint coordinates, `WAREHOUSE_MAP`, Stage 0-6 markers, backup/180-degree turn, costmap clear, retry defaults, result strings, OCR/model manifest hashes, task topics, phase acknowledgement, and velocity isolation tests.

- [ ] **Step 3: Run focused tests and confirm RED**

  Run:

  ```powershell
  python -m unittest ucar_ws.src.stop.test.test_vehicle_characterization -v
  ```

  Expected: new mission-wiring assertions fail against the current `sim_point_index` implementation.

- [ ] **Step 4: Commit the RED contracts**

  ```powershell
  git add ucar_ws/src/stop/test/test_vehicle_characterization.py
  git commit -m "test(stop): require phase-aware waypoint routing"
  ```

### Task 4: Wire the memory policy into the vehicle mission

**Files:**
- Modify: `ucar_ws/src/stop/scripts/mission_orchestrator.py`

- [ ] **Step 1: Add route state without changing ROS contracts**

  Replace `sim_point_index` with:

  ```python
  waypoint_memory = WaypointMemory(len(FIND_POINTS_LIST))
  physical_point_index = None
  phase2_route = []
  phase2_route_cursor = 0
  ```

  Reset all four at `start_mission()`.

- [ ] **Step 2: Record all canonical workshops from every OCR callback**

  Normalize each recognized text using the existing `WAREHOUSE_MAP` aliases and record every matching canonical workshop at `current_point_index`. Do this before checking the current phase target. Do not alter native OCR messages or topic names.

- [ ] **Step 3: Centralize route advancement**

  Add `advance_to_next_waypoint()`:

  - real phase: increment through `0, 1, 2`;
  - simulation phase: advance `phase2_route_cursor` and select the corresponding route entry;
  - call `go_to_find_point()` after selecting;
  - let existing exhaustion handling emit `failed:not_found` when the selected route is finished.

  Replace both raw waypoint increments in scan exhaustion and navigation retry exhaustion with this helper.

- [ ] **Step 4: Remember the physical parking waypoint and build Phase 2 route**

  When the physical target is confirmed, mark the waypoint scanned. When physical parking completes, retain that waypoint as `physical_point_index`. In `switch_to_phase2()`, look up the acknowledged simulation workshop with `unique_waypoint()`, construct `phase2_route`, select its first entry, and then execute the unchanged backup/turn/costmap-clear sequence.

- [ ] **Step 5: Guarantee fresh Phase 2 perception**

  Before navigating to the first Phase 2 route entry, reset `search_item_stage`, `rotate_num`, bounding-box, LiDAR/PCA, alignment, and target-found state through the existing reset path. Do not inject a synthetic detection or skip Stage 0.

- [ ] **Step 6: Run focused and full stop tests**

  Run:

  ```powershell
  python -m unittest ucar_ws.src.stop.test.test_waypoint_memory ucar_ws.src.stop.test.test_vehicle_characterization -v
  python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
  ```

  Expected: all stop tests pass.

- [ ] **Step 7: Commit the mission wiring**

  ```powershell
  git add ucar_ws/src/stop/scripts/mission_orchestrator.py
  git commit -m "feat(stop): route simulation delivery through remembered workshops"
  ```

### Task 5: Document behavior and run local integration verification

**Files:**
- Modify: `ucar_ws/src/stop/README_INTEGRATION.md`
- Modify: `docs/superpowers/specs/2026-08-11-stop-waypoint-memory-design.md` only if implementation reveals a factual correction

- [ ] **Step 1: Document operator-visible route semantics**

  State that Phase 1 records canonical workshop observations, Phase 2 prefers only a unique remembered waypoint, OCR and parking always rerun, conflicting memory falls back safely, and the two requested workshops must differ.

- [ ] **Step 2: Run repository-level verification**

  Run:

  ```powershell
  python -m unittest discover -s ucar_ws/src/task_orchestrator/test -p "test_*.py" -v
  python -m unittest discover -s ucar_ws/src/stop/test -p "test_*.py" -v
  python -m unittest discover -s ucar_ws/src/llm_spark/test -p "test_*.py" -v
  python -m compileall -q ucar_ws/src/stop ucar_ws/src/task_orchestrator ucar_ws/src/llm_spark
  git diff --check
  git status --short
  ```

  Expected: all tests and compile checks pass; only planned documentation changes remain before commit.

- [ ] **Step 3: Commit documentation**

  ```powershell
  git add ucar_ws/src/stop/README_INTEGRATION.md
  git commit -m "docs(stop): explain remembered workshop routing"
  ```

### Task 6: Review, back up, deploy, and verify on the vehicle without motion

**Files:**
- Deploy only changed stop source/test/documentation files to `/home/ucar/ucar_ws/src/stop`
- Create a timestamped backup under `/home/ucar/ucar_ws/deploy_backups/`

- [ ] **Step 1: Review the complete branch diff**

  Compare from `165864e`, confirm no task-orchestrator interface, coordinates, models, OCR implementation, parking controller, launch ownership, or secret changed.

- [ ] **Step 2: Create a timestamped vehicle backup with a short manifest**

  Back up the current `/home/ucar/ucar_ws/src/stop` before overwriting. Record source branch, commit, timestamp, purpose, and changed file list. Do not delete earlier backups.

- [ ] **Step 3: Deploy the minimal changed file set**

  Copy the helper, mission script, tests, and documentation to their matching paths. Do not replace model files or unrelated packages.

- [ ] **Step 4: Run vehicle tests and build**

  Run remotely from `/home/ucar/ucar_ws`:

  ```bash
  python -m unittest discover -s src/stop/test -p 'test_*.py' -v
  python -m unittest discover -s src/task_orchestrator/test -p 'test_*.py' -v
  python -m unittest discover -s src/llm_spark/test -p 'test_*.py' -v
  catkin_make
  ```

  If the vehicle requires its previously established Python command or catkin package whitelist invocation, use that exact proven form and record it.

- [ ] **Step 5: Stop before motion**

  Do not start `start_competition.sh`, publish navigation goals, or command the base. Report the backup path, deployed commit, test totals, build result, and the exact supervised runtime command for the user.

## Plan self-review

- Every approved scenario has an exact deterministic route assertion.
- Ambiguous OCR memory cannot trigger a direct waypoint preference.
- Fresh Phase 2 OCR/parking is an explicit wiring and regression requirement.
- The plan changes no external interface and preserves the vehicle snapshot protections.
- The only new production abstraction is one ROS-independent helper in an already installed Python package.
- All commands use existing repository test runners and the vehicle's established build flow; no placeholder dependency or invented topic is introduced.
